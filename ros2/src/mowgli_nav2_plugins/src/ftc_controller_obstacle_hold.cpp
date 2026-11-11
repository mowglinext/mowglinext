// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// FTCController's obstacle checks on the robot itself and what it does when
// it cannot go on: the plan-window collision check (deviation off), the
// body-in-lethal check, the zero-velocity hold with its abort deadline, the
// collision_monitor hold detector, and the bounded, boundary-safe
// reverse-escape.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <nav2_ros_common/node_utils.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "mowgli_interfaces/ftc_abort_reason.hpp"
#include "mowgli_nav2_plugins/ftc_carrot_lead.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_obstacle_wait.hpp"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include "mowgli_nav2_plugins/ftc_resync.hpp"
#include "mowgli_nav2_plugins/ftc_stall.hpp"
#include "mowgli_nav2_plugins/ftc_start_index.hpp"
#include "mowgli_nav2_plugins/ftc_zone_grid.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

namespace mowgli_nav2_plugins
{

// ── Collision checking ────────────────────────────────────────────────────────

bool FTCController::checkCollision(int max_points)
{
  if (!config_.check_obstacles)
  {
    return false;
  }

  // Lock the costmap while reading cells — the costmap is updated from the
  // costmap_ros_ thread, so getCost()/getOrientedFootprint() here would
  // otherwise race the update and read torn/half-written cells (TOCTOU).
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_map_->getMutex());

  unsigned int mx = 0;
  unsigned int my = 0;

  visualization_msgs::msg::Marker obstacle_marker;

  // Clamp lookahead to the available plan length.
  if (static_cast<std::size_t>(max_points) > global_plan_.size())
  {
    max_points = static_cast<int>(global_plan_.size());
  }

  // Check robot footprint at current pose.
  if (config_.obstacle_footprint)
  {
    std::vector<geometry_msgs::msg::Point> footprint;
    costmap_ros_->getOrientedFootprint(footprint);

    for (const auto& fp_pt : footprint)
    {
      if (costmap_map_->worldToMap(fp_pt.x, fp_pt.y, mx, my))
      {
        const unsigned char cost = costmap_map_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          RCLCPP_WARN(logger_, "FTCController: lethal footprint collision at current pose.");
          return true;
        }
      }
    }
  }

  // Check costmap cells along the lookahead path segments, in the COSTMAP
  // frame: global_plan_ is in map, costmap_map_ in odom, and map->odom is not
  // identity (fusion_graph publishes it), so raw plan coordinates sample the
  // wrong cells — the bug updateLateralDeviation fixed for its own window.
  std::vector<geometry_msgs::msg::PoseStamped> window;
  planWindowInCostmapFrame(current_index_,
                           current_index_ + static_cast<std::size_t>(std::max(0, max_points)),
                           window);
  for (const auto& pose : window)
  {
    if (costmap_map_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
    {
      const unsigned char cost = costmap_map_->getCost(mx, my);

      if (config_.debug_obstacle)
      {
        debugObstacle(
            obstacle_marker, static_cast<double>(mx), static_cast<double>(my), cost, max_points);
      }

      // Only abort on lethal-or-inscribed cells: the robot footprint will hit
      // the obstacle. Inflation-gradient cells (128..252) are routinely
      // crossed by coverage strips that intentionally pass near obstacles —
      // collision_monitor's PolygonStop is the runtime guard for those (see
      // CLAUDE.md invariant 5).
      if (cost >= 253u)
      {
        RCLCPP_WARN(logger_, "FTCController: lethal obstacle on path (cost=%u).", cost);
        return true;
      }
    }
  }

  return false;
}

bool FTCController::currentBodyInLethal()
{
  if (costmap_map_ == nullptr || costmap_ros_ == nullptr)
  {
    return false;
  }
  const ObstacleDeviation::Footprint footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3)
  {
    return false;  // no polygon to rasterise — cannot assert
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    return false;  // pose unavailable this tick
  }
  // getRobotPose returns the pose in the costmap's global frame (odom), the
  // same frame as costmap_map_ — no transform needed. Lock against the costmap
  // update thread for the rasterised read.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_map_->getMutex());
  return ObstacleDeviation::footprintBlocked(*costmap_map_,
                                             robot_pose,
                                             0.0,
                                             footprint,
                                             ObstacleDeviation::BoundaryGuard{},
                                             ObstacleDeviation::kLethalOnlyThreshold);
}

// ── Lateral deviation (skirt obstacles) ───────────────────────────────────────

// Decide whether to stall the controller for `obstacle_wait_timeout_s` or
// throw and let the BT escalate. Called from updateLateralDeviation when
// the AVOIDANCE search runs out of headroom inside max_lateral_deviation
// (both sides blocked at the obstacle pose, OR growDeviationUntilClear
// exceeded the cap). Returns true if the caller should bail out of
// updateLateralDeviation for this tick (keeps lateral_deviation_ at its
// current value, output is zeroed in computeVelocityCommands). Returns
// false when the timeout has elapsed — caller proceeds as if the throw
// were direct, EXCEPT we still throw from inside here to consolidate the
// log message + is_crashed_ latch.
bool FTCController::waitOrThrowForObstacle(const std::string& reason)
{
  // Any blocked tick breaks the continuous-clear evidence accumulated while
  // waiting. It also owns a hard zero command, so freeze the carrot/PID state
  // before returning to computeVelocityCommands.
  obstacle_followable_time_ = 0.0;
  holdObstacleMotion();

  if (!obstacle_wait_start_.has_value())
  {
    obstacle_wait_start_ = clock_->now();
    RCLCPP_INFO(logger_,
                "FTCController: %s — holding zero velocity up to %.1fs for the costmap to clear.",
                reason.c_str(),
                config_.obstacle_wait_timeout_s);
  }
  const double elapsed = (clock_->now() - obstacle_wait_start_.value()).seconds();
  if (elapsed > config_.obstacle_wait_timeout_s)
  {
    is_crashed_ = true;
    // Issue #743: this whole function exists to wait for, then abort on, an
    // obstacle — mark it (mowgli_interfaces/ftc_abort_reason.hpp) so
    // FollowStrip's detour confirmation does not have to re-derive "was this
    // an obstacle" from a different costmap/body model.
    throw nav2_core::ControllerException(
        std::string(mowgli_interfaces::ftc_abort_reason::kObstacleAbortMarker) + "FTCController: " +
        reason + ", aborting strip after " + std::to_string(static_cast<int>(elapsed)) + "s wait.");
  }
  obstacle_waiting_ = true;
  return true;
}

void FTCController::holdObstacleMotion()
{
  current_movement_speed_ = 0.0;
  stall_time_ = 0.0;
  is_stalled_ = false;
  obstacle_recovery_active_ = true;
  last_recovery_angular_cmd_ = 0.0;

  // The controller does not run calculate_velocity_commands() while holding,
  // so its derivative history would otherwise become stale and the integral
  // terms would survive the stop. Track the current errors as the new baseline
  // and clear stored energy on every hold tick.
  i_lon_error_ = 0.0;
  i_lat_error_ = 0.0;
  i_angle_error_ = 0.0;
  last_lat_error_ = lat_error_;
  last_lon_error_ = lon_error_;
  last_angle_error_ = angle_error_;
  d_lat_filt_ = 0.0;
  d_lon_filt_ = 0.0;
  d_angle_filt_ = 0.0;
}

bool FTCController::collisionMonitorHoldConfirmed(double dt)
{
  const FtcCmHoldCfg cfg{config_.cm_hold_s, config_.cm_stall_only_s};
  const bool cm_holds = CollisionMonitorActionHolds(cm_action_.load());
  if (!CmHoldStep(cm_holds,
                  last_cmd_fwd_speed_,
                  last_measured_fwd_speed_,
                  dt,
                  cfg,
                  cm_held_s_,
                  cm_stalled_s_))
  {
    return false;
  }
  RCLCPP_WARN(logger_,
              "FTCController: robot stopped %.1fs while commanding %.2f m/s (collision_monitor "
              "action %u) — treating it as blocked.",
              cm_holds ? cm_held_s_ : cm_stalled_s_,
              last_cmd_fwd_speed_,
              static_cast<unsigned>(cm_action_.load()));
  cm_held_s_ = 0.0;
  cm_stalled_s_ = 0.0;
  return true;
}

// Bounded straight reverse-escape for the WEDGED case. SAFETY-CRITICAL: this is
// the only code path that drives the (bladed) robot backwards. It runs ONLY
// when the deviation search found no skirt this tick, and it fails safe — any
// missing precondition (feature off, no footprint, budget spent, rear obstacle)
// falls straight through to the existing waitOrThrowForObstacle behaviour.
ReverseEscapeAction FTCController::reverseEscapeStep(const ObstacleDeviation::Footprint& footprint,
                                                     double dt)
{
  ReverseEscapeCfg cfg;
  cfg.enabled = config_.obstacle_reverse_enabled;
  cfg.max_dist_m = config_.obstacle_reverse_max_dist_m;
  cfg.speed_mps = config_.obstacle_reverse_speed_mps;

  // The rear-clear safety check needs an explicit footprint AND the ACTUAL robot
  // pose in the costmap frame — NOT a path pose. On a laterally-deviated
  // approach the path pose differs from the real robot pose by the deviation, so
  // probing there could check the wrong spot. getRobotPose() is the same source
  // getOrientedFootprint uses for the current-pose collision check; it returns
  // the pose in the costmap's global frame (odom), matching costmap_map_. If
  // either the footprint or the pose is missing, refuse to reverse (fail safe →
  // wait/abort).
  const bool have_footprint = footprint.size() >= 3;
  geometry_msgs::msg::PoseStamped robot_pose;
  const bool have_pose = have_footprint && costmap_ros_->getRobotPose(robot_pose);

  // Integrate any reverse motion since the previous tick (odom-based, hard
  // capped), so the budget reflects real travel, not commanded travel.
  if (reverse_escape_active_)
  {
    reverse_distance_done_ =
        ReverseEscapeAdvance(cfg, reverse_distance_done_, last_measured_fwd_speed_, dt);
    reverse_elapsed_s_ += std::max(0.0, dt);
  }

  // Boundary-safe: never reverse while outside the recorded area, and only over ground the robot
  // has just driven forward in a straight line (so the reverse stays where it came from). Decided
  // when the reverse would START; once under way the pose has moved back over the breadcrumbs, so
  // only the "left the area" condition can still stop it.
  if (config_.reverse_boundary_safe)
  {
    const double wanted = std::max(0.0, cfg.max_dist_m - reverse_distance_done_);
    const bool refused = reverse_escape_active_ ? outside_area_.load()
                                                : reverseAllowedDistance(wanted) < wanted - 0.02;
    if (refused)
    {
      return ReverseEscapeAction::kExhausted;
    }
  }

  bool rear_clear = false;
  if (have_pose)
  {
    // Probe the footprint at the pose we would occupy after backing the REAL
    // robot pose up a short look-behind distance (the remaining budget, capped
    // to a probe horizon). Re-checked every tick as we creep back, so a clear
    // probe here guarantees the ~1 cm of travel this tick stays clear.
    // True-lethal threshold only — we must not reverse into a real obstacle, but
    // inflation halos behind us (which the robot legitimately hugs) must not
    // veto the escape.
    constexpr double kRearProbeHorizon = 0.20;  // m
    const double remaining = std::max(0.0, cfg.max_dist_m - reverse_distance_done_);
    const double back_probe = std::min(kRearProbeHorizon, remaining);
    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    geometry_msgs::msg::PoseStamped rear = robot_pose;
    rear.pose.position.x -= back_probe * std::cos(yaw);
    rear.pose.position.y -= back_probe * std::sin(yaw);
    rear_clear = !ObstacleDeviation::footprintBlocked(*costmap_map_,
                                                      rear,
                                                      0.0,
                                                      footprint,
                                                      ObstacleDeviation::BoundaryGuard{},
                                                      ObstacleDeviation::kLethalOnlyThreshold);
  }

  return have_pose
             ? ReverseEscapeDecide(cfg, reverse_distance_done_, rear_clear, reverse_elapsed_s_)
             : ReverseEscapeAction::kExhausted;
}

double FTCController::reverseAllowedDistance(double wanted)
{
  if (wanted <= 0.0)
  {
    return 0.0;
  }
  if (!config_.reverse_boundary_safe)
  {
    return wanted;
  }
  if (outside_area_.load())
  {
    RCLCPP_WARN_THROTTLE(
        logger_,
        *clock_,
        5000,
        "FTCController: reverse refused — the robot is outside the recorded area.");
    return 0.0;
  }
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    return 0.0;  // no pose, no reverse
  }
  const retrace::Pose2D current{robot_pose.pose.position.x,
                                robot_pose.pose.position.y,
                                tf2::getYaw(robot_pose.pose.orientation)};
  retrace::Refusal refusal = retrace::Refusal::kNone;
  for (const double candidate : {wanted, 0.20, 0.12})
  {
    if (candidate > wanted)
    {
      continue;
    }
    const retrace::Plan plan = reverse_crumbs_.plan(current, candidate);
    if (plan.refusal == retrace::Refusal::kNone)
    {
      return candidate;
    }
    refusal = plan.refusal;
  }
  RCLCPP_WARN_THROTTLE(logger_,
                       *clock_,
                       5000,
                       "FTCController: reverse refused — %s.",
                       refusal == retrace::Refusal::kNoHistory ? "not enough ground driven to undo"
                       : refusal == retrace::Refusal::kWrongDirection
                           ? "the last piece was itself a reverse"
                           : "the last piece was not a straight drive behind the robot");
  return 0.0;
}

double FTCController::planArcLength(std::size_t index) const
{
  double length = 0.0;
  const std::size_t last = std::min(index, global_plan_.empty() ? 0 : global_plan_.size() - 1);
  for (std::size_t i = 0; i < last; ++i)
  {
    length += std::hypot(global_plan_[i + 1].pose.position.x - global_plan_[i].pose.position.x,
                         global_plan_[i + 1].pose.position.y - global_plan_[i].pose.position.y);
  }
  return length;
}

double FTCController::progressSinceReverseEngaged() const
{
  if (!reverse_budget_touched_ || global_plan_.empty())
  {
    return 0.0;
  }
  const std::size_t last = global_plan_.size() - 1;
  const std::size_t from = std::min(reverse_engaged_index_, last);
  const std::size_t to = std::min<std::size_t>(current_index_, last);
  double length = 0.0;
  for (std::size_t i = from; i < to; ++i)
  {
    length += std::hypot(global_plan_[i + 1].pose.position.x - global_plan_[i].pose.position.x,
                         global_plan_[i + 1].pose.position.y - global_plan_[i].pose.position.y);
  }
  return length;
}

bool FTCController::reverseEscapeOrWait(const std::string& reason,
                                        const ObstacleDeviation::Footprint& footprint,
                                        double dt)
{
  ReverseEscapeCfg cfg;
  cfg.enabled = config_.obstacle_reverse_enabled;
  cfg.max_dist_m = config_.obstacle_reverse_max_dist_m;
  cfg.speed_mps = config_.obstacle_reverse_speed_mps;

  // This tick is NOT followable: an engaged escape stays committed (see
  // ReverseEscapeShouldRelease).
  reverse_followable_time_ = 0.0;
  const ReverseEscapeAction action = reverseEscapeStep(footprint, dt);

  if (action == ReverseEscapeAction::kReverse)
  {
    if (!reverse_escape_active_)
    {
      RCLCPP_WARN(logger_,
                  "FTCController: WEDGED (%s) — bounded straight reverse-escape (<= %.2fm at "
                  "%.2f m/s), rear footprint clear.",
                  reason.c_str(),
                  cfg.max_dist_m,
                  cfg.speed_mps);
    }
    reverse_escape_active_ = true;
    if (!reverse_budget_touched_)
    {
      reverse_budget_touched_ = true;
      reverse_engaged_index_ = current_index_;
    }
    // A reverse-escape is an ACTIVE maneuver, not a passive hold — drop any
    // wait state so the two states never fight over cmd_vel.
    obstacle_waiting_ = false;
    obstacle_wait_start_.reset();
    obstacle_followable_time_ = 0.0;
    // The post-hold angular slew (obstacle_restart_angular_acceleration) is
    // only meant for the probation after a hold. Its other reset lives in the
    // wait branch we just left, so clear it here or it stays armed for the
    // rest of the sub-path, including PRE_ROTATE pivots.
    obstacle_recovery_active_ = false;
    last_recovery_angular_cmd_ = 0.0;
    return true;  // caller returns; computeVelocityCommands emits the reverse.
  }

  // kNone (disabled) or kExhausted (budget spent / rear blocked / no footprint
  // or robot pose): give up on reversing and fall through to the wait-before-
  // abort behaviour.
  if (reverse_escape_active_)
  {
    RCLCPP_WARN(logger_,
                "FTCController: reverse-escape exhausted after %.2fm (%s) — holding/aborting.",
                reverse_distance_done_,
                reason.c_str());
  }
  reverse_escape_active_ = false;
  return waitOrThrowForObstacle(reason);
}

}  // namespace mowgli_nav2_plugins
