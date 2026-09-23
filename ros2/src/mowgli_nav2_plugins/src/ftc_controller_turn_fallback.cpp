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
// FTCController's side of the TURN FALLBACK (ftc_turn_fallback.hpp): planning
// it from the controller's plan, costmaps and TF where the offset lattice would
// go WEDGED, the bounded straight reverse, splicing runtime pivot corners and
// the connector into the working plan (executed by the existing PIVOT and
// FOLLOWING states, which re-check every cycle), completion at the rejoin
// (republishing the remainder for the progress trackers), the deadline, and the
// fall back to hold/abort. Kept out of ftc_controller.cpp to keep that file
// from growing further.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

namespace mowgli_nav2_plugins
{

// ── Turn fallback (ftc_turn_fallback.hpp) ─────────────────────────────────────

TurnFallbackPlan FTCController::planTurnFallback(std::size_t carrot_idx,
                                                 const BoundaryGuard& guard,
                                                 const ObstacleDeviation::Footprint& body,
                                                 double max_reverse_m,
                                                 std::size_t& window_first,
                                                 TurnFallbackProblem* problem_out)
{
  TurnFallbackPlan none;
  none.verdict = TurnFallbackVerdict::kBadInput;
  window_first = 0;
  if (costmap_map_ == nullptr || global_plan_.size() < 3)
  {
    return none;
  }
  TurnFallbackCfg cfg;
  cfg.max_reverse_m = std::max(0.0, max_reverse_m);
  cfg.max_rejoin_arc_m = config_.turn_fallback_max_rejoin_arc_m;
  cfg.min_turn_rad = config_.turn_fallback_min_turn_deg * (M_PI / 180.0);
  cfg.blockage_scan_m = config_.avoidance_horizon_m;

  TurnFallbackProblem problem;
  problem.costmap = costmap_map_;
  problem.guard = guard;
  problem.footprint = costmap_ros_->getRobotFootprint();
  problem.body = body;
  geometry_msgs::msg::PoseStamped robot_pose;  // costmap frame (odom)
  if (!costmap_ros_->getRobotPose(robot_pose))
  {
    none.why = "robot pose unavailable";
    return none;
  }
  problem.robot = {robot_pose.pose.position.x,
                   robot_pose.pose.position.y,
                   tf2::getYaw(robot_pose.pose.orientation)};

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const std::string plan_frame = global_plan_.front().header.frame_id;
  geometry_msgs::msg::TransformStamped plan_to_costmap;
  double rx = problem.robot.x;
  double ry = problem.robot.y;
  const bool same_frame = plan_frame.empty() || plan_frame == costmap_frame;
  try
  {
    if (!same_frame)
    {
      plan_to_costmap = tf_buffer_->lookupTransform(costmap_frame, plan_frame, tf2::TimePointZero);
      const auto robot_in_plan =
          tf_buffer_->lookupTransform(plan_frame, "base_link", tf2::TimePointZero);
      rx = robot_in_plan.transform.translation.x;
      ry = robot_in_plan.transform.translation.y;
    }
  }
  catch (const tf2::TransformException& ex)
  {
    none.why = std::string("no transform: ") + ex.what();
    return none;
  }
  const auto [first, last] = FallbackWindow(global_plan_,
                                            carrot_idx,
                                            rx,
                                            ry,
                                            kFallbackRobotSearchBackM,
                                            cfg.blockage_scan_m + cfg.max_rejoin_arc_m);
  window_first = first;
  if (!planWindowInCostmapFrame(first, last, problem.plan))
  {
    none.why = "plan window not in the costmap frame";
    return none;
  }
  const auto to_costmap = [&](const geometry_msgs::msg::PoseStamped& p)
  {
    if (same_frame)
    {
      return p;
    }
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(p, out, plan_to_costmap);
    return out;
  };
  const LatticeSolverCfg solver_cfg = latticeSolverCfg();
  problem.followable = [&, first = first](std::size_t w)
  {
    const std::size_t j = first + w;
    const auto [leg_first, leg_last] = PivotLeg(pivot_corners_, j, global_plan_.size());
    return LatticeFeasibleFrom(*costmap_map_,
                               guard,
                               body,
                               global_plan_,
                               j,
                               leg_first,
                               leg_last,
                               NextPivotCorner(pivot_corners_, j),
                               to_costmap,
                               solver_cfg,
                               config_.avoidance_horizon_m);
  };
  const TurnFallbackPlan plan = PlanTurnFallback(problem, cfg);
  if (problem_out != nullptr)
  {
    problem.followable = nullptr;  // captures locals of this frame
    *problem_out = std::move(problem);
  }
  return plan;
}

bool FTCController::tryTurnFallback(std::size_t carrot_idx,
                                    const BoundaryGuard& guard,
                                    const ObstacleDeviation::Footprint& body)
{
  // Only at the ONSET of a wedge while following the plan: not while a
  // reverse-escape or an obstacle hold already owns the output, and never a
  // second fallback on top of a running one.
  if (!config_.turn_fallback_enabled || current_state_ != PlannerState::FOLLOWING ||
      turn_fallback_.phase != TurnFallbackPhase::kIdle || reverse_escape_active_ ||
      obstacle_waiting_)
  {
    return false;
  }
  if (turn_fallback_.rearm_idx.has_value() && current_index_ < *turn_fallback_.rearm_idx)
  {
    RCLCPP_WARN_THROTTLE(logger_,
                         *clock_,
                         2000,
                         "FTCController: turn fallback not re-armed yet (idx %u, needs %zu: "
                         "%.1f m past the last rejoin) — WEDGED handling as before.",
                         current_index_,
                         *turn_fallback_.rearm_idx,
                         kTurnFallbackRearmM);
    return false;
  }
  // The reverse obeys the operator's reverse switch, like the reverse-escape.
  const double max_reverse =
      config_.obstacle_reverse_enabled ? config_.turn_fallback_max_reverse_m : 0.0;
  std::size_t first = 0;
  TurnFallbackProblem problem;
  const auto t0 = std::chrono::steady_clock::now();
  const TurnFallbackPlan plan =
      planTurnFallback(carrot_idx, guard, body, max_reverse, first, &problem);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (plan.verdict != TurnFallbackVerdict::kPlanned)
  {
    RCLCPP_WARN(logger_,
                "FTCController: turn fallback not possible at idx=%u (%s%s%s): blockage at idx %zu "
                "%.2f m ahead, plan turn %.0f deg, rear clear %.2f m, %zu candidates, %.1f ms — "
                "WEDGED handling as before.",
                current_index_,
                ToString(plan.verdict),
                plan.why.empty() ? "" : ": ",
                plan.why.c_str(),
                first + plan.blocked,
                plan.blocked_arc_m,
                plan.turn_rad * 180.0 / M_PI,
                plan.reverse_limit_m,
                plan.evaluated,
                ms);
    return false;
  }
  const TurnFallbackClearances cl =
      MeasureTurnFallbackClearances(problem, plan, kTurnFallbackClearanceLogCapM);
  const auto finite_or = [](double v)
  {
    return std::isfinite(v) ? v : -1.0;
  };
  RCLCPP_WARN(
      logger_,
      "FTCController: TURN FALLBACK at idx=%u — the lattice is WEDGED by lethal cells under "
      "the body at idx %zu (%.2f m ahead) in a %.0f deg turn of the plan. Plan: reverse "
      "%.2f m, pivot %+.0f deg, straight %.2f m, pivot %+.0f deg, rejoin idx %zu, "
      "skipping %.2f m of path. Clearances (m, -1 = leg not driven): reverse %.2f, pivot "
      "%.2f, straight %.2f, pivot %.2f; %zu candidates, %.1f ms.",
      current_index_,
      first + plan.blocked,
      plan.blocked_arc_m,
      plan.turn_rad * 180.0 / M_PI,
      plan.reverse_m,
      plan.rotate_start_rad * 180.0 / M_PI,
      plan.connector_m,
      plan.rotate_rejoin_rad * 180.0 / M_PI,
      first + plan.rejoin,
      plan.skipped_arc_m,
      finite_or(cl.reverse_m),
      finite_or(cl.rotate_start_m),
      finite_or(cl.connector_m),
      finite_or(cl.rotate_rejoin_m),
      plan.evaluated,
      ms);
  turn_fallback_.started = clock_->now();
  turn_fallback_.reverse_done_m = 0.0;
  turn_fallback_.reverse_time_s = 0.0;
  ++turn_fallback_.count;
  if (plan.reverse_m > kTurnFallbackReverseDoneM)
  {
    // Back up first; the pivots are planned again from where the reverse ends.
    turn_fallback_.phase = TurnFallbackPhase::kReverse;
    turn_fallback_.reverse_target_m = plan.reverse_m;
    holdObstacleMotion();
    obstacle_recovery_active_ = false;
  }
  else if (!spliceTurnFallback(plan, first))
  {
    RCLCPP_WARN(logger_,
                "FTCController: turn fallback could not be spliced (no robot pose) — WEDGED "
                "handling as before.");
    turn_fallback_.phase = TurnFallbackPhase::kIdle;
    return false;
  }
  turn_fallback_engaged_now_ = true;
  return true;
}

bool FTCController::spliceTurnFallback(const TurnFallbackPlan& plan, std::size_t window_first)
{
  const std::size_t j = window_first + plan.rejoin;
  if (j + 2 >= global_plan_.size())
  {
    return false;
  }
  const std::string plan_frame = global_plan_[j].header.frame_id;
  double sx = 0.0;
  double sy = 0.0;
  double syaw = 0.0;
  try
  {
    const auto tf =
        tf_buffer_->lookupTransform(plan_frame.empty() ? std::string("map") : plan_frame,
                                    "base_link",
                                    tf2::TimePointZero);
    sx = tf.transform.translation.x;
    sy = tf.transform.translation.y;
    syaw = tf2::getYaw(tf.transform.rotation);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(logger_, "FTCController: turn fallback splice: TF failed (%s).", ex.what());
    return false;
  }

  // The same geometry the planner checked, rebuilt in the plan frame from
  // where the robot actually stands.
  const auto& rejoin = global_plan_[j];
  const double jx = rejoin.pose.position.x;
  const double jy = rejoin.pose.position.y;
  const double jyaw = tf2::getYaw(rejoin.pose.orientation);
  const auto make = [&rejoin](double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header = rejoin.header;
    p.pose.position.x = x;
    p.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    p.pose.orientation = tf2::toMsg(q);
    return p;
  };
  std::vector<geometry_msgs::msg::PoseStamped> spliced;
  const double connector = std::hypot(jx - sx, jy - sy);
  std::size_t rejoin_idx = 0;
  if (plan.connector_m <= 0.0)
  {
    // Single rotation where the robot stands, straight onto the rejoin heading.
    if (std::abs(WrapPivotAngle(jyaw - syaw)) >= kPivotCornerDetectTurnRad)
    {
      spliced.push_back(make(sx, sy, syaw));
    }
    spliced.push_back(make(sx, sy, jyaw));
    rejoin_idx = spliced.size();  // the plan resumes after the rejoin pose
  }
  else
  {
    const double heading = std::atan2(jy - sy, jx - sx);
    if (std::abs(WrapPivotAngle(heading - syaw)) >= kPivotCornerDetectTurnRad)
    {
      spliced.push_back(make(sx, sy, syaw));  // corner: incoming heading
    }
    spliced.push_back(make(sx, sy, heading));  // corner: outgoing heading
    const std::size_t n = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(connector / kTurnFallbackConnectorStepM)));
    for (std::size_t k = 1; k < n; ++k)
    {
      const double f = static_cast<double>(k) / static_cast<double>(n);
      spliced.push_back(make(sx + f * (jx - sx), sy + f * (jy - sy), heading));
    }
    if (std::abs(WrapPivotAngle(jyaw - heading)) >= kPivotCornerDetectTurnRad)
    {
      spliced.push_back(make(jx, jy, heading));  // corner: incoming heading
    }
    rejoin_idx = spliced.size();
    spliced.push_back(rejoin);  // corner: outgoing heading = the plan pose itself
  }
  // The rest of the plan, after the rejoin pose, unchanged (incl. its pivot
  // corners and the duplicated tail pose the state machine relies on).
  spliced.insert(spliced.end(),
                 global_plan_.begin() + static_cast<std::ptrdiff_t>(j + 1),
                 global_plan_.end());

  global_plan_ = std::move(spliced);
  {
    std::vector<PlanPose2D> poses;
    poses.reserve(global_plan_.size());
    for (const auto& ps : global_plan_)
    {
      poses.push_back({ps.pose.position.x, ps.pose.position.y, tf2::getYaw(ps.pose.orientation)});
    }
    pivot_corners_ = FindPivotCorners(poses);
  }

  // Nothing of the old leg carries over: no offset, no obstacle hold, no
  // reverse-escape in progress. The fallback's own reverse is charged to the
  // reverse-escape budget, refilled only by real progress past the start.
  is_avoiding_ = false;
  target_lateral_deviation_ = 0.0;
  lateral_deviation_ = 0.0;
  avoidance_clear_start_.reset();
  lattice_return_start_.reset();
  lattice_switch_start_.reset();
  obstacle_waiting_ = false;
  obstacle_wait_start_.reset();
  obstacle_followable_time_ = 0.0;
  obstacle_recovery_active_ = false;
  last_recovery_angular_cmd_ = 0.0;
  reverse_escape_active_ = false;
  reverse_followable_time_ = 0.0;
  if (turn_fallback_.reverse_done_m > 0.0)
  {
    reverse_distance_done_ =
        std::min(config_.obstacle_reverse_max_dist_m,
                 std::max(reverse_distance_done_, turn_fallback_.reverse_done_m));
    reverse_budget_touched_ = true;
  }
  reverse_engaged_index_ = 0;

  current_index_ = 0;
  current_progress_ = 0.0;
  turn_fallback_.phase = TurnFallbackPhase::kRunning;
  turn_fallback_.rejoin_idx = rejoin_idx;
  turn_fallback_.skipped_arc_m = plan.skipped_arc_m;
  // Visualisation only (/<plugin>/global_plan, what newPathReceived publishes):
  // the plan FTC now drives, detour included.
  {
    nav_msgs::msg::Path working;
    working.header = plan_header_;
    working.header.stamp = clock_->now();
    working.poses = global_plan_;
    global_plan_pub_->publish(working);
  }
  const bool pivot_first = !pivot_corners_.empty() && pivot_corners_.front() == 0;
  RCLCPP_INFO(logger_,
              "FTCController: turn fallback spliced at (%.2f, %.2f): %s%.2f m straight, rejoin at "
              "working idx %zu (%.2f, %.2f); %zu pivot corner(s) in the plan now.",
              sx,
              sy,
              pivot_first ? "pivot, " : "",
              plan.connector_m > 0.0 ? connector : 0.0,
              rejoin_idx,
              jx,
              jy,
              pivot_corners_.size());
  state_entered_time_ = clock_->now();
  if (pivot_first)
  {
    current_state_ = PlannerState::PIVOT;
    enterPivot();
  }
  else
  {
    // No rotation worth a pivot: follow the connector (or the plan) straight
    // away, from a standstill, like a fresh plan after PRE_ROTATE.
    current_state_ = PlannerState::FOLLOWING;
    tf2::fromMsg(global_plan_[0].pose, current_control_point_);
    angle_error_raw_prev_ = std::numeric_limits<double>::quiet_NaN();
    current_movement_speed_ = config_.speed_slow;
    stall_time_ = 0.0;
    is_stalled_ = false;
    i_lon_error_ = 0.0;
    i_lat_error_ = 0.0;
    i_angle_error_ = 0.0;
    last_lat_error_ = 0.0;
    last_lon_error_ = 0.0;
    last_angle_error_ = 0.0;
    d_lat_filt_ = 0.0;
    d_lon_filt_ = 0.0;
    d_angle_filt_ = 0.0;
  }
  return true;
}

void FTCController::turnFallbackReverseTick(double dt, geometry_msgs::msg::TwistStamped& cmd_vel)
{
  auto& fb = turn_fallback_;
  // Odometry, not commanded travel, spends the reverse (as the reverse-escape).
  fb.reverse_done_m =
      std::min(fb.reverse_target_m, fb.reverse_done_m + std::abs(last_measured_fwd_speed_) * dt);
  fb.reverse_time_s += dt;
  const double remaining = fb.reverse_target_m - fb.reverse_done_m;
  const double time_cap =
      kTurnFallbackReverseTimeFactor * fb.reverse_target_m /
          std::max(kTurnFallbackMinReverseSpeedMps, config_.obstacle_reverse_speed_mps) +
      kTurnFallbackReverseTimeSlackS;

  const ObstacleDeviation::Footprint footprint = costmap_ros_->getRobotFootprint();
  geometry_msgs::msg::PoseStamped robot_pose;
  const bool have_pose = footprint.size() >= 3 && costmap_ros_->getRobotPose(robot_pose);
  bool rear_clear = false;
  if (have_pose && remaining > kTurnFallbackReverseDoneM)
  {
    // Every tick: the straight rear sweep over what is left to reverse (at
    // most the reverse-escape's 0.20 m look-behind) is free of TRUE-lethal
    // cells, so the next few centimetres certainly are.
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_map_->getMutex());
    rear_clear = ReverseSweepClear(*costmap_map_,
                                   footprint,
                                   {robot_pose.pose.position.x,
                                    robot_pose.pose.position.y,
                                    tf2::getYaw(robot_pose.pose.orientation)},
                                   std::min(kTurnFallbackRearProbeM, remaining));
  }
  if (remaining > kTurnFallbackReverseDoneM && rear_clear && fb.reverse_time_s < time_cap)
  {
    cmd_vel.twist.linear.x = -config_.obstacle_reverse_speed_mps;
    cmd_vel.twist.angular.z = 0.0;
    return;
  }

  // The reverse is over: done, rear blocked, pose lost, or too slow. Plan the
  // pivots from where the robot actually is, without reversing any further.
  const char* why =
      remaining <= kTurnFallbackReverseDoneM
          ? "done"
          : (!have_pose ? "robot pose lost" : (!rear_clear ? "rear blocked" : "time cap"));
  TurnFallbackPlan plan;
  std::size_t first = 0;
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_map_->getMutex());
    std::unique_lock<std::mutex> boundary_lock(boundary_mutex_, std::defer_lock);
    BoundaryGuard guard{};
    bool guard_ok = true;
    if (config_.confine_deviation_to_zone)
    {
      boundary_lock.lock();
      guard_ok = buildBoundaryGuard(guard);
    }
    if (guard_ok)
    {
      const ObstacleDeviation::Footprint body = ObstacleDeviation::expandFootprintLateral(
          footprint, std::max(0.0, config_.obstacle_clearance_margin));
      plan = planTurnFallback(current_index_, guard, body, 0.0, first);
    }
  }
  RCLCPP_INFO(logger_,
              "FTCController: turn fallback reversed %.2f of %.2f m (%s); from here: %s.",
              fb.reverse_done_m,
              fb.reverse_target_m,
              why,
              ToString(plan.verdict));
  if (plan.verdict == TurnFallbackVerdict::kPlanned && spliceTurnFallback(plan, first))
  {
    return;  // zero velocity this tick; the first pivot starts next tick
  }
  failTurnFallback(std::string("no safe pivot after reversing (") + ToString(plan.verdict) +
                   (plan.why.empty() ? "" : ": " + plan.why) + ")");
}

void FTCController::turnFallbackProgress()
{
  auto& fb = turn_fallback_;
  const double elapsed = (clock_->now() - fb.started).seconds();
  if (fb.phase == TurnFallbackPhase::kRunning && current_state_ == PlannerState::FOLLOWING &&
      current_index_ >= fb.rejoin_idx)
  {
    // Re-arm only once the robot has driven kTurnFallbackRearmM past the rejoin.
    std::size_t rearm = fb.rejoin_idx;
    double arc = 0.0;
    while (rearm + 1 < global_plan_.size() && arc < kTurnFallbackRearmM)
    {
      arc +=
          std::hypot(global_plan_[rearm + 1].pose.position.x - global_plan_[rearm].pose.position.x,
                     global_plan_[rearm + 1].pose.position.y - global_plan_[rearm].pose.position.y);
      ++rearm;
    }
    fb.rearm_idx = rearm;
    // The goal checker and FollowStrip track the plan published on
    // <plugin>/global_plan: hand them what is left, from the rejoin pose (an
    // exact copy of a pose of the goal's path), so their progress cursors do
    // not stay behind at the skipped turn.
    nav_msgs::msg::Path remaining;
    remaining.header = plan_header_;
    remaining.header.stamp = clock_->now();
    remaining.poses.assign(global_plan_.begin() + static_cast<std::ptrdiff_t>(fb.rejoin_idx),
                           global_plan_.end());
    progress_plan_pub_->publish(remaining);
    RCLCPP_WARN(
        logger_,
        "FTCController: TURN FALLBACK complete — rejoined the plan at working idx %zu after "
        "%.1f s (%.2f m of path skipped, fallback #%zu of this goal); republished the "
        "remaining %zu poses.",
        fb.rejoin_idx,
        elapsed,
        fb.skipped_arc_m,
        fb.count,
        remaining.poses.size());
    fb.phase = TurnFallbackPhase::kIdle;
    return;
  }
  if (elapsed > config_.turn_fallback_timeout_s)
  {
    RCLCPP_ERROR(logger_,
                 "FTCController: turn fallback did not reach its rejoin within %.0f s — aborting.",
                 config_.turn_fallback_timeout_s);
    fb.phase = TurnFallbackPhase::kIdle;
    is_crashed_ = true;
    throw nav2_core::ControllerException(
        "FTCController: turn fallback timed out before reaching its rejoin.");
  }
}

void FTCController::failTurnFallback(const std::string& why)
{
  RCLCPP_WARN(logger_,
              "FTCController: turn fallback abandoned (%s) — holding, then aborting like any "
              "WEDGED.",
              why.c_str());
  // The distance already reversed counts against the reverse-escape budget, so
  // the old WEDGED path cannot add a second full reverse on top.
  reverse_distance_done_ =
      std::min(config_.obstacle_reverse_max_dist_m,
               std::max(reverse_distance_done_, turn_fallback_.reverse_done_m));
  reverse_budget_touched_ = true;
  reverse_engaged_index_ = current_index_;
  reverse_escape_active_ = false;
  // No second attempt from this spot.
  std::size_t rearm = current_index_;
  double arc = 0.0;
  while (rearm + 1 < global_plan_.size() && arc < kTurnFallbackRearmM)
  {
    arc +=
        std::hypot(global_plan_[rearm + 1].pose.position.x - global_plan_[rearm].pose.position.x,
                   global_plan_[rearm + 1].pose.position.y - global_plan_[rearm].pose.position.y);
    ++rearm;
  }
  turn_fallback_.rearm_idx = rearm;
  turn_fallback_.phase = TurnFallbackPhase::kIdle;
  waitOrThrowForObstacle("turn fallback: " + why);
}

}  // namespace mowgli_nav2_plugins
