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
// FTCController's lateral obstacle deviation: the legacy single-offset skirt
// and the whole-profile offset lattice (ftc_offset_lattice.hpp,
// ftc_lattice_solver.hpp), the zone guard they are confined by, and the plan
// window they are solved on.

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

void FTCController::updateLateralDeviation(double dt)
{
  // Bail if no costmap or path — tests sometimes run without one of either.
  if (costmap_map_ == nullptr || global_plan_.empty())
  {
    return;
  }

  // Lock the costmap for the duration — the ObstacleDeviation helpers below
  // (isPathClearWithDeviation / findFirstObstacleIndex / chooseDeviationSide)
  // all read costmap cells and would otherwise race the costmap update thread.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_map_->getMutex());

  const std::size_t start_idx =
      std::min(static_cast<std::size_t>(current_index_), global_plan_.size() - 1);

  // Sample obstacles in the COSTMAP frame, not the path frame. global_plan_ is
  // in the plan frame (map); costmap_map_ is the local costmap in its own
  // global frame (odom). map->odom is NOT identity here — fusion_graph_node
  // publishes it and it absorbs all GPS corrections — so feeding raw map-frame
  // path coords into worldToMap() samples the WRONG cells (almost always free /
  // off-window) and the obstacle is never detected: the robot drives the
  // nominal line straight into it, while collision_monitor (which reads
  // /scan_costmap in the robot frame) is the only thing that reacts. Transform
  // the lookahead window into the costmap frame BEFORE sampling, then index it
  // from 0 (the ObstacleDeviation helpers clamp to the window size).
  std::vector<geometry_msgs::msg::PoseStamped> window;
  // A pivot corner ends the leg: poses past it have another heading, so a
  // lateral offset (and "which side to skirt") means something else there.
  // The next leg is examined once the robot has pivoted onto it.
  const std::optional<std::size_t> corner = nextPivotCorner();
  {
    const std::size_t win_end =
        ClipWindowToCorner(start_idx,
                           std::min(global_plan_.size(),
                                    start_idx + static_cast<std::size_t>(
                                                    std::max(0, config_.obstacle_lookahead))),
                           corner);
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    const std::string plan_frame = global_plan_[start_idx].header.frame_id;
    window.reserve(win_end - start_idx);
    if (plan_frame.empty() || plan_frame == costmap_frame)
    {
      window.assign(global_plan_.begin() + static_cast<std::ptrdiff_t>(start_idx),
                    global_plan_.begin() + static_cast<std::ptrdiff_t>(win_end));
    }
    else
    {
      geometry_msgs::msg::TransformStamped plan_to_costmap;
      try
      {
        plan_to_costmap =
            tf_buffer_->lookupTransform(costmap_frame, plan_frame, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex)
      {
        // Without the transform the path cannot be checked for obstacles at
        // all. Never drive on unchecked (that is what "skip avoidance this
        // tick" used to do) and never fall back to the broken raw-coord
        // sampling: fail like the carrot's own TF lookup does.
        throw nav2_core::ControllerException("FTCController: obstacle-check TF " + plan_frame +
                                             "->" + costmap_frame + " failed: " + ex.what());
      }
      for (std::size_t i = start_idx; i < win_end; ++i)
      {
        geometry_msgs::msg::PoseStamped p;
        tf2::doTransform(global_plan_[i], p, plan_to_costmap);
        window.push_back(p);
      }
    }
  }

  // Build the zone-boundary guard for the lateral-OFFSET checks only. The
  // offset sample points are the window poses, which were transformed into the
  // local-costmap (odom) frame above; the boundary costmap lives in the global-
  // costmap (map) frame. So the guard's affine maps boundary_frame <- odom
  // (map <- odom). An offset that would skirt the robot out of the mowing zone
  // (lethal in the boundary costmap) is then rejected; if the only obstacle-
  // clear side exits the zone, growDeviationUntilClear exceeds the cap and the
  // existing wait-or-abort path stops the robot instead of leaving. The mutex
  // is held for the rest of the function so the helpers can read boundary_costmap_.
  std::unique_lock<std::mutex> boundary_lock(boundary_mutex_, std::defer_lock);
  ObstacleDeviation::BoundaryGuard guard{};
  if (config_.confine_deviation_to_zone)
  {
    boundary_lock.lock();
    if (!buildBoundaryGuard(guard))
    {
      // The zone is unknown: obstacles are still DETECTED (that never needed
      // the zone), but no offset may leave the line — an obstacle then takes
      // the wait / reverse / abort path instead of a blind skirt. Returning
      // here used to skip detection altogether.
      guard = ObstacleDeviation::BoundaryGuard{};
      guard.deny_all = true;
    }
  }

  // Robot chassis FOOTPRINT (base frame), fetched once per tick when
  // use_footprint_clearance is on. Passed to the ObstacleDeviation helpers so
  // they sample the true rectangular body (at true-lethal 254) instead of the
  // ±half_width swept line (at inscribed 253). Detection uses the raw footprint;
  // the CLEARANCE search uses a laterally-expanded copy (+obstacle_clearance_
  // margin) so pass-by room grows without widening detection reach — the exact
  // split the half-width model expressed via clearanceHalfWidth(). An EMPTY
  // footprint (feature off, or none published) makes every helper fall back to
  // the half_width line model, so behaviour is unchanged in that case.
  ObstacleDeviation::Footprint detect_footprint;
  ObstacleDeviation::Footprint clearance_footprint;
  if (config_.use_footprint_clearance)
  {
    detect_footprint = costmap_ros_->getRobotFootprint();
    // Clearance (skirt) search probes only the FRONT of the chassis, then
    // widens it laterally by the margin — the "less-conservative footprint"
    // middle ground (spec Part A). Detection keeps the full footprint above.
    clearance_footprint = ObstacleDeviation::expandFootprintLateral(
        ObstacleDeviation::clipFootprintFront(detect_footprint,
                                              config_.obstacle_footprint_front_length_m),
        std::max(0.0, config_.obstacle_clearance_margin));
  }

  // Whole-profile planner (ftc_offset_lattice.hpp). It needs the real chassis
  // polygon; without one it falls back to the legacy single-offset search.
  const bool use_lattice = config_.use_offset_lattice && detect_footprint.size() >= 3;
  // Carrot capped ON a pivot corner: nothing of this leg lies ahead of it, and
  // no offset may be carried through the pivot. Bring any skirt back to the
  // line now (the slew below). The lattice planned the corner pose at zero
  // offset as a hard constraint, so this is the profile it was already
  // converging to; for both planners the arrival gate, currentBodyInLethal and
  // the pivot-sweep gate hold the robot if the body would touch anything.
  const bool carrot_on_corner = corner.has_value() && start_idx >= *corner;
  if (carrot_on_corner)
  {
    target_lateral_deviation_ = 0.0;
    lattice_return_start_.reset();
  }
  else if (use_lattice)
  {
    const ObstacleDeviation::Footprint body =
        ObstacleDeviation::expandFootprintLateral(detect_footprint,
                                                  std::max(0.0, config_.obstacle_clearance_margin));
    if (!planOffsetLattice(start_idx, guard, body, dt))
    {
      return;
    }
  }
  if (!use_lattice && !carrot_on_corner)
  {
    // The decision to STOP avoiding must be gated on whether the NOMINAL path
    // (zero deviation) is clear within the lookahead — i.e. has the robot
    // advanced far enough that the obstacle has left the forward window? It
    // must NOT be gated on whether the currently-applied offset is clear:
    // that is trivially true the instant we pick a clearing offset, so the old
    // code declared "AVOIDANCE complete" ~0.2 s after entering, blended the
    // offset back to ~0, re-detected the same obstacle, and re-entered — an
    // endless flap at a tiny ±deviation_step offset (logged as repeated
    // "entering AVOIDANCE ... at idx=N" / "AVOIDANCE complete" pairs at the
    // same idx). The robot never offset enough to skirt anything; the
    // sub-deadband ±step carrot shift just dithered it left-right in place.
    // Body-aware (footprint, else ±obstacle_body_half_width), NOT just the path
    // centerline — otherwise an obstacle in the lateral band the chassis hits but
    // the inscribed-inflation radius misses never flips clear_at_zero false, so
    // avoidance never engages. No zone guard here: this asks "does the body hit an
    // obstacle on the nominal line", independent of the mowing-zone boundary. A
    // lethal LOCAL cell is an obstacle even where the global keepout costmap is
    // also lethal — that subtraction is what hid two mapped trees on 2026-09-16
    // (ObstacleDeviation::isObstacleCell).
    const bool clear_at_zero =
        ObstacleDeviation::isPathClearWithDeviation(*costmap_map_,
                                                    window,
                                                    0,
                                                    config_.obstacle_lookahead,
                                                    0.0,
                                                    ObstacleDeviation::BoundaryGuard{},
                                                    config_.obstacle_body_half_width,
                                                    detect_footprint);

    if (clear_at_zero)
    {
      // Nominal path is clear ahead — the wedge (if any) is gone. Cancel any
      // reverse-escape in progress so it can't leak forward motion into a fresh
      // block, and hand a full budget to the next genuine wedge.
      reverse_escape_active_ = false;
      reverse_distance_done_ = 0.0;
      reverse_elapsed_s_ = 0.0;
      if (is_avoiding_)
      {
        // The nominal path reads clear — but the obstacle sits at the
        // lookahead-window edge and the observation_persistence:0 costmap
        // re-marks it each scan, so this test flickers true/false. Blending the
        // skirt back at the FIRST clear tick (the old behaviour) caused the
        // ±step left-right flap: complete → re-enter on the other side, never
        // growing a deviation big enough to actually go around. HOLD the
        // committed skirt until the path has stayed clear CONTINUOUSLY for
        // obstacle_clear_hold_s — i.e. the robot has physically passed the
        // obstacle — then blend back and finish.
        if (!avoidance_clear_start_.has_value())
        {
          avoidance_clear_start_ = clock_->now();
        }
        const double clear_for = (clock_->now() - avoidance_clear_start_.value()).seconds();
        if (clear_for >= config_.obstacle_clear_hold_s)
        {
          target_lateral_deviation_ = 0.0;
          if (std::abs(lateral_deviation_) < 0.01)
          {
            is_avoiding_ = false;
            avoidance_clear_start_.reset();
            RCLCPP_INFO(logger_,
                        "FTCController: AVOIDANCE complete (path clear for %.1fs), back on path.",
                        clear_for);
          }
        }
        // else: keep target_lateral_deviation_ at its committed value — hold the
        // skirt through the flicker; do NOT zero it yet.
      }
      else
      {
        // Not avoiding and the path is clear: nominal line tracking. A pending
        // wait is released by the single continuous-followable debounce below,
        // shared with the valid-skirt case.
        target_lateral_deviation_ = 0.0;
      }
    }
    else
    {
      // Obstacle (re)appeared on the nominal path — still committed. Cancel the
      // nominal-path clear hold so a brief clear gap between scans doesn't count
      // toward avoidance completion. The separate followable-skirt debounce is
      // reset only if the clearance search fails (inside waitOrThrowForObstacle),
      // allowing a continuously valid skirt to release a pending wait.
      avoidance_clear_start_.reset();
      // Obstacle present on the nominal path within the lookahead. Commit to a
      // deviation that keeps the OFFSET path clear and HOLD it until the robot
      // has passed the obstacle (clear_at_zero becomes true). The deviation is
      // monotonically non-decreasing while the obstacle remains — we never
      // reduce it toward the path here, which is what stopped the flap.
      if (!is_avoiding_)
      {
        const int obs_idx =
            ObstacleDeviation::findFirstObstacleIndex(*costmap_map_,
                                                      window,
                                                      0,
                                                      config_.obstacle_lookahead,
                                                      config_.obstacle_body_half_width,
                                                      detect_footprint);
        if (obs_idx < 0)
        {
          // Footprint collision but no path-pose hit (e.g. inflated cell next
          // to robot from a transient scan return) — nothing to deviate around.
          return;
        }
        // Cul-de-sac guard (spec Part A): only skirt an obstacle whose FAR edge is
        // visible inside the lookahead. If the obstacle stays blocked to the end of
        // the window (a wall / pocket), skirting sideways boxes the robot in — the
        // exact wedge this spec targets. Refuse the skirt and hand off to the
        // bounded reverse-escape / wait-or-abort path; the coverage detour-and-
        // continue net (decideDetour) then routes a blade-off transit around it.
        if (config_.require_clear_exit &&
            !ObstacleDeviation::hasClearExit(*costmap_map_,
                                             window,
                                             0,
                                             config_.obstacle_lookahead,
                                             config_.obstacle_body_half_width,
                                             detect_footprint))
        {
          if (reverseEscapeOrWait("no clear exit past obstacle — refusing to skirt into a pocket",
                                  detect_footprint,
                                  dt))
          {
            return;
          }
          // Wait window elapsed without the far edge appearing — throw to abort the
          // strip (waitOrThrowForObstacle throws once past the timeout), letting the
          // BT escalate to the coverage detour. Returning here would re-enter this
          // same branch every tick. reverseEscapeOrWait only returns false after it
          // has thrown, so this line is unreachable, but keep the return for safety.
          return;
        }
        // Side choice is a CLEARANCE question ("which side has room for the
        // body plus margin to pass"), so it uses the widened footprint (or
        // half-width) — not the bare detection body used by findFirstObstacleIndex
        // above.
        target_lateral_deviation_ =
            ObstacleDeviation::chooseDeviationSide(*costmap_map_,
                                                   window[static_cast<std::size_t>(obs_idx)],
                                                   config_.max_lateral_deviation,
                                                   config_.deviation_step,
                                                   guard,
                                                   clearanceHalfWidth(),
                                                   clearance_footprint);
        if (target_lateral_deviation_ == 0.0)
        {
          // Both sides blocked at the obstacle pose. Before bailing, try a
          // bounded straight reverse-escape (rear footprint permitting), then
          // fall back to holding a wait window so transient costmap state (LIDAR
          // noise, a person crossing the path, inflation around the dock not yet
          // cleared by post-undock observations) can clear without burning a BT
          // retry.
          if (reverseEscapeOrWait("obstacle blocks both sides, cannot skirt", detect_footprint, dt))
          {
            return;
          }
        }
        is_avoiding_ = true;
        // Latch the chosen side for the whole episode. target is guaranteed
        // nonzero here (the both-sides-blocked path above either waits and
        // returns or throws, so we never reach this with target == 0).
        avoid_sign_ = (target_lateral_deviation_ >= 0.0) ? 1.0 : -1.0;
        // Do NOT clear obstacle_wait_start_/obstacle_waiting_ here. Finding a
        // candidate side only means we skip the both-sides-blocked wait call
        // below (line ~1659 above) — growDeviationUntilClear (below) can still
        // reject this same candidate for exceeding max_lateral_deviation and
        // re-enter waitOrThrowForObstacle. Clearing the clock here first would
        // hand that second call a fresh 5s window every time chooseDeviationSide
        // flip-flops between "found a side" and "both sides blocked" near a
        // marginal gap — silently deferring the abort indefinitely (field:
        // observed ~40s stall, cmd_vel pinned at zero, vs. the intended 5s cap).
        // The wait state is cleared once, below, only after a candidate has
        // genuinely passed the max_lateral_deviation check.
        RCLCPP_INFO(logger_,
                    "FTCController: entering AVOIDANCE (target_dev=%.2fm at idx=%d)",
                    target_lateral_deviation_,
                    static_cast<int>(start_idx) + obs_idx);
      }

      // Floor the SEARCH START to min_lateral_deviation. growDeviationUntilClear
      // now samples the full chassis width (±obstacle_body_half_width per pose),
      // so a one-step offset can no longer "clear" the centerline while the body
      // still overlaps the obstacle. This floor is retained as a secondary guard:
      // it makes AVOIDANCE commit to a real, human-visible skirt (≥ a body
      // half-width + margin) rather than a sub-deadband 5 cm carrot nudge, and it
      // gives margin beyond the exact body edge. grow still increases past min (or
      // reports > max) if min itself is blocked — so this never forces the carrot
      // into an obstacle the way a blind post-grow floor would, since clearance is
      // not monotonic in the offset.
      double dev_init = target_lateral_deviation_;
      if (config_.min_lateral_deviation > 0.0 && std::abs(dev_init) < config_.min_lateral_deviation)
      {
        // Use the LATCHED avoidance side, not the sign of dev_init: a transient
        // clear_at_zero tick can leave dev_init == 0 mid-episode, and deriving
        // the sign from it would flip the skirt onto the blocked side. Clamp
        // the floor to max so a min > max misconfig can't make grow start past
        // the cap (which would abort the strip on every obstacle).
        const double floor_mag =
            std::min(config_.min_lateral_deviation, config_.max_lateral_deviation);
        dev_init = avoid_sign_ * floor_mag;
      }

      // Grow the deviation until the offset path is clear (keeps current side).
      target_lateral_deviation_ =
          ObstacleDeviation::growDeviationUntilClear(*costmap_map_,
                                                     window,
                                                     0,
                                                     config_.obstacle_lookahead,
                                                     dev_init,
                                                     config_.max_lateral_deviation,
                                                     config_.deviation_step,
                                                     guard,
                                                     clearanceHalfWidth(),
                                                     clearance_footprint);

      if (std::abs(target_lateral_deviation_) > config_.max_lateral_deviation)
      {
        // Same reverse-escape-then-wait as the both-sides-blocked case. If the
        // obstacle is transient, the next tick will pull target_dev back
        // under the cap and we resume cleanly.
        if (reverseEscapeOrWait("lateral deviation needed > max_lateral_deviation",
                                detect_footprint,
                                dt))
        {
          return;
        }
      }
    }

    // Path is now followable inside the deviation cap. If a previous tick put us
    // into a hard zero hold, require this result to remain continuously valid
    // before moving again. A single scan can momentarily erase an obstacle cell;
    // immediately clearing the wait here caused the field-observed stop/go loop.
  }

  if (obstacle_waiting_)
  {
    if (!ObstacleWaitReadyToResume(
            true, dt, config_.obstacle_clear_hold_s, obstacle_followable_time_))
    {
      holdObstacleMotion();
      return;
    }
    RCLCPP_INFO(logger_, "FTCController: obstacle cleared, resuming after wait.");
    obstacle_waiting_ = false;
    obstacle_followable_time_ = 0.0;
  }
  else if (obstacle_wait_start_.has_value())
  {
    // Keep the original wait start through a moving probation window. If the
    // obstacle reappears before the path has remained stable for a second
    // clear-hold period, waitOrThrowForObstacle sees the original episode's
    // deadline instead of granting another complete wait/restart cycle.
    if (ObstacleWaitReadyToResume(
            true, dt, config_.obstacle_clear_hold_s, obstacle_followable_time_))
    {
      RCLCPP_INFO(logger_, "FTCController: obstacle recovery stable; ending restart limits.");
      obstacle_wait_start_.reset();
      obstacle_followable_time_ = 0.0;
      obstacle_recovery_active_ = false;
      last_recovery_angular_cmd_ = 0.0;
    }
  }
  else
  {
    obstacle_followable_time_ = 0.0;
  }

  // The path is followable THIS tick. That used to cancel an engaged
  // reverse-escape at once and hand it a fresh budget — with feasibility
  // flickering at the edge of what fits, the robot wagged reverse / forward at
  // 5–10 Hz on the spot and the 0.30 m bound was never spent (282 engagements in
  // 3 minutes at one path index, field 2026-09-20). The escape is now a
  // committed manoeuvre, and its budget has to be earned back by real progress.
  if (reverse_escape_active_)
  {
    const bool can_continue =
        reverseEscapeStep(costmap_ros_->getRobotFootprint(), dt) == ReverseEscapeAction::kReverse;
    if (!ReverseEscapeShouldRelease(
            can_continue, dt, config_.obstacle_clear_hold_s, reverse_followable_time_))
    {
      return;  // keep backing out; computeVelocityCommands emits the reverse
    }
    RCLCPP_INFO(logger_,
                "FTCController: reverse-escape released after %.2fm (%s) — following the profile.",
                reverse_distance_done_,
                can_continue ? "profile held" : "budget spent or rear blocked");
    reverse_escape_active_ = false;
    reverse_followable_time_ = 0.0;
  }
  if (reverse_budget_touched_)
  {
    ReverseEscapeCfg refill_cfg;
    refill_cfg.max_dist_m = config_.obstacle_reverse_max_dist_m;
    if (ReverseBudgetEarnsRefill(refill_cfg, progressSinceReverseEngaged()))
    {
      reverse_distance_done_ = 0.0;
      reverse_elapsed_s_ = 0.0;
      reverse_budget_touched_ = false;
    }
  }

  // Step 2: slew lateral_deviation_ toward target_lateral_deviation_ at the
  // configured blend rate (m/s of lateral shift).
  const double max_step = config_.deviation_blend_rate * dt;
  const double delta = target_lateral_deviation_ - lateral_deviation_;
  lateral_deviation_ += std::clamp(delta, -max_step, max_step);
}

bool FTCController::buildBoundaryGuard(BoundaryGuard& guard)
{
  if (boundary_costmap_ == nullptr)
  {
    // Fail-safe: global costmap not received yet — we cannot know where the
    // zone boundary is, so refuse to deviate blind (skip this tick, same
    // posture as the TF-missing path).
    RCLCPP_WARN_THROTTLE(logger_,
                         *clock_,
                         5000,
                         "FTCController: confine_deviation_to_zone but no global costmap "
                         "received yet — obstacles are detected but the robot will not leave "
                         "its line to skirt them.");
    return false;
  }
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  try
  {
    const auto tf = tf_buffer_->lookupTransform(boundary_frame_, costmap_frame, tf2::TimePointZero);
    const double yaw = tf2::getYaw(tf.transform.rotation);
    guard.costmap = boundary_costmap_.get();
    guard.tx = tf.transform.translation.x;
    guard.ty = tf.transform.translation.y;
    guard.cos_yaw = std::cos(yaw);
    guard.sin_yaw = std::sin(yaw);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(logger_,
                         *clock_,
                         5000,
                         "FTCController: no transform %s <- %s for boundary guard (%s) — "
                         "obstacles are detected but the robot will not leave its line.",
                         boundary_frame_.c_str(),
                         costmap_frame.c_str(),
                         ex.what());
    return false;
  }
  return true;
}

LatticeSolverCfg FTCController::latticeSolverCfg() const
{
  LatticeSolverCfg cfg;
  cfg.lattice.offset_step = std::max(0.01, config_.deviation_step);
  cfg.lattice.max_offset = std::max(0.0, config_.max_lateral_deviation);
  cfg.station_spacing_m =
      OffsetLatticeStationSpacing(cfg.lattice.offset_step, config_.avoidance_max_slope);
  cfg.lead_m = CarrotMaxLead(config_.carrot_max_lead, config_.speed_fast, config_.kp_lon);
  cfg.reaction_m = config_.avoidance_reaction_m;
  cfg.min_horizon_m = config_.avoidance_min_horizon_m;
  return cfg;
}

bool FTCController::planWindowInCostmapFrame(std::size_t first,
                                             std::size_t last,
                                             std::vector<geometry_msgs::msg::PoseStamped>& out)
{
  out.clear();
  last = std::min(last, global_plan_.size());
  if (first >= last)
  {
    return true;
  }
  out.reserve(last - first);
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const std::string plan_frame = global_plan_[first].header.frame_id;
  if (plan_frame.empty() || plan_frame == costmap_frame)
  {
    out.assign(global_plan_.begin() + static_cast<std::ptrdiff_t>(first),
               global_plan_.begin() + static_cast<std::ptrdiff_t>(last));
    return true;
  }
  try
  {
    const auto plan_to_costmap =
        tf_buffer_->lookupTransform(costmap_frame, plan_frame, tf2::TimePointZero);
    for (std::size_t i = first; i < last; ++i)
    {
      geometry_msgs::msg::PoseStamped p;
      tf2::doTransform(global_plan_[i], p, plan_to_costmap);
      out.push_back(p);
    }
  }
  catch (const tf2::TransformException& ex)
  {
    // Same rule as the legacy window: no transform, no obstacle check, no motion.
    throw nav2_core::ControllerException("FTCController: offset-lattice TF " + plan_frame + "->" +
                                         costmap_frame + " failed: " + ex.what());
  }
  return true;
}

bool FTCController::planOffsetLattice(std::size_t carrot_idx,
                                      const BoundaryGuard& guard,
                                      const std::vector<geometry_msgs::msg::Point>& footprint,
                                      double dt)
{
  const LatticeSolverCfg solver_cfg = latticeSolverCfg();
  const double ds = solver_cfg.station_spacing_m;

  // The carrot is station 0, but the ROBOT trails it by up to the carrot lead
  // cap, and its lateral loop chases the carrot's offset. A node is therefore
  // only free when the body fits at that offset on every pose from `lead`
  // behind the station up to the station itself — otherwise the profile comes
  // back to the line as soon as the CARROT has passed the obstacle and pulls
  // the chassis, still beside it, into it.
  const double lead = solver_cfg.lead_m;

  // Resample the plan at `ds`, from `lead` behind the carrot to the horizon —
  // within the current LEG only: behind the carrot no further back than the
  // last pivot corner, ahead no further than the next one (the heading, and so
  // every lateral offset, is discontinuous across a pivot). A pivot corner
  // inside the horizon becomes the last station and must be planned at ZERO
  // offset: no skirt is carried through a pivot.
  const auto [leg_first, leg_last] = PivotLeg(pivot_corners_, carrot_idx, global_plan_.size());
  const LatticeWindow window = ResampleLatticeWindow(global_plan_,
                                                     carrot_idx,
                                                     leg_first,
                                                     leg_last,
                                                     nextPivotCorner(),
                                                     ds,
                                                     lead,
                                                     config_.avoidance_horizon_m);
  const bool corner_is_last_station = window.corner_is_last_station;
  if (window.pose_idx.empty())
  {
    return true;  // no window (callers never pass an empty plan): keep the target
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  {
    std::vector<geometry_msgs::msg::PoseStamped> span;
    if (!planWindowInCostmapFrame(window.pose_idx.front(), window.pose_idx.back() + 1, span))
    {
      return true;  // no TF this tick: keep the current target, do not escape
    }
    poses.reserve(window.pose_idx.size());
    for (const std::size_t i : window.pose_idx)
    {
      poses.push_back(span[i - window.pose_idx.front()]);
    }
  }

  // Which (pose, offset) nodes the body cannot occupy, and the degrading solve
  // (reaction slack -> none -> ignore the stations under the body, each over a
  // shrinking horizon): ftc_lattice_solver.hpp, shared with the offline replay
  // tests so they exercise exactly this decision.
  // The bare chassis: what level 3 tests the stations under the body with.
  const ObstacleDeviation::Footprint contact = costmap_ros_->getRobotFootprint();
  LatticeSolver solver(*costmap_map_,
                       guard,
                       footprint,
                       std::move(poses),
                       window.carrot_pos,
                       corner_is_last_station,
                       solver_cfg,
                       contact);
  const std::vector<double>& stations = solver.Stations();
  const auto stations_in = [&solver](double metres)
  {
    return solver.StationsIn(metres);
  };

  const int preferred = is_avoiding_ ? (avoid_sign_ >= 0.0 ? 1 : -1) : 0;
  OffsetLatticeResult plan;
  int plan_level = 0;
  std::size_t planned_stations = stations.size();
  // `only_side` != 0 forbids every offset on the other side (hard), used below
  // to ask "is the side we committed to still passable?".
  const auto solve = [&](int only_side)
  {
    const LatticeSolution sol = solver.Solve(lateral_deviation_, preferred, only_side);
    plan = sol.plan;
    plan_level = sol.level;
    planned_stations = sol.planned_stations;
  };
  solve(0);

  // COMMITTED SIDE. The soft latch inside the DP only holds if the preference it
  // is given is itself stable; following the plan's side tick by tick makes it
  // free to flip. With someone stepping in front of the robot — two thin legs,
  // moving, re-marked from scratch every scan, left and right equally cheap —
  // the side flipped at 10 Hz: the robot stopped and wagged left-right before
  // one side finally won (operator report 2026-09-17). Once engaged the side is
  // kept for as long as it stays passable; the other side is only adopted when
  // the committed one is closed, or after the free plan has asked for it
  // continuously for obstacle_clear_hold_s.
  if (plan.feasible && is_avoiding_)
  {
    double free_extreme = 0.0;
    for (const double o : plan.offsets)
    {
      free_extreme = std::fabs(o) > std::fabs(free_extreme) ? o : free_extreme;
    }
    const int committed = avoid_sign_ >= 0.0 ? 1 : -1;
    const bool wants_other_side = free_extreme * static_cast<double>(committed) < -1e-6;
    if (!wants_other_side)
    {
      lattice_switch_start_.reset();
    }
    else
    {
      if (!lattice_switch_start_.has_value())
      {
        lattice_switch_start_ = clock_->now();
      }
      const bool debounced = (clock_->now() - lattice_switch_start_.value()).seconds() >=
                             config_.obstacle_clear_hold_s;
      if (!debounced)
      {
        const OffsetLatticeResult free_plan = plan;
        const int free_level = plan_level;
        const std::size_t free_stations = planned_stations;
        solve(committed);
        if (!plan.feasible)
        {
          // The committed side is closed: switching is not a preference any more.
          plan = free_plan;
          plan_level = free_level;
          planned_stations = free_stations;
          lattice_switch_start_.reset();
          avoid_sign_ = -avoid_sign_;
          RCLCPP_INFO(logger_, "FTCController: lattice committed side is closed — switching side.");
        }
      }
      else
      {
        lattice_switch_start_.reset();
        avoid_sign_ = -avoid_sign_;
        RCLCPP_INFO(
            logger_,
            "FTCController: lattice switching side (the other side stayed cheaper for %.1fs).",
            config_.obstacle_clear_hold_s);
      }
    }
  }
  else if (!is_avoiding_)
  {
    lattice_switch_start_.reset();
  }

  if (plan.feasible && planned_stations < stations.size())
  {
    RCLCPP_INFO_THROTTLE(logger_,
                         *clock_,
                         2000,
                         "FTCController: lattice horizon cut to %.2fm of %.2fm (blocked across the "
                         "whole lattice further on) — continuing",
                         stations[planned_stations - 1],
                         stations.back());
  }
  if (plan.feasible && plan_level > 1)
  {
    RCLCPP_WARN_THROTTLE(logger_,
                         *clock_,
                         2000,
                         "FTCController: lattice profile found only at fallback level %d (%s)",
                         plan_level,
                         plan_level == 2 ? "no reaction slack"
                                         : "bare chassis under the body, no clearance margin");
  }

  if (!plan.feasible)
  {
    lattice_return_start_.reset();
    // A blockage IN A TURN of the plan (the hedge past the recorded line at a
    // U-turn, field 2026-09-22) is improvised around instead: reverse, pivot,
    // straight, pivot, rejoin. Anything else — or a turn with no safe way
    // round — takes the WEDGED path below, unchanged.
    if (tryTurnFallback(carrot_idx, guard, footprint))
    {
      return false;
    }
    const ObstacleDeviation::Footprint detect = costmap_ros_->getRobotFootprint();
    reverseEscapeOrWait("no collision-free offset profile within the lattice", detect, dt);
    return false;
  }

  // Signed extreme of the profile: which side this plan commits to.
  double extreme = 0.0;
  for (const double o : plan.offsets)
  {
    extreme = std::fabs(o) > std::fabs(extreme) ? o : extreme;
  }

  if (std::fabs(extreme) > 1e-6 || std::fabs(lateral_deviation_) > 0.01)
  {
    RCLCPP_INFO_THROTTLE(logger_,
                         *clock_,
                         2000,
                         "FTCController: lattice profile peak=%.2fm cost=%.3f over %zu stations "
                         "(%.2fm), applied=%.2fm target=%.2fm",
                         extreme,
                         plan.cost,
                         stations.size(),
                         stations.back(),
                         lateral_deviation_,
                         target_lateral_deviation_);
  }

  // Offset to hold NOW: the profile previewed one carrot LEAD ahead. Taking it
  // at the carrot itself starts every skirt at the last station that still
  // clears — the plan is then only ever executed from its "no slack" fallback
  // (replay 2026-09-17). Previewing by the distance the chassis trails the
  // carrot makes the body, not the carrot, follow the profile; the blend rate
  // below turns the step into a smooth lateral move.
  const std::size_t preview =
      std::min(plan.offsets.size() - 1, std::max<std::size_t>(1, stations_in(lead)));
  const double wanted = plan.offsets[preview];

  // Moving AWAY from the line is applied at once. Coming BACK is debounced: the
  // costmap keeps no memory (observation_persistence 0), so an obstacle that
  // drops out of one scan would otherwise start a return into it.
  const bool is_return = std::fabs(wanted) < std::fabs(target_lateral_deviation_) - 1e-6;
  // A pivot corner ends this leg at ZERO offset. Once the path left to it is no
  // longer than the return itself needs (slope-limited, plus the reaction
  // slack), holding a planned return back for the debounce would only turn a
  // feasible, collision-checked profile into a WEDGED one a few ticks later.
  const bool corner_forces_return =
      corner_is_last_station && planned_stations == stations.size() &&
      stations.back() <=
          std::fabs(target_lateral_deviation_) / std::max(1e-3, config_.avoidance_max_slope) +
              config_.avoidance_reaction_m;
  if (!is_return || corner_forces_return)
  {
    lattice_return_start_.reset();
    target_lateral_deviation_ = wanted;
  }
  else
  {
    if (!lattice_return_start_.has_value())
    {
      lattice_return_start_ = clock_->now();
    }
    if ((clock_->now() - lattice_return_start_.value()).seconds() >= config_.obstacle_clear_hold_s)
    {
      target_lateral_deviation_ = wanted;
    }
  }

  // AVOIDING is sticky: with no memory in the costmap an obstacle at range drops
  // out of individual scans, and a plan that reads "all clear" for one tick must
  // not release the committed side. The state ends once the profile has stayed
  // flat, with nothing applied, for obstacle_clear_hold_s.
  const bool engaged = std::fabs(extreme) > 1e-6 || std::fabs(lateral_deviation_) > 0.01 ||
                       std::fabs(target_lateral_deviation_) > 1e-6;
  bool avoiding_now = engaged;
  if (engaged)
  {
    avoidance_clear_start_.reset();
  }
  else if (is_avoiding_)
  {
    if (!avoidance_clear_start_.has_value())
    {
      avoidance_clear_start_ = clock_->now();
    }
    avoiding_now =
        (clock_->now() - avoidance_clear_start_.value()).seconds() < config_.obstacle_clear_hold_s;
  }

  // The side is committed when the avoidance ENGAGES; afterwards only the block
  // above may change it.
  if (!is_avoiding_ && std::fabs(extreme) > 1e-6)
  {
    avoid_sign_ = (extreme >= 0.0) ? 1.0 : -1.0;
  }
  if (avoiding_now && !is_avoiding_)
  {
    RCLCPP_INFO(logger_,
                "FTCController: entering AVOIDANCE (lattice, peak offset %.2fm, cost %.3f, at "
                "idx=%zu)",
                extreme,
                plan.cost,
                carrot_idx);
  }
  else if (!avoiding_now && is_avoiding_)
  {
    lattice_return_start_.reset();
    avoidance_clear_start_.reset();
    RCLCPP_INFO(logger_, "FTCController: AVOIDANCE complete (lattice), back on path.");
  }
  is_avoiding_ = avoiding_now;
  return true;
}

void FTCController::applyLateralDeviationToCarrot()
{
  if (lateral_deviation_ == 0.0)
  {
    return;
  }
  // Shift the carrot's translation in its own y-axis (left of heading).
  const Eigen::Vector3d lateral(0.0, lateral_deviation_, 0.0);
  current_control_point_.translation() += current_control_point_.linear() * lateral;
}

void FTCController::debugObstacle(
    visualization_msgs::msg::Marker& marker, double x, double y, unsigned char cost, int max_ids)
{
  if (marker.points.empty())
  {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.header.stamp = clock_->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
  }

  marker.id = static_cast<int>(marker.points.size()) + 1;

  if (cost < 127u)
  {
    marker.color.g = 1.0f;
    marker.color.r = 0.0f;
  }
  else if (cost < 255u)
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
  }
  marker.color.a = 1.0f;

  geometry_msgs::msg::Point p;
  costmap_map_->mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y), p.x, p.y);
  p.z = 0.0;
  marker.points.push_back(p);

  if (static_cast<int>(marker.points.size()) >= max_ids || cost > 0u)
  {
    obstacle_marker_pub_->publish(marker);
    marker.points.clear();
  }
}

}  // namespace mowgli_nav2_plugins
