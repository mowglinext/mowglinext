// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure "collision_monitor is holding the robot" decision, factored out of
// FTCController so it is unit-testable without ROS (test_ftc_collision_monitor_
// hold.cpp).
//
// FTC's obstacle checks read the LOCAL costmap; collision_monitor reads the raw
// /scan_collision points downstream of FTC and can scale FTC's command to zero
// (its "approach" polygons keep 1.2 s before collision) while every FTC check
// still says the path is clear. Field 2026-10-10: at the dock spike of the
// boundary collision_monitor held the robot for 30 s, FTC kept FOLLOWING a
// stopped robot, and the goal finally died on controller_server's progress
// checker ("Failed to make progress", no obstacle marker), so FollowStrip did
// not detour and the whole ring unit was dropped.
//
// The decision: FTC has been commanding forward motion and the robot has not
// moved for `hold_s` while collision_monitor reports STOP or APPROACH; or, with
// no collision_monitor verdict at all, for the longer `stall_only_s`. The caller
// then hands the situation to its EXISTING WEDGED handling (bounded reverse-
// escape → hold → obstacle-marked abort). Nothing here commands motion.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace mowgli_nav2_plugins
{

struct FtcCmHoldCfg
{
  /// Seconds stopped under a collision_monitor STOP/APPROACH verdict before
  /// FTC treats it as blocked. <= 0 disables this trigger.
  double hold_s = 3.0;
  /// Seconds stopped with no collision_monitor verdict at all (state topic
  /// silent or DO_NOTHING) before FTC treats it as blocked. <= 0 disables.
  /// Must stay well below controller_server's movement_time_allowance.
  double stall_only_s = 10.0;
  /// Forward command below this is not "trying to move" (m/s).
  double min_cmd_speed = 0.05;
  /// Measured forward speed below this fraction of the command is "not moving".
  double moving_ratio = 0.25;
};

/// nav2_msgs/msg/CollisionMonitorState action types that scale the command
/// down to (near) zero. SLOWDOWN / LIMIT still let the robot move.
inline bool CollisionMonitorActionHolds(std::uint8_t action_type)
{
  constexpr std::uint8_t kStop = 1;
  constexpr std::uint8_t kApproach = 3;
  return action_type == kStop || action_type == kApproach;
}

/// Advance the hold timers by one control tick. `cmd_fwd` is the forward speed
/// FTC commanded on the previous tick, `measured_fwd` the odom forward speed now.
/// `held_s` / `stalled_s` are caller-owned state. Returns true once the hold is
/// confirmed; the caller resets both timers when it acts on it.
inline bool CmHoldStep(bool cm_holds,
                       double cmd_fwd,
                       double measured_fwd,
                       double dt,
                       const FtcCmHoldCfg& cfg,
                       double& held_s,
                       double& stalled_s)
{
  const bool stalled =
      cmd_fwd > cfg.min_cmd_speed && std::abs(measured_fwd) < cfg.moving_ratio * cmd_fwd;
  if (!stalled)
  {
    held_s = 0.0;
    stalled_s = 0.0;
    return false;
  }
  const double step = std::max(0.0, dt);
  stalled_s += step;
  held_s = cm_holds ? held_s + step : 0.0;

  constexpr double kTimeEps = 1e-9;
  const bool cm_confirmed = cfg.hold_s > 0.0 && held_s + kTimeEps >= cfg.hold_s;
  const bool stall_confirmed = cfg.stall_only_s > 0.0 && stalled_s + kTimeEps >= cfg.stall_only_s;
  return cm_confirmed || stall_confirmed;
}

/// Plan progress (carrot arc length, m) the robot must make after a confirmed
/// hold before another confirmation counts as a NEW blockage.
inline constexpr double kCmHoldRepeatProgressM = 1.0;

/// The second confirmation of the same blockage. After the first one FTC backs
/// off (bounded reverse) and waits for the path to read clear — which it always
/// does, since the local costmap never saw what collision_monitor sees — so the
/// wait released, its deadline was reset after the probation, and every later
/// confirmation started a fresh wait: a stop / go cycle every ~5 s until the
/// progress checker aborted without the obstacle marker (FollowStrip then never
/// detours). A confirmation within `min_progress_m` of the previous one is the
/// same blockage and must end the goal as an obstacle abort instead.
inline bool CmHoldRepeats(std::optional<double> previous_arc_m,
                          double arc_m,
                          double min_progress_m = kCmHoldRepeatProgressM)
{
  return previous_arc_m.has_value() && std::isfinite(arc_m) &&
         arc_m - *previous_arc_m < min_progress_m;
}

}  // namespace mowgli_nav2_plugins
