// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure anti-wheelspin stall decision, factored out of
// FTCController::update_control_point so it is unit-testable without ROS
// (see ftc_controller.cpp, test_ftc_stall.cpp). When the carrot commands a
// forward speed but the robot's ACTUAL forward speed (odom feedback) stays
// well below it for longer than a grace period, the wheels are slipping or
// the chassis is blocked. Rather than ramp to speed_fast and floor it —
// which spins the wheels and digs holes in soft turf (operator report) —
// ease the target down to a slow crawl until traction returns.
//
// The decision also reports `in_stall` so the caller can bound the actual
// COMMANDED velocity: easing the carrot's target speed alone is not enough,
// because the lon PID is floored up to min_speed_mps for smooth normal
// driving and would still push forward into the obstruction. While in_stall
// the caller caps cmd_vel at stall_crawl_speed (bypassing that floor) and
// freezes the carrot so lon_error can't run away ahead of a stuck robot.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins
{

struct FtcStallCfg
{
  double stall_speed_ratio = 0.35;  // <= 0 disables the check
  double stall_grace_s = 0.6;
  double stall_crawl_speed = 0.08;
};

struct FtcStallResult
{
  double target_speed;  ///< Path speed after crawl easing.
  bool in_stall;  ///< True while the crawl clamp is active (past the grace).
};

// Apply stall easing to `target_speed` (the speed already computed from the
// path's straight-ahead lookahead). `cmd_speed` is the controller's
// currently-commanded, already-ramped movement speed — the reference the
// stall check compares the odom-measured forward speed against ("we told
// it to go this fast; is it actually?"). `stall_time` is caller-owned
// debounce state (mirrors AnchorSlewStep's in-place mutation), accumulated
// while stalling and reset to 0 the instant traction returns. Returns the
// eased target plus `in_stall` so the caller can also bound the commanded
// velocity and freeze the carrot (see the header comment above).
inline FtcStallResult StallDecision(double target_speed,
                                    double cmd_speed,
                                    double measured_fwd_speed,
                                    double dt,
                                    const FtcStallCfg& cfg,
                                    double& stall_time)
{
  if (cfg.stall_speed_ratio <= 0.0)
  {
    return {target_speed, false};  // feature disabled.
  }

  const double actual_fwd = std::abs(measured_fwd_speed);
  const bool stalling =
      cmd_speed > cfg.stall_crawl_speed && actual_fwd < cfg.stall_speed_ratio * cmd_speed;
  stall_time = stalling ? (stall_time + dt) : 0.0;

  if (stall_time > cfg.stall_grace_s)
  {
    return {std::min(target_speed, cfg.stall_crawl_speed), true};
  }
  return {target_speed, false};
}

}  // namespace mowgli_nav2_plugins
