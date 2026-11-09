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
// Stall detection for the dock APPROACH, pure and ROS-free.
//
// Field 2026-10-06: the mower drove the final approach along its line, hit the garage
// edge over the dock 0.29 m short of the dock pose, and kept pushing (cmd_vel 0.16 m/s,
// not moving) for 30 s until the approach timeout, then backed off and would have retried
// into the same edge. The docking server cannot tell: SimpleChargingDock's own
// use_stall_detection is CONTACT detection (a stall counts as "docked"), which would book
// pushing against the garage as a successful dock. This is the opposite question: the dock
// server is still driving the approach (feedback state CONTROLLING), the robot is not
// charging, and the chassis neither moves nor turns.
//
// It reads the GNSS-anchored fused pose, not the wheels: a wheel that spins against an
// obstacle reports travel it never made (the same reasoning as the dig detector).

#pragma once

#include <cmath>

namespace mowgli_behavior
{

/// How long the approach may stand still before it is called blocked [s]. The graceful
/// controller's slowest approach speed is 0.16 m/s, so 4 s of no motion is >= 0.6 m of
/// commanded-but-not-travelled path, far above any legitimate pause.
inline constexpr double kDockStallWindowS = 4.0;

/// The chassis counts as having moved when it has travelled this far [m] ...
inline constexpr double kDockStallMinMoveM = 0.03;

/// ... or turned this far [rad] (the approach starts with an in-place alignment). ~5.7 deg.
inline constexpr double kDockStallMinTurnRad = 0.10;

/// A pose older than this is no evidence either way [s]: the detector is reset instead of
/// reading a frozen pose as "not moving".
inline constexpr double kDockStallMaxPoseAgeS = 1.0;

class DockStallDetector
{
public:
  /// Forget everything (the dock server left CONTROLLING, a new attempt began, no pose).
  void Reset()
  {
    has_anchor_ = false;
  }

  /// Feed one pose sample taken while the server is CONTROLLING the approach. Returns true
  /// once the chassis has neither moved nor turned for kDockStallWindowS.
  bool Update(double now_s, double x, double y, double yaw)
  {
    if (!has_anchor_)
    {
      Anchor(now_s, x, y, yaw);
      return false;
    }
    const double moved = std::hypot(x - anchor_x_, y - anchor_y_);
    const double turned = std::fabs(WrapPi(yaw - anchor_yaw_));
    if (moved >= kDockStallMinMoveM || turned >= kDockStallMinTurnRad)
    {
      Anchor(now_s, x, y, yaw);
      return false;
    }
    return now_s - anchor_t_ >= kDockStallWindowS;
  }

private:
  static double WrapPi(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  void Anchor(double t, double x, double y, double yaw)
  {
    has_anchor_ = true;
    anchor_t_ = t;
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_yaw_ = yaw;
  }

  bool has_anchor_{false};
  double anchor_t_{0.0};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};
};

}  // namespace mowgli_behavior
