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
// How far AHEAD of the robot FTC's virtual carrot may run.
//
// The carrot advances open-loop at the target speed, so whenever something the
// controller cannot see holds the chassis (the dig detector's hard stop and
// bounded reverse, a collision_monitor slowdown) the carrot walks away from it.
// A far carrot is not just a catch-up surge: the robot steers at it along a
// CHORD, and on a curved path a chord of length L leaves the arc of radius R by
// about L^2 / (8 R) — towards the INSIDE of the curve, i.e. towards whatever the
// ring goes around.
//
// Field, 2026-09-17: the cap was a literal 1.0 m. A dig hard-stop at the start
// of the ring around a 1.0 m drawn obstacle let the carrot reach 0.95 m; the
// robot cut the ring 0.41 m inwards, ended 5 cm inside the obstacle's keepout
// band and every transit out answered START_OCCUPIED.
//
// The cap is therefore DERIVED from what the longitudinal loop needs: with a
// proportional loop the steady-state lead at cruise is speed / kp_lon (0.20 m at
// 0.20 m/s, kp_lon 1.0). Anything beyond a modest headroom over that is not
// tracking, it is runaway.

#pragma once

#include <algorithm>

namespace mowgli_nav2_plugins
{

/// Headroom over the steady-state lead, so the ramp to cruise speed and
/// ordinary speed ripple never trip the cap.
inline constexpr double kCarrotLeadHeadroom = 1.5;

/// Floor for the derived cap: below roughly one costmap cell of lead the
/// longitudinal loop has nothing to pull on and the robot would stall.
inline constexpr double kCarrotLeadFloorM = 0.10;

/// Lead cap, metres. `configured_m > 0` is an operator override and is returned
/// as is; otherwise it is derived from the cruise speed and the longitudinal
/// gain. A non-positive gain cannot define a steady-state lead, so the floor is
/// returned rather than an unbounded value.
inline double CarrotMaxLead(double configured_m, double cruise_speed_mps, double kp_lon)
{
  if (configured_m > 0.0)
  {
    return configured_m;
  }
  if (kp_lon <= 0.0 || cruise_speed_mps <= 0.0)
  {
    return kCarrotLeadFloorM;
  }
  return std::max(kCarrotLeadFloorM, kCarrotLeadHeadroom * cruise_speed_mps / kp_lon);
}

/// True when the carrot must NOT advance this tick. Only the LONGITUDINAL lead
/// counts: a lateral offset (an obstacle skirt, or the first metre after a
/// transit hands over beside the line) is not the carrot running away, and
/// freezing on it would park the robot next to its path.
inline bool CarrotLeadExceeded(double carrot_x_in_robot_frame_m, double max_lead_m)
{
  return carrot_x_in_robot_frame_m > max_lead_m;
}

}  // namespace mowgli_nav2_plugins
