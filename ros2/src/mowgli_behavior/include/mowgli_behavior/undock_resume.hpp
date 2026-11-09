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
// Resuming an interrupted undock (#879 review).
//
// A Pause (or an emergency, or a failed BackUp) can stop the undock reverse
// after the robot has left the charge contacts. On the next Start the robot is
// off the dock, so the tree used to take NotDockedBranch and run
// SeedYawFromMotion — a 1 m FORWARD drive, while the robot still faces the dock
// it was reversing out of: it drove back onto the dock and pushed against it
// until the seed's timeout. Instead, an interrupted undock is finished: the
// remaining part of the reverse, measured from the GNSS position recorded by
// RecordUndockStart, then the usual heading calibration.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_behavior
{

/// Below this much remaining reverse [m] the undock counts as finished: a
/// shorter remainder is within the GNSS noise of the measurement itself.
constexpr double kMinResumeUndockM = 0.10;

/// How far the interrupted undock still has to reverse [m], given where it
/// started and where the robot is now. Never more than `undock_distance`, so a
/// resumed undock can never reverse farther in total than a normal one would
/// from the recorded start; 0 when the remainder is below kMinResumeUndockM.
inline double remainingUndockDistance(
    double start_x, double start_y, double x, double y, double undock_distance)
{
  if (!std::isfinite(undock_distance) || undock_distance <= 0.0)
    return 0.0;
  const double travelled = std::hypot(x - start_x, y - start_y);
  if (!std::isfinite(travelled))
    return 0.0;
  const double remaining = std::clamp(undock_distance - travelled, 0.0, undock_distance);
  return remaining < kMinResumeUndockM ? 0.0 : remaining;
}

}  // namespace mowgli_behavior
