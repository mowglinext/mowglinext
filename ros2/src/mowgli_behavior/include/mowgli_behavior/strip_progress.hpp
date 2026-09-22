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
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mowgli_behavior
{

/// Longest stretch of PATH (arc length) FollowStrip's progress cursor may
/// advance in one update [m]. The robot moves ~0.03 m per 10 Hz tick, so this
/// leaves a wide margin for a slow tick while making a jump to the NEIGHBOURING
/// serpentine swath impossible: that swath is only 0.13 m away sideways but, along
/// the path, the whole turn-around and the rest of the swath further on.
///
/// Field 2026-09-22: the cursor searched the nearest pose over the next 400 poses
/// (~12 m). Whenever the robot was more than half a swath spacing (6.5 cm) off its
/// line — every obstacle skirt — the nearest pose was on the next swath, and the
/// monotonic cursor never came back. Units were booked "reached 100 % → MOWED" at
/// ~80 % of FTC's own index, resumes after an abort started up to 37 m of path too
/// far ahead, and ~20 m² of a 152 m² lawn was planned but never driven.
constexpr double kMaxProgressAdvanceM = 1.0;

/// Monotonic progress cursor into a coverage unit: the pose nearest to the robot
/// (rx, ry) among those at most `max_advance_m` of arc length ahead of `cursor`.
/// Never moves backwards; an empty path leaves the cursor unchanged.
inline std::size_t advanceProgressCursor(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                                         std::size_t cursor,
                                         double rx,
                                         double ry,
                                         double max_advance_m = kMaxProgressAdvanceM)
{
  if (poses.empty())
  {
    return cursor;
  }
  cursor = std::min(cursor, poses.size() - 1);
  double best_d2 = std::numeric_limits<double>::max();
  std::size_t best = cursor;
  double arc = 0.0;
  for (std::size_t i = cursor; i < poses.size(); ++i)
  {
    const auto& p = poses[i].pose.position;
    if (i > cursor)
    {
      const auto& q = poses[i - 1].pose.position;
      arc += std::hypot(p.x - q.x, p.y - q.y);
      if (arc > max_advance_m)
      {
        break;
      }
    }
    const double d2 = (p.x - rx) * (p.x - rx) + (p.y - ry) * (p.y - ry);
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

}  // namespace mowgli_behavior
