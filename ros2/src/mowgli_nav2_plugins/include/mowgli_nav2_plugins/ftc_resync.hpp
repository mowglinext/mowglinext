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
// Where FTC may re-anchor its carrot when the robot has drifted too far from it.
//
// The recovery used to take the nearest pose of the WHOLE plan. A coverage
// sub-path is not a line, it is concentric headland rings 0.16 m apart followed
// by parallel swaths: a robot 0.4 m off ring 0 is standing ON ring 2 or 3, so
// "nearest pose anywhere" is a pose tens of metres further along the plan.
// Field, 2026-09-17: two resyncs jumped the index 24 -> 2617 and 2737 -> 3584
// (about 96 m and 31 m of planned mowing), and the skipped rings were booked as
// mowed (coverage 1.5 % -> 13 % in one minute).
//
// Progress along a coverage plan is only meaningful in path order, so the
// search is bounded to a window of PATH LENGTH around the current index.

#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace mowgli_nav2_plugins
{

/// Resync search window, as a multiple of `max_follow_distance` of PATH LENGTH
/// on each side of the carrot. The robot cannot be further along the path than
/// it is away from the carrot, so 2x leaves slack for a curved path.
inline constexpr double kResyncWindowFactor = 2.0;

struct ResyncResult
{
  std::size_t index{0};
  double distance_m{0.0};
};

/// Nearest plan pose to (rx, ry) among the poses whose distance ALONG the path
/// from `current_index` is at most `window_m`, in either direction. Ties keep
/// the pose closest to `current_index` in path order, so a degenerate window
/// never moves the carrot. An empty plan returns index 0 at distance 0.
inline ResyncResult FindResyncIndex(const std::vector<std::pair<double, double>>& plan,
                                    std::size_t current_index,
                                    double rx,
                                    double ry,
                                    double window_m)
{
  if (plan.empty())
  {
    return {};
  }
  const std::size_t start = current_index < plan.size() ? current_index : plan.size() - 1;
  const auto dist_to_robot = [&](std::size_t i)
  {
    return std::hypot(plan[i].first - rx, plan[i].second - ry);
  };
  const auto step_len = [&](std::size_t a, std::size_t b)
  {
    return std::hypot(plan[a].first - plan[b].first, plan[a].second - plan[b].second);
  };

  ResyncResult best{start, dist_to_robot(start)};

  double along = 0.0;
  for (std::size_t i = start; i + 1 < plan.size(); ++i)
  {
    along += step_len(i, i + 1);
    if (along > window_m)
    {
      break;
    }
    const double d = dist_to_robot(i + 1);
    if (d < best.distance_m)
    {
      best = {i + 1, d};
    }
  }

  along = 0.0;
  for (std::size_t i = start; i > 0; --i)
  {
    along += step_len(i, i - 1);
    if (along > window_m)
    {
      break;
    }
    const double d = dist_to_robot(i - 1);
    if (d < best.distance_m)
    {
      best = {i - 1, d};
    }
  }
  return best;
}

}  // namespace mowgli_nav2_plugins
