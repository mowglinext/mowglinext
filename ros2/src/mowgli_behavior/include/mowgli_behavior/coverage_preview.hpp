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
#include <cstdint>
#include <utility>
#include <vector>

#include "mowgli_interfaces/msg/coverage_plan_preview.hpp"
#include "nav_msgs/msg/path.hpp"

namespace mowgli_behavior
{
// Build a display-only preview. The planner/executor continues using its
// original double-precision paths; this simplifies each independent path to
// <= tolerance metres and then stores only XY as float32.
inline mowgli_interfaces::msg::CoveragePlanPreview makeCoveragePreview(
    const std::vector<nav_msgs::msg::Path>& paths, double tolerance_m = 0.01)
{
  mowgli_interfaces::msg::CoveragePlanPreview out;
  out.subpath_offsets.push_back(0);
  if (!paths.empty())
    out.header = paths.front().header;
  const double tol2 = tolerance_m * tolerance_m;
  for (const auto& path : paths)
  {
    const auto& poses = path.poses;
    if (poses.size() < 2)
      continue;
    std::vector<bool> keep(poses.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, poses.size() - 1}};
    while (!stack.empty())
    {
      const auto [first, last] = stack.back();
      stack.pop_back();
      const double ax = poses[first].pose.position.x, ay = poses[first].pose.position.y;
      const double bx = poses[last].pose.position.x, by = poses[last].pose.position.y;
      const double dx = bx - ax, dy = by - ay, denom = dx * dx + dy * dy;
      double max_dist = tol2;
      std::size_t farthest = first;
      for (std::size_t i = first + 1; i < last; ++i)
      {
        const double px = poses[i].pose.position.x, py = poses[i].pose.position.y;
        const double t =
            denom > 0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / denom, 0.0, 1.0) : 0.0;
        const double ex = px - (ax + t * dx), ey = py - (ay + t * dy);
        const double d2 = ex * ex + ey * ey;
        if (d2 > max_dist)
        {
          max_dist = d2;
          farthest = i;
        }
      }
      if (farthest != first)
      {
        keep[farthest] = true;
        stack.emplace_back(first, farthest);
        stack.emplace_back(farthest, last);
      }
    }
    for (std::size_t i = 0; i < poses.size(); ++i)
      if (keep[i])
      {
        out.xy.push_back(static_cast<float>(poses[i].pose.position.x));
        out.xy.push_back(static_cast<float>(poses[i].pose.position.y));
      }
    out.subpath_offsets.push_back(static_cast<std::uint32_t>(out.xy.size() / 2));
  }
  return out;
}
}  // namespace mowgli_behavior
