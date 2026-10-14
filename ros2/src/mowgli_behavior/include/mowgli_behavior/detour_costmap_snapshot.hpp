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

#include <cmath>
#include <limits>
#include <optional>

#include "mowgli_behavior/detour_resume.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace mowgli_behavior
{

// GetCostmap returns raw Nav2 costs. Preserve Costmap2DPublisher's exact
// OccupancyGrid conversion so the existing detour policy still tests true
// lethal cells (100), rather than treating the inscribed band (99) as lethal.
inline std::optional<DetourCostmap> detourCostmapSnapshot(const nav2_msgs::msg::Costmap& map)
{
  const auto& m = map.metadata;
  const auto& p = m.origin.position;
  const auto& q = m.origin.orientation;
  const std::size_t width = m.size_x;
  const std::size_t height = m.size_y;
  if (map.header.frame_id != "map" || width == 0 || height == 0 || !std::isfinite(m.resolution) ||
      m.resolution <= 0.0 || !std::isfinite(1.0 / m.resolution) || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.z) || q.x != 0.0 || q.y != 0.0 || q.z != 0.0 ||
      !std::isfinite(q.w) || std::abs(q.w) != 1.0 ||
      width > std::numeric_limits<std::size_t>::max() / height || map.data.size() != width * height)
  {
    return std::nullopt;
  }
  // Prevent overflowing world extents from reaching cell-index arithmetic.
  if (!std::isfinite(p.x + static_cast<double>(width) * m.resolution) ||
      !std::isfinite(p.y + static_cast<double>(height) * m.resolution))
  {
    return std::nullopt;
  }
  DetourCostmap result;
  result.origin_x = p.x;
  result.origin_y = p.y;
  result.resolution = m.resolution;
  result.width = m.size_x;
  result.height = m.size_y;
  result.data.reserve(map.data.size());
  for (const auto cost : map.data)
  {
    result.data.push_back(cost == 0     ? 0
                          : cost == 253 ? 99
                          : cost == 254 ? 100
                          : cost == 255 ? -1
                                        : 1 + (97 * (cost - 1)) / 251);
  }
  return result;
}

}  // namespace mowgli_behavior
