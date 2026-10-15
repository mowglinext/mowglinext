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

// One-way rejection of transit target centres known to be ineligible in this
// snapshot. Costmap2D uses double resolution; GetCostmap transports float32.
// Check every cell consistent with that rounding, rather than guessing which
// adjacent cell the producer selected. Ambiguous clear targets may be rejected.
// This is not geometry authorization or a promise about subsequent map updates.
inline bool detourTransitTargetCenterClear(const DetourCostmap& cm, double x, double y)
{
  if (!cm.valid() || !std::isfinite(cm.resolution) ||
      cm.resolution < std::numeric_limits<float>::denorm_min() ||
      cm.resolution > std::numeric_limits<float>::max() || !std::isfinite(cm.origin_x) ||
      !std::isfinite(cm.origin_y) || !std::isfinite(x) || !std::isfinite(y) ||
      cm.width > std::numeric_limits<std::size_t>::max() / cm.height)
    return false;
  const float wire = static_cast<float>(cm.resolution);
  if (static_cast<double>(wire) != cm.resolution)
    return false;  // This predicate requires a validated wire snapshot.
  const float previous = std::nextafter(wire, 0.0F);
  const float next = std::nextafter(wire, std::numeric_limits<float>::infinity());
  if (!std::isfinite(next))
    return false;
  // Midpoints between binary32 values are represented exactly in binary64.
  // Closed bounds deliberately include both round-to-nearest tie outcomes.
  const double lower = (static_cast<double>(previous) + wire) / 2.0;
  const double upper = (static_cast<double>(next) + wire) / 2.0;
  const auto indices =
      [&](double coordinate, double origin, uint32_t size, uint32_t& first, uint32_t& last)
  {
    if (coordinate < origin)
      return false;
    const double delta = coordinate - origin;
    if (!std::isfinite(delta))
      return false;
    const double lo = delta == 0.0 ? 0.0 : std::max(0.0, std::nextafter(delta / upper, -INFINITY));
    const double hi = delta == 0.0 ? 0.0 : std::nextafter(delta / lower, INFINITY);
    // Bound every conversion before casting, including expanded edge cells.
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi || hi >= size)
      return false;
    first = static_cast<uint32_t>(lo);
    last = static_cast<uint32_t>(hi);
    return true;
  };
  uint32_t x0, x1, y0, y1;
  if (!indices(x, cm.origin_x, cm.width, x0, x1) || !indices(y, cm.origin_y, cm.height, y0, y1))
    return false;
  for (uint64_t row = y0; row <= y1; ++row)
    for (uint64_t col = x0; col <= x1; ++col)
    {
      const auto value = cm.data[static_cast<std::size_t>(row) * cm.width + col];
      // Snapshot conversion preserves raw253/254/255 exactly as99/100/-1.
      if (value < 0 || value >= 99)
        return false;
    }
  return true;
}

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
