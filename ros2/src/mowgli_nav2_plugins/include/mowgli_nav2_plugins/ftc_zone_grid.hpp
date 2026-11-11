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
// FTC's zone grid: the global costmap binarised for the lateral-offset guard,
// built from a full snapshot and kept current from its delta updates
// (occupancy_grid_update.hpp explains why both are needed).

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "mowgli_nav2_plugins/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace mowgli_nav2_plugins
{

/// FTC's zone grid binarises the global costmap: OccupancyGrid 100/99 =
/// lethal/inscribed (keepout boundary or a global obstacle — both are things we
/// must not skirt into) -> 254; unknown (-1) and free -> 0.
inline unsigned char ZoneCellCost(std::int8_t occupancy)
{
  return occupancy >= 99 ? 254u : 0u;
}

/// A zone costmap built from a full global-costmap snapshot.
inline std::unique_ptr<nav2_costmap_2d::Costmap2D> ZoneCostmapFromGrid(
    const nav_msgs::msg::OccupancyGrid& og)
{
  auto cm = std::make_unique<nav2_costmap_2d::Costmap2D>(og.info.width,
                                                         og.info.height,
                                                         og.info.resolution,
                                                         og.info.origin.position.x,
                                                         og.info.origin.position.y);
  unsigned char* cells = cm->getCharMap();
  const std::size_t n = static_cast<std::size_t>(og.info.width) * og.info.height;
  for (std::size_t i = 0; i < n && i < og.data.size(); ++i)
  {
    cells[i] = ZoneCellCost(og.data[i]);
  }
  return cm;
}

/// Apply a delta to the binarised zone costmap. Returns false, leaving it
/// untouched, when the update does not fit.
template <class Update>
bool ApplyZoneGridUpdate(nav2_costmap_2d::Costmap2D& zone, const Update& u)
{
  const std::size_t sx = zone.getSizeInCellsX();
  if (!OccupancyGridUpdateFits(u, sx, zone.getSizeInCellsY()))
  {
    return false;
  }
  unsigned char* cells = zone.getCharMap();
  for (std::size_t row = 0; row < u.height; ++row)
  {
    const std::size_t dst = (u.y + row) * sx + u.x;
    for (std::size_t col = 0; col < u.width; ++col)
    {
      cells[dst + col] = ZoneCellCost(u.data[row * u.width + col]);
    }
  }
  return true;
}

}  // namespace mowgli_nav2_plugins
