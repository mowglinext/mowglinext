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
// Keeping a copy of /global_costmap/costmap current.
//
// The global costmap publishes in DELTA mode (always_send_full_costmap: false,
// nav2_params_base.yaml): the full grid on /global_costmap/costmap arrives on
// first connect and on a resize only; every later change — a boundary edit
// that keeps the extent, an obstacle accepted in the middle of the lawn —
// arrives on /global_costmap/costmap_updates as a map_msgs/OccupancyGridUpdate.
// A subscriber that reads only the full topic works on its first snapshot for
// the rest of the stack's life. These helpers apply an update to the cached
// copy; they are templated on the update type so a consumer only needs
// map_msgs where it actually subscribes.

#pragma once

#include <cstddef>
#include <cstdint>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace mowgli_nav2_plugins
{

/// Does the update rectangle lie inside a grid of `width` x `height` cells and
/// carry exactly one value per cell? An update that does not fit belongs to a
/// grid we no longer have (a resize republishes the full grid): drop it.
template <class Update>
bool OccupancyGridUpdateFits(const Update& u, std::size_t width, std::size_t height)
{
  const std::int64_t x = u.x;
  const std::int64_t y = u.y;
  const std::int64_t w = u.width;
  const std::int64_t h = u.height;
  return x >= 0 && y >= 0 && w > 0 && h > 0 && x + w <= static_cast<std::int64_t>(width) &&
         y + h <= static_cast<std::int64_t>(height) &&
         static_cast<std::int64_t>(u.data.size()) == w * h;
}

/// Write the update's cells into `grid` (row-major, like the message). Returns
/// false, leaving `grid` untouched, when the update does not fit it.
template <class Update>
bool ApplyOccupancyGridUpdate(nav_msgs::msg::OccupancyGrid& grid, const Update& u)
{
  if (grid.data.size() != static_cast<std::size_t>(grid.info.width) * grid.info.height ||
      !OccupancyGridUpdateFits(u, grid.info.width, grid.info.height))
  {
    return false;
  }
  for (std::size_t row = 0; row < u.height; ++row)
  {
    const std::size_t dst = (u.y + row) * grid.info.width + u.x;
    for (std::size_t col = 0; col < u.width; ++col)
    {
      grid.data[dst + col] = u.data[row * u.width + col];
    }
  }
  grid.header.stamp = u.header.stamp;
  return true;
}

}  // namespace mowgli_nav2_plugins
