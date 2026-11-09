// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Package-private grid_map layer -> nav_msgs/OccupancyGrid export.
//
// map_server rebuilds the WHOLE garden ~/mow_progress grid every
// mow_progress_publish_period_s while mowing. grid_map's
// GridMapRosConverter::toOccupancyGrid visits each cell through a
// GridMapIterator and reads it with GridMap::at(layer, index) — a
// std::string-keyed hash lookup per cell — before scattering it into the
// reversed OccupancyGrid order. Here the layer matrix is read directly: per
// column, at most two contiguous runs of the (column-major, circular) buffer,
// written backwards into the message. The per-cell arithmetic is upstream's,
// expression for expression; only the traversal changed, so the message is
// byte-identical for any non-empty grid (on a 0-cell grid upstream's iterator
// still runs once and writes out of bounds; this returns an empty grid).
// Pinned against the upstream converter by test_occupancy_grid_export.cpp.
//
// Index convention (CLAUDE.md Invariant 14): upstream writes buffer cell
// (i, j) — unwrapped through the circular-buffer start index — to
// data[nCells - (i + j * size_x) - 1]; this does the same.

#ifndef MOWGLI_MAP__OCCUPANCY_GRID_EXPORT_HPP_
#define MOWGLI_MAP__OCCUPANCY_GRID_EXPORT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include <grid_map_core/GridMap.hpp>

namespace mowgli_map
{

/// Drop-in replacement for GridMapRosConverter::toOccupancyGrid (same header,
/// info and data). `layer` must exist.
inline void layer_to_occupancy_grid(const grid_map::GridMap& grid_map,
                                    const std::string& layer,
                                    float data_min,
                                    float data_max,
                                    nav_msgs::msg::OccupancyGrid& occupancy_grid)
{
  occupancy_grid.header.frame_id = grid_map.getFrameId();
  occupancy_grid.header.stamp = rclcpp::Time(grid_map.getTimestamp());
  occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
  occupancy_grid.info.resolution = grid_map.getResolution();
  occupancy_grid.info.width = grid_map.getSize()(0);
  occupancy_grid.info.height = grid_map.getSize()(1);
  const grid_map::Position position = grid_map.getPosition() - 0.5 * grid_map.getLength().matrix();
  occupancy_grid.info.origin.position.x = position.x();
  occupancy_grid.info.origin.position.y = position.y();
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.x = 0.0;
  occupancy_grid.info.origin.orientation.y = 0.0;
  occupancy_grid.info.origin.orientation.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  const int size_x = grid_map.getSize()(0);
  const int size_y = grid_map.getSize()(1);
  const std::size_t n_cells = static_cast<std::size_t>(size_x) * static_cast<std::size_t>(size_y);
  occupancy_grid.data.resize(n_cells);
  if (n_cells == 0U)
  {
    return;
  }

  // Occupancy probabilities are in the range [0,100]. Unknown is -1.
  const float cell_min = 0;
  const float cell_max = 100;
  const float cell_range = cell_max - cell_min;
  const auto convert = [&](float raw) -> int8_t
  {
    // Upstream's if/else as a select, so the loop vectorises: the clamped
    // branch is computed for NaN too (std::max(0, NaN) is 0, so it stays
    // finite) and then discarded.
    const float value = (raw - data_min) / (data_max - data_min);
    float clamped = (0.0f < value) ? value : 0.0f;  // std::max(0.0f, value)
    clamped = (1.0f < clamped) ? 1.0f : clamped;  // std::min(clamped, 1.0f)
    const float known = cell_min + clamped * cell_range;
    return static_cast<int8_t>(std::isnan(value) ? -1.0f : known);
  };

  const grid_map::Matrix& data = grid_map[layer];
  const grid_map::Index& start = grid_map.getStartIndex();
  // GridMap::setStartIndex() accepts any index and upstream wraps it to the
  // buffer (wrapIndexToRange, a true modulo); do the same.
  const auto wrap = [](int index, int size)
  {
    const int r = index % size;
    return r < 0 ? r + size : r;
  };
  const int start_x = wrap(start(0), size_x);
  const int start_y = wrap(start(1), size_y);
  int8_t* const out = occupancy_grid.data.data();
  // Unwrapped columns [0, tail_y) are buffer columns [start_y, size_y).
  const int tail_y = size_y - start_y;
  for (int j = 0; j < size_y; ++j)
  {
    // Unwrapped column j is buffer column (start_y + j) mod size_y, formed
    // without the sum (which can overflow int); inside it, unwrapped row i is
    // buffer row (start_x + i) mod size_x.
    const int buffer_col = j < tail_y ? start_y + j : j - tail_y;
    const float* const column = data.data() + static_cast<std::size_t>(buffer_col) * size_x;
    int8_t* const last = out + (n_cells - 1U - static_cast<std::size_t>(j) * size_x);
    const int head =
        size_x - start_x;  // unwrapped rows [0, head) are buffer rows [start_x, size_x)
    for (int i = 0; i < head; ++i)
    {
      *(last - i) = convert(column[start_x + i]);
    }
    for (int i = head; i < size_x; ++i)
    {
      *(last - i) = convert(column[i - head]);
    }
  }
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__OCCUPANCY_GRID_EXPORT_HPP_
