// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// layer_to_occupancy_grid must produce the SAME message, byte for byte, as
// grid_map::GridMapRosConverter::toOccupancyGrid — header, info and data —
// for every non-empty grid geometry (including a wrapped circular buffer) and
// every float value (NaN, infinities, signed zero, out-of-range, fractions).

#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "mowgli_map/occupancy_grid_export.hpp"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <gtest/gtest.h>

namespace
{

constexpr const char* kLayer = "mowed";

grid_map::GridMap make_map(double length_x, double length_y, double resolution, int shift_cells)
{
  grid_map::GridMap map({kLayer});
  map.setFrameId("map");
  map.setTimestamp(1234567890123456789ULL);
  map.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(1.3, -0.7));
  map[kLayer].setConstant(0.0F);
  if (shift_cells != 0)
  {
    // Moving the map wraps its circular buffer: start index != (0, 0).
    map.move(map.getPosition() +
             grid_map::Position(shift_cells * resolution, -2.0 * shift_cells * resolution));
  }
  return map;
}

void fill_random(grid_map::GridMap& map, unsigned seed)
{
  const std::vector<float> specials = {0.0F,
                                       100.0F,
                                       -0.0F,
                                       -1.0F,
                                       150.0F,
                                       99.999F,
                                       0.0001F,
                                       50.0F,
                                       std::numeric_limits<float>::quiet_NaN(),
                                       std::numeric_limits<float>::infinity(),
                                       -std::numeric_limits<float>::infinity(),
                                       std::numeric_limits<float>::denorm_min()};
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> any(-50.0F, 150.0F);
  std::uniform_int_distribution<std::size_t> pick(0, specials.size() - 1);
  auto& layer = map[kLayer];
  for (Eigen::Index k = 0; k < layer.size(); ++k)
  {
    layer.data()[k] = (rng() % 3 == 0) ? specials[pick(rng)] : any(rng);
  }
}

void expect_same_as_upstream(const grid_map::GridMap& map, float data_min, float data_max)
{
  nav_msgs::msg::OccupancyGrid expected;
  nav_msgs::msg::OccupancyGrid actual;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, kLayer, data_min, data_max, expected);
  // Pre-fill with garbage of a different size: the result must not depend on it.
  actual.data.assign(17, 42);
  mowgli_map::layer_to_occupancy_grid(map, kLayer, data_min, data_max, actual);
  ASSERT_EQ(actual.data.size(), expected.data.size());
  EXPECT_EQ(actual.header, expected.header);
  EXPECT_EQ(actual.info, expected.info);
  std::size_t mismatches = 0;
  for (std::size_t k = 0; k < expected.data.size(); ++k)
  {
    mismatches += actual.data[k] != expected.data[k] ? 1U : 0U;
  }
  EXPECT_EQ(mismatches, 0U) << "of " << expected.data.size() << " cells";
}

struct Geometry
{
  double length_x;
  double length_y;
  double resolution;
  int shift_cells;
};

const std::vector<Geometry> kGeometries = {
    {0.05, 0.05, 0.05, 0},  // a single cell
    {7.35, 3.05, 0.05, 0},  // odd sizes
    {7.35, 3.05, 0.05, 5},  // odd sizes, wrapped
    {30.0, 25.0, 0.05, 0},  // a garden-sized grid
    {30.0, 25.0, 0.05, 37},
    {12.0, 20.0, 0.08, 11},  // global-costmap resolution, wrapped
};

TEST(OccupancyGridExport, MatchesUpstreamForMowProgressValues)
{
  // map_server's real content: only 0 (unmowed) and 100 (mowed).
  for (const auto& g : kGeometries)
  {
    grid_map::GridMap map = make_map(g.length_x, g.length_y, g.resolution, g.shift_cells);
    map[kLayer] = map[kLayer].unaryExpr(
        [](float)
        {
          return 0.0F;
        });
    auto& layer = map[kLayer];
    for (Eigen::Index k = 0; k < layer.size(); k += 3)
    {
      layer.data()[k] = 100.0F;
    }
    SCOPED_TRACE(std::to_string(map.getSize()(0)) + "x" + std::to_string(map.getSize()(1)));
    expect_same_as_upstream(map, 0.0F, 100.0F);
  }
}

TEST(OccupancyGridExport, MatchesUpstreamForEveryFloatValue)
{
  unsigned seed = 1;
  for (const auto& g : kGeometries)
  {
    grid_map::GridMap map = make_map(g.length_x, g.length_y, g.resolution, g.shift_cells);
    fill_random(map, seed++);
    SCOPED_TRACE(std::to_string(map.getSize()(0)) + "x" + std::to_string(map.getSize()(1)) +
                 " start " + std::to_string(map.getStartIndex()(0)) + "," +
                 std::to_string(map.getStartIndex()(1)));
    expect_same_as_upstream(map, 0.0F, 100.0F);
    expect_same_as_upstream(map, -20.0F, 80.0F);
    expect_same_as_upstream(map, 100.0F, 0.0F);  // inverted range
    expect_same_as_upstream(map, 50.0F, 50.0F);  // empty range: 0/0 and x/0
  }
}

TEST(OccupancyGridExport, WrappedBufferIsActuallyExercised)
{
  const grid_map::GridMap map = make_map(7.35, 3.05, 0.05, 5);
  EXPECT_NE(map.getStartIndex()(0), 0);
  EXPECT_NE(map.getStartIndex()(1), 0);
}

// Invariant 14: the cell at map position (x, y) lands in OccupancyGrid column
// floor((x - origin.x) / res), row floor((y - origin.y) / res), data index
// row * width + col — checked here from the position side, independently of
// how either converter walks the buffer.
TEST(OccupancyGridExport, CellLandsAtItsMapPosition)
{
  for (int shift : {0, 23})
  {
    grid_map::GridMap map = make_map(6.0, 4.0, 0.05, shift);
    fill_random(map, 99U + static_cast<unsigned>(shift));
    nav_msgs::msg::OccupancyGrid grid;
    mowgli_map::layer_to_occupancy_grid(map, kLayer, 0.0F, 100.0F, grid);
    const double res = grid.info.resolution;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
      grid_map::Position p;
      ASSERT_TRUE(map.getPosition(*it, p));
      const auto col =
          static_cast<std::size_t>(std::floor((p.x() - grid.info.origin.position.x) / res));
      const auto row =
          static_cast<std::size_t>(std::floor((p.y() - grid.info.origin.position.y) / res));
      ASSERT_LT(col, grid.info.width);
      ASSERT_LT(row, grid.info.height);
      const float raw = map.at(kLayer, *it);
      const int8_t cell = grid.data[row * grid.info.width + col];
      if (std::isnan(raw))
      {
        EXPECT_EQ(cell, -1);
      }
      else
      {
        EXPECT_EQ(cell, static_cast<int8_t>(std::min(std::max(0.0F, raw / 100.0F), 1.0F) * 100.0F));
      }
    }
  }
}

}  // namespace
