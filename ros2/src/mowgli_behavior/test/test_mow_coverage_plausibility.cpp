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
// Unit tests for the coverage-completion plausibility cross-check
// (mow_coverage_plausibility.hpp, issue #680). ROS-free by construction.

#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_behavior/mow_coverage_plausibility.hpp"

namespace
{

using mowgli_behavior::ComputeMowedFraction;
using mowgli_behavior::kMinPlausibleMowedFraction;
using mowgli_behavior::MowProgressGridView;
using mowgli_behavior::PointInPolygon;

std::vector<std::pair<double, double>> Square(double side)
{
  return {{0.0, 0.0}, {side, 0.0}, {side, side}, {0.0, side}};
}

// Builds a 10x10 m grid at 0.1 m resolution (100x100 cells), origin at
// (0,0), all cells set to `value`.
struct SyntheticGrid
{
  std::vector<int8_t> data;
  MowProgressGridView view;

  explicit SyntheticGrid(int8_t value)
  {
    view.resolution = 0.1;
    view.origin_x = 0.0;
    view.origin_y = 0.0;
    view.width = 100;
    view.height = 100;
    data.assign(static_cast<std::size_t>(view.width) * static_cast<std::size_t>(view.height),
                value);
    view.data = &data;
  }
};

TEST(PointInPolygonTest, Inside)
{
  const auto square = Square(10.0);
  EXPECT_TRUE(PointInPolygon(5.0, 5.0, square));
}

TEST(PointInPolygonTest, Outside)
{
  const auto square = Square(10.0);
  EXPECT_FALSE(PointInPolygon(-1.0, 5.0, square));
  EXPECT_FALSE(PointInPolygon(15.0, 5.0, square));
}

TEST(PointInPolygonTest, DegenerateIsNeverInside)
{
  const std::vector<std::pair<double, double>> line = {{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_FALSE(PointInPolygon(0.5, 0.0, line));
}

// The exact scenario issue #680 reports: FollowStrip's bookkeeping says
// "fully mowed" after driving only the headland ring, so mow_progress shows
// a thin stamped band around the boundary and an untouched interior.
TEST(ComputeMowedFractionTest, HeadlandOnlyBandFallsBelowThePlausibilityFloor)
{
  SyntheticGrid grid(0);
  // Stamp a ~0.3 m band around the 10x10 m square's perimeter.
  for (int32_t row = 0; row < grid.view.height; ++row)
  {
    for (int32_t col = 0; col < grid.view.width; ++col)
    {
      const bool near_edge =
          row < 3 || row >= grid.view.height - 3 || col < 3 || col >= grid.view.width - 3;
      if (near_edge)
      {
        grid.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.view.width) +
                  static_cast<std::size_t>(col)] = 100;
      }
    }
  }

  const auto outer = Square(10.0);
  const double fraction = ComputeMowedFraction(grid.view, outer, {});

  EXPECT_LT(fraction, kMinPlausibleMowedFraction);
}

// A genuinely complete pass — the whole interior stamped — must read as
// plausible so this check never false-trips on a healthy mow.
TEST(ComputeMowedFractionTest, FullyMowedInteriorIsPlausible)
{
  SyntheticGrid grid(100);
  const auto outer = Square(10.0);

  const double fraction = ComputeMowedFraction(grid.view, outer, {});

  EXPECT_GE(fraction, kMinPlausibleMowedFraction);
  EXPECT_NEAR(fraction, 1.0, 0.01);
}

// An obstacle hole is never supposed to be mowed — it must not count against
// the fraction even though every one of its cells reads "not mowed".
TEST(ComputeMowedFractionTest, ObstacleHolesAreExcludedFromTheDenominator)
{
  SyntheticGrid grid(0);
  const auto outer = Square(10.0);
  // A 2x2 m hole in the middle, never stamped.
  const std::vector<std::pair<double, double>> hole = {{4.0, 4.0},
                                                       {6.0, 4.0},
                                                       {6.0, 6.0},
                                                       {4.0, 6.0}};

  // Stamp everything inside the outer boundary except the hole.
  for (int32_t row = 0; row < grid.view.height; ++row)
  {
    for (int32_t col = 0; col < grid.view.width; ++col)
    {
      const double x = grid.view.origin_x + (static_cast<double>(col) + 0.5) * grid.view.resolution;
      const double y = grid.view.origin_y + (static_cast<double>(row) + 0.5) * grid.view.resolution;
      if (PointInPolygon(x, y, outer) && !PointInPolygon(x, y, hole))
      {
        grid.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.view.width) +
                  static_cast<std::size_t>(col)] = 100;
      }
    }
  }

  const double fraction = ComputeMowedFraction(grid.view, outer, {hole});

  EXPECT_NEAR(fraction, 1.0, 0.01);
}

TEST(ComputeMowedFractionTest, DegenerateOuterPolygonIsNotPlausible)
{
  SyntheticGrid grid(100);
  const std::vector<std::pair<double, double>> degenerate = {{0.0, 0.0}, {1.0, 0.0}};

  EXPECT_EQ(ComputeMowedFraction(grid.view, degenerate, {}), 0.0);
}

TEST(ComputeMowedFractionTest, EmptyGridDataIsNotPlausible)
{
  MowProgressGridView view;
  view.resolution = 0.1;
  view.width = 100;
  view.height = 100;
  view.data = nullptr;
  const auto outer = Square(10.0);

  EXPECT_EQ(ComputeMowedFraction(view, outer, {}), 0.0);
}

}  // namespace
