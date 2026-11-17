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
// Keeping the cached global costmap current from its delta updates
// (occupancy_grid_update.hpp, ftc_zone_grid.hpp).

#include <cstdint>
#include <vector>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "mowgli_nav2_plugins/ftc_zone_grid.hpp"
#include "mowgli_nav2_plugins/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <gtest/gtest.h>

namespace
{

namespace mnp = mowgli_nav2_plugins;

nav_msgs::msg::OccupancyGrid Grid(unsigned int w, unsigned int h, std::int8_t value)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = w;
  g.info.height = h;
  g.info.resolution = 0.1;
  g.data.assign(static_cast<std::size_t>(w) * h, value);
  return g;
}

map_msgs::msg::OccupancyGridUpdate Update(
    int x, int y, unsigned int w, unsigned int h, std::int8_t v)
{
  map_msgs::msg::OccupancyGridUpdate u;
  u.x = x;
  u.y = y;
  u.width = w;
  u.height = h;
  u.data.assign(static_cast<std::size_t>(w) * h, v);
  u.header.stamp.sec = 42;
  return u;
}

TEST(OccupancyGridUpdate, WritesTheRectangleRowMajor)
{
  auto grid = Grid(5, 4, 0);
  auto u = Update(1, 2, 3, 2, 100);
  u.data[0] = 7;  // (x 1, y 2)
  ASSERT_TRUE(mnp::ApplyOccupancyGridUpdate(grid, u));
  EXPECT_EQ(grid.data[2 * 5 + 1], 7);
  EXPECT_EQ(grid.data[2 * 5 + 3], 100);
  EXPECT_EQ(grid.data[3 * 5 + 2], 100);
  EXPECT_EQ(grid.data[1 * 5 + 1], 0);  // row above untouched
  EXPECT_EQ(grid.data[2 * 5 + 4], 0);  // column past the rectangle untouched
  EXPECT_EQ(grid.header.stamp.sec, 42);
}

TEST(OccupancyGridUpdate, AnUpdateThatDoesNotFitIsDroppedWhole)
{
  for (const auto& u : std::vector<map_msgs::msg::OccupancyGridUpdate>{
           Update(4, 0, 2, 1, 100),  // past the right edge (a resized grid)
           Update(0, 3, 1, 2, 100),  // past the top
           Update(-1, 0, 1, 1, 100),  // negative origin
           Update(0, 0, 0, 1, 100)})  // empty
  {
    auto grid = Grid(5, 4, 0);
    EXPECT_FALSE(mnp::ApplyOccupancyGridUpdate(grid, u));
    for (const auto v : grid.data)
    {
      EXPECT_EQ(v, 0);
    }
  }
  auto grid = Grid(5, 4, 0);
  auto short_data = Update(0, 0, 2, 2, 100);
  short_data.data.pop_back();
  EXPECT_FALSE(mnp::ApplyOccupancyGridUpdate(grid, short_data));
}

TEST(ZoneGrid, BinarisesAtInscribedAndUpdatesInPlace)
{
  auto grid = Grid(4, 4, 0);
  grid.data[0] = 99;
  grid.data[1] = 98;
  grid.data[2] = -1;
  grid.data[3] = 100;
  auto zone = mnp::ZoneCostmapFromGrid(grid);
  ASSERT_NE(zone, nullptr);
  EXPECT_EQ(zone->getCost(0, 0), 254);
  EXPECT_EQ(zone->getCost(1, 0), 0);
  EXPECT_EQ(zone->getCost(2, 0), 0);  // unknown is not a wall
  EXPECT_EQ(zone->getCost(3, 0), 254);

  // An obstacle accepted in the middle of the lawn arrives as a delta only.
  ASSERT_TRUE(mnp::ApplyZoneGridUpdate(*zone, Update(1, 2, 2, 1, 100)));
  EXPECT_EQ(zone->getCost(1, 2), 254);
  EXPECT_EQ(zone->getCost(2, 2), 254);
  // ...and a boundary edit can also free cells.
  ASSERT_TRUE(mnp::ApplyZoneGridUpdate(*zone, Update(0, 0, 1, 1, 0)));
  EXPECT_EQ(zone->getCost(0, 0), 0);
  EXPECT_FALSE(mnp::ApplyZoneGridUpdate(*zone, Update(3, 3, 2, 2, 100)));
}

}  // namespace
