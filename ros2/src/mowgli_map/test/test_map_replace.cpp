// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Regression tests for "saving the map from the GUI times out"
// (HTTP 500 … CallService /map_server_node/add_area: context deadline
// exceeded). The GUI replaces a map with clear_map → add_area × N →
// save_areas; every add_area used to rasterise the WHOLE area list twice in
// the service callback and arm a full keepout-mask rebuild whose cost was
// O(grid cells × boundary vertices), on the node's single executor thread.
//
// Properties pinned here:
//   1. the fast keepout mask is BIT-IDENTICAL to the per-cell definition it
//      replaced (the reference below is that definition, kept deliberately
//      naive) — the mask is what keeps the robot inside the garden, so a
//      faster mask that differs by one cell is not acceptable;
//   2. a realistic replace is cheap: add_area does no grid work at all, and
//      the single rebuild at the end fits a budget that the old code missed by
//      orders of magnitude;
//   3. the publish timer rebuilds ONCE after a burst of edits, never between
//      two add_area calls (no half-built map reaches Nav2);
//   4. areas.dat is replaced atomically — a failed save leaves the previous
//      map intact.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/polygon_raster.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <gtest/gtest.h>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>

namespace
{

using Polygon = geometry_msgs::msg::Polygon;
using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kOutsideMargin = 0.597;  // shipped enforce_boundary_margin_m (floored)
constexpr double kInnerMargin = 0.20;  // shipped boundary_inner_margin_m
constexpr double kObstacleMargin = 0.276;  // shipped keepout_obstacle_margin
constexpr int8_t kSoftPenalty = 50;  // mirrors kSoftPenaltyMaskCost

double seconds_since(const Clock::time_point& start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

/// A wobbly closed curve around (cx, cy): what a boundary recorded by driving
/// looks like — many short edges, concave in places.
Polygon make_blob(double cx, double cy, double radius, int vertices, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> jitter(-0.04, 0.04);
  Polygon poly;
  for (int i = 0; i < vertices; ++i)
  {
    const double a = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(vertices);
    const double r =
        radius * (1.0 + 0.18 * std::sin(3.0 * a) + 0.07 * std::cos(7.0 * a)) + radius * jitter(rng);
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(cx + r * std::cos(a));
    pt.y = static_cast<float>(cy + r * std::sin(a));
    poly.points.push_back(pt);
  }
  return poly;
}

struct TestArea
{
  std::string name;
  Polygon polygon;
  std::vector<Polygon> obstacles;
  bool is_navigation{false};
};

/// ~1500 + ~600 + ~250 m² of lawn, 400 / 250 / 100 boundary vertices, ten
/// drawn obstacles — the size of garden the field reports come from.
std::vector<TestArea> realistic_garden()
{
  std::vector<TestArea> areas(3);
  areas[0].name = "back lawn";
  areas[0].polygon = make_blob(0.0, 0.0, 21.5, 400, 1);
  areas[1].name = "side lawn";
  areas[1].polygon = make_blob(34.0, 4.0, 13.5, 250, 2);
  areas[2].name = "front lawn";
  areas[2].polygon = make_blob(6.0, 31.0, 8.8, 100, 3);
  for (int i = 0; i < 6; ++i)
  {
    const double a = 2.0 * kPi * i / 6.0;
    areas[0].obstacles.push_back(
        make_blob(11.0 * std::cos(a), 11.0 * std::sin(a), 0.6, 8, 10 + static_cast<unsigned>(i)));
  }
  for (int i = 0; i < 3; ++i)
  {
    areas[1].obstacles.push_back(
        make_blob(30.0 + 4.0 * i, 4.0, 0.5, 8, 20 + static_cast<unsigned>(i)));
  }
  areas[2].obstacles.push_back(make_blob(6.0, 31.0, 0.8, 12, 30));
  return areas;
}

// ── The per-cell DEFINITION of the keepout mask ─────────────────────────────
// Same arithmetic as the implementation this PR replaced: float ray casting
// for "inside", double closest-edge distance for the bands.

bool ref_point_in_polygon(float x, float y, const Polygon& polygon)
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const float xi = pts[i].x, yi = pts[i].y;
    const float xj = pts[j].x, yj = pts[j].y;
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<int8_t> reference_mask(const nav_msgs::msg::OccupancyGrid& like,
                                   const std::vector<TestArea>& areas)
{
  const int nx = static_cast<int>(like.info.width);
  const int ny = static_cast<int>(like.info.height);
  const double res = static_cast<double>(like.info.resolution);

  // A grid_map with the published geometry gives the exact cell centres and
  // the exact PolygonIterator cell set the NO_GO classification overlay uses.
  grid_map::GridMap grid({"no_go"});
  grid.setGeometry(grid_map::Length(nx * res, ny * res),
                   res,
                   grid_map::Position(like.info.origin.position.x + nx * res * 0.5,
                                      like.info.origin.position.y + ny * res * 0.5));
  EXPECT_EQ(grid.getSize()(0), nx);
  EXPECT_EQ(grid.getSize()(1), ny);
  grid["no_go"].setConstant(0.0F);
  for (const auto& area : areas)
  {
    for (const auto& obs : area.obstacles)
    {
      grid_map::Polygon gm;
      for (const auto& p : obs.points)
      {
        gm.addVertex(grid_map::Position(p.x, p.y));
      }
      for (grid_map::PolygonIterator it(grid, gm); !it.isPastEnd(); ++it)
      {
        grid.at("no_go", *it) = 1.0F;
      }
    }
  }

  std::vector<int8_t> data(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 100);
  const auto& no_go = grid["no_go"];
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      grid_map::Position pos;
      grid.getPosition(grid_map::Index(r, c), pos);
      const float x = static_cast<float>(pos.x());
      const float y = static_cast<float>(pos.y());
      const double xd = static_cast<double>(x);
      const double yd = static_cast<double>(y);

      bool inside_any = false;
      bool inner = false;
      bool outer = false;
      for (const auto& area : areas)
      {
        const double d = mowgli_map::point_to_polygon_distance(xd, yd, area.polygon);
        if (ref_point_in_polygon(x, y, area.polygon))
        {
          inside_any = true;
          inner = inner || d < kInnerMargin;
        }
        else
        {
          outer = outer || d <= kOutsideMargin;
        }
      }
      int8_t value = 100;
      if (inside_any)
      {
        value = inner ? kSoftPenalty : 0;
      }
      else if (outer)
      {
        value = kSoftPenalty;
      }
      for (const auto& area : areas)
      {
        for (const auto& obs : area.obstacles)
        {
          if (ref_point_in_polygon(x, y, obs) ||
              mowgli_map::point_to_polygon_distance(xd, yd, obs) <= kObstacleMargin)
          {
            value = 100;
          }
        }
      }
      if (no_go(r, c) == 1.0F)
      {
        value = 100;
      }
      // Invariant 14: og_col = nx-1-r, og_row = ny-1-c, data[row*width+col].
      data[static_cast<std::size_t>((ny - 1 - c) * nx + (nx - 1 - r))] = value;
    }
  }
  return data;
}

class MapReplaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    areas_file_ = (std::filesystem::temp_directory_path() /
                   ("mowgli_map_replace_" + std::to_string(::getpid()) + ".dat"))
                      .string();
    std::filesystem::remove(areas_file_);
    std::filesystem::remove(areas_file_ + ".tmp");
  }

  void TearDown() override
  {
    node_.reset();
    std::filesystem::remove(areas_file_);
    std::filesystem::remove(areas_file_ + ".tmp");
  }

  void make_node(double resolution, bool persist, double publish_rate = 1.0)
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", resolution);
    opts.append_parameter_override("map_size_x", 20.0);
    opts.append_parameter_override("map_size_y", 20.0);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", publish_rate);
    opts.append_parameter_override("areas_file_path", persist ? areas_file_ : std::string());
    opts.append_parameter_override("robot_yaml_path", areas_file_ + ".robot.yaml");
    opts.append_parameter_override("enforce_boundary_margin_m", kOutsideMargin);
    opts.append_parameter_override("boundary_inner_margin_m", kInnerMargin);
    opts.append_parameter_override("keepout_obstacle_margin", kObstacleMargin);
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  bool add_area(const TestArea& area, uint32_t id = 0)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = area.name;
    req->area.area = area.polygon;
    req->area.obstacles = area.obstacles;
    req->area.id = id;
    req->is_navigation_area = area.is_navigation;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node_->add_area_for_test(req, res);
    return res->success;
  }

  std::string areas_file_;
  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

std::vector<TestArea> overlapping_scene()
{
  std::vector<TestArea> areas(3);
  areas[0].name = "a";
  areas[0].polygon = make_blob(0.0, 0.0, 6.0, 90, 1);
  areas[0].obstacles = {make_blob(1.5, 1.0, 0.5, 8, 2), make_blob(-5.2, 0.0, 0.4, 6, 3)};
  areas[1].name = "b (overlaps a)";
  areas[1].polygon = make_blob(8.0, 1.0, 4.0, 60, 4);
  areas[1].obstacles = {make_blob(8.5, 0.5, 0.3, 5, 5)};
  areas[2].name = "c (navigation, 0.3 m seam from a)";
  areas[2].polygon = make_blob(0.0, 11.4, 3.0, 40, 6);
  areas[2].is_navigation = true;
  return areas;
}

void expect_mask_matches_definition(const nav_msgs::msg::OccupancyGrid& mask,
                                    const std::vector<TestArea>& areas)
{
  ASSERT_FALSE(mask.data.empty());
  const auto expected = reference_mask(mask, areas);
  ASSERT_EQ(mask.data.size(), expected.size());
  std::size_t mismatches = 0;
  std::size_t free_cells = 0;
  std::size_t soft_cells = 0;
  std::size_t lethal_cells = 0;
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    mismatches += (mask.data[i] != expected[i]) ? 1 : 0;
    free_cells += (expected[i] == 0) ? 1 : 0;
    soft_cells += (expected[i] == kSoftPenalty) ? 1 : 0;
    lethal_cells += (expected[i] == 100) ? 1 : 0;
  }
  EXPECT_EQ(mismatches, 0u) << "of " << expected.size() << " cells";
  // The reference itself must not be degenerate.
  EXPECT_GT(free_cells, 1000u);
  EXPECT_GT(soft_cells, 500u);
  EXPECT_GT(lethal_cells, 1000u);
}

TEST_F(MapReplaceTest, FastKeepoutMaskIsBitIdenticalToThePerCellDefinition)
{
  // Arrange: a coarse grid keeps the naive reference affordable; the geometry
  // (overlapping areas and bands, concave edges, an obstacle on a boundary, a
  // narrow seam) is what matters, not the cell count.
  make_node(0.1, /*persist=*/false);
  const auto areas = overlapping_scene();
  for (const auto& area : areas)
  {
    ASSERT_TRUE(add_area(area));
  }

  // Act
  const auto mask = node_->build_keepout_mask_for_test();

  // Assert
  expect_mask_matches_definition(mask, areas);
}

TEST_F(MapReplaceTest, FastKeepoutMaskIsBitIdenticalOnARealisticGarden)
{
  // Arrange: the 400-vertex garden, at 0.1 m so the naive reference stays in
  // the seconds.
  make_node(0.1, /*persist=*/false);
  const auto areas = realistic_garden();
  for (const auto& area : areas)
  {
    ASSERT_TRUE(add_area(area));
  }

  // Act
  const auto mask = node_->build_keepout_mask_for_test();

  // Assert
  expect_mask_matches_definition(mask, areas);
}

TEST(PolygonRasterTest, LineFillVisitsExactlyTheCellsOfGridMapsPolygonIterator)
{
  // Arrange: the CLASSIFICATION layer used to be stamped with
  // grid_map::PolygonIterator; the line fill must select the same cells.
  grid_map::GridMap grid({"expected", "actual"});
  grid.setGeometry(grid_map::Length(30.0, 24.0), 0.05, grid_map::Position(1.3, -0.7));
  grid["expected"].setConstant(0.0F);
  grid["actual"].setConstant(0.0F);
  const std::vector<Polygon> polygons{make_blob(0.0, 0.0, 8.0, 200, 41),
                                      make_blob(4.0, -3.0, 0.6, 8, 42),
                                      make_blob(14.0, 9.0, 5.0, 60, 43)};  // partly off-grid

  mowgli_map::raster::CellAxes axes;
  grid_map::Position pos;
  for (int r = 0; r < grid.getSize()(0); ++r)
  {
    grid.getPosition(grid_map::Index(r, 0), pos);
    axes.x_of_row.push_back(pos.x());
  }
  for (int c = 0; c < grid.getSize()(1); ++c)
  {
    grid.getPosition(grid_map::Index(0, c), pos);
    axes.y_of_col.push_back(pos.y());
  }
  const mowgli_map::raster::CellWindow whole{{0, grid.getSize()(0) - 1},
                                             {0, grid.getSize()(1) - 1}};

  // Act
  auto& actual = grid["actual"];
  for (const auto& polygon : polygons)
  {
    grid_map::Polygon gm;
    for (const auto& p : polygon.points)
    {
      gm.addVertex(grid_map::Position(p.x, p.y));
    }
    for (grid_map::PolygonIterator it(grid, gm); !it.isPastEnd(); ++it)
    {
      grid.at("expected", *it) += 1.0F;
    }
    mowgli_map::raster::for_each_cell_inside<double>(polygon,
                                                     axes,
                                                     whole,
                                                     [&actual](int r, int c)
                                                     {
                                                       actual(r, c) += 1.0F;
                                                     });
  }

  // Assert
  EXPECT_GT(grid["expected"].sum(), 50000.0F);
  EXPECT_TRUE((grid["expected"].array() == grid["actual"].array()).all())
      << ((grid["expected"].array() != grid["actual"].array()).count()) << " cells differ";
}

TEST(PolygonRasterTest, CoveringRangeNeverDropsACellAndStaysInBounds)
{
  // Arrange: a decreasing axis, as grid_map hands out.
  const std::vector<double> axis{4.5, 3.5, 2.5, 1.5, 0.5};

  // Act + Assert: every index whose value is in [lo, hi] is covered...
  const auto inner = mowgli_map::raster::covering_range(axis, 1.4, 3.6);
  EXPECT_LE(inner.first, 1);
  EXPECT_GE(inner.last, 3);
  // ...a range beyond either end clamps instead of indexing out of bounds...
  const auto above = mowgli_map::raster::covering_range(axis, 10.0, 20.0);
  EXPECT_GE(above.first, 0);
  EXPECT_LE(above.last, 4);
  const auto below = mowgli_map::raster::covering_range(axis, -20.0, -10.0);
  EXPECT_GE(below.first, 0);
  EXPECT_LE(below.last, 4);
  // ...and nonsense input selects nothing.
  EXPECT_TRUE(mowgli_map::raster::covering_range(axis, 3.0, 1.0).empty());
  EXPECT_TRUE(mowgli_map::raster::covering_range({}, 0.0, 1.0).empty());
  EXPECT_TRUE(mowgli_map::raster::covering_range(axis, std::nan(""), 1.0).empty());
}

TEST_F(MapReplaceTest, ReplacingARealisticGardenIsCheap)
{
  // Arrange: shipped resolution and margins, persistence on (as on the robot).
  make_node(0.05, /*persist=*/true);
  const auto areas = realistic_garden();

  // Act: what the GUI's "Save map" does, including the mask rebuild that the
  // publish timer used to run between two add_area calls.
  double add_total_s = 0.0;
  double add_worst_s = 0.0;
  for (const auto& area : areas)
  {
    const auto t0 = Clock::now();
    ASSERT_TRUE(add_area(area));
    const double dt = seconds_since(t0);
    add_total_s += dt;
    add_worst_s = std::max(add_worst_s, dt);
  }
  const auto t_mask = Clock::now();
  const auto mask = node_->build_keepout_mask_for_test();
  const double mask_s = seconds_since(t_mask);

  // Assert
  std::printf(
      "[ perf ] grid %u x %u cells, add_area total %.3f s (worst %.3f s), "
      "keepout mask rebuild %.3f s\n",
      mask.info.width,
      mask.info.height,
      add_total_s,
      add_worst_s,
      mask_s);
  ASSERT_GT(static_cast<std::size_t>(mask.info.width) * mask.info.height, 1000000u)
      << "the scenario must stay a realistically large grid";
  // Budgets are ~20x what a desktop needs so a loaded CI runner passes, and
  // ~50x below what the per-cell implementation needed on the same machine.
  EXPECT_LT(add_worst_s, 0.25) << "add_area must not rasterise inside the service callback";
  EXPECT_LT(mask_s, 2.0) << "the mask rebuild must not scale with cells x vertices";
}

TEST_F(MapReplaceTest, MaskIsRebuiltOnceAfterABurstOfEditsNotBetweenThem)
{
  // Arrange: a fast publish timer, so only the settle window can hold the
  // rebuild back; a latched subscriber counts what Nav2 would receive.
  make_node(0.1, /*persist=*/false, /*publish_rate=*/20.0);
  auto listener = std::make_shared<rclcpp::Node>("keepout_listener");
  std::vector<nav_msgs::msg::OccupancyGrid> received;
  auto sub = listener->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/keepout_mask",
      rclcpp::QoS(1).transient_local(),
      [&received](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
      {
        received.push_back(*msg);
      });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(listener);
  const auto spin_for = [&exec](std::chrono::milliseconds duration)
  {
    const auto until = Clock::now() + duration;
    while (Clock::now() < until)
    {
      exec.spin_some(std::chrono::milliseconds(20));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };
  const auto areas = overlapping_scene();

  // Act: three add_area calls 300 ms apart — a GUI replace on a slow link.
  for (const auto& area : areas)
  {
    ASSERT_TRUE(add_area(area));
    spin_for(std::chrono::milliseconds(300));
  }
  const std::size_t during_burst = received.size();
  spin_for(std::chrono::milliseconds(2200));

  // Assert
  EXPECT_EQ(during_burst, 0u) << "a half-built map reached the keepout filter";
  ASSERT_EQ(received.size(), 1u) << "exactly one rebuild after the burst";
  expect_mask_matches_definition(received.front(), areas);
}

TEST_F(MapReplaceTest, AreasFileIsReplacedAtomically)
{
  // Arrange: one area already persisted.
  make_node(0.1, /*persist=*/true);
  const auto areas = overlapping_scene();
  ASSERT_TRUE(add_area(areas[0]));
  ASSERT_TRUE(std::filesystem::exists(areas_file_));
  EXPECT_FALSE(std::filesystem::exists(areas_file_ + ".tmp")) << "temp file left behind";
  const auto read_all = [](const std::string& path)
  {
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  };
  const std::string good = read_all(areas_file_);
  ASSERT_NE(good.find("area_count: 1"), std::string::npos);

  // Act: the next save cannot even open its temp file (stand-in for a full or
  // read-only SD card).
  std::filesystem::create_directory(areas_file_ + ".tmp");
  EXPECT_THROW(node_->save_areas_for_test(areas_file_), std::exception);
  const std::string after_failure = read_all(areas_file_);
  std::filesystem::remove(areas_file_ + ".tmp");

  // Assert: the previous map is still there, byte for byte...
  EXPECT_EQ(after_failure, good) << "a failed save damaged the only copy of the map";

  // ...and once the medium recovers, the save goes through and round-trips.
  ASSERT_TRUE(add_area(areas[1]));
  EXPECT_NE(read_all(areas_file_).find("area_count: 2"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(areas_file_ + ".tmp"));
  node_.reset();
  make_node(0.1, /*persist=*/true);
  auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  req->index = 1;
  node_->get_mowing_area_for_test(req, res);
  ASSERT_TRUE(res->success);
  EXPECT_EQ(res->area.name, areas[1].name);
  EXPECT_EQ(res->area.area.points.size(), areas[1].polygon.points.size());
  EXPECT_EQ(res->area.obstacles.size(), areas[1].obstacles.size());
}

}  // namespace
