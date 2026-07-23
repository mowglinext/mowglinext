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

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"
#include <gtest/gtest.h>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture — creates a MapServerNode with a small 10×10 m map
// ─────────────────────────────────────────────────────────────────────────────

// Global init/shutdown for all test suites
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class MapServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);

    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 — grid_map creation with correct layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, GridMapHasAllRequiredLayers)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::OCCUPANCY)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::CLASSIFICATION)));
}

TEST_F(MapServerTest, GridMapGeometryIsCorrect)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // 10 m / 0.1 m resolution = 100 cells per axis
  EXPECT_EQ(m.getSize()(0), 100);
  EXPECT_EQ(m.getSize()(1), 100);

  EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  EXPECT_EQ(m.getFrameId(), "map");
}

TEST_F(MapServerTest, GridMapLayersInitialisedToDefaults)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // All occupancy cells must be 0.0 (free)
  const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
  EXPECT_FLOAT_EQ(occ.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(occ.maxCoeff(), 0.0F);

  // All classification cells must be 0.0 (UNKNOWN)
  const auto& cls = m[std::string(mowgli_map::layers::CLASSIFICATION)];
  EXPECT_FLOAT_EQ(cls.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(cls.maxCoeff(), 0.0F);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 — classification enum values
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, CellTypeEnumValues)
{
  using mowgli_map::CellType;

  EXPECT_EQ(static_cast<uint8_t>(CellType::UNKNOWN), 0u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::LAWN), 1u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_PERMANENT), 2u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_TEMPORARY), 3u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::NO_GO_ZONE), 4u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::DOCKING_AREA), 5u);
}

TEST_F(MapServerTest, CellTypeNamesAreCorrect)
{
  using mowgli_map::cell_type_name;
  using mowgli_map::CellType;

  EXPECT_EQ(cell_type_name(CellType::UNKNOWN), "UNKNOWN");
  EXPECT_EQ(cell_type_name(CellType::LAWN), "LAWN");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_PERMANENT), "OBSTACLE_PERMANENT");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_TEMPORARY), "OBSTACLE_TEMPORARY");
  EXPECT_EQ(cell_type_name(CellType::NO_GO_ZONE), "NO_GO_ZONE");
  EXPECT_EQ(cell_type_name(CellType::DOCKING_AREA), "DOCKING_AREA");
}

TEST_F(MapServerTest, ClassificationLayerDefaultIsUnknown)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& cls = node_->map()[std::string(mowgli_map::layers::CLASSIFICATION)];
  EXPECT_FLOAT_EQ(cls.minCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
  EXPECT_FLOAT_EQ(cls.maxCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 — map clear resets all layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, ClearMapResetsAllLayersToDefault)
{
  // Dirty the kept layers
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    auto& m = node_->map();
    grid_map::Index centre_idx;
    ASSERT_TRUE(m.getIndex(grid_map::Position(0.0, 0.0), centre_idx));
    m.at(std::string(mowgli_map::layers::OCCUPANCY), centre_idx) = 1.0F;
    m.at(std::string(mowgli_map::layers::CLASSIFICATION), centre_idx) =
        static_cast<float>(mowgli_map::CellType::NO_GO_ZONE);
  }

  // Now clear
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
  }

  // Verify all layers are back to defaults
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& m = node_->map();

    const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
    const auto& cls = m[std::string(mowgli_map::layers::CLASSIFICATION)];

    EXPECT_FLOAT_EQ(occ.maxCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.maxCoeff(), mowgli_map::defaults::CLASSIFICATION);

    EXPECT_FLOAT_EQ(occ.minCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.minCoeff(), mowgli_map::defaults::CLASSIFICATION);
  }
}

TEST_F(MapServerTest, ClearMapDoesNotChangeGeometry)
{
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
    const auto& m = node_->map();
    EXPECT_EQ(m.getSize()(0), 100);
    EXPECT_EQ(m.getSize()(1), 100);
    EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Area type tests — mowing vs navigation classification + persistence
// ─────────────────────────────────────────────────────────────────────────────

class AreaTypeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("areas_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  static geometry_msgs::msg::Polygon make_rect(double x0, double y0, double x1, double y1)
  {
    geometry_msgs::msg::Polygon p;
    auto add = [&](double x, double y)
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(x);
      pt.y = static_cast<float>(y);
      pt.z = 0.0F;
      p.points.push_back(pt);
    };
    add(x0, y0);
    add(x1, y0);
    add(x1, y1);
    add(x0, y1);
    return p;
  }

  bool add_area(const std::string& name,
                const geometry_msgs::msg::Polygon& poly,
                bool is_navigation)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = name;
    req->area.area = poly;
    req->is_navigation_area = is_navigation;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node_->add_area_for_test(req, res);
    return res->success;
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

// Sample a keepout OccupancyGrid at a world (x, y) point using the SAME
// flat-index convention map_server uses on the publish side: col=0 ↔ origin.x
// (X_min), row=0 ↔ origin.y (Y_min), data[row*width + col]. Reproducing it
// here (rather than reusing the producer's r/c→og math) is the point of the
// test — if the producer ever swaps width/height again, this read lands on a
// different cell and the assertions fail.
static int8_t mask_at(const nav_msgs::msg::OccupancyGrid& m, double x, double y)
{
  const int col = static_cast<int>(std::floor((x - m.info.origin.position.x) / m.info.resolution));
  const int row = static_cast<int>(std::floor((y - m.info.origin.position.y) / m.info.resolution));
  if (col < 0 || row < 0 || col >= static_cast<int>(m.info.width) ||
      row >= static_cast<int>(m.info.height))
  {
    return -2;  // out of bounds sentinel
  }
  return m.data[static_cast<std::size_t>(row) * m.info.width + col];
}

TEST_F(AreaTypeTest, NavigationAreaIsNotStoredAsMowing)
{
  ASSERT_TRUE(add_area("nav_corridor", make_rect(-2, -2, 2, 2), /*is_navigation=*/true));

  auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req->index = 0;
  auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  node_->get_mowing_area_for_test(req, res);

  ASSERT_TRUE(res->success);
  EXPECT_EQ(res->area.name, "nav_corridor");
  EXPECT_TRUE(res->area.is_navigation_area) << "navigation area was misclassified as a mowing area";
}

TEST_F(AreaTypeTest, MowingAndNavigationAreasArePreservedSideBySide)
{
  ASSERT_TRUE(add_area("mow_lawn", make_rect(-3, -3, 0, 0), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("nav_corridor", make_rect(0, 0, 3, 3), /*is_navigation=*/true));

  // Index 0 — mowing
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    req->index = 0;
    auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
    node_->get_mowing_area_for_test(req, res);
    ASSERT_TRUE(res->success);
    EXPECT_EQ(res->area.name, "mow_lawn");
    EXPECT_FALSE(res->area.is_navigation_area);
  }
  // Index 1 — navigation
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    req->index = 1;
    auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
    node_->get_mowing_area_for_test(req, res);
    ASSERT_TRUE(res->success);
    EXPECT_EQ(res->area.name, "nav_corridor");
    EXPECT_TRUE(res->area.is_navigation_area);
  }
}

TEST_F(AreaTypeTest, NavigationAreaSurvivesSaveLoadRoundTrip)
{
  ASSERT_TRUE(add_area("mow_lawn", make_rect(-3, -3, 0, 0), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("nav_corridor", make_rect(0, 0, 3, 3), /*is_navigation=*/true));

  // Persist to a temp file, then reload from disk into a fresh node.
  const std::string tmp_path =
      std::string(std::getenv("TEST_TMPDIR") ? std::getenv("TEST_TMPDIR") : "/tmp") +
      "/mowgli_areas_roundtrip.dat";
  node_->save_areas_for_test(tmp_path);

  // Reload into the same node — clears in-memory areas first.
  node_->load_areas_for_test(tmp_path);

  auto req0 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req0->index = 0;
  auto res0 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  node_->get_mowing_area_for_test(req0, res0);
  ASSERT_TRUE(res0->success);
  EXPECT_EQ(res0->area.name, "mow_lawn");
  EXPECT_FALSE(res0->area.is_navigation_area);

  auto req1 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req1->index = 1;
  auto res1 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  node_->get_mowing_area_for_test(req1, res1);
  ASSERT_TRUE(res1->success);
  EXPECT_EQ(res1->area.name, "nav_corridor");
  EXPECT_TRUE(res1->area.is_navigation_area) << "navigation flag lost across save/load round trip";

  std::remove(tmp_path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Promotion idempotency (FIX B): a single promote → exactly one permanent
// obstacle; a re-promote of the same keepout is a no-op; a genuinely distinct
// obstacle is still added.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(AreaTypeTest, PromoteObstacleIsIdempotent)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));

  const auto obs = make_rect(0.0, 0.0, 0.5, 0.5);  // centroid (0.25, 0.25)
  EXPECT_TRUE(node_->apply_promoted_obstacle_for_test(0, obs));
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 1u);
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u);

  // Re-promote the identical polygon — must be a no-op (no stacking).
  EXPECT_TRUE(node_->apply_promoted_obstacle_for_test(0, obs));
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 1u) << "re-promote stacked a duplicate";
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u) << "re-promote stacked a duplicate";

  // A near-identical polygon (centroid within kObstacleDedupEpsilonM) is also
  // treated as the same keepout.
  const auto obs_shifted = make_rect(0.02, 0.02, 0.52, 0.52);  // centroid (0.27, 0.27)
  EXPECT_TRUE(node_->apply_promoted_obstacle_for_test(0, obs_shifted));
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 1u) << "near-duplicate stacked";

  // A genuinely distinct obstacle (centroid well beyond the epsilon) is added.
  const auto obs2 = make_rect(1.5, 1.5, 2.0, 2.0);  // centroid (1.75, 1.75)
  EXPECT_TRUE(node_->apply_promoted_obstacle_for_test(0, obs2));
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 2u) << "distinct obstacle was wrongly merged";
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 2u);
}

// ─────────────────────────────────────────────────────────────────────────────
// Keepout mask — lethal-outside-areas boundary policy + index convention.
//
// Worked example mirroring areas.dat's quadrilateral shape: a single mowing
// rectangle from (-3,-2) to (3,2). With the default lethal_outside_areas=true
// and enforce_boundary_margin_m=0.25, the mask must be:
//   * FREE (0) for a point well INSIDE the rectangle,
//   * FREE (0) for a point just OUTSIDE the edge but within 0.25 m,
//   * LETHAL (100) for a point far OUTSIDE the rectangle (> 0.25 m past edge).
// The mask is read back with the independent OccupancyGrid convention in
// mask_at(), so a swapped width/height (the historical 90°-rotation bug,
// CLAUDE.md #14) would put the interior point on a lethal cell and fail.
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(AreaTypeTest, KeepoutMaskMarksOutsideAreasLethal)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -2, 3, 2), /*is_navigation=*/false));

  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_GT(mask.info.width, 0u);
  ASSERT_GT(mask.info.height, 0u);
  ASSERT_EQ(mask.data.size(), static_cast<std::size_t>(mask.info.width) * mask.info.height);

  // Deep interior → FREE. (If width/height were swapped this lands elsewhere.)
  EXPECT_EQ(mask_at(mask, 0.0, 0.0), 0) << "interior of mowing area must be free";
  EXPECT_EQ(mask_at(mask, 2.0, 1.0), 0) << "interior corner of mowing area must be free";

  // Just outside the +X edge but within enforce_boundary_margin_m (0.25) → FREE.
  EXPECT_EQ(mask_at(mask, 3.10, 0.0), 0) << "RTK-drift slack band outside edge must be free";

  // Far outside the rectangle (> 0.25 m past the edge) → LETHAL.
  EXPECT_EQ(mask_at(mask, 4.0, 0.0), 100) << "cell well outside all areas must be lethal";
  EXPECT_EQ(mask_at(mask, 0.0, 3.5), 100) << "cell well outside all areas must be lethal";
}

// Navigation areas count toward the allowed (free) region just like mowing
// areas — this is what lets the operator draw the dock/transit corridor as a
// navigation area so the hard boundary does not strand docking.
TEST_F(AreaTypeTest, KeepoutMaskTreatsNavigationAreasAsAllowed)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 0, 0), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("corridor", make_rect(0, 0, 3, 3), /*is_navigation=*/true));

  const auto mask = node_->build_keepout_mask_for_test();

  EXPECT_EQ(mask_at(mask, -1.5, -1.5), 0) << "inside mowing area must be free";
  EXPECT_EQ(mask_at(mask, 1.5, 1.5), 0) << "inside navigation area must be free";
  // A point outside BOTH areas, well past any edge, must be lethal.
  EXPECT_EQ(mask_at(mask, -2.5, 2.5), 100) << "outside both areas must be lethal";
}

// No areas defined (fresh install / empty areas.dat): the mask must NOT make
// the whole world lethal — publish_keepout_mask early-returns and never caches
// a mask, so the costmap sees no keepout filter mask at all (everything
// drivable). Asserting the cached mask is empty captures that contract.
TEST_F(AreaTypeTest, KeepoutMaskEmptyWhenNoAreas)
{
  const auto mask = node_->build_keepout_mask_for_test();
  EXPECT_TRUE(mask.data.empty())
      << "with zero areas, no keepout mask is produced (world stays drivable)";
}

// ─────────────────────────────────────────────────────────────────────────────
// Drawn-obstacle margin (mowgli_robot.yaml.obstacle_margin) — the keepout
// twin of coverage_server's F2C hole buffering. A drawn obstacle (a tree)
// must project a LETHAL band obstacle_margin wide around its polygon so
// transit paths keep off root zones the 2D LiDAR cannot see.
// ─────────────────────────────────────────────────────────────────────────────

class ObstacleMarginTest : public AreaTypeTest
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("areas_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    opts.append_parameter_override("obstacle_margin", 0.3);
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  bool add_area_with_obstacle(const geometry_msgs::msg::Polygon& area,
                              const geometry_msgs::msg::Polygon& obstacle)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = "lawn_with_tree";
    req->area.area = area;
    req->area.obstacles.push_back(obstacle);
    req->is_navigation_area = false;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node_->add_area_for_test(req, res);
    return res->success;
  }
};

TEST_F(ObstacleMarginTest, DrawnObstacleGetsLethalMarginBand)
{
  // 8×8 m lawn with a 1×1 m drawn obstacle (tree) centred at the origin.
  ASSERT_TRUE(add_area_with_obstacle(make_rect(-4, -4, 4, 4), make_rect(-0.5, -0.5, 0.5, 0.5)));

  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());

  // Inside the drawn obstacle → LETHAL (classification NO_GO overlay).
  EXPECT_EQ(mask_at(mask, 0.0, 0.0), 100) << "inside drawn obstacle must be lethal";
  // 0.2 m outside the polygon edge, within the 0.3 m margin → LETHAL.
  EXPECT_EQ(mask_at(mask, 0.75, 0.0), 100) << "cell inside the obstacle_margin band must be lethal";
  // Well outside the margin band (edge + 0.3 m + slack) → FREE lawn.
  EXPECT_EQ(mask_at(mask, 1.5, 0.0), 0) << "lawn beyond the margin band must stay free";
}

TEST_F(AreaTypeTest, DrawnObstacleWithDefaultMarginIsEdgeTight)
{
  // Default node (obstacle_margin = 0.15): the drawn obstacle is lethal and
  // projects a 0.15 m margin band.
  auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
  req->area.name = "lawn_with_tree";
  req->area.area = make_rect(-4, -4, 4, 4);
  req->area.obstacles.push_back(make_rect(-0.5, -0.5, 0.5, 0.5));
  req->is_navigation_area = false;
  auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
  node_->add_area_for_test(req, res);
  ASSERT_TRUE(res->success);

  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());

  EXPECT_EQ(mask_at(mask, 0.0, 0.0), 100) << "inside drawn obstacle must be lethal";
  EXPECT_EQ(mask_at(mask, 0.75, 0.0), 0)
      << "without obstacle_margin the band outside the polygon stays free";
}

// ─────────────────────────────────────────────────────────────────────────────
// Datum-change migration (issue #216) — areas.dat is stamped with the datum
// its metre coordinates were recorded against; on load under a DIFFERENT
// datum, every polygon and the dock pose must be re-projected into the new
// frame (old-ENU → WGS84 → new-ENU) instead of silently shifting across the
// garden, and both areas.dat and mowgli_robot.yaml must be re-persisted.
// ─────────────────────────────────────────────────────────────────────────────

#include <cstdio>
#include <fstream>
#include <sstream>

namespace
{

// Independent re-implementation of the equirectangular reprojection chain
// (NOT reusing mowgli_interfaces/wgs84_projection.hpp — the point of the
// test is to catch a regression in the production math).
constexpr double kTestMetersPerDeg = 6378137.0 * M_PI / 180.0;

void expected_reproject(
    double old_lat, double old_lon, double new_lat, double new_lon, double& x, double& y)
{
  const double lat = old_lat + y / kTestMetersPerDeg;
  const double lon = old_lon + x / (kTestMetersPerDeg * std::cos(old_lat * M_PI / 180.0));
  x = (lon - new_lon) * kTestMetersPerDeg * std::cos(new_lat * M_PI / 180.0);
  y = (lat - new_lat) * kTestMetersPerDeg;
}

std::string read_file(const std::string& path)
{
  std::ifstream in(path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

double yaml_scalar(const std::string& content, const std::string& key)
{
  const auto pos = content.find(key + ":");
  if (pos == std::string::npos)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::stod(content.substr(pos + key.size() + 1));
}

}  // namespace

class DatumMigrationTest : public ::testing::Test
{
protected:
  // Munich sim datum as the "recorded against" anchor; the second datum is
  // 1e-5° north / 2e-5° west of it — a ~1.1 m / ~1.5 m base relocation.
  static constexpr double kOldLat = 48.137154;
  static constexpr double kOldLon = 11.576124;
  static constexpr double kNewLat = 48.137164;
  static constexpr double kNewLon = 11.576104;

  void SetUp() override
  {
    const std::string tmp_dir = std::getenv("TEST_TMPDIR") ? std::getenv("TEST_TMPDIR") : "/tmp";
    areas_path_ = tmp_dir + "/mowgli_datum_migration_areas.dat";
    yaml_path_ = tmp_dir + "/mowgli_datum_migration_robot.yaml";
    std::remove(areas_path_.c_str());
    write_robot_yaml(1.0, 2.0, 0.5);
  }

  void TearDown() override
  {
    std::remove(areas_path_.c_str());
    std::remove(yaml_path_.c_str());
  }

  void write_robot_yaml(double x, double y, double yaw) const
  {
    std::ofstream out(yaml_path_, std::ios::trunc);
    out << "/**:\n"
        << "  ros__parameters:\n"
        << "    dock_pose_x: " << x << "\n"
        << "    dock_pose_y: " << y << "\n"
        << "    dock_pose_yaw: " << yaw << "\n";
  }

  std::shared_ptr<mowgli_map::MapServerNode> make_node(double datum_lat, double datum_lon) const
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("areas_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    opts.append_parameter_override("datum_lat", datum_lat);
    opts.append_parameter_override("datum_lon", datum_lon);
    opts.append_parameter_override("robot_yaml_path", yaml_path_);
    opts.append_parameter_override("dock_pose_x", 1.0);
    opts.append_parameter_override("dock_pose_y", 2.0);
    opts.append_parameter_override("dock_pose_yaw", 0.5);
    return std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  static geometry_msgs::msg::Polygon make_rect(double x0, double y0, double x1, double y1)
  {
    geometry_msgs::msg::Polygon p;
    auto add = [&](double x, double y)
    {
      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(x);
      pt.y = static_cast<float>(y);
      pt.z = 0.0F;
      p.points.push_back(pt);
    };
    add(x0, y0);
    add(x1, y0);
    add(x1, y1);
    add(x0, y1);
    return p;
  }

  static void add_area(mowgli_map::MapServerNode& node,
                       const std::string& name,
                       const geometry_msgs::msg::Polygon& poly,
                       const geometry_msgs::msg::Polygon* obstacle = nullptr)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = name;
    req->area.area = poly;
    if (obstacle != nullptr)
    {
      req->area.obstacles.push_back(*obstacle);
    }
    req->is_navigation_area = false;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node.add_area_for_test(req, res);
    ASSERT_TRUE(res->success);
  }

  static geometry_msgs::msg::Polygon area_polygon(mowgli_map::MapServerNode& node, uint32_t index)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    req->index = index;
    auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
    node.get_mowing_area_for_test(req, res);
    EXPECT_TRUE(res->success);
    return res->area.area;
  }

  std::string areas_path_;
  std::string yaml_path_;
};

TEST_F(DatumMigrationTest, SaveStampsCurrentDatum)
{
  auto node = make_node(kOldLat, kOldLon);
  add_area(*node, "lawn", make_rect(-2, -2, 2, 2));
  node->save_areas_for_test(areas_path_);

  const std::string content = read_file(areas_path_);
  EXPECT_NEAR(yaml_scalar(content, "datum_lat"), kOldLat, 1e-9);
  EXPECT_NEAR(yaml_scalar(content, "datum_lon"), kOldLon, 1e-9);
}

TEST_F(DatumMigrationTest, LoadWithSameDatumLeavesEverythingUntouched)
{
  {
    auto node = make_node(kOldLat, kOldLon);
    add_area(*node, "lawn", make_rect(-2, -2, 2, 2));
    node->save_areas_for_test(areas_path_);
  }

  auto node = make_node(kOldLat, kOldLon);
  node->load_areas_for_test(areas_path_);

  const auto poly = area_polygon(*node, 0);
  ASSERT_EQ(poly.points.size(), 4U);
  EXPECT_NEAR(poly.points[0].x, -2.0, 1e-4);
  EXPECT_NEAR(poly.points[0].y, -2.0, 1e-4);
  EXPECT_NEAR(node->docking_pose_for_test().position.x, 1.0, 1e-9);
  EXPECT_NEAR(node->docking_pose_for_test().position.y, 2.0, 1e-9);
}

TEST_F(DatumMigrationTest, DatumChangeReprojectsAreasObstaclesAndDock)
{
  const auto obstacle = make_rect(-0.5, -0.5, 0.5, 0.5);
  {
    auto node = make_node(kOldLat, kOldLon);
    const auto poly = make_rect(-2, -2, 2, 2);
    add_area(*node, "lawn", poly, &obstacle);
    node->save_areas_for_test(areas_path_);
  }

  // Same persisted state, but the stack now boots with a MOVED datum.
  auto node = make_node(kNewLat, kNewLon);
  node->load_areas_for_test(areas_path_);

  // Every vertex must land where the independent reprojection chain puts it.
  const auto poly = area_polygon(*node, 0);
  ASSERT_EQ(poly.points.size(), 4U);
  double ex = -2.0;
  double ey = -2.0;
  expected_reproject(kOldLat, kOldLon, kNewLat, kNewLon, ex, ey);
  EXPECT_NEAR(poly.points[0].x, ex, 1e-3);
  EXPECT_NEAR(poly.points[0].y, ey, 1e-3);
  // Sanity: the shift is metre-scale, not a no-op (≈ +1.48 m E, −1.11 m N).
  EXPECT_GT(std::abs(poly.points[0].x - (-2.0)), 1.0);
  EXPECT_GT(std::abs(poly.points[0].y - (-2.0)), 1.0);

  // Obstacle holes ride along with the same shift.
  auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req->index = 0;
  auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  node->get_mowing_area_for_test(req, res);
  ASSERT_TRUE(res->success);
  ASSERT_EQ(res->area.obstacles.size(), 1U);
  double ox = -0.5;
  double oy = -0.5;
  expected_reproject(kOldLat, kOldLon, kNewLat, kNewLon, ox, oy);
  EXPECT_NEAR(res->area.obstacles[0].points[0].x, ox, 1e-3);
  EXPECT_NEAR(res->area.obstacles[0].points[0].y, oy, 1e-3);

  // Dock pose migrated in-memory, yaw untouched (pure translation)…
  double dx = 1.0;
  double dy = 2.0;
  expected_reproject(kOldLat, kOldLon, kNewLat, kNewLon, dx, dy);
  const auto& dock = node->docking_pose_for_test();
  EXPECT_TRUE(node->docking_pose_set_for_test());
  EXPECT_NEAR(dock.position.x, dx, 1e-6);
  EXPECT_NEAR(dock.position.y, dy, 1e-6);
  const double yaw = 2.0 * std::atan2(dock.orientation.z, dock.orientation.w);
  EXPECT_NEAR(yaw, 0.5, 1e-9);

  // …and spliced back into mowgli_robot.yaml (6-decimal persist precision).
  const std::string yaml = read_file(yaml_path_);
  EXPECT_NEAR(yaml_scalar(yaml, "dock_pose_x"), dx, 1e-5);
  EXPECT_NEAR(yaml_scalar(yaml, "dock_pose_y"), dy, 1e-5);
  EXPECT_NEAR(yaml_scalar(yaml, "dock_pose_yaw"), 0.5, 1e-5);

  // areas.dat re-stamped with the new datum → the migration runs once.
  const std::string content = read_file(areas_path_);
  EXPECT_NEAR(yaml_scalar(content, "datum_lat"), kNewLat, 1e-9);
  EXPECT_NEAR(yaml_scalar(content, "datum_lon"), kNewLon, 1e-9);

  // Idempotence: a second load under the same datum must not move anything.
  node->load_areas_for_test(areas_path_);
  const auto poly2 = area_polygon(*node, 0);
  EXPECT_NEAR(poly2.points[0].x, poly.points[0].x, 1e-4);
  EXPECT_NEAR(poly2.points[0].y, poly.points[0].y, 1e-4);
}

TEST_F(DatumMigrationTest, UnstampedLegacyFileIsAdoptedWithoutShift)
{
  // Pre-#216 areas.dat: no datum stamp. Simulate by saving from a node whose
  // datum is unset (stamp is only written when a datum is configured).
  {
    auto node = make_node(0.0, 0.0);
    add_area(*node, "lawn", make_rect(-2, -2, 2, 2));
    node->save_areas_for_test(areas_path_);
  }
  ASSERT_TRUE(read_file(areas_path_).find("datum_lat") == std::string::npos);

  auto node = make_node(kNewLat, kNewLon);
  node->load_areas_for_test(areas_path_);

  // Coordinates adopted as-is (they are anchored to the current datum by
  // definition — there is no old datum to migrate from)…
  const auto poly = area_polygon(*node, 0);
  EXPECT_NEAR(poly.points[0].x, -2.0, 1e-4);
  EXPECT_NEAR(poly.points[0].y, -2.0, 1e-4);
  EXPECT_NEAR(node->docking_pose_for_test().position.x, 1.0, 1e-9);

  // …but the file gains a stamp so the NEXT datum change migrates correctly.
  const std::string content = read_file(areas_path_);
  EXPECT_NEAR(yaml_scalar(content, "datum_lat"), kNewLat, 1e-9);
  EXPECT_NEAR(yaml_scalar(content, "datum_lon"), kNewLon, 1e-9);
}

TEST_F(DatumMigrationTest, NodeWithoutDatumNeverMigrates)
{
  {
    auto node = make_node(kOldLat, kOldLon);
    add_area(*node, "lawn", make_rect(-2, -2, 2, 2));
    node->save_areas_for_test(areas_path_);
  }

  // Datum unset (0/0 template default) — the stamped file must be loaded
  // verbatim and left untouched on disk.
  auto node = make_node(0.0, 0.0);
  node->load_areas_for_test(areas_path_);

  const auto poly = area_polygon(*node, 0);
  EXPECT_NEAR(poly.points[0].x, -2.0, 1e-4);
  EXPECT_NEAR(poly.points[0].y, -2.0, 1e-4);

  const std::string content = read_file(areas_path_);
  EXPECT_NEAR(yaml_scalar(content, "datum_lat"), kOldLat, 1e-9);
  EXPECT_NEAR(yaml_scalar(content, "datum_lon"), kOldLon, 1e-9);
}
