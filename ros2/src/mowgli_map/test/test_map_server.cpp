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
