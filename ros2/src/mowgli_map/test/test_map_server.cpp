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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"
#include <gtest/gtest.h>
#include <mowgli_interfaces/msg/dig_event.hpp>
#include <mowgli_interfaces/msg/map_obstacle_info.hpp>
#include <mowgli_interfaces/srv/add_mowing_area.hpp>
#include <mowgli_interfaces/srv/clear_obstacle.hpp>
#include <mowgli_interfaces/srv/get_mowing_area.hpp>
#include <mowgli_interfaces/srv/promote_obstacle.hpp>
#include <mowgli_interfaces/srv/set_docking_point.hpp>

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

  // id=0 (default) mints a fresh one, matching every pre-#637 call site
  // below; pass a non-zero value to exercise the round-trip-preserve path
  // (mowglinext#637).
  bool add_area(const std::string& name,
                const geometry_msgs::msg::Polygon& poly,
                bool is_navigation,
                uint32_t id = 0)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = name;
    req->area.area = poly;
    req->is_navigation_area = is_navigation;
    req->area.id = id;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node_->add_area_for_test(req, res);
    return res->success;
  }

  // Fetch a single area's response (name/geometry/id/...) by index.
  mowgli_interfaces::srv::GetMowingArea::Response::SharedPtr get_area(uint32_t index)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    req->index = index;
    auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
    node_->get_mowing_area_for_test(req, res);
    return res;
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
// Stable per-area id (mowglinext#637). The positional index GetMowingArea/
// StartInArea use is fragile — the GUI's edit/delete flow rebuilds the WHOLE
// area list (clear_map + add_area per area) on any single-area change, which
// can reassign every index at once. This id is meant to survive that.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(AreaTypeTest, NewAreaGetsANonZeroId)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-2, -2, 2, 2), /*is_navigation=*/false));
  EXPECT_NE(get_area(0)->area.id, 0u)
      << "a freshly added area must never keep the msg default id=0";
}

TEST_F(AreaTypeTest, TwoNewAreasGetDistinctIds)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 0, 0), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("nav", make_rect(0, 0, 3, 3), /*is_navigation=*/true));
  const uint32_t id0 = get_area(0)->area.id;
  const uint32_t id1 = get_area(1)->area.id;
  EXPECT_NE(id0, 0u);
  EXPECT_NE(id1, 0u);
  EXPECT_NE(id0, id1) << "two areas added in the same session must not share an id";
}

TEST_F(AreaTypeTest, AreaIdSurvivesSaveLoadRoundTrip)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-2, -2, 2, 2), /*is_navigation=*/false));
  const uint32_t id_before = get_area(0)->area.id;
  ASSERT_NE(id_before, 0u);

  const std::string tmp_path =
      std::string(std::getenv("TEST_TMPDIR") ? std::getenv("TEST_TMPDIR") : "/tmp") +
      "/mowgli_areas_id_roundtrip.dat";
  node_->save_areas_for_test(tmp_path);
  node_->load_areas_for_test(tmp_path);

  EXPECT_EQ(get_area(0)->area.id, id_before) << "id must not change across a save/load round trip";
  std::remove(tmp_path.c_str());
}

TEST_F(AreaTypeTest, ReAddingAnAreaWithAnExplicitIdPreservesIt)
{
  // Simulates the GUI's edit/delete flow: clear_map, then re-add every area
  // in whatever order the client holds them, round-tripping each one's
  // existing id so an edit to ONE area does not disturb the others'.
  constexpr uint32_t kPreservedId = 4242;
  ASSERT_TRUE(add_area("lawn", make_rect(-2, -2, 2, 2), /*is_navigation=*/false, kPreservedId));
  EXPECT_EQ(get_area(0)->area.id, kPreservedId)
      << "a caller-supplied id must be honored, not silently overwritten by a fresh mint";
}

TEST_F(AreaTypeTest, ADuplicateRoundTrippedIdIsRejectedAndMintedAfresh)
{
  // The GUI's edit/delete flow replays one add_area per area, so a client
  // holding a duplicated or stale list can offer the same id twice. Two areas
  // sharing an identity would silently corrupt everything keyed on it — the
  // resume cursor, the mow-progress bookkeeping, the GUI's selection.
  constexpr uint32_t kId = 77;
  ASSERT_TRUE(add_area("first", make_rect(-3, -3, 0, 0), /*is_navigation=*/false, kId));
  ASSERT_TRUE(add_area("second", make_rect(0, 0, 3, 3), /*is_navigation=*/false, kId));

  EXPECT_EQ(get_area(0)->area.id, kId) << "the first claim on an id keeps it";
  const uint32_t second_id = get_area(1)->area.id;
  EXPECT_NE(second_id, kId) << "two areas must never share an id";
  EXPECT_NE(second_id, 0u) << "the duplicate must get a real id, not none at all";
}

TEST_F(AreaTypeTest, PreservingAHighIdAdvancesTheCounterPastIt)
{
  // Round-trip an id far above whatever this fresh node's counter starts
  // at, then add a genuinely NEW area (id left 0) in the same session — it
  // must not collide with the preserved one.
  constexpr uint32_t kPreservedId = 999;
  ASSERT_TRUE(
      add_area("preserved", make_rect(-3, -3, 0, 0), /*is_navigation=*/false, kPreservedId));
  ASSERT_TRUE(add_area("fresh", make_rect(0, 0, 3, 3), /*is_navigation=*/false));

  EXPECT_EQ(get_area(0)->area.id, kPreservedId);
  const uint32_t fresh_id = get_area(1)->area.id;
  EXPECT_NE(fresh_id, 0u);
  EXPECT_NE(fresh_id, kPreservedId)
      << "a freshly minted id collided with a round-tripped one — next_area_id_ did not advance";
}

TEST_F(AreaTypeTest, LegacyAreasFileWithoutIdsGetsIdsAssignedAndReSaved)
{
  // Exactly the pre-#637 on-disk format — no area_N_id / next_area_id
  // lines, mirroring LegacyAreasFileWithoutObstacleIdentityStillLoads
  // (DigProposalTest) for the analogous #502 obstacle-identity migration.
  const std::string tmp_path =
      std::string(std::getenv("TEST_TMPDIR") ? std::getenv("TEST_TMPDIR") : "/tmp") +
      "/mowgli_areas_legacy_no_id.dat";
  {
    std::ofstream out(tmp_path);
    out << "# Mowgli ROS2 - Persisted areas and docking point\n\n";
    out << "area_count: 2\n\n";
    out << "area_0_name: lawn\n";
    out << "area_0_polygon: -3,-3;3,-3;3,3;-3,3\n";
    out << "area_0_is_navigation: 0\n";
    out << "area_0_obstacle_count: 0\n\n";
    out << "area_1_name: corridor\n";
    out << "area_1_polygon: -1,-1;1,-1;1,1;-1,1\n";
    out << "area_1_is_navigation: 1\n";
    out << "area_1_obstacle_count: 0\n\n";
  }

  node_->load_areas_for_test(tmp_path);

  const uint32_t id0 = get_area(0)->area.id;
  const uint32_t id1 = get_area(1)->area.id;
  EXPECT_NE(id0, 0u) << "a legacy area with no id line must still end up with a real one";
  EXPECT_NE(id1, 0u);
  EXPECT_NE(id0, id1);

  // The migration must have re-saved the file — a second, independent load
  // (a fresh node, exactly what happens across a real container restart)
  // must see the SAME ids, not a fresh mint every time, which would defeat
  // the whole point: an external caller that cached id0 would silently
  // start pointing at a different area after the robot restarts.
  rclcpp::NodeOptions opts2;
  opts2.append_parameter_override("resolution", 0.1);
  opts2.append_parameter_override("map_size_x", 10.0);
  opts2.append_parameter_override("map_size_y", 10.0);
  opts2.append_parameter_override("map_frame", "map");
  opts2.append_parameter_override("tool_width", 0.2);
  opts2.append_parameter_override("map_file_path", "");
  opts2.append_parameter_override("areas_file_path", "");
  opts2.append_parameter_override("publish_rate", 1.0);
  auto node2 = std::make_shared<mowgli_map::MapServerNode>(opts2);
  node2->load_areas_for_test(tmp_path);
  auto req0 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req0->index = 0;
  auto res0 = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  node2->get_mowing_area_for_test(req0, res0);
  ASSERT_TRUE(res0->success);
  EXPECT_EQ(res0->area.id, id0) << "areas.dat was not actually re-saved with the assigned id";

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
// Wheel-slip dig reports → INERT proposal (operator review, nothing applied).
//
// hardware_bridge_node detects the robot digging a hole (wheels turning while
// the GNSS-anchored pose stays put), stops and reverses out, then publishes a
// DigEvent. map_server records a PROPOSAL the operator can accept or reject in
// the GUI. Until they accept it, it is NOT a keepout, NOT a coverage hole and
// NOT in areas.dat: the robot stands ~0.2-0.3 m from the dig point, and a
// keepout stamped there refused every plan from its own pose (START_OCCUPIED,
// 2026-09-10 and 2026-09-17). Issue #500's re-dig loop is handled by
// FollowStrip's dig skip zone instead (mowgli_behavior/dig_skip.hpp).
// ─────────────────────────────────────────────────────────────────────────────

namespace
{
mowgli_interfaces::msg::DigEvent::SharedPtr make_dig_event(double x, double y)
{
  auto msg = std::make_shared<mowgli_interfaces::msg::DigEvent>();
  msg->header.frame_id = "map";
  msg->position.x = x;
  msg->position.y = y;
  msg->wheel_distance = 0.45;
  msg->map_distance = 0.02;
  msg->position_sigma = 0.004;
  return msg;
}
}  // namespace

TEST_F(AreaTypeTest, DigInsideMowingAreaBecomesAnInertProposal)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  ASSERT_EQ(node_->area_obstacle_count_for_test(0), 0u);

  node_->on_dig_event_for_test(make_dig_event(1.0, 1.0));

  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u) << "the proposal must be recorded";
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 0u)
      << "a proposal must not enter the keepout polygon store";

  const auto info = node_->obstacle_info_for_test(0, 0);
  EXPECT_TRUE(info.pending) << "a single inferred dig is a proposal, not a keepout";
  EXPECT_EQ(info.source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
  EXPECT_NE(info.id, 0u) << "a proposal needs a handle the operator can accept or discard";
  EXPECT_NE(info.name.find("Dig at"), std::string::npos)
      << "the proposal must carry its evidence: " << info.name;
}

// REQUIREMENT A, pinned: nothing the dig pipeline does on its own may mark a
// cell lethal under or around the robot, or change what coverage plans. Field
// 2026-09-17: the pending keepout (0.60 m polygon + 0.276 m band) was stamped
// 0.27 m from the robot; every transit was refused with START_OCCUPIED, the
// re-plan grew from 9 to 10 sub-paths, and the mission died mid-lawn.
TEST_F(AreaTypeTest, DigEventLeavesTheKeepoutMaskAndTheCoverageHolesUntouched)
{
  // Arrange
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  const auto mask_before = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask_before.data.empty());

  // Act
  node_->on_dig_event_for_test(make_dig_event(0.0, 0.0));

  // Assert: the mask is bit-identical...
  const auto mask_after = node_->build_keepout_mask_for_test();
  EXPECT_EQ(mask_after.info.width, mask_before.info.width);
  EXPECT_EQ(mask_after.info.height, mask_before.info.height);
  EXPECT_EQ(mask_after.data, mask_before.data) << "a dig event changed the keepout mask";
  EXPECT_EQ(mask_at(mask_after, 0.0, 0.0), 0) << "the dig point itself must stay plannable";

  // ...the coverage planner sees no new hole, and the GUI sees the proposal.
  auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
  req->index = 0;
  node_->get_mowing_area_for_test(req, res);
  ASSERT_TRUE(res->success);
  EXPECT_TRUE(res->area.obstacles.empty()) << "a proposal must never be a coverage hole";
  EXPECT_TRUE(res->area.obstacle_info.empty());
  ASSERT_EQ(res->area.proposed_obstacles.size(), 1u);
  ASSERT_EQ(res->area.proposed_obstacle_info.size(), 1u);
  EXPECT_TRUE(res->area.proposed_obstacle_info[0].pending);
  EXPECT_EQ(res->area.proposed_obstacle_info[0].source,
            mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
  EXPECT_NE(res->area.proposed_obstacle_info[0].id, 0u);
}

TEST_F(AreaTypeTest, DigOutsideEveryMowingAreaIsNotProposed)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));

  // Digs during transit or docking can happen well outside any mowing area.
  // There is no area to attach a proposal to.
  node_->on_dig_event_for_test(make_dig_event(50.0, 50.0));

  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 0u);
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 0u);
}

TEST_F(AreaTypeTest, DigInsideNavigationAreaOnlyIsNotProposed)
{
  // A navigation corridor is not a mowing area — it cannot own an obstacle,
  // matching the ~/promote_obstacle contract.
  ASSERT_TRUE(add_area("corridor", make_rect(-2, -2, 2, 2), /*is_navigation=*/true));

  node_->on_dig_event_for_test(make_dig_event(0.0, 0.0));

  EXPECT_FALSE(node_->mowing_area_containing_for_test(0.0, 0.0).has_value());
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 0u);
}

TEST_F(AreaTypeTest, RepeatedDigsAtTheSameSpotDoNotStackProposals)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));

  // The robot can re-detect the same patch on a later pass; proposals are
  // deduped by centroid, so the operator's list must not accumulate.
  node_->on_dig_event_for_test(make_dig_event(1.0, 1.0));
  node_->on_dig_event_for_test(make_dig_event(1.0, 1.0));
  node_->on_dig_event_for_test(make_dig_event(1.01, 1.01));

  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u) << "repeated digs stacked proposals";
}

namespace
{
/// Operator accept of the proposal at (area 0, obstacle 0).
void accept_first_proposal(mowgli_map::MapServerNode& node)
{
  auto req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  auto res = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  req->pending_id = node.obstacle_info_for_test(0, 0).id;
  node.promote_obstacle_for_test(req, res);
  ASSERT_TRUE(res->success) << res->message;
}
}  // namespace

// The proposal is sized to the PHYSICAL dig (the two drive-wheel ruts), not to
// the chassis: the old 0.60 m heading-biased box only existed because it was
// stamped as a session keepout, and it blanked out a large patch of lawn on
// every accept. An accepted proposal = compact disc + the mask band, the body
// counted exactly once (by the band).
TEST_F(AreaTypeTest, AcceptedDigProposalIsACompactHoleTheSizeOfTheWheelRuts)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  node_->on_dig_event_for_test(make_dig_event(0.0, 0.0));  // map_distance 0.02
  ASSERT_EQ(node_->area_obstacle_count_for_test(0), 1u);

  accept_first_proposal(*node_);

  EXPECT_FALSE(node_->obstacle_info_for_test(0, 0).pending);
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 1u);
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  // radius 0.19 + 0.01 slip growth = 0.20; default band 0.20 -> lethal to ~0.40.
  EXPECT_EQ(mask_at(mask, 0.0, 0.0), 100) << "the hole itself must be lethal once accepted";
  EXPECT_EQ(mask_at(mask, 0.16, 0.0), 100) << "a wheel rut (half a track away) must be lethal";
  EXPECT_EQ(mask_at(mask, 0.0, 0.16), 100) << "whatever the heading was";
  const std::vector<std::pair<double, double>> outside{{0.65, 0.0},
                                                       {-0.65, 0.0},
                                                       {0.0, 0.65},
                                                       {0.0, -0.65}};
  for (const auto& [x, y] : outside)
  {
    EXPECT_EQ(mask_at(mask, x, y), 0)
        << "the old chassis-sized box reached here (" << x << ", " << y << "); the hole must not";
  }
}

// Accepting while the robot still stands next to the dig would re-create the
// 2026-09-10 / 2026-09-17 strand with one click: polygon + band becomes lethal
// under the robot and every plan from its pose is START_OCCUPIED. The accept
// is REFUSED (not deferred) with a message the GUI shows as-is.
TEST_F(AreaTypeTest, AcceptIsRefusedWhileTheRobotStandsInsideTheResultingKeepout)
{
  // Arrange: the bridge reversed the robot 0.25 m out of the dig.
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  node_->on_dig_event_for_test(make_dig_event(0.0, 0.0));
  node_->set_robot_position_for_test(-0.25, 0.0);
  const auto mask_before = node_->build_keepout_mask_for_test();
  auto req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  req->pending_id = node_->obstacle_info_for_test(0, 0).id;

  // Act
  auto refused = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(req, refused);

  // Assert: nothing applied, the proposal is still there to accept later.
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("robot is standing"), std::string::npos) << refused->message;
  EXPECT_TRUE(node_->obstacle_info_for_test(0, 0).pending);
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 0u);
  EXPECT_EQ(node_->build_keepout_mask_for_test().data, mask_before.data);

  // Once the robot has driven away the same request goes through, and the
  // robot's cell is not lethal afterwards.
  node_->set_robot_position_for_test(-1.0, 0.0);
  auto accepted = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(req, accepted);
  ASSERT_TRUE(accepted->success) << accepted->message;
  EXPECT_EQ(mask_at(node_->build_keepout_mask_for_test(), -1.0, 0.0), 0);
}

// Issue #500: three latches inside 0.13 m are ONE hole. The centroid dedup
// (0.10 m) alone would have listed two.
TEST_F(AreaTypeTest, DigInsideAnExistingProposalIsTheSameHole)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));

  node_->on_dig_event_for_test(make_dig_event(1.00, 1.00));
  node_->on_dig_event_for_test(make_dig_event(1.13, 1.00));

  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u);
}

// A proposal sitting at a spot must not make a real promotion there a no-op:
// the dedup guard of an APPLIED keepout ignores inert proposals.
TEST_F(AreaTypeTest, PromotingAPolygonOverAProposalStillAppliesIt)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  node_->on_dig_event_for_test(make_dig_event(1.0, 1.0));  // disc centred on (1, 1)
  ASSERT_EQ(node_->obstacle_polygon_count_for_test(), 0u);

  EXPECT_TRUE(node_->apply_promoted_obstacle_for_test(0, make_rect(0.7, 0.7, 1.3, 1.3)));

  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 1u);
  const auto mask = node_->build_keepout_mask_for_test();
  EXPECT_EQ(mask_at(mask, 1.0, 1.0), 100);
}

TEST_F(AreaTypeTest, MowingAreaContainingResolvesTheRightArea)
{
  ASSERT_TRUE(add_area("north", make_rect(0, 0, 2, 2), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("south", make_rect(0, -4, 2, -2), /*is_navigation=*/false));

  EXPECT_EQ(node_->mowing_area_containing_for_test(1.0, 1.0), std::optional<size_t>(0));
  EXPECT_EQ(node_->mowing_area_containing_for_test(1.0, -3.0), std::optional<size_t>(1));
  EXPECT_FALSE(node_->mowing_area_containing_for_test(1.0, -1.0).has_value())
      << "the gap between two areas belongs to neither";
}

// ─────────────────────────────────────────────────────────────────────────────
// Keepout mask — lethal-outside-areas boundary policy + index convention.
//
// Worked example mirroring areas.dat's quadrilateral shape: a single mowing
// rectangle from (-3,-2) to (3,2). With the default lethal_outside_areas=true
// and enforce_boundary_margin_m=0.40, the mask must be:
//   * FREE (0) for a point well INSIDE the rectangle,
//   * MID-COST (50, traversable-but-penalised) for a point just OUTSIDE the
//     edge but within 0.40 m — a start/goal there must not fail "Start
//     occupied", yet A* must not corner-cut through the band,
//   * LETHAL (100) for a point far OUTSIDE the rectangle (> 0.40 m past edge).
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

  // Just outside the +X edge but within enforce_boundary_margin_m (0.40) →
  // the mid-cost slack band (50): traversable for a boundary start pose but
  // penalised so transits do not corner-cut outside the polygon.
  EXPECT_EQ(mask_at(mask, 3.10, 0.0), 50) << "RTK-drift slack band must be mid-cost, not lethal";
  // 0.28 m past the edge: LETHAL under the old 0.25 m margin — pins the
  // widened 0.40 m slack (outer-ring "Start occupied" transit-skip fix).
  EXPECT_EQ(mask_at(mask, 3.28, 0.0), 50) << "widened slack band (0.40 m) must stay non-lethal";

  // Far outside the rectangle (> 0.40 m past the edge) → LETHAL.
  EXPECT_EQ(mask_at(mask, 3.55, 0.0), 100) << "cell just past the 0.40 m slack must be lethal";
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
// Transit boundary clearance (boundary_inner_margin_m) + the dock exemption
// (dock_inner_margin_exempt_radius_m). This is deliberately a SOFT mid-cost
// band (kSoftPenaltyMaskCost, = 50 here), never lethal (100) — a lethal
// version was tried during review and rejected on two independent grounds:
//   1. It collides with chassis_safety_inset (also 0.20 m by default): the
//      outermost coverage ring sits exactly that far inside the line, so a
//      lethal band there plus inflation would swallow the ring itself and
//      reopen the START_OCCUPIED skip cascade (issue #487).
//   2. It walls off any area-to-area seam narrower than roughly twice the
//      inflated margin.
// An even earlier lethal attempt (0.15 m) was also reverted on 2026-04-23
// (commit 7f4b43d5) because GNSS drift near a dock close to the recorded
// edge landed the robot's own position in a lethal cell the planner could
// not route out of. These tests pin the soft-cost design: the penalty band
// is real (transit is nudged away from the edge) but nothing it touches —
// an edge cell, a dock-adjacent cell, or a seam between two areas — is ever
// unplannable.
// ─────────────────────────────────────────────────────────────────────────────

class BoundaryInnerMarginTest : public AreaTypeTest
{
protected:
  static constexpr int8_t kSoftPenalty = 50;  // mirrors kSoftPenaltyMaskCost

  // 20x20 m map; dock 0.1 m inside the +X edge of a 16x16 m area (edge at
  // x=8) — a dock placed close to the recorded edge, same shape as the
  // 2026-04-23 regression.
  void SetUp() override
  {
    node_ = make_node(0.20, 2.5);
  }

  static std::shared_ptr<mowgli_map::MapServerNode> make_node(double boundary_inner_margin_m,
                                                              double dock_exempt_radius_m)
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 20.0);
    opts.append_parameter_override("map_size_y", 20.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("areas_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    opts.append_parameter_override("boundary_inner_margin_m", boundary_inner_margin_m);
    opts.append_parameter_override("dock_inner_margin_exempt_radius_m", dock_exempt_radius_m);
    opts.append_parameter_override("dock_pose_x", 7.5);
    opts.append_parameter_override("dock_pose_y", 0.0);
    opts.append_parameter_override("dock_pose_yaw", 0.0);
    return std::make_shared<mowgli_map::MapServerNode>(opts);
  }
};

TEST_F(BoundaryInnerMarginTest, InteriorFarFromEveryEdgeStaysFullyFree)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, 0.0, 0.0), 0)
      << "far from every edge and far from the dock, must be free";
}

TEST_F(BoundaryInnerMarginTest, EdgeFarFromTheDockGetsTheSoftPenaltyNeverLethal)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  // 0.1 m inside the -X edge (x=-8), 15.6 m from the dock at (7.5, 0) — well
  // outside the 2.5 m exemption radius, so the margin applies. It must be
  // the soft mid-cost, NEVER the lethal value (100) — that distinction is
  // the entire point of this rework.
  EXPECT_EQ(mask_at(mask, -7.9, 0.0), kSoftPenalty)
      << "within boundary_inner_margin_m of an edge, far from the dock, must be soft-penalised, "
         "not lethal — a lethal cell here is exactly the regression under review";
}

TEST_F(BoundaryInnerMarginTest, OutermostCoverageRingBandIsNeverLethal)
{
  // The geometry the maintainer flagged: chassis_safety_inset (0.20 m
  // default) places the outermost coverage ring's centreline right inside
  // this same 0.20 m band. A cell just inside that line (-7.85, deliberately
  // off the exact boundary_inner_margin_m cutoff to avoid a floating-point
  // edge case in the test itself) must stay plannable — soft-penalised at
  // worst — or every blade-off transit starting from the outer ring fails
  // "Start occupied" (issue #487).
  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, -7.85, 0.0), kSoftPenalty)
      << "the band the outer ring's own line sits in must never be lethal";
}

TEST_F(BoundaryInnerMarginTest, EdgeNearTheDockCarriesNoPenaltyAtAll)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  // 0.1 m inside the +X edge (x=8) — same distance-to-edge as the previous
  // test — but only 0.4 m from the dock at (7.5, 0): inside the 2.5 m
  // exemption radius, so it must carry NO penalty at all (0), not merely
  // "not lethal".
  EXPECT_EQ(mask_at(mask, 7.9, 0.0), 0)
      << "within the dock exemption radius, the inner-margin penalty must not apply";
}

TEST_F(BoundaryInnerMarginTest, NarrowSeamBetweenTwoAreasStaysCrossable)
{
  // Two 4x8 m areas separated by a 0.3 m navigation gap (x in [-0.15, 0.15])
  // — narrower than 2x boundary_inner_margin_m (0.40 m). A lethal design
  // would wall this off entirely; the soft-cost design must still allow a
  // straight crossing, just at the penalised cost.
  ASSERT_TRUE(add_area("west_lawn", make_rect(-4.15, -4, -0.15, 4), /*is_navigation=*/false));
  ASSERT_TRUE(add_area("east_lawn", make_rect(0.15, -4, 4.15, 4), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());

  // Both sides of the seam are inside their own area, close to its edge, far
  // from the dock — soft-penalised, never lethal, so a straight-line
  // crossing is never blocked.
  EXPECT_EQ(mask_at(mask, -0.2, 0.0), kSoftPenalty) << "just inside the west area's edge";
  EXPECT_EQ(mask_at(mask, 0.2, 0.0), kSoftPenalty) << "just inside the east area's edge";
  // The gap between the two polygons is neither "inside an area" nor within
  // outside_free_margin of one necessarily being tested here — this asserts
  // it is at least never the lethal keepout default either.
  EXPECT_NE(mask_at(mask, 0.0, 0.0), 100) << "the seam between two close areas must stay crossable";
}

TEST_F(BoundaryInnerMarginTest, ZeroExemptRadiusRemovesTheDockCarveOutButStillNeverLethal)
{
  // With the exemption disabled, the near-dock cell falls back to the plain
  // soft penalty — it must NOT become lethal even then, since the mid-cost
  // design no longer depends on the dock exemption for safety.
  node_ = make_node(/*boundary_inner_margin_m=*/0.20, /*dock_exempt_radius_m=*/0.0);

  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, 7.9, 0.0), kSoftPenalty)
      << "with the exemption radius at 0, a near-dock cell falls back to the soft penalty, "
         "never lethal";
}

TEST_F(BoundaryInnerMarginTest, ZeroBoundaryMarginDisablesThePenaltyEntirely)
{
  // The pre-2026-09 default: no penalty at all, anywhere, dock or not.
  node_ = make_node(/*boundary_inner_margin_m=*/0.0, /*dock_exempt_radius_m=*/2.5);

  ASSERT_TRUE(add_area("lawn", make_rect(-8, -8, 8, 8), /*is_navigation=*/false));
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, -7.9, 0.0), 0)
      << "boundary_inner_margin_m=0 must restore the legacy edge-tight behaviour";
}

// ─────────────────────────────────────────────────────────────────────────────
// Drawn-obstacle keepout band (keepout_obstacle_margin, DERIVED at launch by
// robot_config_util.keepout_obstacle_margin). A drawn obstacle (a tree) must
// project a LETHAL band that wide around its polygon. The band is the body
// model of the mask's consumer — SmacPlanner2D is a point check and the mask
// is not inflated downstream — so it is the body half-width, counted ONCE,
// and deliberately NOT coverage_server.obstacle_margin.
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
    opts.append_parameter_override("keepout_obstacle_margin", 0.3);
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
  EXPECT_EQ(mask_at(mask, 0.75, 0.0), 100)
      << "cell inside the keepout_obstacle_margin band must be lethal";
  // Well outside the margin band (edge + 0.3 m + slack) → FREE lawn.
  EXPECT_EQ(mask_at(mask, 1.5, 0.0), 0) << "lawn beyond the margin band must stay free";
}

TEST_F(AreaTypeTest, DrawnObstacleWithDefaultMarginDerivesFromTheChassis)
{
  // Default node: keepout_obstacle_margin is the -1 "derive" sentinel, which
  // falls back to this node's chassis_width / 2 (0.40 / 2 = 0.20 m) — never a
  // free-standing literal.
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
  EXPECT_EQ(mask_at(mask, 0.65, 0.0), 100) << "0.15 m out is inside the chassis/2 = 0.20 m band";
  EXPECT_EQ(mask_at(mask, 0.85, 0.0), 0) << "0.35 m out is beyond the chassis/2 band";
}

// ─────────────────────────────────────────────────────────────────────────────
// "Count the body exactly once per consumer."
//
// The mask's consumer is SmacPlanner2D: a POINT check (centre cell >=
// INSCRIBED) with no footprint test, reading a global costmap whose plugin
// order is [.., inflation_layer, keepout_filter] — the mask is NOT inflated.
// So the lethal region of a drawn obstacle must be polygon + the body
// half-width and nothing more. Before 2026-09-17 it was polygon +
// coverage's obstacle_margin (0.275) + the inflation band (~0.2) on top: a
// robot ON its coverage line 0.275 m from the obstacle read as occupied, and
// detour transits timed out / answered START_OCCUPIED.
// ─────────────────────────────────────────────────────────────────────────────

class KeepoutBodyOnceTest : public AreaTypeTest
{
protected:
  // The shipped chassis: chassis_width 0.45 / 2 + 0.05 costmap margin.
  static constexpr double kBodyHalfWidthM = 0.275;
  static constexpr double kResolutionM = 0.05;

  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", kResolutionM);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("areas_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    opts.append_parameter_override("keepout_obstacle_margin", kBodyHalfWidthM);
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }
};

TEST_F(KeepoutBodyOnceTest, LethalRegionIsPolygonPlusBodyHalfWidthAndNothingMore)
{
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

  // Walk outward from the polygon edge (x = 0.5) along +x and find where the
  // lethal region ends. A cell is lethal when its CENTRE is within the band.
  double last_lethal = 0.0;
  double first_free = 0.0;
  for (double d = 0.0; d < 1.0; d += kResolutionM / 5.0)
  {
    if (mask_at(mask, 0.5 + d, 0.0) == 100)
    {
      last_lethal = d;
    }
    else if (first_free == 0.0)
    {
      first_free = d;
    }
  }
  EXPECT_GT(first_free, 0.0);
  EXPECT_LT(last_lethal, first_free) << "the lethal band must be one contiguous ring";
  // The band ends at the body half-width, to within one mask cell — NOT at
  // half-width + coverage's margin, and NOT at half-width + an inflation band.
  EXPECT_GE(last_lethal, kBodyHalfWidthM - kResolutionM);
  EXPECT_LE(last_lethal, kBodyHalfWidthM + kResolutionM);

  // A robot ON its coverage line: the shipped planning floor puts the
  // centreline 0.389 m from the polygon (robot_config_util). That pose, and
  // anything within the worst-case mask->global-costmap rasterisation slack
  // inside it (0.08 m * sqrt 2 = 0.113 m), must be FREE for Smac's point check.
  constexpr double kCoverageLineM = 0.389;
  constexpr double kRasterSlackM = 0.1132;
  EXPECT_EQ(mask_at(mask, 0.5 + kCoverageLineM, 0.0), 0)
      << "a robot on its coverage line must never be START_OCCUPIED";
  EXPECT_EQ(mask_at(mask, 0.5 + kCoverageLineM - kRasterSlackM + kResolutionM, 0.0), 0);
}

TEST(DigProposalPolygon, IsARegularPolygonCentredOnTheDigThatContainsTheWholeDisc)
{
  const double r = 0.19;
  const auto poly = mowgli_map::dig_proposal_polygon(-3.23, 11.01, r);

  ASSERT_EQ(poly.points.size(), static_cast<std::size_t>(mowgli_map::kDigProposalVertices));
  const auto c = mowgli_map::polygon_centroid(poly);
  EXPECT_NEAR(c.x, -3.23, 1e-4);
  EXPECT_NEAR(c.y, 11.01, 1e-4);
  // Circumscribed: every vertex is outside the disc, every edge touches it.
  for (const auto& p : poly.points)
  {
    EXPECT_NEAR(std::hypot(p.x + 3.23, p.y - 11.01), r / std::cos(M_PI / 8.0), 1e-4);
  }
  EXPECT_NEAR(mowgli_map::point_to_polygon_distance(-3.23, 11.01, poly), r, 1e-4);
  // Not closed (ROS convention) and not degenerate: a GUI that drops the last
  // vertex of an unclosed ring must still be left with a real shape.
  EXPECT_NE(poly.points.front(), poly.points.back());
}

TEST(DigProposalPolygon, RadiusGrowsWithTheSlipCreepAndIsBoundedBothWays)
{
  using mowgli_map::dig_proposal_radius;
  EXPECT_NEAR(dig_proposal_radius(0.189, 0.0), 0.189, 1e-9);
  EXPECT_NEAR(dig_proposal_radius(0.189, 0.04), 0.209, 1e-9) << "ruts are half the creep longer";
  EXPECT_NEAR(dig_proposal_radius(0.189, 5.0), 0.189 + mowgli_map::kMaxDigSlipGrowthM, 1e-9)
      << "a garbage map_distance must not blank out the lawn";
  EXPECT_NEAR(dig_proposal_radius(0.189, -1.0), 0.189, 1e-9);
  EXPECT_NEAR(dig_proposal_radius(0.0, 0.0), mowgli_map::kMinDigProposalRadiusM, 1e-9)
      << "the hole must stay selectable in the GUI and cover at least one map cell";
  EXPECT_NEAR(dig_proposal_radius(std::nan(""), std::nan("")),
              mowgli_map::kMinDigProposalRadiusM,
              1e-9);
}

// Body counted exactly once: with the body-wide band (0.275) an accepted dig
// is lethal out to polygon + band and NOT a chassis length further.
TEST_F(KeepoutBodyOnceTest, AcceptedDigIsTheRutDiscPlusTheBodyBandAndNothingMore)
{
  ASSERT_TRUE(add_area("lawn", make_rect(-3, -3, 3, 3), /*is_navigation=*/false));
  node_->on_dig_event_for_test(make_dig_event(0.0, 0.0));
  ASSERT_EQ(node_->area_obstacle_count_for_test(0), 1u);
  accept_first_proposal(*node_);

  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, 0.025, 0.0), 100) << "the hole itself must be lethal";
  EXPECT_EQ(mask_at(mask, 0.40, 0.0), 100) << "disc 0.20 + band 0.275 reaches here";
  EXPECT_EQ(mask_at(mask, 0.65, 0.0), 0) << "the body must not be counted a second time";
  EXPECT_EQ(mask_at(mask, -0.65, 0.0), 0);
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
#include <iterator>
#include <sstream>

#include <unistd.h>

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

// ─────────────────────────────────────────────────────────────────────────────
// Dig proposals (#502): a detected dig is an INERT proposal — it never edits
// the operator's saved map, the keepout mask or the coverage holes on its own.
// Accepting is what applies AND persists it; discarding drops it for good.
//
// Also covers obstacle identity: name + provenance survive save/load, and an
// areas.dat written before identity existed still loads.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{

std::string temp_areas_path(const std::string& tag)
{
  const char* dir = std::getenv("TEST_TMPDIR");
  return std::string(dir != nullptr ? dir : "/tmp") + "/mowgli_areas_" + tag + ".dat";
}

}  // namespace

class DigProposalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    areas_path_ = temp_areas_path("dig_proposal");
    std::remove(areas_path_.c_str());

    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    // A real robot HAS a persistence path — that is the whole point of the
    // test: even with somewhere to write, a dig must not write.
    opts.append_parameter_override("areas_file_path", areas_path_);
    opts.append_parameter_override("publish_rate", 1.0);
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
    std::remove(areas_path_.c_str());
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

  void add_lawn(const geometry_msgs::msg::Polygon* obstacle = nullptr,
                const std::string& obstacle_name = {})
  {
    auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
    req->area.name = "lawn";
    req->area.area = make_rect(-3, -3, 3, 3);
    if (obstacle != nullptr)
    {
      req->area.obstacles.push_back(*obstacle);
      mowgli_interfaces::msg::MapObstacleInfo info;
      info.name = obstacle_name;
      info.source = mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER;
      req->area.obstacle_info.push_back(info);
    }
    req->is_navigation_area = false;
    auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
    node_->add_area_for_test(req, res);
    ASSERT_TRUE(res->success);
  }

  void dig_at(double x, double y)
  {
    auto msg = std::make_shared<mowgli_interfaces::msg::DigEvent>();
    msg->header.frame_id = "map";
    msg->position.x = x;
    msg->position.y = y;
    msg->wheel_distance = 0.45;
    msg->map_distance = 0.02;
    msg->position_sigma = 0.004;
    node_->on_dig_event_for_test(msg);
  }

  void add_other_areas()
  {
    // Three lawns reproduce the three GUI copies. Include an overlapping
    // navigation area too: ownership must not depend on geometric containment.
    for (uint32_t i = 1; i <= 3; ++i)
    {
      auto req = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
      req->area.name = "other_" + std::to_string(i);
      req->is_navigation_area = i == 3;
      req->area.area = i == 3 ? make_rect(-2, -2, 2, 2) : make_rect(4 * i, -3, 4 * i + 3, 3);
      auto res = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Response>();
      node_->add_area_for_test(req, res);
      ASSERT_TRUE(res->success);
    }
  }

  void expect_other_areas_empty()
  {
    for (uint32_t i = 1; i <= 3; ++i)
    {
      SCOPED_TRACE(i);
      const auto area = fetch_area(i);
      EXPECT_TRUE(area.obstacles.empty()) << "another area's keepout leaked into this response";
      EXPECT_TRUE(area.obstacle_info.empty());
      EXPECT_TRUE(area.proposed_obstacles.empty()) << "another area's proposal leaked here";
      EXPECT_TRUE(area.proposed_obstacle_info.empty());
    }
  }

  mowgli_interfaces::msg::MapArea fetch_area(uint32_t index)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
    req->index = index;
    auto res = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Response>();
    node_->get_mowing_area_for_test(req, res);
    EXPECT_TRUE(res->success);
    return res->area;
  }

  std::string areas_path_;
  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

TEST_F(DigProposalTest, DigNeverReachesTheAreasFile)
{
  add_lawn();
  dig_at(1.0, 1.0);

  // Even an EXPLICIT save must skip the proposal.
  node_->save_areas_for_test(areas_path_);
  const std::string content = read_file(areas_path_);

  EXPECT_NE(content.find("area_0_name: lawn"), std::string::npos) << "the area itself must persist";
  EXPECT_NE(content.find("area_0_obstacle_count: 0"), std::string::npos)
      << "a dig proposal must not be counted in areas.dat:\n"
      << content;
  EXPECT_EQ(content.find("area_0_obstacle_0:"), std::string::npos)
      << "a dig proposal must not be written to areas.dat:\n"
      << content;
}

TEST_F(DigProposalTest, PendingDigIsNeitherACoverageHoleNorLethal)
{
  add_lawn();
  dig_at(1.0, 1.0);

  // Coverage does NOT see the proposal as a hole (the plan, its sub-path
  // count and the resume cursor stay deterministic within the session)...
  const auto area = fetch_area(0);
  EXPECT_TRUE(area.obstacles.empty()) << "a proposal must not change the coverage plan";
  EXPECT_TRUE(area.obstacle_info.empty());
  ASSERT_EQ(area.proposed_obstacles.size(), 1U) << "the operator must be able to review it";
  ASSERT_EQ(area.proposed_obstacle_info.size(), area.proposed_obstacles.size())
      << "proposed_obstacle_info must stay index-aligned with proposed_obstacles";
  EXPECT_TRUE(area.proposed_obstacle_info[0].pending);

  // ...and Nav2 does NOT see it as lethal: the robot stands right next to it
  // and must always be able to plan out (2026-09-17 START_OCCUPIED strand).
  const auto mask = node_->build_keepout_mask_for_test();
  ASSERT_FALSE(mask.data.empty());
  EXPECT_EQ(mask_at(mask, 1.0, 1.0), 0) << "a dig proposal must never be lethal on its own";
}

TEST_F(DigProposalTest, PendingDigIsReturnedOnlyWithItsOwningArea)
{
  add_lawn();
  add_other_areas();
  dig_at(1.0, 1.0);
  const auto info = node_->obstacle_info_for_test(0, 0);

  // Repeated GUI polls must return one polygon with its original identity.
  for (int poll = 0; poll < 2; ++poll)
  {
    const auto owner = fetch_area(0);
    EXPECT_TRUE(owner.obstacles.empty());
    ASSERT_EQ(owner.proposed_obstacles.size(), 1u);
    ASSERT_EQ(owner.proposed_obstacle_info.size(), 1u);
    EXPECT_EQ(owner.proposed_obstacle_info[0].id, info.id);
    EXPECT_EQ(owner.proposed_obstacle_info[0].name, info.name);
    EXPECT_EQ(owner.proposed_obstacle_info[0].source,
              mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
    EXPECT_TRUE(owner.proposed_obstacle_info[0].pending);
    expect_other_areas_empty();
  }
  EXPECT_EQ(mask_at(node_->build_keepout_mask_for_test(), 1.0, 1.0), 0);

  auto req = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Request>();
  req->obstacle_id = info.id;
  auto res = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Response>();
  node_->discard_obstacle_for_test(req, res);
  ASSERT_TRUE(res->success);
  EXPECT_TRUE(fetch_area(0).obstacles.empty());
  EXPECT_TRUE(fetch_area(0).proposed_obstacles.empty());
  expect_other_areas_empty();
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 0u);
}

TEST_F(DigProposalTest, AcceptedDigKeepsItsAreaAndProvenanceBeforeAndAfterReload)
{
  add_lawn();
  add_other_areas();
  dig_at(1.0, 1.0);
  auto req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  req->pending_id = node_->obstacle_info_for_test(0, 0).id;
  req->name = "dig patch";
  auto res = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(req, res);
  ASSERT_TRUE(res->success);

  for (bool reload : {false, true})
  {
    if (reload)
      node_->load_areas_for_test(areas_path_);
    const auto owner = fetch_area(0);
    ASSERT_EQ(owner.obstacles.size(), 1u) << "an accepted proposal IS a coverage hole";
    ASSERT_EQ(owner.obstacle_info.size(), 1u);
    EXPECT_TRUE(owner.proposed_obstacles.empty()) << "accepted: no longer a proposal";
    EXPECT_FALSE(owner.obstacle_info[0].pending);
    EXPECT_EQ(owner.obstacle_info[0].name, "dig patch");
    EXPECT_EQ(owner.obstacle_info[0].source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
    expect_other_areas_empty();
    EXPECT_EQ(mask_at(node_->build_keepout_mask_for_test(), 1.0, 1.0), 100);
  }
}

TEST_F(DigProposalTest, PromotedPolygonIsReturnedOnlyWithItsOwningArea)
{
  add_lawn();
  add_other_areas();
  const auto polygon = make_rect(0.5, 0.5, 1.5, 1.5);
  ASSERT_TRUE(node_->apply_promoted_obstacle_for_test(0, polygon));
  const auto owner = fetch_area(0);
  ASSERT_EQ(owner.obstacles.size(), 1u);
  EXPECT_EQ(owner.obstacles[0], polygon);
  ASSERT_EQ(owner.obstacle_info.size(), 1u);
  EXPECT_EQ(owner.obstacle_info[0].source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER);
  EXPECT_FALSE(owner.obstacle_info[0].pending);
  expect_other_areas_empty();
  EXPECT_EQ(mask_at(node_->build_keepout_mask_for_test(), 1.0, 1.0), 100);
}

TEST_F(DigProposalTest, AcceptingAProposalPersistsItWithItsProvenance)
{
  add_lawn();
  dig_at(1.0, 1.0);
  const auto pending_id = node_->obstacle_info_for_test(0, 0).id;
  ASSERT_NE(pending_id, 0u);

  auto req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  req->pending_id = pending_id;
  req->name = "compost corner";
  auto res = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(req, res);
  ASSERT_TRUE(res->success) << res->message;

  const auto info = node_->obstacle_info_for_test(0, 0);
  EXPECT_FALSE(info.pending);
  EXPECT_EQ(info.name, "compost corner");
  EXPECT_EQ(info.source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG)
      << "accepting must not erase where the keepout came from";

  // on_promote_obstacle persists immediately.
  const std::string content = read_file(areas_path_);
  EXPECT_NE(content.find("area_0_obstacle_count: 1"), std::string::npos) << content;
  EXPECT_NE(content.find("area_0_obstacle_0_name: compost corner"), std::string::npos) << content;
  EXPECT_NE(content.find("area_0_obstacle_0_source: 2"), std::string::npos) << content;

  // …and it survives a reload with its identity intact.
  node_->load_areas_for_test(areas_path_);
  const auto reloaded = node_->obstacle_info_for_test(0, 0);
  EXPECT_EQ(reloaded.name, "compost corner");
  EXPECT_EQ(reloaded.source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_DIG);
  EXPECT_FALSE(reloaded.pending);
}

TEST_F(DigProposalTest, AcceptingAnUnknownPendingIdFails)
{
  add_lawn();

  auto req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  req->pending_id = 4242;
  auto res = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->message.empty());
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 0u);
}

TEST_F(DigProposalTest, DiscardingAProposalRemovesItAndWritesNothing)
{
  add_lawn();
  dig_at(1.0, 1.0);
  const auto pending_id = node_->obstacle_info_for_test(0, 0).id;

  auto req = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Request>();
  req->obstacle_id = pending_id;
  auto res = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Response>();
  node_->discard_obstacle_for_test(req, res);
  ASSERT_TRUE(res->success) << res->message;

  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 0u);
  EXPECT_EQ(node_->obstacle_polygon_count_for_test(), 0u)
      << "a proposal never enters the flat keepout store, discarded or not";

  node_->save_areas_for_test(areas_path_);
  EXPECT_NE(read_file(areas_path_).find("area_0_obstacle_count: 0"), std::string::npos);

  // Discarding twice is an explicit failure, not a silent no-op.
  auto res2 = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Response>();
  node_->discard_obstacle_for_test(req, res2);
  EXPECT_FALSE(res2->success);
}

TEST_F(DigProposalTest, AcceptedKeepoutCannotBeDiscardedAsAProposal)
{
  add_lawn();
  dig_at(1.0, 1.0);
  const auto pending_id = node_->obstacle_info_for_test(0, 0).id;

  auto promote_req = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Request>();
  promote_req->pending_id = pending_id;
  auto promote_res = std::make_shared<mowgli_interfaces::srv::PromoteObstacle::Response>();
  node_->promote_obstacle_for_test(promote_req, promote_res);
  ASSERT_TRUE(promote_res->success);

  auto req = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Request>();
  req->obstacle_id = pending_id;
  auto res = std::make_shared<mowgli_interfaces::srv::ClearObstacle::Response>();
  node_->discard_obstacle_for_test(req, res);

  EXPECT_FALSE(res->success) << "~/discard_obstacle only drops PROPOSALS, not the saved map";
  EXPECT_EQ(node_->area_obstacle_count_for_test(0), 1u);
}

TEST_F(DigProposalTest, UserDrawnObstacleNameRoundTripsThroughSaveAndLoad)
{
  const auto tree = make_rect(-0.5, -0.5, 0.5, 0.5);
  add_lawn(&tree, "apple tree");

  node_->save_areas_for_test(areas_path_);
  node_->load_areas_for_test(areas_path_);

  const auto info = node_->obstacle_info_for_test(0, 0);
  EXPECT_EQ(info.name, "apple tree");
  EXPECT_EQ(info.source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER);
  EXPECT_FALSE(info.pending);

  const auto area = fetch_area(0);
  ASSERT_EQ(area.obstacle_info.size(), area.obstacles.size());
  EXPECT_EQ(area.obstacle_info[0].name, "apple tree");
}

TEST_F(DigProposalTest, LegacyAreasFileWithoutObstacleIdentityStillLoads)
{
  // Exactly the pre-#502 on-disk format — no _name / _source lines. Cedric's
  // robot has a live file in this shape; it must keep loading.
  {
    std::ofstream out(areas_path_);
    out << "# Mowgli ROS2 - Persisted areas and docking point\n\n";
    out << "area_count: 1\n\n";
    out << "area_0_name: lawn\n";
    out << "area_0_polygon: -3,-3;3,-3;3,3;-3,3\n";
    out << "area_0_is_navigation: 0\n";
    out << "area_0_obstacle_count: 1\n";
    out << "area_0_obstacle_0: -0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5\n\n";
  }

  node_->load_areas_for_test(areas_path_);

  ASSERT_EQ(node_->area_obstacle_count_for_test(0), 1u) << "legacy obstacle was dropped on load";
  const auto info = node_->obstacle_info_for_test(0, 0);
  EXPECT_TRUE(info.name.empty());
  EXPECT_EQ(info.source, mowgli_interfaces::msg::MapObstacleInfo::SOURCE_USER)
      << "an obstacle with no recorded provenance is operator-drawn";
  EXPECT_FALSE(info.pending) << "nothing loaded from disk may be pending";

  const auto area = fetch_area(0);
  ASSERT_EQ(area.obstacles.size(), 1U);
  EXPECT_NEAR(area.obstacles[0].points[0].x, -0.5, 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// Dock calibration GPS-position capture (issue #446). An earlier revision
// gated this path on cross-checking the fused yaw against /imu/cog_heading
// and REJECTING on disagreement — but on the dock a fresh COG sample is
// essentially never available (cog_to_imu_node's stationary latch inflates
// its variance well past any usable threshold within seconds of the last
// forward motion), so that gate also rejected the MOTION calibration call
// that is the only non-circular way to fix a stale yaw (maintainer review on
// PR #597). This suite covers the replacement: average the RAW (yaw-
// independent) antenna position, then lever-arm-correct it ONCE with
// whatever yaw THIS call is about to persist — correct regardless of what
// the fused yaw was doing while the antenna samples were captured.
// ─────────────────────────────────────────────────────────────────────────────

class DockCalibrationCaptureTest : public ::testing::Test
{
protected:
  // gtest's own SetUp() constructs the node with NO initial dock yaw. Tests
  // that need a specific one (PRESERVE reusing it, or MOTION's "stale stored
  // yaw" narrative) call construct_node_with_dock_yaw() explicitly instead —
  // do NOT call both for the same test, that would build (and leak into the
  // ROS graph, briefly, under the same node name) two node instances.
  void SetUp() override
  {
    construct_node_with_dock_yaw(0.0);
  }

  // dock_pose_yaw seeds the PERSISTED yaw PRESERVE mode reuses (and, for the
  // MotionMode test, documents the STALE value MOTION mode must ignore) — see
  // map_server_node.cpp's dock_x/dock_y/dock_yaw constructor block, which
  // only initializes docking_pose_ when at least one of the three is nonzero.
  void construct_node_with_dock_yaw(double initial_dock_yaw_rad)
  {
    construct_node_with_dock_pose(0.0, 0.0, initial_dock_yaw_rad);
  }

  // Full stored dock pose (+ optionally a real file to persist into) — what
  // the yaw-only preserve_position write has to leave untouched.
  void construct_node_with_dock_pose(double dock_x,
                                     double dock_y,
                                     double initial_dock_yaw_rad,
                                     const std::string& robot_yaml_path = "")
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("tool_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);
    opts.append_parameter_override("dock_pose_yaw", initial_dock_yaw_rad);
    opts.append_parameter_override("dock_pose_x", dock_x);
    opts.append_parameter_override("dock_pose_y", dock_y);
    if (!robot_yaml_path.empty())
    {
      opts.append_parameter_override("robot_yaml_path", robot_yaml_path);
    }
    node_.reset();  // destroy the SetUp()-constructed node before building the replacement
    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  /// Satisfies gates (1) is_charging, (2) GPS accuracy, and (3) yaw
  /// convergence. Neither value feeds the antenna-averaging/correction path
  /// any more — push_gps_antenna_for_test() and set_gps_lever_arm_for_test()
  /// below do that — so a test only needs this to get past the gates.
  void arm_gates_one_through_three(double converged_yaw_rad = 0.0)
  {
    node_->set_charging_status_for_test(true);
    node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
    node_->push_converged_yaw_for_test(converged_yaw_rad, 20);
  }

  static constexpr size_t kMinAntennaSamples = 10;  // default dock_set_gps_avg_min_samples_

  /// Push `count` (default kMinAntennaSamples) identical raw antenna
  /// samples so the average is exactly (east, north).
  void push_antenna_samples(double east, double north, size_t count = kMinAntennaSamples)
  {
    for (size_t i = 0; i < count; ++i)
    {
      node_->push_gps_antenna_for_test(east, north);
    }
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

TEST_F(DockCalibrationCaptureTest, RejectsWhenTooFewAntennaSamples)
{
  arm_gates_one_through_three();
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  push_antenna_samples(1.0, 2.0, kMinAntennaSamples - 1);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(node_->docking_pose_set_for_test());
}

// dock_set_gps_avg_min_samples/_window_s were hardcoded C++ defaults with no
// declare_parameter call until the #497 field rejection ("only 3 RTK-Fixed
// /gps/fix samples in 3.0 s") showed the pair could be mathematically
// unreachable at a 1 Hz gnss_profile_rate_hz — see the call site in
// map_server_node.cpp. This proves the override actually reaches the gate
// instead of being silently ignored.
TEST_F(DockCalibrationCaptureTest, AvgMinSamplesIsConfigurable)
{
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("resolution", 0.1);
  opts.append_parameter_override("map_size_x", 10.0);
  opts.append_parameter_override("map_size_y", 10.0);
  opts.append_parameter_override("map_frame", "map");
  opts.append_parameter_override("tool_width", 0.2);
  opts.append_parameter_override("map_file_path", "");
  opts.append_parameter_override("publish_rate", 1.0);
  opts.append_parameter_override("dock_set_gps_avg_min_samples", 3);
  node_.reset();
  node_ = std::make_shared<mowgli_map::MapServerNode>(opts);

  arm_gates_one_through_three();
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  push_antenna_samples(1.0, 2.0, /*count=*/3);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_TRUE(res->success);
}

TEST_F(DockCalibrationCaptureTest, RejectsWhenLeverArmNotYetResolved)
{
  arm_gates_one_through_three();
  push_antenna_samples(1.0, 2.0);
  // No set_gps_lever_arm_for_test() call, and no TF broadcaster is running in
  // this unit test, so the real base_footprint→gps_link lookup must fail.

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(node_->docking_pose_set_for_test());
}

TEST_F(DockCalibrationCaptureTest, PreserveModeCorrectsAntennaAverageWithExistingDockYaw)
{
  // Existing (persisted) yaw the PRESERVE path will reuse for the
  // lever-arm correction — docking_pose_ is only initialized from
  // dock_pose_x/y/yaw at construction, so rebuild the node with it set.
  const double existing_yaw = 0.15;
  construct_node_with_dock_yaw(existing_yaw);
  arm_gates_one_through_three(existing_yaw);

  const double lever_x = 0.30;
  const double lever_y = 0.0;
  node_->set_gps_lever_arm_for_test(lever_x, lever_y);

  // Choose a true base position and compute what the antenna would read at
  // that position with the EXISTING yaw — i.e. antenna = base + R(yaw)*lever.
  const double true_base_x = 2.0;
  const double true_base_y = 5.0;
  const double antenna_east =
      true_base_x + std::cos(existing_yaw) * lever_x - std::sin(existing_yaw) * lever_y;
  const double antenna_north =
      true_base_y + std::sin(existing_yaw) * lever_x + std::cos(existing_yaw) * lever_y;
  push_antenna_samples(antenna_east, antenna_north);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  ASSERT_TRUE(res->success);
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, true_base_x, 1e-6);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, true_base_y, 1e-6);
  // PRESERVE must not have touched the yaw.
  EXPECT_NEAR(2.0 * std::atan2(node_->docking_pose_for_test().orientation.z,
                               node_->docking_pose_for_test().orientation.w),
              existing_yaw,
              1e-9);
}

// The maintainer review's explicit ask: a stored yaw well off the true
// heading must not corrupt (or block) a MOTION-mode capture, because the
// fresh req->yaw_rad — not the stale stored value, and not any /imu/cog_heading
// cross-check — is what the antenna average gets corrected with.
TEST_F(DockCalibrationCaptureTest, MotionModeCorrectsPositionWithFreshYawDespiteStaleStoredYaw)
{
  const double stale_stored_yaw = 0.01;  // what a bad prior calibration left behind
  const double fresh_motion_yaw = 0.27;  // ~15° away — the MOTION-measured correction
  construct_node_with_dock_yaw(stale_stored_yaw);
  // Gate (3) only needs a CONVERGED fused yaw, which on the dock reads back
  // as the stale stored value while charging (fusion_graph gauge-resets onto
  // dock_pose) — it is never consulted for the correction itself any more.
  arm_gates_one_through_three(stale_stored_yaw);

  const double lever_x = 0.30;
  const double lever_y = 0.0;
  node_->set_gps_lever_arm_for_test(lever_x, lever_y);

  // The antenna samples were captured before the fresh MOTION yaw was known,
  // but they are RAW (yaw-independent) — they carry no trace of
  // stale_stored_yaw at all. Compute them from the TRUE base position using
  // the FRESH yaw, since that is what the chassis was actually doing.
  const double true_base_x = 2.0;
  const double true_base_y = 5.0;
  const double antenna_east =
      true_base_x + std::cos(fresh_motion_yaw) * lever_x - std::sin(fresh_motion_yaw) * lever_y;
  const double antenna_north =
      true_base_y + std::sin(fresh_motion_yaw) * lever_x + std::cos(fresh_motion_yaw) * lever_y;
  push_antenna_samples(antenna_east, antenna_north);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = fresh_motion_yaw;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  ASSERT_TRUE(res->success);
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, true_base_x, 1e-6);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, true_base_y, 1e-6);
  EXPECT_NEAR(2.0 * std::atan2(node_->docking_pose_for_test().orientation.z,
                               node_->docking_pose_for_test().orientation.w),
              fresh_motion_yaw,
              1e-9);
}

TEST_F(DockCalibrationCaptureTest, ManualPositionSetIsUnaffectedByAntennaAveraging)
{
  // use_gps_position=false is the operator-driven map-drag path — the
  // antenna-averaging/lever-arm correction above is scoped to
  // use_gps_position=true only, so a manual set must succeed with no
  // antenna samples and no lever arm at all.
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(1.0, 1.0, 0.01);
  node_->push_converged_yaw_for_test(0.0, 20);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = false;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::REQUEST;
  req->docking_pose.position.x = 5.0;
  req->docking_pose.position.y = 6.0;
  req->docking_pose.orientation.w = 1.0;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_TRUE(res->success);
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 5.0, 1e-6);
}

// ─────────────────────────────────────────────────────────────────────────────
// Two-step persistence of the one-click dock calibration (2026-09-17).
//
// The calibration measures the dock yaw by REVERSING OFF the dock, so when the
// yaw exists the robot is ~1.5 m from the charger. Step 1 writes that yaw ONLY
// (preserve_position) and must therefore pass WITHOUT is_charging; step 2
// captures X/Y after the re-dock, through every gate. Before this, step 1 was
// a use_gps_position=true capture and gate (1) rejected it every single time.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{
double yaw_of(const geometry_msgs::msg::Pose& pose)
{
  return 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
}
}  // namespace

TEST_F(DockCalibrationCaptureTest, YawOnlyMotionWriteSucceedsOffDockAndKeepsStoredPosition)
{
  // Arrange — a stored dock pose, robot OFF the dock, fused yaw never settled
  // (no convergence samples at all), RTK good.
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  const double motion_yaw = -0.9346;

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->preserve_position = true;
  req->use_gps_position = false;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = motion_yaw;
  // docking_pose is left zero-initialised: it must NOT zero the stored dock.
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();

  // Act
  node_->set_docking_point_for_test(req, res);

  // Assert
  ASSERT_TRUE(res->success) << res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, 3.067, 1e-9);
  EXPECT_NEAR(yaw_of(node_->docking_pose_for_test()), motion_yaw, 1e-9);
}

TEST_F(DockCalibrationCaptureTest, YawOnlyMotionWritePersistsYawAndLeavesXyInTheYamlFile)
{
  const std::string path = "/tmp/mowgli_test_dock_yaw_only_" + std::to_string(::getpid()) + ".yaml";
  {
    std::ofstream out(path);
    out << "mowgli:\n  ros__parameters:\n    dock_pose_x: 6.029\n"
           "    dock_pose_y: 3.067\n    dock_pose_yaw: -0.85\n";
  }
  construct_node_with_dock_pose(6.029, 3.067, -0.85, path);
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->preserve_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = -0.9346;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_TRUE(res->message.empty()) << "file write must have succeeded: " << res->message;
  std::ifstream in(path);
  const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("dock_pose_x: 6.029"), std::string::npos) << content;
  EXPECT_NE(content.find("dock_pose_y: 3.067"), std::string::npos) << content;
  EXPECT_NE(content.find("dock_pose_yaw: -0.9346"), std::string::npos) << content;
  std::remove(path.c_str());
}

TEST_F(DockCalibrationCaptureTest, YawOnlyMotionWriteStillRequiresRtkAccuracy)
{
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.30);  // RTK-Float-grade sigma

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->preserve_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = 1.0;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->message.empty());
  EXPECT_NEAR(yaw_of(node_->docking_pose_for_test()), -0.85, 1e-9);
}

TEST_F(DockCalibrationCaptureTest, YawOnlyMotionWriteIsRejectedWhenNoDockPositionIsStored)
{
  // SetUp() built a node with NO dock pose: preserving (0, 0) would publish a
  // phantom dock (and its lethal body polygon) at the datum.
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->preserve_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = 1.0;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(node_->docking_pose_set_for_test());
}

TEST_F(DockCalibrationCaptureTest, GpsPositionCaptureIsStillRejectedOffDock)
{
  // The regression's exact request (MOTION + use_gps_position off the dock):
  // it must STAY rejected — the charging gate was right, the caller was wrong.
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  node_->push_converged_yaw_for_test(0.0, 20);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  push_antenna_samples(1.0, 2.0);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = 1.0;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("not detected on dock"), std::string::npos) << res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
}

TEST_F(DockCalibrationCaptureTest, PreservePositionCannotBeCombinedWithAPositionCapture)
{
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  arm_gates_one_through_three();

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->preserve_position = true;
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("invalid request"), std::string::npos) << res->message;
}

TEST_F(DockCalibrationCaptureTest, PositionCaptureAfterRedockUsesTheYawWrittenByTheYawOnlyStep)
{
  // The full two-step sequence: step 1 off the dock (yaw only), step 2 on the
  // dock (X/Y, PRESERVE). Step 2's lever-arm correction must use the FRESH
  // yaw from step 1, not the stale one the node booted with.
  const double stale_yaw = 0.0;
  const double fresh_yaw = M_PI / 2.0;
  construct_node_with_dock_pose(9.0, 9.0, stale_yaw);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);

  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  auto yaw_req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  yaw_req->preserve_position = true;
  yaw_req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  yaw_req->yaw_rad = fresh_yaw;
  auto yaw_res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(yaw_req, yaw_res);
  ASSERT_TRUE(yaw_res->success) << yaw_res->message;

  arm_gates_one_through_three(stale_yaw);
  // True base (2, 5), chassis facing +Y: the antenna sits 0.30 m further north.
  push_antenna_samples(2.0, 5.30);
  auto pos_req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  pos_req->use_gps_position = true;
  pos_req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto pos_res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pos_req, pos_res);

  ASSERT_TRUE(pos_res->success) << pos_res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 2.0, 1e-6);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, 5.0, 1e-6);
  EXPECT_NEAR(yaw_of(node_->docking_pose_for_test()), fresh_yaw, 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// Pending on-dock antenna capture (field test 2026-09-17).
//
// Capturing the position AFTER the re-dock was circular: the re-dock steers on
// the STORED pose, so a wrong stored position makes the robot stop short of
// the contacts, never charge, and never reach the capture. The antenna is
// therefore captured at the START — on the dock, charging, RTK-Fixed — held
// in memory, and joined with the motion yaw in ONE off-dock write.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{
std::shared_ptr<std_srvs::srv::Trigger::Response> trigger_response()
{
  return std::make_shared<std_srvs::srv::Trigger::Response>();
}

std::shared_ptr<mowgli_interfaces::srv::SetDockingPoint::Request> pending_antenna_request(
    double motion_yaw)
{
  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_pending_antenna = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::MOTION;
  req->yaw_rad = motion_yaw;
  return req;
}
}  // namespace

TEST_F(DockCalibrationCaptureTest, CaptureDockAntennaIsRejectedOffTheDock)
{
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(1.0, 2.0);

  auto res = trigger_response();
  node_->capture_dock_antenna_for_test(res);

  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("not detected on dock"), std::string::npos) << res->message;
  EXPECT_FALSE(node_->pending_antenna_for_test().valid);
}

TEST_F(DockCalibrationCaptureTest, CaptureDockAntennaIsRejectedUntilTheWindowHasEnoughSamples)
{
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(1.0, 2.0, kMinAntennaSamples - 1);

  auto res = trigger_response();
  node_->capture_dock_antenna_for_test(res);

  EXPECT_FALSE(res->success);
  EXPECT_FALSE(node_->pending_antenna_for_test().valid);
}

TEST_F(DockCalibrationCaptureTest, CaptureDockAntennaNeedsNoConvergedYawAndPersistsNothing)
{
  // No push_converged_yaw_for_test(): no yaw is involved in the capture.
  construct_node_with_dock_pose(9.0, 9.0, 0.3);
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(4.0, 7.0);

  auto res = trigger_response();
  node_->capture_dock_antenna_for_test(res);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_TRUE(node_->pending_antenna_for_test().valid);
  EXPECT_NEAR(node_->pending_antenna_for_test().antenna.east, 4.0, 1e-9);
  EXPECT_NEAR(node_->pending_antenna_for_test().antenna.north, 7.0, 1e-9);
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 9.0, 1e-9);  // stored pose untouched
}

TEST_F(DockCalibrationCaptureTest, PendingAntennaAndMotionYawWriteXyAndYawTogetherOffTheDock)
{
  // The 2026-09-17 field geometry: stored position 0.35 m short, stale yaw.
  const double fresh_yaw = -54.0 * M_PI / 180.0;
  const double true_base_x = 6.263;
  const double true_base_y = 2.811;
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);

  // On the dock: capture the raw antenna (0.30 m ahead of the base).
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(true_base_x + 0.30 * std::cos(fresh_yaw),
                       true_base_y + 0.30 * std::sin(fresh_yaw));
  auto cap = trigger_response();
  node_->capture_dock_antenna_for_test(cap);
  ASSERT_TRUE(cap->success) << cap->message;

  // Off the dock, fused yaw unsettled: ONE write with the motion yaw.
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(fresh_yaw), res);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, true_base_x, 1e-6);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, true_base_y, 1e-6);
  EXPECT_NEAR(yaw_of(node_->docking_pose_for_test()), fresh_yaw, 1e-9);
  EXPECT_NEAR(res->stored_pose.position.x, true_base_x, 1e-6);
  EXPECT_FALSE(node_->pending_antenna_for_test().valid) << "a capture is single-use";
}

TEST_F(DockCalibrationCaptureTest, PendingAntennaWriteWorksOnAFreshInstallWithNoStoredPose)
{
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(2.30, 5.0);
  auto cap = trigger_response();
  node_->capture_dock_antenna_for_test(cap);
  ASSERT_TRUE(cap->success) << cap->message;
  node_->set_charging_status_for_test(false);

  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(0.0), res);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_TRUE(node_->docking_pose_set_for_test());
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 2.0, 1e-6);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, 5.0, 1e-6);
}

TEST_F(DockCalibrationCaptureTest, PendingAntennaWritePersistsXyAndYawToTheYamlFile)
{
  const std::string path = "/tmp/mowgli_test_dock_pending_" + std::to_string(::getpid()) + ".yaml";
  {
    std::ofstream out(path);
    out << "mowgli:\n  ros__parameters:\n    dock_pose_x: 6.029\n"
           "    dock_pose_y: 3.067\n    dock_pose_yaw: -0.85\n";
  }
  construct_node_with_dock_pose(6.029, 3.067, -0.85, path);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(2.30, 5.0);
  auto cap = trigger_response();
  node_->capture_dock_antenna_for_test(cap);
  ASSERT_TRUE(cap->success) << cap->message;
  node_->set_charging_status_for_test(false);

  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(0.0), res);

  ASSERT_TRUE(res->success) << res->message;
  EXPECT_TRUE(res->message.empty()) << res->message;
  std::ifstream in(path);
  const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("dock_pose_x: 2.000000"), std::string::npos) << content;
  EXPECT_NE(content.find("dock_pose_y: 5.000000"), std::string::npos) << content;
  EXPECT_NE(content.find("dock_pose_yaw: 0.000000"), std::string::npos) << content;
  std::remove(path.c_str());
}

TEST_F(DockCalibrationCaptureTest, PendingAntennaWriteIsRejectedWhenNothingWasCaptured)
{
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);

  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(1.0), res);

  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("no dock antenna capture is pending"), std::string::npos)
      << res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
  EXPECT_NEAR(yaw_of(node_->docking_pose_for_test()), -0.85, 1e-9);
  EXPECT_NEAR(res->stored_pose.position.x, 6.029, 1e-9);
}

TEST_F(DockCalibrationCaptureTest, ExpiredPendingAntennaIsRejectedAndDropped)
{
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(2.30, 5.0);
  auto cap = trigger_response();
  node_->capture_dock_antenna_for_test(cap);
  ASSERT_TRUE(cap->success) << cap->message;
  node_->set_charging_status_for_test(false);
  node_->age_pending_antenna_for_test(301.0);  // default TTL 300 s

  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(0.0), res);

  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("old"), std::string::npos) << res->message;
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
  EXPECT_FALSE(node_->pending_antenna_for_test().valid);
}

TEST_F(DockCalibrationCaptureTest, PendingAntennaWriteStillRequiresRtkAccuracyAndKeepsTheCapture)
{
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  push_antenna_samples(2.30, 5.0);
  auto cap = trigger_response();
  node_->capture_dock_antenna_for_test(cap);
  ASSERT_TRUE(cap->success) << cap->message;
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.30);  // momentary RTK-Float

  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(pending_antenna_request(0.0), res);

  EXPECT_FALSE(res->success);
  EXPECT_TRUE(node_->pending_antenna_for_test().valid) << "a retry must still find it";
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
}

TEST_F(DockCalibrationCaptureTest, AntennaSamplesAreOnlyKeptWhileCharging)
{
  // Approach: RTK-Fixed samples arrive while the robot drives in — not kept.
  node_->set_charging_status_for_test(false);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  for (size_t i = 0; i < kMinAntennaSamples; ++i)
  {
    node_->on_fixed_antenna_sample_for_test(1.0 + 0.1 * static_cast<double>(i), 2.0);
  }
  node_->set_charging_status_for_test(true);

  auto res = trigger_response();
  node_->capture_dock_antenna_for_test(res);

  EXPECT_FALSE(res->success) << "approach samples must not count as on-dock samples";
}

TEST_F(DockCalibrationCaptureTest, LeavingTheDockDropsTheOnDockAntennaSamples)
{
  node_->set_charging_status_for_test(true);
  node_->push_gps_pose_cov_for_test(0.0, 0.0, 0.01);
  for (size_t i = 0; i < kMinAntennaSamples; ++i)
  {
    node_->on_fixed_antenna_sample_for_test(1.0, 2.0);
  }
  node_->set_charging_status_for_test(false);  // undocked (or contact lost)
  node_->set_charging_status_for_test(true);  // re-docked, possibly somewhere else

  auto res = trigger_response();
  node_->capture_dock_antenna_for_test(res);

  EXPECT_FALSE(res->success);
}

TEST_F(DockCalibrationCaptureTest, RejectedGpsCaptureLeavesTheStoredPoseUntouchedInMemory)
{
  // The request's zero-initialised docking_pose used to be copied into
  // docking_pose_ BEFORE the antenna-sample check could reject — leaving the
  // in-memory dock at (0, 0) for the next yaw-only write to "preserve".
  construct_node_with_dock_pose(6.029, 3.067, -0.85);
  arm_gates_one_through_three();
  node_->set_gps_lever_arm_for_test(0.30, 0.0);
  push_antenna_samples(1.0, 2.0, kMinAntennaSamples - 1);

  auto req = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Request>();
  req->use_gps_position = true;
  req->yaw_source = mowgli_interfaces::srv::SetDockingPoint::Request::PRESERVE;
  auto res = std::make_shared<mowgli_interfaces::srv::SetDockingPoint::Response>();
  node_->set_docking_point_for_test(req, res);

  ASSERT_FALSE(res->success);
  EXPECT_NEAR(node_->docking_pose_for_test().position.x, 6.029, 1e-9);
  EXPECT_NEAR(node_->docking_pose_for_test().position.y, 3.067, 1e-9);
}
