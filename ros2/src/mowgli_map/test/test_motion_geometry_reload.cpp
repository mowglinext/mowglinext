// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

namespace
{
using Geometry = visualization_msgs::msg::MarkerArray;

void ExpectBothDeclaredHolesProtected(const Geometry& geometry,
                                      mowgli_interfaces::motion::Pose first,
                                      mowgli_interfaces::motion::Pose second)
{
  const auto snapshot = mowgli_interfaces::motion::Snapshot::parse(geometry);
  ASSERT_TRUE(snapshot->valid);
  ASSERT_GE(snapshot->holes.size(), 2u);
  const mowgli_interfaces::motion::Ring tiny_body{{-0.005, -0.005},
                                                  {0.005, -0.005},
                                                  {0.005, 0.005},
                                                  {-0.005, 0.005}};
  EXPECT_FALSE(snapshot->permits(first, tiny_body, 0, 0, 0, false));
  EXPECT_FALSE(snapshot->permits(second, tiny_body, 0, 0, 0, false));
  EXPECT_TRUE(snapshot->permits({2, 2, 0}, tiny_body, 0, 0, 0, false));
}

class MotionGeometryReload : public ::testing::Test
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
    path_ = (std::filesystem::temp_directory_path() /
             ("mowgli_motion_reload_" + std::to_string(::getpid()) + ".dat"))
                .string();
    Write(ValidArea());
    rclcpp::NodeOptions options;
    options.append_parameter_override("resolution", 0.20);
    options.append_parameter_override("areas_file_path", path_);
    options.append_parameter_override("map_file_path", std::string{});
    options.append_parameter_override("robot_yaml_path", path_ + ".robot.yaml");
    node_ = std::make_shared<mowgli_map::MapServerNode>(options);
    observer_ = rclcpp::Node::make_shared("motion_reload_observer");
    reload_ = observer_->create_client<std_srvs::srv::Trigger>("/map_server_node/load_areas");
    subscription_ =
        observer_->create_subscription<Geometry>("/map_server_node/transit_geometry",
                                                 rclcpp::QoS(1).transient_local().reliable(),
                                                 [this](Geometry::ConstSharedPtr geometry)
                                                 {
                                                   received_.push_back(*geometry);
                                                 });
    executor_.add_node(observer_);
    executor_.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (subscription_->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
    {
      Pump();
    }
    ASSERT_GT(subscription_->get_publisher_count(), 0u);
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    executor_.remove_node(observer_);
    subscription_.reset();
    reload_.reset();
    observer_.reset();
    node_.reset();
    std::filesystem::remove(path_);
    std::filesystem::remove(path_ + ".tmp");
    std::filesystem::remove(path_ + ".robot.yaml");
  }

  void Pump()
  {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  void ObserveNewPublication(std::size_t before)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received_.size() == before && std::chrono::steady_clock::now() < deadline)
    {
      Pump();
    }
    ASSERT_GT(received_.size(), before);
  }

  void Write(const std::string& contents)
  {
    std::ofstream output(path_, std::ios::trunc);
    ASSERT_TRUE(output.is_open());
    output << contents;
    output.close();
    ASSERT_TRUE(output.good());
  }

  std::string ValidArea(const std::string& count = "1")
  {
    return "area_count: " + count +
           "\narea_0_name: lawn\narea_0_id: 1\narea_0_polygon: -3,-3;3,-3;3,3;-3,3\n"
           "area_0_obstacle_count: 0\nnext_area_id: 2\n";
  }

  void StartWithPublishedValidGeometry()
  {
    Write(ValidArea());
    ASSERT_NO_THROW(node_->load_areas_for_test(path_));
    const auto before = received_.size();
    node_->build_keepout_mask_for_test();
    ObserveNewPublication(before);
    ASSERT_FALSE(received_.back().markers.empty());
    ASSERT_EQ(received_.back().markers.front().ns, "area");
    ASSERT_GE(received_.back().markers.front().points.size(), 3u);
  }

  void ExpectRevokedAfterFailedReload(const std::string& malformed)
  {
    StartWithPublishedValidGeometry();
    ASSERT_FALSE(received_.empty());
    ASSERT_FALSE(received_.back().markers.empty());
    Write(malformed);
    const auto before = received_.size();
    ASSERT_TRUE(reload_->wait_for_service(std::chrono::seconds(3)));
    auto result = reload_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor_.spin_until_future_complete(result, std::chrono::seconds(3)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(result.get()->success);
    ObserveNewPublication(before);
    ASSERT_TRUE(received_.back().markers.empty()) << "Failed reload retained old authorization";
    // A mask rebuild must not grant the partially parsed replacement either.
    const auto after_failure = received_.size();
    node_->build_keepout_mask_for_test();
    ObserveNewPublication(after_failure);
    ASSERT_TRUE(received_.back().markers.empty()) << "Partial map was reauthorized by mask rebuild";
  }

  std::string path_;
  std::shared_ptr<mowgli_map::MapServerNode> node_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<Geometry>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<Geometry> received_;
};

TEST_F(MotionGeometryReload, StartupPublishesPersistedPermissionWithoutOccupancyMap)
{
  // Exercise the real wall timer, without the test-only mask rebuild or /map.
  // Persisted loading first revokes permission; require the subsequent valid
  // publication rather than mistaking that initial empty message for readiness.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((received_.empty() || received_.back().markers.empty()) &&
         std::chrono::steady_clock::now() < deadline)
  {
    Pump();
  }
  ASSERT_FALSE(received_.empty());
  const auto snapshot = mowgli_interfaces::motion::Snapshot::parse(received_.back());
  ASSERT_TRUE(snapshot->valid);
  const mowgli_interfaces::motion::Ring body{{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  EXPECT_TRUE(snapshot->permits({0, 0, 0}, body, 0.1, 0, 0.1, false));
}

TEST_F(MotionGeometryReload, ParameterLawnPublishesWithPausedSimulationClockAndNoOccupancyMap)
{
  executor_.remove_node(node_);
  node_.reset();
  subscription_.reset();
  received_.clear();
  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.20);
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("areas_file_path", std::string{});
  options.append_parameter_override("map_file_path", std::string{});
  options.append_parameter_override("robot_yaml_path", path_ + ".robot.yaml");
  options.append_parameter_override("area_names", std::vector<std::string>{"main_mow"});
  options.append_parameter_override("area_polygons",
                                    std::vector<std::string>{"-4.5,-3;4.5,-3;4.5,3;-4.5,3"});
  node_ = std::make_shared<mowgli_map::MapServerNode>(options);
  subscription_ =
      observer_->create_subscription<Geometry>("/map_server_node/transit_geometry",
                                               rclcpp::QoS(1).transient_local().reliable(),
                                               [this](Geometry::ConstSharedPtr geometry)
                                               {
                                                 received_.push_back(*geometry);
                                               });
  executor_.add_node(node_);
  ObserveNewPublication(0);
  const auto snapshot = mowgli_interfaces::motion::Snapshot::parse(received_.back());
  ASSERT_TRUE(snapshot->valid);
  const mowgli_interfaces::motion::Ring body{{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  EXPECT_TRUE(snapshot->permits({0, 0, 0}, body, 0.1, 0, 0.1, false));
  EXPECT_FALSE(snapshot->permits({5, 0, 0}, body, 0.1, 0, 0.1, false));
}

TEST_F(MotionGeometryReload, MalformedAreaCountRevokesPreviouslyPublishedGeometry)
{
  ExpectRevokedAfterFailedReload("area_count: not_an_integer\n");
}

TEST_F(MotionGeometryReload, FailureAfterOneParsedAreaCannotPublishPartialPermission)
{
  ExpectRevokedAfterFailedReload(ValidArea("2") +
                                 "area_1_polygon: 4,-3;6,-3;6,3;4,3\n"
                                 "area_1_obstacle_count: invalid\n");
}

TEST_F(MotionGeometryReload, CorrectedReloadRestoresNormalGeometryPublication)
{
  ExpectRevokedAfterFailedReload("area_count: not_an_integer\n");
  Write(ValidArea());
  ASSERT_NO_THROW(node_->load_areas_for_test(path_));
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_FALSE(received_.back().markers.empty());
  EXPECT_EQ(received_.back().markers.front().ns, "area");
}

TEST_F(MotionGeometryReload, MissingFilePreservesUnchangedLiveGeometry)
{
  StartWithPublishedValidGeometry();
  const auto missing = path_ + ".missing";
  std::filesystem::remove(missing);
  EXPECT_THROW(node_->load_areas_for_test(missing), std::exception);
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_FALSE(received_.back().markers.empty());
  EXPECT_EQ(received_.back().markers.front().ns, "area");
}

TEST_F(MotionGeometryReload, ShortDeclaredObstacleCannotDisappearOnReload)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0.60,0.20;0.61,0.21\n");
}

TEST_F(MotionGeometryReload, MissingCoordinatePairCannotBeSkippedFromDeclaredHole)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0,0;1,0;1;1,1;0,1\n");
}

TEST_F(MotionGeometryReload, MissingCoordinatePairCannotEnlargeAreaPermission)
{
  ExpectRevokedAfterFailedReload(ValidArea() + "area_0_polygon: -3,-3;3,-3;3;3,3;-3,3\n");
}

TEST_F(MotionGeometryReload, NegativeObstacleCountCannotRemoveRecordedHoles)
{
  ExpectRevokedAfterFailedReload(ValidArea() + "area_0_obstacle_count: -1\n");
}

TEST_F(MotionGeometryReload, NegativeAreaCountRejectsReplacement)
{
  ExpectRevokedAfterFailedReload("area_count: -1\n");
}

TEST_F(MotionGeometryReload, MissingObstacleCountCannotHideDeclaredHole)
{
  ExpectRevokedAfterFailedReload(
      "area_count: 1\narea_0_polygon: -3,-3;3,-3;3,3;-3,3\n"
      "area_0_obstacle_0: -1,-1;1,-1;1,1;-1,1\n");
}

TEST_F(MotionGeometryReload, ZeroObstacleCountCannotHideDeclaredHole)
{
  ExpectRevokedAfterFailedReload(ValidArea() + "area_0_obstacle_0: -1,-1;1,-1;1,1;-1,1\n");
}

TEST_F(MotionGeometryReload, WhitespaceAroundIndexedKeyCannotHideDeclaredHole)
{
  ExpectRevokedAfterFailedReload(ValidArea() + "  area_0_obstacle_0 \t: -1,-1;1,-1;1,1;-1,1\n");
}

TEST_F(MotionGeometryReload, UndersizedObstacleCountCannotHideLaterHole)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: -2,-2;-1,-2;-1,-1;-2,-1\n"
                                 "area_0_obstacle_1: 0,0;1,0;1,1;0,1\n");
}

TEST_F(MotionGeometryReload, MissingAreaCountCannotHideDeclaredArea)
{
  ExpectRevokedAfterFailedReload("area_0_polygon: -3,-3;3,-3;3,3;-3,3\n");
}

TEST_F(MotionGeometryReload, UndersizedAreaCountCannotHideLaterArea)
{
  ExpectRevokedAfterFailedReload(ValidArea() + "area_1_polygon: 4,-3;6,-3;6,3;4,3\n");
}

TEST_F(MotionGeometryReload, NoncanonicalHoleIndexCannotHideDeclaredHole)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: -2,-2;-1,-2;-1,-1;-2,-1\n"
                                 "area_0_obstacle_00: 0,0;1,0;1,1;0,1\n");
}

TEST_F(MotionGeometryReload, LegacyMapWithoutOptionalHoleCountRemainsUsable)
{
  StartWithPublishedValidGeometry();
  Write("area_count: 1\narea_0_polygon: -3,-3;3,-3;3,3;-3,3\n");
  ASSERT_NO_THROW(node_->load_areas_for_test(path_));
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_EQ(received_.back().markers.size(), 1u);
  const auto snapshot = mowgli_interfaces::motion::Snapshot::parse(received_.back());
  ASSERT_TRUE(snapshot->valid);
  const mowgli_interfaces::motion::Ring body{{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  EXPECT_TRUE(snapshot->permits({0, 0, 0}, body, 0.1, 0, 1, true));
}

TEST_F(MotionGeometryReload, CoordinateNumberTailCannotBeSilentlyAccepted)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0,0;1oops,0;1,1;0,1\n");
}

TEST_F(MotionGeometryReload, NonfiniteDeclaredHoleRevokesPermission)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0,0;nan,0;1,1;0,1\n");
}

TEST_F(MotionGeometryReload, SelfIntersectingDeclaredHoleRevokesPermission)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0,0;1,1;0,1;1,0\n");
}

TEST_F(MotionGeometryReload, DegenerateDeclaredHoleRevokesPermission)
{
  ExpectRevokedAfterFailedReload(ValidArea() +
                                 "area_0_obstacle_count: 1\n"
                                 "area_0_obstacle_0: 0,0;1,0;2,0\n");
}

TEST_F(MotionGeometryReload, ValidClosedAreaAndObstacleRingsRemainPublished)
{
  StartWithPublishedValidGeometry();
  Write(ValidArea() +
        "area_0_polygon: -3,-3;3,-3;3,3;-3,3;-3,-3\n"
        "area_0_obstacle_count: 1\n"
        "area_0_obstacle_0: 0,0;0,1;1,1;1,0;0,0\n");
  ASSERT_NO_THROW(node_->load_areas_for_test(path_));
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_EQ(received_.back().markers.size(), 2u);
  EXPECT_EQ(received_.back().markers[0].ns, "area");
  EXPECT_EQ(received_.back().markers[1].ns, "obstacle");
}

TEST_F(MotionGeometryReload, AddAreaWithShortDeclaredObstacleRejectsWithoutChangingLiveMap)
{
  StartWithPublishedValidGeometry();
  const auto old_geometry = received_.back();
  const auto generation = node_->area_list_generation_for_test();
  auto add =
      observer_->create_client<mowgli_interfaces::srv::AddMowingArea>("/map_server_node/add_area");
  ASSERT_TRUE(add->wait_for_service(std::chrono::seconds(3)));
  auto request = std::make_shared<mowgli_interfaces::srv::AddMowingArea::Request>();
  request->area.name = "invalid replacement";
  for (const auto& xy : std::vector<std::pair<float, float>>{{4, -3}, {6, -3}, {6, 3}, {4, 3}})
  {
    geometry_msgs::msg::Point32 point;
    point.x = xy.first;
    point.y = xy.second;
    request->area.area.points.push_back(point);
  }
  geometry_msgs::msg::Polygon short_hole;
  short_hole.points.push_back(request->area.area.points[0]);
  short_hole.points.push_back(request->area.area.points[1]);
  request->area.obstacles.push_back(short_hole);
  auto result = add->async_send_request(request);
  ASSERT_EQ(executor_.spin_until_future_complete(result, std::chrono::seconds(3)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(result.get()->success);
  EXPECT_EQ(node_->area_list_generation_for_test(), generation);
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_EQ(received_.back().markers.size(), old_geometry.markers.size());
  EXPECT_EQ(received_.back().markers.front().points, old_geometry.markers.front().points);
}

TEST_F(MotionGeometryReload, DistinctSameCentroidDeclaredHolesBothRemainForbiddenAfterReload)
{
  StartWithPublishedValidGeometry();
  Write(ValidArea() +
        "area_0_obstacle_count: 2\n"
        "area_0_obstacle_0: -1,-0.1;1,-0.1;1,0.1;-1,0.1\n"
        "area_0_obstacle_1: -0.1,-1;0.1,-1;0.1,1;-0.1,1\n");
  ASSERT_NO_THROW(node_->load_areas_for_test(path_));
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_EQ(received_.back().markers.size(), 3u);
  ExpectBothDeclaredHolesProtected(received_.back(), {0.8, 0, 0}, {0, 0.8, 0});
}

TEST_F(MotionGeometryReload, DistinctNearbyCentroidDeclaredHolesBothRemainForbiddenAfterReload)
{
  StartWithPublishedValidGeometry();
  Write(ValidArea() +
        "area_0_obstacle_count: 2\n"
        "area_0_obstacle_0: 0.20,0.20;0.25,0.20;0.25,0.25;0.20,0.25\n"
        "area_0_obstacle_1: 0.27,0.20;0.32,0.20;0.32,0.25;0.27,0.25\n");
  ASSERT_NO_THROW(node_->load_areas_for_test(path_));
  const auto before = received_.size();
  node_->build_keepout_mask_for_test();
  ObserveNewPublication(before);
  ASSERT_EQ(received_.back().markers.size(), 3u);
  ExpectBothDeclaredHolesProtected(received_.back(), {0.225, 0.225, 0}, {0.295, 0.225, 0});
}

TEST_F(MotionGeometryReload, ParameterMapPreservesDistinctSameCentroidDeclaredHoles)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=parameter_geometry_map"});
  options.append_parameter_override("resolution", 0.20);
  options.append_parameter_override("areas_file_path", std::string{});
  options.append_parameter_override("map_file_path", std::string{});
  options.append_parameter_override("robot_yaml_path", path_ + ".robot.yaml");
  options.append_parameter_override("area_names", std::vector<std::string>{"parameter lawn"});
  options.append_parameter_override("area_polygons",
                                    std::vector<std::string>{"-3,-3;3,-3;3,3;-3,3"});
  options.append_parameter_override("area_obstacles",
                                    std::vector<std::string>{
                                        "-1,-0.1;1,-0.1;1,0.1;-1,0.1|-0.1,-1;0.1,-1;0.1,1;-0.1,1"});
  auto parameter_node = std::make_shared<mowgli_map::MapServerNode>(options);
  std::vector<Geometry> published;
  auto subscription =
      observer_->create_subscription<Geometry>("/parameter_geometry_map/transit_geometry",
                                               rclcpp::QoS(1).transient_local().reliable(),
                                               [&published](Geometry::ConstSharedPtr geometry)
                                               {
                                                 published.push_back(*geometry);
                                               });
  executor_.add_node(parameter_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
  {
    Pump();
  }
  ASSERT_GT(subscription->get_publisher_count(), 0u);
  parameter_node->build_keepout_mask_for_test();
  const auto publication_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((published.empty() || published.back().markers.empty()) &&
         std::chrono::steady_clock::now() < publication_deadline)
  {
    Pump();
  }
  ASSERT_FALSE(published.empty());
  ExpectBothDeclaredHolesProtected(published.back(), {0.8, 0, 0}, {0, 0.8, 0});
  executor_.remove_node(parameter_node);
}

TEST_F(MotionGeometryReload, OrphanParameterObstacleEntriesRejectStartup)
{
  for (const bool lawn_present : {false, true})
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("areas_file_path", std::string{});
    options.append_parameter_override("map_file_path", std::string{});
    options.append_parameter_override("robot_yaml_path", path_ + ".robot.yaml");
    if (lawn_present)
    {
      options.append_parameter_override("area_names", std::vector<std::string>{"lawn"});
      options.append_parameter_override("area_polygons",
                                        std::vector<std::string>{"-3,-3;3,-3;3,3;-3,3"});
    }
    options.append_parameter_override("area_obstacles",
                                      lawn_present
                                          ? std::vector<std::string>{"", "-1,-1;1,-1;1,1;-1,1"}
                                          : std::vector<std::string>{"-1,-1;1,-1;1,1;-1,1"});
    EXPECT_THROW(std::make_shared<mowgli_map::MapServerNode>(options), std::invalid_argument);
  }
}

TEST_F(MotionGeometryReload, OmittedTrailingParameterObstacleEntriesPreserveLegalMotion)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=trailing_parameter_geometry_map"});
  options.append_parameter_override("resolution", 0.20);
  options.append_parameter_override("areas_file_path", std::string{});
  options.append_parameter_override("map_file_path", std::string{});
  options.append_parameter_override("robot_yaml_path", path_ + ".robot.yaml");
  options.append_parameter_override("area_names", std::vector<std::string>{"first", "second"});
  options.append_parameter_override("area_polygons",
                                    std::vector<std::string>{"-3,-3;3,-3;3,3;-3,3",
                                                             "3,-3;6,-3;6,3;3,3"});
  options.append_parameter_override("area_obstacles",
                                    std::vector<std::string>{"-1,-1;1,-1;1,1;-1,1"});
  auto parameter_node = std::make_shared<mowgli_map::MapServerNode>(options);
  std::vector<Geometry> published;
  auto subscription =
      observer_->create_subscription<Geometry>("/trailing_parameter_geometry_map/transit_geometry",
                                               rclcpp::QoS(1).transient_local().reliable(),
                                               [&published](Geometry::ConstSharedPtr geometry)
                                               {
                                                 published.push_back(*geometry);
                                               });
  executor_.add_node(parameter_node);
  parameter_node->build_keepout_mask_for_test();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((published.empty() || published.back().markers.empty()) &&
         std::chrono::steady_clock::now() < deadline)
    Pump();
  ASSERT_FALSE(published.empty());
  const auto snapshot = mowgli_interfaces::motion::Snapshot::parse(published.back());
  ASSERT_TRUE(snapshot->valid);
  // Parameter holes are also retained in the existing global obstacle list.
  ASSERT_EQ(snapshot->holes.size(), 2u);
  const mowgli_interfaces::motion::Ring body{{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}};
  EXPECT_FALSE(snapshot->permits({0, 0, 0}, body, 0.1, 0, 1, true));
  EXPECT_TRUE(snapshot->permits({4, 0, 0}, body, 0.1, 0, 1, true));
  executor_.remove_node(parameter_node);
}
}  // namespace
