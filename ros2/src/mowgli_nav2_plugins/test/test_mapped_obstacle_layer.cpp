// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <memory>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mowgli_nav2_plugins/mapped_obstacle_layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include <gtest/gtest.h>

namespace
{
using Snapshot = visualization_msgs::msg::MarkerArray;
using Ring = mowgli_nav2_plugins::transit::Ring;
Ring box(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}
void append(Snapshot& snapshot, const std::string& kind, const Ring& ring)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = kind;
  if (kind == "area")
    marker.text = "mowing";
  marker.id = snapshot.markers.size();
  marker.action = marker.ADD;
  marker.type = marker.LINE_STRIP;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.275;  // Planner-only margin must not enter the local map.
  for (const auto& p : ring)
  {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    marker.points.push_back(point);
  }
  snapshot.markers.push_back(marker);
}
Snapshot geometry(bool hole = true)
{
  Snapshot snapshot;
  append(snapshot, "area", box(-1, -1, 1, 1));
  if (hole)
    append(snapshot, "obstacle", box(.8, .8, 1.2, 1.2));
  append(snapshot, "dock_body", box(-1.2, -.3, -.4, .3));
  append(snapshot, "dock_corridor", box(-2, -.3, -1.2, .3));
  return snapshot;
}

class MappedObstacleTest : public testing::Test
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
    auto options = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("plugins", std::vector<std::string>{"sensor", "inflation", "mapped"}),
         rclcpp::Parameter("sensor.plugin", "nav2_costmap_2d::ObstacleLayer"),
         rclcpp::Parameter("sensor.observation_sources", ""),
         rclcpp::Parameter("sensor.footprint_clearing_enabled", false),
         rclcpp::Parameter("inflation.plugin", "nav2_costmap_2d::InflationLayer"),
         rclcpp::Parameter("inflation.inflation_radius", .68),
         rclcpp::Parameter("mapped.plugin", "mowgli_nav2_plugins::MappedObstacleLayer"),
         rclcpp::Parameter("global_frame", "odom"),
         rclcpp::Parameter("robot_base_frame", "base_link"),
         rclcpp::Parameter("rolling_window", true),
         rclcpp::Parameter("resolution", .02),
         rclcpp::Parameter("width", 6),
         rclcpp::Parameter("height", 6),
         rclcpp::Parameter("track_unknown_space", false),
         rclcpp::Parameter("footprint_padding", 0.0),
         rclcpp::Parameter("footprint",
                           "[[0.53,0.275],[0.53,-0.275],[-0.17,-0.275],[-0.17,0.275]]")});
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
    const auto* plugins = costmap->getLayeredCostmap()->getPlugins();
    sensor = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>((*plugins)[0]);
    layer = std::dynamic_pointer_cast<mowgli_nav2_plugins::MappedObstacleLayer>((*plugins)[2]);
    ASSERT_TRUE(sensor && layer);
  }
  void TearDown() override
  {
    layer.reset();
    sensor.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap.reset();
  }
  void transform(double x = 0, double y = 0, double yaw = 0)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "map";
    tf.header.stamp = costmap->now();
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2);
    tf.transform.rotation.w = std::cos(yaw / 2);
    ASSERT_TRUE(costmap->getTfBuffer()->setTransform(tf, "mapped_test", true));
  }
  void update(double x = 0, double y = 0)
  {
    costmap->getLayeredCostmap()->updateMap(x, y, 0);
  }
  void receive(const Snapshot& snapshot)
  {
    layer->receiveGeometry(std::make_shared<Snapshot>(snapshot));
  }
  unsigned char at(double x, double y)
  {
    unsigned mx = 0, my = 0;
    EXPECT_TRUE(costmap->getCostmap()->worldToMap(x, y, mx, my));
    return costmap->getCostmap()->getCost(mx, my);
  }
  void closed()
  {
    const auto* map = costmap->getCostmap();
    EXPECT_TRUE(std::all_of(map->getCharMap(),
                            map->getCharMap() + map->getSizeInCellsX() * map->getSizeInCellsY(),
                            [](unsigned char c)
                            {
                              return c == nav2_costmap_2d::LETHAL_OBSTACLE;
                            }));
    EXPECT_FALSE(layer->isCurrent());
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> sensor;
  std::shared_ptr<mowgli_nav2_plugins::MappedObstacleLayer> layer;
};

TEST_F(MappedObstacleTest, MissingInvalidAndEmptyStayClosedUntilValidRestoration)
{
  transform();
  update();
  closed();
  receive(geometry());
  closed();  // Immediate revocation before the normal update.
  update();
  ASSERT_TRUE(layer->isCurrent());
  EXPECT_EQ(at(0, 0), 0);
  for (int fault = 0; fault < 9; ++fault)
  {
    auto invalid = geometry();
    switch (fault)
    {
      case 0:
        invalid.markers.clear();
        break;
      case 1:
        invalid.markers[1].header.frame_id = "odom";
        break;
      case 2:
        invalid.markers[1].points[0].x = NAN;
        break;
      case 3:
        invalid.markers[1].points.resize(2);
        break;
      case 4:
        invalid.markers[1].ns = "unknown";
        break;
      case 5:
        invalid.markers[1].action = invalid.markers[1].DELETE;
        break;
      case 6:
        invalid.markers.resize(513);
        break;
      case 7:
        invalid.markers[1].points[0].z = 0.1;
        break;
      case 8:
        std::swap(invalid.markers[1].points[1], invalid.markers[1].points[2]);
        break;
    }
    SCOPED_TRACE(fault);
    receive(invalid);
    closed();
    update();
    closed();
    receive(geometry());
    update();
    EXPECT_TRUE(layer->isCurrent());
    EXPECT_EQ(at(0, 0), 0);
    EXPECT_EQ(at(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

TEST_F(MappedObstacleTest, OnlyRawHolesWithoutOuterDockOrPlannerMargin)
{
  transform();
  receive(geometry());
  update();
  EXPECT_EQ(at(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(at(1.4, 1), 0);  // Neither 0.275 m planner margin nor inflation.
  EXPECT_EQ(at(2, 0), 0);  // Outside the allowed union remains legal body slack.
  EXPECT_EQ(at(-.8, 0), 0);  // Dock body and approach remain sensor-authoritative.
  EXPECT_EQ(at(-1.6, 0), 0);
  auto tiny = geometry(false);
  append(tiny, "obstacle", box(.305, .305, .307, .307));
  receive(tiny);
  update();
  EXPECT_EQ(at(.306, .306), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(MappedObstacleTest, CurrentSensorLethalAndInflationSurviveHoleRemovalAndRecovery)
{
  transform();
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = -1.5;
  *y = 0;
  *z = .3;
  geometry_msgs::msg::Point origin;
  sensor->addStaticObservation(nav2_costmap_2d::Observation(origin, cloud, 3, 0, 8, 0),
                               true,
                               false);
  receive(geometry());
  update();
  EXPECT_EQ(at(-1.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(at(-1.2, 0), 0);
  receive(geometry(false));
  update();
  EXPECT_EQ(at(1, 1), 0);
  EXPECT_EQ(at(-1.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(at(-1.2, 0), 0);
  EXPECT_FALSE(layer->isClearable());
  layer->reset();
  closed();
  update();
  EXPECT_EQ(at(0, 0), 0);
  EXPECT_EQ(at(-1.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(MappedObstacleTest, RollingWindowAndChangedMapOdomTransformDoNotLeaveOldCells)
{
  receive(geometry());
  update();
  closed();  // No map->odom transform yet.
  transform();
  update();
  EXPECT_EQ(at(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  update(20, 0);
  EXPECT_EQ(at(20, 0), 0);
  update();
  EXPECT_EQ(at(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  transform(0, 0, M_PI_2);
  update();
  EXPECT_EQ(at(1, 1), 0);
  EXPECT_EQ(at(-1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  receive(geometry(false));
  update();
  EXPECT_EQ(at(-1, 1), 0);
}

TEST_F(MappedObstacleTest, FutureOnlyTransformCannotMoveHolesAwayFromCurrentBody)
{
  receive(geometry());
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "map";
  tf.header.stamp = costmap->now() + rclcpp::Duration::from_seconds(10);
  tf.transform.translation.x = 10;
  tf.transform.rotation.w = 1;
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(tf, "forecast", false));
  update();
  closed();
  costmap->getTfBuffer()->clear();
  transform();
  update();
  EXPECT_TRUE(layer->isCurrent());
  EXPECT_EQ(at(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getTfBuffer()->clear();
  update();
  closed();
}

class GoalChecker : public nav2_core::GoalChecker
{
public:
  bool xy_reached{true};
  void initialize(const nav2::LifecycleNode::WeakPtr&,
                  const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override
  {
  }
  bool isGoalReached(const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Twist&,
                     const nav_msgs::msg::Path&) override
  {
    return false;
  }
  bool isGoalXYReached(const geometry_msgs::msg::Pose&,
                       const geometry_msgs::msg::Pose&,
                       const geometry_msgs::msg::Twist&,
                       const nav_msgs::msg::Path&) override
  {
    return xy_reached;
  }
  bool getTolerances(geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Twist&, double&) override
  {
    pose.position.x = pose.position.y = .03;
    return true;
  }
};

TEST_F(MappedObstacleTest, ActualShimAndRppRefuseUnsensedMappedObstacles)
{
  transform();
  auto node = std::make_shared<nav2::LifecycleNode>("mapped_controller");
  node->declare_parameter("controller_frequency", 10.0);
  node->declare_parameter("FollowPath.primary_controller.plugin",
                          std::string("nav2_regulated_pure_pursuit_controller::"
                                      "RegulatedPurePursuitController"));
  node->declare_parameter("FollowPath.rotate_to_goal_heading", true);
  node->declare_parameter("FollowPath.primary_controller.use_rotate_to_heading", false);
  node->declare_parameter("FollowPath.primary_controller.use_collision_detection", true);
  node->declare_parameter("FollowPath.closed_loop", false);
  nav2_rotation_shim_controller::RotationShimController shim;
  shim.configure(node, "FollowPath", costmap->getTfBuffer(), costmap);
  shim.activate();
  GoalChecker checker;
  geometry_msgs::msg::PoseStamped pose, goal;
  pose.header.frame_id = "odom";
  pose.pose.orientation.w = 1;
  goal = pose;
  goal.pose.orientation.z = std::sin(.4 / 2);
  goal.pose.orientation.w = std::cos(.4 / 2);
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.assign(2, goal);
  for (auto& p : path.poses)
    p.header.frame_id = "base_link";
  const auto command = [&]
  {
    return shim.computeVelocityCommands(pose, geometry_msgs::msg::Twist(), &checker, path, goal)
        .twist;
  };
  const auto refuse = [&]
  {
    try
    {
      const auto output = command();
      EXPECT_DOUBLE_EQ(output.linear.x, 0);
      EXPECT_DOUBLE_EQ(output.angular.z, 0);
    }
    catch (const nav2_core::ControllerException&)
    {
    }
  };
  receive(geometry(false));
  update();
  EXPECT_GT(command().angular.z, 0);  // RED control: local sensors see no hole.
  auto snapshot = geometry(false);
  append(snapshot, "obstacle", box(.55, -.23, .57, -.17));
  receive(snapshot);
  update();
  shim.reset();
  refuse();  // Actual shim sees raw polygon costs without any sensor returns.
  checker.xy_reached = false;
  goal.pose.orientation.z = 0;
  goal.pose.orientation.w = 1;
  goal.pose.position.x = .8;
  path.poses.assign(1, goal);  // Actual RPP via supported shim sampling fallback.
  path.poses[0].header.frame_id = "base_link";
  receive(geometry(false));
  update();
  shim.reset();
  EXPECT_GT(command().linear.x, 0);
  snapshot = geometry(false);
  append(snapshot, "obstacle", box(.6, -.1, .7, .1));
  receive(snapshot);
  update();
  shim.reset();
  refuse();
  shim.deactivate();
  shim.cleanup();
}
}  // namespace
