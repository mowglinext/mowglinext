// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Issue #924 acceptance tests. Negative cases intentionally fail on dev:
// an obstacle-free local costmap is not driving-geometry authorization.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
// Reuse the existing read-only test friend to establish DDS delivery. No state
// is injected: the production OccupancyGrid subscription builds this map.
struct FTCSpeedLimitTestAccess
{
  static void AuthorizeRotation(FTCController& controller, double error, double angular)
  {
    controller.current_state_ = FTCController::PlannerState::PRE_ROTATE;
    controller.angle_error_ = error;
    geometry_msgs::msg::Twist command;
    command.angular.z = angular;
    controller.authorizeCommand(command);
  }
  static bool ReceivedAuthorization(FTCController& controller,
                                    const visualization_msgs::msg::MarkerArray& message)
  {
    const auto expected = mowgli_interfaces::motion::Snapshot::parse(message);
    std::lock_guard<std::mutex> lock(controller.authorization_mutex_);
    return controller.authorization_ && controller.authorization_->valid == expected->valid &&
           controller.authorization_->identity == expected->identity;
  }
  static bool ReceivedGrid(FTCController& controller,
                           const nav_msgs::msg::OccupancyGrid& grid,
                           double x,
                           double y,
                           int cost)
  {
    std::lock_guard<std::mutex> lock(controller.boundary_mutex_);
    const auto* map = controller.boundary_costmap_.get();
    unsigned int mx = 0;
    unsigned int my = 0;
    return map && controller.boundary_frame_ == grid.header.frame_id &&
           map->getSizeInCellsX() == grid.info.width &&
           map->getSizeInCellsY() == grid.info.height && map->worldToMap(x, y, mx, my) &&
           (cost < 0 || map->getCost(mx, my) == cost);
  }
};
}  // namespace mowgli_nav2_plugins

namespace
{
using Pose = geometry_msgs::msg::PoseStamped;
constexpr char kPlugin[] = "FollowCoveragePath";

Pose MakePose(double x, double y = 0.0, double yaw = 0.0)
{
  Pose pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

nav_msgs::msg::Path Plan(double yaw = 0.0)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 120; ++i)
  {
    const double distance = i * 0.05;
    plan.poses.push_back(MakePose(distance * std::cos(yaw), distance * std::sin(yaw), yaw));
  }
  return plan;
}

// Real controller, real DDS input and configured local costmap. Only time and
// robot TF are supplied by the fixture. No hardware, mux or serial endpoint is
// launched; the observation is the controller's returned outgoing command.
class MotionAuthorization : public ::testing::TestWithParam<bool>
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
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", "");
    for (const auto& parameter :
         std::vector<rclcpp::Parameter>{{"speed_fast", 0.20},
                                        {"speed_slow", 0.16},
                                        {"max_cmd_vel_speed", 0.30},
                                        {"max_cmd_vel_ang", 0.8},
                                        {"kp_lon", 1.0},
                                        {"acceleration", 1.0},
                                        {"stall_grace_s", 30.0},
                                        {"goal_timeout", 30.0},
                                        {"check_obstacles", GetParam()},
                                        {"enable_obstacle_deviation", GetParam()},
                                        {"confine_deviation_to_zone", true},
                                        {"use_offset_lattice", true},
                                        {"obstacle_reverse_enabled", false},
                                        {"turn_fallback_enabled", false}})
    {
      node_->declare_parameter(std::string(kPlugin) + "." + parameter.get_name(),
                               parameter.get_parameter_value());
    }
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    for (const auto& parameter : std::vector<rclcpp::Parameter>{
             {"plugins", std::vector<std::string>{}},
             {"global_frame", "odom"},
             {"robot_base_frame", "base_footprint"},
             {"rolling_window", false},
             {"width", 20},
             {"height", 20},
             {"resolution", 0.05},
             {"origin_x", -5.0},
             {"origin_y", -5.0},
             {"footprint", "[[0.53,0.275],[0.53,-0.275],[-0.17,-0.275],[-0.17,0.275]]"},
             {"footprint_padding", 0.0}})
    {
      if (costmap_->has_parameter(parameter.get_name()))
      {
        ASSERT_TRUE(costmap_->set_parameter(parameter).successful);
      }
      else
      {
        costmap_->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
      }
    }
    costmap_->configure();
    tf_ = costmap_->getTfBuffer();
    clock_ = node_->get_clock();
    costmap_clock_ = costmap_->get_clock();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_enable_ros_time_override(costmap_clock_->get_clock_handle()), RCL_RET_OK);
    SetTime();
    Transform("map", "odom", true);
    Transform("base_footprint", "base_link", true);
    Transform("odom", "base_footprint", false);
    controller_.configure(node_, kPlugin, tf_, costmap_);
    controller_.activate();
    controller_.newPathReceived(Plan());
    grid_node_ = rclcpp::Node::make_shared("authorization_fixture");
    publisher_ = grid_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(1).transient_local());
    geometry_publisher_ = grid_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local());
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(grid_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(grid_node_);
    controller_.deactivate();
    controller_.cleanup();
    costmap_->cleanup();
  }

  void SetTime()
  {
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), time_ns_), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(costmap_clock_->get_clock_handle(), time_ns_), RCL_RET_OK);
  }

  void Transform(const std::string& parent, const std::string& child, bool fixed)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    if (child == "base_footprint")
    {
      transform.transform.translation.x = robot_x_;
    }
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test", fixed));
  }

  nav_msgs::msg::OccupancyGrid Grid(int8_t value)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = clock_->now();
    grid.info.width = 400;
    grid.info.height = 400;
    grid.info.resolution = 0.05;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(grid.info.width * grid.info.height, value);
    return grid;
  }

  void Deliver(const nav_msgs::msg::OccupancyGrid& grid,
               double witness_x = 0.0,
               double witness_y = 0.0,
               int witness_cost = -1)
  {
    // #942 originally supplied only a grid. Production permission now requires
    // recorded polygons too: supply matching fixture geometry through the REAL
    // authority subscription, retaining every original safety assertion.
    visualization_msgs::msg::MarkerArray snapshot;
    const auto rectangle = [&](double x0, double y0, double x1, double y1, const std::string& kind)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = kind;
      marker.text = kind == "area" ? "mowing" : "";
      marker.id = static_cast<int>(snapshot.markers.size());
      marker.type = marker.LINE_STRIP;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1;
      for (const auto& xy :
           std::vector<std::pair<double, double>>{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}})
      {
        geometry_msgs::msg::Point point;
        point.x = xy.first;
        point.y = xy.second;
        marker.points.push_back(point);
      }
      snapshot.markers.push_back(marker);
    };
    const bool known = std::none_of(grid.data.begin(),
                                    grid.data.end(),
                                    [](int8_t value)
                                    {
                                      return value < 0;
                                    });
    const bool any_free = std::any_of(grid.data.begin(),
                                      grid.data.end(),
                                      [](int8_t value)
                                      {
                                        return value < 99;
                                      });
    if (known && any_free)
    {
      const double ox = grid.info.origin.position.x, oy = grid.info.origin.position.y;
      const double resolution = grid.info.resolution;
      rectangle(
          ox, oy, ox + grid.info.width * resolution, oy + grid.info.height * resolution, "area");
      // Combine identical consecutive row spans into one raw rectangular hole.
      // These polygons encode the fixture's authored forbidden rectangles, not
      // an authorization verdict or a permissive production grid fallback.
      for (unsigned int y = 0; y < grid.info.height; ++y)
        for (unsigned int x = 0; x < grid.info.width;)
        {
          if (grid.data[y * grid.info.width + x] < 99)
          {
            ++x;
            continue;
          }
          const unsigned int first = x;
          while (x < grid.info.width && grid.data[y * grid.info.width + x] >= 99)
            ++x;
          const double x0 = ox + first * resolution, x1 = ox + x * resolution;
          const double y0 = oy + y * resolution, y1 = oy + (y + 1) * resolution;
          if (snapshot.markers.size() > 1)
          {
            auto& last = snapshot.markers.back();
            if (last.points[0].x == x0 && last.points[1].x == x1 && last.points[2].y == y0)
            {
              last.points[2].y = y1;
              last.points[3].y = y1;
              continue;
            }
          }
          rectangle(x0, y0, x1, y1, "obstacle");
        }
    }
    ASSERT_NO_FATAL_FAILURE(DeliverGeometry(snapshot));
    publisher_->publish(grid);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ReceivedGrid(
              controller_, grid, witness_x, witness_y, witness_cost))
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    FAIL() << "Production geometry subscription did not consume the fixture";
  }

  void DeliverGeometry(const visualization_msgs::msg::MarkerArray& snapshot)
  {
    geometry_publisher_->publish(snapshot);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ReceivedAuthorization(controller_,
                                                                              snapshot))
        return;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    FAIL() << "Production polygon subscription did not consume the fixture";
  }

  geometry_msgs::msg::Twist Tick()
  {
    time_ns_ += 100000000;
    SetTime();
    Transform("odom", "base_footprint", false);
    Pose current_pose;
    EXPECT_TRUE(costmap_->getRobotPose(current_pose)) << "Fixture pose/TF must be valid and fresh";
    geometry_msgs::msg::Twist measured;
    measured.linear.x = 0.20;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist;
  }

  double PeakForward()
  {
    double peak = 0.0;
    for (int i = 0; i < 10; ++i)
    {
      peak = std::max(peak, Tick().linear.x);
    }
    return peak;
  }

  void ExpectNoMotion()
  {
    double linear_peak = 0.0;
    double angular_peak = 0.0;
    for (int i = 0; i < 10; ++i)
    {
      try
      {
        const auto cmd = Tick();
        EXPECT_TRUE(std::isfinite(cmd.linear.x));
        EXPECT_TRUE(std::isfinite(cmd.angular.z));
        linear_peak = std::max(linear_peak, std::abs(cmd.linear.x));
        angular_peak = std::max(angular_peak, std::abs(cmd.angular.z));
      }
      catch (const nav2_core::ControllerException&)
      {
        break;  // Rejection must not erase any earlier nonzero output.
      }
    }
    EXPECT_DOUBLE_EQ(linear_peak, 0.0) << "Unauthorized forward/reverse command";
    EXPECT_DOUBLE_EQ(angular_peak, 0.0) << "Unauthorized rotation command";
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Clock::SharedPtr costmap_clock_;
  mowgli_nav2_plugins::FTCController controller_;
  rclcpp::Node::SharedPtr grid_node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometry_publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  int64_t time_ns_{1000000000000};
  double robot_x_{0.0};
};

TEST_P(MotionAuthorization, ClearGeometryControlProducesForwardMotion)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  EXPECT_GT(PeakForward(), 0.0);
}

TEST_P(MotionAuthorization, ClearGeometryControlProducesPreRotation)
{
  // Same initial pose, body and pi/2 plan as the forbidden-sweep case below;
  // only the keepout is absent. Blocking every rotation cannot satisfy both.
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  controller_.newPathReceived(Plan(std::acos(-1.0) / 2.0));
  double angular_peak = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    const auto cmd = Tick();
    ASSERT_TRUE(std::isfinite(cmd.linear.x));
    ASSERT_TRUE(std::isfinite(cmd.angular.z));
    EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);  // PRE_ROTATE must turn in place.
    angular_peak = std::max(angular_peak, cmd.angular.z);
  }
  EXPECT_GT(angular_peak, 0.0) << "Authorized pi/2 rotation must produce an angular command";
}

TEST_P(MotionAuthorization, MissingGeometryMustNotAuthorizeNominalMotion)
{
  // No geometry publication, no docking/recovery exception, valid pose/TF.
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, GridAloneCannotGrantPermission)
{
  const auto grid = Grid(0);
  publisher_->publish(grid);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool received = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor_.spin_some();
    received =
        mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ReceivedGrid(controller_, grid, 0, 0, 0);
    if (received)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(received);
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, AccumulatorRevolutionsDoNotEnlargeRemainingRotation)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  visualization_msgs::msg::MarkerArray snapshot;
  for (const auto& entry :
       std::vector<std::pair<std::string, std::vector<std::pair<double, double>>>>{
           {"area", {{-5, -5}, {10, -5}, {10, 10}, {-5, 10}}},
           {"obstacle", {{-0.5, -0.1}, {-0.4, -0.1}, {-0.4, 0.1}, {-0.5, 0.1}}}})
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = entry.first;
    marker.text = entry.first == "area" ? "mowing" : "";
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    for (const auto& xy : entry.second)
    {
      geometry_msgs::msg::Point point;
      point.x = xy.first;
      point.y = xy.second;
      marker.points.push_back(point);
    }
    snapshot.markers.push_back(marker);
  }
  ASSERT_NO_FATAL_FAILURE(DeliverGeometry(snapshot));
  for (double sign : {-1.0, 1.0})
  {
    EXPECT_NO_THROW(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeRotation(
        controller_, sign * (2 * M_PI + 0.2), sign * 0.5));
    // Actual command opposite the target takes the long directed sweep.
    EXPECT_THROW(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeRotation(
                     controller_, sign * (2 * M_PI + 0.2), -sign * 0.5),
                 nav2_core::ControllerException);
  }
}

TEST_P(MotionAuthorization, ClearGridCannotKeepRevokedPolygonPermissionAlive)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  ASSERT_GT(PeakForward(), 0);
  ASSERT_NO_FATAL_FAILURE(DeliverGeometry(visualization_msgs::msg::MarkerArray{}));
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, StalePoseRejectsRotationButAlwaysAllowsStop)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  ASSERT_GT(PeakForward(), 0);
  time_ns_ += 1000000000;
  SetTime();  // Deliberately do not supply a new physical pose/TF observation.
  EXPECT_THROW(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeRotation(controller_,
                                                                               0.2,
                                                                               0.5),
               nav2_core::ControllerException);
  EXPECT_NO_THROW(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeRotation(controller_, 0.2, 0));
  ASSERT_NO_FATAL_FAILURE(DeliverGeometry(visualization_msgs::msg::MarkerArray{}));
  EXPECT_NO_THROW(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeRotation(controller_, 0.2, 0));
}

TEST_P(MotionAuthorization, LegalOuterEdgeOverhangStillMakesForwardProgress)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  visualization_msgs::msg::MarkerArray snapshot;
  visualization_msgs::msg::Marker area;
  area.header.frame_id = "map";
  area.ns = "area";
  area.text = "mowing";
  area.type = area.LINE_STRIP;
  area.action = area.ADD;
  area.pose.orientation.w = 1;
  for (const auto& xy : std::vector<std::pair<double, double>>{{0, 0}, {6, 0}, {6, 6}, {0, 6}})
  {
    geometry_msgs::msg::Point point;
    point.x = xy.first;
    point.y = xy.second;
    area.points.push_back(point);
  }
  snapshot.markers.push_back(area);
  ASSERT_NO_FATAL_FAILURE(DeliverGeometry(snapshot));
  // Rear and half the body protrude outside the recorded perimeter at start.
  double distance = 0;
  int moving_ticks = 0;
  for (int i = 0; i < 50; ++i)
  {
    const auto command = Tick();
    ASSERT_GE(command.linear.x, 0);
    moving_ticks += command.linear.x > 0 ? 1 : 0;
    distance += command.linear.x * 0.1;
    robot_x_ = distance;
  }
  EXPECT_GT(distance, 0.5);
  EXPECT_GE(moving_ticks, 40) << "A startup stop must not become a coverage deadlock";
}

TEST_P(MotionAuthorization, UnknownGeometryMustNotAuthorizeNominalMotion)
{
  // Verify delivery independently of how unknown is stored internally.
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(-1)));
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, RevokedGeometryMustStopAnAlreadyDispatchedPath)
{
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(0)));
  ASSERT_GT(PeakForward(), 0.0);
  // Replacement forbids every point. The old controller path remains active;
  // no new plan dispatch is involved (planning revocation is #905/#906).
  ASSERT_NO_FATAL_FAILURE(Deliver(Grid(100), 0.0, 0.0, 254));
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, NominalCoverageMustNotCommandAcrossForbiddenBoundary)
{
  auto grid = Grid(0);
  // The permitted reference-point region ends at x=0.70. Initial body fits;
  // beyond the line is forbidden, with no overhang or recovery exception.
  for (unsigned int my = 0; my < grid.info.height; ++my)
  {
    for (unsigned int mx = 114; mx < grid.info.width; ++mx)
    {
      grid.data[my * grid.info.width + mx] = 100;
    }
  }
  ASSERT_NO_FATAL_FAILURE(Deliver(grid, 0.775, 0.0, 254));
  for (int i = 0; i < 60; ++i)
  {
    try
    {
      const auto cmd = Tick();
      ASSERT_TRUE(std::isfinite(cmd.linear.x));
      ASSERT_TRUE(std::isfinite(cmd.angular.z));
      ASSERT_NEAR(cmd.angular.z, 0.0, 1e-9);  // Pure translation oracle.
      const double next_x = robot_x_ + 0.1 * cmd.linear.x;
      ASSERT_LE(next_x, 0.70) << "Commanded segment crosses the forbidden boundary from x="
                              << robot_x_;
      robot_x_ = next_x;
    }
    catch (const nav2_core::ControllerException&)
    {
      break;  // Rejection before crossing is allowed.
    }
  }
}

TEST_P(MotionAuthorization, PreRotateMustRejectAForbiddenFootprintSweep)
{
  auto grid = Grid(0);
  // Hole [0.30,0.40] x [0.35,0.45]. Axle and initial body are clear.
  // Interior witness (0.325,0.375) has body coordinates (0.495,0.035)
  // at pi/4: strictly inside the 0.53 m front and 0.275 m side, but outside
  // BOTH rotation endpoints (yaw 0 and pi/2). The nominal +Y path and its
  // final heading's body are clear. This forbids rotation without restricting the
  // approved outer-boundary overhang policy.
  for (unsigned int mx = 106; mx < 108; ++mx)
  {
    for (unsigned int my = 107; my < 109; ++my)
    {
      grid.data[my * grid.info.width + mx] = 100;
    }
  }
  ASSERT_NO_FATAL_FAILURE(Deliver(grid, 0.325, 0.375, 254));
  ASSERT_GT(0.375, 0.275);
  ASSERT_GT(0.325, 0.275);
  ASSERT_LT((0.325 + 0.375) / std::sqrt(2.0), 0.53);
  controller_.newPathReceived(Plan(std::acos(-1.0) / 2.0));
  ExpectNoMotion();
}

TEST_P(MotionAuthorization, TranslationMustRejectAKeepoutInTheChassisSweep)
{
  auto grid = Grid(0);
  // Hole [0.60,0.70] x [0.20,0.25]. The +X axle path at y=0 is clear,
  // as is the initial body (front x=0.53). The hole lies inside the body's
  // lateral span, so translating past axle x=0.07 would drive the chassis
  // through it. A hole cannot use the approved outer-boundary overhang policy.
  for (unsigned int mx = 112; mx < 114; ++mx)
  {
    grid.data[104 * grid.info.width + mx] = 100;
  }
  ASSERT_NO_FATAL_FAILURE(Deliver(grid, 0.625, 0.225, 254));
  constexpr double front = 0.53;
  constexpr double rear = -0.17;
  constexpr double half_width = 0.275;
  constexpr double hole_min_x = 0.60;
  constexpr double hole_max_x = 0.70;
  ASSERT_LT(front, hole_min_x);  // Initial complete body does not touch the hole.
  ASSERT_LT(0.0, 0.20);  // Every axle segment at y=0 avoids the hole.
  ASSERT_LT(0.25, half_width);  // Positive lateral overlap with the complete body.
  for (int i = 0; i < 60; ++i)
  {
    try
    {
      const auto cmd = Tick();
      ASSERT_TRUE(std::isfinite(cmd.linear.x));
      ASSERT_TRUE(std::isfinite(cmd.angular.z));
      ASSERT_NEAR(cmd.angular.z, 0.0, 1e-9);  // Pure translation oracle.
      const double next_x = robot_x_ + 0.1 * cmd.linear.x;
      // At yaw=0 the exact full-body sweep of this straight segment is
      // [min(axle endpoints)+rear, max(axle endpoints)+front] x [-width,width].
      // Reject positive-area intersection, independently of controller checks.
      const double sweep_min_x = std::min(robot_x_, next_x) + rear;
      const double sweep_max_x = std::max(robot_x_, next_x) + front;
      ASSERT_TRUE(sweep_max_x <= hole_min_x || sweep_min_x >= hole_max_x)
          << "Chassis sweep intersects keepout: axle x=" << robot_x_ << " -> " << next_x
          << ", body sweep x=[" << sweep_min_x << "," << sweep_max_x << "]";
      robot_x_ = next_x;
    }
    catch (const nav2_core::ControllerException&)
    {
      break;  // Rejection before any chassis intersection is allowed.
    }
  }
}

INSTANTIATE_TEST_SUITE_P(LidarModes,
                         MotionAuthorization,
                         ::testing::Values(false, true),
                         [](const auto& info)
                         {
                           return info.param ? "Lidar" : "NoLidar";
                         });
}  // namespace
