// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Real goal checker, RotationShim/RPP and Costmap2DROS with a bounded software
// plant. These tests establish admission/arrival behavior, not hardware stopping.
#include <thread>

#include <nav2_controller/plugins/simple_goal_checker.hpp>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include "costmap_test_support.hpp"
#include "mowgli_nav2_plugins/coverage_entry_goal_checker.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
namespace mn = mowgli_nav2_plugins;
using Pose = geometry_msgs::msg::PoseStamped;
Pose pose(double x, double y, double yaw = 0)
{
  Pose p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = sin(yaw / 2);
  p.pose.orientation.w = cos(yaw / 2);
  return p;
}
visualization_msgs::msg::Marker marker(const std::vector<std::pair<double, double>>& ring,
                                       std::string role = "mowing",
                                       std::string ns = "area")
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.ns = ns;
  m.type = m.LINE_STRIP;
  m.action = m.ADD;
  m.text = role;
  m.pose.orientation.w = 1;
  for (auto [x, y] : ring)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    m.points.push_back(p);
  }
  return m;
}
visualization_msgs::msg::MarkerArray rectangle(double lo = -1, double hi = 1)
{
  visualization_msgs::msg::MarkerArray m;
  m.markers.push_back(marker({{-3, lo}, {-1, lo}, {-1, hi}, {-3, hi}}));
  return m;
}
struct Harness
{
  std::shared_ptr<nav2::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cm;
  nav2_rotation_shim_controller::RotationShimController shim;
  nav2::TransformBuffer::SharedPtr tf;
  mn::CoverageEntryGoalChecker entry;
  nav2_controller::SimpleGoalChecker ordinary;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub;
  std::int64_t clocktick = 0;
  Harness()
  {
    rclcpp::NodeOptions opts;
    opts.arguments({"--ros-args", "--params-file", MOWGLI_TRANSIT_PARAMS_PATH});
    node = std::make_shared<nav2::LifecycleNode>("controller_server", "", opts);
    cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>("checker_costmap", "/", false);
    for (auto p : std::vector<rclcpp::Parameter>{
             {"plugins", std::vector<std::string>{}},
             {"global_frame", "odom"},
             {"robot_base_frame", "base_footprint"},
             {"rolling_window", false},
             {"width", 10},
             {"height", 10},
             {"resolution", .05},
             {"origin_x", -5.},
             {"origin_y", -5.},
             {"footprint", "[[0.53,0.275],[0.53,-0.275],[-0.17,-0.275],[-0.17,0.275]]"},
             {"footprint_padding", .01}})
    {
      if (cm->has_parameter(p.get_name()))
        cm->set_parameter(p);
      else
        cm->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    cm->configure();
    tf = cm->getTfBuffer();
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "map";
      t.child_frame_id = "odom";
      t.transform.rotation.w = 1;
      tf->setTransform(t, "explicit identity map/odom", true);
    }
    if (rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()) != RCL_RET_OK)
      abort();
    time();
    shim.configure(node, "FollowPath", tf, cm);
    shim.activate();
    entry.initialize(node, "coverage_entry_goal_checker", cm);
    {
      geometry_msgs::msg::Twist stopped;
      nav_msgs::msg::Path path;
      check(!entry.isGoalReached(pose(-2.95, .02).pose, pose(-2.997, .003).pose, stopped, path),
            "pre-controller-missing-coast-contract-denied");
    }
    node->declare_parameter("FollowCoveragePath.pivot_coast_time_bound_s", .5);
    ordinary.initialize(node, "transit_goal_checker", cm);
    pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local());
  }
  ~Harness()
  {
    shim.deactivate();
    shim.cleanup();
    mowgli_nav2_plugins_test::WaitForTfListenerSpinning(*cm);
    cm->cleanup();
  }
  void time()
  {
    if (rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
                                  1000000000000LL + clocktick * 100000000LL) != RCL_RET_OK)
      abort();
    clocktick++;
  }
  void geometry(const visualization_msgs::msg::MarkerArray& g)
  {
    pub->publish(g);
    for (int i = 0; i < 20; i++)
    {
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void check(bool b, const char* n)
  {
    EXPECT_TRUE(b) << n;
  }
  void transform(double x, double y, double yaw)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = sin(yaw / 2);
    t.transform.rotation.w = cos(yaw / 2);
    tf->setTransform(t, "probe", true);
  }
  void staticControls()
  {
    geometry(rectangle());
    auto g = pose(-2.9969917838565916, .0029821079164083497, 3.0622);
    geometry_msgs::msg::Twist stopped;
    nav_msgs::msg::Path p;
    check(!entry.isGoalReached(pose(-3.0217157814, .0215044462).pose, g.pose, stopped, p),
          "captured-outside-denied");
    check(entry.isGoalReached(pose(-3, .003).pose, g.pose, stopped, p),
          "exact-closed-edge-stopped");
    check(!entry.isGoalReached(pose(-3 - 2e-8, .003).pose, g.pose, stopped, p),
          "original-epsilon-preserved");
    auto holes = rectangle();
    holes.markers.push_back(
        marker({{-2.99, -.05}, {-2.91, -.05}, {-2.91, .1}, {-2.99, .1}}, "", "obstacle"));
    geometry(holes);
    check(!entry.isGoalReached(pose(-2.95, .02).pose, g.pose, stopped, p), "hole-interior-denied");
    check(!entry.isGoalReached(pose(-2.99, .02).pose, g.pose, stopped, p), "hole-edge-denied");
    auto nav = rectangle();
    nav.markers.front().text = "navigation";
    geometry(nav);
    check(!entry.isGoalReached(pose(-2.95, .02).pose, g.pose, stopped, p),
          "navigation-only-denied");
    auto islands = rectangle();
    islands.markers.push_back(marker({{-3.2, .01}, {-3.01, .01}, {-3.01, .2}, {-3.2, .2}}));
    geometry(islands);
    check(!entry.isGoalReached(pose(-3.02, .02).pose, g.pose, stopped, p),
          "disconnected-nearby-mow-denied");
    auto bad = rectangle();
    bad.markers.front().points[1].x = std::numeric_limits<double>::quiet_NaN();
    geometry(bad);
    check(!entry.isGoalReached(pose(-2.95, .02).pose, g.pose, stopped, p),
          "nonfinite-geometry-denied");
    geometry({});
    check(!entry.isGoalReached(pose(-2.95, .02).pose, g.pose, stopped, p),
          "live-empty-snapshot-revokes");
    geometry(rectangle());
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "map";
      t.child_frame_id = "odom";
      t.transform.translation.x = .4;
      t.transform.translation.y = -.2;
      t.transform.rotation.w = 1;
      tf->setTransform(t, "nonzero map/odom control", true);
      auto shifted = pose(g.pose.position.x - .4, g.pose.position.y + .2);
      check(entry.isGoalReached(pose(-3.35, .22).pose, shifted.pose, stopped, p),
            "odom-query-transformed-to-map");
      tf->clear();
      check(!entry.isGoalReached(pose(-3.35, .22).pose, shifted.pose, stopped, p),
            "missing-map-transform-denied");
      t.transform.translation.x = 0;
      t.transform.translation.y = 0;
      tf->setTransform(t, "restore identity", true);
    }
    stopped.linear.x = .009;
    check(entry.isGoalReached(pose(-2.994, .003).pose, g.pose, stopped, p),
          "current-half-second-coast-used");
    node->set_parameter(rclcpp::Parameter("FollowCoveragePath.pivot_coast_time_bound_s", 2.));
    check(!entry.isGoalReached(pose(-2.994, .003).pose, g.pose, stopped, p),
          "dynamic-coast-contract-used");
    node->set_parameter(rclcpp::Parameter("FollowCoveragePath.pivot_coast_time_bound_s", .5));
    stopped.linear.x = .01;
    check(!entry.isGoalReached(pose(-2.999, .003).pose, g.pose, stopped, p),
          "uncertified-residual-coast-denied");
    check(entry.isGoalXYReached(pose(-2.999, .003).pose, g.pose, stopped, p),
          "xy-brakes-while-terminal-waits");
  }
  void run(std::string name,
           double x,
           double y,
           double yaw,
           double v,
           bool normal = false,
           double width = 1)
  {
    geometry(rectangle(-width, width));
    auto& checker = normal ? static_cast<nav2_core::GoalChecker&>(ordinary)
                           : static_cast<nav2_core::GoalChecker&>(entry);
    checker.reset();
    double w = 0;
    const auto goal = pose(-2.9969917838565916, .0029821079164083497, 3.0622);
    nav_msgs::msg::Path raw;
    raw.header.frame_id = "map";
    raw.poses = {pose(x, y, yaw), goal};
    shim.reset();
    shim.newPathReceived(raw);
    int n = 0;
    bool done = false;
    for (; n < 200; n++)
    {
      transform(x, y, yaw);
      auto p = pose(x, y, yaw);
      nav_msgs::msg::Path plan;
      plan.header.frame_id = "base_footprint";
      plan.poses = {
          pose(0, 0),
          pose((goal.pose.position.x - x) * cos(yaw) + (goal.pose.position.y - y) * sin(yaw),
               -(goal.pose.position.x - x) * sin(yaw) + (goal.pose.position.y - y) * cos(yaw))};
      for (auto& q : plan.poses)
        q.header.frame_id = "base_footprint";
      geometry_msgs::msg::Twist vel;
      vel.linear.x = v;
      vel.angular.z = w;
      done = checker.isGoalReached(p.pose, goal.pose, vel, plan);
      if (done)
        break;
      auto cmd = shim.computeVelocityCommands(p, vel, &checker, plan, goal);
      v += (cmd.twist.linear.x - v) * (1 - exp(-.1 / .15));
      w += (cmd.twist.angular.z - w) * (1 - exp(-.1 / .15));
      x += v * cos(yaw) * .1;
      y += v * sin(yaw) * .1;
      yaw += w * .1;
      time();
    }

    bool coastclear = true;
    for (int k = 0; k < 20; k++)
    {
      v *= exp(-.1 / .15);
      w *= exp(-.1 / .15);
      x += v * cos(yaw) * .1;
      y += v * sin(yaw) * .1;
      yaw += w * .1;
      coastclear =
          coastclear &&
          mn::transit::inside({x, y}, {{-3, -width}, {-1, -width}, {-1, width}, {-3, width}});
    }
    check(done, (name + "-eventual-arrival").c_str());
    if (!normal)
      check(coastclear, (name + "-residual-coast-contained").c_str());
  }
};
class CoverageEntryGoalCheckerTest : public ::testing::Test
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
};
TEST_F(CoverageEntryGoalCheckerTest, GeometryFramesAndLiveCoastContract)
{
  Harness h;
  h.staticControls();
}
TEST_F(CoverageEntryGoalCheckerTest, CapturedOutsideAcquiresInsideAndCoastsInward)
{
  Harness h;
  h.run("captured", -3.0217157814254207, .021504446211422223, 0, .2);
}
TEST_F(CoverageEntryGoalCheckerTest, InteriorOutwardApproachStopsInside)
{
  Harness h;
  h.run("interior-outward", -2.6, .003, M_PI, .2);
}
TEST_F(CoverageEntryGoalCheckerTest, NarrowMowPassageStillArrives)
{
  Harness h;
  h.run("narrow", -3.04, 0, 0, .2, false, .28);
}
TEST_F(CoverageEntryGoalCheckerTest, ObliqueEntryStillArrives)
{
  Harness h;
  h.run("oblique", -3.2, -.05, .2, .2);
}
TEST_F(CoverageEntryGoalCheckerTest, OrdinaryNavigationRetainsProximitySemantics)
{
  Harness h;
  h.run("ordinary-nav", -3.0217, .0215, 0, .2, true);
}
