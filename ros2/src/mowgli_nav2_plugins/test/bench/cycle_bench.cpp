// Copyright (C) 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// Per-control-cycle cost of following ONE long coverage sub-path, as a function
// of its pose count. Not a test: a standalone benchmark (see README.md here).
//
// Three consumers run every controller_server cycle while FollowCoveragePath is
// active:
//   server  — controller_server's own path bookkeeping (Nav2 1.5.2
//             ControllerServer::transformedPlanAndGoal + the tracking-feedback
//             block of computeAndPublishVelocity), with the plugin named on the
//             command line: the upstream FeasiblePathHandler, or Mowgli's
//             LazyPrunePathHandler. Robot exactly on the plan.
//   ftc     — FTCController::computeVelocityCommands, closed loop on a kinematic
//             robot (the test_ftc_turn_fallback_controller harness, LiDAR flags on,
//             empty costmap).
//   checker — PathProgressGoalChecker::isGoalReached.
//
//   cycle_bench [server|ftc|checker] [cycles] [nav2_controller::FeasiblePathHandler|
//                                              mowgli_nav2_plugins/LazyPrunePathHandler]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <nav2_core/path_handler.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/path_utils.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"
#include <pluginlib/class_loader.hpp>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
struct FTCSpeedLimitTestAccess
{
  static void FreeMowingGeometry(FTCController& c)
  {
    auto g = std::make_shared<CornerGeometry>();
    g->valid = true;
    g->mowing.allowed = {{{-50, -50}, {50, -50}, {50, 250}, {-50, 250}}};
    c.corner_geometry_ = std::move(g);
  }
};
}  // namespace mowgli_nav2_plugins

namespace
{
using Clock = std::chrono::steady_clock;
using Pose = geometry_msgs::msg::PoseStamped;
constexpr double kDt = 0.1;  // controller_frequency 10 Hz (nav2_params_base.yaml)
constexpr double kStep = 0.03;  // coverage_server densify step
const std::vector<std::size_t> kSizes{1000, 10000, 50000, 100000, 200000, 400000};

Pose MakePose(double x, double y, double yaw)
{
  Pose p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

// Serpentine coverage: 10 m swaths 0.16 m apart joined by forward semicircles.
nav_msgs::msg::Path Serpentine(std::size_t n)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.reserve(n);
  const double len = 10.0, r = 0.08;
  double y = 0.0, dir = 1.0;
  while (path.poses.size() < n)
  {
    const double x0 = dir > 0 ? 0.0 : len;
    for (double s = 0.0; s < len - 1e-9 && path.poses.size() < n; s += kStep)
      path.poses.push_back(MakePose(x0 + dir * s, y, dir > 0 ? 0.0 : M_PI));
    const double cx = dir > 0 ? len : 0.0;
    for (double a = 0.0; a < M_PI - 1e-9 && path.poses.size() < n; a += kStep / r)
      path.poses.push_back(MakePose(cx + dir * r * std::sin(a),
                                    y + r - r * std::cos(a),
                                    (dir > 0 ? 0.0 : M_PI) + a));
    y += 2.0 * r;
    dir = -dir;
  }
  return path;
}

double Median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

double Us(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::micro>(b - a).count();
}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> MakeCostmap(const std::string& frame,
                                                           int size_m,
                                                           double origin)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
  for (const auto& p : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("plugins", std::vector<std::string>{}),
           rclcpp::Parameter("global_frame", frame),
           rclcpp::Parameter("robot_base_frame", "base_footprint"),
           rclcpp::Parameter("rolling_window", false),
           rclcpp::Parameter("width", size_m),
           rclcpp::Parameter("height", size_m),
           rclcpp::Parameter("resolution", 0.05),
           rclcpp::Parameter("origin_x", origin),
           rclcpp::Parameter("origin_y", origin),
           rclcpp::Parameter("footprint",
                             "[[0.53, 0.275], [0.53, -0.275], [-0.17, -0.275], [-0.17, 0.275]]"),
           rclcpp::Parameter("transform_tolerance", 0.5)})
  {
    if (costmap->has_parameter(p.get_name()))
      costmap->set_parameter(p);
    else
      costmap->declare_parameter(p.get_name(), p.get_parameter_value());
  }
  costmap->configure();
  return costmap;
}

// ── server ───────────────────────────────────────────────────────────────────
int RunServer(int cycles, const std::string& handler_type)
{
  auto parent = std::make_shared<nav2::LifecycleNode>("controller_server", "");
  auto costmap = MakeCostmap("map", 400, -200.0);
  // Shipped values: PathHandler.max_robot_pose_search_dist 10.0; prune_distance
  // and controller_server search_window are the 2.0 m defaults.
  parent->declare_parameter("PathHandler.max_robot_pose_search_dist", 10.0);
  pluginlib::ClassLoader<nav2_core::PathHandler> loader("nav2_core", "nav2_core::PathHandler");
  auto handler = loader.createUniqueInstance(handler_type);
  handler->initialize(parent, parent->get_logger(), "PathHandler", costmap, costmap->getTfBuffer());
  std::printf("path handler %s\n%8s %12s %12s %12s %12s %12s\n",
              handler_type.c_str(),
              "poses",
              "setplan_ms",
              "handler_us",
              "pathlen_us",
              "search_us",
              "cycle_us");
  for (const std::size_t n : kSizes)
  {
    const auto path = Serpentine(n);
    const auto t0 = Clock::now();
    handler->setPlan(path);  // ControllerServer::setPlannerPath
    nav_msgs::msg::Path current_path = path;
    const double setplan_ms = Us(t0, Clock::now()) / 1e3;
    std::size_t start_index = 0;
    std::vector<double> h_us, l_us, s_us, c_us;
    double sink = 0.0;
    for (int c = 0; c < cycles; ++c)
    {
      // 0.5 m/s at 10 Hz, robot on the plan.
      Pose pose =
          path.poses[std::min<std::size_t>(static_cast<std::size_t>(c * 0.05 / kStep), n - 1)];
      const auto a = Clock::now();
      auto [closest, end] = handler->findPlanSegment(pose);
      const auto local = handler->transformLocalPlan(closest, end);
      const auto b = Clock::now();
      const auto found = nav2_util::distance_from_path(current_path, pose.pose, start_index, 2.0);
      start_index = found.closest_segment_index;
      const auto d = Clock::now();
      sink += nav2_util::geometry_utils::calculate_path_length(current_path, start_index);
      const auto e = Clock::now();
      sink += static_cast<double>(local.poses.size());
      h_us.push_back(Us(a, b));
      s_us.push_back(Us(b, d));
      l_us.push_back(Us(d, e));
      c_us.push_back(Us(a, e));
    }
    std::printf("%8zu %12.2f %12.1f %12.1f %12.1f %12.1f   (sink %.0f)\n",
                n,
                setplan_ms,
                Median(h_us),
                Median(l_us),
                Median(s_us),
                Median(c_us),
                sink);
    std::fflush(stdout);
  }
  return 0;
}

// ── ftc ──────────────────────────────────────────────────────────────────────
void DeclareFtcParams(nav2::LifecycleNode& node)
{
  const auto set = [&node](const std::string& key, const rclcpp::ParameterValue& value)
  {
    node.declare_parameter(std::string("FollowCoveragePath.") + key, value);
  };
  // Same values as test_ftc_turn_fallback_controller (nav2_params_base.yaml).
  set("speed_fast", rclcpp::ParameterValue(0.20));
  set("speed_slow", rclcpp::ParameterValue(0.16));
  set("speed_fast_threshold", rclcpp::ParameterValue(0.5));
  set("speed_fast_threshold_angle", rclcpp::ParameterValue(10.0));
  set("speed_angular", rclcpp::ParameterValue(45.0));
  set("acceleration", rclcpp::ParameterValue(0.2));
  set("min_speed_mps", rclcpp::ParameterValue(0.15));
  set("kp_lon", rclcpp::ParameterValue(1.0));
  set("kp_lat", rclcpp::ParameterValue(0.8));
  set("kd_lat", rclcpp::ParameterValue(0.5));
  set("kp_ang", rclcpp::ParameterValue(1.5));
  set("kp_ang_following", rclcpp::ParameterValue(1.0));
  set("derivative_filter_tau", rclcpp::ParameterValue(0.2));
  set("max_cmd_vel_speed", rclcpp::ParameterValue(0.30));
  set("max_cmd_vel_ang", rclcpp::ParameterValue(0.8));
  set("max_goal_distance_error", rclcpp::ParameterValue(0.50));
  set("max_goal_angle_error", rclcpp::ParameterValue(30.0));
  set("goal_timeout", rclcpp::ParameterValue(10.0));
  set("max_follow_distance", rclcpp::ParameterValue(2.0));
  set("forward_only", rclcpp::ParameterValue(true));
  set("check_obstacles", rclcpp::ParameterValue(true));
  set("enable_obstacle_deviation", rclcpp::ParameterValue(true));
  set("obstacle_lookahead", rclcpp::ParameterValue(30));
  set("use_footprint_clearance", rclcpp::ParameterValue(true));
  set("obstacle_clearance_margin", rclcpp::ParameterValue(0.05));
  set("use_offset_lattice", rclcpp::ParameterValue(true));
  set("max_lateral_deviation", rclcpp::ParameterValue(1.0));
  set("confine_deviation_to_zone", rclcpp::ParameterValue(false));
  set("obstacle_reverse_enabled", rclcpp::ParameterValue(true));
  set("turn_fallback_enabled", rclcpp::ParameterValue(true));
}

geometry_msgs::msg::TransformStamped Tf(
    const std::string& parent, const std::string& child, double t, double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.rotation.z = std::sin(yaw / 2.0);
  tf.transform.rotation.w = std::cos(yaw / 2.0);
  return tf;
}

int RunFtc(int cycles)
{
  std::printf("%8s %12s %12s %12s\n", "poses", "newpath_ms", "cycle_us", "driven_m");
  for (const std::size_t n : kSizes)
  {
    auto node = std::make_shared<nav2::LifecycleNode>("controller_server", "");
    DeclareFtcParams(*node);
    auto costmap = MakeCostmap("odom", 20, -5.0);
    auto tf = costmap->getTfBuffer();
    auto clock = node->get_clock();
    if (rcl_enable_ros_time_override(clock->get_clock_handle()) != RCL_RET_OK)
      return 1;
    double t = 1000.0, x = 0.0, y = 0.0, yaw = 0.0, v = 0.0, w = 0.0;
    const auto set_time = [&]
    {
      if (rcl_set_ros_time_override(clock->get_clock_handle(), std::llround(t * 1e9)) != RCL_RET_OK)
        std::abort();
    };
    set_time();
    tf->setTransform(Tf("map", "odom", t, 0, 0, 0), "bench", true);
    tf->setTransform(Tf("base_footprint", "base_link", t, 0, 0, 0), "bench", true);
    tf->setTransform(Tf("odom", "base_footprint", t, x, y, yaw), "bench", false);
    mowgli_nav2_plugins::FTCController ftc;
    ftc.configure(node, "FollowCoveragePath", tf, costmap);
    ftc.activate();
    mowgli_nav2_plugins::FTCSpeedLimitTestAccess::FreeMowingGeometry(ftc);
    const auto path = Serpentine(n);
    const auto t0 = Clock::now();
    ftc.newPathReceived(path);
    const double newpath_ms = Us(t0, Clock::now()) / 1e3;
    std::vector<double> c_us;
    for (int c = 0; c < cycles; ++c)
    {
      geometry_msgs::msg::Twist odom;
      odom.linear.x = v;
      odom.angular.z = w;
      const auto a = Clock::now();
      const auto cmd = ftc.computeVelocityCommands(
          Pose{}, odom, nullptr, nav_msgs::msg::Path{}, path.poses.back());
      c_us.push_back(Us(a, Clock::now()));
      const double k = std::min(1.0, kDt / 0.15);
      v += (cmd.twist.linear.x - v) * k;
      w += (cmd.twist.angular.z - w) * k;
      x += v * std::cos(yaw) * kDt;
      y += v * std::sin(yaw) * kDt;
      yaw += w * kDt;
      t += kDt;
      set_time();
      tf->setTransform(Tf("odom", "base_footprint", t, x, y, yaw), "bench", false);
    }
    // Skip the PRE_ROTATE / acceleration start: time the steady FOLLOWING cycles.
    c_us.erase(c_us.begin(), c_us.begin() + std::min<std::size_t>(c_us.size() / 4, 50));
    std::printf("%8zu %12.2f %12.1f %12.2f\n", n, newpath_ms, Median(c_us), x);
    std::fflush(stdout);
    ftc.deactivate();
    ftc.cleanup();
  }
  return 0;
}

// ── checker ──────────────────────────────────────────────────────────────────
int RunChecker(int cycles)
{
  auto node = std::make_shared<nav2::LifecycleNode>("controller_server", "");
  auto costmap = MakeCostmap("map", 20, -5.0);
  mowgli_nav2_plugins::PathProgressGoalChecker checker;
  checker.initialize(node, "coverage_goal_checker", costmap);
  auto feeder = std::make_shared<rclcpp::Node>("plan_feeder");
  auto pub = feeder->create_publisher<nav_msgs::msg::Path>(
      "/controller_server/FollowCoveragePath/global_plan", rclcpp::QoS(1).transient_local());
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node->get_node_base_interface());
  std::printf("%8s %12s\n", "poses", "check_us");
  for (const std::size_t n : kSizes)
  {
    const auto path = Serpentine(n);
    checker.reset();
    pub->publish(path);
    // Let the plan reach the checker's subscription (its callback copies the
    // plan and builds the arc table once per goal; not timed here).
    const auto t0 = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - t0).count() < 2.0)
      ex.spin_some(std::chrono::milliseconds(10));
    std::vector<double> c_us;
    for (int c = 0; c < cycles; ++c)
    {
      const auto& q =
          path.poses[std::min<std::size_t>(static_cast<std::size_t>(c * 0.05 / kStep), n - 1)].pose;
      geometry_msgs::msg::Twist vel;
      vel.linear.x = 0.5;
      const auto a = Clock::now();
      checker.isGoalReached(q, path.poses.back().pose, vel, nav_msgs::msg::Path{});
      c_us.push_back(Us(a, Clock::now()));
    }
    std::printf("%8zu %12.2f\n", n, Median(c_us));
    std::fflush(stdout);
  }
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  const std::string mode = args.size() > 1 ? args[1] : "server";
  const int cycles = args.size() > 2 ? std::atoi(args[2].c_str()) : 200;
  const std::string handler =
      args.size() > 3 ? args[3] : std::string("nav2_controller::FeasiblePathHandler");
  int rc = 2;
  if (mode == "server")
    rc = RunServer(cycles, handler);
  else if (mode == "ftc")
    rc = RunFtc(cycles);
  else if (mode == "checker")
    rc = RunChecker(cycles);
  else
    std::fprintf(stderr, "usage: cycle_bench [server|ftc|checker] [cycles] [path handler]\n");
  rclcpp::shutdown();
  return rc;
}
