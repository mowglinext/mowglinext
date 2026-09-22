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
// FTCController through a turn fallback, closed loop: the real controller, a
// real (configured, never updated) Costmap2DROS whose master grid the test
// paints, a kinematic diff-drive robot with the firmware's first-order speed
// response, and a controlled ROS clock stepping 0.1 s per control cycle.
//
//   * A U-turn whose apex overhangs a hedge: WEDGED would abort the goal; with
//     the fallback the robot pivots, drives straight, pivots, rejoins the
//     return swath and finishes the plan, never with a lethal cell under its
//     footprint, and hands the remainder to the goal checker's topic.
//   * The same scene with turn_fallback_enabled false: the old WEDGED ->
//     reverse-escape -> hold -> abort, unchanged.
//   * A wall across a straight: not a turn, the old path.
//   * An obstacle that appears once the fallback is under way: it stops and the
//     goal is aborted (hold, then ControllerException), never driven into.
//   * The no-LiDAR overlay (obstacle flags off): the fallback is unreachable.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace
{

namespace mn = mowgli_nav2_plugins;
using Pose = geometry_msgs::msg::PoseStamped;

constexpr double kDt = 0.1;  // controller_frequency 10 Hz
constexpr double kTau = 0.15;  // firmware wheel / yaw-rate loop response (s)
constexpr double kStep = 0.05;  // plan pose spacing
const char* const kPlugin = "FollowCoveragePath";

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

/// Swath A along +x to x_turn, a left U-turn of radius r, swath B back to x_end.
nav_msgs::msg::Path UTurnPlan(double x0, double x_turn, double r, double x_end)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x = x0; x < x_turn - 1e-9; x += kStep)
  {
    path.poses.push_back(MakePose(x, 0.0, 0.0));
  }
  const int n = static_cast<int>(std::ceil(M_PI * r / kStep));
  for (int k = 0; k <= n; ++k)
  {
    const double a = -M_PI / 2.0 + M_PI * k / n;
    path.poses.push_back(MakePose(x_turn + r * std::cos(a), r + r * std::sin(a), a + M_PI / 2.0));
  }
  for (double x = x_turn - kStep; x >= x_end - 1e-9; x -= kStep)
  {
    path.poses.push_back(MakePose(x, 2.0 * r, M_PI));
  }
  return path;
}

nav_msgs::msg::Path StraightPlan(double x0, double x1)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x = x0; x <= x1 + 1e-9; x += kStep)
  {
    path.poses.push_back(MakePose(x, 0.0, 0.0));
  }
  return path;
}

class Harness
{
public:
  struct Options
  {
    bool fallback{true};
    bool lidar{true};  // false = the no-LiDAR overlay's obstacle flags
  };

  explicit Harness(const Options& opt)
  {
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", "");
    DeclareFtcParams(opt);

    // The costmap declares its parameters when constructed; set them before
    // configure(). No layers: the test paints the master grid itself, and the
    // costmap is never activated, so no update thread repaints it.
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    for (const auto& p : std::vector<rclcpp::Parameter>{
             rclcpp::Parameter("plugins", std::vector<std::string>{}),
             rclcpp::Parameter("global_frame", "odom"),
             rclcpp::Parameter("robot_base_frame", "base_footprint"),
             rclcpp::Parameter("rolling_window", false),
             rclcpp::Parameter("width", 20),
             rclcpp::Parameter("height", 20),
             rclcpp::Parameter("resolution", 0.05),
             rclcpp::Parameter("origin_x", -5.0),
             rclcpp::Parameter("origin_y", -5.0),
             rclcpp::Parameter("footprint",
                               "[[0.53, 0.275], [0.53, -0.275], [-0.17, -0.275], [-0.17, 0.275]]"),
             rclcpp::Parameter("footprint_padding", 0.01),
             rclcpp::Parameter("transform_tolerance", 0.5)})
    {
      // Some are declared by the constructor, the rest in on_configure (with
      // declare-if-not-declared, which keeps a value declared here first).
      if (costmap_->has_parameter(p.get_name()))
      {
        EXPECT_TRUE(costmap_->set_parameter(p).successful) << p.get_name();
      }
      else
      {
        costmap_->declare_parameter(p.get_name(), p.get_parameter_value());
      }
    }
    costmap_->configure();
    tf_ = costmap_->getTfBuffer();
    {
      const auto* cm = costmap_->getCostmap();
      EXPECT_EQ(cm->getSizeInCellsX(), 400u);
      EXPECT_NEAR(cm->getOriginX(), -5.0, 1e-9);
    }

    // Controlled clock: every control cycle is exactly kDt.
    clock_ = node_->get_clock();
    EXPECT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    SetTime(1000.0);
    PublishStaticTf();
    PublishRobotTf();

    ftc_.configure(node_, kPlugin, tf_, costmap_);
    ftc_.activate();

    probe_ = std::make_shared<rclcpp::Node>("progress_probe");
    probe_sub_ = probe_->create_subscription<nav_msgs::msg::Path>(
        "/controller_server/FollowCoveragePath/global_plan",
        rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::Path::SharedPtr msg)
        {
          progress_plans_.push_back(*msg);
        });
    executor_.add_node(probe_);
  }

  ~Harness()
  {
    ftc_.deactivate();
    ftc_.cleanup();
    costmap_->cleanup();
    executor_.remove_node(probe_);
  }

  void MarkLethal(double x, double y)
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    auto* cm = costmap_->getCostmap();
    ASSERT_TRUE(cm->worldToMap(x, y, mx, my));
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
    cm->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  void MarkWallX(double x, double y0, double y1)
  {
    for (double y = y0; y <= y1 + 1e-9; y += 0.05)
    {
      MarkLethal(x, y);
    }
  }

  void SetPlan(const nav_msgs::msg::Path& path)
  {
    goal_ = path.poses.back();
    ftc_.newPathReceived(path);
  }

  void SetRobot(double x, double y, double yaw)
  {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    v_ = 0.0;
    w_ = 0.0;
    PublishRobotTf();
  }

  /// One control cycle: command, then integrate the robot. False once the
  /// controller has thrown (the goal would be aborted).
  bool Step()
  {
    geometry_msgs::msg::Twist odom;
    odom.linear.x = v_;
    odom.angular.z = w_;
    geometry_msgs::msg::TwistStamped cmd;
    try
    {
      cmd = ftc_.computeVelocityCommands(Pose{}, odom, nullptr, nav_msgs::msg::Path{}, goal_);
    }
    catch (const nav2_core::ControllerException& e)
    {
      aborted_ = e.what();
      return false;
    }
    last_cmd_ = cmd.twist;
    // Kinematic diff drive behind the firmware's first-order response.
    const double k = std::min(1.0, kDt / kTau);
    v_ += (cmd.twist.linear.x - v_) * k;
    w_ += (cmd.twist.angular.z - w_) * k;
    x_ += v_ * std::cos(yaw_) * kDt;
    y_ += v_ * std::sin(yaw_) * kDt;
    yaw_ = std::atan2(std::sin(yaw_ + w_ * kDt), std::cos(yaw_ + w_ * kDt));
    SetTime(t_ + kDt);
    PublishRobotTf();
    Track(cmd.twist);
    executor_.spin_some();
    return true;
  }

  /// Run until the controller throws, reaches the plan end at rest, or
  /// `max_s` of simulated time passes.
  void Run(double max_s, const std::function<void(Harness&)>& each_step = {})
  {
    const int steps = static_cast<int>(max_s / kDt);
    int idle = 0;
    for (int i = 0; i < steps; ++i)
    {
      if (each_step)
      {
        each_step(*this);
      }
      if (!Step())
      {
        return;
      }
      const bool at_end = std::hypot(x_ - goal_.pose.position.x, y_ - goal_.pose.position.y) < 0.6;
      idle = (at_end && std::abs(last_cmd_.linear.x) < 1e-9 && std::abs(last_cmd_.angular.z) < 1e-9)
                 ? idle + 1
                 : 0;
      if (idle > 20)
      {
        finished_ = true;
        return;
      }
    }
  }

  /// Let late DDS deliveries to the probe arrive.
  void Drain()
  {
    for (int i = 0; i < 50 && progress_plans_.empty(); ++i)
    {
      executor_.spin_some(std::chrono::milliseconds(20));
    }
  }

  /// The last command was an in-place rotation.
  bool RotatingInPlace() const
  {
    return std::abs(last_cmd_.linear.x) < 1e-6 && std::abs(last_cmd_.angular.z) > 0.05;
  }

  // ── Observations ───────────────────────────────────────────────────────────
  bool finished_{false};
  std::optional<std::string> aborted_;
  /// Steps with the body inside a lethal cell (bare footprint), must stay 0.
  int steps_in_lethal_{0};
  double min_clearance_m_{1e9};
  /// Total in-place rotation (rad) and number of in-place rotation episodes.
  double in_place_rotation_rad_{0.0};
  int in_place_episodes_{0};
  double max_reverse_speed_{0.0};
  std::vector<nav_msgs::msg::Path> progress_plans_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

private:
  void DeclareFtcParams(const Options& opt)
  {
    const auto set = [this](const std::string& key, const rclcpp::ParameterValue& value)
    {
      node_->declare_parameter(std::string(kPlugin) + "." + key, value);
    };
    // nav2_params_base.yaml + the template injections a real robot runs with.
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
    set("check_obstacles", rclcpp::ParameterValue(opt.lidar));
    set("enable_obstacle_deviation", rclcpp::ParameterValue(opt.lidar));
    set("obstacle_lookahead", rclcpp::ParameterValue(30));
    set("use_footprint_clearance", rclcpp::ParameterValue(true));
    set("obstacle_clearance_margin", rclcpp::ParameterValue(0.05));
    set("use_offset_lattice", rclcpp::ParameterValue(true));
    set("max_lateral_deviation", rclcpp::ParameterValue(1.0));
    // The zone guard needs /global_costmap/costmap; this scene has no zone.
    set("confine_deviation_to_zone", rclcpp::ParameterValue(false));
    set("obstacle_wait_timeout_s", rclcpp::ParameterValue(2.5));
    set("obstacle_clear_hold_s", rclcpp::ParameterValue(1.5));
    set("obstacle_reverse_enabled", rclcpp::ParameterValue(true));
    set("obstacle_reverse_max_dist_m", rclcpp::ParameterValue(0.30));
    set("obstacle_reverse_speed_mps", rclcpp::ParameterValue(0.15));
    set("turn_fallback_enabled", rclcpp::ParameterValue(opt.fallback));
  }

  static std::int64_t Ns(double seconds)
  {
    return static_cast<std::int64_t>(std::llround(seconds * 1e9));
  }

  void SetTime(double t)
  {
    t_ = t;
    EXPECT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), Ns(t)), RCL_RET_OK);
  }

  geometry_msgs::msg::TransformStamped Tf(
      const std::string& parent, const std::string& child, double x, double y, double yaw) const
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(Ns(t_), RCL_ROS_TIME);
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    return t;
  }

  void PublishStaticTf()
  {
    tf_->setTransform(Tf("map", "odom", 0.0, 0.0, 0.0), "test", true);
    tf_->setTransform(Tf("base_footprint", "base_link", 0.0, 0.0, 0.0), "test", true);
  }

  void PublishRobotTf()
  {
    tf_->setTransform(Tf("odom", "base_footprint", x_, y_, yaw_), "test", false);
  }

  void Track(const geometry_msgs::msg::Twist& cmd)
  {
    const auto footprint = costmap_->getRobotFootprint();
    auto* cm = costmap_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
    const double clearance = mn::FootprintClearance(*cm, footprint, {x_, y_, yaw_}, 1.0);
    min_clearance_m_ = std::min(min_clearance_m_, clearance);
    if (clearance <= 0.0)
    {
      ++steps_in_lethal_;
    }
    const bool rotating_in_place = std::abs(cmd.linear.x) < 1e-6 && std::abs(cmd.angular.z) > 0.05;
    if (rotating_in_place)
    {
      episode_rad_ += std::abs(w_) * kDt;
      in_place_rotation_rad_ += std::abs(w_) * kDt;
      if (episode_rad_ >= kEpisodeMinRad && !episode_counted_)
      {
        ++in_place_episodes_;
        episode_counted_ = true;
      }
    }
    else
    {
      episode_rad_ = 0.0;
      episode_counted_ = false;
    }
    max_reverse_speed_ = std::max(max_reverse_speed_, -cmd.linear.x);
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  mn::FTCController ftc_;
  rclcpp::Node::SharedPtr probe_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr probe_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  Pose goal_;
  geometry_msgs::msg::Twist last_cmd_;
  double t_{0.0};
  double v_{0.0};
  double w_{0.0};
  /// An in-place rotation counts as an episode once it has turned this far
  /// (POST_ROTATE's few degrees of heading trim at the goal do not).
  static constexpr double kEpisodeMinRad = 20.0 * M_PI / 180.0;
  double episode_rad_{0.0};
  bool episode_counted_{false};
};

class FtcTurnFallbackController : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

/// The row end of the field wedges: a U-turn whose apex overhangs x = 3.3,
/// the hedge at x = 3.45.
void UTurnIntoHedge(Harness& h)
{
  h.MarkWallX(3.45, -1.5, 2.0);
  h.SetRobot(0.0, 0.0, 0.0);
  h.SetPlan(UTurnPlan(0.0, 3.0, 0.3, 0.0));
}

TEST_F(FtcTurnFallbackController, UTurnIntoAHedgeIsImprovisedNotAborted)
{
  Harness h({});
  UTurnIntoHedge(h);
  h.Run(120.0);
  EXPECT_FALSE(h.aborted_.has_value()) << *h.aborted_;
  EXPECT_TRUE(h.finished_);
  // It ended on the return swath, at its end.
  EXPECT_NEAR(h.y_, 0.6, 0.15);
  EXPECT_LT(h.x_, 0.6);
  // Pivots, not an arc into the hedge: at least one in-place rotation, most of
  // the half turn done in place.
  EXPECT_GE(h.in_place_episodes_, 1);
  EXPECT_GT(h.in_place_rotation_rad_, M_PI / 3.0);
  // Never a lethal cell under the (padded) footprint.
  EXPECT_EQ(h.steps_in_lethal_, 0);
  EXPECT_GT(h.min_clearance_m_, 0.0);
  // The goal checker's topic got the remainder from the rejoin pose, an exact
  // pose of the plan on the return swath.
  h.Drain();
  ASSERT_FALSE(h.progress_plans_.empty());
  const auto& front = h.progress_plans_.back().poses.front().pose;
  EXPECT_NEAR(front.position.y, 0.6, 1e-9);
  EXPECT_EQ(h.progress_plans_.back().header.frame_id, "map");
}

TEST_F(FtcTurnFallbackController, WithoutTheFallbackTheSameUTurnAborts)
{
  Harness h({false, true});
  UTurnIntoHedge(h);
  h.Run(120.0);
  ASSERT_TRUE(h.aborted_.has_value());
  EXPECT_NE(h.aborted_->find("no collision-free offset profile"), std::string::npos) << *h.aborted_;
  EXPECT_FALSE(h.finished_);
  EXPECT_EQ(h.steps_in_lethal_, 0);
  // Today's WEDGED path: a straight reverse (no pivot), then hold, then abort.
  EXPECT_GT(h.max_reverse_speed_, 0.0);
  EXPECT_EQ(h.in_place_episodes_, 0);
  h.Drain();
  EXPECT_TRUE(h.progress_plans_.empty());
}

TEST_F(FtcTurnFallbackController, WallAcrossAStraightKeepsTheWedgedPath)
{
  Harness h({});
  h.MarkWallX(3.0, -1.5, 1.5);
  h.SetRobot(0.0, 0.0, 0.0);
  h.SetPlan(StraightPlan(0.0, 6.0));
  h.Run(120.0);
  ASSERT_TRUE(h.aborted_.has_value());
  EXPECT_NE(h.aborted_->find("no collision-free offset profile"), std::string::npos) << *h.aborted_;
  EXPECT_EQ(h.in_place_episodes_, 0);
  EXPECT_EQ(h.steps_in_lethal_, 0);
  h.Drain();
  EXPECT_TRUE(h.progress_plans_.empty());
}

TEST_F(FtcTurnFallbackController, ObstacleAppearingDuringTheFallbackStopsAndAborts)
{
  Harness h({});
  UTurnIntoHedge(h);
  // On the first tick of the fallback's first pivot, something appears beside
  // the front of the body, where the pivot is about to sweep it (whichever way
  // it turns). The per-tick sweep gate must stop the rotation, hold, and abort
  // the goal — the plan's own sweep check was done before it was there.
  bool dropped = false;
  h.Run(120.0,
        [&dropped](Harness& s)
        {
          if (dropped || !s.RotatingInPlace())
          {
            return;
          }
          for (double r = 0.50; r <= 0.60 + 1e-9; r += 0.05)
          {
            for (double a = 0.55; a <= 1.0 + 1e-9; a += 0.05)
            {
              for (const double side : {1.0, -1.0})
              {
                // Never under the current footprint (+ a cell): a cell there
                // would be an obstacle teleported into the robot.
                const double bx = r * std::cos(side * a);
                const double by = r * std::sin(side * a);
                if (bx > -0.25 && bx < 0.60 && std::abs(by) < 0.34)
                {
                  continue;
                }
                s.MarkLethal(s.x_ + r * std::cos(s.yaw_ + side * a),
                             s.y_ + r * std::sin(s.yaw_ + side * a));
              }
            }
          }
          dropped = true;
        });
  ASSERT_TRUE(dropped);
  ASSERT_TRUE(h.aborted_.has_value());
  EXPECT_NE(h.aborted_->find("pivot sweep"), std::string::npos) << *h.aborted_;
  EXPECT_FALSE(h.finished_);
  EXPECT_EQ(h.steps_in_lethal_, 0);
  h.Drain();
  EXPECT_TRUE(h.progress_plans_.empty());  // it never reached the rejoin
}

TEST_F(FtcTurnFallbackController, NoLidarOverlayNeverReachesTheFallback)
{
  // Obstacle flags off (nav2_params_no_lidar.yaml): FTC tracks the plan and
  // nothing reads the local costmap — the fallback code is not reachable.
  Harness h({true, false});
  h.SetRobot(0.0, 0.0, 0.0);
  h.SetPlan(UTurnPlan(0.0, 3.0, 0.3, 0.0));
  h.Run(120.0);
  EXPECT_FALSE(h.aborted_.has_value()) << *h.aborted_;
  EXPECT_TRUE(h.finished_);
  EXPECT_EQ(h.in_place_episodes_, 0);  // the U-turn is driven as the arc
  h.Drain();
  EXPECT_TRUE(h.progress_plans_.empty());
}

}  // namespace
