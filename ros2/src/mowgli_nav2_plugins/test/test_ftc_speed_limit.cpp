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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_fixture.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
struct FTCSpeedLimitTestAccess
{
  static bool GeometryReady(FTCController& controller)
  {
    std::lock_guard<std::mutex> lock(controller.authorization_mutex_);
    return controller.authorization_ != nullptr;
  }
  static bool TurnFallbackReversing(const FTCController& controller)
  {
    return controller.turnFallbackReversing();
  }

  static bool Stalled(const FTCController& controller)
  {
    return controller.is_stalled_;
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

nav_msgs::msg::Path StraightPlan(double first = 0.0)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (double x = first; x <= 6.0; x += 0.05)
  {
    plan.poses.push_back(MakePose(x));
  }
  return plan;
}

// This fixture exercises the real controller with an empty configured costmap
// and explicit TF. It advances only the ROS clock: the fixed robot pose and
// moving odometry isolate command bounds from drivetrain response and DDS.
class FtcSpeedLimit : public ::testing::Test
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
                                        {"speed_fast_threshold", 0.5},
                                        {"min_speed_mps", 0.15},
                                        {"max_cmd_vel_speed", 0.30},
                                        {"max_cmd_vel_ang", 0.8},
                                        {"kp_lon", 1.0},
                                        {"acceleration", 1.0},
                                        {"stall_grace_s", 30.0},
                                        {"max_follow_distance", 2.0},
                                        {"goal_timeout", 30.0},
                                        {"check_obstacles", false},
                                        {"enable_obstacle_deviation", false},
                                        {"confine_deviation_to_zone", false},
                                        {"use_offset_lattice", true},
                                        {"use_footprint_clearance", true},
                                        {"obstacle_reverse_enabled", true},
                                        {"obstacle_reverse_speed_mps", 0.15},
                                        {"obstacle_reverse_max_dist_m", 0.30},
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
             {"footprint_padding", 0.01}})
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
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_enable_ros_time_override(costmap_->get_clock()->get_clock_handle()), RCL_RET_OK);
    SetTime();
    SetTransform("map", "odom", 0.0, true);
    SetTransform("base_footprint", "base_link", 0.0, true);
    SetTransform("odom", "base_footprint", 0.0, false);
    controller_.configure(node_, kPlugin, tf_, costmap_);
    controller_.activate();
    ASSERT_TRUE(mowgli_nav2_plugins::test::supplyLawn(
        node_,
        [this]()
        {
          return mowgli_nav2_plugins::FTCSpeedLimitTestAccess::GeometryReady(controller_);
        }));
    controller_.newPathReceived(StraightPlan());
  }

  void TearDown() override
  {
    controller_.deactivate();
    controller_.cleanup();
    costmap_->cleanup();
  }

  void SetParameter(const std::string& key, double value)
  {
    ASSERT_TRUE(node_->set_parameter({std::string(kPlugin) + "." + key, value}).successful);
  }

  void SetParameter(const std::string& key, bool value)
  {
    ASSERT_TRUE(node_->set_parameter({std::string(kPlugin) + "." + key, value}).successful);
  }

  double Tick()
  {
    time_ns_ += 100000000;
    SetTime();
    SetTransform("odom", "base_footprint", robot_x_, false);
    geometry_msgs::msg::Twist measured;
    measured.linear.x = measured_speed_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist.linear.x;
  }

  double PeakForward(int ticks = 30)
  {
    double peak = 0.0;
    for (int i = 0; i < ticks; ++i)
    {
      const double command = Tick();
      EXPECT_GE(command, 0.0);
      EXPECT_TRUE(std::isfinite(command));
      peak = std::max(peak, command);
    }
    return peak;
  }

  void Wall(double x)
  {
    auto* map = costmap_->getCostmap();
    for (double y = -1.5; y <= 2.0; y += 0.05)
    {
      unsigned int mx = 0;
      unsigned int my = 0;
      ASSERT_TRUE(map->worldToMap(x, y, mx, my));
      map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  void SetTime()
  {
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), time_ns_), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(costmap_->get_clock()->get_clock_handle(), time_ns_),
              RCL_RET_OK);
  }

  void SetTransform(const std::string& parent, const std::string& child, double x, bool fixed)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test", fixed));
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  mowgli_nav2_plugins::FTCController controller_;
  std::int64_t time_ns_{1000000000000};
  double robot_x_{0.0};
  double measured_speed_{0.20};
};

class FtcPercentageLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
};

TEST_P(FtcPercentageLimit, UsesPercentOfConfiguredMaximum)
{
  SetParameter("speed_fast", 0.80);
  controller_.setSpeedLimit(GetParam(), true);
  const double peak = PeakForward();
  EXPECT_NEAR(peak, 0.30 * GetParam() / 100.0, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(Nav2Percent, FtcPercentageLimit, ::testing::Values(5.0, 50.0, 100.0));

TEST_F(FtcSpeedLimit, SmallAbsoluteLimitWinsOverMinimumMovementSpeed)
{
  controller_.setSpeedLimit(0.05, false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.05 + 1e-9);
}

class FtcClearLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<bool>
{
};

TEST_P(FtcClearLimit, ZeroClearsAnExistingLimit)
{
  controller_.setSpeedLimit(0.05, false);
  PeakForward();
  controller_.setSpeedLimit(0.0, GetParam());
  EXPECT_GT(PeakForward(), 0.15);
}

INSTANTIATE_TEST_SUITE_P(Nav2Clear, FtcClearLimit, ::testing::Bool());

TEST_F(FtcSpeedLimit, AbsoluteLimitCannotRaiseConfiguredMaximum)
{
  SetParameter("speed_fast", 0.80);
  controller_.setSpeedLimit(1.0, false);
  EXPECT_LE(PeakForward(), 0.30 + 1e-9);
}

TEST_F(FtcSpeedLimit, DynamicMaximumCannotRemoveAbsoluteLimit)
{
  controller_.setSpeedLimit(0.05, false);
  SetParameter("max_cmd_vel_speed", 0.40);
  SetParameter("speed_fast", 0.80);
  EXPECT_LE(PeakForward(), 0.05 + 1e-9);
}

TEST_F(FtcSpeedLimit, ActivePercentageTracksDynamicMaximum)
{
  controller_.setSpeedLimit(50.0, true);
  SetParameter("max_cmd_vel_speed", 0.10);
  EXPECT_LE(PeakForward(), 0.05 + 1e-9);
}

TEST_F(FtcSpeedLimit, DynamicFastSpeedCannotLiftAnActivePercentageLimit)
{
  controller_.setSpeedLimit(5.0, true);
  SetParameter("speed_fast", 0.80);
  EXPECT_LE(PeakForward(), 0.015 + 1e-9);
}

TEST_F(FtcSpeedLimit, ClearRestoresTheLatestConfiguredMaximum)
{
  controller_.setSpeedLimit(0.05, false);
  SetParameter("max_cmd_vel_speed", 0.10);
  controller_.setSpeedLimit(0.0, false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.10 + 1e-9);
}

TEST_F(FtcSpeedLimit, LegacyNegativeClearStillRestoresNormalFollowing)
{
  controller_.setSpeedLimit(0.05, false);
  controller_.setSpeedLimit(-1.0, false);
  EXPECT_GT(PeakForward(), 0.15);
}

class FtcInvalidLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
};

TEST_P(FtcInvalidLimit, NonfiniteInputPreservesAnExistingExternalCap)
{
  controller_.setSpeedLimit(0.05, false);
  controller_.setSpeedLimit(GetParam(), false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.05 + 1e-9);
}

INSTANTIATE_TEST_SUITE_P(InvalidNav2Input,
                         FtcInvalidLimit,
                         ::testing::Values(std::numeric_limits<double>::quiet_NaN(),
                                           std::numeric_limits<double>::infinity(),
                                           -std::numeric_limits<double>::infinity()));

TEST_F(FtcSpeedLimit, HealthyMotionAtALowCapIsNotMistakenForAStall)
{
  SetParameter("stall_grace_s", 0.6);
  measured_speed_ = 0.05;
  controller_.setSpeedLimit(0.05, false);
  for (int i = 0; i < 60; ++i)
  {
    const double command = Tick();
    ASSERT_LE(command, 0.05 + 1e-9);
    ASSERT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Stalled(controller_));
    robot_x_ += command * 0.10;
  }
  EXPECT_GT(robot_x_, 0.20);
}

TEST_F(FtcSpeedLimit, ReverseEscapeObeysTheSignedExternalCap)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_x_ = 2.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  controller_.newPathReceived(StraightPlan(robot_x_));
  Wall(3.0);
  controller_.setSpeedLimit(0.05, false);
  double reverse = 0.0;
  for (int i = 0; i < 15; ++i)
  {
    const double command = Tick();
    EXPECT_LE(std::abs(command), 0.05 + 1e-9);
    reverse = std::min(reverse, command);
  }
  EXPECT_LT(reverse, 0.0);
}

TEST_F(FtcSpeedLimit, TurnFallbackReverseObeysTheSignedExternalCap)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  SetParameter("turn_fallback_enabled", true);
  robot_x_ = 2.85;
  SetTransform("odom", "base_footprint", robot_x_, false);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses.push_back(MakePose(2.85));
  plan.poses.push_back(MakePose(2.90));
  plan.poses.push_back(MakePose(2.95));
  const int steps = static_cast<int>(std::ceil(M_PI * 0.30 / 0.05));
  for (int i = 0; i <= steps; ++i)
  {
    const double angle = -M_PI / 2.0 + M_PI * i / steps;
    plan.poses.push_back(
        MakePose(3.0 + 0.30 * std::cos(angle), 0.30 + 0.30 * std::sin(angle), angle + M_PI / 2.0));
  }
  for (double x = 2.95; x >= 0.0; x -= 0.05)
  {
    plan.poses.push_back(MakePose(x, 0.60, M_PI));
  }
  controller_.newPathReceived(plan);
  Wall(3.45);
  controller_.setSpeedLimit(0.05, false);
  bool saw_fallback_reverse = false;
  for (int i = 0; i < 15; ++i)
  {
    const double command = Tick();
    if (command < 0.0)
    {
      ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_));
      saw_fallback_reverse = true;
      EXPECT_LE(std::abs(command), 0.05 + 1e-9);
      break;
    }
  }
  EXPECT_TRUE(saw_fallback_reverse);
}
}  // namespace
