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
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
struct FTCSpeedLimitTestAccess
{
  static int State(const FTCController& controller)
  {
    return static_cast<int>(controller.current_state_);
  }

  static std::size_t Index(const FTCController& controller)
  {
    return controller.current_index_;
  }

  static bool TurnFallbackReversing(const FTCController& controller)
  {
    return controller.turnFallbackReversing();
  }

  static bool Stalled(const FTCController& controller)
  {
    return controller.is_stalled_;
  }
  static void ApproachFixture(FTCController& controller, double body_distance, bool second = false)
  {
    controller.current_state_ = FTCController::PlannerState::FOLLOWING;
    controller.current_index_ =
        second ? controller.pivot_corners_.back() : controller.pivot_corners_.front();
    tf2::fromMsg(controller.global_plan_[controller.current_index_].pose,
                 controller.current_control_point_);
    controller.local_control_point_.setIdentity();
    controller.local_control_point_.translation().x() = body_distance;
  }

  static void ApproachStateTick(FTCController& controller)
  {
    (void)controller.update_planner_state();
  }

  static bool ApproachTimerArmed(const FTCController& controller)
  {
    return controller.pivot_approach_start_.has_value();
  }

  static bool SpliceApproachFallback(FTCController& controller)
  {
    TurnFallbackPlan plan;
    plan.rejoin = 21;
    plan.connector_m = 0.925;
    return controller.spliceTurnFallback(plan, 0);
  }

  static void LeaveFallbackInitialPivot(FTCController& controller)
  {
    controller.current_state_ = FTCController::PlannerState::FOLLOWING;
    controller.current_index_ = 1;
    tf2::fromMsg(controller.global_plan_[1].pose, controller.current_control_point_);
    controller.local_control_point_.setIdentity();
    controller.local_control_point_.translation().x() = 0.925;
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
    SetTime();
    SetTransform("map", "odom", 0.0, true);
    SetTransform("base_footprint", "base_link", 0.0, true);
    SetTransform("odom", "base_footprint", 0.0, false);
    controller_.configure(node_, kPlugin, tf_, costmap_);
    controller_.activate();
    StartPlan(StraightPlan());
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

  void StartPlan(const nav_msgs::msg::Path& plan)
  {
    controller_.newPathReceived(plan);
    // Begin these command-bound tests at rest before injecting their isolated
    // moving-odom feedback. PRE_ROTATE now correctly waits for translation.
    controller_.computeVelocityCommands(
        Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{});
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
  StartPlan(StraightPlan(robot_x_));
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
  StartPlan(plan);
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

// Public compute loop with true plant feedback. Controller: 10 Hz; numerical
// integration: 50 Hz. Ideal and tau=0.07 s responses are declared test plants,
// not robot calibration. Private access observes state only.
class FtcPivotArrival : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
protected:
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;

  void RobotTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation.z = std::sin(yaw_ / 2);
    transform.transform.rotation.w = std::cos(yaw_ / 2);
    ASSERT_TRUE(tf_->setTransform(transform, "arrival-plant", false));
  }

  nav_msgs::msg::Path CornerPlan(double sign = 1.0, double spacing = 0.6)
  {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i <= 20; ++i)
      plan.poses.push_back(MakePose(0.05 * i));
    plan.poses.push_back(MakePose(1, 0, sign * M_PI / 2));
    plan.poses.push_back(MakePose(1, sign * spacing, sign * M_PI / 2));
    plan.poses.push_back(MakePose(1, sign * spacing, sign * M_PI));
    for (int i = 1; i <= 20; ++i)
      plan.poses.push_back(MakePose(1 - 0.05 * i, sign * spacing, sign * M_PI));
    return plan;
  }

  geometry_msgs::msg::Twist Command()
  {
    time_ns_ += 100000000;
    SetTime();
    RobotTransform();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = v_;
    measured.angular.z = w_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist;
  }

  void Advance(const geometry_msgs::msg::Twist& command, double tau)
  {
    for (int i = 0; i < 5; ++i)
    {
      const double a = tau == 0 ? 0 : std::exp(-0.02 / tau);
      const double ah = tau == 0 ? 0 : std::exp(-0.01 / tau);
      const double v = command.linear.x + (v_ - command.linear.x) * ah;
      const double w = command.angular.z + (w_ - command.angular.z) * ah;
      const double delta = w * 0.02;
      const double sinc = std::abs(delta) < 1e-12 ? 1 : std::sin(delta / 2) / (delta / 2);
      x_ += v * 0.02 * sinc * std::cos(yaw_ + delta / 2);
      y_ += v * 0.02 * sinc * std::sin(yaw_ + delta / 2);
      yaw_ += delta;
      v_ = command.linear.x + (v_ - command.linear.x) * a;
      w_ = command.angular.z + (w_ - command.angular.z) * a;
    }
  }

  void RunArrival(
      double sign, double angle = 0, double spacing = 0.6, std::size_t trim = 0, double cap = .3)
  {
    auto plan = CornerPlan(sign, spacing);
    // The captured first tip/start from the retained ring replay are tested
    // separately outside the repository. This reachable approach also starts
    // 125 mm off line and exercises the real point-bearing correction.
    x_ = 0;
    y_ = -sign * 0.125;
    yaw_ = 0;
    if (trim)
    {
      x_ = plan.poses[trim].pose.position.x;
      y_ = plan.poses[trim].pose.position.y;
      if (trim < 20)
        y_ -= sign * .125;  // interrupted incoming leg retains a real lateral error
      yaw_ =
          2 * std::atan2(plan.poses[trim].pose.orientation.z, plan.poses[trim].pose.orientation.w);
      plan.poses.erase(plan.poses.begin(), plan.poses.begin() + trim);
    }
    const auto transform = [angle](double& x, double& y)
    {
      const double ox = x;
      x = std::cos(angle) * x - std::sin(angle) * y + 0.3;
      y = std::sin(angle) * ox + std::cos(angle) * y - 0.4;
    };
    transform(x_, y_);
    yaw_ += angle;
    for (auto& p : plan.poses)
    {
      transform(p.pose.position.x, p.pose.position.y);
      const double yaw = 2 * std::atan2(p.pose.orientation.z, p.pose.orientation.w) + angle;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
    }
    double gx = 1, gy = 0;
    transform(gx, gy);
    RobotTransform();
    controller_.newPathReceived(plan);
    std::ofstream trace;
    if (const char* dir = std::getenv("FTC_ARRIVAL_TRACE_DIR"))
    {
      const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
      std::string name = info->name();
      std::replace(name.begin(), name.end(), '/', '-');
      trace.open(std::string(dir) + "/" + name + ".csv");
      trace << "t,x,y,yaw,v,w,cmd_v,cmd_w,state,index,distance\n" << std::setprecision(17);
    }
    bool braked = false, pivoted = trim >= 20, left = false;
    int completed = 0;
    for (int tick = 0; tick < 700; ++tick)
    {
      const int before = Access::State(controller_);
      const auto command = Command();
      const int state = Access::State(controller_);
      EXPECT_LE(command.linear.x, cap);
      const double distance = std::hypot(x_ - gx, y_ - gy);
      if (trace)
        trace << tick * .1 << ',' << x_ << ',' << y_ << ',' << yaw_ << ',' << v_ << ',' << w_ << ','
              << command.linear.x << ',' << command.angular.z << ',' << state << ','
              << Access::Index(controller_) << ',' << distance << '\n';
      if (!pivoted && state == 1 && command.linear.x > 0 && command.linear.x < .15)
        braked = true;
      if (!pivoted && state == 5)
      {
        EXPECT_LE(distance, 0.02);
        pivoted = true;
      }
      if ((state == 0 || state == 5) && std::abs(v_) > .01)
        EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
      if (pivoted && before == 5 && state == 1)
      {
        ++completed;
        if (completed == (trim >= 20 ? 1 : 2))
        {
          left = true;
          break;
        }
        pivoted = false;
        gx = 1;
        gy = sign * spacing;
        transform(gx, gy);
      }
      Advance(command, GetParam());
    }
    if (trim < 20)
      EXPECT_TRUE(braked);
    EXPECT_TRUE(pivoted);
    EXPECT_TRUE(left);
  }

  double x_{0}, y_{0}, yaw_{0}, v_{0}, w_{0};
};

TEST_P(FtcPivotArrival, ApproachTimerUsesBodyAndCannotBeResetByRetreat)
{
  SetParameter("goal_timeout", 10.0);
  controller_.newPathReceived(CornerPlan());
  Access::ApproachFixture(controller_, .41);  // carrot is at the corner
  Access::ApproachStateTick(controller_);
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 11000000000LL;
  SetTime();
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  Access::ApproachFixture(controller_, .23);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 6000000000LL;
  SetTime();
  Access::ApproachFixture(controller_, .41);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  time_ns_ += 4100000000LL;
  SetTime();
  EXPECT_THROW(Access::ApproachStateTick(controller_), nav2_core::ControllerException);
  Access::ApproachFixture(controller_, .41, true);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  Access::ApproachFixture(controller_, .23, true);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  controller_.newPathReceived(CornerPlan());
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, FallbackReplacementCannotInheritSameIndexApproachDeadline)
{
  SetParameter("goal_timeout", 10.0);
  auto plan = CornerPlan();
  for (auto& pose : plan.poses)
    pose.pose.position.x *= .925;
  yaw_ = -M_PI / 2;
  RobotTransform();
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, .23);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 9000000000LL;
  SetTime();
  ASSERT_TRUE(Access::SpliceApproachFallback(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 2000000000LL;
  SetTime();
  Access::LeaveFallbackInitialPivot(controller_);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, UnsupportedFrameAndExternalCapKeepFiniteBudgets)
{
  SetParameter("goal_timeout", 10.0);
  auto plan = CornerPlan();
  plan.header.frame_id = "odom";
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, .41);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 10100000000LL;
  SetTime();
  EXPECT_THROW(Access::ApproachStateTick(controller_), nav2_core::ControllerException);

  controller_.newPathReceived(CornerPlan());
  controller_.setSpeedLimit(.1, false);  // window is .08 m at this cap
  Access::ApproachFixture(controller_, .1);
  Access::ApproachStateTick(controller_);
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  Access::ApproachFixture(controller_, .07);
  Access::ApproachStateTick(controller_);
  EXPECT_TRUE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, BrakesAndReachesRadialTipLeft)
{
  RunArrival(1);
}
TEST_P(FtcPivotArrival, BrakesAndReachesRadialTipRight)
{
  RunArrival(-1);
}
TEST_P(FtcPivotArrival, RigidTransformPreservesArrival)
{
  RunArrival(1, .7);
}
TEST_P(FtcPivotArrival, CloseConsecutiveCorners)
{
  RunArrival(1, 0, .00367);
}
TEST_P(FtcPivotArrival, ResumedIncomingTwin)
{
  RunArrival(1, 0, .6, 20);
}
TEST_P(FtcPivotArrival, ResumedOutgoingTwin)
{
  RunArrival(1, 0, .6, 21);
}

TEST_P(FtcPivotArrival, TranslationHoldsPreRotateUntilStopped)
{
  auto plan = CornerPlan();
  controller_.newPathReceived(plan);
  v_ = .15;
  yaw_ = -.5;
  const auto command = Command();
  EXPECT_DOUBLE_EQ(command.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0);
  EXPECT_EQ(Access::State(controller_), 0);
}

TEST_P(FtcPivotArrival, InterruptedIncomingApproach)
{
  // New path begins halfway along the incoming leg with an actual lateral
  // error; PRE_ROTATE must align with the tip before handing back to FOLLOWING.
  RunArrival(1, 0, .6, 18);
}

TEST_P(FtcPivotArrival, ExternalCapWinsDuringApproach)
{
  controller_.setSpeedLimit(.03, false);
  RunArrival(1, 0, .6, 18, .03);
}

TEST_P(FtcPivotArrival, SettlingCannotResetPreRotateDeadline)
{
  SetParameter("goal_timeout", .3);
  controller_.newPathReceived(CornerPlan());
  v_ = .15;
  for (int i = 0; i < 3; ++i)
  {
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.linear.x, 0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
  }
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, StationaryLateralMissNeverCountsAsArrivalAndTimesOut)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 18);
  x_ = 1;
  y_ = -.125;
  yaw_ = M_PI / 2;
  RobotTransform();
  controller_.newPathReceived(plan);
  bool threw = false;
  for (int i = 0; i < 350; ++i)
  {
    try
    {
      Command();  // blocked plant: measured zero, pose unchanged
      EXPECT_NE(Access::State(controller_), 5);
    }
    catch (const nav2_core::ControllerException&)
    {
      threw = true;
      break;
    }
  }
  EXPECT_TRUE(threw);
}

TEST_P(FtcPivotArrival, MissingAndStaleTfRefuseApproach)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 16);
  x_ = .8;
  RobotTransform();
  controller_.newPathReceived(plan);
  time_ns_ += 1000000000;
  SetTime();
  EXPECT_THROW(controller_.computeVelocityCommands(
                   Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{}),
               nav2_core::ControllerException);
  tf_->clear();
  EXPECT_THROW(controller_.computeVelocityCommands(
                   Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{}),
               nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, WrongPathFrameRefusesPivot)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 20);
  plan.header.frame_id = "odom";
  for (auto& p : plan.poses)
    p.header.frame_id = "odom";
  controller_.newPathReceived(plan);
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, FutureTfDoesNotForecastTheApproachCap)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 18);
  x_ = .96;
  RobotTransform();
  controller_.newPathReceived(plan);
  controller_.computeVelocityCommands(
      Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{});
  time_ns_ += 100000000;
  SetTime();
  RobotTransform();
  const auto now = time_ns_;
  time_ns_ += 100000000;
  x_ = .999;
  RobotTransform();
  time_ns_ = now;
  SetTime();
  const auto command = controller_
                           .computeVelocityCommands(Pose{},
                                                    geometry_msgs::msg::Twist{},
                                                    nullptr,
                                                    nav_msgs::msg::Path{},
                                                    Pose{})
                           .twist;
  // Current distance .04 / declared .8 s horizon, rather than future .001.
  EXPECT_NEAR(command.linear.x, .05, 1e-12);
}

TEST_P(FtcPivotArrival, TranslationSettlingRetainsPivotDeadline)
{
  controller_.newPathReceived(CornerPlan());
  bool pivoted = false;
  for (int i = 0; i < 300; ++i)
  {
    const auto command = Command();
    if (Access::State(controller_) == 5)
    {
      pivoted = true;
      break;
    }
    Advance(command, GetParam());
  }
  ASSERT_TRUE(pivoted);
  SetParameter("goal_timeout", .3);
  v_ = .15;  // persistent measured translation; rotation must remain inhibited
  for (int i = 0; i < 2; ++i)
  {
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.linear.x, 0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
    EXPECT_EQ(Access::State(controller_), 5);
  }
  Command();
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, ObstacleOverlayRetainsArrivalAndSettling)
{
  SetParameter("enable_obstacle_deviation", true);
  RunArrival(1);
}

TEST_F(FtcSpeedLimit, InvalidStartupPivotCoastAssumptionIsRejected)
{
  auto invalid_node = std::make_shared<nav2::LifecycleNode>("invalid_controller", "");
  invalid_node->declare_parameter(std::string(kPlugin) + ".pivot_coast_time_bound_s", 0.0);
  mowgli_nav2_plugins::FTCController invalid;
  EXPECT_THROW(invalid.configure(invalid_node, kPlugin, tf_, costmap_),
               nav2_core::ControllerException);
}

TEST_F(FtcSpeedLimit, PivotCoastAssumptionHasValidatedDynamicRange)
{
  const std::string name = std::string(kPlugin) + ".pivot_coast_time_bound_s";
  EXPECT_DOUBLE_EQ(node_->get_parameter(name).as_double(), .5);
  for (double invalid : {0.0,
                         -.1,
                         2.01,
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(node_->set_parameter({name, invalid}).successful);
  for (double valid : {.01, .5, 2.0})
    EXPECT_TRUE(node_->set_parameter({name, valid}).successful);
}

INSTANTIATE_TEST_SUITE_P(DeclaredPlants, FtcPivotArrival, ::testing::Values(0.0, 0.07));
}  // namespace
