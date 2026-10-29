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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
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
  static bool BoundaryReady(FTCController& controller)
  {
    std::lock_guard<std::mutex> lock(controller.boundary_mutex_);
    return controller.boundary_costmap_ != nullptr;
  }
  static bool TurnFallbackReversing(const FTCController& controller)
  {
    return controller.turnFallbackReversing();
  }
  static double TurnFallbackReverseTarget(const FTCController& controller)
  {
    return controller.turn_fallback_.reverse_target_m;
  }
  static bool TurnFallbackRunning(const FTCController& controller)
  {
    return controller.turn_fallback_.phase == FTCController::TurnFallbackPhase::kRunning;
  }
  static double WorkingPlanStartX(const FTCController& controller)
  {
    return controller.global_plan_.front().pose.position.x;
  }
  static void SetCurrentRosElapsed(FTCController& controller, double ros_dt)
  {
    // Inject only the stored prior ROS instant. The node clock and the fresh
    // TF stamps stay unchanged, isolating one shortened accounting interval.
    controller.last_time_ = controller.clock_->now() - rclcpp::Duration::from_seconds(ros_dt);
  }

  static bool Stalled(const FTCController& controller)
  {
    return controller.is_stalled_;
  }
  static bool ApproachingGoal(const FTCController& controller)
  {
    return controller.current_state_ == FTCController::PlannerState::WAITING_FOR_GOAL_APPROACH;
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

  double Tick(double elapsed_s = 0.10, nav2_core::GoalChecker* goal_checker = nullptr)
  {
    time_ns_ += static_cast<int64_t>(std::llround(elapsed_s * 1e9));
    SetTime();
    SetTransform("odom", "base_footprint", robot_x_, false);
    geometry_msgs::msg::Twist measured;
    measured.linear.x = measured_speed_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, goal_checker, nav_msgs::msg::Path{}, Pose{})
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

  void SetTransform(const std::string& parent,
                    const std::string& child,
                    double x,
                    bool fixed,
                    double z = 0.0,
                    double yaw = 0.0)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.translation.z = z;
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
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

class RequestedGoalTolerance : public nav2_core::GoalChecker
{
public:
  double x{0.03};
  double y{0.03};
  bool available{true};
  unsigned int queries{0};

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
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Twist&, double&) override
  {
    ++queries;
    pose.position.x = x;
    pose.position.y = y;
    return available;
  }
};

class FtcRequestedGoalTolerance : public FtcSpeedLimit
{
protected:
  void WaitAtFinalCarrot(double robot_x, RequestedGoalTolerance& checker)
  {
    robot_x_ = robot_x;
    measured_speed_ = 0.0;
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i <= 16; ++i)
      plan.poses.push_back(MakePose(0.05 * i));
    controller_.newPathReceived(plan);
    for (int i = 0; i < 100; ++i)
    {
      Tick(0.10, &checker);
      if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ApproachingGoal(controller_))
        return;
    }
    FAIL() << "Controller did not enter its timed terminal approach";
  }

  double FollowShortPath(nav2_core::GoalChecker* goal_checker,
                         std::vector<double>* commands = nullptr,
                         double heading = 0.0)
  {
    robot_x_ = 0.0;
    measured_speed_ = 0.0;
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i <= 16; ++i)
      plan.poses.push_back(
          MakePose(0.05 * i * std::cos(heading), 0.05 * i * std::sin(heading), heading));
    controller_.newPathReceived(plan);
    for (int i = 0; i < 100; ++i)
    {
      const double command = Tick(0.10, goal_checker);
      if (commands != nullptr)
        commands->push_back(command);
      EXPECT_GE(command, 0.0);
      EXPECT_LE(command, 0.20);
      robot_x_ += command * 0.10;
      measured_speed_ = command;
    }
    return robot_x_;
  }
};

TEST_F(FtcRequestedGoalTolerance, StricterCheckerKeepsFollowingUntilRequestedPosition)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  const double reached = FollowShortPath(&checker);
  EXPECT_GT(checker.queries, 0u);
  EXPECT_GE(reached, 0.80 - checker.x);
  EXPECT_LE(reached, 0.80 + 0.02);
  EXPECT_DOUBLE_EQ(Tick(0.10, &checker), 0.0);
}

TEST_F(FtcRequestedGoalTolerance, StrictXYApproachIgnoresBaseLinkHeightAtRotatedHeading)
{
  SetParameter("max_goal_distance_error", 0.50);
  SetParameter("goal_timeout", 1.0);
  // Shipped Webots base_footprint->base_link height: the path remains at z=0.
  SetTransform("base_footprint", "base_link", 0.0, true, 0.093);
  constexpr double heading = 0.60;
  SetTransform("map", "odom", 0.0, true, 0.0, heading);
  RequestedGoalTolerance checker;
  double reached = 0.0;
  // The actual outgoing commands move odom's X axis; the map-frame path is
  // rotated with that axis. A 3D terminal comparison can never finish and
  // eventually throws at goal_timeout even once the XY position is reached.
  ASSERT_NO_THROW(reached = FollowShortPath(&checker, nullptr, heading));
  EXPECT_GE(reached, 0.80 - checker.x);
  EXPECT_LE(reached, 0.80 + 0.02);
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ApproachingGoal(controller_));
  EXPECT_DOUBLE_EQ(Tick(0.10, &checker), 0.0);
}

TEST_F(FtcRequestedGoalTolerance, DefaultLooseAndInvalidCheckersKeepConfiguredApproach)
{
  SetParameter("max_goal_distance_error", 0.50);
  std::vector<double> baseline_commands;
  const double baseline = FollowShortPath(nullptr, &baseline_commands);
  EXPECT_LT(baseline, 0.80 - 0.03);
  RequestedGoalTolerance checker;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{0.50, 0.50},
                                              {0.80, 0.80},
                                              {std::numeric_limits<double>::quiet_NaN(), 0.03},
                                              {0.03, std::numeric_limits<double>::infinity()},
                                              {-1.0, 0.03},
                                              {0.03, 0.0}})
  {
    checker.x = xy.first;
    checker.y = xy.second;
    std::vector<double> commands;
    EXPECT_DOUBLE_EQ(FollowShortPath(&checker, &commands), baseline);
    EXPECT_EQ(commands, baseline_commands);
  }
  checker.x = checker.y = 0.03;
  checker.available = false;
  std::vector<double> commands;
  EXPECT_DOUBLE_EQ(FollowShortPath(&checker, &commands), baseline);
  EXPECT_EQ(commands, baseline_commands);
}

class FtcUnreachableRequestedGoal : public FtcRequestedGoalTolerance,
                                    public ::testing::WithParamInterface<double>
{
};

TEST_P(FtcUnreachableRequestedGoal, StrictApproachRetainsExistingTimeout)
{
  SetParameter("max_goal_distance_error", 0.50);
  SetParameter("goal_timeout", 0.30);
  robot_x_ = GetParam();
  measured_speed_ = 0.0;
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    plan.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(plan);
  RequestedGoalTolerance checker;
  bool aborted = false;
  double peak = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    try
    {
      // Hold the actual robot pose: one case cannot reach the precise goal;
      // the other has overshot it and forward_only forbids backing up.
      const double command = Tick(0.10, &checker);
      EXPECT_GE(command, 0.0);
      peak = std::max(peak, command);
    }
    catch (const nav2_core::ControllerException&)
    {
      aborted = true;
      break;
    }
  }
  EXPECT_TRUE(aborted) << "Strict terminal approach must retain FTC's goal_timeout";
  if (robot_x_ > 0.80)
    EXPECT_DOUBLE_EQ(peak, 0.0);
  else
    EXPECT_GT(peak, 0.0);
}

INSTANTIATE_TEST_SUITE_P(StalledOrOvershot,
                         FtcUnreachableRequestedGoal,
                         ::testing::Values(0.74, 0.84));

TEST_F(FtcRequestedGoalTolerance, StrictApproachResumesAfterTransientObstacleClears)
{
  SetParameter("max_goal_distance_error", 0.50);
  SetParameter("enable_obstacle_deviation", true);
  SetParameter("obstacle_reverse_enabled", false);
  RequestedGoalTolerance checker;
  ASSERT_NO_FATAL_FAILURE(WaitAtFinalCarrot(0.74, checker));
  Wall(1.20);
  EXPECT_DOUBLE_EQ(Tick(0.10, &checker), 0.0);
  EXPECT_DOUBLE_EQ(Tick(0.10, &checker), 0.0);
  costmap_->getCostmap()->resetMap(0, 0, 400, 400);
  double resumed = 0.0;
  for (int i = 0; i < 25; ++i)
    resumed = std::max(resumed, Tick(0.10, &checker));
  EXPECT_GT(resumed, 0.0) << "A cleared hold must restore terminal approach motion";
}

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

class FtcReverseBudget
    : public FtcSpeedLimit,
      public ::testing::WithParamInterface<std::tuple<double, double, bool, double>>
{
};

TEST_P(FtcReverseBudget, ReverseEscapeDoesNotCommandPastItsRemainingDistanceBudget)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  const double budget = std::get<3>(GetParam());
  SetParameter("obstacle_reverse_max_dist_m", budget);
  robot_x_ = 2.0;
  measured_speed_ = 0.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  controller_.newPathReceived(StraightPlan(robot_x_));
  Wall(3.0);

  double reverse_distance = 0.0;
  bool reversed = false;
  bool stopped = false;
  double elapsed_s = 0.10;
  for (int i = 0; i < 128; ++i)
  {
    const double hold_s = std::get<2>(GetParam()) && i % 2 != 0 ? 0.10 : std::get<1>(GetParam());
    const double command = Tick(elapsed_s);
    elapsed_s = hold_s;
    if (command < 0.0)
    {
      reversed = true;
      reverse_distance -= command * hold_s;
      robot_x_ += command * hold_s;
      // Hold the command for a supported interval with ideal actuation.
      // Check both accurate and under-reported odometry; a distance budget
      // must not permit unbounded commanded reversing when feedback freezes.
      measured_speed_ = command * std::get<0>(GetParam());
      EXPECT_LE(reverse_distance, budget)
          << "The next held command must fit the remaining reverse budget";
    }
    else if (reversed)
    {
      stopped = true;
      break;
    }
  }
  EXPECT_TRUE(reversed);
  EXPECT_TRUE(stopped);
  // Conservative pre-taper charging intentionally hands off before using all
  // physical travel. Preserve useful progress, including >80% at shipped scale.
  EXPECT_GT(reverse_distance, budget * (budget > 0.10 ? 0.80 : 0.50));
}

INSTANTIATE_TEST_SUITE_P(ReverseFeedback,
                         FtcReverseBudget,
                         ::testing::Values(std::tuple{1.0, 0.05, false, 0.100},
                                           std::tuple{1.0, 0.10, false, 0.100},
                                           std::tuple{1.0, 0.20, false, 0.100},
                                           std::tuple{1.0, 0.50, false, 0.100},
                                           std::tuple{1.0, 0.05, true, 0.100},
                                           std::tuple{0.0, 0.05, false, 0.100},
                                           std::tuple{0.0, 0.10, false, 0.100},
                                           std::tuple{0.0, 0.20, false, 0.100},
                                           std::tuple{0.0, 0.50, false, 0.100},
                                           std::tuple{0.0, 0.05, true, 0.100},
                                           std::tuple{1.0, 0.05, false, 0.295},
                                           std::tuple{1.0, 0.10, false, 0.295},
                                           std::tuple{1.0, 0.20, false, 0.295},
                                           std::tuple{1.0, 0.50, false, 0.295},
                                           std::tuple{1.0, 0.05, true, 0.295},
                                           std::tuple{0.0, 0.05, false, 0.295},
                                           std::tuple{0.0, 0.10, false, 0.295},
                                           std::tuple{0.0, 0.20, false, 0.295},
                                           std::tuple{0.0, 0.50, false, 0.295},
                                           std::tuple{0.0, 0.05, true, 0.295}));

class FtcReverseBoundary : public FtcSpeedLimit
{
protected:
  void SetUp() override
  {
    FtcSpeedLimit::SetUp();
    fixture_ = std::make_shared<rclcpp::Node>("reverse_boundary_fixture");
    boundary_pub_ =
        fixture_->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",
                                                                 rclcpp::QoS(1).transient_local());
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(fixture_);
    SetParameter("check_obstacles", true);
    SetParameter("enable_obstacle_deviation", true);
    SetParameter("confine_deviation_to_zone", true);
    SetParameter("obstacle_reverse_max_dist_m", 0.10);
    robot_x_ = 2.0;
    measured_speed_ = 0.0;
    SetTransform("odom", "base_footprint", robot_x_, false);
    controller_.newPathReceived(StraightPlan(robot_x_));
    Wall(3.0);
    PublishBoundary(true);
  }

  void PublishBoundary(bool valid)
  {
    nav_msgs::msg::OccupancyGrid message;
    if (valid)
    {
      message.header.frame_id = "map";
      message.info.width = 400;
      message.info.height = 400;
      message.info.resolution = 0.05;
      message.info.origin.position.x = -5.0;
      message.info.origin.position.y = -5.0;
      message.info.origin.orientation.w = 1.0;
      message.data.assign(160000, 0);
    }
    boundary_pub_->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::BoundaryReady(controller_) == valid)
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    FAIL() << "Boundary snapshot was not delivered through the production topic";
  }

  double EngageReverse()
  {
    for (int i = 0; i < 30; ++i)
    {
      const double command = Tick();
      if (command < 0.0)
      {
        return command;
      }
    }
    ADD_FAILURE() << "Expected legal reverse escape before invalidating boundary";
    return 0.0;
  }

  std::shared_ptr<rclcpp::Node> fixture_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr boundary_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(FtcReverseBoundary, BoundaryLossHoldsAndRestoringItDoesNotRefillReverseBudget)
{
  double command = EngageReverse();
  ASSERT_LT(command, 0.0);
  double reverse_distance = -command * 0.10;
  robot_x_ += command * 0.10;
  PublishBoundary(false);
  ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::GeometryReady(controller_));
  for (int i = 0; i < 5; ++i)
  {
    command = Tick();
    EXPECT_DOUBLE_EQ(command, 0.0) << "A missing required guard must not reuse an old rear check";
    reverse_distance -= std::min(0.0, command) * 0.10;
    robot_x_ += command * 0.10;
  }
  PublishBoundary(true);
  bool resumed = false;
  bool stopped = false;
  for (int i = 0; i < 32; ++i)
  {
    command = Tick();
    if (command < 0.0)
    {
      resumed = true;
      reverse_distance -= command * 0.10;
      robot_x_ += command * 0.10;
    }
    else if (resumed)
    {
      stopped = true;
      break;
    }
  }
  EXPECT_TRUE(resumed);
  EXPECT_TRUE(stopped);
  EXPECT_GT(reverse_distance, 0.05);
  EXPECT_LE(reverse_distance, 0.10) << "Boundary loss/restoration must not refill the budget";
}

TEST_F(FtcReverseBoundary, MissingRequiredBoundaryKeepsTheExistingAbortDeadline)
{
  SetParameter("obstacle_wait_timeout_s", 0.30);
  ASSERT_LT(EngageReverse(), 0.0);
  PublishBoundary(false);
  EXPECT_DOUBLE_EQ(Tick(), 0.0);
  EXPECT_DOUBLE_EQ(Tick(), 0.0);
  EXPECT_DOUBLE_EQ(Tick(), 0.0);
  EXPECT_THROW(Tick(0.20), nav2_core::ControllerException);
}

TEST_F(FtcReverseBoundary, BoundaryLossAfterExhaustionDoesNotFreezeTheAbortDeadline)
{
  SetParameter("obstacle_wait_timeout_s", 0.30);
  ASSERT_LT(EngageReverse(), 0.0);
  bool held_after_reverse = false;
  for (int i = 0; i < 32; ++i)
  {
    if (Tick() == 0.0)
    {
      held_after_reverse = true;
      break;
    }
  }
  ASSERT_TRUE(held_after_reverse);
  PublishBoundary(false);
  EXPECT_THROW(Tick(0.50), nav2_core::ControllerException);
}

TEST_F(FtcReverseBoundary, RestoredBoundaryAndClearPathReleaseTheReverseHold)
{
  ASSERT_LT(EngageReverse(), 0.0);
  PublishBoundary(false);
  EXPECT_DOUBLE_EQ(Tick(), 0.0);
  costmap_->getCostmap()->resetMap(0, 0, 400, 400);
  PublishBoundary(true);
  bool followed_forward = false;
  for (int i = 0; i < 40; ++i)
  {
    if (Tick() > 0.0)
    {
      followed_forward = true;
      break;
    }
  }
  EXPECT_TRUE(followed_forward);
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

class FtcTurnFallbackBudget : public FtcSpeedLimit,
                              public ::testing::WithParamInterface<std::tuple<double, double>>
{
};

TEST_P(FtcTurnFallbackBudget, ReverseFitsItsTargetAndReplansFromTheActualRetreat)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  SetParameter("turn_fallback_enabled", true);
  robot_x_ = 2.85;
  measured_speed_ = 0.0;
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

  double command = 0.0;
  for (int i = 0; i < 30 && command >= 0.0; ++i)
  {
    command = Tick();
  }
  ASSERT_LT(command, 0.0) << "The production lattice must engage a legal fallback reverse";
  ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_));
  const double target =
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReverseTarget(controller_);
  ASSERT_GT(target, 0.0);
  ASSERT_LE(target, 0.40);

  const double feedback_scale = std::get<0>(GetParam());
  const double hold_s = std::get<1>(GetParam());
  double distance = 0.0;
  bool stopped = false;
  for (int i = 0; i < 128; ++i)
  {
    ASSERT_LT(command, 0.0);
    distance -= command * hold_s;
    robot_x_ += command * hold_s;
    measured_speed_ = command * feedback_scale;
    EXPECT_LE(distance, target) << "The next held fallback command must fit its selected target";
    EXPECT_LE(distance, 0.40) << "Fallback execution must retain the configured reverse ceiling";
    command = Tick(hold_s);
    if (!mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_))
    {
      stopped = true;
      EXPECT_DOUBLE_EQ(command, 0.0) << "Replanning must stop before the new pivot starts";
      break;
    }
  }
  EXPECT_TRUE(stopped);
  EXPECT_GT(distance, target * 0.80) << "Conservative accounting must preserve necessary retreat";
  ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackRunning(controller_))
      << "The safe retreat must replan the pivot/connector rather than abandon recovery";
  EXPECT_NEAR(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::WorkingPlanStartX(controller_),
              robot_x_,
              1e-9)
      << "The restored working path must start at the actual retreated pose";
}

INSTANTIATE_TEST_SUITE_P(FallbackFeedback,
                         FtcTurnFallbackBudget,
                         ::testing::Values(std::tuple{0.0, 0.10},
                                           std::tuple{0.5, 0.10},
                                           std::tuple{1.0, 0.10},
                                           std::tuple{0.0, 0.50},
                                           std::tuple{0.5, 0.50},
                                           std::tuple{1.0, 0.50}));

// These controls measure actual command-hold time independently of the ROS
// clock. No OS clock changes, stale TF or altered authorization are involved.
class FtcReverseClockBudget : public FtcSpeedLimit,
                              public ::testing::WithParamInterface<std::tuple<double, int>>
{
protected:
  void SetUp() override
  {
    FtcSpeedLimit::SetUp();
    // Exercise physical/system-time mode, not a plant driven by simulated time.
    // Restore fresh TF after switching off the original fixture overrides.
    ASSERT_EQ(rcl_disable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_disable_ros_time_override(costmap_->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_FALSE(clock_->ros_time_is_active());
    ASSERT_FALSE(costmap_->get_clock()->ros_time_is_active());
    time_ns_ = clock_->now().nanoseconds();
    SetTransform("map", "odom", 0.0, true);
    SetTransform("base_footprint", "base_link", 0.0, true);
    SetTransform("odom", "base_footprint", robot_x_, false);
    controller_.newPathReceived(StraightPlan());
  }

  double Tick()
  {
    time_ns_ = clock_->now().nanoseconds();
    SetTransform("odom", "base_footprint", robot_x_, false);
    geometry_msgs::msg::Twist measured;
    measured.linear.x = measured_speed_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist.linear.x;
  }

  double HoldCommand()
  {
    const auto started = std::chrono::steady_clock::now();
    std::this_thread::sleep_until(started + std::chrono::milliseconds(100));
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  }

  void InjectElapsed()
  {
    const int clock_case = std::get<1>(GetParam());
    if (clock_case != 0)
    {
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetCurrentRosElapsed(controller_,
                                                                         clock_case == 1 ? -0.10
                                                                                         : 0.001);
    }
  }

  double EngageFallback()
  {
    SetParameter("check_obstacles", true);
    SetParameter("enable_obstacle_deviation", true);
    SetParameter("turn_fallback_enabled", true);
    robot_x_ = 2.85;
    measured_speed_ = 0.0;
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
      plan.poses.push_back(MakePose(3.0 + 0.30 * std::cos(angle),
                                    0.30 + 0.30 * std::sin(angle),
                                    angle + M_PI / 2.0));
    }
    for (double x = 2.95; x >= 0.0; x -= 0.05)
    {
      plan.poses.push_back(MakePose(x, 0.60, M_PI));
    }
    controller_.newPathReceived(plan);
    Wall(3.45);
    double command = 0.0;
    for (int i = 0; i < 30 && command >= 0.0; ++i)
    {
      command = Tick();
    }
    return command;
  }
};

TEST_P(FtcReverseClockBudget, OneShortRosIntervalCannotRefundFallbackCommandTravel)
{
  double command = EngageFallback();
  ASSERT_LT(command, 0.0);
  ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_));
  const double target =
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReverseTarget(controller_);
  ASSERT_GT(target, 0.0);
  ASSERT_LE(target, 0.40);
  double distance = 0.0;
  bool stopped = false;
  for (int i = 0; i < 128; ++i)
  {
    ASSERT_LT(command, 0.0);
    const double hold_s = HoldCommand();
    ASSERT_LE(hold_s, 0.50) << "Clock control requires a supported actual hold interval";
    distance -= command * hold_s;
    robot_x_ += command * hold_s;
    measured_speed_ = command * std::get<0>(GetParam());
    EXPECT_LE(distance, target) << "A ROS clock step cannot refund held fallback travel";
    EXPECT_LE(distance, 0.40) << "Fallback execution must retain the configured reverse ceiling";
    if (i == 0)
    {
      InjectElapsed();
    }
    command = Tick();
    if (!mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_))
    {
      stopped = true;
      EXPECT_DOUBLE_EQ(command, 0.0);
      break;
    }
  }
  EXPECT_TRUE(stopped);
  EXPECT_GT(distance, target * 0.80);
  ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackRunning(controller_));
  EXPECT_NEAR(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::WorkingPlanStartX(controller_),
              robot_x_,
              1e-9);
}

TEST_P(FtcReverseClockBudget, RepeatedShortRosIntervalsCannotRefundEscapeCommandTravel)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  SetParameter("obstacle_reverse_max_dist_m", 0.10);
  robot_x_ = 2.0;
  measured_speed_ = 0.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  controller_.newPathReceived(StraightPlan(robot_x_));
  Wall(3.0);
  double command = 0.0;
  for (int i = 0; i < 30 && command >= 0.0; ++i)
  {
    command = Tick();
  }
  ASSERT_LT(command, 0.0);
  ASSERT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_));
  double distance = 0.0;
  bool stopped = false;
  for (int i = 0; i < 128; ++i)
  {
    ASSERT_LT(command, 0.0);
    const double hold_s = HoldCommand();
    ASSERT_LE(hold_s, 0.50) << "Clock control requires a supported actual hold interval";
    distance -= command * hold_s;
    robot_x_ += command * hold_s;
    measured_speed_ = command * std::get<0>(GetParam());
    EXPECT_LE(distance, 0.10) << "ROS clock undercount cannot refund held escape travel";
    InjectElapsed();
    command = Tick();
    if (command >= 0.0)
    {
      stopped = true;
      break;
    }
  }
  EXPECT_TRUE(stopped);
  EXPECT_GT(distance, 0.05);
}

INSTANTIATE_TEST_SUITE_P(ClockFeedback,
                         FtcReverseClockBudget,
                         ::testing::Values(std::tuple{0.0, 0},
                                           std::tuple{1.0, 0},
                                           std::tuple{0.0, 1},
                                           std::tuple{1.0, 1},
                                           std::tuple{0.0, 2},
                                           std::tuple{1.0, 2}));
}  // namespace
