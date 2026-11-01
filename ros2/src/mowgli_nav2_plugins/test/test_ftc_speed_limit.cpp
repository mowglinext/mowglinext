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
#include <functional>
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
#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_forward_sweep.hpp"
#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
struct FTCSpeedLimitTestAccess
{
  static bool SpliceTerminalPlan(FTCController& controller, nav2_core::GoalChecker* checker)
  {
    TurnFallbackPlan plan;
    plan.rejoin = controller.global_plan_.size() - 4;
    plan.connector_m = 0.0;
    controller.active_goal_checker_ = checker;
    const bool result = controller.spliceTurnFallback(plan, 0);
    controller.active_goal_checker_ = nullptr;
    return result;
  }
  static void SetFreeBoundary(FTCController& controller, double origin_x, double origin_y)
  {
    controller.boundary_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        400, 400, 0.05, origin_x, origin_y, nav2_costmap_2d::FREE_SPACE);
    controller.boundary_frame_ = "map";
  }
  static bool CurrentBodyBlocked(FTCController& controller)
  {
    return controller.currentBodyInLethal();
  }
  static bool TerminalPointEligible(const FTCController& controller)
  {
    return controller.terminal_point_plan_;
  }
  static void AngularHistory(FTCController& controller, double integral, double previous)
  {
    controller.i_angle_error_ = integral;
    controller.last_angle_error_ = previous;
    controller.d_angle_filt_ = 0.0;
  }
  static double AngleError(const FTCController& controller)
  {
    return controller.angle_error_;
  }
  static bool RotationBlocked(FTCController& controller, double angle)
  {
    const double saved = controller.angle_error_;
    const bool eligibility = controller.terminal_point_plan_;
    // Inspect each geometric direction independently of terminal retargeting.
    controller.terminal_point_plan_ = false;
    controller.angle_error_ = angle;
    const bool blocked = controller.pivotSweepBlocked();
    controller.angle_error_ = saved;
    controller.terminal_point_plan_ = eligibility;
    return blocked;
  }
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

  static rclcpp::Time SupportedEpoch(FTCController& controller)
  {
    return controller.supportedPoseEpoch();
  }
  static bool ForwardBlockedAt(FTCController& controller, double speed, const rclcpp::Time& epoch)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    return controller.forwardCommandSweepBlocked(command, command, &epoch);
  }
  static void AuthorizeForwardAt(FTCController& controller, double speed, const rclcpp::Time& epoch)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    controller.authorizeCommand(command, command, &epoch);
  }
  static bool ForwardBlocked(FTCController& controller, double speed)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    return controller.forwardCommandSweepBlocked(command, command);
  }
  static void EnterPostRotate(FTCController& controller)
  {
    controller.current_state_ = FTCController::PlannerState::POST_ROTATE;
  }
  static bool Finished(const FTCController& controller)
  {
    return controller.current_state_ == FTCController::PlannerState::FINISHED;
  }
  static void AuthorizeForward(FTCController& controller, double speed)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    controller.authorizeCommand(command, command);
  }
  static void SetAuthorization(FTCController& controller,
                               const visualization_msgs::msg::MarkerArray& markers)
  {
    controller.authorization_ = mowgli_interfaces::motion::Snapshot::parse(markers);
  }

  static bool PreRotating(const FTCController& controller)
  {
    return controller.current_state_ == FTCController::PlannerState::PRE_ROTATE;
  }
  static bool Pivoting(const FTCController& controller)
  {
    return controller.current_state_ == FTCController::PlannerState::PIVOT;
  }
  static std::optional<std::size_t> StopCorner(const FTCController& controller)
  {
    return controller.stopCornerTarget();
  }
  static void RefreshRotation(FTCController& controller)
  {
    controller.update_control_point(0.0, false, true);
  }
  static bool RotationDeparture(FTCController& controller)
  {
    return controller.rotationDeparturePermitted(geometry_msgs::msg::Twist{});
  }
  static void LateralDeviation(FTCController& controller, double offset)
  {
    controller.lateral_deviation_ = offset;
  }

  static bool StrictGoalStraightPlan(const FTCController& controller)
  {
    return controller.strict_goal_straight_plan_;
  }

  static bool StrictGoalDeparted(const FTCController& controller)
  {
    return controller.strict_goal_departed_;
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

TEST_F(FtcSpeedLimit, GoalCheckerBindsCompleteCurrentPlanAfterReset)
{
  mowgli_nav2_plugins::PathProgressGoalChecker checker;
  checker.initialize(node_, "coverage", costmap_);
  auto loop = StraightPlan();
  for (int i = 119; i >= 0; --i)
    loop.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(loop);
  checker.reset();
  auto at_start = MakePose(0.0).pose;
  EXPECT_FALSE(checker.isGoalReached(
      at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  Tick(0.1, &checker);
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_FALSE(checker.isGoalReached(
        at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
    EXPECT_FALSE(checker.isGoalXYReached(
        at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  }
  // The next full plan may have the same stamp, size and start. Its actual
  // controller revision, rather than those heuristics, replaces the loop.
  nav_msgs::msg::Path terminal;
  terminal.header = loop.header;
  terminal.poses.assign(loop.poses.size(), MakePose(0.0));
  controller_.newPathReceived(terminal);
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(
      at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  Tick(0.1, &checker);
  EXPECT_TRUE(checker.isGoalReached(
      at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
}

TEST_F(FtcSpeedLimit, FallbackSpliceImmediatelyRebindsGoalChecker)
{
  mowgli_nav2_plugins::PathProgressGoalChecker checker;
  checker.initialize(node_, "coverage", costmap_);
  auto loop = StraightPlan();
  for (int i = 119; i >= 0; --i)
    loop.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(loop);
  checker.reset();
  Tick(0.1, &checker);
  const auto at_start = MakePose(0.0).pose;
  EXPECT_FALSE(checker.isGoalReached(
      at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  // Exercise the actual splice commit with the selected checker, independent
  // of lattice eligibility. No subsequent compute/output call supplies a bind.
  ASSERT_TRUE(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SpliceTerminalPlan(controller_, &checker));
  EXPECT_TRUE(checker.isGoalReached(
      at_start, at_start, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
}

class RequestedGoalTolerance : public nav2_core::GoalChecker
{
public:
  double x{0.03};
  double y{0.03};
  bool available{true};
  unsigned int queries{0};
  std::function<void(unsigned int)> on_query;

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
    if (on_query)
      on_query(queries);
    pose.position.x = x;
    pose.position.y = y;
    return available;
  }
};

class FtcRequestedGoalTolerance : public FtcSpeedLimit
{
protected:
  geometry_msgs::msg::Twist StepAt(double x,
                                   double y,
                                   double yaw,
                                   const geometry_msgs::msg::Twist& measured,
                                   RequestedGoalTolerance& checker,
                                   double dt = 0.1,
                                   double forecast_yaw = std::numeric_limits<double>::quiet_NaN())
  {
    time_ns_ += static_cast<int64_t>(std::llround(dt * 1e9));
    SetTime();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
    EXPECT_TRUE(tf_->setTransform(transform, "terminal_2d", false));
    if (std::isfinite(forecast_yaw))
    {
      transform.header.stamp = rclcpp::Time(time_ns_ + 100000000, RCL_ROS_TIME);
      transform.transform.rotation.z = std::sin(forecast_yaw / 2.0);
      transform.transform.rotation.w = std::cos(forecast_yaw / 2.0);
      EXPECT_TRUE(tf_->setTransform(transform, "terminal_forecast", false));
    }
    return controller_.computeVelocityCommands(Pose{}, measured, &checker, {}, Pose{}).twist;
  }

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

  void CheckTerminalEpochChange(double checked_remaining)
  {
    using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
    RequestedGoalTolerance checker;
    SetParameter("max_goal_distance_error", 0.5);
    SetParameter("max_goal_angle_error", 1.0);
    WaitAtFinalCarrot(0.76, checker);
    constexpr double x = 0.775, y = -0.03;
    const double initial_bearing = std::atan2(0.03, 0.025);
    // An exactly aligned translated pose avoids an accidental tiny negative
    // bearing making the old sign check stop only through roundoff.
    const double updated_y = checked_remaining == 0.0 ? 0.0 : y;
    const double updated_yaw = checked_remaining == 0.0 ? 0.0 : initial_bearing - checked_remaining;
    auto* map = costmap_->getCostmap();
    // A finer test raster resolves an obstacle outside the checked small
    // arc but inside the unbounded PID's 0.08-rad intended rotation.
    map->resizeMap(2000, 2000, 0.005, -5.0, -5.0);
    geometry_msgs::msg::Twist stopped;
    StepAt(x, y, 0.0, stopped, checker);
    unsigned int mx, my;
    ASSERT_TRUE(
        map->worldToMap(x + std::cos(updated_yaw) * 0.5525 + std::sin(updated_yaw) * 0.25,
                        updated_y + std::sin(updated_yaw) * 0.5525 - std::cos(updated_yaw) * 0.25,
                        mx,
                        my));
    map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    ASSERT_FALSE(Access::CurrentBodyBlocked(controller_));
    SetParameter("ki_ang", 1.0);
    SetParameter("kd_ang", 0.0);
    Access::AngularHistory(controller_, 10.0, initial_bearing);
    checker.queries = 0;
    bool updated = false;
    checker.on_query = [&](unsigned int query)
    {
      // Common tolerance, point projection, alignment decision, then sweep.
      // Advance TF only after the old bearing selected terminal alignment.
      if (query != 4)
        return;
      EXPECT_NEAR(Access::AngleError(controller_), initial_bearing, 1e-12);
      ++time_ns_;
      SetTime();
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base_footprint";
      transform.transform.translation.x = x;
      transform.transform.translation.y = updated_y;
      transform.transform.rotation.z = std::sin(updated_yaw / 2.0);
      transform.transform.rotation.w = std::cos(updated_yaw / 2.0);
      EXPECT_TRUE(tf_->setTransform(transform, "supported_epoch_update", false));
      updated = true;
    };
    const auto command = StepAt(x, y, 0.0, stopped, checker);
    ASSERT_TRUE(updated);
    EXPECT_NEAR(Access::AngleError(controller_), checked_remaining, 1e-12);
    ASSERT_FALSE(Access::CurrentBodyBlocked(controller_));
    ASSERT_FALSE(Access::RotationBlocked(controller_, checked_remaining));
    ASSERT_TRUE(Access::RotationBlocked(controller_, 0.08));
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    if (checked_remaining == 0.0)
      EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    else
    {
      EXPECT_GT(command.angular.z, 0.0);
      EXPECT_LE(command.angular.z * 0.1, checked_remaining);
    }
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

// Exact production export of old points 180..210, with the explicit T twin.
// The local model feeds actual returned commands back as measured velocity;
// it does not inject a private controller state or edit the retained route.
class FtcStopCorner : public FtcRequestedGoalTolerance
{
protected:
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  static constexpr double tip_x = 2.454825207353694;
  static constexpr double tip_y = .8013791580379307;
  static constexpr double outgoing = 3.140918321144495;

  void SetUp() override
  {
    FtcRequestedGoalTolerance::SetUp();
    for (const auto& [name, value] :
         std::vector<std::pair<std::string, double>>{{"speed_angular", 45},
                                                     {"speed_fast_threshold_angle", 10},
                                                     {"acceleration", .2},
                                                     {"derivative_filter_tau", .2},
                                                     {"goal_timeout", 10},
                                                     {"kd_ang", 0},
                                                     {"kd_lat", .5},
                                                     {"kd_lon", 0},
                                                     {"ki_ang", 0},
                                                     {"ki_lat", 0},
                                                     {"ki_lon", 0},
                                                     {"kp_ang", 1.5},
                                                     {"kp_ang_following", 1},
                                                     {"kp_lat", .8},
                                                     {"kp_lon", 1},
                                                     {"max_goal_angle_error", 30}})
      SetParameter(name, value);
    SetParameter("check_obstacles", true);
    SetParameter("enable_obstacle_deviation", true);
    SetParameter("confine_deviation_to_zone", true);
    ASSERT_TRUE(costmap_->set_parameter({"footprint_padding", 0.0}).successful);
    Access::SetFreeBoundary(controller_, -5, -5);
  }

  std::pair<double, double> Rotate(double x, double y, double rotation)
  {
    x -= tip_x;
    y -= tip_y;
    return {x * std::cos(rotation) - y * std::sin(rotation),
            x * std::sin(rotation) + y * std::cos(rotation)};
  }

  void Geometry(double rotation, bool forbid_forward = false)
  {
    visualization_msgs::msg::MarkerArray geometry;
    const auto ring =
        [&](const std::string& kind, const std::vector<std::pair<double, double>>& points)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = kind;
      marker.text = kind == "area" ? "mowing" : "";
      marker.type = marker.LINE_STRIP;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1;
      for (const auto& [x, y] : points)
      {
        const auto [xx, yy] = Rotate(x, y, rotation);
        geometry_msgs::msg::Point point;
        point.x = xx;
        point.y = yy;
        marker.points.push_back(point);
      }
      geometry.markers.push_back(marker);
    };
    if (forbid_forward)
      // Reference at T is valid; all westward restart motion is outside.
      ring("area", {{tip_x, -5}, {15, -5}, {15, 15}, {tip_x, 15}});
    else
    {
      ring("area",
           {{-4.5, double(float(.8))},
            {4.5, double(float(.8))},
            {4.5, double(float(6.8))},
            {-4.5, double(float(6.8))}});
      ring("obstacle",
           {{double(float(2.7)), double(float(1.2))},
            {double(float(3.3)), double(float(1.2))},
            {double(float(3.3)), double(float(1.8))},
            {double(float(2.7)), double(float(1.8))}});
    }
    Access::SetAuthorization(controller_, geometry);
  }

  nav_msgs::msg::Path Captured(double rotation = 0, std::size_t first = 0)
  {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (const auto& [x, y, yaw] : std::vector<std::tuple<double, double, double>>{
             {2.23100004768372, 1.5434185857568339, -1.5707963267948963},
             {2.23100004768372, 1.4565814142431661, -1.5707963267948963},
             {2.23100004768372, 1.3697442427294986, -1.5707963267948963},
             {2.23100004768372, 1.2829070712158308, -1.5707963267948963},
             {2.23100004768372, 1.1960698997021633, -1.4726215563702667},
             {2.239632103129413, 1.1084271699667103, -1.2762720155207805},
             {2.266342926297124, 1.0203733865857147, -1.0799224746715197},
             {2.309718949441999, 0.9392225550549176, -0.8835729338221637},
             {2.3680932563201402, 0.868093256320143, -0.6872233929727825},
             {2.439222555054915, 0.8097189494419957, -0.49087385212338086},
             {2.454825207353694, 0.8013791580379307, -0.49087385212338086},
             {2.454825207353694, 0.8013791580379307, 3.140918321144495},
             {2.3553602334489576, 0.8014462305071719, 3.140918321144495},
             {2.2558952595442214, 0.8015133029764131, 3.140918321144495},
             {2.1564302856394857, 0.8015803754456543, 3.140918321144495},
             {2.0569653117347495, 0.8016474479148955, 3.140918321144495},
             {1.9575003378300133, 0.8017145203841367, 3.140918321144495},
             {1.8580353639252771, 0.8017815928533779, 3.140918321144495},
             {1.758570390020541, 0.8018486653226191, 3.140918321144495},
             {1.659105416115805, 0.8019157377918603, 3.140918321144495},
             {1.559640442211069, 0.8019828102611015, 3.140918321144495},
             {1.4601754683063328, 0.8020498827303427, 3.140918321144495},
             {1.3607104944015966, 0.8021169551995839, 3.140918321144496},
             {1.2612455204968605, 0.802184027668825, 3.140918321144495},
             {1.1617805465921245, 0.8022511001380662, 3.140918321144495},
             {1.0623155726873883, 0.8023181726073074, 3.140918321144495},
             {0.9628505987826523, 0.8023852450765486, 3.140918321144495},
             {0.8633856248779161, 0.8024523175457898, 3.140918321144495},
             {0.76392065097318, 0.802519390015031, 3.140918321144495},
             {0.664455677068444, 0.8025864624842722, 3.140918321144495},
             {0.5649907031637076, 0.8026535349535134, 3.140918321144495},
             {0.4655257292589716, 0.8027206074227546, 3.140918321144495},
         })
    {
      const auto [xx, yy] = Rotate(x, y, rotation);
      plan.poses.push_back(MakePose(xx, yy, yaw + rotation));
    }
    plan.poses.erase(plan.poses.begin(), plan.poses.begin() + first);
    return plan;
  }
};

TEST_F(FtcStopCorner, CapturedApproachStopsPivotsAndDepartsWithRotatedControl)
{
  for (const double rotation : {0.0, M_PI / 4})
  {
    Geometry(rotation);
    const auto plan = Captured(rotation);
    controller_.newPathReceived(plan);
    double x = plan.poses.front().pose.position.x;
    double y = plan.poses.front().pose.position.y;
    double yaw = -M_PI / 2 + rotation;
    geometry_msgs::msg::Twist measured;
    RequestedGoalTolerance checker;
    bool pivoted = false, departed = false, slowed = false, coast_checked = false;
    for (int tick = 0; tick < 300; ++tick)
    {
      const auto command = StepAt(x, y, yaw, measured, checker);
      pivoted = pivoted || Access::Pivoting(controller_);
      slowed = slowed || (!pivoted && command.linear.x > 0 && command.linear.x < .15);
      if (Access::Pivoting(controller_))
      {
        EXPECT_DOUBLE_EQ(command.linear.x, 0);
        if (!coast_checked)
        {
          geometry_msgs::msg::Twist coasting;
          coasting.linear.x = .02;
          const auto hold = StepAt(x, y, yaw, coasting, checker);
          EXPECT_DOUBLE_EQ(hold.linear.x, 0);
          EXPECT_DOUBLE_EQ(hold.angular.z, 0);
          coast_checked = true;
        }
      }
      // The unchanged raw geometry and response guard authorize every output.
      x += command.linear.x * .1 * std::cos(yaw);
      y += command.linear.x * .1 * std::sin(yaw);
      yaw += command.angular.z * .1;
      measured = command;
      const double westward = -(x * std::cos(rotation) + y * std::sin(rotation));
      if (pivoted && !Access::Pivoting(controller_) && westward > .6)
      {
        departed = true;
        break;
      }
    }
    EXPECT_TRUE(slowed) << rotation;
    EXPECT_TRUE(pivoted) << rotation;
    EXPECT_TRUE(departed) << rotation;
    EXPECT_TRUE(coast_checked);
  }
}

TEST_F(FtcStopCorner, RestartBothTwinHalvesUsesGeometryRatherThanSmallerAngleTolerance)
{
  Geometry(0);
  RequestedGoalTolerance checker;
  for (const std::size_t first : {10u, 11u})
  {
    // Incoming and outgoing halves both resume at the outgoing heading.
    controller_.newPathReceived(Captured(0, first));
    const auto unsafe = StepAt(0, 0, outgoing + .30, {}, checker);
    EXPECT_TRUE(Access::PreRotating(controller_));
    EXPECT_DOUBLE_EQ(unsafe.linear.x, 0);
    EXPECT_LT(unsafe.angular.z, 0);
    controller_.newPathReceived(Captured(0, first));
    auto safe = StepAt(0, 0, outgoing - .30, {}, checker);
    EXPECT_FALSE(Access::PreRotating(controller_));
    EXPECT_DOUBLE_EQ(safe.linear.x, 0);
    safe = StepAt(0, 0, outgoing - .30, {}, checker);
    EXPECT_GT(safe.linear.x, 0);
  }
}

TEST_F(FtcStopCorner, RetainedNearCornerCheckpointResumesThroughBrakedApproach)
{
  // Actual public FollowStrip halt/reload selected old189, not the T twin.
  // Also interrupt at/just beyond T: a zero approach cap must not deadlock PRE.
  for (const double rotation : {0.0, M_PI / 4})
  {
    for (const auto& initial : std::vector<std::pair<double, double>>{
             {2.4342059812035313, .8028570068674588}, {tip_x, tip_y}, {tip_x + .002, tip_y}})
    {
      Geometry(rotation);
      controller_.newPathReceived(Captured(rotation, 9));
      auto [x, y] = Rotate(initial.first, initial.second, rotation);
      double yaw = -.21795608461903856 + rotation;
      geometry_msgs::msg::Twist measured;
      RequestedGoalTolerance checker;
      bool pivoted = false, departed = false;
      for (int tick = 0; tick < 300; ++tick)
      {
        const auto command = StepAt(x, y, yaw, measured, checker);
        pivoted = pivoted || Access::Pivoting(controller_);
        x += command.linear.x * .1 * std::cos(yaw);
        y += command.linear.x * .1 * std::sin(yaw);
        yaw += command.angular.z * .1;
        measured = command;
        if (pivoted && !Access::Pivoting(controller_) &&
            -(x * std::cos(rotation) + y * std::sin(rotation)) > .6)
        {
          departed = true;
          break;
        }
      }
      EXPECT_TRUE(pivoted) << rotation << " / " << initial.first;
      EXPECT_TRUE(departed) << rotation << " / " << initial.first;
    }
  }
}

TEST_F(FtcStopCorner, OutgoingTwinRestartQualifiesStraightDepartureDirection)
{
  for (const double rotation : {0.0, M_PI / 4})
  {
    Geometry(rotation);
    controller_.newPathReceived(Captured(rotation, 11));
    auto [x, y] = Rotate(tip_x + .002, tip_y, rotation);
    double yaw = outgoing + .014 + rotation;
    geometry_msgs::msg::Twist measured;
    RequestedGoalTolerance checker;
    bool departed = false;
    for (int tick = 0; tick < 300; ++tick)
    {
      const auto command = StepAt(x, y, yaw, measured, checker);
      x += command.linear.x * .1 * std::cos(yaw);
      y += command.linear.x * .1 * std::sin(yaw);
      yaw += command.angular.z * .1;
      measured = command;
      if (-(x * std::cos(rotation) + y * std::sin(rotation)) > .6)
      {
        departed = true;
        break;
      }
    }
    EXPECT_TRUE(departed) << rotation;
  }
}

TEST_F(FtcStopCorner, PreRotateHoldsTranslationAndRejectsForecastAlignment)
{
  Geometry(0);
  RequestedGoalTolerance checker;
  controller_.newPathReceived(Captured(0, 11));
  geometry_msgs::msg::Twist moving;
  moving.linear.x = .02;
  const auto coast = StepAt(0, 0, outgoing + .30, moving, checker);
  EXPECT_DOUBLE_EQ(coast.linear.x, 0);
  EXPECT_DOUBLE_EQ(coast.angular.z, 0);
  EXPECT_TRUE(Access::PreRotating(controller_));
  const auto current = StepAt(0, 0, outgoing + .30, {}, checker, .1, outgoing);
  EXPECT_TRUE(Access::PreRotating(controller_));
  EXPECT_DOUBLE_EQ(current.linear.x, 0);
  EXPECT_LT(current.angular.z, 0);
}

TEST_F(FtcStopCorner, DepartureRechecksChangedSupportedHeading)
{
  Geometry(0);
  controller_.newPathReceived(Captured(0, 11));
  SetTransform("odom", "base_footprint", 0, false, 0, outgoing);
  Access::RefreshRotation(controller_);
  EXPECT_NEAR(Access::AngleError(controller_), 0, 1e-12);
  // A new complete current epoch arrives after the initial angle decision.
  ++time_ns_;
  SetTime();
  SetTransform("odom", "base_footprint", 0, false, 0, outgoing + .30);
  EXPECT_FALSE(Access::RotationDeparture(controller_));
  EXPECT_NEAR(Access::AngleError(controller_), -.30, 1e-12);
}

TEST_F(FtcStopCorner, InvalidOrStaleDepartureEvidenceThrows)
{
  Geometry(0);
  controller_.newPathReceived(Captured(0, 11));
  SetTransform("odom", "base_footprint", 0, false, 0, outgoing);
  time_ns_ += 400000000;
  SetTime();
  EXPECT_THROW(Access::RotationDeparture(controller_), nav2_core::ControllerException);
  SetTransform("odom", "base_footprint", 0, false, 0, outgoing);
  Access::SetAuthorization(controller_, {});
  EXPECT_THROW(Access::RotationDeparture(controller_), nav2_core::ControllerException);
  Geometry(0);
  tf_->clear();
  EXPECT_THROW(Access::RotationDeparture(controller_), nav2_core::ControllerException);
}

TEST_F(FtcStopCorner, ValidPermanentDepartureDenialRetainsOriginalTimeout)
{
  Geometry(0, true);
  controller_.newPathReceived(Captured(0, 11));
  RequestedGoalTolerance checker;
  bool failed = false;
  for (int tick = 0; tick < 110; ++tick)
  {
    try
    {
      const auto command = StepAt(0, 0, outgoing, {}, checker);
      EXPECT_DOUBLE_EQ(command.linear.x, 0);
      EXPECT_NEAR(command.angular.z, 0, 1e-12);
      EXPECT_TRUE(Access::PreRotating(controller_));
    }
    catch (const nav2_core::ControllerException&)
    {
      EXPECT_GE(tick, 99);
      failed = true;
      break;
    }
  }
  EXPECT_TRUE(failed);
}

TEST_F(FtcStopCorner, StopTargetRequiresExplicitLocalForwardMonotoneUndeviatedLeg)
{
  // A free lawn isolates eligibility from the captured boundary response.
  RequestedGoalTolerance checker;
  for (const bool backward : {false, true})
  {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.poses = backward ? std::vector<Pose>{MakePose(-.1, 0, M_PI),
                                              MakePose(-.15, 0, 0),
                                              MakePose(0, 0, 0),
                                              MakePose(0, 0, M_PI / 2),
                                              MakePose(0, .5, M_PI / 2),
                                              MakePose(0, 1, M_PI / 2)}
                          : std::vector<Pose>{MakePose(-.15),
                                              MakePose(-.075),
                                              MakePose(0),
                                              MakePose(0, 0, M_PI / 2),
                                              MakePose(0, .5, M_PI / 2),
                                              MakePose(0, 1, M_PI / 2)};
    controller_.newPathReceived(plan);
    StepAt(backward ? -.1 : -.15, 0, backward ? M_PI : 0, {}, checker);
    EXPECT_EQ(Access::StopCorner(controller_).has_value(), !backward);
    Access::LateralDeviation(controller_, .01);
    EXPECT_FALSE(Access::StopCorner(controller_).has_value());
    Access::LateralDeviation(controller_, 0);
  }
  controller_.newPathReceived(StraightPlan());
  StepAt(0, 0, 0, {}, checker);
  EXPECT_FALSE(Access::StopCorner(controller_).has_value());
}

TEST_F(FtcRequestedGoalTolerance, CapturedTerminalCurveReachesStrictPointAndFinishes)
{
  // Original old266..268 and retained initial pose. Translate everything,
  // including the float32-derived raw mowing union/hole, into this test grid.
  constexpr double initial_x = -4.385790475580301;
  constexpr double initial_y = 0.9773281974209458;
  constexpr double initial_yaw = 2.39348777754403;
  for (const double rotation : {0.0, M_PI / 2.0})
  {
    for (const auto& [name, value] :
         std::vector<std::pair<std::string, double>>{{"speed_angular", 45.0},
                                                     {"speed_fast_threshold_angle", 10.0},
                                                     {"acceleration", 0.2},
                                                     {"derivative_filter_tau", 0.2},
                                                     {"goal_timeout", 10.0},
                                                     {"kd_ang", 0.0},
                                                     {"kd_lat", 0.5},
                                                     {"kd_lon", 0.0},
                                                     {"ki_ang", 0.0},
                                                     {"ki_lat", 0.0},
                                                     {"ki_lon", 0.0},
                                                     {"kp_ang", 1.5},
                                                     {"kp_ang_following", 1.0},
                                                     {"kp_lat", 0.8},
                                                     {"kp_lon", 1.0},
                                                     {"max_goal_distance_error", 0.5},
                                                     {"max_goal_angle_error", 30.0},
                                                     {"stall_grace_s", 0.6},
                                                     {"obstacle_lookahead", 30.0},
                                                     {"obstacle_body_half_width", 0.12},
                                                     {"obstacle_clearance_margin", 0.05},
                                                     {"obstacle_reverse_speed_mps", 0.1},
                                                     {"obstacle_wait_timeout_s", 2.5}})
    {
      // obstacle_lookahead is an integer parameter and is already irrelevant
      // to this empty local grid; retain its declaration type.
      if (name == "obstacle_lookahead")
        ASSERT_TRUE(node_->set_parameter({std::string(kPlugin) + "." + name, 30}).successful);
      else
        SetParameter(name, value);
    }
    SetParameter("check_obstacles", true);
    SetParameter("enable_obstacle_deviation", true);
    SetParameter("confine_deviation_to_zone", true);
    SetParameter("turn_fallback_enabled", true);
    ASSERT_TRUE(costmap_->set_parameter({"footprint_padding", 0.0}).successful);
    const auto relocate = [rotation](double x, double y)
    {
      x -= initial_x;
      y -= initial_y;
      return std::pair{x * std::cos(rotation) - y * std::sin(rotation),
                       x * std::sin(rotation) + y * std::cos(rotation)};
    };
    double boundary_x = std::numeric_limits<double>::infinity();
    double boundary_y = std::numeric_limits<double>::infinity();
    for (const auto& [bx, by] :
         std::vector<std::pair<double, double>>{{-5, -5}, {15, -5}, {15, 15}, {-5, 15}})
    {
      const auto [xx, yy] = relocate(bx, by);
      boundary_x = std::min(boundary_x, xx);
      boundary_y = std::min(boundary_y, yy);
    }
    mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetFreeBoundary(controller_,
                                                                  boundary_x,
                                                                  boundary_y);
    visualization_msgs::msg::MarkerArray geometry;
    const auto ring =
        [&](const std::string& kind, const std::vector<std::pair<double, double>>& points)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = kind;
      marker.text = kind == "area" ? "mowing" : "";
      marker.type = marker.LINE_STRIP;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1.0;
      for (const auto& [x, y] : points)
      {
        const auto [xx, yy] = relocate(x, y);
        geometry_msgs::msg::Point point;
        point.x = xx;
        point.y = yy;
        marker.points.push_back(point);
      }
      geometry.markers.push_back(marker);
    };
    ring("area",
         {{-4.5, double(float(0.8))},
          {4.5, double(float(0.8))},
          {4.5, double(float(6.8))},
          {-4.5, double(float(6.8))}});
    ring("obstacle",
         {{double(float(2.7)), double(float(1.2))},
          {double(float(3.3)), double(float(1.2))},
          {double(float(3.3)), double(float(1.8))},
          {double(float(2.7)), double(float(1.8))}});
    mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetAuthorization(controller_, geometry);
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (const auto& [x, y, yaw] : std::vector<std::tuple<double, double, double>>{
             {-4.3751058093707016, 0.91013697398898796, 2.3553524305651767},
             {-4.3957156970559916, 0.93078160044328406, 2.2805366314624038},
             {-4.4147248269874906, 0.95290898282541625, 2.2057208323595971}})
    {
      const auto [xx, yy] = relocate(x, y);
      plan.poses.push_back(MakePose(xx, yy, yaw + rotation));
    }
    controller_.newPathReceived(plan);
    RequestedGoalTolerance checker;
    double x = 0.0, y = 0.0, yaw = initial_yaw + rotation;
    geometry_msgs::msg::Twist measured, command;
    bool finished = false, entered_band = false;
    for (int tick = 0; tick < 200; ++tick)
    {
      command = StepAt(x, y, yaw, measured, checker);
      const auto& goal = plan.poses.back().pose.position;
      entered_band = entered_band || std::hypot(x - goal.x, y - goal.y) <= checker.x;
      if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Finished(controller_))
      {
        finished = true;
        break;
      }
      // Unclipped ideal 2D Euler integration, with the actual command fed back
      // as measured v/w on the next compute. No private FSM/PID state seeding.
      x += command.linear.x * 0.1 * std::cos(yaw);
      y += command.linear.x * 0.1 * std::sin(yaw);
      yaw += command.angular.z * 0.1;
      measured = command;
    }
    const auto& goal = plan.poses.back().pose.position;
    RecordProperty(rotation == 0.0 ? "final_distance_m" : "rotated_final_distance_m",
                   std::to_string(std::hypot(x - goal.x, y - goal.y)));
    EXPECT_TRUE(finished) << "rotation=" << rotation;
    EXPECT_TRUE(entered_band);
    EXPECT_LE(std::hypot(x - goal.x, y - goal.y), checker.x);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}

TEST_F(FtcRequestedGoalTolerance, TerminalAlignmentRejectsOppositePidRotationWithLocalObstacle)
{
  RequestedGoalTolerance checker;
  SetParameter("max_goal_distance_error", 0.5);
  WaitAtFinalCarrot(0.76, checker);
  auto* map = costmap_->getCostmap();
  const double x = 0.775, y = -0.03;
  const double bearing = std::atan2(0.03, 0.025);
  const auto mark = [&](double dy)
  {
    unsigned int mx, my;
    ASSERT_TRUE(map->worldToMap(x + 0.4, y + dy, mx, my));
    map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  };
  geometry_msgs::msg::Twist stopped;
  // Update actual 2D TF, then verify the asymmetry with the real local sweep.
  StepAt(x, y, 0.0, stopped, checker);
  mark(-0.4);
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::RotationBlocked(controller_, bearing));
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::RotationBlocked(controller_, -bearing));
  SetParameter("ki_ang", 1.0);
  SetParameter("kd_ang", 1.0);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AngularHistory(controller_, -10.0, 10.0);
  const auto rejected = StepAt(x, y, 0.0, stopped, checker);
  EXPECT_DOUBLE_EQ(rejected.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(rejected.angular.z, 0.0);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AngularHistory(controller_, 0.0, bearing);
  const auto ccw = StepAt(x, y, 0.0, stopped, checker);
  EXPECT_DOUBLE_EQ(ccw.linear.x, 0.0);
  EXPECT_GT(ccw.angular.z, 0.0);
  map->resetMap(0, 0, map->getSizeInCellsX(), map->getSizeInCellsY());
  mark(0.4);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::RotationBlocked(controller_, bearing));
  const auto blocked = StepAt(x, y, 0.0, stopped, checker);
  EXPECT_DOUBLE_EQ(blocked.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(blocked.angular.z, 0.0);
}

TEST_F(FtcRequestedGoalTolerance, TerminalAlignmentFutureYawCannotHideCurrentLethalBody)
{
  RequestedGoalTolerance checker;
  SetParameter("max_goal_distance_error", 0.5);
  WaitAtFinalCarrot(0.76, checker);
  constexpr double x = 0.775, y = -0.03;
  geometry_msgs::msg::Twist stopped;
  StepAt(x, y, 0.0, stopped, checker);
  unsigned int mx, my;
  auto* map = costmap_->getCostmap();
  ASSERT_TRUE(map->worldToMap(x + 0.5, y, mx, my));
  map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::CurrentBodyBlocked(controller_));
  const auto command = StepAt(x, y, 0.0, stopped, checker, 0.1, M_PI / 2.0);
  // Latest TF has rotated the chassis away from the cell. The supported
  // current pose still contains it, before the requested CCW turn begins.
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::CurrentBodyBlocked(controller_));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(FtcRequestedGoalTolerance, TerminalAlignmentStopsWhenSweepEpochIsAlreadyAligned)
{
  CheckTerminalEpochChange(0.0);
}

TEST_F(FtcRequestedGoalTolerance, TerminalAlignmentCommandStaysInsideNewSweepEpochArc)
{
  CheckTerminalEpochChange(0.02);
}

TEST_F(FtcRequestedGoalTolerance, TerminalPointEligibilityRejectsReturnPivotAndNonMapPlans)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  auto plan = StraightPlan();
  controller_.newPathReceived(plan);
  EXPECT_TRUE(Access::TerminalPointEligible(controller_));
  plan.poses.push_back(MakePose(0.0));
  controller_.newPathReceived(plan);
  EXPECT_FALSE(Access::TerminalPointEligible(controller_));
  plan = StraightPlan();
  plan.poses.insert(plan.poses.begin() + 1, MakePose(0.0, 0.0, M_PI / 2));
  controller_.newPathReceived(plan);
  EXPECT_FALSE(Access::TerminalPointEligible(controller_));
  plan = StraightPlan();
  plan.header.frame_id = "odom";
  controller_.newPathReceived(plan);
  EXPECT_FALSE(Access::TerminalPointEligible(controller_));
  plan = StraightPlan();
  plan.poses[1].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  controller_.newPathReceived(plan);
  EXPECT_FALSE(Access::TerminalPointEligible(controller_));
}

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

TEST_F(FtcRequestedGoalTolerance, StrictApproachSlowsBeforeBoundaryResponseReachesGoal)
{
  SetParameter("max_goal_distance_error", 0.50);
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "area";
  marker.text = "mowing";
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1.0;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{-5, -5}, {0.80, -5}, {0.80, 5}, {-5, 5}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetAuthorization(controller_, markers);
  RequestedGoalTolerance checker;
  std::vector<double> commands;
  const double reached = FollowShortPath(&checker, &commands);
  EXPECT_GE(reached, 0.80 - checker.x);
  EXPECT_LE(reached, 0.80);
  EXPECT_TRUE(std::any_of(commands.begin(),
                          commands.end(),
                          [](double speed)
                          {
                            return speed > 0.0 && speed < 0.15;
                          }));
}

TEST_F(FtcRequestedGoalTolerance, ClosedPathStartsMovingBeforeDepartingGoalBand)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    plan.poses.push_back(MakePose(0.05 * i));
  for (int i = 15; i >= 0; --i)
    plan.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(plan);
  measured_speed_ = 0.0;
  double furthest = 0.0;
  for (int i = 0; i < 25; ++i)
  {
    const double command = Tick(0.10, &checker);
    robot_x_ += command * 0.10;
    measured_speed_ = command;
    furthest = std::max(furthest, robot_x_);
  }
  EXPECT_GT(furthest, checker.x);
}

TEST_F(FtcRequestedGoalTolerance, SelfNearPathKeepsInitialFollowingCommands)
{
  SetParameter("max_goal_distance_error", 0.50);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    plan.poses.push_back(MakePose(0.05 * i));
  for (int i = 15; i >= 2; --i)
    plan.poses.push_back(MakePose(0.05 * i));
  plan.poses.push_back(MakePose(0.06));
  auto first_commands = [&](nav2_core::GoalChecker* checker)
  {
    robot_x_ = 0.0;
    measured_speed_ = 0.0;
    controller_.newPathReceived(plan);
    std::vector<double> commands;
    for (int i = 0; i < 5; ++i)
    {
      const double command = Tick(0.10, checker);
      commands.push_back(command);
      robot_x_ += command * 0.10;
      measured_speed_ = command;
    }
    return commands;
  };
  const auto baseline = first_commands(nullptr);
  RequestedGoalTolerance checker;
  const auto strict = first_commands(&checker);
  EXPECT_EQ(strict, baseline);
  EXPECT_GT(robot_x_, 0.0);
}

TEST_F(FtcRequestedGoalTolerance, RejectedStalePoseDoesNotArmDeparture)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    plan.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(plan);
  measured_speed_ = 0.0;
  ASSERT_DOUBLE_EQ(Tick(0.10, &checker), 0.0);
  ASSERT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::StrictGoalDeparted(controller_));

  // Advance the clock without publishing a new robot transform. The stale
  // pose is outside the goal band, but cannot authorize forward motion.
  time_ns_ += 500000000;
  SetTime();
  geometry_msgs::msg::Twist measured;
  EXPECT_THROW(controller_.computeVelocityCommands(
                   Pose{}, measured, &checker, nav_msgs::msg::Path{}, Pose{}),
               nav2_core::ControllerException);
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::StrictGoalDeparted(controller_));
  EXPECT_GT(Tick(0.10, &checker), 0.0);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::StrictGoalDeparted(controller_));
}

TEST_F(FtcRequestedGoalTolerance, CrossingAndNearCrossingAreNotStraightTerminalPlans)
{
  for (const double offset : {0.0, 0.005})
  {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    // The goal is visited before the end; no explicit pivot marker is used.
    for (const auto& xy : std::vector<std::pair<double, double>>{
             {-0.10, 0.0}, {0.0, 0.0}, {0.04, 0.04}, {0.08, 0.0}, {0.0, offset}})
      plan.poses.push_back(MakePose(xy.first, xy.second));
    controller_.newPathReceived(plan);
    EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::StrictGoalStraightPlan(controller_));
  }
  nav_msgs::msg::Path straight;
  straight.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    straight.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(straight);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::StrictGoalStraightPlan(controller_));
}

TEST_F(FtcRequestedGoalTolerance, FutureTransformDoesNotParkBeforeCurrentPosition)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  WaitAtFinalCarrot(0.76, checker);
  // Cache an extrapolated pose 100 ms ahead while the actual clock remains here.
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.78, false);
  time_ns_ -= 100000000;
  EXPECT_GT(Tick(0.0, &checker), 0.0);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ApproachingGoal(controller_));
}

TEST_F(FtcSpeedLimit, ForwardAuthorizationStartsAtCurrentNotFuturePose)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "area";
  marker.text = "mowing";
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1.0;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{-5, -5}, {0.80, -5}, {0.80, 5}, {-5, 5}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetAuthorization(controller_, markers);
  SetTransform("odom", "base_footprint", 0.70, false);
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.72, false);
  time_ns_ -= 100000000;
  EXPECT_NO_THROW(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeForward(controller_, 0.15));
}

TEST_F(FtcSpeedLimit, ForwardLethalSweepStartsAtCurrentNotFuturePose)
{
  Wall(1.36);
  SetTransform("odom", "base_footprint", 0.70, false);
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.73, false);
  time_ns_ -= 100000000;
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ForwardBlocked(controller_, 0.15));
}

TEST_F(FtcSpeedLimit, BothForwardGuardsReuseSelectedEpochAfterNewTransform)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "area";
  marker.text = "mowing";
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1.0;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{-5, -5}, {0.80, -5}, {0.80, 5}, {-5, 5}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SetAuthorization(controller_, markers);
  Wall(1.36);
  SetTransform("odom", "base_footprint", 0.70, false);
  const auto epoch = mowgli_nav2_plugins::FTCSpeedLimitTestAccess::SupportedEpoch(controller_);
  time_ns_ += 100000000;
  SetTime();
  SetTransform("odom", "base_footprint", 0.73, false);
  EXPECT_FALSE(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ForwardBlockedAt(controller_, 0.15, epoch));
  EXPECT_NO_THROW(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeForwardAt(controller_, 0.15, epoch));
}

TEST_F(FtcSpeedLimit, MissingCurrentBracketRejectsBothForwardGuards)
{
  tf_->clear();
  SetTransform("map", "odom", 0.0, true);
  SetTransform("base_footprint", "base_link", 0.0, true);
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.72, false);
  time_ns_ -= 100000000;
  EXPECT_THROW(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeForward(controller_, 0.15),
               nav2_core::ControllerException);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ForwardBlocked(controller_, 0.15));
}

TEST_F(FtcSpeedLimit, StaleSupportedPoseRejectsBothForwardGuards)
{
  time_ns_ += 1000000000;
  SetTime();
  EXPECT_THROW(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeForward(controller_, 0.15),
               nav2_core::ControllerException);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ForwardBlocked(controller_, 0.15));
}

TEST_F(FtcSpeedLimit, LatestCompletePastChainIsAcceptedWhenEdgesAreAsynchronous)
{
  tf_->clear();
  SetTransform("base_footprint", "base_link", 0.0, true);
  time_ns_ -= 50000000;
  SetTransform("map", "odom", 0.0, false);
  SetTransform("odom", "base_footprint", 0.70, false);
  time_ns_ += 150000000;
  SetTransform("odom", "base_footprint", 0.72, false);
  time_ns_ -= 100000000;
  EXPECT_NO_THROW(
      mowgli_nav2_plugins::FTCSpeedLimitTestAccess::AuthorizeForward(controller_, 0.15));
}

TEST_F(FtcRequestedGoalTolerance, PostRotateUsesSupportedYawForStrictRequest)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  WaitAtFinalCarrot(0.78, checker);
  mowgli_nav2_plugins::FTCSpeedLimitTestAccess::EnterPostRotate(controller_);
  SetTransform("odom", "base_footprint", 0.78, false, 0.0, 0.4);
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.78, false, 0.0, 0.0);
  time_ns_ -= 100000000;
  Tick(0.0, &checker);
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Finished(controller_));
}

TEST_F(FtcRequestedGoalTolerance, EqualToleranceEntryInsideCapDoesNotAddForwardCommand)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  checker.x = checker.y = 0.50;
  robot_x_ = 0.70;
  measured_speed_ = 0.0;
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i <= 16; ++i)
    plan.poses.push_back(MakePose(0.05 * i));
  controller_.newPathReceived(plan);
  for (int i = 0; i < 100; ++i)
  {
    const double command = Tick(0.10, &checker);
    if (mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ApproachingGoal(controller_))
    {
      EXPECT_DOUBLE_EQ(command, 0.0);
      return;
    }
  }
  FAIL() << "Controller did not enter its timed terminal approach";
}

TEST_F(FtcRequestedGoalTolerance, EqualToleranceApproachesFromSupportedCurrentPosition)
{
  SetParameter("max_goal_distance_error", 0.50);
  RequestedGoalTolerance checker;
  checker.x = checker.y = 0.50;
  WaitAtFinalCarrot(0.70, checker);
  time_ns_ += 100000000;
  SetTime();
  robot_x_ = 0.27;
  SetTransform("odom", "base_footprint", robot_x_, false);
  time_ns_ += 100000000;
  SetTransform("odom", "base_footprint", 0.32, false);
  time_ns_ -= 100000000;
  EXPECT_GT(Tick(0.0, &checker), 0.0);
  EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ApproachingGoal(controller_));
}

TEST_F(FtcSpeedLimit, ForwardLethalSweepUsesCompleteMapBaseEpoch)
{
  Wall(1.36);
  tf_->clear();
  SetTransform("base_footprint", "base_link", 0.0, true);
  time_ns_ -= 50000000;
  SetTransform("map", "odom", 0.0, false);
  SetTransform("odom", "base_footprint", 0.70, false);
  time_ns_ += 150000000;
  SetTransform("odom", "base_footprint", 0.76, false);
  time_ns_ -= 100000000;
  EXPECT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::ForwardBlocked(controller_, 0.15));
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

TEST_F(FtcSpeedLimit, PreRotatePreservesRecoveryWhenOnlyTheBodyResponseIsBlocked)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  namespace motion = mowgli_interfaces::motion;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_x_ = 2.85;
  measured_speed_ = 0.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  controller_.newPathReceived(StraightPlan(robot_x_));
  Wall(3.45);
  visualization_msgs::msg::MarkerArray markers;
  const auto add =
      [&](const std::string& kind, const std::vector<std::pair<double, double>>& points)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = kind;
    marker.text = kind == "area" ? "mowing" : "";
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.pose.orientation.w = 1;
    for (const auto& [x, y] : points)
    {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  };
  add("area", {{-5, -5}, {15, -5}, {15, 15}, {-5, 15}});
  add("obstacle", {{3.45, -1.5}, {3.50, -1.5}, {3.50, 2}, {3.45, 2}});
  Access::SetAuthorization(controller_, markers);
  const auto geometry = motion::Snapshot::parse(markers);
  ASSERT_TRUE(geometry->valid);
  motion::Ring footprint;
  for (const auto& point : costmap_->getRobotFootprint())
    footprint.push_back({point.x, point.y});
  geometry_msgs::msg::Twist restart;
  restart.linear.x = .16;
  const motion::Pose pose{robot_x_, 0, 0};
  EXPECT_TRUE(mowgli_nav2_plugins::ForwardResponseReferencePermitted(
      *geometry, footprint, pose, restart, {}, .1, .5));
  EXPECT_FALSE(mowgli_nav2_plugins::ForwardResponsePermitted(
      *geometry, footprint, pose, restart, {}, .1, .5));
  EXPECT_THROW(Access::AuthorizeForward(controller_, .16), nav2_core::ControllerException);
  bool reversed = false;
  for (int tick = 0; tick < 15; ++tick)
  {
    const double command = Tick();
    EXPECT_LE(command, 0);
    if (command < 0)
    {
      reversed = true;
      break;
    }
  }
  EXPECT_TRUE(reversed) << "PRE must preserve the existing FOLLOWING reverse-recovery path";
}

TEST_F(FtcRequestedGoalTolerance, CompletedPivotPreservesBlockedDepartureRecovery)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  namespace motion = mowgli_interfaces::motion;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses = {MakePose(-.02, -.1, M_PI / 2), MakePose(-.02, 0, M_PI / 2), MakePose(-.02, 0, 0)};
  for (double x = .03; x < 4; x += .05)
    plan.poses.push_back(MakePose(x));
  controller_.newPathReceived(plan);
  RequestedGoalTolerance checker;
  // Enter the pivot through normal pose updates, before any obstacle exists.
  double approach_y = -.1;
  geometry_msgs::msg::Twist measured;
  for (int tick = 0; tick < 100 && !Access::Pivoting(controller_); ++tick)
  {
    measured = StepAt(-.02, approach_y, M_PI / 2, measured, checker);
    approach_y += measured.linear.x * .1;
  }
  ASSERT_TRUE(Access::Pivoting(controller_));
  // The wall is outside the complete padded rotation disc (0.62 > 0.611 m),
  // but inside the forward restart response. Only FOLLOWING owns its recovery.
  Wall(.60);
  visualization_msgs::msg::MarkerArray markers;
  const auto add =
      [&](const std::string& kind, const std::vector<std::pair<double, double>>& points)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = kind;
    marker.text = kind == "area" ? "mowing" : "";
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.pose.orientation.w = 1;
    for (const auto& [x, y] : points)
    {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  };
  add("area", {{-5, -5}, {15, -5}, {15, 15}, {-5, 15}});
  add("obstacle", {{.60, -1.5}, {.65, -1.5}, {.65, 2}, {.60, 2}});
  Access::SetAuthorization(controller_, markers);
  SetTransform("odom", "base_footprint", -.02, false, 0, 0);
  EXPECT_FALSE(Access::RotationBlocked(controller_, M_PI / 2));
  EXPECT_THROW(Access::AuthorizeForward(controller_, .16), nav2_core::ControllerException);
  bool reversed = false;
  for (int tick = 0; tick < 15; ++tick)
  {
    const auto command = StepAt(-.02, 0, 0, {}, checker);
    EXPECT_LE(command.linear.x, 0);
    if (command.linear.x < 0)
    {
      reversed = true;
      break;
    }
  }
  EXPECT_FALSE(Access::Pivoting(controller_));
  EXPECT_TRUE(reversed);
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
