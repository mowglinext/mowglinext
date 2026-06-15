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

#include "mowgli_behavior/docking_nodes.hpp"

#include <cmath>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/exceptions.h"

namespace mowgli_behavior
{

namespace
{
/// Set a PoseStamped orientation to a planar yaw (radians).
void setYaw(geometry_msgs::msg::PoseStamped& ps, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();
}
}  // namespace

// ---------------------------------------------------------------------------
// DockApproach
// ---------------------------------------------------------------------------

BT::NodeStatus DockApproach::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Members hold the defaults; only overwrite when a port is supplied.
  if (auto r = getInput<double>("lstage"))
    lstage_ = r.value();
  if (auto r = getInput<double>("overshoot"))
    overshoot_ = r.value();
  if (auto r = getInput<double>("step"))
    step_ = r.value();
  if (auto r = getInput<double>("arrival_tol"))
    arrival_tol_ = r.value();

  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    dock_x_ = ctx->dock_x;
    dock_y_ = ctx->dock_y;
    dock_yaw_ = ctx->dock_yaw;
  }

  // A zero dock pose means it was never injected from mowgli_robot.yaml — refuse
  // rather than drive the robot toward the map origin.
  if (dock_x_ == 0.0 && dock_y_ == 0.0)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "DockApproach: dock pose is unset (0,0) — check dock_pose_x/y/yaw params");
    return BT::NodeStatus::FAILURE;
  }

  if (!plan_client_)
  {
    plan_client_ = rclcpp_action::create_client<ComputePath>(ctx->node, "/compute_path_to_pose");
  }
  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<FollowPath>(ctx->node, "/follow_path");
  }
  if (!plan_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockApproach: /compute_path_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  // Staging pose: lstage back along the dock axis (-û), facing the dock yaw.
  const double ux = std::cos(dock_yaw_);
  const double uy = std::sin(dock_yaw_);

  ComputePath::Goal goal;
  goal.use_start = false;  // plan from the robot's current pose
  // Reeds-Shepp Hybrid-A*, NOT the transit "GridBased" (DUBIN). The dock staging
  // pose sits in the tight, obstacle-adjacent dock area where forward-only Dubins
  // cannot set up the approach ("no valid path", field 2026-06-10); R-S may
  // reverse to reach it. GridBasedRS is the dedicated R-S planner (nav2_params).
  goal.planner_id = "GridBasedRS";
  goal.goal.header.frame_id = "map";
  goal.goal.header.stamp = ctx->node->now();
  goal.goal.pose.position.x = dock_x_ - lstage_ * ux;
  goal.goal.pose.position.y = dock_y_ - lstage_ * uy;
  setYaw(goal.goal, dock_yaw_);

  plan_goal_handle_.reset();
  plan_result_requested_ = false;
  phase_ = Phase::Planning;
  plan_goal_future_ = plan_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockApproach: planning to staging (%.2f,%.2f)@%.0f° for dock (%.2f,%.2f)",
              goal.goal.pose.position.x,
              goal.goal.pose.position.y,
              dock_yaw_ * 180.0 / M_PI,
              dock_x_,
              dock_y_);
  return BT::NodeStatus::RUNNING;
}

nav_msgs::msg::Path DockApproach::buildConcatPath(const nav_msgs::msg::Path& smac_plan) const
{
  const double ux = std::cos(dock_yaw_);
  const double uy = std::sin(dock_yaw_);
  const double sx = dock_x_ - lstage_ * ux;  // staging
  const double sy = dock_y_ - lstage_ * uy;
  const double tx = dock_x_ + overshoot_ * ux;  // overshoot target past the dock
  const double ty = dock_y_ + overshoot_ * uy;
  const double dist = std::hypot(tx - sx, ty - sy);
  const int n = std::max(1, static_cast<int>(dist / step_));

  nav_msgs::msg::Path full;
  full.header.frame_id = "map";
  full.poses = smac_plan.poses;
  // Append the straight tail, skipping i=0 (it coincides with the Smac goal).
  for (int i = 1; i <= n; ++i)
  {
    const double t = static_cast<double>(i) / n;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = sx + (tx - sx) * t;
    ps.pose.position.y = sy + (ty - sy) * t;
    setYaw(ps, dock_yaw_);
    full.poses.push_back(ps);
  }
  return full;
}

bool DockApproach::nearDock(const std::shared_ptr<BTContext>& ctx) const
{
  try
  {
    const auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const double dx = tf.transform.translation.x - dock_x_;
    const double dy = tf.transform.translation.y - dock_y_;
    return std::hypot(dx, dy) <= arrival_tol_;
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockApproach: TF lookup failed: %s", e.what());
    return false;
  }
}

BT::NodeStatus DockApproach::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (phase_ == Phase::Planning)
  {
    // 1. Wait for the plan goal handle.
    if (!plan_goal_handle_)
    {
      if (plan_goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      plan_goal_handle_ = plan_goal_future_.get();
      if (!plan_goal_handle_)
      {
        RCLCPP_ERROR(ctx->node->get_logger(), "DockApproach: plan goal rejected");
        return BT::NodeStatus::FAILURE;
      }
    }
    // 2. Request and wait for the plan result.
    if (!plan_result_requested_)
    {
      plan_result_future_ = plan_client_->async_get_result(plan_goal_handle_);
      plan_result_requested_ = true;
    }
    if (plan_result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    const auto wrapped = plan_result_future_.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result->path.poses.empty())
    {
      RCLCPP_WARN(ctx->node->get_logger(), "DockApproach: planning failed (no path to staging)");
      return BT::NodeStatus::FAILURE;
    }
    // 3. Concatenate the straight dock-aligned tail and start following.
    const auto full = buildConcatPath(wrapped.result->path);
    if (!follow_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(ctx->node->get_logger(), "DockApproach: /follow_path not available");
      return BT::NodeStatus::FAILURE;
    }
    FollowPath::Goal fgoal;
    fgoal.path = full;
    fgoal.controller_id = "FollowPath";
    fgoal.goal_checker_id = "stopped_goal_checker";
    follow_goal_handle_.reset();
    follow_goal_future_ = follow_client_->async_send_goal(fgoal);
    phase_ = Phase::Following;
    RCLCPP_INFO(ctx->node->get_logger(),
                "DockApproach: following %zu-pose path (Smac transit + %.2fm straight tail)",
                full.poses.size(),
                lstage_ + overshoot_);
    return BT::NodeStatus::RUNNING;
  }

  // Phase::Following
  if (!follow_goal_handle_)
  {
    if (follow_goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    follow_goal_handle_ = follow_goal_future_.get();
    if (!follow_goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "DockApproach: follow goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = follow_goal_handle_->get_status();
  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockApproach: follow succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      // FTC ABORTs stalled against the cradle — the expected seat. Accept it
      // only when the robot actually reached the dock; the BT's IsCharging
      // check then confirms the electrical contact.
      if (nearDock(ctx))
      {
        RCLCPP_INFO(ctx->node->get_logger(),
                    "DockApproach: follow aborted at the dock (seated, momentum stall) — OK");
        return BT::NodeStatus::SUCCESS;
      }
      RCLCPP_WARN(ctx->node->get_logger(), "DockApproach: follow aborted away from the dock");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockApproach::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (plan_goal_handle_)
  {
    plan_client_->async_cancel_goal(plan_goal_handle_);
    plan_goal_handle_.reset();
  }
  if (follow_goal_handle_)
  {
    follow_client_->async_cancel_goal(follow_goal_handle_);
    follow_goal_handle_.reset();
  }
  RCLCPP_INFO(ctx->node->get_logger(), "DockApproach: halted, goals canceled");
}

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_id = "home_dock";
  if (auto res = getInput<std::string>("dock_id"))
  {
    dock_id = res.value();
  }

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<DockAction>(ctx->node, "/dock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: /dock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  DockAction::Goal goal_msg;
  goal_msg.dock_id = dock_id;
  goal_msg.dock_type = dock_type;
  goal_msg.navigate_to_staging_pose = true;

  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: goal sent (dock_id='%s', dock_type='%s')",
              dock_id.c_str(),
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "DockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

BT::NodeStatus UndockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  std::string dock_type = "simple_charging_dock";
  if (auto res = getInput<std::string>("dock_type"))
  {
    dock_type = res.value();
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<UndockAction>(ctx->node, "/undock_robot");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: /undock_robot action server not available");
    return BT::NodeStatus::FAILURE;
  }

  UndockAction::Goal goal_msg;
  goal_msg.dock_type = dock_type;

  auto send_goal_options = rclcpp_action::Client<UndockAction>::SendGoalOptions{};
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "UndockRobot: goal sent (dock_type='%s')",
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "UndockRobot: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: undocking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void UndockRobot::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

BT::NodeStatus RecordResumeUndockFailure::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->resume_undock_failures++;
  RCLCPP_WARN(ctx->node->get_logger(),
              "RecordResumeUndockFailure: resume undock failures = %d",
              ctx->resume_undock_failures);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
