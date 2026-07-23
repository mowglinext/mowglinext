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

#include "action_msgs/msg/goal_status.hpp"

namespace mowgli_behavior
{

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

  // Mark the blade-off dock transit active ONLY on the RUNNING path (the
  // action-server-unavailable early return above leaves this false). Read by
  // IsDocking so BoundaryGuard exempts this transit — see bt_context.hpp.
  ctx->docking_active = true;
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
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      ctx->docking_active = false;
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  // Clear the dock-transit flag UNCONDITIONALLY (even if goal_handle_ was not
  // yet confirmed): BehaviorTree.CPP invokes onHalted() whenever a RUNNING
  // DockRobot is halted by a parent (e.g. a new operator command, or the root
  // ReactiveSequence re-priority), and the flag must never survive that halt —
  // otherwise BoundaryGuard would stay exempted after the transit ended.
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->docking_active = false;
  if (goal_handle_)
  {
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
