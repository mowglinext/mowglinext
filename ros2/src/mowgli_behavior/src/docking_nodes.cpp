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
#include "mowgli_behavior/cancel_goal.hpp"
#include "mowgli_behavior/dock_alignment.hpp"

namespace mowgli_behavior
{
namespace
{

/// rclcpp_action's result code -> our ROS-free outcome (action_outcome.hpp).
GoalOutcome OutcomeFromResultCode(const rclcpp_action::ResultCode code)
{
  switch (code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return GoalOutcome::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      return GoalOutcome::kCanceled;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      return GoalOutcome::kAborted;
  }
}

// The pure helper repeats the action_msgs constants; keep them honest.
static_assert(kGoalStatusSucceeded == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
static_assert(kGoalStatusAborted == action_msgs::msg::GoalStatus::STATUS_ABORTED);
static_assert(kGoalStatusCanceled == action_msgs::msg::GoalStatus::STATUS_CANCELED);
static_assert(kGoalStatusAccepted == action_msgs::msg::GoalStatus::STATUS_ACCEPTED);
static_assert(kGoalStatusExecuting == action_msgs::msg::GoalStatus::STATUS_EXECUTING);

}  // namespace

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

DockRobot::~DockRobot()
{
  if (dispatch_)
  {
    dispatch_->Retire();
  }
}

void DockRobot::log_contact_delta(const std::shared_ptr<BTContext>& ctx, uint16_t num_retries)
{
  const auto d =
      ComputeDockContactDelta(ctx->gps_x, ctx->gps_y, ctx->dock_x, ctx->dock_y, ctx->dock_yaw);

  RCLCPP_INFO(ctx->node->get_logger(),
              "DockRobot: contact claimed (retry %u) at (%.3f, %.3f) vs dock "
              "(%.3f, %.3f, %.2f°) — along=%+.3f m cross=%+.3f m range=%.3f m",
              num_retries,
              ctx->gps_x,
              ctx->gps_y,
              ctx->dock_x,
              ctx->dock_y,
              ctx->dock_yaw * 180.0 / M_PI,
              d.along_m,
              d.cross_m,
              d.range_m);
}

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->last_dock_succeeded = false;

  // A guard can halt us before the server accepts the previous request. Do
  // not preempt that request (or its staging navigator) with a replacement.
  if (dispatch_)
  {
    dispatch_->Retire();
    if (!dispatch_->Finished())
    {
      waiting_for_retired_ = true;
      retirement_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      ctx->docking_active = false;
      return BT::NodeStatus::RUNNING;
    }
  }
  return sendGoal();
}

BT::NodeStatus DockRobot::sendGoal()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  waiting_for_retired_ = false;

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

  dispatch_ =
      std::make_shared<DockingGoalDispatch<DockAction>>(action_client_, ctx->node->get_logger());
  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  // Keep the client alive only until acceptance, even if the tree is destroyed
  // first. The dispatch itself holds a weak client, avoiding a callback cycle.
  send_goal_options.goal_response_callback =
      [dispatch = dispatch_, client = action_client_](const GoalHandle::SharedPtr& handle)
  {
    dispatch->Accept(client, handle);
  };
  send_goal_options.feedback_callback =
      [dispatch = dispatch_, ctx](GoalHandle::SharedPtr,
                                  const std::shared_ptr<const DockAction::Feedback> fb)
  {
    // Log once on each ENTRY into WAIT_FOR_CHARGE — the server re-enters it on
    // every retry, and each entry is a separate claimed contact worth recording.
    if (fb && dispatch->FeedbackChanged(fb->state) &&
        fb->state == DockAction::Feedback::WAIT_FOR_CHARGE)
    {
      log_contact_delta(ctx, fb->num_retries);
    }
  };
  // Ask for the RESULT as well. Setting this callback makes rclcpp_action
  // request the result the moment the goal is accepted, which is the only
  // signal that cannot be lost when the server finishes a goal in the same
  // instant it accepts it — "Robot is already docked, no need to dock" left a
  // polled handle stuck on ACCEPTED for 11.5 h on 2026-09-17.
  send_goal_options.result_callback =
      [dispatch = dispatch_](const GoalHandle::WrappedResult& result)
  {
    dispatch->Record(OutcomeFromResultCode(result.code));
  };
  action_client_->async_send_goal(goal_msg, send_goal_options);

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

  if (waiting_for_retired_)
  {
    if (dispatch_->Finished())
    {
      return sendGoal();
    }
    if (std::chrono::steady_clock::now() >= retirement_deadline_)
    {
      RCLCPP_ERROR(
          ctx->node->get_logger(),
          "DockRobot: previous goal has not terminated after cancellation; refusing replacement");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  const auto snapshot = dispatch_->Get();
  if (!snapshot.responded)
  {
    return BT::NodeStatus::RUNNING;
  }
  if (!snapshot.handle)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "DockRobot: goal was rejected by the action server");
    ctx->docking_active = false;
    return BT::NodeStatus::FAILURE;
  }
  const auto status = ResolveGoalStatus(snapshot.handle->get_status(), snapshot.outcome);

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      dispatch_->Retire();
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      ctx->last_dock_succeeded = true;
      ctx->docking_active = false;
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      dispatch_->Retire();
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking aborted");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      dispatch_->Retire();
      RCLCPP_WARN(ctx->node->get_logger(), "DockRobot: docking canceled");
      ctx->docking_active = false;
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void DockRobot::onHalted()
{
  // Clear the dock-transit flag UNCONDITIONALLY (even if acceptance was not
  // yet confirmed): BehaviorTree.CPP invokes onHalted() whenever a RUNNING
  // DockRobot is halted by a parent (e.g. a new operator command, or the root
  // ReactiveSequence re-priority), and the flag must never survive that halt —
  // otherwise BoundaryGuard would stay exempted after the transit ended.
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->docking_active = false;
  if (dispatch_)
  {
    dispatch_->Retire();
  }
}

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

UndockRobot::~UndockRobot()
{
  if (dispatch_)
  {
    dispatch_->Retire();
  }
}

BT::NodeStatus UndockRobot::onStart()
{
  if (dispatch_)
  {
    dispatch_->Retire();
    if (!dispatch_->Finished())
    {
      waiting_for_retired_ = true;
      retirement_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      return BT::NodeStatus::RUNNING;
    }
  }
  return sendGoal();
}

BT::NodeStatus UndockRobot::sendGoal()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  waiting_for_retired_ = false;

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
  dispatch_ =
      std::make_shared<DockingGoalDispatch<UndockAction>>(action_client_, ctx->node->get_logger());
  send_goal_options.goal_response_callback =
      [dispatch = dispatch_, client = action_client_](const GoalHandle::SharedPtr& handle)
  {
    dispatch->Accept(client, handle);
  };
  // See DockRobot::onStart — the terminal status can be lost, the result cannot.
  send_goal_options.result_callback =
      [dispatch = dispatch_](const GoalHandle::WrappedResult& result)
  {
    dispatch->Record(OutcomeFromResultCode(result.code));
  };
  action_client_->async_send_goal(goal_msg, send_goal_options);

  RCLCPP_INFO(ctx->node->get_logger(),
              "UndockRobot: goal sent (dock_type='%s')",
              dock_type.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus UndockRobot::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (waiting_for_retired_)
  {
    if (dispatch_->Finished())
    {
      return sendGoal();
    }
    if (std::chrono::steady_clock::now() >= retirement_deadline_)
    {
      RCLCPP_ERROR(
          ctx->node->get_logger(),
          "UndockRobot: previous goal has not terminated after cancellation; refusing replacement");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  const auto snapshot = dispatch_->Get();
  if (!snapshot.responded)
  {
    return BT::NodeStatus::RUNNING;
  }
  if (!snapshot.handle)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "UndockRobot: goal was rejected by the action server");
    return BT::NodeStatus::FAILURE;
  }
  const auto status = ResolveGoalStatus(snapshot.handle->get_status(), snapshot.outcome);

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      dispatch_->Retire();
      RCLCPP_INFO(ctx->node->get_logger(), "UndockRobot: undocking succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      dispatch_->Retire();
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      dispatch_->Retire();
      RCLCPP_WARN(ctx->node->get_logger(), "UndockRobot: undocking canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void UndockRobot::onHalted()
{
  if (dispatch_)
  {
    dispatch_->Retire();
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
