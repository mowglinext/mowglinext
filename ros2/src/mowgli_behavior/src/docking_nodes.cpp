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

#include <chrono>
#include <mutex>

#include "action_msgs/msg/goal_status.hpp"
#include "mowgli_behavior/cancel_goal.hpp"
#include "mowgli_behavior/dock_alignment.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

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

void DockRobot::log_contact_delta(const std::shared_ptr<BTContext>& ctx, uint16_t num_retries) const
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

bool RobotIsAtDockStation(BTContext& ctx)
{
  if (!ctx.tf_buffer)
  {
    return false;
  }
  double dock_x, dock_y, dock_yaw;
  DockStationZone zone;
  {
    std::lock_guard<std::mutex> lock(ctx.context_mutex);
    if (!ctx.dock_pose_known)
    {
      return false;
    }
    dock_x = ctx.dock_x;
    dock_y = ctx.dock_y;
    dock_yaw = ctx.dock_yaw;
    zone = ctx.dock_station_zone;
  }
  try
  {
    const auto tf = ctx.tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const auto& q = tf.transform.rotation;
    const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return IsFacingStationInNoPivotZone(tf.transform.translation.x,
                                        tf.transform.translation.y,
                                        yaw,
                                        dock_x,
                                        dock_y,
                                        dock_yaw,
                                        zone);
  }
  catch (const tf2::TransformException&)
  {
    return false;  // no pose: the ordinary paths apply
  }
}

BT::NodeStatus IsAtDockStation::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return RobotIsAtDockStation(*ctx) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void DockRobot::clearStaleNavigation(const std::shared_ptr<BTContext>& ctx)
{
  // Field 2026-10-09 (issue #943): a transit goal nobody owned any more kept bt_navigator busy,
  // docking_server's staging leg was refused ("Goal handle is not known to this client", error
  // 999) and the mower stayed on the lawn. Nothing else may be navigating once the tree asks to
  // dock: the node that sent such a goal has been halted or has moved on. Only goals accepted
  // BEFORE this instant are cancelled, so the staging goal docking_server sends for this request
  // is never touched, however late the navigator handles the cancel.
  bool ready = false;
  if (!nav_client_)
  {
    nav_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ctx->node,
                                                                        "/navigate_to_pose");
    // A client created this instant has not discovered the server yet: the session's first
    // dock is exactly when a stale goal matters. Later docks reuse the discovered client.
    ready = nav_client_->wait_for_action_server(kStaleNavigationDiscoveryWait);
  }
  else
  {
    ready = nav_client_->action_server_is_ready();
  }
  if (!ready)
  {
    return;  // no navigator, nothing that could be stale
  }
  if (cancelGoalsBeforeQuietly(nav_client_, ctx->node->now(), ctx->node->get_logger(), "DockRobot"))
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "DockRobot: cancelled any navigate_to_pose goal still running before the dock "
                "request");
  }
}

BT::NodeStatus DockRobot::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->last_dock_succeeded = false;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    if (ctx->latest_power.charger_enabled)
    {
      ctx->last_dock_succeeded = true;
      return BT::NodeStatus::SUCCESS;
    }
  }

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
  // Already on the approach line, too close to pivot: the transit planner would
  // refuse the leg to the staging pose, and it is not needed. docking_server
  // drives straight in and, if that fails, reverses to staging for its retry.
  const bool at_station = RobotIsAtDockStation(*ctx);
  goal_msg.navigate_to_staging_pose = !at_station;
  if (at_station)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "DockRobot: already at the station without charge, skipping the staging leg");
  }

  last_feedback_state_ = DockAction::Feedback::NONE;
  stall_detector_.Reset();

  auto send_goal_options = rclcpp_action::Client<DockAction>::SendGoalOptions{};
  goal_abandonment_ = std::make_shared<GoalAbandonment>();
  send_goal_options.goal_response_callback =
      cancelWhenAbandoned<DockAction>(action_client_, goal_abandonment_);
  send_goal_options.feedback_callback =
      [this, ctx](GoalHandle::SharedPtr, const std::shared_ptr<const DockAction::Feedback> fb)
  {
    // Log once on each ENTRY into WAIT_FOR_CHARGE — the server re-enters it on
    // every retry, and each entry is a separate claimed contact worth recording.
    const uint16_t prev = last_feedback_state_.exchange(fb->state);
    if (fb->state == DockAction::Feedback::WAIT_FOR_CHARGE &&
        prev != DockAction::Feedback::WAIT_FOR_CHARGE)
    {
      log_contact_delta(ctx, fb->num_retries);
    }
  };
  // Ask for the RESULT as well. Setting this callback makes rclcpp_action
  // request the result the moment the goal is accepted, which is the only
  // signal that cannot be lost when the server finishes a goal in the same
  // instant it accepts it — "Robot is already docked, no need to dock" left a
  // polled handle stuck on ACCEPTED for 11.5 h on 2026-09-17.
  // A halted attempt can deliver its result after this attempt starts. Keep
  // its callback's slot separate so a late CANCELED cannot fail the new goal.
  outcome_ = std::make_shared<ActionOutcomeSlot>();
  send_goal_options.result_callback =
      [slot = outcome_, logger = ctx->node->get_logger(), dock_id, dock_type](
          const GoalHandle::WrappedResult& result)
  {
    slot->Record(OutcomeFromResultCode(result.code));
    // Log from the result callback: the BT may already be halted or following
    // another dock goal when this result arrives. Capture this request's
    // identity and logger by value, without extending the BT node's lifetime.
    const char* status = result.code == rclcpp_action::ResultCode::SUCCEEDED  ? "SUCCEEDED"
                         : result.code == rclcpp_action::ResultCode::ABORTED  ? "ABORTED"
                         : result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED"
                                                                              : "UNKNOWN";
    const auto error_code =
        result.result ? std::to_string(result.result->error_code) : "unavailable";
    const auto error_msg = result.result ? result.result->error_msg : "result payload missing";
    const auto severity = result.code == rclcpp_action::ResultCode::SUCCEEDED ||
                                  result.code == rclcpp_action::ResultCode::CANCELED
                              ? RCUTILS_LOG_SEVERITY_INFO
                              : RCUTILS_LOG_SEVERITY_WARN;
    RCLCPP_LOG(severity,
               logger,
               "DockRobot: result status=%s dock_id='%s' dock_type='%s' goal_id=%s "
               "error_code=%s error_msg='%s'",
               status,
               dock_id.c_str(),
               dock_type.c_str(),
               rclcpp_action::to_string(result.goal_id).c_str(),
               error_code.c_str(),
               error_msg.c_str());
  };
  clearStaleNavigation(ctx);
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
  bool charging;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    charging = ctx->latest_power.charger_enabled;
  }
  if (charging)
  {
    onHalted();
    ctx->last_dock_succeeded = true;
    return BT::NodeStatus::SUCCESS;
  }

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

  const auto status = ResolveGoalStatus(goal_handle_->get_status(), outcome_->Get());

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "DockRobot: docking succeeded");
      ctx->last_dock_succeeded = true;
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
      if (approach_stalled(ctx))
      {
        // The server is still driving the approach but the chassis has not moved: something
        // physical is in the way (field 2026-10-06: the garage edge over the dock, 0.29 m
        // short, pushed against for 30 s). Stop and report; do NOT let the server retry, a
        // retry would drive into the same edge. FAILURE takes the tree's existing dock-failure
        // path, which publishes NAV_TO_DOCK_FAILED and so raises the "blocked" notification.
        RCLCPP_WARN(ctx->node->get_logger(),
                    "DockRobot: DOCK BLOCKED — the dock approach has not moved or turned for "
                    "%.0f s while the dock server is controlling it and the mower is not "
                    "charging (an obstacle such as the garage edge is in the way at fused "
                    "pose %.2f, %.2f). Cancelling the dock attempt, no retry",
                    kDockStallWindowS,
                    ctx->fused_x,
                    ctx->fused_y);
        cancelGoalQuietly(action_client_, goal_handle_, ctx->node->get_logger(), "DockRobot");
        goal_handle_.reset();
        ctx->docking_active = false;
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
  }
}

bool DockRobot::approach_stalled(const std::shared_ptr<BTContext>& ctx)
{
  // Only while the server is actually driving the final approach. NAV_TO_STAGING has its own
  // Nav2 recovery, and WAIT_FOR_CHARGE legitimately stands still waiting for the charger.
  if (last_feedback_state_.load() != DockAction::Feedback::CONTROLLING)
  {
    stall_detector_.Reset();
    return false;
  }
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  const auto now = std::chrono::steady_clock::now();
  const double pose_age_s = std::chrono::duration<double>(now - ctx->fused_pose_time).count();
  // Charging means the contacts are made: that is a dock, not a blockage.
  if (ctx->latest_power.charger_enabled || !ctx->fused_pose_valid ||
      pose_age_s > kDockStallMaxPoseAgeS)
  {
    stall_detector_.Reset();
    return false;
  }
  const double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
  return stall_detector_.Update(now_s, ctx->fused_x, ctx->fused_y, ctx->fused_yaw);
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
  // A reactive charger fast path can halt us before the goal response arrives.
  // Abandon that pending request too, so it cannot start navigating later.
  abandonGoal(action_client_,
              goal_handle_,
              goal_handle_future_,
              goal_abandonment_,
              ctx->node->get_logger(),
              "DockRobot");
  goal_handle_.reset();
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
  // See DockRobot::onStart: request the result, with a fresh slot per attempt
  // so a halted attempt's late callback cannot overwrite this goal's outcome.
  outcome_ = std::make_shared<ActionOutcomeSlot>();
  send_goal_options.result_callback = [slot = outcome_](const GoalHandle::WrappedResult& result)
  {
    slot->Record(OutcomeFromResultCode(result.code));
  };
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

  const auto status = ResolveGoalStatus(goal_handle_->get_status(), outcome_->Get());

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
    cancelGoalQuietly(action_client_, goal_handle_, ctx->node->get_logger(), "UndockRobot");
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
