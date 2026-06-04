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

#include "mowgli_behavior/navigation_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/// Parse a pose string "x;y;yaw" and fill a PoseStamped (frame_id = "map").
geometry_msgs::msg::PoseStamped parsePoseString(const std::string& pose_str,
                                                const rclcpp::Node::SharedPtr& node)
{
  std::istringstream ss(pose_str);
  std::string token;
  double x = 0.0, y = 0.0, yaw = 0.0;

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'x' in goal string");
  }
  x = std::stod(token);

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'y' in goal string");
  }
  y = std::stod(token);

  if (!std::getline(ss, token, ';'))
  {
    throw std::invalid_argument("NavigateToPose: missing 'yaw' in goal string");
  }
  yaw = std::stod(token);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

}  // namespace

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

void StopMoving::publish_zero(const rclcpp::Node::SharedPtr& node)
{
  geometry_msgs::msg::TwistStamped zero{};
  zero.header.stamp = node->now();
  zero.header.frame_id = "base_footprint";
  pub_->publish(zero);
}

BT::NodeStatus StopMoving::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!pub_)
  {
    pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_emergency", 10);
  }

  duration_sec_ = 0.5;
  getInput("duration_sec", duration_sec_);
  start_time_ = ctx->node->now();

  publish_zero(ctx->node);
  RCLCPP_INFO(ctx->node->get_logger(),
              "StopMoving: streaming zero velocity for %.2fs",
              duration_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopMoving::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  publish_zero(ctx->node);

  const double elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed >= duration_sec_)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void StopMoving::onHalted()
{
  // Nothing to cancel — publisher is fire-and-forget.
}

// ---------------------------------------------------------------------------
// ClearCostmap
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCostmap::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!global_client_)
  {
    global_client_ = ctx->node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");
  }
  if (!local_client_)
  {
    local_client_ = ctx->node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/local_costmap/clear_entirely_local_costmap");
  }

  // Nav2's clear_entirely_* services use nav2_msgs/ClearEntireCostmap, NOT
  // std_srvs/Empty. An earlier version of this node used Empty which
  // silently failed at the DDS type-match stage — ClearCostmap returned
  // SUCCESS but the costmap was never actually cleared, leaving stale
  // obstacle marks (observed on the 2026-04-24 'Start occupied' loop).
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  // Just send the requests. If the service isn't ready, async_send_request
  // will fail silently (no response). This avoids DDS discovery issues
  // where service_is_ready() and wait_for_service() never return true
  // even though the services exist (Cyclone DDS on ARM).
  global_client_->async_send_request(request);
  local_client_->async_send_request(request);
  RCLCPP_INFO(ctx->node->get_logger(), "ClearCostmap: sent clear requests");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

void NavigateToPose::ensureActionClient(const rclcpp::Node::SharedPtr& node)
{
  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<Nav2Goal>(node, "/navigate_to_pose");
  }
}

BT::NodeStatus NavigateToPose::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto goal_res = getInput<std::string>("goal");
  if (!goal_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "NavigateToPose: missing required port 'goal': %s",
                 goal_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  try
  {
    target_pose = parsePoseString(goal_res.value(), ctx->node);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "NavigateToPose: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  ensureActionClient(ctx->node);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateToPose: action server '/navigate_to_pose' not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Goal::Goal goal_msg;
  goal_msg.pose = target_pose;

  auto send_goal_options = rclcpp_action::Client<Nav2Goal>::SendGoalOptions{};

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_handle_.reset();

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateToPose: goal sent (x=%.2f y=%.2f yaw=%.2f)",
              target_pose.pose.position.x,
              target_pose.pose.position.y,
              0.0 /* yaw logged for info, already in quaternion */);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Resolve the goal handle the first time it is ready.
  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "NavigateToPose: goal was rejected by the action server");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = goal_handle_->get_status();

  switch (status)
  {
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(), "NavigateToPose: goal succeeded");
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToPose: goal aborted");
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(ctx->node->get_logger(), "NavigateToPose: goal canceled");
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void NavigateToPose::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateToPose: canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

// ---------------------------------------------------------------------------
// NavigateInsideBoundary
// ---------------------------------------------------------------------------

void NavigateInsideBoundary::ResetState()
{
  service_future_ = {};
  set_param_future_ = {};
  clear_future_ = {};
  goal_handle_future_ = {};
  goal_handle_.reset();
  backup_goal_handle_future_ = {};
  backup_goal_handle_.reset();
  backup_result_future_ = {};
  backup_result_requested_ = false;
  recovery_pose_ = geometry_msgs::msg::Pose{};
  distance_outside_ = 0.0;
  pending_nav_result_ = BT::NodeStatus::FAILURE;
  keepout_disabled_ = false;
  fallback_attempted_ = false;
  phase_ = Phase::WaitingForService;
}

void NavigateInsideBoundary::RequestKeepoutEnable(bool enabled)
{
  // Fire-and-forget: drop the returned future on the floor. AsyncParametersClient
  // posts the request to the executor, so the parameter change goes out even
  // when this BT node is being halted / destroyed shortly after.
  if (!keepout_params_client_)
    return;
  if (!keepout_params_client_->service_is_ready())
    return;
  (void)keepout_params_client_->set_parameters(
      {rclcpp::Parameter("keepout_filter.enabled", enabled)});
}

BT::NodeStatus NavigateInsideBoundary::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!service_client_)
  {
    service_client_ = ctx->node->create_client<RecoverySrv>("/map_server_node/get_recovery_point");
  }
  if (!clear_client_)
  {
    clear_client_ =
        ctx->node->create_client<ClearSrv>("/global_costmap/clear_entirely_global_costmap");
  }
  if (!keepout_params_client_)
  {
    keepout_params_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(ctx->node,
                                                        "/global_costmap/global_costmap");
  }
  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<Nav2Goal>(ctx->node, "/navigate_to_pose");
  }
  if (!backup_action_client_)
  {
    backup_action_client_ = rclcpp_action::create_client<BackUpAction>(ctx->node, "/backup");
  }

  if (!service_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "NavigateInsideBoundary: /map_server_node/get_recovery_point unavailable");
    return BT::NodeStatus::FAILURE;
  }

  ResetState();
  service_future_ =
      service_client_->async_send_request(std::make_shared<RecoverySrv::Request>()).share();

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateInsideBoundary: requesting recovery pose from map server");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateInsideBoundary::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Phase 1: service response for the recovery pose.
  if (phase_ == Phase::WaitingForService)
  {
    if (service_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    auto resp = service_future_.get();
    if (!resp || !resp->success)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateInsideBoundary: recovery pose request failed: %s",
                  resp ? resp->message.c_str() : "null response");
      return BT::NodeStatus::FAILURE;
    }

    recovery_pose_ = resp->recovery_pose;
    distance_outside_ = resp->distance_outside;

    // Disable the global_costmap's keepout filter so Smac can plan from
    // the robot's current cell (which is in the keepout-lethal zone —
    // that's why we triggered recovery in the first place). If the
    // parameter service isn't ready, skip and try the Nav2 leg anyway;
    // worst case Smac aborts with "Start occupied" and the outer
    // RetryUntilSuccessful escalates to the lethal handler.
    if (!keepout_params_client_->service_is_ready())
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateInsideBoundary: global_costmap params service not ready — "
                  "proceeding without disabling keepout filter");
      phase_ = Phase::WaitingForGoalHandle;
      return SendNav2Goal();
    }
    set_param_future_ = keepout_params_client_->set_parameters(
        {rclcpp::Parameter("keepout_filter.enabled", false)});
    keepout_disabled_ = true;
    phase_ = Phase::DisablingKeepout;
    RCLCPP_INFO(ctx->node->get_logger(),
                "NavigateInsideBoundary: disabling global_costmap keepout_filter "
                "(recovery target=(%.2f, %.2f), %.2fm outside)",
                recovery_pose_.position.x, recovery_pose_.position.y, distance_outside_);
    return BT::NodeStatus::RUNNING;
  }

  // Phase 2: set_parameters acknowledged → kick off costmap clear.
  if (phase_ == Phase::DisablingKeepout)
  {
    if (set_param_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    auto results = set_param_future_.get();
    if (results.empty() || !results.front().successful)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateInsideBoundary: set_parameters(keepout_filter.enabled=false) failed: %s",
                  results.empty() ? "empty response" : results.front().reason.c_str());
      // Continue anyway — Smac may still fail, but we tried.
      keepout_disabled_ = false;
    }

    // Clear the existing lethals stamped by the (now-disabled) keepout
    // filter. Without this, the master costmap still carries the old
    // lethal cells until the next costmap update sweeps them out.
    if (!clear_client_->service_is_ready())
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateInsideBoundary: clear_entirely_global_costmap not ready — "
                  "proceeding without clear");
      return SendNav2Goal();
    }
    clear_future_ =
        clear_client_->async_send_request(std::make_shared<ClearSrv::Request>()).share();
    phase_ = Phase::ClearingCostmap;
    return BT::NodeStatus::RUNNING;
  }

  // Phase 3: clear acknowledged → send Nav2 goal.
  if (phase_ == Phase::ClearingCostmap)
  {
    if (clear_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    (void)clear_future_.get();  // ClearEntireCostmap returns an empty response
    return SendNav2Goal();
  }

  // Phase 4: wait for the Nav2 goal handle.
  if (phase_ == Phase::WaitingForGoalHandle)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "NavigateInsideBoundary: nav2 rejected recovery goal — trying open-loop backup");
      pending_nav_result_ = BT::NodeStatus::FAILURE;
      if (!fallback_attempted_)
      {
        return BeginFallbackBackup();
      }
      return BeginReEnableKeepout();
    }
    phase_ = Phase::WaitingForResult;
  }

  // Phase 5: poll the Nav2 result.
  if (phase_ == Phase::WaitingForResult)
  {
    const auto status = goal_handle_->get_status();
    switch (status)
    {
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        RCLCPP_INFO(ctx->node->get_logger(), "NavigateInsideBoundary: recovery complete");
        pending_nav_result_ = BT::NodeStatus::SUCCESS;
        return BeginReEnableKeepout();
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        // Planner aborted (e.g. "Start occupied" because the robot's
        // current cell is still lethal after our clear). Fall back to an
        // open-loop reverse to physically pull the chassis off the
        // boundary edge — that breaks the deadlock for the outer
        // RetryUntilSuccessful, which will tick onStart() again from a
        // fresh (post-backup) pose. Only do this ONCE per onStart()
        // cycle so we don't spin in place.
        RCLCPP_WARN(ctx->node->get_logger(),
                    "NavigateInsideBoundary: nav2 aborted%s",
                    fallback_attempted_ ? "" : " — trying open-loop backup");
        pending_nav_result_ = BT::NodeStatus::FAILURE;
        if (!fallback_attempted_)
        {
          return BeginFallbackBackup();
        }
        return BeginReEnableKeepout();
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        RCLCPP_WARN(ctx->node->get_logger(), "NavigateInsideBoundary: nav2 canceled");
        pending_nav_result_ = BT::NodeStatus::FAILURE;
        return BeginReEnableKeepout();
      default:
        return BT::NodeStatus::RUNNING;
    }
  }

  // Phase 5b: open-loop reverse fallback when the planner can't escape the
  // boundary lethal cell. Returns SUCCESS on completion so the outer
  // IsBoundaryViolation check can decide whether we're back inside.
  if (phase_ == Phase::FallbackBackingUp)
  {
    // Wait for goal acceptance.
    if (!backup_goal_handle_)
    {
      if (backup_goal_handle_future_.wait_for(std::chrono::milliseconds(0)) !=
          std::future_status::ready)
      {
        return BT::NodeStatus::RUNNING;
      }
      backup_goal_handle_ = backup_goal_handle_future_.get();
      if (!backup_goal_handle_)
      {
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "NavigateInsideBoundary: /backup rejected fallback goal");
        pending_nav_result_ = BT::NodeStatus::FAILURE;
        return BeginReEnableKeepout();
      }
    }

    if (!backup_result_requested_)
    {
      backup_result_future_ = backup_action_client_->async_get_result(backup_goal_handle_);
      backup_result_requested_ = true;
    }

    if (backup_result_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }

    auto result = backup_result_future_.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "NavigateInsideBoundary: fallback backup complete — succeeding so "
                  "outer IsBoundaryViolation can re-check from new pose");
      // We have NOT reached the recovery target, but we have moved. The
      // wrapping <Inverter><IsBoundaryViolation/></Inverter> in the BT
      // will fail this iteration if we're still outside, which lets the
      // outer RetryUntilSuccessful give us a fresh planner attempt from
      // the new pose. Returning SUCCESS here (rather than FAILURE) is
      // what gates that re-evaluation.
      pending_nav_result_ = BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateInsideBoundary: fallback backup did not succeed (code=%d)",
                  static_cast<int>(result.code));
      pending_nav_result_ = BT::NodeStatus::FAILURE;
    }
    return BeginReEnableKeepout();
  }

  // Phase 6: re-enable keepout before returning the latched Nav2 result.
  if (phase_ == Phase::ReEnablingKeepout)
  {
    if (set_param_future_.valid() &&
        set_param_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (set_param_future_.valid())
    {
      auto results = set_param_future_.get();
      if (results.empty() || !results.front().successful)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "NavigateInsideBoundary: re-enable keepout failed: %s — "
                    "next costmap update with keepout disabled leaves the polygon unprotected",
                    results.empty() ? "empty response" : results.front().reason.c_str());
      }
    }
    keepout_disabled_ = false;
    return pending_nav_result_;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateInsideBoundary::SendNav2Goal()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "NavigateInsideBoundary: /navigate_to_pose unavailable");
    pending_nav_result_ = BT::NodeStatus::FAILURE;
    return BeginReEnableKeepout();
  }

  Nav2Goal::Goal goal_msg;
  goal_msg.pose.header.stamp = ctx->node->now();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose = recovery_pose_;

  goal_handle_future_ = action_client_->async_send_goal(goal_msg);

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateInsideBoundary: nav2 goal sent (x=%.2f y=%.2f, %.2fm outside)",
              recovery_pose_.position.x, recovery_pose_.position.y, distance_outside_);
  phase_ = Phase::WaitingForGoalHandle;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateInsideBoundary::BeginReEnableKeepout()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!keepout_disabled_)
  {
    // We never disabled (e.g. params service unavailable on entry), so
    // nothing to re-enable. Return the latched Nav2 outcome directly.
    return pending_nav_result_;
  }
  if (!keepout_params_client_ || !keepout_params_client_->service_is_ready())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateInsideBoundary: cannot re-enable keepout — params service unavailable; "
                "boundary protection will return on the next manual enable");
    keepout_disabled_ = false;
    return pending_nav_result_;
  }
  set_param_future_ = keepout_params_client_->set_parameters(
      {rclcpp::Parameter("keepout_filter.enabled", true)});
  phase_ = Phase::ReEnablingKeepout;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateInsideBoundary::BeginFallbackBackup()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Mark the slot consumed before any short-circuit return so we never
  // attempt this twice in the same onStart() cycle.
  fallback_attempted_ = true;

  if (!backup_action_client_ || !backup_action_client_->action_server_is_ready())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateInsideBoundary: /backup action server unavailable for fallback");
    return BeginReEnableKeepout();
  }

  // Fixed conservative reverse: 0.5 m at 0.16 m/s (matches v_linear_min so
  // the wheel-PI envelope can actually drive it through the static-friction
  // deadband). Generous time_allowance so slow motors don't trip the
  // behavior's own watchdog.
  constexpr double kBackupDist = 0.5;
  constexpr double kBackupSpeed = 0.16;

  BackUpAction::Goal goal_msg;
  goal_msg.target.x = -kBackupDist;
  goal_msg.target.y = 0.0;
  goal_msg.target.z = 0.0;
  goal_msg.speed = kBackupSpeed;
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(kBackupDist / kBackupSpeed * 3.0);

  backup_goal_handle_future_ = backup_action_client_->async_send_goal(goal_msg);
  backup_goal_handle_.reset();
  backup_result_requested_ = false;
  phase_ = Phase::FallbackBackingUp;

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateInsideBoundary: fallback backup goal sent (%.2f m @ %.2f m/s)",
              kBackupDist, kBackupSpeed);
  return BT::NodeStatus::RUNNING;
}

void NavigateInsideBoundary::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateInsideBoundary: canceling goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  if (backup_goal_handle_)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "NavigateInsideBoundary: canceling fallback backup");
    backup_action_client_->async_cancel_goal(backup_goal_handle_);
    backup_goal_handle_.reset();
  }
  if (keepout_disabled_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "NavigateInsideBoundary: halt — restoring keepout filter");
    RequestKeepoutEnable(true);
    keepout_disabled_ = false;
  }
}

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

BT::NodeStatus BackUp::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<BackUpAction>(ctx->node, "/backup");
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "BackUp: /backup action server not available");
    return BT::NodeStatus::FAILURE;
  }

  double dist = 0.5;
  double speed = 0.15;
  getInput("backup_dist", dist);
  getInput("backup_speed", speed);

  auto goal_msg = BackUpAction::Goal{};
  // BackUp target is negative X (reverse) in base_link frame
  goal_msg.target.x = -dist;
  goal_msg.target.y = 0.0;
  goal_msg.speed = speed;
  // Generous timeout: slow motors need extra time. 3x nominal duration.
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(dist / speed * 3.0);

  RCLCPP_INFO(ctx->node->get_logger(), "BackUp: reversing %.2fm at %.2f m/s", dist, speed);

  goal_handle_future_ = action_client_->async_send_goal(goal_msg);
  goal_handle_ = nullptr;
  result_requested_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BackUp::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal acceptance
  if (!goal_handle_)
  {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "BackUp: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Request result future only once
  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped = result_future_.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: complete");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(ctx->node->get_logger(),
              "BackUp: action ended with code %d",
              static_cast<int>(wrapped.code));
  return BT::NodeStatus::FAILURE;
}

void BackUp::onHalted()
{
  if (goal_handle_)
  {
    auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(ctx->node->get_logger(), "BackUp: halted, goal cancelled");
  }
  goal_handle_ = nullptr;
}

// ---------------------------------------------------------------------------
// SetNavMode
// ---------------------------------------------------------------------------

BT::NodeStatus SetNavMode::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto mode_res = getInput<std::string>("mode");
  if (!mode_res)
  {
    return BT::NodeStatus::FAILURE;
  }
  const std::string mode = mode_res.value();

  if (mode == ctx->current_nav_mode)
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Reconfigure controller speed via dynamic parameter API.
  auto param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(ctx->node, "/controller_server");

  if (!param_client->wait_for_service(std::chrono::milliseconds(200)))
  {
    // controller_server's lifecycle ramp is ~15-20 s on this hardware; if
    // COMMAND_START arrives during that window the BT used to return
    // FAILURE here, the parent GPSModeSelector Fallback would fail, and
    // the BT would silently hold in IDLE — operator-visible symptom was
    // "I clicked Start, nothing happened" (issue #197). Returning SUCCESS
    // without latching current_nav_mode means the next BT tick re-enters
    // SetNavMode and retries; the controller server has no inflight motion
    // during this boot window so the deferred mode swap is harmless. We
    // log once per process via WARN_ONCE to avoid spamming the boot log.
    RCLCPP_WARN_ONCE(ctx->node->get_logger(),
                     "SetNavMode: controller_server param service not ready yet — "
                     "deferring '%s' (BT will retry on next tick)",
                     mode.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // Apply the operator-configured speeds (from mowgli_robot.yaml via
  // behavior_tree_node → BTContext), NOT hardcoded magic numbers. We set the
  // knob each controller actually reads: FollowPath is RPP (via RotationShim),
  // whose speed knob is desired_linear_vel; FollowCoveragePath is FTC, whose
  // knob is speed_fast (it ignores desired_linear_vel — setting that was a
  // long-standing no-op that left coverage speed unconfigurable here).
  //
  // "degraded" (Float-quality GPS) runs at half the configured speed, floored
  // at the host min-drive clamp: hardware_bridge zeroes |vx| < kMinLinVel
  // (0.15 m/s), so a half-speed below that would stall the robot entirely.
  constexpr double kMinDriveSpeed = 0.15;
  const double transit =
      (mode == "precise") ? ctx->transit_speed : std::max(0.5 * ctx->transit_speed, kMinDriveSpeed);
  const double mowing =
      (mode == "precise") ? ctx->mowing_speed : std::max(0.5 * ctx->mowing_speed, kMinDriveSpeed);

  const std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("FollowPath.desired_linear_vel", transit),
      rclcpp::Parameter("FollowCoveragePath.speed_fast", mowing),
  };

  param_client->set_parameters(params);
  ctx->current_nav_mode = mode;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SetNavMode: mode '%s' — transit %.2f m/s, mowing %.2f m/s",
              mode.c_str(),
              transit,
              mowing);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
