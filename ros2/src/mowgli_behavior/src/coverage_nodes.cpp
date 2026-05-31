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

#include "mowgli_behavior/coverage_nodes.hpp"

#include <cmath>

#include "action_msgs/msg/goal_status.hpp"
#include "tf2/exceptions.h"

namespace mowgli_behavior
{

// ===========================================================================
// GetNextStrip — fetch next unmowed strip from map_server
// ===========================================================================

BT::NodeStatus GetNextStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetNextStrip>(
        "/map_server_node/get_next_strip");
  }

  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetNextStrip::Request>();
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);
  request->area_index = area_idx;

  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    request->robot_x = tf.transform.translation.x;
    request->robot_y = tf.transform.translation.y;
  }
  catch (const tf2::TransformException&)
  {
    request->robot_x = 0.0;
    request->robot_y = 0.0;
  }
  request->prefer_headland = false;

  // Synchronous service call — poll future without spinning (avoids executor deadlock)
  auto future = client_->async_send_request(request);
  {
    auto timeout = std::chrono::seconds(5);
    auto start = std::chrono::steady_clock::now();
    bool completed = false;
    while (rclcpp::ok())
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        completed = true;
        break;
      }
      if (std::chrono::steady_clock::now() - start > timeout)
      {
        break;
      }
    }
    if (!completed)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service call timed out");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextStrip: service returned failure");
    return BT::NodeStatus::FAILURE;
  }

  if (response->coverage_complete)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextStrip: coverage complete (%.1f%%)",
                response->coverage_percent);
    return BT::NodeStatus::FAILURE;  // FAILURE = no more strips → loop ends
  }

  if (response->strip_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(), "GetNextStrip: empty strip path");
    return BT::NodeStatus::FAILURE;
  }

  ctx->current_strip_path = response->strip_path;
  ctx->current_transit_goal = response->transit_goal;
  ctx->coverage_percent = response->coverage_percent;

  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextStrip: %zu poses, %.1f%% coverage, %u strips left",
              response->strip_path.poses.size(),
              response->coverage_percent,
              response->strips_remaining);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetNextStrip::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void GetNextStrip::onHalted()
{
}

// ===========================================================================
// FollowStrip — follow strip path with FTCController
// ===========================================================================

BT::NodeStatus FollowStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->current_strip_path.poses.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: no strip path in context");
    return BT::NodeStatus::FAILURE;
  }

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(true);
  blade_start_time_ = std::chrono::steady_clock::now();
  goal_sent_ = false;

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowStrip: blade enabled, waiting %.1fs for spinup",
              kBladeSpinupDelaySec);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for blade to spin up before sending the path goal
  if (!goal_sent_)
  {
    auto elapsed = std::chrono::steady_clock::now() - blade_start_time_;
    if (elapsed < std::chrono::duration<double>(kBladeSpinupDelaySec))
      return BT::NodeStatus::RUNNING;

    // Spinup complete — send path goal
    Nav2FollowPath::Goal goal;
    goal.path = ctx->current_strip_path;
    goal.controller_id = "FollowCoveragePath";
    goal.goal_checker_id = "coverage_goal_checker";

    follow_handle_.reset();
    follow_future_ = follow_client_->async_send_goal(goal);
    goal_sent_ = true;

    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowStrip: sent %zu poses to FTCController",
                goal.path.poses.size());

    return BT::NodeStatus::RUNNING;
  }

  if (!follow_handle_)
  {
    if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    follow_handle_ = follow_future_.get();
    if (!follow_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "FollowStrip: goal rejected");
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = follow_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "FollowStrip: strip completed");
    follow_handle_.reset();
    setBladeEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "FollowStrip: aborted/canceled");
    follow_handle_.reset();
    setBladeEnabled(false);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void FollowStrip::onHalted()
{
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  follow_handle_.reset();
  setBladeEnabled(false);
}

void FollowStrip::setBladeEnabled(bool enabled)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200)))
    return;

  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  blade_client_->async_send_request(req);
}

// ===========================================================================
// TransitToStrip — navigate to strip start using Nav2
// ===========================================================================

BT::NodeStatus TransitToStrip::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Nothing to transit to if there is no coverage path (e.g. the area is
  // already fully mowed → PlanCoverageArea returned an empty path with a
  // stale transit goal). Skip cleanly; FollowStrip will handle the empty
  // path via its own AreaUnreachable fallthrough.
  if (ctx->current_strip_path.poses.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: goal frame='%s' pos=(%.2f, %.2f)",
              ctx->current_transit_goal.header.frame_id.c_str(),
              ctx->current_transit_goal.pose.position.x,
              ctx->current_transit_goal.pose.position.y);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "TransitToStrip: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal goal;
  goal.pose = ctx->current_transit_goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "TransitToStrip: navigating to (%.2f, %.2f)",
              goal.pose.pose.position.x,
              goal.pose.pose.position.y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TransitToStrip::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = nav_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "TransitToStrip: arrived at strip start");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "TransitToStrip: navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TransitToStrip::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// DetourAroundObstacle — short side-step via global planner so the robot
// gets out from in front of an obstacle that aborted the strip.
// ===========================================================================

BT::NodeStatus DetourAroundObstacle::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  double forward_m = 0.8;
  double lateral_m = 0.6;
  getInput("forward_m", forward_m);
  getInput("lateral_m", lateral_m);

  // Read current pose from map → base_footprint. If TF isn't ready, bail —
  // the BT will fall through to SkipStrip and we don't risk sending a
  // stale-pose-based goal.
  geometry_msgs::msg::TransformStamped t_map_base;
  try
  {
    t_map_base = ctx->tf_buffer->lookupTransform("map",
                                                 "base_footprint",
                                                 tf2::TimePointZero,
                                                 tf2::durationFromSec(0.2));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Yaw from quaternion: standard ZYX Euler extraction. Avoids pulling
  // in tf2_geometry_msgs just for tf2::getYaw().
  const auto& q = t_map_base.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Body-frame (forward, lateral) → map frame, added to current position.
  // Lateral positive = left (right-hand-rule with z-up).
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ctx->node->now();
  goal.pose.position.x = t_map_base.transform.translation.x + cy * forward_m - sy * lateral_m;
  goal.pose.position.y = t_map_base.transform.translation.y + sy * forward_m + cy * lateral_m;
  goal.pose.position.z = 0.0;

  // Keep the same heading. The global planner adjusts the path heading;
  // we just don't want to hand Nav2 a wildly different goal yaw.
  goal.pose.orientation = t_map_base.transform.rotation;

  RCLCPP_INFO(ctx->node->get_logger(),
              "DetourAroundObstacle: goal=(%.2f, %.2f) "
              "from (%.2f, %.2f), forward=%.2f lateral=%.2f",
              goal.pose.position.x,
              goal.pose.position.y,
              t_map_base.transform.translation.x,
              t_map_base.transform.translation.y,
              forward_m,
              lateral_m);

  if (!nav_client_)
  {
    nav_client_ = rclcpp_action::create_client<Nav2Navigate>(ctx->node, "/navigate_to_pose");
  }
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "DetourAroundObstacle: navigate_to_pose not available");
    return BT::NodeStatus::FAILURE;
  }

  Nav2Navigate::Goal nav_goal;
  nav_goal.pose = goal;

  nav_handle_.reset();
  nav_future_ = nav_client_->async_send_goal(nav_goal);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetourAroundObstacle::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!nav_handle_)
  {
    if (nav_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    nav_handle_ = nav_future_.get();
    if (!nav_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = nav_handle_->get_status();
  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "DetourAroundObstacle: detour complete");
    nav_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "DetourAroundObstacle: detour navigation failed");
    nav_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void DetourAroundObstacle::onHalted()
{
  if (nav_handle_)
  {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  nav_handle_.reset();
}

// ===========================================================================
// GetNextUnmowedArea — iterate areas, find first with strips remaining
// ===========================================================================

BT::NodeStatus GetNextUnmowedArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetCoverageStatus>(
        "/map_server_node/get_coverage_status");
  }

  if (!client_->service_is_ready())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "GetNextUnmowedArea: get_coverage_status service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Reset per-run state
  getInput<uint32_t>("max_areas", max_areas_);
  current_area_idx_ = 0;
  areas_queried_ = 0;
  areas_complete_ = 0;

  // Honor a one-shot user-selected target area (set by ~/start_in_area).
  // We start the iteration from the requested index AND clip max_areas_ to
  // (target + 1) so the BT mows just that area and exits MowingSequence,
  // instead of rolling over to the next area. The optional is consumed
  // here so subsequent COMMAND_START runs use the normal ordering.
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    if (ctx->target_area_index.has_value())
    {
      const int target = *ctx->target_area_index;
      if (target >= 0)
      {
        current_area_idx_ = static_cast<uint32_t>(target);
        max_areas_ = current_area_idx_ + 1;
        RCLCPP_INFO(ctx->node->get_logger(),
                    "GetNextUnmowedArea: targeted run — mowing only area %u (single-area mode)",
                    current_area_idx_);
      }
      ctx->target_area_index.reset();
    }
  }

  // Skip any area that has already burned its attempt budget this
  // session (BTContext::kMaxAreaAttempts dispatches of PlanCoverageArea
  // + FollowStrip). An area is added to attempted_areas only after
  // either completing successfully (strips_remaining == 0) or
  // exhausting its budget — so a single boundary-recovery preemption
  // does NOT permanently disable the area. attempted_areas is cleared
  // by EndSession at session end.
  while (current_area_idx_ < max_areas_ &&
         ctx->attempted_areas.count(current_area_idx_) > 0)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u already attempted this session, skipping",
                current_area_idx_);
    current_area_idx_++;
  }
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all areas already attempted this session");
    return BT::NodeStatus::FAILURE;
  }

  // Fire off the first async request
  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextUnmowedArea::onRunning()
{
  // Check if current async call has completed
  if (pending_future_->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    // Still waiting — check 2s timeout
    if (std::chrono::steady_clock::now() - call_start_ > std::chrono::seconds(2))
    {
      auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "GetNextUnmowedArea: get_coverage_status timed out for area %u after 2s — "
                   "returning FAILURE (BT should retry, not assume mowing complete)",
                   current_area_idx_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  return processResponse();
}

BT::NodeStatus GetNextUnmowedArea::processResponse()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto response = pending_future_->future.get();

  if (!response->success)
  {
    // Area index out of range — no more areas to check.
    if (areas_queried_ == 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "GetNextUnmowedArea: no mowing areas defined in map_server "
                  "(first get_coverage_status returned success=false). "
                  "Record an area via the GUI before starting mowing.");
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: all %u area(s) complete",
                  areas_complete_);
    }
    return BT::NodeStatus::FAILURE;
  }

  areas_queried_++;

  if (response->strips_remaining > 0)
  {
    // Increment the per-area attempt counter. After kMaxAreaAttempts
    // dispatches, mark the area attempted (gives up) — otherwise let
    // the area roll over for another PlanCoverageArea + FollowStrip
    // pass. This is what lets us re-enter an area after a
    // BoundaryGuard recovery preempts the original FollowStrip
    // mid-mow: the cached strip_path is gone but mow_progress has
    // captured everything we mowed before the excursion, so the
    // re-plan covers only the remaining swaths.
    // Count only CONSECUTIVE no-progress dispatches toward the give-up
    // budget. If this area's coverage advanced beyond the best we've seen,
    // the previous pass mowed something despite any FollowStrip stutter
    // (e.g. a hard-to-skirt obstacle at a strip end) — reset the counter so
    // a steadily-progressing area is never abandoned mid-climb.
    auto& n = ctx->area_attempt_count[current_area_idx_];
    auto last_it = ctx->area_last_coverage.find(current_area_idx_);
    const bool made_progress =
        (last_it == ctx->area_last_coverage.end()) ||
        (response->coverage_percent > last_it->second + BTContext::kAreaProgressEpsilonPct);
    if (made_progress)
    {
      ctx->area_last_coverage[current_area_idx_] = response->coverage_percent;
      n = 0;
    }
    n++;
    if (n >= BTContext::kMaxAreaAttempts)
    {
      ctx->attempted_areas.insert(current_area_idx_);
      RCLCPP_WARN(ctx->node->get_logger(),
                  "GetNextUnmowedArea: area %u hit max attempts (%u), giving up at %.1f%% "
                  "with %u strips remaining",
                  current_area_idx_, n, response->coverage_percent,
                  response->strips_remaining);
      // Continue iterating — fall through to "skip" path by advancing
      // current_area_idx_ and re-running the skip loop logic.
      current_area_idx_++;
      while (current_area_idx_ < max_areas_ &&
             ctx->attempted_areas.count(current_area_idx_) > 0)
      {
        current_area_idx_++;
      }
      if (current_area_idx_ >= max_areas_)
      {
        return BT::NodeStatus::FAILURE;
      }
      // Issue the next async request for the next candidate area.
      auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
      request->area_index = current_area_idx_;
      pending_future_.emplace(client_->async_send_request(request));
      call_start_ = std::chrono::steady_clock::now();
      return BT::NodeStatus::RUNNING;
    }

    setOutput("area_index", current_area_idx_);
    ctx->current_area = static_cast<int>(current_area_idx_);

    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u has %u strips remaining (%.1f%% done) "
                "— dispatch attempt %u/%u",
                current_area_idx_,
                response->strips_remaining,
                response->coverage_percent,
                n, BTContext::kMaxAreaAttempts);
    return BT::NodeStatus::SUCCESS;
  }

  areas_complete_++;
  // Treat fully-mowed areas as attempted too, so we don't keep polling
  // them on subsequent AreaLoop iterations.
  ctx->attempted_areas.insert(current_area_idx_);
  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextUnmowedArea: area %u complete (%.1f%%)",
              current_area_idx_,
              response->coverage_percent);

  // Move to the next area, skipping any already-attempted ones.
  current_area_idx_++;
  while (current_area_idx_ < max_areas_ &&
         ctx->attempted_areas.count(current_area_idx_) > 0)
  {
    current_area_idx_++;
  }
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all %u area(s) complete",
                areas_complete_);
    return BT::NodeStatus::FAILURE;
  }

  // Fire off the next async request
  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

void GetNextUnmowedArea::onHalted()
{
  // Nothing to cancel — service calls complete on their own.
  // State will be reset in onStart() on next invocation.
}

// ===========================================================================
// GetNextSegment (Path C — cell-based coverage)
// ===========================================================================

BT::NodeStatus GetNextSegment::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::GetNextSegment>(
        "/map_server_node/get_next_segment");
  }
  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetNextSegment::Request>();
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);
  request->area_index = area_idx;

  // Robot pose from TF.
  double yaw = 0.0;
  try
  {
    auto tf = ctx->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    request->robot_x = tf.transform.translation.x;
    request->robot_y = tf.transform.translation.y;
    const auto& q = tf.transform.rotation;
    yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  catch (const tf2::TransformException&)
  {
    request->robot_x = 0.0;
    request->robot_y = 0.0;
  }
  request->robot_yaw_rad = yaw;

  // prefer_dir: NaN → use the robot's current heading as a hint so the
  // server's auto-MBR direction kicks in. The server's
  // compute_optimal_mow_angle is internal to ensure_strip_layout —
  // for now we pass the robot heading so the first segment doesn't
  // require a 180° rotation. A future iteration can extract the
  // per-area auto angle into a service parameter.
  double prefer_dir = std::numeric_limits<double>::quiet_NaN();
  getInput<double>("prefer_dir_yaw_rad", prefer_dir);
  if (!std::isfinite(prefer_dir))
    prefer_dir = yaw;
  request->prefer_dir_yaw_rad = prefer_dir;

  bool boust = true;
  getInput<bool>("boustrophedon", boust);
  request->boustrophedon = boust;

  double max_len = 0.0;
  getInput<double>("max_segment_length_m", max_len);
  request->max_segment_length_m = max_len;

  auto future = client_->async_send_request(request);
  {
    auto timeout = std::chrono::seconds(5);
    auto start = std::chrono::steady_clock::now();
    bool completed = false;
    while (rclcpp::ok())
    {
      if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready)
      {
        completed = true;
        break;
      }
      if (std::chrono::steady_clock::now() - start > timeout)
        break;
    }
    if (!completed)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service call timed out");
      return BT::NodeStatus::FAILURE;
    }
  }

  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(), "GetNextSegment: service returned failure");
    return BT::NodeStatus::FAILURE;
  }
  if (response->coverage_complete)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextSegment: area %u coverage complete (%.1f%%)",
                area_idx,
                response->coverage_percent);
    return BT::NodeStatus::FAILURE;  // FAILURE → outer loop picks next area.
  }
  if (response->segment_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(), "GetNextSegment: empty segment path");
    return BT::NodeStatus::FAILURE;
  }

  ctx->current_strip_path = response->segment_path;
  ctx->current_transit_goal = response->target_cell_pose;
  ctx->coverage_percent = response->coverage_percent;
  ctx->current_segment_is_long_transit = response->is_long_transit;
  ctx->current_segment_phase = response->phase;
  ctx->current_segment_termination_reason = response->termination_reason;

  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextSegment: area %u, %zu poses, %.1f%% coverage, "
              "transit=%s, end=%s, dead_cells=%u",
              area_idx,
              response->segment_path.poses.size(),
              response->coverage_percent,
              response->is_long_transit ? "yes" : "no",
              response->termination_reason.c_str(),
              response->dead_cells_count);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetNextSegment::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void GetNextSegment::onHalted()
{
}

// ===========================================================================
// IsShortSegment (condition)
// ===========================================================================

BT::NodeStatus IsShortSegment::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  return ctx->current_segment_is_long_transit ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

// ===========================================================================
// MarkSegmentBlocked
// ===========================================================================

BT::NodeStatus MarkSegmentBlocked::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto helper = ctx->helper_node;

  // Don't bump fail_count when the previous GetNextSegment ended at a
  // known obstacle / dead cell — the failure is already accounted for.
  if (ctx->current_segment_termination_reason == "obstacle" ||
      ctx->current_segment_termination_reason == "dead_zone" ||
      ctx->current_segment_termination_reason == "boundary")
  {
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "MarkSegmentBlocked: skipping bump — segment ended at "
                 "known '%s'",
                 ctx->current_segment_termination_reason.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (ctx->current_strip_path.poses.empty())
  {
    return BT::NodeStatus::SUCCESS;
  }

  if (!client_)
  {
    client_ = helper->create_client<mowgli_interfaces::srv::MarkSegmentBlocked>(
        "/map_server_node/mark_segment_blocked");
  }
  if (!client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(ctx->node->get_logger(), "MarkSegmentBlocked: service unavailable, skipping");
    return BT::NodeStatus::SUCCESS;
  }

  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);

  auto request = std::make_shared<mowgli_interfaces::srv::MarkSegmentBlocked::Request>();
  request->area_index = area_idx;
  request->failed_path = ctx->current_strip_path;

  future_ = client_->async_send_request(request).future.share();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MarkSegmentBlocked::onRunning()
{
  if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  auto resp = future_.get();
  if (resp && resp->success)
  {
    if (resp->cells_promoted_dead > 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "MarkSegmentBlocked: %u cells bumped, %u promoted DEAD",
                  resp->cells_marked_blocked,
                  resp->cells_promoted_dead);
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "MarkSegmentBlocked: %u cells bumped",
                  resp->cells_marked_blocked);
    }
  }
  // Always SUCCESS — this is bookkeeping. The outer Fallback handles
  // the actual control-flow recovery (next segment / next area).
  return BT::NodeStatus::SUCCESS;
}

void MarkSegmentBlocked::onHalted()
{
}

// ===========================================================================
// PlanCoverageArea — opennav_coverage migration
// ===========================================================================

PlanCoverageArea::ComputeCoveragePath::Goal PlanCoverageArea::buildGoal(
    const mowgli_interfaces::msg::MapArea& piece,
    double operation_width,
    double headland_width) const
{
  ComputeCoveragePath::Goal goal;
  goal.generate_headland = true;
  goal.generate_route = true;
  goal.generate_path = true;
  goal.use_gml_file = false;
  goal.frame_id = "map";

  // Build [outer, hole0, hole1, ...] in opennav_coverage_msgs/Coordinates form.
  auto polygon_to_coords = [](const geometry_msgs::msg::Polygon& poly) {
    opennav_coverage_msgs::msg::Coordinates coords;
    coords.coordinates.reserve(poly.points.size() + 1);
    for (const auto& p : poly.points)
    {
      opennav_coverage_msgs::msg::Coordinate c;
      c.axis1 = p.x;
      c.axis2 = p.y;
      coords.coordinates.push_back(c);
    }
    // Close the ring if the source isn't closed (F2C expects closed rings).
    if (!poly.points.empty())
    {
      const auto& first = poly.points.front();
      const auto& last = poly.points.back();
      if (first.x != last.x || first.y != last.y)
      {
        opennav_coverage_msgs::msg::Coordinate c;
        c.axis1 = first.x;
        c.axis2 = first.y;
        coords.coordinates.push_back(c);
      }
    }
    return coords;
  };

  goal.polygons.push_back(polygon_to_coords(piece.area));
  for (const auto& hole : piece.obstacles)
  {
    if (hole.points.size() >= 3)
    {
      goal.polygons.push_back(polygon_to_coords(hole));
    }
  }

  goal.headland_mode.mode = "CONSTANT";
  goal.headland_mode.width = static_cast<float>(headland_width);

  goal.swath_mode.objective = "LENGTH";
  goal.swath_mode.mode = "BRUTE_FORCE";
  goal.swath_mode.best_angle = 0.0f;
  goal.swath_mode.step_angle = static_cast<float>(M_PI / 180.0);

  goal.route_mode.mode = "BOUSTROPHEDON";

  goal.path_mode.mode = "DUBIN";
  // DISCONTINUOUS = straight swaths separated by sharp orientation
  // changes — diff-drive friendly. CONTINUOUS Dubin curves require
  // a min_turning_radius wider than our headland inset, which sends
  // the robot outside the polygon at every U-turn.
  goal.path_mode.continuity_mode = "DISCONTINUOUS";
  goal.path_mode.turn_point_distance = 0.05f;

  // operation_width is set as a coverage_server parameter, not in the goal.
  // Caller must launch coverage_server with operation_width matching mower
  // cut width.
  (void)operation_width;
  return goal;
}

namespace
{

/// Compute axis-aligned bbox + signed area + perimeter of a ROS Polygon.
/// Used to log diagnostics on each ComputeCoveragePath piece so we can
/// see whether F2C choked on a degenerate shape (sliver, self-intersect,
/// zero area, etc.).
struct PolygonStats
{
  double min_x{0}, min_y{0}, max_x{0}, max_y{0};
  double signed_area{0};
  double perimeter{0};
};

PolygonStats polygon_stats(const geometry_msgs::msg::Polygon& poly)
{
  PolygonStats s;
  const size_t n = poly.points.size();
  if (n < 3)
  {
    return s;
  }
  s.min_x = s.max_x = poly.points[0].x;
  s.min_y = s.max_y = poly.points[0].y;
  for (size_t i = 0; i < n; ++i)
  {
    const auto& a = poly.points[i];
    const auto& b = poly.points[(i + 1) % n];
    s.min_x = std::min(s.min_x, static_cast<double>(a.x));
    s.max_x = std::max(s.max_x, static_cast<double>(a.x));
    s.min_y = std::min(s.min_y, static_cast<double>(a.y));
    s.max_y = std::max(s.max_y, static_cast<double>(a.y));
    s.signed_area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    s.perimeter += std::hypot(static_cast<double>(b.x) - a.x,
                              static_cast<double>(b.y) - a.y);
  }
  s.signed_area *= 0.5;
  return s;
}

}  // namespace

BT::NodeStatus PlanCoverageArea::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Reset per-tick state.
  pieces_.clear();
  current_piece_ = 0;
  pieces_succeeded_ = 0;
  pieces_failed_ = 0;
  accumulated_path_ = nav_msgs::msg::Path{};
  accumulated_path_.header.frame_id = "map";
  accumulated_path_.header.stamp = ctx->node->get_clock()->now();
  have_first_coverage_pose_ = false;
  goal_handle_.reset();
  phase_start_ = std::chrono::steady_clock::now();

  uint32_t area_index = 0;
  getInput<uint32_t>("area_index", area_index);

  if (!srv_client_)
  {
    auto helper = ctx->helper_node;
    srv_client_ = helper->create_client<mowgli_interfaces::srv::GetRemainingAreaPolygon>(
        "/map_server_node/get_remaining_area_polygon");
  }
  if (!srv_client_->service_is_ready())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageArea: get_remaining_area_polygon service not ready");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<ComputeCoveragePath>(
        ctx->node, "/compute_coverage_path");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PlanCoverageArea: compute_coverage_path action not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request =
      std::make_shared<mowgli_interfaces::srv::GetRemainingAreaPolygon::Request>();
  request->area_id = area_index;
  srv_future_.emplace(srv_client_->async_send_request(request));
  phase_ = Phase::QueryRemaining;

  RCLCPP_INFO(ctx->node->get_logger(),
              "PlanCoverageArea: querying remaining polygon for area %u", area_index);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanCoverageArea::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // ── Phase: wait for ~/get_remaining_area_polygon response ────────────────
  if (phase_ == Phase::QueryRemaining)
  {
    if (!srv_future_)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (srv_future_->future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(3))
      {
        RCLCPP_ERROR(ctx->node->get_logger(),
                     "PlanCoverageArea: get_remaining_area_polygon timed out (3s)");
        srv_future_.reset();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    auto resp = srv_future_->future.get();
    srv_future_.reset();
    if (!resp->success)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "PlanCoverageArea: get_remaining_area_polygon failed: %s",
                   resp->error.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (resp->pieces.empty())
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "PlanCoverageArea: area already fully mowed, nothing to plan");
      ctx->current_strip_path = accumulated_path_;
      return BT::NodeStatus::SUCCESS;
    }

    pieces_ = resp->pieces;
    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoverageArea: area has %zu remaining piece(s) to plan",
                pieces_.size());
    phase_ = Phase::PlanNextPiece;
  }

  // ── Phase: dispatch ComputeCoveragePath for the next piece ───────────────
  if (phase_ == Phase::PlanNextPiece)
  {
    if (current_piece_ >= pieces_.size())
    {
      // Done iterating. Succeed if at least one piece produced poses;
      // otherwise the robot has nothing to do and we let the BT bubble
      // FAILURE up to the AreaUnreachable branch.
      ctx->current_strip_path = accumulated_path_;
      RCLCPP_INFO(ctx->node->get_logger(),
                  "PlanCoverageArea: combined path has %zu poses (%zu piece(s) ok, %zu failed)",
                  accumulated_path_.poses.size(), pieces_succeeded_, pieces_failed_);
      if (accumulated_path_.poses.size() >= 2)
      {
        const auto& first = accumulated_path_.poses.front().pose.position;
        const auto& last = accumulated_path_.poses.back().pose.position;
        const double dx = last.x - first.x, dy = last.y - first.y;
        RCLCPP_INFO(ctx->node->get_logger(),
                    "PlanCoverageArea: path geometry first=(%.2f,%.2f) last=(%.2f,%.2f) "
                    "first→last dist=%.2fm",
                    first.x, first.y, last.x, last.y, std::hypot(dx, dy));
      }
      if (accumulated_path_.poses.empty())
      {
        return BT::NodeStatus::FAILURE;
      }
      // Transit goal = the real F2C mowing start. The mowed path no longer
      // carries a robot→F2C-start straight prefix, so an obstacle-aware,
      // boundary-aware TransitToStrip (Nav2 Smac + global obstacle_layer)
      // routes the robot AROUND obstacles and inside the polygon to the
      // coverage start; FollowStrip then begins mowing right at the F2C
      // start. Fall back to the path front only if we never captured an
      // F2C start (no piece produced poses).
      if (have_first_coverage_pose_)
      {
        ctx->current_transit_goal = first_coverage_pose_;
      }
      else
      {
        ctx->current_transit_goal = accumulated_path_.poses.front();
        ctx->current_transit_goal.header = accumulated_path_.header;
      }
      return BT::NodeStatus::SUCCESS;
    }

    double op_w = 0.20;
    getInput<double>("operation_width_m", op_w);

    // Derive the F2C headland_width from the chassis-based ring inset so
    // the F2C swaths start tangent to the inner edge of the ring's cut
    // zone. Read chassis_width from the BT node parameters
    // (mowgli_robot.yaml).
    double chassis_width = 0.40;
    ctx->node->get_parameter_or("chassis_width", chassis_width, chassis_width);
    constexpr double safety_margin = 0.05;
    const double ring_inset = 0.5 * chassis_width + safety_margin;
    const double f2c_headland_width = ring_inset + 0.5 * op_w;

    // Pre-dispatch geometry log: any F2C failure for this piece will be
    // diagnosable from these numbers (sliver / self-intersection / too
    // small for headland_width / hole inversion / ...).
    const auto& piece = pieces_[current_piece_];
    const auto outer_stats = polygon_stats(piece.area);
    const double bbox_w = outer_stats.max_x - outer_stats.min_x;
    const double bbox_h = outer_stats.max_y - outer_stats.min_y;
    RCLCPP_INFO(ctx->node->get_logger(),
                "PlanCoverageArea: dispatching piece %zu/%zu — outer %zu pts, "
                "bbox=%.2fx%.2fm, signed_area=%.2fm² (CCW=%c), perimeter=%.2fm, "
                "%zu holes, headland_width=%.2fm, op_w=%.2fm",
                current_piece_ + 1, pieces_.size(),
                piece.area.points.size(),
                bbox_w, bbox_h, outer_stats.signed_area,
                outer_stats.signed_area > 0 ? 'Y' : 'N',
                outer_stats.perimeter,
                piece.obstacles.size(), f2c_headland_width, op_w);
    for (size_t h = 0; h < piece.obstacles.size(); ++h)
    {
      const auto hs = polygon_stats(piece.obstacles[h]);
      RCLCPP_INFO(ctx->node->get_logger(),
                  "PlanCoverageArea:   hole %zu — %zu pts, bbox=%.2fx%.2fm, "
                  "signed_area=%.3fm² (%s)",
                  h, piece.obstacles[h].points.size(),
                  hs.max_x - hs.min_x, hs.max_y - hs.min_y, hs.signed_area,
                  hs.signed_area < 0 ? "CW=hole-correct" : "CCW=hole-flipped!");
    }

    // Sanity: piece must be at least 2*headland_width wide AND tall, else
    // F2C has no room for any swath inside the inset field.
    const double min_dim = 2.0 * f2c_headland_width + op_w;
    if (bbox_w < min_dim || bbox_h < min_dim)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu too small for F2C "
                  "(bbox=%.2fx%.2fm, need ≥%.2fm); skipping piece",
                  current_piece_, bbox_w, bbox_h, min_dim);
      pieces_failed_++;
      current_piece_++;
      return BT::NodeStatus::RUNNING;
    }

    auto goal = buildGoal(piece, op_w, f2c_headland_width);
    goal_handle_.reset();
    goal_future_ = action_client_->async_send_goal(goal);
    phase_ = Phase::WaitingForGoal;
    phase_start_ = std::chrono::steady_clock::now();
  }

  // ── Phase: wait for goal handle ──────────────────────────────────────────
  if (phase_ == Phase::WaitingForGoal)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(3))
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "PlanCoverageArea: piece %zu goal handshake timeout — skipping",
                    current_piece_);
        pieces_failed_++;
        current_piece_++;
        phase_ = Phase::PlanNextPiece;
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu goal rejected — skipping",
                  current_piece_);
      pieces_failed_++;
      current_piece_++;
      phase_ = Phase::PlanNextPiece;
      return BT::NodeStatus::RUNNING;
    }
    result_future_ = action_client_->async_get_result(goal_handle_);
    phase_ = Phase::WaitingForResult;
    phase_start_ = std::chrono::steady_clock::now();
  }

  // ── Phase: wait for the path result ──────────────────────────────────────
  if (phase_ == Phase::WaitingForResult)
  {
    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      // F2C planning can take seconds on large fields; allow 30 s.
      if (std::chrono::steady_clock::now() - phase_start_ > std::chrono::seconds(30))
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "PlanCoverageArea: piece %zu result timeout (30s) — skipping",
                    current_piece_);
        pieces_failed_++;
        current_piece_++;
        phase_ = Phase::PlanNextPiece;
        return BT::NodeStatus::RUNNING;
      }
      return BT::NodeStatus::RUNNING;
    }
    auto wrapped = result_future_.get();
    bool piece_ok = true;
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu ComputeCoveragePath did not succeed "
                  "(code=%d) — skipping",
                  current_piece_, static_cast<int>(wrapped.code));
      piece_ok = false;
    }
    else if (wrapped.result->error_code != 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu F2C error_code=%u — skipping "
                  "(see pre-dispatch geometry log)",
                  current_piece_, wrapped.result->error_code);
      piece_ok = false;
    }
    else if (wrapped.result->nav_path.poses.empty())
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu produced empty path — skipping",
                  current_piece_);
      piece_ok = false;
    }

    if (!piece_ok)
    {
      pieces_failed_++;
    }
    else
    {
      // F2C v2 (mowgli_coverage server) returns a complete coverage
      // path including headland traversal — no need to prepend the
      // hand-rolled perimeter ring anymore.
      const auto& piece_path = wrapped.result->nav_path;
      const bool is_first_piece = accumulated_path_.poses.empty();
      if (!piece_path.poses.empty())
      {
        // Capture the FIRST piece's F2C start as the obstacle-aware
        // TransitToStrip goal. This is the REAL mowing start: the
        // robot→F2C-start leg is deliberately NOT prepended to the mowed
        // path. An obstacle-aware, boundary-aware TransitToStrip (Nav2
        // Smac + the global obstacle_layer) drives the robot there
        // blade-off, and FollowStrip then begins mowing exactly at the
        // F2C start. The old hand-rolled straight prefix ignored the area
        // polygon entirely and got mowed blade-on straight out of the
        // field ("un swath qui sort du terrain au point de départ").
        if (is_first_piece && !have_first_coverage_pose_)
        {
          first_coverage_pose_ = piece_path.poses.front();
          first_coverage_pose_.header = piece_path.header;
          have_first_coverage_pose_ = true;
        }
        // Between consecutive F2C pieces (within the already-planned
        // field), densify a short transit segment from the previous
        // accumulated end-pose to this piece's first pose. Required
        // because each F2C piece plans independently — the gap between
        // pieces would otherwise be picked up by FTC's setPlan as the
        // "closest path pose to robot", landing deep in the next piece
        // (skipping 5-95 % of poses). Only inter-piece gaps, which lie
        // inside the field, are bridged here; the first piece gets no
        // prefix (TransitToStrip owns the robot→F2C-start leg).
        if (!is_first_piece)
        {
          const auto& prev_end = accumulated_path_.poses.back().pose.position;
          const double rx = prev_end.x;
          const double ry = prev_end.y;
          const auto& first_pose = piece_path.poses.front().pose.position;
          const double tdx = first_pose.x - rx;
          const double tdy = first_pose.y - ry;
          const double tlen = std::hypot(tdx, tdy);
          // Skip the transit when the previous end is already at/near the
          // F2C start — densifying a sub-10 cm leg gives one redundant
          // pose at the same location.
          if (tlen > 0.10)
          {
            const double yaw = std::atan2(tdy, tdx);
            const double sin_h = std::sin(yaw * 0.5);
            const double cos_h = std::cos(yaw * 0.5);
            constexpr double kStep = 0.10;  // m between transit poses
            const int n_steps = std::max(1, static_cast<int>(tlen / kStep));
            for (int k = 0; k < n_steps; ++k)
            {
              const double t = static_cast<double>(k) /
                               static_cast<double>(n_steps);
              geometry_msgs::msg::PoseStamped ps;
              ps.header = piece_path.header;
              ps.pose.position.x = rx + t * tdx;
              ps.pose.position.y = ry + t * tdy;
              ps.pose.orientation.z = sin_h;
              ps.pose.orientation.w = cos_h;
              accumulated_path_.poses.push_back(ps);
            }
            RCLCPP_INFO(ctx->node->get_logger(),
                        "PlanCoverageArea: piece %zu inter-piece transit %d "
                        "poses from (%.2f,%.2f) to F2C start (%.2f,%.2f), %.2fm",
                        current_piece_,
                        n_steps,
                        rx,
                        ry,
                        first_pose.x,
                        first_pose.y,
                        tlen);
          }
        }
      }
      accumulated_path_.poses.insert(
          accumulated_path_.poses.end(),
          piece_path.poses.begin(),
          piece_path.poses.end());
      RCLCPP_INFO(ctx->node->get_logger(),
                  "PlanCoverageArea: piece %zu — appended %zu F2C poses "
                  "(total=%zu)",
                  current_piece_, piece_path.poses.size(),
                  accumulated_path_.poses.size());
      pieces_succeeded_++;
    }

    current_piece_++;
    phase_ = Phase::PlanNextPiece;
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void PlanCoverageArea::onHalted()
{
  if (goal_handle_ && action_client_)
  {
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_handle_.reset();
  srv_future_.reset();
  pieces_.clear();
}

}  // namespace mowgli_behavior
