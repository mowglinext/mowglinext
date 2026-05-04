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

#include <mutex>

#include "action_msgs/msg/goal_status.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "opennav_coverage_msgs/msg/coordinate.hpp"
#include "opennav_coverage_msgs/msg/coordinates.hpp"
#include "opennav_coverage_msgs/msg/headland_mode.hpp"
#include "opennav_coverage_msgs/msg/swath_mode.hpp"

namespace mowgli_behavior
{

// ===========================================================================
// ComputeCoveragePath
// ===========================================================================

BT::NodeStatus ComputeCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);

  // Fetch the target polygon synchronously (small, cheap call). Polls the
  // future without spinning the node so we don't recurse into the BT
  // blackboard.
  if (!area_client_)
  {
    area_client_ = ctx->helper_node->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
  }
  if (!area_client_->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ComputeCoveragePath: get_mowing_area not available");
    return BT::NodeStatus::FAILURE;
  }

  auto area_req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  area_req->index = area_idx;
  auto area_fut = area_client_->async_send_request(area_req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (area_fut.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() > deadline)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "ComputeCoveragePath: get_mowing_area timed out for area %u",
                   area_idx);
      return BT::NodeStatus::FAILURE;
    }
  }
  auto area_resp = area_fut.get();
  if (!area_resp->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ComputeCoveragePath: get_mowing_area failed for area %u", area_idx);
    return BT::NodeStatus::FAILURE;
  }

  const auto& area = area_resp->area;
  if (area.is_navigation_area)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "ComputeCoveragePath: area %u is navigation-only, skipping",
                area_idx);
    ctx->current_coverage_path = nav_msgs::msg::Path{};
    return BT::NodeStatus::SUCCESS;
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<CoverageAction>(
        ctx->node, "/compute_coverage_path");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ComputeCoveragePath: /compute_coverage_path action not available");
    return BT::NodeStatus::FAILURE;
  }

  CoverageAction::Goal goal;
  goal.frame_id = "map";
  goal.generate_headland = true;
  goal.generate_route = true;
  goal.generate_path = true;

  // Build a closed-ring Coordinates from a Polygon (Fields2Cover requires
  // first vertex == last vertex; mowgli polygons are stored open).
  auto to_closed_ring = [](const geometry_msgs::msg::Polygon& poly)
  {
    opennav_coverage_msgs::msg::Coordinates ring;
    if (poly.points.empty())
      return ring;
    for (const auto& pt : poly.points)
    {
      opennav_coverage_msgs::msg::Coordinate c;
      c.axis1 = pt.x;
      c.axis2 = pt.y;
      ring.coordinates.push_back(c);
    }
    const auto& first = poly.points.front();
    const auto& last = poly.points.back();
    if (std::fabs(first.x - last.x) > 1e-6 || std::fabs(first.y - last.y) > 1e-6)
    {
      opennav_coverage_msgs::msg::Coordinate c;
      c.axis1 = first.x;
      c.axis2 = first.y;
      ring.coordinates.push_back(c);
    }
    return ring;
  };

  // Field boundary first, holes (interior obstacles) after.
  goal.polygons.push_back(to_closed_ring(area.area));

  for (const auto& obs_poly : area.obstacles)
  {
    if (obs_poly.points.size() < 3)
      continue;
    goal.polygons.push_back(to_closed_ring(obs_poly));
  }

  // Match the opennav_coverage demo verbatim — known-working combo on
  // F2C 1.2.1. Empty mode strings let the server fall back to the
  // CONSTANT/0.30 m headland and BOUSTROPHEDON route in nav2_params.yaml.
  goal.headland_mode.mode = "";
  goal.headland_mode.width = 0.30f;

  goal.swath_mode.objective = "LENGTH";
  goal.swath_mode.mode = "SET_ANGLE";
  goal.swath_mode.best_angle = 0.0f;
  goal.swath_mode.step_angle = 1.7453e-2f;

  // No obstacles in the planning request for now — F2C 1.2.1 hangs on
  // small fields with interior holes. We'll re-add when persistent
  // obstacles need polygon-hole replans (task #21). The dock exclusion
  // zone is handled by the keepout_filter on the costmap, not here.
  if (goal.polygons.size() > 1)
    goal.polygons.resize(1);

  goal_handle_.reset();
  result_requested_ = false;
  goal_future_ = action_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "ComputeCoveragePath: planning for area %u (%zu vertices, %zu holes)",
              area_idx,
              area.area.points.size(),
              area.obstacles.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputeCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for goal handle.
  if (!goal_handle_)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "ComputeCoveragePath: goal rejected");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Request the async result once and poll without blocking — calling
  // .get() inside the BT tick deadlocks with the rclcpp executor that
  // delivers the result.
  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  auto wrapped = result_future_.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ComputeCoveragePath: action result code %d",
                 static_cast<int>(wrapped.code));
    goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  if (wrapped.result->error_code != 0)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "ComputeCoveragePath: planner error_code=%u",
                 wrapped.result->error_code);
    goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }
  ctx->current_coverage_path = wrapped.result->nav_path;
  if (ctx->current_coverage_path.header.frame_id.empty())
    ctx->current_coverage_path.header.frame_id = "map";

  // Prepend the robot's current pose to the coverage path so MPPI starts
  // tracking from the robot (closest_point = path[0]) instead of jumping
  // to a random nearest point. MPPI's PathHandler::transformPath breaks
  // its forward-walk the first time a pose lands outside the local
  // costmap — for a multi-swath coverage path this typically truncates
  // to zero poses unless the path begins near the robot.
  try
  {
    auto base_to_map = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0));
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.position.x = base_to_map.transform.translation.x;
    robot_pose.pose.position.y = base_to_map.transform.translation.y;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation = base_to_map.transform.rotation;
    ctx->current_coverage_path.poses.insert(
        ctx->current_coverage_path.poses.begin(), robot_pose);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ComputeCoveragePath: TF map→base_footprint unavailable, sending "
                "path as-is (%s)", ex.what());
  }

  // Stamp the path + every pose with the map frame and the current ROS
  // time. MPPI's transform pipeline interpolates against per-pose stamps
  // — Time(0) sentinel triggered an empty transformed_global_plan.
  ctx->current_coverage_path.header.stamp = ctx->node->now();
  for (auto& ps : ctx->current_coverage_path.poses)
  {
    ps.header.frame_id = ctx->current_coverage_path.header.frame_id;
    ps.header.stamp = ctx->current_coverage_path.header.stamp;
  }
  if (!ctx->current_coverage_path.poses.empty())
  {
    const auto& first = ctx->current_coverage_path.poses.front().pose.position;
    const auto& last = ctx->current_coverage_path.poses.back().pose.position;
    RCLCPP_INFO(ctx->node->get_logger(),
                "ComputeCoveragePath: %zu nav_path poses, "
                "first=(%.2f,%.2f) last=(%.2f,%.2f) frame='%s'",
                ctx->current_coverage_path.poses.size(),
                first.x, first.y, last.x, last.y,
                ctx->current_coverage_path.header.frame_id.c_str());
  }
  goal_handle_.reset();
  return BT::NodeStatus::SUCCESS;
}

void ComputeCoveragePath::onHalted()
{
  if (goal_handle_)
    action_client_->async_cancel_goal(goal_handle_);
  goal_handle_.reset();
}

// ===========================================================================
// FollowCoveragePath
// ===========================================================================

BT::NodeStatus FollowCoveragePath::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->current_coverage_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowCoveragePath: empty path — returning SUCCESS immediately");
    return BT::NodeStatus::SUCCESS;
  }

  // Bail when F2C planned a near-trivial path: the robot is near the
  // last pose, coverage_goal_checker (xy_goal_tolerance 0.5 m) will
  // fire "Reached the goal" on setPlan and we'd loop forever
  // (FollowCoveragePath instant-completes → GetNextUnmowedArea finds
  // remaining cells → ComputeCoveragePath returns the same path with
  // last-pose still near robot → repeat). Returning FAILURE here lets
  // the outer AreaLoop advance — combined with the 90% coverage
  // threshold in GetCoverageStatus this converges cleanly when F2C
  // can no longer add meaningful coverage.
  const auto& goal_pose = ctx->current_coverage_path.poses.back().pose.position;
  try
  {
    auto base_to_map = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
    const double dx = goal_pose.x - base_to_map.transform.translation.x;
    const double dy = goal_pose.y - base_to_map.transform.translation.y;
    const double dist = std::hypot(dx, dy);
    if (dist < 0.6)  // ≥ coverage_goal_checker.xy_goal_tolerance + margin
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "FollowCoveragePath: skipping — goal pose %.2fm from robot "
                  "(within goal-tolerance, would auto-complete). Area is done.",
                  dist);
      return BT::NodeStatus::FAILURE;
    }
  }
  catch (const tf2::TransformException&)
  {
    // Best-effort guard; if TF is missing just proceed and let MPPI try.
  }

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<Nav2FollowPath>(ctx->node, "/follow_path");
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowCoveragePath: /follow_path action not available");
    return BT::NodeStatus::FAILURE;
  }

  setBladeEnabled(true);
  goal_sent_ = false;
  follow_handle_.reset();

  Nav2FollowPath::Goal goal;
  goal.path = ctx->current_coverage_path;
  goal.controller_id = "FollowCoveragePath";
  goal.goal_checker_id = "coverage_goal_checker";

  follow_future_ = follow_client_->async_send_goal(goal);
  goal_sent_ = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowCoveragePath: sent %zu poses to MPPI",
              goal.path.poses.size());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowCoveragePath::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!follow_handle_)
  {
    if (follow_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    follow_handle_ = follow_future_.get();
    if (!follow_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(), "FollowCoveragePath: goal rejected");
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto status = follow_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(), "FollowCoveragePath: complete — painting mow_progress");
    paintPath(ctx->current_coverage_path);
    setBladeEnabled(false);
    follow_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    RCLCPP_WARN(ctx->node->get_logger(), "FollowCoveragePath: aborted/canceled");
    setBladeEnabled(false);
    follow_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void FollowCoveragePath::onHalted()
{
  if (follow_handle_)
    follow_client_->async_cancel_goal(follow_handle_);
  follow_handle_.reset();
  setBladeEnabled(false);
}

void FollowCoveragePath::setBladeEnabled(bool enabled)
{
  if (blade_on_ == enabled)
    return;
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
  blade_on_ = enabled;
}

void FollowCoveragePath::paintPath(const nav_msgs::msg::Path& path)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (path.poses.empty())
    return;
  if (!paint_client_)
  {
    paint_client_ = ctx->node->create_client<mowgli_interfaces::srv::PaintSwath>(
        "/map_server_node/paint_swath");
  }
  if (!paint_client_->wait_for_service(std::chrono::milliseconds(500)))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowCoveragePath: paint_swath unavailable, mow_progress not updated");
    return;
  }
  auto req = std::make_shared<mowgli_interfaces::srv::PaintSwath::Request>();
  req->swath_path = path;
  paint_client_->async_send_request(req);
}

// ===========================================================================
// GetNextUnmowedArea — preserved from prior architecture (uses
// GetCoverageStatus.strips_remaining as a coverage-threshold shim).
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

  getInput<uint32_t>("max_areas", max_areas_);
  current_area_idx_ = 0;
  areas_queried_ = 0;
  areas_complete_ = 0;

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
                    "GetNextUnmowedArea: targeted run — mowing only area %u",
                    current_area_idx_);
      }
      ctx->target_area_index.reset();
    }
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextUnmowedArea::onRunning()
{
  if (pending_future_->future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() - call_start_ > std::chrono::seconds(2))
    {
      auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "GetNextUnmowedArea: get_coverage_status timed out for area %u",
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
    if (areas_queried_ == 0)
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "GetNextUnmowedArea: no mowing areas defined — record an area first");
    }
    else
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "GetNextUnmowedArea: all %u area(s) complete",
                  areas_complete_);
    }
    return BT::NodeStatus::FAILURE;
  }

  ++areas_queried_;

  if (response->strips_remaining > 0)
  {
    setOutput("area_index", current_area_idx_);
    ctx->current_area = static_cast<int>(current_area_idx_);

    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: area %u selected (%.1f%% done)",
                current_area_idx_,
                response->coverage_percent);
    return BT::NodeStatus::SUCCESS;
  }

  ++areas_complete_;
  RCLCPP_INFO(ctx->node->get_logger(),
              "GetNextUnmowedArea: area %u complete (%.1f%%)",
              current_area_idx_,
              response->coverage_percent);

  ++current_area_idx_;
  if (current_area_idx_ >= max_areas_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "GetNextUnmowedArea: all %u area(s) complete",
                areas_complete_);
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetCoverageStatus::Request>();
  request->area_index = current_area_idx_;
  pending_future_.emplace(client_->async_send_request(request));
  call_start_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

void GetNextUnmowedArea::onHalted()
{
}

}  // namespace mowgli_behavior
