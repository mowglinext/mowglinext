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

#include <algorithm>
#include <cmath>
#include <limits>
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

  // Match the opennav_coverage demo verbatim — known-working combo on
  // F2C 1.2.1. Empty mode strings let the server fall back to the
  // CONSTANT/0.30 m headland and BOUSTROPHEDON route in nav2_params.yaml.
  goal.headland_mode.mode = "";
  // Headland inset budget at the polygon perimeter:
  //   F2C uses min_turning_radius (0.10 m, set in nav2_params.yaml's
  //   coverage_server) for the strip-end U-turn arc — a forward-only
  //   Dubins semicircle with apex 0.10 m past strip endpoint. MPPI
  //   overshoots the arc apex by ~0.70 m at cruise: max turn rate
  //   (wz_max=1.5) limits the actual achievable turn radius at
  //   vx=0.20 m/s to ~0.133 m, so the 0.10 m arc gets cut and
  //   PathFollow drags the trajectory past the apex toward its target
  //   on the next strip. Total margin = arc_apex + MPPI_overshoot +
  //   safety = 0.10 + 0.70 + 0.20 = 1.00 m.
  //
  // History (this is attempt #6 at this constant):
  //   0.30 → 0.50 → 0.80: each enough for the planned arc but not
  //                       MPPI's tracking error
  //   0.80 + Reeds-Shepp/vx_min=-0.15: creep-reverse local-min stall
  //   0.80 + Dubins/min_turn=0.10:    sim #19, 28% coverage / 10 min
  //                                   but 60+ boundary violations
  //                                   barely past the polygon edge
  //   1.00 + Dubins (this):           absorbs MPPI's tracking overshoot
  //                                   without changing any compute
  //
  // Cost of 1.00 m: a 0.20 m wider unmowed border per side. For a
  // 6 × 6 m sim polygon (36 m²) the loss is ~10%; on real lawns the
  // percentage shrinks substantially.
  constexpr float kHeadlandWidth = 1.00f;
  goal.headland_mode.width = kHeadlandWidth;

  goal.swath_mode.objective = "LENGTH";
  goal.swath_mode.mode = "SET_ANGLE";
  goal.swath_mode.best_angle = 0.0f;
  goal.swath_mode.step_angle = 1.7453e-2f;

  // Field boundary first, holes (interior obstacles) after.
  goal.polygons.push_back(to_closed_ring(area.area));

  // Re-enable interior obstacles as F2C holes (task #21). F2C 1.2.1 hangs
  // when a hole touches/escapes the headland-reduced field, so we filter:
  //   - drop polygons with < 3 unique points or area < MIN_HOLE_AREA_M2
  //   - require every vertex to lie strictly inside the field, with at
  //     least (headland_width + 0.20 m) clearance from the boundary —
  //     guarantees the hole survives F2C's headland inset
  //   - cap total holes at MAX_F2C_HOLES so a noisy obstacle_tracker
  //     can't push planning into the seconds
  // Obstacles outside (or too close to) the boundary are still respected
  // via the keepout_filter on the costmap; they just don't enter the
  // F2C plan as holes.
  constexpr double kMinHoleAreaM2 = 0.04;            // 20 cm × 20 cm
  constexpr double kHoleBoundaryClearanceM = 0.20;   // extra margin past headland inset
  constexpr std::size_t kMaxF2cHoles = 16;
  const double min_clearance = static_cast<double>(kHeadlandWidth) + kHoleBoundaryClearanceM;

  std::size_t filtered_too_small = 0, filtered_outside = 0, filtered_capped = 0;
  for (const auto& obs : area.obstacles)
  {
    if (obs.points.size() < 3 || polygonArea(obs) < kMinHoleAreaM2) {
      ++filtered_too_small;
      continue;
    }
    if (!isHoleSafeForF2C(obs, area.area, kMinHoleAreaM2, min_clearance)) {
      ++filtered_outside;
      continue;
    }
    if (goal.polygons.size() > kMaxF2cHoles) {
      ++filtered_capped;
      continue;
    }
    goal.polygons.push_back(to_closed_ring(obs));
  }

  if (filtered_too_small || filtered_outside || filtered_capped)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "ComputeCoveragePath: filtered obstacles "
                "(too_small=%zu, on_or_outside_boundary=%zu, capped=%zu)",
                filtered_too_small, filtered_outside, filtered_capped);
  }

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

  // The F2C coverage_plan goes to MPPI AS-IS. The previous version
  // prepended the robot's current pose to path[0] under the theory
  // that MPPI's PathHandler::transformPath would truncate the plan
  // unless it started near the robot — but this prepended a non-
  // strip-shaped lead-in segment from (robot_x, robot_y) to
  // (F2C_first_strip_pose), and MPPI's PathHandler closest-point
  // logic then got confused alternating between "follow the
  // lead-in" and "follow the strip", producing the looping /
  // tangled actual driven track the user observed (mow_progress
  // captured 2026-05-05 showed loops, not strips).
  //
  // The correct architecture (per GHANSHYAM-13/coverage-path-planning):
  //   1. Compute coverage path (this node)
  //   2. Drive the robot to the F2C plan's first pose using a
  //      separate Nav2 goal (NavigateToFirstStripPose BT node, added
  //      below) — Smac plans, RPP follows
  //   3. Then run FollowCoveragePath on the unmodified F2C plan
  //
  // With the robot already at path[0] when FollowCoveragePath starts,
  // MPPI's PathHandler's closest-point search lands at index 0 cleanly
  // and the local plan walks forward through the strips without any
  // synthetic lead-in confusing it.

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
// NavigateToFirstStripPose — pre-FollowCoveragePath transit to F2C plan[0]
// ===========================================================================

BT::NodeStatus NavigateToFirstStripPose::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->current_coverage_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateToFirstStripPose: empty coverage path — nothing to do");
    return BT::NodeStatus::SUCCESS;
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<NavAction>(
        ctx->node, "/navigate_to_pose");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "NavigateToFirstStripPose: /navigate_to_pose action not available");
    return BT::NodeStatus::FAILURE;
  }

  // If we're already within 0.30 m of the first strip pose, skip — the
  // existing pose is good enough for MPPI to start tracking. This keeps
  // the second-and-later areas in a multi-area mow from doing a needless
  // round-trip to a strip endpoint that's already reachable.
  const auto& target = ctx->current_coverage_path.poses.front().pose.position;
  try
  {
    auto base_to_map = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
    const double dx = target.x - base_to_map.transform.translation.x;
    const double dy = target.y - base_to_map.transform.translation.y;
    if (std::hypot(dx, dy) < 0.30)
    {
      RCLCPP_INFO(ctx->node->get_logger(),
                  "NavigateToFirstStripPose: already within 0.30 m of strip[0] — skipping");
      return BT::NodeStatus::SUCCESS;
    }
  }
  catch (const tf2::TransformException&)
  {
    // proceed with NavigateToPose if we can't measure
  }

  NavAction::Goal goal;
  goal.pose = ctx->current_coverage_path.poses.front();
  if (goal.pose.header.frame_id.empty())
    goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = ctx->node->now();

  goal_handle_.reset();
  result_requested_ = false;
  goal_future_ = action_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateToFirstStripPose: navigating to F2C strip[0] = (%.2f, %.2f)",
              goal.pose.pose.position.x, goal.pose.pose.position.y);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToFirstStripPose::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "NavigateToFirstStripPose: goal rejected by /navigate_to_pose");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }

  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  const auto wrapped = result_future_.get();
  switch (wrapped.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(),
                  "NavigateToFirstStripPose: arrived at strip[0]");
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      RCLCPP_WARN(ctx->node->get_logger(),
                  "NavigateToFirstStripPose: nav_to_pose terminated code=%d",
                  static_cast<int>(wrapped.code));
      return BT::NodeStatus::FAILURE;
  }
}

void NavigateToFirstStripPose::onHalted()
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

  // Reset the driven-track accumulator. Every onRunning tick we'll
  // append the robot's current pose; on SUCCESS we paint mow_progress
  // along *that* track, not along the planned path.
  driven_trajectory_ = nav_msgs::msg::Path();
  driven_trajectory_.header.frame_id = "map";
  recordPose();

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

  // Sample the robot's current pose every tick into the driven-track
  // accumulator (decimated by min_pose_step_m_). This is what we paint
  // on SUCCESS — not the planned path.
  recordPose();

  const auto status = follow_handle_->get_status();

  if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowCoveragePath: complete — painting %zu driven poses (planned: %zu)",
                driven_trajectory_.poses.size(),
                ctx->current_coverage_path.poses.size());
    paintTrajectory(driven_trajectory_);
    setBladeEnabled(false);
    follow_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
  {
    // Even on a partial drive we paint what the robot actually visited so
    // a re-plan from this state knows which cells are already done.
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowCoveragePath: aborted/canceled — painting %zu driven poses anyway",
                driven_trajectory_.poses.size());
    paintTrajectory(driven_trajectory_);
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

void FollowCoveragePath::recordPose()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException&)
  {
    return;  // skip this tick — TF not ready
  }
  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  if (!driven_trajectory_.poses.empty())
  {
    const auto& last = driven_trajectory_.poses.back().pose.position;
    const double dx = x - last.x;
    const double dy = y - last.y;
    if (dx * dx + dy * dy < min_pose_step_m_ * min_pose_step_m_)
      return;  // didn't move far enough — drop sample
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.header.stamp = ctx->node->now();
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation = tf.transform.rotation;
  driven_trajectory_.poses.push_back(ps);
}

void FollowCoveragePath::paintTrajectory(const nav_msgs::msg::Path& trajectory)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (trajectory.poses.empty())
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
  req->swath_path = trajectory;
  paint_client_->async_send_request(req);
}

// ===========================================================================
// MowAreaWithCoverage — sends a /navigate_complete_coverage goal to
// bt_navigator, which runs the coverage_bt.xml internally. Blade ON
// while the action is RUNNING; mow_progress painted along the driven
// track on SUCCESS.
// ===========================================================================

BT::NodeStatus MowAreaWithCoverage::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);

  // Fetch the polygon (+ filtered holes) from map_server.
  if (!area_client_)
  {
    area_client_ = ctx->helper_node->create_client<mowgli_interfaces::srv::GetMowingArea>(
        "/map_server_node/get_mowing_area");
  }
  if (!area_client_->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "MowAreaWithCoverage: get_mowing_area not available");
    return BT::NodeStatus::FAILURE;
  }
  auto req = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  req->index = area_idx;
  auto fut = area_client_->async_send_request(req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (fut.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() > deadline)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "MowAreaWithCoverage: get_mowing_area timed out for area %u",
                   area_idx);
      return BT::NodeStatus::FAILURE;
    }
  }
  auto resp = fut.get();
  if (!resp->success)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "MowAreaWithCoverage: get_mowing_area returned failure for area %u",
                 area_idx);
    return BT::NodeStatus::FAILURE;
  }
  if (resp->area.is_navigation_area)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "MowAreaWithCoverage: area %u is navigation-only — skipping",
                area_idx);
    return BT::NodeStatus::SUCCESS;
  }

  if (!action_client_)
  {
    action_client_ = rclcpp_action::create_client<NavCovAction>(
        ctx->node, "/navigate_complete_coverage");
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "MowAreaWithCoverage: /navigate_complete_coverage action not available");
    return BT::NodeStatus::FAILURE;
  }

  // Apply the same hole filter we used in the manual ComputeCoveragePath
  // to ensure F2C 1.2.1 doesn't hang on degenerate holes (size, vertex-
  // inside-field, boundary clearance).
  constexpr double kMinHoleAreaM2 = 0.04;
  constexpr double kHoleBoundaryClearanceM = 0.20;
  constexpr float kHeadlandWidth = 1.00f;
  constexpr std::size_t kMaxF2cHoles = 16;
  const double min_clearance =
      static_cast<double>(kHeadlandWidth) + kHoleBoundaryClearanceM;

  NavCovAction::Goal goal;
  goal.frame_id = "map";

  // Outer field polygon — opennav_coverage requires the first polygon
  // in the goal's polygons[] array to be the field boundary.
  geometry_msgs::msg::Polygon outer = resp->area.area;
  // F2C wants the ring closed (first == last); mowgli stores polygons
  // open. Match the to_closed_ring helper from ComputeCoveragePath.
  if (!outer.points.empty())
  {
    const auto& first = outer.points.front();
    const auto& last = outer.points.back();
    if (std::fabs(first.x - last.x) > 1e-6 || std::fabs(first.y - last.y) > 1e-6)
      outer.points.push_back(first);
  }
  goal.polygons.push_back(outer);

  // Inner rings (holes) — same filter pass as the old ComputeCoveragePath.
  std::size_t filtered_too_small = 0, filtered_outside = 0, filtered_capped = 0;
  for (const auto& obs : resp->area.obstacles)
  {
    if (obs.points.size() < 3 ||
        ComputeCoveragePath::polygonArea(obs) < kMinHoleAreaM2) {
      ++filtered_too_small;
      continue;
    }
    if (!ComputeCoveragePath::isHoleSafeForF2C(
            obs, resp->area.area, kMinHoleAreaM2, min_clearance)) {
      ++filtered_outside;
      continue;
    }
    if (goal.polygons.size() > kMaxF2cHoles) {
      ++filtered_capped;
      continue;
    }
    geometry_msgs::msg::Polygon hole = obs;
    if (!hole.points.empty())
    {
      const auto& f = hole.points.front();
      const auto& l = hole.points.back();
      if (std::fabs(f.x - l.x) > 1e-6 || std::fabs(f.y - l.y) > 1e-6)
        hole.points.push_back(f);
    }
    goal.polygons.push_back(hole);
  }

  if (filtered_too_small || filtered_outside || filtered_capped)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "MowAreaWithCoverage: filtered obstacles "
                "(too_small=%zu, on_or_outside_boundary=%zu, capped=%zu)",
                filtered_too_small, filtered_outside, filtered_capped);
  }

  goal_handle_.reset();
  result_requested_ = false;
  goal_future_ = action_client_->async_send_goal(goal);

  // Reset driven-track accumulator and turn the blade on.
  driven_trajectory_ = nav_msgs::msg::Path();
  driven_trajectory_.header.frame_id = "map";
  setBladeEnabled(true);
  recordPose();

  RCLCPP_INFO(ctx->node->get_logger(),
              "MowAreaWithCoverage: sent /navigate_complete_coverage goal "
              "for area %u (%zu vertices, %zu holes)",
              area_idx, resp->area.area.points.size(), goal.polygons.size() - 1);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MowAreaWithCoverage::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!goal_handle_)
  {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    goal_handle_ = goal_future_.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "MowAreaWithCoverage: goal rejected by /navigate_complete_coverage");
      setBladeEnabled(false);
      return BT::NodeStatus::FAILURE;
    }
  }

  // Sample driven-track every tick.
  recordPose();

  if (!result_requested_)
  {
    result_future_ = action_client_->async_get_result(goal_handle_);
    result_requested_ = true;
  }
  if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  const auto wrapped = result_future_.get();
  switch (wrapped.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(),
                  "MowAreaWithCoverage: complete — painting %zu driven poses",
                  driven_trajectory_.poses.size());
      paintTrajectory(driven_trajectory_);
      setBladeEnabled(false);
      goal_handle_.reset();
      return BT::NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      RCLCPP_WARN(ctx->node->get_logger(),
                  "MowAreaWithCoverage: terminated code=%d, error_code=%u — "
                  "painting %zu driven poses anyway",
                  static_cast<int>(wrapped.code),
                  wrapped.result ? wrapped.result->error_code : 0u,
                  driven_trajectory_.poses.size());
      paintTrajectory(driven_trajectory_);
      setBladeEnabled(false);
      goal_handle_.reset();
      return BT::NodeStatus::FAILURE;
  }
}

void MowAreaWithCoverage::onHalted()
{
  if (goal_handle_)
    action_client_->async_cancel_goal(goal_handle_);
  goal_handle_.reset();
  setBladeEnabled(false);
}

void MowAreaWithCoverage::setBladeEnabled(bool enabled)
{
  if (blade_on_ == enabled) return;
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!blade_client_)
  {
    blade_client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }
  if (!blade_client_->wait_for_service(std::chrono::milliseconds(200))) return;
  auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  req->mow_enabled = enabled ? 1u : 0u;
  blade_client_->async_send_request(req);
  blade_on_ = enabled;
}

void MowAreaWithCoverage::recordPose()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException&) { return; }
  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;
  if (!driven_trajectory_.poses.empty())
  {
    const auto& last = driven_trajectory_.poses.back().pose.position;
    const double dx = x - last.x, dy = y - last.y;
    if (dx * dx + dy * dy < min_pose_step_m_ * min_pose_step_m_) return;
  }
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.header.stamp = ctx->node->now();
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = 0.0;
  ps.pose.orientation = tf.transform.rotation;
  driven_trajectory_.poses.push_back(ps);
}

void MowAreaWithCoverage::paintTrajectory(const nav_msgs::msg::Path& trajectory)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (trajectory.poses.empty()) return;
  if (!paint_client_)
  {
    paint_client_ = ctx->node->create_client<mowgli_interfaces::srv::PaintSwath>(
        "/map_server_node/paint_swath");
  }
  if (!paint_client_->wait_for_service(std::chrono::milliseconds(500))) return;
  auto req = std::make_shared<mowgli_interfaces::srv::PaintSwath::Request>();
  req->swath_path = trajectory;
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

// ===========================================================================
// ComputeCoveragePath — geometry helpers (static, unit-tested)
// ===========================================================================

double ComputeCoveragePath::polygonArea(const geometry_msgs::msg::Polygon& p)
{
  if (p.points.size() < 3) return 0.0;
  double a = 0.0;
  const auto& pts = p.points;
  for (std::size_t i = 0, n = pts.size(); i < n; ++i) {
    const auto& cur = pts[i];
    const auto& nxt = pts[(i + 1) % n];
    a += static_cast<double>(cur.x) * static_cast<double>(nxt.y) -
         static_cast<double>(nxt.x) * static_cast<double>(cur.y);
  }
  return std::abs(a) * 0.5;
}

bool ComputeCoveragePath::pointInPolygon(double x, double y,
                                         const geometry_msgs::msg::Polygon& p)
{
  if (p.points.size() < 3) return false;
  bool inside = false;
  const auto& pts = p.points;
  for (std::size_t i = 0, j = pts.size() - 1, n = pts.size(); i < n; j = i++) {
    const double xi = pts[i].x, yi = pts[i].y;
    const double xj = pts[j].x, yj = pts[j].y;
    // 1e-12 guards a horizontal segment from dividing by zero — a vertex
    // exactly on @y is otherwise ambiguous (this is the standard
    // "even-odd" point-in-polygon edge case).
    const bool intersects = ((yi > y) != (yj > y)) &&
                            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);
    if (intersects) inside = !inside;
  }
  return inside;
}

double ComputeCoveragePath::distanceToPolygonBoundary(
    double x, double y, const geometry_msgs::msg::Polygon& p)
{
  if (p.points.empty()) return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  const auto& pts = p.points;
  for (std::size_t i = 0, n = pts.size(); i < n; ++i) {
    const auto& a = pts[i];
    const auto& b = pts[(i + 1) % n];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    double t = len_sq > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / len_sq : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double px = a.x + t * dx, py = a.y + t * dy;
    const double d = std::hypot(x - px, y - py);
    if (d < best) best = d;
  }
  return best;
}

bool ComputeCoveragePath::isHoleSafeForF2C(
    const geometry_msgs::msg::Polygon& obs,
    const geometry_msgs::msg::Polygon& field,
    double min_area_m2,
    double min_clearance_m)
{
  if (obs.points.size() < 3) return false;
  if (polygonArea(obs) < min_area_m2) return false;
  for (const auto& v : obs.points) {
    if (!pointInPolygon(v.x, v.y, field)) return false;
    if (distanceToPolygonBoundary(v.x, v.y, field) < min_clearance_m) return false;
  }
  return true;
}

}  // namespace mowgli_behavior
