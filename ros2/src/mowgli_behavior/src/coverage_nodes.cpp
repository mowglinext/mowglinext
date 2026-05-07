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

  // Stash the per-strip endpoints (swath start/end pairs) separately so
  // FollowSwathsWithSpin can iterate them without parsing the connected
  // nav_path. F2C already emits this structured data in the action result
  // — coverage_path.swaths[] is the ordered list of straight-line strips
  // and coverage_path.turns[] is the per-transition arc paths we don't
  // want to follow. Discarding turns gives us pure strip endpoints for
  // explicit Spin + FollowPath orchestration.
  ctx->current_swaths = wrapped.result->coverage_path.swaths;

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

  // Prefer the SWATH endpoint over nav_path[0]. F2C's nav_path may
  // start with a connector arc *before* the first strip — so
  // nav_path[0] is the start of the lead-in, not the strip start
  // itself. Sim 48 BV at (-3.01, 1.65): NavigateToFirstStripPose
  // landed the robot 0.65 m WEST of strip[0] (planned at X=-2.36),
  // and FollowSwathsWithSpin's MPPI then carved the boundary trying
  // to recover. Driving to swath[0].start directly avoids this.
  if (ctx->current_swaths.empty() && ctx->current_coverage_path.poses.empty())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "NavigateToFirstStripPose: no swaths or coverage path — nothing to do");
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

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ctx->node->now();

  if (!ctx->current_swaths.empty())
  {
    const auto& sw = ctx->current_swaths.front();
    target_pose.pose.position.x = sw.start.x;
    target_pose.pose.position.y = sw.start.y;
    // Goal yaw = strip[0] heading so NavigateToPose's RPP arrives
    // already aligned with the strip — the subsequent Spin in
    // FollowSwathsWithSpin then has near-zero delta and the
    // FollowPath starts cleanly on the strip line.
    const double yaw = std::atan2(sw.end.y - sw.start.y,
                                   sw.end.x - sw.start.x);
    target_pose.pose.orientation.z = std::sin(yaw / 2.0);
    target_pose.pose.orientation.w = std::cos(yaw / 2.0);
  }
  else
  {
    target_pose = ctx->current_coverage_path.poses.front();
    if (target_pose.header.frame_id.empty())
      target_pose.header.frame_id = "map";
    target_pose.header.stamp = ctx->node->now();
  }

  // Skip if already at target (within 0.30 m) — multi-area mow case.
  try
  {
    auto base_to_map = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
    const double dx = target_pose.pose.position.x - base_to_map.transform.translation.x;
    const double dy = target_pose.pose.position.y - base_to_map.transform.translation.y;
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
  goal.pose = target_pose;

  goal_handle_.reset();
  result_requested_ = false;
  goal_future_ = action_client_->async_send_goal(goal);

  RCLCPP_INFO(ctx->node->get_logger(),
              "NavigateToFirstStripPose: navigating to swath[0].start=(%.2f, %.2f)",
              target_pose.pose.position.x, target_pose.pose.position.y);
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
// MowHeadlandPerimeter — closed-loop perimeter sweep using F2C's
// /coverage_server/planning_field (inset polygon = field minus
// headland_width). Mows the perimeter band before strips so strip-end
// transitions happen inside the already-mowed headland zone.
// ===========================================================================

namespace
{
/// Build a closed-loop nav_msgs/Path tracing @p poly's vertices with
/// dense intermediate poses every @p step_m so MPPI's PathHandler has
/// enough waypoints to track each edge. Starting vertex is whichever
/// is closest to (start_x, start_y); loop returns to that vertex.
nav_msgs::msg::Path buildPerimeterPath(
    const geometry_msgs::msg::PolygonStamped& poly,
    double start_x, double start_y, double step_m,
    const rclcpp::Time& now_stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = poly.header.frame_id.empty() ? "map" : poly.header.frame_id;
  path.header.stamp = now_stamp;

  if (poly.polygon.points.size() < 3) return path;

  // Find closest vertex to robot.
  std::size_t start_idx = 0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < poly.polygon.points.size(); ++i)
  {
    const auto& p = poly.polygon.points[i];
    const double d = std::hypot(p.x - start_x, p.y - start_y);
    if (d < best) { best = d; start_idx = i; }
  }

  const std::size_t n = poly.polygon.points.size();
  // Walk vertices: start_idx, start_idx+1, ..., start_idx (back to start).
  for (std::size_t k = 0; k <= n; ++k)
  {
    const auto& a = poly.polygon.points[(start_idx + k) % n];
    const auto& b = poly.polygon.points[(start_idx + k + 1) % n];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double L = std::hypot(dx, dy);
    if (L < 1e-6) continue;
    const double ux = dx / L;
    const double uy = dy / L;
    const double yaw = std::atan2(dy, dx);
    const double qz = std::sin(yaw / 2.0);
    const double qw = std::cos(yaw / 2.0);
    const std::size_t steps = std::max<std::size_t>(2,
        static_cast<std::size_t>(std::ceil(L / step_m)));

    for (std::size_t s = 0; s < steps; ++s)
    {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = a.x + ux * (L * t);
      ps.pose.position.y = a.y + uy * (L * t);
      ps.pose.orientation.z = qz;
      ps.pose.orientation.w = qw;
      path.poses.push_back(ps);
    }
    // The last vertex of the loop is appended once we exit the for-k.
    if (k + 1 == n)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = b.x;
      ps.pose.position.y = b.y;
      ps.pose.orientation.z = qz;
      ps.pose.orientation.w = qw;
      path.poses.push_back(ps);
    }
  }
  return path;
}
}  // namespace

BT::NodeStatus MowHeadlandPerimeter::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Wait for planning_field if not yet received (transient_local sub
  // should normally have it immediately after ComputeCoveragePath).
  if (!ctx->current_planning_field_valid)
  {
    waiting_for_planning_field_ = true;
    wait_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(ctx->node->get_logger(),
                "MowHeadlandPerimeter: waiting for /coverage_server/planning_field");
    return BT::NodeStatus::RUNNING;
  }

  // Build the perimeter path and dispatch FollowPath.
  geometry_msgs::msg::TransformStamped tf;
  double rx = 0.0, ry = 0.0;
  try
  {
    tf = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.5));
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "MowHeadlandPerimeter: TF lookup failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PolygonStamped planning_field;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    planning_field = ctx->current_planning_field;
  }
  if (planning_field.polygon.points.size() < 3)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "MowHeadlandPerimeter: planning_field has <3 vertices — skipping");
    return BT::NodeStatus::SUCCESS;
  }

  if (!follow_client_)
  {
    follow_client_ = rclcpp_action::create_client<FollowPathAction>(
        ctx->node, "/follow_path");
  }
  if (!follow_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "MowHeadlandPerimeter: /follow_path not available");
    return BT::NodeStatus::FAILURE;
  }

  FollowPathAction::Goal goal;
  goal.path = buildPerimeterPath(planning_field, rx, ry, 0.10, ctx->node->now());
  // RPP for perimeter (was MPPI). Same wobble failure mode applies —
  // see FollowSwathsWithSpin::startFollow for the rationale.
  goal.controller_id = "FollowPath";
  goal.goal_checker_id = "coverage_goal_checker";

  follow_handle_.reset();
  result_requested_ = false;
  follow_future_ = follow_client_->async_send_goal(goal);

  driven_trajectory_ = nav_msgs::msg::Path();
  driven_trajectory_.header.frame_id = "map";
  setBladeEnabled(true);
  recordPose();

  RCLCPP_INFO(ctx->node->get_logger(),
              "MowHeadlandPerimeter: sent perimeter follow_path with %zu poses "
              "(%zu polygon vertices, starting near robot at (%.2f, %.2f))",
              goal.path.poses.size(), planning_field.polygon.points.size(), rx, ry);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MowHeadlandPerimeter::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  recordPose();

  if (waiting_for_planning_field_)
  {
    if (ctx->current_planning_field_valid)
    {
      waiting_for_planning_field_ = false;
      // Re-run the start logic now that we have data.
      return onStart();
    }
    if (std::chrono::steady_clock::now() - wait_start_ > std::chrono::seconds(5))
    {
      RCLCPP_WARN(ctx->node->get_logger(),
                  "MowHeadlandPerimeter: timed out waiting for planning_field — skipping");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!follow_handle_)
  {
    if (follow_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      return BT::NodeStatus::RUNNING;
    follow_handle_ = follow_future_.get();
    if (!follow_handle_)
    {
      RCLCPP_ERROR(ctx->node->get_logger(),
                   "MowHeadlandPerimeter: /follow_path goal rejected");
      setBladeEnabled(false);
      paintTrajectory(driven_trajectory_);
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!result_requested_)
  {
    result_future_ = follow_client_->async_get_result(follow_handle_);
    result_requested_ = true;
  }
  if (result_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    return BT::NodeStatus::RUNNING;

  const auto wrapped = result_future_.get();
  switch (wrapped.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(ctx->node->get_logger(),
                  "MowHeadlandPerimeter: complete — painting %zu driven poses",
                  driven_trajectory_.poses.size());
      paintTrajectory(driven_trajectory_);
      setBladeEnabled(false);
      follow_handle_.reset();
      return BT::NodeStatus::SUCCESS;
    default:
      RCLCPP_WARN(ctx->node->get_logger(),
                  "MowHeadlandPerimeter: terminated code=%d — painting %zu driven poses anyway",
                  static_cast<int>(wrapped.code), driven_trajectory_.poses.size());
      paintTrajectory(driven_trajectory_);
      setBladeEnabled(false);
      follow_handle_.reset();
      // Don't fail the BT — strips can still proceed.
      return BT::NodeStatus::SUCCESS;
  }
}

void MowHeadlandPerimeter::onHalted()
{
  if (follow_handle_)
  {
    follow_client_->async_cancel_goal(follow_handle_);
  }
  setBladeEnabled(false);
}

void MowHeadlandPerimeter::setBladeEnabled(bool enabled)
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

void MowHeadlandPerimeter::recordPose()
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
  ps.pose.orientation = tf.transform.rotation;
  driven_trajectory_.poses.push_back(ps);
}

void MowHeadlandPerimeter::paintTrajectory(const nav_msgs::msg::Path& trajectory)
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
// FollowSwathsWithSpin — pivot-in-place orchestrator over F2C swaths.
//
// State machine:
//   onStart  → SPIN to align with strip[0] heading
//   SPIN     → on success: FOLLOW
//   FOLLOW   → on success: NEXT_STRIP
//   NEXT_STRIP → strip_idx++; if more strips → SPIN, else DONE_OK
// ===========================================================================

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

double FollowSwathsWithSpin::swathHeadingRad(std::size_t idx) const
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto& sw = ctx->current_swaths.at(idx);
  return std::atan2(sw.end.y - sw.start.y, sw.end.x - sw.start.x);
}

double FollowSwathsWithSpin::swathLengthM(std::size_t idx) const
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  const auto& sw = ctx->current_swaths.at(idx);
  return std::hypot(sw.end.x - sw.start.x, sw.end.y - sw.start.y);
}

BT::NodeStatus FollowSwathsWithSpin::startSpinTo(double target_heading)
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  // Look up current robot yaw via TF.
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = ctx->tf_buffer->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.2));
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowSwathsWithSpin: TF lookup failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  const double current_yaw = yawFromQuat(tf.transform.rotation);
  const double delta = wrapAngle(target_heading - current_yaw);

  if (!spin_client_)
  {
    spin_client_ = rclcpp_action::create_client<SpinAction>(ctx->node, "/spin");
  }
  if (!spin_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowSwathsWithSpin: /spin action server not available");
    return BT::NodeStatus::FAILURE;
  }

  SpinAction::Goal goal;
  goal.target_yaw = delta;
  goal.time_allowance = rclcpp::Duration::from_seconds(15.0);
  spin_handle_.reset();
  spin_result_requested_ = false;
  spin_future_ = spin_client_->async_send_goal(goal);
  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowSwathsWithSpin: spin strip[%zu] heading %.1f° "
              "(current yaw %.1f° → delta %.1f°)",
              strip_idx_, target_heading * 180.0 / M_PI,
              current_yaw * 180.0 / M_PI, delta * 180.0 / M_PI);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowSwathsWithSpin::startDrive()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!latest_odom_)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowSwathsWithSpin: no /odom message received yet — "
                 "cannot start strip drive");
    return BT::NodeStatus::FAILURE;
  }

  // Capture initial yaw + position in odom frame at the moment the spin
  // ended. Yaw is what we'll PID against (gyro lock — drift-free vs map),
  // position is the reference for measuring strip travel distance.
  drive_target_yaw_odom_ = yawFromQuat(latest_odom_->pose.pose.orientation);
  drive_start_x_odom_ = latest_odom_->pose.pose.position.x;
  drive_start_y_odom_ = latest_odom_->pose.pose.position.y;
  drive_target_distance_m_ = swathLengthM(strip_idx_);
  drive_start_time_ = ctx->node->now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowSwathsWithSpin: drive strip[%zu] gyro-lock yaw=%.1f° "
              "len=%.2fm (start odom=(%.2f,%.2f))",
              strip_idx_, drive_target_yaw_odom_ * 180.0 / M_PI,
              drive_target_distance_m_,
              drive_start_x_odom_, drive_start_y_odom_);
  return BT::NodeStatus::RUNNING;
}

double FollowSwathsWithSpin::minRangeInArc(double arc_min_rad,
                                            double arc_max_rad) const
{
  if (!latest_scan_) return std::numeric_limits<double>::infinity();
  const auto& s = *latest_scan_;
  const std::size_t n = s.ranges.size();
  if (n == 0) return std::numeric_limits<double>::infinity();
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i)
  {
    const double a = s.angle_min + i * s.angle_increment;
    if (a < arc_min_rad || a > arc_max_rad) continue;
    const double r = s.ranges[i];
    if (!std::isfinite(r) || r < s.range_min || r > s.range_max) continue;
    if (r < best) best = r;
  }
  return best;
}

void FollowSwathsWithSpin::tickDrive()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (!latest_odom_) return;  // wait for odom

  // 1. Distance check — strip done?
  const double dx = latest_odom_->pose.pose.position.x - drive_start_x_odom_;
  const double dy = latest_odom_->pose.pose.position.y - drive_start_y_odom_;
  const double traveled = std::hypot(dx, dy);
  if (traveled >= drive_target_distance_m_)
  {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = ctx->node->now();
    stop.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop);
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowSwathsWithSpin: strip[%zu] complete (traveled %.2fm)",
                strip_idx_, traveled);
    state_ = State::NEXT_STRIP;
    return;
  }

  // 2. Watchdog — abort the strip if it takes more than 3× nominal time.
  const double nominal_time = drive_target_distance_m_ / 0.15;  // cruise speed
  const double elapsed = (ctx->node->now() - drive_start_time_).seconds();
  if (elapsed > 3.0 * nominal_time + 10.0)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowSwathsWithSpin: strip[%zu] watchdog (%.0fs elapsed, "
                "traveled %.2fm of %.2fm) — advancing",
                strip_idx_, elapsed, traveled, drive_target_distance_m_);
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = ctx->node->now();
    stop.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop);
    state_ = State::NEXT_STRIP;
    return;
  }

  // 3. LiDAR forward-arc clearance — if blocked, swerve.
  // Arcs in robot frame: forward = 0 rad, +pi/2 = left, -pi/2 = right.
  constexpr double kFwdHalfArc = 0.44;     // ±25°
  constexpr double kSideMin = 0.44;        // 25° (start of side arc)
  constexpr double kSideMax = 1.57;        // 90°
  constexpr double kAvoidThreshold = 0.50; // m
  constexpr double kCruiseSpeed = 0.15;    // m/s
  constexpr double kAvoidLinear = 0.07;    // m/s while swerving
  constexpr double kAvoidAngular = 0.5;    // rad/s during swerve
  constexpr double kHeadingKp = 1.0;       // P-gain on yaw error
  constexpr double kHeadingMaxAng = 0.4;   // rad/s cap during normal drive

  const double fwd_clear = minRangeInArc(-kFwdHalfArc, +kFwdHalfArc);
  const double left_clear = minRangeInArc(+kSideMin, +kSideMax);
  const double right_clear = minRangeInArc(-kSideMax, -kSideMin);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = ctx->node->now();
  cmd.header.frame_id = "base_link";

  if (fwd_clear < kAvoidThreshold)
  {
    // Obstacle ahead — swerve to the side with more clearance.
    cmd.twist.linear.x = kAvoidLinear;
    cmd.twist.angular.z = (left_clear > right_clear) ? +kAvoidAngular
                                                     : -kAvoidAngular;
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "strip[%zu] AVOID fwd=%.2f L=%.2f R=%.2f → wz=%+.2f",
                 strip_idx_, fwd_clear, left_clear, right_clear,
                 cmd.twist.angular.z);
  }
  else
  {
    // Heading PID against locked odom yaw.
    const double current_yaw = yawFromQuat(latest_odom_->pose.pose.orientation);
    const double yaw_err = wrapAngle(drive_target_yaw_odom_ - current_yaw);
    cmd.twist.linear.x = kCruiseSpeed;
    cmd.twist.angular.z = std::clamp(kHeadingKp * yaw_err,
                                     -kHeadingMaxAng, kHeadingMaxAng);
  }

  cmd_vel_pub_->publish(cmd);
}

BT::NodeStatus FollowSwathsWithSpin::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  uint32_t area_idx = 0;
  getInput<uint32_t>("area_index", area_idx);
  area_idx_ = area_idx;

  if (ctx->current_swaths.empty())
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "FollowSwathsWithSpin: ctx->current_swaths empty — did "
                 "ComputeCoveragePath run first?");
    return BT::NodeStatus::FAILURE;
  }

  // Lazy-create the publisher / subscribers on first activation. They
  // outlive any single strip; reused across strips and across re-entries.
  if (!cmd_vel_pub_)
  {
    cmd_vel_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_nav", rclcpp::QoS(10));
  }
  if (!scan_sub_)
  {
    scan_sub_ = ctx->node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          latest_scan_ = msg;
        });
  }
  if (!odom_sub_)
  {
    odom_sub_ = ctx->node->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::QoS(10),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          latest_odom_ = msg;
        });
  }

  strip_idx_ = 0;
  current_strip_heading_ = swathHeadingRad(strip_idx_);

  driven_trajectory_ = nav_msgs::msg::Path();
  driven_trajectory_.header.frame_id = "map";
  setBladeEnabled(true);
  recordPose();

  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowSwathsWithSpin: starting %zu strips for area %u "
              "(gyro-locked drive + LiDAR-reactive avoidance)",
              ctx->current_swaths.size(), area_idx_);

  state_ = State::SPIN;
  return startSpinTo(current_strip_heading_);
}

BT::NodeStatus FollowSwathsWithSpin::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  recordPose();

  switch (state_)
  {
    case State::SPIN:
    {
      if (!spin_handle_)
      {
        if (spin_future_.wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready)
          return BT::NodeStatus::RUNNING;
        spin_handle_ = spin_future_.get();
        if (!spin_handle_)
        {
          RCLCPP_ERROR(ctx->node->get_logger(),
                       "FollowSwathsWithSpin: /spin goal rejected");
          state_ = State::DONE_FAIL;
          break;
        }
      }
      if (!spin_result_requested_)
      {
        spin_result_future_ = spin_client_->async_get_result(spin_handle_);
        spin_result_requested_ = true;
      }
      if (spin_result_future_.wait_for(std::chrono::milliseconds(0)) !=
          std::future_status::ready)
        return BT::NodeStatus::RUNNING;
      const auto wrapped = spin_result_future_.get();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_WARN(ctx->node->get_logger(),
                    "FollowSwathsWithSpin: spin code=%d — proceeding to follow anyway",
                    static_cast<int>(wrapped.code));
        // Don't fail on spin imperfection; FollowPath will refine heading.
      }
      // Spin done — start the gyro-locked drive for this strip.
      const auto status = startDrive();
      if (status == BT::NodeStatus::FAILURE)
      {
        state_ = State::DONE_FAIL;
        break;
      }
      state_ = State::DRIVE;
      break;
    }
    case State::DRIVE:
    {
      tickDrive();
      // tickDrive transitions state_ to NEXT_STRIP on completion or
      // watchdog timeout; otherwise keep RUNNING.
      if (state_ == State::DRIVE) return BT::NodeStatus::RUNNING;
      break;
    }
    case State::NEXT_STRIP:
    {
      ++strip_idx_;
      if (strip_idx_ >= ctx->current_swaths.size())
      {
        state_ = State::DONE_OK;
        break;
      }
      current_strip_heading_ = swathHeadingRad(strip_idx_);
      state_ = State::SPIN;
      const auto status = startSpinTo(current_strip_heading_);
      if (status == BT::NodeStatus::FAILURE) state_ = State::DONE_FAIL;
      break;
    }
    case State::DONE_OK:
    case State::DONE_FAIL:
      break;
  }

  if (state_ == State::DONE_OK)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "FollowSwathsWithSpin: complete — %zu strips done, painting "
                "%zu driven poses", ctx->current_swaths.size(),
                driven_trajectory_.poses.size());
    paintTrajectory(driven_trajectory_);
    setBladeEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }
  if (state_ == State::DONE_FAIL)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "FollowSwathsWithSpin: failed at strip[%zu] — painting %zu "
                "driven poses anyway", strip_idx_, driven_trajectory_.poses.size());
    paintTrajectory(driven_trajectory_);
    setBladeEnabled(false);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void FollowSwathsWithSpin::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (spin_handle_) spin_client_->async_cancel_goal(spin_handle_);
  // Stop the robot — we're driving via direct cmd_vel, not an action.
  if (cmd_vel_pub_)
  {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = ctx->node->now();
    stop.header.frame_id = "base_link";
    cmd_vel_pub_->publish(stop);
  }
  setBladeEnabled(false);
  RCLCPP_INFO(ctx->node->get_logger(),
              "FollowSwathsWithSpin: halted at strip[%zu]/%zu", strip_idx_,
              ctx->current_swaths.size());
}

void FollowSwathsWithSpin::setBladeEnabled(bool enabled)
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

void FollowSwathsWithSpin::recordPose()
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
  ps.pose.orientation = tf.transform.rotation;
  driven_trajectory_.poses.push_back(ps);
}

void FollowSwathsWithSpin::paintTrajectory(const nav_msgs::msg::Path& trajectory)
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
