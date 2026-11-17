// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/authorized_transit_planner.hpp"

#include "mowgli_nav2_plugins/transit_raster.hpp"
#include "mowgli_nav2_plugins/transit_seams.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mowgli_nav2_plugins
{
using BasePlanner = nav2_smac_planner::SmacPlanner2DT<TransitNode>;
namespace
{
transit::Point point(const geometry_msgs::msg::PoseStamped& pose)
{
  return {pose.pose.position.x, pose.pose.position.y};
}
constexpr double kTerminalOutsideLimit = 0.15;
constexpr double kAnchorLimit = 0.30;
// A seam never bridges more than the slack a terminal leg is already granted outside the union.
constexpr double kMaxSeamTolerance = kTerminalOutsideLimit;
// How far a start inside the dock body may have to travel to reach clear ground: the body is
// 0.80 m deep and 0.55 m wide, so the way out is at most that, plus the cell grid.
constexpr double kDockExitLimit = 1.0;
// Margin between the charging station and anything the robot body may sweep while it pivots out
// of the dock body (localization error, the station's own contact arms).
constexpr double kStationClearance = 0.10;
}  // namespace

void AuthorizedTransitPlanner::configure(const nav2::LifecycleNode::WeakPtr& parent,
                                         std::string name,
                                         nav2::TransformBuffer::SharedPtr tf,
                                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (node->declare_or_get_parameter(name + ".tolerance", 0.0) != 0.0)
    throw nav2_core::PlannerException("Authorized transit requires exact goal anchors");
  // Widest gap between two recorded areas that still counts as one border drawn twice (see
  // transit_seams.hpp). 0 keeps the exact union; production sets it in nav2_params_base.yaml.
  seam_tolerance_ = node->declare_or_get_parameter(name + ".seam_tolerance", 0.0);
  if (!std::isfinite(seam_tolerance_) || seam_tolerance_ < 0 || seam_tolerance_ > kMaxSeamTolerance)
    throw nav2_core::PlannerException("Authorized transit seam_tolerance must be within 0..0.15 m");
  BasePlanner::configure(parent, name, tf, costmap_ros);
  if (_downsample_costmap)
    throw nav2_core::PlannerException(
        "Authorized transit requires the original costmap resolution");
  shared_map_ = _costmap;
  _costmap = &private_map_;
  _collision_checker.setCostmap(_costmap);
  // Keep the raw search result for a safe fallback if smoothing crosses a
  // polygon edge. The base smoother is a no-op; the original smoother is
  // applied below, then continuous geometry and obstacle costs are checked.
  transit_smoother_ = std::move(_smoother);
  nav2_smac_planner::SmootherParams no_op;
  no_op.tolerance_ = 1e-10;
  no_op.max_its_ = 1;
  no_op.w_data_ = 1;
  no_op.w_smooth_ = 0;
  no_op.holonomic_ = true;
  no_op.do_refinement_ = false;
  no_op.refinement_num_ = 0;
  _smoother = std::make_unique<nav2_smac_planner::Smoother>(no_op);
  _smoother->initialize(1e-50);
  geometry_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry",
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
      {
        onGeometry(msg);
      },
      rclcpp::QoS(1).transient_local());
  config_guard_ = node->add_on_set_parameters_callback(
      [name](const auto& parameters)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : parameters)
          if (p.get_name() == name + ".downsample_costmap" && p.as_bool())
          {
            result.successful = false;
            result.reason = "Authorized transit cannot downsample polygon authorization";
          }
          else if (p.get_name() == name + ".tolerance" && p.as_double() != 0.0)
          {
            result.successful = false;
            result.reason = "Authorized transit requires exact goal anchors";
          }
          else if (p.get_name() == name + ".seam_tolerance")
          {
            result.successful = false;
            result.reason = "Authorized transit reads seam_tolerance at configure only";
          }
        return result;
      });
}

void AuthorizedTransitPlanner::cleanup()
{
  geometry_sub_.reset();
  if (auto node = _node.lock(); node && config_guard_)
    node->remove_on_set_parameters_callback(config_guard_.get());
  config_guard_.reset();
  transit_smoother_.reset();
  {
    std::lock_guard lock(geometry_mutex_);
    geometry_valid_ = false;
    geometry_ = {};
    ++geometry_revision_;
  }
  BasePlanner::cleanup();
}

void AuthorizedTransitPlanner::onGeometry(
    visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
{
  transit::Geometry next;
  bool valid = !message->markers.empty();
  std::vector<transit::Ring> dock_bodies, dock_corridors;
  double dock_pose_ahead_of_rear = 0;
  for (const auto& marker : message->markers)
  {
    if (marker.header.frame_id != _global_frame || marker.points.size() < 3 ||
        marker.action != visualization_msgs::msg::Marker::ADD)
    {
      valid = false;
      break;
    }
    transit::Ring ring;
    for (const auto& p : marker.points)
    {
      valid &= std::isfinite(p.x) && std::isfinite(p.y);
      ring.push_back({p.x, p.y});
    }
    if (marker.ns == "dock_corridor")
      dock_corridors.push_back(ring);
    else if (marker.ns == "dock_body")
    {
      dock_bodies.push_back(ring);
      dock_pose_ahead_of_rear = marker.scale.y;
    }
    if (marker.ns == "area" || marker.ns == "dock_corridor")
      next.allowed.push_back(std::move(ring));
    else if (marker.ns == "obstacle" || marker.ns == "dock_body" || marker.ns == "peer_dock")
    {
      valid &= std::isfinite(marker.scale.x) && marker.scale.x >= 0;
      next.blocked_margins.push_back(marker.scale.x);
      next.start_exempt.push_back(marker.ns == "dock_body");
      next.margin_start_exempt.push_back(marker.ns == "peer_dock");
      next.blocked.push_back(std::move(ring));
    }
    else
      valid = false;
  }
  if (dock_bodies.size() == 1 && dock_corridors.size() == 1)
    next.dock =
        transit::dockFrame(dock_bodies.front(), dock_corridors.front(), dock_pose_ahead_of_rear);
  // Bridge borders drawn twice a few centimetres apart. Obstacles and the dock body are not
  // involved: seams only ever join two allowed rings, and blocked rings still win over them.
  if (valid)
  {
    auto seams = transit::seamRings(next.allowed, seam_tolerance_);
    if (!seams.empty())
      RCLCPP_INFO(_logger,
                  "Transit geometry: %zu seam stretch(es) bridge gaps up to %.2f m between "
                  "neighbouring areas",
                  seams.size(),
                  seam_tolerance_);
    next.allowed.insert(next.allowed.end(),
                        std::make_move_iterator(seams.begin()),
                        std::make_move_iterator(seams.end()));
  }
  std::lock_guard lock(geometry_mutex_);
  geometry_ = std::move(next);
  geometry_valid_ = valid && !geometry_.allowed.empty();
  ++geometry_revision_;
}

bool AuthorizedTransitPlanner::costSegmentClear(transit::Point a, transit::Point b) const
{
  std::vector<double> cuts;
  return costSegmentClear(a, b, cuts);
}

// The exit leg from a start in the dock zone: the mask stamps that zone lethal, so its cells cannot
// judge a leg leaving it. Cost is checked from the point where the leg has left the zone.
bool AuthorizedTransitPlanner::costClearOutsideStartExempt(transit::Point a,
                                                           transit::Point b,
                                                           const transit::Geometry& geometry) const
{
  const int steps = std::max(1,
                             static_cast<int>(std::ceil(transit::distance(a, b) /
                                                        (0.25 * private_map_.getResolution()))));
  for (int i = 0; i <= steps; ++i)
  {
    const auto p = transit::interpolate(a, b, static_cast<double>(i) / steps);
    if (!geometry.insideStartExempt(p) && !geometry.insideMarginStartExempt(p))
      return costSegmentClear(p, b);
  }
  return true;
}

bool AuthorizedTransitPlanner::costSegmentClear(transit::Point a,
                                                transit::Point b,
                                                std::vector<double>& ts) const
{
  // Split at grid lines: every crossed cell is checked, even a very short
  // segment clipping a lethal corner. The copy retains shared obstacle costs.
  ts.assign({0, 1});
  const auto split = [&](double start, double end, double origin)
  {
    if (start == end)
      return;
    const double resolution = private_map_.getResolution();
    const int first = static_cast<int>(std::floor((std::min(start, end) - origin) / resolution));
    const int last = static_cast<int>(std::floor((std::max(start, end) - origin) / resolution));
    for (int i = first + 1; i <= last; ++i)
      ts.push_back((origin + i * resolution - start) / (end - start));
  };
  split(a.x, b.x, private_map_.getOriginX());
  split(a.y, b.y, private_map_.getOriginY());
  std::sort(ts.begin(), ts.end());
  for (std::size_t i = 0; i < ts.size(); ++i)
  {
    for (double t : {ts[i], i ? (ts[i - 1] + ts[i]) / 2 : ts[i]})
    {
      const auto p = transit::interpolate(a, b, t);
      unsigned x, y;
      if (!private_map_.worldToMap(p.x, p.y, x, y) || private_map_.getCost(x, y) >= 253)
        return false;
      const double gx = (p.x - private_map_.getOriginX()) / private_map_.getResolution();
      const double gy = (p.y - private_map_.getOriginY()) / private_map_.getResolution();
      const bool on_x = std::abs(gx - std::round(gx)) <= transit::kEpsilon;
      const bool on_y = std::abs(gy - std::round(gy)) <= transit::kEpsilon;
      if ((on_x && x > 0 && private_map_.getCost(x - 1, y) >= 253) ||
          (on_y && y > 0 && private_map_.getCost(x, y - 1) >= 253) ||
          (on_x && on_y && x > 0 && y > 0 && private_map_.getCost(x - 1, y - 1) >= 253))
        return false;
    }
  }
  return true;
}

geometry_msgs::msg::PoseStamped AuthorizedTransitPlanner::anchor(
    const geometry_msgs::msg::PoseStamped& endpoint, const transit::Geometry& geometry) const
{
  const auto p = point(endpoint);
  unsigned mx, my;
  if (!private_map_.worldToMap(p.x, p.y, mx, my))
    throw nav2_core::NoValidPathCouldBeFound("Transit endpoint outside costmap");
  const int radius = static_cast<int>(std::ceil(kAnchorLimit / private_map_.getResolution()));
  double best = INFINITY;
  auto result = endpoint;
  for (int y = std::max(0, static_cast<int>(my) - radius);
       y <= std::min(static_cast<int>(private_map_.getSizeInCellsY()) - 1,
                     static_cast<int>(my) + radius);
       ++y)
    for (int x = std::max(0, static_cast<int>(mx) - radius);
         x <= std::min(static_cast<int>(private_map_.getSizeInCellsX()) - 1,
                       static_cast<int>(mx) + radius);
         ++x)
    {
      transit::Point q;
      private_map_.mapToWorld(x, y, q.x, q.y);
      const double d = transit::distance(p, q);
      if (d < best && d <= kAnchorLimit && private_map_.getCost(x, y) < 253 &&
          geometry.authorized(q) && geometry.terminal(p, q, kTerminalOutsideLimit) &&
          costSegmentClear(p, q))
      {
        best = d;
        result.pose.position.x = q.x;
        result.pose.position.y = q.y;
      }
    }
  if (!std::isfinite(best))
    throw nav2_core::NoValidPathCouldBeFound("No bounded authorized terminal connection");
  return result;
}

geometry_msgs::msg::PoseStamped AuthorizedTransitPlanner::dockExitAnchor(
    const geometry_msgs::msg::PoseStamped& start,
    const transit::Geometry& geometry,
    const transit::Geometry& relaxed,
    const std::function<void()>& check) const
{
  const auto p = point(start);
  // Only our own station has a head to keep off: a start in a peer station's margin just takes the
  // nearest way out, and `relaxed` still keeps that station's object blocked.
  const bool in_own_zone = geometry.insideStartExempt(p);
  if (in_own_zone && !geometry.dock)
    throw nav2_core::StartOccupied(
        "Transit start inside the dock body, but the dock orientation is unknown");
  const auto start_local = in_own_zone ? geometry.dock->local(p) : transit::Point{0, 0};
  const bool beyond_head = in_own_zone && start_local.x > geometry.dock->body_front;
  if (in_own_zone)
  {
    const auto& dock = *geometry.dock;
    // The station head sits where the DOCKED robot's nose is: footprint front reach ahead of the
    // dock pose (x = 0). Before taking its exit leg, the transit controller (RotationShim) may
    // pivot in place about base_footprint to any heading, so on the approach axis the whole disc
    // swept by the footprint around the start must stay short of the head by kStationClearance.
    // This is heading-independent on purpose: the heading the shim pivots to is sampled along the
    // path, not along the exit leg. A start BESIDE the body or BEYOND the head (it only got there
    // by driving around the station) has no such line: refusing it would strand the robot, and it
    // leaves sideways or away from the head below.
    double front_reach = 0, sweep_radius = 0;
    for (const auto& vertex : _costmap_ros->getRobotFootprint())
    {
      front_reach = std::max(front_reach, vertex.x);
      sweep_radius = std::max(sweep_radius, std::hypot(vertex.x, vertex.y));
    }
    if (sweep_radius <= 0)
      front_reach = sweep_radius = _costmap_ros->getRobotRadius();
    const bool on_approach_axis = std::abs(start_local.y) <= dock.body_half_width && !beyond_head;
    // Rear to the station (parked backwards on the tray): the exit leg lies straight ahead, nothing
    // to pivot. A robot FACING it stays refused; the BT reverses it out (IsAtDockStation) and
    // docking skips the staging leg, so neither ever needs this plan.
    const double start_yaw = 2 * std::atan2(start.pose.orientation.z, start.pose.orientation.w);
    const bool facing_away =
        std::cos(start_yaw - std::atan2(dock.axis.y, dock.axis.x)) < -std::sqrt(0.5);
    if (!std::isfinite(start_local.x) || !std::isfinite(start_local.y) ||
        (on_approach_axis && !facing_away &&
         start_local.x + sweep_radius > front_reach - kStationClearance))
      throw nav2_core::StartOccupied(
          "Transit start inside the dock body too close to the charging station to pivot out");
  }
  unsigned mx, my;
  if (!private_map_.worldToMap(p.x, p.y, mx, my))
    throw nav2_core::StartOccupied("Transit start inside the dock body, outside the costmap");
  const int radius = static_cast<int>(std::ceil(kDockExitLimit / private_map_.getResolution()));
  double best = INFINITY;
  auto result = start;
  for (int y = std::max(0, static_cast<int>(my) - radius);
       y <= std::min(static_cast<int>(private_map_.getSizeInCellsY()) - 1,
                     static_cast<int>(my) + radius);
       ++y)
  {
    check();
    for (int x = std::max(0, static_cast<int>(mx) - radius);
         x <= std::min(static_cast<int>(private_map_.getSizeInCellsX()) - 1,
                       static_cast<int>(mx) + radius);
         ++x)
    {
      transit::Point q;
      private_map_.mapToWorld(x, y, q.x, q.y);
      const double d = transit::distance(p, q);
      // The exit leg never closes on the station head: from anywhere up to the head, the exit cell
      // lies no further along the dock axis than the start, which leaves backwards (the undock
      // direction) or sideways. From beyond the head it lies no nearer to it.
      if (d >= best || d > kDockExitLimit)
        continue;
      if (in_own_zone)
      {
        const double exit_x = geometry.dock->local(q).x;
        if (beyond_head ? exit_x < start_local.x : exit_x > start_local.x)
          continue;
      }
      // The exit cell must be clear of EVERYTHING, the dock body included. Only the leg from the
      // start to it is judged without the dock body: every other obstacle and the authorized union
      // still apply to it, and so do the obstacle costs.
      if (private_map_.getCost(x, y) < 253 && geometry.authorized(q) && geometry.clear(q) &&
          relaxed.segmentAuthorized(p, q) && relaxed.segmentClear(p, q) &&
          costClearOutsideStartExempt(p, q, geometry))
      {
        best = d;
        result.pose.position.x = q.x;
        result.pose.position.y = q.y;
      }
    }
  }
  if (!std::isfinite(best))
    throw nav2_core::StartOccupied("Transit start inside the dock body without a clear way out");
  RCLCPP_INFO(_logger,
              "Transit starts in a dock zone at (%.2f, %.2f), %.2f m behind our dock pose: "
              "leaving it through (%.2f, %.2f), %.2f m away",
              p.x,
              p.y,
              -start_local.x,
              result.pose.position.x,
              result.pose.position.y,
              best);
  return result;
}

bool AuthorizedTransitPlanner::pathValid(const nav_msgs::msg::Path& path,
                                         const transit::Geometry& geometry) const
{
  if (path.poses.empty())
    return false;
  for (std::size_t i = 0; i < path.poses.size(); ++i)
  {
    const auto a = point(path.poses[i ? i - 1 : i]), b = point(path.poses[i]);
    if (!geometry.segmentAuthorized(a, b) || !geometry.segmentClear(a, b) ||
        !costSegmentClear(a, b))
    {
      RCLCPP_DEBUG(_logger,
                   "Invalid segment %zu (%0.9f,%0.9f)->(%0.9f,%0.9f), auth=%d clear=%d cost=%d",
                   i,
                   a.x,
                   a.y,
                   b.x,
                   b.y,
                   geometry.segmentAuthorized(a, b),
                   geometry.segmentClear(a, b),
                   costSegmentClear(a, b));
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path AuthorizedTransitPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const std::vector<geometry_msgs::msg::PoseStamped>& viapoints,
    std::function<bool()> cancel_checker)
{
  std::lock_guard planning_lock(planning_mutex_);
  const auto begin = std::chrono::steady_clock::now();
  auto phase_begin = begin;
  const auto profile = [&](const char* phase)
  {
    const auto now = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(_logger,
                 "Transit phase %s: %.3f ms (elapsed %.3f ms)",
                 phase,
                 std::chrono::duration<double, std::milli>(now - phase_begin).count(),
                 std::chrono::duration<double, std::milli>(now - begin).count());
    phase_begin = now;
  };
  double budget;
  {
    std::lock_guard lock(_mutex);
    budget = _max_planning_time;
  }
  const auto remaining = [&]
  {
    return budget - std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  };
  const auto check = [&]
  {
    if (cancel_checker())
      throw nav2_core::PlannerCancelled("Transit planning cancelled");
    if (remaining() <= 0)
      throw nav2_core::PlannerTimedOut("Authorized transit exceeded its total planning budget");
  };
  check();
  transit::Geometry geometry;
  uint64_t revision;
  {
    std::lock_guard lock(geometry_mutex_);
    if (!geometry_valid_)
      throw nav2_core::NoValidPathCouldBeFound("Transit geometry unavailable or map edit pending");
    geometry = geometry_;
    revision = geometry_revision_;
  }
  if (!viapoints.empty())
    throw nav2_core::NoValidPathCouldBeFound("Transit viapoints are not supported");
  if (start.header.frame_id != _global_frame || goal.header.frame_id != _global_frame ||
      !std::isfinite(point(start).x) || !std::isfinite(point(start).y) ||
      !std::isfinite(point(goal).x) || !std::isfinite(point(goal).y))
    throw nav2_core::NoValidPathCouldBeFound(
        "Transit endpoints must be finite in the global frame");
  {
    std::unique_lock lock(*shared_map_->getMutex());
    private_map_ = *shared_map_;
  }
  profile("snapshot");
  const auto occupied = [&](const auto& endpoint, const transit::Geometry& view)
  {
    unsigned x, y;
    const auto p = point(endpoint);
    return !view.clear(p) ||
           (private_map_.worldToMap(p.x, p.y, x, y) && private_map_.getCost(x, y) >= 253);
  };
  // A failed docking stops next to the station, in the dock body or its body band. A start there
  // is judged without that ring and without the mask's lethal cells for it, and connected to clear
  // ground by a bounded exit leg that never closes on the station (dockExitAnchor); a GOAL in it
  // stays refused, and the search and every path segment keep the whole ring blocked.
  // A start in a peer station's margin takes the same bounded way out.
  const bool start_in_dock_body =
      geometry.insideStartExempt(point(start)) || geometry.insideMarginStartExempt(point(start));
  const transit::Geometry relaxed =
      start_in_dock_body ? geometry.relaxedForStart(point(start)) : transit::Geometry{};
  if (start_in_dock_body ? !relaxed.clear(point(start)) : occupied(start, geometry))
    throw nav2_core::StartOccupied("Transit start occupied by an obstacle");
  if (occupied(goal, geometry))
    throw nav2_core::GoalOccupied("Transit goal occupied by an obstacle");
  // Terminal exceptions never enter the search grid, so even nearby endpoints
  // cannot turn outside slack into a through-corridor.
  const auto start_anchor = start_in_dock_body ? dockExitAnchor(start, geometry, relaxed, check)
                                               : anchor(start, geometry);
  const auto goal_anchor = anchor(goal, geometry);
  profile("anchors");
  // Everything from here on uses the FULL geometry: the dock body stays forbidden for the
  // connectivity check, the rasterised search grid and every path segment. Only the start
  // judgement above and the bounded exit leg inside dockExitAnchor see it relaxed.
  if (!geometry.boundsMayConnect(point(start_anchor), point(goal_anchor), check))
  {
    profile("static-disconnected");
    throw nav2_core::NoValidPathCouldBeFound("Authorized transit polygon bounds are disconnected");
  }
  profile("connectivity");
  const auto authorization = transit::rasterAuthorization(geometry,
                                                          {private_map_.getSizeInCellsX(),
                                                           private_map_.getSizeInCellsY(),
                                                           private_map_.getResolution(),
                                                           private_map_.getOriginX(),
                                                           private_map_.getOriginY()},
                                                          check);
  for (unsigned y = 0; y < private_map_.getSizeInCellsY(); ++y)
  {
    check();
    for (unsigned x = 0; x < private_map_.getSizeInCellsX(); ++x)
    {
      if (!authorization[std::size_t{y} * private_map_.getSizeInCellsX() + x])
        private_map_.setCost(x, y, 254);
    }
  }
  profile("authorization");
  std::size_t edge_checks = 0;
  // Buffers belong to this synchronous plan; only capacity is reused, never
  // a geometry verdict or obstacle observation from a previous edge/plan.
  std::vector<double> geometry_cuts, cost_cuts;
  TransitNode::edge_allowed = [&](uint64_t a, uint64_t b)
  {
    ++edge_checks;
    check();
    transit::Point p, q;
    const auto width = private_map_.getSizeInCellsX();
    private_map_.mapToWorld(a % width, a / width, p.x, p.y);
    private_map_.mapToWorld(b % width, b / width, q.x, q.y);
    return geometry.segmentAuthorized(p, q, geometry_cuts) &&
           geometry.segmentClear(p, q, geometry_cuts) && costSegmentClear(p, q, cost_cuts);
  };
  struct ResetContext
  {
    ~ResetContext()
    {
      TransitNode::edge_allowed = {};
    }
  } reset_context;
  nav_msgs::msg::Path raw;
  try
  {
    raw = BasePlanner::createPlan(start_anchor,
                                  goal_anchor,
                                  {},
                                  [&]
                                  {
                                    check();
                                    return false;
                                  });
  }
  catch (...)
  {
    profile("search-failed");
    RCLCPP_DEBUG(_logger, "Transit edge checks: %zu", edge_checks);
    throw;
  }
  profile("search");
  RCLCPP_DEBUG(_logger, "Transit edge checks: %zu", edge_checks);
  // Smac's world conversion rounds the origin through float. Restore the
  // exact cell centres used by TransitNode's edge checks before validation;
  // otherwise a shifted diagonal can clip a forbidden corner by micrometres.
  for (std::size_t i = 0; i < raw.poses.size(); ++i)
  {
    auto& pose = raw.poses[i];
    // The last pose is replaced with the exact goal by Smac. All preceding
    // poses use its float origin, whose error can exceed a cell on a translated
    // garden even though the extent is small.
    if (i + 1 < raw.poses.size())
    {
      pose.pose.position.x += private_map_.getOriginX() -
                              static_cast<double>(static_cast<float>(private_map_.getOriginX()));
      pose.pose.position.y += private_map_.getOriginY() -
                              static_cast<double>(static_cast<float>(private_map_.getOriginY()));
    }
    unsigned x, y;
    if (!private_map_.worldToMap(pose.pose.position.x, pose.pose.position.y, x, y))
      throw nav2_core::NoValidPathCouldBeFound("Search path outside the private grid");
    private_map_.mapToWorld(x, y, pose.pose.position.x, pose.pose.position.y);
  }
  if (!pathValid(raw, geometry))
    throw nav2_core::NoValidPathCouldBeFound("Raw search path failed continuous validation");
  if (transit::distance(point(raw.poses.back()), point(goal_anchor)) > transit::kEpsilon)
    throw nav2_core::NoValidPathCouldBeFound("Search did not reach the authorized goal anchor");
  profile("raw-validation");
  auto smoothed = raw;
  check();
  transit_smoother_->smooth(smoothed,
                            &private_map_,
                            remaining(),
                            _costmap_ros->getUseRadius() ? std::vector<geometry_msgs::msg::Point>()
                                                         : _costmap_ros->getRobotFootprint());
  const bool smoothing_valid = pathValid(smoothed, geometry);
  RCLCPP_DEBUG(_logger, "Transit smoothing accepted: %d", smoothing_valid);
  auto result = smoothing_valid ? std::move(smoothed) : std::move(raw);
  profile("smoothing-validation");
  // Validate the bounded terminal legs against original obstacle costs, before
  // outside-center search cells were made lethal in the private view.
  result.poses.insert(result.poses.begin(), start);
  result.poses.push_back(goal);
  {
    std::lock_guard lock(geometry_mutex_);
    if (!geometry_valid_ || revision != geometry_revision_)
      throw nav2_core::NoValidPathCouldBeFound("Transit map changed during planning");
  }
  check();
  return result;
}
}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::AuthorizedTransitPlanner, nav2_core::GlobalPlanner)
