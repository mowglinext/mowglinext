// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/authorized_transit_planner.hpp"

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
}  // namespace

void AuthorizedTransitPlanner::configure(const nav2::LifecycleNode::WeakPtr& parent,
                                         std::string name,
                                         nav2::TransformBuffer::SharedPtr tf,
                                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (node->declare_or_get_parameter(name + ".tolerance", 0.0) != 0.0)
    throw nav2_core::PlannerException("Authorized transit requires exact goal anchors");
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
    if (marker.ns == "area" || marker.ns == "dock_corridor")
      next.allowed.push_back(std::move(ring));
    else if (marker.ns == "obstacle")
    {
      valid &= std::isfinite(marker.scale.x) && marker.scale.x >= 0;
      next.blocked_margins.push_back(marker.scale.x);
      next.blocked.push_back(std::move(ring));
    }
    else
      valid = false;
  }
  std::lock_guard lock(geometry_mutex_);
  geometry_ = std::move(next);
  geometry_valid_ = valid && !geometry_.allowed.empty();
  ++geometry_revision_;
}

bool AuthorizedTransitPlanner::costSegmentClear(transit::Point a, transit::Point b) const
{
  // Split at grid lines: every crossed cell is checked, even a very short
  // segment clipping a lethal corner. The copy retains shared obstacle costs.
  std::vector<double> ts{0, 1};
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
  const auto occupied = [&](const auto& endpoint)
  {
    unsigned x, y;
    const auto p = point(endpoint);
    return !geometry.clear(p) ||
           (private_map_.worldToMap(p.x, p.y, x, y) && private_map_.getCost(x, y) >= 253);
  };
  if (occupied(start))
    throw nav2_core::StartOccupied("Transit start occupied by an obstacle");
  if (occupied(goal))
    throw nav2_core::GoalOccupied("Transit goal occupied by an obstacle");
  // Terminal exceptions never enter the search grid, so even nearby endpoints
  // cannot turn outside slack into a through-corridor.
  const auto start_anchor = anchor(start, geometry), goal_anchor = anchor(goal, geometry);
  for (unsigned y = 0; y < private_map_.getSizeInCellsY(); ++y)
  {
    check();
    for (unsigned x = 0; x < private_map_.getSizeInCellsX(); ++x)
    {
      transit::Point p;
      private_map_.mapToWorld(x, y, p.x, p.y);
      if (!geometry.authorized(p) || !geometry.clear(p))
        private_map_.setCost(x, y, 254);
    }
  }
  TransitNode::edge_allowed = [&](uint64_t a, uint64_t b)
  {
    check();
    transit::Point p, q;
    const auto width = private_map_.getSizeInCellsX();
    private_map_.mapToWorld(a % width, a / width, p.x, p.y);
    private_map_.mapToWorld(b % width, b / width, q.x, q.y);
    return geometry.segmentAuthorized(p, q) && geometry.segmentClear(p, q) &&
           costSegmentClear(p, q);
  };
  struct ResetContext
  {
    ~ResetContext()
    {
      TransitNode::edge_allowed = {};
    }
  } reset_context;
  auto raw = BasePlanner::createPlan(start_anchor,
                                     goal_anchor,
                                     {},
                                     [&]
                                     {
                                       check();
                                       return false;
                                     });
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
  auto smoothed = raw;
  check();
  transit_smoother_->smooth(smoothed,
                            &private_map_,
                            remaining(),
                            _costmap_ros->getUseRadius() ? std::vector<geometry_msgs::msg::Point>()
                                                         : _costmap_ros->getRobotFootprint());
  auto result = pathValid(smoothed, geometry) ? std::move(smoothed) : std::move(raw);
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
