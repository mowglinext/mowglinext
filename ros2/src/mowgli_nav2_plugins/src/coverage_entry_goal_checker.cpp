// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/coverage_entry_goal_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "pluginlib/class_list_macros.hpp"
namespace mowgli_nav2_plugins
{
void CoverageEntryGoalChecker::initialize(
    const nav2::LifecycleNode::WeakPtr& parent,
    const std::string&,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  node_ = parent;
  auto node = parent.lock();
  frame_ = costmap->getGlobalFrameID();
  tf_ = costmap->getTfBuffer();
  subscription_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry",
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
      {
        auto next = CornerGeometry::parse(*message);
        std::lock_guard lock(mutex_);
        geometry_ = std::move(next);
      },
      rclcpp::QoS(1).transient_local());
}

bool CoverageEntryGoalChecker::arrived(const geometry_msgs::msg::Pose& query,
                                       const geometry_msgs::msg::Pose& goal,
                                       double radius)
{
  geometry_msgs::msg::PoseStamped current, target;
  current.header.frame_id = target.header.frame_id = frame_;
  current.pose = query;
  target.pose = goal;
  try
  {
    if (frame_ != "map")
    {
      const auto transform = tf_->lookupTransform("map", frame_, tf2::TimePointZero);
      tf2::doTransform(current, current, transform);
      tf2::doTransform(target, target, transform);
    }
  }
  catch (const tf2::TransformException&)
  {
    return false;
  }
  const transit::Point p{current.pose.position.x, current.pose.position.y};
  const transit::Point g{target.pose.position.x, target.pose.position.y};
  if (!std::isfinite(radius) || radius < 0 || transit::distance(p, g) > 0.30)
    return false;
  const double tolerance = entryTolerance();
  std::lock_guard lock(mutex_);
  if (!geometry_.valid || !geometry_.mowing.clear(p) || !geometry_.mowing.clear(g))
    return false;
  // Only a MOW ring containing the unchanged goal grants arrival; a nearby
  // disconnected MOW island or the navigation union cannot end transit.
  // "Containing" allows coverage_entry_tolerance_m outside the recorded line —
  // the same allowance FollowStrip's dispatch check uses, so a transit that
  // ends here is one FollowStrip accepts (no transit/dispatch ping-pong). With
  // tolerance 0 this is the exact strict rule: the disk must clear every edge.
  const auto signed_distance = [](transit::Point q, const transit::Ring& ring)
  {
    double edge = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      edge = std::min(edge, transit::edgeDistance(q, ring[j], ring[i]));
    return transit::inside(q, ring) ? -edge : edge;
  };
  bool inside = false;
  for (const auto& ring : geometry_.mowing.allowed)
  {
    if (ring.size() < 3)
      continue;
    const bool goal_in = signed_distance(g, ring) + transit::kEpsilon < tolerance ||
                         (tolerance <= 0 && transit::inside(g, ring));
    const bool disk_in = signed_distance(p, ring) + radius + transit::kEpsilon < tolerance ||
                         (tolerance <= 0 && radius <= 0 && transit::inside(p, ring));
    inside = inside || (goal_in && disk_in);
  }
  if (!inside)
    return false;
  for (const auto& hole : geometry_.mowing.blocked)
    if (transit::ringDistance(p, hole) <= radius + transit::kEpsilon)
      return false;
  return true;
}

double CoverageEntryGoalChecker::entryTolerance() const
{
  // Injected on controller_server as FollowCoveragePath.coverage_entry_tolerance_m
  // (robot_config_util.coverage_entry_tolerance), shared with FTC's gates.
  // Absent or invalid keeps the exact recorded line.
  const auto node = node_.lock();
  if (!node)
    return 0.0;
  rclcpp::Parameter parameter;
  if (!node->get_parameter("FollowCoveragePath.coverage_entry_tolerance_m", parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    return 0.0;
  const double tolerance = parameter.as_double();
  return std::isfinite(tolerance) && tolerance > 0.0 ? std::min(tolerance, 0.5) : 0.0;
}

bool CoverageEntryGoalChecker::isGoalXYReached(const geometry_msgs::msg::Pose& query,
                                               const geometry_msgs::msg::Pose& goal,
                                               const geometry_msgs::msg::Twist&,
                                               const nav_msgs::msg::Path&)
{
  // Both RotationShim and RPP use this pose-aware hook before stopping translation.
  return arrived(query, goal, 0);
}
bool CoverageEntryGoalChecker::isGoalReached(const geometry_msgs::msg::Pose& query,
                                             const geometry_msgs::msg::Pose& goal,
                                             const geometry_msgs::msg::Twist& velocity,
                                             const nav_msgs::msg::Path&)
{
  const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
  // Same translation-settled threshold and declared coast horizon as FTC's
  // PRE_ROTATE contract. Residual translation can sweep in any direction.
  double coast = 0;
  const auto node = node_.lock();
  if (!node)
    return false;
  try
  {
    rclcpp::Parameter parameter;
    if (!node->get_parameter("FollowCoveragePath.pivot_coast_time_bound_s", parameter) ||
        parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      return false;
    coast = parameter.as_double();
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
  {
    return false;
  }
  if (!std::isfinite(coast) || coast < 0.01 || coast > 2.0)
    return false;
  return std::isfinite(speed) && speed <= 0.01 && arrived(query, goal, speed * coast);
}
bool CoverageEntryGoalChecker::getTolerances(geometry_msgs::msg::Pose& pose,
                                             geometry_msgs::msg::Twist& velocity,
                                             double& path_length)
{
  pose = geometry_msgs::msg::Pose{};
  velocity = geometry_msgs::msg::Twist{};
  pose.position.x = pose.position.y = path_length = 0.30;
  pose.orientation.z = std::sin(3.14 / 2);
  pose.orientation.w = std::cos(3.14 / 2);
  velocity.linear.x = velocity.linear.y = 0.01;
  return true;
}
}  // namespace mowgli_nav2_plugins
PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::CoverageEntryGoalChecker, nav2_core::GoalChecker)
