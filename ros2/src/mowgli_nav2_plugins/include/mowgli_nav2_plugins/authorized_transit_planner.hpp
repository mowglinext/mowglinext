// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <functional>
#include <mutex>

#include <nav2_smac_planner/smac_planner_2d.hpp>

#include "mowgli_nav2_plugins/transit_geometry.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace mowgli_nav2_plugins
{
// Retain the pinned Smac Node2D costs and search, adding continuous edge
// authorization. Context is scoped to the synchronous search on this thread.
class TransitNode : public nav2_smac_planner::Node2D
{
public:
  using Node2D::Node2D;
  using NodePtr = TransitNode*;
  using NodeVector = std::vector<NodePtr>;
  using Graph = std::unique_ptr<std::vector<TransitNode>>;
  static inline thread_local std::function<bool(uint64_t, uint64_t)> edge_allowed;

  bool backtracePath(CoordinateVector& path)
  {
    if (!parent)
      return false;
    Node2D* current = this;
    while (current)
    {
      auto coordinate = current->getCoords(current->getIndex());
      // Upstream Node2D backtraces integer cell corners. Transit edges are
      // authorized between cell centres, so keep that same geometry in output.
      coordinate.x += 0.5F;
      coordinate.y += 0.5F;
      path.push_back(coordinate);
      current = current->parent;
    }
    return true;
  }

  void getNeighbors(std::function<bool(const uint64_t&, TransitNode*&)>& checker,
                    nav2_smac_planner::GridCollisionChecker* collision_checker,
                    const bool& unknown,
                    NodeVector& neighbors)
  {
    std::function<bool(const uint64_t&, Node2D*&)> bridge =
        [&](const uint64_t& index, Node2D*& node)
    {
      TransitNode* derived = nullptr;
      const bool valid = checker(index, derived);
      node = derived;
      return valid;
    };
    Node2D::NodeVector candidates;
    Node2D::getNeighbors(bridge, collision_checker, unknown, candidates);
    for (auto* candidate : candidates)
      if (edge_allowed && edge_allowed(getIndex(), candidate->getIndex()))
        neighbors.push_back(static_cast<TransitNode*>(candidate));
  }
};

class AuthorizedTransitPlanner : public nav2_smac_planner::SmacPlanner2DT<TransitNode>
{
public:
  void configure(const nav2::LifecycleNode::WeakPtr& parent,
                 std::string name,
                 nav2::TransformBuffer::SharedPtr tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal,
                                 const std::vector<geometry_msgs::msg::PoseStamped>& viapoints,
                                 std::function<bool()> cancel_checker) override;

private:
  friend class AuthorizedTransitPlannerTest;
  void onGeometry(visualization_msgs::msg::MarkerArray::ConstSharedPtr message);
  bool costSegmentClear(transit::Point a, transit::Point b) const;
  geometry_msgs::msg::PoseStamped anchor(const geometry_msgs::msg::PoseStamped& endpoint,
                                         const transit::Geometry& geometry) const;
  bool pathValid(const nav_msgs::msg::Path& path, const transit::Geometry& geometry) const;
  nav2_costmap_2d::Costmap2D private_map_;
  nav2_costmap_2d::Costmap2D* shared_map_{nullptr};
  std::unique_ptr<nav2_smac_planner::Smoother> transit_smoother_;
  std::mutex geometry_mutex_, planning_mutex_;
  transit::Geometry geometry_;
  bool geometry_valid_{false};
  uint64_t geometry_revision_{0};
  nav2::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr geometry_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr config_guard_;
};
}  // namespace mowgli_nav2_plugins
