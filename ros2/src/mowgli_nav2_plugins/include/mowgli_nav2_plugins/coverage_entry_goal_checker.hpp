// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <mutex>

#include "mowgli_nav2_plugins/ftc_corner_geometry.hpp"
#include "nav2_core/goal_checker.hpp"
namespace mowgli_nav2_plugins
{
class CoverageEntryGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const nav2::LifecycleNode::WeakPtr& parent,
                  const std::string& name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void reset() override
  {
  }
  bool isGoalReached(const geometry_msgs::msg::Pose& query,
                     const geometry_msgs::msg::Pose& goal,
                     const geometry_msgs::msg::Twist& velocity,
                     const nav_msgs::msg::Path& path) override;
  bool isGoalXYReached(const geometry_msgs::msg::Pose& query,
                       const geometry_msgs::msg::Pose& goal,
                       const geometry_msgs::msg::Twist& velocity,
                       const nav_msgs::msg::Path& path) override;
  bool getTolerances(geometry_msgs::msg::Pose& pose,
                     geometry_msgs::msg::Twist& velocity,
                     double& path_length) override;

private:
  double entryTolerance() const;
  bool arrived(const geometry_msgs::msg::Pose& query,
               const geometry_msgs::msg::Pose& goal,
               double coast_radius);
  nav2::LifecycleNode::WeakPtr node_;
  std::string frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::mutex mutex_;
  CornerGeometry geometry_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
};
}  // namespace mowgli_nav2_plugins
