// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>
#include <mutex>

#include "mowgli_nav2_plugins/mowing_geometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_behavior
{
// Owned by the BT blackboard, rather than action state: map replacements must
// also revoke admission while waiting for blade readiness or reacquisition.
class CoverageEntryGeometry
{
public:
  static std::shared_ptr<CoverageEntryGeometry> create(const rclcpp::Node::SharedPtr& node)
  {
    auto cache = std::make_shared<CoverageEntryGeometry>();
    cache->subscription_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/map_server_node/transit_geometry",
        rclcpp::QoS(1).reliable().transient_local(),
        [weak = std::weak_ptr<CoverageEntryGeometry>(cache)](
            const visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
        {
          if (auto current = weak.lock())
          {
            auto next = std::make_shared<const mowgli_nav2_plugins::MowingGeometry>(
                mowgli_nav2_plugins::MowingGeometry::parse(*message));
            std::lock_guard<std::mutex> lock(current->mutex_);
            // Empty/malformed publications replace a prior valid snapshot too.
            current->snapshot_ = std::move(next);
          }
        });
    return cache;
  }

  bool contains(double x, double y) const
  {
    std::shared_ptr<const mowgli_nav2_plugins::MowingGeometry> snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = snapshot_;
    }
    return snapshot && snapshot->contains({x, y});
  }

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const mowgli_nav2_plugins::MowingGeometry> snapshot_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
};
}  // namespace mowgli_behavior
