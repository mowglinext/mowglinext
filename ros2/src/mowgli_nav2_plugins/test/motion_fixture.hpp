// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <chrono>
#include <thread>
#include <vector>

#include "mowgli_nav2_plugins/ftc_controller.hpp"

namespace mowgli_nav2_plugins::test
{
// Existing controller tests exercise PID/recovery behavior in an ordinary
// recorded lawn. Supply that precondition through the production DDS topic.
template <typename Ready>
bool supplyLawn(const nav2::LifecycleNode::SharedPtr& node, Ready ready)
{
  auto fixture = std::make_shared<rclcpp::Node>("recorded_lawn_fixture");
  auto publisher = fixture->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local());
  visualization_msgs::msg::MarkerArray message;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "area";
  marker.text = "mowing";
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{-5, -5}, {15, -5}, {15, 15}, {-5, 15}})
  {
    geometry_msgs::msg::Point point;
    point.x = xy.first;
    point.y = xy.second;
    marker.points.push_back(point);
  }
  message.markers.push_back(marker);
  publisher->publish(message);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(fixture);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    if (ready())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}
}  // namespace mowgli_nav2_plugins::test
