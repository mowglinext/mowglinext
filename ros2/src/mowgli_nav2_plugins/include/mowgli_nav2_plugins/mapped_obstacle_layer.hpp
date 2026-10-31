// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <mutex>

#include "mowgli_nav2_plugins/transit_geometry.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mowgli_nav2_plugins
{
// Raw map obstacles for local footprint collision checking. Outer boundaries,
// planner body margins and dock exclusions do not belong in this layer.
class MappedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double*, double*, double*, double*) override;
  void updateCosts(nav2_costmap_2d::Costmap2D&, int, int, int, int) override;
  void reset() override;
  bool isClearable() override
  {
    return false;
  }
  void receiveGeometry(visualization_msgs::msg::MarkerArray::ConstSharedPtr);

private:
  void closeMap();
  std::mutex geometry_mutex_;
  std::vector<transit::Ring> obstacles_, local_obstacles_;
  bool valid_{false}, transformed_{false};
  double transform_tolerance_{0.3};
  nav2::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr geometry_sub_;
};
}  // namespace mowgli_nav2_plugins
