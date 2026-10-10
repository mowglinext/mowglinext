// Copyright (C) 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <mutex>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mowgli_nav2_plugins
{
// Geometry and authorization are taken from ONE mask message. The last layer
// applies the mask after inflation; resizing costs one closed update cycle so
// the preceding sensor layers can rebuild at the new geometry before planning.
class GardenKeepoutLayer : public nav2_costmap_2d::Layer
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
  void receiveMask(nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask);
  bool setFilterEnabled(bool enabled);

private:
  std::mutex mask_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_mask_, active_mask_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;
  double resolution_{0.08};
  int64_t max_cells_{4000000};
  bool geometry_changed_{false};
  bool filter_enabled_{true};
};

// A rolling window used to discard remote sensor marks implicitly. On a
// garden-sized fixed grid, rebuild from the bounded observation buffers each
// cycle instead: absent obstacles disappear without retaining ghost walls.
// Map-drawn obstacles remain in the independent keepout mask.
class RecentObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(double, double, double, double*, double*, double*, double*) override;
};
}  // namespace mowgli_nav2_plugins
