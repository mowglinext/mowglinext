// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/mapped_obstacle_layer.hpp"

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_nav2_plugins/transit_raster.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_nav2_plugins
{
void MappedObstacleLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node)
    throw std::runtime_error("MappedObstacleLayer requires a lifecycle node");
  enabled_ = true;
  current_ = false;
  transform_tolerance_ = node->declare_or_get_parameter("transform_tolerance", 0.3);
  if (!std::isfinite(transform_tolerance_) || transform_tolerance_ < 0)
    throw std::runtime_error("Invalid costmap transform tolerance");
  geometry_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry",
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
      {
        receiveGeometry(std::move(message));
      },
      rclcpp::QoS(1).transient_local().reliable(),
      callback_group_);
  reset();
}

void MappedObstacleLayer::closeMap()
{
  auto* map = layered_costmap_->getCostmap();
  std::fill_n(map->getCharMap(),
              std::size_t{map->getSizeInCellsX()} * map->getSizeInCellsY(),
              nav2_costmap_2d::LETHAL_OBSTACLE);
  current_ = false;
}

void MappedObstacleLayer::reset()
{
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  // Ordinary recovery clears cannot erase persistent map obstacles. The
  // retained snapshot is rebuilt on the next normal layered-map update.
  closeMap();
}

void MappedObstacleLayer::receiveGeometry(
    visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
{
  // Match LayeredCostmap's lock order, and revoke the old map immediately on
  // receipt. A consumer between receipt and the next update must stay closed.
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> grid_lock(*map->getMutex());
  std::lock_guard<std::mutex> geometry_lock(geometry_mutex_);
  closeMap();
  valid_ = false;
  transformed_ = false;
  obstacles_.clear();
  // Reuse the complete snapshot's bounded strict parser: map frame, finite
  // planar coordinates, simple rings, known kinds and a valid allowed union.
  // Its chassis/outer-boundary policy is not copied into the local costmap.
  std::shared_ptr<const mowgli_interfaces::motion::Snapshot> snapshot;
  try
  {
    if (message)
      snapshot = mowgli_interfaces::motion::Snapshot::parse(*message);
  }
  catch (const std::exception&)
  {
    return;  // Invalid topology must revoke the old map even if parsing throws.
  }
  valid_ = snapshot && snapshot->valid;
  if (valid_)
    for (const auto& marker : message->markers)
    {
      if (marker.ns != "obstacle")
        continue;
      transit::Ring ring;
      for (const auto& p : marker.points)
        ring.push_back({p.x, p.y});
      obstacles_.push_back(std::move(ring));
    }
}

void MappedObstacleLayer::updateBounds(
    double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  transformed_ = valid_;
  local_obstacles_.clear();
  if (valid_)
  {
    try
    {
      tf2::Transform transform;
      transform.setIdentity();
      if (layered_costmap_->getGlobalFrameID() != "map")
      {
        const auto node = node_.lock();
        if (!node)
          throw std::runtime_error("Mapped-obstacle lifecycle node unavailable");
        const auto now = node->now();
        const auto latest =
            tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), "map", tf2::TimePointZero);
        const rclcpp::Time stamp(latest.header.stamp, now.get_clock_type());
        // A future-dated localizer forecast is not the robot's current map
        // pose. Select a supported epoch no later than ROS now, as FTC does.
        const auto selected = stamp.nanoseconds() == 0 ? now : std::min(now, stamp);
        const auto supported =
            tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), "map", selected);
        const auto& q = supported.transform.rotation;
        const auto& t = supported.transform.translation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        const double age = (now - selected).seconds();
        if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3 || !std::isfinite(t.x) ||
            !std::isfinite(t.y) || !std::isfinite(t.z) || age < 0 || age > transform_tolerance_)
          throw std::runtime_error("Invalid or stale mapped-obstacle transform");
        tf2::fromMsg(supported.transform, transform);
      }
      for (const auto& ring : obstacles_)
      {
        transit::Ring local;
        for (const auto& p : ring)
        {
          const auto q = transform * tf2::Vector3(p.x, p.y, 0);
          if (!std::isfinite(q.x()) || !std::isfinite(q.y()))
            throw std::runtime_error("Nonfinite mapped-obstacle transform");
          local.push_back({q.x(), q.y()});
        }
        local_obstacles_.push_back(std::move(local));
      }
    }
    catch (const std::exception&)
    {
      transformed_ = false;
    }
  }
  // Full bounds are intentional: the master resets these cells before all
  // layers rebuild. Removal, rolling-window shifts and map->odom changes then
  // leave no old polygon cells, while current sensor costs are merged normally.
  const auto* map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
}

void MappedObstacleLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& map, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  if (!transformed_)
  {
    closeMap();
    return;
  }
  const transit::RasterGrid grid{map.getSizeInCellsX(),
                                 map.getSizeInCellsY(),
                                 map.getResolution(),
                                 map.getOriginX(),
                                 map.getOriginY()};
  const auto mark = [&](unsigned x, unsigned y)
  {
    if (int(x) >= min_i && int(x) < max_i && int(y) >= min_j && int(y) < max_j)
      map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  };
  const auto check = [] {};
  for (const auto& ring : local_obstacles_)
  {
    transit::rasterInside(ring, grid, check, mark);
    // Cover every cell touched by a raw edge, including sub-cell holes. This
    // is grid quantization only, never the planner's chassis-width margin.
    transit::rasterEdges(ring, grid, grid.resolution * std::sqrt(0.5), false, check, mark);
  }
  current_ = true;
}
}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::MappedObstacleLayer, nav2_costmap_2d::Layer)
