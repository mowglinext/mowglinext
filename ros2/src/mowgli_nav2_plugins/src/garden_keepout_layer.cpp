// Copyright (C) 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/garden_keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mowgli_nav2_plugins
{
void GardenKeepoutLayer::onInitialize()
{
  const auto node = node_.lock();
  if (!node || layered_costmap_->isRolling())
    throw std::runtime_error("GardenKeepoutLayer requires a non-rolling costmap");
  const auto topic =
      node->declare_or_get_parameter(name_ + ".mask_topic", std::string("/keepout_mask"));
  max_cells_ = node->declare_or_get_parameter(name_ + ".max_cells", int64_t{4000000});
  resolution_ = node->declare_or_get_parameter("resolution", 0.08);
  if (max_cells_ <= 0 || !std::isfinite(resolution_) || resolution_ <= 0.0)
    throw std::runtime_error("Invalid garden costmap resolution or cell budget");
  enabled_ = true;  // Authorization is mandatory, never dynamically disabled.
  current_ = false;
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask)
      {
        receiveMask(std::move(mask));
      },
      rclcpp::QoS(1).transient_local().reliable(),
      callback_group_);
  // Preserve the standard KeepoutFilter recovery service. Disabling boundary
  // costs never disables geometry/readiness: unavailable masks remain closed.
  toggle_service_ = node->rclcpp_lifecycle::LifecycleNode::create_service<std_srvs::srv::SetBool>(
      (std::string(node->get_namespace()) == "/" ? "" : std::string(node->get_namespace())) + "/" +
          name_ + "/toggle_filter",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res)
      {
        res->success = setFilterEnabled(req->data);
        res->message =
            res->success ? "Keepout recovery toggle applied" : "Garden geometry unavailable";
      });
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  std::fill_n(map->getCharMap(),
              map->getSizeInCellsX() * map->getSizeInCellsY(),
              nav2_costmap_2d::LETHAL_OBSTACLE);
}

void GardenKeepoutLayer::receiveMask(nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask)
{
  // Use the same lock order as LayeredCostmap::updateMap. Closing immediately
  // also prevents a planner locking the map between receipt and the next tick
  // from seeing the previously authorized garden during a replace/edit.
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> grid_lock(*map->getMutex());
  std::lock_guard<std::mutex> mask_lock(mask_mutex_);
  pending_mask_ = std::move(mask);
  filter_enabled_ = true;  // Map edits terminate any previous recovery exemption.
  std::fill_n(map->getCharMap(),
              map->getSizeInCellsX() * map->getSizeInCellsY(),
              nav2_costmap_2d::LETHAL_OBSTACLE);
  current_ = false;
}

bool GardenKeepoutLayer::setFilterEnabled(bool enabled)
{
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> grid_lock(*map->getMutex());
  std::lock_guard<std::mutex> mask_lock(mask_mutex_);
  if (!enabled && (!active_mask_ || !current_))
    return false;
  filter_enabled_ = enabled;
  if (enabled)
  {
    std::fill_n(map->getCharMap(),
                map->getSizeInCellsX() * map->getSizeInCellsY(),
                nav2_costmap_2d::LETHAL_OBSTACLE);
    current_ = false;
  }
  return true;
}

void GardenKeepoutLayer::reset()
{
  // A recovery clear must not remove map authorization. Reapply the retained
  // snapshot on the next update; never permit a free intermediate grid.
  auto* map = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  std::fill_n(map->getCharMap(),
              map->getSizeInCellsX() * map->getSizeInCellsY(),
              nav2_costmap_2d::LETHAL_OBSTACLE);
  current_ = false;
}

void GardenKeepoutLayer::updateBounds(
    double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<std::mutex> lock(mask_mutex_);
  geometry_changed_ = false;
  if (pending_mask_)
  {
    active_mask_ = std::move(pending_mask_);
    const auto& m = *active_mask_;
    const auto& p = m.info.origin;
    const uint64_t count = uint64_t{m.info.width} * m.info.height;
    const double width = std::ceil(m.info.width * double{m.info.resolution} / resolution_);
    const double height = std::ceil(m.info.height * double{m.info.resolution} / resolution_);
    const bool valid = m.header.frame_id == layered_costmap_->getGlobalFrameID() && count > 0 &&
                       count == m.data.size() && std::isfinite(m.info.resolution) &&
                       m.info.resolution > 0 && std::isfinite(p.position.x) &&
                       std::isfinite(p.position.y) && p.orientation.x == 0 &&
                       p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 1 &&
                       width > 0 && height > 0 && width <= std::numeric_limits<unsigned>::max() &&
                       height <= std::numeric_limits<unsigned>::max() && width <= max_cells_ &&
                       height <= max_cells_ && width * height <= max_cells_ &&
                       std::all_of(m.data.begin(),
                                   m.data.end(),
                                   [](int8_t c)
                                   {
                                     return c >= 0 && c <= 100;
                                   });
    if (!valid)
    {
      if (count != 0)
        RCLCPP_ERROR(logger_,
                     "Garden mask invalid or exceeds %ld-cell resource budget; transit disabled",
                     static_cast<long>(max_cells_));
      active_mask_.reset();
    }
    else
    {
      auto* map = layered_costmap_->getCostmap();
      if (map->getSizeInCellsX() != width || map->getSizeInCellsY() != height ||
          map->getOriginX() != p.position.x || map->getOriginY() != p.position.y ||
          map->getResolution() != resolution_)
      {
        layered_costmap_->resizeMap(static_cast<unsigned>(width),
                                    static_cast<unsigned>(height),
                                    resolution_,
                                    p.position.x,
                                    p.position.y,
                                    true);
        geometry_changed_ = true;
      }
    }
  }
  const auto* map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInCellsX() * resolution_);
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInCellsY() * resolution_);
}

void GardenKeepoutLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& map, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mask_mutex_);
  const bool ready = active_mask_ && !geometry_changed_;
  for (int y = min_j; y < max_j; ++y)
  {
    for (int x = min_i; x < max_i; ++x)
    {
      unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      if (ready && !filter_enabled_)
      {
        cost = nav2_costmap_2d::FREE_SPACE;
      }
      else if (ready)
      {
        const auto& m = *active_mask_;
        const double mx = (x + 0.5) * resolution_ / m.info.resolution;
        const double my = (y + 0.5) * resolution_ / m.info.resolution;
        if (mx < m.info.width && my < m.info.height)
        {
          const int value =
              m.data[static_cast<size_t>(my) * m.info.width + static_cast<size_t>(mx)];
          cost = static_cast<unsigned char>(std::lround(value * 254.0 / 100.0));
        }
      }
      const auto old_cost = map.getCost(x, y);
      map.setCost(x,
                  y,
                  old_cost == nav2_costmap_2d::NO_INFORMATION ? cost : std::max(old_cost, cost));
    }
  }
  current_ = ready;
}

void RecentObstacleLayer::updateBounds(double robot_x,
                                       double robot_y,
                                       double robot_yaw,
                                       double* min_x,
                                       double* min_y,
                                       double* max_x,
                                       double* max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  resetMaps();
  nav2_costmap_2d::ObstacleLayer::updateBounds(
      robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  // Resetting the layer also removes marks outside this tick's sensor rays;
  // include the full old extent so the master's stale cells/inflation clear.
  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInCellsX() * getResolution());
  *max_y = std::max(*max_y, getOriginY() + getSizeInCellsY() * getResolution());
}
}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::GardenKeepoutLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::RecentObstacleLayer, nav2_costmap_2d::Layer)
