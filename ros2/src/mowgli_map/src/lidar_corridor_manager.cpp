// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// CRUD for LidarIgnoreCorridorEntry — see its doc comment
// (map_server_node.hpp) for what the concept is and why it is kept
// deliberately separate from AreaEntry/area_manager.cpp: a corridor is not
// classified, is not part of any keepout mask, and its only consumer is
// costmap_scan_filter_node over lidar_ignore_corridors_pub_.

#include <algorithm>
#include <mutex>
#include <utility>

#include "mowgli_map/map_server_node.hpp"

namespace mowgli_map
{

void MapServerNode::on_add_lidar_ignore_corridor(
    const mowgli_interfaces::srv::AddLidarIgnoreCorridor::Request::SharedPtr req,
    mowgli_interfaces::srv::AddLidarIgnoreCorridor::Response::SharedPtr res)
{
  const auto& polyline = req->corridor.polyline;
  if (polyline.points.size() < 2)
  {
    res->success = false;
    res->id = 0;
    RCLCPP_WARN(get_logger(), "add_lidar_ignore_corridor: polyline needs at least 2 points.");
    return;
  }

  LidarIgnoreCorridorEntry entry;
  entry.name = req->corridor.name;
  entry.polyline = polyline;
  // SAFETY: clamp here, not in the filter — a value outside this bound never
  // reaches costmap_scan_filter_node, so a stale/buggy client can never widen
  // it further downstream. See kMin/kMaxLidarIgnoreCorridorWidthM's comment.
  entry.width_m = std::clamp(req->corridor.width_m,
                             kMinLidarIgnoreCorridorWidthM,
                             kMaxLidarIgnoreCorridorWidthM);
  if (entry.width_m != req->corridor.width_m)
  {
    RCLCPP_WARN(get_logger(),
                "add_lidar_ignore_corridor('%s'): width_m %.3f clamped to %.3f (allowed [%.2f, "
                "%.2f] m).",
                entry.name.c_str(),
                req->corridor.width_m,
                entry.width_m,
                kMinLidarIgnoreCorridorWidthM,
                kMaxLidarIgnoreCorridorWidthM);
  }

  // Same round-trip-id contract as on_add_area (mowglinext#637): the GUI's
  // edit/delete flow rebuilds the whole list (clear + re-add per surviving
  // entry), so an untouched corridor's existing id must round-trip to keep
  // its identity; a genuinely new one (id == 0) gets a freshly minted one.
  entry.id = (req->corridor.id != 0) ? req->corridor.id : next_lidar_corridor_id_++;
  if (req->corridor.id != 0 && std::any_of(lidar_ignore_corridors_.begin(),
                                           lidar_ignore_corridors_.end(),
                                           [&entry](const LidarIgnoreCorridorEntry& existing)
                                           {
                                             return existing.id == entry.id;
                                           }))
  {
    RCLCPP_WARN(get_logger(),
                "add_lidar_ignore_corridor('%s'): id %u already taken — minting %u instead.",
                entry.name.c_str(),
                entry.id,
                next_lidar_corridor_id_);
    entry.id = next_lidar_corridor_id_++;
  }
  next_lidar_corridor_id_ = std::max(next_lidar_corridor_id_, entry.id + 1);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    lidar_ignore_corridors_.push_back(std::move(entry));
  }

  RCLCPP_INFO(get_logger(),
              "Added LiDAR-ignore corridor '%s' (%zu points, width %.2f m).",
              req->corridor.name.c_str(),
              polyline.points.size(),
              req->corridor.width_m);

  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "Auto-save after corridor add failed: %s", ex.what());
    }
  }

  res->success = true;
  res->id = lidar_ignore_corridors_.back().id;
  publish_lidar_ignore_corridors();
}

void MapServerNode::on_get_lidar_ignore_corridors(
    const mowgli_interfaces::srv::GetLidarIgnoreCorridors::Request::SharedPtr /*req*/,
    mowgli_interfaces::srv::GetLidarIgnoreCorridors::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  res->corridors.reserve(lidar_ignore_corridors_.size());
  for (const auto& entry : lidar_ignore_corridors_)
  {
    mowgli_interfaces::msg::LidarIgnoreCorridor corridor;
    corridor.name = entry.name;
    corridor.polyline = entry.polyline;
    corridor.width_m = entry.width_m;
    corridor.id = entry.id;
    res->corridors.push_back(std::move(corridor));
  }
  res->success = true;
}

void MapServerNode::on_clear_lidar_ignore_corridors(
    const mowgli_interfaces::srv::ClearLidarIgnoreCorridors::Request::SharedPtr /*req*/,
    mowgli_interfaces::srv::ClearLidarIgnoreCorridors::Response::SharedPtr res)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    lidar_ignore_corridors_.clear();
    // next_lidar_corridor_id_ is NOT reset — same "never reuse an id" rule
    // as next_area_id_ across ~/clear_map.
  }

  if (!areas_file_path_.empty())
  {
    try
    {
      save_areas_to_file(areas_file_path_);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN(get_logger(), "Auto-save after clearing corridors failed: %s", ex.what());
    }
  }

  res->success = true;
  publish_lidar_ignore_corridors();
}

void MapServerNode::publish_lidar_ignore_corridors()
{
  mowgli_interfaces::msg::LidarIgnoreCorridorArray msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    msg.corridors.reserve(lidar_ignore_corridors_.size());
    for (const auto& entry : lidar_ignore_corridors_)
    {
      mowgli_interfaces::msg::LidarIgnoreCorridor corridor;
      corridor.name = entry.name;
      corridor.polyline = entry.polyline;
      corridor.width_m = entry.width_m;
      corridor.id = entry.id;
      msg.corridors.push_back(std::move(corridor));
    }
  }
  lidar_ignore_corridors_pub_->publish(msg);
}

}  // namespace mowgli_map
