// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Coverage statistics + service handlers (GetCoverageStatus, PaintSwath,
// ClearDeadCells). Strip / segment / headland planners are gone — that work
// has moved to opennav_coverage, which calls back into PaintSwath after each
// FollowCoveragePath run to keep mow_progress in sync. The cell-tracking
// invariants (mow_progress decay, classification, fail_count promotion to
// LAWN_DEAD, decay back to LAWN) all still live in progress_tracker.cpp and
// area_manager.cpp.

#include <cmath>
#include <cstdint>
#include <string>

#include <std_srvs/srv/trigger.hpp>

#include "mowgli_map/map_server_node.hpp"

namespace mowgli_map
{

void MapServerNode::compute_coverage_stats(size_t area_index,
                                           uint32_t& total,
                                           uint32_t& mowed,
                                           uint32_t& obstacle_cells) const
{
  total = 0;
  mowed = 0;
  obstacle_cells = 0;

  if (area_index >= areas_.size())
    return;

  const auto& area = areas_[area_index];
  const auto& progress_layer = map_[std::string(layers::MOW_PROGRESS)];
  const auto& class_layer = map_[std::string(layers::CLASSIFICATION)];

  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    grid_map::Position pos;
    map_.getPosition(*it, pos);

    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(pos.x());
    pt.y = static_cast<float>(pos.y());

    if (!point_in_polygon(pt, area.polygon))
      continue;

    auto cell_type = static_cast<CellType>(static_cast<int>(class_layer((*it)(0), (*it)(1))));
    if (cell_type == CellType::OBSTACLE_PERMANENT || cell_type == CellType::OBSTACLE_TEMPORARY)
    {
      obstacle_cells++;
      continue;
    }

    total++;
    if (progress_layer((*it)(0), (*it)(1)) >= 0.3f)
      mowed++;
  }
}

void MapServerNode::on_get_coverage_status(
    const mowgli_interfaces::srv::GetCoverageStatus::Request::SharedPtr req,
    mowgli_interfaces::srv::GetCoverageStatus::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (req->area_index >= areas_.size())
  {
    res->success = false;
    return;
  }

  if (areas_[req->area_index].is_navigation_area)
  {
    res->success = true;
    res->total_cells = 0;
    res->mowed_cells = 0;
    res->obstacle_cells = 0;
    res->coverage_percent = 100.0f;
    res->strips_remaining = 0;
    return;
  }

  compute_coverage_stats(req->area_index, res->total_cells, res->mowed_cells, res->obstacle_cells);
  res->coverage_percent =
      res->total_cells > 0 ? 100.0f * res->mowed_cells / res->total_cells : 0.0f;
  // Coverage-threshold shim for GetNextUnmowedArea: 1 = work remaining,
  // 0 = area complete. opennav_coverage handles the actual swath count.
  res->strips_remaining = (res->coverage_percent < 99.0f) ? 1u : 0u;
  res->success = true;
}

void MapServerNode::on_paint_swath(
    const mowgli_interfaces::srv::PaintSwath::Request::SharedPtr req,
    mowgli_interfaces::srv::PaintSwath::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  uint32_t painted = 0;
  for (const auto& pose : req->swath_path.poses)
  {
    mark_cells_mowed(pose.pose.position.x, pose.pose.position.y);
    ++painted;
  }

  res->success = true;
  res->cells_painted = painted;

  RCLCPP_DEBUG(get_logger(), "PaintSwath: painted %u poses", painted);
}

void MapServerNode::on_clear_dead_cells(const std_srvs::srv::Trigger::Request::SharedPtr,
                                        std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  auto& fc = map_[std::string(layers::FAIL_COUNT)];
  auto& cls = map_[std::string(layers::CLASSIFICATION)];
  uint32_t cleared = 0;
  for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    auto t = static_cast<CellType>(static_cast<int>(cls((*it)(0), (*it)(1))));
    if (t == CellType::LAWN_DEAD)
    {
      cls((*it)(0), (*it)(1)) = static_cast<float>(CellType::LAWN);
      ++cleared;
    }
    fc((*it)(0), (*it)(1)) = 0.0F;
  }

  res->success = true;
  res->message = "cleared " + std::to_string(cleared) + " LAWN_DEAD cells";
  RCLCPP_INFO(get_logger(), "ClearDeadCells: %s", res->message.c_str());
}

}  // namespace mowgli_map
