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

// Costmap filter mask publishers (keepout + speed) split out of
// map_server_node.cpp. ROS interface is unchanged: same /keepout_mask,
// /speed_mask, /costmap_filter_info, /speed_filter_info topics, same
// transient_local QoS, same X→col / Y→row OccupancyGrid convention
// (see CLAUDE.md invariant #14 — width=nx, height=ny, never swap).

#include <algorithm>
#include <cmath>
#include <limits>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <grid_map_core/iterators/PolygonIterator.hpp>

namespace mowgli_map
{

void MapServerNode::publish_keepout_mask()
{
  if (areas_.empty())
  {
    return;
  }

  // grid_map: size(0) = cells along X, size(1) = cells along Y.
  //   r=0 → X_max (decreasing), c=0 → Y_max (decreasing).
  // OccupancyGrid: width = X cells, height = Y cells.
  //   col=0 → X_min (at origin.x), row=0 → Y_min (at origin.y).
  // Both flip + swap roles: the OccupancyGrid's (row, col) is the grid_map's
  //   (cols - 1 - c, nx - 1 - r) mapping (see the grid_map → OccupancyGrid
  //   convention note in CLAUDE.md). Previously this publisher had the dimensions swapped —
  //   width/height set from the wrong grid_map axis — so every cell's
  //   value landed at a 90°-rotated position, marking interior polygon
  //   cells as lethal and breaking Smac planning with "Start occupied".
  const int nx = map_.getSize()(0);  // cells along X
  const int ny = map_.getSize()(1);  // cells along Y
  const float res = static_cast<float>(resolution_);

  nav_msgs::msg::OccupancyGrid mask;
  mask.header.stamp = now();
  mask.header.frame_id = map_frame_;
  mask.info.resolution = res;
  mask.info.width = static_cast<uint32_t>(nx);
  mask.info.height = static_cast<uint32_t>(ny);
  mask.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  mask.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  mask.info.origin.position.z = 0.0;
  mask.info.origin.orientation.w = 1.0;
  mask.data.resize(static_cast<std::size_t>(nx * ny), 100);  // default: keepout

  // A cell inside ANY area (mowing or navigation) is free (0).
  // A cell outside all areas but within `outside_free_margin` of any area
  // polygon edge is also free (0) — this prevents "Start occupied" when
  // the robot is near the boundary.
  // Cells beyond the margin stay 100 (keepout/lethal).
  //
  // outside_free_margin selects the boundary policy:
  //   * lethal_outside_areas_ = true  (default, operator intent): use the
  //     small enforce_boundary_margin_m_ (~robot radius). Everything beyond
  //     that slack is LETHAL, so the planner never routes outside the union
  //     of areas and MPPI never steers the robot out of the authorised zone
  //     (fixes the 0.32 m concave-boundary excursion). The dock corridor
  //     carve-out below still keeps a non-lethal transit/docking lane.
  //   * lethal_outside_areas_ = false: legacy behaviour — the wider
  //     keepout_nav_margin_ free band is honoured.
  const double outside_free_margin =
      lethal_outside_areas_ ? enforce_boundary_margin_m_ : keepout_nav_margin_;
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      grid_map::Position pos;
      const grid_map::Index idx(r, c);
      if (!map_.getPosition(idx, pos))
      {
        continue;
      }

      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(pos.x());
      pt.y = static_cast<float>(pos.y());
      pt.z = 0.0F;

      const int og_col = nx - 1 - r;  // grid_map r=0 (X_max) → OG col nx-1
      const int og_row = ny - 1 - c;  // grid_map c=0 (Y_max) → OG row ny-1
      const auto flat_idx = static_cast<std::size_t>(og_row * nx + og_col);

      bool inside_any = false;
      double inside_min_edge_dist = std::numeric_limits<double>::max();
      bool within_outside_margin = false;
      for (const auto& area : areas_)
      {
        if (point_in_polygon(pt, area.polygon))
        {
          inside_any = true;
          if (boundary_inner_margin_m_ > 0.0)
          {
            double d = point_to_polygon_distance(static_cast<double>(pt.x),
                                                 static_cast<double>(pt.y),
                                                 area.polygon);
            if (d < inside_min_edge_dist)
            {
              inside_min_edge_dist = d;
            }
          }
          // Keep scanning other polygons — a cell can be inside A but near the
          // edge of B. We want the nearest edge distance overall.
          continue;
        }
        if (!within_outside_margin && outside_free_margin > 0.0)
        {
          double dist = point_to_polygon_distance(static_cast<double>(pt.x),
                                                  static_cast<double>(pt.y),
                                                  area.polygon);
          if (dist <= outside_free_margin)
          {
            within_outside_margin = true;
          }
        }
      }

      // Shrunk-polygon rule: cells inside a mowing area but within
      // boundary_inner_margin_m_ of the nearest edge become LETHAL in the
      // keepout mask. Effect: the Smac planner never drafts a path that
      // comes within that margin of the polygon edge, giving the FTC
      // controller room to track without spilling over. Combined with
      // inflation_layer, the total soft-wall is ~ margin + inflation_radius.
      bool inner_buffer = inside_any && boundary_inner_margin_m_ > 0.0 &&
                          inside_min_edge_dist < boundary_inner_margin_m_;

      if ((inside_any || within_outside_margin) && !inner_buffer)
      {
        mask.data[flat_idx] = 0;
      }
    }
  }

  // Overlay obstacle polygons: cells inside any obstacle -> 100 (lethal).
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      grid_map::Position pos;
      const grid_map::Index idx(r, c);
      if (!map_.getPosition(idx, pos))
      {
        continue;
      }

      geometry_msgs::msg::Point32 pt;
      pt.x = static_cast<float>(pos.x());
      pt.y = static_cast<float>(pos.y());
      pt.z = 0.0F;

      for (const auto& obs : obstacle_polygons_)
      {
        if (point_in_polygon(pt, obs))
        {
          const int og_col = nx - 1 - r;
          const int og_row = ny - 1 - c;
          mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 100;
          break;
        }
      }
    }
  }

  // Overlay no-go zones from classification layer.
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (int r = 0; r < nx; ++r)
  {
    for (int c = 0; c < ny; ++c)
    {
      if (cls(r, c) == no_go_val)
      {
        const int og_col = nx - 1 - r;
        const int og_row = ny - 1 - c;
        mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 100;
      }
    }
  }

  // Dock corridor carve-out: force every cell inside the corridor polygon
  // back to free (0), no matter what the previous passes set. Smac needs
  // a non-lethal lane through the corridor for post-undock transit, so
  // this carve overrides obstacle_polygons_, the inner-margin buffer, and
  // any classification-layer no-go that happens to overlap. The dock body
  // itself is NOT carved — it stays lethal via OBSTACLE_PERMANENT.
  if (has_dock_exclusion_ && dock_corridor_polygon_.points.size() >= 3)
  {
    for (int r = 0; r < nx; ++r)
    {
      for (int c = 0; c < ny; ++c)
      {
        grid_map::Position pos;
        const grid_map::Index idx(r, c);
        if (!map_.getPosition(idx, pos))
        {
          continue;
        }
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(pos.x());
        pt.y = static_cast<float>(pos.y());
        pt.z = 0.0F;
        if (point_in_polygon(pt, dock_corridor_polygon_))
        {
          const int og_col = nx - 1 - r;
          const int og_row = ny - 1 - c;
          mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 0;
        }
      }
    }
  }

  cached_keepout_mask_ = mask;
  keepout_mask_pub_->publish(mask);

  // Publish filter info only once (transient_local latches it for late
  // subscribers).  Republishing every cycle causes Nav2 KeepoutFilter to
  // re-subscribe to the mask topic each time, blocking the costmap update
  // thread and starving the planner of CPU.
  if (!keepout_filter_info_sent_)
  {
    nav2_msgs::msg::CostmapFilterInfo info;
    info.header.stamp = mask.header.stamp;
    info.header.frame_id = map_frame_;
    info.type = 0;  // KEEPOUT = 0
    info.filter_mask_topic = "/keepout_mask";
    info.base = 0.0F;
    info.multiplier = 1.0F;
    keepout_filter_info_pub_->publish(info);
    keepout_filter_info_sent_ = true;
  }
}

void MapServerNode::publish_speed_mask()
{
  if (areas_.empty())
  {
    return;
  }

  // See publish_keepout_mask for the X/Y→OccupancyGrid convention.
  const int nx = map_.getSize()(0);  // cells along X
  const int ny = map_.getSize()(1);  // cells along Y
  const float res = static_cast<float>(resolution_);

  const double headland_radius = tool_width_;

  nav_msgs::msg::OccupancyGrid mask;
  mask.header.stamp = now();
  mask.header.frame_id = map_frame_;
  mask.info.resolution = res;
  mask.info.width = static_cast<uint32_t>(nx);
  mask.info.height = static_cast<uint32_t>(ny);
  mask.info.origin.position.x = map_.getPosition().x() - map_.getLength().x() * 0.5;
  mask.info.origin.position.y = map_.getPosition().y() - map_.getLength().y() * 0.5;
  mask.info.origin.position.z = 0.0;
  mask.info.origin.orientation.w = 1.0;
  mask.data.resize(static_cast<std::size_t>(nx * ny), 0);  // default: full speed

  for (const auto& area : areas_)
  {
    const auto& pts = area.polygon.points;
    const std::size_t n = pts.size();
    if (n < 3)
      continue;

    for (int r = 0; r < nx; ++r)
    {
      for (int c = 0; c < ny; ++c)
      {
        grid_map::Position pos;
        const grid_map::Index idx(r, c);
        if (!map_.getPosition(idx, pos))
        {
          continue;
        }

        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(pos.x());
        pt.y = static_cast<float>(pos.y());
        pt.z = 0.0F;

        if (!point_in_polygon(pt, area.polygon))
        {
          continue;
        }

        double min_dist_sq = std::numeric_limits<double>::max();

        for (std::size_t i = 0, j = n - 1; i < n; j = i++)
        {
          const double ax = static_cast<double>(pts[j].x);
          const double ay = static_cast<double>(pts[j].y);
          const double bx = static_cast<double>(pts[i].x);
          const double by = static_cast<double>(pts[i].y);

          const double dx = bx - ax;
          const double dy = by - ay;
          const double len_sq = dx * dx + dy * dy;

          double dist_sq = 0.0;
          if (len_sq < 1e-12)
          {
            const double ex = pos.x() - ax;
            const double ey = pos.y() - ay;
            dist_sq = ex * ex + ey * ey;
          }
          else
          {
            const double t =
                std::clamp(((pos.x() - ax) * dx + (pos.y() - ay) * dy) / len_sq, 0.0, 1.0);
            const double proj_x = ax + t * dx - pos.x();
            const double proj_y = ay + t * dy - pos.y();
            dist_sq = proj_x * proj_x + proj_y * proj_y;
          }

          if (dist_sq < min_dist_sq)
          {
            min_dist_sq = dist_sq;
          }
        }

        if (min_dist_sq <= headland_radius * headland_radius)
        {
          const int og_col = nx - 1 - r;
          const int og_row = ny - 1 - c;
          mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 50;
        }
      }
    }
  }

  cached_speed_mask_ = mask;
  speed_mask_pub_->publish(mask);

  if (!speed_filter_info_sent_)
  {
    nav2_msgs::msg::CostmapFilterInfo info;
    info.header.stamp = mask.header.stamp;
    info.header.frame_id = map_frame_;
    info.type = 1;  // SPEED_LIMIT = 1 (percentage mode)
    info.filter_mask_topic = "/speed_mask";
    info.base = 100.0F;
    info.multiplier = -1.0F;
    speed_filter_info_pub_->publish(info);
    speed_filter_info_sent_ = true;
  }
}

}  // namespace mowgli_map
