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

// Costmap filter mask publisher (keepout) split out of map_server_node.cpp.
// ROS interface is unchanged: same /keepout_mask + /costmap_filter_info
// topics, same transient_local QoS, same X→col / Y→row OccupancyGrid
// convention (see CLAUDE.md invariant #14 — width=nx, height=ny, never swap).
// The speed-mask publisher was removed: nothing consumed /speed_mask (no
// SpeedFilter plugin in the Nav2 global or local costmap).

#include <algorithm>
#include <cmath>
#include <limits>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include <grid_map_core/iterators/PolygonIterator.hpp>

namespace mowgli_map
{

// Mask value for the outside-slack band (cells beyond every area polygon but
// within enforce_boundary_margin_m of an edge). NON-ZERO so the band is
// traversable-but-penalised: with CostmapFilterInfo base=0/multiplier=1 the
// KeepoutFilter turns 50 into cost ~127 — far below Smac's INSCRIBED(253)
// validity cutoff, so a start/goal pose in the band never fails "Start
// occupied", while A* only routes THROUGH the band when the inside route is
// much longer. A free (0) band would invite corner-cutting transits up to
// enforce_boundary_margin_m (0.40 m) outside the polygon — past the 0.30 m
// soft_boundary_margin_m deadband — firing spurious /boundary_violation
// recoveries mid-transit.
constexpr int8_t kOutsideSlackMaskCost = 50;

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
  // polygon edge is mid-cost (kOutsideSlackMaskCost) — traversable for a
  // start/goal pose near the boundary (prevents "Start occupied") but
  // penalised so the planner does not draft corner-cutting routes outside
  // the polygon.
  // Cells beyond the margin stay 100 (keepout/lethal).
  //
  // outside_free_margin selects the boundary policy:
  //   * lethal_outside_areas_ = true  (default, operator intent): use the
  //     small enforce_boundary_margin_m_ (0.40 m — chassis half-width plus
  //     costmap-cell/drift headroom). Everything beyond
  //     that slack is LETHAL, so the planner never routes outside the union
  //     of areas and MPPI never steers the robot out of the authorised zone
  //     (fixes the 0.32 m concave-boundary excursion). The dock corridor
  //     carve-out below still keeps a non-lethal transit/docking lane.
  //   * lethal_outside_areas_ = false: legacy behaviour — the wider
  //     keepout_nav_margin_ free band is honoured.
  const double outside_free_margin =
      lethal_outside_areas_ ? enforce_boundary_margin_m_ : keepout_nav_margin_;

  // Perf: the passes below are per-cell over the grid, whose count scales with
  // the map EXTENT (extent/resolution²) and is dominated by the empty margin
  // around a small polygon on a large map. Restrict iteration to the grid-index
  // bounding box that could hold a non-default cell: the union of every
  // area/obstacle/dock polygon, expanded by the free/obstacle margins (+1 cell).
  // Every cell outside the box keeps the default 100 (lethal) — identical to
  // what the full loop computes, because a cell only turns free (0) when inside
  // an area, within outside_free_margin of an area edge, or inside the dock
  // corridor, all of which lie within the box. Output is bit-identical; only the
  // iteration range shrinks. The Invariant-14 mapping below still uses nx/ny.
  double bx_min = std::numeric_limits<double>::max();
  double bx_max = std::numeric_limits<double>::lowest();
  double by_min = std::numeric_limits<double>::max();
  double by_max = std::numeric_limits<double>::lowest();
  const auto accumulate_polygon = [&](const geometry_msgs::msg::Polygon& poly)
  {
    for (const auto& p : poly.points)
    {
      bx_min = std::min(bx_min, static_cast<double>(p.x));
      bx_max = std::max(bx_max, static_cast<double>(p.x));
      by_min = std::min(by_min, static_cast<double>(p.y));
      by_max = std::max(by_max, static_cast<double>(p.y));
    }
  };
  for (const auto& area : areas_)
  {
    accumulate_polygon(area.polygon);
    for (const auto& obs : area.obstacles)
    {
      accumulate_polygon(obs);
    }
  }
  for (const auto& obs : obstacle_polygons_)
  {
    accumulate_polygon(obs);
  }
  if (has_dock_exclusion_)
  {
    accumulate_polygon(dock_body_polygon_);
    accumulate_polygon(dock_corridor_polygon_);
    accumulate_polygon(dock_exclusion_polygon_);
  }

  // Full-grid fallback (used if no polygon accumulated or a corner fails to map).
  int r0 = 0;
  int r1 = nx - 1;
  int c0 = 0;
  int c1 = ny - 1;
  if (bx_max >= bx_min)  // at least one polygon vertex accumulated
  {
    const double margin_expand = std::max(outside_free_margin, obstacle_margin_m_) + resolution_;
    const double cx = map_.getPosition().x();
    const double cy = map_.getPosition().y();
    const double hx = map_.getLength().x() * 0.5;
    const double hy = map_.getLength().y() * 0.5;
    const double eps = resolution_ * 0.5;
    const auto to_index = [&](double wx, double wy, grid_map::Index& out) -> bool
    {
      const grid_map::Position q(std::clamp(wx, cx - hx + eps, cx + hx - eps),
                                 std::clamp(wy, cy - hy + eps, cy + hy - eps));
      return map_.getIndex(q, out);
    };
    grid_map::Index i_a;
    grid_map::Index i_b;
    // grid_map r increases as X decreases, c as Y decreases; the two diagonal
    // world corners bound both axes after min/max.
    if (to_index(bx_min - margin_expand, by_min - margin_expand, i_a) &&
        to_index(bx_max + margin_expand, by_max + margin_expand, i_b))
    {
      r0 = std::clamp(std::min(i_a(0), i_b(0)), 0, nx - 1);
      r1 = std::clamp(std::max(i_a(0), i_b(0)), 0, nx - 1);
      c0 = std::clamp(std::min(i_a(1), i_b(1)), 0, ny - 1);
      c1 = std::clamp(std::max(i_a(1), i_b(1)), 0, ny - 1);
    }
  }

  for (int r = r0; r <= r1; ++r)
  {
    for (int c = c0; c <= c1; ++c)
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

      if (inside_any)
      {
        if (!inner_buffer)
        {
          mask.data[flat_idx] = 0;
        }
      }
      else if (within_outside_margin)
      {
        mask.data[flat_idx] = kOutsideSlackMaskCost;
      }
    }
  }

  // Overlay obstacle polygons: cells inside any obstacle -> 100 (lethal).
  // Two sources share this pass: obstacle_polygons_ (dynamic LiDAR-promoted)
  // and every area's DRAWN entry.obstacles (whose interiors are also lethal
  // via the classification NO_GO_ZONE overlay below — the polygon pass here
  // is what carries the margin band). obstacle_margin_m_
  // (mowgli_robot.yaml.obstacle_margin) additionally marks cells within that
  // distance OUTSIDE each polygon — the transit-side twin of
  // coverage_server's F2C hole buffering, so both planners keep the same
  // distance from a drawn tree/root zone.
  const auto cell_hits_obstacle =
      [this](const geometry_msgs::msg::Point32& pt, const geometry_msgs::msg::Polygon& obs)
  {
    if (point_in_polygon(pt, obs))
    {
      return true;
    }
    return obstacle_margin_m_ > 0.0 &&
           point_to_polygon_distance(static_cast<double>(pt.x), static_cast<double>(pt.y), obs) <=
               obstacle_margin_m_;
  };
  for (int r = r0; r <= r1; ++r)
  {
    for (int c = c0; c <= c1; ++c)
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

      bool lethal = false;
      for (const auto& obs : obstacle_polygons_)
      {
        if (cell_hits_obstacle(pt, obs))
        {
          lethal = true;
          break;
        }
      }
      for (std::size_t a = 0; !lethal && a < areas_.size(); ++a)
      {
        for (const auto& obs : areas_[a].obstacles)
        {
          if (cell_hits_obstacle(pt, obs))
          {
            lethal = true;
            break;
          }
        }
      }
      if (lethal)
      {
        const int og_col = nx - 1 - r;
        const int og_row = ny - 1 - c;
        mask.data[static_cast<std::size_t>(og_row * nx + og_col)] = 100;
      }
    }
  }

  // Overlay no-go zones from classification layer.
  const auto& cls = map_[std::string(layers::CLASSIFICATION)];
  const float no_go_val = static_cast<float>(CellType::NO_GO_ZONE);
  for (int r = r0; r <= r1; ++r)
  {
    for (int c = c0; c <= c1; ++c)
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
    for (int r = r0; r <= r1; ++r)
    {
      for (int c = c0; c <= c1; ++c)
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

}  // namespace mowgli_map
