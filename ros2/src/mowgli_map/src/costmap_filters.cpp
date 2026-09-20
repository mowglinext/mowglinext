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
#include <cstdint>
#include <limits>
#include <vector>

#include "mowgli_map/internal_helpers.hpp"
#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/polygon_raster.hpp"

namespace mowgli_map
{

// Mask value for every soft (traversable-but-penalised) band this publisher
// paints — the outside-slack band (cells beyond every area polygon but
// within enforce_boundary_margin_m of an edge) AND the inside transit margin
// (cells inside an area but within boundary_inner_margin_m of its edge, see
// below). NON-ZERO so both bands stay traversable: with CostmapFilterInfo
// base=0/multiplier=1 the KeepoutFilter turns 50 into cost ~127 — far below
// Smac's INSCRIBED(253) validity cutoff, so a start/goal pose in either band
// never fails "Start occupied" and A* only routes THROUGH the band when the
// alternative is much longer. This is deliberately NEVER a lethal value for
// either band: a free outside band would invite corner-cutting transits up
// to enforce_boundary_margin_m outside the polygon — past the
// 0.30 m soft_boundary_margin_m deadband — firing spurious
// /boundary_violation recoveries mid-transit. That pressure GREW when the
// launch-injected floor went from the chassis half-width to the chassis
// circumscribed radius (0.40 -> 0.597 m on the shipped chassis, 2026-09-17):
// the band now reaches past lethal_boundary_margin_m (0.5 m) too, so the
// mid-cost value is the only thing keeping the planner off it. Watch
// /boundary_violation in the field. A lethal inside band very
// nearly stranded the robot near the dock once already (map_server_node.hpp)
// and, independently, collides with chassis_safety_inset — the outermost
// coverage ring is planned exactly chassis_safety_inset inside the line
// (0.20 m by default, the same default as boundary_inner_margin_m), so a
// lethal band there plus inflation_radius (0.20 m) would swallow the ring
// itself and reopen the START_OCCUPIED skip cascade (issue #487).
constexpr int8_t kSoftPenaltyMaskCost = 50;

raster::CellAxes MapServerNode::make_cell_axes(const grid_map::GridMap& map, bool through_float)
{
  const auto coordinate = [through_float](double value)
  {
    return through_float ? static_cast<double>(static_cast<float>(value)) : value;
  };
  raster::CellAxes axes;
  const int rows = map.getSize()(0);
  const int cols = map.getSize()(1);
  axes.x_of_row.reserve(static_cast<std::size_t>(rows));
  axes.y_of_col.reserve(static_cast<std::size_t>(cols));
  grid_map::Position pos;
  for (int r = 0; r < rows; ++r)
  {
    map.getPosition(grid_map::Index(r, 0), pos);
    axes.x_of_row.push_back(coordinate(pos.x()));
  }
  for (int c = 0; c < cols; ++c)
  {
    map.getPosition(grid_map::Index(0, c), pos);
    axes.y_of_col.push_back(coordinate(pos.y()));
  }
  return axes;
}

void MapServerNode::publish_keepout_mask()
{
  if (areas_.empty())
  {
    return;
  }

  // The NO_GO overlay below reads the CLASSIFICATION layer, which add_area no
  // longer stamps inside its service callback.
  ensure_classification_current_locked();

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

  // A cell inside ANY area (mowing or navigation) is free (0), unless it
  // falls in the inside transit margin below (mid-cost, never lethal).
  // A cell outside all areas but within `outside_free_margin` of any area
  // polygon edge is mid-cost (kSoftPenaltyMaskCost) — traversable for a
  // start/goal pose near the boundary (prevents "Start occupied") but
  // penalised so the planner does not draft corner-cutting routes outside
  // the polygon.
  // Cells beyond the margin stay 100 (keepout/lethal).
  //
  // outside_free_margin selects the boundary policy:
  //   * lethal_outside_areas_ = true  (default, operator intent): use the
  //     enforce_boundary_margin_m_ band (0.40 m standalone default, and FLOORED
  //     at the live chassis CIRCUMSCRIBED RADIUS — 0.597 m shipped — by
  //     full_system.launch.py: the band has to hold the whole body overhanging
  //     the recorded line, which it does by design now that
  //     chassis_safety_inset is 0, and at a row END the footprint noses 0.53 m
  //     past the line, not just the 0.275 m half-width). Everything beyond
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
      if (!obs.pending)  // a proposal is not part of any mask, not even its extent
      {
        accumulate_polygon(obs.polygon);
      }
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
    const double margin_expand =
        std::max(outside_free_margin, keepout_obstacle_margin_m_) + resolution_;
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

  // Cell centres, rounded through float: the per-cell definition of this mask
  // tests `static_cast<float>(centre)` and measures distances from that same
  // rounded point, so the rasteriser has to see the rounded coordinates too.
  const raster::CellAxes axes = make_cell_axes(map_, /*through_float=*/true);
  const raster::CellWindow window{{r0, r1}, {c0, c1}};
  const std::size_t window_cols = static_cast<std::size_t>(c1 - c0 + 1);
  const auto window_cell = [r0, c0, window_cols](int r, int c)
  {
    return static_cast<std::size_t>(r - r0) * window_cols + static_cast<std::size_t>(c - c0);
  };
  const auto og_cell = [nx, ny](int r, int c)
  {
    // Invariant 14: grid_map r=0 is X_max → OG col nx-1; c=0 is Y_max → OG row ny-1.
    return static_cast<std::size_t>((ny - 1 - c) * nx + (nx - 1 - r));
  };

  // Pass 1 — areas. Polygon-driven (see polygon_raster.hpp): per area, one ray
  // cast per grid line marks the inside cells, then each boundary EDGE visits
  // only the cells of its own margin-grown bounding box. The flags reproduce
  // the per-cell rule exactly:
  //   inside ANY area                                   → free (0)
  //   …and closer than boundary_inner_margin_m_ to the
  //     edge of an area it is INSIDE, away from the dock → soft (50)
  //   outside every area, within outside_free_margin of
  //     the edge of an area                              → soft (50)
  constexpr uint8_t kInsideAny = 1U << 0;
  constexpr uint8_t kInnerBand = 1U << 1;
  constexpr uint8_t kOuterBand = 1U << 2;
  const double inner_margin = std::max(boundary_inner_margin_m_, 0.0);
  const double outer_margin = std::max(outside_free_margin, 0.0);
  const double band = std::max(inner_margin, outer_margin);
  std::vector<uint8_t> flags(static_cast<std::size_t>(r1 - r0 + 1) * window_cols, 0);
  std::vector<uint8_t> inside_this_area(flags.size(), 0);
  for (const auto& area : areas_)
  {
    std::fill(inside_this_area.begin(), inside_this_area.end(), 0);
    raster::for_each_cell_inside<float>(area.polygon,
                                        axes,
                                        window,
                                        [&](int r, int c)
                                        {
                                          inside_this_area[window_cell(r, c)] = 1;
                                          flags[window_cell(r, c)] |= kInsideAny;
                                        });
    if (band <= 0.0)
    {
      continue;
    }
    raster::for_each_cell_near_edges(area.polygon,
                                     axes,
                                     window,
                                     band,
                                     [&](int r, int c, double distance)
                                     {
                                       const std::size_t k = window_cell(r, c);
                                       if (inside_this_area[k] != 0)
                                       {
                                         if (inner_margin > 0.0 && distance < inner_margin)
                                         {
                                           flags[k] |= kInnerBand;
                                         }
                                       }
                                       else if (outer_margin > 0.0 && distance <= outer_margin)
                                       {
                                         flags[k] |= kOuterBand;
                                       }
                                     });
  }

  // Cells within dock_inner_margin_exempt_radius_m_ of the dock pose are
  // exempt from the inner penalty, in EVERY direction, so the dock approach
  // carries no bias at all — not even the soft cost. Unlike
  // dock_corridor_polygon_ (a fixed rectangle carved out further down in this
  // function), this isotropic exemption doesn't depend on getting the
  // corridor's orientation/size right — it directly covers wherever GNSS
  // drift actually puts the robot's own position near the dock.
  const bool dock_exempts = has_dock_exclusion_ && dock_inner_margin_exempt_radius_m_ > 0.0;
  const auto near_dock = [&](int r, int c)
  {
    const double ddx = axes.x_of_row[static_cast<std::size_t>(r)] - docking_pose_.position.x;
    const double ddy = axes.y_of_col[static_cast<std::size_t>(c)] - docking_pose_.position.y;
    return (ddx * ddx + ddy * ddy) <=
           dock_inner_margin_exempt_radius_m_ * dock_inner_margin_exempt_radius_m_;
  };

  // Inside transit margin: the inner band gets the SAME soft mid-cost as the
  // outside-slack band — deliberately NEVER lethal (see kSoftPenaltyMaskCost).
  // Effect: the global planner (Smac, used for point-to-point TRANSIT) prefers
  // a route that stays that far inside the recorded edge when one exists, but
  // is never blocked from starting, ending, or passing through this band —
  // coverage/mowing itself never sees this mask (FTC tracks the F2C path
  // against the LOCAL costmap instead), and neither does a narrow seam between
  // two adjacent areas, which stays fully crossable, just costed.
  for (int r = r0; r <= r1; ++r)
  {
    for (int c = c0; c <= c1; ++c)
    {
      const uint8_t f = flags[window_cell(r, c)];
      if ((f & kInsideAny) != 0)
      {
        const bool inner_penalty = (f & kInnerBand) != 0 && !(dock_exempts && near_dock(r, c));
        mask.data[og_cell(r, c)] = inner_penalty ? kSoftPenaltyMaskCost : 0;
      }
      else if ((f & kOuterBand) != 0)
      {
        mask.data[og_cell(r, c)] = kSoftPenaltyMaskCost;
      }
    }
  }

  // Pass 2 — obstacle polygons: cells inside any obstacle -> 100 (lethal).
  // Two sources share this pass: obstacle_polygons_ (dynamic LiDAR-promoted)
  // and every area's DRAWN entry.obstacles (whose interiors are also lethal
  // via the classification NO_GO_ZONE overlay below — the polygon pass here
  // is what carries the margin band). keepout_obstacle_margin_m_ additionally
  // marks cells within that distance OUTSIDE each polygon: the body half-width,
  // because the mask's consumer (Smac 2D) is a point check and the mask is not
  // inflated downstream — polygon + this band IS the lethal region, the body
  // counted exactly once. It is deliberately smaller than coverage_server's
  // obstacle_margin so a robot on its coverage line is never START_OCCUPIED.
  // Each obstacle only visits its own margin-grown bounding box.
  const auto stamp_window_cells =
      [&](const geometry_msgs::msg::Polygon& polygon, double margin, int8_t value, const auto& hits)
  {
    const raster::CellWindow w = raster::polygon_window(polygon, axes, window, margin);
    for (int r = w.rows.first; r <= w.rows.last; ++r)
    {
      for (int c = w.cols.first; c <= w.cols.last; ++c)
      {
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(axes.x_of_row[static_cast<std::size_t>(r)]);
        pt.y = static_cast<float>(axes.y_of_col[static_cast<std::size_t>(c)]);
        pt.z = 0.0F;
        if (hits(pt, polygon))
        {
          mask.data[og_cell(r, c)] = value;
        }
      }
    }
  };
  const double obstacle_band = std::max(keepout_obstacle_margin_m_, 0.0);
  const auto cell_hits_obstacle =
      [this](const geometry_msgs::msg::Point32& pt, const geometry_msgs::msg::Polygon& obs)
  {
    if (point_in_polygon(pt, obs))
    {
      return true;
    }
    return keepout_obstacle_margin_m_ > 0.0 &&
           point_to_polygon_distance(static_cast<double>(pt.x), static_cast<double>(pt.y), obs) <=
               keepout_obstacle_margin_m_;
  };
  for (const auto& obs : obstacle_polygons_)
  {
    stamp_window_cells(obs, obstacle_band, 100, cell_hits_obstacle);
  }
  for (const auto& area : areas_)
  {
    for (const auto& obs : area.obstacles)
    {
      // PENDING proposals (wheel-slip dig reports) are NEVER lethal: the
      // robot stands ~0.2-0.3 m from a fresh dig point, and a keepout
      // there refused every plan from its own pose (START_OCCUPIED,
      // 2026-09-10 and 2026-09-17). Only an operator accept applies one.
      if (!obs.pending)
      {
        stamp_window_cells(obs.polygon, obstacle_band, 100, cell_hits_obstacle);
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
    stamp_window_cells(dock_corridor_polygon_,
                       0.0,
                       0,
                       [](const geometry_msgs::msg::Point32& pt,
                          const geometry_msgs::msg::Polygon& corridor)
                       {
                         return point_in_polygon(pt, corridor);
                       });
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
