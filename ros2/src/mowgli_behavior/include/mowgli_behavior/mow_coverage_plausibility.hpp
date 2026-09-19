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

#pragma once

// Pure, ROS-free coverage-completion plausibility check (issue #680).
//
// FollowStrip's swath-completion bookkeeping can report an area "fully
// mowed" (every planned swath driven and marked done) while, in physical
// reality, only the headland ring around the boundary was ever cut — the
// interior was never touched, and nothing caught the discrepancy before it
// reached the operator as a clean, confident "MOWING_COMPLETE". This header
// cross-checks that bookkeeping against map_server_node's independently-
// stamped ~/mow_progress grid (blade-verified swept cells), which does not
// depend on the same swath-count arithmetic that produced the false
// positive.
//
// Deliberately does NOT diagnose *why* the mismatch happened (root
// CLAUDE.md's Invariant 16 already found fusion_graph's own marginal
// covariance unusable for gating at any threshold, and this header does not
// reopen that question) — it only refuses to let a suspiciously low
// stamped fraction pass through as an unqualified success.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace mowgli_behavior
{

/// Minimal ROS-free view over a nav_msgs/OccupancyGrid's geometry + data, so
/// this header stays testable without rclcpp/nav_msgs linkage. Axis-aligned
/// (no origin rotation) — valid because root CLAUDE.md Invariant 4 pins the
/// map frame to GPS ENU with no rotation transform. Cell (row, col)'s centre
/// is at (origin_x + (col+0.5)*resolution, origin_y + (row+0.5)*resolution),
/// data[row*width+col] — the standard nav_msgs/OccupancyGrid convention (not
/// mowgli_map's internal grid_map row/col flip, root CLAUDE.md Invariant 14
/// — that flip is internal to how map_server_node BUILDS the message; once
/// published, a consumer reads it the standard way).
struct MowProgressGridView
{
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int32_t width{0};
  int32_t height{0};
  const std::vector<int8_t>* data{nullptr};
};

/// A cell counts as "mowed" once its stamped value reaches at least this.
/// map_server_node's stamp_mow_progress only ever writes 0.0F or 100.0F, so
/// any threshold strictly between the two works; kept well clear of both
/// for readability rather than tuned to a boundary.
constexpr int8_t kMowedCellThreshold = 50;

/// A completed pass is trusted only once at least this fraction of the
/// area's interior (outer boundary minus obstacle holes) is actually
/// stamped mowed. 0.5 sits well above what a headland-only pass leaves
/// behind (a thin perimeter band — a small fraction of most areas'
/// interior, per the issue's own before/after screenshot) and well below
/// full coverage, so odd-shaped or obstacle-heavy areas don't false-trip on
/// a genuinely complete mow.
constexpr double kMinPlausibleMowedFraction = 0.5;

/// Ray-casting point-in-polygon (even-odd rule). Kept independent of
/// mowgli_map's own copy (progress_tracker.cpp) so this package's geometry
/// stays dependency-free and unit-testable without linking mowgli_map.
inline bool PointInPolygon(double x,
                           double y,
                           const std::vector<std::pair<double, double>>& polygon) noexcept
{
  const std::size_t n = polygon.size();
  if (n < 3)
  {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double xi = polygon[i].first;
    const double yi = polygon[i].second;
    const double xj = polygon[j].first;
    const double yj = polygon[j].second;

    const bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect)
    {
      inside = !inside;
    }
  }
  return inside;
}

/// Fraction (0..1) of the area's interior cells — inside `outer`, outside
/// every polygon in `holes` — that are mowed per `grid`. Only samples the
/// grid cells within the outer polygon's bounding box. Returns 0.0 (the
/// safe "not plausible" default) for a degenerate outer polygon
/// (< 3 vertices), a grid with no data / non-positive geometry, or an
/// interior with zero sampled cells — never divides by zero.
inline double ComputeMowedFraction(const MowProgressGridView& grid,
                                   const std::vector<std::pair<double, double>>& outer,
                                   const std::vector<std::vector<std::pair<double, double>>>& holes)
{
  if (outer.size() < 3 || grid.data == nullptr || grid.width <= 0 || grid.height <= 0 ||
      grid.resolution <= 0.0)
  {
    return 0.0;
  }

  double min_x = outer.front().first;
  double max_x = outer.front().first;
  double min_y = outer.front().second;
  double max_y = outer.front().second;
  for (const auto& p : outer)
  {
    min_x = std::min(min_x, p.first);
    max_x = std::max(max_x, p.first);
    min_y = std::min(min_y, p.second);
    max_y = std::max(max_y, p.second);
  }

  const auto to_col = [&](double x)
  {
    return static_cast<int32_t>(std::floor((x - grid.origin_x) / grid.resolution));
  };
  const auto to_row = [&](double y)
  {
    return static_cast<int32_t>(std::floor((y - grid.origin_y) / grid.resolution));
  };

  const int32_t col_lo = std::max<int32_t>(0, to_col(min_x));
  const int32_t col_hi = std::min<int32_t>(grid.width - 1, to_col(max_x));
  const int32_t row_lo = std::max<int32_t>(0, to_row(min_y));
  const int32_t row_hi = std::min<int32_t>(grid.height - 1, to_row(max_y));

  std::uint64_t interior_cells = 0;
  std::uint64_t mowed_cells = 0;
  for (int32_t row = row_lo; row <= row_hi; ++row)
  {
    const double cell_y = grid.origin_y + (static_cast<double>(row) + 0.5) * grid.resolution;
    for (int32_t col = col_lo; col <= col_hi; ++col)
    {
      const double cell_x = grid.origin_x + (static_cast<double>(col) + 0.5) * grid.resolution;
      if (!PointInPolygon(cell_x, cell_y, outer))
      {
        continue;
      }
      bool in_hole = false;
      for (const auto& hole : holes)
      {
        if (PointInPolygon(cell_x, cell_y, hole))
        {
          in_hole = true;
          break;
        }
      }
      if (in_hole)
      {
        continue;
      }

      ++interior_cells;
      const auto idx = static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
                       static_cast<std::size_t>(col);
      if (idx < grid.data->size() && (*grid.data)[idx] >= kMowedCellThreshold)
      {
        ++mowed_cells;
      }
    }
  }

  if (interior_cells == 0)
  {
    return 0.0;
  }
  return static_cast<double>(mowed_cells) / static_cast<double>(interior_cells);
}

}  // namespace mowgli_behavior
