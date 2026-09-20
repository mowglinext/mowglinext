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

// Package-private polygon rasterisation for map_server_node.
//
// Every grid pass map_server runs used to ask, for EACH CELL, "is this centre
// inside the polygon?" and "how far is it from the polygon?" — both O(vertices)
// — so a rebuild cost O(cells x vertices): 5.6 s measured on a desktop core
// for a ~2350 m² garden recorded with 750 boundary points at 0.05 m (several
// times that on a Raspberry Pi), on the node's only executor thread, once per
// add_area of a GUI "Save map". That is what timed the save out.
//
// The helpers here produce the SAME cell sets from the polygon's side instead:
//   * inside   — one ray-cast per grid LINE (not per cell): the crossings of a
//                line with the polygon are computed once, and a cell is inside
//                when an odd number of them lie to its right;
//   * distance — each EDGE visits only the cells of its own bounding box grown
//                by the margin, and reports the exact distance to that edge.
// The arithmetic is the per-cell arithmetic, expression for expression, so the
// result is bit-identical (pinned by test_map_replace.cpp); only the iteration
// order changed. Pure and ROS-free on purpose.

#ifndef MOWGLI_MAP__POLYGON_RASTER_HPP_
#define MOWGLI_MAP__POLYGON_RASTER_HPP_

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/polygon.hpp>

#include "mowgli_map/internal_helpers.hpp"

namespace mowgli_map::raster
{

/// Cell-centre coordinates of a grid_map, per index. grid_map positions are
/// separable (x depends on the row index only, y on the column index only) and
/// DECREASE with the index — see CLAUDE.md Invariant 14.
struct CellAxes
{
  std::vector<double> x_of_row;
  std::vector<double> y_of_col;
};

/// Inclusive index range; empty when last < first.
struct IndexRange
{
  int first{0};
  int last{-1};

  [[nodiscard]] bool empty() const noexcept
  {
    return last < first;
  }
};

/// Inclusive rectangle of cells.
struct CellWindow
{
  IndexRange rows;
  IndexRange cols;
};

inline IndexRange intersect(const IndexRange& a, const IndexRange& b) noexcept
{
  return {std::max(a.first, b.first), std::min(a.last, b.last)};
}

/// Indices whose coordinate lies in [lo, hi], for a DECREASING axis, widened
/// by one cell on each side. The widening only ever adds candidates — callers
/// run the exact per-cell test on every index returned — so it absorbs any
/// float/double rounding at the ends for free.
inline IndexRange covering_range(const std::vector<double>& decreasing_axis, double lo, double hi)
{
  if (decreasing_axis.empty() || !(lo <= hi))
  {
    return {};
  }
  const auto first_le_hi = std::partition_point(decreasing_axis.begin(),
                                                decreasing_axis.end(),
                                                [hi](double v)
                                                {
                                                  return v > hi;
                                                });
  const auto first_lt_lo = std::partition_point(decreasing_axis.begin(),
                                                decreasing_axis.end(),
                                                [lo](double v)
                                                {
                                                  return v >= lo;
                                                });
  const int n = static_cast<int>(decreasing_axis.size());
  const int first = static_cast<int>(first_le_hi - decreasing_axis.begin()) - 1;
  const int last = static_cast<int>(first_lt_lo - decreasing_axis.begin());
  return {std::max(first, 0), std::min(last, n - 1)};
}

/// X coordinates where the horizontal line at `y` crosses the polygon, sorted.
/// Exactly the crossing test + intersection of the classic even-odd ray cast
/// (MapServerNode::point_in_polygon in float, grid_map::Polygon::isInside in
/// double — hence the template).
template <typename Scalar>
void row_crossings(const geometry_msgs::msg::Polygon& polygon, Scalar y, std::vector<Scalar>& out)
{
  out.clear();
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 3)
  {
    return;
  }
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const Scalar xi = static_cast<Scalar>(pts[i].x);
    const Scalar yi = static_cast<Scalar>(pts[i].y);
    const Scalar xj = static_cast<Scalar>(pts[j].x);
    const Scalar yj = static_cast<Scalar>(pts[j].y);
    if ((yi > y) != (yj > y))
    {
      out.push_back((xj - xi) * (y - yi) / (yj - yi) + xi);
    }
  }
  std::sort(out.begin(), out.end());
}

/// Even-odd rule: inside when an odd number of crossings satisfy `x < crossing`.
template <typename Scalar>
[[nodiscard]] bool inside_row(const std::vector<Scalar>& sorted_crossings, Scalar x)
{
  const auto first_right = std::upper_bound(sorted_crossings.begin(), sorted_crossings.end(), x);
  return ((sorted_crossings.end() - first_right) & 1) != 0;
}

/// Calls `fn(row, col)` once for every cell of `window` whose centre is inside
/// the polygon. `Scalar` selects the precision the centre and the vertices are
/// compared in.
template <typename Scalar, typename Fn>
void for_each_cell_inside(const geometry_msgs::msg::Polygon& polygon,
                          const CellAxes& axes,
                          const CellWindow& window,
                          Fn&& fn)
{
  if (polygon.points.size() < 3)
  {
    return;
  }
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto& p : polygon.points)
  {
    y_min = std::min(y_min, static_cast<double>(p.y));
    y_max = std::max(y_max, static_cast<double>(p.y));
  }
  const IndexRange cols = intersect(window.cols, covering_range(axes.y_of_col, y_min, y_max));

  std::vector<Scalar> crossings;
  for (int c = cols.first; c <= cols.last; ++c)
  {
    row_crossings(polygon,
                  static_cast<Scalar>(axes.y_of_col[static_cast<std::size_t>(c)]),
                  crossings);
    if (crossings.empty())
    {
      continue;
    }
    const IndexRange rows = intersect(window.rows,
                                      covering_range(axes.x_of_row,
                                                     static_cast<double>(crossings.front()),
                                                     static_cast<double>(crossings.back())));
    for (int r = rows.first; r <= rows.last; ++r)
    {
      if (inside_row(crossings, static_cast<Scalar>(axes.x_of_row[static_cast<std::size_t>(r)])))
      {
        fn(r, c);
      }
    }
  }
}

/// Calls `fn(row, col, distance)` for every (edge, cell) pair where the cell
/// lies in the edge's bounding box grown by `margin` — a superset of the cells
/// within `margin` of the polygon. A cell near several edges is reported once
/// per edge, each time with the exact distance to THAT edge, so
/// "min over edges <= m" is "any reported distance <= m". Same edge walk as
/// closest_edge_point(), including the closing edge.
template <typename Fn>
void for_each_cell_near_edges(const geometry_msgs::msg::Polygon& polygon,
                              const CellAxes& axes,
                              const CellWindow& window,
                              double margin,
                              Fn&& fn)
{
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 2)
  {
    return;
  }
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = static_cast<double>(pts[j].x);
    const double ay = static_cast<double>(pts[j].y);
    const double bx = static_cast<double>(pts[i].x);
    const double by = static_cast<double>(pts[i].y);
    const IndexRange rows = intersect(window.rows,
                                      covering_range(axes.x_of_row,
                                                     std::min(ax, bx) - margin,
                                                     std::max(ax, bx) + margin));
    const IndexRange cols = intersect(window.cols,
                                      covering_range(axes.y_of_col,
                                                     std::min(ay, by) - margin,
                                                     std::max(ay, by) + margin));
    for (int r = rows.first; r <= rows.last; ++r)
    {
      const double px = axes.x_of_row[static_cast<std::size_t>(r)];
      for (int c = cols.first; c <= cols.last; ++c)
      {
        const double py = axes.y_of_col[static_cast<std::size_t>(c)];
        fn(r, c, closest_point_on_segment(px, py, ax, ay, bx, by).distance);
      }
    }
  }
}

/// Cells of `window` that can lie inside the polygon or within `margin` of it.
inline CellWindow polygon_window(const geometry_msgs::msg::Polygon& polygon,
                                 const CellAxes& axes,
                                 const CellWindow& window,
                                 double margin)
{
  if (polygon.points.empty())
  {
    return {};
  }
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto& p : polygon.points)
  {
    x_min = std::min(x_min, static_cast<double>(p.x));
    x_max = std::max(x_max, static_cast<double>(p.x));
    y_min = std::min(y_min, static_cast<double>(p.y));
    y_max = std::max(y_max, static_cast<double>(p.y));
  }
  return {intersect(window.rows, covering_range(axes.x_of_row, x_min - margin, x_max + margin)),
          intersect(window.cols, covering_range(axes.y_of_col, y_min - margin, y_max + margin))};
}

}  // namespace mowgli_map::raster

#endif  // MOWGLI_MAP__POLYGON_RASTER_HPP_
