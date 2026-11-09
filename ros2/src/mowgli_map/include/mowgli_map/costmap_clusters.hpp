// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Package-private clustering stage of obstacle_tracker_node, run on every
// global-costmap update (1 Hz, full or delta).
//
// The global costmap spans the whole garden, and everything outside the
// recorded areas is lethal there, so the flood fill visits a large share of the
// grid on every update and the "outside" component alone is hundreds of
// thousands of cells. The tracker used to store every one of them as a point,
// then std::hypot each against the centroid, only to reject the component for
// exceeding max_obstacle_radius. Now:
//   * a component whose points already span more than 2 x max_radius (+1 cell)
//     along x or y stops collecting points: two points that far apart cannot
//     both lie within max_radius of ANY centroid, so associate_clusters() would
//     have rejected it. The span is measured on the stored coordinates — the
//     very doubles the radius is later computed from — so the argument holds at
//     any coordinate magnitude. The component is still flooded, so the visited
//     set is unchanged;
//   * the radius scan stops as soon as it exceeds max_radius (cluster_extent);
//   * the flood fill uses a padded "open" byte mask (no bounds checks, no
//     std::vector<bool>) and no per-cell heap allocation.
// Every component that can pass the radius filter comes out exactly as before:
// same order, same point order (row-major seeds, LIFO stack, neighbours +x, -x,
// +y, -y), same coordinates — which matters because the centroid sums the
// points in that order. Pinned against the original code by
// test_costmap_clusters.cpp.

#ifndef MOWGLI_MAP__COSTMAP_CLUSTERS_HPP_
#define MOWGLI_MAP__COSTMAP_CLUSTERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace mowgli_map
{

using CellCluster = std::vector<std::pair<double, double>>;

/// 4-connected components of the cells whose value is >= `min_cost`, as
/// map-frame cell centres. Components with fewer than `min_points` cells, and
/// components that provably have a bounding radius > `max_radius` (see the
/// file comment), are not returned.
inline std::vector<CellCluster> cluster_costmap_cells(const nav_msgs::msg::OccupancyGrid& grid,
                                                      int8_t min_cost,
                                                      int min_points,
                                                      double max_radius)
{
  std::vector<CellCluster> clusters;
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (w <= 0 || h <= 0)
  {
    return clusters;
  }
  const double res = grid.info.resolution;
  const double ox = grid.info.origin.position.x;
  const double oy = grid.info.origin.position.y;

  // Two stored points more than `max_span` metres apart along one axis put the
  // farther of them more than max_radius + res / 2 from ANY centroid; the half
  // cell absorbs the rounding of the subtraction and std::hypot (each a few
  // ulp of the radius itself). A NaN/infinite bound or a non-positive
  // resolution disables the early drop; a negative bound can only drop
  // clusters the radius filter rejects anyway (max_r >= 0 > bound). So does a
  // non-finite cell centre (non-finite origin, or one far enough out to
  // overflow): the original radius was then NaN, which std::max ignores, and
  // such a cluster passed.
  const auto finite_centres = [res](double origin, int cells)
  {
    return std::isfinite(origin + 0.5 * res) && std::isfinite(origin + (cells - 1 + 0.5) * res);
  };
  double max_span = std::numeric_limits<double>::infinity();
  if (res > 0.0 && std::isfinite(2.0 * max_radius + res) && finite_centres(ox, w) &&
      finite_centres(oy, h))
  {
    max_span = 2.0 * max_radius + res;
  }

  // open[] = "obstacle cell not yet visited", with a one-cell closed border so
  // a neighbour test is a single load with no bounds check.
  const std::ptrdiff_t pw = static_cast<std::ptrdiff_t>(w) + 2;
  const std::ptrdiff_t ph = static_cast<std::ptrdiff_t>(h) + 2;
  std::vector<uint8_t> open(static_cast<std::size_t>(pw) * static_cast<std::size_t>(ph), 0U);
  for (int y = 0; y < h; ++y)
  {
    const int8_t* const row = grid.data.data() + static_cast<std::size_t>(y) * w;
    uint8_t* const out = open.data() + (y + 1) * pw + 1;
    for (int x = 0; x < w; ++x)
    {
      out[x] = row[x] >= min_cost ? 1U : 0U;
    }
  }

  struct Cell
  {
    int x;
    int y;
    std::ptrdiff_t padded;
  };
  std::vector<Cell> stack;
  // Neighbour order of the original flood fill: +x, -x, +y, -y.
  const std::ptrdiff_t step[4] = {1, -1, pw, -pw};
  constexpr int kDx[4] = {1, -1, 0, 0};
  constexpr int kDy[4] = {0, 0, 1, -1};

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const std::ptrdiff_t seed = (y + 1) * pw + x + 1;
      if (!open[seed])
      {
        continue;
      }

      CellCluster cluster;
      bool too_wide = false;
      double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
      stack.clear();
      stack.push_back({x, y, seed});
      open[seed] = 0U;

      while (!stack.empty())
      {
        const Cell c = stack.back();
        stack.pop_back();
        if (!too_wide)
        {
          const double px = ox + (c.x + 0.5) * res;
          const double py = oy + (c.y + 0.5) * res;
          if (cluster.empty())
          {
            min_x = max_x = px;
            min_y = max_y = py;
          }
          min_x = std::min(min_x, px);
          max_x = std::max(max_x, px);
          min_y = std::min(min_y, py);
          max_y = std::max(max_y, py);
          if (max_x - min_x > max_span || max_y - min_y > max_span)
          {
            too_wide = true;
            CellCluster().swap(cluster);  // release it; keep flooding
          }
          else
          {
            cluster.emplace_back(px, py);
          }
        }

        for (int k = 0; k < 4; ++k)
        {
          const std::ptrdiff_t n = c.padded + step[k];
          if (open[n])
          {
            open[n] = 0U;
            stack.push_back({c.x + kDx[k], c.y + kDy[k], n});
          }
        }
      }

      if (!too_wide && static_cast<int>(cluster.size()) >= min_points)
      {
        clusters.push_back(std::move(cluster));
      }
    }
  }
  return clusters;
}

struct ClusterExtent
{
  double cx{0.0};
  double cy{0.0};
  double max_r{0.0};  ///< max distance of a point to the centroid
};

/// Centroid and bounding radius of a non-empty cluster, computed exactly as
/// associate_clusters() always did (same summation order, std::hypot). The
/// radius scan stops once it exceeds `reject_above`: the caller rejects any
/// cluster with max_r > reject_above, and max_r only grows, so the decision is
/// unchanged.
inline ClusterExtent cluster_extent(const CellCluster& cluster, double reject_above)
{
  ClusterExtent e;
  for (const auto& [x, y] : cluster)
  {
    e.cx += x;
    e.cy += y;
  }
  e.cx /= static_cast<double>(cluster.size());
  e.cy /= static_cast<double>(cluster.size());
  for (const auto& [x, y] : cluster)
  {
    e.max_r = std::max(e.max_r, std::hypot(x - e.cx, y - e.cy));
    if (e.max_r > reject_above)
    {
      break;
    }
  }
  return e;
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__COSTMAP_CLUSTERS_HPP_
