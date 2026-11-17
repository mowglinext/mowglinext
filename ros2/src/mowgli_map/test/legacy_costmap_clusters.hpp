// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Reference for test_costmap_clusters.cpp and bench/: the flood fill that
// ObstacleTrackerNode::process_costmap() ran inline before it moved to
// mowgli_map/costmap_clusters.hpp, copied verbatim (only the function wrapper
// is new). Do not "fix" it — it is the definition the fast version must match.

#ifndef MOWGLI_MAP_TEST__LEGACY_COSTMAP_CLUSTERS_HPP_
#define MOWGLI_MAP_TEST__LEGACY_COSTMAP_CLUSTERS_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace legacy
{

inline std::vector<std::vector<std::pair<double, double>>> cluster_costmap(
    const nav_msgs::msg::OccupancyGrid& msg, int min_cluster_points_)
{
  const int w = static_cast<int>(msg.info.width);
  const int h = static_cast<int>(msg.info.height);
  const double res = msg.info.resolution;
  const double ox = msg.info.origin.position.x;
  const double oy = msg.info.origin.position.y;

  std::vector<std::vector<std::pair<double, double>>> clusters;
  if (w == 0 || h == 0)
    return clusters;

  constexpr int8_t OBSTACLE_COST = 50;
  std::vector<bool> visited(w * h, false);

  // Flood-fill to find connected components of obstacle cells
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const int idx = y * w + x;
      if (visited[idx] || msg.data[idx] < OBSTACLE_COST)
        continue;

      // BFS flood-fill
      std::vector<std::pair<double, double>> cluster;
      std::vector<std::pair<int, int>> queue;
      queue.emplace_back(x, y);
      visited[idx] = true;

      while (!queue.empty())
      {
        auto [cx, cy] = queue.back();
        queue.pop_back();

        // Convert cell to map frame
        double mx = ox + (cx + 0.5) * res;
        double my = oy + (cy + 0.5) * res;
        cluster.emplace_back(mx, my);

        // 4-connected neighbors
        for (auto [dx, dy] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
        {
          int nx = cx + dx, ny = cy + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h)
          {
            int nidx = ny * w + nx;
            if (!visited[nidx] && msg.data[nidx] >= OBSTACLE_COST)
            {
              visited[nidx] = true;
              queue.emplace_back(nx, ny);
            }
          }
        }
      }

      if (static_cast<int>(cluster.size()) >= min_cluster_points_)
      {
        clusters.push_back(std::move(cluster));
      }
    }
  }
  return clusters;
}

/// associate_clusters()'s centroid + bounding-radius computation before it
/// moved to mowgli_map::cluster_extent, verbatim. Returns {cx, cy, max_r}.
struct Extent
{
  double cx;
  double cy;
  double max_r;
};
inline Extent cluster_extent(const std::vector<std::pair<double, double>>& cluster)
{
  double cx = 0.0;
  double cy = 0.0;
  for (const auto& [x, y] : cluster)
  {
    cx += x;
    cy += y;
  }
  cx /= static_cast<double>(cluster.size());
  cy /= static_cast<double>(cluster.size());

  double max_r = 0.0;
  for (const auto& [x, y] : cluster)
  {
    max_r = std::max(max_r, std::hypot(x - cx, y - cy));
  }
  return {cx, cy, max_r};
}

}  // namespace legacy

#endif  // MOWGLI_MAP_TEST__LEGACY_COSTMAP_CLUSTERS_HPP_
