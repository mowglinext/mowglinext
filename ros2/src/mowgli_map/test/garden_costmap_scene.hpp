// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Synthetic, deterministic garden-sized global costmaps for
// test_costmap_clusters.cpp and bench/map_server_bench.cpp: what
// obstacle_tracker_node actually clusters. The global costmap spans the whole
// garden (+ map_server's 5 m padding) at 0.08 m; outside the recorded lawn it is
// lethal (100) with a mid-cost slack band (50) just outside the edge; inside are
// inflated obstacles (lethal core + decaying gradient) and isolated grass
// returns.

#ifndef MOWGLI_MAP_TEST__GARDEN_COSTMAP_SCENE_HPP_
#define MOWGLI_MAP_TEST__GARDEN_COSTMAP_SCENE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace garden_scene
{

struct CostmapScene
{
  double length_x;  // m, garden extent incl. padding
  double length_y;
  double resolution;
  int obstacles;
  int grass_returns;
  unsigned seed;
};

/// Wavy L-shaped lawn outline (a recorded perimeter is a few hundred points).
inline std::vector<std::pair<double, double>> lawn_outline(double hx, double hy, unsigned seed)
{
  const std::vector<std::pair<double, double>> corners = {
      {-hx, -hy}, {hx, -hy}, {hx, 0.2 * hy}, {0.3 * hx, 0.2 * hy}, {0.3 * hx, hy}, {-hx, hy}};
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> wobble(-0.15, 0.15);
  std::vector<std::pair<double, double>> ring;
  for (std::size_t i = 0; i < corners.size(); ++i)
  {
    const auto [ax, ay] = corners[i];
    const auto [bx, by] = corners[(i + 1) % corners.size()];
    const int steps = std::max(1, static_cast<int>(std::hypot(bx - ax, by - ay) / 0.4));
    for (int s = 0; s < steps; ++s)
    {
      const double t = static_cast<double>(s) / steps;
      ring.emplace_back(ax + t * (bx - ax) + wobble(rng), ay + t * (by - ay) + wobble(rng));
    }
  }
  return ring;
}

inline bool inside(const std::vector<std::pair<double, double>>& ring, double x, double y)
{
  bool in = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const auto [xi, yi] = ring[i];
    const auto [xj, yj] = ring[j];
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi))
      in = !in;
  }
  return in;
}

inline double edge_distance(const std::vector<std::pair<double, double>>& ring, double x, double y)
{
  double best = 1e18;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const auto [ax, ay] = ring[j];
    const auto [bx, by] = ring[i];
    const double dx = bx - ax, dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 > 0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / l2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(x - (ax + t * dx), y - (ay + t * dy)));
  }
  return best;
}

inline nav_msgs::msg::OccupancyGrid make_costmap(const CostmapScene& s)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = static_cast<float>(s.resolution);
  g.info.width = static_cast<uint32_t>(std::lround(s.length_x / s.resolution));
  g.info.height = static_cast<uint32_t>(std::lround(s.length_y / s.resolution));
  g.info.origin.position.x = -0.5 * s.length_x;
  g.info.origin.position.y = -0.5 * s.length_y;
  g.info.origin.orientation.w = 1.0;
  const int w = static_cast<int>(g.info.width);
  const int h = static_cast<int>(g.info.height);
  g.data.assign(static_cast<std::size_t>(w) * h, 0);
  const double res = g.info.resolution;
  const auto ring = lawn_outline(0.5 * s.length_x - 5.0, 0.5 * s.length_y - 5.0, s.seed);

  // Bounding box of the lawn: outside it every cell is plainly lethal.
  double minx = 1e18, maxx = -1e18, miny = 1e18, maxy = -1e18;
  for (const auto& [x, y] : ring)
  {
    minx = std::min(minx, x);
    maxx = std::max(maxx, x);
    miny = std::min(miny, y);
    maxy = std::max(maxy, y);
  }
  constexpr double kSlackBand = 0.6;
  for (int r = 0; r < h; ++r)
  {
    const double y = g.info.origin.position.y + (r + 0.5) * res;
    for (int c = 0; c < w; ++c)
    {
      const double x = g.info.origin.position.x + (c + 0.5) * res;
      int8_t& v = g.data[static_cast<std::size_t>(r) * w + c];
      if (x < minx - kSlackBand || x > maxx + kSlackBand || y < miny - kSlackBand ||
          y > maxy + kSlackBand)
      {
        v = 100;
      }
      else if (!inside(ring, x, y))
      {
        v = edge_distance(ring, x, y) <= kSlackBand ? 50 : 100;
      }
    }
  }

  std::mt19937 rng(s.seed * 7919U + 1U);
  std::uniform_real_distribution<double> ux(minx, maxx), uy(miny, maxy), ur(0.08, 0.40);
  for (int o = 0; o < s.obstacles; ++o)
  {
    double cx = 0.0, cy = 0.0;
    do
    {
      cx = ux(rng);
      cy = uy(rng);
    } while (!inside(ring, cx, cy));
    const double radius = ur(rng);
    constexpr double kInflation = 0.677;  // circumscribed radius + one cell
    const int reach = static_cast<int>(std::ceil((radius + kInflation) / res));
    const int cc = static_cast<int>((cx - g.info.origin.position.x) / res);
    const int cr = static_cast<int>((cy - g.info.origin.position.y) / res);
    for (int r = std::max(0, cr - reach); r <= std::min(h - 1, cr + reach); ++r)
    {
      for (int c = std::max(0, cc - reach); c <= std::min(w - 1, cc + reach); ++c)
      {
        const double d = std::hypot(g.info.origin.position.x + (c + 0.5) * res - cx,
                                    g.info.origin.position.y + (r + 0.5) * res - cy) -
                         radius;
        int8_t cost = 0;
        if (d <= 0.0)
          cost = 100;
        else if (d <= 0.17)
          cost = 99;  // inscribed
        else if (d <= kInflation)
          cost = static_cast<int8_t>(std::lround(98.0 * std::exp(-7.0 * (d - 0.17))));
        int8_t& v = g.data[static_cast<std::size_t>(r) * w + c];
        v = std::max(v, cost);
      }
    }
  }
  std::uniform_int_distribution<int> uc(0, w - 1), urow(0, h - 1);
  for (int k = 0; k < s.grass_returns; ++k)
  {
    const int c = uc(rng);
    const int r = urow(rng);
    g.data[static_cast<std::size_t>(r) * w + c] = static_cast<int8_t>(50 + (k % 51));
    if (k % 3 == 0 && c + 1 < w)
      g.data[static_cast<std::size_t>(r) * w + c + 1] = 100;
  }
  return g;
}

}  // namespace garden_scene

#endif  // MOWGLI_MAP_TEST__GARDEN_COSTMAP_SCENE_HPP_
