// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// The obstacle tracker's clustering stage (mowgli_map/costmap_clusters.hpp)
// against the code it replaced (legacy_costmap_clusters.hpp):
//   * without the early drop, the flood fill returns the SAME clusters, in the
//     same order, with the same points in the same order (exact doubles);
//   * with it, the clusters associate_clusters() goes on with — those inside
//     [min_radius, max_radius] — are the same, with the same centroid and
//     radius, including shapes sized right at the 2 x max_radius span bound and
//     coordinates so large that neighbouring cell centres round together.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "garden_costmap_scene.hpp"
#include "legacy_costmap_clusters.hpp"
#include "mowgli_map/costmap_clusters.hpp"
#include <gtest/gtest.h>

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

nav_msgs::msg::OccupancyGrid empty_grid(int w, int h, double res = 0.08)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = static_cast<uint32_t>(w);
  g.info.height = static_cast<uint32_t>(h);
  g.info.resolution = static_cast<float>(res);
  g.info.origin.position.x = -3.17;
  g.info.origin.position.y = 11.03;
  g.data.assign(static_cast<std::size_t>(w) * h, 0);
  return g;
}

void set(nav_msgs::msg::OccupancyGrid& g, int x, int y, int8_t v)
{
  if (x >= 0 && y >= 0 && x < static_cast<int>(g.info.width) && y < static_cast<int>(g.info.height))
    g.data[static_cast<std::size_t>(y) * g.info.width + x] = v;
}

nav_msgs::msg::OccupancyGrid random_grid(int w, int h, double density, unsigned seed)
{
  auto g = empty_grid(w, h);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::uniform_int_distribution<int> cost(-1, 100);
  for (auto& v : g.data)
  {
    v = u(rng) < density ? static_cast<int8_t>(50 + rng() % 51)
                         : static_cast<int8_t>(cost(rng) % 50);
  }
  return g;
}

/// Lines, discs and rings whose extent straddles 2 x max_radius.
nav_msgs::msg::OccupancyGrid boundary_shapes(double max_radius, double res)
{
  const int span = static_cast<int>(std::ceil(2.0 * max_radius / res));
  auto g = empty_grid(12 * span + 40, 12 * span + 40, res);
  int ox = 2;
  int oy = 2;
  for (int len = span - 3; len <= span + 4; ++len)
  {
    for (int x = 0; x <= len; ++x)
      set(g, ox + x, oy, 100);  // horizontal line
    for (int y = 0; y <= len; ++y)
      set(g, ox + len + 3, oy + y, 77);  // vertical line
    for (int d = 0; d <= len; ++d)
    {
      set(g, ox + d, oy + 3 + d, 99);  // staircase diagonal (4-connected)
      set(g, ox + d + 1, oy + 3 + d, 99);
    }
    ox += len + 6;
    if (ox > static_cast<int>(g.info.width) - span - 8)
    {
      ox = 2;
      oy += span + 8;
    }
  }
  // Filled discs and rings with radii around max_radius.
  oy += span + 8;
  ox = span / 2 + 3;
  for (double r = max_radius - 2 * res; r <= max_radius + 2 * res; r += 0.5 * res)
  {
    const int rc = static_cast<int>(std::ceil(r / res)) + 1;
    for (int y = -rc; y <= rc; ++y)
    {
      for (int x = -rc; x <= rc; ++x)
      {
        const double d = std::hypot(x * res, y * res);
        if (d <= r)
          set(g, ox + x, oy + y, 100);
        if (std::abs(d - r) < 0.75 * res)
          set(g, ox + x, oy + y + 2 * rc + 4, 60);
      }
    }
    ox += 2 * rc + 4;
  }
  return g;
}

using Accepted = std::vector<std::vector<double>>;  // {cx, cy, max_r}

Accepted legacy_stage(const nav_msgs::msg::OccupancyGrid& g,
                      int min_points,
                      double min_r,
                      double max_r)
{
  Accepted out;
  for (const auto& c : legacy::cluster_costmap(g, min_points))
  {
    const auto e = legacy::cluster_extent(c);
    if (e.max_r < min_r || e.max_r > max_r)
      continue;
    out.push_back({e.cx, e.cy, e.max_r});
  }
  return out;
}

Accepted new_stage(const nav_msgs::msg::OccupancyGrid& g,
                   int min_points,
                   double min_r,
                   double max_r)
{
  Accepted out;
  for (const auto& c : mowgli_map::cluster_costmap_cells(g, 50, min_points, max_r))
  {
    const auto e = mowgli_map::cluster_extent(c, max_r);
    if (e.max_r < min_r || e.max_r > max_r)
      continue;
    out.push_back({e.cx, e.cy, e.max_r});
  }
  return out;
}

void expect_same_flood_fill(const nav_msgs::msg::OccupancyGrid& g, int min_points)
{
  const auto expected = legacy::cluster_costmap(g, min_points);
  const auto actual = mowgli_map::cluster_costmap_cells(g, 50, min_points, kInf);
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    ASSERT_EQ(actual[i], expected[i]) << "cluster " << i;
  }
}

TEST(CostmapClusters, FloodFillMatchesTheOriginalOnGardenScenes)
{
  for (const auto& scene : {garden_scene::CostmapScene{30.0, 25.0, 0.08, 8, 150, 1},
                            garden_scene::CostmapScene{22.0, 41.0, 0.08, 20, 900, 5},
                            garden_scene::CostmapScene{18.0, 18.0, 0.05, 12, 400, 9}})
  {
    const auto g = garden_scene::make_costmap(scene);
    for (int min_points : {1, 5})
    {
      SCOPED_TRACE(std::to_string(g.info.width) + "x" + std::to_string(g.info.height) +
                   " min_points " + std::to_string(min_points));
      expect_same_flood_fill(g, min_points);
    }
  }
}

TEST(CostmapClusters, FloodFillMatchesTheOriginalOnRandomAndDegenerateGrids)
{
  unsigned seed = 1;
  for (const auto& [w, h] :
       std::vector<std::pair<int, int>>{{1, 1}, {1, 37}, {41, 1}, {2, 2}, {63, 17}, {120, 90}})
  {
    for (double density : {0.0, 0.3, 0.55, 0.8, 1.0})
    {
      SCOPED_TRACE(std::to_string(w) + "x" + std::to_string(h) + " density " +
                   std::to_string(density));
      const auto g = random_grid(w, h, density, seed++);
      expect_same_flood_fill(g, 1);
      expect_same_flood_fill(g, 5);
    }
  }
  EXPECT_TRUE(mowgli_map::cluster_costmap_cells(empty_grid(0, 0), 50, 1, kInf).empty());
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalOnGardenScenes)
{
  for (const auto& scene : {garden_scene::CostmapScene{30.0, 25.0, 0.08, 8, 150, 1},
                            garden_scene::CostmapScene{60.0, 50.0, 0.08, 30, 600, 2},
                            garden_scene::CostmapScene{18.0, 18.0, 0.05, 12, 400, 9}})
  {
    const auto g = garden_scene::make_costmap(scene);
    for (double max_r : {0.5, 1.5, 4.0})
    {
      SCOPED_TRACE("max_r " + std::to_string(max_r));
      const auto expected = legacy_stage(g, 5, 0.08, max_r);
      EXPECT_EQ(new_stage(g, 5, 0.08, max_r), expected);
      EXPECT_FALSE(expected.empty());
    }
  }
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalAtTheSpanBound)
{
  for (double res : {0.05, 0.08})
  {
    for (double max_r : {0.24, 0.5, 1.5})
    {
      SCOPED_TRACE("res " + std::to_string(res) + " max_r " + std::to_string(max_r));
      const auto g = boundary_shapes(max_r, res);
      const auto expected = legacy_stage(g, 1, 0.0, max_r);
      EXPECT_EQ(new_stage(g, 1, 0.0, max_r), expected);
      // The scene really straddles the bound: some shapes pass, some do not.
      EXPECT_FALSE(expected.empty());
      EXPECT_LT(expected.size(), legacy::cluster_costmap(g, 1).size());
    }
  }
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalOnRandomGrids)
{
  unsigned seed = 100;
  for (double density : {0.3, 0.5, 0.6})
  {
    const auto g = random_grid(150, 110, density, seed++);
    for (double max_r : {0.1, 0.25, 0.6, 2.0})
    {
      SCOPED_TRACE("density " + std::to_string(density) + " max_r " + std::to_string(max_r));
      EXPECT_EQ(new_stage(g, 1, 0.0, max_r), legacy_stage(g, 1, 0.0, max_r));
      EXPECT_EQ(new_stage(g, 5, 0.08, max_r), legacy_stage(g, 5, 0.08, max_r));
    }
  }
}

TEST(CostmapClusters, EarlyDropSkipsTheOutsideComponent)
{
  // The lethal "outside the lawn" region is one component far wider than any
  // obstacle: the original returns it, the clustering stage must not.
  const auto g = garden_scene::make_costmap({30.0, 25.0, 0.08, 8, 150, 1});
  const auto all = legacy::cluster_costmap(g, 5);
  const auto kept = mowgli_map::cluster_costmap_cells(g, 50, 5, 1.5);
  std::size_t largest = 0;
  for (const auto& c : all)
    largest = std::max(largest, c.size());
  EXPECT_GT(largest, 10000U);
  EXPECT_LT(kept.size(), all.size());
  const double max_span = 2.0 * 1.5 + g.info.resolution;
  for (const auto& c : kept)
  {
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (const auto& [x, y] : c)
    {
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
    EXPECT_LE(max_x - min_x, max_span);
    EXPECT_LE(max_y - min_y, max_span);
  }
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalAtHugeCoordinates)
{
  // Far from the origin, neighbouring cell centres round to the SAME double.
  // The old radius is computed from those stored doubles, so a component many
  // cells wide can still pass the radius filter. The early drop must judge the
  // stored coordinates, not the cell count (counterexample from review: 42x4
  // cells at x = 2^60 have radius 0.12 m in the old code).
  for (double origin : {std::ldexp(1.0, 60), std::ldexp(1.0, 40), 1.0e6, -1.0e6})
  {
    SCOPED_TRACE("origin " + std::to_string(origin));
    auto g = empty_grid(42, 4, 0.08);
    g.info.origin.position.x = origin;
    std::fill(g.data.begin(), g.data.end(), 100);
    for (double max_r : {0.12, 1.5, 3.0})
    {
      const auto expected = legacy_stage(g, 5, 0.08, max_r);
      EXPECT_EQ(new_stage(g, 5, 0.08, max_r), expected);
    }
  }
  auto g = empty_grid(42, 4, 0.08);
  g.info.origin.position.x = std::ldexp(1.0, 60);
  std::fill(g.data.begin(), g.data.end(), 100);
  EXPECT_FALSE(legacy_stage(g, 5, 0.08, 1.5).empty());  // the case really is accepted
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalForDegenerateRadii)
{
  const auto g = random_grid(90, 70, 0.55, 3);
  for (double max_r : {-1.0, -0.01, 0.0, std::numeric_limits<double>::quiet_NaN(), kInf})
  {
    SCOPED_TRACE("max_r " + std::to_string(max_r));
    EXPECT_EQ(new_stage(g, 1, 0.0, max_r), legacy_stage(g, 1, 0.0, max_r));
    EXPECT_EQ(new_stage(g, 1, 0.5, max_r), legacy_stage(g, 1, 0.5, max_r));  // min > max
  }
}

TEST(CostmapClusters, AcceptedClustersMatchTheOriginalForNonFiniteOrigins)
{
  // A NaN/infinite origin makes every old radius NaN, which std::max skips, so
  // the old code accepted the cluster with max_r = 0 however wide it was (the
  // early drop must stand down; counterexample from review). NaN != NaN, so
  // compare bit patterns.
  const auto same_bits = [](const Accepted& a, const Accepted& b)
  {
    if (a.size() != b.size())
      return false;
    for (std::size_t i = 0; i < a.size(); ++i)
    {
      for (std::size_t k = 0; k < 3; ++k)
      {
        if (!(a[i][k] == b[i][k] || (std::isnan(a[i][k]) && std::isnan(b[i][k]))))
          return false;
      }
    }
    return true;
  };
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  for (const auto& [x, y] :
       std::vector<std::pair<double, double>>{{0.0, kNaN}, {kNaN, 0.0}, {0.0, kInf}, {-kInf, 0.0}})
  {
    SCOPED_TRACE("origin " + std::to_string(x) + ", " + std::to_string(y));
    auto line = empty_grid(5, 1, 1.0);
    line.info.origin.position.x = x;
    line.info.origin.position.y = y;
    std::fill(line.data.begin(), line.data.end(), 100);
    const auto expected = legacy_stage(line, 3, 0.0, 1.0);
    EXPECT_FALSE(expected.empty());  // the case really is accepted
    EXPECT_TRUE(same_bits(new_stage(line, 3, 0.0, 1.0), expected));

    auto g = random_grid(90, 70, 0.55, 5);
    g.info.origin.position.x = x;
    g.info.origin.position.y = y;
    EXPECT_TRUE(same_bits(new_stage(g, 1, 0.0, 0.3), legacy_stage(g, 1, 0.0, 0.3)));
  }
}

TEST(CostmapClusters, NegativeDimensionsYieldNoClusters)
{
  // width/height above INT_MAX turn negative in the int the tracker has always
  // used. Both negative: the old code returned nothing; one negative: it threw
  // from std::vector<bool>. Neither may allocate a mask here.
  for (const auto& [w, h] : std::vector<std::pair<uint32_t, uint32_t>>{{4294967293U, 4294967295U},
                                                                       {4294967293U, 1U},
                                                                       {1U, 4294967295U}})
  {
    nav_msgs::msg::OccupancyGrid g;
    g.info.width = w;
    g.info.height = h;
    g.info.resolution = 0.08F;
    std::vector<mowgli_map::CellCluster> clusters;
    EXPECT_NO_THROW(clusters = mowgli_map::cluster_costmap_cells(g, 50, 1, 1.0));
    EXPECT_TRUE(clusters.empty());
  }
}

TEST(CostmapClusters, ExtentStopsOnlyAboveTheRejectBound)
{
  const auto g = random_grid(120, 90, 0.55, 7);
  for (const auto& c : legacy::cluster_costmap(g, 1))
  {
    const auto full = legacy::cluster_extent(c);
    for (double bound : {0.05, 0.3, 1.0, kInf})
    {
      const auto e = mowgli_map::cluster_extent(c, bound);
      EXPECT_EQ(e.cx, full.cx);
      EXPECT_EQ(e.cy, full.cy);
      EXPECT_EQ(e.max_r > bound, full.max_r > bound);
      if (full.max_r <= bound)
      {
        EXPECT_EQ(e.max_r, full.max_r);
      }
    }
  }
}

}  // namespace
