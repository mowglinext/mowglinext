// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Offline production solver benchmark. No ROS graph, simulation or actuation.
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include <time.h>

namespace mn = mowgli_nav2_plugins;
using Pose = geometry_msgs::msg::PoseStamped;

Pose PoseAt(double x, double y, double yaw)
{
  Pose p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}

std::uint64_t CpuNs()
{
  timespec t{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  return static_cast<std::uint64_t>(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}

int main(int argc, char** argv)
{
  const std::string scene = argc > 1 ? argv[1] : "clear";
  const unsigned seed = argc > 2 ? std::stoul(argv[2]) : 42;
  const int repeats = argc > 3 ? std::stoi(argv[3]) : 20;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const double res = scene == "fine" ? 0.025 : 0.05;
  nav2_costmap_2d::Costmap2D map(240, 240, res, -3.0, -3.0, 0);
  nav2_costmap_2d::Costmap2D zone(240, 240, res, -3.0, -3.0, 0);
  const bool holdout = seed >= 10000;
  const double angle = holdout ? -0.7 + unit(rng) * 1.4 : 0.0;
  const double ca = std::cos(angle), sa = std::sin(angle);
  const double wall_x = scene == "near_wall" ? 0.8 : 1.8 + 0.2 * unit(rng);
  for (unsigned y = 0; y < map.getSizeInCellsY(); ++y)
  {
    for (unsigned x = 0; x < map.getSizeInCellsX(); ++x)
    {
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      const double along = wx * ca + wy * sa;
      const double lateral = -wx * sa + wy * ca;
      bool blocked = false;
      if (scene == "solid" || scene == "unknown")
        blocked = true;
      else if (scene == "far_wall" || scene == "near_wall")
        blocked = along >= wall_x && along < wall_x + 0.15;
      else if (scene == "post" || scene == "zone")
        blocked = std::hypot(along - 1.6, lateral) < 0.13;
      else if (scene == "clutter")
        blocked = unit(rng) < 0.002;
      if (blocked)
        map.setCost(x, y, scene == "unknown" ? 255 : 254);
      if (scene == "inflated")
        map.setCost(x, y, 253);
      if (scene == "zone" && lateral < -0.35)
        zone.setCost(x, y, 254);
    }
  }
  mn::ObstacleDeviation::Footprint body;
  for (const auto& xy : std::vector<std::pair<double, double>>{
           {0.54, 0.335}, {0.54, -0.335}, {-0.18, -0.335}, {-0.18, 0.335}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    body.push_back(p);
  }
  std::vector<Pose> poses;
  for (int k = -6; k <= 50; ++k)
  {
    const double along = k * 0.05;
    poses.push_back(PoseAt(along * ca, along * sa, angle));
  }
  mn::BoundaryGuard guard;
  if (scene == "zone")
    guard.costmap = &zone;
  mn::LatticeSolverCfg cfg;
  cfg.lattice.max_offset = 1.0;
  cfg.station_spacing_m = 0.05;
  cfg.lead_m = 0.30;
  cfg.reaction_m = 0.5;
  cfg.min_horizon_m = 1.0;
  mn::LatticeSolution result;
  std::uint64_t checksum = 1469598103934665603ULL;
  auto hash = [&](std::uint64_t v)
  {
    checksum = (checksum ^ v) * 1099511628211ULL;
  };
  // Warm-up is excluded, then each timed call includes construction and caches.
  {
    mn::LatticeSolver warm(map, guard, body, poses, 6, false, cfg);
    warm.Solve(0.0, 0, 0);
  }
  const auto wall_start = std::chrono::steady_clock::now();
  const auto cpu_start = CpuNs();
  for (int i = 0; i < repeats; ++i)
  {
    mn::LatticeSolver solver(map, guard, body, poses, 6, false, cfg);
    result = solver.Solve(0.0, 0, 0);
    hash(result.plan.feasible);
    hash(result.level);
    hash(result.planned_stations);
    hash(std::bit_cast<std::uint64_t>(result.plan.cost));
    for (double d : result.plan.offsets)
      hash(std::bit_cast<std::uint64_t>(d));
  }
  const auto cpu_ns = CpuNs() - cpu_start;
  const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - wall_start)
                           .count();
  std::cout << "{\"scene\":\"" << scene << "\",\"seed\":" << seed << ",\"repeats\":" << repeats
            << ",\"cpu_ns\":" << cpu_ns << ",\"wall_ns\":" << wall_ns << ",\"hash\":" << checksum
            << ",\"feasible\":" << result.plan.feasible << ",\"level\":" << result.level
            << ",\"stations\":" << result.planned_stations
            << ",\"cost_bits\":" << std::bit_cast<std::uint64_t>(result.plan.cost)
            << ",\"offset_bits\":[";
  for (std::size_t i = 0; i < result.plan.offsets.size(); ++i)
  {
    if (i)
      std::cout << ',';
    std::cout << std::bit_cast<std::uint64_t>(result.plan.offsets[i]);
  }
  std::cout << "]}\n";
}
