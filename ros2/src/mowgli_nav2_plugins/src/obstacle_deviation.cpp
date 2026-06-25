// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins
{

namespace
{

/// Sample a single (x, y) world coordinate from the costmap. Returns
/// the cost, or 0 if the point is outside the costmap (treat as free —
/// the planner already validated overall reachability).
unsigned char sampleCell(const nav2_costmap_2d::Costmap2D& costmap, double x, double y)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap.worldToMap(x, y, mx, my))
  {
    return 0u;
  }
  return costmap.getCost(mx, my);
}

/// Yaw from a geometry_msgs Quaternion (ZYX convention, planar). Inlined to
/// avoid pulling in tf2_geometry_msgs (header-only template, no library).
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Apply a lateral offset `dev` (positive = left of heading) to `pose`.
/// Returns world-frame (x, y) of the offset point.
inline void offsetLateral(const geometry_msgs::msg::PoseStamped& pose,
                          double dev,
                          double& out_x,
                          double& out_y)
{
  const double yaw = yawFromQuaternion(pose.pose.orientation);
  // Left perpendicular = +pi/2 from heading.
  out_x = pose.pose.position.x + dev * std::cos(yaw + M_PI_2);
  out_y = pose.pose.position.y + dev * std::sin(yaw + M_PI_2);
}

/// Is the offset sample point (x, y) blocked? True if the LOCAL (obstacle)
/// costmap cell is lethal, OR — when a boundary guard is supplied — if the
/// point projected into the guard costmap's frame lands on a lethal cell
/// (out-of-zone). Used ONLY for lateral-offset deviation checks so the skirt
/// never leaves the mowing zone.
bool offsetBlocked(const nav2_costmap_2d::Costmap2D& local,
                   double x,
                   double y,
                   const ObstacleDeviation::BoundaryGuard& g)
{
  if (sampleCell(local, x, y) >= ObstacleDeviation::kLethalThreshold)
  {
    return true;
  }
  if (g.costmap != nullptr)
  {
    const double bx = g.tx + g.cos_yaw * x - g.sin_yaw * y;
    const double by = g.ty + g.sin_yaw * x + g.cos_yaw * y;
    if (sampleCell(*g.costmap, bx, by) >= ObstacleDeviation::kLethalThreshold)
    {
      return true;
    }
  }
  return false;
}

/// Sample the robot BODY — a lateral span [center_dev - half_width,
/// center_dev + half_width] perpendicular to `pose`'s heading — against the
/// costmap. Returns true if ANY sampled point is blocked (local lethal/inscribed
/// cell, or out-of-zone when `g.costmap != nullptr`). Sample spacing is kept
/// <= the costmap resolution so a thin obstacle cannot slip between samples.
/// `half_width <= 0` collapses to a single centerline sample at `center_dev`
/// (legacy behaviour). This is what makes detection / clearance respect the
/// chassis sweep instead of just the path point — the costmap inscribed-
/// inflation radius is far narrower than the real half-width here.
bool bodyBlocked(const nav2_costmap_2d::Costmap2D& local,
                 const geometry_msgs::msg::PoseStamped& pose,
                 double center_dev,
                 double half_width,
                 const ObstacleDeviation::BoundaryGuard& g)
{
  int n = 1;
  if (half_width > 0.0)
  {
    const double res = std::max(local.getResolution(), 0.01);
    n = std::max(3, static_cast<int>(std::ceil((2.0 * half_width) / res)) + 1);
  }
  for (int k = 0; k < n; ++k)
  {
    double dev = center_dev;
    if (half_width > 0.0)
    {
      const double frac = static_cast<double>(k) / static_cast<double>(n - 1);  // 0..1
      dev += -half_width + frac * 2.0 * half_width;
    }
    double x = 0.0;
    double y = 0.0;
    offsetLateral(pose, dev, x, y);
    if (offsetBlocked(local, x, y, g))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

int ObstacleDeviation::findFirstObstacleIndex(
    const nav2_costmap_2d::Costmap2D& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    std::size_t start_idx,
    int lookahead_count,
    double half_width)
{
  if (lookahead_count <= 0 || path.empty())
  {
    return -1;
  }
  const std::size_t end_idx =
      std::min(path.size(), start_idx + static_cast<std::size_t>(lookahead_count));
  for (std::size_t i = start_idx; i < end_idx; ++i)
  {
    // Detection scans the NOMINAL path body (no zone guard — the guard only
    // rejects skirting OUT of zone, which is meaningless for "is there an
    // obstacle ahead").
    if (bodyBlocked(costmap, path[i], 0.0, half_width, BoundaryGuard{}))
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

double ObstacleDeviation::chooseDeviationSide(const nav2_costmap_2d::Costmap2D& costmap,
                                              const geometry_msgs::msg::PoseStamped& obstacle_pose,
                                              double max_search,
                                              double step,
                                              const BoundaryGuard& guard,
                                              double half_width)
{
  if (step <= 0.0 || max_search <= 0.0)
  {
    return 0.0;
  }
  // Sweep both sides in lockstep, returning whichever direction first places
  // the whole BODY (±half_width) on clear, in-zone cells. Bias to the LEFT on
  // ties (avoids zigzag flicker when both sides are equally clear at the first
  // sample distance).
  for (double d = step; d <= max_search; d += step)
  {
    const bool left_clear = !bodyBlocked(costmap, obstacle_pose, d, half_width, guard);
    const bool right_clear = !bodyBlocked(costmap, obstacle_pose, -d, half_width, guard);

    if (left_clear)
    {
      return d;
    }
    if (right_clear)
    {
      return -d;
    }
  }
  return 0.0;
}

bool ObstacleDeviation::isPathClearWithDeviation(
    const nav2_costmap_2d::Costmap2D& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    std::size_t start_idx,
    int lookahead_count,
    double deviation,
    const BoundaryGuard& guard,
    double half_width)
{
  if (lookahead_count <= 0 || path.empty())
  {
    return true;
  }
  const std::size_t end_idx =
      std::min(path.size(), start_idx + static_cast<std::size_t>(lookahead_count));
  for (std::size_t i = start_idx; i < end_idx; ++i)
  {
    // Body must clear at the offset `deviation` — sample the full ±half_width
    // span, not just the offset centerline.
    if (bodyBlocked(costmap, path[i], deviation, half_width, guard))
    {
      return false;
    }
  }
  return true;
}

double ObstacleDeviation::growDeviationUntilClear(
    const nav2_costmap_2d::Costmap2D& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    std::size_t start_idx,
    int lookahead_count,
    double initial_deviation,
    double max_deviation,
    double step,
    const BoundaryGuard& guard,
    double half_width)
{
  if (step <= 0.0 || max_deviation <= 0.0)
  {
    return initial_deviation;
  }
  // Pick search sign: keep the existing side if non-zero, else default to
  // left (the caller is expected to seed via chooseDeviationSide first, so
  // this branch is only hit on edge cases like a fresh deviation request
  // with no obstacle pose available).
  const double sign = (initial_deviation == 0.0) ? 1.0 : ((initial_deviation > 0.0) ? 1.0 : -1.0);

  // Start from at least |initial| (already-active deviation), grow upward.
  double mag = std::max(std::abs(initial_deviation), step);
  while (mag <= max_deviation)
  {
    const double candidate = sign * mag;
    if (isPathClearWithDeviation(costmap, path, start_idx, lookahead_count, candidate, guard,
                                 half_width))
    {
      return candidate;
    }
    mag += step;
  }
  // Failed to find clearance within max_deviation; signal by returning a
  // value past the cap. Caller checks |result| > max_deviation.
  return sign * (max_deviation + step);
}

}  // namespace mowgli_nav2_plugins
