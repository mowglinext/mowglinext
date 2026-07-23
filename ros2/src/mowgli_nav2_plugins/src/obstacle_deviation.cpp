// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
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

/// Is the sample point (x, y) blocked? True if the LOCAL (obstacle) costmap
/// cell is at/above `local_threshold`, OR — when a boundary guard is supplied —
/// if the point projected into the guard costmap's frame lands on a lethal cell
/// (out-of-zone). The guard is always checked at kLethalThreshold (zone cells
/// are stamped LETHAL). Used for both the lateral-offset line samples and the
/// footprint-interior samples so the skirt never leaves the mowing zone.
bool cellBlocked(const nav2_costmap_2d::Costmap2D& local,
                 double x,
                 double y,
                 const ObstacleDeviation::BoundaryGuard& g,
                 unsigned char local_threshold)
{
  if (sampleCell(local, x, y) >= local_threshold)
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

/// Sample the robot BODY as a lateral span [center_dev - half_width,
/// center_dev + half_width] perpendicular to `pose`'s heading — the FALLBACK
/// line model used when no footprint polygon is provided. Returns true if ANY
/// sampled point is blocked (local >= kLethalThreshold — i.e. lethal OR
/// inscribed — or out-of-zone when `g.costmap != nullptr`). Sample spacing is
/// kept <= the costmap resolution so a thin obstacle cannot slip between
/// samples. `half_width <= 0` collapses to a single centerline sample at
/// `center_dev` (legacy behaviour). Relies on the inscribed-inflation band as a
/// body-width proxy — the footprint model below is the explicit-shape
/// replacement.
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
    if (cellBlocked(local, x, y, g, ObstacleDeviation::kLethalThreshold))
    {
      return true;
    }
  }
  return false;
}

/// Even-odd (ray-casting) point-in-polygon test for a base-frame polygon.
/// `poly` is the footprint vertices; (px, py) the query point in the same
/// frame. Robust for the convex chassis rectangle and any simple polygon.
bool pointInPolygon(const ObstacleDeviation::Footprint& poly, double px, double py)
{
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double yi = poly[i].y;
    const double yj = poly[j].y;
    if ((yi > py) != (yj > py))
    {
      const double denom = (yj - yi) != 0.0 ? (yj - yi) : 1e-12;
      const double x_cross = (poly[j].x - poly[i].x) * (py - yi) / denom + poly[i].x;
      if (px < x_cross)
      {
        inside = !inside;
      }
    }
  }
  return inside;
}

/// Dispatch: sample the robot body at `pose` offset laterally by `center_dev`.
/// Prefers the explicit footprint polygon (true-lethal threshold) when
/// `footprint` is non-empty; otherwise falls back to the ±half_width line model
/// (lethal-or-inscribed threshold). Keeps every public helper on one code path.
bool regionBlocked(const nav2_costmap_2d::Costmap2D& local,
                   const geometry_msgs::msg::PoseStamped& pose,
                   double center_dev,
                   double half_width,
                   const ObstacleDeviation::Footprint& footprint,
                   const ObstacleDeviation::BoundaryGuard& g)
{
  if (!footprint.empty())
  {
    return ObstacleDeviation::footprintBlocked(
        local, pose, center_dev, footprint, g, ObstacleDeviation::kLethalOnlyThreshold);
  }
  return bodyBlocked(local, pose, center_dev, half_width, g);
}

}  // namespace

bool ObstacleDeviation::footprintBlocked(const nav2_costmap_2d::Costmap2D& costmap,
                                         const geometry_msgs::msg::PoseStamped& pose,
                                         double center_dev,
                                         const Footprint& footprint,
                                         const BoundaryGuard& guard,
                                         unsigned char threshold)
{
  if (footprint.size() < 3)
  {
    return false;  // Not a rasterisable polygon — nothing to sample.
  }

  // Candidate origin: pose position shifted laterally (left of heading) by
  // center_dev. The footprint is rotated by the pose heading about that origin.
  const double yaw = yawFromQuaternion(pose.pose.orientation);
  double ox = 0.0;
  double oy = 0.0;
  offsetLateral(pose, center_dev, ox, oy);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // Transform footprint vertices into the costmap (world) frame and take the
  // bounding box.
  Footprint world_poly;
  world_poly.reserve(footprint.size());
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& v : footprint)
  {
    geometry_msgs::msg::Point w;
    w.x = ox + c * v.x - s * v.y;
    w.y = oy + s * v.x + c * v.y;
    world_poly.push_back(w);
    min_x = std::min(min_x, w.x);
    min_y = std::min(min_y, w.y);
    max_x = std::max(max_x, w.x);
    max_y = std::max(max_y, w.y);
  }

  // Rasterise the interior over the ACTUAL costmap cells in the bounding box
  // (cell centres, aligned to the grid) so no lethal cell inside the chassis is
  // skipped — a bbox-relative grid at resolution spacing can straddle cells and
  // miss a thin obstacle grazing the chassis edge. First-hit early-out.
  // Clamp the bounding box into the costmap; if it lies fully outside, nothing
  // to sample. worldToMapEnforceBounds keeps partially-off boxes in range.
  int mx0 = 0;
  int my0 = 0;
  int mx1 = 0;
  int my1 = 0;
  costmap.worldToMapEnforceBounds(min_x, min_y, mx0, my0);
  costmap.worldToMapEnforceBounds(max_x, max_y, mx1, my1);
  for (int my = my0; my <= my1; ++my)
  {
    for (int mx = mx0; mx <= mx1; ++mx)
    {
      double cx = 0.0;
      double cy = 0.0;
      costmap.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cx, cy);
      if (pointInPolygon(world_poly, cx, cy) && cellBlocked(costmap, cx, cy, guard, threshold))
      {
        return true;
      }
    }
  }
  return false;
}

ObstacleDeviation::Footprint ObstacleDeviation::expandFootprintLateral(const Footprint& footprint,
                                                                       double margin)
{
  if (margin <= 0.0 || footprint.empty())
  {
    return footprint;
  }
  // Centroid y — vertices above it push +margin, below push -margin, widening
  // the polygon laterally without touching its longitudinal (x) extent.
  double cy = 0.0;
  for (const auto& v : footprint)
  {
    cy += v.y;
  }
  cy /= static_cast<double>(footprint.size());

  Footprint out;
  out.reserve(footprint.size());
  for (const auto& v : footprint)
  {
    geometry_msgs::msg::Point p = v;
    p.y += (v.y >= cy) ? margin : -margin;
    out.push_back(p);
  }
  return out;
}

ObstacleDeviation::Footprint ObstacleDeviation::clipFootprintFront(const Footprint& footprint,
                                                                   double front_length_m)
{
  if (front_length_m <= 0.0 || footprint.empty())
  {
    return footprint;
  }
  double max_x = std::numeric_limits<double>::lowest();
  double min_x = std::numeric_limits<double>::max();
  for (const auto& v : footprint)
  {
    max_x = std::max(max_x, v.x);
    min_x = std::min(min_x, v.x);
  }
  // Requested front length covers the whole body — nothing to clip.
  if (front_length_m >= (max_x - min_x))
  {
    return footprint;
  }
  const double cut = max_x - front_length_m;
  Footprint out;
  out.reserve(footprint.size());
  for (const auto& v : footprint)
  {
    geometry_msgs::msg::Point p = v;
    p.x = std::max(p.x, cut);  // project rear vertices onto the cut plane
    out.push_back(p);
  }
  return out;
}

bool ObstacleDeviation::hasClearExit(const nav2_costmap_2d::Costmap2D& costmap,
                                     const std::vector<geometry_msgs::msg::PoseStamped>& path,
                                     std::size_t start_idx,
                                     int lookahead_count,
                                     double half_width,
                                     const Footprint& footprint)
{
  if (lookahead_count <= 0 || path.empty())
  {
    return true;  // nothing ahead to trap us
  }
  const std::size_t end_idx =
      std::min(path.size(), start_idx + static_cast<std::size_t>(lookahead_count));
  bool obstacle_seen = false;
  for (std::size_t i = start_idx; i < end_idx; ++i)
  {
    // Nominal (zero-deviation) body sample — no zone guard (see doc comment).
    const bool blocked = regionBlocked(costmap, path[i], 0.0, half_width, footprint, BoundaryGuard{});
    if (blocked)
    {
      obstacle_seen = true;
      continue;
    }
    if (obstacle_seen)
    {
      return true;  // clear pose past the obstacle → the far edge is in view
    }
  }
  // No obstacle at all → trivially has an exit. Obstacle that never reopened
  // within the window → no visible far edge → refuse to skirt into it.
  return !obstacle_seen;
}

int ObstacleDeviation::findFirstObstacleIndex(
    const nav2_costmap_2d::Costmap2D& costmap,
    const std::vector<geometry_msgs::msg::PoseStamped>& path,
    std::size_t start_idx,
    int lookahead_count,
    double half_width,
    const Footprint& footprint)
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
    if (regionBlocked(costmap, path[i], 0.0, half_width, footprint, BoundaryGuard{}))
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
                                              double half_width,
                                              const Footprint& footprint)
{
  if (step <= 0.0 || max_search <= 0.0)
  {
    return 0.0;
  }
  // Sweep both sides in lockstep, returning whichever direction first places
  // the whole BODY (footprint, else ±half_width) on clear, in-zone cells. Bias
  // to the LEFT on ties (avoids zigzag flicker when both sides are equally clear
  // at the first sample distance).
  for (double d = step; d <= max_search; d += step)
  {
    const bool left_clear = !regionBlocked(costmap, obstacle_pose, d, half_width, footprint, guard);
    const bool right_clear =
        !regionBlocked(costmap, obstacle_pose, -d, half_width, footprint, guard);

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
    double half_width,
    const Footprint& footprint)
{
  if (lookahead_count <= 0 || path.empty())
  {
    return true;
  }
  const std::size_t end_idx =
      std::min(path.size(), start_idx + static_cast<std::size_t>(lookahead_count));
  for (std::size_t i = start_idx; i < end_idx; ++i)
  {
    // Body must clear at the offset `deviation` — sample the full footprint (or
    // the ±half_width span), not just the offset centerline.
    if (regionBlocked(costmap, path[i], deviation, half_width, footprint, guard))
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
    double half_width,
    const Footprint& footprint)
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
    if (isPathClearWithDeviation(
            costmap, path, start_idx, lookahead_count, candidate, guard, half_width, footprint))
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
