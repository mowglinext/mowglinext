// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// Frozen scalar oracle from dev 6445b2c01. Keep independent of the optimized
// sampler: differential tests protect every cell/guard/threshold decision.
// It is a deliberate copy of the pre-optimisation production code. Do not
// edit it to follow obstacle_deviation.cpp; change it only when the intended
// collision-check behaviour changes, so a drift in either side fails the test.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

#include "mowgli_nav2_plugins/obstacle_deviation.hpp"
namespace footprint_reference
{
using mowgli_nav2_plugins::BoundaryGuard;
using mowgli_nav2_plugins::ObstacleDeviation;
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

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

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

bool cellBlocked(const nav2_costmap_2d::Costmap2D& local,
                 double x,
                 double y,
                 const ObstacleDeviation::BoundaryGuard& g,
                 unsigned char local_threshold)
{
  if (ObstacleDeviation::isObstacleCell(sampleCell(local, x, y), local_threshold))
  {
    return true;
  }
  return g.isLethalAt(x, y);
}

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

bool FootprintBlocked(const nav2_costmap_2d::Costmap2D& costmap,
                      const geometry_msgs::msg::PoseStamped& pose,
                      double center_dev,
                      const ObstacleDeviation::Footprint& footprint,
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
  ObstacleDeviation::Footprint world_poly;
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
      // Cost first: on an empty lawn it is 0 and the polygon test never runs.
      if (cellBlocked(costmap, cx, cy, guard, threshold) && pointInPolygon(world_poly, cx, cy))
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace footprint_reference
