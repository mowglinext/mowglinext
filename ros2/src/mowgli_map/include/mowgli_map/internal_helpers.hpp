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

// Internal-only helpers shared between map_server_node's translation
// units (costmap_filters.cpp, progress_tracker.cpp). Not part of the
// public API of mowgli_map — do not include from outside the package.

#ifndef MOWGLI_MAP__INTERNAL_HELPERS_HPP_
#define MOWGLI_MAP__INTERNAL_HELPERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace mowgli_map
{

/// Two promoted obstacles whose centroids are within this many metres are
/// treated as the SAME keepout. Chosen well below the obstacle-tracker's
/// association_dist (0.5 m) and the default obstacle inflation (~0.15 m) so
/// two genuinely distinct nearby obstacles are never collapsed, yet a
/// re-promote or YAML reload of an identical polygon (centroid delta ≈ 0) is
/// deduped. See apply_promoted_obstacle() and the area-load paths.
constexpr double kObstacleDedupEpsilonM = 0.10;

/// Centroid (average vertex) of a polygon, in the polygon's own frame.
inline geometry_msgs::msg::Point32 polygon_centroid(const geometry_msgs::msg::Polygon& poly)
{
  geometry_msgs::msg::Point32 c;
  const auto& pts = poly.points;
  if (pts.empty())
  {
    return c;
  }
  double sx = 0.0;
  double sy = 0.0;
  for (const auto& p : pts)
  {
    sx += static_cast<double>(p.x);
    sy += static_cast<double>(p.y);
  }
  c.x = static_cast<float>(sx / static_cast<double>(pts.size()));
  c.y = static_cast<float>(sy / static_cast<double>(pts.size()));
  return c;
}

/// True when `candidate`'s centroid lies within `eps` metres of any existing
/// polygon's centroid. Used to make obstacle promotion and YAML loading
/// idempotent: a re-promote or reload of the same keepout becomes a no-op.
inline bool has_duplicate_obstacle(const std::vector<geometry_msgs::msg::Polygon>& existing,
                                   const geometry_msgs::msg::Polygon& candidate,
                                   double eps)
{
  const auto cc = polygon_centroid(candidate);
  for (const auto& poly : existing)
  {
    const auto ec = polygon_centroid(poly);
    if (std::hypot(static_cast<double>(ec.x) - static_cast<double>(cc.x),
                   static_cast<double>(ec.y) - static_cast<double>(cc.y)) <= eps)
    {
      return true;
    }
  }
  return false;
}

/// Closest point on the polygon perimeter to (px, py), plus its distance.
struct ClosestEdge
{
  double x{0.0};
  double y{0.0};
  double distance{std::numeric_limits<double>::max()};
};

inline ClosestEdge closest_edge_point(double px,
                                      double py,
                                      const geometry_msgs::msg::Polygon& polygon)
{
  ClosestEdge best;
  const auto& pts = polygon.points;
  const std::size_t n = pts.size();
  if (n < 2)
  {
    return best;
  }

  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = static_cast<double>(pts[j].x);
    const double ay = static_cast<double>(pts[j].y);
    const double bx = static_cast<double>(pts[i].x);
    const double by = static_cast<double>(pts[i].y);

    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;

    double t = 0.0;
    if (len2 > 1e-12)
    {
      t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0);
    }

    const double cx = ax + t * dx;
    const double cy = ay + t * dy;
    const double dist = std::hypot(px - cx, py - cy);
    if (dist < best.distance)
    {
      best = {cx, cy, dist};
    }
  }
  return best;
}

/// Minimum distance from point (px, py) to the edges of a polygon.
inline double point_to_polygon_distance(double px,
                                        double py,
                                        const geometry_msgs::msg::Polygon& polygon)
{
  return closest_edge_point(px, py, polygon).distance;
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__INTERNAL_HELPERS_HPP_
