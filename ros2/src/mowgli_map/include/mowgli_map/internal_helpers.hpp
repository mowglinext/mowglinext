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

/// Default side of the square keepout stamped at a wheel-slip dig [m].
/// One chassis LENGTH (the Nav2 robot_footprint is 0.60 m x 0.40 m). The
/// previous default was one tool width (0.18 m) - narrower than the body,
/// so "route around the dig" still dragged the chassis across it and issue
/// #500 saw the robot re-latch a dig 3x in 18.4 s inside 0.13 m.
constexpr double kDefaultDigKeepoutSizeM = 0.60;

/// Hard floor for the dig keepout, whatever dig_obstacle_size is set to.
/// Below one costmap cell the keepout cannot be represented at all.
constexpr double kMinDigKeepoutSizeM = 0.05;

/// How far BEHIND the dig point (against the robot's heading) the LETHAL
/// band of the dig keepout may reach [m] - polygon plus the obstacle_margin
/// the mask paints around every obstacle polygon. The hole is under the
/// wheels, i.e. at the dig point itself, and the bridge reverses the robot
/// out of it by dig_reverse_dist (0.30 m commanded, ~0.20 m real on a
/// slipping tyre - 2026-09-10 bag). A square centred on the dig plus the
/// 0.15 m margin reached 0.45 m behind it, so the reversed robot was still
/// standing INSIDE its own keepout: Smac answered START_OCCUPIED to every
/// transit and the mission looped until an operator stopped it (twice on
/// 2026-09-10). 0.10 m still covers the tyre contact patch; everything the
/// robot needs to route around (the hole and the ground it was pushing into)
/// lies AHEAD of the dig point.
constexpr double kDigKeepoutBehindM = 0.10;

/// Dig keepout polygon (map frame).
///
/// With a known heading: a rectangle `size` wide (lateral, centred on the dig
/// point) spanning `size` ahead of the dig point along the heading, and whose
/// rear edge is placed so that polygon + `lethal_margin` (the mask's
/// obstacle_margin band) reaches exactly kDigKeepoutBehindM behind the dig
/// point, whatever the configured margin. Without a heading (no TF yet): the
/// legacy square of side `size` centred on the dig point, the only
/// orientation-free choice.
inline geometry_msgs::msg::Polygon dig_keepout_polygon(
    double x, double y, double yaw, bool have_heading, double size, double lethal_margin)
{
  const double s = std::max(size, kMinDigKeepoutSizeM);
  const double half = s * 0.5;
  double along_min = -half, along_max = half, cy = 1.0, sy = 0.0;
  if (have_heading)
  {
    along_max = s;
    along_min = std::min(std::max(lethal_margin, 0.0) - kDigKeepoutBehindM,
                         along_max - kMinDigKeepoutSizeM);
    cy = std::cos(yaw);
    sy = std::sin(yaw);
  }
  // Corners in the dig frame (along, lateral), CCW.
  const double corners[4][2] = {{along_min, -half},
                                {along_max, -half},
                                {along_max, half},
                                {along_min, half}};
  geometry_msgs::msg::Polygon poly;
  for (const auto& c : corners)
  {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x + c[0] * cy - c[1] * sy);
    p.y = static_cast<float>(y + c[0] * sy + c[1] * cy);
    poly.points.push_back(p);
  }
  return poly;
}

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
