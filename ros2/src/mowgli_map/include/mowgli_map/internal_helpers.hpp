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

// ── Wheel-slip dig PROPOSAL geometry ─────────────────────────────────────────
//
// A dig proposal is sized to the PHYSICAL dig, not to the chassis. The old
// polygon was one chassis length (0.60 m) and biased ahead of the heading for
// one reason only: it was stamped as a session KEEPOUT the moment the dig
// happened, so it had to contain the body and must not reach back under the
// reversed robot. A dig is an operator proposal now and nothing is stamped,
// so that reason is gone — and a 0.60 m box made every accepted dig blank out
// a big patch of lawn (keepout band and coverage obstacle_margin are added ON
// TOP of the polygon: the body is counted exactly once, by them, never by the
// polygon itself).
//
// What digs is the drive wheels: two ruts, one under each tyre, at the dig
// point (base_link = the drive axle centre) +/- wheel_track/2. The detector
// does not say which wheel slipped nor which way the robot faced, so the
// proposal is the smallest REGULAR shape centred on the dig point that covers
// both contact patches whatever the heading: a disc of radius
//   hypot(wheel_track/2 + wheel_width/2, wheel_radius/2)
// (robot_config_util.dig_proposal_radius, injected as `dig_proposal_radius`;
// 0.189 m for the shipped robot), grown by half the distance the chassis crept
// during the slip window (DigEvent.map_distance) — the ruts are that much
// longer.

/// Fallback for `dig_proposal_radius` when no launch file injects it (tests,
/// ad-hoc runs). The real value is DERIVED from the wheel geometry of the
/// merged robot config.
constexpr double kFallbackDigProposalRadiusM = 0.19;

/// Lower bound of the proposal radius [m]. Two reasons, both about a polygon
/// that is too small to be useful: at the 0.10 m map resolution a smaller disc
/// can fall between cell centres (an ACCEPTED proposal would then stamp no
/// NO_GO cell at all), and the operator must be able to see and click the hole
/// in the GUI map.
constexpr double kMinDigProposalRadiusM = 0.10;

/// Upper bound of what DigEvent.map_distance may add to the radius [m]. The
/// detector latches when the map moved LESS than a fraction of the wheel
/// travel, so a large value here is a stale/garbage report, not a long rut.
constexpr double kMaxDigSlipGrowthM = 0.10;

/// Vertices of the regular polygon approximating the disc.
constexpr int kDigProposalVertices = 8;

/// Radius of the proposal for one dig report.
inline double dig_proposal_radius(double base_radius_m, double map_distance_m)
{
  const double base = std::isfinite(base_radius_m) ? base_radius_m : 0.0;
  const double creep = std::isfinite(map_distance_m) ? map_distance_m : 0.0;
  const double growth = std::min(std::max(creep * 0.5, 0.0), kMaxDigSlipGrowthM);
  return std::max(base + growth, kMinDigProposalRadiusM);
}

/// Regular polygon CIRCUMSCRIBING the disc of `radius` centred on (x, y), map
/// frame, CCW, NOT closed (ROS polygon convention). Circumscribed so the whole
/// disc is inside it; orientation-free, so no heading is needed.
inline geometry_msgs::msg::Polygon dig_proposal_polygon(double x, double y, double radius)
{
  const double r = std::max(radius, kMinDigProposalRadiusM);
  const double step = 2.0 * M_PI / static_cast<double>(kDigProposalVertices);
  const double vertex_r = r / std::cos(step * 0.5);
  geometry_msgs::msg::Polygon poly;
  for (int i = 0; i < kDigProposalVertices; ++i)
  {
    const double a = step * (static_cast<double>(i) + 0.5);
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(x + vertex_r * std::cos(a));
    p.y = static_cast<float>(y + vertex_r * std::sin(a));
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

/// Closest point to (px, py) on the segment a→b, plus its distance. The ONE
/// place this arithmetic lives: closest_edge_point() below and the edge-driven
/// keepout rasteriser (polygon_raster.hpp) must agree to the last bit.
inline ClosestEdge closest_point_on_segment(
    double px, double py, double ax, double ay, double bx, double by)
{
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
  return {cx, cy, std::hypot(px - cx, py - cy)};
}

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
    const ClosestEdge candidate = closest_point_on_segment(px,
                                                           py,
                                                           static_cast<double>(pts[j].x),
                                                           static_cast<double>(pts[j].y),
                                                           static_cast<double>(pts[i].x),
                                                           static_cast<double>(pts[i].y));
    if (candidate.distance < best.distance)
    {
      best = candidate;
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
