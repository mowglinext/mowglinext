// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// How another fleet member's charging station may be applied to THIS robot's map.
// Pure, testable. Hard rule: a neighbouring station must never block our own dock
// or undock, whatever its pose says. Docks side by side are normal (0.53 m apart on
// 2026-10-28), and a peer's pose can simply be wrong.
#ifndef MOWGLI_MAP__PEER_DOCK_RINGS_HPP_
#define MOWGLI_MAP__PEER_DOCK_RINGS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/polygon.hpp>

namespace mowgli_map
{

enum class PeerDockUse
{
  kFull,  // forbidden with the body margin
  kBareRing,  // the margin would close our approach corridor: the object only
  kIgnored,  // it overlaps our own station or corridor: never let it block them
};

namespace peer_dock_detail
{

inline double segment_distance(double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  const double t =
      len2 > 0.0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0) : 0.0;
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

// Separating-axis test: true when an edge normal of `a` separates the two.
inline bool separated_by_edges_of(const geometry_msgs::msg::Polygon& a,
                                  const geometry_msgs::msg::Polygon& b)
{
  const auto project = [](const geometry_msgs::msg::Polygon& polygon, double nx, double ny)
  {
    double lo = std::numeric_limits<double>::max();
    double hi = std::numeric_limits<double>::lowest();
    for (const auto& p : polygon.points)
    {
      const double d = nx * p.x + ny * p.y;
      lo = std::min(lo, d);
      hi = std::max(hi, d);
    }
    return std::pair<double, double>(lo, hi);
  };
  for (std::size_t i = 0; i + 1 < a.points.size(); ++i)
  {
    const double nx = -(a.points[i + 1].y - a.points[i].y);
    const double ny = a.points[i + 1].x - a.points[i].x;
    if (nx == 0.0 && ny == 0.0)
    {
      continue;
    }
    const auto [a_lo, a_hi] = project(a, nx, ny);
    const auto [b_lo, b_hi] = project(b, nx, ny);
    if (a_hi < b_lo || b_hi < a_lo)
    {
      return true;
    }
  }
  return false;
}

inline double vertices_to_edges(const geometry_msgs::msg::Polygon& vertices,
                                const geometry_msgs::msg::Polygon& edges)
{
  double best = std::numeric_limits<double>::max();
  for (const auto& p : vertices.points)
  {
    for (std::size_t i = 0; i + 1 < edges.points.size(); ++i)
    {
      best = std::min(best,
                      segment_distance(p.x,
                                       p.y,
                                       edges.points[i].x,
                                       edges.points[i].y,
                                       edges.points[i + 1].x,
                                       edges.points[i + 1].y));
    }
  }
  return best;
}

}  // namespace peer_dock_detail

/// Two closed convex rings (first point repeated last) share area or touch.
inline bool convex_rings_overlap(const geometry_msgs::msg::Polygon& a,
                                 const geometry_msgs::msg::Polygon& b)
{
  return !peer_dock_detail::separated_by_edges_of(a, b) &&
         !peer_dock_detail::separated_by_edges_of(b, a);
}

/// Gap between two closed convex rings; 0 when they overlap.
inline double convex_ring_clearance(const geometry_msgs::msg::Polygon& a,
                                    const geometry_msgs::msg::Polygon& b)
{
  if (convex_rings_overlap(a, b))
  {
    return 0.0;
  }
  return std::min(peer_dock_detail::vertices_to_edges(a, b),
                  peer_dock_detail::vertices_to_edges(b, a));
}

/// `own_*` are empty when this robot has no dock yet: then nothing can be blocked.
inline PeerDockUse classify_peer_dock(const geometry_msgs::msg::Polygon& peer,
                                      const geometry_msgs::msg::Polygon& own_body,
                                      const geometry_msgs::msg::Polygon& own_corridor,
                                      double margin)
{
  if (own_body.points.size() < 3 || own_corridor.points.size() < 3)
  {
    return PeerDockUse::kFull;
  }
  if (convex_rings_overlap(peer, own_body) || convex_rings_overlap(peer, own_corridor))
  {
    return PeerDockUse::kIgnored;
  }
  return convex_ring_clearance(peer, own_corridor) <= margin ? PeerDockUse::kBareRing
                                                             : PeerDockUse::kFull;
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__PEER_DOCK_RINGS_HPP_
