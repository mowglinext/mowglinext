// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <span>
#include <vector>

namespace mowgli_nav2_plugins::transit
{
struct Point
{
  double x, y;
};
using Ring = std::vector<Point>;
constexpr double kEpsilon = 1e-8;

inline Point interpolate(Point a, Point b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}
inline double cross(Point a, Point b)
{
  return a.x * b.y - a.y * b.x;
}
inline double distance(Point a, Point b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
inline Point closestOnEdge(Point p, Point a, Point b)
{
  const double length2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
  const double t =
      length2 == 0
          ? 0
          : std::clamp(((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / length2, 0.0, 1.0);
  return interpolate(a, b, t);
}
inline double edgeDistance(Point p, Point a, Point b)
{
  return distance(p, closestOnEdge(p, a, b));
}
inline bool inside(Point p, const Ring& ring)
{
  if (ring.size() < 3 || !std::isfinite(p.x) || !std::isfinite(p.y))
    return false;
  bool result = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const auto a = ring[j], b = ring[i];
    if (p.x >= std::min(a.x, b.x) - kEpsilon && p.x <= std::max(a.x, b.x) + kEpsilon &&
        p.y >= std::min(a.y, b.y) - kEpsilon && p.y <= std::max(a.y, b.y) + kEpsilon &&
        edgeDistance(p, a, b) <= kEpsilon)
      return true;
    if ((a.y > p.y) != (b.y > p.y) && p.x < a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y))
      result = !result;
  }
  return result;
}
inline double ringDistance(Point p, const Ring& ring)
{
  if (inside(p, ring))
    return 0;
  double result = INFINITY;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
    result = std::min(result, edgeDistance(p, ring[j], ring[i]));
  return result;
}

// Reject only distance work for a provably remote ring. Crossing checks still
// run independently. Widen for rounding in closestOnEdge's clamped interpolation;
// extreme/non-finite intermediate padding simply disables the fast rejection.
inline bool remoteRing(Point a, Point b, const Ring& ring, double margin)
{
  double min_x = INFINITY, min_y = INFINITY, max_x = -INFINITY, max_y = -INFINITY;
  for (const auto& p : ring)
  {
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
  }
  const double magnitude = std::max({std::abs(min_x),
                                     std::abs(min_y),
                                     std::abs(max_x),
                                     std::abs(max_y),
                                     std::abs(a.x),
                                     std::abs(a.y),
                                     std::abs(b.x),
                                     std::abs(b.y),
                                     std::abs(margin)});
  const double padding =
      std::max(0.0, margin) + 8 * (std::nextafter(magnitude, INFINITY) - magnitude);
  return std::isfinite(padding) &&
         (std::max(a.x, b.x) < min_x - padding || std::min(a.x, b.x) > max_x + padding ||
          std::max(a.y, b.y) < min_y - padding || std::min(a.y, b.y) > max_y + padding);
}

// Split at EVERY polygon crossing, then classify each open interval. Sampling
// at a fixed spacing can miss arbitrarily thin gaps and diagonal corner cuts.
inline void cuts(Point a, Point b, std::span<const Ring> rings, std::vector<double>& result)
{
  result.assign({0, 1});
  const Point d{b.x - a.x, b.y - a.y};
  const double length2 = d.x * d.x + d.y * d.y;
  for (const auto& ring : rings)
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
    {
      const Point e{ring[i].x - ring[j].x, ring[i].y - ring[j].y};
      const Point q{ring[j].x - a.x, ring[j].y - a.y};
      const double denominator = cross(d, e);
      if (std::abs(denominator) > kEpsilon)
      {
        const double t = cross(q, e) / denominator, u = cross(q, d) / denominator;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
          result.push_back(t);
      }
      else if (length2 > 0 && std::abs(cross(q, d)) <= kEpsilon)
      {
        for (auto p : {ring[j], ring[i]})
          result.push_back(std::clamp(((p.x - a.x) * d.x + (p.y - a.y) * d.y) / length2, 0.0, 1.0));
      }
    }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
}
inline std::vector<double> cuts(Point a, Point b, const std::vector<Ring>& rings)
{
  std::vector<double> result;
  cuts(a, b, rings, result);
  return result;
}

// Area centroid (shoelace). A repeated closing vertex adds a zero-length edge and changes nothing.
inline std::optional<Point> areaCentroid(const Ring& ring)
{
  double area2 = 0, cx = 0, cy = 0;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const double c = cross(ring[j], ring[i]);
    area2 += c;
    cx += (ring[j].x + ring[i].x) * c;
    cy += (ring[j].y + ring[i].y) * c;
  }
  if (ring.size() < 3 || !(std::abs(area2) > kEpsilon) || !std::isfinite(area2))
    return std::nullopt;
  return Point{cx / (3 * area2), cy / (3 * area2)};
}

// The dock's own frame, recovered from the two rings map_server builds around the dock pose
// (`rebuild_dock_polygons`): the body spans local x in [-dock_body_length, 0] and the approach
// corridor lies directly behind it, further along -x. +x points from the corridor through the
// body TOWARD THE CHARGING STATION (the robot docks nose-first, staging_x_offset < 0), and x = 0 is
// the body's station-side edge, i.e. the line through the dock pose.
struct DockFrame
{
  Point origin;  // on the station-side edge of the body, on its centre line
  Point axis;  // unit vector, toward the station

  Point local(Point p) const
  {
    const Point d{p.x - origin.x, p.y - origin.y};
    return {d.x * axis.x + d.y * axis.y, -d.x * axis.y + d.y * axis.x};
  }
};
// How far the corridor's station-side edge may sit from the body's rear edge and still count as
// "directly behind it" (float32 polygon points, nothing more).
constexpr double kDockRingAdjacency = 0.02;
inline std::optional<DockFrame> dockFrame(const Ring& body, const Ring& corridor)
{
  const auto body_centre = areaCentroid(body), corridor_centre = areaCentroid(corridor);
  if (!body_centre || !corridor_centre)
    return std::nullopt;
  Point axis{body_centre->x - corridor_centre->x, body_centre->y - corridor_centre->y};
  const double length = std::hypot(axis.x, axis.y);
  if (!std::isfinite(length) || !(length > kEpsilon))
    return std::nullopt;
  axis = {axis.x / length, axis.y / length};
  const auto along = [&](Point p)
  {
    return (p.x - body_centre->x) * axis.x + (p.y - body_centre->y) * axis.y;
  };
  double body_front = -INFINITY, body_rear = INFINITY, corridor_front = -INFINITY;
  for (const auto& p : body)
  {
    body_front = std::max(body_front, along(p));
    body_rear = std::min(body_rear, along(p));
  }
  for (const auto& p : corridor)
    corridor_front = std::max(corridor_front, along(p));
  // Anything else is not the map_server dock layout: fail closed rather than guess a heading.
  if (std::abs(corridor_front - body_rear) > kDockRingAdjacency)
    return std::nullopt;
  return DockFrame{{body_centre->x + body_front * axis.x, body_centre->y + body_front * axis.y},
                   axis};
}

struct Geometry
{
  std::vector<Ring> allowed;
  std::vector<Ring> blocked;
  std::vector<double> blocked_margins;
  // Parallel to `blocked` (a missing entry means false): rings a START may lie inside. Only the
  // dock body is one — the robot legitimately ends a coverage unit in front of the dock, and a
  // start there must plan its way OUT. Goals, search cells and every path segment still treat the
  // ring as blocked.
  std::vector<bool> start_exempt;
  // Set only when the snapshot carries exactly one dock body and one approach corridor; without
  // it a start inside the dock body cannot be oriented and stays START_OCCUPIED.
  std::optional<DockFrame> dock;
  double obstacle_margin{0};

  double margin(std::size_t i) const
  {
    return i < blocked_margins.size() ? blocked_margins[i] : obstacle_margin;
  }

  bool startExempt(std::size_t i) const
  {
    return i < start_exempt.size() && start_exempt[i];
  }
  bool insideStartExempt(Point p) const
  {
    for (std::size_t i = 0; i < blocked.size(); ++i)
      if (startExempt(i) && inside(p, blocked[i]))
        return true;
    return false;
  }
  // The same geometry without the start-exempt rings: what a start inside one of them is judged
  // and connected against.
  Geometry withoutStartExempt() const
  {
    Geometry result;
    result.allowed = allowed;
    result.dock = dock;
    result.obstacle_margin = obstacle_margin;
    for (std::size_t i = 0; i < blocked.size(); ++i)
    {
      if (startExempt(i))
        continue;
      result.blocked.push_back(blocked[i]);
      result.blocked_margins.push_back(margin(i));
    }
    return result;
  }

  bool authorized(Point p) const
  {
    return std::any_of(allowed.begin(),
                       allowed.end(),
                       [&](const auto& r)
                       {
                         return inside(p, r);
                       });
  }
  bool clear(Point p) const
  {
    for (std::size_t i = 0; i < blocked.size(); ++i)
      if (ringDistance(p, blocked[i]) <= margin(i) + kEpsilon)
        return false;
    return true;
  }

  // A negative-only preflight for authorized anchors. Overlapping ring bounds
  // are a superset of actual connectivity; a positive result still requires
  // the exact search and current obstacle checks. No verdict survives a plan.
  template <typename Check>
  bool boundsMayConnect(Point start, Point goal, const Check& check) const
  {
    if (allowed.size() <= 1)
      return !allowed.empty();
    std::vector<bool> visited(allowed.size(), false);
    std::vector<std::size_t> pending;
    for (std::size_t i = 0; i < allowed.size(); ++i)
    {
      check();
      if (inside(start, allowed[i]))
      {
        if (inside(goal, allowed[i]))
          return true;
        visited[i] = true;
        pending.push_back(i);
      }
    }
    while (!pending.empty())
    {
      check();
      const auto current = pending.back();
      pending.pop_back();
      if (inside(goal, allowed[current]))
        return true;
      Point lo{INFINITY, INFINITY}, hi{-INFINITY, -INFINITY};
      for (const auto& p : allowed[current])
      {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
      }
      for (std::size_t i = 0; i < allowed.size(); ++i)
        if (!visited[i])
        {
          check();
          // Each closed ring includes its own epsilon boundary tolerance.
          if (!remoteRing(lo, hi, allowed[i], 2 * kEpsilon))
          {
            visited[i] = true;
            pending.push_back(i);
          }
        }
    }
    return false;
  }

  bool segmentAuthorized(Point a, Point b) const
  {
    std::vector<double> scratch;
    return segmentAuthorized(a, b, scratch);
  }
  bool segmentAuthorized(Point a, Point b, std::vector<double>& scratch) const
  {
    if (!authorized(a) || !authorized(b))
      return false;
    cuts(a, b, allowed, scratch);
    for (std::size_t i = 1; i < scratch.size(); ++i)
      if (!authorized(interpolate(a, b, (scratch[i - 1] + scratch[i]) / 2)))
        return false;
    return true;
  }
  bool segmentClear(Point a, Point b) const
  {
    std::vector<double> scratch;
    return segmentClear(a, b, scratch);
  }
  bool segmentClear(Point a, Point b, std::vector<double>& scratch) const
  {
    if (!clear(a) || !clear(b))
      return false;
    for (std::size_t index = 0; index < blocked.size(); ++index)
    {
      const auto& ring = blocked[index];
      cuts(a, b, std::span<const Ring>(&ring, 1), scratch);
      if (scratch.size() > 2)
        return false;
      if (remoteRing(a, b, ring, margin(index) + kEpsilon))
        continue;
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
        if (std::min({edgeDistance(a, ring[j], ring[i]),
                      edgeDistance(b, ring[j], ring[i]),
                      edgeDistance(ring[j], a, b),
                      edgeDistance(ring[i], a, b)}) <= margin(index) + kEpsilon)
          return false;
    }
    return true;
  }
  // Only the terminal-to-area leg may leave the union. It must enter the
  // uniquely nearest boundary location once and remain in the union of rings
  // sharing that location until its anchor. A seam is not a competing boundary.
  bool terminal(Point endpoint, Point anchor, double outside_limit) const
  {
    if (authorized(endpoint))
      return segmentAuthorized(endpoint, anchor) && segmentClear(endpoint, anchor);
    double nearest = INFINITY;
    for (const auto& ring : allowed)
      nearest = std::min(nearest, ringDistance(endpoint, ring));
    if (!std::isfinite(nearest) || nearest > outside_limit)
      return false;
    std::vector<Ring> intended;
    Point nearest_point{};
    bool found_point = false;
    for (const auto& ring : allowed)
    {
      if (std::abs(ringDistance(endpoint, ring) - nearest) > kEpsilon)
        continue;
      intended.push_back(ring);
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      {
        const auto q = closestOnEdge(endpoint, ring[j], ring[i]);
        if (std::abs(distance(endpoint, q) - nearest) > kEpsilon)
          continue;
        // Distinct nearest points remain ambiguous even if their rings are
        // joined elsewhere: connectivity must not authorize crossing a gap.
        if (found_point && distance(q, nearest_point) > kEpsilon)
          return false;
        if (!found_point)
          nearest_point = q;
        found_point = true;
      }
    }
    const auto in_intended = [&](Point p)
    {
      return std::any_of(intended.begin(),
                         intended.end(),
                         [&](const auto& r)
                         {
                           return inside(p, r);
                         });
    };
    if (!found_point || !in_intended(anchor) || !segmentClear(endpoint, anchor))
      return false;
    const auto ts = cuts(endpoint, anchor, intended);
    bool entered = false;
    for (std::size_t i = 1; i < ts.size(); ++i)
    {
      const bool in = in_intended(interpolate(endpoint, anchor, (ts[i - 1] + ts[i]) / 2));
      if (entered && !in)
        return false;
      if (!in && distance(endpoint, interpolate(endpoint, anchor, ts[i])) > outside_limit)
        return false;
      entered |= in;
    }
    return entered;
  }
};
}  // namespace mowgli_nav2_plugins::transit
