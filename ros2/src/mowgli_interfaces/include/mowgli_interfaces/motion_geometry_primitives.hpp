// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace mowgli_interfaces::motion
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

// Split at EVERY polygon crossing, then classify each open interval. Sampling
// at a fixed spacing can miss arbitrarily thin gaps and diagonal corner cuts.
inline std::vector<double> cuts(Point a, Point b, const std::vector<Ring>& rings)
{
  std::vector<double> result{0, 1};
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
  return result;
}

struct Geometry
{
  std::vector<Ring> allowed;
  std::vector<Ring> blocked;
  std::vector<double> blocked_margins;
  double obstacle_margin{0};

  double margin(std::size_t i) const
  {
    return i < blocked_margins.size() ? blocked_margins[i] : obstacle_margin;
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
  bool segmentAuthorized(Point a, Point b) const
  {
    if (!authorized(a) || !authorized(b))
      return false;
    const auto ts = cuts(a, b, allowed);
    for (std::size_t i = 1; i < ts.size(); ++i)
      if (!authorized(interpolate(a, b, (ts[i - 1] + ts[i]) / 2)))
        return false;
    return true;
  }
  bool segmentClear(Point a, Point b) const
  {
    if (!clear(a) || !clear(b))
      return false;
    for (std::size_t index = 0; index < blocked.size(); ++index)
    {
      const auto& ring = blocked[index];
      if (cuts(a, b, {ring}).size() > 2)
        return false;
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
        if (std::min({edgeDistance(a, ring[j], ring[i]),
                      edgeDistance(b, ring[j], ring[i]),
                      edgeDistance(ring[j], a, b),
                      edgeDistance(ring[i], a, b)}) <= margin(index) + kEpsilon)
          return false;
    }
    return true;
  }
};
}  // namespace mowgli_interfaces::motion
