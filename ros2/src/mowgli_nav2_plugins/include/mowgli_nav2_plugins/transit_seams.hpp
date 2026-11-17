// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "mowgli_nav2_plugins/transit_geometry.hpp"

namespace mowgli_nav2_plugins::transit
{
// Two areas drawn side by side look joined in the map editor but rarely are: the operator placed
// each border by eye, so a gap of a few centimetres runs along it and the exact union treats that
// gap as a wall. The transit then leaves through the one spot where the rings do touch, which can
// be the far end of the garden (operator report 2026-11-10).
//
// seamRings() returns extra ALLOWED rings that bridge such gaps. For a pair of rings, each edge of
// one ring gets a strip over the stretch(es) of it lying within `tolerance` of the other ring: the
// stretch itself, pushed outward from its own ring by `tolerance`. They are ordinary rings, so the
// exact segment, raster and connectivity rules keep working unchanged on top of them.
//
// Bound: a strip is the stretch of a recorded edge swept `tolerance` sideways, so every point of
// it is within `tolerance` of that recorded area, and it is kept only if it lands on the other
// ring. Only ONE ring of a pair grows strips, so two rings further apart than `tolerance` are
// never joined by their own pair.

// Bisection steps: 2^-40 of an edge is far below kEpsilon for any garden-sized edge.
constexpr int kSeamSearchSteps = 40;

namespace seam_detail
{
// Where on edge (a, b) the distance to edge (c, d) is smallest. The distance from a point moving
// along (a, b) to the convex set (c, d) is convex: ternary search.
inline double nearestParameter(Point a, Point b, Point c, Point d)
{
  double lo = 0, hi = 1;
  for (int i = 0; i < 2 * kSeamSearchSteps; ++i)
  {
    const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
    if (edgeDistance(interpolate(a, b, m1), c, d) <= edgeDistance(interpolate(a, b, m2), c, d))
      hi = m2;
    else
      lo = m1;
  }
  return (lo + hi) / 2;
}
// The end of the within-tolerance stretch between `in` (within) and `out`.
inline double stretchEnd(
    Point a, Point b, Point c, Point d, double tolerance, double in, double out)
{
  if (edgeDistance(interpolate(a, b, out), c, d) <= tolerance)
    return out;
  for (int i = 0; i < kSeamSearchSteps; ++i)
  {
    const double mid = (in + out) / 2;
    (edgeDistance(interpolate(a, b, mid), c, d) <= tolerance ? in : out) = mid;
  }
  return in;
}
inline double signedArea2(const Ring& ring)
{
  double result = 0;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
    result += cross(ring[j], ring[i]);
  return result;
}
// Strips grown from the edges of `ring` toward `other`.
inline std::vector<Ring> strips(const Ring& ring, const Ring& other, double tolerance)
{
  std::vector<Ring> result;
  const double area2 = signedArea2(ring);
  if (ring.size() < 3 || other.size() < 3 || !(std::abs(area2) > kEpsilon))
    return result;
  // Counter-clockwise ring: the outside is on the right of each directed edge.
  const double outward = area2 > 0 ? 1 : -1;
  std::vector<std::pair<double, double>> stretches;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const Point a = ring[j], b = ring[i];
    const double length = distance(a, b);
    if (!(length > kEpsilon) || remoteRing(a, b, other, tolerance))
      continue;
    stretches.clear();
    for (std::size_t k = 0, l = other.size() - 1; k < other.size(); l = k++)
    {
      const Point c = other[l], d = other[k];
      if (std::max(a.x, b.x) < std::min(c.x, d.x) - tolerance ||
          std::min(a.x, b.x) > std::max(c.x, d.x) + tolerance ||
          std::max(a.y, b.y) < std::min(c.y, d.y) - tolerance ||
          std::min(a.y, b.y) > std::max(c.y, d.y) + tolerance)
        continue;
      const double nearest = nearestParameter(a, b, c, d);
      if (!(edgeDistance(interpolate(a, b, nearest), c, d) <= tolerance))
        continue;
      const double begin = stretchEnd(a, b, c, d, tolerance, nearest, 0);
      const double end = stretchEnd(a, b, c, d, tolerance, nearest, 1);
      // The two edges coincide over the whole stretch (the distance is convex, so zero at both
      // ends is zero in between): a true shared border, which the exact union already joins.
      if (edgeDistance(interpolate(a, b, begin), c, d) <= kEpsilon &&
          edgeDistance(interpolate(a, b, end), c, d) <= kEpsilon)
        continue;
      stretches.emplace_back(begin, end);
    }
    std::sort(stretches.begin(), stretches.end());
    const Point push{outward * (b.y - a.y) / length * tolerance,
                     -outward * (b.x - a.x) / length * tolerance};
    for (std::size_t s = 0; s < stretches.size();)
    {
      auto [begin, end] = stretches[s];
      for (++s; s < stretches.size() && stretches[s].first <= end; ++s)
        end = std::max(end, stretches[s].second);
      if (!((end - begin) * length > kEpsilon))
        continue;
      const Point p1 = interpolate(a, b, begin), p2 = interpolate(a, b, end);
      const Point q1{p1.x + push.x, p1.y + push.y}, q2{p2.x + push.x, p2.y + push.y};
      // A strip must land on the other ring: one pushed away from it (two borders in line, a
      // corner next to the other area) would only add ground outside both. And where the stretch
      // already is ground of the other ring (overlapping areas) there is nothing to bridge.
      // Dropping a strip never authorizes anything.
      const bool reaches =
          inside(q1, other) || inside(q2, other) || inside(interpolate(q1, q2, 0.5), other);
      const bool covered =
          inside(p1, other) && inside(p2, other) && inside(q1, other) && inside(q2, other);
      if (reaches && !covered)
        result.push_back({p1, p2, q2, q1});
    }
  }
  return result;
}
}  // namespace seam_detail

inline std::vector<Ring> seamRings(const std::vector<Ring>& rings, double tolerance)
{
  std::vector<Ring> result;
  if (!(tolerance > 0) || !std::isfinite(tolerance))
    return result;
  for (std::size_t first = 0; first < rings.size(); ++first)
    for (std::size_t second = first + 1; second < rings.size(); ++second)
    {
      auto forward = seam_detail::strips(rings[first], rings[second], tolerance);
      auto backward = seam_detail::strips(rings[second], rings[first], tolerance);
      // Either side bridges the gap; the side with fewer, longer edges does it with fewer rings,
      // and every ring is one more polygon each search edge is tested against.
      auto& fewer = forward.empty() || (!backward.empty() && backward.size() < forward.size())
                        ? backward
                        : forward;
      result.insert(result.end(),
                    std::make_move_iterator(fewer.begin()),
                    std::make_move_iterator(fewer.end()));
    }
  return result;
}
}  // namespace mowgli_nav2_plugins::transit
