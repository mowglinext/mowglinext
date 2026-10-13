// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "mowgli_interfaces/motion_geometry_types.hpp"
#include <boost/multiprecision/cpp_int.hpp>

namespace mowgli_interfaces::motion
{
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
// Same boundary/parity arithmetic for a complete ring or a conservatively
// selected set of its original edges. True means the boundary matched.
inline double parityCrossingX(double y, Point a, Point b)
{
  return a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
}
inline bool insideEdge(Point p, Point a, Point b, bool& parity)
{
  if (p.x >= std::min(a.x, b.x) - kEpsilon && p.x <= std::max(a.x, b.x) + kEpsilon &&
      p.y >= std::min(a.y, b.y) - kEpsilon && p.y <= std::max(a.y, b.y) + kEpsilon &&
      edgeDistance(p, a, b) <= kEpsilon)
    return true;
  if ((a.y > p.y) != (b.y > p.y) && p.x < parityCrossingX(p.y, a, b))
    parity = !parity;
  return false;
}
inline bool inside(Point p, const Ring& ring)
{
  if (ring.size() < 3 || !std::isfinite(p.x) || !std::isfinite(p.y))
    return false;
  bool result = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
    if (insideEdge(p, ring[j], ring[i], result))
      return true;
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
// Append every robust crossing of one original edge. The filtering caller
// must retain all edges whose closed boxes overlap the closed chord box.
inline void appendEdgeCuts(Point a, Point b, Point c, Point d, std::vector<double>& result)
{
  const long double dx = static_cast<long double>(b.x) - a.x;
  const long double dy = static_cast<long double>(b.y) - a.y;
  // Comparisons of the original coordinates prove disjoint closed boxes
  // without arithmetic error. Parallel distant edges need no determinant
  // or exact fraction; touching boxes still take the complete predicate.
  if (std::max(a.x, b.x) < std::min(c.x, d.x) || std::min(a.x, b.x) > std::max(c.x, d.x) ||
      std::max(a.y, b.y) < std::min(c.y, d.y) || std::min(a.y, b.y) > std::max(c.y, d.y))
    return;
  const long double ex = static_cast<long double>(d.x) - c.x;
  const long double ey = static_cast<long double>(d.y) - c.y;
  const long double qx = static_cast<long double>(c.x) - a.x;
  const long double qy = static_cast<long double>(c.y) - a.y;
  const long double denominator = dx * ey - dy * ex;
  // Absolute determinant thresholds miss real shallow crossings. Bound
  // subtraction/product rounding by coordinate scale, then resolve only
  // ambiguous determinants exactly (binary doubles are exact rationals).
  const long double error = 16 * std::numeric_limits<long double>::epsilon() *
                            ((std::abs(a.x) + std::abs(b.x)) * std::abs(ey) +
                             (std::abs(a.y) + std::abs(b.y)) * std::abs(ex) +
                             (std::abs(d.x) + std::abs(c.x)) * std::abs(dy) +
                             (std::abs(d.y) + std::abs(c.y)) * std::abs(dx));
  const long double numerator_t = qx * ey - qy * ex;
  const long double numerator_u = qx * dy - qy * dx;
  const long double error_t = 16 * std::numeric_limits<long double>::epsilon() *
                              ((std::abs(c.x) + std::abs(a.x)) * std::abs(ey) +
                               (std::abs(c.y) + std::abs(a.y)) * std::abs(ex) +
                               (std::abs(d.x) + std::abs(c.x)) * std::abs(qy) +
                               (std::abs(d.y) + std::abs(c.y)) * std::abs(qx));
  const long double error_u = 16 * std::numeric_limits<long double>::epsilon() *
                              ((std::abs(c.x) + std::abs(a.x)) * std::abs(dy) +
                               (std::abs(c.y) + std::abs(a.y)) * std::abs(dx) +
                               (std::abs(b.x) + std::abs(a.x)) * std::abs(qy) +
                               (std::abs(b.y) + std::abs(a.y)) * std::abs(qx));
  // A well-determined denominator can prove that an intersection lies
  // outside either segment without resolving its fraction accurately.
  // Dense distant edges otherwise fall back to rationals solely because
  // their (irrelevant) fraction is large. Keep ambiguous crossings exact.
  if (std::abs(denominator) > error)
  {
    const long double sign = denominator > 0 ? 1 : -1;
    const long double nt = sign * numerator_t, nu = sign * numerator_u;
    const long double d = std::abs(denominator);
    const long double comparison_pad =
        16 * std::numeric_limits<long double>::epsilon() * (std::abs(nt) + std::abs(nu) + d);
    if (nt < -error_t - comparison_pad || nt > d + error + error_t + comparison_pad ||
        nu < -error_u - comparison_pad || nu > d + error + error_u + comparison_pad)
      return;
  }
  // Resolve poorly conditioned intersection fractions exactly as well.
  // The fast branch's fraction error is far below the boundary tolerance.
  const long double fraction_resolution = std::abs(denominator) * 1e-14L;
  if (std::abs(denominator) > error && error < fraction_resolution &&
      error_t < fraction_resolution && error_u < fraction_resolution)
  {
    const long double t = numerator_t / denominator;
    const long double u = numerator_u / denominator;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
      result.push_back(static_cast<double>(t));
  }
  else
  {
    using Exact = boost::multiprecision::cpp_rational;
    const Exact edx = Exact(b.x) - Exact(a.x), edy = Exact(b.y) - Exact(a.y);
    const Exact eex = Exact(d.x) - Exact(c.x);
    const Exact eey = Exact(d.y) - Exact(c.y);
    const Exact eqx = Exact(c.x) - Exact(a.x), eqy = Exact(c.y) - Exact(a.y);
    const Exact determinant = edx * eey - edy * eex;
    if (determinant != 0)
    {
      const Exact t = (eqx * eey - eqy * eex) / determinant;
      const Exact u = (eqx * edy - eqy * edx) / determinant;
      if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        result.push_back(t.convert_to<double>());
    }
    else if (edx * edx + edy * edy > 0 && eqx * edy - eqy * edx == 0)
      for (auto p : {c, d})
      {
        const Exact t = ((Exact(p.x) - Exact(a.x)) * edx + (Exact(p.y) - Exact(a.y)) * edy) /
                        (edx * edx + edy * edy);
        result.push_back(std::clamp(t.convert_to<double>(), 0.0, 1.0));
      }
  }
}
inline void finishCuts(std::vector<double>& result)
{
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
}
inline std::vector<double> cuts(Point a, Point b, const std::vector<Ring>& rings)
{
  std::vector<double> result{0, 1};
  for (const auto& ring : rings)
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      appendEdgeCuts(a, b, ring[j], ring[i], result);
  finishCuts(result);
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
    if (a.x == b.x && a.y == b.y)
      return true;
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
