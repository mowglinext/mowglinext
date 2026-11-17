// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
/// @file ring_repair.hpp
/// @brief Make a recorded ring valid before map_server publishes it as transit geometry.
///
/// /map_server_node/transit_geometry is parsed by the BT's coverage-entry check, the
/// transit goal checker and FTC's rotation gates (mowgli_nav2_plugins::MowingGeometry).
/// They reject the WHOLE snapshot when one ring is not boost::geometry-valid, fail
/// closed: no coverage entry is ever "contained" and no transit can ever arrive.
///
/// An operator-recorded boundary can carry a few-centimetre zigzag where the robot
/// went back and forth. On 2026-11-09 one such zigzag (4 vertices, a loop of ~0.3 m
/// perimeter) made the field boundary self-intersecting; every unit start then
/// needed a transit that could never end, and RotationShim spun the robot on the
/// spot until the transit bound cancelled it. The coverage planner already repairs
/// such rings for itself (coverage_planning.cpp, makeRingValid); this does the same
/// for the published geometry, so every consumer sees one valid ring.
///
/// The repair only removes SMALL loops: at a crossing of two non-adjacent edges the
/// shorter of the two chains between them is replaced by the crossing point, and
/// only when that chain (closed through the crossing point) is at most
/// kMaxRepairLoopPerimeterM long — the boundary moves by at most half of it. A
/// larger crossing is a genuinely broken ring that must not be guessed at: it is
/// returned unchanged and the consumers keep refusing it.
#ifndef MOWGLI_MAP__RING_REPAIR_HPP_
#define MOWGLI_MAP__RING_REPAIR_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <boost/geometry.hpp>

namespace mowgli_map
{

/// Largest loop (perimeter, m) removed at a self-crossing.
inline constexpr double kMaxRepairLoopPerimeterM = 0.5;

enum class RingRepair
{
  kAlreadyValid,  // published as recorded
  kRepaired,  // small loops removed, now valid
  kUnrepairable,  // still invalid: published as recorded, consumers refuse it
};

struct RingRepairResult
{
  geometry_msgs::msg::Polygon polygon;
  RingRepair status{RingRepair::kAlreadyValid};
  std::size_t loops_removed{0};
};

namespace ring_repair_detail
{

struct Xy
{
  double x;
  double y;
};

inline double cross(const Xy& o, const Xy& a, const Xy& b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/// Proper crossing of segments ab and cd (interiors cross); the crossing point.
inline std::optional<Xy> proper_crossing(const Xy& a, const Xy& b, const Xy& c, const Xy& d)
{
  const double d1 = cross(c, d, a);
  const double d2 = cross(c, d, b);
  const double d3 = cross(a, b, c);
  const double d4 = cross(a, b, d);
  if (!((d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0)) || d1 == 0.0 || d2 == 0.0 ||
      d3 == 0.0 || d4 == 0.0)
    return std::nullopt;
  const double t = d1 / (d1 - d2);
  return Xy{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

inline double length(const Xy& a, const Xy& b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

/// Open vertex list: no repeated closing vertex, no zero-length edge.
inline std::vector<Xy> open_vertices(const geometry_msgs::msg::Polygon& polygon)
{
  std::vector<Xy> out;
  for (const auto& p : polygon.points)
  {
    const Xy v{p.x, p.y};
    if (out.empty() || length(out.back(), v) > 1e-9)
      out.push_back(v);
  }
  while (out.size() > 1 && length(out.front(), out.back()) <= 1e-9)
    out.pop_back();
  return out;
}

inline bool is_valid(const std::vector<Xy>& ring)
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  bg::model::polygon<Point> shape;
  for (const auto& v : ring)
    shape.outer().emplace_back(v.x, v.y);
  bg::correct(shape);
  return ring.size() >= 3 && bg::is_valid(shape) && std::abs(bg::area(shape)) >= 1e-10;
}

/// Remove ONE small loop at the first self-crossing found. False when there is no
/// proper crossing, or the smaller side is too large to be a recording artefact.
inline bool remove_one_loop(std::vector<Xy>& ring)
{
  const std::size_t n = ring.size();
  for (std::size_t i = 0; i < n; ++i)
  {
    for (std::size_t j = i + 2; j < n; ++j)
    {
      if (i == 0 && j == n - 1)
        continue;  // adjacent through the closing edge
      const Xy& a = ring[i];
      const Xy& b = ring[(i + 1) % n];
      const Xy& c = ring[j];
      const Xy& d = ring[(j + 1) % n];
      const auto x = proper_crossing(a, b, c, d);
      if (!x)
        continue;
      // Inner chain: x -> ring[i+1] .. ring[j] -> x. Outer: x -> ring[j+1] .. ring[i] -> x.
      double inner = length(*x, ring[i + 1]) + length(ring[j], *x);
      for (std::size_t k = i + 1; k < j; ++k)
        inner += length(ring[k], ring[k + 1]);
      double outer = length(*x, d) + length(a, *x);
      for (std::size_t k = j + 1; k < n + i; ++k)
        outer += length(ring[k % n], ring[(k + 1) % n]);
      std::vector<Xy> next;
      if (inner <= outer)
      {
        if (inner > kMaxRepairLoopPerimeterM)
          return false;
        next.assign(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(i) + 1);
        next.push_back(*x);
        next.insert(next.end(), ring.begin() + static_cast<std::ptrdiff_t>(j) + 1, ring.end());
      }
      else
      {
        if (outer > kMaxRepairLoopPerimeterM)
          return false;
        next.push_back(*x);
        next.insert(next.end(),
                    ring.begin() + static_cast<std::ptrdiff_t>(i) + 1,
                    ring.begin() + static_cast<std::ptrdiff_t>(j) + 1);
      }
      ring = std::move(next);
      return true;
    }
  }
  return false;
}

}  // namespace ring_repair_detail

/// Return @p polygon made boost::geometry-valid by removing small self-crossing loops,
/// or unchanged (kAlreadyValid / kUnrepairable). Never drops the ring.
inline RingRepairResult repair_ring(const geometry_msgs::msg::Polygon& polygon)
{
  namespace d = ring_repair_detail;
  RingRepairResult result;
  result.polygon = polygon;
  auto ring = d::open_vertices(polygon);
  if (d::is_valid(ring))
    return result;
  // Each removal drops at least one vertex, so this terminates.
  while (ring.size() > 3 && !d::is_valid(ring) && d::remove_one_loop(ring))
    ++result.loops_removed;
  if (result.loops_removed == 0 || !d::is_valid(ring))
  {
    result.status = RingRepair::kUnrepairable;
    return result;
  }
  geometry_msgs::msg::Polygon repaired;
  for (const auto& v : ring)
  {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(v.x);
    p.y = static_cast<float>(v.y);
    repaired.points.push_back(p);
  }
  // Point32 is float: re-check what is actually published.
  if (!d::is_valid(d::open_vertices(repaired)))
  {
    result.status = RingRepair::kUnrepairable;
    return result;
  }
  result.status = RingRepair::kRepaired;
  result.polygon = std::move(repaired);
  return result;
}

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__RING_REPAIR_HPP_
