// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/ftc_corner_geometry.hpp"

#include "mowgli_nav2_plugins/mowing_geometry.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace mowgli_nav2_plugins
{
namespace
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
Polygon polygon(const transit::Ring& ring)
{
  Polygon result;
  for (const auto& p : ring)
    result.outer().emplace_back(p.x, p.y);
  bg::correct(result);
  return result;
}
}  // namespace

CornerGeometry CornerGeometry::parse(const visualization_msgs::msg::MarkerArray& message)
{
  auto parsed = MowingGeometry::parse(message);
  CornerGeometry next;
  next.valid = parsed.valid;
  next.mowing = std::move(parsed.mowing);
  return next;
}

bool CornerGeometry::stoppingNeighborhoodClear(transit::Point axle,
                                               double radius,
                                               const transit::Ring& body) const
{
  if (!valid || !std::isfinite(axle.x) || !std::isfinite(axle.y) || !std::isfinite(radius) ||
      radius < 0 || body.size() < 3 || body.size() > 64)
    return false;
  double reach = 0;
  for (const auto& p : body)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      return false;
    reach = std::max(reach, std::hypot(p.x, p.y));
  }
  const auto shape = polygon(body);
  if (reach > 3 || !bg::is_valid(shape) || std::abs(bg::area(shape)) < 1e-10 ||
      !std::isfinite(radius + reach))
    return false;
  bool interior = false;
  for (const auto& ring : mowing.allowed)
  {
    if (!transit::inside(axle, ring))
      continue;
    bool clear = true;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      clear = clear && transit::edgeDistance(axle, ring[j], ring[i]) > radius + transit::kEpsilon;
    interior = interior || clear;
  }
  if (!interior)
    return false;
  // The whole translated/rotated filled chassis lies in this expanded disk.
  // ringDistance also returns zero when a blocked ring contains the axle.
  for (const auto& hole : mowing.blocked)
    if (!(transit::ringDistance(axle, hole) > radius + reach + transit::kEpsilon))
      return false;
  return true;
}

std::optional<double> CornerGeometry::stoppingBudget(transit::Point axle,
                                                     double requested,
                                                     const transit::Ring& body) const
{
  if (!valid || !std::isfinite(axle.x) || !std::isfinite(axle.y) || !std::isfinite(requested) ||
      requested <= 0 || body.size() < 3 || body.size() > 64)
    return std::nullopt;
  double reach = 0;
  for (const auto& p : body)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      return std::nullopt;
    reach = std::max(reach, std::hypot(p.x, p.y));
  }
  if (reach > 3)
    return std::nullopt;
  double clearance = 0;
  for (const auto& ring : mowing.allowed)
  {
    if (!transit::inside(axle, ring))
      continue;
    double ring_clearance = INFINITY;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      ring_clearance = std::min(ring_clearance, transit::edgeDistance(axle, ring[j], ring[i]));
    clearance = std::max(clearance, ring_clearance);
  }
  for (const auto& hole : mowing.blocked)
    clearance = std::min(clearance, transit::ringDistance(axle, hole) - reach);
  if (!std::isfinite(clearance) || !(clearance > transit::kEpsilon))
    return std::nullopt;
  const double budget = std::min(requested, std::nextafter(clearance - transit::kEpsilon, 0.0));
  // Subtraction/addition rounding can lose the strict clearance. Revalidate
  // once with the unchanged whole-disk proof; never iterate toward permission.
  if (!std::isfinite(budget) || !(budget > 0) || budget > requested ||
      !stoppingNeighborhoodClear(axle, budget, body))
    return std::nullopt;
  return budget;
}

bool CornerGeometry::rotationClear(transit::Point axle,
                                   double yaw,
                                   double turn,
                                   const transit::Ring& body) const
{
  if (!valid || !mowing.authorized(axle) || !std::isfinite(yaw) || !std::isfinite(turn) ||
      std::abs(turn) > 2 * M_PI || body.size() < 3 || body.size() > 64)
    return false;
  double reach = 0;
  for (const auto& p : body)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      return false;
    reach = std::max(reach, std::hypot(p.x, p.y));
  }
  if (reach > 3)
    return false;
  const auto shape = polygon(body);
  if (!bg::is_valid(shape) || std::abs(bg::area(shape)) < 1e-10)
    return false;
  // The signed interval is directed, including the opposite arc longer than
  // pi. Do not wrap it: the complete selected arc must pass the same proof.
  // Reuse the rotation-only portion of the proven chord-hull construction:
  // every body point's arc lies within r*angle^2/8 of its endpoint chord.
  // The convex hull covers the filled chassis, including holes wholly inside it.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / .02)));
  const double angle = std::abs(turn) / steps;
  const double padding = reach * angle * angle / 8 + 1e-12;
  for (int i = 0; i < steps; ++i)
  {
    bg::model::multi_point<Point> cloud;
    for (int k : {i, i + 1})
    {
      const double a = yaw + turn * k / steps;
      for (const auto& p : body)
        for (double dx : {-padding, padding})
          for (double dy : {-padding, padding})
            cloud.emplace_back(axle.x + p.x * std::cos(a) - p.y * std::sin(a) + dx,
                               axle.y + p.x * std::sin(a) + p.y * std::cos(a) + dy);
    }
    Polygon hull;
    bg::convex_hull(cloud, hull);
    for (const auto& hole : mowing.blocked)
      if (bg::intersects(hull, polygon(hole)))
        return false;
  }
  return true;
}
}  // namespace mowgli_nav2_plugins
