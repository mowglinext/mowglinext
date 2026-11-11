// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/ftc_corner_geometry.hpp"

#include <cstdio>
#include <string>
#include <vector>

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
  if (!valid || !mowing.authorizedWithin(axle, entry_tolerance) || !std::isfinite(yaw) ||
      !std::isfinite(turn) || std::abs(turn) > 2 * M_PI || body.size() < 3 || body.size() > 64)
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

std::string SpliceAuthorityViolation(const CornerGeometry& geometry,
                                     const std::vector<PlanPose2D>& poses,
                                     std::size_t upto,
                                     const transit::Ring& body,
                                     double departure_cap_m)
{
  const std::vector<std::size_t> corners = FindPivotCorners(poses);
  if (!geometry.valid && !corners.empty() && corners.front() <= upto)
  {
    return "no mowing geometry (/map_server_node/transit_geometry) to execute its pivots against";
  }
  const auto where = [](const char* what, const PlanPose2D& p)
  {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s at (%.2f, %.2f)", what, p.x, p.y);
    return std::string(buf);
  };
  for (const std::size_t k : corners)
  {
    if (k > upto)
    {
      break;
    }
    const PlanPose2D& in = poses[k];
    const PlanPose2D& out = poses[k + 1];
    const transit::Point axle{in.x, in.y};
    if (!geometry.mowing.authorizedWithin(axle, geometry.entry_tolerance))
    {
      return where("pivot outside the mowing area", in);
    }
    const double turn = WrapPivotAngle(out.yaw - in.yaw);
    if (!geometry.rotationClear(axle, in.yaw, turn, body) &&
        !geometry.rotationClear(axle, in.yaw, turn - std::copysign(2.0 * M_PI, turn), body))
    {
      return where("pivot sweep crosses a drawn obstacle", in);
    }
    // The straight run after the corner, as rotationRestartPermitted measures it.
    const double c = std::cos(out.yaw);
    const double s = std::sin(out.yaw);
    double straight = 0.0;
    for (std::size_t i = k + 2; i < poses.size(); ++i)
    {
      const double dx = poses[i].x - out.x;
      const double dy = poses[i].y - out.y;
      const double along = dx * c + dy * s;
      const double lateral = -dx * s + dy * c;
      if (!std::isfinite(along) || std::abs(lateral) > 1e-6 || along + 1e-6 < straight ||
          std::abs(WrapPivotAngle(poses[i].yaw - out.yaw)) > 1e-6)
      {
        break;
      }
      straight = along;
    }
    const double run = std::min(std::max(0.0, departure_cap_m), straight);
    if (run > 0.0 && !geometry.mowing.segmentAuthorized(axle, {in.x + run * c, in.y + run * s}))
    {
      return where("departure after the pivot leaves the mowing area", in);
    }
  }
  return {};
}
}  // namespace mowgli_nav2_plugins
