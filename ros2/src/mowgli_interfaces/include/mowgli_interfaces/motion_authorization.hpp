// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "mowgli_interfaces/motion_geometry_primitives.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mowgli_interfaces::motion
{
namespace bg = boost::geometry;
using BPoint = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<BPoint>;
using MultiPolygon = bg::model::multi_polygon<Polygon>;

inline Polygon polygon(const Ring& ring)
{
  Polygon result;
  for (const auto& p : ring)
    result.outer().emplace_back(p.x, p.y);
  bg::correct(result);
  return result;
}

struct Pose
{
  double x, y, yaw;
};

// Immutable complete snapshot of #905's polygon publication. Stamps describe
// delivery, not geometry identity. Empty/invalid snapshots revoke permission.
struct Snapshot
{
  bool valid{false};
  std::string identity;
  Geometry transit;
  Geometry mowing;
  MultiPolygon transit_union;
  MultiPolygon mowing_union;
  std::vector<Polygon> holes;

  static std::shared_ptr<const Snapshot> parse(const visualization_msgs::msg::MarkerArray& msg)
  {
    auto next = std::make_shared<Snapshot>();
    if (msg.markers.empty() || msg.markers.size() > 512)
      return next;
    std::ostringstream identity;
    identity << std::setprecision(17);
    std::size_t count = 0;
    for (const auto& marker : msg.markers)
    {
      if (marker.header.frame_id != "map" || marker.action != marker.ADD ||
          marker.type != marker.LINE_STRIP || marker.points.size() < 3 ||
          (count += marker.points.size()) > 32768 || !std::isfinite(marker.scale.x) ||
          marker.scale.x < 0)
        return next;
      Ring ring;
      identity << marker.ns << ':' << marker.text << ':' << marker.scale.x << ';';
      for (const auto& p : marker.points)
      {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 1e6 ||
            std::abs(p.y) > 1e6 || p.z != 0)
          return next;
        ring.push_back({p.x, p.y});
        identity << p.x << ',' << p.y << ';';
      }
      auto poly = polygon(ring);
      if (!bg::is_valid(poly) || std::abs(bg::area(poly)) < 1e-10)
        return next;
      if (marker.ns == "area")
      {
        if (marker.text != "mowing" && marker.text != "navigation")
          return next;  // Coverage must not inherit navigation-area permission.
        next->transit.allowed.push_back(ring);
        if (marker.text == "mowing")
          next->mowing.allowed.push_back(ring);
      }
      else if (marker.ns == "dock_corridor")
        next->transit.allowed.push_back(ring);
      else if (marker.ns == "obstacle")
        next->holes.push_back(std::move(poly));
      else
        return next;
    }
    const auto unite = [](const std::vector<Ring>& rings)
    {
      MultiPolygon result;
      for (const auto& ring : rings)
      {
        auto poly = polygon(ring);
        if (result.empty())
          result.push_back(std::move(poly));
        else
        {
          MultiPolygon combined;
          bg::union_(result, poly, combined);
          result = std::move(combined);
        }
      }
      return result;
    };
    next->transit_union = unite(next->transit.allowed);
    next->mowing_union = unite(next->mowing.allowed);
    next->valid = !next->transit_union.empty() && bg::is_valid(next->transit_union) &&
                  (next->mowing_union.empty() || bg::is_valid(next->mowing_union));
    next->identity = identity.str();
    return next;
  }

  // Constant body-frame Twist integrated about the rear axle. The chord hull
  // of each angular interval is enlarged by a proved arc-to-chord bound, NOT
  // merely sampled at poses. Every point swept by the convex chassis lies in
  // this envelope, including thin holes and collisions between endpoint bodies.
  // Reference-point permission is separate: default edge coverage permits body
  // overhang outside outer rings, but never body intersection with a raw hole.
  bool permits(
      Pose start, const Ring& footprint, double v, double w, double seconds, bool coverage) const
  {
    if (!valid || !std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
        !std::isfinite(v) || !std::isfinite(w) || !std::isfinite(seconds) || seconds < 0 ||
        seconds > 30 || footprint.size() < 3 || footprint.size() > 64 || std::abs(v) > 2 ||
        std::abs(w) > 4)
      return false;
    const auto fp = polygon(footprint);
    if (!bg::is_valid(fp) || std::abs(bg::area(fp)) < 1e-10)
      return false;
    double radius = 0;
    for (const auto& p : footprint)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y))
        return false;
      radius = std::max(radius, std::hypot(p.x, p.y));
    }
    if (radius > 3)
      return false;
    const auto& region = coverage ? mowing : transit;
    const auto& united = coverage ? mowing_union : transit_union;
    if (!region.authorized({start.x, start.y}))
      return false;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(w) * seconds / 0.02)));
    const auto pose = [&](double t)
    {
      const double turn = w * t;
      // sinc form avoids catastrophic cancellation for near-zero omega.
      const double half = turn / 2;
      const double travel = v * t * (std::abs(half) < 1e-8 ? 1 : std::sin(half) / half);
      return Pose{start.x + travel * std::cos(start.yaw + half),
                  start.y + travel * std::sin(start.yaw + half),
                  start.yaw + turn};
    };
    const auto hull = [](const Ring& points, double padding)
    {
      bg::model::multi_point<BPoint> cloud;
      for (const auto& p : points)
        for (double dx : {-padding, padding})
          for (double dy : {-padding, padding})
            cloud.emplace_back(p.x + dx, p.y + dy);
      Polygon result;
      bg::convex_hull(cloud, result);
      return result;
    };
    for (int i = 0; i < steps; ++i)
    {
      const Pose a = pose(seconds * i / steps), b = pose(seconds * (i + 1) / steps);
      const double angle = std::abs(w) * seconds / steps;
      // 1-cos(angle/2) <= angle^2/8, including rounding conservatism.
      // Axle radius is |v/w|; writing |v|*dt*angle/8 avoids division by tiny w.
      const double axle_pad = std::abs(v) * (seconds / steps) * angle / 8;
      if (axle_pad == 0)
      {
        if (!region.segmentAuthorized({a.x, a.y}, {b.x, b.y}))
          return false;
      }
      else
      {
        MultiPolygon outside;
        bg::difference(hull({{a.x, a.y}, {b.x, b.y}}, axle_pad + 1e-12), united, outside);
        if (!outside.empty())
          return false;
      }
      Ring vertices;
      for (const Pose p : {a, b})
        for (const auto& q : footprint)
          vertices.push_back({p.x + q.x * std::cos(p.yaw) - q.y * std::sin(p.yaw),
                              p.y + q.x * std::sin(p.yaw) + q.y * std::cos(p.yaw)});
      const auto sweep = hull(vertices, axle_pad + radius * angle * angle / 8 + 1e-12);
      for (const auto& hole : holes)
        if (bg::intersects(sweep, hole))
          return false;
    }
    return true;
  }
};
}  // namespace mowgli_interfaces::motion
