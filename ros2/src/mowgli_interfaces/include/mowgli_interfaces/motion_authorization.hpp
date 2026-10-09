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

  // Split a short circular axle arc at every polygon supporting-line crossing.
  // tan(theta/2) gives a quadratic without constructing a huge circle centre
  // when omega is tiny. Extra crossings outside an edge merely add intervals.
  // Unlike a padded chord, this accepts inward arcs starting on the perimeter.
  static bool arcAuthorized(const Geometry& region, Pose start, double v, double w, double dt)
  {
    const long double turn = static_cast<long double>(w) * dt;
    const auto point = [&](double fraction)
    {
      const double half = static_cast<double>(turn * fraction / 2);
      const double travel = v * dt * fraction * (std::abs(half) < 1e-8 ? 1 : std::sin(half) / half);
      return Point{start.x + travel * std::cos(start.yaw + half),
                   start.y + travel * std::sin(start.yaw + half)};
    };
    if (!region.authorized(point(0)) || !region.authorized(point(1)))
      return false;
    std::vector<double> fractions{0, 1};
    const long double fx = std::cos(start.yaw), fy = std::sin(start.yaw);
    for (const auto& ring : region.allowed)
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
      {
        const long double ex = ring[i].x - ring[j].x, ey = ring[i].y - ring[j].y;
        const long double dx = start.x - ring[j].x, dy = start.y - ring[j].y;
        const long double c = (ex * dy - ey * dx) * static_cast<long double>(w) / v;
        const long double a = 2 * (ex * fx + ey * fy) + c;
        const long double b = 2 * (ex * fy - ey * fx);
        const auto add = [&](long double root)
        {
          const long double fraction = 2 * std::atan(root) / turn;
          if (std::isfinite(fraction) && fraction > 0 && fraction < 1)
            fractions.push_back(static_cast<double>(fraction));
        };
        if (a == 0)
        {
          if (b != 0)
            add(-c / b);
        }
        else
        {
          const long double discriminant = b * b - 4 * a * c;
          if (discriminant >= 0)
          {
            const long double q = -0.5L * (b + std::copysign(std::sqrt(discriminant), b));
            if (q != 0)
            {
              add(q / a);
              add(c / q);
            }
            else
              add(-b / (2 * a));
          }
        }
      }
    std::sort(fractions.begin(), fractions.end());
    for (std::size_t i = 1; i < fractions.size(); ++i)
      if (!region.authorized(point((fractions[i - 1] + fractions[i]) / 2)))
        return false;
    return true;
  }

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
      // This transport carries map-coordinate points, not marker-local shapes.
      // Accept the publisher's identity pose and legacy unset quaternion only.
      const auto& pose = marker.pose;
      if (pose.position.x != 0 || pose.position.y != 0 || pose.position.z != 0 ||
          pose.orientation.x != 0 || pose.orientation.y != 0 || pose.orientation.z != 0 ||
          (pose.orientation.w != 0 && pose.orientation.w != 1))
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
      if (v != 0 && axle_pad == 0)
      {
        if (!region.segmentAuthorized({a.x, a.y}, {b.x, b.y}))
          return false;
      }
      else if (v != 0)
      {
        if (!arcAuthorized(region, a, v, w, seconds / steps))
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
