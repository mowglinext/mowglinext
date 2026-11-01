// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_nav2_plugins/ftc_motion_critic.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_nav2_plugins/ftc_motion_geometry_index.hpp"

namespace mowgli_nav2_plugins
{
LatticeGeometryBlocked MakeMotionLatticeCritic(
    std::shared_ptr<const mowgli_interfaces::motion::Snapshot> snapshot,
    mowgli_interfaces::motion::Ring footprint,
    mowgli_interfaces::motion::Pose costmap_in_map,
    std::shared_ptr<const MotionRegionIndex> prepared)
{
  namespace motion = mowgli_interfaces::motion;
  namespace bg = boost::geometry;
  double radius = 0;
  bool valid = snapshot && snapshot->valid && !snapshot->mowing.allowed.empty() &&
               footprint.size() >= 3 && footprint.size() <= 64 && std::isfinite(costmap_in_map.x) &&
               std::isfinite(costmap_in_map.y) && std::isfinite(costmap_in_map.yaw);
  for (const auto& p : footprint)
  {
    valid = valid && std::isfinite(p.x) && std::isfinite(p.y);
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  if (!valid || radius > 3)
    return [](const auto&, const auto&)
    {
      return true;
    };
  const auto body = motion::polygon(footprint);
  valid = valid && radius <= 3 && bg::is_valid(body) && std::abs(bg::area(body)) >= 1e-10;
  if (!valid)
    return [](const auto&, const auto&)
    {
      return true;
    };

  // Prepared state is reusable only for this exact immutable snapshot.
  if (!prepared || !prepared->matches(snapshot.get()))
    prepared = std::make_shared<MotionRegionIndex>(snapshot);

  auto query = prepared->newQuery();

  // Distant holes cannot intersect the swept chassis. Cache their boxes once
  // per decision; the exact polygon predicate still handles every near hole.
  std::vector<bg::model::box<motion::BPoint>> boxes;
  for (const auto& hole : snapshot->holes)
  {
    boxes.emplace_back();
    bg::envelope(hole, boxes.back());
  }
  return [snapshot = std::move(snapshot),
          query = std::move(query),
          footprint = std::move(footprint),
          boxes = std::move(boxes),
          costmap_in_map,
          radius](const auto& from, const auto& to)
  {
    const auto valid_pose = [](const auto& p)
    {
      const auto& q = p.pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y) &&
             std::isfinite(norm) && std::abs(norm - 1) <= 1e-6;
    };
    if (!valid_pose(from) || !valid_pose(to))
      return true;
    const double c = std::cos(costmap_in_map.yaw), s = std::sin(costmap_in_map.yaw);
    const auto in_map = [&](const auto& p)
    {
      return motion::Pose{costmap_in_map.x + c * p.pose.position.x - s * p.pose.position.y,
                          costmap_in_map.y + s * p.pose.position.x + c * p.pose.position.y,
                          costmap_in_map.yaw + tf2::getYaw(p.pose.orientation)};
    };
    const auto a = in_map(from), b = in_map(to);
    if (!std::isfinite(a.yaw) || !std::isfinite(b.yaw) ||
        !query->segmentAuthorized({a.x, a.y}, {b.x, b.y}))
      return true;
    std::vector<std::size_t> nearby;
    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
      const auto& box = boxes[i];
      // A disc around the reference chord contains the whole chassis, at all
      // interpolated headings. Add the same conservative roundoff as the hull.
      const double reach = radius + 1e-12;
      const double infinity = std::numeric_limits<double>::infinity();
      if (std::nextafter(std::max(a.x, b.x) + reach, infinity) < bg::get<bg::min_corner, 0>(box) ||
          std::nextafter(std::min(a.x, b.x) - reach, -infinity) > bg::get<bg::max_corner, 0>(box) ||
          std::nextafter(std::max(a.y, b.y) + reach, infinity) < bg::get<bg::min_corner, 1>(box) ||
          std::nextafter(std::min(a.y, b.y) - reach, -infinity) > bg::get<bg::max_corner, 1>(box))
        continue;
      nearby.push_back(i);
    }
    if (nearby.empty())
      return false;
    const double turn = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(turn) / 0.02)));
    const double angle = std::abs(turn) / steps;
    // With linear axle translation, only the body's angular arc deviates from
    // each endpoint chord: r*(1-cos(angle/2)) <= r*angle^2/8.
    const double padding = radius * angle * angle / 8 + 1e-12;
    for (int i = 0; i < steps; ++i)
    {
      bg::model::multi_point<motion::BPoint> vertices;
      for (int k : {i, i + 1})
      {
        const double f = static_cast<double>(k) / steps;
        const double yaw = a.yaw + turn * f;
        for (const auto& p : footprint)
        {
          const double x = a.x + f * (b.x - a.x) + p.x * std::cos(yaw) - p.y * std::sin(yaw);
          const double y = a.y + f * (b.y - a.y) + p.x * std::sin(yaw) + p.y * std::cos(yaw);
          for (double dx : {-padding, padding})
            for (double dy : {-padding, padding})
              vertices.emplace_back(x + dx, y + dy);
        }
      }
      motion::Polygon sweep;
      bg::convex_hull(vertices, sweep);
      for (const auto hole : nearby)
        if (bg::intersects(sweep, snapshot->holes[hole]))
          return true;
    }
    return false;
  };
}
}  // namespace mowgli_nav2_plugins
