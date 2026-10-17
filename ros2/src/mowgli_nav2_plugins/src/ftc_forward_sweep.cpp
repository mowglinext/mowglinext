// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/ftc_forward_sweep.hpp"

#include <cmath>
#include <limits>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace mowgli_nav2_plugins
{
bool ForwardSweepBlocked(const nav2_costmap_2d::Costmap2D& costmap,
                         const mowgli_interfaces::motion::Ring& footprint,
                         mowgli_interfaces::motion::Pose pose,
                         const geometry_msgs::msg::Twist& proposed,
                         const geometry_msgs::msg::Twist& measured,
                         double command_horizon,
                         double coast_time_bound)
{
  namespace motion = mowgli_interfaces::motion;
  namespace bg = boost::geometry;
  if (proposed.linear.x == 0 && proposed.angular.z == 0)
    return false;  // STOP unconditional.
  if (proposed.linear.x <= 0)
    return false;  // Reverse/rotation have their existing guards.
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(proposed.linear.x) || !std::isfinite(proposed.angular.z) ||
      !std::isfinite(measured.linear.x) || !std::isfinite(measured.angular.z) ||
      std::abs(measured.linear.x) > 2 || std::abs(measured.angular.z) > 4 ||
      proposed.linear.x > 2 || std::abs(proposed.angular.z) > 4 ||
      !std::isfinite(command_horizon) || command_horizon <= 0 || command_horizon > 1 ||
      !std::isfinite(coast_time_bound) || coast_time_bound < 0 || coast_time_bound > 2 ||
      footprint.size() < 3 || footprint.size() > 64)
    return true;
  double radius = 0;
  for (const auto& p : footprint)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      return true;
    radius = std::max(radius, std::hypot(p.x, p.y));
  }
  const auto body = motion::polygon(footprint);
  if (radius > 3 || !bg::is_valid(body) || std::abs(bg::area(body)) < 1e-10)
    return true;
  const double horizon = command_horizon + coast_time_bound;
  const double travel = std::max(proposed.linear.x, std::abs(measured.linear.x)) * horizon;
  const double low = std::min({0., proposed.angular.z, measured.angular.z}) * horizon;
  const double high = std::max({0., proposed.angular.z, measured.angular.z}) * horizon;
  const int n = std::max(1, static_cast<int>(std::ceil((high - low) / .02)));
  const double angle = (high - low) / n;
  // Every reference displacement is an integral of heading vectors in this
  // sector with total length<=travel. Every body yaw lies in the same interval.
  // The Minkowski sum below deliberately drops their time correlation.
  bg::model::multi_point<motion::BPoint> refs, bodies;
  refs.emplace_back(0, 0);
  for (int i = 0; i <= n; ++i)
  {
    const double yaw = pose.yaw + low + (high - low) * i / n;
    const double c = std::cos(yaw), s = std::sin(yaw);
    refs.emplace_back(travel * c, travel * s);
    if (measured.linear.x < 0)
      refs.emplace_back(-travel * c, -travel * s);
    for (const auto& p : footprint)
      bodies.emplace_back(p.x * c - p.y * s, p.x * s + p.y * c);
  }
  motion::Polygon ref_hull, body_hull;
  bg::convex_hull(refs, ref_hull);
  bg::convex_hull(bodies, body_hull);
  // Include the original reference/body samples too: a zero-angle reference
  // hull is degenerate, but its line segment is still part of the envelope.
  const double padding = (radius + travel) * angle * angle / 8 + 1e-12;
  bg::model::multi_point<motion::BPoint> vertices;
  for (const auto& a : refs)
    for (const auto& b : body_hull.outer())
      for (double dx : {-padding, padding})
        for (double dy : {-padding, padding})
          vertices.emplace_back(pose.x + bg::get<0>(a) + bg::get<0>(b) + dx,
                                pose.y + bg::get<1>(a) + bg::get<1>(b) + dy);
  motion::Polygon sweep;
  bg::convex_hull(vertices, sweep);
  bg::model::box<motion::BPoint> bounds;
  bg::envelope(sweep, bounds);
  const double res = costmap.getResolution(), ox = costmap.getOriginX(), oy = costmap.getOriginY();
  if (!std::isfinite(res) || !std::isfinite(ox) || !std::isfinite(oy) || res <= 0 ||
      costmap.getSizeInCellsX() == 0 || costmap.getSizeInCellsY() == 0)
    return true;
  const double x0 = bg::get<bg::min_corner, 0>(bounds), y0 = bg::get<bg::min_corner, 1>(bounds);
  const double x1 = bg::get<bg::max_corner, 0>(bounds), y1 = bg::get<bg::max_corner, 1>(bounds);
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1))
    return true;
  if (x0 < ox || y0 < oy || x1 >= ox + costmap.getSizeInCellsX() * res ||
      y1 >= oy + costmap.getSizeInCellsY() * res)
    return true;
  // Complete CLOSED cell squares, not cell centres or an outline raster.
  const auto first_x = static_cast<unsigned>(std::max(0., std::floor((x0 - ox) / res) - 1));
  const auto first_y = static_cast<unsigned>(std::max(0., std::floor((y0 - oy) / res) - 1));
  const auto last_x =
      std::min(costmap.getSizeInCellsX() - 1, static_cast<unsigned>(std::floor((x1 - ox) / res)));
  const auto last_y =
      std::min(costmap.getSizeInCellsY() - 1, static_cast<unsigned>(std::floor((y1 - oy) / res)));
  for (unsigned y = first_y; y <= last_y; ++y)
    for (unsigned x = first_x; x <= last_x; ++x)
      if (costmap.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        const bg::model::box<motion::BPoint> cell(motion::BPoint(ox + x * res, oy + y * res),
                                                  motion::BPoint(ox + (x + 1) * res,
                                                                 oy + (y + 1) * res));
        if (bg::intersects(sweep, cell))
          return true;
      }
  return false;
}
}  // namespace mowgli_nav2_plugins
