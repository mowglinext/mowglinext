// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "mowgli_nav2_plugins/transit_geometry.hpp"
#include <boost/geometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace mowgli_nav2_plugins
{
// Complete MOW permission snapshot from map_server's existing publication.
// Outer rings grant permission; every raw obstacle/dock ring forbids it.
struct MowingGeometry
{
  bool valid{false};
  transit::Geometry mowing;

  /// `tolerance`: coverage_entry_tolerance_m (0 keeps the exact recorded line).
  bool contains(transit::Point axle, double tolerance = 0.0) const
  {
    return valid && mowing.authorizedWithin(axle, tolerance) && mowing.clear(axle);
  }

  static MowingGeometry parse(const visualization_msgs::msg::MarkerArray& message)
  {
    namespace bg = boost::geometry;
    using Point = bg::model::d2::point_xy<double>;
    using Polygon = bg::model::polygon<Point>;
    MowingGeometry next;
    if (message.markers.empty() || message.markers.size() > 512)
      return next;
    std::size_t count = 0;
    for (const auto& m : message.markers)
    {
      if (m.header.frame_id != "map" || m.action != m.ADD || m.type != m.LINE_STRIP ||
          m.points.size() < 3 || (count += m.points.size()) > 32768 || !std::isfinite(m.scale.x) ||
          m.scale.x < 0)
        return {};
      const auto& p = m.pose;
      if (p.position.x != 0 || p.position.y != 0 || p.position.z != 0 || p.orientation.x != 0 ||
          p.orientation.y != 0 || p.orientation.z != 0 ||
          (p.orientation.w != 0 && p.orientation.w != 1))
        return {};
      transit::Ring ring;
      for (const auto& v : m.points)
      {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || std::abs(v.x) > 1e6 ||
            std::abs(v.y) > 1e6 || v.z != 0)
          return {};
        ring.push_back({v.x, v.y});
      }
      Polygon shape;
      for (const auto& p : ring)
        shape.outer().emplace_back(p.x, p.y);
      bg::correct(shape);
      if (!bg::is_valid(shape) || std::abs(bg::area(shape)) < 1e-10)
        return {};
      if (m.ns == "area")
      {
        if (m.text != "mowing" && m.text != "navigation")
          return {};
        if (m.text == "mowing")
          next.mowing.allowed.push_back(std::move(ring));
      }
      else if (m.ns == "obstacle" || m.ns == "dock_body" || m.ns == "peer_dock")
        next.mowing.blocked.push_back(std::move(ring));
      else if (m.ns != "dock_corridor")
        return {};
    }
    next.valid = !next.mowing.allowed.empty();
    return next;
  }
};
}  // namespace mowgli_nav2_plugins
