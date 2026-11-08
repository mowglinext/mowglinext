// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "mowgli_nav2_plugins/transit_geometry.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace mowgli_nav2_plugins
{
// Coverage rotation/departure input from the existing complete map publication.
// Navigation areas never grant mowing permission; blocked rings remain RAW.
struct CornerGeometry
{
  bool valid{false};
  transit::Geometry mowing;
  /// coverage_entry_tolerance_m, set by FTC after parse: how far outside the
  /// recorded line the axle still has mowing authority (the planner grows the
  /// outermost ring's field outward by op_width/2). 0 = exact line.
  double entry_tolerance{0.0};
  static CornerGeometry parse(const visualization_msgs::msg::MarkerArray& message);
  // Sufficient interior certificate for a full radial stopping budget. Outer
  // permission constrains the axle; RAW holes constrain the filled body at any yaw.
  bool stoppingNeighborhoodClear(transit::Point axle,
                                 double radius,
                                 const transit::Ring& body) const;
  // Derive a conservative budget from one mowing ring and RAW body clearance,
  // then certify the entire chosen disk. A failed certificate has no budget.
  std::optional<double> stoppingBudget(transit::Point axle,
                                       double requested,
                                       const transit::Ring& body) const;
  bool rotationClear(transit::Point axle, double yaw, double turn, const transit::Ring& body) const;
};
}  // namespace mowgli_nav2_plugins
