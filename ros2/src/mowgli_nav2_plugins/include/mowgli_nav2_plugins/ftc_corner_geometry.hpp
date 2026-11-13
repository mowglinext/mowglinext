// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "mowgli_nav2_plugins/ftc_pivot.hpp"
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

/// Why a runtime splice (turn fallback, contour bypass) could not be executed,
/// or an empty string. A splice with no pivot corner up to `upto` passes
/// whatever the geometry: it goes straight to FOLLOWING, no rotation gate. Every pivot corner of
/// `poses[0 .. upto]` is checked the way the PIVOT state will check it before rotating and before
/// driving off: mowing authority at the axle, the body's sweep against the raw drawn holes on the
/// short OR the opposite arc (the rotation gate tries both; like any pivot, the body may overhang
/// the recorded line), and the straight departure after it (capped at
/// `departure_cap_m`, the avoidance horizon) inside the mowing area. The fallback planners only see
/// lethal local cells and the global band; a plan they accept that fails these checks stops in
/// PIVOT until goal_timeout and aborts beside the obstacle, so it is refused here instead.
std::string SpliceAuthorityViolation(const CornerGeometry& geometry,
                                     const std::vector<PlanPose2D>& poses,
                                     std::size_t upto,
                                     const transit::Ring& body,
                                     double departure_cap_m);
}  // namespace mowgli_nav2_plugins
