// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#ifndef MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_
#define MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_nav2_plugins
{

/// Optional second costmap that confines a lateral-OFFSET deviation to the
/// mowing zone. When `costmap != nullptr`, an offset sample point is also
/// projected into this costmap's frame (via the affine below) and treated
/// as blocked if its cell is lethal — so an obstacle-clear side that would
/// skirt the robot OUT of the zone (zone boundary = lethal in the global
/// keepout costmap) is rejected. The guard applies ONLY to the offset
/// checks (chooseDeviationSide / growDeviationUntilClear's non-zero
/// deviation), never to the nominal-path or findFirstObstacle checks.
///
/// The affine maps the OFFSET-sample frame into this boundary costmap's
/// frame, e.g. boundary_frame (map) <- offset_frame:
///   bx = tx + cos_yaw * x - sin_yaw * y
///   by = ty + sin_yaw * x + cos_yaw * y
///
/// Defined at namespace scope (aliased as ObstacleDeviation::BoundaryGuard)
/// so the helper signatures can take `= {}` defaults — a nested struct with
/// default member initializers can't be value-initialised in a default
/// argument inside its own enclosing class.
struct BoundaryGuard
{
  const nav2_costmap_2d::Costmap2D* costmap{nullptr};
  double tx{0.0};
  double ty{0.0};
  double cos_yaw{1.0};
  double sin_yaw{0.0};
};

/// Pure-function helpers for the FTC controller's obstacle-deviation
/// behaviour. Kept separate from FTCController so they can be unit-tested
/// against a synthetic Costmap2D without spinning a full controller.
class ObstacleDeviation
{
public:
  /// Lethal-cost threshold (matches nav2_costmap_2d::LETHAL_OBSTACLE).
  static constexpr unsigned char kLethalThreshold = 253u;

  /// Zone-boundary guard for the lateral-OFFSET checks (see ::BoundaryGuard).
  using BoundaryGuard = ::mowgli_nav2_plugins::BoundaryGuard;

  /// Scan path poses [start_idx, start_idx + lookahead_count) and return the
  /// first index whose costmap cell is lethal. Returns -1 if none / costmap
  /// lookup fails. When `half_width > 0`, each pose is sampled across the robot
  /// body span (±half_width perpendicular to heading, spacing ≤ costmap
  /// resolution) so an off-centerline obstacle the chassis would hit is caught;
  /// `half_width == 0` keeps the legacy single-centerline sample.
  static int findFirstObstacleIndex(
      const nav2_costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::msg::PoseStamped>& path,
      std::size_t start_idx,
      int lookahead_count,
      double half_width = 0.0);

  /// Decide which side of `obstacle_pose` is free. Scans perpendicular to
  /// the obstacle's heading by `step` increments out to `max_search`.
  /// Returns the smallest signed offset (positive = left, negative = right)
  /// at which the projected point is in a non-lethal cell. Returns 0.0 if
  /// neither side is reachable within max_search (caller treats as "give up").
  /// When `guard.costmap != nullptr`, a side is only "free" if it is also
  /// inside the zone boundary (not lethal in the guard costmap).
  static double chooseDeviationSide(const nav2_costmap_2d::Costmap2D& costmap,
                                    const geometry_msgs::msg::PoseStamped& obstacle_pose,
                                    double max_search,
                                    double step,
                                    const BoundaryGuard& guard = {},
                                    double half_width = 0.0);

  /// Check whether the laterally-offset path is clear in the lookahead
  /// window. For each pose in [start_idx, start_idx + lookahead_count), the
  /// pose is shifted perpendicularly by `deviation` (positive = left of
  /// path heading) and the resulting cell is sampled. Returns true if no
  /// sampled cell is lethal. When `guard.costmap != nullptr`, an offset cell
  /// that is out-of-zone (lethal in the guard costmap) also counts as blocked.
  static bool isPathClearWithDeviation(const nav2_costmap_2d::Costmap2D& costmap,
                                       const std::vector<geometry_msgs::msg::PoseStamped>& path,
                                       std::size_t start_idx,
                                       int lookahead_count,
                                       double deviation,
                                       const BoundaryGuard& guard = {},
                                       double half_width = 0.0);

  /// Search for the smallest |deviation| that makes the path clear, starting
  /// from `initial_deviation` and growing in `step` increments up to
  /// `max_deviation`. Sign is preserved from `initial_deviation` (or chosen
  /// fresh by chooseDeviationSide if 0). Returns the chosen deviation, or
  /// the unchanged max-magnitude value if no clearance found (caller checks
  /// |result| > max_deviation - step). `guard` is forwarded to the offset
  /// clearance checks.
  static double growDeviationUntilClear(const nav2_costmap_2d::Costmap2D& costmap,
                                        const std::vector<geometry_msgs::msg::PoseStamped>& path,
                                        std::size_t start_idx,
                                        int lookahead_count,
                                        double initial_deviation,
                                        double max_deviation,
                                        double step,
                                        const BoundaryGuard& guard = {},
                                        double half_width = 0.0);
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__OBSTACLE_DEVIATION_HPP_
