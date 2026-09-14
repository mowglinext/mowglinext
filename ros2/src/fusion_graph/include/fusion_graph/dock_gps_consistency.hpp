// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure dock-prior vs RTK-Fixed GPS consistency check, factored out of the
// dock re-anchor path in OnGnss so it is unit-testable without ROS/GTSAM
// (same shape as rtk_wrongfix_gate.hpp; see fusion_graph_node_callbacks_a.cpp).
//
// While `is_charging`, fusion_graph re-asserts a firm prior (σ_xy 3 cm) at
// the operator-calibrated dock_pose once per node and suppresses live GPS
// factors. That is right when the receiver is off by the usual 5-30 cm
// between ambiguity sets — but on 2026-09-02 (issue #512) a ZED-F9P coming
// out of a power cycle reported RTK Fixed at 14-20 mm while placing the
// docked robot 1.88-2.45 m from dock_pose for ~45 min, and the prior pinned
// the pose silently: nothing was logged and the fused σ read 0.031 m.
//
// OPERATOR CONSTRAINT (issue #512, last comment): the dock is often under a
// terrace with NO RTK fix, and undocking is deliberately not gated on GPS
// quality. So this check yields the prior ONLY on a confident-but-
// contradicting fix — a FRESH RTK-Fixed sample whose reported σ is small AND
// whose position disagrees with dock_pose by more than the threshold. No
// fix, Float, or a large σ → the prior keeps pinning exactly as today.

#pragma once

#include <cmath>

namespace fusion_graph
{

// Map-frame point; a tiny value type so the helpers below stay pure.
struct MapXY
{
  double x = 0.0;
  double y = 0.0;
};

// Where the GNSS antenna sits in the map frame when base_link is at
// (dock_x, dock_y, dock_yaw): dock + R(yaw)·lever_arm. The GPS sample is an
// ANTENNA position (the graph's GnssLeverArmFactor applies the same rotation
// in its residual), so the disagreement must be measured antenna-to-antenna,
// not antenna-to-base_link — with a ~20-30 cm lever arm that alone would read
// as a spurious offset.
inline MapXY DockAntennaMapXY(
    double dock_x, double dock_y, double dock_yaw_rad, double lever_arm_x_m, double lever_arm_y_m)
{
  const double c = std::cos(dock_yaw_rad);
  const double s = std::sin(dock_yaw_rad);
  return MapXY{dock_x + c * lever_arm_x_m - s * lever_arm_y_m,
               dock_y + s * lever_arm_x_m + c * lever_arm_y_m};
}

// Planar distance (m) between where the dock says the antenna is and where
// the GPS sample puts it.
inline double DockGpsDisagreementM(double dock_x, double dock_y, double gps_x, double gps_y)
{
  return std::hypot(gps_x - dock_x, gps_y - dock_y);
}

// True ONLY when a fresh RTK-Fixed fix with a trustworthy-looking σ
// (0 < σ ≤ max_gps_sigma_m) disagrees with the dock by MORE than
// max_disagreement_m — i.e. the dock prior should yield for this node and
// the GPS sample be fused instead. Every other case returns false so the
// prior keeps pinning: no fix / Float (rtk_fixed_fresh=false), a large or
// unknown σ (the receiver is not claiming precision it lacks), a small
// disagreement (the normal 5-30 cm ambiguity-set shift), or either
// threshold ≤ 0 (the check is disabled).
inline bool DockPriorShouldYield(bool rtk_fixed_fresh,
                                 double gps_sigma_m,
                                 double disagreement_m,
                                 double max_disagreement_m,
                                 double max_gps_sigma_m)
{
  if (max_disagreement_m <= 0.0 || max_gps_sigma_m <= 0.0)
    return false;
  if (!rtk_fixed_fresh)
    return false;
  if (!std::isfinite(gps_sigma_m) || gps_sigma_m <= 0.0 || gps_sigma_m > max_gps_sigma_m)
    return false;
  if (!std::isfinite(disagreement_m))
    return false;
  return disagreement_m > max_disagreement_m;
}

}  // namespace fusion_graph
