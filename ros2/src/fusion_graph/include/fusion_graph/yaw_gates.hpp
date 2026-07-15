// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure yaw-robustness gates (Level 1 COG discipline + Level 2 LiDAR yaw yield),
// factored out of the node so they are unit-testable without ROS/GTSAM.
// See fusion_graph_node.hpp for the design rationale. The essence, borrowed from
// OpenMower's xbot_positioning: absolute yaw from a single antenna is weakly
// observable, so the GPS course-over-ground (COG) may only NUDGE a
// gyro-integrated heading, and only when the fix is RTK-Fixed AND the robot is
// translating forward above a speed floor; otherwise the gyro carries yaw.

#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

// Whether a COG (GPS-course) yaw factor should be applied this sample.
// Before the graph is initialized the COG is always accepted (the initial yaw
// seed needs it). Once initialized it must be RTK-Fixed-fresh (if require_rtk)
// and the robot must be moving forward above min_speed (COG is undefined at
// rest, noisy when slow, 180°-flipped in reverse where wheel_vx < 0).
inline bool CogShouldApply(bool initialized, bool rtk_fresh, double wheel_vx,
                           bool require_rtk, double min_speed_mps)
{
  if (!initialized)
    return true;  // seeding: never gate
  if (require_rtk && !rtk_fresh)
    return false;
  if (wheel_vx < min_speed_mps)
    return false;  // rest / slow / reverse → gyro carries yaw
  return true;
}

// Soft COG σ: never tighter than the floor, so COG trends the gyro heading
// rather than snapping to a noisy per-fix course.
inline double CogEffectiveSigma(double msg_variance, double min_sigma_rad)
{
  double var = msg_variance;
  if (!std::isfinite(var) || var <= 0.0)
    var = 0.05 * 0.05;
  return std::max(std::sqrt(var), min_sigma_rad);
}

// LiDAR (scan-match / loop-closure) yaw σ: floored so LiDAR yields yaw to the
// gyro (can't bake a wrong heading), while its position σ stays tight.
inline double ScanYawSigma(double raw_sigma_theta, double floor_rad)
{
  return std::max(raw_sigma_theta, floor_rad);
}

}  // namespace fusion_graph
