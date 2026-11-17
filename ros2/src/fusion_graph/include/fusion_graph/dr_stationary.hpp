// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure dead-reckoning stationary hold for OnImu (fusion_graph_node_callbacks_a.cpp).
//
// The graph snaps a truly-stationary node's dθ to 0 (wheels still AND gyro below
// the hand-push threshold, graph_manager_node.cpp). Dead reckoning used to
// integrate the raw gyro regardless, so a robot parked off the dock drifted its
// odom→base yaw by the residual gyro bias and the graph pulled it back at every
// 5 s stationary node — a sawtooth in the published heading. The hold applies
// the SAME rule, in rate form, to the per-sample DR integration.

#pragma once

#include <cmath>

#include "fusion_graph/graph_params.hpp"

namespace fusion_graph
{

struct DrStationaryThresholds
{
  double wheel_speed_mps = 0.0;  // |wheel vx| below this = wheels still
  double wheel_yaw_rate_rad_per_s = 0.0;  // |wheel wz| below this = wheels still
  double gyro_rate_rad_per_s = 0.0;  // |gyro| above this = externally rotated
};

// Per-tick graph thresholds → rates. The graph compares one tick's accumulated
// wheel motion against stationary_thresh_*; at the tuned cadence that is a rate
// of threshold / kTunedNodePeriodS (the same tick_scale the graph applies).
inline DrStationaryThresholds DrStationaryFromGraphParams(const GraphParams& p)
{
  DrStationaryThresholds t;
  t.wheel_speed_mps = p.stationary_thresh_xy_m / kTunedNodePeriodS;
  t.wheel_yaw_rate_rad_per_s = p.stationary_thresh_theta / kTunedNodePeriodS;
  t.gyro_rate_rad_per_s = p.stationary_gyro_thresh_rad_per_s;
  return t;
}

// True when DR must not integrate this gyro sample: the wheels report no
// motion and the gyro is below the external-rotation threshold. A hand-pushed
// rotation (gyro above the threshold) still integrates, as in the graph.
inline bool DrHoldYaw(double wheel_vx,
                      double wheel_wz,
                      double gyro_z,
                      const DrStationaryThresholds& t)
{
  return std::abs(wheel_vx) < t.wheel_speed_mps &&
         std::abs(wheel_wz) < t.wheel_yaw_rate_rad_per_s &&
         std::abs(gyro_z) <= t.gyro_rate_rad_per_s;
}

}  // namespace fusion_graph
