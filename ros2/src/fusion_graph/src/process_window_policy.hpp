// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "fusion_graph/graph_params.hpp"
#include "fusion_graph/slip_window.hpp"
#include <gtsam/geometry/Pose2.h>

namespace fusion_graph
{
// Values needed from one complete process window. Source integration and IMU
// preintegration remain with the caller; this selector only chooses policy.
struct ProcessWindowPolicyInput
{
  double dx = 0.0;
  double dy = 0.0;
  double wheel_travel_m = 0.0;
  double dtheta_wheel = 0.0;
  double dtheta_gyro = 0.0;
  double dt_total = 0.0;
  double max_abs_gyro_rad_per_s = 0.0;
  double last_node_time_s = 0.0;
  double now_s = 0.0;
};

// A value snapshot, not a second owner of GraphManager's live policy fields.
// Passing it by value permits private evaluation without advancing live state.
struct ProcessWindowPolicyState
{
  SlipWindow slip_window;
  double residual_ema = 0.0;
  uint64_t hand_push_count = 0;
  uint64_t slip_veto_count = 0;
  bool wheel_stationary_now = false;
  double last_wheel_sigma_x_eff = 0.0;
};

struct ProcessWindowPolicyResult
{
  gtsam::Pose2 fallback_mean;
  bool wheel_translation_enabled;
  bool gyro_rotation_enabled;
  bool hand_push_detected;
  bool slip_detected;
  double wheel_sigma_x;
  double wheel_sigma_y;
  double wheel_sigma_theta;
  ProcessWindowPolicyState state;
};

// Pure selection: no graph, source-history, accumulator, or live-state writes.
// The caller decides whether and when to adopt the proposed state.
ProcessWindowPolicyResult SelectProcessWindowPolicy(const GraphParams& params,
                                                    const ProcessWindowPolicyInput& input,
                                                    ProcessWindowPolicyState state);
}  // namespace fusion_graph
