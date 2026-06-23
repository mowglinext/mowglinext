// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Small pure-math helper for the mobile RTK wrong-fix gate. The caller compares
// the new fix against the LAST ACCEPTED GNSS reference, while still tracking
// raw-fix cadence separately for diagnostics. Kept header-only so unit tests
// can exercise the policy without spinning a ROS node.

#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

enum class GnssMobileGateDecision
{
  kAccepted,
  kDownweighted,
  kRejected,
};

inline const char* GnssMobileGateDecisionToString(const GnssMobileGateDecision decision)
{
  switch (decision)
  {
    case GnssMobileGateDecision::kAccepted:
      return "accepted";
    case GnssMobileGateDecision::kDownweighted:
      return "downweighted";
    case GnssMobileGateDecision::kRejected:
      return "rejected";
  }
  return "accepted";
}

struct GnssMobileGateParams
{
  double gps_sigma_multiplier{2.0};
  double min_margin_m{0.01};
  double downweight_innovation_multiplier{2.0};
};

struct GnssMobileGateMetrics
{
  double dt_gps_s{0.0};
  double cmd_vx_mps{0.0};
  double wheel_delta_m{0.0};
  double cmd_delta_m{0.0};
  double delta_gps_m{0.0};
  double expected_motion_m{0.0};
  double gps_sigma_xy{0.0};
  double allowed_delta_m{0.0};
  double innovation_m{0.0};
  double forward_innovation_m{0.0};
  double lateral_innovation_m{0.0};
  GnssMobileGateDecision decision{GnssMobileGateDecision::kAccepted};
};

inline GnssMobileGateMetrics EvaluateGnssMobileGate(double dt_gps_s,
                                                    double cmd_vx_mps,
                                                    double cmd_delta_m,
                                                    double wheel_delta_m,
                                                    double delta_gps_x_m,
                                                    double delta_gps_y_m,
                                                    double gps_sigma_xy,
                                                    double heading_yaw_rad,
                                                    const GnssMobileGateParams& params)
{
  GnssMobileGateMetrics m;
  m.dt_gps_s = std::max(dt_gps_s, 0.0);
  m.cmd_vx_mps = cmd_vx_mps;
  m.cmd_delta_m = std::max(cmd_delta_m, 0.0);
  m.wheel_delta_m = std::max(wheel_delta_m, 0.0);
  m.delta_gps_m = std::hypot(delta_gps_x_m, delta_gps_y_m);
  m.expected_motion_m = std::max(m.wheel_delta_m, m.cmd_delta_m);
  m.gps_sigma_xy = std::max(gps_sigma_xy, 0.0);

  const double noise_budget = m.gps_sigma_xy * std::max(params.gps_sigma_multiplier, 0.0);
  const double accept_gate = noise_budget + std::max(params.min_margin_m, 0.0);
  const double downweight_gate =
      accept_gate * std::max(params.downweight_innovation_multiplier, 1.0);

  m.allowed_delta_m = m.expected_motion_m + accept_gate;
  m.innovation_m = std::abs(m.delta_gps_m - m.expected_motion_m);

  const double hx = std::cos(heading_yaw_rad);
  const double hy = std::sin(heading_yaw_rad);
  const double delta_forward = delta_gps_x_m * hx + delta_gps_y_m * hy;
  const double delta_lateral = -delta_gps_x_m * hy + delta_gps_y_m * hx;
  m.forward_innovation_m = std::abs(std::abs(delta_forward) - m.expected_motion_m);
  m.lateral_innovation_m = std::abs(delta_lateral);

  const bool accept = m.innovation_m <= accept_gate && m.lateral_innovation_m <= accept_gate;
  const bool downweight =
      m.innovation_m <= downweight_gate && m.lateral_innovation_m <= downweight_gate;

  if (accept)
  {
    m.decision = GnssMobileGateDecision::kAccepted;
  }
  else if (downweight)
  {
    m.decision = GnssMobileGateDecision::kDownweighted;
  }
  else
  {
    m.decision = GnssMobileGateDecision::kRejected;
  }

  return m;
}

}  // namespace fusion_graph
