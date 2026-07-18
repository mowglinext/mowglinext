// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Total-least-squares heading fit through a chronological sequence of GPS
// (x, y) samples. Extracted (task #47) from mowgli_behavior/calibration_
// nodes.cpp's CalibrateHeadingFromUndock, where it was the private
// fit_motion_yaw() helper — reused verbatim (same algorithm, same formulas)
// by mowgli_localization/calibrate_imu_yaw_node.cpp's dock-yaw reverse
// maneuver, which previously derived dock_pose_yaw from only the two
// trajectory ENDPOINTS (atan2 of net displacement). An endpoint-only
// bearing assumes a perfectly straight reverse; any curvature in the
// maneuver (e.g. the pre-#39/#37 yaw-loop wiggle) biases that single
// atan2 by a few degrees with no way to detect it. A line fit through
// every buffered sample is robust to that curvature and gives a tighter
// σ_yaw from the SAME baseline length (σ shrinks ~1/√n rather than being
// endpoint-limited) — both mowgli_localization and mowgli_behavior already
// depend on mowgli_interfaces, making it a natural shared home alongside
// gnss_status_utils.hpp and robot_yaml_scalar.hpp.

#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace mowgli_interfaces::motion_yaw_fit
{

// Fits a straight line through `s` (chronological (x, y) samples) via the
// principal axis of the centred 2×2 covariance, then resolves the ±π line
// ambiguity using the chronological displacement (first sample -> last).
// Returns (yaw, sigma_yaw): yaw is the MOTION direction (not yet flipped to
// a chassis heading — callers reversing must add π), sigma_yaw is the 1σ
// angular uncertainty derived from perpendicular residuals about the fitted
// line. Requires at least 2 samples; returns NaN/NaN otherwise.
inline std::pair<double, double> FitMotionYaw(const std::vector<std::pair<double, double>>& s)
{
  const std::size_t n = s.size();
  if (n < 2u)
  {
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  }

  double xsum = 0.0;
  double ysum = 0.0;
  for (const auto& p : s)
  {
    xsum += p.first;
    ysum += p.second;
  }
  const double xbar = xsum / static_cast<double>(n);
  const double ybar = ysum / static_cast<double>(n);

  double Sxx = 0.0;
  double Syy = 0.0;
  double Sxy = 0.0;
  for (const auto& p : s)
  {
    const double dx = p.first - xbar;
    const double dy = p.second - ybar;
    Sxx += dx * dx;
    Syy += dy * dy;
    Sxy += dx * dy;
  }

  // Principal axis of the centred 2×2 covariance: yaw = ½·atan2(2·Sxy, Sxx−Syy).
  // This gives the line direction up to a ±π ambiguity.
  double yaw = 0.5 * std::atan2(2.0 * Sxy, Sxx - Syy);

  // Resolve sign by chronological order: motion is from samples.front to
  // samples.back, dot with the current yaw vector must be positive.
  const double dx_chron = s.back().first - s.front().first;
  const double dy_chron = s.back().second - s.front().second;
  if (dx_chron * std::cos(yaw) + dy_chron * std::sin(yaw) < 0.0)
  {
    yaw += M_PI;
  }
  while (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  while (yaw < -M_PI)
    yaw += 2.0 * M_PI;

  // Perpendicular residuals → σ_yaw ≈ rms_perp / baseline.
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  double sum_perp2 = 0.0;
  for (const auto& p : s)
  {
    const double dx = p.first - xbar;
    const double dy = p.second - ybar;
    const double perp = -dx * sy + dy * cy;
    sum_perp2 += perp * perp;
  }
  const double rms_perp = std::sqrt(sum_perp2 / static_cast<double>(n));
  const double baseline = std::hypot(dx_chron, dy_chron);
  const double sigma_yaw = (baseline > 0.01) ? (rms_perp / baseline) : 0.1;
  return {yaw, sigma_yaw};
}

}  // namespace mowgli_interfaces::motion_yaw_fit
