// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0

#ifndef MOWGLI_BEHAVIOR__DOCK_ALIGNMENT_HPP_
#define MOWGLI_BEHAVIOR__DOCK_ALIGNMENT_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_behavior
{

// ── Why this exists ────────────────────────────────────────────────────────
// Docking can fail in a way that leaves NO evidence anywhere. opennav_docking
// logs "Made contact with dock" from SimpleChargingDock::isDocked(), which is a
// pure POSITION test against docking_threshold and ignores yaw entirely — so a
// seat that is laterally offset reports contact exactly like a good one, and
// the only downstream symptom is v_charge staying 0.0.
//
// Issue #486 (2026-08-24) is what that costs: reconstructing one failed dock
// took an encoder-tick reconstruction across two log files, and the answer —
// the approach was NOT short, it drove 1.551 m and a SUCCESSFUL approach the
// same day drove 1.560 m — only fell out at the very end. Nine millimetres of
// depth separated a failure from a success, which means the difference was
// lateral, and nothing in the system had ever written down a lateral number.
//
// The two helpers here are the two numbers that were missing. Both are pure so
// they can be unit-tested; both are diagnostics — nothing reads their output
// except a log line.

/// Where the robot came to rest relative to the dock, in the DOCK's own axes.
struct DockContactDelta
{
  /// Positive INTO the cradle. Negative means the robot stopped short.
  double along_m{0.0};
  /// Positive to the robot's left as it faces the dock. This is the one that
  /// was missing: issue #446 reports "always about 10 cm too far left".
  double cross_m{0.0};
  /// Straight-line distance, i.e. what isDocked() compares to docking_threshold.
  double range_m{0.0};
};

/// Decomposes a map-frame robot position into along/cross-track offsets from
/// the dock pose. `dock_yaw` points INTO the dock (the robot's heading when
/// seated), matching dock_pose_yaw in mowgli_robot.yaml.
inline DockContactDelta ComputeDockContactDelta(
    double robot_x, double robot_y, double dock_x, double dock_y, double dock_yaw)
{
  const double dx = robot_x - dock_x;
  const double dy = robot_y - dock_y;
  const double c = std::cos(dock_yaw);
  const double s = std::sin(dock_yaw);
  return DockContactDelta{dx * c + dy * s, -dx * s + dy * c, std::hypot(dx, dy)};
}

/// Distance opennav_docking projects BACK along dock_pose_yaw to build the dock
/// staging pose — mirrors `staging_x_offset` in nav2_params_base.yaml. Used only
/// to express a yaw error as the lateral miss it causes at the START of the
/// approach; keep the two in step if that offset is ever retuned.
inline constexpr double kDockStagingRunwayM = 1.5;

/// Band floor, in radians. Mirrors the default `dock_pose_yaw_sigma_rad`
/// (0.035 rad = 2.0°) in mowgli_robot.yaml — the confidence the persisted dock
/// pose declares in its own yaw. Inside that band the undock fit and the
/// persisted value are the same measurement taken twice.
inline constexpr double kDockYawDeclaredSigmaRad = 0.035;

/// Verdict on whether the persisted dock yaw still matches the measured one.
struct DockYawDrift
{
  /// True when the disagreement clears the band, i.e. is worth reporting.
  bool is_stale{false};
  /// measured − persisted, wrapped to (−π, π].
  double delta_rad{0.0};
  /// The threshold `delta_rad` had to clear.
  double band_rad{0.0};
  /// Lateral miss this yaw error causes at the staging pose, in metres.
  double staging_lateral_m{0.0};
};

/// Compares a freshly measured dock axis against the persisted `dock_pose_yaw`.
///
/// The undock BackUp is a straight reverse out of the cradle, so the chassis
/// heading fitted from it IS the dock axis — which makes it a free, per-undock
/// check on a value nothing else ever re-validates (the YAML writeback was
/// removed for the Invariant-6 single-writer collapse, so a stale value can sit
/// there indefinitely). It is not free of consequence: dock_pose_yaw places the
/// staging pose, so a yaw error becomes a LATERAL miss of runway·sin(Δ) at the
/// start of every approach, and the graceful controller does not reliably null
/// cross-track (see the controller.k_phi field notes in nav2_params_base.yaml).
///
/// A disagreement counts only once it clears BOTH the fit's own uncertainty
/// (3σ) and the persisted pose's declared σ, so a noisy endpoint-fallback
/// undock cannot cry wolf.
inline DockYawDrift EvaluateDockYawDrift(double measured_yaw,
                                         double persisted_yaw,
                                         double sigma_yaw)
{
  double delta = measured_yaw - persisted_yaw;
  while (delta > M_PI)
    delta -= 2.0 * M_PI;
  while (delta <= -M_PI)
    delta += 2.0 * M_PI;

  DockYawDrift out{};
  out.delta_rad = delta;
  out.band_rad = std::max(3.0 * std::fabs(sigma_yaw), kDockYawDeclaredSigmaRad);
  out.is_stale = std::fabs(delta) > out.band_rad;
  out.staging_lateral_m = std::fabs(kDockStagingRunwayM * std::sin(delta));
  return out;
}

}  // namespace mowgli_behavior

#endif  // MOWGLI_BEHAVIOR__DOCK_ALIGNMENT_HPP_
