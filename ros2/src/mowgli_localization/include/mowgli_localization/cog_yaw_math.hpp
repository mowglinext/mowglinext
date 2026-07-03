// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// cog_yaw_math.hpp
//
// Pure (ROS-free) core of the COG-to-body-heading computation used by
// cog_to_imu_node. Factored out of the node so the reverse-motion yaw
// algebra can be unit-tested in isolation — the node only wires up the
// GPS/wheel/IMU plumbing and calls compute_cog_body_yaw() / compute_cog_yaw_var().
//
// WHY this lives here: the on-robot 2026-05-27 reverse+rotate teleop
// (vx=-0.2, ω=+0.3 commanded; gyro steady +0.493 rad/s) showed the fused
// map-frame yaw diverging ~73° from the gyro because the lever-arm
// correction was applied in its forward form during reverse motion. The
// corrected reverse branch is small but easy to regress, so it gets a
// dedicated test (test/test_cog_yaw_math.cpp).

#ifndef MOWGLI_LOCALIZATION__COG_YAW_MATH_HPP_
#define MOWGLI_LOCALIZATION__COG_YAW_MATH_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_localization
{

// Wrap an angle to (-pi, pi].
inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Recover the body heading at the current sample from a GPS displacement
// baseline (dx, dy from anchor → current), the wheel direction, the
// time-averaged turn rate and the antenna lever arm.
//
// The antenna sits at r=(lever_arm_x, lever_arm_y) in body frame, so its
// body-frame velocity is (vx - ω·r_y, ω·r_x) with SIGNED vx. The GPS
// displacement points along the antenna's world-frame velocity at the
// baseline midpoint, i.e.
//   atan2(dy, dx) = ψ_mid + atan2(ω·r_x, vx - ω·r_y)
// where ψ_mid = ψ_current - ω·dt/2.
//
//   Forward (vx>0): atan2(ω·r_x, vx-ω·r_y) is a small first-quadrant offset.
//                   base_yaw = atan2(dy, dx); subtract the offset, add the
//                   half-baseline drift.
//   Reverse (vx<0): vx is negative, so atan2(ω·r_x, vx-ω·r_y) lands near
//                   ±π (the antenna velocity points roughly opposite the
//                   body-forward axis). base_yaw = atan2(-dy, -dx) already
//                   flips the displacement 180°, so the residual lever
//                   offset to remove is the SAME atan2 reduced about ±π
//                   (i.e. measured from the body's reverse-travel axis).
//
// WHY signed vx matters (2026-05-27 reverse-yaw incident): the previous
// code computed lever_corr = atan2(ω·r_x, +|vx| - ω·r_y) — the forward
// form — and subtracted it in reverse too. That left the published COG
// yaw ~73° off the gyro during reverse+rotate (the antenna offset is in
// the 2nd/3rd quadrant in reverse, not the 1st), so the COG unary factor
// fought the gyro between-factor in fusion_graph and the fused yaw ran
// off the wrong way. Forward behaviour is unchanged.
inline double compute_cog_body_yaw(double dx,
                                   double dy,
                                   int wheel_sign,
                                   double omega_avg,
                                   double dt_baseline,
                                   double vx_signed,
                                   double lever_arm_x,
                                   double lever_arm_y)
{
  const double drift_corr = omega_avg * dt_baseline * 0.5;
  // SIGNED vx: forward → small +offset; reverse → near ±π.
  const double lever_full = std::atan2(omega_avg * lever_arm_x, vx_signed - omega_avg * lever_arm_y);

  double base_yaw;
  double lever_corr;
  if (wheel_sign > 0)
  {
    base_yaw = std::atan2(dy, dx);
    lever_corr = lever_full;
  }
  else
  {
    base_yaw = std::atan2(-dy, -dx);
    // Reduce the near-±π reverse lever offset about π so it becomes the
    // small offset relative to the (already-flipped) reverse-travel axis,
    // then subtract it exactly like the forward branch.
    lever_corr = wrap_angle(lever_full - M_PI);
  }

  return wrap_angle(base_yaw + drift_corr - lever_corr);
}

// σ_yaw² contribution from the lever-arm correction's ω-sensitivity.
// ∂/∂ω atan2(ω·rx, v - ω·ry) = (rx·(v-ω·ry) + ω·rx·ry) / ((v-ω·ry)² + (ω·rx)²)
// Uses |vx| (effective speed magnitude) for the denominator regardless of
// travel direction — the |∂lever/∂ω| magnitude is symmetric in vx sign, so
// the σ inflation is identical forward and reverse.
inline double compute_lever_sigma(double omega_avg,
                                  double v_eff,
                                  double lever_arm_x,
                                  double lever_arm_y,
                                  double omega_noise_rps)
{
  const double denom_lever =
      std::pow(v_eff - omega_avg * lever_arm_y, 2.0) + std::pow(omega_avg * lever_arm_x, 2.0);
  const double dlever_domega =
      (lever_arm_x * (v_eff - omega_avg * lever_arm_y) + omega_avg * lever_arm_x * lever_arm_y) /
      std::max(denom_lever, 1e-6);
  return std::abs(dlever_domega) * omega_noise_rps;
}

// Lever-arm-sweep dominance gate.
//
// When the robot rotates, the GPS antenna (offset from base_link by the
// lever arm) sweeps an arc: its tangential speed is |omega| * lever_radius.
// If that sweep speed is large relative to the real chassis translation
// |vx|, the GPS displacement between fixes is dominated by the sweep, not by
// travel — so the course-over-ground heading atan2(dy,dx) points along the
// sweep tangent (~90° off the body heading), not where the robot is going.
// Publishing a COG yaw from such a sample injects a wrong absolute-heading
// unary into fusion_graph that fights the gyro.
//
// The fixed `min_omega_for_anchor` gate (reject if |omega| >= 0.5 rad/s)
// misses the docking case: the graceful controller's alignment pivots run at
// 0.1-0.3 rad/s — below 0.5 — yet with a 0.3 m lever arm that is already
// 0.03-0.09 m/s of sweep against ~0 real translation, so the COG is pure
// sweep garbage. Field 2026-05-27: those sub-threshold pivots corrupted the
// fused yaw and the dock approached from the wrong heading.
//
// Returns true when the sweep dominates (COG should be rejected):
//   |omega| * lever_radius > ratio * |vx|
// ratio ~1.0 means "reject once the antenna's rotational speed exceeds the
// chassis forward speed". Gentle curved driving (high vx, low omega) passes;
// in-place / rotation-dominant pivots (low vx, any omega) are rejected.
inline bool cog_sweep_dominates(double omega, double lever_radius, double vx,
                                double ratio)
{
  const double sweep_speed = std::abs(omega) * lever_radius;
  return sweep_speed > ratio * std::abs(vx);
}

// Deadbanded absolute-rotation increment for the stationary-latch staleness
// gate.
//
// The stationary yaw latch holds the heading from the last forward-motion
// segment and republishes it at 2 Hz while the robot is still. After an
// in-place pivot that latch is stale by the pivot angle, yet the
// instantaneous-omega republish gate passes again the moment the pivot stops.
// To catch this we integrate rotation since the latch was set and drop the
// latch past a threshold. The integral MUST be deadbanded: a plain integral of
// |gyro_z| accumulates the gyro noise floor (~0.01-0.03 rad/s) over a
// multi-second stationary dwell and would falsely invalidate a perfectly good
// latch. Below `deadband` the sample is treated as noise and contributes
// nothing; at or above it the true rotation |omega|*dt is accumulated. A real
// pivot (|omega| well over the deadband) trips the threshold within a fraction
// of a turn; a still robot never does.
inline double cog_latch_rotation_increment(double omega, double dt, double deadband)
{
  if (dt <= 0.0 || std::abs(omega) <= deadband)
  {
    return 0.0;
  }
  return std::abs(omega) * dt;
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__COG_YAW_MATH_HPP_
