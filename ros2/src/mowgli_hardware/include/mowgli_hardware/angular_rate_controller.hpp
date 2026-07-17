// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// angular_rate_controller — closed-loop angular-rate (yaw) controller for the
// cmd_vel → firmware path.
//
// WHY this exists (the long version, because four prior open-loop attempts
// failed here and the next person needs to know not to repeat them):
//
//   The firmware's PWM→rotation response is NONLINEAR and load-dependent.
//   Measured on-robot 2026-05-27 with the command passed straight through:
//     commanded 0.2 rad/s → 0.07 actual (36%)
//     commanded 0.3 rad/s → 0.22 actual (74%)
//     commanded 0.4 rad/s → 0.30 actual (75%)
//   i.e. a soft deadband plus a ~0.7-0.75 gain that itself varies with
//   speed, battery voltage and grass load. Every Nav2 controller (RPP,
//   graceful-dock, Spin) assumes commanded ω == actual ω, so this mismatch
//   shows up as under-rotation, stalls ("Failed to make progress" during
//   dock approach), and — when an earlier fix over-corrected — left/right
//   oscillation.
//
//   Open-loop amplitude surgery cannot fix a nonlinear, drifting curve:
//     * floor-to-0.85 boost (5e62bda2)         → 2.3x over-rotation, oscillation
//     * pulse modulation (PR #221/#223)        → broke MPPI, reverted (#225)
//     * floor-to-0.5 clamp (c61b2eb8)          → over-rotation
//     * pulse + min-burst (39198702/9b667013)  → stall / 17% under-rotation
//   All of them guess at the inverse curve; none measures the result.
//
//   This controller closes the loop on the GYRO (the honest chassis-rotation
//   sensor — immune to the wheel slip that corrupts encoder-based feedback).
//   A PI on (target − measured) drives the firmware command until the
//   measured yaw rate equals the target, automatically absorbing the
//   nonlinear gain and the soft deadband: the integral winds up until the
//   output crosses the deadband (the "kick" emerges from the loop), then
//   settles at whatever command yields the target rate. Because it tracks
//   the *measured* rate it is correct at every operating point without a
//   hand-tuned curve. The transient wheel/IMU disagreement the kick creates
//   is absorbed by fusion_graph's slip-veto (graph + dead-reckoning), which
//   is why this is safe now though the closed-loop boost (2a371798) had to
//   be dropped before the slip-veto existed.
//
//   USB round-trip latency (~50-90 ms host↔STM32) caps the usable gains;
//   defaults are deliberately gentle. For a 0.3 m/s mower that is plenty.
//
// ============================================================================
// 2026-07-17 WEAVE FIX — Option B (task #24, USER-APPROVED sign-off on the
// task #8 design proposal). Left-right weave on straights/U-turns was traced
// to a 3-loop integrating cascade with no bandwidth separation: FTC heading
// PID -> this host omega-PI (ki=2.0) -> two independent per-wheel firmware
// PIs. The host integrator wound up/down on the ~50-90 ms USB round trip and
// fought the wheel loops. Option B removes that integrator (ki: 2.0 -> 0.0)
// and refits the feed-forward gain (kff: 1.0 -> 1.35, from the measured
// curve above) to do the deadband + gain compensation open-loop instead.
// kp is unchanged and now purely damps.
//
// *** UNVALIDATED ON HARDWARE. THIS CHANGES PHYSICAL STEERING ON A BLADED
// ROBOT. Do not deploy blade-on until the full validation sequence from the
// task #8 proposal has run, in order:
//   1. Bench step-response: re-run the inverse-curve measurement (the
//      commanded/actual table above) against the new kff and confirm the
//      fit; refit kff again if the curve has drifted.
//   2. Blade-OFF straight-line weave amplitude vs the ki=2.0 baseline.
//   3. Blade-OFF coverage swaths including U-turns.
//   4. Blade-OFF dock approach x5 (this is the loop that produces the
//      dither the target low-pass exists for — watch for regressions there).
//   5. Only then: blade-ON, supervised.
// If any step regresses, the safe rollback is params-only: set
// angular_rate_ki back to 2.0 (and angular_rate_kff back to 1.0) via the
// hardware_bridge_node parameter — no rebuild required. ***
// ============================================================================
//
// Pure / ROS-free so it is unit-testable; all state is caller-owned.

#ifndef MOWGLI_HARDWARE__ANGULAR_RATE_CONTROLLER_HPP_
#define MOWGLI_HARDWARE__ANGULAR_RATE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_hardware
{

struct AngularRateParams
{
  // 2026-07-17 WEAVE FIX (Option B, task #24, USER-APPROVED, UNVALIDATED ON
  // HARDWARE — see the big warning below): ki defaults to 0 and kff is
  // refit toward the measured inverse gain. Root cause (task #8 design
  // proposal): this host PI, the FTC heading PID, and the two independent
  // per-wheel firmware PIs formed a 3-loop integrating cascade with no
  // bandwidth separation — the host integrator (ki=2.0) wound up and down on
  // a ~50-90 ms USB round trip, fighting the wheel loops and producing the
  // left-right weave. Removing the host integrator (ki=0) collapses the
  // cascade to 2 loops; the feed-forward term (kff) now does the deadband
  // + gain compensation open-loop, refit from the measured curve in the
  // header comment above (0.72-0.75 gain in the 0.2-0.4 rad/s band →
  // inverse ≈ 1.33-1.35). kp stays small and now purely damps.
  //
  // Prior gentle defaults (ki=2.0, kff=1.0) exploited USB round-trip latency
  // (~50-90 ms ≈ 1-2 cmd ticks) plus the firmware's hard sub-deadband edge:
  // a large kp would limit-cycle around the deadband, so most of the work
  // was done by the integrator (smooth, zero steady-state error) with kp
  // only damping. That mechanism is still available (set ki>0 to re-enable
  // it) but is no longer the default.
  double kff = 1.35;         ///< feed-forward: baseline command = kff * target.
  double kp = 0.4;           ///< proportional gain on (target - measured) rad/s.
  double ki = 0.0;           ///< integral gain (1/s); 0 = feedforward-dominant (Option B).
  double max_cmd = 1.5;      ///< output clamp (rad/s), also anti-windup ceiling.
  double integral_max = 1.5; ///< |integral term| clamp (rad/s) — anti-windup.
  double min_target = 1.0e-3;///< |target| below this → output 0, reset state.
  // Low-pass time constant (s) on the TARGET before the PI. The dock
  // graceful controller emits small left-right alternating heading
  // corrections; feeding those raw makes the PI wind up to break the
  // deadband, overshoot the tiny target, the graceful loop fights back,
  // and the result is left-right pulsing that biases the heading
  // (user-observed 2026-05-27, absent during mowing where RPP commands a
  // smooth same-sign wz). Low-passing the target extracts the NET intent
  // (the dither averages out) so the PI tracks a smooth command. For a
  // sustained command the filter settles to it (no steady-state effect),
  // so mowing is essentially unchanged. 0 disables the filter.
  double target_lp_tau = 0.2;
};

struct AngularRateState
{
  double integral = 0.0;        ///< accumulated (error·dt), rad/s after ki applied.
  double last_target = 0.0;     ///< for sign-flip detection (uses filtered target).
  double filtered_target = 0.0; ///< low-pass state for the target wz.
};

/// Closed-loop angular-rate command.
///
/// \param target_wz   desired yaw rate from the controller (rad/s, signed).
/// \param measured_wz bias-corrected gyro_z (rad/s, signed).
/// \param dt          seconds since the previous call (clamped internally).
/// \param p           gains / limits.
/// \param st          caller-owned integrator + sign memory.
/// \return the yaw-rate command to forward to the firmware (rad/s).
inline double compute_angular_rate_cmd(double target_wz, double measured_wz, double dt,
                                       const AngularRateParams& p, AngularRateState& st)
{
  // Guard dt against pauses / first call.
  const double dt_eff = (dt > 0.0 && dt < 1.0) ? dt : 0.05;

  // Hard zero on the RAW target = explicit stop: reset everything (including
  // the filter) and return 0 immediately, so a "done" command stops the
  // chassis promptly rather than coasting through the low-pass tail.
  if (std::abs(target_wz) <= p.min_target)
  {
    st.integral = 0.0;
    st.last_target = 0.0;
    st.filtered_target = 0.0;
    return 0.0;
  }

  // Low-pass the target to extract the net intent from dithering corrections
  // (see AngularRateParams::target_lp_tau). tau<=0 disables (passthrough).
  if (p.target_lp_tau > 0.0)
  {
    const double alpha = dt_eff / (p.target_lp_tau + dt_eff);
    st.filtered_target += alpha * (target_wz - st.filtered_target);
  }
  else
  {
    st.filtered_target = target_wz;
  }
  const double tgt = st.filtered_target;

  // Direction change on the FILTERED target: a stale integrator from the
  // opposite spin would fight the new command. Drop it. Using the filtered
  // target means micro-dither (which barely moves the filtered sign) no
  // longer triggers a reset every tick.
  if (st.last_target != 0.0 && std::signbit(st.last_target) != std::signbit(tgt))
  {
    st.integral = 0.0;
  }
  st.last_target = tgt;

  const double error = tgt - measured_wz;

  // Integrate the error (units rad/s · s = rad), scaled by ki to rad/s, and
  // clamp for anti-windup so a stalled chassis can't accumulate an unbounded
  // command that then overshoots when traction returns.
  st.integral += p.ki * error * dt_eff;
  st.integral = std::clamp(st.integral, -p.integral_max, p.integral_max);

  double out = p.kff * tgt + p.kp * error + st.integral;
  out = std::clamp(out, -p.max_cmd, p.max_cmd);
  return out;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__ANGULAR_RATE_CONTROLLER_HPP_
