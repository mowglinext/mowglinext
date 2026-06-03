// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// wheel_rate_controller — per-wheel closed-loop velocity controller with online
// feedforward auto-calibration. This is the host-side replacement for the
// wheel-level PI loop that used to live in the STM32 firmware (motors_handler in
// cpp_main.cpp). Moving it to the host makes it tunable / observable without
// reflashing and lets it learn the drivetrain's feedforward curve online.
//
// WHAT IT DOES
//   Given a per-wheel target speed (m/s) and the firmware-reported measured
//   wheel speed (m/s), it produces a signed PWM command for that wheel:
//
//       pwm = kff * target + sign(target) * deadband     (feedforward)
//           + kp * (target - measured)                   (proportional)
//           + integral                                   (pre-scaled by ki)
//
//   The integral bridges the brushed-DC static-friction deadband exactly as the
//   firmware loop did: while the target says "move" and the encoder says
//   "stalled", ki * err * dt accumulates until the PWM crosses the mechanical
//   break-free point, the wheel starts turning, the error collapses, and the
//   integral settles at whatever PWM holds the target speed.
//
// ONLINE AUTO-CALIBRATION (the new part)
//   At steady state the integral holds the portion of the required PWM that the
//   feedforward failed to supply — i.e. a live measurement of the feedforward
//   error. Two slow estimators fold that knowledge back into the feedforward so
//   the loop converges toward feedforward-only (small, fast, latency-tolerant
//   feedback term):
//     * deadband  — captured from the break-free PWM at motion onset (EMA).
//     * kff (slope) — fitted from the steady-state output at cruise, holding the
//                     learned deadband, then the integral is UNLOADED by the same
//                     amount so the output is bump-less and the integral decays
//                     to ~0 instead of double-counting.
//   Adaptation is gated (steady error, above a min speed, sustained for a settle
//   window) and the caller suppresses it whenever it would be unsafe or
//   meaningless to learn (e.g. while charging / mechanically docked). The learned
//   (kff, deadband) per wheel are persisted by the bridge node, mirroring the IMU
//   calibration, so a calibrated robot starts calibrated.
//
// LATENCY NOTE
//   Unlike the firmware loop (zero-latency encoder read), this loop eats the USB
//   round-trip (~50-90 ms host<->STM32). That caps the usable feedback gains, so
//   defaults are gentle and the learned feedforward is expected to carry the
//   steady state. This is the same trade-off angular_rate_controller.hpp already
//   accepts; see its header for the broader rationale on host-side motor loops.
//
// Pure / ROS-free so it is unit-testable; all state is caller-owned.

#ifndef MOWGLI_HARDWARE__WHEEL_RATE_CONTROLLER_HPP_
#define MOWGLI_HARDWARE__WHEEL_RATE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_hardware
{

struct WheelRateParams
{
  // --- closed-loop gains ---
  // This per-wheel velocity loop must stay RESPONSIVE. The gyro angular-rate
  // controller (the outer loop, in the node) drives the wheel differential to
  // hit a target yaw rate and was tuned against a fast inner loop. If this loop
  // lags (e.g. a per-wheel slew limiter), the outer loop winds up and the
  // chassis oscillates violently — observed on-robot 2026-06 (gyro swung ±1.7
  // rad/s on a ±0.6 command, slipping and spiking localizer covariance). So
  // there is NO per-wheel slew limit here: feedforward + P respond to a stepped
  // target immediately. Linear over-acceleration is instead tamed UPSTREAM by
  // slew-limiting the LINEAR vx in the node (before the diff-drive split), which
  // does not slow rotation. Dead time (~50-90 ms USB round-trip) still caps the
  // integral gain, so ki is gentle and the integrator is frozen while the target
  // is actively changing (see target_settle_eps) to avoid winding on the lag.
  double kp = 30.0;          ///< proportional gain (PWM per m/s of error).
  double ki = 600.0;         ///< integral gain (PWM per m/s of error per second).
  double max_pwm = 255.0;    ///< output clamp (PAC5210 8-bit magnitude) + anti-windup ceiling.
  double integral_max = 60.0;   ///< |integral term| clamp (PWM) — anti-windup.
  double min_target = 1.0e-3;   ///< |target| below this → output 0, reset state.
  double zero_speed_eps = 0.02;  ///< |measured| below this counts as "stopped" (m/s).
  // Freeze the integrator while the commanded target is changing by more than
  // this per call (a node-level linear ramp, or a rotational step): during a
  // transient the tracking error is mostly the expected dead-time lag, and
  // integrating it would wind up and overshoot. ff + P handle the transient
  // (keeping the loop responsive); the integrator only trims the residual — and
  // bridges the static-friction deadband — once the target is steady. ≤0 → always integrate.
  double target_settle_eps = 0.005;

  // --- feedforward seed (refined online; persisted by the node) ---
  double kff_init = 300.0;       ///< PWM per m/s. Matches the firmware PWM_PER_MPS seed.
  double deadband_init = 40.0;   ///< break-free PWM seed (brushed-DC static friction).

  // --- online calibration (kff slope only; deadband is NOT learned) ---
  bool adapt_enabled = true;     ///< master enable; caller also gates on !charging etc.
  double learn_rate = 0.015;     ///< EMA step for the kff update (small = slow, stable).
  double learn_v_min = 0.12;     ///< only fit kff when |target| ≥ this (avoids low-speed mis-attribution).
  double err_tol = 0.03;         ///< |error| must be ≤ this (m/s) to count as "steady".
  double settle_s = 0.5;         ///< error must stay steady this long before adapting.
  double move_eps = 0.01;        ///< |measured| below this counts as "not moving".
  // Only learn while the integrator is NOT near its clamp. A near-saturated
  // integral means the loop is fighting hard (slip / oscillation / actuator
  // saturation) — not a clean steady state — so folding it into kff would
  // mis-learn (see the 2026-06 runaway). 0.8 blocks the top 20% (saturation)
  // while still allowing normal feedforward mismatches to be learned. Fraction
  // of integral_max.
  double learn_integral_frac = 0.8;
  // Bounds keep a bad learning episode from driving the feedforward to nonsense.
  double kff_min = 100.0;
  double kff_max = 600.0;
  // deadband is no longer learned online (it ran away to its clamp); these
  // bounds only clamp the seeded / persisted value on load.
  double deadband_min = 0.0;
  double deadband_max = 120.0;
};

struct WheelRateState
{
  // Closed-loop state.
  double integral = 0.0;        ///< accumulated (error·dt)·ki, in PWM units.
  double last_target = 0.0;     ///< previous commanded target (sign-flip + change detection).

  // Learned, persisted feedforward. Seeded from params on first use (kff==0
  // sentinel means "not initialised yet").
  double kff = 0.0;             ///< PWM per m/s (learned slope).
  double deadband = 0.0;        ///< break-free PWM (learned offset).

  // Adaptation bookkeeping.
  double settle_timer = 0.0;    ///< seconds the error has been continuously "steady".
  double last_measured = 0.0;   ///< previous measured speed (motion-onset edge detection).
};

/// Reset the runtime (integrator, settle) WITHOUT touching the learned
/// feedforward (kff/deadband). The node calls this on a hard stop (emergency /
/// cmd watchdog) so no stale integral carries across the stop.
inline void reset_wheel_runtime(WheelRateState& st)
{
  st.integral = 0.0;
  st.last_target = 0.0;
  st.settle_timer = 0.0;
}

namespace detail
{
inline double wheel_sign(double v)
{
  return (v > 0.0) ? 1.0 : ((v < 0.0) ? -1.0 : 0.0);
}
}  // namespace detail

/// Per-wheel closed-loop velocity command with online feedforward calibration.
///
/// \param target_mps   desired wheel speed (m/s, signed) — host diff-drive split output.
/// \param measured_mps firmware-reported wheel speed (m/s, signed).
/// \param dt           seconds since the previous call (clamped internally).
/// \param p            gains / limits / calibration knobs.
/// \param st           caller-owned integrator + learned feedforward (one per wheel).
/// \return signed PWM to send to the firmware for this wheel.
inline double compute_wheel_pwm(double target_mps, double measured_mps, double dt,
                                const WheelRateParams& p, WheelRateState& st)
{
  // Lazily seed the learned feedforward from params (kff==0 → uninitialised).
  if (st.kff == 0.0)
  {
    st.kff = p.kff_init;
    st.deadband = p.deadband_init;
  }

  const double dt_eff = (dt > 0.0 && dt < 1.0) ? dt : 0.02;

  // Explicit stop: target ~0. Reset the loop and command 0 (avoids a
  // stopped-wheel hum from a residual integral, same rule the firmware loop
  // used). The linear ramp lives upstream in the node, so a stop arrives here
  // as target ≈ 0 directly.
  if (std::abs(target_mps) <= p.min_target)
  {
    st.integral = 0.0;
    st.last_target = 0.0;
    st.settle_timer = 0.0;
    st.last_measured = measured_mps;
    return 0.0;
  }

  const double tgt = target_mps;

  // Is the commanded target actively changing (a node-level linear ramp, or a
  // rotational step)? Gates the integrator below. Evaluated before last_target
  // is updated. ≤0 eps disables the freeze (always integrate).
  const bool target_changing =
      p.target_settle_eps > 0.0 && std::abs(tgt - st.last_target) > p.target_settle_eps;

  // Direction reversal / stop-to-go: a stale integral from the opposite sign
  // would fight the new command. Drop it.
  if (st.last_target != 0.0 && std::signbit(st.last_target) != std::signbit(tgt))
  {
    st.integral = 0.0;
    st.settle_timer = 0.0;
  }
  st.last_target = tgt;

  const double error = tgt - measured_mps;
  const double sgn = detail::wheel_sign(tgt);

  // Integrate (pre-scaled to PWM) with anti-windup clamp — but NOT while the
  // commanded target is actively changing. During a ramp/step the tracking
  // error is dominated by the expected dead-time lag; integrating it winds up
  // and overshoots. ff + P (below) respond to the change immediately, so the
  // loop stays responsive (critical for the outer gyro loop); the integrator
  // only trims the residual once the target is steady — and bridges the
  // static-friction deadband when a steady low target leaves the wheel stalled.
  if (!target_changing)
  {
    st.integral += p.ki * error * dt_eff;
    st.integral = std::clamp(st.integral, -p.integral_max, p.integral_max);
  }

  // Feedforward + PI. The proportional term is separated out so the
  // calibrator can fold only the *persistent* (ff + integral) part.
  const double ff = st.kff * tgt + sgn * st.deadband;
  const double prop = p.kp * error;
  const double out_unclamped = ff + prop + st.integral;
  double out = std::clamp(out_unclamped, -p.max_pwm, p.max_pwm);

  // Back-calculation anti-windup: when the output saturates, bleed the integral
  // by the excess that never reached the motor so it can't keep winding past
  // what the actuator can deliver (and then overshoot on recovery).
  if (out != out_unclamped)
  {
    st.integral += (out - out_unclamped);
    st.integral = std::clamp(st.integral, -p.integral_max, p.integral_max);
  }

  // ----- online feedforward calibration (kff slope only) -----
  // REDESIGNED 2026-06 after the previous version mis-learned catastrophically:
  // it captured the "break-free PWM" from |output| at every motion onset, which
  // latched onto the LARGE output produced during the angular oscillation/slip
  // and ratcheted the deadband to its clamp (120) on both wheels → massive
  // over-drive and over-speed. Two changes make it robust:
  //   1. NO online deadband learning at all — deadband stays at the seeded /
  //      persisted value (a constant offset the integral trims around). Only
  //      the slope kff is learned.
  //   2. Learn ONLY from a *clean* steady state: error settled for settle_s,
  //      above a min speed, the target not changing, AND the integrator small
  //      (well within its clamp). A large integral means the loop is fighting
  //      (bad ff, slip, oscillation) — not a trustworthy sample to fold in.
  if (p.adapt_enabled)
  {
    const bool moving = std::abs(measured_mps) >= p.move_eps;

    // Steady-state gate. target_changing already excludes ramps/steps; require
    // small error, a meaningful speed, and actual motion.
    if (!target_changing && std::abs(error) <= p.err_tol && std::abs(tgt) >= p.learn_v_min &&
        moving)
    {
      st.settle_timer += dt_eff;
    }
    else
    {
      st.settle_timer = 0.0;
    }

    // Cruise slope fit — ONLY from a clean steady state (settled AND integral
    // well within its clamp). persistent = ff + integral (output minus the
    // transient P term) = the PWM truly needed. Fit kff to reproduce it holding
    // the fixed deadband, then unload the (already small) integral → bump-less.
    const bool clean_steady = st.settle_timer >= p.settle_s &&
                              std::abs(st.integral) < p.learn_integral_frac * p.integral_max;
    if (clean_steady)
    {
      const double persistent = out - prop;
      const double desired_kff = (std::abs(persistent) - st.deadband) / std::abs(tgt);
      st.kff += p.learn_rate * (desired_kff - st.kff);
      st.kff = std::clamp(st.kff, p.kff_min, p.kff_max);

      const double new_ff = st.kff * tgt + sgn * st.deadband;
      st.integral = std::clamp(persistent - new_ff, -p.integral_max, p.integral_max);
    }
  }

  st.last_measured = measured_mps;
  return out;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__WHEEL_RATE_CONTROLLER_HPP_
