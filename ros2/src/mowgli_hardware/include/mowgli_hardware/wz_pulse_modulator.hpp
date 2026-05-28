// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// pulse_modulate_wz — sub-deadband angular-velocity pulse-width (duty-cycle)
// modulator for the cmd_vel → firmware path.
//
// WHY this exists:
//   The chassis has a PWM static-friction deadband on pure rotation
//   (~0.5 rad/s on PWM 40). Any commanded |wz| below it produces motor
//   buzz but no rotation, so the dock graceful-controller's fine heading
//   corrections (0.05-0.3 rad/s) never moved the robot and DockRobot
//   never settled.
//
//   The previous mitigation FLOORED every sub-deadband |wz| up to the
//   deadband amplitude. That over-rotates: measured on-robot 2026-05-27,
//   a commanded 0.3 rad/s produced 0.38-0.49 rad/s of actual yaw (127-164%)
//   because the firmware ran at the full deadband amplitude for the whole
//   command duration. The dock controller's fine corrections all jumped to
//   0.5 rad/s → overshoot → reverse → ping-pong, so DockRobot oscillated.
//
//   This modulator fixes the over-rotation by holding the AMPLITUDE at the
//   deadband (enough to break stiction) but cutting the DUTY CYCLE so the
//   time-average rate equals the commanded wz. duty = |wz_cmd| / deadband.
//   A sigma-delta / Bresenham accumulator integrates `duty` each tick and
//   fires a full-deadband pulse whenever it crosses 1.0, then subtracts 1.0.
//   Over many ticks the long-run average is exactly wz_cmd, and the pulses
//   are naturally spaced.
//
//   The gyro sees the pulses while the wheel encoders barely do; pre-
//   fusion_graph that wheel/IMU mismatch corrupted the localizer (which is
//   why an earlier pulse attempt, PR #221 / commit 00952173, was reverted in
//   09abe1ac). fusion_graph now slip-vetoes the mismatch in both the graph
//   between-factors and the dead-reckoning, so pulsing is safe again.
//
// This is a free function with no ROS dependency so it can be unit-tested
// in isolation. State (the accumulator) is owned by the caller.

#ifndef MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_
#define MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_

#include <cmath>

namespace mowgli_hardware
{

// Smallest commanded |wz| we treat as a real command; anything below is
// floating-point dust / rotate-to-heading-reached and maps to exactly 0.
inline constexpr double kWzMinCmdToConsider = 1.0e-3;

/// Pulse-width-modulate a sub-deadband angular-velocity command.
///
/// \param wz_cmd   commanded angular velocity (rad/s, signed).
/// \param deadband chassis pivot deadband / pulse amplitude (rad/s, > 0).
/// \param accum    sigma-delta accumulator, owned by the caller; carries
///                 phase between ticks. Reset to 0 on sign flip / return to 0.
/// \return the wz to actually send this tick:
///   - |wz_cmd| <= kWzMinCmdToConsider          → 0 (and accum reset to 0)
///   - |wz_cmd| >= deadband                      → wz_cmd unchanged (passthrough)
///   - otherwise                                 → ±deadband on the fraction of
///     ticks given by duty = |wz_cmd|/deadband, else 0; long-run average == wz_cmd.
///
/// \param burst_remaining  caller-owned counter for the minimum-burst-width
///                 logic; carries an in-progress burst between ticks.
/// \param min_burst_ticks  minimum number of CONSECUTIVE ticks a pulse must
///                 stay ON. A single-tick pulse (50-100 ms) is too short to
///                 overcome the chassis static friction — measured on-robot
///                 2026-05-27: a sub-deadband command pulsed one tick at a
///                 time produced gyro_z ≈ 0 (the robot never rotated, so RPP
///                 rotate-to-heading stalled and DockRobot failed "no
///                 progress"). Firing min_burst_ticks consecutive ticks gives
///                 the motors a sustained burst long enough to actually rotate
///                 the chassis; the OFF interval is stretched proportionally
///                 so the long-run average still equals wz_cmd. Must be >= 1.
inline double pulse_modulate_wz(double wz_cmd, double deadband, double& accum,
                                int& burst_remaining, int min_burst_ticks)
{
  const double mag = std::abs(wz_cmd);

  // Treat dust / rotate-to-heading-reached as a clean stop and drop phase
  // so the next command starts fresh.
  if (mag <= kWzMinCmdToConsider)
  {
    accum = 0.0;
    burst_remaining = 0;
    return 0.0;
  }

  // At or above the deadband the firmware can rotate on its own — pass
  // through unchanged. Drop phase so re-entering the sub-deadband regime
  // doesn't fire a stale pulse.
  if (deadband <= 0.0 || mag >= deadband)
  {
    accum = 0.0;
    burst_remaining = 0;
    return wz_cmd;
  }

  // A direction change must not carry stale phase, so reset when the sign of
  // the command disagrees with the sign held in the (signed) accumulator.
  if (accum != 0.0 && std::signbit(accum) != std::signbit(wz_cmd))
  {
    accum = 0.0;
    burst_remaining = 0;
  }

  const int w = (min_burst_ticks > 1) ? min_burst_ticks : 1;

  // "Debt" model: every tick we owe `duty` more on-ticks; firing pays them
  // back. Accrue every tick (including burst ticks) so the average is exact.
  const double duty = mag / deadband;  // in (0, 1)
  // Keep the accumulator signed so it doubles as the "last sign" memory.
  accum += std::copysign(duty, wz_cmd);

  // Continue an in-progress burst — already paid for when it started.
  if (burst_remaining > 0)
  {
    --burst_remaining;
    return std::copysign(deadband, wz_cmd);
  }

  // Start a new burst only once we've accrued a full burst's worth of debt,
  // so each pulse is >= w consecutive ticks. Pay w upfront; over a period of
  // w/duty ticks that yields w ON-ticks → on-fraction == duty (average exact).
  if (std::abs(accum) >= static_cast<double>(w))
  {
    accum -= std::copysign(static_cast<double>(w), wz_cmd);
    burst_remaining = w - 1;  // this tick + (w-1) more
    return std::copysign(deadband, wz_cmd);
  }
  return 0.0;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_
