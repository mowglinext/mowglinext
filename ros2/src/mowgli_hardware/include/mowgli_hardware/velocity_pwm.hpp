// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// velocity_pwm — static velocity (m/s) → signed PWM feedforward for the STM32
// drive base. Inverts the firmware's static-friction deadband + linear gain so
// the chained ros2_control pid_controller's integrator carries almost nothing.
//
//   pwm(v) = pwm_per_mps * v + sign(v) * deadband_pwm     (|v| >= v_eps)
//   pwm(0) = 0                                            (|v| <  v_eps)
//
// Pure / ROS-free so it is unit-testable.

#ifndef MOWGLI_HARDWARE__VELOCITY_PWM_HPP_
#define MOWGLI_HARDWARE__VELOCITY_PWM_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_hardware
{

/// Map a signed wheel speed (m/s) to a signed PWM, clamped to ±max_pwm. Below
/// v_eps (and for non-finite input, e.g. an unready controller) returns 0 so a
/// stopped/uninitialised wheel doesn't buzz against the deadband.
inline double velocity_to_pwm(
    double v_mps, double pwm_per_mps, double deadband_pwm, double max_pwm, double v_eps)
{
  if (!std::isfinite(v_mps) || std::abs(v_mps) < v_eps)
  {
    return 0.0;
  }
  const double sign = (v_mps > 0.0) ? 1.0 : -1.0;
  const double pwm = pwm_per_mps * v_mps + sign * deadband_pwm;
  return std::clamp(pwm, -max_pwm, max_pwm);
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__VELOCITY_PWM_HPP_
