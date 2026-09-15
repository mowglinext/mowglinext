// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <cmath>

namespace mowgli_hardware
{

// Slowest wheel speed the drive loop must still be able to turn: the inner
// wheel of a 0.20 m coverage arc at mowing speed. The firmware PI is applied
// UNSCALED in PWM units, so at this speed the feedforward alone (pwm_per_mps ×
// 0.03 ≈ 8.5 PWM at the template scale) sits far below the ~40 PWM
// static-friction deadband and the integrator has to make up the difference.
inline constexpr double kDriveGainProbeSpeedMps = 0.03;

// Highest PWM the firmware wheel loop can reach at kDriveGainProbeSpeedMps:
// feedforward + full integrator + the P term at the same error. If this stays
// below the motor's stiction deadband, the wheel never turns at that speed —
// the field failure of 2026-09-15 (kp 0.2 / ki 0.092 / integral_limit 15 gave
// a ceiling of ~24 PWM, 15.6 % of moving time with one wheel stalled).
inline constexpr double DriveGainsBridgePwm(double kp, double integral_limit, double pwm_per_mps)
{
  return integral_limit + kp * kDriveGainProbeSpeedMps + pwm_per_mps * kDriveGainProbeSpeedMps;
}

// True when the gain set can push a wheel through the deadband. NaN/inf in any
// input is rejected (false) so a corrupted parameter can never pass the gate.
inline bool DriveGainsBridgeDeadband(double kp,
                                     double integral_limit,
                                     double pwm_per_mps,
                                     double deadband_pwm)
{
  if (!std::isfinite(kp) || !std::isfinite(integral_limit) || !std::isfinite(pwm_per_mps) ||
      !std::isfinite(deadband_pwm))
  {
    return false;
  }
  return DriveGainsBridgePwm(kp, integral_limit, pwm_per_mps) >= deadband_pwm;
}

}  // namespace mowgli_hardware
