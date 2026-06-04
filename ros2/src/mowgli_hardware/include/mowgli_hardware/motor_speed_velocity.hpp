// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// motor_speed_velocity — Tier-1 low-lag wheel-velocity estimate.
//
// At mowing speed the encoder yields only ~1 tick per firmware packet, so a
// tick-delta velocity is quantised to ~0.15 m/s steps; masking that needs a
// ~0.12 s EMA whose lag dominates the closed-loop dead time. The PAC5210 motor
// controller measures speed internally (its own fast commutation loop) far more
// smoothly, and the firmware forwards it (signed controller units) in the odom
// velocity_mm_s field. This helper converts that to m/s by AUTO-CALIBRATING a
// per-wheel scale against the (accurate-mean but quantised) tick-derived
// velocity: the slow EMA on the ratio averages out the tick quantisation, so
// the result is BOTH smooth (motor-speed shape) AND correctly scaled — without
// the EMA lag. host-owns-scale is preserved: ticks_per_meter still sets the
// true ground scale (it enters via meas_mps), and the controller units only
// supply the shape.
//
// Pure / ROS-free so it is unit-testable.

#ifndef MOWGLI_HARDWARE__MOTOR_SPEED_VELOCITY_HPP_
#define MOWGLI_HARDWARE__MOTOR_SPEED_VELOCITY_HPP_

#include <cmath>
#include <cstdint>

namespace mowgli_hardware
{

/// \param meas_mps        tick-derived wheel velocity this packet (m/s, signed) — accurate mean, quantised.
/// \param fallback_mps    velocity to return until the scale calibrates (the EMA'd tick velocity).
/// \param motor_speed_units  firmware-forwarded signed motor-controller speed (PAC5210 units).
/// \param scale           in/out: learned m/s per speed-unit (0 = uncalibrated; caller-owned, per wheel).
/// \param scale_alpha     EMA step for the scale update (small = slow, stable).
/// \return smooth velocity (m/s) once calibrated, else \p fallback_mps.
inline double motor_speed_velocity(double meas_mps, double fallback_mps, int16_t motor_speed_units,
                                   double& scale, double scale_alpha)
{
  // Calibrate only on clean, sign-agreeing, above-deadband samples — a stalled
  // / near-zero / direction-transient sample would corrupt the scale.
  if (std::abs(static_cast<int>(motor_speed_units)) >= 2 && std::abs(meas_mps) > 0.02 &&
      ((motor_speed_units > 0) == (meas_mps > 0)))
  {
    const double ratio = meas_mps / static_cast<double>(motor_speed_units);
    if (ratio > 0.0 && ratio < 1.0)  // sane: < 1 m/s per speed-unit
    {
      scale = (scale <= 0.0) ? ratio : scale + scale_alpha * (ratio - scale);
    }
  }
  if (scale > 0.0)
  {
    return scale * static_cast<double>(motor_speed_units);
  }
  return fallback_mps;  // not calibrated yet → caller's fallback (EMA'd tick velocity)
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__MOTOR_SPEED_VELOCITY_HPP_
