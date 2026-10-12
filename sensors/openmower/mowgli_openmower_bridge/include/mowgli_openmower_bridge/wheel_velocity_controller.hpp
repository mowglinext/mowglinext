// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file wheel_velocity_controller.hpp
 * @brief Per-wheel velocity loop closing on the xESC tachometer.
 *
 * The Mowgli STM32 closes the wheel-velocity loop in firmware, so the rest of
 * MowgliNext (Nav2, FTC) commands metres per second and expects them to be
 * tracked. OpenMower's controllers only take a duty cycle and OpenMower's own
 * comms node maps m/s to duty 1:1, open loop. This PI + feed-forward loop
 * restores velocity tracking on the host. With kp = ki = 0 it degrades to
 * OpenMower's open-loop mapping (`duty = duty_per_mps * target`).
 *
 * Pure and header-only: no time source, no I/O.
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_openmower_bridge
{

struct WheelLoopGains
{
  double duty_per_mps{1.0};  ///< feed-forward: OpenMower's 1 m/s == 100 % duty
  double kp{0.5};  ///< [duty per m/s error]
  double ki{2.0};  ///< [duty per (m/s · s)]
  double integral_limit{0.3};  ///< anti-windup clamp on the integral term [duty]
  double max_duty{1.0};  ///< output clamp
  bool closed_loop{true};  ///< false → feed-forward only
};

class WheelVelocityController
{
public:
  WheelVelocityController() = default;

  explicit WheelVelocityController(WheelLoopGains gains) : gains_(gains)
  {
  }

  void set_gains(const WheelLoopGains& gains)
  {
    gains_ = gains;
  }

  /// A zero target is an exact stop: the integrator resets and 0 is returned.
  [[nodiscard]] double Update(double target_mps, double measured_mps, double dt_s)
  {
    if (!std::isfinite(target_mps) || target_mps == 0.0)
    {
      integral_ = 0.0;
      return 0.0;
    }
    const double ff = gains_.duty_per_mps * target_mps;
    if (!gains_.closed_loop || !std::isfinite(measured_mps) || dt_s <= 0.0)
    {
      return Clamp(ff);
    }
    const double error = target_mps - measured_mps;
    integral_ = std::clamp(integral_ + gains_.ki * error * dt_s,
                           -gains_.integral_limit,
                           gains_.integral_limit);
    return Clamp(ff + gains_.kp * error + integral_);
  }

  void Reset()
  {
    integral_ = 0.0;
  }

  [[nodiscard]] double integral() const noexcept
  {
    return integral_;
  }

private:
  [[nodiscard]] double Clamp(double duty) const
  {
    const double limit = std::abs(gains_.max_duty);
    return std::clamp(duty, -limit, limit);
  }

  WheelLoopGains gains_{};
  double integral_{0.0};
};

/// Differential-drive split of a body twist into per-wheel ground speeds.
struct WheelSpeeds
{
  double left_mps{0.0};
  double right_mps{0.0};
};

[[nodiscard]] inline WheelSpeeds SplitTwist(double linear_x, double angular_z, double wheel_track)
{
  const double half = 0.5 * wheel_track * angular_z;
  return WheelSpeeds{linear_x - half, linear_x + half};
}

}  // namespace mowgli_openmower_bridge
