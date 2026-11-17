// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "mowgli_interfaces/linear_command_policy.hpp"

namespace mowgli_hardware
{

/// Limit a non-zero motion command without delaying an explicit stop.
///
/// `target == 0` always returns zero immediately. A direction reversal first
/// decelerates toward zero at `deceleration_limit`; only a later update may
/// accelerate away from rest in the opposite direction.
inline double limit_motion_command_slew(double target,
                                        double previous,
                                        double dt_s,
                                        double acceleration_limit,
                                        double deceleration_limit)
{
  if (target == 0.0)
  {
    return 0.0;
  }

  if (std::signbit(target) != std::signbit(previous) && previous != 0.0)
  {
    const double max_delta = deceleration_limit * std::max(0.0, dt_s);
    return previous > 0.0 ? std::max(0.0, previous - max_delta)
                          : std::min(0.0, previous + max_delta);
  }

  const bool increasing_magnitude = std::abs(target) > std::abs(previous);
  const double rate_limit = increasing_magnitude ? acceleration_limit : deceleration_limit;
  const double max_delta = rate_limit * std::max(0.0, dt_s);
  return previous + std::clamp(target - previous, -max_delta, max_delta);
}

struct MotionCommand
{
  double vx{0.0};
  double wz{0.0};
};

struct CmdVelSlewLimits
{
  double linear_accel{0.30};
  double linear_decel{0.60};
  double angular_accel{1.0};
  double angular_decel{2.0};
};

/// The bridge's input deadband, finite slew and output deadband, in wire order.
/// Fresh BT-reported autonomous control may start and continue below the soft deadband.
/// This never imposes a minimum target or grants firmware motion/blade permission.
class CmdVelPipeline
{
public:
  MotionCommand shape(MotionCommand target,
                      bool autonomous,
                      double min_linear_vel,
                      const CmdVelSlewLimits& limits,
                      std::int64_t now_ns)
  {
    double dt_s = 0.1;
    if (have_shaped_)
    {
      const double elapsed_s = static_cast<double>(now_ns - last_shaped_ns_) * 1.0e-9;
      if (elapsed_s > 0.0 && elapsed_s <= 0.6)
      {
        dt_s = std::min(elapsed_s, 0.2);
        if (autonomous != last_autonomous_)
        {
          restart_from_applied();
        }
      }
      else
      {
        shaped_ = {};
      }
    }
    target.vx = filter_linear(target.vx, autonomous, min_linear_vel);
    shaped_.vx = limit_motion_command_slew(
        target.vx, shaped_.vx, dt_s, limits.linear_accel, limits.linear_decel);
    shaped_.wz = limit_motion_command_slew(
        target.wz, shaped_.wz, dt_s, limits.angular_accel, limits.angular_decel);
    last_shaped_ns_ = now_ns;
    have_shaped_ = true;
    last_autonomous_ = autonomous;
    return {filter_linear(shaped_.vx, autonomous, min_linear_vel), shaped_.wz};
  }

  /// Only the successfully written float32 packet is an accepted output.
  void record_applied(float vx, float wz)
  {
    applied_ = {vx, wz};
  }

  /// A mode handover must not inherit a ramp step suppressed on the wire.
  void restart_from_applied()
  {
    shaped_ = applied_;
  }

  void reset()
  {
    *this = CmdVelPipeline{};
  }

private:
  friend struct HardwareBridgeBladeStatusTestPeer;

  static double filter_linear(double vx, bool autonomous, double minimum)
  {
    // Preserve the existing manual/input policy, including its dust threshold.
    return mowgli_interfaces::apply_linear_soft_deadband(vx, minimum, autonomous);
  }

  MotionCommand shaped_{};
  MotionCommand applied_{};
  std::int64_t last_shaped_ns_{0};
  bool have_shaped_{false};
  bool last_autonomous_{false};
};

}  // namespace mowgli_hardware
