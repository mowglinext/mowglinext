// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <algorithm>
#include <cmath>

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

}  // namespace mowgli_hardware
