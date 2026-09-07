// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace mowgli_hardware
{

using TimerDuration = std::chrono::nanoseconds;

// Mirrors the firmware's PKT_ID_SET_KINEMATICS wheel-base clamp. Keeping the
// host in this range prevents host odometry and firmware inverse kinematics
// from silently using different geometry.
inline constexpr double kMinRuntimeWheelTrackM = 0.15;
inline constexpr double kMaxRuntimeWheelTrackM = 0.60;

struct TimerRateRange
{
  double minimum_hz;
  double maximum_hz;
};

// Preferred cadences keep each periodic task within the rate it was designed
// and tested for. They are operational limits, not validity limits: usable
// requests outside a range are clamped at startup with a warning.
inline constexpr TimerRateRange kHeartbeatRateRange{1.0, 50.0};
inline constexpr TimerRateRange kPublishRateRange{10.0, 500.0};
inline constexpr TimerRateRange kHighLevelRateRange{1.0, 20.0};
inline constexpr TimerRateRange kDigMonitorRateRange{1.0, 50.0};

inline void require_finite_positive(double value, const char* name)
{
  if (!std::isfinite(value) || value <= 0.0)
  {
    throw std::invalid_argument(std::string{name} + " must be finite and positive");
  }
}

inline void require_finite_nonnegative_timeout(double value, const char* name)
{
  if (!std::isfinite(value) || value < 0.0)
  {
    throw std::invalid_argument(std::string{name} + " must be finite and non-negative");
  }
}

inline double normalize_runtime_wheel_track(double value)
{
  if (!std::isfinite(value))
  {
    throw std::invalid_argument("wheel_track must be finite");
  }
  return std::clamp(value, kMinRuntimeWheelTrackM, kMaxRuntimeWheelTrackM);
}

/** Lowest positive rate whose reciprocal is representable as TimerDuration. */
inline double minimum_timer_rate_hz()
{
  // Round upward so the descriptor cannot admit a rate whose reciprocal is
  // already outside the int64 nanosecond range after floating-point rounding.
  const double exact = 1.0e9 / static_cast<double>(TimerDuration::max().count());
  return std::nextafter(exact, std::numeric_limits<double>::infinity());
}

inline constexpr double maximum_timer_rate_hz()
{
  return 1.0e9;
}

/**
 * Convert a timer cadence in Hz to a positive, representable wall-timer
 * duration. Nanoseconds preserve sub-millisecond cadence. Rates whose period
 * is outside the representable [1 ns, nanoseconds::max()] interval are
 * rejected rather than creating a zero-duration or CPU-hammering timer.
 */
inline TimerDuration timer_period_from_rate_hz(double rate_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz < minimum_timer_rate_hz() ||
      rate_hz > maximum_timer_rate_hz())
  {
    throw std::invalid_argument("timer rate must be finite and within the representable range");
  }

  const long double period_ns = 1.0e9L / static_cast<long double>(rate_hz);
  const long double max_period_ns = static_cast<long double>(TimerDuration::max().count());
  if (period_ns < 1.0L || period_ns > max_period_ns)
  {
    throw std::invalid_argument("timer rate must have a representable period in [1 ns, max]");
  }

  // Bounds were checked as long doubles before this conversion. Truncation
  // preserves historic exact periods for integral-ms defaults while allowing
  // sub-millisecond rates.
  return TimerDuration{static_cast<TimerDuration::rep>(period_ns)};
}

inline double normalize_operational_timer_rate(double rate_hz, TimerRateRange range)
{
  // This conversion is the validity check: it rejects non-finite, non-positive,
  // unrepresentable and zero-duration timer rates before clamping policy is
  // applied.
  (void)timer_period_from_rate_hz(rate_hz);
  return std::clamp(rate_hz, range.minimum_hz, range.maximum_hz);
}

}  // namespace mowgli_hardware
