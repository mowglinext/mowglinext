// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file xesc_odometry.hpp
 * @brief Wheel odometry from the two drive controllers' signed tick counters.
 *
 * Mirrors the semantics of mowgli_hardware's OdometryPublisher: deltas are
 * aggregated over a ~50 ms window before a velocity is derived (single-tick
 * noise otherwise turns into phantom velocity spikes at low speed), the
 * worst-wheel odometer is kept for slip diagnostics, and a stationary flag
 * gates the IMU bias estimator. Input ticks already carry the per-motor
 * inversion; this class is sign-agnostic.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>

namespace mowgli_openmower_bridge
{

/// Largest wheel-surface speed believed from a tick delta: four times the
/// 0.5 m/s the stack ever commands. Anything faster is a counter
/// DISCONTINUITY, not motion — an xESC brown-out restarts its tachometer at 0,
/// and the unsigned difference then wraps to billions of ticks (the
/// simulation measured a 2.2e7 m/s /wheel_odom spike from exactly that).
constexpr double kMaxPlausibleWheelSpeedMps = 2.0;
/// Absolute slack so a scheduling hiccup on a slow tick is never misread.
constexpr int64_t kTickDeltaSlack = 50;

[[nodiscard]] inline bool IsPlausibleTickDelta(int64_t delta, double ticks_per_meter, double dt_s)
{
  const double limit =
      kMaxPlausibleWheelSpeedMps * ticks_per_meter * std::max(dt_s, 0.0) + kTickDeltaSlack;
  return std::abs(static_cast<double>(delta)) <= limit;
}

struct WheelOdometrySample
{
  double vx_mps{0.0};
  double vyaw_radps{0.0};
  double d_left_m{0.0};
  double d_right_m{0.0};
  double dt_s{0.0};
};

class XescOdometry
{
public:
  static constexpr double kDefaultWindowS = 0.05;

  XescOdometry(double ticks_per_meter, double wheel_track, double window_s = kDefaultWindowS)
      : ticks_per_meter_(ticks_per_meter), wheel_track_(wheel_track), window_s_(window_s)
  {
  }

  void set_kinematics(double ticks_per_meter, double wheel_track)
  {
    ticks_per_meter_ = ticks_per_meter;
    wheel_track_ = wheel_track;
  }

  /// First call after Reset() only primes the counters and returns nullopt.
  [[nodiscard]] std::optional<WheelOdometrySample> Update(int64_t left_ticks,
                                                          int64_t right_ticks,
                                                          double dt_s)
  {
    if (!primed_)
    {
      primed_ = true;
      prev_left_ = left_ticks;
      prev_right_ = right_ticks;
      return std::nullopt;
    }
    const int64_t d_left = left_ticks - prev_left_;
    const int64_t d_right = right_ticks - prev_right_;
    prev_left_ = left_ticks;
    prev_right_ = right_ticks;

    acc_left_ += d_left;
    acc_right_ += d_right;
    acc_dt_ += std::max(0.0, dt_s);
    if (acc_dt_ < window_s_)
    {
      return std::nullopt;
    }

    WheelOdometrySample s{};
    s.dt_s = acc_dt_;
    s.d_left_m = static_cast<double>(acc_left_) / ticks_per_meter_;
    s.d_right_m = static_cast<double>(acc_right_) / ticks_per_meter_;
    s.vx_mps = 0.5 * (s.d_left_m + s.d_right_m) / s.dt_s;
    s.vyaw_radps = (s.d_right_m - s.d_left_m) / wheel_track_ / s.dt_s;

    stationary_ = (acc_left_ == 0 && acc_right_ == 0);
    tyre_travelled_m_ += std::max(std::abs(s.d_left_m), std::abs(s.d_right_m));

    acc_left_ = 0;
    acc_right_ = 0;
    acc_dt_ = 0.0;
    return s;
  }

  void Reset()
  {
    primed_ = false;
    acc_left_ = 0;
    acc_right_ = 0;
    acc_dt_ = 0.0;
    stationary_ = true;
  }

  [[nodiscard]] bool stationary() const noexcept
  {
    return stationary_;
  }

  /// Worst-wheel odometer [m], never reset (see CLAUDE.md Invariant 16).
  [[nodiscard]] double tyre_travelled() const noexcept
  {
    return tyre_travelled_m_;
  }

private:
  double ticks_per_meter_;
  double wheel_track_;
  double window_s_;
  bool primed_{false};
  int64_t prev_left_{0};
  int64_t prev_right_{0};
  int64_t acc_left_{0};
  int64_t acc_right_{0};
  double acc_dt_{0.0};
  bool stationary_{true};
  double tyre_travelled_m_{0.0};
};

/**
 * @brief Per-wheel measured speed for the velocity loop: a first-order
 *        low-pass in TIME, not per sample.
 *
 * Each sample weighs 1 - exp(-dt / tau): proportional to the time it covers.
 * A fixed per-sample weight averages tick/dt RATIOS instead, and whenever a
 * sample's ticks do not cover exactly its dt (a controller sampled slightly
 * before the poll that read it, a jittery poll period) that average is biased
 * upward — 1.08 m/s for a true 0.30 in the unit test, 0.23 m/s of real
 * ground speed for a 0.30 command on a loaded CI runner, because the loop
 * believed the wheel was faster than it was.
 */
class WheelSpeedFilter
{
public:
  /// 0.056 s = the weight the former per-sample 0.3 gave a nominal 20 ms tick.
  static constexpr double kDefaultTimeConstantS = 0.056;

  WheelSpeedFilter() = default;

  explicit WheelSpeedFilter(double time_constant_s) : tau_s_(std::max(time_constant_s, 1e-3))
  {
  }

  [[nodiscard]] double Update(int64_t d_ticks, double ticks_per_meter, double dt_s)
  {
    if (dt_s <= 0.0 || ticks_per_meter <= 0.0)
    {
      return value_;
    }
    const double raw = static_cast<double>(d_ticks) / ticks_per_meter / dt_s;
    const double weight = 1.0 - std::exp(-dt_s / tau_s_);
    value_ += weight * (raw - value_);
    return value_;
  }

  void Reset()
  {
    value_ = 0.0;
  }

  [[nodiscard]] double value() const noexcept
  {
    return value_;
  }

private:
  double tau_s_{kDefaultTimeConstantS};
  double value_{0.0};
};

}  // namespace mowgli_openmower_bridge
