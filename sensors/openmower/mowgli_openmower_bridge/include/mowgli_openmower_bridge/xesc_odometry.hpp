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

/// Per-wheel measured speed with a first-order low-pass, for the velocity loop.
class WheelSpeedFilter
{
public:
  WheelSpeedFilter() = default;

  explicit WheelSpeedFilter(double alpha) : alpha_(std::clamp(alpha, 0.0, 1.0))
  {
  }

  [[nodiscard]] double Update(int64_t d_ticks, double ticks_per_meter, double dt_s)
  {
    if (dt_s <= 0.0 || ticks_per_meter <= 0.0)
    {
      return value_;
    }
    const double raw = static_cast<double>(d_ticks) / ticks_per_meter / dt_s;
    value_ += alpha_ * (raw - value_);
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
  double alpha_{0.3};
  double value_{0.0};
};

}  // namespace mowgli_openmower_bridge
