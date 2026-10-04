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
#include <deque>
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
 * @brief Per-wheel measured speed for the velocity loop: total ticks over
 *        total time in a short sliding window.
 *
 * Ticks and the interval they are divided by never line up exactly (a
 * controller samples a little before the poll that reads it, the poll period
 * jitters), so one sample reads low and the next high. Any per-SAMPLE
 * average of those ratios is biased upward — even an exponential weight in
 * time, which saturates for samples comparable to its time constant — and
 * the loop then drives the wheel slower than commanded (0.23-0.28 m/s for a
 * 0.30 command on a loaded CI runner). The sum over a window does not care
 * how the ticks were split between samples.
 */
class WheelSpeedFilter
{
public:
  /// Long enough to span several 20 ms samples, short enough for the loop.
  static constexpr double kDefaultWindowS = 0.08;

  WheelSpeedFilter() = default;

  explicit WheelSpeedFilter(double window_s) : window_s_(std::max(window_s, 1e-3))
  {
  }

  [[nodiscard]] double Update(int64_t d_ticks, double ticks_per_meter, double dt_s)
  {
    if (dt_s <= 0.0 || ticks_per_meter <= 0.0)
    {
      return value_;
    }
    samples_.push_back({d_ticks, dt_s});
    sum_ticks_ += d_ticks;
    sum_dt_ += dt_s;
    // Keep the newest sample even when it alone exceeds the window (a stall).
    while (samples_.size() > 1u && sum_dt_ - samples_.front().dt_s >= window_s_)
    {
      sum_ticks_ -= samples_.front().d_ticks;
      sum_dt_ -= samples_.front().dt_s;
      samples_.pop_front();
    }
    value_ = static_cast<double>(sum_ticks_) / ticks_per_meter / sum_dt_;
    return value_;
  }

  void Reset()
  {
    samples_.clear();
    sum_ticks_ = 0;
    sum_dt_ = 0.0;
    value_ = 0.0;
  }

  [[nodiscard]] double value() const noexcept
  {
    return value_;
  }

private:
  struct Sample
  {
    int64_t d_ticks;
    double dt_s;
  };

  double window_s_{kDefaultWindowS};
  std::deque<Sample> samples_;
  int64_t sum_ticks_{0};
  double sum_dt_{0.0};
  double value_{0.0};
};

}  // namespace mowgli_openmower_bridge
