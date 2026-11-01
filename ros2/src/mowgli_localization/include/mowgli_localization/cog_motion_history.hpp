// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <optional>

namespace mowgli_localization
{

// Source-stamped, held motion inputs for the COG baseline. These records describe
// accepted inputs, not freshness inferred from callback arrival. A bounded-history
// miss is explicit; a newer motion sample never substitutes for an older fix.
// The node's default mutually-exclusive callback group serializes access.
class CogMotionHistory
{
public:
  struct Motion
  {
    double vx;
    double wheel_omega;
    double gyro_z;
  };
  struct Baseline
  {
    bool direction_consistent;
    double absolute_rotation;
  };

  explicit CogMotionHistory(std::size_t capacity = 512)
      : capacity_(std::max(capacity, std::size_t{1}))
  {
  }

  // Returns true only when a ROS-clock rewind invalidated the previous epoch.
  bool ObserveClock(std::int64_t now_ns)
  {
    const bool rewind = now_ns_ && now_ns < *now_ns_;
    if (rewind)
    {
      wheels_.clear();
      gyros_.clear();
    }
    now_ns_ = now_ns;
    return rewind;
  }

  bool AddWheel(std::int64_t stamp_ns, double vx, double omega)
  {
    if (!ValidStamp(stamp_ns) || !std::isfinite(vx) || !std::isfinite(omega) ||
        (!wheels_.empty() && stamp_ns <= wheels_.back().stamp_ns))
      return false;
    wheels_.push_back({stamp_ns, vx, omega});
    if (wheels_.size() > capacity_)
      wheels_.pop_front();
    return true;
  }

  bool AddGyro(std::int64_t stamp_ns, double z)
  {
    if (!ValidStamp(stamp_ns) || !std::isfinite(z) ||
        (!gyros_.empty() && stamp_ns <= gyros_.back().stamp_ns))
      return false;
    gyros_.push_back({stamp_ns, z});
    if (gyros_.size() > capacity_)
      gyros_.pop_front();
    return true;
  }

  std::optional<Motion> At(std::int64_t stamp_ns) const
  {
    const auto wheel = Held(wheels_, stamp_ns);
    if (!wheel)
      return std::nullopt;
    // Preserve the producer's wheel-only startup behavior until an IMU input
    // exists. Once observed, gyro history must cover the requested source.
    const auto gyro = Held(gyros_, stamp_ns);
    if (!gyros_.empty() && !gyro)
      return std::nullopt;
    return Motion{wheel->vx, wheel->omega, gyro ? gyro->z : 0.0};
  }

  std::optional<Baseline> Between(std::int64_t start_ns,
                                  std::int64_t end_ns,
                                  double minimum_speed) const
  {
    const auto first = At(start_ns);
    if (start_ns > end_ns || !first || !At(end_ns))
      return std::nullopt;
    const auto same_direction = [&](double vx)
    {
      return std::abs(vx) >= minimum_speed && (vx >= 0.0) == (first->vx >= 0.0);
    };
    bool consistent = same_direction(first->vx);
    for (const auto& wheel : wheels_)
      if (wheel.stamp_ns > start_ns && wheel.stamp_ns <= end_ns)
        consistent = consistent && same_direction(wheel.vx);

    return Baseline{consistent, *AbsoluteRotation(start_ns, end_ns)};
  }

  std::optional<double> AbsoluteRotation(std::int64_t start_ns,
                                         std::int64_t end_ns,
                                         double deadband = 0.0) const
  {
    const auto first = At(start_ns);
    if (start_ns > end_ns || !first || !At(end_ns))
      return std::nullopt;
    const auto rate = [deadband](double z)
    {
      return std::abs(z) <= deadband ? 0.0 : std::abs(z);
    };
    double rotation = 0.0;
    double z = first->gyro_z;
    std::int64_t previous_ns = start_ns;
    for (const auto& gyro : gyros_)
    {
      if (gyro.stamp_ns <= start_ns || gyro.stamp_ns > end_ns)
        continue;
      rotation += rate(z) * static_cast<double>(gyro.stamp_ns - previous_ns) * 1e-9;
      previous_ns = gyro.stamp_ns;
      z = gyro.z;
    }
    return rotation + rate(z) * static_cast<double>(end_ns - previous_ns) * 1e-9;
  }

private:
  struct Wheel
  {
    std::int64_t stamp_ns;
    double vx;
    double omega;
  };
  struct Gyro
  {
    std::int64_t stamp_ns;
    double z;
  };

  template <typename T>
  static const T* Held(const std::deque<T>& samples, std::int64_t stamp_ns)
  {
    const auto next = std::upper_bound(samples.begin(),
                                       samples.end(),
                                       stamp_ns,
                                       [](std::int64_t stamp, const T& sample)
                                       {
                                         return stamp < sample.stamp_ns;
                                       });
    return next == samples.begin() ? nullptr : &*std::prev(next);
  }

  bool ValidStamp(std::int64_t stamp_ns) const
  {
    return now_ns_ && stamp_ns >= 0 && stamp_ns <= *now_ns_;
  }

  std::size_t capacity_;
  std::optional<std::int64_t> now_ns_;
  std::deque<Wheel> wheels_;
  std::deque<Gyro> gyros_;
};
}  // namespace mowgli_localization
