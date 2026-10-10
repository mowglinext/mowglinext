// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file imu_bias_estimator.hpp
 * @brief At-rest gyro / accelerometer bias estimate.
 *
 * The OpenMower LowLevel board streams a raw IMU. fusion_graph's gyro
 * between-factor integrates yaw rate, so a static bias of a few mrad/s turns
 * into degrees per minute of heading drift. While the robot is provably at
 * rest (on the charger, wheels stationary) N samples are averaged into
 * offsets that are subtracted from every later sample. Accel Z is never
 * corrected (gravity must survive for the fusion). Pure, no time source.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace mowgli_openmower_bridge
{

struct ImuSample
{
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
  double gx{0.0};
  double gy{0.0};
  double gz{0.0};
};

struct ImuBias
{
  double ax{0.0};
  double ay{0.0};
  double gx{0.0};
  double gy{0.0};
  double gz{0.0};
  double gz_variance{0.0};
};

class ImuBiasEstimator
{
public:
  explicit ImuBiasEstimator(std::size_t samples_required = 200u)
      : samples_required_(samples_required == 0u ? 1u : samples_required)
  {
  }

  void Start()
  {
    collecting_ = true;
    count_ = 0u;
    sums_ = {};
    sq_gz_ = 0.0;
  }

  void Abort()
  {
    collecting_ = false;
    count_ = 0u;
  }

  /// Feed one raw sample while at rest. Returns true when a bias just became ready.
  [[nodiscard]] bool Feed(const ImuSample& s)
  {
    if (!collecting_)
    {
      return false;
    }
    if (!std::isfinite(s.ax) || !std::isfinite(s.ay) || !std::isfinite(s.gx) ||
        !std::isfinite(s.gy) || !std::isfinite(s.gz))
    {
      return false;
    }
    sums_[0] += s.ax;
    sums_[1] += s.ay;
    sums_[2] += s.gx;
    sums_[3] += s.gy;
    sums_[4] += s.gz;
    sq_gz_ += s.gz * s.gz;
    ++count_;
    if (count_ < samples_required_)
    {
      return false;
    }
    const double n = static_cast<double>(count_);
    bias_.ax = sums_[0] / n;
    bias_.ay = sums_[1] / n;
    bias_.gx = sums_[2] / n;
    bias_.gy = sums_[3] / n;
    bias_.gz = sums_[4] / n;
    bias_.gz_variance = std::max(0.0, sq_gz_ / n - bias_.gz * bias_.gz);
    ready_ = true;
    collecting_ = false;
    return true;
  }

  [[nodiscard]] ImuSample Apply(const ImuSample& s) const
  {
    if (!ready_)
    {
      return s;
    }
    ImuSample out = s;
    out.ax -= bias_.ax;
    out.ay -= bias_.ay;
    out.gx -= bias_.gx;
    out.gy -= bias_.gy;
    out.gz -= bias_.gz;
    return out;
  }

  [[nodiscard]] bool collecting() const noexcept
  {
    return collecting_;
  }
  [[nodiscard]] bool ready() const noexcept
  {
    return ready_;
  }
  [[nodiscard]] const ImuBias& bias() const noexcept
  {
    return bias_;
  }
  [[nodiscard]] std::size_t count() const noexcept
  {
    return count_;
  }

private:
  std::size_t samples_required_;
  bool collecting_{false};
  bool ready_{false};
  std::size_t count_{0u};
  std::array<double, 5> sums_{};
  double sq_gz_{0.0};
  ImuBias bias_{};
};

}  // namespace mowgli_openmower_bridge
