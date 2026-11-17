// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>

namespace fusion_graph
{
// Outer computational limit shared with the existing process integration guard.
// Holding a rate below this limit is an estimator assumption, not an accuracy,
// travel or stopping guarantee.
inline constexpr double kMotionIntegrationGapS = 1.0;
inline constexpr int64_t kMotionSupportGapNs = 1000000000;
// After a loss, BOTH streams must stay continuous (no gap >= kMotionSupportGapNs,
// same epoch) for this long before publication resumes. A single returning
// sample still cannot renew support. Until 2026-11-08 a loss latched until a
// node restart: one STM32 reflash / serial reconnect (~9 s silence) left the
// robot with no odom->base_footprint TF for good, collision_monitor zeroed
// every command and it could not undock. Graph and dead-reckoning state are
// kept through a loss, so resuming is a publication decision only.
inline constexpr int64_t kMotionRecoveryNs = 2000000000;
// hardware_bridge stamps IMU / wheel samples a few ms AHEAD of their receipt
// (field bag 2026-11-09: up to +6 ms, on ~45 % of samples). Treating every such
// sample as "from the future" made the 20 Hz TF thread skip ticks at random:
// odom->base_footprint gaps of 100-200 ms on 13 % of intervals, and
// collision_monitor (transform_tolerance 0.1 s) declared its scan source
// invalid and stopped the robot. Inputs up to this far ahead of now() are
// current; anything further is still refused as a clock fault.
inline constexpr int64_t kMotionInputFutureToleranceNs = 100000000;

// All access belongs under FusionGraphNode::tf_state_mu_. A new callback alone
// cannot renew support: only a validated, advancing acquisition stamp may do so.
class MotionInputSupport
{
public:
  void Observe(int64_t now, uint64_t epoch)
  {
    if (epoch != epoch_ || (last_now_ && now < *last_now_))
    {
      lost_ = lost_ || established_;
      wheel_.reset();
      imu_.reset();
      recovery_.reset();
      epoch_ = epoch;
    }
    last_now_ = now;
    if (established_ && ((wheel_ && now >= *wheel_ && now - *wheel_ >= kMotionSupportGapNs) ||
                         (imu_ && now >= *imu_ && now - *imu_ >= kMotionSupportGapNs)))
    {
      lost_ = true;
      recovery_.reset();  // any gap restarts the recovery window
    }
  }

  void Accept(bool wheel, int64_t stamp, int64_t now, uint64_t epoch)
  {
    // Check the old interval before replacing its endpoint, including the first
    // returning sample when no timer observed the gap.
    Observe(now, epoch);
    auto& previous = wheel ? wheel_ : imu_;
    if (established_ && previous && stamp > *previous && stamp - *previous >= kMotionSupportGapNs)
    {
      lost_ = true;
      recovery_.reset();
    }
    if (!previous || stamp > *previous)
      previous = stamp;
    // Both streams present again: the recovery window starts now and survives
    // only as long as Observe/Accept see no further gap.
    if (lost_ && wheel_ && imu_ && !recovery_)
      recovery_ = now;
  }

  bool Allows(int64_t now, int64_t target, uint64_t epoch)
  {
    Observe(now, epoch);
    if (lost_ && recovery_ && now >= *recovery_ && now - *recovery_ >= kMotionRecoveryNs)
    {
      lost_ = false;
      recovery_.reset();
      ++recoveries_;
    }
    if (lost_ || !wheel_ || !imu_ || *wheel_ > now + kMotionInputFutureToleranceNs ||
        *imu_ > now + kMotionInputFutureToleranceNs)
      return false;
    if (target < now || target - *wheel_ >= kMotionSupportGapNs ||
        target - *imu_ >= kMotionSupportGapNs)
      return false;
    established_ = true;
    return true;
  }

  bool lost() const
  {
    return lost_;
  }
  bool established() const
  {
    return established_;
  }
  uint64_t recoveries() const
  {
    return recoveries_;
  }

private:
  std::optional<int64_t> wheel_;
  std::optional<int64_t> imu_;
  std::optional<int64_t> last_now_;
  std::optional<int64_t> recovery_;
  uint64_t recoveries_ = 0;
  uint64_t epoch_ = 0;
  bool established_ = false;
  bool lost_ = false;
};
}  // namespace fusion_graph
