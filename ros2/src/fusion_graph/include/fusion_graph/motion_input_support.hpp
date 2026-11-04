// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>

namespace fusion_graph
{
// Outer computational limit shared with the existing process integration guard.
// Holding a rate below this limit is an estimator assumption, not an accuracy,
// travel or stopping guarantee. Loss after support requires a node restart.
inline constexpr double kMotionIntegrationGapS = 1.0;
inline constexpr int64_t kMotionSupportGapNs = 1000000000;

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
      epoch_ = epoch;
    }
    last_now_ = now;
    if (established_ && ((wheel_ && now >= *wheel_ && now - *wheel_ >= kMotionSupportGapNs) ||
                         (imu_ && now >= *imu_ && now - *imu_ >= kMotionSupportGapNs)))
      lost_ = true;
  }

  void Accept(bool wheel, int64_t stamp, int64_t now, uint64_t epoch)
  {
    // Check the old interval before replacing its endpoint, including the first
    // returning sample when no timer observed the gap.
    Observe(now, epoch);
    auto& previous = wheel ? wheel_ : imu_;
    if (established_ && previous && stamp > *previous && stamp - *previous >= kMotionSupportGapNs)
      lost_ = true;
    if (!previous || stamp > *previous)
      previous = stamp;
  }

  bool Allows(int64_t now, int64_t target, uint64_t epoch)
  {
    Observe(now, epoch);
    if (lost_ || !wheel_ || !imu_ || *wheel_ > now || *imu_ > now)
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

private:
  std::optional<int64_t> wheel_;
  std::optional<int64_t> imu_;
  std::optional<int64_t> last_now_;
  uint64_t epoch_ = 0;
  bool established_ = false;
  bool lost_ = false;
};
}  // namespace fusion_graph
