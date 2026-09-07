// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Receipt-provenance timing policy for cog_to_imu.
//
// Physical observation intervals are derived exclusively from receiver receipt
// stamps. Callback cadence, ROS publication rate, and wall-clock delivery gaps
// are deliberately absent from this API.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>

#include "mowgli_interfaces/gnss_observation_freshness.hpp"

namespace mowgli_localization
{

inline double DeriveMaximumCogObservationIntervalSeconds(const double physical_observation_rate_hz,
                                                         const double period_tolerance_factor)
{
  if (!std::isfinite(physical_observation_rate_hz) || physical_observation_rate_hz <= 0.0)
  {
    throw std::invalid_argument("physical GNSS observation rate must be finite and positive");
  }
  if (!std::isfinite(period_tolerance_factor) || period_tolerance_factor < 1.0)
  {
    throw std::invalid_argument("COG observation-period tolerance factor must be >= 1");
  }
  return period_tolerance_factor / physical_observation_rate_hz;
}

inline std::int64_t CogSecondsToNanoseconds(const double seconds)
{
  constexpr double kNanosecondsPerSecond = 1.0e9;
  const double nanoseconds = seconds * kNanosecondsPerSecond;
  if (!std::isfinite(seconds) || seconds < 0.0 ||
      nanoseconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("COG timing duration is invalid or out of range");
  }
  return static_cast<std::int64_t>(std::llround(nanoseconds));
}

enum class CogObservationTimingDecision
{
  kSeed,
  kAdvance,
  kDuplicate,
  kTooSoon,
  kGapReseed,
  kInvalidProvenance,
  kReceiptRewind,
  kRosTimeDiscontinuity,
};

struct CogObservationTimingResult
{
  CogObservationTimingDecision decision{CogObservationTimingDecision::kInvalidProvenance};

  bool accepts_current_observation() const
  {
    return decision == CogObservationTimingDecision::kSeed ||
           decision == CogObservationTimingDecision::kAdvance ||
           decision == CogObservationTimingDecision::kGapReseed;
  }

  bool reseeds_baseline() const
  {
    return decision == CogObservationTimingDecision::kSeed ||
           decision == CogObservationTimingDecision::kGapReseed;
  }
};

// Bounded receipt-only timing tracker for NavSatFix.
//
// A small receipt history distinguishes a delayed duplicate from a genuinely
// unseen stamp rewind: the former is ignored without touching the active
// baseline, while the latter resets the receipt epoch and drops the triggering
// sample. An excessive forward gap accepts the current observation only as the
// seed of a clean baseline.
class CogObservationTiming
{
public:
  CogObservationTiming(const double minimum_interval_s,
                       const double maximum_interval_s,
                       const double maximum_provenance_age_s,
                       const std::size_t receipt_history_capacity = 64)
      : minimum_interval_ns_(CogSecondsToNanoseconds(minimum_interval_s)),
        maximum_interval_ns_(CogSecondsToNanoseconds(maximum_interval_s)),
        maximum_provenance_age_ns_(CogSecondsToNanoseconds(maximum_provenance_age_s)),
        receipt_history_capacity_(receipt_history_capacity)
  {
    if (maximum_interval_ns_ <= minimum_interval_ns_)
    {
      throw std::invalid_argument("COG maximum observation interval must exceed minimum interval");
    }
    if (receipt_history_capacity_ == 0)
    {
      throw std::invalid_argument("COG receipt history capacity must be positive");
    }
  }

  CogObservationTimingResult Observe(const std::int64_t receipt_time_ns,
                                     const std::int64_t ros_now_ns)
  {
    if (last_ros_now_ns_ && ros_now_ns < *last_ros_now_ns_)
    {
      ResetReceiptEpoch();
      last_ros_now_ns_ = ros_now_ns;
      return {CogObservationTimingDecision::kRosTimeDiscontinuity};
    }
    last_ros_now_ns_ = ros_now_ns;

    // A cached or delayed duplicate is never a new physical sample, even if it
    // arrives after the provenance-age window. It must not disturb a newer
    // valid baseline.
    if (ReceiptWasAccepted(receipt_time_ns))
    {
      return {CogObservationTimingDecision::kDuplicate};
    }

    if (!mowgli_interfaces::gnss_observation_freshness::IsReceiptFresh(receipt_time_ns,
                                                                       ros_now_ns,
                                                                       maximum_provenance_age_ns_))
    {
      ResetReceiptEpoch();
      return {CogObservationTimingDecision::kInvalidProvenance};
    }

    if (!last_receipt_time_ns_)
    {
      Accept(receipt_time_ns);
      return {CogObservationTimingDecision::kSeed};
    }

    if (receipt_time_ns < *last_receipt_time_ns_)
    {
      ResetReceiptEpoch();
      return {CogObservationTimingDecision::kReceiptRewind};
    }

    const std::int64_t interval_ns = receipt_time_ns - *last_receipt_time_ns_;
    if (interval_ns < minimum_interval_ns_)
    {
      return {CogObservationTimingDecision::kTooSoon};
    }

    Accept(receipt_time_ns);
    if (interval_ns > maximum_interval_ns_)
    {
      return {CogObservationTimingDecision::kGapReseed};
    }
    return {CogObservationTimingDecision::kAdvance};
  }

  void Reset()
  {
    ResetReceiptEpoch();
    last_ros_now_ns_.reset();
  }

  std::optional<std::int64_t> last_receipt_time_ns() const
  {
    return last_receipt_time_ns_;
  }

private:
  bool ReceiptWasAccepted(const std::int64_t receipt_time_ns) const
  {
    return std::find(recent_receipts_.begin(), recent_receipts_.end(), receipt_time_ns) !=
           recent_receipts_.end();
  }

  void Accept(const std::int64_t receipt_time_ns)
  {
    last_receipt_time_ns_ = receipt_time_ns;
    recent_receipts_.push_back(receipt_time_ns);
    while (recent_receipts_.size() > receipt_history_capacity_)
    {
      recent_receipts_.pop_front();
    }
  }

  void ResetReceiptEpoch()
  {
    last_receipt_time_ns_.reset();
    recent_receipts_.clear();
  }

  std::int64_t minimum_interval_ns_;
  std::int64_t maximum_interval_ns_;
  std::int64_t maximum_provenance_age_ns_;
  std::size_t receipt_history_capacity_;
  std::optional<std::int64_t> last_receipt_time_ns_;
  std::optional<std::int64_t> last_ros_now_ns_;
  std::deque<std::int64_t> recent_receipts_;
};

enum class StationaryLatchState
{
  kNoLatch,
  kFresh,
  kExpired,
  kInvalidProvenance,
  kClockDiscontinuity,
};

struct StationaryLatchResult
{
  StationaryLatchState state{StationaryLatchState::kNoLatch};
  double age_s{0.0};

  bool fresh() const
  {
    return state == StationaryLatchState::kFresh;
  }
};

// The latch has two explicit clocks:
// - ROS receipt provenance rejects future/negative age and ROS rewinds;
// - monotonic elapsed time makes pause/liveness expiry independent of ROS time.
// Both ages must remain bounded; drift inflation uses the larger elapsed age.
class StationaryLatchClock
{
public:
  explicit StationaryLatchClock(const double maximum_age_s)
      : maximum_age_ns_(CogSecondsToNanoseconds(maximum_age_s))
  {
  }

  bool Latch(const std::int64_t receipt_time_ns,
             const std::int64_t ros_now_ns,
             const std::int64_t monotonic_now_ns)
  {
    if (monotonic_now_ns < 0 || !mowgli_interfaces::gnss_observation_freshness::IsReceiptFresh(
                                    receipt_time_ns, ros_now_ns, maximum_age_ns_))
    {
      Reset();
      return false;
    }
    receipt_time_ns_ = receipt_time_ns;
    latch_monotonic_time_ns_ = monotonic_now_ns;
    last_ros_now_ns_ = ros_now_ns;
    last_monotonic_now_ns_ = monotonic_now_ns;
    return true;
  }

  StationaryLatchResult Check(const std::int64_t ros_now_ns, const std::int64_t monotonic_now_ns)
  {
    if (!receipt_time_ns_ || !latch_monotonic_time_ns_)
    {
      return {StationaryLatchState::kNoLatch, 0.0};
    }
    if ((last_ros_now_ns_ && ros_now_ns < *last_ros_now_ns_) ||
        (last_monotonic_now_ns_ && monotonic_now_ns < *last_monotonic_now_ns_))
    {
      Reset();
      return {StationaryLatchState::kClockDiscontinuity, 0.0};
    }
    last_ros_now_ns_ = ros_now_ns;
    last_monotonic_now_ns_ = monotonic_now_ns;

    if (ros_now_ns < *receipt_time_ns_)
    {
      Reset();
      return {StationaryLatchState::kInvalidProvenance, 0.0};
    }

    const std::int64_t ros_age_ns = ros_now_ns - *receipt_time_ns_;
    const std::int64_t monotonic_age_ns = monotonic_now_ns - *latch_monotonic_time_ns_;
    if (ros_age_ns > maximum_age_ns_ || monotonic_age_ns > maximum_age_ns_)
    {
      Reset();
      return {StationaryLatchState::kExpired, 0.0};
    }

    constexpr double kNanosecondsPerSecond = 1.0e9;
    const double age_s =
        static_cast<double>(std::max(ros_age_ns, monotonic_age_ns)) / kNanosecondsPerSecond;
    return {StationaryLatchState::kFresh, age_s};
  }

  void Reset()
  {
    receipt_time_ns_.reset();
    latch_monotonic_time_ns_.reset();
    last_ros_now_ns_.reset();
    last_monotonic_now_ns_.reset();
  }

private:
  std::int64_t maximum_age_ns_;
  std::optional<std::int64_t> receipt_time_ns_;
  std::optional<std::int64_t> latch_monotonic_time_ns_;
  std::optional<std::int64_t> last_ros_now_ns_;
  std::optional<std::int64_t> last_monotonic_now_ns_;
};

}  // namespace mowgli_localization
