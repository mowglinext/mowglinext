// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Shared GNSS observation-identity and freshness policy.
//
// Receipt provenance is expressed in the ROS clock domain. Delivery liveness
// is expressed independently in a monotonic clock domain. Callers must never
// substitute callback time for receipt time or mix the two clock domains.

#pragma once

#include <cstdint>
#include <optional>

namespace mowgli_interfaces::gnss_observation_freshness
{

enum class ObservationUpdate
{
  kNewObservation,
  kSourceRestart,
  kCachedPublication,
  kOutOfOrder,
  kInvalidProvenance,
  kRosTimeDiscontinuity,
  kMonotonicTimeDiscontinuity,
};

inline bool IsAcceptedObservation(const ObservationUpdate update)
{
  return update == ObservationUpdate::kNewObservation ||
         update == ObservationUpdate::kSourceRestart;
}

inline bool IsReceiptFresh(const std::int64_t receipt_time_ns,
                           const std::int64_t ros_now_ns,
                           const std::int64_t maximum_age_ns)
{
  if (receipt_time_ns <= 0 || maximum_age_ns < 0 || ros_now_ns < receipt_time_ns)
  {
    return false;
  }
  return ros_now_ns - receipt_time_ns <= maximum_age_ns;
}

class ObservationTracker
{
public:
  // Record one delivered status/fix.
  //
  // sequence=0 selects the backward-compatible receipt-stamp identity used by
  // NavSatFix and legacy status producers. A non-zero sequence is stronger:
  // it distinguishes genuine numerically identical observations even if their
  // receipt timestamps happen to be equal.
  ObservationUpdate Observe(const std::uint64_t sequence,
                            const std::int64_t receipt_time_ns,
                            const std::int64_t ros_now_ns,
                            const std::int64_t monotonic_delivery_time_ns)
  {
    last_delivery_time_ns_ = monotonic_delivery_time_ns;

    if (last_ros_now_ns_ && ros_now_ns < *last_ros_now_ns_)
    {
      ResetObservationEpoch();
      last_ros_now_ns_ = ros_now_ns;
      return ObservationUpdate::kRosTimeDiscontinuity;
    }
    last_ros_now_ns_ = ros_now_ns;

    // Receipt stamps are mandatory for the audited Universal GNSS path. A
    // future stamp is invalid evidence, not a fresh observation.
    if (receipt_time_ns <= 0 || receipt_time_ns > ros_now_ns)
    {
      return ObservationUpdate::kInvalidProvenance;
    }

    if (!last_receipt_time_ns_)
    {
      Accept(sequence, receipt_time_ns);
      return ObservationUpdate::kNewObservation;
    }

    if (sequence != 0 && last_sequence_ != 0)
    {
      if (sequence == last_sequence_)
      {
        return receipt_time_ns == *last_receipt_time_ns_ ? ObservationUpdate::kCachedPublication
                                                         : ObservationUpdate::kInvalidProvenance;
      }

      if (sequence > last_sequence_)
      {
        if (receipt_time_ns < *last_receipt_time_ns_)
        {
          return ObservationUpdate::kOutOfOrder;
        }
        Accept(sequence, receipt_time_ns);
        return ObservationUpdate::kNewObservation;
      }

      // A lower sequence paired with a later receipt stamp is a receiver
      // restart/reconnect epoch, not a delayed packet from the old epoch.
      if (receipt_time_ns > *last_receipt_time_ns_)
      {
        Accept(sequence, receipt_time_ns);
        return ObservationUpdate::kSourceRestart;
      }
      return ObservationUpdate::kOutOfOrder;
    }

    // Legacy/receipt-only identity. This is also the transition path when one
    // side has not yet supplied a non-zero sequence.
    if (receipt_time_ns == *last_receipt_time_ns_)
    {
      if (sequence != 0 && last_sequence_ == 0)
      {
        Accept(sequence, receipt_time_ns);
        return ObservationUpdate::kNewObservation;
      }
      return ObservationUpdate::kCachedPublication;
    }
    if (receipt_time_ns < *last_receipt_time_ns_)
    {
      return ObservationUpdate::kOutOfOrder;
    }

    Accept(sequence, receipt_time_ns);
    return ObservationUpdate::kNewObservation;
  }

  bool ObservationIsFresh(const std::int64_t ros_now_ns, const std::int64_t maximum_age_ns) const
  {
    return last_receipt_time_ns_ &&
           IsReceiptFresh(*last_receipt_time_ns_, ros_now_ns, maximum_age_ns);
  }

  bool DeliveryIsLive(const std::int64_t monotonic_now_ns,
                      const std::int64_t maximum_silence_ns) const
  {
    if (!last_delivery_time_ns_ || maximum_silence_ns < 0 ||
        monotonic_now_ns < *last_delivery_time_ns_)
    {
      return false;
    }
    return monotonic_now_ns - *last_delivery_time_ns_ <= maximum_silence_ns;
  }

  std::optional<std::int64_t> last_receipt_time_ns() const
  {
    return last_receipt_time_ns_;
  }

  std::uint64_t last_sequence() const
  {
    return last_sequence_;
  }

  void Reset()
  {
    ResetObservationEpoch();
    last_ros_now_ns_.reset();
    last_delivery_time_ns_.reset();
  }

private:
  void Accept(const std::uint64_t sequence, const std::int64_t receipt_time_ns)
  {
    last_sequence_ = sequence;
    last_receipt_time_ns_ = receipt_time_ns;
  }

  void ResetObservationEpoch()
  {
    last_sequence_ = 0;
    last_receipt_time_ns_.reset();
  }

  std::uint64_t last_sequence_ = 0;
  std::optional<std::int64_t> last_receipt_time_ns_;
  std::optional<std::int64_t> last_ros_now_ns_;
  std::optional<std::int64_t> last_delivery_time_ns_;
};

// Consumer-facing physical-observation freshness.
//
// ObservationTracker above intentionally keeps callback delivery liveness and
// receiver-receipt provenance as separate facts. Downstream authorization
// needs one additional rule: elapsed liveness advances only when the identity
// tracker accepts a genuinely new observation, never when DDS delivers a
// cached publication. This wrapper supplies that rule without mixing clocks:
//
// - receipt age is checked entirely in the ROS clock domain;
// - elapsed observation liveness is checked entirely in the monotonic domain;
// - both checks must pass before physical evidence is fresh.
//
// Invalid/future provenance invalidates authority immediately. An out-of-order
// sequence-bearing sample is treated as a delayed duplicate and cannot refresh
// the current deadline. In receipt-only compatibility mode, a receipt rewind
// cannot be distinguished from a producer epoch reset, so it fails closed
// until a later advancing receipt is accepted.
class PhysicalObservationTracker
{
public:
  ObservationUpdate Observe(const std::uint64_t sequence,
                            const std::int64_t receipt_time_ns,
                            const std::int64_t ros_now_ns,
                            const std::int64_t monotonic_delivery_time_ns)
  {
    if (const auto discontinuity = CheckClockContinuity(ros_now_ns, monotonic_delivery_time_ns))
    {
      return *discontinuity;
    }

    const ObservationUpdate update =
        identity_.Observe(sequence, receipt_time_ns, ros_now_ns, monotonic_delivery_time_ns);
    if (IsAcceptedObservation(update))
    {
      last_observation_monotonic_time_ns_ = monotonic_delivery_time_ns;
      evidence_valid_ = true;
    }
    else if (update == ObservationUpdate::kInvalidProvenance ||
             (update == ObservationUpdate::kOutOfOrder && sequence == 0))
    {
      evidence_valid_ = false;
    }
    else if (update == ObservationUpdate::kRosTimeDiscontinuity)
    {
      ResetEvidence();
    }
    return update;
  }

  bool ObservationIsFresh(const std::int64_t ros_now_ns,
                          const std::int64_t monotonic_now_ns,
                          const std::int64_t maximum_age_ns)
  {
    if (CheckClockContinuity(ros_now_ns, monotonic_now_ns) || !evidence_valid_ ||
        !last_observation_monotonic_time_ns_ || maximum_age_ns < 0 ||
        monotonic_now_ns < *last_observation_monotonic_time_ns_)
    {
      return false;
    }

    return identity_.ObservationIsFresh(ros_now_ns, maximum_age_ns) &&
           monotonic_now_ns - *last_observation_monotonic_time_ns_ <= maximum_age_ns;
  }

  bool DeliveryIsLive(const std::int64_t monotonic_now_ns,
                      const std::int64_t maximum_silence_ns) const
  {
    return identity_.DeliveryIsLive(monotonic_now_ns, maximum_silence_ns);
  }

  std::optional<std::int64_t> last_receipt_time_ns() const
  {
    return identity_.last_receipt_time_ns();
  }

  std::uint64_t last_sequence() const
  {
    return identity_.last_sequence();
  }

  // Consume the transition marker set when either clock moves backward. This
  // lets a semantic consumer reset its own debounce/latch epoch without
  // duplicating clock-discontinuity detection.
  bool ConsumeEpochReset()
  {
    const bool pending = epoch_reset_pending_;
    epoch_reset_pending_ = false;
    return pending;
  }

  void Reset()
  {
    identity_.Reset();
    last_observation_monotonic_time_ns_.reset();
    last_ros_now_ns_.reset();
    last_monotonic_now_ns_.reset();
    evidence_valid_ = false;
    epoch_reset_pending_ = false;
  }

private:
  std::optional<ObservationUpdate> CheckClockContinuity(const std::int64_t ros_now_ns,
                                                        const std::int64_t monotonic_now_ns)
  {
    if (last_ros_now_ns_ && ros_now_ns < *last_ros_now_ns_)
    {
      ResetEvidence();
      epoch_reset_pending_ = true;
      last_ros_now_ns_ = ros_now_ns;
      last_monotonic_now_ns_ = monotonic_now_ns;
      return ObservationUpdate::kRosTimeDiscontinuity;
    }
    if (monotonic_now_ns < 0 ||
        (last_monotonic_now_ns_ && monotonic_now_ns < *last_monotonic_now_ns_))
    {
      ResetEvidence();
      epoch_reset_pending_ = true;
      last_ros_now_ns_ = ros_now_ns;
      last_monotonic_now_ns_ = monotonic_now_ns;
      return ObservationUpdate::kMonotonicTimeDiscontinuity;
    }

    last_ros_now_ns_ = ros_now_ns;
    last_monotonic_now_ns_ = monotonic_now_ns;
    return std::nullopt;
  }

  void ResetEvidence()
  {
    identity_.Reset();
    last_observation_monotonic_time_ns_.reset();
    evidence_valid_ = false;
  }

  ObservationTracker identity_;
  std::optional<std::int64_t> last_observation_monotonic_time_ns_;
  std::optional<std::int64_t> last_ros_now_ns_;
  std::optional<std::int64_t> last_monotonic_now_ns_;
  bool evidence_valid_ = false;
  bool epoch_reset_pending_ = false;
};

}  // namespace mowgli_interfaces::gnss_observation_freshness
