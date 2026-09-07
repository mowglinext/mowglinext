// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_localization
{

// Result of adding one delivery to the bounded association state. Complete
// pairs are deliberately withheld until the monotonic association window
// closes, so a conflicting sequence on the same receipt stamp can fail closed
// before any projection is published.
enum class AssociationEvent
{
  kNone,
  kWaiting,
  kDuplicateFix,
  kCachedStatus,
  kClosedReceipt,
  kInvalidProvenance,
  kInvalidIdentity,
  kAmbiguousReceipt,
  kReceiptRewind,
  kSourceRestart,
  kClockDiscontinuity,
  kCapacityEviction,
};

struct AssociatedNavSatObservation
{
  sensor_msgs::msg::NavSatFix fix;
  mowgli_interfaces::msg::GnssStatus status;
};

struct AssociationBatch
{
  AssociationEvent event{AssociationEvent::kNone};
  std::int64_t event_receipt_time_ns{0};
  std::vector<AssociatedNavSatObservation> ready;
  std::vector<sensor_msgs::msg::NavSatFix> fix_only_fallbacks;
  std::size_t expired_incomplete{0};
  std::size_t expired_ambiguous{0};
  std::size_t expired_invalid_provenance{0};
  std::size_t capacity_evictions{0};
};

// Exact, bounded rendez-vous for independently delivered /gps/fix and
// /gps/status observations.
//
// Identity and ordering invariants:
// - receipt provenance (header.stamp) is the only cross-topic pairing key;
// - coordinates are never compared;
// - a non-zero status sequence identifies genuine versus cached status;
// - one receipt carrying different status sequences is ambiguous and dropped;
// - completed, expired, evicted, and rejected receipts cannot be resurrected;
// - queue deadlines use monotonic time; receipt validity uses ROS time;
// - a ROS/monotonic clock rewind clears the association epoch and drops the
//   triggering delivery.
class NavSatStatusAssociation
{
public:
  NavSatStatusAssociation(std::size_t maximum_pending_receipts,
                          std::int64_t association_window_ns,
                          std::int64_t maximum_provenance_age_ns);

  AssociationBatch AddFix(const sensor_msgs::msg::NavSatFix& fix,
                          std::int64_t receipt_time_ns,
                          std::int64_t ros_now_ns,
                          std::int64_t monotonic_now_ns);

  AssociationBatch AddStatus(const mowgli_interfaces::msg::GnssStatus& status,
                             std::int64_t receipt_time_ns,
                             std::int64_t ros_now_ns,
                             std::int64_t monotonic_now_ns);

  // Close every association whose monotonic window has elapsed. Complete,
  // unambiguous pairs are returned exactly once. A fix-only entry is returned
  // separately for the safe AbsolutePose-only fallback; it never gains typed
  // RTK authority. Status-only and ambiguous entries are dropped.
  AssociationBatch Poll(std::int64_t ros_now_ns, std::int64_t monotonic_now_ns);

  void Reset();

  std::size_t pending_size() const
  {
    return pending_.size();
  }

private:
  struct PendingObservation
  {
    std::int64_t first_delivery_time_ns{0};
    std::optional<sensor_msgs::msg::NavSatFix> fix;
    std::optional<mowgli_interfaces::msg::GnssStatus> status;
    bool ambiguous{false};
  };

  using PendingMap = std::map<std::int64_t, PendingObservation>;

  bool Prepare(std::int64_t ros_now_ns, std::int64_t monotonic_now_ns, AssociationBatch& batch);
  void CollectDue(std::int64_t ros_now_ns, std::int64_t monotonic_now_ns, AssociationBatch& batch);
  PendingMap::iterator CreatePending(std::int64_t receipt_time_ns,
                                     std::int64_t monotonic_now_ns,
                                     AssociationBatch& batch);
  void CloseAndErase(PendingMap::iterator entry);
  void CloseReceipt(std::int64_t receipt_time_ns);
  void ResetAssociationEpoch();
  bool ReceiptIsClosed(std::int64_t receipt_time_ns) const;
  bool ReceiptCanStart(std::int64_t receipt_time_ns, AssociationBatch& batch) const;
  bool ProvenanceIsValid(std::int64_t receipt_time_ns, std::int64_t ros_now_ns) const;

  std::size_t maximum_pending_receipts_;
  std::int64_t association_window_ns_;
  std::int64_t maximum_provenance_age_ns_;

  PendingMap pending_;
  std::optional<std::int64_t> greatest_receipt_seen_ns_;
  std::optional<std::int64_t> greatest_closed_receipt_ns_;
  std::optional<std::uint64_t> latest_status_sequence_;
  std::optional<std::int64_t> latest_status_receipt_ns_;
  std::optional<std::int64_t> last_ros_now_ns_;
  std::optional<std::int64_t> last_monotonic_now_ns_;
};

}  // namespace mowgli_localization
