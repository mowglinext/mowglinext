// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_localization/navsat_status_association.hpp"

#include <algorithm>
#include <utility>

#include "mowgli_interfaces/gnss_observation_freshness.hpp"

namespace mowgli_localization
{

NavSatStatusAssociation::NavSatStatusAssociation(const std::size_t maximum_pending_receipts,
                                                 const std::int64_t association_window_ns,
                                                 const std::int64_t maximum_provenance_age_ns)
    : maximum_pending_receipts_(std::max<std::size_t>(maximum_pending_receipts, 1U)),
      association_window_ns_(std::max<std::int64_t>(association_window_ns, 0)),
      maximum_provenance_age_ns_(std::max<std::int64_t>(maximum_provenance_age_ns, 0))
{
}

AssociationBatch NavSatStatusAssociation::AddFix(const sensor_msgs::msg::NavSatFix& fix,
                                                 const std::int64_t receipt_time_ns,
                                                 const std::int64_t ros_now_ns,
                                                 const std::int64_t monotonic_now_ns)
{
  AssociationBatch batch;
  batch.event_receipt_time_ns = receipt_time_ns;
  if (!Prepare(ros_now_ns, monotonic_now_ns, batch))
  {
    return batch;
  }
  if (!ProvenanceIsValid(receipt_time_ns, ros_now_ns))
  {
    batch.event = AssociationEvent::kInvalidProvenance;
    return batch;
  }

  auto entry = pending_.find(receipt_time_ns);
  if (entry == pending_.end())
  {
    if (!ReceiptCanStart(receipt_time_ns, batch))
    {
      return batch;
    }
    entry = CreatePending(receipt_time_ns, monotonic_now_ns, batch);
  }

  if (entry->second.fix.has_value())
  {
    batch.event = AssociationEvent::kDuplicateFix;
    return batch;
  }

  entry->second.fix = fix;
  if (batch.event == AssociationEvent::kNone)
  {
    batch.event = AssociationEvent::kWaiting;
  }
  return batch;
}

AssociationBatch NavSatStatusAssociation::AddStatus(
    const mowgli_interfaces::msg::GnssStatus& status,
    const std::int64_t receipt_time_ns,
    const std::int64_t ros_now_ns,
    const std::int64_t monotonic_now_ns)
{
  AssociationBatch batch;
  batch.event_receipt_time_ns = receipt_time_ns;
  if (!Prepare(ros_now_ns, monotonic_now_ns, batch))
  {
    return batch;
  }
  if (!ProvenanceIsValid(receipt_time_ns, ros_now_ns))
  {
    batch.event = AssociationEvent::kInvalidProvenance;
    return batch;
  }

  const std::uint64_t sequence = status.position_observation_sequence;
  auto entry = pending_.find(receipt_time_ns);

  // Universal GNSS always supplies a non-zero position sequence. Accepting a
  // legacy zero here would make same-stamp distinct observations impossible to
  // disambiguate, so typed RTK authority fails closed instead.
  if (sequence == 0)
  {
    if (entry != pending_.end())
    {
      CloseAndErase(entry);
    }
    else
    {
      CloseReceipt(receipt_time_ns);
    }
    batch.event = AssociationEvent::kInvalidIdentity;
    return batch;
  }

  if (ReceiptIsClosed(receipt_time_ns) && entry == pending_.end())
  {
    batch.event = AssociationEvent::kClosedReceipt;
    return batch;
  }

  if (entry != pending_.end() && entry->second.status.has_value())
  {
    if (entry->second.status->position_observation_sequence == sequence)
    {
      batch.event = AssociationEvent::kCachedStatus;
    }
    else
    {
      entry->second.status.reset();
      entry->second.ambiguous = true;
      batch.event = AssociationEvent::kAmbiguousReceipt;
    }
    return batch;
  }

  bool source_restart = false;
  if (latest_status_sequence_ && latest_status_receipt_ns_)
  {
    const std::uint64_t latest_sequence = *latest_status_sequence_;
    const std::int64_t latest_receipt = *latest_status_receipt_ns_;

    // The SAME position observation, re-published under a LATER receipt stamp.
    // This is the receiver's normal steady state, not an identity violation:
    // Universal GNSS publishes fix+status together on a timer (publish_rate_hz,
    // 10 Hz by default) while the receipt clock advances on ANY accepted field
    // merge — a satellite-count or CN0 update is enough — whereas
    // position_observation_sequence advances only on an accepted POSITION
    // observation. So between two fixes the stamp moves and the sequence does
    // not, and treating that as ambiguous silenced /gps/absolute_pose and
    // /gps/pose_cov completely (100 % of observations dropped at 10 Hz publish
    // / 5 Hz position rate). Handle it exactly like a cached status: emit
    // nothing for this receipt — one output per PHYSICAL observation is the
    // point of the whole gate — and leave other pending associations alone.
    if (sequence == latest_sequence && receipt_time_ns > latest_receipt)
    {
      if (entry != pending_.end())
      {
        CloseAndErase(entry);
      }
      else
      {
        CloseReceipt(receipt_time_ns);
      }
      latest_status_receipt_ns_ = receipt_time_ns;
      batch.event = AssociationEvent::kCachedStatus;
      return batch;
    }

    // A repeated sequence on an EARLIER stamp, or two sequences on one stamp,
    // does violate the upstream identity invariant. Mark any still-pending
    // counterpart as ambiguous and never guess which observation the fix
    // represents.
    if ((sequence == latest_sequence && receipt_time_ns < latest_receipt) ||
        (sequence != latest_sequence && receipt_time_ns == latest_receipt))
    {
      auto prior = pending_.find(latest_receipt);
      if (prior != pending_.end())
      {
        prior->second.status.reset();
        prior->second.ambiguous = true;
      }
      if (entry != pending_.end())
      {
        entry->second.status.reset();
        entry->second.ambiguous = true;
      }
      else
      {
        CloseReceipt(receipt_time_ns);
      }
      batch.event = AssociationEvent::kAmbiguousReceipt;
      return batch;
    }

    // A lower sequence with a later receipt is an explicit receiver restart.
    // Preserve only a fix already waiting on this exact new receipt; every
    // older association belongs to the previous source epoch.
    source_restart = sequence < latest_sequence && receipt_time_ns > latest_receipt;
    if (source_restart)
    {
      std::optional<PendingObservation> exact_waiting_fix;
      if (entry != pending_.end() && entry->second.fix.has_value() && !entry->second.ambiguous)
      {
        exact_waiting_fix = std::move(entry->second);
      }
      ResetAssociationEpoch();
      if (exact_waiting_fix)
      {
        pending_.emplace(receipt_time_ns, std::move(*exact_waiting_fix));
        greatest_receipt_seen_ns_ = receipt_time_ns;
      }
      entry = pending_.find(receipt_time_ns);
      batch.event = AssociationEvent::kSourceRestart;
    }
    else if (sequence > latest_sequence && receipt_time_ns < latest_receipt)
    {
      // A continuing sequence whose receipt time moves backward is neither a
      // valid backlog observation nor a restart. Isolate this receipt only.
      if (entry != pending_.end())
      {
        CloseAndErase(entry);
      }
      else
      {
        CloseReceipt(receipt_time_ns);
      }
      batch.event = AssociationEvent::kReceiptRewind;
      return batch;
    }
  }

  if (entry == pending_.end())
  {
    if (!ReceiptCanStart(receipt_time_ns, batch))
    {
      return batch;
    }
    entry = CreatePending(receipt_time_ns, monotonic_now_ns, batch);
  }

  entry->second.status = status;
  if (!latest_status_receipt_ns_ || receipt_time_ns > *latest_status_receipt_ns_ || source_restart)
  {
    latest_status_sequence_ = sequence;
    latest_status_receipt_ns_ = receipt_time_ns;
  }
  if (batch.event == AssociationEvent::kNone)
  {
    batch.event = AssociationEvent::kWaiting;
  }
  return batch;
}

AssociationBatch NavSatStatusAssociation::Poll(const std::int64_t ros_now_ns,
                                               const std::int64_t monotonic_now_ns)
{
  AssociationBatch batch;
  Prepare(ros_now_ns, monotonic_now_ns, batch);
  return batch;
}

void NavSatStatusAssociation::Reset()
{
  ResetAssociationEpoch();
  last_ros_now_ns_.reset();
  last_monotonic_now_ns_.reset();
}

bool NavSatStatusAssociation::Prepare(const std::int64_t ros_now_ns,
                                      const std::int64_t monotonic_now_ns,
                                      AssociationBatch& batch)
{
  if ((last_ros_now_ns_ && ros_now_ns < *last_ros_now_ns_) ||
      (last_monotonic_now_ns_ && monotonic_now_ns < *last_monotonic_now_ns_))
  {
    ResetAssociationEpoch();
    last_ros_now_ns_ = ros_now_ns;
    last_monotonic_now_ns_ = monotonic_now_ns;
    batch.event = AssociationEvent::kClockDiscontinuity;
    return false;
  }

  last_ros_now_ns_ = ros_now_ns;
  last_monotonic_now_ns_ = monotonic_now_ns;
  CollectDue(ros_now_ns, monotonic_now_ns, batch);
  return true;
}

void NavSatStatusAssociation::CollectDue(const std::int64_t ros_now_ns,
                                         const std::int64_t monotonic_now_ns,
                                         AssociationBatch& batch)
{
  for (auto entry = pending_.begin(); entry != pending_.end();)
  {
    const std::int64_t first_delivery = entry->second.first_delivery_time_ns;
    if (monotonic_now_ns < first_delivery ||
        monotonic_now_ns - first_delivery < association_window_ns_)
    {
      ++entry;
      continue;
    }

    if (!ProvenanceIsValid(entry->first, ros_now_ns))
    {
      ++batch.expired_invalid_provenance;
    }
    else if (entry->second.ambiguous)
    {
      ++batch.expired_ambiguous;
    }
    else if (entry->second.fix && entry->second.status)
    {
      batch.ready.push_back(AssociatedNavSatObservation{std::move(*entry->second.fix),
                                                        std::move(*entry->second.status)});
    }
    else if (entry->second.fix)
    {
      batch.fix_only_fallbacks.push_back(std::move(*entry->second.fix));
    }
    else
    {
      ++batch.expired_incomplete;
    }

    CloseReceipt(entry->first);
    entry = pending_.erase(entry);
  }
}

NavSatStatusAssociation::PendingMap::iterator NavSatStatusAssociation::CreatePending(
    const std::int64_t receipt_time_ns,
    const std::int64_t monotonic_now_ns,
    AssociationBatch& batch)
{
  if (pending_.size() >= maximum_pending_receipts_)
  {
    const auto oldest = std::min_element(pending_.begin(),
                                         pending_.end(),
                                         [](const auto& lhs, const auto& rhs)
                                         {
                                           if (lhs.second.first_delivery_time_ns !=
                                               rhs.second.first_delivery_time_ns)
                                           {
                                             return lhs.second.first_delivery_time_ns <
                                                    rhs.second.first_delivery_time_ns;
                                           }
                                           return lhs.first < rhs.first;
                                         });
    CloseAndErase(oldest);
    ++batch.capacity_evictions;
    batch.event = AssociationEvent::kCapacityEviction;
  }

  greatest_receipt_seen_ns_ = greatest_receipt_seen_ns_
                                  ? std::max(*greatest_receipt_seen_ns_, receipt_time_ns)
                                  : receipt_time_ns;
  return pending_
      .emplace(receipt_time_ns,
               PendingObservation{monotonic_now_ns, std::nullopt, std::nullopt, false})
      .first;
}

void NavSatStatusAssociation::CloseAndErase(const PendingMap::iterator entry)
{
  CloseReceipt(entry->first);
  pending_.erase(entry);
}

void NavSatStatusAssociation::CloseReceipt(const std::int64_t receipt_time_ns)
{
  greatest_closed_receipt_ns_ = greatest_closed_receipt_ns_
                                    ? std::max(*greatest_closed_receipt_ns_, receipt_time_ns)
                                    : receipt_time_ns;
}

void NavSatStatusAssociation::ResetAssociationEpoch()
{
  pending_.clear();
  greatest_receipt_seen_ns_.reset();
  greatest_closed_receipt_ns_.reset();
  latest_status_sequence_.reset();
  latest_status_receipt_ns_.reset();
}

bool NavSatStatusAssociation::ReceiptIsClosed(const std::int64_t receipt_time_ns) const
{
  return greatest_closed_receipt_ns_ && receipt_time_ns <= *greatest_closed_receipt_ns_;
}

bool NavSatStatusAssociation::ReceiptCanStart(const std::int64_t receipt_time_ns,
                                              AssociationBatch& batch) const
{
  if (ReceiptIsClosed(receipt_time_ns))
  {
    batch.event = AssociationEvent::kClosedReceipt;
    return false;
  }
  if (greatest_receipt_seen_ns_ && receipt_time_ns < *greatest_receipt_seen_ns_)
  {
    batch.event = AssociationEvent::kReceiptRewind;
    return false;
  }
  return true;
}

bool NavSatStatusAssociation::ProvenanceIsValid(const std::int64_t receipt_time_ns,
                                                const std::int64_t ros_now_ns) const
{
  return mowgli_interfaces::gnss_observation_freshness::IsReceiptFresh(receipt_time_ns,
                                                                       ros_now_ns,
                                                                       maximum_provenance_age_ns_);
}

}  // namespace mowgli_localization
