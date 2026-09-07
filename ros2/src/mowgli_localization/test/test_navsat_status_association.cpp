// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "mowgli_localization/navsat_projection_utils.hpp"
#include "mowgli_localization/navsat_status_association.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_interfaces::msg::AbsolutePose;
using mowgli_interfaces::msg::GnssStatus;
using mowgli_localization::AssociatedNavSatObservation;
using mowgli_localization::AssociationBatch;
using mowgli_localization::AssociationEvent;
using mowgli_localization::NavSatStatusAssociation;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

constexpr std::int64_t kWindow = 10;
constexpr std::int64_t kMaximumAge = 1000;

NavSatFix MakeFix(const double latitude = 48.0,
                  const double longitude = 2.0,
                  const std::int8_t navsat_status = NavSatStatus::STATUS_FIX)
{
  NavSatFix fix;
  fix.latitude = latitude;
  fix.longitude = longitude;
  fix.status.status = navsat_status;
  fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 0.0001;
  fix.position_covariance[4] = 0.0001;
  return fix;
}

GnssStatus MakeStatus(const std::uint64_t sequence,
                      const std::uint8_t fix_type,
                      const std::uint8_t rtk_mode)
{
  GnssStatus status;
  status.position_observation_sequence = sequence;
  status.fix_valid = fix_type != GnssStatus::FIX_TYPE_NO_FIX;
  status.fix_type = fix_type;
  status.rtk_mode = rtk_mode;
  return status;
}

GnssStatus Fixed(const std::uint64_t sequence)
{
  return MakeStatus(sequence, GnssStatus::FIX_TYPE_RTK_FIXED, GnssStatus::RTK_MODE_FIXED);
}

GnssStatus Float(const std::uint64_t sequence)
{
  return MakeStatus(sequence, GnssStatus::FIX_TYPE_RTK_FLOAT, GnssStatus::RTK_MODE_FLOAT);
}

GnssStatus NoFix(const std::uint64_t sequence)
{
  return MakeStatus(sequence, GnssStatus::FIX_TYPE_NO_FIX, GnssStatus::RTK_MODE_NONE);
}

std::uint8_t Flags(const AssociatedNavSatObservation& observation)
{
  return mowgli_localization::ResolveAbsolutePoseFlags(std::make_optional(observation.status),
                                                       observation.fix.status.status);
}

void AppendReady(AssociationBatch batch, std::vector<AssociatedNavSatObservation>& output)
{
  for (auto& observation : batch.ready)
  {
    output.push_back(std::move(observation));
  }
}

TEST(NavSatStatusAssociationTest, FixFirstThenMatchingStatusProjectsExactlyOnce)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  EXPECT_EQ(association.AddFix(MakeFix(), 100, 100, 0).event, AssociationEvent::kWaiting);
  EXPECT_EQ(association.AddStatus(Fixed(1), 100, 100, 1).event, AssociationEvent::kWaiting);

  auto ready = association.Poll(100, 10);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_NE(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FIXED, 0U);
  EXPECT_TRUE(association.Poll(100, 11).ready.empty());
}

TEST(NavSatStatusAssociationTest, StatusFirstThenMatchingFixProjectsExactlyOnce)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Fixed(1), 100, 100, 0);
  association.AddFix(MakeFix(), 100, 100, 1);

  auto ready = association.Poll(100, 10);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_EQ(ready.ready.front().status.position_observation_sequence, 1U);
}

TEST(NavSatStatusAssociationTest, NewFloatEpochCannotInheritOlderFixedStatus)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  association.AddStatus(Fixed(1), 100, 100, 1);
  ASSERT_EQ(association.Poll(100, 10).ready.size(), 1U);

  association.AddFix(MakeFix(), 101, 101, 11);
  association.AddStatus(Float(2), 101, 101, 12);
  auto ready = association.Poll(101, 21);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_NE(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FLOAT, 0U);
  EXPECT_EQ(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FIXED, 0U);
}

TEST(NavSatStatusAssociationTest, NewFixedEpochCannotInheritOlderFloatStatus)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Float(1), 100, 100, 0);
  association.AddFix(MakeFix(), 100, 100, 1);
  ASSERT_EQ(association.Poll(100, 10).ready.size(), 1U);

  association.AddStatus(Fixed(2), 101, 101, 11);
  association.AddFix(MakeFix(), 101, 101, 12);
  auto ready = association.Poll(101, 21);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_NE(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FIXED, 0U);
  EXPECT_EQ(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FLOAT, 0U);
}

TEST(NavSatStatusAssociationTest, CachedStatusDoesNotCreateAnotherAssociation)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Fixed(7), 100, 100, 0);
  EXPECT_EQ(association.AddStatus(Fixed(7), 100, 100, 1).event, AssociationEvent::kCachedStatus);
  association.AddFix(MakeFix(), 100, 100, 2);

  EXPECT_EQ(association.Poll(100, 10).ready.size(), 1U);
  EXPECT_EQ(association.AddStatus(Fixed(7), 100, 100, 11).event, AssociationEvent::kClosedReceipt);
  EXPECT_TRUE(association.Poll(100, 20).ready.empty());
}

TEST(NavSatStatusAssociationTest, DuplicateFixProducesOneProjectedObservation)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  EXPECT_EQ(association.AddFix(MakeFix(), 100, 100, 1).event, AssociationEvent::kDuplicateFix);
  association.AddStatus(Fixed(1), 100, 100, 2);
  EXPECT_EQ(association.Poll(100, 10).ready.size(), 1U);
}

TEST(NavSatStatusAssociationTest, SameCoordinatesWithNewReceiptAndSequenceAreNew)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  std::vector<AssociatedNavSatObservation> output;
  const NavSatFix identical_fix = MakeFix(48.123, 2.456);

  association.AddFix(identical_fix, 100, 100, 0);
  association.AddStatus(Fixed(1), 100, 100, 1);
  AppendReady(association.Poll(100, 10), output);

  association.AddFix(identical_fix, 101, 101, 11);
  association.AddStatus(Fixed(2), 101, 101, 12);
  AppendReady(association.Poll(101, 21), output);

  ASSERT_EQ(output.size(), 2U);
  EXPECT_DOUBLE_EQ(output[0].fix.latitude, output[1].fix.latitude);
  EXPECT_NE(output[0].status.position_observation_sequence,
            output[1].status.position_observation_sequence);
}

TEST(NavSatStatusAssociationTest, UnrelatedStaleStatusNeverAuthorizesNewerFix)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Fixed(1), 100, 101, 0);
  association.AddFix(MakeFix(), 101, 101, 1);

  auto expired = association.Poll(101, 11);
  EXPECT_TRUE(expired.ready.empty());
  ASSERT_EQ(expired.fix_only_fallbacks.size(), 1U);
  EXPECT_EQ(expired.expired_incomplete, 1U);
  EXPECT_EQ(expired.fix_only_fallbacks.front().status.status, NavSatStatus::STATUS_FIX);
}

TEST(NavSatStatusAssociationTest, ExactDelayedStatusWinsAfterUnrelatedStatus)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 101, 0);
  association.AddStatus(Float(2), 101, 101, 1);
  association.AddStatus(Fixed(1), 100, 101, 2);

  auto ready = association.Poll(101, 10);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_EQ(ready.ready.front().status.position_observation_sequence, 1U);
  EXPECT_NE(Flags(ready.ready.front()) & AbsolutePose::FLAG_GPS_RTK_FIXED, 0U);
}

TEST(NavSatStatusAssociationTest, MissingCounterpartExpiresAndCannotBeResurrected)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  auto expired = association.Poll(100, 10);
  EXPECT_EQ(expired.expired_incomplete, 0U);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.fix_only_fallbacks.size(), 1U);

  EXPECT_EQ(association.AddStatus(Fixed(1), 100, 100, 11).event, AssociationEvent::kClosedReceipt);
  EXPECT_TRUE(association.Poll(100, 21).ready.empty());
}

TEST(NavSatStatusAssociationTest, StatusThatExpiresCannotBeResurrectedByLateFix)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Fixed(1), 100, 100, 0);
  EXPECT_EQ(association.Poll(100, 10).expired_incomplete, 1U);
  EXPECT_EQ(association.AddFix(MakeFix(), 100, 100, 11).event, AssociationEvent::kClosedReceipt);
}

TEST(NavSatStatusAssociationTest, CapacityOverflowEvictsOldestDeterministically)
{
  NavSatStatusAssociation association(2, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 102, 0);
  association.AddFix(MakeFix(), 101, 102, 1);
  auto overflow = association.AddFix(MakeFix(), 102, 102, 2);
  EXPECT_EQ(overflow.event, AssociationEvent::kCapacityEviction);
  EXPECT_EQ(overflow.capacity_evictions, 1U);
  EXPECT_EQ(association.pending_size(), 2U);
  EXPECT_EQ(association.AddStatus(Fixed(1), 100, 102, 3).event, AssociationEvent::kClosedReceipt);

  association.AddStatus(Fixed(2), 101, 102, 4);
  association.AddStatus(Fixed(3), 102, 102, 5);
  auto ready = association.Poll(102, 12);
  ASSERT_EQ(ready.ready.size(), 2U);
  EXPECT_EQ(ready.ready[0].status.position_observation_sequence, 2U);
  EXPECT_EQ(ready.ready[1].status.position_observation_sequence, 3U);
}

TEST(NavSatStatusAssociationTest, ReceiptAndRosClockRewindsFailClosed)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  EXPECT_EQ(association.AddFix(MakeFix(), 99, 100, 1).event, AssociationEvent::kReceiptRewind);

  EXPECT_EQ(association.AddStatus(Fixed(1), 5, 5, 2).event, AssociationEvent::kClockDiscontinuity);
  EXPECT_EQ(association.pending_size(), 0U);

  association.AddFix(MakeFix(), 6, 6, 3);
  association.AddStatus(Fixed(1), 6, 6, 4);
  EXPECT_EQ(association.Poll(6, 13).ready.size(), 1U);
}

TEST(NavSatStatusAssociationTest, FutureAndInvalidProvenanceProduceNoProjection)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  EXPECT_EQ(association.AddFix(MakeFix(), 101, 100, 0).event, AssociationEvent::kInvalidProvenance);
  EXPECT_EQ(association.AddStatus(Fixed(1), 0, 100, 1).event, AssociationEvent::kInvalidProvenance);
  EXPECT_EQ(association.AddStatus(Fixed(0), 100, 100, 2).event, AssociationEvent::kInvalidIdentity);
  EXPECT_TRUE(association.Poll(100, 20).ready.empty());
}

TEST(NavSatStatusAssociationTest, RosForwardJumpCannotReleaseNowStalePair)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  association.AddStatus(Fixed(1), 100, 100, 1);

  auto expired = association.Poll(1101, 10);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.expired_invalid_provenance, 1U);
}

TEST(NavSatStatusAssociationTest, ConflictingSequencesOnSameReceiptAreAmbiguous)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  association.AddStatus(Fixed(10), 100, 100, 1);
  EXPECT_EQ(association.AddStatus(Float(11), 100, 100, 2).event,
            AssociationEvent::kAmbiguousReceipt);

  auto expired = association.Poll(100, 10);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.expired_ambiguous, 1U);
}

TEST(NavSatStatusAssociationTest, FixedFloatNoFixFixedTransitionsStayEpochLocal)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  std::vector<AssociatedNavSatObservation> output;

  association.AddFix(MakeFix(), 100, 100, 0);
  association.AddStatus(Fixed(1), 100, 100, 1);
  AppendReady(association.Poll(100, 10), output);

  association.AddFix(MakeFix(), 101, 101, 11);
  association.AddStatus(Float(2), 101, 101, 12);
  AppendReady(association.Poll(101, 21), output);

  association.AddFix(MakeFix(48.0, 2.0, NavSatStatus::STATUS_NO_FIX), 102, 102, 22);
  association.AddStatus(NoFix(3), 102, 102, 23);
  AppendReady(association.Poll(102, 32), output);

  association.AddFix(MakeFix(), 103, 103, 33);
  association.AddStatus(Fixed(4), 103, 103, 34);
  AppendReady(association.Poll(103, 43), output);

  ASSERT_EQ(output.size(), 4U);
  EXPECT_EQ(output[0].status.fix_type, GnssStatus::FIX_TYPE_RTK_FIXED);
  EXPECT_EQ(output[1].status.fix_type, GnssStatus::FIX_TYPE_RTK_FLOAT);
  EXPECT_EQ(output[2].status.fix_type, GnssStatus::FIX_TYPE_NO_FIX);
  EXPECT_EQ(output[3].status.fix_type, GnssStatus::FIX_TYPE_RTK_FIXED);
  EXPECT_EQ(output[3].status.position_observation_sequence, 4U);
}

TEST(NavSatStatusAssociationTest, LaterReceiptWithResetSequenceStartsNewSourceEpoch)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddStatus(Fixed(50), 100, 100, 0);
  association.AddFix(MakeFix(), 101, 101, 1);

  auto restart = association.AddStatus(Fixed(1), 101, 101, 2);
  EXPECT_EQ(restart.event, AssociationEvent::kSourceRestart);
  EXPECT_EQ(association.pending_size(), 1U);

  auto ready = association.Poll(101, 11);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_EQ(ready.ready.front().status.position_observation_sequence, 1U);
}

TEST(NavSatStatusAssociationTest, RepeatedSequenceOnLaterReceiptIsCachedNotAmbiguous)
{
  // Universal GNSS advances the receipt stamp on ANY accepted field merge
  // (satellite count, CN0, jamming state), but position_observation_sequence
  // only on an accepted POSITION observation. The same physical observation
  // therefore reappears under a newer stamp. That is a cached republication,
  // not an identity violation, and it must not poison the association.
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 100, 100, 0);
  association.AddStatus(Fixed(7), 100, 100, 1);

  association.AddFix(MakeFix(), 101, 101, 2);
  EXPECT_EQ(association.AddStatus(Fixed(7), 101, 101, 3).event, AssociationEvent::kCachedStatus);

  // The original observation still projects exactly once; the republication
  // contributes nothing and nothing is reported ambiguous.
  auto ready = association.Poll(101, 12);
  ASSERT_EQ(ready.ready.size(), 1U);
  EXPECT_EQ(ready.ready.front().status.position_observation_sequence, 7U);
  EXPECT_EQ(ready.expired_ambiguous, 0U);
  EXPECT_TRUE(ready.fix_only_fallbacks.empty());
}

TEST(NavSatStatusAssociationTest, RepeatedSequenceOnEarlierReceiptStaysAmbiguous)
{
  NavSatStatusAssociation association(8, kWindow, kMaximumAge);
  association.AddFix(MakeFix(), 101, 101, 0);
  association.AddStatus(Fixed(7), 101, 101, 1);

  // Same observation identity claimed on an OLDER stamp: still a violation.
  EXPECT_EQ(association.AddStatus(Fixed(7), 100, 101, 2).event,
            AssociationEvent::kAmbiguousReceipt);

  auto expired = association.Poll(101, 12);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.expired_ambiguous, 1U);
}

TEST(NavSatStatusAssociationTest, TimerPublishFasterThanPositionRateProjectsEveryObservationOnce)
{
  // Regression for the real receiver cadence: fix+status are published
  // together on a 10 Hz timer while the position rate is 5 Hz, so the stamp
  // advances every tick and the sequence every other tick. Every physical
  // observation must project exactly once — no drops, no duplicates.
  NavSatStatusAssociation association(32, kWindow, kMaximumAge);

  std::int64_t clock = 100;
  std::uint64_t sequence = 1;
  std::size_t projected = 0;
  std::size_t ambiguous = 0;
  std::size_t fix_only = 0;

  for (int tick = 0; tick < 20; ++tick)
  {
    if (tick > 0 && tick % 2 == 0)
    {
      ++sequence;
    }
    const std::int64_t receipt = clock;
    association.AddFix(MakeFix(), receipt, clock, clock);
    association.AddStatus(Fixed(sequence), receipt, clock, clock);

    clock += 1;
    const AssociationBatch batch = association.Poll(clock, clock);
    projected += batch.ready.size();
    ambiguous += batch.expired_ambiguous;
    fix_only += batch.fix_only_fallbacks.size();
  }

  clock += kWindow + 1;
  const AssociationBatch drain = association.Poll(clock, clock);
  projected += drain.ready.size();
  ambiguous += drain.expired_ambiguous;
  fix_only += drain.fix_only_fallbacks.size();

  EXPECT_EQ(projected, 10U);  // 20 publish ticks, 10 position observations
  EXPECT_EQ(ambiguous, 0U);
  EXPECT_EQ(fix_only, 0U);
}

}  // namespace
