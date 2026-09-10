// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

#include "mowgli_interfaces/gnss_observation_freshness.hpp"
#include <gtest/gtest.h>

namespace gnss = mowgli_interfaces::gnss_observation_freshness;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

bool IsAcceptedEvidence(const gnss::ObservationUpdate update)
{
  return update == gnss::ObservationUpdate::kNewObservation ||
         update == gnss::ObservationUpdate::kSourceRestart;
}

TEST(GnssObservationFreshness, SequencePreservesCachedAndIdenticalObservationSemantics)
{
  gnss::ObservationTracker tracker;

  EXPECT_EQ(tracker.Observe(10, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(10, 100 * kSecond, 101 * kSecond, 2 * kSecond),
            gnss::ObservationUpdate::kCachedPublication);

  // Numerically identical observations can share every payload value. A new
  // upstream sequence remains distinct even if ROS time is paused.
  EXPECT_EQ(tracker.Observe(11, 100 * kSecond, 101 * kSecond, 3 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.last_sequence(), 11U);
}

TEST(GnssObservationFreshness, ReceiptOnlyGatePreventsDuplicateFactorsAndLatchRefresh)
{
  gnss::ObservationTracker tracker;
  int factor_submissions = 0;
  int rtk_streak = 0;
  std::int64_t last_rtk_receipt_ns = 0;

  const auto feed_fixed = [&](const std::int64_t receipt_ns,
                              const std::int64_t ros_now_ns,
                              const std::int64_t delivery_ns)
  {
    const auto update = tracker.Observe(0, receipt_ns, ros_now_ns, delivery_ns);
    if (IsAcceptedEvidence(update))
    {
      ++factor_submissions;
      ++rtk_streak;
      last_rtk_receipt_ns = receipt_ns;
    }
    return update;
  };

  EXPECT_EQ(feed_fixed(10 * kSecond, 10 * kSecond, 1 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_EQ(feed_fixed(10 * kSecond, 11 * kSecond, 2 * kSecond),
            gnss::ObservationUpdate::kCachedPublication);
  EXPECT_EQ(feed_fixed(10 * kSecond, 12 * kSecond, 3 * kSecond),
            gnss::ObservationUpdate::kCachedPublication);
  EXPECT_EQ(factor_submissions, 1);
  EXPECT_EQ(rtk_streak, 1);
  EXPECT_EQ(last_rtk_receipt_ns, 10 * kSecond);

  // Same coordinates are deliberately absent from the identity decision. A
  // new receiver-receipt stamp is accepted normally.
  EXPECT_EQ(feed_fixed(13 * kSecond, 13 * kSecond, 4 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_EQ(factor_submissions, 2);
  EXPECT_EQ(rtk_streak, 2);
  EXPECT_EQ(last_rtk_receipt_ns, 13 * kSecond);

  // A delayed duplicate cannot refresh latches or submit another factor.
  EXPECT_EQ(feed_fixed(10 * kSecond, 14 * kSecond, 5 * kSecond),
            gnss::ObservationUpdate::kOutOfOrder);
  EXPECT_EQ(factor_submissions, 2);
  EXPECT_EQ(rtk_streak, 2);
  EXPECT_EQ(last_rtk_receipt_ns, 13 * kSecond);
}

TEST(GnssObservationFreshness, DeliveryLivenessCannotRefreshObservationFreshness)
{
  gnss::ObservationTracker tracker;
  ASSERT_TRUE(IsAcceptedEvidence(tracker.Observe(25, 10 * kSecond, 10 * kSecond, 1 * kSecond)));

  EXPECT_EQ(tracker.Observe(25, 10 * kSecond, 20 * kSecond, 5 * kSecond),
            gnss::ObservationUpdate::kCachedPublication);
  EXPECT_FALSE(tracker.ObservationIsFresh(20 * kSecond, 2 * kSecond));
  EXPECT_TRUE(tracker.DeliveryIsLive(6 * kSecond, 2 * kSecond));
  EXPECT_FALSE(tracker.DeliveryIsLive(8 * kSecond, 2 * kSecond));
}

TEST(GnssObservationFreshness, ClockRewindAndFutureProvenanceFailClosed)
{
  gnss::ObservationTracker tracker;
  ASSERT_TRUE(IsAcceptedEvidence(tracker.Observe(0, 100 * kSecond, 100 * kSecond, 1 * kSecond)));

  EXPECT_EQ(tracker.Observe(0, 5 * kSecond, 5 * kSecond, 2 * kSecond),
            gnss::ObservationUpdate::kRosTimeDiscontinuity);
  EXPECT_FALSE(tracker.ObservationIsFresh(5 * kSecond, 2 * kSecond));

  // Cached data from the old clock epoch is now future provenance.
  EXPECT_EQ(tracker.Observe(0, 100 * kSecond, 5 * kSecond, 3 * kSecond),
            gnss::ObservationUpdate::kInvalidProvenance);
  EXPECT_FALSE(tracker.ObservationIsFresh(5 * kSecond, 2 * kSecond));

  EXPECT_EQ(tracker.Observe(0, 6 * kSecond, 6 * kSecond, 4 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_FALSE(tracker.ObservationIsFresh(5 * kSecond, 2 * kSecond));
  EXPECT_TRUE(tracker.ObservationIsFresh(6 * kSecond, 2 * kSecond));
}

TEST(GnssObservationFreshness, LaterReceiptWithResetSequenceStartsNewSourceEpoch)
{
  gnss::ObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(50, 50 * kSecond, 50 * kSecond, 1 * kSecond),
            gnss::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(1, 51 * kSecond, 51 * kSecond, 2 * kSecond),
            gnss::ObservationUpdate::kSourceRestart);
  EXPECT_EQ(tracker.last_sequence(), 1U);
}

}  // namespace
