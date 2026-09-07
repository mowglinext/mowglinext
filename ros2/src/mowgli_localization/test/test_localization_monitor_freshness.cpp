// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdint>

#include "mowgli_interfaces/gnss_observation_freshness.hpp"
#include "mowgli_localization/localization_monitor_policy.hpp"
#include <gtest/gtest.h>

namespace freshness = mowgli_interfaces::gnss_observation_freshness;
namespace ml = mowgli_localization;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

TEST(LocalizationMonitorFreshness, GenuineFixedObservationReportsFresh)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  const bool fresh = tracker.ObservationIsFresh(10 * kSecond, 1 * kSecond, 2 * kSecond);
  EXPECT_EQ(ml::EvaluateLocalizationMode(fresh, true, true), ml::LocalizationMode::RTK_FIXED);
}

TEST(LocalizationMonitorFreshness, RepeatedCallbacksDoNotRefreshObservation)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(0, 10 * kSecond, 11 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  EXPECT_EQ(tracker.Observe(0, 10 * kSecond, 12 * kSecond, 3 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  EXPECT_EQ(tracker.last_receipt_time_ns(), 10 * kSecond);
}

TEST(LocalizationMonitorFreshness, CachedDeliveryEventuallyReportsDeadReckoning)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);

  EXPECT_FALSE(fresh);
  EXPECT_TRUE(tracker.DeliveryIsLive(4 * kSecond, 1 * kSecond));
  EXPECT_EQ(ml::EvaluateLocalizationMode(fresh, true, true), ml::LocalizationMode::DEAD_RECKONING);
}

TEST(LocalizationMonitorFreshness, GenuineObservationRestoresMode)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_FALSE(tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond));
  ASSERT_EQ(tracker.Observe(0, 13 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  EXPECT_EQ(ml::EvaluateLocalizationMode(fresh, true, true), ml::LocalizationMode::RTK_FIXED);
}

TEST(LocalizationMonitorPolicy, FreshFloatAndGpsOnlySemanticsAreUnchanged)
{
  EXPECT_EQ(ml::EvaluateLocalizationMode(true, true, false), ml::LocalizationMode::RTK_FLOAT);
  EXPECT_EQ(ml::EvaluateLocalizationMode(true, false, false), ml::LocalizationMode::GPS_ONLY);
  EXPECT_EQ(ml::EvaluateLocalizationMode(false, true, true), ml::LocalizationMode::DEAD_RECKONING);
}

TEST(PhysicalObservationFreshness, StartupBeforeFirstObservationIsStale)
{
  freshness::PhysicalObservationTracker tracker;
  EXPECT_FALSE(tracker.ObservationIsFresh(10 * kSecond, 1 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, SamePayloadNewSequenceIsGenuine)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(10, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(11, 10 * kSecond, 10 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.last_sequence(), 11U);
  EXPECT_TRUE(tracker.ObservationIsFresh(10 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, DelayedDuplicateDoesNotRefresh)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(10, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(11, 11 * kSecond, 11 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(10, 10 * kSecond, 12 * kSecond, 3 * kSecond),
            freshness::ObservationUpdate::kOutOfOrder);
  EXPECT_FALSE(tracker.ObservationIsFresh(14 * kSecond, 5 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, ZeroProvenanceInvalidatesAuthority)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(2, 0, 10 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kInvalidProvenance);
  EXPECT_FALSE(tracker.ObservationIsFresh(10 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, FutureProvenanceInvalidatesAuthority)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(2, 12 * kSecond, 11 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kInvalidProvenance);
  EXPECT_FALSE(tracker.ObservationIsFresh(11 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, ReceiptOnlyRewindFailsClosed)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(0, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(0, 9 * kSecond, 11 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kOutOfOrder);
  EXPECT_FALSE(tracker.ObservationIsFresh(11 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, RosRewindDuringCallbackResetsEpoch)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(5, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(5, 5 * kSecond, 5 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kRosTimeDiscontinuity);
  EXPECT_FALSE(tracker.ObservationIsFresh(5 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, RosRewindDuringEvaluationResetsEpoch)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(5, 100 * kSecond, 100 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_FALSE(tracker.ObservationIsFresh(5 * kSecond, 2 * kSecond, 2 * kSecond));
  EXPECT_FALSE(tracker.last_receipt_time_ns().has_value());
  EXPECT_TRUE(tracker.ConsumeEpochReset());
  EXPECT_FALSE(tracker.ConsumeEpochReset());
}

TEST(PhysicalObservationFreshness, ReceiverSequenceRestartUsesNewEpochObservation)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(50, 50 * kSecond, 50 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(1, 51 * kSecond, 51 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kSourceRestart);
  EXPECT_EQ(tracker.last_sequence(), 1U);
  EXPECT_EQ(tracker.last_receipt_time_ns(), 51 * kSecond);
  EXPECT_TRUE(tracker.ObservationIsFresh(51 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, LargeForwardJumpExpiresAuthority)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_FALSE(tracker.ObservationIsFresh(1000 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, OneHertzPhysicalTenHertzCallbacksFollowsPhysicalIdentity)
{
  freshness::PhysicalObservationTracker tracker;
  std::uint64_t sequence = 1;
  std::int64_t receipt = 10 * kSecond;
  std::int64_t cached_callbacks = 0;
  for (std::int64_t tenth = 0; tenth <= 20; ++tenth)
  {
    const std::int64_t ros_now = 10 * kSecond + tenth * (kSecond / 10);
    if (tenth > 0 && tenth % 10 == 0)
    {
      ++sequence;
      receipt = ros_now;
      EXPECT_EQ(tracker.Observe(sequence, receipt, ros_now, tenth * (kSecond / 10)),
                freshness::ObservationUpdate::kNewObservation);
    }
    else if (tenth == 0)
    {
      EXPECT_EQ(tracker.Observe(sequence, receipt, ros_now, 0),
                freshness::ObservationUpdate::kNewObservation);
    }
    else
    {
      ++cached_callbacks;
      EXPECT_EQ(tracker.Observe(sequence, receipt, ros_now, tenth * (kSecond / 10)),
                freshness::ObservationUpdate::kCachedPublication);
    }
  }
  EXPECT_EQ(cached_callbacks, 18);
  EXPECT_EQ(tracker.last_sequence(), 3U);
  EXPECT_TRUE(tracker.ObservationIsFresh(12 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, SparsePublicationDoesNotInferAcquisitionCadence)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(10, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.Observe(20, 11 * kSecond, 11 * kSecond, 2 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_EQ(tracker.last_sequence(), 20U);
  EXPECT_TRUE(tracker.ObservationIsFresh(11 * kSecond, 2 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, DeliveryLivenessRemainsSeparateFromObservationFreshness)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  EXPECT_TRUE(tracker.DeliveryIsLive(4 * kSecond, 1 * kSecond));
  EXPECT_FALSE(tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond));
}

TEST(PhysicalObservationFreshness, MonotonicRewindResetsAuthority)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 10 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_FALSE(tracker.ObservationIsFresh(10 * kSecond, 9 * kSecond, 2 * kSecond));
  EXPECT_FALSE(tracker.last_receipt_time_ns().has_value());
}

TEST(PhysicalObservationFreshness, NegativeMaximumAgeCannotAuthorize)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  EXPECT_FALSE(tracker.ObservationIsFresh(10 * kSecond, 1 * kSecond, -1));
}

}  // namespace
