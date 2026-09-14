// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <cstdint>
#include <optional>

#include "mowgli_localization/cog_observation_timing.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_localization::CogObservationTiming;
using mowgli_localization::CogObservationTimingDecision;
using mowgli_localization::DeriveMaximumCogObservationIntervalSeconds;
using mowgli_localization::StationaryLatchClock;
using mowgli_localization::StationaryLatchState;

constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kToleranceFactor = 1.5;

std::int64_t Seconds(const double seconds)
{
  return static_cast<std::int64_t>(std::llround(seconds * static_cast<double>(kSecond)));
}

CogObservationTiming TimingForRate(const double rate_hz)
{
  return CogObservationTiming(0.05,
                              DeriveMaximumCogObservationIntervalSeconds(rate_hz, kToleranceFactor),
                              2.0);
}

// Minimal moving-baseline harness around the production timing gate. A heading
// is counted only when a timing-accepted observation can pair with a retained,
// spatially distinct anchor. This directly exposes the old 1 Hz failure: a gap
// decision reseeds and therefore cannot publish from the previous position.
class MovingCogHarness
{
public:
  explicit MovingCogHarness(const double rate_hz) : timing_(TimingForRate(rate_hz))
  {
  }

  CogObservationTimingDecision Deliver(const std::int64_t receipt_ns,
                                       const std::int64_t ros_now_ns,
                                       const double position_m)
  {
    const auto result = timing_.Observe(receipt_ns, ros_now_ns);
    switch (result.decision)
    {
      case CogObservationTimingDecision::kSeed:
      case CogObservationTimingDecision::kGapReseed:
        anchor_m_ = position_m;
        ++physical_observations_;
        break;
      case CogObservationTimingDecision::kAdvance:
        ++physical_observations_;
        if (!anchor_m_)
        {
          anchor_m_ = position_m;
        }
        else if (std::abs(position_m - *anchor_m_) >= 0.10)
        {
          ++headings_;
          anchor_m_ = position_m;
        }
        break;
      case CogObservationTimingDecision::kInvalidProvenance:
      case CogObservationTimingDecision::kReceiptRewind:
      case CogObservationTimingDecision::kRosTimeDiscontinuity:
        anchor_m_.reset();
        break;
      case CogObservationTimingDecision::kDuplicate:
      case CogObservationTimingDecision::kTooSoon:
        break;
    }
    return result.decision;
  }

  int physical_observations() const
  {
    return physical_observations_;
  }

  int headings() const
  {
    return headings_;
  }

private:
  CogObservationTiming timing_;
  std::optional<double> anchor_m_;
  int physical_observations_{0};
  int headings_{0};
};

TEST(CogObservationTimingTest, GenuineOneHertzMovingObservationsProduceHeading)
{
  MovingCogHarness harness(1.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0), CogObservationTimingDecision::kSeed);
  EXPECT_EQ(harness.Deliver(Seconds(2.0), Seconds(2.0), 0.20),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, GenuineFiveHertzBehaviorRemainsValid)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.2), Seconds(1.2), 0.20),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, GenuineSevenHertzMovingObservationsProduceHeading)
{
  MovingCogHarness harness(7.0);
  const std::int64_t period = Seconds(1.0 / 7.0);
  harness.Deliver(kSecond, kSecond, 0.0);
  EXPECT_EQ(harness.Deliver(kSecond + period, kSecond + period, 0.20),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, GenuineTenHertzMovingObservationsProduceHeading)
{
  MovingCogHarness harness(10.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.1), Seconds(1.1), 0.20),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, CachedReceiptWithSameCoordinatesDoesNotAdvance)
{
  MovingCogHarness harness(1.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.0), Seconds(1.1), 0.0),
            CogObservationTimingDecision::kDuplicate);
  EXPECT_EQ(harness.physical_observations(), 1);
  EXPECT_EQ(harness.headings(), 0);
}

TEST(CogObservationTimingTest, TenHertzCachedPublicationAroundOneHertzReceiverIsIgnored)
{
  MovingCogHarness harness(1.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  for (int callback = 1; callback < 10; ++callback)
  {
    EXPECT_EQ(harness.Deliver(Seconds(1.0), Seconds(1.0 + 0.1 * callback), 0.0),
              CogObservationTimingDecision::kDuplicate);
  }
  harness.Deliver(Seconds(2.0), Seconds(2.0), 0.20);

  EXPECT_EQ(harness.physical_observations(), 2);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, SameCoordinatesWithNewReceiptAreGenuineObservation)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.2), Seconds(1.2), 0.0),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.physical_observations(), 2);
  EXPECT_EQ(harness.headings(), 0);
}

TEST(CogObservationTimingTest, IntervalInsideFiftyPercentJitterToleranceIsRetained)
{
  auto timing = TimingForRate(5.0);
  EXPECT_EQ(timing.Observe(Seconds(1.0), Seconds(1.0)).decision,
            CogObservationTimingDecision::kSeed);
  EXPECT_EQ(timing.Observe(Seconds(1.299), Seconds(1.299)).decision,
            CogObservationTimingDecision::kAdvance);
}

TEST(CogObservationTimingTest, ExistingMinimumIntervalStillRejectsTooFastNewStamp)
{
  auto timing = TimingForRate(10.0);
  EXPECT_EQ(timing.Observe(Seconds(1.0), Seconds(1.0)).decision,
            CogObservationTimingDecision::kSeed);
  EXPECT_EQ(timing.Observe(Seconds(1.049), Seconds(1.049)).decision,
            CogObservationTimingDecision::kTooSoon);
  EXPECT_EQ(timing.Observe(Seconds(1.1), Seconds(1.1)).decision,
            CogObservationTimingDecision::kAdvance);
}

TEST(CogObservationTimingTest, IntervalBeyondPhysicalGapLimitReseeds)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(1.301), Seconds(1.301), 0.20),
            CogObservationTimingDecision::kGapReseed);
  EXPECT_EQ(harness.headings(), 0);
  EXPECT_EQ(harness.Deliver(Seconds(1.501), Seconds(1.501), 0.40),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, DelayedDuplicateIsIgnoredWithoutBreakingBaseline)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  harness.Deliver(Seconds(1.2), Seconds(1.2), 0.20);
  EXPECT_EQ(harness.Deliver(Seconds(1.0), Seconds(1.25), 0.0),
            CogObservationTimingDecision::kDuplicate);
  EXPECT_EQ(harness.Deliver(Seconds(1.4), Seconds(1.4), 0.40),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 2);
}

TEST(CogObservationTimingTest, UnseenReceiptRewindFailsClosedAndNextSampleSeeds)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  harness.Deliver(Seconds(1.2), Seconds(1.2), 0.20);
  EXPECT_EQ(harness.Deliver(Seconds(1.1), Seconds(1.25), 0.10),
            CogObservationTimingDecision::kReceiptRewind);
  EXPECT_EQ(harness.Deliver(Seconds(1.3), Seconds(1.3), 0.30), CogObservationTimingDecision::kSeed);
  EXPECT_EQ(harness.Deliver(Seconds(1.5), Seconds(1.5), 0.50),
            CogObservationTimingDecision::kAdvance);
  EXPECT_EQ(harness.headings(), 2);
}

TEST(CogObservationTimingTest, ZeroFutureAndLargeForwardJumpCannotReuseOldBaseline)
{
  MovingCogHarness harness(5.0);
  EXPECT_EQ(harness.Deliver(0, Seconds(1.0), 0.0),
            CogObservationTimingDecision::kInvalidProvenance);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(5.0), Seconds(2.0), 0.20),
            CogObservationTimingDecision::kInvalidProvenance);
  EXPECT_EQ(harness.Deliver(Seconds(2.0), Seconds(2.0), 0.20), CogObservationTimingDecision::kSeed);
  EXPECT_EQ(harness.Deliver(Seconds(5.0), Seconds(5.0), 0.50),
            CogObservationTimingDecision::kGapReseed);
  EXPECT_EQ(harness.headings(), 0);
}

TEST(CogObservationTimingTest, RosClockRewindDropsTriggerAndStartsCleanEpoch)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(10.0), Seconds(10.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(9.0), Seconds(9.0), 0.20),
            CogObservationTimingDecision::kRosTimeDiscontinuity);
  EXPECT_EQ(harness.Deliver(Seconds(9.1), Seconds(9.1), 0.30), CogObservationTimingDecision::kSeed);
}

TEST(CogObservationTimingTest, ReceiverReconnectGapSeedsBeforeProducingAgain)
{
  MovingCogHarness harness(5.0);
  harness.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  EXPECT_EQ(harness.Deliver(Seconds(3.0), Seconds(3.0), 0.20),
            CogObservationTimingDecision::kGapReseed);
  EXPECT_EQ(harness.headings(), 0);
  harness.Deliver(Seconds(3.2), Seconds(3.2), 0.40);
  EXPECT_EQ(harness.headings(), 1);
}

TEST(CogObservationTimingTest, PublicationCadenceCannotChangePhysicalGapPolicy)
{
  // Fast callbacks carrying the same 1 Hz receipt do not advance.
  MovingCogHarness one_hertz(1.0);
  one_hertz.Deliver(Seconds(1.0), Seconds(1.0), 0.0);
  one_hertz.Deliver(Seconds(1.0), Seconds(1.1), 0.0);
  EXPECT_EQ(one_hertz.physical_observations(), 1);

  // Conversely, receiving only every tenth 10 Hz observation yields a real
  // one-second provenance gap. Callback frequency cannot widen the 0.15 s
  // physical policy, so the visible sample safely reseeds.
  auto ten_hertz = TimingForRate(10.0);
  ten_hertz.Observe(Seconds(1.0), Seconds(1.0));
  EXPECT_EQ(ten_hertz.Observe(Seconds(2.0), Seconds(2.0)).decision,
            CogObservationTimingDecision::kGapReseed);
}

TEST(CogStationaryLatchClockTest, NegativeRosAgeAndRewindAreNeverFresh)
{
  StationaryLatchClock latch(10.0);
  ASSERT_TRUE(latch.Latch(Seconds(100.0), Seconds(100.0), Seconds(50.0)));
  const auto rewound = latch.Check(Seconds(99.0), Seconds(51.0));
  EXPECT_EQ(rewound.state, StationaryLatchState::kClockDiscontinuity);
  EXPECT_FALSE(rewound.fresh());
  EXPECT_EQ(latch.Check(Seconds(101.0), Seconds(52.0)).state, StationaryLatchState::kNoLatch);
}

TEST(CogStationaryLatchClockTest, MonotonicAgeExpiresEvenWhenRosTimePauses)
{
  StationaryLatchClock latch(10.0);
  ASSERT_TRUE(latch.Latch(Seconds(100.0), Seconds(100.0), Seconds(50.0)));
  const auto expired = latch.Check(Seconds(100.0), Seconds(61.0));
  EXPECT_EQ(expired.state, StationaryLatchState::kExpired);
  EXPECT_FALSE(expired.fresh());
}

TEST(CogObservationTimingTest, InvalidConfigurationFailsVisibly)
{
  EXPECT_THROW(DeriveMaximumCogObservationIntervalSeconds(0.0, 1.5), std::invalid_argument);
  EXPECT_THROW(DeriveMaximumCogObservationIntervalSeconds(5.0, 0.9), std::invalid_argument);
  EXPECT_THROW(CogObservationTiming(0.05, 0.05, 2.0), std::invalid_argument);
}

}  // namespace
