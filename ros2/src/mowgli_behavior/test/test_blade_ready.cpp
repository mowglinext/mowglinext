// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <limits>

#include "mowgli_behavior/blade_ready.hpp"
#include <gtest/gtest.h>

using namespace mowgli_behavior;

class BladeReadyTest : public ::testing::Test
{
protected:
  BladeReadyConfig config;
  BladeReadyGate gate;
  BladeReadyGate::Clock::time_point start{std::chrono::seconds(10)};

  void begin(bool telemetry_seen = true, int64_t previous_stamp = 9000000000)
  {
    gate.start(start, 10000000000, previous_stamp, telemetry_seen);
  }

  BladeReadyResult step(double elapsed,
                        int64_t stamp,
                        double rpm = 3000.0,
                        bool requested = true,
                        bool active = true,
                        bool delivered = true,
                        bool accepted = true,
                        int64_t ros_now = 0)
  {
    const auto offset = std::chrono::duration_cast<BladeReadyGate::Clock::duration>(
        std::chrono::duration<double>(elapsed));
    return gate.step(config,
                     start + offset,
                     ros_now ? ros_now : 10000000000 + static_cast<int64_t>(elapsed * 1e9),
                     stamp,
                     delivered,
                     requested,
                     active,
                     rpm,
                     accepted);
  }
};

TEST_F(BladeReadyTest, RequiresDistinctPostRequestSamplesAcrossTheStabilityWindow)
{
  begin();
  EXPECT_EQ(step(0.1, 9900000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(0.2, 10200000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(0.6, 10200000000), BladeReadyResult::kWaiting);  // cached republication
  EXPECT_EQ(step(0.7, 10700000000), BladeReadyResult::kTelemetryReady);
}

TEST_F(BladeReadyTest, MissingTelemetryUsesTimerOnlyAfterAcceptedOn)
{
  begin(false, 0);
  EXPECT_EQ(step(0.0, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(1.4, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(1.6, 0), BladeReadyResult::kTimerFallback);
}

TEST_F(BladeReadyTest, DelayedAcknowledgementDoesNotConsumeTheFallbackDelay)
{
  begin(false, 0);
  EXPECT_EQ(step(1.5, 0, 0.0, false, false, false, false), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.0, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(3.4, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(3.6, 0), BladeReadyResult::kTimerFallback);
}

TEST_F(BladeReadyTest, KnownTelemetryNeverFallsBackEvenIfItDisappears)
{
  begin();
  EXPECT_EQ(step(2.0, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(6.0, 0), BladeReadyResult::kFailed);
}

TEST_F(BladeReadyTest, TelemetryArrivingDuringTheFallbackWindowDisablesFallback)
{
  begin(false, 0);
  EXPECT_EQ(step(1.0, 11000000000, 0.0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.0, 12000000000, 0.0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(6.0, 16000000000, 0.0), BladeReadyResult::kFailed);
}

TEST_F(BladeReadyTest, InactiveLowInvalidAndStaleSamplesResetStability)
{
  for (int bad = 0; bad < 7; ++bad)
  {
    begin();
    EXPECT_EQ(step(0.1, 10100000000), BladeReadyResult::kWaiting);
    EXPECT_EQ(step(0.5,
                   bad == 5 ? 9000000000 : 10500000000,
                   bad == 0   ? 999.0
                   : bad == 1 ? std::numeric_limits<double>::quiet_NaN()
                   : bad == 2 ? std::numeric_limits<double>::infinity()
                              : 3000.0,
                   bad != 3,
                   bad != 4,
                   bad != 6),
              BladeReadyResult::kWaiting);
    EXPECT_EQ(step(0.6, 10600000000), BladeReadyResult::kWaiting);
    EXPECT_EQ(step(1.0, 11000000000), BladeReadyResult::kTelemetryReady);
  }
}

TEST_F(BladeReadyTest, FrozenStampAndClockJumpsCannotReleaseTheGate)
{
  begin();
  EXPECT_EQ(step(0.1, 10100000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(0.5, 10100000000, 3000.0, true, true, true, true, 9000000000),
            BladeReadyResult::kWaiting);  // source stamp now in the future
  EXPECT_EQ(step(0.9, 10200000000, 3000.0, true, true, true, true, 20000000000),
            BladeReadyResult::kWaiting);  // jump forwards makes it stale
  EXPECT_EQ(step(6.0, 10600000000, 3000.0, true, true, true, true, 1000000000),
            BladeReadyResult::kFailed);  // steady deadline survives a backwards ROS clock
}

TEST_F(BladeReadyTest, NewEnableDiscardsEarlierReadiness)
{
  begin();
  EXPECT_EQ(step(0.1, 10100000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(0.5, 10500000000), BladeReadyResult::kTelemetryReady);
  gate.start(start + std::chrono::seconds(1), 11000000000, 10500000000, true);
  EXPECT_EQ(step(1.5, 10500000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(1.6, 11600000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.0, 12000000000), BladeReadyResult::kTelemetryReady);
}

TEST_F(BladeReadyTest, NewHandoffCannotReuseFrozenTelemetryWithPausedRosClock)
{
  begin();
  EXPECT_EQ(step(0.1, 10100000000), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(0.5, 10500000000), BladeReadyResult::kTelemetryReady);
  gate.start(start + std::chrono::seconds(1), 10500000000, 10500000000, true);
  for (double elapsed : {1.1, 1.5, 3.0, 6.9})
    EXPECT_EQ(step(elapsed, 10500000000, 3000.0, true, true, true, true, 10500000000),
              BladeReadyResult::kWaiting);
  EXPECT_EQ(step(7.0, 10500000000, 3000.0, true, true, true, true, 10500000000),
            BladeReadyResult::kFailed);
}

TEST_F(BladeReadyTest, StickyProvenancePreventsFallbackAfterAStatusReset)
{
  begin(false, 0);
  EXPECT_EQ(step(0.0, 0), BladeReadyResult::kWaiting);
  gate.noteTelemetrySeen(true);  // report observed by callback, then reset before next tick
  EXPECT_EQ(step(2.0, 0), BladeReadyResult::kWaiting);
  EXPECT_EQ(step(6.0, 0), BladeReadyResult::kFailed);
}

TEST_F(BladeReadyTest, UniqueObservationSilenceResetsStabilityWhenRosClockPauses)
{
  begin();
  EXPECT_EQ(step(0.1, 10100000000, 3000.0, true, true, true, true, 10100000000),
            BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.0, 10100000000, 3000.0, true, true, true, true, 10100000000),
            BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.1, 10200000000, 3000.0, true, true, true, true, 10200000000),
            BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.3, 10300000000, 3000.0, true, true, true, true, 10300000000),
            BladeReadyResult::kWaiting);
  EXPECT_EQ(step(2.5, 10400000000, 3000.0, true, true, true, true, 10400000000),
            BladeReadyResult::kTelemetryReady);
}

TEST(BladeReadyConfig, RejectsInvalidAndUnboundedValues)
{
  BladeReadyConfig config;
  EXPECT_NO_THROW(config.validate());
  for (double invalid : {0.0,
                         -1.0,
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()})
  {
    config = {};
    config.min_rpm = invalid;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config = {};
    config.fallback_sec = invalid;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config = {};
    config.timeout_sec = invalid;
    EXPECT_THROW(config.validate(), std::invalid_argument);
  }
  config = {};
  config.timeout_sec = config.fallback_sec;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = {};
  config.timeout_sec = 31.0;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}
