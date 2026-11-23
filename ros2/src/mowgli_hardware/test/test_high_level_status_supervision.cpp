// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_hardware/high_level_status_supervision.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::high_level_mode_is_supervised;
using mowgli_hardware::high_level_status_lost;
using mowgli_hardware::kDefaultHighLevelStatusTimeoutS;

namespace
{
constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kReceipt = 100 * kSecond;
constexpr uint8_t kNull = 0u;
constexpr uint8_t kIdle = 1u;
constexpr uint8_t kAutonomous = 2u;
constexpr uint8_t kRecording = 3u;
constexpr uint8_t kManualMowing = 4u;
}  // namespace

TEST(HighLevelStatusSupervision, OnlyTheFirmwareStoppedModesAreUnsupervised)
{
  EXPECT_FALSE(high_level_mode_is_supervised(kNull));
  EXPECT_FALSE(high_level_mode_is_supervised(kIdle));
  EXPECT_TRUE(high_level_mode_is_supervised(kAutonomous));
  EXPECT_TRUE(high_level_mode_is_supervised(kRecording));
  EXPECT_TRUE(high_level_mode_is_supervised(kManualMowing));
  EXPECT_TRUE(high_level_mode_is_supervised(200u));  // unknown mode: assume it actuates
}

TEST(HighLevelStatusSupervision, SilenceBeyondTheTimeoutIsLostInEveryActuatingMode)
{
  for (const uint8_t mode : {kAutonomous, kRecording, kManualMowing})
  {
    EXPECT_FALSE(high_level_status_lost(true, mode, kReceipt + 5 * kSecond, kReceipt, 5.0))
        << +mode;
    EXPECT_TRUE(high_level_status_lost(true, mode, kReceipt + 5 * kSecond + 1, kReceipt, 5.0))
        << +mode;
  }
}

TEST(HighLevelStatusSupervision, IdleNullAndNeverReceivedAreNeverLost)
{
  const std::int64_t long_after = kReceipt + 3600 * kSecond;
  EXPECT_FALSE(high_level_status_lost(true, kIdle, long_after, kReceipt, 5.0));
  EXPECT_FALSE(high_level_status_lost(true, kNull, long_after, kReceipt, 5.0));
  EXPECT_FALSE(high_level_status_lost(false, kAutonomous, long_after, 0, 5.0));
}

TEST(HighLevelStatusSupervision, ZeroOrNegativeTimeoutDisablesIt)
{
  const std::int64_t long_after = kReceipt + 3600 * kSecond;
  EXPECT_FALSE(high_level_status_lost(true, kAutonomous, long_after, kReceipt, 0.0));
  EXPECT_FALSE(high_level_status_lost(true, kAutonomous, long_after, kReceipt, -1.0));
}

TEST(HighLevelStatusSupervision, AReceiptNotOlderThanNowIsFresh)
{
  EXPECT_FALSE(high_level_status_lost(true, kAutonomous, kReceipt, kReceipt, 5.0));
  EXPECT_FALSE(high_level_status_lost(true, kAutonomous, kReceipt - kSecond, kReceipt, 5.0));
}

TEST(HighLevelStatusSupervision, DefaultBoundIsSeveralMissedOneHertzRepublishes)
{
  EXPECT_GE(kDefaultHighLevelStatusTimeoutS, 3.0);
  EXPECT_LE(kDefaultHighLevelStatusTimeoutS, 10.0);
}
