// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <limits>

#include "mowgli_localization/cog_motion_history.hpp"
#include <gtest/gtest.h>

namespace mowgli_localization
{
namespace
{
constexpr int64_t S = 1000000000;

TEST(CogMotionHistory, DelayedFixUsesItsReverseMotionDespiteNewForwardInput)
{
  CogMotionHistory history;
  history.ObserveClock(3 * S);
  ASSERT_TRUE(history.AddWheel(S, -.3, 0));
  ASSERT_TRUE(history.AddWheel(3 * S, .3, .2));
  ASSERT_TRUE(history.AddGyro(S, 0));
  ASSERT_TRUE(history.AddGyro(3 * S, .4));
  const auto old = history.At(2 * S);
  ASSERT_TRUE(old);
  EXPECT_DOUBLE_EQ(old->vx, -.3);
  EXPECT_DOUBLE_EQ(old->wheel_omega, 0);
  EXPECT_DOUBLE_EQ(old->gyro_z, 0);
  EXPECT_TRUE(history.Between(S, 2 * S, .05)->direction_consistent);
}

TEST(CogMotionHistory, EveryDirectionAndRestTransitionInvalidatesBaseline)
{
  for (const double transition : {-.3, 0.0, .02})
  {
    CogMotionHistory history;
    history.ObserveClock(4 * S);
    ASSERT_TRUE(history.AddWheel(S, .3, 0));
    ASSERT_TRUE(history.AddWheel(2 * S, transition, 0));
    ASSERT_TRUE(history.AddWheel(3 * S, .3, 0));
    EXPECT_FALSE(history.Between(S, 4 * S, .05)->direction_consistent);
    EXPECT_TRUE(history.Between(3 * S, 4 * S, .05)->direction_consistent);
  }
}

TEST(CogMotionHistory, BaselineRotationUsesSourceIntervalsNotCallbackOrder)
{
  CogMotionHistory history;
  history.ObserveClock(5 * S);
  ASSERT_TRUE(history.AddWheel(S, .3, 0));
  ASSERT_TRUE(history.AddGyro(S, .1));
  ASSERT_TRUE(history.AddGyro(2 * S, -.2));
  ASSERT_TRUE(history.AddGyro(3 * S, 0));
  ASSERT_TRUE(history.AddGyro(5 * S, .8));
  EXPECT_NEAR(history.Between(S, 4 * S, .05)->absolute_rotation, .3, 1e-15);
  EXPECT_DOUBLE_EQ(history.Between(3 * S, 4 * S, .05)->absolute_rotation, 0);
}

TEST(CogMotionHistory, LatchRotationIncludesCompletedPivotAndRetainsNoiseDeadband)
{
  CogMotionHistory history;
  history.ObserveClock(4 * S);
  ASSERT_TRUE(history.AddWheel(S, .3, 0));
  ASSERT_TRUE(history.AddWheel(2 * S, 0, 0));
  ASSERT_TRUE(history.AddGyro(S, .02));
  ASSERT_TRUE(history.AddGyro(2 * S, .7));
  ASSERT_TRUE(history.AddGyro(2500000000, 0));
  EXPECT_NEAR(*history.AbsoluteRotation(S, 4 * S, .05), .35, 1e-15);
  EXPECT_NEAR(*history.AbsoluteRotation(S, 4 * S), .37, 1e-15);
  EXPECT_FALSE(history.AbsoluteRotation(S - 1, 4 * S, .05));
}

TEST(CogMotionHistory, InvalidFutureDuplicateAndReorderedMotionCannotReplaceAcceptedInput)
{
  CogMotionHistory history;
  history.ObserveClock(3 * S);
  ASSERT_TRUE(history.AddWheel(S, .3, .1));
  ASSERT_TRUE(history.AddGyro(S, .1));
  EXPECT_FALSE(history.AddWheel(S, -.3, 0));
  EXPECT_FALSE(history.AddWheel(S - 1, -.3, 0));
  EXPECT_FALSE(history.AddWheel(-1, -.3, 0));
  EXPECT_FALSE(history.AddWheel(4 * S, -.3, 0));
  EXPECT_FALSE(history.AddWheel(2 * S, std::numeric_limits<double>::quiet_NaN(), 0));
  EXPECT_FALSE(history.AddGyro(S, -.1));
  EXPECT_FALSE(history.AddGyro(S - 1, -.1));
  EXPECT_FALSE(history.AddGyro(4 * S, -.1));
  EXPECT_FALSE(history.AddGyro(2 * S, std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(history.At(2 * S)->vx, .3);
  EXPECT_DOUBLE_EQ(history.At(2 * S)->gyro_z, .1);
}

TEST(CogMotionHistory, RetiredAndUncoveredHistoryNeverUsesNewerInput)
{
  CogMotionHistory history(2);
  history.ObserveClock(4 * S);
  ASSERT_TRUE(history.AddWheel(S, -.3, 0));
  ASSERT_TRUE(history.AddWheel(2 * S, .3, 0));
  ASSERT_TRUE(history.AddWheel(3 * S, .3, 0));
  EXPECT_FALSE(history.At(S));
  EXPECT_FALSE(history.Between(S, 3 * S, .05));
  ASSERT_TRUE(history.AddGyro(3 * S, .1));
  EXPECT_FALSE(history.At(2 * S));
  EXPECT_TRUE(history.At(3 * S));
}

TEST(CogMotionHistory, RosClockRewindRequiresNewMotionCoverage)
{
  CogMotionHistory history;
  history.ObserveClock(3 * S);
  ASSERT_TRUE(history.AddWheel(2 * S, .3, 0));
  ASSERT_TRUE(history.AddGyro(2 * S, .1));
  ASSERT_TRUE(history.ObserveClock(S));
  EXPECT_FALSE(history.At(S));
  ASSERT_TRUE(history.AddWheel(S, -.3, 0));
  EXPECT_DOUBLE_EQ(history.At(S)->vx, -.3);
  EXPECT_DOUBLE_EQ(history.At(S)->gyro_z, 0);
  EXPECT_FALSE(history.ObserveClock(S + 1));
}
}  // namespace
}  // namespace mowgli_localization
