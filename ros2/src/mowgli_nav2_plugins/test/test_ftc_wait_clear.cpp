// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_nav2_plugins/ftc_wait_clear.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

TEST(FtcWaitClear, OneClearTickCannotReleaseObstacleWait)
{
  double clear_time = 0.0;
  EXPECT_TRUE(mnp::ObstacleWaitMustContinue(true, true, 0.1, 1.5, clear_time));
  EXPECT_NEAR(clear_time, 0.1, 1e-12);
}

TEST(FtcWaitClear, BlockedTickResetsPartialClearEvidence)
{
  double clear_time = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(mnp::ObstacleWaitMustContinue(true, true, 0.1, 1.5, clear_time));
  }
  EXPECT_TRUE(mnp::ObstacleWaitMustContinue(true, false, 0.1, 1.5, clear_time));
  EXPECT_EQ(clear_time, 0.0);
  EXPECT_TRUE(mnp::ObstacleWaitMustContinue(true, true, 0.1, 1.5, clear_time));
}

TEST(FtcWaitClear, ContinuousFollowabilityReleasesAfterHold)
{
  double clear_time = 0.0;
  for (int i = 0; i < 14; ++i)
  {
    EXPECT_TRUE(mnp::ObstacleWaitMustContinue(true, true, 0.1, 1.5, clear_time));
  }
  EXPECT_FALSE(mnp::ObstacleWaitMustContinue(true, true, 0.1, 1.5, clear_time));
  EXPECT_EQ(clear_time, 0.0);
}

TEST(FtcWaitClear, LeavingWaitResetsClearEvidence)
{
  double clear_time = 0.7;
  EXPECT_FALSE(mnp::ObstacleWaitMustContinue(false, true, 0.1, 1.5, clear_time));
  EXPECT_EQ(clear_time, 0.0);
}
