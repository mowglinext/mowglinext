// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_nav2_plugins/ftc_obstacle_wait.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

TEST(ObstacleWait, RequiresContinuousFollowableWindow)
{
  double clear_s = 0.0;
  for (int i = 0; i < 14; ++i)
  {
    EXPECT_FALSE(mnp::ObstacleWaitReadyToResume(true, 0.1, 1.5, clear_s));
  }
  EXPECT_TRUE(mnp::ObstacleWaitReadyToResume(true, 0.1, 1.5, clear_s));
}

TEST(ObstacleWait, OneBlockedScanResetsEvidence)
{
  double clear_s = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    mnp::ObstacleWaitReadyToResume(true, 0.1, 1.5, clear_s);
  }
  ASSERT_NEAR(clear_s, 1.0, 1e-9);

  EXPECT_FALSE(mnp::ObstacleWaitReadyToResume(false, 0.1, 1.5, clear_s));
  EXPECT_DOUBLE_EQ(clear_s, 0.0);
  EXPECT_FALSE(mnp::ObstacleWaitReadyToResume(true, 0.1, 1.5, clear_s));
}

TEST(ObstacleWait, AlternatingScansNeverProduceStopGoLoop)
{
  double clear_s = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    const bool followable = i % 2 == 0;
    EXPECT_FALSE(mnp::ObstacleWaitReadyToResume(followable, 0.1, 1.5, clear_s));
  }
}
