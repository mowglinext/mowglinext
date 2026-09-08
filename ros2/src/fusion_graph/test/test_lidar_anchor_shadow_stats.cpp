// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include "fusion_graph/lidar_anchor_shadow_stats.hpp"
#include <gtest/gtest.h>

using fusion_graph::LidarAnchorShadowStats;

TEST(LidarAnchorShadowStats, FloorStaysAtMinimumUntilEnoughSamples)
{
  LidarAnchorShadowStats s(300, 50);
  for (int i = 0; i < 49; ++i)
    s.Push(0.30);
  EXPECT_FALSE(s.ready());
  EXPECT_NEAR(s.EffectiveFloor(0.9, 0.05, 0.5), 0.05, 1e-12);
  s.Push(0.30);
  EXPECT_TRUE(s.ready());
  EXPECT_NEAR(s.EffectiveFloor(0.9, 0.05, 0.5), 0.30, 1e-12);
}

// The field shape: most estimates near 0.08, a 15 % tail at 0.15 → the p90
// floor lands at 0.15, not at the fixed 0.05.
TEST(LidarAnchorShadowStats, P90OfAFieldLikeDistributionIsTheFloor)
{
  LidarAnchorShadowStats s(300, 50);
  for (int i = 0; i < 100; ++i)
    s.Push(i < 85 ? 0.08 : 0.15);
  EXPECT_NEAR(s.Quantile(0.5), 0.08, 1e-12);
  EXPECT_NEAR(s.EffectiveFloor(0.9, 0.05, 0.5), 0.15, 0.011);
}

TEST(LidarAnchorShadowStats, ClampsToTheConfiguredBand)
{
  LidarAnchorShadowStats s(300, 50);
  for (int i = 0; i < 60; ++i)
    s.Push(0.01);
  EXPECT_NEAR(s.EffectiveFloor(0.9, 0.05, 0.5), 0.05, 1e-12);  // never below the minimum
  LidarAnchorShadowStats t(300, 50);
  for (int i = 0; i < 60; ++i)
    t.Push(3.0);
  EXPECT_NEAR(t.EffectiveFloor(0.9, 0.05, 0.5), 0.5, 1e-12);  // never above the cap
}

TEST(LidarAnchorShadowStats, WindowForgetsOldSamples)
{
  LidarAnchorShadowStats s(100, 10);
  for (int i = 0; i < 100; ++i)
    s.Push(0.50);
  for (int i = 0; i < 100; ++i)
    s.Push(0.05);
  EXPECT_EQ(s.size(), 100u);
  EXPECT_EQ(s.total(), 200u);
  EXPECT_NEAR(s.Quantile(0.9), 0.05, 1e-12);
}

TEST(LidarAnchorShadowStats, IgnoresNaNAndNegatives)
{
  LidarAnchorShadowStats s(10, 1);
  s.Push(-1.0);
  s.Push(std::nan(""));
  EXPECT_EQ(s.size(), 0u);
  s.Push(0.2);
  EXPECT_NEAR(s.Quantile(0.9), 0.2, 1e-12);
}
