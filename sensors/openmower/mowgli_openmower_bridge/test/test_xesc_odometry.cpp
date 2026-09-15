// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_xesc_odometry.cpp
 * @brief Windowed wheel odometry from signed tick counters.
 */

#include "mowgli_openmower_bridge/xesc_odometry.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::WheelSpeedFilter;
using mowgli_openmower_bridge::XescOdometry;

TEST(XescOdometry, FirstCallPrimesOnly)
{
  XescOdometry odom(1600.0, 0.325);
  EXPECT_FALSE(odom.Update(1000, 2000, 0.02).has_value());
  EXPECT_TRUE(odom.stationary());
}

TEST(XescOdometry, AggregatesToTheWindowThenReportsStraightVelocity)
{
  XescOdometry odom(1600.0, 0.325, 0.05);
  (void)odom.Update(0, 0, 0.02);
  // 32 ticks per 20 ms on both wheels = 0.02 m per tick step → 1.0 m/s
  EXPECT_FALSE(odom.Update(32, 32, 0.02).has_value());
  EXPECT_FALSE(odom.Update(64, 64, 0.02).has_value());
  const auto s = odom.Update(96, 96, 0.02);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->dt_s, 0.06, 1e-9);
  EXPECT_NEAR(s->vx_mps, 1.0, 1e-9);
  EXPECT_NEAR(s->vyaw_radps, 0.0, 1e-9);
  EXPECT_FALSE(odom.stationary());
  EXPECT_NEAR(odom.tyre_travelled(), 0.06, 1e-9);
}

TEST(XescOdometry, PivotYieldsYawRateAndWorstWheelOdometer)
{
  XescOdometry odom(1000.0, 0.5, 0.05);
  (void)odom.Update(0, 0, 0.05);
  const auto s = odom.Update(-25, 25, 0.05);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->vx_mps, 0.0, 1e-9);
  // (0.025 - (-0.025)) / 0.5 / 0.05 = 2 rad/s
  EXPECT_NEAR(s->vyaw_radps, 2.0, 1e-9);
  EXPECT_NEAR(odom.tyre_travelled(), 0.025, 1e-9);
}

TEST(XescOdometry, ResetRePrimes)
{
  XescOdometry odom(1000.0, 0.5, 0.05);
  (void)odom.Update(0, 0, 0.05);
  odom.Reset();
  EXPECT_FALSE(odom.Update(5000, 5000, 0.05).has_value());
  const auto s = odom.Update(5000, 5000, 0.05);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->vx_mps, 0.0, 1e-9);
  EXPECT_TRUE(odom.stationary());
}

TEST(WheelSpeedFilter, LowPassesTowardsRawSpeed)
{
  WheelSpeedFilter f(0.5);
  EXPECT_NEAR(f.Update(20, 1000.0, 0.02), 0.5, 1e-9);  // raw 1.0 m/s, half way
  EXPECT_NEAR(f.Update(20, 1000.0, 0.02), 0.75, 1e-9);
  EXPECT_NEAR(f.Update(0, 1000.0, 0.0), 0.75, 1e-9);  // zero dt keeps value
  f.Reset();
  EXPECT_DOUBLE_EQ(f.value(), 0.0);
}
