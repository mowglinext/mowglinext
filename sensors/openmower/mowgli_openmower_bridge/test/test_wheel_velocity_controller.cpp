// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_wheel_velocity_controller.cpp
 * @brief The host-side wheel PI + feed-forward loop and the twist split.
 */

#include <cmath>

#include "mowgli_openmower_bridge/wheel_velocity_controller.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::SplitTwist;
using mowgli_openmower_bridge::WheelLoopGains;
using mowgli_openmower_bridge::WheelVelocityController;

TEST(WheelVelocityController, ZeroTargetIsAnExactStopAndResetsIntegral)
{
  WheelVelocityController c;
  (void)c.Update(0.3, 0.0, 0.02);
  (void)c.Update(0.3, 0.0, 0.02);
  EXPECT_GT(c.integral(), 0.0);
  EXPECT_DOUBLE_EQ(c.Update(0.0, 0.2, 0.02), 0.0);
  EXPECT_DOUBLE_EQ(c.integral(), 0.0);
}

TEST(WheelVelocityController, OpenLoopMatchesOpenMowerMapping)
{
  WheelLoopGains g;
  g.closed_loop = false;
  g.duty_per_mps = 1.0;
  WheelVelocityController c(g);
  EXPECT_DOUBLE_EQ(c.Update(0.4, 0.0, 0.02), 0.4);
  EXPECT_DOUBLE_EQ(c.Update(-0.25, 1.0, 0.02), -0.25);
}

TEST(WheelVelocityController, ProportionalTermPushesTowardTarget)
{
  WheelLoopGains g;
  g.kp = 0.5;
  g.ki = 0.0;
  WheelVelocityController c(g);
  // Measured slower than target → more duty than feed-forward alone.
  EXPECT_GT(c.Update(0.4, 0.2, 0.02), 0.4);
  // Measured faster → less.
  EXPECT_LT(c.Update(0.4, 0.6, 0.02), 0.4);
}

TEST(WheelVelocityController, IntegralIsClampedAndOutputBounded)
{
  WheelLoopGains g;
  g.kp = 0.0;
  g.ki = 100.0;
  g.integral_limit = 0.2;
  g.max_duty = 1.0;
  WheelVelocityController c(g);
  double duty = 0.0;
  for (int i = 0; i < 200; ++i)
  {
    duty = c.Update(0.5, 0.0, 0.02);  // stalled wheel
  }
  EXPECT_DOUBLE_EQ(c.integral(), 0.2);
  EXPECT_DOUBLE_EQ(duty, 0.7);  // 0.5 ff + 0.2 integral
  g.max_duty = 0.6;
  c.set_gains(g);
  EXPECT_DOUBLE_EQ(c.Update(0.5, 0.0, 0.02), 0.6);
}

TEST(WheelVelocityController, NonFiniteMeasurementFallsBackToFeedForward)
{
  WheelVelocityController c;
  EXPECT_DOUBLE_EQ(c.Update(0.3, std::nan(""), 0.02), 0.3);
  EXPECT_DOUBLE_EQ(c.Update(std::nan(""), 0.1, 0.02), 0.0);
}

TEST(SplitTwist, DifferentialDriveKinematics)
{
  const auto straight = SplitTwist(0.4, 0.0, 0.325);
  EXPECT_DOUBLE_EQ(straight.left_mps, 0.4);
  EXPECT_DOUBLE_EQ(straight.right_mps, 0.4);
  const auto pivot = SplitTwist(0.0, 1.0, 0.325);
  EXPECT_DOUBLE_EQ(pivot.left_mps, -0.1625);
  EXPECT_DOUBLE_EQ(pivot.right_mps, 0.1625);
}
