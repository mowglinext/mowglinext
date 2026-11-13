// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Dead-reckoning stationary hold (dr_stationary.hpp): the per-sample DR yaw
// integration must follow the graph's stationary snap, not drift on the
// residual gyro bias of a parked robot.

#include "fusion_graph/dr_stationary.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
fg::DrStationaryThresholds Shipped()
{
  fg::GraphParams gp;
  gp.stationary_thresh_xy_m = 1.0e-3;
  gp.stationary_thresh_theta = 2.0e-3;
  gp.stationary_gyro_thresh_rad_per_s = 0.10;
  return fg::DrStationaryFromGraphParams(gp);
}
}  // namespace

TEST(DrStationary, RatesMatchTheGraphGateAtTheTunedCadence)
{
  const auto t = Shipped();
  EXPECT_NEAR(t.wheel_speed_mps, 0.025, 1e-12);
  EXPECT_NEAR(t.wheel_yaw_rate_rad_per_s, 0.05, 1e-12);
  EXPECT_DOUBLE_EQ(t.gyro_rate_rad_per_s, 0.10);
}

TEST(DrStationary, ParkedRobotWithResidualGyroBiasHoldsYaw)
{
  // Field residual after the dock calibration: ≈ -0.023 rad/s.
  EXPECT_TRUE(fg::DrHoldYaw(0.0, 0.0, -0.023, Shipped()));
}

TEST(DrStationary, HandRotationAboveTheGyroThresholdStillIntegrates)
{
  EXPECT_FALSE(fg::DrHoldYaw(0.0, 0.0, 0.3, Shipped()));
}

TEST(DrStationary, AnyWheelMotionIntegrates)
{
  EXPECT_FALSE(fg::DrHoldYaw(0.10, 0.0, 0.0, Shipped()));  // driving straight
  EXPECT_FALSE(fg::DrHoldYaw(0.0, 0.20, 0.01, Shipped()));  // slow pivot, gyro lagging
}
