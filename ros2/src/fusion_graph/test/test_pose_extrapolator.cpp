// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for PoseExtrapolator. Pure logic, no ROS plumbing.

#include <cmath>

#include "fusion_graph/pose_extrapolator.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

TEST(PoseExtrapolator, ReturnsNulloptBeforeFirstFusion)
{
  fg::PoseExtrapolator ex;
  EXPECT_FALSE(ex.Extrapolate(1.0).has_value());
}

TEST(PoseExtrapolator, ReturnsBaselineWhenNoGyro)
{
  fg::PoseExtrapolator ex;
  const gtsam::Pose2 baseline(1.0, 2.0, 0.5);
  ex.OnFusionPose(10.0, baseline);

  auto out = ex.Extrapolate(10.020);
  ASSERT_TRUE(out.has_value());
  // No IMU sample yet → no extrapolation, baseline returned as-is.
  EXPECT_DOUBLE_EQ(out->x(), baseline.x());
  EXPECT_DOUBLE_EQ(out->y(), baseline.y());
  EXPECT_DOUBLE_EQ(out->theta(), baseline.theta());
}

TEST(PoseExtrapolator, ExtrapolatesYawAtConstantRate)
{
  fg::PoseExtrapolator ex;
  const gtsam::Pose2 baseline(0.0, 0.0, 0.0);
  ex.OnFusionPose(10.0, baseline);
  // Pivot at 0.5 rad/s.
  ex.OnImuGyro(10.005, 0.5);

  // 10 ms in the future of fusion → expected yaw = 0 + 0.5 × 0.010
  auto out = ex.Extrapolate(10.010);
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->x(), 0.0);
  EXPECT_DOUBLE_EQ(out->y(), 0.0);
  EXPECT_NEAR(out->theta(), 0.005, 1.0e-9);

  // 20 ms forward → 0.010 rad.
  out = ex.Extrapolate(10.020);
  ASSERT_TRUE(out.has_value());
  EXPECT_NEAR(out->theta(), 0.010, 1.0e-9);
}

TEST(PoseExtrapolator, NegativeDtReturnsBaseline)
{
  fg::PoseExtrapolator ex;
  ex.OnFusionPose(10.0, gtsam::Pose2(0.0, 0.0, 0.3));
  ex.OnImuGyro(9.9, 1.0);
  // Query BEFORE the fusion stamp — no time-travel, return baseline.
  auto out = ex.Extrapolate(9.95);
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->theta(), 0.3);
}

TEST(PoseExtrapolator, CapsExtrapolationAt200ms)
{
  fg::PoseExtrapolator ex;
  ex.OnFusionPose(10.0, gtsam::Pose2(0.0, 0.0, 0.0));
  ex.OnImuGyro(10.0, 1.0);
  // 300 ms forward — beyond the 200 ms cap. Caller stalled; we
  // return the baseline rather than a wildly extrapolated value.
  auto out = ex.Extrapolate(10.300);
  ASSERT_TRUE(out.has_value());
  EXPECT_DOUBLE_EQ(out->theta(), 0.0);
}

TEST(PoseExtrapolator, RebaselinesOnFreshFusionPose)
{
  fg::PoseExtrapolator ex;
  // First baseline, integrate yaw forward.
  ex.OnFusionPose(10.0, gtsam::Pose2(0.0, 0.0, 0.0));
  ex.OnImuGyro(10.0, 1.0);
  auto first = ex.Extrapolate(10.050);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->theta(), 0.050, 1.0e-9);

  // New fusion pose arrives. The integration window resets.
  ex.OnFusionPose(10.040, gtsam::Pose2(0.10, 0.0, 0.040));
  auto second = ex.Extrapolate(10.060);
  ASSERT_TRUE(second.has_value());
  // dt = 20 ms × 1 rad/s = 0.020 rad on top of the new baseline 0.040.
  EXPECT_NEAR(second->theta(), 0.060, 1.0e-9);
  EXPECT_NEAR(second->x(), 0.10, 1.0e-9);
}

TEST(PoseExtrapolator, NegativeGyroDoesYawNegative)
{
  fg::PoseExtrapolator ex;
  ex.OnFusionPose(10.0, gtsam::Pose2(0.0, 0.0, 1.0));
  ex.OnImuGyro(10.0, -2.0);
  auto out = ex.Extrapolate(10.010);
  ASSERT_TRUE(out.has_value());
  // 10 ms × -2 rad/s = -0.020 added to baseline 1.0
  EXPECT_NEAR(out->theta(), 0.980, 1.0e-9);
}
