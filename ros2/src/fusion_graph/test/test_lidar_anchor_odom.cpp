// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>

#include "fusion_graph/lidar_anchor_odom.hpp"
#include <gtest/gtest.h>

using fusion_graph::ContinuousOdom;

TEST(ContinuousOdom, FollowsSmallStepsExactly)
{
  ContinuousOdom o;
  o.Advance(Sophus::SE2d(0.0, Eigen::Vector2d(1.0, 2.0)));
  const auto& p = o.Advance(Sophus::SE2d(0.1, Eigen::Vector2d(1.2, 2.1)));
  EXPECT_NEAR(p.translation().x(), 0.2, 1e-9);
  EXPECT_NEAR(p.translation().y(), 0.1, 1e-9);
  EXPECT_NEAR(p.so2().log(), 0.1, 1e-9);
  EXPECT_EQ(o.rebases(), 0u);
}

// The 2026-09-07 shape: dead reckoning snaps from (-0.76, -5.93) back to the
// origin in one 50 ms tick (odom_rebase_dist_m). The continuous pose must not
// move, and the steps after the re-base must keep integrating.
TEST(ContinuousOdom, DropsTheRebaseStepAndKeepsIntegrating)
{
  ContinuousOdom o(0.5, 1.0);
  o.Advance(Sophus::SE2d(-1.9, Eigen::Vector2d(-0.70, -5.90)));
  o.Advance(Sophus::SE2d(-1.9, Eigen::Vector2d(-0.76, -5.93)));
  const Sophus::SE2d before = o.pose();
  o.Advance(Sophus::SE2d(-1.9, Eigen::Vector2d(-0.01, -0.02)));  // re-base
  EXPECT_NEAR((o.pose().translation() - before.translation()).norm(), 0.0, 1e-9);
  EXPECT_EQ(o.rebases(), 1u);
  o.Advance(Sophus::SE2d(-1.9, Eigen::Vector2d(-0.05, -0.13)));  // 0.117 m on
  EXPECT_NEAR((o.pose().translation() - before.translation()).norm(), std::hypot(0.04, 0.11), 1e-6);
}

TEST(ContinuousOdom, DropsAYawResetToo)
{
  ContinuousOdom o(0.5, 1.0);
  o.Advance(Sophus::SE2d(2.5, Eigen::Vector2d(3.0, 3.0)));
  o.Advance(Sophus::SE2d(0.0, Eigen::Vector2d(3.0, 3.0)));  // set_pose-style yaw reset
  EXPECT_NEAR(o.pose().so2().log(), 0.0, 1e-9);  // never moved from the identity start
  EXPECT_EQ(o.rebases(), 1u);
}

TEST(ContinuousOdom, FirstSampleIsNotAStep)
{
  ContinuousOdom o;
  const auto& p = o.Advance(Sophus::SE2d(1.0, Eigen::Vector2d(40.0, -40.0)));
  EXPECT_NEAR(p.translation().norm(), 0.0, 1e-12);
  EXPECT_EQ(o.rebases(), 0u);
}
