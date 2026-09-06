// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit test for GraphManager::RigidTransformAll — a pure gauge shift of the
// live trajectory (dock re-pin). Ported from the removed keyframe-map test:
// the trajectory must move by exactly the correction so every absolute
// reference the node holds (persisted graph, LiDAR occupancy grid) stays in
// one gauge with it.

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose2.h>

namespace fg = fusion_graph;

namespace
{
fg::GraphParams TickParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  return gp;
}
}  // namespace

TEST(RigidTransform, LiveTrajectoryShiftsByCorrection)
{
  // Arrange: a short dead-reckoned trajectory.
  fg::GraphManager gm(TickParams());
  gm.Initialize(gtsam::Pose2(0, 0, 0), 0.0);
  for (int i = 1; i <= 3; ++i)
  {
    gm.AddWheelTwist(0.5, 0, 0, 0.1);
    gm.AddGyroDelta(0, 0.1);
    gm.Tick(0.1 * i);
  }
  auto before = gm.LatestSnapshot();
  ASSERT_TRUE(before.has_value());
  const double node_x_before = before->pose.x();
  const double node_y_before = before->pose.y();

  // Act: a pure-translation gauge correction (e.g. a dock re-pin).
  const gtsam::Pose2 correction(0.5, -0.3, 0.0);
  gm.RigidTransformAll(correction);

  // Assert: the live trajectory moved by the same correction.
  auto after = gm.LatestSnapshot();
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->pose.x(), node_x_before + 0.5, 1e-6);
  EXPECT_NEAR(after->pose.y(), node_y_before - 0.3, 1e-6);
  EXPECT_NEAR(after->pose.theta(), before->pose.theta(), 1e-6);
}
