// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager corrections from the 2026-11 fusion_graph review:
//   - RebaseISAM2 keeps the head's uncertainty and the window's linkage
//     (it used to replace everything with independent 5 cm / 0.05 rad priors);
//   - AnchorYaw corrects heading without pinning XY;
//   - a GNSS fix can be retimed with an antenna point in the node frame;
//   - IMU preintegration survives a reload (bias keys are not persisted).

#include <cmath>
#include <filesystem>
#include <string>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams Params()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  gp.cov_update_every_n = 1;  // fresh marginal on every snapshot
  gp.max_graph_nodes = 0;  // keep the whole window
  return gp;
}

// Drive straight; returns the next tick time.
double Drive(fg::GraphManager& gm, double t, int ticks, double vx = 0.5)
{
  constexpr double kDt = 0.1;
  for (int i = 0; i < ticks; ++i)
  {
    t += kDt;
    gm.AddWheelTwist(vx, 0.0, 0.0, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(t);
  }
  return t;
}

}  // namespace

TEST(GraphCorrections, RebaseKeepsTheHeadMarginal)
{
  fg::GraphManager gm(Params());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  Drive(gm, 0.0, 100);  // 5 m of dead reckoning, no absolute evidence
  const auto before = gm.LatestSnapshot();
  ASSERT_TRUE(before);

  gm.RebaseISAM2();
  const auto after = gm.LatestSnapshot();
  ASSERT_TRUE(after);

  // Pose unchanged; uncertainty NOT collapsed to the old 5 cm rebase prior.
  EXPECT_NEAR(after->pose.x(), before->pose.x(), 1e-6);
  EXPECT_NEAR(after->pose.y(), before->pose.y(), 1e-6);
  EXPECT_GT(before->covariance(0, 0), 2.0 * 0.05 * 0.05);
  EXPECT_NEAR(after->covariance(0, 0), before->covariance(0, 0), 0.05 * before->covariance(0, 0));
  EXPECT_NEAR(after->covariance(2, 2), before->covariance(2, 2), 0.05 * before->covariance(2, 2));
}

TEST(GraphCorrections, DelayedFixOnAnOldNodeStillMovesTheHeadAfterRebase)
{
  fg::GraphManager gm(Params());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  double t = Drive(gm, 0.0, 100);
  gm.RebaseISAM2();

  // A fix taken at node 50 says the robot was 0.2 m to the left there.
  const auto old = gm.GetPose(50);
  ASSERT_TRUE(old);
  ASSERT_TRUE(gm.QueueGnss(old->x(), old->y() + 0.2, 0.01, false, 50U));
  t = Drive(gm, t, 1);
  const auto head = gm.LatestSnapshot();
  ASSERT_TRUE(head);
  // Linked window: the head follows. (With independent priors it stayed at 0.)
  EXPECT_GT(head->pose.y(), 0.05);
}

TEST(GraphCorrections, AnchorYawCorrectsHeading)
{
  fg::GraphManager gm(Params());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  Drive(gm, 0.0, 5);
  const auto head = gm.LatestSnapshot();
  ASSERT_TRUE(head);
  gm.AnchorYaw(head->node_index, 0.5, 0.01);
  const auto after = gm.LatestSnapshot();
  ASSERT_TRUE(after);
  EXPECT_NEAR(after->pose.theta(), 0.5, 0.1);
  EXPECT_GT(after->estimate_revision, head->estimate_revision);
}

TEST(GraphCorrections, RetimedAntennaPointReplacesTheLeverArm)
{
  // The fix at x = 1.0 was acquired 1 m AHEAD of node 0: with the antenna
  // point (1, 0) in node 0's frame it agrees with node 0 at the origin.
  fg::GraphManager retimed(Params());
  retimed.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(retimed.QueueGnss(1.0, 0.0, 0.01, false, 0U, 0, gtsam::Vector2(1.0, 0.0)));
  Drive(retimed, 0.0, 1, 0.0);
  EXPECT_NEAR(retimed.GetPose(0)->x(), 0.0, 0.01);

  // Without it the same fix drags node 0 to x ≈ 1.
  fg::GraphManager plain(Params());
  plain.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(plain.QueueGnss(1.0, 0.0, 0.01, false, 0U));
  Drive(plain, 0.0, 1, 0.0);
  EXPECT_GT(plain.GetPose(0)->x(), 0.5);
}

TEST(GraphCorrections, ImuPreintegrationSurvivesAReload)
{
  // Load() restores pose keys only, so the reloaded graph has no bias
  // variables whichever mode wrote it; save without preint to keep this test
  // about the reload, not about serializing the bias values.
  auto gp = Params();
  const std::string prefix =
      (std::filesystem::path(testing::TempDir()) / "fg_preint_reload").string();
  {
    fg::GraphManager gm(gp);
    gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
    Drive(gm, 0.0, 10);
    ASSERT_TRUE(gm.Save(prefix));
  }
  gp.use_imu_preint = true;
  fg::GraphManager reloaded(gp);
  ASSERT_TRUE(reloaded.Load(prefix));
  const auto t0 = reloaded.LatestSnapshot()->timestamp;
  Drive(reloaded, t0, 5);
  // The first node after the load references the previous bias key, which
  // Load() does not restore; it must be re-created, not reset the graph.
  EXPECT_TRUE(reloaded.IsInitialized());
  EXPECT_EQ(reloaded.Stats().isam_resets, 0U);
  EXPECT_GE(reloaded.Stats().total_nodes, 16U);
}
