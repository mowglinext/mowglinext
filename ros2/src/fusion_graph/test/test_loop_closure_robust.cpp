// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tests for the DCS-wrapped loop-closure factor (item #11). Pin the
// behaviour that a single bad LC must NOT destroy the trajectory:
// even after upstream guard rails (ICP rmse / divergence) and the
// rmse acceptance gate, a degenerate match can still squeak through
// on symmetric outdoor scenery. The DCS m-estimator wrapping makes
// the per-factor weight quadratically decay beyond ~1 σ, so a 10 m
// "loop closure" between two nodes that are actually 0 m apart gets
// downweighted to ~0 instead of corrupting iSAM2.

#include <cmath>
#include <cstdio>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams MakeParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x = 0.05;
  gp.wheel_sigma_y = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  // Disable adaptive noise so it doesn't confuse this test.
  gp.adaptive_noise_enabled_gain = 0.0;
  return gp;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// Outlier loop closure: build a clean trajectory of 5 nodes around a
// straight line, then inject an LC factor with a wildly wrong delta
// (claims X1 should be 5 m apart from X0 when actually they're at the
// same place). DCS must downweight the bad LC and the trajectory
// should stay near truth.
// ─────────────────────────────────────────────────────────────────────
TEST(RobustLoopClosure, OutlierLcDoesNotShiftTrajectory)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // Drive forward at 0.20 m/s for 5 ticks (0.5 s). After 5 nodes we
  // expect X4 ≈ (1.0, 0, 0).
  constexpr double kDt = 0.1;
  for (int i = 0; i < 5; ++i)
  {
    gm.AddWheelTwist(0.20, 0.0, 0.0, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (i + 1));
  }
  auto x4_before = gm.GetPose(4);
  ASSERT_TRUE(x4_before.has_value());
  std::printf("[RobustLC] X4 pre-LC: (%.3f, %.3f, %.3f)\n",
              x4_before->x(), x4_before->y(), x4_before->theta());
  ASSERT_NEAR(x4_before->x(), 1.0, 0.05);

  // Inject a deliberately bad loop closure: claim X1 should be at
  // (5, 5) relative to X0, when actually they're at (0.10, 0).
  const gtsam::Pose2 bad_delta(5.0, 5.0, 0.0);
  // Use tight sigmas so the DCS kernel actually has to fight: a
  // loose σ would naturally downweight the factor without needing
  // the robust kernel at all.
  gm.AddLoopClosure(0, 1, bad_delta, 0.05, 0.02);

  // After the bad LC, X4 must still be near (1.0, 0). If DCS didn't
  // engage, iSAM2 would pull the trajectory toward (5, 5) and X4
  // would land somewhere around (4, 4).
  auto x4_after = gm.GetPose(4);
  ASSERT_TRUE(x4_after.has_value());
  std::printf("[RobustLC] X4 post-LC: (%.3f, %.3f, %.3f)\n",
              x4_after->x(), x4_after->y(), x4_after->theta());

  // Without DCS, x4_after.x ≈ 4-5 and x4_after.y ≈ 4-5 — drift of
  // multiple metres. With DCS the factor is downweighted to near
  // zero contribution and the trajectory stays within ~50 cm of
  // truth.
  EXPECT_LT(std::abs(x4_after->x() - 1.0), 1.0);
  EXPECT_LT(std::abs(x4_after->y()), 1.0);
}

// ─────────────────────────────────────────────────────────────────────
// Consistent loop closure: deliver an LC delta that matches the
// existing wheel-between-derived relative pose. DCS should NOT
// downweight a consistent factor — the trajectory must remain
// indistinguishable from the no-LC case.
// ─────────────────────────────────────────────────────────────────────
TEST(RobustLoopClosure, ConsistentLcLeavesTrajectoryStable)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr double kDt = 0.1;
  for (int i = 0; i < 10; ++i)
  {
    gm.AddWheelTwist(0.20, 0.0, 0.0, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (i + 1));
  }
  auto x9_before = gm.GetPose(9);
  ASSERT_TRUE(x9_before.has_value());

  // Consistent LC: claim X9 - X0 ≈ (1.8, 0, 0), which matches the
  // wheel-integrated 9 * 0.20 * 0.1 = 0.18 per node × 10 nodes.
  // Actually X9 - X0 should be 9 nodes × 0.20 m/s × 0.1 s = 1.80 m.
  gm.AddLoopClosure(0, 9, gtsam::Pose2(1.80, 0.0, 0.0), 0.05, 0.02);

  auto x9_after = gm.GetPose(9);
  ASSERT_TRUE(x9_after.has_value());
  std::printf("[RobustLC] consistent LC: X9 before=(%.3f,%.3f,%.3f) after=(%.3f,%.3f,%.3f)\n",
              x9_before->x(), x9_before->y(), x9_before->theta(),
              x9_after->x(), x9_after->y(), x9_after->theta());

  // The two poses should be near-identical (the LC adds redundant
  // information that DOESN'T move iSAM2). Allow a small Δ since the
  // LC tightens the marginal posterior slightly.
  EXPECT_NEAR(x9_after->x(), x9_before->x(), 0.05);
  EXPECT_NEAR(x9_after->y(), x9_before->y(), 0.05);
}
