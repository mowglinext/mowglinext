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

  // Drive forward at 0.20 m/s for 5 ticks (0.5 s). The first Tick creates
  // node 0, so after 5 ticks GetPose(4) is the 5th node at 5 × 0.20 m/s ×
  // 0.1 s = 1.0 m of integrated distance spread over 5 × 0.02 m steps →
  // X4 ≈ (0.10, 0, 0).
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
              x4_before->x(),
              x4_before->y(),
              x4_before->theta());
  ASSERT_NEAR(x4_before->x(), 0.10, 0.02);

  // Inject a deliberately bad loop closure: claim node 1 is at (5, 5)
  // relative to node 0, when the true X1 - X0 delta is only ~0.02 m.
  const gtsam::Pose2 bad_delta(5.0, 5.0, 0.0);
  // Use tight sigmas so the DCS kernel actually has to fight: a
  // loose σ would naturally downweight the factor without needing
  // the robust kernel at all.
  gm.AddLoopClosure(0, 1, bad_delta, 0.05, 0.02);

  // After the bad LC, X4 must still be near (0.10, 0). If DCS didn't
  // engage, iSAM2 would pull the trajectory toward (5, 5) and X4
  // would land metres away.
  auto x4_after = gm.GetPose(4);
  ASSERT_TRUE(x4_after.has_value());
  std::printf("[RobustLC] X4 post-LC: (%.3f, %.3f, %.3f)\n",
              x4_after->x(),
              x4_after->y(),
              x4_after->theta());

  // Without DCS, x4_after would drift metres toward (5, 5). With DCS the
  // factor is downweighted to near-zero contribution and the trajectory
  // stays within ~10 cm of truth.
  EXPECT_LT(std::abs(x4_after->x() - 0.10), 0.5);
  EXPECT_LT(std::abs(x4_after->y()), 0.5);
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
  // The node-period gate uses now_s - last_node_time_s_, and now_s =
  // 0.1*(i+1) lands a few inter-tick deltas at 0.0999…8 < node_period_s
  // (0.1) in IEEE-754, so 10 Tick() calls do NOT create exactly nodes
  // 0..9 — a couple of ticks are skipped (their motion still accumulates
  // into the next node). Query the actual latest node index rather than a
  // hardcoded 9, and feed the LC the true node-0 → latest relative delta.
  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const uint64_t last_idx = snap->node_index;
  auto x_last_before = gm.GetPose(last_idx);
  auto x0 = gm.GetPose(0);
  ASSERT_TRUE(x_last_before.has_value());
  ASSERT_TRUE(x0.has_value());

  // Consistent LC: feed exactly the wheel-integrated node-0 → latest delta
  // (straight forward, no rotation) — a redundant, consistent factor DCS
  // must NOT downweight.
  const double consistent_dx = x_last_before->x() - x0->x();
  gm.AddLoopClosure(0, last_idx, gtsam::Pose2(consistent_dx, 0.0, 0.0), 0.05, 0.02);

  auto x_last_after = gm.GetPose(last_idx);
  ASSERT_TRUE(x_last_after.has_value());
  std::printf("[RobustLC] consistent LC: X%llu before=(%.3f,%.3f,%.3f) after=(%.3f,%.3f,%.3f)\n",
              static_cast<unsigned long long>(last_idx),
              x_last_before->x(),
              x_last_before->y(),
              x_last_before->theta(),
              x_last_after->x(),
              x_last_after->y(),
              x_last_after->theta());

  // The two poses should be near-identical (the LC adds redundant
  // information that DOESN'T move iSAM2). Allow a small Δ since the
  // LC tightens the marginal posterior slightly.
  EXPECT_NEAR(x_last_after->x(), x_last_before->x(), 0.05);
  EXPECT_NEAR(x_last_after->y(), x_last_before->y(), 0.05);
}
