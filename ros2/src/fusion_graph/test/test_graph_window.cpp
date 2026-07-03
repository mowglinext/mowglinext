// Sliding-window cap on the factor graph.
//
// The graph used to grow unbounded: every Tick() appended a node and
// the periodic RebaseISAM2() rebuilt the Bayes tree keeping ALL nodes
// as priors. After a few sessions the persisted graph carried 48,000+
// nodes, which blew up marginal-covariance cost (cov_xx drifted to
// ~1 m) and kept stale far-away nodes anchoring the trajectory shape.
//
// GraphParams.max_graph_nodes caps the window: RebaseISAM2 keeps only
// the most-recent N pose nodes and drops the rest. The dropped nodes'
// constraints were already collapsed into the kept priors during the
// rebase, so the current estimate is unchanged — only the unbounded
// history is shed.

#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams MakeParams(uint64_t window)
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x = 0.05;
  gp.wheel_sigma_y = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_thresh_xy_m = 1.0e-3;
  gp.stationary_thresh_theta = 2.0e-3;
  gp.stationary_sigma_theta = 1.0e-3;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  gp.max_graph_nodes = window;
  return gp;
}

// Drive a steady straight line so every tick creates a fresh node and
// the pose advances predictably (lets us assert the kept estimate is
// still correct after the window drops old nodes).
void DriveForward(fg::GraphManager& gm, int ticks, double dt, double vx)
{
  for (int i = 0; i < ticks; ++i)
  {
    gm.AddWheelTwist(vx, 0.0, 0.0, dt);
    gm.AddGyroDelta(0.0, dt);
    gm.Tick(dt * (i + 1));
  }
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// After driving past the window size and rebasing, the live graph must
// hold at most ~max_graph_nodes nodes — not the full history.
// ─────────────────────────────────────────────────────────────────────
TEST(GraphWindow, RebaseCapsNodeCount)
{
  constexpr uint64_t kWindow = 200;
  fg::GraphManager gm(MakeParams(kWindow));
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // 1000 ticks → 1000 nodes if uncapped.
  DriveForward(gm, 1000, 0.1, 0.5);

  const uint64_t before = gm.Stats().total_nodes;
  gm.RebaseISAM2();
  const uint64_t after_live = gm.LiveNodeCount();

  std::printf("[GraphWindow] total_nodes(monotonic)=%llu live_after_rebase=%llu (window=%llu)\n",
              static_cast<unsigned long long>(before),
              static_cast<unsigned long long>(after_live),
              static_cast<unsigned long long>(kWindow));

  // total_nodes keeps counting monotonically (it's the next-index, not
  // the live node count) — the cap applies to the live graph.
  EXPECT_GT(before, kWindow);
  EXPECT_LE(after_live, kWindow + 1);  // +1 slack for the anchor node
}

// ─────────────────────────────────────────────────────────────────────
// The windowed rebase must not corrupt the current pose estimate: the
// latest node should still report the dead-reckoned position.
// ─────────────────────────────────────────────────────────────────────
TEST(GraphWindow, EstimatePreservedAcrossWindowedRebase)
{
  constexpr uint64_t kWindow = 200;
  fg::GraphManager gm(MakeParams(kWindow));
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 1000;
  constexpr double kDt = 0.1;
  constexpr double kVx = 0.5;
  DriveForward(gm, kTicks, kDt, kVx);

  const auto before = gm.LatestSnapshot();
  ASSERT_TRUE(before.has_value());
  const double x_before = before->pose.x();

  gm.RebaseISAM2();

  const auto after = gm.LatestSnapshot();
  ASSERT_TRUE(after.has_value());
  const double x_after = after->pose.x();

  std::printf("[GraphWindow] x before=%.3f after=%.3f (expected≈%.3f)\n",
              x_before,
              x_after,
              kVx * kDt * kTicks);

  // The rebase is gauge-preserving on the kept nodes — the latest pose
  // must be unchanged to within optimizer noise.
  EXPECT_NEAR(x_after, x_before, 0.05);
  // And it should still reflect the ~50 m of straight-line travel.
  EXPECT_NEAR(x_after, kVx * kDt * kTicks, 1.0);
}

// ─────────────────────────────────────────────────────────────────────
// Loop-closure candidates must stay within the max_graph_nodes window.
//
// 2026-06-09 OOM: FindLoopClosureCandidates scanned ALL retained scans and
// only filtered on distance + age + HasPoseAt. BEFORE a maintenance rebase
// runs, HasPoseAt is true for every accumulated node, so a robot dwelling
// among old nodes formed loop closures to ancient indices (LC to node 2098
// while the live index was ~10138) — unbounded iSAM2 factor growth that the
// host OOM-killer ended. The window bound must exclude any candidate older
// than next_index_ - max_graph_nodes, regardless of rebase timing.
// ─────────────────────────────────────────────────────────────────────
TEST(GraphWindow, LoopClosureCandidatesStayWithinWindow)
{
  constexpr uint64_t kWindow = 200;
  fg::GraphManager gm(MakeParams(kWindow));
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // A small body-frame scan attached to every node so each is a candidate.
  // Geometry is irrelevant — this exercises the index/window filter, not ICP.
  const std::vector<Eigen::Vector2d> scan = {{0.10, 0.0},
                                             {0.20, 0.05},
                                             {0.15, -0.05},
                                             {0.05, 0.10}};

  // Crawl forward so every node stays within a couple of metres (distance gate
  // never filters) while the index marches well past the window.
  constexpr double kDt = 0.1;
  constexpr int kTicks = 600;
  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(0.02, 0.0, 0.0, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (i + 1));
    const auto s = gm.LatestSnapshot();
    if (s)
      gm.AttachScan(s->node_index, scan);
  }

  // Deliberately do NOT rebase: that is exactly the pre-rebase window where the
  // leak occurred (every accumulated node still has a live pose).
  const auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const uint64_t latest = snap->node_index;
  ASSERT_GT(latest, kWindow);  // there ARE out-of-window nodes that could leak

  // Huge max_dist + zero min_age so neither the distance nor the age gate fires
  // — only the new window bound can keep ancient nodes out.
  const auto cands = gm.FindLoopClosureCandidates(latest,
                                                  /*max_dist_m=*/1.0e6,
                                                  /*min_age_s=*/0.0,
                                                  /*max_candidates=*/1000);
  const uint64_t window_cutoff = latest + 1 - kWindow;  // == next_index_ - window
  uint64_t min_idx = std::numeric_limits<uint64_t>::max();
  for (uint64_t idx : cands)
  {
    EXPECT_GE(idx, window_cutoff) << "FindLoopClosureCandidates offered node " << idx
                                  << " older than the window cutoff " << window_cutoff;
    min_idx = std::min(min_idx, idx);
  }
  std::printf("[GraphWindow] latest=%llu window=%llu cutoff=%llu cands=%zu min_idx=%llu\n",
              static_cast<unsigned long long>(latest),
              static_cast<unsigned long long>(kWindow),
              static_cast<unsigned long long>(window_cutoff),
              cands.size(),
              static_cast<unsigned long long>(cands.empty() ? 0 : min_idx));
  // In-window candidates DO still exist (the filter bounds, not disables, LC).
  EXPECT_FALSE(cands.empty());
}
