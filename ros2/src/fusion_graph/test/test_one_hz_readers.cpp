// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The 1 Hz diagnostics / markers readers no longer materialise the whole
// iSAM2 estimate: LiveNodeCount counts the linearisation point's pose keys
// and GetAllPoses retracts the linearisation point by the delta directly.
// This pins that they still report exactly what a full calculateEstimate()
// reports (GetPose, unchanged, as a cross-check), through the fault and
// lifecycle paths.

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>
#include <string>
#include <typeinfo>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <unistd.h>

namespace fusion_graph
{
struct GraphManagerTestAccess
{
  static gtsam::Values Estimate(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    return g.isam_.calculateEstimate();
  }
};
}  // namespace fusion_graph

namespace fg = fusion_graph;

namespace
{

// Bit for bit, so a -0.0 / +0.0 difference counts too.
void ExpectSamePose(const gtsam::Pose2& a, const gtsam::Pose2& b)
{
  const double pa[] = {a.x(), a.y(), a.rotation().c(), a.rotation().s()};
  const double pb[] = {b.x(), b.y(), b.rotation().c(), b.rotation().s()};
  EXPECT_EQ(std::memcmp(pa, pb, sizeof(pa)), 0)
      << "(" << pa[0] << ", " << pa[1] << ", " << pa[2] << ", " << pa[3] << ") vs (" << pb[0]
      << ", " << pb[1] << ", " << pb[2] << ", " << pb[3] << ")";
}

// Whether a call threw, and with which exception type.
template <typename F>
std::string Outcome(F&& f)
{
  try
  {
    f();
    return "ok";
  }
  catch (const std::exception& e)
  {
    return typeid(e).name();
  }
}

}  // namespace

// The 1 Hz readers (LiveNodeCount, GetAllPoses, GetPose) must report exactly
// what a full calculateEstimate() reports — what they returned before they
// stopped materialising it — through every fault and lifecycle path: stale
// and deep GNSS, wrong fixes, deep ForceAnchor, rebases, rigid re-pin,
// save -> reset -> load, clear_graph. A NaN that reaches the graph must make
// them fail exactly when the full estimate fails.
TEST(OneHzReaders, MatchTheFullEstimateUnderFaults)
{
  using Access = fg::GraphManagerTestAccess;
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.max_graph_nodes = 400;
  fg::GraphManager gm(gp);
  const auto prefix =
      (std::filesystem::temp_directory_path() / ("fg_readers_" + std::to_string(::getpid())))
          .string();
  std::mt19937_64 rng(3);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  auto check_readers = [&](int tick)
  {
    SCOPED_TRACE("tick " + std::to_string(tick));
    const auto est = Access::Estimate(gm);
    size_t pose_keys = 0;
    for (const auto& kv : est)
      pose_keys += gtsam::Symbol(kv.key).chr() == 'x';
    EXPECT_EQ(gm.LiveNodeCount(), pose_keys);
    const auto poses = gm.GetAllPoses();
    ASSERT_EQ(poses.size(), pose_keys);
    for (const auto& [idx, p] : poses)
    {
      ExpectSamePose(p, est.at<gtsam::Pose2>(fg::PoseKey(idx)));
      if (idx % 50 == 0)
        ExpectSamePose(*gm.GetPose(idx), p);
    }
  };

  gm.Initialize(gtsam::Pose2(), 0.0);
  double x = 0.0;
  int ops = 0;
  for (int tick = 1; tick <= 1500; ++tick)
  {
    const double now = 0.101 * tick;
    gm.AddWheelTwist(0.3, 0.0, 0.02, 0.1);
    gm.AddGyroDelta(0.02 + (u01(rng) < 0.02 ? 1.5 : 0.0), 0.1);  // gyro spike
    x += 0.03;
    const double r = u01(rng);
    if (r < 0.04)
      gm.QueueGnss(x + 2.0, 0.0, 0.01, true, std::nullopt, 0);  // wrong fix
    else if (r < 0.08)
      gm.QueueGnss(x, 0.0, 0.01, true, gm.FindNodeAtOrBefore(now - 20.0 * u01(rng)), 0);
    else if (r < 0.9)
      gm.QueueGnss(x + 0.01 * n01(rng), 0.0, r < 0.12 ? 3.0 : 0.01, true, std::nullopt, 0);
    gm.Tick(now);
    if (tick % 7 == 0)
      check_readers(tick);
    switch (tick % 150)
    {
      case 30:
        if (const auto deep = gm.FindNodeAtOrBefore(now - 25.0))
          gm.ForceAnchor(*deep, *gm.GetPose(*deep), 0.005, 0.035);
        break;
      case 60:
        gm.RebaseISAM2();
        break;
      case 90:
        gm.RigidTransformAll(gtsam::Pose2(0.2 * n01(rng), 0.2 * n01(rng), 0.05 * n01(rng)));
        break;
      case 120:
        ASSERT_TRUE(gm.Save(prefix));
        gm.Reset();
        ASSERT_TRUE(gm.Load(prefix));
        break;
      case 149:
        if (tick < 1000)
          continue;
        gm.Reset();  // clear_graph, re-seeded at the current pose
        gm.Initialize(gtsam::Pose2(x, 0.0, 0.0), now);
        break;
      default:
        continue;
    }
    ++ops;
    check_readers(tick);
  }
  EXPECT_EQ(ops, 44);
  for (const char* ext : {".checkpoint", ".checkpoint.previous"})
    std::filesystem::remove(prefix + ext);

  // NaN reaching the graph: whatever the full estimate does, the readers do.
  gm.QueueGnss(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.01, true, std::nullopt, 0);
  gm.AddWheelTwist(0.3, 0.0, 0.0, 0.1);
  gm.Tick(0.101 * 1501);
  if (gm.IsInitialized())
  {
    const auto full = Outcome(
        [&]
        {
          Access::Estimate(gm);
        });
    EXPECT_EQ(Outcome(
                  [&]
                  {
                    gm.LiveNodeCount();
                  }),
              full);
    EXPECT_EQ(Outcome(
                  [&]
                  {
                    gm.GetAllPoses();
                  }),
              full);
  }
}

// A pose with a signed zero must come out exactly as calculateEstimate()
// returns it: Values::retract retracts even a zero delta, giving +0.0.
TEST(OneHzReaders, SignedZeroPosesMatchTheFullEstimate)
{
  using Access = fg::GraphManagerTestAccess;
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(-0.0, 1.0, -0.0), 0.0);
  for (int round = 0; round < 2; ++round)
  {
    const auto est = Access::Estimate(gm);
    const auto poses = gm.GetAllPoses();
    ASSERT_EQ(poses.size(), 1u);
    ExpectSamePose(poses.at(0), est.at<gtsam::Pose2>(fg::PoseKey(0)));
    gm.RebaseISAM2();  // the rebuilt window: every delta exactly zero
  }
}
