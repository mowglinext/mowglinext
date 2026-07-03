// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the RTK-anchored keyframe map (GraphManager keyframe API).
// The store methods do not require an initialized graph — they only touch the
// keyframes_ map — so the tests construct a GraphManager and call them directly.

#include <cstdio>
#include <string>
#include <vector>

#include "fusion_graph/graph_manager.hpp"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose2.h>

namespace fg = fusion_graph;

namespace
{
fg::GraphParams KfParams(double spacing, uint64_t cap)
{
  fg::GraphParams gp;
  gp.kf_spacing_m = spacing;
  gp.max_keyframes = cap;
  return gp;
}

std::vector<Eigen::Vector2d> DummyScan(int n = 10)
{
  std::vector<Eigen::Vector2d> s;
  for (int i = 0; i < n; ++i)
    s.emplace_back(0.1 * i, 0.2 * i);
  return s;
}
}  // namespace

TEST(KeyframeMap, AddAssignsSequentialIds)
{
  fg::GraphManager gm(KfParams(0.2, 0));
  auto id0 = gm.AddKeyframe(gtsam::Pose2(0, 0, 0), DummyScan());
  auto id1 = gm.AddKeyframe(gtsam::Pose2(1, 0, 0), DummyScan());
  ASSERT_TRUE(id0.has_value());
  ASSERT_TRUE(id1.has_value());
  EXPECT_EQ(*id0, 0u);
  EXPECT_EQ(*id1, 1u);
  EXPECT_EQ(gm.KeyframeCount(), 2u);
}

TEST(KeyframeMap, SpatialDecimationRejectsNear)
{
  fg::GraphManager gm(KfParams(0.5, 0));  // decimation radius 0.25 m
  auto id0 = gm.AddKeyframe(gtsam::Pose2(0, 0, 0), DummyScan());
  ASSERT_TRUE(id0.has_value());
  // within 0.25 m → rejected
  auto idn = gm.AddKeyframe(gtsam::Pose2(0.1, 0.0, 0), DummyScan());
  EXPECT_FALSE(idn.has_value());
  EXPECT_EQ(gm.KeyframeCount(), 1u);
  // beyond 0.25 m → accepted
  auto idf = gm.AddKeyframe(gtsam::Pose2(0.5, 0.0, 0), DummyScan());
  EXPECT_TRUE(idf.has_value());
  EXPECT_EQ(gm.KeyframeCount(), 2u);
}

TEST(KeyframeMap, GetKeyframeRoundTrips)
{
  fg::GraphManager gm(KfParams(0.2, 0));
  const gtsam::Pose2 p(3.0, 4.0, 0.5);
  auto scan = DummyScan(5);
  auto id = gm.AddKeyframe(p, scan);
  ASSERT_TRUE(id.has_value());
  auto kf = gm.GetKeyframe(*id);
  ASSERT_TRUE(kf.has_value());
  EXPECT_NEAR(kf->abs_pose.x(), 3.0, 1e-12);
  EXPECT_NEAR(kf->abs_pose.y(), 4.0, 1e-12);
  EXPECT_NEAR(kf->abs_pose.theta(), 0.5, 1e-12);
  ASSERT_EQ(kf->scan_body.size(), scan.size());
  EXPECT_FALSE(gm.GetKeyframe(9999).has_value());
}

TEST(KeyframeMap, FindNearSortedNoWindow)
{
  fg::GraphManager gm(KfParams(0.2, 0));
  gm.AddKeyframe(gtsam::Pose2(0, 0, 0), DummyScan());  // id0 @ origin
  gm.AddKeyframe(gtsam::Pose2(1, 0, 0), DummyScan());  // id1
  gm.AddKeyframe(gtsam::Pose2(2, 0, 0), DummyScan());  // id2
  gm.AddKeyframe(gtsam::Pose2(5, 0, 0), DummyScan());  // id3 (far)
  // radius 2.5 → ids 0,1,2 (not 3), sorted by ascending distance
  auto near = gm.FindKeyframesNearXY(0.0, 0.0, 2.5, 10);
  ASSERT_EQ(near.size(), 3u);
  EXPECT_EQ(near[0], 0u);
  EXPECT_EQ(near[1], 1u);
  EXPECT_EQ(near[2], 2u);
  // max_candidates cap
  auto cap = gm.FindKeyframesNearXY(0.0, 0.0, 100.0, 2);
  EXPECT_EQ(cap.size(), 2u);
}

TEST(KeyframeMap, MaxKeyframesEviction)
{
  fg::GraphManager gm(KfParams(0.1, 3));  // cap 3, tiny decimation
  for (int i = 0; i < 5; ++i)
    gm.AddKeyframe(gtsam::Pose2(i * 1.0, 0, 0), DummyScan());
  EXPECT_EQ(gm.KeyframeCount(), 3u);
  // oldest (ids 0,1) evicted; ids 2,3,4 remain
  EXPECT_FALSE(gm.GetKeyframe(0).has_value());
  EXPECT_FALSE(gm.GetKeyframe(1).has_value());
  EXPECT_TRUE(gm.GetKeyframe(2).has_value());
  EXPECT_TRUE(gm.GetKeyframe(4).has_value());
}

TEST(KeyframeMap, ClearEmpties)
{
  fg::GraphManager gm(KfParams(0.2, 0));
  gm.AddKeyframe(gtsam::Pose2(0, 0, 0), DummyScan());
  gm.AddKeyframe(gtsam::Pose2(1, 0, 0), DummyScan());
  EXPECT_EQ(gm.KeyframeCount(), 2u);
  gm.ClearKeyframes();
  EXPECT_EQ(gm.KeyframeCount(), 0u);
  auto id = gm.AddKeyframe(gtsam::Pose2(0, 0, 0), DummyScan());
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, 0u);  // ids restart after clear
}

// ── Phase C: the queued scan-to-keyframe ABSOLUTE constraint is consumed ──
namespace
{
fg::GraphParams TickParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x = 0.05;
  gp.wheel_sigma_y = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  return gp;
}
}  // namespace

TEST(KeyframeApply, AbsoluteConstraintPullsNode)
{
  fg::GraphManager gm(TickParams());
  gm.Initialize(gtsam::Pose2(0, 0, 0), 0.0);

  // Drive forward 5 ticks @ 0.5 m/s (exact tick times → a node each tick).
  for (int i = 1; i <= 5; ++i)
  {
    gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);
    gm.Tick(0.1 * i);
  }
  auto before = gm.LatestSnapshot();
  ASSERT_TRUE(before.has_value());
  const double x_dr = before->pose.x();

  // The 6th node dead-reckons to ~dr6; we queue an ABSOLUTE xy prior 0.10 m
  // ahead at a MODERATE sigma (a few sigma of conflict — a realistic keyframe
  // correction, not the 35-sigma blow-up that Huber guards against in
  // production). The PoseTranslationPrior must pull the new node clearly past
  // pure dead reckoning toward the target, without overshooting it.
  const double dr6 = x_dr + 0.05;  // pure dead reckoning for node 6
  const double target_x = dr6 + 0.10;  // prior pulls 0.10 m further
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  gm.AddGyroDelta(0.0, 0.1);
  gm.QueueScanToKeyframe(gtsam::Vector2(target_x, 0.0), 0.03, /*robust=*/false);
  auto out = gm.Tick(0.1 * 6);
  ASSERT_TRUE(out.has_value());

  const double x_after = out->pose.x();
  EXPECT_GT(x_after, dr6 + 0.03) << "constraint did not pull the node: x_after=" << x_after
                                 << " dr6=" << dr6 << " target=" << target_x;
  EXPECT_LT(x_after, target_x + 0.03) << "node overshot the prior";
}

// ── Phase D: keyframe persistence (round-trip, back-compat, datum guard) ──
TEST(KeyframePersistence, RoundTripBackCompatDatum)
{
  const std::string prefix = "/tmp/fg_kf_test";
  for (const char* ext : {".graph", ".scans", ".meta", ".keyframes"})
    std::remove((prefix + ext).c_str());

  // (1) Build a small graph + 2 keyframes, datum (48, 2), and Save.
  {
    auto gp = TickParams();
    gp.datum_lat = 48.0;
    gp.datum_lon = 2.0;
    fg::GraphManager gm(gp);
    gm.Initialize(gtsam::Pose2(0, 0, 0), 0.0);
    for (int i = 1; i <= 5; ++i)
    {
      gm.AddWheelTwist(0.5, 0, 0, 0.1);
      gm.AddGyroDelta(0, 0.1);
      gm.Tick(0.1 * i);
    }
    gm.AddKeyframe(gtsam::Pose2(1.0, 2.0, 0.3), DummyScan(7));
    gm.AddKeyframe(gtsam::Pose2(3.0, 2.0, 0.4), DummyScan(9));
    ASSERT_TRUE(gm.Save(prefix));
  }

  // (2) Reload with the SAME datum → keyframes restored intact.
  {
    auto gp = TickParams();
    gp.datum_lat = 48.0;
    gp.datum_lon = 2.0;
    fg::GraphManager gm(gp);
    ASSERT_TRUE(gm.Load(prefix));
    EXPECT_EQ(gm.KeyframeCount(), 2u);
    auto near = gm.FindKeyframesNearXY(1.0, 2.0, 0.5, 5);
    ASSERT_EQ(near.size(), 1u);
    auto kf = gm.GetKeyframe(near[0]);
    ASSERT_TRUE(kf.has_value());
    EXPECT_NEAR(kf->abs_pose.x(), 1.0, 1e-9);
    EXPECT_NEAR(kf->abs_pose.y(), 2.0, 1e-9);
    EXPECT_NEAR(kf->abs_pose.theta(), 0.3, 1e-9);
    EXPECT_EQ(kf->scan_body.size(), 7u);
  }

  // (3) Datum mismatch (different garden) → Load rejected.
  {
    auto gp = TickParams();
    gp.datum_lat = 49.0;
    gp.datum_lon = 3.0;
    fg::GraphManager gm(gp);
    EXPECT_FALSE(gm.Load(prefix));
  }

  // (4) Back-compat: remove .keyframes; Load still succeeds (empty map).
  {
    std::remove((prefix + ".keyframes").c_str());
    auto gp = TickParams();
    gp.datum_lat = 48.0;
    gp.datum_lon = 2.0;
    fg::GraphManager gm(gp);
    EXPECT_TRUE(gm.Load(prefix));
    EXPECT_EQ(gm.KeyframeCount(), 0u);
  }

  for (const char* ext : {".graph", ".scans", ".meta", ".keyframes"})
    std::remove((prefix + ext).c_str());
}

// ── Phase E: keyframes co-transform with the live graph on a gauge shift ──
TEST(KeyframeGauge, KeyframesFollowRigidTransform)
{
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

  auto id = gm.AddKeyframe(gtsam::Pose2(1.0, 1.0, 0.0), DummyScan());
  ASSERT_TRUE(id.has_value());

  // A pure-translation gauge correction (e.g. a dock re-pin).
  const gtsam::Pose2 correction(0.5, -0.3, 0.0);
  gm.RigidTransformAll(correction);

  // The keyframe abs_pose moved by the same correction...
  auto kf = gm.GetKeyframe(*id);
  ASSERT_TRUE(kf.has_value());
  EXPECT_NEAR(kf->abs_pose.x(), 1.5, 1e-9);  // 1.0 + 0.5
  EXPECT_NEAR(kf->abs_pose.y(), 0.7, 1e-9);  // 1.0 - 0.3
  // ...and so did the live trajectory (map + graph stay in one gauge).
  auto after = gm.LatestSnapshot();
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->pose.x(), node_x_before + 0.5, 1e-6);
}
