// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/motion_source_history.hpp"
#include <gtest/gtest.h>

namespace fusion_graph
{
class GraphManagerSourceTestAccess
{
public:
  static std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> Snapshot(GraphManager& graph)
  {
    std::lock_guard<std::mutex> lock(graph.mu_);
    return {graph.isam_.getFactorsUnsafe(), graph.isam_.calculateEstimate()};
  }
};
namespace
{
GraphParams Params()
{
  GraphParams p;
  p.node_period_s = 0.04;
  p.stationary_node_period_s = 0.0;
  p.gyro_bias_estimation_enabled = false;
  p.adaptive_noise_enabled_gain = 0.0;
  return p;
}
void Seed(GraphManager& graph)
{
  graph.SetConsumedMotionBaseline(1, 0);
  ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(), 0, 0, 1));
}
void Consume(GraphManager& graph,
             uint64_t id,
             int64_t start,
             int64_t end,
             double vx = 0.15,
             double raw_gyro = 0.0,
             double wheel_wz = 0.0)
{
  const double dt = static_cast<double>(end - start) * 1e-9;
  graph.AddWheelTwist(vx, 0.0, wheel_wz, dt);
  const double bias = graph.AddGyroDelta(raw_gyro, dt);
  ASSERT_TRUE(graph.AddConsumedMotionStep(
      {1,
       start,
       end,
       {{MotionSourceKind::Wheel, id, end}, {start, end}, vx, 0.0, wheel_wz},
       {{MotionSourceKind::Gyro, id, end}, {start, end}, raw_gyro},
       bias,
       true,
       true}));
}
}  // namespace

TEST(SourceEpochQuery, ExistingAndNewInteriorQueriesDoNotInsertOrAdvance)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000, 0.15, 0.5, 0.5);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.08, 0.001, false, 50000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  const auto before = GraphManagerSourceTestAccess::Snapshot(graph);
  const auto latest = *graph.LatestSnapshot();
  const auto count = graph.LiveNodeCount();
  const auto acquired = graph.EstimatePoseAtSource(1, 50000000);
  ASSERT_TRUE(acquired);
  EXPECT_LT(
      acquired->localCoordinates(before.second.at<gtsam::Pose2>(gtsam::Symbol('a', 0))).norm(),
      1e-12);
  const auto interior = graph.EstimatePoseAtSource(1, 75000000);
  ASSERT_TRUE(interior);
  const auto repeated = graph.EstimatePoseAtSource(1, 75000000);
  ASSERT_TRUE(repeated);
  EXPECT_LT(interior->localCoordinates(*repeated).norm(), 1e-12);
  const auto after = GraphManagerSourceTestAccess::Snapshot(graph);
  EXPECT_EQ(before.first.size(), after.first.size());
  EXPECT_EQ(before.second.size(), after.second.size());
  EXPECT_EQ(graph.LiveNodeCount(), count);
  EXPECT_EQ(graph.LatestSnapshot()->revision, latest.revision);
  EXPECT_EQ(graph.LatestSnapshot()->source_stamp_ns, latest.source_stamp_ns);
  EXPECT_EQ(graph.SourceYawFactorCount(), 1U);
  EXPECT_EQ(graph.SourceGnssFactorCount(), 0U);
  // A later real observation still needs its own live acquisition. The
  // repeated read-only query must not consume a key or leave a ghost alias.
  ASSERT_TRUE(graph.QueueYawAtEpoch(interior->theta(), 0.1, false, 75000000, 2, 1));
  ASSERT_TRUE(graph.TickSource(100000002, 100000000, 1));
  EXPECT_EQ(graph.LiveNodeCount(), count + 1);
  const auto final = GraphManagerSourceTestAccess::Snapshot(graph);
  EXPECT_TRUE(final.second.exists(gtsam::Symbol('a', 1)));
}

TEST(SourceEpochQuery, HistoricalHeadingSurvivesARealInterveningHalfTurn)
{
  GraphManager graph(Params());
  Seed(graph);
  for (int i = 1; i <= 175; ++i)
  {
    const double omega = i <= 75 ? 0.0 : M_PI / 4.0;
    Consume(graph, i, (i - 1) * 40000000LL, i * 40000000LL, i <= 75 ? 0.15 : 0.0, omega, omega);
    ASSERT_TRUE(graph.TickSource(i * 40000000LL, i * 40000000LL, 1));
  }
  EXPECT_NEAR(graph.LatestSnapshot()->pose.theta(), M_PI, 1e-10);
  for (int64_t source : {800000000LL, 1600000000LL, 2400000000LL, 1610000000LL})
  {
    const auto historical = graph.EstimatePoseAtSource(1, source);
    ASSERT_TRUE(historical);
    EXPECT_NEAR(historical->theta(), 0.0, 1e-10);
  }
  EXPECT_FALSE(graph.EstimatePoseAtSource(2, 1600000000));
  EXPECT_FALSE(graph.EstimatePoseAtSource(1, 7040000000));
  const auto current = graph.EstimatePoseAtSource(1, 7000000000);
  ASSERT_TRUE(current);
  EXPECT_NEAR(current->theta(), M_PI, 1e-10);
}

TEST(SourceEpochQuery, FutureRetiredAndReanchoredHistoryFailClosed)
{
  auto p = Params();
  p.max_graph_nodes = 3;
  GraphManager graph(p);
  Seed(graph);
  for (int i = 1; i <= 4; ++i)
  {
    Consume(graph, i, (i - 1) * 100000000LL, i * 100000000LL);
    ASSERT_TRUE(graph.TickSource(i * 100000000LL, i * 100000000LL, 1));
  }
  graph.RebaseISAM2();
  EXPECT_FALSE(graph.EstimatePoseAtSource(1, 150000000));
  ASSERT_TRUE(graph.EstimatePoseAtSource(1, 250000000));
  Consume(graph, 5, 400000000, 450000000);
  // Consumed support alone is insufficient: this point is not committed.
  EXPECT_FALSE(graph.EstimatePoseAtSource(1, 425000000));
  graph.RigidTransformAll(gtsam::Pose2(1.0, 2.0, 0.2));
  EXPECT_FALSE(graph.EstimatePoseAtSource(1, 350000000));
  const auto boundary = graph.EstimatePoseAtSource(1, 400000000);
  ASSERT_TRUE(boundary);
  EXPECT_NEAR(boundary->theta(), 0.2, 1e-10);
  graph.ResetConsumedMotionEpoch(2);
  EXPECT_FALSE(graph.EstimatePoseAtSource(1, 400000000));
  EXPECT_FALSE(graph.EstimatePoseAtSource(2, 400000000));
}

TEST(SourceEpochQuery, ConditionalMeanUsesOptimizedAbsoluteGyroBias)
{
  auto p = Params();
  p.use_imu_preint = true;
  p.prior_sigma_theta = 1e-6;
  p.gyro_noise_density_rad_per_s = 1e-4;
  p.gyro_bias_prior_sigma_rad_per_s = .5;
  p.gyro_bias_rw_rad_per_s = 1e-6;
  GraphManager graph(p);
  Seed(graph);
  Consume(graph, 1, 0, 1000000000, 0.15, .12, .10);
  ASSERT_TRUE(graph.QueueYawAtEpoch(.1, 1e-6, false, 1000000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(1000000000, 1000000000, 1));
  Consume(graph, 2, 1000000000, 2000000000, 0.15, .12, .10);
  ASSERT_TRUE(graph.TickSource(2000000000, 2000000000, 1));
  EXPECT_NEAR(graph.LatestSnapshot()->pose.theta(), .2, 1e-6);
  const auto before = GraphManagerSourceTestAccess::Snapshot(graph);
  EXPECT_NEAR(before.second.at<double>(gtsam::Symbol('b', 2)), .02, 1e-6);
  const auto midpoint = graph.EstimatePoseAtSource(1, 1500000000);
  ASSERT_TRUE(midpoint);
  EXPECT_NEAR(midpoint->theta(), .15, 1e-6);
  EXPECT_EQ(graph.LiveNodeCount(), 3U);
  EXPECT_EQ(graph.SourceYawFactorCount(), 1U);
  const auto after = GraphManagerSourceTestAccess::Snapshot(graph);
  EXPECT_EQ(before.first.size(), after.first.size());
  EXPECT_EQ(before.second.size(), after.second.size());
}
}  // namespace fusion_graph
