// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/motion_source_history.hpp"
#include <gtest/gtest.h>
#include <gtsam/nonlinear/Marginals.h>

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
constexpr int64_t kSecond = 1000000000;
GraphParams Params()
{
  GraphParams p;
  p.node_period_s = 0.1;
  p.stationary_node_period_s = 0.1;
  p.gyro_bias_estimation_enabled = false;
  p.adaptive_noise_enabled_gain = 0.0;
  p.lever_arm_x = 0.0;
  p.lever_arm_y = 0.0;
  p.cov_update_every_n = 1;
  p.slip_residual_thresh_rad = 100.0;
  return p;
}
ConsumedMotionStep Step(uint64_t id, int64_t start, int64_t end, double vx = 0.15, double wz = 0.0)
{
  return {1,
          start,
          end,
          {{MotionSourceKind::Wheel, id, end}, {start, end}, vx, 0.0, wz},
          {{MotionSourceKind::Gyro, id, end}, {start, end}, wz},
          0.0,
          true,
          true};
}
void Consume(
    GraphManager& graph, uint64_t id, int64_t start, int64_t end, double vx = 0.15, double wz = 0.0)
{
  const double dt = static_cast<double>(end - start) * 1e-9;
  graph.AddWheelTwist(vx, 0.0, wz, dt);
  graph.AddGyroDelta(wz, dt);
  ASSERT_TRUE(graph.AddConsumedMotionStep(Step(id, start, end, vx, wz)));
}
void Seed(GraphManager& graph)
{
  graph.SetConsumedMotionBaseline(1, 0);
  ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(), 0, 0, 1));
}
}  // namespace

TEST(SourceEpoch, DoesNotInventStateWhenOnlyScheduleAdvances)
{
  GraphManager graph(Params());
  Seed(graph);
  EXPECT_FALSE(graph.TickSource(kSecond, 0, 1));
  EXPECT_EQ(graph.Stats().total_nodes, 1U);
  EXPECT_EQ(graph.LatestSnapshot()->source_stamp_ns, 0);
  Consume(graph, 1, 0, 100000000);
  const auto output = graph.TickSource(kSecond, 100000000, 1);
  ASSERT_TRUE(output);
  EXPECT_EQ(output->source_stamp_ns, 100000000);
  EXPECT_NEAR(output->pose.x(), 0.015, 1e-10);
}

TEST(SourceEpoch, IntegerCadenceDoesNotSkipExactBoundary)
{
  auto p = Params();
  p.node_period_s = 0.02;
  p.stationary_node_period_s = 0.02;
  GraphManager graph(p);
  Seed(graph);
  for (int i = 1; i <= 5; ++i)
  {
    Consume(graph, i, (i - 1) * 20000000LL, i * 20000000LL);
    ASSERT_TRUE(graph.TickSource(i * 20000000LL, i * 20000000LL, 1));
  }
  EXPECT_EQ(graph.Stats().total_nodes, 6U);
}

TEST(SourceEpoch, InteriorGnssRemovesFloorArtifactAndAssimilatesEveryFix)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  EXPECT_TRUE(graph.QueueGnssAtEpoch(0.0225, 0.0, 0.003, false, 150000000, 1, 1));
  EXPECT_FALSE(graph.TickSource(200000000, 100000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 0U);
  Consume(graph, 2, 100000000, 200000000);
  ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 1U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 0.03, 1e-9);
  EXPECT_FALSE(graph.QueueGnssAtEpoch(99.0, 0.0, 0.003, false, 150000000, 1, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 1U);
}

TEST(SourceEpoch, DelayedGnssChangesRevisionWithoutCreatingCadenceNode)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  const auto first = graph.TickSource(100000000, 100000000, 1);
  ASSERT_TRUE(first);
  EXPECT_TRUE(graph.QueueGnssAtEpoch(0.009, 0.0, 0.003, false, 50000000, 1, 1));
  const auto output = graph.TickSource(100000001, 100000000, 1);
  ASSERT_TRUE(output);
  EXPECT_GT(output->revision, first->revision);
  EXPECT_EQ(output->node_index, first->node_index);
  EXPECT_EQ(output->source_stamp_ns, first->source_stamp_ns);
  EXPECT_EQ(graph.SourceGnssFactorCount(), 1U);
}

TEST(SourceEpoch, BootstrapDiscardsMotionBeforeSeedAndReplaysOnlySupportedSuffix)
{
  GraphManager graph(Params());
  graph.SetConsumedMotionBaseline(1, 0);
  Consume(graph, 1, 0, 100000000);
  Consume(graph, 2, 100000000, 200000000);
  ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(1.0, 2.0, 0.0), 200000000, 50000000, 1));
  ASSERT_TRUE(graph.TickSource(300000000, 200000000, 1));
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 1.0225, 1e-10);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.y(), 2.0, 1e-10);
}

TEST(SourceEpoch, GraphAndAcquisitionUseSameTurningExpmap)
{
  auto p = Params();
  p.pivot_gate_dtheta_rad = 100.0;
  GraphManager graph(p);
  Seed(graph);
  Consume(graph, 1, 0, 100000000, 0.15, 0.5);
  Consume(graph, 2, 100000000, 200000000, 0.15, 0.5);
  ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
  const auto expected = gtsam::Pose2::Expmap(gtsam::Vector3(0.03, 0.0, 0.1));
  EXPECT_LT(expected.localCoordinates(graph.LatestSnapshot()->pose).norm(), 1e-10);
  const auto half = gtsam::Pose2::Expmap(gtsam::Vector3(0.015, 0.0, 0.05));
  ASSERT_TRUE(graph.QueueGnssAtEpoch(half.x(), half.y(), 0.003, false, 100000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(200000001, 200000000, 1));
  EXPECT_LT(expected.localCoordinates(graph.LatestSnapshot()->pose).norm(), 1e-9);
}

TEST(SourceEpoch, MissingIntervalAndEpochResetRequireAbsoluteSeed)
{
  GraphManager graph(Params());
  Seed(graph);
  EXPECT_FALSE(graph.AddConsumedMotionStep(Step(1, 100000000, 200000000)));
  graph.ResetConsumedMotionEpoch(2);
  EXPECT_FALSE(graph.IsInitialized());
  EXPECT_FALSE(graph.LatestSnapshot());
  graph.SetConsumedMotionBaseline(2, 200000000);
  EXPECT_FALSE(graph.TickSource(300000000, 200000000, 2));
  EXPECT_FALSE(graph.QueueGnssAtEpoch(0, 0, 0.003, false, 200000000, 1, 1));
}

TEST(SourceEpoch, WheelYawPayloadIsImmutableUnderIdentity)
{
  MotionSourceHistory history(5);
  history.Reset(1);
  const auto first = Step(1, 0, 100000000);
  ASSERT_EQ(history.Append(first), MotionHistoryStatus::Ok);
  auto changed = Step(2, 100000000, 200000000);
  changed.wheel = first.wheel;
  changed.wheel.wz = 0.3;
  EXPECT_EQ(history.Append(changed), MotionHistoryStatus::SourceConflict);
}

TEST(SourceEpoch, PoseOnlyRestoredGraphRequiresCurrentAbsoluteSeed)
{
  GraphManager graph(Params());
  // The legacy pose-only API has exactly the same absent source provenance
  // as Load. Its legacy Tick behavior is tested separately.
  graph.Initialize(gtsam::Pose2(9.0, 8.0, 0.0), 100.0);
  graph.SetConsumedMotionBaseline(1, 10 * kSecond);
  EXPECT_FALSE(graph.IsInitialized());
  EXPECT_FALSE(graph.LatestSnapshot());
  ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(1.0, 2.0, 0.0), 10 * kSecond, 10 * kSecond, 1));
  EXPECT_DOUBLE_EQ(graph.LatestSnapshot()->pose.x(), 1.0);
}

TEST(SourceEpoch, DelayedOrderAndReceiptSchedulePreservePerfectTrajectory)
{
  for (bool reverse : {false, true})
    for (int64_t receipt : {300000000LL, 900000000LL})
    {
      GraphManager graph(Params());
      Seed(graph);
      Consume(graph, 1, 0, 100000000);
      ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
      Consume(graph, 2, 100000000, 200000000);
      ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
      const int64_t first = reverse ? 175000000 : 125000000;
      const int64_t second = reverse ? 125000000 : 175000000;
      ASSERT_TRUE(graph.QueueGnssAtEpoch(first * 1e-9 * 0.15, 0.0, 0.003, true, first, 1, 1));
      ASSERT_TRUE(graph.QueueGnssAtEpoch(second * 1e-9 * 0.15, 0.0, 0.003, true, second, 2, 1));
      ASSERT_TRUE(graph.TickSource(receipt, 200000000, 1));
      EXPECT_EQ(graph.SourceGnssFactorCount(), 2U);
      EXPECT_EQ(graph.Stats().total_nodes, 3U);
      EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 0.03, 1e-9);
    }
}

TEST(SourceEpoch, RetiringWholeAcquisitionEdgeBoundsStatesAndPreservesPosterior)
{
  auto p = Params();
  p.max_graph_nodes = 3;
  GraphManager reference(Params()), windowed(p);
  for (auto* graph : {&reference, &windowed})
  {
    Seed(*graph);
    for (int i = 1; i <= 4; ++i)
    {
      Consume(*graph, i, (i - 1) * 100000000LL, i * 100000000LL);
      ASSERT_TRUE(graph->TickSource(i * 100000000LL, i * 100000000LL, 1));
      const int64_t t = i * 100000000LL - 50000000;
      ASSERT_TRUE(graph->QueueGnssAtEpoch(t * 1e-9 * 0.15 + 0.001, 0.0, 0.003, false, t, i, 1));
      ASSERT_TRUE(graph->TickSource(i * 100000000LL + 1, i * 100000000LL, 1));
    }
  }
  EXPECT_TRUE(windowed.NeedsSourceRebase());
  const auto before = GraphManagerSourceTestAccess::Snapshot(windowed);
  windowed.RebaseISAM2();
  const auto after = GraphManagerSourceTestAccess::Snapshot(windowed);
  const auto before_cov =
      gtsam::Marginals(before.first, before.second).marginalCovariance(PoseKey(4));
  const auto after_cov =
      gtsam::Marginals(after.first, before.second).marginalCovariance(PoseKey(4));
  EXPECT_LT((before_cov - after_cov).norm(), 1e-8);
  EXPECT_LE(windowed.LiveNodeCount(), 3U);
  EXPECT_FALSE(windowed.NeedsSourceRebase());
  EXPECT_GE(*windowed.OldestRetainedSourceNs(1), 300000000);
  EXPECT_FALSE(windowed.QueueGnssAtEpoch(0.0, 0.0, 0.003, false, 50000000, 99, 1));
  EXPECT_LT(reference.GetPose(4)->localCoordinates(*windowed.GetPose(4)).norm(), 1e-8);
  ASSERT_TRUE(windowed.QueueGnssAtEpoch(0.0535, 0.0, 0.003, false, 350000000, 100, 1));
  ASSERT_TRUE(windowed.TickSource(500000000, 400000000, 1));
  EXPECT_EQ(windowed.SourceGnssFactorCount(), 5U);
  ASSERT_TRUE(reference.QueueGnssAtEpoch(0.0535, 0.0, 0.003, false, 350000000, 100, 1));
  ASSERT_TRUE(reference.TickSource(500000000, 400000000, 1));
  EXPECT_LT(reference.GetPose(4)->localCoordinates(*windowed.GetPose(4)).norm(), 1e-8);
  const auto full_now = GraphManagerSourceTestAccess::Snapshot(reference);
  const auto window_now = GraphManagerSourceTestAccess::Snapshot(windowed);
  const auto full_cov =
      gtsam::Marginals(full_now.first, full_now.second).marginalCovariance(PoseKey(4));
  const auto window_cov =
      gtsam::Marginals(window_now.first, full_now.second).marginalCovariance(PoseKey(4));
  // Compare the two likelihoods at ONE linearization. iSAM2's cached
  // marginal retains old nonlinear Jacobians below its relinearization
  // threshold; comparing that cache with a freshly rebuilt solver is not
  // an information-preservation oracle.
  EXPECT_LT((full_cov - window_cov).norm(), 1e-8);
}

TEST(SourceEpoch, PreintegratedGyroRetainsBiasAndAcquisitionSupport)
{
  auto p = Params();
  p.use_imu_preint = true;
  GraphManager graph(p);
  Seed(graph);
  Consume(graph, 1, 0, 100000000, 0.15, 0.5);
  Consume(graph, 2, 100000000, 200000000, 0.15, 0.5);
  ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
  const auto half = gtsam::Pose2::Expmap(gtsam::Vector3(0.015, 0.0, 0.05));
  ASSERT_TRUE(graph.QueueGnssAtEpoch(half.x(), half.y(), 0.003, false, 100000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(200000001, 200000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 1U);
  EXPECT_TRUE(graph.IsInitialized());
}

TEST(SourceEpoch, IntegerCadenceRemainsExactAtWallClockMagnitude)
{
  auto p = Params();
  p.node_period_s = 0.02;
  p.stationary_node_period_s = 0.02;
  GraphManager graph(p);
  const int64_t origin = 1793200000000000123LL;
  graph.SetConsumedMotionBaseline(1, origin);
  ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(), origin, origin, 1));
  for (int i = 1; i <= 10; ++i)
  {
    Consume(graph, i, origin + (i - 1) * 20000000LL, origin + i * 20000000LL);
    ASSERT_TRUE(graph.TickSource(origin + i * 20000000LL, origin + i * 20000000LL, 1));
  }
  EXPECT_EQ(graph.Stats().total_nodes, 11U);
}

TEST(SourceEpoch, DistinctObservationsAtOneEpochShareAnAcquisitionPose)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  for (uint64_t id : {1U, 2U})
    ASSERT_TRUE(graph.QueueGnssAtEpoch(0.0075, 0.0, 0.003, false, 50000000, id, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 2U);
  EXPECT_EQ(graph.LiveNodeCount(), 3U);  // x0,x1 and one shared acquisition.
}

TEST(SourceEpoch, SlipAndStationaryMeanPoliciesAreFrozenForInteriorStates)
{
  auto p = Params();
  p.slip_window_s = 0.0;
  p.slip_residual_thresh_rad = 0.001;
  p.slip_gyro_max_rad = 0.001;
  p.slip_wheel_min_rad = 0.001;
  GraphManager graph(p);
  Seed(graph);
  graph.AddWheelTwist(0.15, 0.0, 0.5, 0.1);
  graph.AddGyroDelta(0.0, 0.1);
  auto step = Step(1, 0, 100000000);
  step.wheel.wz = 0.5;
  ASSERT_TRUE(graph.AddConsumedMotionStep(step));
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  EXPECT_EQ(graph.Stats().slip_veto, 1U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 0.0, 1e-12);
  ASSERT_TRUE(graph.QueueGnssAtEpoch(0.0, 0.0, 0.003, false, 50000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 0.0, 1e-12);

  p = Params();
  p.lever_arm_x = 0.2;
  GraphManager parked(p);
  Seed(parked);
  Consume(parked, 1, 0, 100000000, 0.0, 0.02);
  ASSERT_TRUE(parked.TickSource(100000000, 100000000, 1));
  EXPECT_NEAR(parked.LatestSnapshot()->pose.theta(), 0.0, 1e-12);
  ASSERT_TRUE(parked.QueueGnssAtEpoch(0.2, 0.0, 0.003, false, 50000000, 1, 1));
  ASSERT_TRUE(parked.TickSource(100000001, 100000000, 1));
  EXPECT_NEAR(parked.LatestSnapshot()->pose.theta(), 0.0, 1e-12);
}

TEST(SourceEpoch, PreintegrationRejectsUnsupportedRepeatedGyroSource)
{
  auto p = Params();
  p.use_imu_preint = true;
  GraphManager graph(p);
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  auto reused = Step(2, 100000000, 200000000);
  reused.gyro = Step(1, 0, 100000000).gyro;
  EXPECT_FALSE(graph.AddConsumedMotionStep(reused));
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  EXPECT_FALSE(graph.AddConsumedMotionStep(reused));
  GraphManager nonconsecutive(p);
  Seed(nonconsecutive);
  Consume(nonconsecutive, 1, 0, 100000000);
  Consume(nonconsecutive, 2, 100000000, 200000000);
  auto older = Step(3, 200000000, 300000000);
  older.gyro = Step(1, 0, 100000000).gyro;
  EXPECT_FALSE(nonconsecutive.AddConsumedMotionStep(older));
}

TEST(SourceEpoch, LiveReanchorKeepsQueuedFutureObservationAndBoundaryDedup)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  ASSERT_TRUE(graph.QueueGnssAtEpoch(0.015, 0.0, 0.003, false, 100000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  ASSERT_EQ(graph.SourceGnssFactorCount(), 1U);
  ASSERT_TRUE(graph.QueueGnssAtEpoch(1.0225, 0.0, 0.003, false, 150000000, 2, 1));
  graph.RigidTransformAll(gtsam::Pose2(1.0, 0.0, 0.0));
  EXPECT_FALSE(graph.QueueGnssAtEpoch(0.015, 0.0, 0.003, false, 100000000, 1, 1));
  EXPECT_FALSE(graph.QueueGnssAtEpoch(1.0225, 0.0, 0.003, false, 150000000, 2, 1));
  Consume(graph, 2, 100000000, 200000000);
  ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 2U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 1.03, 1e-8);
}

TEST(SourceEpoch, DelayedHeadingSourceTimeEliminatesLegacyCurrentNodeBias)
{
  constexpr double sigma = 0.18232899787743528;
  std::vector<TickOutput> outputs;
  for (int mode : {0, 1, 2, 3})
  {
    GraphParams p;
    p.node_period_s = 0.04;
    GraphManager graph(p);
    Seed(graph);
    for (int i = 1; i <= 30; ++i)
    {
      Consume(graph, i, (i - 1) * 40000000LL, i * 40000000LL, 0.15, 0.2);
      if ((mode == 1 && i == 20) || (mode == 2 && i == 30))
      {
        ASSERT_TRUE(graph.QueueYawAtEpoch(0.16, sigma, false, 800000000, 1, 1));
      }
      if (mode == 3 && i == 30)
        graph.QueueYaw(0.16, sigma, false);
      graph.TickSource(i * 40000000LL, i * 40000000LL, 1);
    }
    EXPECT_EQ(graph.SourceYawFactorCount(), mode == 1 || mode == 2 ? 1U : 0U);
    EXPECT_EQ(graph.SourceGnssFactorCount(), 0U);
    outputs.push_back(*graph.LatestSnapshot());
  }
  for (size_t i : {0U, 1U, 2U})
    EXPECT_NEAR(outputs[i].pose.theta(), 0.24, 1e-10);
  EXPECT_LT(outputs[1].pose.localCoordinates(outputs[2].pose).norm(), 1e-10);
  // Retained negative control: identical old QueueYaw attaches to the head.
  EXPECT_GT(std::abs(outputs[3].pose.theta() - 0.24), 0.006);
}

TEST(SourceEpoch, GnssAndTwoHeadingProducersShareEpochButHaveDistinctIdentity)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  ASSERT_TRUE(graph.QueueGnssAtEpoch(0.0075, 0.0, 0.003, false, 50000000, 14, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.0, 0.15, false, 50000000, 14, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.0, 0.10, true, 50000000, 15, 1));
  EXPECT_FALSE(graph.QueueYawAtEpoch(1.0, 0.15, false, 50000000, 14, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  EXPECT_EQ(graph.SourceGnssFactorCount(), 1U);
  EXPECT_EQ(graph.SourceYawFactorCount(), 2U);
  EXPECT_EQ(graph.LiveNodeCount(), 3U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.x(), 0.015, 1e-10);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.theta(), 0.0, 1e-10);
}

TEST(SourceEpoch, FutureHeadingWaitsForBracketAndDelayedOrderPreservesMean)
{
  std::vector<TickOutput> outputs;
  for (bool reverse : {false, true})
  {
    GraphManager graph(Params());
    Seed(graph);
    const int64_t first = reverse ? 150000000 : 50000000;
    const int64_t second = reverse ? 50000000 : 150000000;
    ASSERT_TRUE(graph.QueueYawAtEpoch(0.5 * first * 1e-9, 0.15, false, first, 1, 1));
    ASSERT_TRUE(graph.QueueYawAtEpoch(0.5 * second * 1e-9, 0.10, true, second, 2, 1));
    EXPECT_FALSE(graph.TickSource(100000000, 0, 1));
    EXPECT_EQ(graph.SourceYawFactorCount(), 0U);
    Consume(graph, 1, 0, 100000000, 0.15, 0.5);
    ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
    EXPECT_EQ(graph.SourceYawFactorCount(), 1U);
    Consume(graph, 2, 100000000, 200000000, 0.15, 0.5);
    ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
    EXPECT_EQ(graph.SourceYawFactorCount(), 2U);
    EXPECT_EQ(graph.Stats().total_nodes, 3U);
    outputs.push_back(*graph.LatestSnapshot());
  }
  EXPECT_NEAR(outputs[0].pose.theta(), 0.1, 1e-10);
  EXPECT_LT(outputs[0].pose.localCoordinates(outputs[1].pose).norm(), 1e-10);
}

TEST(SourceEpoch, SourceHeadingPreservesHuberDownweighting)
{
  std::vector<double> yaw;
  for (bool robust : {false, true})
  {
    GraphManager graph(Params());
    Seed(graph);
    Consume(graph, 1, 0, 100000000);
    ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
    ASSERT_TRUE(graph.QueueYawAtEpoch(0.6, 0.1, robust, 50000000, 1, 1));
    ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
    EXPECT_EQ(graph.SourceYawFactorCount(), 1U);
    yaw.push_back(std::abs(graph.LatestSnapshot()->pose.theta()));
  }
  EXPECT_GT(yaw[0], 1e-3);
  EXPECT_LT(yaw[1], yaw[0]);
}

TEST(SourceEpoch, HeadingWindowRetiresOldEpochWithoutRetargeting)
{
  auto p = Params();
  p.max_graph_nodes = 3;
  GraphManager graph(p);
  Seed(graph);
  for (int i = 1; i <= 4; ++i)
  {
    Consume(graph, i, (i - 1) * 100000000LL, i * 100000000LL);
    ASSERT_TRUE(graph.TickSource(i * 100000000LL, i * 100000000LL, 1));
    ASSERT_TRUE(graph.QueueYawAtEpoch(0.0, 0.15, false, i * 100000000LL - 50000000, i, 1));
    ASSERT_TRUE(graph.TickSource(i * 100000000LL + 1, i * 100000000LL, 1));
  }
  graph.RebaseISAM2();
  EXPECT_LE(graph.LiveNodeCount(), 3U);
  EXPECT_FALSE(graph.QueueYawAtEpoch(0.0, 0.15, false, 50000000, 99, 1));
  EXPECT_FALSE(graph.QueueYawAtEpoch(0.0, 0.15, false, 350000000, 4, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.0, 0.15, false, 350000000, 99, 1));
  ASSERT_TRUE(graph.TickSource(500000000, 400000000, 1));
  EXPECT_EQ(graph.SourceYawFactorCount(), 5U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.theta(), 0.0, 1e-10);
}

TEST(SourceEpoch, HeadingReanchorPreservesFutureAndBoundaryDedupThenResetClearsEpoch)
{
  GraphManager graph(Params());
  Seed(graph);
  Consume(graph, 1, 0, 100000000);
  ASSERT_TRUE(graph.TickSource(100000000, 100000000, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.0, 0.15, false, 100000000, 1, 1));
  ASSERT_TRUE(graph.TickSource(100000001, 100000000, 1));
  ASSERT_TRUE(graph.QueueYawAtEpoch(0.2, 0.15, false, 150000000, 2, 1));
  graph.RigidTransformAll(gtsam::Pose2(1.0, 0.0, 0.2));
  EXPECT_FALSE(graph.QueueYawAtEpoch(0.0, 0.15, false, 100000000, 1, 1));
  EXPECT_FALSE(graph.QueueYawAtEpoch(0.2, 0.15, false, 150000000, 2, 1));
  Consume(graph, 2, 100000000, 200000000);
  ASSERT_TRUE(graph.TickSource(200000000, 200000000, 1));
  EXPECT_EQ(graph.SourceYawFactorCount(), 2U);
  EXPECT_NEAR(graph.LatestSnapshot()->pose.theta(), 0.2, 1e-10);
  graph.ResetConsumedMotionEpoch(2);
  graph.SetConsumedMotionBaseline(2, 200000000);
  EXPECT_FALSE(graph.QueueYawAtEpoch(0.2, 0.15, false, 200000000, 3, 1));
  EXPECT_TRUE(graph.QueueYawAtEpoch(0.2, 0.15, false, 200000000, 3, 2));
  EXPECT_EQ(graph.SourceYawFactorCount(), 0U);
}
}  // namespace fusion_graph
