// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <condition_variable>
#include <future>
#include <stdexcept>

#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/motion_source_history.hpp"
#include <gtest/gtest.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
class GraphManagerRebaseTestAccess
{
public:
  static std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> Snapshot(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    return {manager.isam_.getFactorsUnsafe(), manager.isam_.calculateEstimate()};
  }

  static std::pair<gtsam::Pose2, Eigen::Matrix3d> MarginalSnapshot(GraphManager& manager,
                                                                   uint64_t node_index)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    return {manager.PoseAt(node_index), manager.isam_.marginalCovariance(PoseKey(node_index))};
  }

  static bool AddFactor(GraphManager& manager, const gtsam::NonlinearFactor::shared_ptr& factor)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(factor);
    const bool result = manager.ApplyIsamUpdateLocked(graph, gtsam::Values());
    manager.estimate_dirty_ = true;
    return result;
  }

  static bool HasPendingWorker(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    return manager.rebase_in_progress_ && !manager.rebase_pending_factors_.empty();
  }
};
}  // namespace fusion_graph

namespace
{
namespace fg = fusion_graph;

// A one-shot gate inside a real factor's linearization gives deterministic
// phase-2 interleavings without timing sleeps or a production testing hook.
class RebuildGate
{
public:
  void Arm()
  {
    std::lock_guard<std::mutex> lock(mu_);
    armed_ = true;
  }

  void Pause()
  {
    std::unique_lock<std::mutex> lock(mu_);
    if (!armed_)
      return;
    armed_ = false;
    entered_ = true;
    cv_.notify_all();
    if (!cv_.wait_for(lock,
                      std::chrono::seconds(5),
                      [&]
                      {
                        return released_;
                      }))
      throw std::runtime_error("rebase test gate timed out");
    if (fail_)
      throw std::runtime_error("injected rebase rebuild failure");
  }

  bool Wait()
  {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock,
                        std::chrono::seconds(5),
                        [&]
                        {
                          return entered_;
                        });
  }

  void Release(bool fail = false)
  {
    std::lock_guard<std::mutex> lock(mu_);
    fail_ = fail;
    released_ = true;
    cv_.notify_all();
  }

private:
  std::mutex mu_;
  std::condition_variable cv_;
  bool armed_ = false, entered_ = false, released_ = false, fail_ = false;
};

class GatedPrior : public gtsam::PriorFactor<gtsam::Pose2>
{
public:
  GatedPrior(gtsam::Key key, const gtsam::Pose2& pose, std::shared_ptr<RebuildGate> gate)
      : gtsam::PriorFactor<gtsam::Pose2>(key, pose, gtsam::noiseModel::Isotropic::Sigma(3, 0.2)),
        gate_(std::move(gate))
  {
  }

  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override
  {
    gate_->Pause();
    return gtsam::PriorFactor<gtsam::Pose2>::linearize(values);
  }

private:
  std::shared_ptr<RebuildGate> gate_;
};

std::shared_ptr<RebuildGate> GateRebuild(fg::GraphManager& manager, uint64_t index)
{
  auto gate = std::make_shared<RebuildGate>();
  const auto pose = manager.GetPose(index);
  EXPECT_TRUE(pose);
  EXPECT_TRUE(fg::GraphManagerRebaseTestAccess::AddFactor(
      manager, std::make_shared<GatedPrior>(fg::PoseKey(index), pose.value(), gate)));
  gate->Arm();
  return gate;
}

fg::GraphParams Params(uint64_t window, bool bias = false)
{
  fg::GraphParams params;
  params.node_period_s = 0.05;
  params.max_graph_nodes = window;
  params.cov_update_every_n = 1;
  params.adaptive_noise_enabled_gain = 0.0;
  params.stationary_node_period_s = 0.0;
  params.use_imu_preint = bias;
  params.lever_arm_x = 0.0;
  params.lever_arm_y = 0.0;
  return params;
}

void Drive(fg::GraphManager& manager, int first, int last)
{
  for (int i = first; i <= last; ++i)
  {
    manager.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    manager.AddGyroDelta(0.0, 0.1);
    ASSERT_TRUE(manager.Tick(i * 0.1));
  }
}

void DriveSource(fg::GraphManager& manager, int last)
{
  manager.SetConsumedMotionBaseline(1, 0);
  ASSERT_TRUE(manager.InitializeSource(gtsam::Pose2(), 0, 0, 1));
  for (int i = 1; i <= last; ++i)
  {
    const int64_t start_ns = (i - 1) * 100000000LL;
    const int64_t end_ns = i * 100000000LL;
    const double dt = static_cast<double>(end_ns - start_ns) * 1e-9;
    constexpr double vx = 0.5;
    manager.AddWheelTwist(vx, 0.0, 0.0, dt);
    manager.AddGyroDelta(0.0, dt);
    const fg::ConsumedMotionStep step{
        1,
        start_ns,
        end_ns,
        {{fg::MotionSourceKind::Wheel, static_cast<uint64_t>(i), end_ns},
         {start_ns, end_ns},
         vx,
         0.0,
         0.0},
        {{fg::MotionSourceKind::Gyro, static_cast<uint64_t>(i), end_ns}, {start_ns, end_ns}, 0.0},
        0.0,
        true,
        true};
    ASSERT_TRUE(manager.AddConsumedMotionStep(step));
    ASSERT_TRUE(manager.TickSource(end_ns, end_ns, 1));
  }
}

// Compare the complete retained joint information at ONE linearization point.
// Covariance of each pose alone would miss discarded cross-correlations, and
// comparing optimizer caches would miss a rebase that never changed latest_.
void ExpectInformationPreserved(fg::GraphManager& manager)
{
  const auto [before_graph, before_values] = fg::GraphManagerRebaseTestAccess::Snapshot(manager);
  manager.RebaseISAM2();
  const auto [after_graph, after_values] = fg::GraphManagerRebaseTestAccess::Snapshot(manager);
  gtsam::Ordering removed, retained;
  for (const auto& value : before_values)
  {
    (after_values.exists(value.key) ? retained : removed).push_back(value.key);
  }
  auto reference = before_graph.linearize(before_values);
  if (!removed.empty())
    reference = reference->eliminatePartialSequential(removed).second;
  const auto expected = reference->hessian(retained);
  const auto actual = after_graph.linearize(before_values)->hessian(retained);
  ASSERT_EQ(actual.first.rows(), expected.first.rows());
  EXPECT_LT((actual.first - expected.first).norm() / expected.first.norm(), 1e-10);
  EXPECT_LT((actual.second - expected.second).norm(), 1e-7);
  const gtsam::Marginals before_marginals(before_graph, before_values);
  const gtsam::Marginals after_marginals(after_graph, before_values);
  const auto key = fg::PoseKey(manager.Stats().total_nodes - 1);
  const auto covariance = before_marginals.marginalCovariance(key);
  EXPECT_LT((after_marginals.marginalCovariance(key) - covariance).norm() / covariance.norm(),
            1e-8);
}

TEST(RebaseInformation, NoCutRetainsLikelihoodAndCrossCorrelations)
{
  fg::GraphManager manager(Params(100));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 12);
  ExpectInformationPreserved(manager);
}

TEST(RebaseInformation, CutPreservesJointInformationAndCapsWindow)
{
  fg::GraphManager manager(Params(8));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 12);
  ExpectInformationPreserved(manager);
  EXPECT_EQ(manager.LiveNodeCount(), 8U);
  EXPECT_FALSE(manager.GetPose(4));
  EXPECT_TRUE(manager.GetPose(5));
  Drive(manager, 13, 17);
  ExpectInformationPreserved(manager);
  EXPECT_EQ(manager.LiveNodeCount(), 8U);
}

TEST(RebaseInformation, HistoricalGnssStillCorrectsTheCurrentState)
{
  fg::GraphManager reference(Params(100)), rebased(Params(8));
  for (auto* manager : {&reference, &rebased})
  {
    manager->Initialize(gtsam::Pose2(), 0.0);
    Drive(*manager, 1, 12);
  }
  rebased.RebaseISAM2();
  for (auto* manager : {&reference, &rebased})
  {
    ASSERT_TRUE(manager->QueueGnss(0.31, 0.0, 0.01, false, 6));
    Drive(*manager, 13, 13);
  }
  const auto expected = reference.LatestSnapshot();
  const auto actual = rebased.LatestSnapshot();
  ASSERT_TRUE(expected);
  ASSERT_TRUE(actual);
  EXPECT_GT(expected->pose.x(), 0.65 + 0.005);
  EXPECT_NEAR(actual->pose.x(), expected->pose.x(), 1e-8);
  EXPECT_LT((actual->covariance - expected->covariance).norm(), 1e-8);
}

TEST(RebaseInformation, ForceAnchorLatestRefreshesSnapshotMarginals)
{
  fg::GraphManager manager(Params(100));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 8);

  const auto before = manager.LatestSnapshot();
  ASSERT_TRUE(before);
  const auto before_marginal =
      fg::GraphManagerRebaseTestAccess::MarginalSnapshot(manager, before->node_index);
  manager.ForceAnchor(before->node_index, gtsam::Pose2(0.9, -0.4, 0.8), 0.01, 0.01);

  const auto after = manager.LatestSnapshot();
  ASSERT_TRUE(after);
  const auto actual =
      fg::GraphManagerRebaseTestAccess::MarginalSnapshot(manager, after->node_index);
  EXPECT_NEAR(after->pose.x(), actual.first.x(), 1e-10);
  EXPECT_NEAR(after->pose.y(), actual.first.y(), 1e-10);
  EXPECT_NEAR(after->pose.theta(), actual.first.theta(), 1e-10);
  EXPECT_LT((after->covariance - actual.second).norm(), 1e-10);
  EXPECT_GT((actual.first.translation() - before_marginal.first.translation()).norm(), 1e-4);
  EXPECT_GT((actual.second - before_marginal.second).norm(), 1e-6);
  EXPECT_EQ(after->node_index, before->node_index);
  EXPECT_EQ(after->timestamp, before->timestamp);
  EXPECT_EQ(after->revision, before->revision + 1);
  EXPECT_EQ(manager.Stats().total_nodes, before->node_index + 1);
}

TEST(RebaseInformation, ForceAnchorOlderNodeRefreshesLatestSnapshot)
{
  fg::GraphManager manager(Params(100));
  DriveSource(manager, 8);

  const auto before = manager.LatestSnapshot();
  ASSERT_TRUE(before);
  const uint64_t anchor_index = before->node_index - 3;
  const auto before_marginal =
      fg::GraphManagerRebaseTestAccess::MarginalSnapshot(manager, before->node_index);
  const auto anchor_pose = manager.GetPose(anchor_index);
  ASSERT_TRUE(anchor_pose);
  manager.ForceAnchor(anchor_index, gtsam::Pose2(0.2, -0.1, 1.1), 0.01, 0.01);

  const auto after = manager.LatestSnapshot();
  ASSERT_TRUE(after);
  const auto actual =
      fg::GraphManagerRebaseTestAccess::MarginalSnapshot(manager, after->node_index);
  EXPECT_NEAR(after->pose.x(), actual.first.x(), 1e-10);
  EXPECT_NEAR(after->pose.y(), actual.first.y(), 1e-10);
  EXPECT_NEAR(after->pose.theta(), actual.first.theta(), 1e-10);
  EXPECT_LT((after->covariance - actual.second).norm(), 1e-10);
  EXPECT_GT((actual.first.translation() - before_marginal.first.translation()).norm(), 1e-4);
  EXPECT_GT((actual.second - before_marginal.second).norm(), 1e-6);
  EXPECT_EQ(after->node_index, before->node_index);
  EXPECT_EQ(after->timestamp, before->timestamp);
  EXPECT_EQ(after->source_stamp_ns, before->source_stamp_ns);
  EXPECT_EQ(after->source_epoch, before->source_epoch);
  EXPECT_EQ(after->revision, before->revision + 1);
  EXPECT_EQ(manager.Stats().total_nodes, before->node_index + 1);

  EXPECT_FALSE(manager.TickSource(800000001LL, 800000000LL, 1));
  EXPECT_EQ(manager.Stats().total_nodes, before->node_index + 1);
  constexpr int64_t next_source_ns = 900000000LL;
  const double dt = 0.1;
  constexpr double vx = 0.5;
  manager.AddWheelTwist(vx, 0.0, 0.0, dt);
  manager.AddGyroDelta(0.0, dt);
  const fg::ConsumedMotionStep next_step{1,
                                         800000000LL,
                                         next_source_ns,
                                         {{fg::MotionSourceKind::Wheel, 9, next_source_ns},
                                          {800000000LL, next_source_ns},
                                          vx,
                                          0.0,
                                          0.0},
                                         {{fg::MotionSourceKind::Gyro, 9, next_source_ns},
                                          {800000000LL, next_source_ns},
                                          0.0},
                                         0.0,
                                         true,
                                         true};
  ASSERT_TRUE(manager.AddConsumedMotionStep(next_step));
  const auto next = manager.TickSource(next_source_ns, next_source_ns, 1);
  ASSERT_TRUE(next);
  EXPECT_EQ(next->node_index, before->node_index + 1);
  EXPECT_EQ(next->source_stamp_ns, next_source_ns);
  EXPECT_EQ(next->source_epoch, before->source_epoch);
  EXPECT_GT(next->revision, after->revision);
  EXPECT_EQ(manager.Stats().total_nodes, before->node_index + 2);
}

TEST(RebaseInformation, BiasVariablesKeepTheirTypesAndCorrelations)
{
  fg::GraphManager manager(Params(8, true));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 12);
  ASSERT_NO_THROW(ExpectInformationPreserved(manager));
  const auto [graph, values] = fg::GraphManagerRebaseTestAccess::Snapshot(manager);
  EXPECT_EQ(values.size(), 16U);
  EXPECT_TRUE(values.exists(gtsam::Symbol('b', 5)));
  EXPECT_NO_THROW(values.at<double>(gtsam::Symbol('b', 12)));
  Drive(manager, 13, 14);
  EXPECT_TRUE(manager.IsInitialized());
}

TEST(RebaseInformation, CurvedTrajectoryWithLeverArmPreservesLocalInformation)
{
  auto params = Params(8, true);
  params.lever_arm_x = 0.4;
  params.lever_arm_y = 0.1;
  fg::GraphManager manager(params);
  manager.Initialize(gtsam::Pose2(1.0, 2.0, 0.3), 0.0);
  for (int i = 1; i <= 12; ++i)
  {
    manager.AddWheelTwist(0.5, 0.0, 0.2, 0.1);
    manager.AddGyroDelta(0.21, 0.1);
    if (i == 6)
      manager.QueueGnss(1.55, 2.3, 0.1);
    ASSERT_TRUE(manager.Tick(i * 0.1));
  }
  ExpectInformationPreserved(manager);
}

TEST(RebaseInformation, PendingLiveUpdatesAreReplayed)
{
  fg::GraphManager manager(Params(8));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 12);
  auto gate = GateRebuild(manager, 12);
  auto worker = std::async(std::launch::async,
                           [&]
                           {
                             manager.RebaseISAM2();
                           });
  ASSERT_TRUE(gate->Wait());
  Drive(manager, 13, 13);
  EXPECT_TRUE(fg::GraphManagerRebaseTestAccess::HasPendingWorker(manager));
  gate->Release();
  ASSERT_EQ(worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_NO_THROW(worker.get());
  EXPECT_EQ(manager.LiveNodeCount(), 9U);  // eight snapshot states plus the pending tick
  EXPECT_FALSE(manager.GetPose(4));
  ASSERT_TRUE(manager.GetPose(13));
  EXPECT_NEAR(manager.GetPose(13)->x(), 0.65, 1e-8);
}

TEST(RebaseInformation, PendingObservationOnRemovedStateAbandonsRebase)
{
  fg::GraphManager manager(Params(8));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 12);
  auto gate = GateRebuild(manager, 12);
  auto worker = std::async(std::launch::async,
                           [&]
                           {
                             manager.RebaseISAM2();
                           });
  ASSERT_TRUE(gate->Wait());
  ASSERT_TRUE(manager.QueueGnss(0.06, 0.0, 0.01, false, 1));
  Drive(manager, 13, 13);
  const auto before = manager.GetPose(13);
  gate->Release();
  ASSERT_EQ(worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_NO_THROW(worker.get());
  EXPECT_EQ(manager.LiveNodeCount(), 14U);
  EXPECT_TRUE(manager.GetPose(1));
  ASSERT_TRUE(before);
  EXPECT_NEAR(manager.GetPose(13)->x(), before->x(), 1e-12);
  manager.RebaseISAM2();
  EXPECT_EQ(manager.LiveNodeCount(), 8U);
}

TEST(RebaseInformation, CancelledWorkerCannotCommitOrClearNewWorkersPendingUpdates)
{
  for (const bool fail_old_worker : {false, true})
  {
    SCOPED_TRACE(fail_old_worker);
    fg::GraphManager manager(Params(8));
    manager.Initialize(gtsam::Pose2(), 0.0);
    auto old_gate = GateRebuild(manager, 0);
    auto old_worker = std::async(std::launch::async,
                                 [&]
                                 {
                                   manager.RebaseISAM2();
                                 });
    ASSERT_TRUE(old_gate->Wait());
    manager.Reset();
    manager.Initialize(gtsam::Pose2(10.0, 0.0, 0.0), 0.0);
    auto new_gate = GateRebuild(manager, 0);
    auto new_worker = std::async(std::launch::async,
                                 [&]
                                 {
                                   manager.RebaseISAM2();
                                 });
    ASSERT_TRUE(new_gate->Wait());
    Drive(manager, 1, 1);
    old_gate->Release(fail_old_worker);
    ASSERT_EQ(old_worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_NO_THROW(old_worker.get());
    EXPECT_TRUE(fg::GraphManagerRebaseTestAccess::HasPendingWorker(manager));
    new_gate->Release();
    ASSERT_EQ(new_worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_NO_THROW(new_worker.get());
    EXPECT_FALSE(fg::GraphManagerRebaseTestAccess::HasPendingWorker(manager));
    EXPECT_EQ(manager.LiveNodeCount(), 2U);
    ASSERT_TRUE(manager.GetPose(1));
    EXPECT_NEAR(manager.GetPose(1)->x(), 10.05, 1e-8);
  }
}

TEST(RebaseInformation, RigidTransformCancelsOldSnapshot)
{
  fg::GraphManager manager(Params(8));
  manager.Initialize(gtsam::Pose2(), 0.0);
  Drive(manager, 1, 3);
  auto gate = GateRebuild(manager, 3);
  auto worker = std::async(std::launch::async,
                           [&]
                           {
                             manager.RebaseISAM2();
                           });
  ASSERT_TRUE(gate->Wait());
  manager.RigidTransformAll(gtsam::Pose2(10.0, 0.0, 0.0));
  gate->Release();
  ASSERT_EQ(worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_NO_THROW(worker.get());
  ASSERT_TRUE(manager.GetPose(3));
  EXPECT_NEAR(manager.GetPose(3)->x(), 10.15, 1e-8);
}
}  // namespace
