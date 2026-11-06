// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
struct GraphManagerTestAccess
{
  static auto ProcessState(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    return std::array<double, 15>{g.accum_.dx,
                                  g.accum_.dy,
                                  g.accum_.dtheta_wheel,
                                  g.accum_.dtheta_gyro,
                                  g.accum_.dt_total,
                                  g.accum_.max_abs_gyro_rad_per_s,
                                  g.accum_.gyro_preint_dtheta,
                                  g.accum_.gyro_preint_dt,
                                  g.accum_.gyro_preint_variance,
                                  g.residual_ema_,
                                  g.last_wheel_sigma_x_eff_,
                                  g.current_bias_estimate_,
                                  double(g.wheel_stationary_now_),
                                  double(g.queue_.yaw.has_value()),
                                  double(g.queue_.lidar_map_xy.has_value())};
  }
  static bool HasGnss(GraphManager& g)
  {
    return g.queue_.gnss.has_value();
  }
  static gtsam::ISAM2 CopyIsam(GraphManager& g)
  {
    return g.isam_;
  }
  static gtsam::Key LastFactorKey(GraphManager& g)
  {
    return g.isam_.getFactorsUnsafe().back()->keys().front();
  }
  static bool ResetDuringRebase(GraphManager& g)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(g.mu_);
        if (g.rebase_in_progress_)
        {
          g.ResetLocked();
          return true;
        }
      }
      std::this_thread::yield();
    }
    return false;
  }
  static bool FailingUpdate(GraphManager& g)
  {
    std::lock_guard<std::mutex> lock(g.mu_);
    gtsam::NonlinearFactorGraph factors;
    factors.add(gtsam::PriorFactor<gtsam::Pose2>(PoseKey(99999),
                                                 gtsam::Pose2(),
                                                 GraphManager::MakeDiagonal({.01, .01, .01})));
    return g.ApplyIsamUpdateLocked(factors, gtsam::Values());
  }
  // Hold the manager lock while adding the observation after phase 1. This is
  // the real ApplyIsamUpdateLocked path and real phase-3 replay, not a mock.
  static bool AddDuringRebase(GraphManager& g, uint64_t target)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(g.mu_);
        if (g.rebase_in_progress_)
        {
          g.queue_.gnss =
              GraphManager::UnaryQueue::Gnss{gtsam::Vector2(.02, 0.), .003, false, target};
          g.ApplyQueuedGnssLocked();
          return g.rebase_pending_factors_.size() == 1;
        }
      }
      std::this_thread::yield();
    }
    return false;
  }
};
}  // namespace fusion_graph

namespace fg = fusion_graph;
namespace
{
fg::GraphParams Params()
{
  fg::GraphParams p;
  p.node_period_s = .04;
  p.stationary_node_period_s = 5.;
  p.lever_arm_x = .3;
  p.lever_arm_y = -.08;
  p.cov_update_every_n = 1;
  p.isam2_relinearize_skip = 1;
  p.adaptive_noise_enabled_gain = 0.;
  p.gyro_bias_estimation_enabled = false;
  return p;
}

void FillGraph(fg::GraphManager& g, int nodes)
{
  for (int i = 1; i < nodes; ++i)
  {
    g.AddWheelTwist(.2, 0., 0., .05);
    g.AddGyroDelta(0., .05);
    ASSERT_TRUE(g.Tick(i * .05));
  }
}
}  // namespace

TEST(StationaryGnss, SameTargetMatchesIdenticalReferenceAndLeavesProcessStateUntouched)
{
  for (const bool robust : {false, true})
  {
    const auto p = Params();
    fg::GraphManager g(p);
    const gtsam::Pose2 head(1., -2., .7);
    g.Initialize(head, 0.);
    g.AddWheelTwist(.01, .003, .01, .04);  // sub-threshold process interval
    g.AddGyroDelta(.01, .04);
    g.QueueYaw(.8, .1);
    g.QueueLidarMapXy(gtsam::Vector2(1., -2.), Eigen::Matrix2d::Identity() * .01);
    const auto process_before = fg::GraphManagerTestAccess::ProcessState(g);
    const auto before = *g.LatestSnapshot();
    const auto stats_before = g.Stats();
    auto reference = fg::GraphManagerTestAccess::CopyIsam(g);
    const gtsam::Vector2 antenna = head.transformFrom(gtsam::Point2(p.lever_arm_x, p.lever_arm_y)) +
                                   gtsam::Vector2(.025, -.01);
    const double sigma = p.gps_sigma_floor;
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Isotropic::Sigma(2, sigma);
    if (robust)
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    p.huber_k_gps),
                                                noise);
    gtsam::NonlinearFactorGraph factors;
    factors.add(fg::GnssLeverArmFactor(
        fg::PoseKey(0), antenna, gtsam::Vector2(p.lever_arm_x, p.lever_arm_y), noise));
    reference.update(factors, gtsam::Values());
    ASSERT_TRUE(g.QueueGnss(antenna.x(), antenna.y(), -1., robust, 0));
    EXPECT_FALSE(g.Tick(.2));
    const auto after = *g.LatestSnapshot();
    EXPECT_EQ(after.node_index, before.node_index);
    EXPECT_EQ(after.timestamp, before.timestamp);
    EXPECT_GT(after.estimate_revision, before.estimate_revision);
    EXPECT_TRUE(
        after.pose.equals(reference.calculateEstimate<gtsam::Pose2>(fg::PoseKey(0)), 1e-10));
    EXPECT_TRUE(after.covariance.isApprox(reference.marginalCovariance(fg::PoseKey(0)), 1e-10));
    EXPECT_EQ(fg::GraphManagerTestAccess::ProcessState(g), process_before);
    EXPECT_FALSE(fg::GraphManagerTestAccess::HasGnss(g));
    EXPECT_EQ(g.Stats().successful_updates, stats_before.successful_updates + 1);
    EXPECT_EQ(g.Stats().factor_slots, stats_before.factor_slots + 1);
    EXPECT_EQ(g.Stats().total_nodes, stats_before.total_nodes);
    EXPECT_FALSE(g.Tick(.24));
    EXPECT_EQ(g.LatestSnapshot()->estimate_revision, after.estimate_revision);
    EXPECT_EQ(g.Stats().successful_updates, stats_before.successful_updates + 1);
  }
}

TEST(StationaryGnss, UnsetTargetWaitsForOrdinaryNodeAndRemovedTargetIsDropped)
{
  auto p = Params();
  p.max_graph_nodes = 1;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  ASSERT_TRUE(g.QueueGnss(.3, 0., .003));
  EXPECT_FALSE(g.Tick(.2));
  EXPECT_TRUE(fg::GraphManagerTestAccess::HasGnss(g));
  ASSERT_TRUE(g.Tick(5.));
  EXPECT_FALSE(fg::GraphManagerTestAccess::HasGnss(g));
  EXPECT_EQ(fg::GraphManagerTestAccess::LastFactorKey(g), fg::PoseKey(1));
  ASSERT_TRUE(g.QueueGnss(.3, 0., .003, false, 0));
  g.RebaseISAM2();
  EXPECT_FALSE(g.GetPose(0));
  const auto before = g.Stats();
  EXPECT_FALSE(g.Tick(5.2));
  EXPECT_FALSE(fg::GraphManagerTestAccess::HasGnss(g));
  EXPECT_EQ(g.Stats().successful_updates, before.successful_updates);
  EXPECT_EQ(g.Stats().factor_slots, before.factor_slots);
}

TEST(StationaryGnss, DelayedHistoricalFixKeepsItsTargetAndRefreshesHead)
{
  auto p = Params();
  p.prior_sigma_xy = .001;
  p.lever_arm_x = 0.;
  p.lever_arm_y = 0.;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  g.AddWheelTwist(1., 0., 0., 1.);
  ASSERT_TRUE(g.Tick(1.));
  const auto before = *g.LatestSnapshot();
  ASSERT_TRUE(g.QueueGnss(.01, 0., .003, false, g.FindNodeAtOrBefore(.5)));
  EXPECT_FALSE(g.Tick(1.2));
  EXPECT_EQ(fg::GraphManagerTestAccess::LastFactorKey(g), fg::PoseKey(0));
  EXPECT_GT(g.LatestSnapshot()->pose.x(), 1.);
  EXPECT_LT(g.LatestSnapshot()->pose.x(), 1.02);
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, before.estimate_revision);
  EXPECT_EQ(g.LatestSnapshot()->timestamp, 1.);
}

TEST(StationaryGnss, ParkedVaryingFixesDoNotCreateProcessEdgesOrEmptyUpdates)
{
  fg::GraphManager g(Params());
  g.Initialize(gtsam::Pose2(), 0.);
  const auto before = g.Stats();
  unsigned nodes = 0, fixes = 0;
  for (unsigned i = 1; i <= 500; ++i)
  {
    const double t = i * .04;
    g.AddWheelTwist(0., 0., 0., .04);
    g.AddGyroDelta(0., .04);
    if (i % 5 == 0)
    {
      ASSERT_TRUE(g.QueueGnss(.3 + .001 * std::sin(t),
                              -.08 + .001 * std::cos(t),
                              .003,
                              false,
                              g.FindNodeAtOrBefore(t)));
      ++fixes;
    }
    if (g.Tick(t))
      ++nodes;
  }
  EXPECT_EQ(nodes, 4u);
  const auto after = g.Stats();
  // Every factor is either one ordinary process edge or a consumed GNSS fix.
  EXPECT_EQ(after.factor_slots - before.factor_slots, nodes + fixes);
  // Four fixes share ordinary node updates, the other 96 are factor-only.
  EXPECT_EQ(after.successful_updates - before.successful_updates, fixes);
  EXPECT_EQ(after.total_nodes - before.total_nodes, nodes);
}

TEST(StationaryGnss, MotionClassificationClosesSmallTailAndKeepsGyroRotation)
{
  auto p = Params();
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  g.AddWheelTwist(.05, 0., 0., .34);  // the reproduced 17 mm tail is nonstationary
  ASSERT_TRUE(g.Tick(.34));
  EXPECT_NEAR(g.LatestSnapshot()->pose.x(), .017, 1e-9);
  g.AddGyroDelta(.11, .04);
  ASSERT_TRUE(g.Tick(.4));  // gyro disagreement defeats the coarse throttle
  EXPECT_GT(g.LatestSnapshot()->pose.theta(), .004);
}

TEST(StationaryGnss, MeasurementWorkTriggersMaintenanceAndResetDoesNotUnderflow)
{
  auto p = Params();
  p.lever_arm_x = 0.;
  p.lever_arm_y = 0.;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  for (int i = 0; i < 12; ++i)
  {
    ASSERT_TRUE(g.QueueGnss(.001 * i, 0., .003, false, 0));
    EXPECT_FALSE(g.Tick(.2));
  }
  EXPECT_TRUE(g.RebaseDue(10));
  const auto before = g.Stats();
  g.RebaseISAM2();
  EXPECT_FALSE(g.RebaseDue(10));
  EXPECT_EQ(g.Stats().updates_since_rebase, 0u);
  EXPECT_EQ(g.Stats().successful_rebases, before.successful_rebases + 1);
  EXPECT_EQ(g.Stats().factor_slots, 1u);
  const auto revision = g.LatestSnapshot()->estimate_revision;
  g.Reset();
  g.Initialize(gtsam::Pose2(), 0.);
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, revision);
  EXPECT_GT(g.Stats().successful_updates, before.successful_updates);
  EXPECT_FALSE(g.RebaseDue(10));
  const auto updates = g.Stats().successful_updates;
  EXPECT_FALSE(fg::GraphManagerTestAccess::FailingUpdate(g));
  EXPECT_FALSE(g.IsInitialized());
  EXPECT_EQ(g.Stats().successful_updates, updates);
  g.Initialize(gtsam::Pose2(), 0.);
  EXPECT_EQ(g.Stats().updates_since_rebase, 1u);
  EXPECT_FALSE(g.RebaseDue(10));
}

TEST(StationaryGnss, ForceAnchorAndLoadPublishFreshRevisions)
{
  auto p = Params();
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  auto revision = g.LatestSnapshot()->estimate_revision;
  g.ForceAnchor(0, gtsam::Pose2(.02, .01, .1), .003, .003);
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, revision);
  revision = g.LatestSnapshot()->estimate_revision;
  g.RigidTransformAll(gtsam::Pose2(.2, .1, .3));
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, revision);
  const auto prefix =
      (std::filesystem::temp_directory_path() / "stationary_gnss_revision").string();
  ASSERT_TRUE(g.Save(prefix));
  revision = g.LatestSnapshot()->estimate_revision;
  g.Reset();
  ASSERT_TRUE(g.Load(prefix));
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, revision);
  std::filesystem::remove(prefix + ".checkpoint");
  std::filesystem::remove(prefix + ".checkpoint.previous");
}

TEST(StationaryGnss, GnssOnlyUpdateSurvivesRealAsyncRebase)
{
  auto p = Params();
  p.max_graph_nodes = 6000;
  p.lever_arm_x = 0.;
  p.lever_arm_y = 0.;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  FillGraph(g, 6000);
  const auto head = g.LatestSnapshot()->node_index;
  const auto revision = g.LatestSnapshot()->estimate_revision;
  std::thread worker(
      [&]
      {
        g.RebaseISAM2();
      });
  const bool mirrored = fg::GraphManagerTestAccess::AddDuringRebase(g, head);
  worker.join();
  ASSERT_TRUE(mirrored);
  EXPECT_EQ(g.Stats().successful_rebases, 1u);
  EXPECT_EQ(g.Stats().factor_slots, 6001u);
  EXPECT_EQ(g.Stats().updates_since_rebase, 1u);
  EXPECT_GT(g.LatestSnapshot()->estimate_revision, revision);
  EXPECT_LT(g.LatestSnapshot()->pose.x(), 1.);
}

TEST(StationaryGnss, FailedReplayRetainsLiveGraphAndMaintenanceRetry)
{
  auto p = Params();
  p.max_graph_nodes = 3000;
  p.lever_arm_x = 0.;
  p.lever_arm_y = 0.;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  FillGraph(g, 6000);
  const auto count = g.Stats().successful_updates;
  std::thread worker(
      [&]
      {
        g.RebaseISAM2();
      });
  // Node 0 exists in the live tree but is outside the fresh snapshot window.
  const bool mirrored = fg::GraphManagerTestAccess::AddDuringRebase(g, 0);
  worker.join();
  ASSERT_TRUE(mirrored);
  EXPECT_EQ(g.Stats().successful_rebases, 0u);
  EXPECT_EQ(g.Stats().successful_updates, count + 1);
  EXPECT_TRUE(g.GetPose(0));
  EXPECT_TRUE(g.RebaseDue(2000));
  g.RebaseISAM2();
  EXPECT_EQ(g.Stats().successful_rebases, 1u);
  EXPECT_FALSE(g.RebaseDue(2000));
}

TEST(StationaryGnss, PreintegrationRebasePreservesBiasTypes)
{
  auto p = Params();
  p.use_imu_preint = true;
  p.max_graph_nodes = 10;
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  FillGraph(g, 20);
  g.RebaseISAM2();
  EXPECT_EQ(g.Stats().successful_rebases, 1u);
  EXPECT_EQ(g.LiveNodeCount(), 10u);
  EXPECT_TRUE(g.IsInitialized());
  g.AddWheelTwist(.2, 0., 0., .05);
  g.AddGyroDelta(0., .05);
  EXPECT_TRUE(g.Tick(1.1));
}

TEST(StationaryGnss, ResetCancelsAsyncSnapshotWithoutRetiringNewWork)
{
  auto p = Params();
  fg::GraphManager g(p);
  g.Initialize(gtsam::Pose2(), 0.);
  FillGraph(g, 6000);
  const auto before = g.Stats().successful_updates;
  std::thread worker(
      [&]
      {
        g.RebaseISAM2();
      });
  const bool reset = fg::GraphManagerTestAccess::ResetDuringRebase(g);
  g.Initialize(gtsam::Pose2(4., -3., .8), 1.);
  worker.join();
  ASSERT_TRUE(reset);
  EXPECT_EQ(g.Stats().successful_rebases, 0u);
  EXPECT_EQ(g.Stats().successful_updates, before + 1);
  EXPECT_EQ(g.Stats().updates_since_rebase, 1u);
  EXPECT_EQ(g.LiveNodeCount(), 1u);
  EXPECT_TRUE(g.LatestSnapshot()->pose.equals(gtsam::Pose2(4., -3., .8), 1e-10));
  EXPECT_FALSE(g.RebaseDue(2000));
}

TEST(StationaryGnss, TruthfulSlowCreepAndDelayedFixesDoNotAcquirePeriodSizedLag)
{
  for (double speed : {.005, .010, .020, .025})
  {
    for (int delay_steps : {0, 8})
    {
      SCOPED_TRACE(::testing::Message() << "speed=" << speed << " delay=" << delay_steps * .02);
      auto p = Params();
      p.lever_arm_y = 0.;
      p.gps_sigma_floor = .003;
      p.stationary_sigma_theta = .01;
      p.isam2_relinearize_skip = 5;
      fg::GraphManager g(p);
      g.Initialize(gtsam::Pose2(), 0., .005);
      gtsam::Pose2 anchor, previous_head;
      uint64_t node = 0, revision = g.LatestSnapshot()->estimate_revision;
      double max_error = 0.;
      for (int step = 1; step <= 1000; ++step)
      {
        const double t = step * .02;
        g.AddWheelTwist(speed, 0., 0., .02);
        g.AddGyroDelta(0., .02);
        const int source_step = step - delay_steps;
        if (source_step >= 0 && source_step % 10 == 0)
        {
          const double source_t = source_step * .02;
          ASSERT_TRUE(
              g.QueueGnss(speed * source_t + .3, 0., .003, false, g.FindNodeAtOrBefore(source_t)));
        }
        if (step % 2 == 0)
        {
          g.Tick(t);
          const auto snap = *g.LatestSnapshot();
          const gtsam::Pose2 dr(speed * t, 0., 0.);
          if (snap.node_index != node)
            anchor = snap.pose.compose(dr.inverse());
          else if (snap.estimate_revision != revision)
            anchor = snap.pose.compose(previous_head.inverse()).compose(anchor);
          node = snap.node_index;
          revision = snap.estimate_revision;
          previous_head = snap.pose;
          max_error = std::max(max_error, std::abs(anchor.compose(dr).x() - speed * t));
        }
      }
      // Component discriminator from the retained classification control, not
      // a localization confidence or controller safety tolerance.
      EXPECT_LT(max_error, .0021);
      EXPECT_LT(20. - g.LatestSnapshot()->timestamp, .25);
    }
  }
}
