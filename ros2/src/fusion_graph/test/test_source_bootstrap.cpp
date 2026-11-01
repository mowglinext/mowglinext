// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "graph_motion_state.hpp"
#include <gtest/gtest.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
// This fixture is a separate executable from test_heading_recovery.
class GraphManagerHeadingRecoveryTestAccess
{
public:
  struct Snapshot
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    std::optional<TickOutput> latest;
    std::string state;
    GraphMotionState motion;
  };

  static Snapshot Capture(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    const auto& m = *manager.motion_;
    std::ostringstream state;
    state << std::hexfloat << manager.isam_.get() << ' ' << manager.motion_.get() << ' '
          << manager.heading_seed_.get() << ' ' << manager.initialized_ << ' '
          << manager.next_index_ << ' ' << manager.last_node_time_s_ << ' '
          << manager.current_bias_estimate_ << ' ' << manager.rebase_generation_ << ' '
          << manager.rebase_in_progress_ << ' ' << m.epoch << ' ' << m.baseline_ns << ' '
          << m.consumed_ns << ' ' << m.committed_ns << ' ' << m.schedule_ns << ' '
          << m.active_schedule_ns << ' ' << m.revision << ' ' << m.gnss_factors << ' '
          << m.yaw_factors << ' ' << m.next_acquisition << ' ' << m.relative_conditionals;
    for (const auto& observation : m.observations)
      state << " observation " << observation.id << ' ' << observation.source_ns << ' '
            << observation.xy.transpose() << ' ' << observation.sigma << ' ' << observation.robust
            << ' ' << observation.yaw.has_value() << ' ' << observation.yaw.value_or(0.0);
    for (const auto& [id, time] : m.observation_ids)
      state << " gnss-id " << id << ' ' << time;
    for (const auto& [id, time] : m.yaw_observation_ids)
      state << " yaw-id " << id << ' ' << time;
    for (const auto& id : m.pending_gyro_ids)
      state << " pending-gyro " << id.sequence << ' ' << id.observation_stamp_ns;
    for (const auto& step : m.pending_steps)
      state << " step " << step.epoch << ' ' << step.start_ns << ' ' << step.end_ns << ' '
            << step.wheel.id.sequence << ' ' << step.gyro.id.sequence << ' ' << step.wheel.vx << ' '
            << step.gyro.raw_z << ' ' << step.bias_linearization;
    const auto& a = manager.accum_;
    state << " accumulator " << a.dx << ' ' << a.dy << ' ' << a.dt_total << ' ' << a.dtheta_wheel
          << ' ' << a.dtheta_gyro << ' ' << a.gyro_preint_dtheta << ' ' << a.gyro_preint_dt << ' '
          << a.gyro_preint_variance;
    for (const auto& factor : manager.rebase_pending_factors_)
      state << " rebase-factor " << factor.get();
    state << " rebase-values " << manager.rebase_pending_values_.size();
    return {manager.isam_->getFactorsUnsafe(),
            manager.isam_->calculateEstimate(),
            manager.latest_,
            state.str(),
            m};
  }

  static void InvalidateQueuedHeadingSigma(GraphManager& manager, uint64_t id)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    for (auto& observation : manager.motion_->observations)
      if (observation.yaw && observation.id == id)
      {
        observation.sigma = std::numeric_limits<double>::quiet_NaN();
        return;
      }
    throw std::runtime_error("admitted heading missing from failure fixture");
  }

  static void InvalidateQueuedPositionSigma(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    for (auto& observation : manager.motion_->observations)
      if (!observation.yaw)
        observation.sigma = std::numeric_limits<double>::quiet_NaN();
  }
};
}  // namespace fusion_graph

namespace
{
namespace fg = fusion_graph;
using Access = fg::GraphManagerHeadingRecoveryTestAccess;
constexpr int64_t kStart = 1000000000, kSource = 1100000000, kTail = 1200000000;
constexpr uint64_t kGnss = 10, kYaw = 20;
constexpr double kGpsSigma = .003, kHeadingSigma = .15, kTrueYaw = .7;
constexpr double kLeverX = .2, kLeverY = -.1;

fg::GraphParams Params(bool bias)
{
  fg::GraphParams params;
  params.node_period_s = .1;
  params.use_imu_preint = bias;
  params.lever_arm_x = kLeverX;
  params.lever_arm_y = kLeverY;
  params.gyro_bias_estimation_enabled = false;
  params.adaptive_noise_enabled_gain = 0;
  params.cov_update_every_n = 1;
  return params;
}

gtsam::Vector2 Antenna()
{
  return gtsam::Pose2(1, 2, kTrueYaw).transformFrom(gtsam::Point2(kLeverX, kLeverY));
}

gtsam::Matrix3 Covariance(double heading_sigma)
{
  // World dp/dtheta = (Rl_y,-Rl_x). Rotate XY by R^T to the Pose2 right tangent.
  const gtsam::Vector3 sensitivity(kLeverY, -kLeverX, 1);
  gtsam::Matrix3 result = heading_sigma * heading_sigma * sensitivity * sensitivity.transpose();
  result(0, 0) += kGpsSigma * kGpsSigma;
  result(1, 1) += kGpsSigma * kGpsSigma;
  return result;
}

void Support(fg::GraphManager& manager, bool tail = false)
{
  manager.SetConsumedMotionBaseline(1, kStart);
  for (int i = 1; i <= (tail ? 2 : 1); ++i)
  {
    const int64_t start = kStart + (i - 1) * 100000000;
    const int64_t end = start + 100000000;
    const double speed = i == 1 ? 0.0 : .2, rate = i == 1 ? 0.0 : .3;
    const fg::ConsumedMotionStep step{1,
                                      start,
                                      end,
                                      {{fg::MotionSourceKind::Wheel, static_cast<uint64_t>(i), end},
                                       {start, end},
                                       speed,
                                       0,
                                       rate},
                                      {{fg::MotionSourceKind::Gyro, static_cast<uint64_t>(i), end},
                                       {start, end},
                                       rate},
                                      0,
                                      true,
                                      true};
    ASSERT_TRUE(manager.AddConsumedMotionStep(step));
  }
}

void Admit(fg::GraphManager& manager, double yaw = kTrueYaw, int64_t yaw_time = kSource)
{
  const auto antenna = Antenna();
  ASSERT_TRUE(
      manager.QueueGnssAtEpoch(antenna.x(), antenna.y(), kGpsSigma, true, kSource, kGnss, 1));
  ASSERT_TRUE(manager.QueueYawAtEpoch(yaw, kHeadingSigma, false, yaw_time, kYaw, 1));
}

void Unchanged(const Access::Snapshot& before, const Access::Snapshot& after)
{
  EXPECT_EQ(before.state, after.state);
  ASSERT_EQ(before.factors.size(), after.factors.size());
  for (size_t i = 0; i < before.factors.size(); ++i)
    EXPECT_EQ(before.factors[i].get(), after.factors[i].get());
  ASSERT_EQ(before.values.size(), after.values.size());
  for (const auto& value : before.values)
  {
    ASSERT_TRUE(after.values.exists(value.key));
    if (gtsam::Symbol(value.key).chr() == 'b')
    {
      EXPECT_EQ(value.value.cast<double>(), after.values.at<double>(value.key));
    }
    else
    {
      EXPECT_TRUE((value.value.cast<gtsam::Pose2>().matrix().array() ==
                   after.values.at<gtsam::Pose2>(value.key).matrix().array())
                      .all());
    }
  }
  ASSERT_EQ(before.latest.has_value(), after.latest.has_value());
  if (before.latest)
  {
    EXPECT_EQ(before.latest->revision, after.latest->revision);
    EXPECT_EQ(before.latest->source_stamp_ns, after.latest->source_stamp_ns);
    EXPECT_EQ(before.latest->source_epoch, after.latest->source_epoch);
    EXPECT_EQ(before.latest->node_index, after.latest->node_index);
    EXPECT_EQ(before.latest->timestamp, after.latest->timestamp);
    EXPECT_TRUE(
        (before.latest->pose.matrix().array() == after.latest->pose.matrix().array()).all());
    EXPECT_TRUE((before.latest->covariance.array() == after.latest->covariance.array()).all());
  }
}

void Belief(const Access::Snapshot& snapshot, double heading_sigma, bool bias)
{
  ASSERT_TRUE(snapshot.latest);
  EXPECT_LT(gtsam::Pose2(1, 2, kTrueYaw).localCoordinates(snapshot.latest->pose).norm(), 1e-8);
  EXPECT_LT((snapshot.latest->covariance - Covariance(heading_sigma)).cwiseAbs().maxCoeff(), 1e-10);
  const auto marginal =
      gtsam::Marginals(snapshot.factors, snapshot.values).marginalCovariance(fg::PoseKey(0));
  EXPECT_LT((marginal - snapshot.latest->covariance).cwiseAbs().maxCoeff(), 1e-12);
  if (bias)
  {
    EXPECT_NEAR(snapshot.values.at<double>(gtsam::Symbol('b', 0)), 0.0, 1e-15);
    const auto joint = gtsam::Marginals(snapshot.factors, snapshot.values)
                           .jointMarginalCovariance({fg::PoseKey(0), gtsam::Symbol('b', 0)});
    EXPECT_LT(joint(fg::PoseKey(0), gtsam::Symbol('b', 0)).norm(), 1e-15);
  }
}

TEST(SourceBootstrap, UsesQueuedLikelihoodsOnceAndPreservesBoundaryFutureAndConsumedTail)
{
  for (const bool bias : {false, true})
  {
    SCOPED_TRACE(bias);
    fg::GraphManager manager(Params(bias));
    Support(manager, true);
    Admit(manager);
    const auto antenna = Antenna();
    ASSERT_TRUE(manager.QueueGnssAtEpoch(antenna.x(), antenna.y(), .1, false, kSource, 11, 1));
    ASSERT_TRUE(manager.QueueYawAtEpoch(kTrueYaw, .2, false, kSource, 21, 1));
    ASSERT_TRUE(manager.QueueGnssAtEpoch(3, 4, .1, false, kSource + 300000000, 12, 1));
    ASSERT_TRUE(manager.QueueYawAtEpoch(.8, .2, false, kSource + 300000000, 22, 1));
    const auto before = Access::Capture(manager);
    ASSERT_EQ(manager.InitializeSourceObservations(kSource, kSource, 1, kGnss, kYaw),
              fg::SourceBootstrapResult::Applied);
    const auto initialized = Access::Capture(manager);
    Belief(initialized, kHeadingSigma, bias);
    EXPECT_EQ(initialized.motion.gnss_factors, 1u);
    EXPECT_EQ(initialized.motion.yaw_factors, 1u);
    EXPECT_EQ(initialized.motion.observations.size(), 4u);
    EXPECT_EQ(initialized.motion.observation_ids, before.motion.observation_ids);
    EXPECT_EQ(initialized.motion.yaw_observation_ids, before.motion.yaw_observation_ids);
    EXPECT_EQ(initialized.motion.consumed_ns, kTail);
    EXPECT_EQ(initialized.motion.committed_ns, kSource);
    ASSERT_EQ(initialized.motion.pending_steps.size(), 1u);
    EXPECT_EQ(initialized.motion.pending_steps.front(), before.motion.pending_steps.back());
    EXPECT_EQ(initialized.motion.pending_gyro_ids.size(), 1u);
    EXPECT_EQ(initialized.factors.nrFactors(), bias ? 3u : 2u);
    for (const auto& factor : initialized.factors)
      EXPECT_EQ(dynamic_cast<const gtsam::PriorFactor<gtsam::Pose2>*>(factor.get()), nullptr);
    // A source-only assimilation at the unchanged schedule must process only the other
    // boundary IDs. It must neither consume the remaining motion nor fuse future IDs early.
    ASSERT_TRUE(manager.TickSource(kSource, kTail, 1));
    const auto boundary = Access::Capture(manager);
    EXPECT_EQ(boundary.motion.gnss_factors, 2u);
    EXPECT_EQ(boundary.motion.yaw_factors, 2u);
    EXPECT_EQ(boundary.motion.observations.size(), 2u);
    EXPECT_EQ(boundary.motion.committed_ns, kSource);
    EXPECT_EQ(boundary.motion.pending_steps.size(), 1u);
    EXPECT_FALSE(manager.QueueGnssAtEpoch(3, 4, .1, false, kSource, kGnss, 1));
    EXPECT_FALSE(manager.QueueYawAtEpoch(-1.2, .2, false, kSource, kYaw, 1));
    const auto stable = Access::Capture(manager);
    EXPECT_FALSE(manager.TickSource(kSource, kTail, 1));
    Unchanged(stable, Access::Capture(manager));
  }
}

TEST(SourceBootstrap, MissingAdmissionOrDifferentSourceProofIsNonmutating)
{
  for (int scenario = 0; scenario < 3; ++scenario)
  {
    SCOPED_TRACE(scenario);
    fg::GraphManager manager(Params(false));
    Support(manager);
    if (scenario == 1)
      Admit(manager, kTrueYaw, kSource - 10000000);
    if (scenario == 2)
      Admit(manager);
    const auto before = Access::Capture(manager);
    EXPECT_EQ(manager.InitializeSourceObservations(
                  kSource, kSource, 1, scenario == 2 ? 999 : kGnss, kYaw),
              fg::SourceBootstrapResult::PendingProofUnavailable);
    Unchanged(before, Access::Capture(manager));
    EXPECT_FALSE(manager.IsInitialized());
    EXPECT_FALSE(manager.LatestSnapshot());
  }
}

TEST(SourceBootstrap, InvalidQueuedProofFailsWithoutConsumingEitherObservation)
{
  fg::GraphManager manager(Params(true));
  Support(manager, true);
  Admit(manager);
  Access::InvalidateQueuedHeadingSigma(manager, kYaw);
  const auto before = Access::Capture(manager);
  EXPECT_EQ(manager.InitializeSourceObservations(kSource, kSource, 1, kGnss, kYaw),
            fg::SourceBootstrapResult::Failed);
  const auto after = Access::Capture(manager);
  Unchanged(before, after);
  EXPECT_FALSE(manager.IsInitialized());
  EXPECT_FALSE(manager.LatestSnapshot());
  EXPECT_EQ(after.motion.observations.size(), 2u);
  EXPECT_EQ(after.motion.gnss_factors, 0u);
  EXPECT_EQ(after.motion.yaw_factors, 0u);
}

TEST(SourceBootstrap, RawRecoveryRetiresOnlyHeadingAndKeepsActualGnssLikelihoodAndIdentity)
{
  for (const bool bias : {false, true})
  {
    SCOPED_TRACE(bias);
    fg::GraphManager manager(Params(bias));
    Support(manager);
    Admit(manager, -1.2);
    ASSERT_EQ(manager.InitializeSourceObservations(kSource, kSource, 1, kGnss, kYaw),
              fg::SourceBootstrapResult::Applied);
    const auto seed = manager.OriginalHeadingSeed();
    ASSERT_TRUE(seed);
    ASSERT_TRUE(seed->source_position);
    const auto position = seed->source_position, old_heading = seed->prior;
    const fg::RecoveryYawObservation recovery{1, kSource, 30, kTrueYaw, .1, false};
    ASSERT_EQ(manager.RecoverInvalidSeedHeading(seed, true, recovery),
              fg::HeadingRecoveryResult::Applied);
    const auto repaired = Access::Capture(manager);
    Belief(repaired, .1, bias);
    EXPECT_EQ(std::count(repaired.factors.begin(), repaired.factors.end(), position), 1);
    EXPECT_EQ(std::count(repaired.factors.begin(), repaired.factors.end(), old_heading), 0);
    size_t yaw_factors = 0;
    for (const auto& factor : repaired.factors)
    {
      EXPECT_EQ(dynamic_cast<const gtsam::PoseTranslationPrior<gtsam::Pose2>*>(factor.get()),
                nullptr);
      EXPECT_EQ(dynamic_cast<const gtsam::PriorFactor<gtsam::Pose2>*>(factor.get()), nullptr);
      yaw_factors += dynamic_cast<const fg::YawUnaryFactor*>(factor.get()) != nullptr;
    }
    EXPECT_EQ(yaw_factors, 1u);
    EXPECT_EQ(repaired.motion.observation_ids.at(kGnss), kSource);
    EXPECT_EQ(repaired.motion.yaw_observation_ids.at(kYaw), kSource);
    EXPECT_EQ(repaired.motion.yaw_observation_ids.at(recovery.id), kSource);
    EXPECT_EQ(repaired.motion.gnss_factors, 1u);
    EXPECT_EQ(repaired.motion.yaw_factors, 2u);
    EXPECT_TRUE(repaired.motion.observations.empty());
    EXPECT_TRUE(repaired.motion.relative_conditionals);
    EXPECT_FALSE(manager.OriginalHeadingSeed());
    EXPECT_FALSE(manager.QueueGnssAtEpoch(1, 2, .003, false, kSource, kGnss, 1));
    EXPECT_FALSE(manager.QueueYawAtEpoch(-1.2, kHeadingSigma, false, kSource, kYaw, 1));
    EXPECT_FALSE(manager.QueueYawAtEpoch(kTrueYaw, .1, false, kSource, recovery.id, 1));
    EXPECT_FALSE(manager.TickSource(kSource + 1, kSource, 1));
    Unchanged(repaired, Access::Capture(manager));
  }
}

TEST(SourceBootstrap, RetimedHeadingUsesOnlyOriginalGnssAndPreservesRawYawAndSplitTail)
{
  constexpr int64_t source = kSource + 50000000, schedule = kTail + 100000000;
  for (const bool bias : {false, true})
    for (const bool lever : {false, true})
      for (const int order : {-1, 1})
      {
        SCOPED_TRACE(::testing::Message() << bias << ' ' << lever << ' ' << order);
        auto params = Params(bias);
        params.prior_sigma_theta = .07;
        if (!lever)
          params.lever_arm_x = params.lever_arm_y = 0;
        fg::GraphManager manager(params);
        Support(manager, true);
        const auto antenna =
            gtsam::Pose2(1, 2, kTrueYaw)
                .transformFrom(gtsam::Point2(params.lever_arm_x, params.lever_arm_y));
        ASSERT_TRUE(
            manager.QueueGnssAtEpoch(antenna.x(), antenna.y(), .014, true, source, kGnss, 1));
        const auto yaw_source = source + order * 30000000;
        ASSERT_TRUE(manager.QueueYawAtEpoch(.9, kHeadingSigma, true, yaw_source, kYaw, 1));
        ASSERT_TRUE(manager.QueueYawAtEpoch(.8, .2, false, kTail + 1, 21, 1));
        const auto before = Access::Capture(manager);
        ASSERT_EQ(manager.InitializeSourcePositionWithRetimedHeading(
                      schedule, source, 1, kGnss, kTrueYaw),
                  fg::SourceBootstrapResult::Applied);
        const auto initialized = Access::Capture(manager);
        ASSERT_TRUE(initialized.latest);
        EXPECT_LT(gtsam::Pose2(1, 2, kTrueYaw).localCoordinates(initialized.latest->pose).norm(),
                  1e-10);
        const gtsam::Vector3 sensitivity(params.lever_arm_y, -params.lever_arm_x, 1);
        gtsam::Matrix3 expected = params.prior_sigma_theta * params.prior_sigma_theta *
                                  sensitivity * sensitivity.transpose();
        expected(0, 0) += .014 * .014;
        expected(1, 1) += .014 * .014;
        EXPECT_LT((initialized.latest->covariance - expected).cwiseAbs().maxCoeff(), 1e-12);
        const auto marginal = gtsam::Marginals(initialized.factors, initialized.values)
                                  .marginalCovariance(fg::PoseKey(0));
        EXPECT_LT((marginal - expected).cwiseAbs().maxCoeff(), 1e-12);
        const auto seed = manager.OriginalHeadingSeed();
        ASSERT_TRUE(seed);
        EXPECT_EQ(seed->heading_provenance, fg::HeadingSeedState::HeadingProvenance::RetimedPrior);
        EXPECT_FALSE(seed->source_heading_id);
        EXPECT_EQ(seed->source_gnss_id, kGnss);
        EXPECT_EQ(std::count(initialized.factors.begin(),
                             initialized.factors.end(),
                             seed->source_position),
                  1);
        EXPECT_EQ(initialized.factors.nrFactors(), bias ? 3u : 2u);
        for (const auto& factor : initialized.factors)
          EXPECT_EQ(dynamic_cast<const gtsam::PriorFactor<gtsam::Pose2>*>(factor.get()), nullptr);
        EXPECT_EQ(initialized.motion.gnss_factors, 1u);
        EXPECT_EQ(initialized.motion.yaw_factors, 0u);
        EXPECT_EQ(initialized.motion.observation_ids, before.motion.observation_ids);
        EXPECT_EQ(initialized.motion.yaw_observation_ids, before.motion.yaw_observation_ids);
        ASSERT_EQ(initialized.motion.observations.size(), 2u);
        for (size_t i = 0; i < initialized.motion.observations.size(); ++i)
        {
          const auto& retained = initialized.motion.observations[i];
          const auto& original = before.motion.observations[i + 1];
          EXPECT_EQ(retained.id, original.id);
          EXPECT_EQ(retained.source_ns, original.source_ns);
          EXPECT_EQ(retained.yaw, original.yaw);
          EXPECT_EQ(retained.sigma, original.sigma);
          EXPECT_EQ(retained.robust, original.robust);
        }
        EXPECT_EQ(initialized.latest->source_stamp_ns, source);
        EXPECT_EQ(initialized.latest->source_epoch, 1u);
        EXPECT_EQ(initialized.latest->node_index, 0u);
        EXPECT_EQ(initialized.latest->timestamp, static_cast<double>(schedule) * 1e-9);
        EXPECT_EQ(initialized.motion.schedule_ns, schedule);
        EXPECT_EQ(initialized.motion.committed_ns, source);
        EXPECT_EQ(initialized.motion.consumed_ns, kTail);
        ASSERT_EQ(initialized.motion.pending_steps.size(), 1u);
        auto tail = before.motion.pending_steps.back();
        tail.start_ns = source;
        EXPECT_EQ(initialized.motion.pending_steps.front(), tail);
        ASSERT_EQ(initialized.motion.pending_gyro_ids.size(), 1u);
        EXPECT_TRUE(initialized.motion.pending_gyro_ids.count(tail.gyro.id));
        EXPECT_FALSE(manager.QueueGnssAtEpoch(3, 4, .1, false, source, kGnss, 1));
        // A duplicate input and a same-schedule tick cannot apply that GNSS again
        // or consume the retained process tail before the normal cadence.
        manager.TickSource(schedule, kTail, 1);
        const auto ticked = Access::Capture(manager);
        EXPECT_EQ(ticked.motion.gnss_factors, 1u);
        EXPECT_EQ(ticked.motion.committed_ns, source);
        EXPECT_EQ(ticked.motion.pending_steps.front(), tail);
        EXPECT_EQ(std::count(ticked.factors.begin(), ticked.factors.end(), seed->source_position),
                  1);
      }
}

TEST(SourceBootstrap, RetimedHeadingPreservesEffectiveGnssFloorAndHuberLikelihood)
{
  for (const double reported_sigma : {.001, .014})
    for (const bool robust : {false, true})
    {
      SCOPED_TRACE(::testing::Message() << reported_sigma << ' ' << robust);
      auto params = Params(false);
      fg::GraphManager manager(params);
      Support(manager);
      const auto antenna = Antenna();
      ASSERT_TRUE(manager.QueueGnssAtEpoch(
          antenna.x(), antenna.y(), reported_sigma, robust, kSource, kGnss, 1));
      ASSERT_EQ(
          manager.InitializeSourcePositionWithRetimedHeading(kSource, kSource, 1, kGnss, kTrueYaw),
          fg::SourceBootstrapResult::Applied);
      const auto snapshot = Access::Capture(manager);
      const double sigma = std::max(reported_sigma, params.gps_sigma_floor);
      const gtsam::Vector3 sensitivity(kLeverY, -kLeverX, 1);
      gtsam::Matrix3 expected = params.prior_sigma_theta * params.prior_sigma_theta * sensitivity *
                                sensitivity.transpose();
      expected(0, 0) += sigma * sigma;
      expected(1, 1) += sigma * sigma;
      EXPECT_LT((snapshot.latest->covariance - expected).cwiseAbs().maxCoeff(), 1e-12);
      auto displaced = snapshot.values;
      displaced.update(fg::PoseKey(0), gtsam::Pose2(1 + 10 * sigma, 2, kTrueYaw));
      const auto seed = manager.OriginalHeadingSeed();
      ASSERT_TRUE(seed);
      // Isolate the actual GNSS factor's loss well outside the Huber cutoff.
      const double expected_loss =
          robust ? params.huber_k_gps * (10 - .5 * params.huber_k_gps) : 50.0;
      EXPECT_NEAR(seed->source_position->error(displaced), expected_loss, 1e-10);
    }
}

TEST(SourceBootstrap, RetimedHeadingMissingOrInvalidProofLeavesAllPendingStateUntouched)
{
  for (int scenario = 0; scenario < 7; ++scenario)
  {
    SCOPED_TRACE(scenario);
    auto params = Params(true);
    if (scenario == 5)
      params.prior_sigma_theta = std::numeric_limits<double>::quiet_NaN();
    fg::GraphManager manager(params);
    Support(manager, true);
    if (scenario != 0)
      Admit(manager, .9, kSource + 30000000);
    if (scenario == 6)
      Access::InvalidateQueuedPositionSigma(manager);
    const auto before = Access::Capture(manager);
    const auto result = manager.InitializeSourcePositionWithRetimedHeading(
        kSource,
        scenario == 3 ? kTail + 1 : kSource,
        scenario == 2 ? 2 : 1,
        scenario == 1 ? 999 : kGnss,
        scenario == 4 ? std::numeric_limits<double>::quiet_NaN() : kTrueYaw);
    EXPECT_EQ(result,
              scenario < 4 ? fg::SourceBootstrapResult::PendingProofUnavailable
                           : fg::SourceBootstrapResult::Failed);
    Unchanged(before, Access::Capture(manager));
    EXPECT_FALSE(manager.IsInitialized());
    EXPECT_FALSE(manager.OriginalHeadingSeed());
  }
}

TEST(SourceBootstrap, RetimedHeadingRecoveryRefusesSyntheticFamilyWithoutMutation)
{
  for (const int order : {-1, 1})
  {
    fg::GraphManager manager(Params(false));
    Support(manager);
    Admit(manager, .9, kSource + order * 30000000);
    ASSERT_EQ(
        manager.InitializeSourcePositionWithRetimedHeading(kSource, kSource, 1, kGnss, kTrueYaw),
        fg::SourceBootstrapResult::Applied);
    const auto seed = manager.OriginalHeadingSeed();
    ASSERT_TRUE(seed);
    const auto before = Access::Capture(manager);
    EXPECT_EQ(manager.RecoverInvalidSeedHeading(seed, true, {1, kSource, 30, .8, .1, false}),
              fg::HeadingRecoveryResult::UnsupportedFamily);
    Unchanged(before, Access::Capture(manager));
    EXPECT_EQ(manager.OriginalHeadingSeed(), seed);
    EXPECT_TRUE(seed->source_position);
    EXPECT_FALSE(seed->source_heading_id);
  }
}
}  // namespace
