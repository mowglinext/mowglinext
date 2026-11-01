// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "source_bootstrap_state.hpp"
#include <gtest/gtest.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
class GraphManagerSourceTestAccess
{
public:
  struct Snapshot
  {
    GraphMotionState motion;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    ProcessWindowPolicyState policy;
    ProcessWindowPolicyInput input;
    double feedback;
    std::string fingerprint;
  };
  static Snapshot Capture(GraphManager& manager, int64_t now = 0)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    const auto& a = manager.accum_;
    const auto& m = *manager.motion_;
    ProcessWindowPolicyInput input{a.dx,
                                   a.dy,
                                   a.wheel_travel_m,
                                   a.dtheta_wheel,
                                   a.dtheta_gyro,
                                   a.dt_total,
                                   a.max_abs_gyro_rad_per_s,
                                   manager.last_node_time_s_,
                                   now * 1e-9};
    ProcessWindowPolicyState policy{manager.slip_window_,
                                    manager.residual_ema_,
                                    manager.stats_hand_push_,
                                    manager.stats_slip_veto_,
                                    manager.wheel_stationary_now_,
                                    manager.last_wheel_sigma_x_eff_};
    std::ostringstream f;
    f << std::hexfloat << manager.isam_.get() << ' ' << manager.motion_.get() << ' '
      << manager.heading_seed_.get() << ' ' << m.bootstrap.get() << ' ' << manager.next_index_
      << ' ' << manager.last_node_time_s_ << ' ' << manager.current_bias_estimate_ << ' '
      << m.committed_ns << ' ' << m.consumed_ns << ' ' << m.schedule_ns << ' ' << m.revision << ' '
      << m.gnss_factors << ' ' << m.yaw_factors << ' ' << m.next_acquisition << ' ' << a.dx << ' '
      << a.dy << ' ' << a.wheel_travel_m << ' ' << a.dtheta_wheel << ' ' << a.dtheta_gyro << ' '
      << a.dt_total << ' ' << a.max_abs_gyro_rad_per_s << ' ' << a.gyro_preint_dtheta << ' '
      << a.gyro_preint_dt << ' ' << a.gyro_preint_variance << ' ' << policy.residual_ema << ' '
      << policy.slip_window.Size() << ' ' << policy.hand_push_count << ' ' << policy.slip_veto_count
      << ' ' << policy.wheel_stationary_now << ' ' << policy.last_wheel_sigma_x_eff << ' '
      << manager.rebase_in_progress_ << ' ' << manager.rebase_generation_ << ' '
      << manager.queue_.gnss.has_value() << ' ' << manager.queue_.yaw.has_value() << ' '
      << manager.queue_.lidar_map_xy.has_value() << ' ' << manager.lidar_anchor_factors_;
    for (const auto& observation : m.observations)
      f << " obs " << observation.id << ' ' << observation.source_ns << ' ' << observation.sigma
        << ' ' << observation.xy.transpose() << ' ' << observation.yaw.value_or(0.);
    for (const auto& step : m.pending_steps)
      f << " step " << step.start_ns << ' ' << step.end_ns << ' ' << step.gyro.id.sequence << ' '
        << step.gyro.raw_z << ' ' << step.bias_linearization;
    for (const auto& factor : manager.isam_->getFactorsUnsafe())
      f << " factor " << factor.get();
    if (manager.latest_)
      f << " latest " << manager.latest_->pose.matrix() << ' ' << manager.latest_->covariance << ' '
        << manager.latest_->source_stamp_ns << ' ' << manager.latest_->node_index << ' '
        << manager.latest_->revision;
    return {m,
            manager.isam_->getFactorsUnsafe(),
            manager.isam_->calculateEstimate(),
            policy,
            input,
            manager.current_bias_estimate_,
            f.str()};
  }
  static void CorruptAccumulator(GraphManager& manager, double delta)
  {
    manager.accum_.dx += delta;
  }
  static void CorruptHeading(GraphManager& manager)
  {
    for (auto& observation : manager.motion_->observations)
      if (observation.yaw)
        observation.sigma = std::numeric_limits<double>::quiet_NaN();
  }
};
}  // namespace fusion_graph

namespace
{
namespace fg = fusion_graph;
using Access = fg::GraphManagerSourceTestAccess;
constexpr uint64_t epoch = 7, gps = 71, yaw = 82;
constexpr int64_t start = 1000000000, s = 1100000000, u = 1200000000, v = 1300000000;
constexpr int64_t schedule = 10000000000, due = 10100000000;

fg::GraphParams Params(bool preint = false)
{
  fg::GraphParams p;
  p.node_period_s = .1;
  p.stationary_node_period_s = .1;
  p.use_imu_preint = preint;
  p.gyro_bias_estimation_enabled = false;
  p.lever_arm_x = .2;
  p.lever_arm_y = -.1;
  p.adaptive_noise_enabled_gain = 1.;
  p.cov_update_every_n = 1;
  return p;
}

void Consume(fg::GraphManager& manager,
             int64_t begin,
             int64_t end,
             uint64_t id,
             double speed = .2,
             double rate = .3)
{
  const double dt = (end - begin) * 1e-9;
  manager.AddWheelTwist(speed, 0., rate, dt);
  const double correction = manager.AddGyroDelta(rate, dt);
  const fg::ConsumedMotionStep step{
      epoch,
      begin,
      end,
      {{fg::MotionSourceKind::Wheel, id, end}, {begin, end}, speed, 0., rate},
      {{fg::MotionSourceKind::Gyro, id, end}, {begin, end}, rate},
      correction,
      true,
      true};
  ASSERT_TRUE(manager.AddConsumedMotionStep(step));
}

void Admit(fg::GraphManager& manager, bool earlier)
{
  ASSERT_TRUE(manager.QueueGnssAtEpoch(.2, -.1, .014, false, s, gps, epoch));
  ASSERT_TRUE(
      manager.QueueYawAtEpoch(earlier ? -.03 : .03, .1, false, earlier ? start : u, yaw, epoch));
}

void Initialize(fg::GraphManager& manager, bool earlier = false)
{
  manager.SetConsumedMotionBaseline(epoch, start);
  Consume(manager, start, u, 1);
  Admit(manager, earlier);
  ASSERT_EQ(manager.InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Applied);
}

void SameState(const Access::Snapshot& before, const Access::Snapshot& after)
{
  EXPECT_EQ(before.fingerprint, after.fingerprint);
  ASSERT_EQ(before.values.size(), after.values.size());
  for (const auto& value : before.values)
  {
    ASSERT_TRUE(after.values.exists(value.key));
    if (gtsam::Symbol(value.key).chr() == 'b')
      EXPECT_DOUBLE_EQ(value.value.cast<double>(), after.values.at<double>(value.key));
    else
      EXPECT_EQ(value.value.cast<gtsam::Pose2>().matrix(),
                after.values.at<gtsam::Pose2>(value.key).matrix());
  }
}

TEST(SourceBootstrapPhase, SameSourcePairDelegatesWithoutCreatingPrivateFamily)
{
  fg::GraphManager manager(Params(true));
  manager.SetConsumedMotionBaseline(epoch, start);
  Consume(manager, start, u, 1);
  ASSERT_TRUE(manager.QueueGnssAtEpoch(.2, -.1, .014, false, s, gps, epoch));
  ASSERT_TRUE(manager.QueueYawAtEpoch(0., .1, false, s, yaw, epoch));
  ASSERT_EQ(manager.InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Applied);
  const auto initial = Access::Capture(manager);
  EXPECT_FALSE(initial.motion.bootstrap);
  EXPECT_EQ(initial.factors.nrFactors(), 3u);
  EXPECT_EQ(manager.SourceGnssFactorCount(), 1u);
  EXPECT_EQ(manager.SourceYawFactorCount(), 1u);
}

TEST(SourceBootstrapPhase, PrefixOnlySeedAddsItsFirstSuffixAtActualCadence)
{
  fg::GraphManager manager(Params(true));
  manager.SetConsumedMotionBaseline(epoch, start);
  Consume(manager, start, s, 1);
  Admit(manager, true);
  ASSERT_EQ(manager.InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Applied);
  const auto initial = Access::Capture(manager);
  EXPECT_EQ(initial.motion.bootstrap->family->end_ns(), s);
  EXPECT_EQ(initial.motion.bootstrap->family->windows().size(), 1u);
  EXPECT_FALSE(initial.motion.bootstrap->request.suffix_policy_input);
  EXPECT_FALSE(manager.TickSource(due, s, epoch));
  Consume(manager, s, u, 2);
  ASSERT_TRUE(manager.TickSource(due, u, epoch));
  const auto closed = Access::Capture(manager);
  EXPECT_EQ(closed.motion.bootstrap->family->windows().size(), 2u);
  EXPECT_EQ(manager.LatestSnapshot()->node_index, 1u);
  EXPECT_EQ(manager.LatestSnapshot()->source_stamp_ns, u);
}

TEST(SourceBootstrapPhase, PublicSAndFirstActualCadenceVStayConsistentWithOnePolicyCommit)
{
  for (const bool preint : {false, true})
    for (const bool earlier : {false, true})
    {
      SCOPED_TRACE(::testing::Message() << preint << ' ' << earlier);
      auto params = Params(preint);
      fg::GraphManager manager(params);
      Initialize(manager, earlier);
      const auto initial = Access::Capture(manager);
      ASSERT_TRUE(initial.motion.bootstrap);
      EXPECT_TRUE(initial.motion.bootstrap->provisional);
      EXPECT_EQ(initial.motion.committed_ns, s);
      EXPECT_EQ(initial.motion.consumed_ns, u);
      EXPECT_EQ(manager.Stats().total_nodes, 1u);
      EXPECT_EQ(manager.GetAllPoses().size(), 1u);
      EXPECT_GT(manager.LiveNodeCount(), 1u);
      ASSERT_TRUE(manager.GetPose(0));
      ASSERT_TRUE(manager.LatestSnapshot());
      EXPECT_LT(manager.GetPose(0)->localCoordinates(manager.LatestSnapshot()->pose).norm(), 1e-15);
      EXPECT_EQ(manager.LatestSnapshot()->source_stamp_ns, s);
      EXPECT_EQ(manager.LatestSnapshot()->timestamp, 10.);
      EXPECT_EQ(manager.SourceGnssFactorCount(), 1u);
      EXPECT_EQ(manager.SourceYawFactorCount(), 1u);
      EXPECT_EQ(initial.feedback, 0.);
      EXPECT_EQ(initial.policy.slip_window.Size(), 0u);
      EXPECT_FALSE(manager.EstimatePoseAtSource(epoch, s + 1));
      EXPECT_EQ(manager.OldestRetainedSourceNs(epoch), s);
      const auto birth = manager.OriginalHeadingSeed();
      ASSERT_TRUE(birth);
      EXPECT_EQ(birth->heading_provenance,
                fg::HeadingSeedState::HeadingProvenance::CoupledSourceObservations);
      EXPECT_EQ(birth->source_heading_id, yaw);
      EXPECT_EQ(birth->source_heading_ns, earlier ? start : u);
      EXPECT_FALSE(manager.TickSource(schedule, u, epoch));
      EXPECT_FALSE(manager.Tick(11.));  // Legacy cadence cannot bypass the source transaction.
      SameState(initial, Access::Capture(manager));
      Consume(manager, u, v, 2);
      const auto pending = Access::Capture(manager, due);
      const auto expected = fg::SelectProcessWindowPolicy(params, pending.input, initial.policy);
      const auto output = manager.TickSource(due, v, epoch);
      ASSERT_TRUE(output);
      EXPECT_EQ(output->node_index, 1u);
      EXPECT_EQ(output->source_stamp_ns, v);
      EXPECT_EQ(manager.Stats().total_nodes, 2u);
      EXPECT_EQ(manager.GetAllPoses().size(), 2u);
      EXPECT_FALSE(manager.GetPose(2));
      const auto closed = Access::Capture(manager);
      ASSERT_TRUE(closed.motion.bootstrap);
      EXPECT_FALSE(closed.motion.bootstrap->provisional);
      EXPECT_EQ(closed.motion.bootstrap->family->end_ns(), v);
      EXPECT_EQ(closed.motion.bootstrap->request.original_steps.size(), 2u);
      EXPECT_EQ(closed.motion.bootstrap->request.original_steps.front().start_ns, start);
      EXPECT_EQ(closed.motion.bootstrap->request.original_steps.front().gyro.id.sequence, 1u);
      EXPECT_EQ(closed.policy.slip_window.Size(), expected.state.slip_window.Size());
      EXPECT_DOUBLE_EQ(closed.policy.residual_ema, expected.state.residual_ema);
      EXPECT_EQ(closed.policy.hand_push_count, expected.state.hand_push_count);
      EXPECT_EQ(closed.policy.slip_veto_count, expected.state.slip_veto_count);
      EXPECT_DOUBLE_EQ(closed.policy.last_wheel_sigma_x_eff, expected.state.last_wheel_sigma_x_eff);
      EXPECT_EQ(closed.motion.last_committed_gyro->sequence, 2u);
      EXPECT_TRUE(closed.motion.pending_steps.empty());
      EXPECT_EQ(manager.OriginalHeadingSeed(), birth);
      EXPECT_EQ(std::count(closed.factors.begin(), closed.factors.end(), birth->source_position),
                1);
      EXPECT_TRUE(manager.EstimatePoseAtSource(epoch, s + 50000000));
      SameState(closed, Access::Capture(manager));
      EXPECT_FALSE(manager.QueueGnssAtEpoch(0., 0., .1, false, s, gps, epoch));
      EXPECT_FALSE(manager.QueueYawAtEpoch(0., .1, false, earlier ? start : u, yaw, epoch));
      const auto before_recovery = Access::Capture(manager);
      EXPECT_EQ(manager.RecoverInvalidSeedHeading(birth, true, {epoch, v, 99, 0., .1, false}),
                fg::HeadingRecoveryResult::UnsupportedFamily);
      SameState(before_recovery, Access::Capture(manager));
    }
}

TEST(SourceBootstrapPhase, SharedSinglePhysicalRateAcrossSCanPublishAtOriginalU)
{
  fg::GraphManager manager(Params(true));
  Initialize(manager, true);
  const auto initial = Access::Capture(manager);
  ASSERT_TRUE(manager.TickSource(due, u, epoch));
  const auto closed = Access::Capture(manager);
  ASSERT_EQ(closed.motion.bootstrap->family->windows().size(), 2u);
  const auto& windows = closed.motion.bootstrap->family->windows();
  ASSERT_TRUE(windows[0].gyro);
  ASSERT_TRUE(windows[1].gyro);
  ASSERT_EQ(windows[0].gyro->sources.size(), 1u);
  ASSERT_EQ(windows[1].gyro->sources.size(), 1u);
  EXPECT_EQ(windows[0].gyro->sources[0].source.id, windows[1].gyro->sources[0].source.id);
  EXPECT_EQ(manager.LatestSnapshot()->node_index, 1u);
  EXPECT_EQ(manager.SourceYawFactorCount(), 1u);
  EXPECT_EQ(manager.SourceGnssFactorCount(), 1u);
  EXPECT_TRUE(manager.LatestSnapshot()->covariance.allFinite());
  EXPECT_EQ(initial.feedback, 0.);
}

TEST(SourceBootstrapPhase, OverlappingWheelReceiptsCannotAdvanceAcceptedSourceAccumulator)
{
  for (const bool preint : {false, true})
  {
    SCOPED_TRACE(preint);
    fg::GraphManager manager(Params(preint));
    Initialize(manager, true);
    const auto initial = Access::Capture(manager);
    EXPECT_NEAR(initial.input.dt_total, .1, 1e-15);
    // The next wheel callback spans start..V, crossing both public S and
    // already-consumed U. Only the subsequent accepted U..V step owns motion.
    manager.AddWheelTwist(.4, 0., .3, .3);
    const double correction = manager.AddGyroDelta(.3, .1);
    SameState(initial, Access::Capture(manager));
    const fg::ConsumedMotionStep step{epoch,
                                      u,
                                      v,
                                      {{fg::MotionSourceKind::Wheel, 2, v}, {start, v}, .4, 0., .3},
                                      {{fg::MotionSourceKind::Gyro, 2, v}, {u, v}, .3},
                                      correction,
                                      true,
                                      true};
    auto invalid = step;
    ++invalid.epoch;
    EXPECT_FALSE(manager.AddConsumedMotionStep(invalid));
    SameState(initial, Access::Capture(manager));
    ASSERT_TRUE(manager.AddConsumedMotionStep(step));
    const auto accepted = Access::Capture(manager, due);
    EXPECT_NEAR(accepted.input.dt_total, .2, 1e-15);
    EXPECT_NEAR(accepted.input.dx, .2 * .1 + .4 * .1, 1e-15);
    EXPECT_NEAR(accepted.input.dtheta_wheel, .3 * .2, 1e-15);
    EXPECT_NEAR(accepted.input.dtheta_gyro, .3 * .2, 1e-15);
    EXPECT_FALSE(manager.AddConsumedMotionStep(step));
    SameState(accepted, Access::Capture(manager, due));
    ASSERT_TRUE(manager.TickSource(due, v, epoch));
    EXPECT_EQ(manager.LatestSnapshot()->node_index, 1u);
    EXPECT_EQ(manager.LatestSnapshot()->source_stamp_ns, v);
    // Closing the coupled family retains source ownership for ordinary edges.
    manager.AddWheelTwist(.4, 0., .3, .3);
    const double next_correction = manager.AddGyroDelta(.3, .1);
    const auto before_source = Access::Capture(manager);
    EXPECT_DOUBLE_EQ(before_source.input.dt_total, 0.);
    EXPECT_DOUBLE_EQ(before_source.input.dx, 0.);
    EXPECT_DOUBLE_EQ(before_source.input.dtheta_gyro, 0.);
    auto next = step;
    next.start_ns = v;
    next.end_ns = v + 100000000;
    next.wheel.id = {fg::MotionSourceKind::Wheel, 3, next.end_ns};
    next.wheel.support = {v, next.end_ns};
    next.gyro.id = {fg::MotionSourceKind::Gyro, 3, next.end_ns};
    next.gyro.support = {v, next.end_ns};
    next.bias_linearization = next_correction;
    ASSERT_TRUE(manager.AddConsumedMotionStep(next));
    const auto ordinary = Access::Capture(manager);
    EXPECT_NEAR(ordinary.input.dt_total, .1, 1e-15);
    EXPECT_NEAR(ordinary.input.dx, .4 * .1, 1e-15);
    EXPECT_NEAR(ordinary.input.dtheta_gyro, (.3 - next_correction) * .1, 1e-15);
  }
}

TEST(SourceBootstrapPhase, FailedInitialProofAndFailedPromotionRetainEveryPendingInput)
{
  fg::GraphManager manager(Params(true));
  manager.SetConsumedMotionBaseline(epoch, start);
  Consume(manager, start, u, 1);
  Admit(manager, false);
  const auto pending = Access::Capture(manager);
  EXPECT_EQ(manager.InitializeSourceObservationPair(schedule, s, epoch + 1, gps, yaw),
            fg::SourceBootstrapResult::PendingProofUnavailable);
  SameState(pending, Access::Capture(manager));
  ASSERT_EQ(manager.InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Applied);
  Consume(manager, u, v, 2);
  ASSERT_TRUE(manager.QueueGnssAtEpoch(.2, -.1, .1, false, s, 72, epoch));
  manager.QueueYaw(.04, .2);
  Access::CorruptAccumulator(manager, 1.);
  const auto broken = Access::Capture(manager);
  EXPECT_FALSE(manager.TickSource(due, v, epoch));
  SameState(broken, Access::Capture(manager));
  Access::CorruptAccumulator(manager, -1.);
  ASSERT_TRUE(manager.TickSource(due, v, epoch));
  EXPECT_EQ(manager.SourceGnssFactorCount(), 2u);
  EXPECT_EQ(manager.Stats().total_nodes, 2u);

  fg::GraphManager invalid(Params());
  invalid.SetConsumedMotionBaseline(epoch, start);
  Consume(invalid, start, u, 1);
  Admit(invalid, true);
  Access::CorruptHeading(invalid);
  const auto invalid_before = Access::Capture(invalid);
  EXPECT_EQ(invalid.InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Failed);
  SameState(invalid_before, Access::Capture(invalid));
  EXPECT_FALSE(invalid.IsInitialized());
}

TEST(SourceBootstrapPhase, BoundaryAdditionalAndFutureSourceLikelihoodsArePreservedOnce)
{
  fg::GraphManager manager(Params());
  Initialize(manager);
  ASSERT_TRUE(manager.QueueGnssAtEpoch(.2, -.1, .1, false, s, 72, epoch));
  ASSERT_TRUE(manager.QueueYawAtEpoch(0., .2, false, s, 83, epoch));
  ASSERT_TRUE(manager.QueueYawAtEpoch(.06, .2, false, v, 84, epoch));
  ASSERT_TRUE(manager.QueueGnssAtEpoch(1., 2., .2, false, v + 100000000, 73, epoch));
  EXPECT_FALSE(manager.QueueYawAtEpoch(0., .2, false, start, 85, epoch));
  ASSERT_TRUE(manager.TickSource(schedule, u, epoch));
  EXPECT_EQ(manager.SourceGnssFactorCount(), 2u);
  EXPECT_EQ(manager.SourceYawFactorCount(), 2u);
  const auto boundary = Access::Capture(manager);
  ASSERT_EQ(boundary.motion.bootstrap->request.additional_observations.size(), 2u);
  EXPECT_EQ(boundary.motion.observations.size(), 2u);
  Consume(manager, u, v, 2);
  ASSERT_TRUE(manager.TickSource(due, v, epoch));
  EXPECT_EQ(manager.SourceGnssFactorCount(), 2u);
  EXPECT_EQ(manager.SourceYawFactorCount(), 3u);
  const auto closed = Access::Capture(manager);
  EXPECT_EQ(closed.motion.observations.size(), 1u);
  EXPECT_EQ(closed.motion.observations.front().id, 73u);
  ASSERT_TRUE(manager.QueueGnssAtEpoch(.205, -.1, .1, false, s + 50000000, 74, epoch));
  ASSERT_TRUE(manager.TickSource(due, v, epoch));
  EXPECT_EQ(manager.SourceGnssFactorCount(), 3u);
  EXPECT_EQ(manager.SourceYawFactorCount(), 3u);
  const auto delayed = Access::Capture(manager);
  EXPECT_GT(delayed.motion.next_acquisition, closed.motion.next_acquisition);
  EXPECT_EQ(delayed.motion.bootstrap->family->acquisitions().size(),
            closed.motion.bootstrap->family->acquisitions().size() + 1u);
  EXPECT_TRUE(manager.EstimatePoseAtSource(epoch, s + 50000000));
  SameState(delayed, Access::Capture(manager));
}

TEST(SourceBootstrapPhase, FirstCadenceKeepsAdditiveAnchorAndLegacyUnaryTargetsExpiryAndCount)
{
  for (const bool expired : {false, true})
    for (const bool target_s : {false, true})
    {
      fg::GraphManager manager(Params());
      Initialize(manager);
      manager.ForceAnchor(0, gtsam::Pose2(), .2, .2);
      const auto anchored = Access::Capture(manager);
      ASSERT_EQ(anchored.motion.bootstrap->request.extra_factors.size(), 1u);
      const auto anchor = anchored.motion.bootstrap->request.extra_factors[0];
      ASSERT_TRUE(manager.QueueGnss(.2, -.1, .2, false, 0));
      manager.QueueYaw(.03, .2);
      manager.QueueLidarMapXy(gtsam::Vector2(.02, 0.),
                              gtsam::Matrix2::Identity() * .1,
                              false,
                              target_s ? std::optional<uint64_t>(0) : std::nullopt,
                              gtsam::Vector2::Zero(),
                              expired ? 9. : 11.);
      ASSERT_TRUE(manager.TickSource(due, u, epoch));
      const auto closed = Access::Capture(manager);
      EXPECT_EQ(std::count(closed.factors.begin(), closed.factors.end(), anchor), 1);
      size_t legacy_gnss_at_s = 0, legacy_yaw_at_v = 0, lidar_at_target = 0;
      for (const auto& factor : closed.factors)
      {
        if (dynamic_cast<const fg::GnssLeverArmFactor*>(factor.get()) &&
            factor != manager.OriginalHeadingSeed()->source_position &&
            factor->keys().front() == fg::PoseKey(0))
          ++legacy_gnss_at_s;
        if (dynamic_cast<const fg::YawUnaryFactor*>(factor.get()) &&
            factor->keys().front() == fg::PoseKey(1))
          ++legacy_yaw_at_v;
        if (dynamic_cast<const gtsam::PoseTranslationPrior<gtsam::Pose2>*>(factor.get()) &&
            factor->keys().front() == fg::PoseKey(target_s ? 0 : 1))
          ++lidar_at_target;
      }
      EXPECT_EQ(legacy_gnss_at_s, 1u);
      EXPECT_GE(legacy_yaw_at_v, 1u);
      EXPECT_EQ(lidar_at_target, expired ? 0u : 1u);
      EXPECT_EQ(manager.LidarAnchorFactorCount(), expired ? 0u : 1u);
      EXPECT_EQ(manager.SourceGnssFactorCount(), 1u);
      EXPECT_EQ(manager.SourceYawFactorCount(), 1u);
    }
}

TEST(SourceBootstrapPhase, SaveAndRebaseDeferUntilClosureThenRetireWholeFamily)
{
  auto params = Params(true);
  params.max_graph_nodes = 1;
  fg::GraphManager manager(params);
  Initialize(manager, true);
  const auto initial = Access::Capture(manager);
  EXPECT_FALSE(manager.Save("/tmp/fusion_graph_bootstrap_phase_must_not_save"));
  EXPECT_FALSE(manager.NeedsSourceRebase());
  manager.RebaseISAM2();
  SameState(initial, Access::Capture(manager));
  ASSERT_TRUE(manager.TickSource(due, u, epoch));
  EXPECT_TRUE(manager.NeedsSourceRebase());
  manager.RebaseISAM2();
  const auto rebased = Access::Capture(manager);
  EXPECT_FALSE(rebased.motion.bootstrap);
  EXPECT_FALSE(manager.GetPose(0));
  ASSERT_TRUE(manager.GetPose(1));
  EXPECT_EQ(manager.LiveNodeCount(), 1u);
  EXPECT_FALSE(manager.EstimatePoseAtSource(epoch, s));
  EXPECT_EQ(manager.OldestRetainedSourceNs(epoch), u);
}

TEST(SourceBootstrapPhase, RigidReanchorDropsPrivateFamilyAndResumesOrdinaryPendingTail)
{
  fg::GraphManager manager(Params());
  Initialize(manager, true);
  manager.RigidTransformAll(gtsam::Pose2(1., 2., .1), .05, .05);
  const auto rigid = Access::Capture(manager);
  EXPECT_FALSE(rigid.motion.bootstrap);
  EXPECT_EQ(manager.LiveNodeCount(), 1u);
  EXPECT_EQ(rigid.motion.committed_ns, s);
  ASSERT_TRUE(manager.TickSource(due, u, epoch));
  EXPECT_EQ(manager.LatestSnapshot()->node_index, 1u);
  EXPECT_EQ(manager.LatestSnapshot()->source_stamp_ns, u);
  EXPECT_EQ(Access::Capture(manager).motion.edges.size(), 1u);
}

TEST(SourceBootstrapPhase, ResetAndClockEpochDiscardTheEntireProvisionalTransaction)
{
  for (const bool rewind : {false, true})
  {
    fg::GraphManager manager(Params(true));
    Initialize(manager, true);
    if (rewind)
      manager.ResetConsumedMotionEpoch(epoch + 1);
    else
      manager.Reset();
    EXPECT_FALSE(manager.IsInitialized());
    EXPECT_FALSE(manager.LatestSnapshot());
    EXPECT_FALSE(manager.OriginalHeadingSeed());
    EXPECT_TRUE(manager.GetAllPoses().empty());
    EXPECT_EQ(manager.SourceGnssFactorCount(), 0u);
    EXPECT_EQ(manager.SourceYawFactorCount(), 0u);
    EXPECT_FALSE(manager.TickSource(due, u, epoch));
  }
}
}  // namespace
