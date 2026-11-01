// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <atomic>
#include <condition_variable>
#include <future>
#include <iomanip>
#include <map>
#include <numbers>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "graph_motion_state.hpp"
#include <gtest/gtest.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
class GraphManagerHeadingRecoveryTestAccess
{
public:
  struct Snapshot
  {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    TickOutput latest;
    std::string state;
    uint64_t gnss, yaw, next_acquisition, generation;
    bool rebasing, relative;
    size_t pending, mirrored_factors, mirrored_values;
  };

  static Snapshot Capture(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    const auto& m = *manager.motion_;
    std::ostringstream state;
    state << std::hexfloat << manager.heading_seed_.get() << ' ' << manager.next_index_ << ' '
          << manager.last_node_time_s_ << ' ' << manager.current_bias_estimate_ << ' '
          << manager.rebase_in_progress_ << ' ' << manager.rebase_generation_ << ' '
          << manager.ticks_since_cov_ << ' ' << manager.stats_isam_resets_ << ' ' << m.epoch << ' '
          << m.baseline_set << ' ' << m.baseline_ns << ' ' << m.consumed_ns << ' ' << m.committed_ns
          << ' ' << m.schedule_ns << ' ' << m.active_schedule_ns << ' ' << m.active_end.has_value()
          << ' ' << m.active_end.value_or(0) << ' ' << m.revision << ' ' << m.gnss_factors << ' '
          << m.yaw_factors << ' ' << m.next_acquisition << ' ' << m.relative_conditionals;
    const auto& a = manager.accum_;
    state << ' ' << a.dx << ' ' << a.dy << ' ' << a.wheel_travel_m << ' ' << a.dtheta_wheel << ' '
          << a.dtheta_gyro << ' ' << a.dt_total << ' ' << a.max_abs_gyro_rad_per_s << ' '
          << a.gyro_preint_dtheta << ' ' << a.gyro_preint_dt << ' ' << a.gyro_preint_variance;
    for (const auto& [time, index] : manager.node_time_index_)
      state << " cadence " << time << ' ' << index;
    for (const auto& [id, time] : m.observation_ids)
      state << " gps-id " << id << ' ' << time;
    for (const auto& [id, time] : m.yaw_observation_ids)
      state << " yaw-id " << id << ' ' << time;
    for (const auto& observation : m.observations)
      state << " pending " << observation.id << ' ' << observation.source_ns << ' '
            << observation.xy.transpose() << ' ' << observation.sigma << ' ' << observation.robust
            << ' ' << observation.yaw.has_value() << ' ' << observation.yaw.value_or(0.0);
    for (const auto& step : m.pending_steps)
      state << " step " << step.epoch << ' ' << step.start_ns << ' ' << step.end_ns << ' '
            << step.wheel.id.sequence << ' ' << step.gyro.id.sequence << ' ' << step.wheel.vx << ' '
            << step.wheel.vy << ' ' << step.wheel.wz << ' ' << step.gyro.raw_z << ' '
            << step.bias_linearization << ' ' << step.wheel_translation_enabled << ' '
            << step.gyro_rotation_enabled;
    for (const auto& id : m.pending_gyro_ids)
      state << " gyro-id " << id.sequence << ' ' << id.observation_stamp_ns;
    for (const auto& edge : m.edges)
    {
      state << " edge " << edge.owner << ' ' << edge.bridge.get() << ' '
            << static_cast<int>(edge.bridge->mode());
      for (const auto& [key, time] : edge.acquisitions)
        state << " acquisition " << key << ' ' << time;
      for (const auto& conditional : edge.conditionals)
        state << " conditional " << conditional.key << ' ' << conditional.source_ns << ' '
              << conditional.factor.get();
    }
    for (const auto& factor : manager.rebase_pending_factors_)
      state << " mirror-factor " << factor.get();
    for (const auto& value : manager.rebase_pending_values_)
    {
      state << " mirror-value " << value.key << ' ';
      if (gtsam::Symbol(value.key).chr() == 'b')
        state << value.value.cast<double>();
      else
        state << value.value.cast<gtsam::Pose2>().matrix();
    }
    return {manager.isam_->getFactorsUnsafe(),
            manager.isam_->calculateEstimate(),
            *manager.latest_,
            state.str(),
            m.gnss_factors,
            m.yaw_factors,
            m.next_acquisition,
            manager.rebase_generation_,
            manager.rebase_in_progress_,
            m.relative_conditionals,
            m.observations.size(),
            manager.rebase_pending_factors_.size(),
            manager.rebase_pending_values_.size()};
  }

  static void Limit(GraphManager& manager, uint64_t count)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    manager.params_.max_graph_nodes = count;
  }

  static void MissingFamily(GraphManager& manager)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    for (auto& edge : manager.motion_->edges)
      if (!edge.conditionals.empty())
      {
        edge.conditionals.pop_back();
        return;
      }
    throw std::runtime_error("fixture has no acquisition family");
  }

  static void AddFactor(GraphManager& manager, gtsam::NonlinearFactor::shared_ptr factor)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    gtsam::NonlinearFactorGraph graph;
    graph.push_back(std::move(factor));
    if (!manager.ApplyIsamUpdateLocked(graph, {}))
      throw std::runtime_error("fixture factor could not be installed");
  }

  static void AddUnknownAffine(GraphManager& manager)
  {
    const auto snapshot = Capture(manager);
    const auto key = PoseKey(snapshot.latest.node_index);
    const gtsam::JacobianFactor gaussian(key,
                                         gtsam::Matrix3::Identity(),
                                         gtsam::Vector3::Zero(),
                                         gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
    AddFactor(manager, std::make_shared<gtsam::LinearContainerFactor>(gaussian, snapshot.values));
  }

  // Independent correct-seed control: assemble the declared model from the retained actual
  // likelihoods and source histories. This does not invoke recovery or use its output/values.
  // No transaction checks, factor matching, candidate gauge correction, or seed token policy
  // is shared with the operation under test.
  static void AssembleCorrectModel(GraphManager& manager, const RecoveryYawObservation& observation)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    auto& state = *manager.motion_;
    auto values = manager.isam_->calculateEstimate();
    std::map<const gtsam::NonlinearFactor*, gtsam::NonlinearFactor::shared_ptr> replacements;
    for (auto& edge : state.edges)
    {
      auto relative = std::make_shared<ConditionalProcessBridge>(
          edge.bridge->edge(), ConditionalProcessBridge::Mode::RelativeErrors);
      for (auto& record : edge.conditionals)
      {
        const auto inserted = relative->InsertRelative(record.source_ns, record.key, &values);
        replacements.emplace(record.factor.get(), inserted.factor);
        record.factor = inserted.factor;
      }
      edge.bridge = std::move(relative);
    }
    gtsam::NonlinearFactorGraph graph;
    size_t seed_priors = 0;
    for (const auto& factor : manager.isam_->getFactorsUnsafe())
    {
      if (!factor)
        continue;
      const auto* prior = dynamic_cast<const gtsam::PriorFactor<gtsam::Pose2>*>(factor.get());
      if (prior && prior->key() == PoseKey(0))
      {
        ++seed_priors;
        graph.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose2>>(
            PoseKey(0),
            gtsam::Point2(0, 0),
            gtsam::noiseModel::Isotropic::Sigma(2, manager.params_.prior_sigma_xy));
      }
      else if (const auto found = replacements.find(factor.get()); found != replacements.end())
        graph.push_back(found->second);
      else
        graph.push_back(factor);
    }
    if (seed_priors != 1)
      throw std::runtime_error("correct control must contain exactly one original seed");
    auto edge = std::find_if(state.edges.begin(),
                             state.edges.end(),
                             [&](const auto& entry)
                             {
                               return observation.source_ns > entry.bridge->edge().start_ns &&
                                      observation.source_ns < entry.bridge->edge().end_ns;
                             });
    if (edge == state.edges.end())
      throw std::runtime_error("control recovery must be interior");
    const auto key = gtsam::Symbol('a', state.next_acquisition++);
    const auto acquisition = edge->bridge->InsertRelative(observation.source_ns, key, &values);
    if (!acquisition.initial_value)
      throw std::runtime_error("control recovery time unexpectedly aliases another acquisition");
    values.insert(key, *acquisition.initial_value);
    graph.push_back(acquisition.factor);
    edge->acquisitions.emplace(key, observation.source_ns);
    edge->conditionals.push_back({key, observation.source_ns, acquisition.factor});
    graph.emplace_shared<YawUnaryFactor>(key,
                                         observation.yaw,
                                         gtsam::noiseModel::Isotropic::Sigma(1, observation.sigma));
    gtsam::LevenbergMarquardtParams optimizer;
    optimizer.setMaxIterations(200);
    optimizer.setRelativeErrorTol(1e-13);
    optimizer.setAbsoluteErrorTol(1e-13);
    const auto optimized = gtsam::LevenbergMarquardtOptimizer(graph, values, optimizer).optimize();
    gtsam::ISAM2Params parameters;
    parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = std::max(1, manager.params_.isam2_relinearize_skip);
    manager.isam_ = std::make_unique<gtsam::ISAM2>(parameters);
    manager.isam_->update(graph, optimized);
    manager.current_estimate_ = manager.isam_->calculateEstimate();
    manager.estimate_dirty_ = false;
    manager.latest_->pose =
        manager.isam_->calculateEstimate<gtsam::Pose2>(PoseKey(manager.latest_->node_index));
    manager.latest_->covariance =
        manager.isam_->marginalCovariance(PoseKey(manager.latest_->node_index));
    manager.latest_->revision = ++state.revision;
    if (manager.params_.use_imu_preint)
      manager.current_bias_estimate_ =
          manager.isam_->calculateEstimate<double>(gtsam::Symbol('b', manager.latest_->node_index));
    state.relative_conditionals = true;
    state.yaw_observation_ids.emplace(observation.id, observation.source_ns);
    ++state.yaw_factors;
    manager.heading_seed_.reset();
  }

  static void MakeNextAcquisitionUnrepresentable(GraphManager& manager, int64_t time)
  {
    std::lock_guard<std::mutex> lock(manager.mu_);
    auto& edge = manager.motion_->edges.back();
    if (!edge.acquisitions.empty())
      throw std::runtime_error("branch fixture needs an unobserved final edge");
    auto frozen = edge.bridge->edge();
    const auto original = frozen.trajectory_query;
    const auto end = frozen.end_ns;
    frozen.trajectory_query = std::make_shared<const FrozenProcessEdge::TrajectoryQuery>(
        [original, end, time](int64_t stamp)
        {
          auto result = (*original)(stamp);
          if (stamp == end)
            result.mean = result.mean.compose(gtsam::Pose2(0, 0, 0.1));
          if (stamp == time)
            for (auto& [id, jacobian] : result.gyro_jacobians)
              jacobian *= 1e9;
          return result;
        });
    edge.bridge =
        std::make_shared<ConditionalProcessBridge>(std::move(frozen),
                                                   ConditionalProcessBridge::Mode::RelativeErrors);
  }
};
}  // namespace fusion_graph

namespace
{
namespace fg = fusion_graph;
using Access = fg::GraphManagerHeadingRecoveryTestAccess;
using Snapshot = Access::Snapshot;
constexpr int64_t kStep = 100000000;
constexpr int kLast = 12;
constexpr int64_t kRecoveryTime = 325000000;
constexpr uint64_t kRecoveryId = 700;

fg::GraphParams Params(bool bias = false)
{
  fg::GraphParams p;
  p.node_period_s = p.stationary_node_period_s = 0.1;
  p.cov_update_every_n = 1;
  p.adaptive_noise_enabled_gain = 0;
  p.gyro_bias_estimation_enabled = false;
  p.slip_residual_thresh_rad = p.pivot_gate_dtheta_rad = 100;
  p.lever_arm_x = 0.25;
  p.lever_arm_y = 0.08;
  p.use_imu_preint = bias;
  p.max_graph_nodes = 0;
  return p;
}

double Rate(int i)
{
  return i <= 8 ? 0.25 : -0.4;
}

gtsam::Pose2 Truth(double time)
{
  const double first = std::min(time, 0.8), second = std::max(0.0, time - 0.8);
  return gtsam::Pose2::Expmap(gtsam::Vector3(0.3 * first, 0, 0.25 * first))
      .compose(gtsam::Pose2::Expmap(gtsam::Vector3(0.3 * second, 0, -0.4 * second)));
}

fg::RecoveryYawObservation Recovery()
{
  return {1,
          kRecoveryTime,
          kRecoveryId,
          Truth(kRecoveryTime * 1e-9).theta(),
          std::numbers::pi / 180.0,
          false};
}

void Consume(fg::GraphManager& manager, int i, bool commit = true)
{
  const int64_t start = (i - 1) * kStep, end = i * kStep;
  manager.AddWheelTwist(0.3, 0, Rate(i), 0.1);
  manager.AddGyroDelta(Rate(i), 0.1);
  const fg::ConsumedMotionStep step{
      1,
      start,
      end,
      {{fg::MotionSourceKind::Wheel, static_cast<uint64_t>(i), end}, {start, end}, 0.3, 0, Rate(i)},
      {{fg::MotionSourceKind::Gyro, static_cast<uint64_t>(i), end}, {start, end}, Rate(i)},
      0,
      true,
      true};
  ASSERT_TRUE(manager.AddConsumedMotionStep(step));
  if (commit)
  {
    ASSERT_TRUE(manager.TickSource(end, end, 1));
  }
}

void Gnss(fg::GraphManager& manager, int64_t time, uint64_t id)
{
  const auto xy = Truth(time * 1e-9).transformFrom(gtsam::Point2(0.25, 0.08));
  ASSERT_TRUE(manager.QueueGnssAtEpoch(xy.x(), xy.y(), 0.003, false, time, id, 1));
}

void Yaw(fg::GraphManager& manager, int64_t time, uint64_t id)
{
  ASSERT_TRUE(manager.QueueYawAtEpoch(Truth(time * 1e-9).theta(), 0.2, false, time, id, 1));
}

std::unique_ptr<fg::GraphManager> Replay(double initial_yaw, bool bias, bool interleaved)
{
  auto manager = std::make_unique<fg::GraphManager>(Params(bias));
  manager->SetConsumedMotionBaseline(1, 0);
  EXPECT_TRUE(manager->InitializeSource(gtsam::Pose2(0, 0, initial_yaw), 0, 0, 1));
  for (int i = 1; i <= kLast; ++i)
  {
    Consume(*manager, i);
    if (interleaved)
    {
      const auto time = i * kStep - kStep / 2;
      if (i % 2 == 1)
      {
        Gnss(*manager, time, 100 + i);
        EXPECT_TRUE(manager->TickSource(i * kStep + i, i * kStep, 1));
      }
      if (i == 2 || i == 10)
      {
        Yaw(*manager, time, 200 + i);
        EXPECT_TRUE(manager->TickSource(i * kStep + 20 + i, i * kStep, 1));
      }
    }
  }
  if (!interleaved)
  {
    for (int i = 1; i <= kLast; i += 2)
    {
      Gnss(*manager, i * kStep - kStep / 2, 100 + i);
      EXPECT_TRUE(manager->TickSource(kLast * kStep + i, kLast * kStep, 1));
    }
    for (int i : {2, 10})
    {
      Yaw(*manager, i * kStep - kStep / 2, 200 + i);
      EXPECT_TRUE(manager->TickSource(kLast * kStep + 20 + i, kLast * kStep, 1));
    }
  }
  EXPECT_EQ(manager->SourceGnssFactorCount(), 6u);
  EXPECT_EQ(manager->SourceYawFactorCount(), 2u);
  return manager;
}

void SameValues(const gtsam::Values& a, const gtsam::Values& b, double tolerance)
{
  ASSERT_EQ(a.size(), b.size());
  for (const auto& value : a)
  {
    ASSERT_TRUE(b.exists(value.key));
    if (gtsam::Symbol(value.key).chr() == 'b')
    {
      EXPECT_NEAR(value.value.cast<double>(), b.at<double>(value.key), tolerance);
    }
    else if (tolerance == 0.0)
    {
      EXPECT_TRUE((value.value.cast<gtsam::Pose2>().matrix().array() ==
                   b.at<gtsam::Pose2>(value.key).matrix().array())
                      .all());
    }
    else
    {
      EXPECT_LE(
          value.value.cast<gtsam::Pose2>().localCoordinates(b.at<gtsam::Pose2>(value.key)).norm(),
          tolerance);
    }
  }
}

void Unchanged(const Snapshot& before, const Snapshot& after)
{
  EXPECT_EQ(before.state, after.state);
  ASSERT_EQ(before.factors.size(), after.factors.size());
  for (size_t i = 0; i < before.factors.size(); ++i)
    EXPECT_EQ(before.factors[i].get(), after.factors[i].get());
  SameValues(before.values, after.values, 0.0);
  EXPECT_EQ((before.latest.pose.matrix().array() == after.latest.pose.matrix().array()).all(),
            true);
  EXPECT_EQ((before.latest.covariance.array() == after.latest.covariance.array()).all(), true);
  EXPECT_EQ(before.latest.node_index, after.latest.node_index);
  EXPECT_EQ(before.latest.timestamp, after.latest.timestamp);
  EXPECT_EQ(before.latest.source_stamp_ns, after.latest.source_stamp_ns);
  EXPECT_EQ(before.latest.source_epoch, after.latest.source_epoch);
  EXPECT_EQ(before.latest.revision, after.latest.revision);
}

void SameJoint(const Snapshot& control, const Snapshot& candidate)
{
  ASSERT_EQ(control.values.size(), candidate.values.size());
  for (const auto& value : control.values)
    ASSERT_TRUE(candidate.values.exists(value.key));
  SameValues(control.values, candidate.values, 1e-9);
  gtsam::KeyVector keys;
  for (const auto& value : control.values)
    keys.push_back(value.key);
  const auto expected =
      gtsam::Marginals(control.factors, control.values).jointMarginalCovariance(keys).fullMatrix();
  const auto actual = gtsam::Marginals(candidate.factors, candidate.values)
                          .jointMarginalCovariance(keys)
                          .fullMatrix();
  EXPECT_LT((expected - actual).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((control.latest.covariance - candidate.latest.covariance).cwiseAbs().maxCoeff(), 1e-9);
}

void Physical(const Snapshot& snapshot, int last)
{
  EXPECT_LT(Truth(last * 0.1).localCoordinates(snapshot.latest.pose).norm(), 1e-8);
  for (int i = 0; i <= last; ++i)
    EXPECT_LT(
        Truth(i * 0.1).localCoordinates(snapshot.values.at<gtsam::Pose2>(fg::PoseKey(i))).norm(),
        1e-8);
}

fg::HeadingRecoveryResult Apply(fg::GraphManager& manager,
                                const fg::HeadingSeedToken& seed,
                                const fg::RecoveryYawObservation& observation)
{
#ifdef PR950_ADDITIVE_RECOVERY_COUNTERFACTUAL
  // Negative control only: the former additive latest-pose operation cannot retire the
  // disproven prior or replace complete affine families. Never alter the independent oracle.
  const auto pose = manager.LatestSnapshot()->pose;
  const double offset =
      observation.yaw -
      manager.EstimatePoseAtSource(observation.epoch, observation.source_ns)->theta();
  const auto xy = pose.translation();
  manager.ForceAnchor(manager.LatestSnapshot()->node_index,
                      gtsam::Pose2(xy.x(), xy.y(), pose.theta() + offset),
                      0.005,
                      std::numbers::pi / 180.0);
  (void)seed;
  return fg::HeadingRecoveryResult::Applied;
#else
  return manager.RecoverInvalidSeedHeading(seed, true, observation);
#endif
}

TEST(HeadingRecovery, MovingFamiliesMatchIndependentCorrectModelAndContinue)
{
  for (const bool interleaved : {false, true})
    for (const bool bias : {false, true})
    {
      SCOPED_TRACE(::testing::Message() << "interleaved=" << interleaved << " bias=" << bias);
      auto wrong = Replay(std::numbers::pi, bias, interleaved);
      auto correct = Replay(0, bias, interleaved);
      const auto seed = wrong->OriginalHeadingSeed();
      ASSERT_TRUE(seed);
      const auto observation = Recovery();
      // Inputs beyond the captured frontier are admitted before recovery, and must survive it.
      for (auto* manager : {wrong.get(), correct.get()})
      {
        Gnss(*manager, 1350000000, 800);
        Yaw(*manager, 1350000000, 801);
      }
      const auto before = Access::Capture(*wrong);
      Access::AssembleCorrectModel(*correct, observation);
      ASSERT_EQ(Apply(*wrong, seed, observation), fg::HeadingRecoveryResult::Applied);
      const auto repaired = Access::Capture(*wrong);
      Physical(repaired, kLast);
      SameJoint(Access::Capture(*correct), repaired);
      EXPECT_EQ(repaired.latest.source_stamp_ns, before.latest.source_stamp_ns);
      EXPECT_EQ(repaired.latest.source_epoch, before.latest.source_epoch);
      EXPECT_EQ(repaired.latest.node_index, before.latest.node_index);
      EXPECT_EQ(repaired.latest.timestamp, before.latest.timestamp);
      EXPECT_GT(repaired.latest.revision, before.latest.revision);
      EXPECT_EQ(repaired.pending, 2u);
      EXPECT_EQ(repaired.gnss, 6u);
      EXPECT_EQ(repaired.yaw, 3u);
      EXPECT_TRUE(repaired.relative);
      EXPECT_FALSE(wrong->OriginalHeadingSeed());
      // A later old-edge interior query allocates nothing, and a subsequent observation
      // does allocate under the new model. Then new cadence edges use the same model.
      const auto queried_before = Access::Capture(*wrong);
      const auto historical = wrong->EstimatePoseAtSource(1, 775000000);
      ASSERT_TRUE(historical);
      EXPECT_LT(Truth(0.775).localCoordinates(*historical).norm(), 1e-8);
      Unchanged(queried_before, Access::Capture(*wrong));
      for (auto* manager : {wrong.get(), correct.get()})
      {
        Gnss(*manager, 775000000, 802);
        ASSERT_TRUE(manager->TickSource(kLast * kStep + 100, kLast * kStep, 1));
        Consume(*manager, 13);
        Gnss(*manager, 1250000000, 803);
        ASSERT_TRUE(manager->TickSource(13 * kStep + 100, 13 * kStep, 1));
        Consume(*manager, 14);
      }
      const auto continued = Access::Capture(*wrong);
      Physical(continued, 14);
      SameJoint(Access::Capture(*correct), continued);
      EXPECT_EQ(continued.gnss, 9u);
      EXPECT_EQ(continued.yaw, 4u);
      EXPECT_EQ(continued.pending, 0u);
      EXPECT_TRUE(continued.relative);
      EXPECT_FALSE(wrong->QueueYawAtEpoch(
          observation.yaw, observation.sigma, false, observation.source_ns, observation.id, 1));
      EXPECT_FALSE(wrong->QueueYawAtEpoch(Truth(1.35).theta(), 0.2, false, 1350000000, 801, 1));
      const auto stable = Access::Capture(*wrong);
      EXPECT_FALSE(wrong->TickSource(14 * kStep + 100, 14 * kStep, 1));
      Unchanged(stable, Access::Capture(*wrong));
    }
}

TEST(HeadingRecovery, QueuedAndDirectSameIdentityHaveSamePosterior)
{
  auto direct = Replay(std::numbers::pi, true, true);
  auto queued = Replay(std::numbers::pi, true, true);
  const auto observation = Recovery();
  ASSERT_TRUE(queued->QueueYawAtEpoch(observation.yaw,
                                      observation.sigma,
                                      observation.robust,
                                      observation.source_ns,
                                      observation.id,
                                      observation.epoch));
  ASSERT_EQ(direct->RecoverInvalidSeedHeading(direct->OriginalHeadingSeed(), true, observation),
            fg::HeadingRecoveryResult::Applied);
  ASSERT_EQ(queued->RecoverInvalidSeedHeading(queued->OriginalHeadingSeed(), true, observation),
            fg::HeadingRecoveryResult::Applied);
  SameJoint(Access::Capture(*direct), Access::Capture(*queued));
  const auto snapshot = Access::Capture(*queued);
  EXPECT_EQ(snapshot.pending, 0u);
  EXPECT_EQ(snapshot.yaw, 3u);
  EXPECT_FALSE(queued->TickSource(kLast * kStep + 100, kLast * kStep, 1));
  Unchanged(snapshot, Access::Capture(*queued));
}

TEST(HeadingRecovery, DuplicateAppliedAndConflictingPendingPayloadsDoNotMutate)
{
  for (const bool applied : {false, true})
  {
    SCOPED_TRACE(applied);
    auto manager = Replay(std::numbers::pi, false, true);
    const auto seed = manager->OriginalHeadingSeed();
    auto observation = Recovery();
    ASSERT_TRUE(manager->QueueYawAtEpoch(observation.yaw,
                                         observation.sigma,
                                         observation.robust,
                                         observation.source_ns,
                                         observation.id,
                                         observation.epoch));
    if (applied)
      ASSERT_TRUE(manager->TickSource(kLast * kStep + 100, kLast * kStep, 1));
    else
      observation.yaw += 0.1;
    const auto before = Access::Capture(*manager);
    EXPECT_EQ(manager->RecoverInvalidSeedHeading(seed, true, observation),
              applied ? fg::HeadingRecoveryResult::DuplicateObservation
                      : fg::HeadingRecoveryResult::ObservationConflict);
    Unchanged(before, Access::Capture(*manager));
  }
}

TEST(HeadingRecovery, UnsupportedInputsPreserveGraphSnapshotAndPendingInputs)
{
  for (int scenario = 0; scenario < 6; ++scenario)
  {
    SCOPED_TRACE(scenario);
    auto manager = Replay(scenario == 0 ? 0 : std::numbers::pi, true, true);
    auto seed = manager->OriginalHeadingSeed();
    auto expected = fg::HeadingRecoveryResult::SeedNotInvalid;
    if (scenario == 1)
    {
      manager->Reset();
      manager->SetConsumedMotionBaseline(1, 0);
      ASSERT_TRUE(manager->InitializeSource(gtsam::Pose2(), 0, 0, 1));
      ASSERT_NE(seed, manager->OriginalHeadingSeed());
      expected = fg::HeadingRecoveryResult::SeedUnavailable;
    }
    if (scenario == 2)
    {
      Access::Limit(*manager, 8);
      manager->RebaseISAM2();
      ASSERT_FALSE(manager->GetPose(0));
      expected = fg::HeadingRecoveryResult::UnsupportedPrefix;
    }
    if (scenario == 3)
    {
      Access::AddUnknownAffine(*manager);
      expected = fg::HeadingRecoveryResult::UnsupportedPrefix;
    }
    if (scenario == 4)
    {
      Access::MissingFamily(*manager);
      expected = fg::HeadingRecoveryResult::UnsupportedFamily;
    }
    if (scenario == 5)
    {
      Consume(*manager, 13, false);
      expected = fg::HeadingRecoveryResult::UnsupportedSource;
    }
    Gnss(*manager, 1500000000, 810);
    Yaw(*manager, 1500000000, 811);
    const auto before = Access::Capture(*manager);
    EXPECT_EQ(manager->RecoverInvalidSeedHeading(seed, scenario != 0, Recovery()), expected);
    Unchanged(before, Access::Capture(*manager));
  }
}

TEST(HeadingRecovery, UnsupportedRelativeObservationStaysPendingAndDoesNotBlockOtherData)
{
  auto manager = Replay(std::numbers::pi, true, true);
  ASSERT_EQ(manager->RecoverInvalidSeedHeading(manager->OriginalHeadingSeed(), true, Recovery()),
            fg::HeadingRecoveryResult::Applied);
  constexpr int64_t unsupported_time = 1150000000;
  Access::MakeNextAcquisitionUnrepresentable(*manager, unsupported_time);
  Yaw(*manager, unsupported_time, 900);
  const auto before = Access::Capture(*manager);
  EXPECT_FALSE(manager->TickSource(kLast * kStep + 100, kLast * kStep, 1));
  Unchanged(before, Access::Capture(*manager));
  EXPECT_TRUE(manager->IsInitialized());
  Yaw(*manager, kLast * kStep, 901);
  ASSERT_TRUE(manager->TickSource(kLast * kStep + 200, kLast * kStep, 1));
  const auto after = Access::Capture(*manager);
  EXPECT_TRUE(manager->IsInitialized());
  EXPECT_EQ(after.latest.source_stamp_ns, before.latest.source_stamp_ns);
  EXPECT_EQ(after.pending, 1u);
  EXPECT_EQ(after.yaw, before.yaw + 1);
  EXPECT_EQ(after.factors.nrFactors(), before.factors.nrFactors() + 1);
  EXPECT_GT(after.latest.revision, before.latest.revision);
  EXPECT_FALSE(manager->QueueYawAtEpoch(Truth(1.15).theta(), 0.2, false, unsupported_time, 900, 1));
}

// Gate a real factor linearization in phase 2 of RebaseISAM2; no scheduling sleeps.
class RebaseGate
{
public:
  void Arm()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = true;
  }
  void Pause()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!armed_)
      return;
    armed_ = false;
    entered_ = true;
    changed_.notify_all();
    if (!changed_.wait_for(lock,
                           std::chrono::seconds(5),
                           [&]
                           {
                             return released_;
                           }))
      throw std::runtime_error("heading recovery rebase gate timed out");
  }
  bool Wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock,
                             std::chrono::seconds(5),
                             [&]
                             {
                               return entered_;
                             });
  }
  void Release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    changed_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool armed_ = false, entered_ = false, released_ = false;
};

class GatedPrior : public gtsam::PriorFactor<gtsam::Pose2>
{
public:
  GatedPrior(gtsam::Key key,
             const gtsam::Pose2& pose,
             std::shared_ptr<RebaseGate> gate,
             std::shared_ptr<std::atomic<bool>> fail)
      : gtsam::PriorFactor<gtsam::Pose2>(key, pose, gtsam::noiseModel::Isotropic::Sigma(3, 0.2)),
        gate_(std::move(gate)),
        fail_(std::move(fail))
  {
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override
  {
    gate_->Pause();
    if (fail_->load())
      throw std::runtime_error("injected candidate linearization failure");
    return gtsam::PriorFactor<gtsam::Pose2>::linearize(values);
  }

private:
  std::shared_ptr<RebaseGate> gate_;
  std::shared_ptr<std::atomic<bool>> fail_;
};

TEST(HeadingRecovery, CandidateFailurePreservesWorkerBuffersAndSuccessCancelsStaleWorker)
{
  for (const bool fail_candidate : {false, true})
  {
    SCOPED_TRACE(fail_candidate);
    auto manager = Replay(std::numbers::pi, true, true);
    const auto seed = manager->OriginalHeadingSeed();
    auto gate = std::make_shared<RebaseGate>();
    auto fail = std::make_shared<std::atomic<bool>>(false);
    Access::AddFactor(*manager,
                      std::make_shared<GatedPrior>(
                          fg::PoseKey(kLast), manager->LatestSnapshot()->pose, gate, fail));
    gate->Arm();
    auto worker = std::async(std::launch::async,
                             [&]
                             {
                               manager->RebaseISAM2();
                             });
    ASSERT_TRUE(gate->Wait());
    Gnss(*manager, 775000000, 950);
    ASSERT_TRUE(manager->TickSource(kLast * kStep + 100, kLast * kStep, 1));
    fail->store(fail_candidate);
    const auto before = Access::Capture(*manager);
    ASSERT_TRUE(before.rebasing);
    ASSERT_GT(before.mirrored_factors, 0u);
    ASSERT_GT(before.mirrored_values, 0u);
    EXPECT_EQ(manager->RecoverInvalidSeedHeading(seed, true, Recovery()),
              fail_candidate ? fg::HeadingRecoveryResult::Failed
                             : fg::HeadingRecoveryResult::Applied);
    const auto after = Access::Capture(*manager);
    if (fail_candidate)
      Unchanged(before, after);
    else
    {
      EXPECT_FALSE(after.rebasing);
      EXPECT_EQ(after.generation, before.generation + 1);
      EXPECT_EQ(after.mirrored_factors, 0u);
      EXPECT_EQ(after.mirrored_values, 0u);
      EXPECT_TRUE(after.relative);
    }
    fail->store(false);
    gate->Release();
    ASSERT_EQ(worker.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    EXPECT_NO_THROW(worker.get());
    const auto completed = Access::Capture(*manager);
    if (!fail_candidate)
      Unchanged(after, completed);
    else
    {
      EXPECT_FALSE(completed.rebasing);
      EXPECT_EQ(completed.gnss, before.gnss);
      EXPECT_EQ(completed.latest.source_stamp_ns, before.latest.source_stamp_ns);
      EXPECT_EQ(completed.latest.revision, before.latest.revision);
      EXPECT_EQ(completed.values.size(), before.values.size());
      EXPECT_TRUE(completed.values.exists(gtsam::Symbol('a', before.next_acquisition - 1)));
      EXPECT_NO_THROW(completed.values.at<double>(gtsam::Symbol('b', kLast)));
      EXPECT_EQ(manager->OriginalHeadingSeed(), seed);
    }
  }
}

TEST(HeadingRecovery, LinkedPosePriorPositionCostSeparatesFromYaw)
{
  const gtsam::Pose2 seed(1.0, -2.0, 0.7);
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.4));
  const gtsam::PriorFactor<gtsam::Pose2> prior(fg::PoseKey(0), seed, noise);
  const gtsam::PoseTranslationPrior<gtsam::Pose2> position(
      fg::PoseKey(0), seed.translation(), gtsam::noiseModel::Isotropic::Sigma(2, 0.05));
  for (const auto& delta : {gtsam::Vector3(0.3, -0.2, 1.1), gtsam::Vector3(-0.4, 0.1, -2.2)})
  {
    gtsam::Values values;
    values.insert(fg::PoseKey(0),
                  gtsam::Pose2(seed.x() + delta.x(),
                               seed.y() + delta.y(),
                               seed.theta() + delta.z()));
    const auto residual = prior.unwhitenedError(values);
    EXPECT_NEAR(residual.head<2>().squaredNorm(), delta.head<2>().squaredNorm(), 1e-12);
    EXPECT_NEAR(0.5 * residual.head<2>().squaredNorm() / (0.05 * 0.05),
                position.error(values),
                1e-10);
  }
}

}  // namespace
