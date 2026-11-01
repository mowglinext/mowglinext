// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <iomanip>
#include <numbers>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "source_bootstrap_state.hpp"
#include <gtest/gtest.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
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
    std::shared_ptr<SourceBootstrapState> bootstrap;
    size_t pending;
    uint64_t next_acquisition;
    std::string state;
  };
  static Snapshot Capture(GraphManager& g)
  {
    const auto& m = *g.motion_;
    const auto& a = g.accum_;
    std::ostringstream s;
    s << std::hexfloat << g.isam_.get() << ' ' << g.motion_.get() << ' ' << g.heading_seed_.get()
      << ' ' << m.bootstrap.get() << ' ' << g.next_index_ << ' ' << g.last_node_time_s_ << ' '
      << g.current_bias_estimate_ << ' ' << m.committed_ns << ' ' << m.consumed_ns << ' '
      << m.schedule_ns << ' ' << m.revision << ' ' << m.gnss_factors << ' ' << m.yaw_factors << ' '
      << m.next_acquisition << ' ' << m.relative_conditionals << ' ' << g.rebase_in_progress_ << ' '
      << g.rebase_generation_ << ' ' << g.rebase_pending_factors_.size() << ' '
      << g.rebase_pending_values_.size() << ' ' << a.dx << ' ' << a.dy << ' ' << a.wheel_travel_m
      << ' ' << a.dt_total << ' ' << a.dtheta_gyro << ' ' << a.dtheta_wheel << ' '
      << a.gyro_preint_dtheta << ' ' << a.gyro_preint_dt << ' ' << a.gyro_preint_variance << ' '
      << g.residual_ema_ << ' ' << g.slip_window_.Size() << ' ' << g.stats_hand_push_ << ' '
      << g.stats_slip_veto_;
    for (const auto& o : m.observations)
      s << " obs " << o.id << ' ' << o.source_ns << ' ' << o.yaw.value_or(0.) << ' ' << o.sigma
        << ' ' << o.robust;
    for (const auto& [id, time] : m.yaw_observation_ids)
      s << " yaw-id " << id << ' ' << time;
    for (const auto& f : g.isam_->getFactorsUnsafe())
      s << " factor " << f.get();
    if (m.bootstrap)
    {
      s << " family " << m.bootstrap->family.get();
      for (const auto& f : m.bootstrap->required_source_factors)
        s << " required " << f.get();
      for (const auto& a : m.bootstrap->family->acquisitions())
        s << " acquisition " << a.key << ' ' << a.source_ns;
      if (m.bootstrap->live_birth_heading)
      {
        const auto& b = *m.bootstrap->live_birth_heading;
        s << " birth " << b.id << ' ' << b.source_ns << ' ' << b.key << ' ' << b.factor.get();
      }
    }
    return {g.isam_->getFactorsUnsafe(),
            g.isam_->calculateEstimate(),
            *g.latest_,
            m.bootstrap,
            m.observations.size(),
            m.next_acquisition,
            s.str()};
  }

  // Independent transaction control, not a second recovery implementation:
  // start from the correct global gauge, remove its sole birth likelihood,
  // preserve the entire original topology/validated shared law, then add R.
  // The correct birth heading is an initial guess only; its information is absent.
  static void AssembleReference(GraphManager& g, const RecoveryYawObservation& o)
  {
    auto motion = std::make_shared<GraphMotionState>(*g.motion_);
    auto phase = std::make_shared<SourceBootstrapState>(*motion->bootstrap);
    phase->family = std::make_shared<SharedSourceProcessFamily>(*phase->family);
    motion->bootstrap = phase;
    auto values = g.isam_->calculateEstimate();
    gtsam::NonlinearFactorGraph factors;
    size_t removed = 0;
    for (const auto& f : g.isam_->getFactorsUnsafe())
      if (dynamic_cast<const YawUnaryFactor*>(f.get()))
        ++removed;
      else
        factors.push_back(f);
    ASSERT_EQ(removed, 1u);
    const auto insertion =
        phase->family->Insert(o.source_ns, gtsam::Symbol('a', motion->next_acquisition), &values);
    if (insertion.initial_value)
    {
      ++motion->next_acquisition;
      values.insert(insertion.key, *insertion.initial_value);
      factors.push_back(insertion.factor);
      phase->auxiliary_poses.emplace(insertion.key, o.source_ns);
    }
    factors.emplace_shared<YawUnaryFactor>(insertion.key,
                                           o.yaw,
                                           gtsam::noiseModel::Isotropic::Sigma(1, o.sigma));
    gtsam::LevenbergMarquardtParams optimizer;
    optimizer.setMaxIterations(200);
    optimizer.setRelativeErrorTol(1e-13);
    optimizer.setAbsoluteErrorTol(1e-13);
    const auto optimized =
        gtsam::LevenbergMarquardtOptimizer(factors, values, optimizer).optimize();
    gtsam::ISAM2Params p;
    p.optimizationParams = gtsam::ISAM2GaussNewtonParams(.001);
    p.relinearizeThreshold = .05;
    p.relinearizeSkip = std::max(1, g.params_.isam2_relinearize_skip);
    auto solver = std::make_unique<gtsam::ISAM2>(p);
    solver->update(factors, optimized);
    g.isam_.swap(solver);
    g.current_estimate_ = g.isam_->calculateEstimate();
    g.estimate_dirty_ = false;
    phase->required_source_factors = factors;
    phase->live_birth_heading.reset();
    motion->relative_conditionals = true;
    motion->yaw_observation_ids.emplace(o.id, o.source_ns);
    ++motion->yaw_factors;
    g.motion_ = std::move(motion);
    g.latest_->pose = g.current_estimate_.at<gtsam::Pose2>(PoseKey(g.latest_->node_index));
    g.latest_->covariance = g.isam_->marginalCovariance(PoseKey(g.latest_->node_index));
    g.latest_->revision = ++g.motion_->revision;
    if (g.params_.use_imu_preint)
      g.current_bias_estimate_ =
          g.current_estimate_.at<double>(gtsam::Symbol('b', g.latest_->node_index));
    g.heading_seed_.reset();
  }

  static void DamageLineage(GraphManager& g)
  {
    // A required pointer absent from the live solver is unproven lineage.
    g.motion_->bootstrap = std::make_shared<SourceBootstrapState>(*g.motion_->bootstrap);
    g.motion_->bootstrap->required_source_factors.push_back(
        g.motion_->bootstrap->required_source_factors[0]->clone());
  }
  static bool RebaseCancelled(const GraphManager& g)
  {
    return !g.rebase_in_progress_ && g.rebase_pending_factors_.empty() &&
           g.rebase_pending_values_.empty();
  }
  static void RetirePrefix(GraphManager& g)
  {
    g.params_.max_graph_nodes = 1;
    g.RebaseISAM2();
  }
  static void AddFactor(GraphManager& g, const gtsam::NonlinearFactor::shared_ptr& factor)
  {
    gtsam::NonlinearFactorGraph factors;
    factors.push_back(factor);
    if (!g.ApplyIsamUpdateLocked(factors, {}))
      throw std::runtime_error("fixture update failed");
  }
  static void FakeRebase(GraphManager& g)
  {
    g.rebase_in_progress_ = true;
    ++g.rebase_generation_;
    g.rebase_pending_factors_.push_back(g.isam_->getFactorsUnsafe()[0]);
  }
};
}  // namespace fusion_graph

namespace
{
namespace fg = fusion_graph;
using Access = fg::GraphManagerHeadingRecoveryTestAccess;
constexpr uint64_t epoch = 19, gps = 71, yaw = 82;
constexpr int64_t t = 1000000000, s = 1100000000, u = 1200000000, v = 1300000000;
constexpr int64_t schedule = 10000000000, due = 10100000000;

fg::GraphParams Params(bool bias, bool lever)
{
  fg::GraphParams p;
  p.use_imu_preint = bias;
  p.gyro_bias_estimation_enabled = false;
  p.stationary_node_period_s = .1;
  p.cov_update_every_n = 1;
  p.lever_arm_x = lever ? .2 : 0.;
  p.lever_arm_y = lever ? -.1 : 0.;
  return p;
}

void Consume(fg::GraphManager& g,
             int64_t begin,
             int64_t end,
             uint64_t id,
             double speed = 0.,
             double rate = 0.)
{
  const double dt = (end - begin) * 1e-9;
  g.AddWheelTwist(speed, 0., rate, dt);
  const double bias = g.AddGyroDelta(rate, dt);
  ASSERT_TRUE(g.AddConsumedMotionStep(
      {epoch,
       begin,
       end,
       {{fg::MotionSourceKind::Wheel, id, end}, {begin, end}, speed, 0., rate},
       {{fg::MotionSourceKind::Gyro, id, end}, {begin, end}, rate},
       bias,
       true,
       true}));
}

std::unique_ptr<fg::GraphManager> Replay(bool bias,
                                         bool earlier,
                                         bool lever,
                                         double heading,
                                         bool commit = true,
                                         bool extend = true,
                                         bool moving = false)
{
  const auto p = Params(bias, lever);
  auto g = std::make_unique<fg::GraphManager>(p);
  g->SetConsumedMotionBaseline(epoch, t);
  Consume(*g, t, u, 1, moving ? .2 : 0., moving ? .3 : 0.);
  EXPECT_TRUE(g->QueueGnssAtEpoch(p.lever_arm_x, p.lever_arm_y, .014, false, s, gps, epoch));
  const int64_t heading_time = earlier ? t : u;
  const double heading_at_source = heading + (moving ? .3 * (heading_time - s) * 1e-9 : 0.);
  EXPECT_TRUE(g->QueueYawAtEpoch(heading_at_source, .1, false, heading_time, yaw, epoch));
  EXPECT_EQ(g->InitializeSourceObservationPair(schedule, s, epoch, gps, yaw),
            fg::SourceBootstrapResult::Applied);
  if (commit)
  {
    if (extend)
      Consume(*g, u, v, 2, moving ? .2 : 0., moving ? .3 : 0.);
    EXPECT_TRUE(g->TickSource(due, extend ? v : u, epoch));
  }
  return g;
}

gtsam::Pose2 Truth(int64_t source, bool moving)
{
  const double dt = (source - s) * 1e-9;
  return moving ? gtsam::Pose2::Expmap(gtsam::Vector3(.2 * dt, 0., .3 * dt)) : gtsam::Pose2();
}

fg::RecoveryYawObservation Observation(bool interior, bool moving = false)
{
  const int64_t source = interior ? 1225000000 : v;
  return {epoch, source, 99, Truth(source, moving).theta(), .08, false};
}

gtsam::Matrix FullCovariance(const Access::Snapshot& snapshot)
{
  gtsam::Ordering ordering;
  std::map<gtsam::Key, int> offset, dimension;
  int size = 0;
  for (const auto& value : snapshot.values)
  {
    ordering.push_back(value.key);
    offset.emplace(value.key, size);
    dimension.emplace(value.key, value.value.dim());
    size += value.value.dim();
  }
  const auto net = snapshot.factors.linearize(snapshot.values)
                       ->eliminateSequential(ordering, gtsam::EliminateQR);
  auto r = gtsam::Matrix::Zero(size, size).eval();
  auto sigmas = gtsam::Vector::Ones(size).eval();
  int total_rows = 0;
  for (const auto& conditional : *net)
  {
    EXPECT_EQ(conditional->nrFrontals(), 1u);
    const auto key = *conditional->begin();
    const int row = offset.at(key), rows = conditional->rows();
    EXPECT_EQ(rows, dimension.at(key));
    for (const auto parent : conditional->keys())
      r.block(row, offset.at(parent), rows, dimension.at(parent)) = conditional->getA(parent);
    if (conditional->get_model())
      sigmas.segment(row, rows) = conditional->get_model()->sigmas();
    total_rows += rows;
  }
  EXPECT_EQ(total_rows, size);
  EXPECT_TRUE((r.triangularView<Eigen::StrictlyLower>().toDenseMatrix().array() == 0.).all());
  const auto response =
      r.triangularView<Eigen::Upper>().solve(sigmas.asDiagonal().toDenseMatrix()).eval();
  EXPECT_TRUE(response.allFinite());
  return response * response.transpose();
}

void SameJoint(const Access::Snapshot& a, const Access::Snapshot& b)
{
  ASSERT_EQ(a.values.size(), b.values.size());
  EXPECT_EQ(a.next_acquisition, b.next_acquisition);
  for (const auto& value : a.values)
  {
    ASSERT_TRUE(b.values.exists(value.key));
    if (gtsam::Symbol(value.key).chr() == 'b')
      EXPECT_NEAR(value.value.cast<double>(), b.values.at<double>(value.key), 1e-9);
    else
      EXPECT_LT(value.value.cast<gtsam::Pose2>()
                    .localCoordinates(b.values.at<gtsam::Pose2>(value.key))
                    .norm(),
                1e-9);
  }
  EXPECT_LT((FullCovariance(a) - FullCovariance(b)).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((a.latest.covariance - b.latest.covariance).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_NEAR(a.factors.error(a.values), b.factors.error(b.values), 1e-9);
}

void Unchanged(const Access::Snapshot& a, const Access::Snapshot& b)
{
  EXPECT_EQ(a.state, b.state);
  EXPECT_EQ(a.latest.revision, b.latest.revision);
  EXPECT_EQ(a.latest.source_stamp_ns, b.latest.source_stamp_ns);
  EXPECT_EQ((a.latest.pose.matrix().array() == b.latest.pose.matrix().array()).all(), true);
  EXPECT_EQ((a.latest.covariance.array() == b.latest.covariance.array()).all(), true);
  ASSERT_EQ(a.values.size(), b.values.size());
  for (const auto& value : a.values)
  {
    ASSERT_TRUE(b.values.exists(value.key));
    if (gtsam::Symbol(value.key).chr() == 'b')
      EXPECT_EQ(value.value.cast<double>(), b.values.at<double>(value.key));
    else
      EXPECT_EQ((value.value.cast<gtsam::Pose2>().matrix().array() ==
                 b.values.at<gtsam::Pose2>(value.key).matrix().array())
                    .all(),
                true);
  }
}

TEST(CoupledHeadingRecovery, ExactBirthRetirementMatchesCompleteJointAndContinues)
{
  for (const bool bias : {false, true})
    for (const bool earlier : {false, true})
      for (const bool lever : {false, true})
        for (const bool interior : {false, true})
          for (const bool moving : {false, true})
          {
            SCOPED_TRACE(::testing::Message() << bias << ' ' << earlier << ' ' << lever << ' '
                                              << interior << ' ' << moving);
            auto wrong = Replay(bias, earlier, lever, std::numbers::pi, true, true, moving);
            auto control = Replay(bias, earlier, lever, 0., true, true, moving);
            for (auto* g : {wrong.get(), control.get()})
            {
              const auto p = Params(bias, lever);
              const auto future = Truth(v + 200000000, moving);
              const auto antenna =
                  future.transformFrom(gtsam::Point2(p.lever_arm_x, p.lever_arm_y));
              ASSERT_TRUE(g->QueueGnssAtEpoch(
                  antenna.x(), antenna.y(), .1, false, v + 200000000, 901, epoch));
              ASSERT_TRUE(g->QueueYawAtEpoch(future.theta(), .1, false, v + 200000000, 902, epoch));
            }
            const auto token = wrong->OriginalHeadingSeed();
            const auto before = Access::Capture(*wrong);
            const auto live_yaw = before.bootstrap->live_birth_heading->factor;
            const auto observation = Observation(interior, moving);
            Access::AssembleReference(*control, observation);
            ASSERT_EQ(wrong->RecoverInvalidSeedHeading(token, true, observation),
                      fg::HeadingRecoveryResult::Applied);
            const auto repaired = Access::Capture(*wrong);
            SameJoint(Access::Capture(*control), repaired);
            EXPECT_TRUE(repaired.values.exists(before.bootstrap->live_birth_heading->key));
            for (const auto& factor : before.bootstrap->required_source_factors)
              if (factor != live_yaw)
                EXPECT_EQ(std::count(repaired.factors.begin(), repaired.factors.end(), factor), 1);
            EXPECT_EQ(std::count(repaired.factors.begin(), repaired.factors.end(), live_yaw), 0);
            EXPECT_EQ(std::count(repaired.factors.begin(),
                                 repaired.factors.end(),
                                 token->source_position),
                      1);
            EXPECT_EQ(repaired.latest.node_index, before.latest.node_index);
            EXPECT_EQ(repaired.latest.source_stamp_ns, before.latest.source_stamp_ns);
            EXPECT_EQ(repaired.latest.timestamp, before.latest.timestamp);
            EXPECT_GT(repaired.latest.revision, before.latest.revision);
            EXPECT_EQ(wrong->SourceGnssFactorCount(), 1u);
            EXPECT_EQ(wrong->SourceYawFactorCount(), 2u);
            EXPECT_EQ(repaired.pending, 2u);
            EXPECT_FALSE(wrong->OriginalHeadingSeed());
            EXPECT_EQ(wrong->RecoverInvalidSeedHeading(token, true, observation),
                      fg::HeadingRecoveryResult::SeedUnavailable);
            EXPECT_TRUE(wrong->EstimatePoseAtSource(epoch, earlier ? s : v));
            for (auto* g : {wrong.get(), control.get()})
            {
              const auto p = Params(bias, lever);
              const auto first_antenna =
                  Truth(1175000000, moving)
                      .transformFrom(gtsam::Point2(p.lever_arm_x, p.lever_arm_y));
              ASSERT_TRUE(g->QueueGnssAtEpoch(
                  first_antenna.x(), first_antenna.y(), .03, false, 1175000000, 100, epoch));
              ASSERT_TRUE(g->TickSource(due + 1, v, epoch));
              Consume(*g, v, v + 100000000, 3, .2, .3);
              ASSERT_TRUE(g->TickSource(due + 100000000, v + 100000000, epoch));
              const auto second_antenna =
                  Truth(1125000000, moving)
                      .transformFrom(gtsam::Point2(p.lever_arm_x, p.lever_arm_y));
              ASSERT_TRUE(g->QueueGnssAtEpoch(
                  second_antenna.x(), second_antenna.y(), .03, false, 1125000000, 101, epoch));
              ASSERT_TRUE(g->TickSource(due + 100000001, v + 100000000, epoch));
            }
            SameJoint(Access::Capture(*control), Access::Capture(*wrong));
          }
}

TEST(CoupledHeadingRecovery, FalseInvalidationPendingTailLineageAndQueuedConflictAreNonmutating)
{
  for (int scenario = 0; scenario < 7; ++scenario)
  {
    auto g = Replay(true, true, true, std::numbers::pi, scenario != 0);
    const auto token = g->OriginalHeadingSeed();
    auto o = Observation(true);
    if (scenario == 1)
      Consume(*g, v, v + 100000000, 3);
    if (scenario == 2)
      Access::DamageLineage(*g);
    if (scenario == 3)
      ASSERT_TRUE(g->QueueYawAtEpoch(.2, o.sigma, false, o.source_ns, o.id, epoch));
    if (scenario == 5)
      Access::RetirePrefix(*g);
    if (scenario == 6)
      ASSERT_TRUE(g->QueueYawAtEpoch(o.yaw, o.sigma, false, o.source_ns + 1, o.id, epoch));
    const auto before = Access::Capture(*g);
    const auto result = g->RecoverInvalidSeedHeading(token, scenario != 4, o);
    EXPECT_EQ(result,
              scenario == 4                    ? fg::HeadingRecoveryResult::SeedNotInvalid
              : scenario == 1 || scenario == 0 ? fg::HeadingRecoveryResult::UnsupportedSource
              : scenario == 5                  ? fg::HeadingRecoveryResult::UnsupportedPrefix
              : scenario == 3 || scenario == 6 ? fg::HeadingRecoveryResult::ObservationConflict
                                               : fg::HeadingRecoveryResult::UnsupportedFamily);
    Unchanged(before, Access::Capture(*g));
  }
}

TEST(CoupledHeadingRecovery, PrunedPreSBirthIdentityAndQueuedRecoveryRemainExact)
{
  auto g = Replay(true, true, true, std::numbers::pi);
  const auto token = g->OriginalHeadingSeed();
  g->RebaseISAM2();  // No window retirement: public-floor pruning can still drop T's ID.
  auto same_birth = Observation(true);
  same_birth.id = yaw;
  const auto before = Access::Capture(*g);
  EXPECT_FALSE(g->QueueYawAtEpoch(
      same_birth.yaw, same_birth.sigma, false, same_birth.source_ns, same_birth.id, epoch));
  EXPECT_EQ(g->RecoverInvalidSeedHeading(token, true, same_birth),
            fg::HeadingRecoveryResult::DuplicateObservation);
  Unchanged(before, Access::Capture(*g));
  const auto o = Observation(true);
  ASSERT_TRUE(g->QueueYawAtEpoch(o.yaw, o.sigma, o.robust, o.source_ns, o.id, epoch));
  Access::FakeRebase(*g);
  ASSERT_EQ(g->RecoverInvalidSeedHeading(token, true, o), fg::HeadingRecoveryResult::Applied);
  EXPECT_EQ(g->SourceYawFactorCount(), 2u);
  EXPECT_FALSE(g->QueueYawAtEpoch(o.yaw, o.sigma, o.robust, o.source_ns, o.id, epoch));
  EXPECT_TRUE(Access::RebaseCancelled(*g));
}

TEST(CoupledHeadingRecovery, RankOneSharedRateAndBoundaryAliasKeepCompleteJoint)
{
  for (const bool earlier : {false, true})
    for (const int64_t time : std::array<int64_t, 3>{s, 1150000000, u})
    {
      auto wrong = Replay(true, earlier, true, std::numbers::pi, true, false);
      auto control = Replay(true, earlier, true, 0., true, false);
      const auto token = wrong->OriginalHeadingSeed();
      const fg::RecoveryYawObservation o{epoch, time, 99, 0., .08, false};
      Access::AssembleReference(*control, o);
      ASSERT_EQ(wrong->RecoverInvalidSeedHeading(token, true, o),
                fg::HeadingRecoveryResult::Applied);
      SameJoint(Access::Capture(*control), Access::Capture(*wrong));
    }
}

TEST(CoupledHeadingRecovery, EqualPayloadDistinctHeadingSurvivesExactPointerRetirement)
{
  auto g = Replay(false, false, true, std::numbers::pi);
  const auto token = g->OriginalHeadingSeed();
  const auto birth = Access::Capture(*g).bootstrap->live_birth_heading->factor;
  ASSERT_TRUE(g->QueueYawAtEpoch(std::numbers::pi, .1, false, u, 83, epoch));
  ASSERT_TRUE(g->TickSource(due + 1, v, epoch));
  gtsam::NonlinearFactor::shared_ptr independent;
  for (const auto& factor : Access::Capture(*g).factors)
    if (factor != birth && dynamic_cast<const fg::YawUnaryFactor*>(factor.get()) &&
        factor->equals(*birth, 1e-12))
      independent = factor;
  ASSERT_TRUE(independent);
  ASSERT_EQ(g->RecoverInvalidSeedHeading(token, true, Observation(true)),
            fg::HeadingRecoveryResult::Applied);
  const auto after = Access::Capture(*g);
  EXPECT_EQ(std::count(after.factors.begin(), after.factors.end(), birth), 0);
  EXPECT_EQ(std::count(after.factors.begin(), after.factors.end(), independent), 1);
  EXPECT_EQ(g->SourceYawFactorCount(), 3u);
}

class FailingPrior : public gtsam::PriorFactor<gtsam::Pose2>
{
public:
  FailingPrior(const gtsam::Pose2& pose, std::shared_ptr<bool> fail)
      : gtsam::PriorFactor<gtsam::Pose2>(fg::PoseKey(1),
                                         pose,
                                         gtsam::noiseModel::Isotropic::Sigma(3, .2)),
        fail_(std::move(fail))
  {
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override
  {
    if (*fail_)
      throw std::runtime_error("injected coupled recovery candidate failure");
    return gtsam::PriorFactor<gtsam::Pose2>::linearize(values);
  }

private:
  std::shared_ptr<bool> fail_;
};

TEST(CoupledHeadingRecovery, CandidateFailureRetainsNestedFamilyQueueAndWorkerBuffers)
{
  auto g = Replay(true, false, true, std::numbers::pi);
  const auto token = g->OriginalHeadingSeed();
  auto fail = std::make_shared<bool>(false);
  Access::AddFactor(*g, std::make_shared<FailingPrior>(g->LatestSnapshot()->pose, fail));
  const auto o = Observation(true);
  ASSERT_TRUE(g->QueueYawAtEpoch(o.yaw, o.sigma, false, o.source_ns, o.id, epoch));
  Access::FakeRebase(*g);
  *fail = true;
  const auto before = Access::Capture(*g);
  EXPECT_EQ(g->RecoverInvalidSeedHeading(token, true, o), fg::HeadingRecoveryResult::Failed);
  Unchanged(before, Access::Capture(*g));
  *fail = false;
  ASSERT_EQ(g->RecoverInvalidSeedHeading(token, true, o), fg::HeadingRecoveryResult::Applied);
  EXPECT_TRUE(Access::RebaseCancelled(*g));
  EXPECT_EQ(g->SourceYawFactorCount(), 2u);
}
}  // namespace
