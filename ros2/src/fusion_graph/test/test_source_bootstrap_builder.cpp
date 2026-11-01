// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "fusion_graph/factors.hpp"
#include "source_bootstrap_builder.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace
{
using namespace fusion_graph;
using namespace gtsam;
constexpr int64_t second = 1000000000;
constexpr int64_t t = second, s = 1100000000, u = 1200000000, v = 1300000000;
constexpr uint64_t epoch = 19, gnss_id = 71, heading_id = 82;
const Key xs = Symbol('x', 0), bv = Symbol('b', 1), bs = Symbol('b', 0);

double Seconds(int64_t start, int64_t end)
{
  return static_cast<double>(static_cast<long double>(end) - start) * 1e-9;
}

ConsumedMotionStep Step(int64_t begin, int64_t end, uint64_t sequence = 1)
{
  ConsumedMotionStep step;
  step.epoch = epoch;
  step.start_ns = begin;
  step.end_ns = end;
  step.wheel.id = {MotionSourceKind::Wheel, sequence, end};
  step.wheel.support = {begin, end};
  step.gyro.id = {MotionSourceKind::Gyro, sequence, end};
  step.gyro.support = {begin, end};
  return step;
}

void SetSuffixInput(SourceBootstrapBuildRequest& request)
{
  ProcessWindowPolicyInput input;
  for (const auto& step : request.original_steps)
  {
    const auto begin = std::max(request.source_s_ns, step.start_ns);
    const auto end = std::min(request.frontier_ns, step.end_ns);
    if (end <= begin)
      continue;
    const double dt = Seconds(begin, end);
    input.dx += step.wheel.vx * dt;
    input.dy += step.wheel.vy * dt;
    input.wheel_travel_m += std::hypot(step.wheel.vx, step.wheel.vy) * dt;
    input.dtheta_wheel += step.wheel.wz * dt;
    input.dtheta_gyro += (step.gyro.raw_z - step.bias_linearization) * dt;
    input.dt_total += dt;
    input.max_abs_gyro_rad_per_s =
        std::max(input.max_abs_gyro_rad_per_s, std::abs(step.gyro.raw_z));
  }
  input.last_node_time_s = static_cast<double>(request.initial_schedule_ns) * 1e-9;
  input.now_s = static_cast<double>(request.schedule_ns) * 1e-9;
  request.suffix_policy_input = input;
}

SourceBootstrapBuildRequest Request(bool preint, bool earlier, bool lever, bool promote = false)
{
  SourceBootstrapBuildRequest request;
  request.params.node_period_s = .1;
  request.params.use_imu_preint = preint;
  request.params.lever_arm_x = lever ? .2 : 0.;
  request.params.lever_arm_y = lever ? -.1 : 0.;
  request.epoch = epoch;
  request.source_s_ns = s;
  request.frontier_ns = u;
  request.promote = promote;
  request.initial_schedule_ns = 30 * second;
  request.schedule_ns = request.initial_schedule_ns + (promote ? second / 10 : 0);
  request.original_steps = {Step(t, u)};
  request.position = {Vector2(request.params.lever_arm_x, request.params.lever_arm_y),
                      .014,
                      false,
                      s,
                      gnss_id,
                      std::nullopt};
  request.heading = {Vector2::Zero(), .1, false, earlier ? t : u, heading_id, 0.};
  request.public_policy_baseline.slip_window = SlipWindow(5);
  SetSuffixInput(request);
  return request;
}

struct Layout
{
  Ordering order;
  std::map<Key, int> dimensions, offsets;
  int size = 0;
  void Add(Key key, int dimension)
  {
    order.push_back(key);
    dimensions.emplace(key, dimension);
    offsets.emplace(key, size);
    size += dimension;
  }
};

Layout CandidateLayout(const SourceBootstrapCandidate& candidate)
{
  Layout layout;
  for (const auto& value : candidate.values)
    layout.Add(value.key, value.value.dim());
  return layout;
}

// QR conditionals retain zero sigmas for exact rows. Back substitution with
// those sigmas yields the complete covariance, including exact cross-state
// correlations; no constrained row is converted to information or jittered.
Matrix QrCovariance(const GaussianFactorGraph& graph, const Layout& layout)
{
  const auto net = graph.eliminateSequential(layout.order, EliminateQR);
  Matrix r = Matrix::Zero(layout.size, layout.size);
  Vector sigmas = Vector::Ones(layout.size);
  int total_rows = 0;
  for (const auto& conditional : *net)
  {
    const int rows = conditional->rows();
    // Independent frontals may appear in a different BayesNet iteration
    // order. Match their row blocks to the same key layout as the columns.
    EXPECT_EQ(conditional->nrFrontals(), 1u);
    const Key frontal = *conditional->begin();
    EXPECT_EQ(layout.dimensions.at(frontal), rows);
    const int row = layout.offsets.at(frontal);
    for (Key key : conditional->keys())
      r.block(row, layout.offsets.at(key), rows, layout.dimensions.at(key)) =
          conditional->getA(key);
    if (conditional->get_model())
      sigmas.segment(row, rows) = conditional->get_model()->sigmas();
    total_rows += rows;
  }
  EXPECT_EQ(total_rows, layout.size);
  EXPECT_TRUE((r.triangularView<Eigen::StrictlyLower>().toDenseMatrix().array() == 0.).all());
  const Matrix response =
      r.triangularView<Eigen::Upper>().solve(sigmas.asDiagonal().toDenseMatrix());
  EXPECT_TRUE(response.allFinite());
  return response * response.transpose();
}

struct Oracle
{
  GaussianFactorGraph graph;
  Layout layout;
};

// Independent raw linear Gaussian model for the zero-motion cases. One latent
// producer-rate error is shared across S. All coefficients come from the raw
// timestamps and parameters, never from family windows/covariance getters.
Oracle StationaryOracle(const SourceBootstrapBuildRequest& request,
                        const SourceBootstrapCandidate& candidate)
{
  Oracle oracle;
  oracle.layout = CandidateLayout(candidate);
  const Key epsilon = Symbol('e', 0);
  if (request.params.use_imu_preint)
    oracle.layout.Add(epsilon, 1);
  auto add = [&](const std::vector<std::pair<Key, Matrix>>& terms, const Vector& sigmas)
  {
    oracle.graph.emplace_shared<JacobianFactor>(terms,
                                                Vector::Zero(sigmas.size()),
                                                noiseModel::Constrained::MixedSigmas(sigmas));
  };
  auto window = [&](int64_t begin, int64_t end, Key left, Key right, Key bias)
  {
    const double dt = Seconds(begin, end);
    const double distance = request.params.wheel_creep_speed_mps * dt;
    const Vector3 sigmas(
        std::max(kMinWheelSigmaM, request.params.wheel_sigma_x_per_sqrt_m * std::sqrt(distance)),
        std::max(kMinWheelSigmaM, request.params.wheel_sigma_y_per_sqrt_m * std::sqrt(distance)),
        request.params.use_imu_preint ? .5 : request.params.stationary_sigma_theta);
    add({{left, -Matrix3::Identity()}, {right, Matrix3::Identity()}}, sigmas);
    if (request.params.use_imu_preint)
    {
      Matrix13 yaw;
      yaw << 0., 0., 1.;
      add({{left, -yaw},
           {right, yaw},
           {bias, Matrix11::Constant(dt)},
           {epsilon, Matrix11::Constant(-dt)}},
          Vector1(0.));
    }
  };
  if (candidate.prefix_key)
    window(request.heading.source_ns, s, *candidate.prefix_key, xs, bs);
  if (request.frontier_ns > s)
    window(s, request.frontier_ns, xs, candidate.frontier_key, bv);
  if (request.params.use_imu_preint)
  {
    add({{epsilon, Matrix11::Identity()}}, Vector1(request.params.gyro_noise_density_rad_per_s));
    add({{bs, Matrix11::Identity()}}, Vector1(request.params.gyro_bias_prior_sigma_rad_per_s));
    if (request.frontier_ns > s)
      add({{bs, -Matrix11::Identity()}, {bv, Matrix11::Identity()}},
          Vector1(std::max(1e-6,
                           request.params.gyro_bias_rw_rad_per_s *
                               std::sqrt(Seconds(s, request.frontier_ns)))));
  }
  Matrix23 gnss;
  gnss << 1., 0., -request.params.lever_arm_y, 0., 1., request.params.lever_arm_x;
  add({{xs, gnss}}, Vector2(request.position.sigma, request.position.sigma));
  Matrix13 yaw;
  yaw << 0., 0., 1.;
  add({{candidate.prefix_key ? *candidate.prefix_key : candidate.frontier_key, yaw}},
      Vector1(request.heading.sigma));
  return oracle;
}

void ExpectBaselineUnchanged(const SourceBootstrapBuildRequest& request,
                             const SourceBootstrapBuildRequest& before)
{
  EXPECT_EQ(request.original_steps, before.original_steps);
  EXPECT_EQ(request.next_acquisition, before.next_acquisition);
  EXPECT_EQ(request.public_policy_baseline.slip_window.Size(),
            before.public_policy_baseline.slip_window.Size());
  EXPECT_EQ(request.public_policy_baseline.slip_window.Capacity(),
            before.public_policy_baseline.slip_window.Capacity());
  EXPECT_EQ(request.public_policy_baseline.slip_veto_count,
            before.public_policy_baseline.slip_veto_count);
  EXPECT_EQ(request.public_policy_baseline.hand_push_count,
            before.public_policy_baseline.hand_push_count);
  EXPECT_DOUBLE_EQ(request.public_policy_baseline.residual_ema,
                   before.public_policy_baseline.residual_ema);
  EXPECT_DOUBLE_EQ(request.public_policy_baseline.last_wheel_sigma_x_eff,
                   before.public_policy_baseline.last_wheel_sigma_x_eff);
  EXPECT_EQ(request.public_policy_baseline.wheel_stationary_now,
            before.public_policy_baseline.wheel_stationary_now);
}

TEST(SourceBootstrapBuilder, SharedRawRateOracleMatchesAllPoseBiasMeansAndFullCovariance)
{
  for (bool preint : {false, true})
    for (bool earlier : {false, true})
      for (bool lever : {false, true})
        for (bool promote : {false, true})
        {
          SCOPED_TRACE(::testing::Message() << preint << earlier << lever << promote);
          const auto request = Request(preint, earlier, lever, promote);
          const auto before = request;
          const auto candidate = BuildSourceBootstrapCandidate(request);
          const auto actual_layout = CandidateLayout(candidate);
          const auto oracle = StationaryOracle(request, candidate);
          const Matrix expected = QrCovariance(oracle.graph, oracle.layout);
          const Matrix actual =
              QrCovariance(*candidate.isam->getFactorsUnsafe().linearize(candidate.values),
                           actual_layout);
          EXPECT_LT(
              (actual - expected.topLeftCorner(actual.rows(), actual.cols())).cwiseAbs().maxCoeff(),
              1e-10);
          const auto tree = oracle.graph.eliminateMultifrontal(oracle.layout.order, EliminateQR);
          EXPECT_LT(
              (candidate.source_s_covariance - tree->marginalCovariance(xs)).cwiseAbs().maxCoeff(),
              1e-10);
          for (const auto& value : candidate.values)
            if (Symbol(value.key).chr() == 'b')
              EXPECT_NEAR(value.value.cast<double>(), 0., 1e-12);
            else
              EXPECT_LT(value.value.cast<Pose2>().localCoordinates(Pose2()).norm(), 1e-10);
          EXPECT_EQ(candidate.frontier_key,
                    promote ? Key(Symbol('x', 1)) : Key(Symbol('a', earlier ? 1 : 0)));
          ASSERT_EQ(candidate.raw_bindings.size(), 2u);
          EXPECT_EQ(candidate.raw_bindings[0].id, gnss_id);
          EXPECT_EQ(candidate.raw_bindings[0].source_ns, s);
          EXPECT_EQ(candidate.raw_bindings[0].key, xs);
          EXPECT_FALSE(candidate.raw_bindings[0].yaw);
          EXPECT_EQ(candidate.raw_bindings[1].id, heading_id);
          EXPECT_EQ(candidate.raw_bindings[1].source_ns, earlier ? t : u);
          EXPECT_EQ(candidate.raw_bindings[1].key,
                    earlier ? *candidate.prefix_key : candidate.frontier_key);
          EXPECT_TRUE(candidate.raw_bindings[1].yaw);
          EXPECT_EQ(candidate.auxiliary_poses.size(), (earlier ? 1u : 0u) + (promote ? 0u : 1u));
          if (earlier)
            EXPECT_EQ(candidate.auxiliary_poses.at(*candidate.prefix_key), t);
          if (!promote)
            EXPECT_EQ(candidate.auxiliary_poses.at(candidate.frontier_key), u);
          EXPECT_EQ(candidate.family->source_epoch(), epoch);
          EXPECT_EQ(candidate.family->start_ns(), earlier ? t : s);
          EXPECT_EQ(candidate.family->end_ns(), u);
          int gnss = 0, yaw = 0, wheels = 0, cold_bias = 0, bias_rw = 0;
          for (const auto& factor : candidate.isam->getFactorsUnsafe())
          {
            gnss += dynamic_cast<const GnssLeverArmFactor*>(factor.get()) != nullptr;
            yaw += dynamic_cast<const YawUnaryFactor*>(factor.get()) != nullptr;
            wheels += dynamic_cast<const BetweenFactor<Pose2>*>(factor.get()) != nullptr;
            bias_rw += dynamic_cast<const BetweenFactor<double>*>(factor.get()) != nullptr;
            if (const auto* prior = dynamic_cast<const PriorFactor<double>*>(factor.get()))
            {
              ++cold_bias;
              EXPECT_EQ(prior->keys(), KeyVector{bs});
              EXPECT_DOUBLE_EQ(prior->prior(), 0.);
            }
            EXPECT_EQ(dynamic_cast<const PriorFactor<Pose2>*>(factor.get()), nullptr);
            EXPECT_EQ(dynamic_cast<const GyroPreintFactor*>(factor.get()), nullptr);
          }
          EXPECT_EQ(gnss, 1);
          EXPECT_EQ(yaw, 1);
          EXPECT_EQ(wheels, earlier ? 2 : 1);
          EXPECT_EQ(cold_bias, preint ? 1 : 0);
          EXPECT_EQ(bias_rw, preint ? 1 : 0);
          EXPECT_EQ(candidate.isam->getFactorsUnsafe().nrFactors(),
                    preint ? (earlier ? 7u : 6u) : (earlier ? 4u : 3u));
          if (preint)
            for (const auto& window : candidate.family->windows())
              for (const auto& use : window.gyro->sources)
                EXPECT_EQ(use.source, request.original_steps.front().gyro);
          ASSERT_TRUE(candidate.suffix_policy);
          EXPECT_EQ(candidate.suffix_policy->state.slip_window.Size(), 1u);
          EXPECT_EQ(candidate.suffix_policy->state.slip_window.Capacity(), 5u);
          ExpectBaselineUnchanged(request, before);
        }
}

TEST(SourceBootstrapBuilder, SourceSFrontierHasOnlyTheHistoricalPrefixAndNoZeroDurationEdge)
{
  for (bool preint : {false, true})
  {
    auto request = Request(preint, true, true);
    request.frontier_ns = s;
    request.suffix_policy_input.reset();
    const auto before = request;
    const auto candidate = BuildSourceBootstrapCandidate(request);
    EXPECT_EQ(candidate.frontier_key, xs);
    ASSERT_TRUE(candidate.prefix_key);
    EXPECT_FALSE(candidate.suffix_policy);
    EXPECT_FALSE(candidate.values.exists(bv));
    EXPECT_EQ(candidate.auxiliary_poses.size(), 1u);
    EXPECT_EQ(candidate.family->start_ns(), t);
    EXPECT_EQ(candidate.family->end_ns(), s);
    EXPECT_EQ(candidate.isam->getFactorsUnsafe().nrFactors(), preint ? 5u : 3u);
    EXPECT_TRUE(
        (candidate.source_s_covariance.array() == candidate.frontier_covariance.array()).all());
    const auto oracle = StationaryOracle(request, candidate);
    const auto tree = oracle.graph.eliminateMultifrontal(oracle.layout.order, EliminateQR);
    EXPECT_LT((candidate.source_s_covariance - tree->marginalCovariance(xs)).cwiseAbs().maxCoeff(),
              1e-10);
    ExpectBaselineUnchanged(request, before);
  }
}

TEST(SourceBootstrapBuilder, PrefixCannotAdvanceSuffixReadinessOrReplaceItsScheduleClock)
{
  for (bool long_source : {false, true})
  {
    auto request = Request(false, true, false);
    request.frontier_ns = s + (long_source ? second : second / 10);
    request.schedule_ns += long_source ? second / 10 : second;
    request.original_steps = {Step(t, s), Step(s, request.frontier_ns, 2)};
    for (auto& step : request.original_steps)
    {
      step.wheel.vx = .2;
      step.wheel.wz = .05 / Seconds(step.start_ns, step.end_ns);
    }
    for (int i = 0; i < 4; ++i)
      request.public_policy_baseline.slip_window.Push(.05, 0.);
    request.public_policy_baseline.slip_veto_count = 9;
    SetSuffixInput(request);
    const auto before = request;
    const auto expected = SelectProcessWindowPolicy(request.params,
                                                    *request.suffix_policy_input,
                                                    request.public_policy_baseline);
    const auto candidate = BuildSourceBootstrapCandidate(request);
    ASSERT_TRUE(candidate.prefix_policy);
    ASSERT_TRUE(candidate.suffix_policy);
    EXPECT_TRUE(candidate.prefix_policy->selection.slip_detected);
    EXPECT_EQ(candidate.suffix_policy->slip_detected, long_source);
    EXPECT_EQ(candidate.suffix_policy->state.slip_window.Size(), long_source ? 5u : 1u);
    EXPECT_EQ(candidate.suffix_policy->state.slip_veto_count, long_source ? 10u : 9u);
    EXPECT_EQ(candidate.suffix_policy->state.slip_veto_count, expected.state.slip_veto_count);
    EXPECT_DOUBLE_EQ(candidate.suffix_policy->state.residual_ema, expected.state.residual_ema);
    EXPECT_DOUBLE_EQ(candidate.suffix_policy->wheel_sigma_x, expected.wheel_sigma_x);
    EXPECT_DOUBLE_EQ(candidate.prefix_policy->input.last_node_time_s, 0.);
    EXPECT_DOUBLE_EQ(candidate.prefix_policy->input.now_s, .1);
    ExpectBaselineUnchanged(request, before);
  }
}

TEST(SourceBootstrapBuilder, OffModeRetainsPerStepRecordedBiasWhileOnModeRejectsItsChange)
{
  auto request = Request(false, false, false);
  request.original_steps = {Step(t, 1150000000), Step(1150000000, u, 2)};
  request.original_steps[0].bias_linearization = .03;
  request.original_steps[1].bias_linearization = -.04;
  for (auto& step : request.original_steps)
    step.gyro.raw_z = .2;
  SetSuffixInput(request);
  const auto before = request;
  const auto candidate = BuildSourceBootstrapCandidate(request);
  const double expected_turn = .05 * (.2 - .03) + .05 * (.2 + .04);
  EXPECT_NEAR(candidate.source_s_pose.theta(), -expected_turn, 1e-10);
  EXPECT_NEAR(candidate.frontier_pose.theta(), 0., 1e-10);
  EXPECT_FALSE(candidate.values.exists(bs));
  EXPECT_FALSE(candidate.values.exists(bv));
  ExpectBaselineUnchanged(request, before);
  request.params.use_imu_preint = true;
  EXPECT_THROW(BuildSourceBootstrapCandidate(request), std::invalid_argument);
}

TEST(SourceBootstrapBuilder, InvalidSupportIdentitiesRecipesAndExtrasLeaveInputsUntouched)
{
  for (int invalid = 0; invalid < 5; ++invalid)
  {
    SCOPED_TRACE(invalid);
    auto request = Request(true, false, false);
    if (invalid == 0)
      request.original_steps.front().gyro.support = {u, t};
    else if (invalid == 1)
      request.original_steps = {Step(t, s - 1), Step(s + 1, u, 2)};
    else if (invalid == 2)
      request.additional_observations.push_back(request.heading);
    else if (invalid == 3)
      request.extra_factors.emplace_shared<PriorFactor<Pose2>>(Symbol('x', 99),
                                                               Pose2(),
                                                               noiseModel::Isotropic::Sigma(3, .1));
    else
      request.suffix_policy_input->dx = .02;
    const auto before = request;
    EXPECT_THROW(BuildSourceBootstrapCandidate(request), std::invalid_argument);
    ExpectBaselineUnchanged(request, before);
  }
}

TEST(SourceBootstrapBuilder, PromotedRebuildRetainsRawGnssAndExtraLikelihoodsWithoutOldFrontier)
{
  for (bool preint : {false, true})
  {
    auto birth_request = Request(preint, true, true);
    const auto birth = BuildSourceBootstrapCandidate(birth_request);
    auto promoted_request = birth_request;
    promoted_request.frontier_ns = v;
    promoted_request.schedule_ns += second / 10;
    promoted_request.promote = true;
    promoted_request.original_steps.push_back(Step(u, v, 2));
    promoted_request.prefix_left_key = birth.prefix_key;
    promoted_request.next_acquisition = birth.next_acquisition;
    promoted_request.preserved_primary_gnss = birth.raw_bindings.front().factor;
    promoted_request.additional_observations.push_back(
        {Vector2::Zero(), .08, true, s, heading_id + 1, .015});
    const auto anchor =
        std::make_shared<PriorFactor<Pose2>>(xs,
                                             Pose2(.03, -.01, .02),
                                             noiseModel::Diagonal::Sigmas(Vector3(.05, .05, .08)));
    promoted_request.extra_factors.push_back(anchor);
    SetSuffixInput(promoted_request);
    auto direct_request = promoted_request;
    direct_request.prefix_left_key.reset();
    direct_request.next_acquisition = 0;
    direct_request.preserved_primary_gnss.reset();
    const auto rebuilt = BuildSourceBootstrapCandidate(promoted_request);
    const auto direct = BuildSourceBootstrapCandidate(direct_request);
    EXPECT_EQ(rebuilt.raw_bindings.front().factor, birth.raw_bindings.front().factor);
    EXPECT_EQ(rebuilt.frontier_key, Key(Symbol('x', 1)));
    EXPECT_EQ(rebuilt.prefix_key, birth.prefix_key);
    EXPECT_EQ(rebuilt.auxiliary_poses.size(), 1u);
    EXPECT_FALSE(rebuilt.values.exists(birth.frontier_key));
    ASSERT_EQ(rebuilt.raw_bindings.size(), 3u);
    EXPECT_EQ(rebuilt.raw_bindings.back().source_ns, s);
    EXPECT_EQ(rebuilt.raw_bindings.back().key, xs);
    const auto& factors = rebuilt.isam->getFactorsUnsafe();
    EXPECT_EQ(std::count(factors.begin(), factors.end(), anchor), 1);
    for (const auto& factor : factors)
      EXPECT_EQ(std::count(factor->keys().begin(), factor->keys().end(), birth.frontier_key), 0);
    EXPECT_GT(factors.error(rebuilt.values), 1e-6);
    EXPECT_EQ(factors.nrFactors(), direct.isam->getFactorsUnsafe().nrFactors());
    for (const auto& value : rebuilt.values)
      if (Symbol(value.key).chr() == 'b')
        EXPECT_NEAR(value.value.cast<double>(), direct.values.at<double>(value.key), 1e-10);
      else
        EXPECT_LT(
            value.value.cast<Pose2>().localCoordinates(direct.values.at<Pose2>(value.key)).norm(),
            1e-10);
    const auto layout = CandidateLayout(rebuilt);
    const Matrix actual = QrCovariance(*factors.linearize(rebuilt.values), layout);
    const Matrix expected =
        QrCovariance(*direct.isam->getFactorsUnsafe().linearize(direct.values), layout);
    EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), 1e-10);
  }
}
}  // namespace
