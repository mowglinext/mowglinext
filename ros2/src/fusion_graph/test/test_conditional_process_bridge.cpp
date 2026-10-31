// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/conditional_process_bridge.hpp"
#include "fusion_graph/factors.hpp"
#include <Eigen/QR>
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace
{
using namespace fusion_graph;
using gtsam::Matrix;
using gtsam::Pose2;
using gtsam::Symbol;
using gtsam::Vector;
using gtsam::Vector3;
constexpr int64_t kSecond = 1000000000;
const gtsam::Key kLeft = Symbol('x', 0), kRight = Symbol('x', 1), kBias = Symbol('b', 1);
const gtsam::Key kFirst = Symbol('y', 0), kSecondKey = Symbol('y', 1);

FrozenProcessEdge Edge(double speed = 0.8, double yaw_rate = 0.0, bool bias = false)
{
  auto history = std::make_shared<MotionSourceHistory>(4);
  history->Reset(7);
  ConsumedMotionStep step;
  step.epoch = 7;
  step.end_ns = 2 * kSecond;
  step.wheel.id.sequence = 1;
  step.wheel.vx = speed;
  step.gyro.id.sequence = 1;
  step.gyro.raw_z = yaw_rate + 0.03;
  step.bias_linearization = 0.03;
  EXPECT_EQ(history->Append(step), MotionHistoryStatus::Ok);
  FrozenProcessEdge edge;
  edge.left_key = kLeft;
  edge.right_key = kRight;
  edge.source_epoch = 7;
  edge.end_ns = step.end_ns;
  edge.endpoint_covariance = Vector3(0.04, 0.002, 0.01).asDiagonal();
  edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, 7, 0);
  const Pose2 left(1.0, -0.4, 0.5);
  edge.reference_values.insert(kLeft, left);
  edge.reference_values.insert(kRight, left.compose((*edge.trajectory_query)(edge.end_ns).mean));
  if (bias)
  {
    edge.bias_key = kBias;
    edge.reference_values.insert(kBias, 0.03);
  }
  return edge;
}

FrozenProcessEdge TurnForwardTurnEdge(bool bias)
{
  auto edge = Edge(0.0, 0.0, bias);
  auto history = std::make_shared<MotionSourceHistory>(4);
  history->Reset(edge.source_epoch);
  for (int i = 0; i < 4; ++i)
  {
    ConsumedMotionStep step;
    step.epoch = edge.source_epoch;
    step.start_ns = i * kSecond / 2;
    step.end_ns = (i + 1) * kSecond / 2;
    step.wheel.id.sequence = step.gyro.id.sequence = i + 1;
    step.wheel.vx = i == 1 || i == 2 ? 0.8 : 0.0;
    step.gyro.raw_z = i == 0 ? 0.7 : (i == 3 ? -0.7 : 0.0);
    EXPECT_EQ(history->Append(step), MotionHistoryStatus::Ok);
  }
  edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, edge.source_epoch, 0);
  edge.reference_values.update(kRight,
                               edge.reference_values.at<Pose2>(kLeft).compose(
                                   (*edge.trajectory_query)(edge.end_ns).mean));
  if (bias)
    edge.reference_values.update(kBias, 0.0);
  return edge;
}

gtsam::NonlinearFactorGraph Base(const FrozenProcessEdge& edge)
{
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<Pose2>(kLeft,
                                      edge.reference_values.at<Pose2>(kLeft),
                                      gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.3, 0.1))));
  graph.add(gtsam::BetweenFactor<Pose2>(kLeft,
                                        kRight,
                                        (*edge.trajectory_query)(edge.end_ns).mean,
                                        gtsam::noiseModel::Gaussian::Covariance(
                                            edge.endpoint_covariance)));
  if (edge.bias_key)
  {
    graph.add(
        gtsam::PriorFactor<double>(kBias, 0.03, gtsam::noiseModel::Isotropic::Sigma(1, 0.04)));
    // An absolute bias correlated with both endpoints, rather than an exact plug-in estimate.
    Matrix left = Matrix::Zero(1, 3), right = Matrix::Zero(1, 3);
    left(0, 0) = 0.4;
    left(0, 2) = -0.7;
    right(0, 1) = 0.5;
    const gtsam::JacobianFactor factor(kLeft,
                                       left,
                                       kRight,
                                       right,
                                       kBias,
                                       Matrix::Constant(1, 1, 1.0),
                                       Vector::Zero(1),
                                       gtsam::noiseModel::Isotropic::Sigma(1, 0.03));
    graph.add(std::make_shared<gtsam::LinearContainerFactor>(factor, edge.reference_values));
  }
  return graph;
}

struct Solution
{
  Vector mean;
  Matrix covariance;
};

// Independent sensitivity oracle: solve exact rows first, then least squares in their
// nullspace. It never converts a constrained factor to a Hessian or adds epsilon noise.
Solution Solve(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& refs,
               const gtsam::Ordering& order)
{
  const auto gaussian = graph.linearize(refs);
  std::map<gtsam::Key, size_t> offsets;
  size_t dimensions = 0, rows = 0;
  for (const auto key : order)
  {
    offsets[key] = dimensions;
    dimensions += refs.at(key).dim();
  }
  for (const auto& factor : *gaussian)
    rows += gtsam::JacobianFactor(*factor).rows();
  Matrix soft = Matrix::Zero(rows, dimensions), hard = soft;
  Vector bs = Vector::Zero(rows), bh = bs;
  size_t ns = 0, nh = 0;
  for (const auto& factor : *gaussian)
  {
    const gtsam::JacobianFactor jacobian(*factor);
    const auto model = jacobian.get_model();
    for (size_t row = 0; row < jacobian.rows(); ++row)
    {
      const double sigma = model ? model->sigmas()(row) : 1.0;
      const bool exact = sigma == 0.0;
      const size_t destination = exact ? nh++ : ns++;
      const double weight = exact ? 1.0 : 1.0 / sigma;
      for (auto key = jacobian.begin(); key != jacobian.end(); ++key)
      {
        auto& matrix = exact ? hard : soft;
        matrix.block(destination, offsets.at(*key), 1, refs.at(*key).dim()) =
            weight * jacobian.getA(key).row(row);
      }
      (exact ? bh : bs)(destination) = weight * jacobian.getb()(row);
    }
  }
  soft.conservativeResize(ns, dimensions);
  bs.conservativeResize(ns);
  hard.conservativeResize(nh, dimensions);
  bh.conservativeResize(nh);
  Vector particular = Vector::Zero(dimensions);
  Matrix nullspace = Matrix::Identity(dimensions, dimensions);
  if (nh)
  {
    particular = hard.completeOrthogonalDecomposition().solve(bh);
    nullspace = hard.fullPivLu().kernel();
    EXPECT_LT((hard * particular - bh).norm(), 1e-10);
  }
  const Matrix pinverse = (soft * nullspace).completeOrthogonalDecomposition().pseudoInverse();
  const Matrix sensitivity = nullspace * pinverse;
  return {particular + sensitivity * (bs - soft * particular),
          sensitivity * sensitivity.transpose()};
}

struct Derivatives
{
  Matrix left, target, bias;
};

// Numerical derivatives deliberately independent of the builder's analytic chain.
Derivatives NumericCoordinates(const FrozenProcessEdge& edge, int64_t time, const Pose2& target)
{
  const auto trajectory = (*edge.trajectory_query)(time);
  const auto left = edge.reference_values.at<Pose2>(kLeft);
  const auto f = [mean = trajectory.mean](const Pose2& a, const Pose2& b)
  {
    return Pose2::Logmap(mean.between(a.between(b)));
  };
  Derivatives result{gtsam::numericalDerivative21<Vector3, Pose2, Pose2>(f, left, target, 1e-5),
                     gtsam::numericalDerivative22<Vector3, Pose2, Pose2>(f, left, target, 1e-5),
                     Matrix::Zero(3, 1)};
  if (edge.bias_key)
  {
    const auto fb = [&](double delta)
    {
      return Pose2::Logmap(
          trajectory.mean.retract(trajectory.bias_jacobian * delta).between(left.between(target)));
    };
    result.bias = gtsam::numericalDerivative11<Vector3, double>(fb, 0.0, 1e-5);
  }
  return result;
}

// Independent Brownian bridge generative reference. Endpoint uncertainty comes from the
// unchanged endpoint factors. The bridge has independent terminal-chart increments conditioned
// on their total, and numerical state derivatives. It assembles all acquisitions jointly.
Solution Reference(const FrozenProcessEdge& edge,
                   const std::vector<int64_t>& times,
                   const gtsam::Vector2* lever = nullptr,
                   const std::vector<gtsam::Vector2>& observations = {})
{
  const size_t nbase = edge.bias_key ? 7 : 6;
  gtsam::Ordering base_order{kLeft, kRight};
  if (edge.bias_key)
    base_order.push_back(kBias);
  const auto baseline = Solve(Base(edge), edge.reference_values, base_order);
  const size_t dim = nbase + 3 * times.size();
  Matrix map = Matrix::Zero(dim, nbase), noise = Matrix::Zero(dim, dim);
  map.topRows(nbase).setIdentity();
  const auto end = (*edge.trajectory_query)(edge.end_ns);
  const auto r = NumericCoordinates(edge, edge.end_ns, edge.reference_values.at<Pose2>(kRight));
  Matrix jr(3, nbase);
  jr.leftCols(3) = r.left;
  jr.middleCols(3, 3) = r.target;
  if (edge.bias_key)
    jr.rightCols(1) = r.bias;
  std::vector<Pose2> poses;
  std::vector<Matrix> transports, targets;
  for (size_t i = 0; i < times.size(); ++i)
  {
    const auto trajectory = (*edge.trajectory_query)(times[i]);
    const auto pose = edge.reference_values.at<Pose2>(kLeft).compose(trajectory.mean);
    poses.push_back(pose);
    const auto v = NumericCoordinates(edge, times[i], pose);
    const Matrix inverse = v.target.inverse();
    targets.push_back(inverse);
    const Matrix transport = trajectory.mean.between(end.mean).AdjointMap();
    transports.push_back(transport);
    const double a = double(times[i]) / edge.end_ns;
    Matrix block = inverse * a * transport * jr;
    block.leftCols(3) -= inverse * v.left;
    if (edge.bias_key)
      block.rightCols(1) -= inverse * v.bias;
    map.middleRows(nbase + 3 * i, 3) = block;
    for (size_t j = 0; j <= i; ++j)
    {
      const double b = double(times[j]) / edge.end_ns;
      const Matrix covariance = inverse * transports[i] *
                                ((std::min(a, b) - a * b) * edge.endpoint_covariance) *
                                transports[j].transpose() * targets[j].transpose();
      noise.block<3, 3>(nbase + 3 * i, nbase + 3 * j) = covariance;
      noise.block<3, 3>(nbase + 3 * j, nbase + 3 * i) = covariance.transpose();
    }
  }
  Solution result{map * baseline.mean, map * baseline.covariance * map.transpose() + noise};
  if (lever)
  {
    Matrix h = Matrix::Zero(2 * times.size(), dim);
    Vector residual(2 * times.size());
    for (size_t i = 0; i < times.size(); ++i)
    {
      const auto measurement = observations.at(i);
      GnssLeverArmFactor factor(kFirst, measurement, *lever, gtsam::noiseModel::Unit::Create(2));
      Matrix jacobian;
      const Vector error = factor.evaluateError(poses[i], &jacobian);
      // The original robust factor whitens the entire 2-vector with one Huber weight.
      const double weight = std::min(1.0, 1.345 / (error.norm() / 0.03));
      h.block<2, 3>(2 * i, nbase + 3 * i) = jacobian;
      residual.segment<2>(2 * i) = -error;
      // Store each independent measurement variance for the batch Kalman update below.
      // (Tiny bounded fixtures avoid nonlinear claims; all comparisons share references.)
      noise.block<2, 2>(2 * i, 2 * i) = Matrix::Identity(2, 2) * 0.0009 / weight;
    }
    const Matrix observation_noise = noise.topLeftCorner(2 * times.size(), 2 * times.size());
    const Matrix innovation = h * result.covariance * h.transpose() + observation_noise;
    const Matrix gain = innovation.ldlt().solve(h * result.covariance).transpose();
    result.mean += gain * (residual - h * result.mean);
    result.covariance -= gain * h * result.covariance;
  }
  return result;
}

Solution Actual(FrozenProcessEdge edge, bool reverse, bool fixes)
{
  ConditionalProcessBridge bridge(edge);
  auto graph = Base(edge);
  auto refs = edge.reference_values;
  const std::vector<int64_t> times{kSecond / 2, 3 * kSecond / 2};
  const gtsam::Vector2 lever(0.3, -0.12);
  for (size_t index : (reverse ? std::vector<size_t>{1, 0} : std::vector<size_t>{0, 1}))
  {
    const auto key = index == 0 ? kFirst : kSecondKey;
    auto inserted = bridge.Insert(times[index], key);
    graph.add(inserted.factor);
    const auto pose =
        edge.reference_values.at<Pose2>(kLeft).compose((*edge.trajectory_query)(times[index]).mean);
    refs.insert(key, pose);
    if (fixes)
    {
      const gtsam::Vector2 measured = pose.transformFrom(lever) + gtsam::Vector2(0.08, -0.025);
      const auto robust =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                            gtsam::noiseModel::Isotropic::Sigma(2, 0.03));
      graph.emplace_shared<GnssLeverArmFactor>(key, measured, lever, robust);
    }
  }
  gtsam::Ordering order{kLeft, kRight};
  if (edge.bias_key)
    order.push_back(kBias);
  order.push_back(kFirst);
  order.push_back(kSecondKey);
  return Solve(graph, refs, order);
}

TEST(ConditionalProcessBridge, DiffusionMatchesIndependentJointReferenceAcrossMotionAndOrder)
{
  for (const bool bias : {false, true})
    for (const auto& edge : {Edge(0.8, 0.0, bias),
                             Edge(-0.7, 0.0, bias),
                             Edge(0.8, 0.65, bias),
                             TurnForwardTurnEdge(bias)})
      for (const bool fixes : {false, true})
      {
        const std::vector<int64_t> times{kSecond / 2, 3 * kSecond / 2};
        const gtsam::Vector2 lever(0.3, -0.12);
        std::vector<gtsam::Vector2> observations;
        for (const auto time : times)
          observations.push_back(edge.reference_values.at<Pose2>(kLeft)
                                     .compose((*edge.trajectory_query)(time).mean)
                                     .transformFrom(lever) +
                                 gtsam::Vector2(0.08, -0.025));
        const auto expected = Reference(edge, times, fixes ? &lever : nullptr, observations);
        const auto actual = Actual(edge, false, fixes), reversed = Actual(edge, true, fixes);
        EXPECT_LT((actual.mean - expected.mean).norm(), 2e-9);
        EXPECT_LT((actual.covariance - expected.covariance).norm(), 2e-9);
        EXPECT_LT((actual.mean - reversed.mean).norm(), 1e-10);
        EXPECT_LT((actual.covariance - reversed.covariance).norm(), 1e-10);
        if (!fixes)
        {
          const auto base =
              Solve(Base(edge),
                    edge.reference_values,
                    bias ? gtsam::Ordering{kLeft, kRight, kBias} : gtsam::Ordering{kLeft, kRight});
          EXPECT_LT((actual.covariance.topLeftCorner(base.mean.size(), base.mean.size()) -
                     base.covariance)
                        .norm(),
                    1e-10);
        }
      }
}

TEST(ConditionalProcessBridge, EndpointAndTimeAliasesDoNotCreateValuesOrFactors)
{
  ConditionalProcessBridge bridge(Edge());
  const auto left = bridge.Insert(0, kFirst), right = bridge.Insert(2 * kSecond, kFirst);
  EXPECT_EQ(left.key, kLeft);
  EXPECT_EQ(right.key, kRight);
  EXPECT_FALSE(left.initial_value);
  EXPECT_FALSE(right.factor);
  const auto interior = bridge.Insert(kSecond, kFirst);
  const auto alias = bridge.Insert(kSecond, kSecondKey);
  EXPECT_EQ(alias.key, interior.key);
  EXPECT_FALSE(alias.initial_value);
  EXPECT_FALSE(alias.factor);
  EXPECT_THROW(bridge.Insert(-1, kSecondKey), std::out_of_range);
  EXPECT_THROW(bridge.Insert(2 * kSecond + 1, kSecondKey), std::out_of_range);
  EXPECT_THROW(bridge.Insert(kSecond / 2, kFirst), std::invalid_argument);
}

TEST(ConditionalProcessBridge, InitializationUsesCurrentValuesWithoutChangingFrozenFactor)
{
  auto edge = Edge(0.8, 0.5, true);
  edge.reference_values.update(kRight,
                               edge.reference_values.at<Pose2>(kRight).retract(
                                   Vector3(0.1, 0.03, -0.04)));
  auto current = edge.reference_values;
  current.update(kRight, current.at<Pose2>(kRight).retract(Vector3(0.03, -0.02, 0.01)));
  current.update(kBias, 0.04);
  ConditionalProcessBridge first(edge), second(edge);
  const auto frozen = first.Insert(kSecond, kFirst);
  const auto updated = second.Insert(kSecond, kFirst, &current);
  EXPECT_TRUE(frozen.factor->equals(*updated.factor, 1e-13));
  EXPECT_GT(frozen.initial_value->localCoordinates(*updated.initial_value).norm(), 0.005);
  current.insert(kFirst, *updated.initial_value);
  EXPECT_LT(updated.factor->error(current), 1e-12);
  current.update(kFirst, current.at<Pose2>(kFirst).retract(Vector3(-0.01, 0.02, -0.01)));
  const auto late = second.Insert(3 * kSecond / 2, kSecondKey, &current);
  current.insert(kSecondKey, *late.initial_value);
  EXPECT_LT(late.factor->error(current), 1e-12);
}

TEST(ConditionalProcessBridge, AnisotropicCovarianceRetainsAllPositiveCoordinates)
{
  auto edge = Edge();
  edge.endpoint_covariance = Vector3(1.0, 1e-18, 1e-20).asDiagonal();
  ConditionalProcessBridge bridge(edge);
  const auto insertion = bridge.Insert(kSecond, kFirst);
  const gtsam::JacobianFactor jacobian(*insertion.factor->factor());
  EXPECT_TRUE((jacobian.get_model()->sigmas().array() > 0.0).all());
  edge.endpoint_covariance(0, 0) = -1.0;
  EXPECT_THROW(ConditionalProcessBridge invalid(edge), std::invalid_argument);
  edge.endpoint_covariance.setZero();
  edge.endpoint_covariance(0, 1) = edge.endpoint_covariance(1, 0) = 1e-20;
  EXPECT_THROW(ConditionalProcessBridge invalid(edge), std::invalid_argument);
}

TEST(ConditionalProcessBridge, SharedGyroPureSampleHasExactConstraintsAndMixedDiffusionDoesNot)
{
  for (const bool mixed : {false, true})
  {
    auto edge = Edge(0.0, 0.0);
    edge.endpoint_covariance =
        mixed ? (Vector3(0.02, 0.03, 0.04).asDiagonal()).toDenseMatrix() : gtsam::Matrix3::Zero();
    const auto id = (*edge.trajectory_query)(edge.end_ns).gyro_jacobians.begin()->first;
    edge.gyro_rate_variances[id] = 0.09;
    ConditionalProcessBridge bridge(edge);
    const auto insertion = bridge.Insert(kSecond, kFirst);
    const gtsam::JacobianFactor gaussian(*insertion.factor->factor());
    EXPECT_EQ((gaussian.get_model()->sigmas().array() == 0.0).count(), mixed ? 0 : 3);
    auto graph = Base(Edge(0.0));
    graph.add(insertion.factor);
    auto refs = edge.reference_values;
    refs.insert(kFirst, *insertion.initial_value);
    const auto solution = Solve(graph, refs, {kLeft, kRight, kFirst});
    // At t=1 of a 2 s held sample, rate-error variance is .09 and endpoint .36.
    // Shared-source conditional yaw variance is .09-.18^2/.36 = 0, whereas
    // falsely independent partial samples would leave .045. In mixed mode the
    // diffusion/sample ambiguity gives a positive conditional variance (.01).
    Matrix difference = Matrix::Zero(1, 9);
    difference(0, 2) = -0.5;
    difference(0, 5) = -0.5;
    difference(0, 8) = 1.0;
    const double variance = (difference * solution.covariance * difference.transpose())(0, 0);
    EXPECT_NEAR(variance, mixed ? 0.01 : 0.0, 1e-11);
    EXPECT_GT(std::abs(variance - 0.045), 0.03);
  }
}

TEST(ConditionalProcessBridge, HeldWheelIdentityDoesNotImplyConstantRawErrorModel)
{
  // A single edge with the same held wheel ID throughout has Brownian midpoint variance
  // D/4. A literal single constant velocity error conditioned on its total has ZERO.
  const auto edge = Edge(0.8);
  ConditionalProcessBridge bridge(edge);
  const auto acquisition = bridge.Insert(kSecond, kFirst);
  auto graph = Base(edge);
  graph.add(acquisition.factor);
  auto refs = edge.reference_values;
  refs.insert(kFirst, *acquisition.initial_value);
  const auto actual = Solve(graph, refs, {kLeft, kRight, kFirst});
  const auto expected = Reference(edge, {kSecond});
  EXPECT_LT((actual.covariance - expected.covariance).norm(), 1e-10);
  Matrix difference = Matrix::Zero(1, 9);
  difference(0, 0) = -0.5;
  difference(0, 3) = -0.5;
  difference(0, 6) = 1.0;
  // Headings are uncertain, so use a zero-yaw, zero-translation variant to isolate this
  // scalar statement, avoiding accidental loss of lever/heading covariance above.
  auto stationary = Edge(0.0);
  ConditionalProcessBridge stationary_bridge(stationary);
  const auto y = stationary_bridge.Insert(kSecond, kFirst);
  auto stationary_graph = Base(stationary);
  stationary_graph.add(y.factor);
  auto stationary_refs = stationary.reference_values;
  stationary_refs.insert(kFirst, *y.initial_value);
  const auto scalar = Solve(stationary_graph, stationary_refs, {kLeft, kRight, kFirst});
  EXPECT_NEAR((difference * scalar.covariance * difference.transpose())(0, 0), 0.01, 1e-11);
}

TEST(ConditionalProcessBridge, ExactConstraintsWorkWithIsamAndPrefixElimination)
{
  auto edge = Edge(0.0, 0.0);
  edge.endpoint_covariance.setZero();
  const auto id = (*edge.trajectory_query)(edge.end_ns).gyro_jacobians.begin()->first;
  edge.gyro_rate_variances[id] = 0.09;
  ConditionalProcessBridge bridge(edge);
  const auto acquisition = bridge.Insert(kSecond, kFirst);
  auto graph = Base(Edge(0.0));
  const gtsam::Key upstream = Symbol('x', 9);
  graph[0] = std::make_shared<gtsam::BetweenFactor<Pose2>>(
      upstream, kLeft, Pose2(), gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
  graph.add(gtsam::PriorFactor<Pose2>(upstream,
                                      edge.reference_values.at<Pose2>(kLeft).retract(
                                          Vector3(0.01, -0.02, 0.03)),
                                      gtsam::noiseModel::Isotropic::Sigma(3, 0.2)));
  graph.add(acquisition.factor);
  auto refs = edge.reference_values;
  refs.insert(upstream, refs.at<Pose2>(kLeft));
  refs.insert(kFirst, *acquisition.initial_value);
  const auto expected = Solve(graph, refs, {upstream, kLeft, kRight, kFirst});
  const auto late = bridge.Insert(3 * kSecond / 2, kSecondKey);
  for (const auto factorization : {gtsam::ISAM2Params::CHOLESKY, gtsam::ISAM2Params::QR})
  {
    gtsam::ISAM2Params params;
    params.factorization = factorization;
    gtsam::ISAM2 full(params);
    ASSERT_NO_THROW(full.update(graph, refs));
    EXPECT_LT((full.marginalCovariance(kRight) - expected.covariance.block<3, 3>(6, 6)).norm(),
              1e-10);
    const auto gaussian = graph.linearize(refs);
    const auto elimination = gaussian->eliminatePartialSequential(gtsam::Ordering{upstream},
                                                                  gtsam::EliminatePreferCholesky);
    gtsam::NonlinearFactorGraph retained;
    bool retained_constraint = false;
    for (const auto& factor : *elimination.second)
    {
      const gtsam::JacobianFactor jacobian(*factor);
      retained_constraint |= jacobian.get_model() && jacobian.get_model()->isConstrained();
      retained.add(std::make_shared<gtsam::LinearContainerFactor>(factor, refs));
    }
    EXPECT_TRUE(retained_constraint);
    gtsam::Values remaining;
    remaining.insert(kLeft, refs.at<Pose2>(kLeft));
    remaining.insert(kRight, refs.at<Pose2>(kRight));
    remaining.insert(kFirst, refs.at<Pose2>(kFirst));
    const auto reduced = Solve(retained, remaining, {kLeft, kRight, kFirst});
    EXPECT_LT((reduced.mean - expected.mean.tail(9)).norm(), 1e-10);
    EXPECT_LT((reduced.covariance - expected.covariance.bottomRightCorner(9, 9)).norm(), 1e-10);
    gtsam::ISAM2 rebased(params);
    ASSERT_NO_THROW(rebased.update(retained, remaining));
    EXPECT_LT(
        (rebased.marginalCovariance(kFirst) - expected.covariance.bottomRightCorner(3, 3)).norm(),
        1e-10);
    gtsam::NonlinearFactorGraph late_factors;
    late_factors.add(late.factor);
    gtsam::Values late_values;
    late_values.insert(kSecondKey, *late.initial_value);
    ASSERT_NO_THROW(full.update(late_factors, late_values));
    ASSERT_NO_THROW(rebased.update(late_factors, late_values));
    EXPECT_LT(full.calculateEstimate<Pose2>(kSecondKey)
                  .localCoordinates(rebased.calculateEstimate<Pose2>(kSecondKey))
                  .norm(),
              1e-10);
    EXPECT_LT((full.marginalCovariance(kSecondKey) - rebased.marginalCovariance(kSecondKey)).norm(),
              1e-10);
    auto unreduced_graph = graph;
    unreduced_graph.add(late.factor);
    auto all_refs = refs;
    all_refs.insert(kSecondKey, *late.initial_value);
    retained.add(late.factor);
    remaining.insert(kSecondKey, *late.initial_value);
    const auto full_late =
        Solve(unreduced_graph, all_refs, {upstream, kLeft, kRight, kFirst, kSecondKey});
    const auto reduced_late = Solve(retained, remaining, {kLeft, kRight, kFirst, kSecondKey});
    EXPECT_LT((full_late.mean.tail(12) - reduced_late.mean).norm(), 1e-10);
    EXPECT_LT((full_late.covariance.bottomRightCorner(12, 12) - reduced_late.covariance).norm(),
              1e-10);

    // Retire the complete old edge, as the live-window owner does. This
    // elimination actually touches the hard bridge rows (unlike removing
    // only upstream above), preserving their information at the separator.
    const auto retired = unreduced_graph.linearize(all_refs)->eliminatePartialSequential(
        gtsam::Ordering{upstream, kLeft, kFirst, kSecondKey}, gtsam::EliminatePreferCholesky);
    gtsam::NonlinearFactorGraph separator;
    for (const auto& factor : *retired.second)
      if (factor && !factor->empty())
        separator.add(std::make_shared<gtsam::LinearContainerFactor>(factor, all_refs));
    gtsam::Values endpoint;
    endpoint.insert(kRight, all_refs.at<Pose2>(kRight));
    const auto retired_solution = Solve(separator, endpoint, {kRight});
    EXPECT_LT((retired_solution.mean - full_late.mean.segment(6, 3)).norm(), 1e-10);
    EXPECT_LT((retired_solution.covariance - full_late.covariance.block<3, 3>(6, 6)).norm(), 1e-10);
    gtsam::ISAM2 retired_isam(params);
    ASSERT_NO_THROW(retired_isam.update(separator, endpoint));
    EXPECT_LT((retired_isam.marginalCovariance(kRight) - retired_solution.covariance).norm(),
              1e-10);
  }
}

TEST(ConditionalProcessBridge, RepeatedWheelIdentityAcrossEdgesRetainsIndependentDiffusion)
{
  auto history = std::make_shared<MotionSourceHistory>(2);
  history->Reset(7);
  ConsumedMotionStep step;
  step.epoch = 7;
  step.end_ns = kSecond;
  step.wheel.id.sequence = step.gyro.id.sequence = 1;
  step.wheel.vx = 0.8;
  ASSERT_EQ(history->Append(step), MotionHistoryStatus::Ok);
  step.start_ns = kSecond;
  step.end_ns = 2 * kSecond;
  // BOTH edge snapshots deliberately contain the same held wheel and gyro identities.
  ASSERT_EQ(history->Append(step), MotionHistoryStatus::Ok);
  const gtsam::Key end = Symbol('x', 2);
  gtsam::Values refs;
  refs.insert(kLeft, Pose2());
  refs.insert(kRight, Pose2(0.8, 0.0, 0.0));
  refs.insert(end, Pose2(1.6, 0.0, 0.0));
  const gtsam::Matrix3 covariance = Vector3(0.02, 0.001, 0.002).asDiagonal();
  auto make_edge = [&](gtsam::Key left, gtsam::Key right, int64_t start)
  {
    auto edge = Edge();
    edge.left_key = left;
    edge.right_key = right;
    edge.start_ns = start;
    edge.end_ns = start + kSecond;
    edge.reference_values.clear();
    edge.reference_values.insert(left, refs.at<Pose2>(left));
    edge.reference_values.insert(right, refs.at<Pose2>(right));
    edge.endpoint_covariance = covariance;
    edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, 7, start);
    return edge;
  };
  ConditionalProcessBridge first(make_edge(kLeft, kRight, 0));
  ConditionalProcessBridge second(make_edge(kRight, end, kSecond));
  gtsam::NonlinearFactorGraph graph;
  const auto exact = gtsam::noiseModel::Constrained::MixedSigmas(Vector3::Zero());
  graph.add(gtsam::PriorFactor<Pose2>(kLeft, refs.at<Pose2>(kLeft), exact));
  graph.add(gtsam::PriorFactor<Pose2>(end, refs.at<Pose2>(end), exact));
  const auto noise = gtsam::noiseModel::Gaussian::Covariance(covariance);
  graph.add(gtsam::BetweenFactor<Pose2>(kLeft, kRight, Pose2(0.8, 0.0, 0.0), noise));
  graph.add(gtsam::BetweenFactor<Pose2>(kRight, end, Pose2(0.8, 0.0, 0.0), noise));
  const auto baseline = Solve(graph, refs, {kLeft, kRight, end});
  const auto a = first.Insert(kSecond / 2, kFirst);
  const auto b = second.Insert(3 * kSecond / 2, kSecondKey);
  graph.add(a.factor);
  graph.add(b.factor);
  refs.insert(kFirst, *a.initial_value);
  refs.insert(kSecondKey, *b.initial_value);
  const auto augmented = Solve(graph, refs, {kLeft, kRight, end, kFirst, kSecondKey});
  EXPECT_LT((augmented.covariance.topLeftCorner(9, 9) - baseline.covariance).norm(), 1e-10);
  EXPECT_NEAR(augmented.covariance(3, 3), 0.01, 1e-11);
  // A literal shared constant-rate-error model predicts zero midpoint variance once its
  // exact two-second total is observed. The declared independent process diffusion does not.
  EXPECT_GT(std::abs(augmented.covariance(3, 3) - 0.0), 0.009);
}
}  // namespace
