// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/conditional_process_bridge.hpp"
#include <Eigen/Cholesky>
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>

namespace
{
using namespace fusion_graph;
using gtsam::Key;
using gtsam::Matrix;
using gtsam::Matrix3;
using gtsam::Pose2;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector;
using gtsam::Vector3;
constexpr int64_t kSecond = 1000000000;
const Key kLeft = Symbol('x', 0), kRight = Symbol('x', 1), kBias = Symbol('b', 1);
const std::vector<Key> kKeys{Symbol('a', 0), Symbol('a', 1), Symbol('a', 2)};
const std::vector<int64_t> kTimes{450000000, 1250000000, 1800000000};
using Mode = ConditionalProcessBridge::Mode;

FrozenProcessEdge Edge(bool pure_shared = false, bool bias = true)
{
  auto history = std::make_shared<MotionSourceHistory>(8);
  history->Reset(7);
  for (int i = 0; i < 4; ++i)
  {
    ConsumedMotionStep step;
    step.epoch = 7;
    step.start_ns = i * kSecond / 2;
    step.end_ns = (i + 1) * kSecond / 2;
    step.wheel.id.sequence = i + 1;
    step.wheel.vx = pure_shared ? 0.0 : 0.4;
    step.gyro.id.sequence = pure_shared ? 1 : i / 2 + 1;
    step.gyro.raw_z = pure_shared ? 0.03 : (i < 2 ? 0.68 : 0.18);
    step.bias_linearization = 0.03;
    EXPECT_EQ(history->Append(step), MotionHistoryStatus::Ok);
  }
  FrozenProcessEdge edge;
  edge.left_key = kLeft;
  edge.right_key = kRight;
  edge.source_epoch = 7;
  edge.end_ns = 2 * kSecond;
  edge.endpoint_covariance = pure_shared ? Matrix3::Zero().eval()
                                         : Vector3(0.04, 0.002, 0.01).asDiagonal().toDenseMatrix();
  edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, 7, 0);
  const Pose2 left(1.0, -0.4, 0.5);
  edge.reference_values.insert(kLeft, left);
  const auto endpoint = (*edge.trajectory_query)(edge.end_ns);
  edge.reference_values.insert(kRight, left.compose(endpoint.mean));
  if (bias)
  {
    edge.bias_key = kBias;
    edge.reference_values.insert(kBias, 0.03);
  }
  for (const auto& [id, jacobian] : endpoint.gyro_jacobians)
    edge.gyro_rate_variances.emplace(id, 0.09);
  return edge;
}

Values Displaced(const FrozenProcessEdge& edge, bool zero_third_error = false)
{
  auto values = edge.reference_values;
  for (size_t i = 0; i < kKeys.size(); ++i)
    values.insert(
        kKeys[i],
        values.at<Pose2>(kLeft)
            .compose((*edge.trajectory_query)(kTimes[i]).mean)
            .retract(Vector3(0.02 * (i + 1), -0.01, 0.02 + (zero_third_error ? 0.0 : 0.01 * i))));
  values.update(kLeft, values.at<Pose2>(kLeft).retract(Vector3(0.03, -0.02, 0.11)));
  values.update(kRight, values.at<Pose2>(kRight).retract(Vector3(-0.04, 0.05, -0.09)));
  if (edge.bias_key)
    values.update(kBias, 0.08);
  return values;
}

Values Shifted(const Values& values)
{
  Values shifted;
  const Pose2 action(4.0, -3.0, std::numbers::pi);
  for (const auto& item : values)
    if (item.key == kBias)
      shifted.insert(item.key, item.value);
    else
      shifted.insert(item.key, action.compose(item.value.cast<Pose2>()));
  return shifted;
}

Matrix3 Kernel(const FrozenProcessEdge& edge, int64_t a, int64_t b)
{
  const auto ta = (*edge.trajectory_query)(a), tb = (*edge.trajectory_query)(b);
  const auto end = (*edge.trajectory_query)(edge.end_ns);
  const double progress = double(std::min(a, b) - edge.start_ns) / (edge.end_ns - edge.start_ns);
  const auto pa = ta.mean.between(end.mean).AdjointMap();
  const auto pb = tb.mean.between(end.mean).AdjointMap();
  Matrix3 kernel = pa * (progress * edge.endpoint_covariance) * pb.transpose();
  for (const auto& [id, variance] : edge.gyro_rate_variances)
  {
    const auto ja = ta.gyro_jacobians.find(id), jb = tb.gyro_jacobians.find(id);
    if (ja != ta.gyro_jacobians.end() && jb != tb.gyro_jacobians.end())
      kernel += ja->second * variance * jb->second.transpose();
  }
  return kernel;
}

// Independent common-error batch, without recovering any coefficient from affine rows.
Vector3 Coordinate(const FrozenProcessEdge& edge, int64_t time, Key key, const Values& values)
{
  const auto trajectory = (*edge.trajectory_query)(time);
  const double delta = edge.bias_key ? values.at<double>(kBias) - 0.03 : 0.0;
  const auto mean = trajectory.mean.compose(Pose2::Expmap(trajectory.bias_jacobian * delta));
  const auto discrepancy = mean.between(values.at<Pose2>(kLeft).between(values.at<Pose2>(key)));
  // Independent stable SE(2) logarithm: V(theta)^-1 = alpha I - theta/2 J.
  // In particular keep the exact-zero yaw fixture: central differences of GTSAM's
  // Logmap near zero can lose accuracy through trigonometric cancellation.
  const double theta = discrepancy.theta(), half = theta * 0.5;
  const double square = theta * theta;
  const double alpha = std::abs(theta) < 1e-4 ? 1.0 - square / 12.0 - square * square / 720.0 -
                                                    square * square * square / 30240.0
                                              : half / std::tan(half);
  return Vector3(alpha * discrepancy.x() + half * discrepancy.y(),
                 alpha * discrepancy.y() - half * discrepancy.x(),
                 theta);
}

TEST(RelativeConditional, CanonicalAffineRowsAndFiniteBiasJacobians)
{
  const auto edge = Edge();
  ConditionalProcessBridge affine(edge), relative(edge, Mode::RelativeErrors);
  auto refs = edge.reference_values;
  const auto live = Displaced(edge);
  for (size_t i = 0; i < kKeys.size(); ++i)
  {
    const auto a = affine.Insert(kTimes[i], kKeys[i]);
    const auto b = relative.InsertRelative(kTimes[i], kKeys[i]);
    ASSERT_TRUE(a.factor);
    ASSERT_TRUE(b.factor);
    refs.insert(kKeys[i], *a.initial_value);
    gtsam::JacobianFactor old(*a.factor->factor());
    std::vector<Matrix> jacobians;
    const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(b.factor.get());
    ASSERT_NE(factor, nullptr);
    const auto at_reference = factor->unwhitenedError(refs, &jacobians);
    EXPECT_LT((at_reference + old.getb()).norm(), 1e-11);
    for (size_t j = 0; j < factor->keys().size(); ++j)
      EXPECT_LT((jacobians[j] - old.getA(factor->keys()[j])).norm(), 1e-10);
    ASSERT_TRUE(factor->noiseModel()->equals(*old.get_model(), 1e-12));
    const auto residual = factor->unwhitenedError(live, &jacobians);
    EXPECT_LT((residual - factor->unwhitenedError(Shifted(live))).norm(), 1e-10);
    for (size_t j = 0; j < factor->keys().size(); ++j)
    {
      const auto key = factor->keys()[j];
      for (Eigen::Index column = 0; column < jacobians[j].cols(); ++column)
      {
        constexpr double epsilon = 1e-6;
        auto plus = live, minus = live;
        if (key == kBias)
        {
          plus.update(key, live.at<double>(key) + epsilon);
          minus.update(key, live.at<double>(key) - epsilon);
        }
        else
        {
          Vector3 delta = Vector3::Zero();
          delta(column) = epsilon;
          plus.update(key, live.at<Pose2>(key).retract(delta));
          minus.update(key, live.at<Pose2>(key).retract(-delta));
        }
        const Vector numerical =
            (factor->unwhitenedError(plus) - factor->unwhitenedError(minus)) / (2 * epsilon);
        EXPECT_LT((numerical - jacobians[j].col(column)).norm(), 1e-7);
      }
    }
    EXPECT_TRUE(factor->equals(*factor->clone(), 1e-12));
    ConditionalProcessBridge other(edge, Mode::RelativeErrors);
    const auto different = other.InsertRelative(kTimes[(i + 1) % kTimes.size()], kKeys[i]);
    EXPECT_FALSE(factor->equals(*different.factor));
  }
}

TEST(RelativeConditional, OutOfTimeInsertionsMatchJointCostGradientAndInformation)
{
  for (const bool zero_third_error : {false, true})
  {
    SCOPED_TRACE(zero_third_error);
    const auto edge = Edge();
    const auto live = Displaced(edge, zero_third_error);
    Matrix covariance(9, 9), cross(9, 3);
    const Matrix3 endpoint_inverse = Kernel(edge, edge.end_ns, edge.end_ns).inverse();
    for (int i = 0; i < 3; ++i)
    {
      cross.middleRows<3>(3 * i) = Kernel(edge, kTimes[i], edge.end_ns);
      for (int j = 0; j < 3; ++j)
        covariance.block<3, 3>(3 * i, 3 * j) = Kernel(edge, kTimes[i], kTimes[j]);
    }
    covariance -= cross * endpoint_inverse * cross.transpose();
    const Matrix precision = covariance.inverse();
    const auto batch_residual = [&](const Values& values)
    {
      Vector residual(9);
      const auto endpoint = Coordinate(edge, edge.end_ns, kRight, values);
      for (int i = 0; i < 3; ++i)
        residual.segment<3>(3 * i) = Coordinate(edge, kTimes[i], kKeys[i], values) -
                                     cross.middleRows<3>(3 * i) * endpoint_inverse * endpoint;
      return residual;
    };
    const Vector batch = batch_residual(live);
    // Columns are left, right, scalar bias, then the three interior states.
    const std::vector<Key> keys{kLeft, kRight, kBias, kKeys[0], kKeys[1], kKeys[2]};
    Matrix batch_jacobian(9, 16);
    int column = 0;
    for (const auto key : keys)
      for (int j = 0; j < (key == kBias ? 1 : 3); ++j, ++column)
      {
        constexpr double epsilon = 1e-6;
        auto plus = live, minus = live;
        if (key == kBias)
        {
          plus.update(key, live.at<double>(key) + epsilon);
          minus.update(key, live.at<double>(key) - epsilon);
        }
        else
        {
          Vector3 delta = Vector3::Zero();
          delta(j) = epsilon;
          plus.update(key, live.at<Pose2>(key).retract(delta));
          minus.update(key, live.at<Pose2>(key).retract(-delta));
        }
        batch_jacobian.col(column) = (batch_residual(plus) - batch_residual(minus)) / (2 * epsilon);
      }
    const Matrix batch_information = batch_jacobian.transpose() * precision * batch_jacobian;
    const Vector batch_gradient = batch_jacobian.transpose() * precision * batch;
    const double batch_cost = 0.5 * batch.dot(precision * batch);
    for (const auto& order : {std::vector<int>{1, 0, 2}, std::vector<int>{0, 1, 2}})
    {
      ConditionalProcessBridge bridge(edge, Mode::RelativeErrors);
      double cost = 0.0;
      Matrix information = Matrix::Zero(16, 16);
      Vector gradient = Vector::Zero(16);
      for (const int i : order)
      {
        const auto insertion = bridge.InsertRelative(kTimes[i], kKeys[i]);
        const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(insertion.factor.get());
        ASSERT_NE(factor, nullptr);
        std::vector<Matrix> blocks;
        const auto error = factor->unwhitenedError(live, &blocks);
        const auto whitened = factor->noiseModel()->whiten(error);
        Matrix jacobian = Matrix::Zero(3, 16);
        int offset = 0;
        for (const auto key : keys)
        {
          const auto found = std::find(factor->keys().begin(), factor->keys().end(), key);
          const int width = key == kBias ? 1 : 3;
          if (found != factor->keys().end())
            jacobian.middleCols(offset, width) =
                factor->noiseModel()->Whiten(blocks[found - factor->keys().begin()]);
          offset += width;
        }
        cost += 0.5 * whitened.squaredNorm();
        information += jacobian.transpose() * jacobian;
        gradient += jacobian.transpose() * whitened;
      }
      EXPECT_NEAR(cost, batch_cost, 1e-8);
      EXPECT_LT((gradient - batch_gradient).norm() / batch_gradient.norm(), 1e-7);
      EXPECT_LT((information - batch_information).norm() / batch_information.norm(), 1e-7);
      const Matrix eliminated =
          information.topLeftCorner(7, 7) -
          information.topRightCorner(7, 9) *
              information.bottomRightCorner(9, 9).ldlt().solve(information.bottomLeftCorner(9, 7));
      EXPECT_LT(eliminated.norm(), 1e-8 * information.norm());
    }
  }
}

TEST(RelativeConditional, NoncanonicalEndpointExactInitializationAndReadOnlyQuery)
{
  auto edge = Edge();
  edge.reference_values.update(kRight,
                               edge.reference_values.at<Pose2>(kRight).retract(
                                   Vector3(0.7, -0.3, 0.9)));
  ConditionalProcessBridge bridge(edge, Mode::RelativeErrors);
  auto values = edge.reference_values;
  values.update(kBias, 0.08);
  const auto predicted = bridge.RelativeInitialValue(kTimes[1], values);
  const auto insertion = bridge.InsertRelative(kTimes[1], kKeys[1], &values);
  ASSERT_TRUE(insertion.initial_value);
  EXPECT_LT(predicted.localCoordinates(*insertion.initial_value).norm(), 1e-12);
  values.insert(kKeys[1], *insertion.initial_value);
  const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(insertion.factor.get());
  ASSERT_NE(factor, nullptr);
  EXPECT_LT(factor->unwhitenedError(values).norm(), 1e-10);
  const auto earlier = bridge.RelativeInitialValue(kTimes[0], values);
  const auto second = bridge.InsertRelative(kTimes[0], kKeys[0], &values);
  EXPECT_LT(earlier.localCoordinates(*second.initial_value).norm(), 1e-12);
  values.insert(kKeys[0], *second.initial_value);
  EXPECT_LT(dynamic_cast<const RelativeConditionalFactor*>(second.factor.get())
                ->unwhitenedError(values)
                .norm(),
            1e-10);
  EXPECT_EQ(bridge.InsertRelative(kTimes[0], kKeys[2]).key, kKeys[0]);
  EXPECT_FALSE(bridge.InsertRelative(0, kKeys[2]).factor);
  EXPECT_EQ(bridge.InsertRelative(edge.end_ns, kKeys[2]).key, kRight);
  EXPECT_TRUE(bridge.RelativeInitialValue(kTimes[0], values).equals(values.at<Pose2>(kKeys[0])));
  EXPECT_TRUE(bridge.RelativeInitialValue(0, values).equals(values.at<Pose2>(kLeft)));
  EXPECT_TRUE(bridge.RelativeInitialValue(edge.end_ns, values).equals(values.at<Pose2>(kRight)));
}

TEST(RelativeConditional, ExactSharedNoiseSupportAndConstrainedElimination)
{
  const auto edge = Edge(true);
  ConditionalProcessBridge bridge(edge, Mode::RelativeErrors);
  auto values = edge.reference_values;
  values.update(kBias, 0.08);
  values.update(kRight, values.at<Pose2>(kLeft).compose(Pose2(0, 0, 0.3)));
  for (const int i : {1, 0, 2})
  {
    const auto insertion = bridge.InsertRelative(kTimes[i], kKeys[i], &values);
    values.insert(kKeys[i], *insertion.initial_value);
    const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(insertion.factor.get());
    ASSERT_NE(factor, nullptr);
    ASSERT_NE(dynamic_cast<const gtsam::noiseModel::Constrained*>(factor->noiseModel().get()),
              nullptr);
    EXPECT_EQ((factor->noiseModel()->sigmas().array() == 0.0).count(), 3);
    std::vector<Matrix> blocks;
    EXPECT_LT(factor->unwhitenedError(values, &blocks).norm(), 1e-10);
    const auto target_index =
        std::find(factor->keys().begin(), factor->keys().end(), kKeys[i]) - factor->keys().begin();
    // Every exact conditional row has an invertible target block, so any conditioning
    // increment admits an interior increment satisfying all three constraints exactly.
    const Matrix target = blocks[target_index];
    EXPECT_GT(std::abs(target.determinant()), 1e-6);
    for (size_t j = 0; j < blocks.size(); ++j)
      if (j != static_cast<size_t>(target_index))
      {
        EXPECT_LT((target * target.fullPivLu().solve(-blocks[j]) + blocks[j]).norm(), 1e-10);
      }
    auto infeasible = values;
    infeasible.update(kKeys[i], values.at<Pose2>(kKeys[i]).retract(Vector3(0.01, 0.02, 0.03)));
    EXPECT_GT(factor->unwhitenedError(infeasible).norm(), 1e-3);
    const auto linear = std::dynamic_pointer_cast<gtsam::JacobianFactor>(factor->linearize(values));
    ASSERT_TRUE(linear);
    EXPECT_TRUE(linear->get_model()->isConstrained());
    EXPECT_EQ((linear->get_model()->sigmas().array() == 0.0).count(), 3);
  }
}

TEST(RelativeConditional, UnrepresentableMeanRejectsWithoutAllocatingAcquisition)
{
  auto edge = Edge(true, false);
  const auto original = edge.trajectory_query;
  // A valid rank-one source kernel with a target source sensitivity greater than the
  // endpoint's. This deliberately makes the conditional error exceed the principal branch.
  edge.trajectory_query = std::make_shared<const FrozenProcessEdge::TrajectoryQuery>(
      [original](int64_t time)
      {
        auto trajectory = (*original)(time);
        if (time == kTimes[0])
          for (auto& [id, jacobian] : trajectory.gyro_jacobians)
            jacobian *= 20.0;
        return trajectory;
      });
  ConditionalProcessBridge bridge(edge, Mode::RelativeErrors);
  auto values = edge.reference_values;
  values.update(kRight, values.at<Pose2>(kLeft).compose(Pose2(0, 0, 1.0)));
  EXPECT_THROW(bridge.RelativeInitialValue(kTimes[0], values), std::invalid_argument);
  EXPECT_THROW(bridge.InsertRelative(kTimes[0], kKeys[0], &values), std::invalid_argument);
  values.update(kRight, values.at<Pose2>(kLeft).compose(Pose2(0, 0, 0.1)));
  const auto insertion = bridge.InsertRelative(kTimes[0], kKeys[0], &values);
  ASSERT_TRUE(insertion.factor);
  ASSERT_TRUE(insertion.initial_value);
  values.insert(kKeys[0], *insertion.initial_value);
  EXPECT_LT(dynamic_cast<const RelativeConditionalFactor*>(insertion.factor.get())
                ->unwhitenedError(values)
                .norm(),
            1e-10);
}

TEST(RelativeConditional, ModeIsExplicitAndNoBiasIsSupported)
{
  const auto edge = Edge(false, false);
  ConditionalProcessBridge affine(edge), relative(edge, Mode::RelativeErrors);
  EXPECT_EQ(affine.mode(), Mode::FrozenAffine);
  EXPECT_EQ(relative.mode(), Mode::RelativeErrors);
  EXPECT_THROW(affine.InsertRelative(kTimes[0], kKeys[0]), std::logic_error);
  EXPECT_THROW(relative.Insert(kTimes[0], kKeys[0]), std::logic_error);
  EXPECT_THROW(relative.RelativeInitialValue(-1, edge.reference_values), std::out_of_range);
  const auto inserted = relative.InsertRelative(kTimes[0], kKeys[0]);
  auto values = edge.reference_values;
  values.insert(kKeys[0], *inserted.initial_value);
  const auto* factor = dynamic_cast<const RelativeConditionalFactor*>(inserted.factor.get());
  ASSERT_NE(factor, nullptr);
  EXPECT_EQ(factor->keys().size(), 3u);
  EXPECT_LT(factor->unwhitenedError(values).norm(), 1e-12);
}

}  // namespace
