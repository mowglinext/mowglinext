// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/factors.hpp"
#include "fusion_graph/shared_source_process_family.hpp"
#include <Eigen/Cholesky>
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace
{
using namespace fusion_graph;
using namespace gtsam;
constexpr int64_t second = 1000000000;
constexpr uint64_t epoch = 17;
const Key l = Symbol('x', 0), s = Symbol('x', 1), v = Symbol('x', 2);
const Key qa = Symbol('a', 0), qb = Symbol('a', 1), bs = Symbol('b', 0), bv = Symbol('b', 1);
const Pose2 origin(1., -.4, .6);
const Matrix3 dp = Vector3(.04, .002, .25).asDiagonal();
const Matrix3 df = Vector3(.03, .001, .16).asDiagonal();
SharedNoiseModel Diagonal(const Vector& sigma)
{
  return noiseModel::Diagonal::Sigmas(sigma);
}

MotionGyroSource Source(uint64_t sequence, int64_t start, int64_t end)
{
  return {{MotionSourceKind::Gyro, sequence, end}, {start, end}, 0.};
}
SharedSourceProcessWindow Window(
    int64_t start,
    int64_t end,
    Key left,
    Key right,
    Key bias,
    const Matrix3& diffusion,
    const std::vector<std::tuple<int64_t, int64_t, MotionGyroSource, double>>& uses,
    bool off = false,
    double recorded_bias = 0.0,
    double wheel_vx = 0.0)
{
  auto history = std::make_shared<MotionSourceHistory>(uses.size());
  history->Reset(epoch);
  SharedSourceProcessWindow window;
  auto& edge = window.edge;
  edge.left_key = left;
  edge.right_key = right;
  edge.source_epoch = epoch;
  edge.start_ns = start;
  edge.end_ns = end;
  edge.endpoint_covariance = diffusion;
  edge.reference_values.insert(left, origin);
  edge.reference_values.insert(right, origin);
  if (!off)
  {
    edge.bias_key = bias;
    edge.reference_values.insert(bias, 0.);
    window.gyro.emplace();
    window.gyro->duration_s = double(end - start) * 1e-9;
  }
  for (const auto& [begin, finish, gyro, variance] : uses)
  {
    ConsumedMotionStep step;
    step.epoch = epoch;
    step.start_ns = begin;
    step.end_ns = finish;
    step.wheel.id = {MotionSourceKind::Wheel, 1, 0};
    step.wheel.support = {0, 2 * second};
    step.wheel.vx = wheel_vx;
    step.gyro = gyro;
    step.bias_linearization = recorded_bias;
    EXPECT_EQ(history->Append(step), MotionHistoryStatus::Ok);
    if (!off)
    {
      edge.gyro_rate_variances.emplace(gyro.id, variance);
      window.gyro->sources.push_back({gyro, double(finish - begin) * 1e-9});
      window.gyro->raw_integral_rad += gyro.raw_z * double(finish - begin) * 1e-9;
    }
  }
  edge.trajectory_query = FrozenProcessEdge::HistoryQuery(history, epoch, start);
  return window;
}

std::vector<SharedSourceProcessWindow> Windows(bool split = false, bool off = false)
{
  const auto a = Source(1, 0, 800000000), c = Source(2, 800000000, 1200000000);
  const auto suffix_c = split ? Source(3, 800000000, 1200000000) : c;
  const auto d = Source(split ? 4 : 3, 1200000000, 2 * second);
  return {
      Window(
          0, second, l, s, bs, dp, {{0, 800000000, a, .0004}, {800000000, second, c, .0004}}, off),
      Window(second,
             2 * second,
             s,
             v,
             bv,
             df,
             {{second, 1200000000, suffix_c, .0004}, {1200000000, 2 * second, d, .0004}},
             off)};
}
SharedSourceProcessWindow Unequal()
{
  return Window(0,
                second,
                l,
                s,
                bs,
                Vector3(.04, .003, 1.).asDiagonal(),
                {{0, second / 2, Source(1, 0, second / 2), 4.},
                 {second / 2, second, Source(2, second / 2, second), 1.}});
}
Values References(const std::vector<SharedSourceProcessWindow>& windows)
{
  Values values;
  for (const auto& window : windows)
    for (const auto& value : window.edge.reference_values)
      if (!values.exists(value.key))
        values.insert(value.key, value.value);
  return values;
}
Values Displaced(bool off = false)
{
  auto values = References(Windows(false, off));
  values.update(l, origin.retract(Vector3(.02, -.01, .04)));
  values.update(s, origin.retract(Vector3(-.01, .015, -.02)));
  values.update(v, origin.retract(Vector3(.01, -.015, .03)));
  values.insert(qa, origin.retract(Vector3(-.02, .02, -.03)));
  values.insert(qb, origin.retract(Vector3(.015, -.01, .02)));
  if (!off)
  {
    values.update(bs, .008);
    values.update(bv, -.006);
  }
  return values;
}
Ordering Order(bool off = false)
{
  Ordering result{l, s, v, qa, qb};
  if (!off)
  {
    result.push_back(bs);
    result.push_back(bv);
  }
  return result;
}

// Oracle geometry and covariance are explicit raw-input calculations for the
// zero-rate fixture, not outputs of the family or a shared model builder.
Vector3 Log(const Pose2& pose)
{
  const double angle = pose.theta(), half = angle * .5, square = angle * angle;
  const double alpha = std::abs(angle) < 1e-4 ? 1. - square / 12. - square * square / 720. -
                                                    square * square * square / 30240.
                                              : half / std::tan(half);
  return Vector3(alpha * pose.x() + half * pose.y(), alpha * pose.y() - half * pose.x(), angle);
}
Vector3 Coordinate(const Values& values, Key left, Key target, double dt, double bias)
{
  return Log(
      Pose2(0, 0, -dt * bias).between(values.at<Pose2>(left).between(values.at<Pose2>(target))));
}
Vector OracleResidual(const Values& values, bool off)
{
  const double bp = off ? 0. : values.at<double>(bs), bf = off ? 0. : values.at<double>(bv);
  Vector residual(off ? 6 : 8);
  int offset = 0;
  if (!off)
  {
    residual(0) = WrapAngle(values.at<Pose2>(s).theta() - values.at<Pose2>(l).theta() + bp);
    residual(1) = WrapAngle(values.at<Pose2>(v).theta() - values.at<Pose2>(s).theta() + bf);
    offset = 2;
  }
  residual.segment<3>(offset) =
      Coordinate(values, l, qa, .4, bp) - .4 * Coordinate(values, l, s, 1., bp);
  residual.segment<3>(offset + 3) =
      Coordinate(values, s, qb, .6, bf) - .6 * Coordinate(values, s, v, 1., bf);
  return residual;
}
Matrix OracleCovariance(bool split, bool off)
{
  Matrix covariance = Matrix::Zero(off ? 6 : 8, off ? 6 : 8);
  const int offset = off ? 0 : 2;
  covariance.block<3, 3>(offset, offset) = .24 * dp;
  covariance.block<3, 3>(offset + 3, offset + 3) = .24 * df;
  if (!off)
  {
    // A=[.8,.2,0;0,.2,.8], Fp_yaw=[.08,-.08,0],
    // Ff_yaw=[0,.08,-.08]. Split IDs remove only cross-window source terms.
    constexpr double q = .0004;
    covariance(0, 0) = covariance(1, 1) = .68 * q;
    covariance(0, 1) = covariance(1, 0) = split ? 0. : .04 * q;
    covariance(4, 4) += .0128 * q;
    covariance(7, 7) += .0128 * q;
    covariance(4, 7) = covariance(7, 4) = split ? 0. : -.0064 * q;
    covariance(0, 4) = covariance(4, 0) = .048 * q;
    covariance(1, 4) = covariance(4, 1) = split ? 0. : -.016 * q;
    covariance(0, 7) = covariance(7, 0) = split ? 0. : .016 * q;
    covariance(1, 7) = covariance(7, 1) = -.048 * q;
  }
  return covariance;
}
class DenseOracle : public NoiseModelFactor
{
public:
  DenseOracle(bool split, bool off)
      : NoiseModelFactor(noiseModel::Gaussian::Covariance(OracleCovariance(split, off)),
                         Order(off)),
        off_(off)
  {
  }
  Vector unwhitenedError(const Values& values, OptionalMatrixVecType h = nullptr) const override
  {
    if (h)
    {
      h->resize(keys().size());
      for (size_t i = 0; i < keys().size(); ++i)
      {
        const Key key = keys()[i];
        const bool bias = key == bs || key == bv;
        (*h)[i].resize(off_ ? 6 : 8, bias ? 1 : 3);
        for (int c = 0; c < (bias ? 1 : 3); ++c)
        {
          constexpr double epsilon = 1e-6;
          Values plus = values, minus = values;
          if (bias)
          {
            plus.update(key, values.at<double>(key) + epsilon);
            minus.update(key, values.at<double>(key) - epsilon);
          }
          else
          {
            Vector3 delta = Vector3::Zero();
            delta(c) = epsilon;
            plus.update(key, values.at<Pose2>(key).retract(delta));
            minus.update(key, values.at<Pose2>(key).retract(-delta));
          }
          (*h)[i].col(c) =
              (OracleResidual(plus, off_) - OracleResidual(minus, off_)) / (2 * epsilon);
        }
      }
    }
    return OracleResidual(values, off_);
  }

private:
  bool off_;
};
NonlinearFactorGraph FamilyFactors(SharedSourceProcessFamily& family, bool reverse)
{
  NonlinearFactorGraph graph;
  if (auto gyro = family.endpoint_gyro_factor())
    graph.push_back(gyro);
  for (int i : (reverse ? std::vector<int>{1, 0} : std::vector<int>{0, 1}))
  {
    const auto inserted = family.Insert(i ? 1600000000 : 400000000, i ? qb : qa);
    EXPECT_TRUE(inserted.initial_value);
    EXPECT_TRUE(inserted.factor);
    graph.push_back(inserted.factor);
  }
  return graph;
}
NonlinearFactorGraph Common(bool off)
{
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Pose2>>(l, s, Pose2(), noiseModel::Gaussian::Covariance(dp));
  graph.emplace_shared<BetweenFactor<Pose2>>(s, v, Pose2(), noiseModel::Gaussian::Covariance(df));
  if (!off)
  {
    graph.emplace_shared<PriorFactor<double>>(bs, 0., Diagonal(Vector1(.05)));
    graph.emplace_shared<BetweenFactor<double>>(bs, bv, 0., Diagonal(Vector1(.001)));
  }
  const Vector2 lever(.23, -.08);
  graph.emplace_shared<GnssLeverArmFactor>(s,
                                           origin.transformFrom(lever) + Vector2(.017, -.009),
                                           lever,
                                           Diagonal(Vector2(.003, .003)));
  graph.emplace_shared<GnssLeverArmFactor>(v,
                                           origin.transformFrom(lever) + Vector2(-.006, .01),
                                           lever,
                                           Diagonal(Vector2(.012, .012)));
  graph.emplace_shared<YawUnaryFactor>(qa, origin.theta() + .045, Diagonal(Vector1(.08)));
  graph.emplace_shared<YawUnaryFactor>(qb, origin.theta() - .035, Diagonal(Vector1(.10)));
  return graph;
}
Matrix Covariance(const NonlinearFactorGraph& graph, const Values& values, bool off)
{
  const Matrix h = graph.linearize(values)->hessian(Order(off)).first;
  return h.ldlt().solve(Matrix::Identity(h.rows(), h.cols()));
}

TEST(SharedSourceProcessFamily, HandDerivedUnequalVariancePinnedConditional)
{
  SharedSourceProcessFamily family({Unequal()});
  auto values = References({Unequal()});
  values.update(l, Pose2());
  values.update(s, Pose2(0, 0, .2));
  values.update(bs, .1);
  const auto query = family.Insert(second / 2, qa, &values);
  ASSERT_TRUE(query.initial_value);
  // H=.5, F=[.25,-.25], Q=diag(4,1), C=1.25, B=.3, R_yaw=.45.
  EXPECT_NEAR(query.initial_value->theta(), .19, 1e-12);
  values.insert(qa, Pose2(0, 0, .14));
  EXPECT_NEAR(family.endpoint_gyro_factor()->error(values), .5 * .3 * .3 / 1.25, 1e-12);
  EXPECT_NEAR(query.factor->error(values), .5 * .05 * .05 / .45, 1e-12);
  EXPECT_GT(std::abs(query.factor->error(values) - .5 * .05 * .05 / .5), 1e-4);
}

TEST(SharedSourceProcessFamily, IndependentTwoQueryJointMatchesBothOrdersAndFullPosterior)
{
  for (bool off : {false, true})
    for (bool split : {false, true})
      for (bool reverse : {false, true})
      {
        SCOPED_TRACE(::testing::Message()
                     << "off=" << off << " split=" << split << " reverse=" << reverse);
        SharedSourceProcessFamily family(Windows(split, off));
        auto actual = FamilyFactors(family, reverse);
        NonlinearFactorGraph expected;
        expected.emplace_shared<DenseOracle>(split, off);
        const auto at = Displaced(off);
        EXPECT_GT(expected.error(at), 1e-4);
        EXPECT_NEAR(actual.error(at), expected.error(at), 1e-8);
        const auto a = actual.linearize(at)->hessian(Order(off)),
                   e = expected.linearize(at)->hessian(Order(off));
        EXPECT_LT((a.first - e.first).norm() / e.first.norm(), 1e-7);
        EXPECT_LT((a.second - e.second).norm() / e.second.norm(), 1e-7);
        const auto common = Common(off);
        actual.push_back(common);
        expected.push_back(common);
        LevenbergMarquardtParams params;
        params.setMaxIterations(80);
        params.setRelativeErrorTol(1e-12);
        params.setAbsoluteErrorTol(1e-12);
        const auto am = LevenbergMarquardtOptimizer(actual, at, params).optimize();
        const auto em = LevenbergMarquardtOptimizer(expected, at, params).optimize();
        for (Key key : Order(off))
          if (key == bs || key == bv)
            EXPECT_NEAR(am.at<double>(key), em.at<double>(key), 2e-6);
          else
            EXPECT_LT(am.at<Pose2>(key).localCoordinates(em.at<Pose2>(key)).cwiseAbs().maxCoeff(),
                      2e-6);
        EXPECT_LT(
            (Covariance(actual, am, off) - Covariance(expected, em, off)).cwiseAbs().maxCoeff(),
            2e-8);
        EXPECT_GT(actual.error(am), 1e-4);
      }
}

TEST(SharedSourceProcessFamily, SourceIdentityChangesJointWithoutChangingWindowMarginals)
{
  SharedSourceProcessFamily shared(Windows()), split(Windows(true));
  const auto a = FamilyFactors(shared, false), b = FamilyFactors(split, false);
  const auto at = Displaced();
  const auto ah = a.linearize(at)->hessian(Order()).first,
             bh = b.linearize(at)->hessian(Order()).first;
  EXPECT_GT((ah - bh).norm(), 1e-3);
  const Matrix same = OracleCovariance(false, false), different = OracleCovariance(true, false);
  EXPECT_DOUBLE_EQ(same(0, 0), different(0, 0));
  EXPECT_DOUBLE_EQ(same(1, 1), different(1, 1));
  EXPECT_DOUBLE_EQ(same(4, 4), different(4, 4));
  EXPECT_DOUBLE_EQ(same(7, 7), different(7, 7));
  EXPECT_GT(std::abs(same(0, 1) - different(0, 1)), 1e-6);
}

TEST(SharedSourceProcessFamily, AliasesCopiesAndReadOnlyQueriesHaveIndependentRegistries)
{
  SharedSourceProcessFamily family(Windows());
  for (const auto& [time, key] :
       std::vector<std::pair<int64_t, Key>>{{0, l}, {second, s}, {2 * second, v}})
  {
    const auto alias = family.Insert(time, qa);
    EXPECT_EQ(alias.key, key);
    EXPECT_FALSE(alias.initial_value);
    EXPECT_FALSE(alias.factor);
  }
  EXPECT_TRUE(family.acquisitions().empty());
  const auto first = family.Insert(400000000, qa);
  SharedSourceProcessFamily copied(family), assigned(Windows());
  assigned = family;
  const auto alias = family.Insert(400000000, qb);
  EXPECT_EQ(alias.key, qa);
  EXPECT_FALSE(alias.factor);
  auto values = References(Windows());
  values.insert(qa, *first.initial_value);
  const auto mean = family.ConditionalMean(1600000000, values);
  EXPECT_EQ(family.acquisitions().size(), 1u);
  const auto inserted = family.Insert(1600000000, qb, &values);
  EXPECT_LT(mean.localCoordinates(*inserted.initial_value).norm(), 1e-12);
  EXPECT_EQ(family.acquisitions().size(), 2u);
  EXPECT_EQ(copied.acquisitions().size(), 1u);
  EXPECT_EQ(assigned.acquisitions().size(), 1u);
  EXPECT_EQ(family.source_epoch(), epoch);
  EXPECT_EQ(family.start_ns(), 0);
  EXPECT_EQ(family.end_ns(), 2 * second);
  EXPECT_THROW(copied.Insert(1600000000, bs), std::invalid_argument);
  EXPECT_THROW(copied.Insert(-1, qb), std::out_of_range);
}

TEST(SharedSourceProcessFamily, FiniteBiasJacobiansMatchIndependentNumericalDerivatives)
{
  SharedSourceProcessFamily family(Windows());
  const auto graph = FamilyFactors(family, true);
  const auto values = Displaced();
  for (const auto& base : graph)
  {
    const auto* factor = dynamic_cast<const NoiseModelFactor*>(base.get());
    ASSERT_NE(factor, nullptr);
    EXPECT_TRUE(factor->equals(*factor->clone()));
    std::vector<Matrix> blocks;
    factor->unwhitenedError(values, &blocks);
    for (size_t i = 0; i < factor->keys().size(); ++i)
    {
      const Key key = factor->keys()[i];
      if (key != bs && key != bv)
        continue;
      Values plus = values, minus = values;
      constexpr double epsilon = 1e-6;
      plus.update(key, values.at<double>(key) + epsilon);
      minus.update(key, values.at<double>(key) - epsilon);
      const Vector numeric =
          (factor->unwhitenedError(plus) - factor->unwhitenedError(minus)) / (2 * epsilon);
      EXPECT_LT((numeric - blocks[i].col(0)).norm() / std::max(1., numeric.norm()), 1e-7);
    }
  }
}

TEST(SharedSourceProcessFamily, BranchFailureDoesNotRegisterOrPoisonNextInsertion)
{
  const auto window = Window(0,
                             second,
                             l,
                             s,
                             bs,
                             Vector3(.04, .003, 1.).asDiagonal(),
                             {{0, second / 2, Source(1, 0, second / 2), 4.},
                              {second / 2, second, Source(2, second / 2, second), 1.}},
                             false,
                             0.,
                             .4);
  SharedSourceProcessFamily family({window});
  auto values = References({window});
  values.update(s, origin.compose(Pose2(.4, 0, 0)));
  const auto first = family.Insert(second / 4, qa, &values);
  ASSERT_TRUE(first.initial_value);
  // A valid translating history couples prior-query lateral error into yaw.
  // This large current error asks for a mean outside the principal Log branch.
  values.insert(qa, first.initial_value->compose(Pose2(0, 2., 0)));
  EXPECT_THROW(family.ConditionalMean(3 * second / 4, values), UnsupportedSharedSourceConditional);
  EXPECT_THROW(family.Insert(3 * second / 4, qb, &values), UnsupportedSharedSourceConditional);
  EXPECT_EQ(family.acquisitions().size(), 1u);
  values.update(qa, first.initial_value->compose(Pose2(0, .02, 0)));
  EXPECT_TRUE(family.Insert(3 * second / 4, qb, &values).factor);
  EXPECT_EQ(family.acquisitions().size(), 2u);
}

std::vector<SharedSourceProcessWindow> RankOne()
{
  const auto held = Source(1, 0, 2 * second);
  return {Window(0, second, l, s, bs, Matrix3::Zero(), {{0, second, held, .09}}),
          Window(second, 2 * second, s, v, bv, Matrix3::Zero(), {{second, 2 * second, held, .09}})};
}
TEST(SharedSourceProcessFamily, ExactRankSupportRejectsInconsistentMeanAndWorksInGtsam)
{
  SharedSourceProcessFamily family(RankOne());
  auto values = References(RankOne());
  values.update(l, Pose2());
  values.update(s, Pose2(0, 0, .1));
  values.update(v, Pose2(0, 0, .3));
  EXPECT_THROW(family.Insert(second / 2, qa, &values), UnsupportedSharedSourceConditional);
  EXPECT_TRUE(family.acquisitions().empty());
  values.update(v, Pose2(0, 0, .2));
  const auto query = family.Insert(second / 2, qa, &values);
  ASSERT_TRUE(query.initial_value);
  values.insert(qa, *query.initial_value);
  const auto* gyro = dynamic_cast<const NoiseModelFactor*>(family.endpoint_gyro_factor().get());
  const auto* pinned = dynamic_cast<const NoiseModelFactor*>(query.factor.get());
  ASSERT_NE(gyro, nullptr);
  ASSERT_NE(pinned, nullptr);
  EXPECT_EQ((gyro->noiseModel()->sigmas().array() == 0.).count(), 1);
  EXPECT_EQ((pinned->noiseModel()->sigmas().array() == 0.).count(), 3);
  NonlinearFactorGraph graph;
  graph.push_back(family.endpoint_gyro_factor());
  graph.push_back(query.factor);
  // External soft anchors isolate the exact support behavior of these factors.
  for (Key key : {l, s, v})
    graph.emplace_shared<PriorFactor<Pose2>>(key,
                                             values.at<Pose2>(key),
                                             noiseModel::Isotropic::Sigma(3, .1));
  graph.emplace_shared<PriorFactor<double>>(bs, 0., Diagonal(Vector1(.05)));
  graph.emplace_shared<BetweenFactor<double>>(bs, bv, 0., Diagonal(Vector1(.001)));
  for (auto factorization : {ISAM2Params::CHOLESKY, ISAM2Params::QR})
  {
    ISAM2Params params;
    params.factorization = factorization;
    ISAM2 isam(params);
    ASSERT_NO_THROW(isam.update(graph, values));
    const auto estimate = isam.calculateEstimate();
    const Vector residual = gyro->unwhitenedError(estimate);
    for (Eigen::Index i = 0; i < residual.size(); ++i)
      if (gyro->noiseModel()->sigmas()(i) == 0.)
        EXPECT_NEAR(residual(i), 0., 1e-10);
    EXPECT_LT(pinned->unwhitenedError(estimate).norm(), 1e-10);
  }
}

TEST(SharedSourceProcessFamily, ActiveFloorAndAnisotropicPositiveVariancesAreNotJittered)
{
  auto floor =
      Window(0, second, l, s, bs, Matrix3::Identity(), {{0, second, Source(1, 0, second), 0.}});
  SharedSourceProcessFamily family({floor});
  auto values = References({floor});
  values.update(l, Pose2());
  values.update(s, Pose2(0, 0, .001));
  EXPECT_NEAR(family.endpoint_gyro_factor()->error(values), 50., 1e-10);
  auto off = Windows(false, true);
  off.resize(1);
  off[0].edge.endpoint_covariance = Vector3(1., 1e-18, 1e-20).asDiagonal();
  SharedSourceProcessFamily anisotropic(off);
  const auto query = anisotropic.Insert(second / 2, qa);
  const auto* factor = dynamic_cast<const NoiseModelFactor*>(query.factor.get());
  ASSERT_NE(factor, nullptr);
  EXPECT_TRUE((factor->noiseModel()->sigmas().array() > 0.).all());
  EXPECT_FALSE(anisotropic.endpoint_gyro_factor());
  off[0].edge.endpoint_covariance(0, 0) = -1.;
  EXPECT_THROW(SharedSourceProcessFamily invalid(off), std::invalid_argument);
  off[0].edge.endpoint_covariance.setZero();
  off[0].edge.endpoint_covariance(0, 1) = 1e-20;
  EXPECT_THROW(SharedSourceProcessFamily invalid(off), std::invalid_argument);
}

TEST(SharedSourceProcessFamily, RepeatedSourceDurationsSumBeforeVarianceWithConsistentHistory)
{
  const auto held = Source(1, 0, second);
  auto window = Window(0,
                       second,
                       l,
                       s,
                       bs,
                       Matrix3::Identity(),
                       {{0, 400000000, held, 4.}, {400000000, second, held, 4.}});
  SharedSourceProcessFamily family({window});
  auto values = References({window});
  values.update(l, Pose2());
  values.update(s, Pose2(0, 0, .2));
  // A=.4+.6=1 from metadata, consistently with the history's J_end.z=1.
  // C=1^2*4, rather than (.4^2+.6^2)*4 from independently split uses.
  EXPECT_NEAR(family.endpoint_gyro_factor()->error(values), .5 * .2 * .2 / 4., 1e-12);
}

TEST(SharedSourceProcessFamily, PerIdMetadataDurationMustMatchEndpointTrajectory)
{
  auto window = Window(0,
                       second,
                       l,
                       s,
                       bs,
                       Matrix3::Identity(),
                       {{0, 800000000, Source(1, 0, 800000000), 4.},
                        {800000000, second, Source(2, 800000000, second), 1.}});
  // Totals, zero raw integral, payloads and ID set are unchanged. Only these
  // per-ID weights contradict the .8/.2 history, changing C from 2.60 to .80.
  window.gyro->sources[0].duration_s = .2;
  window.gyro->sources[1].duration_s = .8;
  EXPECT_THROW(SharedSourceProcessFamily invalid({window}), std::invalid_argument);
}

TEST(SharedSourceProcessFamily, PreintegrationRequiresFullRotationAndValidPartialUseSupport)
{
  for (double scale : {.7, 0.})
  {
    auto window = Unequal();
    const auto original = window.edge.trajectory_query;
    window.edge.trajectory_query = std::make_shared<const FrozenProcessEdge::TrajectoryQuery>(
        [original, scale](int64_t time)
        {
          auto result = (*original)(time);
          for (auto& [id, jacobian] : result.gyro_jacobians)
            jacobian *= scale;
          result.bias_jacobian *= scale;
          return result;
        });
    EXPECT_THROW(SharedSourceProcessFamily invalid({window}), std::invalid_argument);
  }
  for (int invalid : {0, 1, 2})
  {
    auto window = Unequal();
    const auto original = window.edge.trajectory_query;
    const auto last_id = window.gyro->sources.back().source.id;
    const int64_t query_time = invalid == 2 ? 3 * second / 4 : second / 4;
    window.edge.trajectory_query = std::make_shared<const FrozenProcessEdge::TrajectoryQuery>(
        [original, last_id, invalid, query_time](int64_t time)
        {
          auto result = (*original)(time);
          if (time == query_time)
          {
            if (invalid == 1)
            {
              // The total still matches elapsed time; one source weight is negative.
              result.gyro_jacobians.begin()->second.z() = .26;
              result.gyro_jacobians[last_id] = Vector3(0, 0, -.01);
            }
            else if (invalid == 2)
            {
              // Total and bias sensitivity are right, but the first ID exceeds
              // its .5 s endpoint consumption.
              result.gyro_jacobians.begin()->second.z() = .7;
              result.gyro_jacobians.at(last_id).z() = .05;
            }
            else
              result.gyro_jacobians.begin()->second.z() *= .7;
          }
          return result;
        });
    SharedSourceProcessFamily family({window});
    EXPECT_THROW(family.Insert(query_time, qa), std::invalid_argument);
    EXPECT_TRUE(family.acquisitions().empty());
  }
}

TEST(SharedSourceProcessFamily, OffModeKeepsRecordedBiasMeanWithoutCreatingBiasOrGyroState)
{
  auto window = Window(0,
                       second,
                       l,
                       s,
                       bs,
                       Matrix3::Identity(),
                       {{0, second, Source(1, 0, second), .0004}},
                       true,
                       .03);
  SharedSourceProcessFamily family({window});
  auto values = References({window});
  values.update(s, origin.compose(Pose2(0, 0, -.03)));
  EXPECT_FALSE(family.endpoint_gyro_factor());
  EXPECT_FALSE(values.exists(bs));
  const auto query = family.Insert(second / 2, qa, &values);
  ASSERT_TRUE(query.initial_value);
  EXPECT_NEAR(query.initial_value->theta(), origin.theta() - .015, 1e-12);
}

TEST(SharedSourceProcessFamily, InvalidIdentityPayloadSupportVarianceAndWindowInputsReject)
{
  auto windows = Windows();
  windows[1].gyro->sources[0].source.raw_z = .1;
  windows[1].gyro->raw_integral_rad = .02;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[1].gyro->sources[0].source.support.end_ns += 1;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[1].edge.gyro_rate_variances.begin()->second *= 2.;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[0].gyro->sources[0].duration_s = -1.;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[0].gyro->raw_integral_rad = .1;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[1].edge.start_ns += 1;
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  windows = Windows();
  windows[1].gyro.reset();
  EXPECT_THROW(SharedSourceProcessFamily invalid(windows), std::invalid_argument);
  EXPECT_THROW(SharedSourceProcessFamily invalid({}), std::invalid_argument);
}
}  // namespace
