// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <limits>
#include <vector>

#include "fusion_graph/motion_source_history.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace fusion_graph;
constexpr int64_t kSecond = 1000000000;

ConsumedMotionStep Step(int64_t start,
                        int64_t end,
                        uint64_t wheel_seq,
                        uint64_t gyro_seq,
                        double vx,
                        double vy,
                        double gyro)
{
  return {0,
          start,
          end,
          {{MotionSourceKind::Wheel, wheel_seq, static_cast<int64_t>(wheel_seq)},
           {-kSecond, 0},
           vx,
           vy},
          {{MotionSourceKind::Gyro, gyro_seq, static_cast<int64_t>(gyro_seq)}, {start, end}, gyro},
          0.03};
}

std::vector<ConsumedMotionStep> Maneuvers()
{
  return {Step(0, kSecond, 1, 1, 0.8, 0.2, 0.4),
          Step(kSecond, 2 * kSecond, 1, 2, 0.8, 0.2, -0.3),
          Step(2 * kSecond, 3 * kSecond, 2, 3, 0.0, 0.0, 0.8),
          Step(3 * kSecond, 4 * kSecond, 3, 4, -0.7, -0.15, -0.6)};
}

// Independent closed-form body-twist integration and explicit world-frame pose multiplication.
// This oracle does not call the history or GTSAM Expmap/compose/Jacobian implementation.
gtsam::Pose2 Integrate(const std::vector<ConsumedMotionStep>& steps, int64_t start, int64_t end)
{
  double x = 0, y = 0, yaw = 0;
  for (const auto& step : steps)
  {
    const int64_t a = std::max(start, step.start_ns), b = std::min(end, step.end_ns);
    if (b <= a)
      continue;
    const double dt = static_cast<double>(b - a) * 1e-9;
    const double w = step.gyro_rotation_enabled ? step.gyro.raw_z - step.bias_linearization : 0.0;
    const double angle = w * dt;
    // Stable Taylor limits avoid cancellation in this oracle when testing derivatives at w=0.
    const double a2 = angle * angle;
    const double s =
        std::abs(angle) < 1e-4 ? dt * (1 - a2 / 6 + a2 * a2 / 120) : std::sin(angle) / w;
    const double c = std::abs(angle) < 1e-4 ? dt * angle * (0.5 - a2 / 24 + a2 * a2 / 720)
                                            : (1 - std::cos(angle)) / w;
    const double gain = step.wheel_translation_enabled ? 1.0 : 0.0;
    const double dx = gain * (s * step.wheel.vx - c * step.wheel.vy);
    const double dy = gain * (c * step.wheel.vx + s * step.wheel.vy);
    x += std::cos(yaw) * dx - std::sin(yaw) * dy;
    y += std::sin(yaw) * dx + std::cos(yaw) * dy;
    yaw += angle;
  }
  return {x, y, yaw};
}

MotionTrajectory Query(const std::vector<ConsumedMotionStep>& steps, int64_t start, int64_t end)
{
  MotionSourceHistory history(steps.size());
  for (const auto& step : steps)
    EXPECT_EQ(history.Append(step), MotionHistoryStatus::Ok);
  return history.Query(0, start, end);
}

gtsam::Vector3 Numeric(const std::vector<ConsumedMotionStep>& steps,
                       int64_t start,
                       int64_t end,
                       const MotionSourceId* source,
                       int component)
{
  constexpr double eps = 1e-6;
  auto plus = steps, minus = steps;
  for (size_t i = 0; i < steps.size(); ++i)
  {
    if (!source)
    {
      plus[i].bias_linearization += eps;
      minus[i].bias_linearization -= eps;
    }
    else if (source->kind == MotionSourceKind::Wheel && steps[i].wheel.id == *source)
    {
      (component == 0 ? plus[i].wheel.vx : plus[i].wheel.vy) += eps;
      (component == 0 ? minus[i].wheel.vx : minus[i].wheel.vy) -= eps;
    }
    else if (source->kind == MotionSourceKind::Gyro && steps[i].gyro.id == *source)
    {
      plus[i].gyro.raw_z += eps;
      minus[i].gyro.raw_z -= eps;
    }
  }
  const auto base = Integrate(steps, start, end);
  return (gtsam::Pose2::Logmap(base.between(Integrate(plus, start, end))) -
          gtsam::Pose2::Logmap(base.between(Integrate(minus, start, end)))) /
         (2 * eps);
}

TEST(MotionSourceHistory, PartialSamplesAcrossStartStopTurnReverseAndLateralMotion)
{
  const auto steps = Maneuvers();
  for (const auto& [start, end] :
       std::vector<std::pair<int64_t, int64_t>>{{0, 4 * kSecond},
                                                {kSecond / 5, 18 * kSecond / 5},
                                                {2 * kSecond, 3 * kSecond},
                                                {13 * kSecond / 4, 15 * kSecond / 4}})
  {
    const auto result = Query(steps, start, end);
    ASSERT_EQ(result.status, MotionHistoryStatus::Ok);
    EXPECT_LT(gtsam::Pose2::Logmap(result.mean.between(Integrate(steps, start, end))).norm(),
              1e-12);
  }
  const auto held = Query({steps.front()}, kSecond / 4, 3 * kSecond / 4);
  // Explicitly held wheel support ended BEFORE consumption; no synthetic support extension.
  EXPECT_EQ(steps.front().wheel.support.end_ns, 0);
  EXPECT_GT(std::abs(held.mean.y()), 0.05);
}

TEST(MotionSourceHistory, AnalyticRightTangentJacobiansMatchIndependentPerturbations)
{
  auto steps = Maneuvers();
  for (double gyro : {0.03, 0.0300000001, 0.4})
  {
    steps[0].gyro.raw_z = gyro;
    const int64_t start = kSecond / 5, end = 18 * kSecond / 5;
    const auto result = Query(steps, start, end);
    ASSERT_EQ(result.status, MotionHistoryStatus::Ok);
    ASSERT_EQ(result.wheel_jacobians.size(), 3u);
    for (const auto& [id, jacobian] : result.wheel_jacobians)
      for (int column = 0; column < 2; ++column)
        EXPECT_LT((jacobian.col(column) - Numeric(steps, start, end, &id, column)).norm(), 1e-7);
    for (const auto& [id, jacobian] : result.gyro_jacobians)
      EXPECT_LT((jacobian - Numeric(steps, start, end, &id, 0)).norm(), 1e-7);
    EXPECT_LT((result.bias_jacobian - Numeric(steps, start, end, nullptr, 0)).norm(), 1e-7);
  }
}

TEST(MotionSourceHistory, SharedSourceNoiseRetainsCrossCovarianceAcrossPartialQueries)
{
  auto first = Step(0, kSecond / 2, 1, 1, 0.9, 0.1, 0.7);
  auto second = first;
  second.start_ns = first.end_ns;
  second.end_ns = kSecond;
  const std::vector<ConsumedMotionStep> steps{first, second};
  const auto whole = Query(steps, 0, kSecond);
  const auto left = Query(steps, 0, kSecond / 3);
  const auto right = Query(steps, kSecond / 3, kSecond);
  ASSERT_EQ(whole.gyro_jacobians.size(), 1u);
  ASSERT_EQ(whole.wheel_jacobians.size(), 1u);
  gtsam::Matrix3 hleft, hright;
  EXPECT_LT(
      gtsam::Pose2::Logmap(whole.mean.between(left.mean.compose(right.mean, hleft, hright))).norm(),
      1e-12);
  const auto id = first.gyro.id;
  const gtsam::Vector3 a = hleft * left.gyro_jacobians.at(id);
  const gtsam::Vector3 b = hright * right.gyro_jacobians.at(id);
  const gtsam::Matrix3 cross = a * b.transpose();
  const gtsam::Vector3 total = whole.gyro_jacobians.at(id);
  EXPECT_LT((total - Numeric(steps, 0, kSecond, &id, 0)).norm(), 1e-7);
  EXPECT_LT((total - a - b).norm(), 1e-12);
  EXPECT_GT(cross.norm(), 0.1);
  EXPECT_LT((total * total.transpose() -
             (a * a.transpose() + b * b.transpose() + cross + cross.transpose()))
                .norm(),
            1e-12);
  EXPECT_GT((total * total.transpose() - (a * a.transpose() + b * b.transpose())).norm(), 0.1);
  EXPECT_LT((whole.wheel_jacobians.at(first.wheel.id) -
             hleft * left.wheel_jacobians.at(first.wheel.id) -
             hright * right.wheel_jacobians.at(first.wheel.id))
                .norm(),
            1e-12);
}

TEST(MotionSourceHistory, DeterministicTranslationGatePreservesHeldSourceIdentityAndYaw)
{
  auto first = Step(0, kSecond, 1, 1, 0.8, -0.2, 0.7);
  first.gyro.support = {0, 3 * kSecond};
  auto second = first, third = first;
  second.start_ns = kSecond;
  second.end_ns = third.start_ns = 2 * kSecond;
  third.end_ns = 3 * kSecond;
  first.wheel_translation_enabled = third.wheel_translation_enabled = false;
  const std::vector<ConsumedMotionStep> steps{first, second, third};
  const int64_t start = kSecond / 5, end = 14 * kSecond / 5;
  const auto result = Query(steps, start, end);
  ASSERT_EQ(result.status, MotionHistoryStatus::Ok);
  ASSERT_EQ(result.wheel_jacobians.size(), 1u);
  ASSERT_EQ(result.gyro_jacobians.size(), 1u);
  EXPECT_LT(gtsam::Pose2::Logmap(result.mean.between(Integrate(steps, start, end))).norm(), 1e-12);
  const auto wheel_id = first.wheel.id, gyro_id = first.gyro.id;
  for (int column = 0; column < 2; ++column)
    EXPECT_LT((result.wheel_jacobians.at(wheel_id).col(column) -
               Numeric(steps, start, end, &wheel_id, column))
                  .norm(),
              1e-7);
  EXPECT_LT((result.gyro_jacobians.at(gyro_id) - Numeric(steps, start, end, &gyro_id, 0)).norm(),
            1e-7);
  EXPECT_LT((result.bias_jacobian - Numeric(steps, start, end, nullptr, 0)).norm(), 1e-7);

  const auto suppressed = Query(steps, 0, kSecond);
  EXPECT_DOUBLE_EQ(suppressed.mean.x(), 0.0);
  EXPECT_DOUBLE_EQ(suppressed.mean.y(), 0.0);
  EXPECT_NEAR(suppressed.mean.theta(), 0.67, 1e-12);
  EXPECT_DOUBLE_EQ(suppressed.wheel_jacobians.at(wheel_id).norm(), 0.0);
  EXPECT_NEAR(suppressed.gyro_jacobians.at(gyro_id).z(), 1.0, 1e-12);
  EXPECT_NEAR(suppressed.bias_jacobian.z(), -1.0, 1e-12);
  auto all_enabled = steps;
  for (auto& step : all_enabled)
    step.wheel_translation_enabled = true;
  const auto enabled = Query(all_enabled, start, end);
  EXPECT_NEAR(result.mean.theta(), enabled.mean.theta(), 1e-12);
  EXPECT_NEAR(result.gyro_jacobians.at(gyro_id).z(), enabled.gyro_jacobians.at(gyro_id).z(), 1e-12);
  EXPECT_NEAR(result.bias_jacobian.z(), enabled.bias_jacobian.z(), 1e-12);
  EXPECT_GT((result.wheel_jacobians.at(wheel_id) - enabled.wheel_jacobians.at(wheel_id)).norm(),
            0.5);
}

TEST(MotionSourceHistory, DeterministicRotationGatePreservesHeldGyroIdentityAndBiasPolicy)
{
  auto first = Step(0, kSecond, 1, 1, 0.8, 0.2, 0.7);
  first.gyro.support = {0, 3 * kSecond};
  auto second = first, third = first;
  second.start_ns = kSecond;
  second.end_ns = third.start_ns = 2 * kSecond;
  third.end_ns = 3 * kSecond;
  second.gyro_rotation_enabled = false;
  second.bias_linearization = 0.08;
  third.bias_linearization = 0.12;
  const std::vector<ConsumedMotionStep> steps{first, second, third};
  const int64_t start = kSecond / 5, end = 14 * kSecond / 5;
  const auto result = Query(steps, start, end);
  ASSERT_EQ(result.status, MotionHistoryStatus::Ok);
  ASSERT_EQ(result.wheel_jacobians.size(), 1u);
  ASSERT_EQ(result.gyro_jacobians.size(), 1u);
  EXPECT_LT(gtsam::Pose2::Logmap(result.mean.between(Integrate(steps, start, end))).norm(), 1e-12);
  const auto wheel_id = first.wheel.id, gyro_id = first.gyro.id;
  for (int column = 0; column < 2; ++column)
    EXPECT_LT((result.wheel_jacobians.at(wheel_id).col(column) -
               Numeric(steps, start, end, &wheel_id, column))
                  .norm(),
              1e-7);
  EXPECT_LT((result.gyro_jacobians.at(gyro_id) - Numeric(steps, start, end, &gyro_id, 0)).norm(),
            1e-7);
  EXPECT_LT((result.bias_jacobian - Numeric(steps, start, end, nullptr, 0)).norm(), 1e-7);
  EXPECT_NEAR(result.mean.theta(), 0.8 * (0.67 + 0.58), 1e-12);

  const auto suppressed = Query(steps, kSecond, 2 * kSecond);
  EXPECT_NEAR(suppressed.mean.x(), 0.8, 1e-12);
  EXPECT_NEAR(suppressed.mean.y(), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(suppressed.mean.theta(), 0.0);
  EXPECT_DOUBLE_EQ(suppressed.gyro_jacobians.at(gyro_id).norm(), 0.0);
  EXPECT_DOUBLE_EQ(suppressed.bias_jacobian.norm(), 0.0);
  EXPECT_NEAR(suppressed.wheel_jacobians.at(wheel_id)(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(suppressed.wheel_jacobians.at(wheel_id)(1, 1), 1.0, 1e-12);
  // A changed gate on the same consumed interval is not a duplicate and cannot rewrite it.
  MotionSourceHistory history(steps.size());
  for (const auto& step : steps)
    ASSERT_EQ(history.Append(step), MotionHistoryStatus::Ok);
  second.gyro_rotation_enabled = true;
  EXPECT_EQ(history.Append(second), MotionHistoryStatus::Overlap);
  EXPECT_TRUE(history.Query(0, kSecond, 2 * kSecond).mean.equals(suppressed.mean, 1e-12));
}

TEST(MotionSourceHistory, DuplicatePayloadCannotReplaceFirstAcceptedData)
{
  MotionSourceHistory history(3);
  auto first = Step(0, kSecond, 1, 1, 0.8, 0.2, 0.4);
  ASSERT_EQ(history.Append(first), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.Append(first), MotionHistoryStatus::Duplicate);
  auto altered = first;
  altered.wheel.vx += 1.0;
  EXPECT_EQ(history.Append(altered), MotionHistoryStatus::SourceConflict);
  altered = first;
  altered.gyro.support.end_ns++;
  EXPECT_EQ(history.Append(altered), MotionHistoryStatus::SourceConflict);
  altered = first;
  altered.gyro.raw_z += 1.0;
  EXPECT_EQ(history.Append(altered), MotionHistoryStatus::SourceConflict);
  EXPECT_EQ(history.size(), 1u);
  EXPECT_LT(gtsam::Pose2::Logmap(
                history.Query(0, 0, kSecond).mean.between(Integrate({first}, 0, kSecond)))
                .norm(),
            1e-12);
}

TEST(MotionSourceHistory, RequiresContiguousOrderedFiniteConsumedIntervals)
{
  MotionSourceHistory history(5);
  auto step = Step(0, kSecond, 1, 1, 0.8, 0.2, 0.4);
  ASSERT_EQ(history.Append(step), MotionHistoryStatus::Ok);
  auto next = Step(kSecond + 1, 2 * kSecond, 2, 2, 0.1, 0.2, 0.3);
  EXPECT_EQ(history.Append(next), MotionHistoryStatus::Gap);
  next.start_ns = kSecond - 1;
  EXPECT_EQ(history.Append(next), MotionHistoryStatus::Overlap);
  next.start_ns = kSecond;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int field = 0; field < 4; ++field)
  {
    auto bad = next;
    double* rates[] = {&bad.wheel.vx, &bad.wheel.vy, &bad.gyro.raw_z, &bad.bias_linearization};
    *rates[field] = nan;
    EXPECT_EQ(history.Append(bad), MotionHistoryStatus::InvalidInput);
  }
  auto bad = next;
  bad.end_ns = bad.start_ns;
  EXPECT_EQ(history.Append(bad), MotionHistoryStatus::InvalidInput);
  bad = next;
  bad.wheel.id.kind = MotionSourceKind::Gyro;
  EXPECT_EQ(history.Append(bad), MotionHistoryStatus::InvalidInput);
  ASSERT_EQ(history.Append(next), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.Query(0, -1, kSecond).status, MotionHistoryStatus::Uncovered);
  EXPECT_EQ(history.Query(0, 0, 2 * kSecond + 1).status, MotionHistoryStatus::Uncovered);
  EXPECT_EQ(history.Query(0, 1, 0).status, MotionHistoryStatus::InvalidInput);
  const auto zero = history.Query(0, kSecond, kSecond);
  EXPECT_EQ(zero.status, MotionHistoryStatus::Ok);
  EXPECT_TRUE(zero.wheel_jacobians.empty());
  EXPECT_TRUE(zero.mean.equals(gtsam::Pose2(), 1e-12));
}

TEST(MotionSourceHistory, ExplicitCapacityPruningEpochAndRetiredIdentityRules)
{
  MotionSourceHistory history(1);
  auto first = Step(0, kSecond, 1, 1, 0.8, 0.2, 0.4);
  auto next = Step(kSecond, 2 * kSecond, 2, 2, 0.1, 0.0, -0.2);
  EXPECT_EQ(history.Query(0, 0, 0).status, MotionHistoryStatus::Uncovered);
  ASSERT_EQ(history.Append(first), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.Append(next), MotionHistoryStatus::CapacityExceeded);
  EXPECT_EQ(history.Query(0, kSecond, 2 * kSecond).status, MotionHistoryStatus::Uncovered);
  EXPECT_EQ(history.PruneBefore(1, kSecond), MotionHistoryStatus::WrongEpoch);
  ASSERT_EQ(history.PruneBefore(0, kSecond / 2), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.size(), 1u);
  ASSERT_EQ(history.PruneBefore(0, kSecond), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.size(), 0u);
  auto held = first;
  held.start_ns = kSecond;
  held.end_ns = 2 * kSecond;
  auto changed_held = held;
  changed_held.wheel.vx += 1.0;
  EXPECT_EQ(history.Append(changed_held), MotionHistoryStatus::SourceConflict);
  ASSERT_EQ(history.Append(held), MotionHistoryStatus::Ok);  // Latest cached identities survive.
  ASSERT_EQ(history.PruneBefore(0, 2 * kSecond), MotionHistoryStatus::Ok);
  next.start_ns = 2 * kSecond;
  next.end_ns = 3 * kSecond;
  ASSERT_EQ(history.Append(next), MotionHistoryStatus::Ok);
  ASSERT_EQ(history.PruneBefore(0, 3 * kSecond), MotionHistoryStatus::Ok);
  held.start_ns = 3 * kSecond;
  held.end_ns = 4 * kSecond;
  EXPECT_EQ(history.Append(held), MotionHistoryStatus::RetiredSource);
  history.Reset(1);
  EXPECT_EQ(history.Append(first), MotionHistoryStatus::WrongEpoch);
  first.epoch = 1;
  EXPECT_EQ(history.Append(first), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.Query(0, 0, kSecond).status, MotionHistoryStatus::WrongEpoch);
  EXPECT_EQ(history.Query(1, 0, kSecond).status, MotionHistoryStatus::Ok);
}

TEST(MotionSourceHistory, IntegerNanosecondClippingPreservesSmallFractionsAtLargeEpoch)
{
  constexpr int64_t origin = 1700000000000000000;
  const auto step = Step(origin, origin + 10, 1, 1, 1e6, -2e6, 0.03);
  const auto result = Query({step}, origin + 2, origin + 7);
  ASSERT_EQ(result.status, MotionHistoryStatus::Ok);
  EXPECT_NEAR(result.mean.x(), 0.005, 1e-15);
  EXPECT_NEAR(result.mean.y(), -0.01, 1e-15);
  EXPECT_NEAR(result.wheel_jacobians.at(step.wheel.id)(0, 0), 5e-9, 1e-20);

  MotionSourceHistory zero_capacity(0);
  EXPECT_EQ(zero_capacity.Append(step), MotionHistoryStatus::CapacityExceeded);
  EXPECT_EQ(zero_capacity.Query(0, origin, origin + 10).status, MotionHistoryStatus::Uncovered);
}

TEST(MotionSourceHistory, LargeHistoryIndexesSourcesDuplicatesAndShortOverlaps)
{
  constexpr size_t count = 20000;
  constexpr int64_t interval = 1000000;
  MotionSourceHistory history(count);
  std::vector<ConsumedMotionStep> steps;
  steps.reserve(count);
  for (size_t i = 0; i < count; ++i)
  {
    steps.push_back(Step(i * interval, (i + 1) * interval, i / 4 + 1, i + 1, 0.8, 0.2, 0.03));
    ASSERT_EQ(history.Append(steps.back()), MotionHistoryStatus::Ok);
  }
  EXPECT_EQ(history.size(), count);
  for (size_t i : {size_t{0}, count / 2, count - 1})
  {
    EXPECT_EQ(history.Append(steps[i]), MotionHistoryStatus::Duplicate);
    auto changed = steps[i];
    changed.wheel.vx += 1.0;
    EXPECT_EQ(history.Append(changed), MotionHistoryStatus::SourceConflict);
  }
  const int64_t end = count * interval;
  const auto tail = history.Query(0, end - 5 * interval, end);
  ASSERT_EQ(tail.status, MotionHistoryStatus::Ok);
  EXPECT_EQ(tail.gyro_jacobians.size(), 5u);
  EXPECT_EQ(tail.wheel_jacobians.size(), 2u);
  EXPECT_LT(
      gtsam::Pose2::Logmap(tail.mean.between(Integrate(steps, end - 5 * interval, end))).norm(),
      1e-12);
  const auto all = history.Query(0, 0, end);
  ASSERT_EQ(all.status, MotionHistoryStatus::Ok);
  EXPECT_EQ(all.gyro_jacobians.size(), count);
  EXPECT_EQ(all.wheel_jacobians.size(), count / 4);
  EXPECT_NEAR(all.mean.x(), 16.0, 1e-8);
  EXPECT_NEAR(all.mean.y(), 4.0, 1e-8);

  ASSERT_EQ(history.PruneBefore(0, end - 7 * interval), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.size(), 7u);
  // The second-latest wheel still has three retained references; pruning its first must not
  // retire its identity. Reuse it after the latest wheel, then remove every reference.
  auto reuse = steps[count - 8];
  reuse.start_ns = end;
  reuse.end_ns = end + interval;
  reuse.gyro = Step(end, end + interval, 1, count + 1, 0.0, 0.0, 0.03).gyro;
  ASSERT_EQ(history.Append(reuse), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.size(), 8u);
  ASSERT_EQ(history.PruneBefore(0, end + interval), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.size(), 0u);
  reuse.start_ns += interval;
  reuse.end_ns += interval;
  EXPECT_EQ(history.Append(reuse), MotionHistoryStatus::RetiredSource);
  reuse.wheel = steps.back().wheel;  // Latest held payload survives even the empty ledger.
  ASSERT_EQ(history.Append(reuse), MotionHistoryStatus::Ok);
  EXPECT_EQ(history.Query(0, 0, end).status, MotionHistoryStatus::Uncovered);
  EXPECT_EQ(history.Query(0, reuse.start_ns, reuse.end_ns).status, MotionHistoryStatus::Ok);
}
}  // namespace
