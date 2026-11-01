// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <array>
#include <cmath>

#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/motion_source_history.hpp"
#include <gtest/gtest.h>

namespace fusion_graph
{
class GraphManagerSourceTestAccess
{
public:
  static std::array<double, 10> Accumulator(const GraphManager& graph)
  {
    std::lock_guard<std::mutex> lock(graph.mu_);
    const auto& a = graph.accum_;
    return {a.dx,
            a.dy,
            a.wheel_travel_m,
            a.dtheta_wheel,
            a.dtheta_gyro,
            a.dt_total,
            a.max_abs_gyro_rad_per_s,
            a.gyro_preint_dtheta,
            a.gyro_preint_dt,
            a.gyro_preint_variance};
  }
};
namespace
{
constexpr uint64_t epoch = 19;
constexpr int64_t start = 20000000000, s = 20100000000;
constexpr int64_t u = 20200000000, v = 20300000000, w = 20400000000;
using Access = GraphManagerSourceTestAccess;

struct Result
{
  bool created;
  TickOutput snapshot;
  GraphStats stats;
};

GraphParams Params(bool preint)
{
  GraphParams p;
  p.use_imu_preint = preint;
  p.cov_update_every_n = 1;
  return p;
}

void Seed(GraphManager& graph, bool raw, int64_t source, int64_t schedule)
{
  if (raw)
  {
    ASSERT_TRUE(graph.QueueGnssAtEpoch(0., 0., .014, false, source, 71, epoch));
    ASSERT_TRUE(graph.QueueYawAtEpoch(0., .1, false, source, 82, epoch));
    ASSERT_EQ(graph.InitializeSourceObservations(schedule, source, epoch, 71, 82),
              SourceBootstrapResult::Applied);
  }
  else
    ASSERT_TRUE(graph.InitializeSource(gtsam::Pose2(), schedule, source, epoch));
}

ConsumedMotionStep Step(int64_t begin,
                        int64_t end,
                        uint64_t gyro_id,
                        const MotionWheelSource& wheel,
                        double gyro,
                        double bias)
{
  return {epoch,
          begin,
          end,
          wheel,
          {{MotionSourceKind::Gyro, gyro_id, end}, {begin, end}, gyro},
          bias,
          true,
          true};
}

Result FirstWindow(double speed, bool aligned, bool raw, bool preint)
{
  GraphManager graph(Params(preint));
  graph.SetConsumedMotionBaseline(epoch, start);
  if (aligned)
    graph.AddWheelTwist(speed, 0., 0., .2);
  const auto first = Step(start,
                          u,
                          1,
                          {{MotionSourceKind::Wheel, aligned ? 2u : 1u, aligned ? u : start},
                           {start, aligned ? u : start},
                           speed,
                           0.,
                           0.},
                          0.,
                          graph.AddGyroDelta(0., .2));
  EXPECT_TRUE(graph.AddConsumedMotionStep(first));
  Seed(graph, raw, s, u);
  graph.AddWheelTwist(speed, 0., 0., aligned ? .1 : .3);
  const auto second = Step(
      u,
      v,
      2,
      {{MotionSourceKind::Wheel, aligned ? 3u : 2u, v}, {aligned ? u : start, v}, speed, 0., 0.},
      0.,
      graph.AddGyroDelta(0., .1));
  EXPECT_TRUE(graph.AddConsumedMotionStep(second));
  const auto output = graph.TickSource(v, v, epoch);
  EXPECT_EQ(graph.SourceGnssFactorCount(), raw ? 1u : 0u);
  EXPECT_EQ(graph.SourceYawFactorCount(), raw ? 1u : 0u);
  return {output.has_value(), *graph.LatestSnapshot(), graph.Stats()};
}

TEST(SourcePolicyAccumulator, SameAndExplicitSourceBootstrapUsePhysicalStationaryThrottle)
{
  for (const bool preint : {false, true})
    for (const bool raw : {false, true})
    {
      SCOPED_TRACE(::testing::Message() << preint << ' ' << raw);
      const auto a = FirstWindow(.075, false, raw, preint);
      const auto b = FirstWindow(.075, true, raw, preint);
      EXPECT_FALSE(a.created);
      EXPECT_FALSE(b.created);
      EXPECT_EQ(a.snapshot.source_stamp_ns, s);
      EXPECT_EQ(b.snapshot.source_stamp_ns, s);
      EXPECT_EQ(a.snapshot.node_index, 0u);
      EXPECT_EQ(b.snapshot.node_index, 0u);
      EXPECT_LT((a.snapshot.covariance - b.snapshot.covariance).norm(), 1e-12);
    }
}

TEST(SourcePolicyAccumulator, FirstSourceWindowNoiseAndMeanIgnoreReceiptAlignment)
{
  for (const bool preint : {false, true})
    for (const bool raw : {false, true})
    {
      SCOPED_TRACE(::testing::Message() << preint << ' ' << raw);
      const auto a = FirstWindow(.2, false, raw, preint);
      const auto b = FirstWindow(.2, true, raw, preint);
      ASSERT_TRUE(a.created);
      ASSERT_TRUE(b.created);
      EXPECT_EQ(a.snapshot.source_stamp_ns, v);
      EXPECT_EQ(b.snapshot.source_stamp_ns, v);
      EXPECT_NEAR(a.snapshot.pose.x(), .04, 1e-12);
      EXPECT_NEAR(b.snapshot.pose.x(), .04, 1e-12);
      EXPECT_NEAR(a.stats.wheel_sigma_x_eff * a.stats.wheel_sigma_x_eff, .00012, 1e-14);
      EXPECT_NEAR(b.stats.wheel_sigma_x_eff * b.stats.wheel_sigma_x_eff, .00012, 1e-14);
      EXPECT_LT((a.snapshot.covariance - b.snapshot.covariance).cwiseAbs().maxCoeff(), 1e-10);
    }
}

Result LaterCut(bool aligned, bool raw, bool preint, bool wheel_slip)
{
  auto p = Params(preint);
  if (wheel_slip)
  {
    p.slip_window_s = 0.;
    p.stationary_node_period_s = .1;
  }
  GraphManager graph(p);
  graph.SetConsumedMotionBaseline(epoch, start);
  Seed(graph, raw, start, start);
  const double gyro = wheel_slip ? 0. : .2;
  const auto append = [&](int64_t begin,
                          int64_t end,
                          uint64_t id,
                          const MotionWheelSource& wheel,
                          bool receipt,
                          double receipt_dt)
  {
    if (receipt)
      graph.AddWheelTwist(.2, 0., .2, receipt_dt);
    const auto step =
        Step(begin, end, id, wheel, gyro, graph.AddGyroDelta(gyro, (end - begin) * 1e-9));
    EXPECT_TRUE(graph.AddConsumedMotionStep(step));
    return graph.TickSource(end, end, epoch);
  };
  const MotionWheelSource initial{{MotionSourceKind::Wheel, 1, u}, {start, u}, .2, 0., .2};
  // Both traces first publish an aligned ordinary edge. Subsequent cadence V
  // cuts inside A's next wheel receipt interval U..W.
  EXPECT_TRUE(append(start, u, 1, initial, true, .2));
  const uint64_t first_slips = graph.Stats().slip_veto;
  const MotionWheelSource middle =
      aligned ? MotionWheelSource{{MotionSourceKind::Wheel, 2, v}, {u, v}, .2, 0., .2} : initial;
  EXPECT_TRUE(append(u, v, 2, middle, aligned, .1));
  const MotionWheelSource last{
      {MotionSourceKind::Wheel, aligned ? 3u : 2u, w}, {aligned ? v : u, w}, .2, 0., .2};
  const auto output = append(v, w, 3, last, true, aligned ? .1 : .2);
  EXPECT_EQ(graph.LatestSnapshot()->node_index, 3u);
  EXPECT_EQ(graph.Stats().slip_veto, first_slips);
  return {output.has_value(), *graph.LatestSnapshot(), graph.Stats()};
}

TEST(SourcePolicyAccumulator, LaterOrdinaryCutsPreserveMotionHandPushSlipAndNoisePolicies)
{
  for (const bool preint : {false, true})
    for (const bool raw : {false, true})
      for (const bool slip : {false, true})
      {
        SCOPED_TRACE(::testing::Message() << preint << ' ' << raw << ' ' << slip);
        const auto a = LaterCut(false, raw, preint, slip);
        const auto b = LaterCut(true, raw, preint, slip);
        ASSERT_TRUE(a.created);
        ASSERT_TRUE(b.created);
        EXPECT_EQ(a.snapshot.source_stamp_ns, w);
        EXPECT_EQ(b.snapshot.source_stamp_ns, w);
        EXPECT_LT(a.snapshot.pose.localCoordinates(b.snapshot.pose).norm(), 1e-12);
        EXPECT_LT((a.snapshot.covariance - b.snapshot.covariance).cwiseAbs().maxCoeff(), 1e-10);
        EXPECT_EQ(a.stats.stationary_hand_push, 0u);
        EXPECT_EQ(b.stats.stationary_hand_push, 0u);
        EXPECT_EQ(a.stats.slip_veto, b.stats.slip_veto);
        EXPECT_DOUBLE_EQ(a.stats.residual_ema_rad, b.stats.residual_ema_rad);
        EXPECT_DOUBLE_EQ(a.stats.wheel_sigma_x_eff, b.stats.wheel_sigma_x_eff);
      }
}

TEST(SourcePolicyAccumulator, CallbackEmaKeepsItsExistingUpdateAndRecordedBiasConvention)
{
  for (const bool preint : {false, true})
  {
    auto p = Params(preint);
    p.stationary_node_period_s = .1;
    p.gyro_bias_ema_tau_s = 1.;
    GraphManager graph(p);
    graph.SetConsumedMotionBaseline(epoch, start);
    Seed(graph, false, start, start);
    const MotionWheelSource wheel{{MotionSourceKind::Wheel, 1, start}, {start, start}, 0., 0., 0.};
    ASSERT_TRUE(graph.AddConsumedMotionStep(Step(start, s, 1, wheel, 0., 0.)));
    ASSERT_TRUE(graph.TickSource(s, s, epoch));
    double expected_ema = 0.;
    int64_t begin = s;
    uint64_t id = 2;
    for (const double raw : {.02, .03, .01})
    {
      graph.AddWheelTwist(0., 0., 0., .3);  // Receipt duration is irrelevant to source integration.
      const double correction = graph.AddGyroDelta(raw, .1);
      expected_ema += (.1 / 1.1) * (raw - expected_ema);
      EXPECT_NEAR(graph.Stats().gyro_bias_z, expected_ema, 1e-15);
      EXPECT_EQ(graph.Stats().gyro_bias_updates, id - 1);
      if (!preint)
        EXPECT_NEAR(correction, expected_ema, 1e-15);
      const auto before = Access::Accumulator(graph);
      const int64_t end = begin + 100000000;
      const auto step = Step(begin, end, id++, wheel, raw, correction);
      auto invalid = step;
      ++invalid.epoch;
      EXPECT_FALSE(graph.AddConsumedMotionStep(invalid));
      EXPECT_EQ(before, Access::Accumulator(graph));
      ASSERT_TRUE(graph.AddConsumedMotionStep(step));
      const auto accepted = Access::Accumulator(graph);
      EXPECT_NEAR(accepted[4], (raw - correction) * .1, 1e-15);
      EXPECT_NEAR(accepted[5], .1, 1e-15);
      EXPECT_FALSE(graph.AddConsumedMotionStep(step));
      EXPECT_EQ(accepted, Access::Accumulator(graph));
      ASSERT_TRUE(graph.TickSource(end, end, epoch));
      begin = end;
    }
  }
}

TEST(SourcePolicyAccumulator, LegacyWithoutSourceBaselineKeepsReceiptAccumulation)
{
  for (const bool preint : {false, true})
  {
    GraphManager graph(Params(preint));
    graph.Initialize(gtsam::Pose2(), 0.);
    graph.AddWheelTwist(.2, 0., 0., .3);
    graph.AddGyroDelta(0., .1);
    const auto input = Access::Accumulator(graph);
    EXPECT_NEAR(input[0], .06, 1e-15);
    EXPECT_NEAR(input[5], .3, 1e-15);
    EXPECT_NEAR(input[8], preint ? .1 : 0., 1e-15);
    ASSERT_TRUE(graph.Tick(.3));
  }
}
}  // namespace
}  // namespace fusion_graph
