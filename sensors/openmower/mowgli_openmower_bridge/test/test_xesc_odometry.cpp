// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_xesc_odometry.cpp
 * @brief Windowed wheel odometry from signed tick counters.
 */

#include <chrono>
#include <cmath>

#include "mowgli_openmower_bridge/xesc_odometry.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::WheelSpeedFilter;
using mowgli_openmower_bridge::XescOdometry;

TEST(XescOdometry, FirstCallPrimesOnly)
{
  XescOdometry odom(1600.0, 0.325);
  EXPECT_FALSE(odom.Update(1000, 2000, 0.02).has_value());
  EXPECT_TRUE(odom.stationary());
}

TEST(XescOdometry, AggregatesToTheWindowThenReportsStraightVelocity)
{
  XescOdometry odom(1600.0, 0.325, 0.05);
  (void)odom.Update(0, 0, 0.02);
  // 32 ticks per 20 ms on both wheels = 0.02 m per tick step → 1.0 m/s
  EXPECT_FALSE(odom.Update(32, 32, 0.02).has_value());
  EXPECT_FALSE(odom.Update(64, 64, 0.02).has_value());
  const auto s = odom.Update(96, 96, 0.02);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->dt_s, 0.06, 1e-9);
  EXPECT_NEAR(s->vx_mps, 1.0, 1e-9);
  EXPECT_NEAR(s->vyaw_radps, 0.0, 1e-9);
  EXPECT_FALSE(odom.stationary());
  EXPECT_NEAR(odom.tyre_travelled(), 0.06, 1e-9);
}

TEST(XescOdometry, PivotYieldsYawRateAndWorstWheelOdometer)
{
  XescOdometry odom(1000.0, 0.5, 0.05);
  (void)odom.Update(0, 0, 0.05);
  const auto s = odom.Update(-25, 25, 0.05);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->vx_mps, 0.0, 1e-9);
  // (0.025 - (-0.025)) / 0.5 / 0.05 = 2 rad/s
  EXPECT_NEAR(s->vyaw_radps, 2.0, 1e-9);
  EXPECT_NEAR(odom.tyre_travelled(), 0.025, 1e-9);
}

TEST(XescOdometry, ResetRePrimes)
{
  XescOdometry odom(1000.0, 0.5, 0.05);
  (void)odom.Update(0, 0, 0.05);
  odom.Reset();
  EXPECT_FALSE(odom.Update(5000, 5000, 0.05).has_value());
  const auto s = odom.Update(5000, 5000, 0.05);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->vx_mps, 0.0, 1e-9);
  EXPECT_TRUE(odom.stationary());
}

TEST(WheelSpeedFilter, ReportsTicksOverTimeInItsWindow)
{
  WheelSpeedFilter f(0.08);
  EXPECT_NEAR(f.Update(20, 1000.0, 0.02), 1.0, 1e-9);  // 20 mm in 20 ms
  EXPECT_NEAR(f.Update(0, 1000.0, 0.02), 0.5, 1e-9);  // 20 mm in 40 ms
  EXPECT_NEAR(f.Update(0, 1000.0, 0.0), 0.5, 1e-9);  // zero dt keeps value
  // Older samples leave the 80 ms window.
  for (int i = 0; i < 4; ++i)
  {
    (void)f.Update(0, 1000.0, 0.02);
  }
  EXPECT_NEAR(f.value(), 0.0, 1e-9);
  f.Reset();
  EXPECT_DOUBLE_EQ(f.value(), 0.0);
}

TEST(WheelSpeedFilter, AStallLongerThanTheWindowStillMeasuresTheStall)
{
  WheelSpeedFilter f(0.08);
  (void)f.Update(6, 1000.0, 0.02);
  EXPECT_NEAR(f.Update(105, 1000.0, 0.35), 0.3, 1e-9);  // 105 mm in 350 ms
}

// The regression: when the ticks of a sample do not cover exactly the
// interval they are divided by (a controller sampled a little before the poll
// that read it, and the poll period jitters), a per-SAMPLE weight averages
// ratios, which over-weights the short intervals and biases the speed UP —
// the wheel loop then under-drives the robot (0.23 m/s for a 0.30 m/s command
// on a loaded CI runner). Weighting each sample by the time it covers keeps
// the long-run estimate at the true rate.
TEST(WheelSpeedFilter, MisalignedJitteryIntervalsDoNotBiasTheSpeed)
{
  WheelSpeedFilter f(0.08);
  // True speed 0.3 m/s at 1600 ticks/m = 480 ticks/s. Poll intervals
  // alternate 5 ms / 35 ms, and each poll reads the ticks of the PREVIOUS
  // interval.
  double sum = 0.0;
  int n = 0;
  double prev_dt = 0.02;
  for (int i = 0; i < 2000; ++i)
  {
    const double dt = (i % 2 == 0) ? 0.005 : 0.035;
    const auto ticks = static_cast<int64_t>(std::llround(480.0 * prev_dt));
    (void)f.Update(ticks, 1600.0, dt);
    prev_dt = dt;
    if (i >= 1000)
    {
      sum += f.value();
      ++n;
    }
  }
  EXPECT_NEAR(sum / n, 0.3, 0.005);
}

TEST(TickPlausibility, NormalMotionIsBelievedAtAnySpeedTheStackCommands)
{
  using mowgli_openmower_bridge::IsPlausibleTickDelta;
  // 0.5 m/s for one 20 ms control tick at 1600 ticks/m = 16 ticks.
  EXPECT_TRUE(IsPlausibleTickDelta(16, 1600.0, 0.02));
  EXPECT_TRUE(IsPlausibleTickDelta(-16, 1600.0, 0.02));
  // A late tick (100 ms) at 1.5 m/s is still motion.
  EXPECT_TRUE(IsPlausibleTickDelta(240, 1600.0, 0.10));
}

TEST(TickPlausibility, ACounterResetIsADiscontinuity)
{
  using mowgli_openmower_bridge::IsPlausibleTickDelta;
  // xESC 2040 brown-out: tacho_absolute restarts, the uint32 diff wraps.
  EXPECT_FALSE(IsPlausibleTickDelta(4294960000LL, 1600.0, 0.02));
  // xESC mini reboot: the signed tachometer drops back to 0 after 30 m.
  EXPECT_FALSE(IsPlausibleTickDelta(-48000, 1600.0, 0.02));
  EXPECT_FALSE(IsPlausibleTickDelta(200, 1600.0, 0.02));
}

namespace
{
using mowgli_openmower_bridge::WheelTickSampler;
using mowgli_openmower_bridge::WheelTickStep;
using Kind = mowgli_openmower_bridge::WheelTickStep::Kind;
using TP = WheelTickSampler::TimePoint;

TP At(int ms)
{
  return TP{} + std::chrono::milliseconds(1000 + ms);
}
}  // namespace

TEST(WheelTickSampler, FirstReadingOnlyPrimes)
{
  WheelTickSampler s;
  EXPECT_EQ(s.Feed({100, 200}, {At(0), At(0)}, 1600.0).kind, Kind::kPrimed);
}

TEST(WheelTickSampler, TicksAreDividedByTheTimeBetweenTheirStatusSamples)
{
  WheelTickSampler s;
  (void)s.Feed({0, 0}, {At(0), At(0)}, 1600.0);
  const WheelTickStep step = s.Feed({10, 9}, {At(20), At(18)}, 1600.0);
  ASSERT_EQ(step.kind, Kind::kSample);
  EXPECT_EQ(step.deltas[0], 10);
  EXPECT_EQ(step.deltas[1], 9);
  EXPECT_NEAR(step.dt_s[0], 0.020, 1e-12);
  EXPECT_NEAR(step.dt_s[1], 0.018, 1e-12);
}

// The regression: a bridge frozen for 350 ms reads 350 ms of motion. Its
// interval must be 350 ms (0.3 m/s), not one clamped control period.
TEST(WheelTickSampler, AStalledBridgeMeasuresTheWholeStall)
{
  WheelTickSampler s;
  (void)s.Feed({0, 0}, {At(0), At(0)}, 1600.0);
  // 0.3 m/s x 0.35 s x 1600 ticks/m = 168 ticks.
  const WheelTickStep step = s.Feed({168, 168}, {At(350), At(350)}, 1600.0);
  ASSERT_EQ(step.kind, Kind::kSample);
  EXPECT_NEAR(static_cast<double>(step.deltas[0]) / 1600.0 / step.dt_s[0], 0.3, 1e-9);
}

TEST(WheelTickSampler, WaitsUntilBothControllersReported)
{
  WheelTickSampler s;
  (void)s.Feed({0, 0}, {At(0), At(0)}, 1600.0);
  EXPECT_EQ(s.Feed({10, 0}, {At(20), At(0)}, 1600.0).kind, Kind::kWaiting);
  const WheelTickStep step = s.Feed({20, 19}, {At(40), At(38)}, 1600.0);
  ASSERT_EQ(step.kind, Kind::kSample);
  EXPECT_EQ(step.deltas[0], 20);  // the left wheel's two reports, together
  EXPECT_NEAR(step.dt_s[0], 0.040, 1e-12);
  EXPECT_NEAR(step.dt_s[1], 0.038, 1e-12);
}

TEST(WheelTickSampler, ACounterResetRePrimesInsteadOfPublishing)
{
  WheelTickSampler s;
  (void)s.Feed({5000, 5000}, {At(0), At(0)}, 1600.0);
  const WheelTickStep reset = s.Feed({0, 5010}, {At(20), At(20)}, 1600.0);
  ASSERT_EQ(reset.kind, Kind::kReset);
  EXPECT_EQ(reset.reset_wheel, 0);
  EXPECT_EQ(reset.reset_jump, -5000);
  EXPECT_EQ(reset.deltas[0], 0);
  // The reset reading is the new baseline.
  const WheelTickStep next = s.Feed({10, 5020}, {At(40), At(40)}, 1600.0);
  ASSERT_EQ(next.kind, Kind::kSample);
  EXPECT_EQ(next.deltas[0], 10);
  EXPECT_EQ(next.deltas[1], 10);
}

TEST(WheelTickSampler, ResetMakesTheNextReadingPrime)
{
  WheelTickSampler s;
  (void)s.Feed({0, 0}, {At(0), At(0)}, 1600.0);
  s.Reset();
  EXPECT_EQ(s.Feed({999999, 0}, {At(20), At(20)}, 1600.0).kind, Kind::kPrimed);
}
