// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_battery_filter.cpp
 * @brief Regression tests for the premature LOW_BATTERY_DOCKING trip.
 *
 * A mowing run ended in LOW_BATTERY_DOCKING while the GUI still showed ~35 %.
 * The BT fired correctly — battery_percent really had crossed 20 % — because
 * battery_percent was interpolated straight off the raw rail voltage, and the
 * motors had pulled the pack down ~1 V for well under a second (PWM startup /
 * stiction). The instant the BT reacted, the load collapsed and the rail
 * recovered to ~25.6 V.
 *
 * These tests pin the two properties that fix requires: a sub-second sag must
 * not move the percent far enough to trip, and a genuine discharge must still
 * be tracked. They also pin the two failure modes the filter itself could
 * introduce — a bogus reading turning into a 0 % gauge, and the smoothing
 * silently changing strength with the publish rate.
 */

#include "mowgli_behavior/battery_filter.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::batteryPercentFromVoltage;
using mowgli_behavior::BatteryVoltageFilter;
using mowgli_behavior::kBatteryFilterTauS;

namespace
{

// The YardForce 500 SLA endpoints behavior_tree_node declares by default.
constexpr float kEmptyV = 24.0f;
constexpr float kFullV = 28.0f;

/// The robot's real cadence: /hardware_bridge/power is published once per
/// firmware status packet, and STATUS_NBT_TIME_MS is 250.
constexpr double kRobotPeriodS = 0.25;
/// What fake_hardware_bridge_node publishes at in sim.
constexpr double kSimPeriodS = 0.10;

/// Feed a constant voltage for @p duration_s at @p period_s, starting at
/// @p t0, and return the clock after the last sample.
double feed(
    BatteryVoltageFilter& filter, float voltage, double t0, double duration_s, double period_s)
{
  double t = t0;
  const double end = t0 + duration_s;
  while (t < end)
  {
    t += period_s;
    filter.update(voltage, t);
  }
  return t;
}

float percentOf(const BatteryVoltageFilter& filter)
{
  return batteryPercentFromVoltage(filter.value().value(), kEmptyV, kFullV);
}

}  // namespace

// ── The incident ───────────────────────────────────────────────────────────

TEST(BatteryFilter, SubSecondMotorSagDoesNotCrossTheDockingThreshold)
{
  // Settled at 25.6 V — the voltage the pack recovered to, ~40 % of the scale
  // and comfortably above the 20 % battery_low_percent trip.
  BatteryVoltageFilter filter;
  double t = feed(filter, 25.6f, 0.0, 20.0, kRobotPeriodS);
  ASSERT_GT(percentOf(filter), 20.0f);

  // Blade engagement drags the rail to 24.8 V — 20.0 % raw, i.e. exactly on
  // the trip — for 600 ms.
  ASSERT_NEAR(batteryPercentFromVoltage(24.8f, kEmptyV, kFullV), 20.0f, 0.01f);
  t = feed(filter, 24.8f, t, 0.6, kRobotPeriodS);

  EXPECT_GT(percentOf(filter), 20.0f) << "a 600 ms sag still trips LOW_BATTERY_DOCKING";

  // Load collapses, rail recovers: the percent must come back, not stay dented.
  feed(filter, 25.6f, t, 5.0, kRobotPeriodS);
  EXPECT_NEAR(percentOf(filter), 40.0f, 1.0f);
}

TEST(BatteryFilter, GenuineDischargeStillReachesTheThreshold)
{
  // The filter must lag, not block. Hold a real, unloaded 24.6 V (15 % of the
  // scale) and the trip has to arrive — a few seconds late is fine, never is
  // not.
  BatteryVoltageFilter filter;
  double t = feed(filter, 25.6f, 0.0, 20.0, kRobotPeriodS);
  ASSERT_GT(percentOf(filter), 20.0f);

  feed(filter, 24.6f, t, 4.0 * kBatteryFilterTauS, kRobotPeriodS);
  EXPECT_LT(percentOf(filter), 20.0f);
}

// ── Rate independence ──────────────────────────────────────────────────────

TEST(BatteryFilter, SmoothingIsTheSameOnTheRobotAndInSim)
{
  // A fixed EWMA weight would make the sim's 10 Hz smooth 2.5x harder than the
  // robot's 4 Hz. Deriving alpha from dt has to make the two agree after the
  // same amount of WALL-CLOCK time.
  BatteryVoltageFilter robot;
  BatteryVoltageFilter sim;
  robot.update(26.0f, 0.0);
  sim.update(26.0f, 0.0);

  feed(robot, 25.0f, 0.0, 3.0, kRobotPeriodS);
  feed(sim, 25.0f, 0.0, 3.0, kSimPeriodS);

  EXPECT_NEAR(*robot.value(), *sim.value(), 0.02f);
}

TEST(BatteryFilter, ReachesMostOfAStepAfterOneTimeConstant)
{
  // A first-order lag covers ~63 % of a step in one tau. This is what makes
  // kBatteryFilterTauS a knob you can size against a transient's duration.
  BatteryVoltageFilter filter;
  filter.update(26.0f, 0.0);
  feed(filter, 25.0f, 0.0, kBatteryFilterTauS, kRobotPeriodS);

  EXPECT_NEAR(*filter.value(), 26.0f - 0.63f, 0.03f);
}

// ── Invalid samples ────────────────────────────────────────────────────────

TEST(BatteryFilter, NoValueUntilAValidSampleArrives)
{
  // The caller keys "do not touch battery_percent" off nullopt. A glitched or
  // disconnected-pack reading must not become a 0 % gauge and a docking trip.
  BatteryVoltageFilter filter;
  EXPECT_FALSE(filter.value().has_value());
  EXPECT_FALSE(filter.update(0.0f, 0.0).has_value());
  EXPECT_FALSE(filter.update(-1.0f, 0.25).has_value());
  EXPECT_FALSE(filter.value().has_value());

  EXPECT_TRUE(filter.update(25.6f, 0.5).has_value());
}

TEST(BatteryFilter, BootstrapsOnTheFirstValidSampleWithoutRampingUp)
{
  // Seeding from a zero-initialised state would ramp the percent up from 0 %
  // over a full time constant on every startup.
  BatteryVoltageFilter filter;
  EXPECT_FLOAT_EQ(filter.update(25.6f, 10.0).value(), 25.6f);
}

TEST(BatteryFilter, HoldsTheLastGoodValueThroughADropout)
{
  BatteryVoltageFilter filter;
  double t = feed(filter, 25.6f, 0.0, 10.0, kRobotPeriodS);
  const float settled = *filter.value();

  for (int i = 0; i < 20; ++i)
  {
    t += kRobotPeriodS;
    EXPECT_FLOAT_EQ(filter.update(0.0f, t).value(), settled);
  }
}

TEST(BatteryFilter, SnapsToTheFreshReadingAfterALongDropout)
{
  // Ten seconds of nothing, then the pack reappears a volt lower (the bridge
  // reconnected, or the robot was moved meanwhile). Because an invalid sample
  // does not advance the filter's clock, the gap counts towards the next
  // valid sample's dt, so that sample carries most of the weight instead of
  // dragging a stale charge back in.
  BatteryVoltageFilter filter;
  double t = feed(filter, 26.0f, 0.0, 10.0, kRobotPeriodS);
  t = feed(filter, 0.0f, t, 10.0, kRobotPeriodS);

  filter.update(25.0f, t + kRobotPeriodS);
  EXPECT_NEAR(*filter.value(), 25.0f, 0.25f);

  // Contrast: with no gap, one sample moves only a small fraction of the step.
  BatteryVoltageFilter steady;
  steady.update(26.0f, 0.0);
  steady.update(25.0f, kRobotPeriodS);
  EXPECT_GT(*steady.value(), 25.8f);
}

// ── Clock and configuration edge cases ─────────────────────────────────────

TEST(BatteryFilter, StalledClockLeavesTheValueUnchanged)
{
  // use_sim_time before /clock starts ticking hands out the same stamp
  // forever. dt = 0 must not divide by zero or freeze into a wrong value.
  BatteryVoltageFilter filter;
  filter.update(26.0f, 7.0);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_FLOAT_EQ(filter.update(24.0f, 7.0).value(), 26.0f);
  }
}

TEST(BatteryFilter, NonPositiveTauDisablesSmoothing)
{
  BatteryVoltageFilter filter(0.0, mowgli_behavior::kBatteryMinValidVoltage);
  filter.update(26.0f, 0.0);
  EXPECT_FLOAT_EQ(filter.update(25.0f, 0.25).value(), 25.0f);
}

TEST(BatteryFilter, ResetRequiresANewBootstrap)
{
  BatteryVoltageFilter filter;
  filter.update(26.0f, 0.0);
  filter.reset();
  EXPECT_FALSE(filter.value().has_value());
  EXPECT_FLOAT_EQ(filter.update(25.0f, 0.25).value(), 25.0f);
}

// ── The percent interpolation ──────────────────────────────────────────────

TEST(BatteryPercent, InterpolatesAndClampsToTheEndpoints)
{
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(kEmptyV, kEmptyV, kFullV), 0.0f);
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(kFullV, kEmptyV, kFullV), 100.0f);
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(26.0f, kEmptyV, kFullV), 50.0f);
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(20.0f, kEmptyV, kFullV), 0.0f);
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(30.0f, kEmptyV, kFullV), 100.0f);
}

TEST(BatteryPercent, DegenerateRangeReportsZeroRatherThanDividingByZero)
{
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(26.0f, 26.0f, 26.0f), 0.0f);
  EXPECT_FLOAT_EQ(batteryPercentFromVoltage(26.0f, 28.0f, 24.0f), 0.0f);
}
