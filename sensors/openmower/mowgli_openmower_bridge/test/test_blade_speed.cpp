// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <chrono>
#include <cstdint>
#include <limits>

#include "mowgli_openmower_bridge/blade_speed.hpp"
#include <gtest/gtest.h>

using mowgli_openmower_bridge::HallSpeedMeter;
using mowgli_openmower_bridge::ShaftRpmFromErpm;
using std::chrono::milliseconds;

namespace
{
// 3000 rpm at 4 pole pairs: 3000 / 60 * 6 * 4 = 1200 hall steps per second.
constexpr uint32_t kStepsPerSecondAt3000 = 1200u;
}  // namespace

// Field report 2026-10-10: a blade spinning in reverse read -3000 rpm, so the
// GUI showed it stopped and map_server (>= 1000 rpm) stamped no mow progress.
TEST(ShaftRpmFromErpm, IsASpeedWhicheverWayTheBladeTurns)
{
  EXPECT_DOUBLE_EQ(ShaftRpmFromErpm(12000.0, 4), 3000.0);
  EXPECT_DOUBLE_EQ(ShaftRpmFromErpm(-12000.0, 4), 3000.0);
  EXPECT_DOUBLE_EQ(ShaftRpmFromErpm(0.0, 4), 0.0);
}

TEST(ShaftRpmFromErpm, RejectsNonsense)
{
  EXPECT_DOUBLE_EQ(ShaftRpmFromErpm(std::numeric_limits<double>::quiet_NaN(), 4), 0.0);
  EXPECT_DOUBLE_EQ(ShaftRpmFromErpm(12000.0, 0), 0.0);
}

TEST(HallSpeedMeter, MeasuresShaftRpmFromHallSteps)
{
  HallSpeedMeter meter(4);
  const auto t0 = HallSpeedMeter::Clock::time_point{} + std::chrono::seconds(10);
  EXPECT_DOUBLE_EQ(meter.Update(5000u, t0), 0.0);  // first sample only anchors
  // 20 ms samples: no estimate before the 200 ms window closes.
  uint32_t ticks = 5000u;
  for (int i = 1; i < 10; ++i)
  {
    ticks += kStepsPerSecondAt3000 / 50u;
    EXPECT_DOUBLE_EQ(meter.Update(ticks, t0 + milliseconds(20 * i)), 0.0);
  }
  ticks += kStepsPerSecondAt3000 / 50u;
  EXPECT_NEAR(meter.Update(ticks, t0 + milliseconds(200)), 3000.0, 1e-6);
  // tacho_absolute grows whatever the direction, so a reversed blade reads the same.
  ticks += kStepsPerSecondAt3000 / 5u;
  EXPECT_NEAR(meter.Update(ticks, t0 + milliseconds(400)), 3000.0, 1e-6);
}

TEST(HallSpeedMeter, StoppedBladeReadsZero)
{
  HallSpeedMeter meter(4);
  const auto t0 = HallSpeedMeter::Clock::time_point{} + std::chrono::seconds(10);
  meter.Update(100u, t0);
  meter.Update(340u, t0 + milliseconds(200));
  EXPECT_DOUBLE_EQ(meter.Update(340u, t0 + milliseconds(400)), 0.0);
}

// A controller reboot zeroes the counter: that is not a 20 000 rpm spike.
TEST(HallSpeedMeter, ACounterResetReanchorsInsteadOfSpiking)
{
  HallSpeedMeter meter(4);
  const auto t0 = HallSpeedMeter::Clock::time_point{} + std::chrono::seconds(10);
  meter.Update(1000000u, t0);
  EXPECT_DOUBLE_EQ(meter.Update(10u, t0 + milliseconds(200)), 0.0);
  EXPECT_NEAR(meter.Update(10u + kStepsPerSecondAt3000 / 5u, t0 + milliseconds(400)), 3000.0, 1e-6);
  meter.Reset();
  EXPECT_DOUBLE_EQ(meter.rpm(), 0.0);
}
