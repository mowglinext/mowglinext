// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the Tier-1 motor-controller-speed auto-scale velocity estimate
// (motor_speed_velocity.hpp).

#include <cmath>

#include "gtest/gtest.h"
#include "mowgli_hardware/motor_speed_velocity.hpp"

using mowgli_hardware::motor_speed_velocity;

namespace
{
constexpr double kAlpha = 0.02;
}

TEST(MotorSpeedVelocity, UncalibratedReturnsFallback)
{
  double scale = 0.0;
  // motor_speed below the calibration deadband (|units| < 2) → no calibration,
  // scale stays 0 → returns the fallback.
  EXPECT_DOUBLE_EQ(motor_speed_velocity(0.0, 0.123, 0, scale, kAlpha), 0.123);
  EXPECT_DOUBLE_EQ(scale, 0.0);
}

TEST(MotorSpeedVelocity, CalibratesToTheRatio)
{
  // Steady truth: 0.20 m/s reported as 40 controller units → scale should
  // converge to 0.005 m/s/unit.
  double scale = 0.0;
  const double meas = 0.20;
  const int16_t units = 40;
  for (int i = 0; i < 2000; ++i)
  {
    motor_speed_velocity(meas, /*fallback=*/0.0, units, scale, kAlpha);
  }
  EXPECT_NEAR(scale, 0.005, 1e-4);
  // Once calibrated it returns scale * units (smooth), not the fallback.
  EXPECT_NEAR(motor_speed_velocity(meas, 0.0, units, scale, kAlpha), 0.20, 2e-3);
}

TEST(MotorSpeedVelocity, AveragesOutTickQuantisation)
{
  // The tick-derived meas is quantised (alternates 0.15 / 0.30, true mean 0.225)
  // while the controller reports a STEADY 45 units. The slow EMA on the ratio
  // converges to the mean ratio (0.225/45 = 0.005), so the output is SMOOTH
  // (constant) even though meas jitters — the whole point of Tier 1.
  double scale = 0.0;
  const int16_t units = 45;
  for (int i = 0; i < 4000; ++i)
  {
    const double meas = (i % 2 == 0) ? 0.15 : 0.30;
    motor_speed_velocity(meas, 0.0, units, scale, kAlpha);
  }
  EXPECT_NEAR(scale, 0.225 / 45.0, 2e-4);
  const double out = motor_speed_velocity(0.15, 0.0, units, scale, kAlpha);
  EXPECT_NEAR(out, 0.225, 3e-3);  // smooth ≈ true mean, NOT the 0.15 jitter sample
}

TEST(MotorSpeedVelocity, SignDisagreementSkipsCalibration)
{
  // meas forward, controller reports reverse → garbage pairing → must not learn.
  double scale = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_DOUBLE_EQ(motor_speed_velocity(0.20, 0.111, -40, scale, kAlpha), 0.111);
  }
  EXPECT_DOUBLE_EQ(scale, 0.0);
}

TEST(MotorSpeedVelocity, RejectsImplausibleRatio)
{
  // ratio >= 1.0 m/s per unit is implausible (would mean 1 unit = >1 m/s) → skip.
  double scale = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    motor_speed_velocity(/*meas=*/3.0, 0.0, /*units=*/2, scale, kAlpha);  // ratio 1.5
  }
  EXPECT_DOUBLE_EQ(scale, 0.0);
}

TEST(MotorSpeedVelocity, ScaleNeverNegative)
{
  double scale = 0.0;
  // Only positive-ratio samples can ever update scale, so it stays >= 0.
  for (int i = 0; i < 500; ++i)
  {
    motor_speed_velocity(0.18, 0.0, 36, scale, kAlpha);
    EXPECT_GE(scale, 0.0);
  }
}

TEST(MotorSpeedVelocity, TracksSpeedChangeWithoutLag)
{
  // After calibrating at one speed, a step to a new controller reading is
  // reflected IMMEDIATELY (output = scale * units) — no EMA lag on the velocity
  // itself (the EMA is only on the slowly-varying scale).
  double scale = 0.0;
  for (int i = 0; i < 3000; ++i)
  {
    motor_speed_velocity(0.20, 0.0, 40, scale, kAlpha);  // calibrate ~0.005
  }
  // New packet: controller jumps to 80 units → output ~0.40 right away.
  EXPECT_NEAR(motor_speed_velocity(0.40, 0.0, 80, scale, kAlpha), 0.40, 5e-3);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
