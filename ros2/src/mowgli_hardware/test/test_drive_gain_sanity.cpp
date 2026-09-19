// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <limits>

#include "mowgli_hardware/drive_gain_sanity.hpp"
#include <gtest/gtest.h>

namespace mowgli_hardware
{

namespace
{
// Template defaults (ros2/src/mowgli_bringup/config/mowgli_robot.yaml).
constexpr double kTemplateKp = 10.0;
constexpr double kTemplateIntegralLimit = 45.0;
constexpr double kTemplatePwmPerMps = 282.135;
constexpr double kTemplateDeadbandPwm = 40.0;
}  // namespace

TEST(DriveGainSanity, TemplateDefaultsBridgeTheDeadband)
{
  // 45 + 10·0.03 + 282.135·0.03 ≈ 53.8 PWM ≥ 40.
  EXPECT_TRUE(DriveGainsBridgeDeadband(
      kTemplateKp, kTemplateIntegralLimit, kTemplatePwmPerMps, kTemplateDeadbandPwm));
  EXPECT_NEAR(DriveGainsBridgePwm(kTemplateKp, kTemplateIntegralLimit, kTemplatePwmPerMps),
              53.76405,
              1e-4);
}

TEST(DriveGainSanity, FieldFailureGainsAreRejected)
{
  // The 2026-09-15 stiction-locked set: 15 + 0.2·0.03 + 282.135·0.03 ≈ 23.5 PWM.
  EXPECT_FALSE(DriveGainsBridgeDeadband(0.2, 15.0, kTemplatePwmPerMps, kTemplateDeadbandPwm));
  EXPECT_LT(DriveGainsBridgePwm(0.2, 15.0, kTemplatePwmPerMps), kTemplateDeadbandPwm);
}

TEST(DriveGainSanity, BoundaryIsInclusive)
{
  // integral_limit chosen so the ceiling lands exactly on the deadband.
  const double ff = kTemplatePwmPerMps * kDriveGainProbeSpeedMps;
  const double p = kTemplateKp * kDriveGainProbeSpeedMps;
  const double exact_limit = kTemplateDeadbandPwm - ff - p;
  EXPECT_TRUE(
      DriveGainsBridgeDeadband(kTemplateKp, exact_limit, kTemplatePwmPerMps, kTemplateDeadbandPwm));
  EXPECT_FALSE(DriveGainsBridgeDeadband(
      kTemplateKp, exact_limit - 1e-6, kTemplatePwmPerMps, kTemplateDeadbandPwm));
}

TEST(DriveGainSanity, IntegralLimitAloneCanBridgeWithZeroGains)
{
  // pwm_per_mps is clamped ≥ 50 by the firmware, so the smallest feedforward
  // contribution is 1.5 PWM; a 40 PWM integral limit still passes.
  EXPECT_TRUE(DriveGainsBridgeDeadband(0.0, 40.0, 50.0, kTemplateDeadbandPwm));
  EXPECT_FALSE(DriveGainsBridgeDeadband(0.0, 30.0, 50.0, kTemplateDeadbandPwm));
}

TEST(DriveGainSanity, NonFiniteInputsNeverPass)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(DriveGainsBridgeDeadband(
      nan, kTemplateIntegralLimit, kTemplatePwmPerMps, kTemplateDeadbandPwm));
  EXPECT_FALSE(
      DriveGainsBridgeDeadband(kTemplateKp, inf, kTemplatePwmPerMps, kTemplateDeadbandPwm));
  EXPECT_FALSE(
      DriveGainsBridgeDeadband(kTemplateKp, kTemplateIntegralLimit, nan, kTemplateDeadbandPwm));
  EXPECT_FALSE(
      DriveGainsBridgeDeadband(kTemplateKp, kTemplateIntegralLimit, kTemplatePwmPerMps, nan));
}

}  // namespace mowgli_hardware
