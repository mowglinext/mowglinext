// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the velocity (m/s) → signed PWM feedforward used by the
// MowgliSystemInterface ros2_control plugin (velocity_pwm.hpp).

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "mowgli_hardware/velocity_pwm.hpp"

using mowgli_hardware::velocity_to_pwm;

namespace
{
constexpr double kSlope = 300.0;     // pwm_per_mps
constexpr double kDeadband = 40.0;   // break-free PWM
constexpr double kMaxPwm = 255.0;
constexpr double kEps = 5.0e-3;
}  // namespace

TEST(VelocityToPwm, ZeroBelowEpsilon)
{
  EXPECT_DOUBLE_EQ(velocity_to_pwm(0.0, kSlope, kDeadband, kMaxPwm, kEps), 0.0);
  EXPECT_DOUBLE_EQ(velocity_to_pwm(0.001, kSlope, kDeadband, kMaxPwm, kEps), 0.0);
  EXPECT_DOUBLE_EQ(velocity_to_pwm(-0.001, kSlope, kDeadband, kMaxPwm, kEps), 0.0);
}

TEST(VelocityToPwm, ForwardAddsDeadband)
{
  // Just above the deadband: ff = slope*v + deadband.
  const double v = 0.1;
  EXPECT_NEAR(velocity_to_pwm(v, kSlope, kDeadband, kMaxPwm, kEps), kSlope * v + kDeadband, 1e-9);
}

TEST(VelocityToPwm, ReverseSubtractsDeadband)
{
  const double v = -0.1;
  EXPECT_NEAR(velocity_to_pwm(v, kSlope, kDeadband, kMaxPwm, kEps), kSlope * v - kDeadband, 1e-9);
}

TEST(VelocityToPwm, OddSymmetry)
{
  const double v = 0.23;
  EXPECT_NEAR(velocity_to_pwm(v, kSlope, kDeadband, kMaxPwm, kEps),
              -velocity_to_pwm(-v, kSlope, kDeadband, kMaxPwm, kEps), 1e-9);
}

TEST(VelocityToPwm, ClampsToMax)
{
  // 1.0 m/s would be 300+40 = 340 PWM, clamped to 255.
  EXPECT_DOUBLE_EQ(velocity_to_pwm(1.0, kSlope, kDeadband, kMaxPwm, kEps), kMaxPwm);
  EXPECT_DOUBLE_EQ(velocity_to_pwm(-1.0, kSlope, kDeadband, kMaxPwm, kEps), -kMaxPwm);
}

TEST(VelocityToPwm, NonFiniteIsZero)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(velocity_to_pwm(nan, kSlope, kDeadband, kMaxPwm, kEps), 0.0);
  EXPECT_DOUBLE_EQ(velocity_to_pwm(inf, kSlope, kDeadband, kMaxPwm, kEps), 0.0);
}

TEST(VelocityToPwm, MonotonicAcrossOperatingRange)
{
  double prev = velocity_to_pwm(0.02, kSlope, kDeadband, kMaxPwm, kEps);
  for (double v = 0.03; v <= 0.6; v += 0.01)
  {
    const double cur = velocity_to_pwm(v, kSlope, kDeadband, kMaxPwm, kEps);
    EXPECT_GE(cur + 1e-9, prev) << "non-monotonic at v=" << v;
    prev = cur;
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
