// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_hardware/cmd_vel_slew.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::limit_motion_command_slew;

TEST(CmdVelSlew, RampsAwayFromRest)
{
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.30, 0.0, 0.1, 0.30, 0.60), 0.03);
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(-0.80, 0.0, 0.1, 1.0, 2.0), -0.1);
}

TEST(CmdVelSlew, UsesSeparateAccelerationAndDecelerationLimits)
{
  EXPECT_NEAR(limit_motion_command_slew(0.30, 0.10, 0.1, 0.30, 0.60), 0.13, 1.0e-12);
  EXPECT_NEAR(limit_motion_command_slew(0.10, 0.30, 0.1, 0.30, 0.60), 0.24, 1.0e-12);
  EXPECT_NEAR(limit_motion_command_slew(-0.80, -0.30, 0.1, 1.0, 2.0), -0.40, 1.0e-12);
  EXPECT_NEAR(limit_motion_command_slew(-0.20, -0.80, 0.1, 1.0, 2.0), -0.60, 1.0e-12);
}

TEST(CmdVelSlew, ExplicitStopIsImmediate)
{
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.0, 0.30, 0.01, 0.30, 0.60), 0.0);
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.0, -0.80, 0.01, 1.0, 2.0), 0.0);
}

TEST(CmdVelSlew, DirectionReversalDeceleratesThroughZeroBeforeAccelerating)
{
  // Worst steering reversal measured in the 2026-09-09 field bag:
  // -0.75 -> +0.273 rad/s in one 100 ms controller period.
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.273, -0.75, 0.1, 1.0, 2.0), -0.55);
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.273, -0.15, 0.1, 1.0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.273, 0.0, 0.1, 1.0, 2.0), 0.1);
}

TEST(CmdVelSlew, ReachesTargetWithoutOvershoot)
{
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(0.20, 0.19, 0.1, 0.30, 0.60), 0.20);
  EXPECT_DOUBLE_EQ(limit_motion_command_slew(-0.20, -0.19, 0.1, 0.30, 0.60), -0.20);
}

namespace
{
class CmdVelPipelineTest : public ::testing::Test
{
protected:
  mowgli_hardware::CmdVelPipeline pipeline;
  std::int64_t now_ns{1'000'000'000};
  mowgli_hardware::MotionCommand step(double vx, double wz = 0.0, bool autonomous = true)
  {
    const auto command = pipeline.shape({vx, wz}, autonomous, 0.05, {}, now_ns);
    now_ns += 100'000'000;
    pipeline.record_applied(static_cast<float>(command.vx), static_cast<float>(command.wz));
    return command;
  }
};

TEST_F(CmdVelPipelineTest, RestResumesKeepPrecisionTargetsWithoutAFloor)
{
  for (double speed : {0.0011, 0.01, 0.025 / 0.8, 0.030 / 0.8, 0.039 / 0.8})
  {
    for (double sign : {1.0, -1.0})
    {
      pipeline.reset();
      const auto first = step(sign * speed);
      EXPECT_NEAR(first.vx, sign * std::min(speed, 0.03), 1e-12);
      for (int i = 0; i < 10; ++i)
      {
        const auto command = step(sign * speed);
        EXPECT_LE(std::abs(command.vx), speed);
        EXPECT_GT(sign * command.vx, 0.0);
      }
    }
  }
}

TEST_F(CmdVelPipelineTest, ManualInputNoiseCannotAccumulateAStartup)
{
  for (int i = 0; i < 30; ++i)
    EXPECT_DOUBLE_EQ(step(i % 2 == 0 ? 0.03 : -0.03, 0.0, false).vx, 0.0);
}

TEST_F(CmdVelPipelineTest, ManualWireGuardPreservesContinuousNormalStartup)
{
  EXPECT_DOUBLE_EQ(step(0.3, 0.0, false).vx, 0.0);
  EXPECT_NEAR(step(0.3, 0.0, false).vx, 0.06, 1e-12);
}

TEST_F(CmdVelPipelineTest, ModeHandoverUsesAppliedOutputForReversal)
{
  for (int i = 0; i < 10; ++i)
    step(0.3, 0.0, false);
  EXPECT_NEAR(step(-0.03).vx, 0.24, 1e-6);
  EXPECT_DOUBLE_EQ(step(0.0).vx, 0.0);
  EXPECT_NEAR(step(-0.03).vx, -0.03, 1e-12);
}

TEST_F(CmdVelPipelineTest, ExplicitStopIsImmediateForBothAxes)
{
  for (int i = 0; i < 10; ++i)
    step(0.3, 0.8);
  const auto stop = step(0.0);
  EXPECT_DOUBLE_EQ(stop.vx, 0.0);
  EXPECT_DOUBLE_EQ(stop.wz, 0.0);
}

TEST_F(CmdVelPipelineTest, StaleStreamRestartsAtNominalStepFromRest)
{
  for (int i = 0; i < 10; ++i)
    step(0.3, 0.8);
  now_ns += 1'000'000'000;
  const auto resumed = step(-0.3, -0.8);
  EXPECT_NEAR(resumed.vx, -0.03, 1e-12);
  EXPECT_NEAR(resumed.wz, -0.1, 1e-12);
}

TEST_F(CmdVelPipelineTest, ClockRegressionRestartsAndLongValidStepIsCapped)
{
  step(0.3);
  now_ns -= 200'000'000;
  EXPECT_NEAR(step(-0.3).vx, -0.03, 1e-12);
  pipeline.reset();
  step(0.3);
  now_ns += 300'000'000;
  EXPECT_NEAR(step(0.3).vx, 0.09, 1e-12);
}

TEST_F(CmdVelPipelineTest, DisconnectOrDigResetCannotRetainRamp)
{
  for (int i = 0; i < 10; ++i)
    step(0.3);
  pipeline.reset();
  EXPECT_NEAR(step(0.3).vx, 0.03, 1e-12);
}

TEST_F(CmdVelPipelineTest, AppliedZeroCannotBeConfusedWithSuppressedRampOnHandover)
{
  EXPECT_DOUBLE_EQ(step(0.3, 0.0, false).vx, 0.0);
  EXPECT_NEAR(step(0.3).vx, 0.03, 1e-12);
  EXPECT_DOUBLE_EQ(step(0.03, 0.0, false).vx, 0.0);
  EXPECT_NEAR(step(0.3).vx, 0.03, 1e-12);
}
}  // namespace

TEST(LinearCommandPolicy, OnlyFreshReportedAutonomyIsExempt)
{
  using mowgli_interfaces::autonomous_precision_allowed;
  constexpr std::int64_t receipt = 10'000'000'000;
  for (bool reported_auto : {false, true})
    for (bool have_status : {false, true})
      for (std::int64_t age : {-1LL, 0LL, 1'000'000'000LL, 2'000'000'000LL, 2'000'000'001LL})
        EXPECT_EQ(autonomous_precision_allowed(reported_auto, have_status, receipt + age, receipt),
                  reported_auto && have_status && age >= 0 && age <= 2'000'000'000);
  EXPECT_FALSE(autonomous_precision_allowed(true, true, receipt, 0));
  EXPECT_TRUE(autonomous_precision_allowed(true, true, receipt + 4'000'000'000, receipt, 5.0));
}

TEST(LinearCommandPolicy, ManualGuardRetainsExactDustAndThresholdBoundaries)
{
  for (double v : {-0.06, -0.05, -0.03, -0.001, -0.0005, 0.0, 0.0005, 0.001, 0.03, 0.05, 0.06})
  {
    const double legacy = std::abs(v) > 1e-3 && std::abs(v) < .05 ? 0.0 : v;
    EXPECT_DOUBLE_EQ(mowgli_interfaces::apply_linear_soft_deadband(v, .05, false), legacy);
    EXPECT_DOUBLE_EQ(mowgli_interfaces::apply_linear_soft_deadband(v, .05, true), v);
    EXPECT_DOUBLE_EQ(mowgli_interfaces::apply_linear_soft_deadband(0.0, .05, true), 0.0);
  }
}
