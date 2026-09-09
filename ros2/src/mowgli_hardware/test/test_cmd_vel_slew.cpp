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
