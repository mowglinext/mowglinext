// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for FTC's "collision_monitor is holding the robot" decision.
// Pure logic, no ROS.

#include "mowgli_nav2_plugins/ftc_collision_monitor_hold.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

namespace
{
constexpr std::uint8_t kDoNothing = 0;
constexpr std::uint8_t kStop = 1;
constexpr std::uint8_t kSlowdown = 2;
constexpr std::uint8_t kApproach = 3;
constexpr std::uint8_t kLimit = 4;
constexpr double kDt = 0.1;

/// Ticks needed (at kDt) until CmHoldStep first confirms, or -1 within `max_ticks`.
int ticksUntilConfirmed(
    bool cm_holds, double cmd, double measured, const mnp::FtcCmHoldCfg& cfg, int max_ticks = 400)
{
  double held = 0.0;
  double stalled = 0.0;
  for (int i = 1; i <= max_ticks; ++i)
  {
    if (mnp::CmHoldStep(cm_holds, cmd, measured, kDt, cfg, held, stalled))
    {
      return i;
    }
  }
  return -1;
}
}  // namespace

TEST(CollisionMonitorActionHolds, OnlyStopAndApproachHold)
{
  EXPECT_TRUE(mnp::CollisionMonitorActionHolds(kStop));
  EXPECT_TRUE(mnp::CollisionMonitorActionHolds(kApproach));
  EXPECT_FALSE(mnp::CollisionMonitorActionHolds(kDoNothing));
  EXPECT_FALSE(mnp::CollisionMonitorActionHolds(kSlowdown));
  EXPECT_FALSE(mnp::CollisionMonitorActionHolds(kLimit));
}

TEST(CmHoldStep, FieldCaseApproachHoldIsConfirmedAfterHoldS)
{
  // 2026-10-10: approach polygon held the robot, wheels at 0, FTC commanding
  // ~0.15 m/s. Must be confirmed after hold_s (3 s), not after the 30 s
  // progress checker.
  const mnp::FtcCmHoldCfg cfg;
  EXPECT_EQ(ticksUntilConfirmed(true, 0.15, 0.0, cfg), 30);
}

TEST(CmHoldStep, StallWithoutCollisionMonitorVerdictNeedsTheLongerTimeout)
{
  const mnp::FtcCmHoldCfg cfg;
  EXPECT_EQ(ticksUntilConfirmed(false, 0.15, 0.0, cfg), 100);
}

TEST(CmHoldStep, MovingRobotNeverConfirms)
{
  const mnp::FtcCmHoldCfg cfg;
  // Slowed by collision_monitor but still moving at half the command.
  EXPECT_EQ(ticksUntilConfirmed(true, 0.15, 0.075, cfg), -1);
}

TEST(CmHoldStep, NoForwardCommandNeverConfirms)
{
  const mnp::FtcCmHoldCfg cfg;
  // Holding still on purpose (pivot hold / obstacle wait) or reversing.
  EXPECT_EQ(ticksUntilConfirmed(true, 0.0, 0.0, cfg), -1);
  EXPECT_EQ(ticksUntilConfirmed(true, 0.03, 0.0, cfg), -1);
  EXPECT_EQ(ticksUntilConfirmed(true, -0.15, -0.15, cfg), -1);
}

TEST(CmHoldStep, MotionResetsBothTimers)
{
  const mnp::FtcCmHoldCfg cfg;
  double held = 0.0;
  double stalled = 0.0;
  for (int i = 0; i < 25; ++i)
  {
    ASSERT_FALSE(mnp::CmHoldStep(true, 0.15, 0.0, kDt, cfg, held, stalled));
  }
  EXPECT_FALSE(mnp::CmHoldStep(true, 0.15, 0.15, kDt, cfg, held, stalled));
  EXPECT_DOUBLE_EQ(held, 0.0);
  EXPECT_DOUBLE_EQ(stalled, 0.0);
}

TEST(CmHoldStep, CollisionMonitorReleaseRestartsOnlyTheHoldTimer)
{
  const mnp::FtcCmHoldCfg cfg;
  double held = 0.0;
  double stalled = 0.0;
  for (int i = 0; i < 25; ++i)
  {
    ASSERT_FALSE(mnp::CmHoldStep(true, 0.15, 0.0, kDt, cfg, held, stalled));
  }
  // Verdict drops to DO_NOTHING while the robot is still stopped.
  EXPECT_FALSE(mnp::CmHoldStep(false, 0.15, 0.0, kDt, cfg, held, stalled));
  EXPECT_DOUBLE_EQ(held, 0.0);
  EXPECT_NEAR(stalled, 2.6, 1e-9);
}

TEST(CmHoldStep, EachTriggerCanBeDisabled)
{
  mnp::FtcCmHoldCfg cfg;
  cfg.hold_s = 0.0;
  EXPECT_EQ(ticksUntilConfirmed(true, 0.15, 0.0, cfg), 100);  // only the stall trigger left
  cfg.stall_only_s = 0.0;
  EXPECT_EQ(ticksUntilConfirmed(true, 0.15, 0.0, cfg), -1);  // both off
}

TEST(CmHoldStep, StallOnlyTimeoutStaysBelowTheProgressChecker)
{
  // controller_server's movement_time_allowance is 30 s: the default must
  // confirm well before it so the abort carries FTC's obstacle marker.
  const mnp::FtcCmHoldCfg cfg;
  EXPECT_LT(cfg.stall_only_s, 30.0);
  EXPECT_LT(cfg.hold_s, cfg.stall_only_s);
}
