// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gtest/gtest.h"
#include "mowgli_lidar_pwm/pwm_controller.hpp"

using mowgli_lidar_pwm::PwmCommand;
using mowgli_lidar_pwm::PwmControllerCfg;
using mowgli_lidar_pwm::PwmControllerState;
using mowgli_lidar_pwm::PwmControllerStep;
using mowgli_lidar_pwm::PwmMotorState;

namespace
{
PwmControllerCfg TestCfg()
{
  PwmControllerCfg cfg;
  cfg.handshake_duty = 0.50;
  cfg.handshake_hold_s = 0.15;
  cfg.run_duty = 0.80;
  cfg.stop_duty = 0.0;
  cfg.spinup_timeout_s = 2.0;
  cfg.spindown_settle_s = 0.5;
  return cfg;
}
}  // namespace

TEST(PwmController, StartsIdleAndStaysIdleWhileInactive)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  const auto cmd = PwmControllerStep(cfg, st, /*active=*/false, /*scan_fresh=*/false, 0.1);

  EXPECT_EQ(cmd.state, PwmMotorState::kIdle);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.stop_duty);
}

TEST(PwmController, ActiveEntersSpinningUpAtHandshakeDuty)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  const auto cmd = PwmControllerStep(cfg, st, /*active=*/true, /*scan_fresh=*/false, 0.01);

  EXPECT_EQ(cmd.state, PwmMotorState::kSpinningUp);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.handshake_duty);
}

TEST(PwmController, SpinningUpSwitchesToRunDutyAfterHandshakeHold)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);  // enter SPINNING_UP
  PwmCommand cmd;
  for (int i = 0; i < 20; ++i) {  // 20 * 0.01 = 0.2s > handshake_hold_s (0.15)
    cmd = PwmControllerStep(cfg, st, true, false, 0.01);
  }

  EXPECT_EQ(cmd.state, PwmMotorState::kSpinningUp);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.run_duty);
}

TEST(PwmController, FreshScanDuringSpinUpConfirmsRunning)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);
  const auto cmd = PwmControllerStep(cfg, st, true, /*scan_fresh=*/true, 0.01);

  EXPECT_EQ(cmd.state, PwmMotorState::kRunning);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.run_duty);
}

TEST(PwmController, RunningDoesNotFaultOnLaterStaleScan)
{
  // Deliberate: once confirmed RUNNING, this class does not re-supervise
  // scan health — that stays owned by the existing BT-level guards. See the
  // class comment in pwm_controller.hpp.
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);
  PwmControllerStep(cfg, st, true, true, 0.01);  // -> RUNNING
  const auto cmd = PwmControllerStep(cfg, st, true, /*scan_fresh=*/false, 0.01);

  EXPECT_EQ(cmd.state, PwmMotorState::kRunning);
}

TEST(PwmController, SpinUpTimesOutToFaultWithoutAScan)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmCommand cmd;
  // 25 * 0.1s = 2.5s > spinup_timeout_s (2.0s)
  for (int i = 0; i < 25; ++i) {
    cmd = PwmControllerStep(cfg, st, true, false, 0.1);
  }

  EXPECT_EQ(cmd.state, PwmMotorState::kFault);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.run_duty);  // keeps trying
}

TEST(PwmController, FaultRecoversToRunningOnFreshScan)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  for (int i = 0; i < 25; ++i) {
    PwmControllerStep(cfg, st, true, false, 0.1);
  }
  ASSERT_EQ(st.state, PwmMotorState::kFault);

  const auto cmd = PwmControllerStep(cfg, st, true, /*scan_fresh=*/true, 0.1);

  EXPECT_EQ(cmd.state, PwmMotorState::kRunning);
}

TEST(PwmController, GoingInactiveFromRunningSpinsDownThenIdles)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);
  PwmControllerStep(cfg, st, true, true, 0.01);  // -> RUNNING
  ASSERT_EQ(st.state, PwmMotorState::kRunning);

  auto cmd = PwmControllerStep(cfg, st, /*active=*/false, false, 0.01);
  EXPECT_EQ(cmd.state, PwmMotorState::kSpinningDown);
  EXPECT_DOUBLE_EQ(cmd.duty_cycle, cfg.stop_duty);

  for (int i = 0; i < 60; ++i) {  // 60 * 0.01 = 0.6s > spindown_settle_s (0.5)
    cmd = PwmControllerStep(cfg, st, false, false, 0.01);
  }
  EXPECT_EQ(cmd.state, PwmMotorState::kIdle);
}

TEST(PwmController, ActiveAgainDuringSpinDownResumesSpinningUp)
{
  const auto cfg = TestCfg();
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);
  PwmControllerStep(cfg, st, true, true, 0.01);  // -> RUNNING
  PwmControllerStep(cfg, st, false, false, 0.01);  // -> SPINNING_DOWN
  ASSERT_EQ(st.state, PwmMotorState::kSpinningDown);

  const auto cmd = PwmControllerStep(cfg, st, /*active=*/true, false, 0.01);

  EXPECT_EQ(cmd.state, PwmMotorState::kSpinningUp);
}

TEST(PwmController, DutyCycleAlwaysClampedToUnitRange)
{
  auto cfg = TestCfg();
  cfg.run_duty = 1.5;   // deliberately out of range
  cfg.stop_duty = -0.2;  // deliberately out of range
  PwmControllerState st;

  PwmControllerStep(cfg, st, true, false, 0.01);
  const auto running = PwmControllerStep(cfg, st, true, true, 0.01);
  EXPECT_LE(running.duty_cycle, 1.0);

  const auto idle = PwmControllerStep(cfg, st, false, false, 1.0);
  EXPECT_GE(idle.duty_cycle, 0.0);
}
