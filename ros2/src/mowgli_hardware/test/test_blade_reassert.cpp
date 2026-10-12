// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the blade intent re-assert. Pure logic, no ROS — mirrors
// test_blade_gate.cpp.

#include "mowgli_hardware/blade_reassert.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// The field case: the BT asked for the blade, the firmware dropped the request
// and reports the blade stopped, and the coverage controller is now driving.
mh::BladeReassertInputs dropped_request_while_driving()
{
  mh::BladeReassertInputs in;
  in.mow_enabled = true;
  in.enable_allowed = true;
  in.intent_authorized = true;
  in.emergency_active = false;
  in.blade_status_fresh = true;
  in.blade_active = false;
  in.cmd_vel_age_s = 0.03;
  in.since_last_blade_cmd_s = 2.0;
  return in;
}

}  // namespace

TEST(BladeReassert, ReassertsADroppedRequestOnceCmdVelFlows)
{
  EXPECT_TRUE(mh::should_reassert_blade_on(dropped_request_while_driving()));
}

// The 2026-10-04 12:33:40 sequence: FollowStrip's ON was sent after a 1.5 s
// spin-up wait with no controller publishing. A re-assert in that window would
// be refused the same way, so it waits for the first cmd_vel instead.
TEST(BladeReassert, WaitsWhileCmdVelIsStale)
{
  auto in = dropped_request_while_driving();
  in.cmd_vel_age_s = 1.5;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  EXPECT_TRUE(mh::blade_intent_mismatch(in));

  in.cmd_vel_age_s = 1e9;  // never sent
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
}

// The firmware reads a stale ON as an OFF, so the window must stay well inside
// its 200 ms CMD_VEL watchdog.
TEST(BladeReassert, CmdVelWindowStaysInsideTheFirmwareWatchdog)
{
  const mh::BladeReassertConfig cfg;
  EXPECT_LE(cfg.max_cmd_vel_age_s, 0.10);

  auto in = dropped_request_while_driving();
  in.cmd_vel_age_s = cfg.max_cmd_vel_age_s;
  EXPECT_TRUE(mh::should_reassert_blade_on(in, cfg));
  in.cmd_vel_age_s = cfg.max_cmd_vel_age_s + 0.001;
  EXPECT_FALSE(mh::should_reassert_blade_on(in, cfg));
}

// SAFETY-CRITICAL: the firmware forgets the blade request on an emergency so
// that clearing it cannot restart the blade (#763). The bridge must not undo
// that by asking again on its own.
TEST(BladeReassert, NeverDuringAnEmergency)
{
  auto in = dropped_request_while_driving();
  in.emergency_active = true;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
}

TEST(BladeReassert, NeverAfterAnEmergencyWithoutANewExplicitRequest)
{
  auto in = dropped_request_while_driving();
  in.intent_authorized = false;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  // Still reported, so the operator sees why the blade is not turning.
  EXPECT_TRUE(mh::blade_intent_mismatch(in));
}

TEST(BladeReassert, NeverWithoutHostIntent)
{
  auto in = dropped_request_while_driving();
  in.mow_enabled = false;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  EXPECT_FALSE(mh::blade_intent_mismatch(in));
}

// The lift handling owns the blade while a lift or its delayed resume is in
// progress; a re-assert there would bypass the lift-clear delay.
TEST(BladeReassert, NeverWhileTheLiftHandlingOwnsTheBlade)
{
  auto in = dropped_request_while_driving();
  in.enable_allowed = false;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  EXPECT_FALSE(mh::blade_intent_mismatch(in));
}

TEST(BladeReassert, NothingToDoWhenTheBladeRuns)
{
  auto in = dropped_request_while_driving();
  in.blade_active = true;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  EXPECT_FALSE(mh::blade_intent_mismatch(in));
}

// Without fresh telemetry the bridge does not know the blade is stopped, and
// must not guess: a blind ON on a stale cmd_vel would stop a running blade.
TEST(BladeReassert, NeverOnStaleBladeTelemetry)
{
  auto in = dropped_request_while_driving();
  in.blade_status_fresh = false;
  EXPECT_FALSE(mh::should_reassert_blade_on(in));
  EXPECT_FALSE(mh::blade_intent_mismatch(in));
}

TEST(BladeReassert, IsRateLimited)
{
  const mh::BladeReassertConfig cfg;
  auto in = dropped_request_while_driving();
  in.since_last_blade_cmd_s = cfg.min_interval_s - 0.01;
  EXPECT_FALSE(mh::should_reassert_blade_on(in, cfg));
  in.since_last_blade_cmd_s = cfg.min_interval_s;
  EXPECT_TRUE(mh::should_reassert_blade_on(in, cfg));
}

TEST(BladeReassert, DefaultInputsAskForNothing)
{
  static_assert(!mh::should_reassert_blade_on(mh::BladeReassertInputs{}),
                "a default-constructed input must never produce a blade ON");
  SUCCEED();
}
