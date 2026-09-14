// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "heartbeat_emergency_policy.hpp"
#include <gtest/gtest.h>

TEST(HeartbeatEmergencyPolicy, NoFlagsLeaveEmergencyUnchanged)
{
  const auto decision = decide_heartbeat_emergency(false, false, false, false);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Unchanged);
  EXPECT_FALSE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, StopAssertsEmergency)
{
  const auto decision = decide_heartbeat_emergency(true, false, false, false);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Assert);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, ReleaseOnlyWithoutPhysicalEmergencyReleases)
{
  const auto decision = decide_heartbeat_emergency(false, true, false, false);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Release);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, ReleaseOnlyWithPhysicalEmergencyIsRejected)
{
  const auto decision = decide_heartbeat_emergency(false, true, true, false);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Unchanged);
  EXPECT_FALSE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, StopDominatesReleaseWithoutPhysicalEmergency)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, true, false, false).action,
            HeartbeatEmergencyAction::Assert);
}

TEST(HeartbeatEmergencyPolicy, StopDominatesReleaseWithPhysicalEmergency)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, true, true, false).action,
            HeartbeatEmergencyAction::Assert);
}

TEST(HeartbeatEmergencyPolicy, ReleaseWorksAfterPreviousStop)
{
  EXPECT_EQ(decide_heartbeat_emergency(true, false, false, false).action,
            HeartbeatEmergencyAction::Assert);
  EXPECT_EQ(decide_heartbeat_emergency(false, true, false, false).action,
            HeartbeatEmergencyAction::Release);
}

TEST(HeartbeatEmergencyPolicy, NormalHeartbeatAutoClearsPureWatchdogLatch)
{
  const auto decision = decide_heartbeat_emergency(false, false, false, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::AutoClearWatchdog);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, PhysicalEmergencyPreventsWatchdogAutoClear)
{
  const auto decision = decide_heartbeat_emergency(false, false, true, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Unchanged);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, WatchdogRecoveryPrecedesExplicitRelease)
{
  const auto decision = decide_heartbeat_emergency(false, true, false, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::AutoClearWatchdog);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, PhysicalEmergencyRejectsReleaseAfterWatchdogLatch)
{
  const auto decision = decide_heartbeat_emergency(false, true, true, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Unchanged);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, WatchdogAutoClearCannotBypassExplicitStop)
{
  const auto decision = decide_heartbeat_emergency(true, true, false, true);
  EXPECT_EQ(decision.action, HeartbeatEmergencyAction::Assert);
  EXPECT_TRUE(decision.clear_heartbeat_only_latch);
}

TEST(HeartbeatEmergencyPolicy, CompleteTruthTable)
{
  struct Case
  {
    bool stop;
    bool release;
    bool physical;
    bool watchdog_latch_active;
    HeartbeatEmergencyAction action;
    bool clear_latch;
  };

  const Case cases[] = {
      {false, false, false, false, HeartbeatEmergencyAction::Unchanged, false},
      {false, false, false, true, HeartbeatEmergencyAction::AutoClearWatchdog, true},
      {false, false, true, false, HeartbeatEmergencyAction::Unchanged, false},
      {false, false, true, true, HeartbeatEmergencyAction::Unchanged, true},
      {false, true, false, false, HeartbeatEmergencyAction::Release, true},
      {false, true, false, true, HeartbeatEmergencyAction::AutoClearWatchdog, true},
      {false, true, true, false, HeartbeatEmergencyAction::Unchanged, false},
      {false, true, true, true, HeartbeatEmergencyAction::Unchanged, true},
      {true, false, false, false, HeartbeatEmergencyAction::Assert, true},
      {true, false, false, true, HeartbeatEmergencyAction::Assert, true},
      {true, false, true, false, HeartbeatEmergencyAction::Assert, true},
      {true, false, true, true, HeartbeatEmergencyAction::Assert, true},
      {true, true, false, false, HeartbeatEmergencyAction::Assert, true},
      {true, true, false, true, HeartbeatEmergencyAction::Assert, true},
      {true, true, true, false, HeartbeatEmergencyAction::Assert, true},
      {true, true, true, true, HeartbeatEmergencyAction::Assert, true},
  };

  for (const auto& test_case : cases)
  {
    const auto decision = decide_heartbeat_emergency(test_case.stop,
                                                     test_case.release,
                                                     test_case.physical,
                                                     test_case.watchdog_latch_active);
    EXPECT_EQ(decision.action, test_case.action);
    EXPECT_EQ(decision.clear_heartbeat_only_latch, test_case.clear_latch);
  }
}
