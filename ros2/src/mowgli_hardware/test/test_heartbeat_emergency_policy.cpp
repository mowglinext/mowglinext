// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "heartbeat_emergency_policy.hpp"
#include <gtest/gtest.h>

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
    SCOPED_TRACE(::testing::Message()
                 << "stop=" << test_case.stop << " release=" << test_case.release << " physical="
                 << test_case.physical << " watchdog=" << test_case.watchdog_latch_active);
    const auto decision = decide_heartbeat_emergency(test_case.stop,
                                                     test_case.release,
                                                     test_case.physical,
                                                     test_case.watchdog_latch_active);
    EXPECT_EQ(decision.action, test_case.action);
    EXPECT_EQ(decision.clear_heartbeat_only_latch, test_case.clear_latch);
  }
}
