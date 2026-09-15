// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_emergency_tracker.cpp
 * @brief Host ↔ LowLevel emergency handshake, mirroring mower_comms_v1.
 */

#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_openmower_bridge/emergency_tracker.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::EMERGENCY_BIT_LATCH;
using mowgli_hardware::EMERGENCY_BIT_LIFT;
using mowgli_hardware::EMERGENCY_BIT_STOP;
using mowgli_openmower_bridge::EmergencyTracker;

TEST(EmergencyTracker, QuietBoardIsNotAnEmergency)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(0u);
  EXPECT_FALSE(ev.active);
  EXPECT_FALSE(ev.latched);
  EXPECT_FALSE(ev.heartbeat_request);
  EXPECT_FALSE(ev.heartbeat_release);
  EXPECT_FALSE(t.is_emergency());
  EXPECT_TRUE(ev.reason.empty());
}

TEST(EmergencyTracker, StopButtonIsActiveAndLatched)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(EMERGENCY_BIT_LATCH | EMERGENCY_BIT_STOP);
  EXPECT_TRUE(ev.active);
  EXPECT_TRUE(ev.latched);
  EXPECT_EQ(ev.reason, "STOP button");
  EXPECT_TRUE(t.is_emergency());
}

TEST(EmergencyTracker, LiftOnlyReportsLift)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(EMERGENCY_BIT_LATCH | EMERGENCY_BIT_LIFT);
  EXPECT_TRUE(ev.active);
  EXPECT_EQ(ev.reason, "Lift detected");
}

TEST(EmergencyTracker, LatchWithoutTriggerIsLatchedButNotActive)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(EMERGENCY_BIT_LATCH);
  EXPECT_FALSE(ev.active);
  EXPECT_TRUE(ev.latched);
  EXPECT_EQ(ev.reason, "Latched (press play button to release)");
}

TEST(EmergencyTracker, HostRequestAsksTheBoardUntilItLatches)
{
  EmergencyTracker t;
  t.RequestEmergency();
  auto ev = t.Evaluate(0u);
  EXPECT_TRUE(ev.heartbeat_request);
  EXPECT_TRUE(ev.latched);
  EXPECT_EQ(ev.reason, "Software emergency stop");
  EXPECT_TRUE(t.is_emergency());

  ev = t.Evaluate(EMERGENCY_BIT_LATCH);  // the board took it
  EXPECT_FALSE(ev.heartbeat_request);
  EXPECT_TRUE(ev.latched);
}

TEST(EmergencyTracker, ReleaseKeepsAskingUntilTheBoardClears)
{
  EmergencyTracker t;
  t.RequestEmergency();
  (void)t.Evaluate(EMERGENCY_BIT_LATCH);
  t.RequestRelease();
  auto ev = t.Evaluate(EMERGENCY_BIT_LATCH);
  EXPECT_TRUE(ev.heartbeat_release);
  EXPECT_FALSE(ev.heartbeat_request);
  EXPECT_TRUE(t.is_emergency());  // still latched on the board

  ev = t.Evaluate(0u);
  EXPECT_FALSE(ev.heartbeat_release);
  EXPECT_FALSE(ev.latched);
  EXPECT_FALSE(t.is_emergency());
}

TEST(EmergencyTracker, ReleaseCannotClearAPhysicalTrigger)
{
  EmergencyTracker t;
  t.RequestRelease();
  const auto ev = t.Evaluate(EMERGENCY_BIT_LATCH | EMERGENCY_BIT_STOP);
  EXPECT_TRUE(ev.active);
  EXPECT_TRUE(ev.latched);
  EXPECT_TRUE(t.is_emergency());
}
