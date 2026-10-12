// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_emergency_tracker.cpp
 * @brief Host <-> LowLevel emergency handshake against the v1 firmware's
 *        latch semantics (OpenMower branch v1-fw, updateEmergency + the
 *        heartbeat handler): a release bit clears the latch unconditionally,
 *        and the next loop re-latches if a trigger is still asserted.
 */

#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_openmower_bridge/emergency_tracker.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::EMERGENCY_BIT_LATCH;
using mowgli_hardware::EMERGENCY_BIT_LIFT;
using mowgli_hardware::EMERGENCY_BIT_STOP;
using mowgli_openmower_bridge::EmergencyTracker;

namespace
{
constexpr uint8_t kLatched = EMERGENCY_BIT_LATCH;
constexpr uint8_t kStopHeld = EMERGENCY_BIT_LATCH | EMERGENCY_BIT_STOP;
}  // namespace

TEST(EmergencyTracker, QuietBoardIsNotAnEmergency)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(0u);
  EXPECT_FALSE(ev.active);
  EXPECT_FALSE(ev.latched);
  EXPECT_FALSE(t.is_emergency());
  EXPECT_TRUE(ev.reason.empty());
  const auto hb = t.NextHeartbeat();
  EXPECT_FALSE(hb.request);
  EXPECT_FALSE(hb.release);
}

TEST(EmergencyTracker, StopButtonIsActiveAndLatched)
{
  EmergencyTracker t;
  const auto ev = t.Evaluate(kStopHeld);
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

TEST(EmergencyTracker, BootLatchWithoutTriggerIsLatchedButNotActive)
{
  // The v1 board boots with emergency_latch = true.
  EmergencyTracker t;
  const auto ev = t.Evaluate(kLatched);
  EXPECT_FALSE(ev.active);
  EXPECT_TRUE(ev.latched);
  EXPECT_EQ(ev.reason, "Latched (press play button to release)");
  EXPECT_TRUE(t.is_emergency());
}

TEST(EmergencyTracker, HostRequestIsReassertedOnEveryHeartbeat)
{
  EmergencyTracker t;
  t.RequestEmergency();
  EXPECT_TRUE(t.Evaluate(0u).latched);
  EXPECT_EQ(t.Evaluate(0u).reason, "Software emergency stop");
  for (int i = 0; i < 10; ++i)
  {
    const auto hb = t.NextHeartbeat();
    EXPECT_TRUE(hb.request);
    EXPECT_FALSE(hb.release);
  }
  (void)t.Evaluate(kLatched);  // the board took it — keep asserting anyway
  EXPECT_TRUE(t.NextHeartbeat().request);
}

TEST(EmergencyTracker, ReleaseIsSentForABoundedNumberOfHeartbeats)
{
  EmergencyTracker t;
  (void)t.Evaluate(kLatched);
  t.RequestRelease();
  for (int i = 0; i < EmergencyTracker::kReleaseHeartbeats; ++i)
  {
    EXPECT_TRUE(t.NextHeartbeat().release) << "heartbeat " << i;
  }
  EXPECT_FALSE(t.NextHeartbeat().release);
  EXPECT_FALSE(t.NextHeartbeat().request);
}

TEST(EmergencyTracker, ReleaseStopsOnceTheBoardClears)
{
  EmergencyTracker t;
  (void)t.Evaluate(kLatched);
  t.RequestRelease();
  EXPECT_TRUE(t.NextHeartbeat().release);
  const auto ev = t.Evaluate(0u);
  EXPECT_FALSE(ev.latched);
  EXPECT_FALSE(t.is_emergency());
  EXPECT_FALSE(t.NextHeartbeat().release);
}

// The regression: a reset pressed while the stop button is still held must
// NOT stay armed and fire on its own when the button is let go.
TEST(EmergencyTracker, ReleaseIsRefusedWhileATriggerIsAsserted)
{
  EmergencyTracker t;
  (void)t.Evaluate(kStopHeld);
  t.RequestRelease();
  EXPECT_TRUE(t.NextHeartbeat().release);  // the board clears, then re-latches
  (void)t.Evaluate(kStopHeld);  // ...and reports the trigger
  EXPECT_FALSE(t.NextHeartbeat().release);

  // Button released: the board is still latched and no release is pending.
  const auto ev = t.Evaluate(kLatched);
  EXPECT_TRUE(ev.latched);
  EXPECT_FALSE(ev.active);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_FALSE(t.NextHeartbeat().release);
  }
  EXPECT_TRUE(t.is_emergency());

  // Only a fresh request clears it.
  t.RequestRelease();
  EXPECT_TRUE(t.NextHeartbeat().release);
  EXPECT_FALSE(t.Evaluate(0u).latched);
  EXPECT_FALSE(t.is_emergency());
}

TEST(EmergencyTracker, EmergencyRequestCancelsAPendingRelease)
{
  EmergencyTracker t;
  (void)t.Evaluate(kLatched);
  t.RequestRelease();
  t.RequestEmergency();
  const auto hb = t.NextHeartbeat();
  EXPECT_TRUE(hb.request);
  EXPECT_FALSE(hb.release);
}

TEST(EmergencyTracker, ReleaseCannotClearAPhysicalTrigger)
{
  EmergencyTracker t;
  t.RequestRelease();
  const auto ev = t.Evaluate(kStopHeld);
  EXPECT_TRUE(ev.active);
  EXPECT_TRUE(ev.latched);
  EXPECT_TRUE(t.is_emergency());
}
