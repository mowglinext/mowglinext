// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "blade_emergency_policy.hpp"
#include <gtest/gtest.h>

TEST(BladeEmergencyPolicy, HeartbeatEmergencyDiscardsPriorOnRequest)
{
  auto decision = decide_blade_intent(0, 0, true, 1, 0, false, false, 0);
  ASSERT_EQ(decision.retained_request, 1);
  ASSERT_EQ(decision.effective_output, 1);

  decision = decide_blade_intent(
      decision.retained_request, decision.request_generation, false, 0, 0, false, true, 1);
  EXPECT_EQ(decision.retained_request, 0);
  EXPECT_EQ(decision.effective_output, 0);
}

TEST(BladeEmergencyPolicy, HeartbeatRecoveryAloneCannotRestoreOldRequest)
{
  auto decision = decide_blade_intent(1, 0, false, 0, 0, false, true, 1);
  ASSERT_EQ(decision.retained_request, 0);

  decision = decide_blade_intent(
      decision.retained_request, decision.request_generation, false, 0, 0, false, false, 1);
  EXPECT_EQ(decision.retained_request, 0);
  EXPECT_EQ(decision.effective_output, 0);
}

TEST(BladeEmergencyPolicy, FreshEnableAfterRecoveryIsAccepted)
{
  const auto recovery = decide_blade_intent(0, 1, false, 0, 0, false, false, 1);
  const auto fresh_enable = decide_blade_intent(
      recovery.retained_request, recovery.request_generation, true, 1, 1, false, false, 1);
  EXPECT_EQ(fresh_enable.retained_request, 1);
  EXPECT_EQ(fresh_enable.effective_output, 1);
}

TEST(BladeEmergencyPolicy, EnableCommandDuringEmergencyIsDiscarded)
{
  const auto during_emergency = decide_blade_intent(0, 1, true, 1, 1, false, true, 1);
  EXPECT_EQ(during_emergency.retained_request, 0);
  EXPECT_EQ(during_emergency.effective_output, 0);

  const auto recovery = decide_blade_intent(during_emergency.retained_request,
                                            during_emergency.request_generation,
                                            false,
                                            0,
                                            0,
                                            false,
                                            false,
                                            1);
  EXPECT_EQ(recovery.retained_request, 0);
  EXPECT_EQ(recovery.effective_output, 0);
}

TEST(BladeEmergencyPolicy, OffCommandAlwaysLeavesIntentAndOutputOff)
{
  const auto decision = decide_blade_intent(1, 0, true, 0, 0, false, false, 0);
  EXPECT_EQ(decision.retained_request, 0);
  EXPECT_EQ(decision.effective_output, 0);
}

TEST(BladeEmergencyPolicy, PhysicalEmergencyAlsoDiscardsOldIntent)
{
  const auto decision = decide_blade_intent(1, 0, false, 0, 0, false, true, 1);
  EXPECT_EQ(decision.retained_request, 0);
  EXPECT_EQ(decision.effective_output, 0);
}

TEST(BladeEmergencyPolicy, IdleGateDiscardsIntentAndDoesNotRearmOnExit)
{
  const auto idle = decide_blade_intent(0, 0, true, 1, 0, true, false, 0);
  EXPECT_EQ(idle.retained_request, 0);
  EXPECT_EQ(idle.effective_output, 0);

  const auto mowing = decide_blade_intent(
      idle.retained_request, idle.request_generation, false, 0, 0, false, false, 0);
  EXPECT_EQ(mowing.retained_request, 0);
  EXPECT_EQ(mowing.effective_output, 0);
}

TEST(BladeEmergencyPolicy, BriefEmergencyBetweenMotorTicksInvalidatesIntent)
{
  const auto blade_on = decide_blade_intent(0, 0, true, 1, 0, false, false, 0);
  ASSERT_EQ(blade_on.retained_request, 1);

  // The emergency asserted and cleared between motor updates. The current
  // state is clear, but the emergency generation changed from 0 to 1.
  const auto after_brief_emergency = decide_blade_intent(
      blade_on.retained_request, blade_on.request_generation, false, 0, 0, false, false, 1);
  EXPECT_EQ(after_brief_emergency.retained_request, 0);
  EXPECT_EQ(after_brief_emergency.effective_output, 0);
}
