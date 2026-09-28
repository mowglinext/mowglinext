// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "emergency_clear_policy.h"
#include <gtest/gtest.h>

TEST(EmergencyClearPolicy, ClearInputsPermitManualReset)
{
  const EmergencyPhysicalInputs inputs{};
  EXPECT_TRUE(emergency_physical_inputs_clear(inputs));
}

TEST(EmergencyClearPolicy, EveryPhysicalInputIndividuallyBlocksReset)
{
  const EmergencyPhysicalInputs cases[] = {
      {1, 0, 0, 0, 0, 0},  // yellow STOP
      {0, 1, 0, 0, 0, 0},  // white STOP
      {0, 0, 1, 0, 0, 0},  // blue wheel lift
      {0, 0, 0, 1, 0, 0},  // red wheel lift
      {0, 0, 0, 0, 1, 0},  // mechanical tilt
      {0, 0, 0, 0, 0, 1},  // accelerometer tilt
  };

  for (const auto& inputs : cases)
  {
    EXPECT_FALSE(emergency_physical_inputs_clear(inputs));
  }
}

TEST(EmergencyClearPolicy, CombinedInputsBlockReset)
{
  const EmergencyPhysicalInputs inputs{1, 0, 0, 1, 1, 0};
  EXPECT_FALSE(emergency_physical_inputs_clear(inputs));
}

TEST(EmergencyClearPolicy, HazardMustClearBeforeLaterManualReset)
{
  EmergencyPhysicalInputs inputs{};
  uint32_t hold_started = 0;

  EXPECT_FALSE(emergency_play_clear_hold_step(100, 2000, true, true, inputs, &hold_started));
  EXPECT_EQ(hold_started, 100u);

  // A STOP input asserted during the hold resets accumulated time. Clearing
  // it while PLAY remains held starts a fresh full interval.
  inputs.stop_white = 1;
  EXPECT_FALSE(emergency_play_clear_hold_step(1100, 2000, true, true, inputs, &hold_started));
  EXPECT_EQ(hold_started, 0u);
  inputs.stop_white = 0;
  EXPECT_FALSE(emergency_play_clear_hold_step(1200, 2000, true, true, inputs, &hold_started));
  EXPECT_EQ(hold_started, 1200u);
  EXPECT_FALSE(emergency_play_clear_hold_step(3199, 2000, true, true, inputs, &hold_started));
  EXPECT_TRUE(emergency_play_clear_hold_step(3200, 2000, true, true, inputs, &hold_started));
}

TEST(EmergencyClearPolicy, ActiveHazardAtHoldExpiryCannotClearLatch)
{
  EmergencyPhysicalInputs inputs{};
  uint32_t hold_started = 0;
  EXPECT_FALSE(emergency_play_clear_hold_step(100, 2000, true, true, inputs, &hold_started));
  inputs.wheel_lift_blue = 1;
  EXPECT_FALSE(emergency_play_clear_hold_step(2100, 2000, true, true, inputs, &hold_started));
  EXPECT_EQ(hold_started, 0u);
  EXPECT_FALSE(emergency_play_clear_hold_step(3000, 2000, true, true, inputs, &hold_started));
}
