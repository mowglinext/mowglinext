// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_hardware/firmware_interlock.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::firmware_actuation_allowed;
using mowgli_hardware::interlock_blade_on;
using mowgli_hardware::interlock_high_level_mode;
using mowgli_hardware::interlock_refuses_blade;
using mowgli_hardware::interlock_refuses_mode;
using mowgli_hardware::interlock_refuses_velocity;
using mowgli_hardware::interlock_transition;
using mowgli_hardware::interlock_velocity;
using mowgli_hardware::InterlockTransition;
using mowgli_hardware::kInterlockHighLevelMode;

TEST(FirmwareInterlock, OnlyACompletedMatchingHandshakeAllowsActuation)
{
  EXPECT_FALSE(firmware_actuation_allowed(false, false));  // no handshake yet
  EXPECT_FALSE(firmware_actuation_allowed(true, false));  // wrong protocol / timed out
  EXPECT_FALSE(firmware_actuation_allowed(false, true));  // inconsistent: never trust
  EXPECT_TRUE(firmware_actuation_allowed(true, true));
}

TEST(FirmwareInterlock, VelocityIsZeroedWhileDeniedAndUntouchedWhileAllowed)
{
  const auto denied = interlock_velocity(false, 0.4, -1.2);
  EXPECT_EQ(denied.vx, 0.0);
  EXPECT_EQ(denied.wz, 0.0);
  const auto allowed = interlock_velocity(true, 0.4, -1.2);
  EXPECT_EQ(allowed.vx, 0.4);
  EXPECT_EQ(allowed.wz, -1.2);
}

TEST(FirmwareInterlock, BladeOnIsForcedOffButOffAlwaysPasses)
{
  EXPECT_EQ(interlock_blade_on(false, 1u), 0u);
  EXPECT_EQ(interlock_blade_on(false, 0u), 0u);
  EXPECT_EQ(interlock_blade_on(true, 1u), 1u);
  EXPECT_EQ(interlock_blade_on(true, 0u), 0u);
}

TEST(FirmwareInterlock, EveryModeIsHeldIdleWhileDenied)
{
  for (uint8_t mode = 0u; mode <= 4u; ++mode)
  {
    EXPECT_EQ(interlock_high_level_mode(false, mode), kInterlockHighLevelMode);
    EXPECT_EQ(interlock_high_level_mode(true, mode), mode);
  }
  EXPECT_EQ(kInterlockHighLevelMode, 1u);  // HL_MODE_IDLE
}

TEST(FirmwareInterlock, OnlyRealRefusalsAreReported)
{
  EXPECT_TRUE(interlock_refuses_velocity(false, 0.1, 0.0));
  EXPECT_TRUE(interlock_refuses_velocity(false, 0.0, -0.1));
  EXPECT_FALSE(interlock_refuses_velocity(false, 0.0, 0.0));
  EXPECT_FALSE(interlock_refuses_velocity(true, 0.1, 0.1));

  EXPECT_TRUE(interlock_refuses_blade(false, 1u));
  EXPECT_FALSE(interlock_refuses_blade(false, 0u));
  EXPECT_FALSE(interlock_refuses_blade(true, 1u));

  EXPECT_TRUE(interlock_refuses_mode(false, 2u));  // AUTONOMOUS
  EXPECT_TRUE(interlock_refuses_mode(false, 3u));  // RECORDING
  EXPECT_TRUE(interlock_refuses_mode(false, 4u));  // MANUAL_MOWING
  EXPECT_FALSE(interlock_refuses_mode(false, 1u));  // IDLE
  EXPECT_FALSE(interlock_refuses_mode(false, 0u));  // NULL is also a stop
  EXPECT_FALSE(interlock_refuses_mode(true, 2u));
}

TEST(FirmwareInterlock, TransitionsAreClassified)
{
  EXPECT_EQ(interlock_transition(true, true), InterlockTransition::kNone);
  EXPECT_EQ(interlock_transition(false, false), InterlockTransition::kNone);
  EXPECT_EQ(interlock_transition(true, false), InterlockTransition::kEngaged);
  EXPECT_EQ(interlock_transition(false, true), InterlockTransition::kReleased);
}
