// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_hardware/firmware_update_gate.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{
constexpr std::uint8_t kHeartbeat = 0x42u;
constexpr std::uint8_t kHighLevel = 0x43u;
constexpr std::uint8_t kCmdVel = 0x50u;
constexpr std::uint8_t kCmdBlade = 0x51u;
constexpr std::uint8_t kConfigReq = 0x11u;
constexpr std::uint8_t kEnterDfu = 0x53u;
}  // namespace

TEST(FirmwareUpdateGate, RequiresConnectedCompatibleCapableIdleFirmware)
{
  EXPECT_TRUE(mh::firmware_update_can_begin(true, true, true, false, false, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(false, true, true, false, false, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, false, true, false, false, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, true, false, false, false, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, true, true, true, false, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, true, true, false, true, 1u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, true, true, false, false, 2u, 1u));
  EXPECT_FALSE(mh::firmware_update_can_begin(true, true, true, false, false, 0u, 1u));
}

TEST(FirmwareUpdateGate, AllowsOnlySafeMaintenancePacketClasses)
{
  for (const auto packet : {kHeartbeat, kHighLevel, kCmdVel, kCmdBlade, kConfigReq, kEnterDfu})
  {
    EXPECT_TRUE(mh::firmware_update_packet_allowed(
        packet, kHeartbeat, kHighLevel, kCmdVel, kCmdBlade, kConfigReq, kEnterDfu));
  }
  for (const auto packet : {0x54u, 0x55u, 0x56u, 0x57u, 0x99u})
  {
    EXPECT_FALSE(mh::firmware_update_packet_allowed(
        packet, kHeartbeat, kHighLevel, kCmdVel, kCmdBlade, kConfigReq, kEnterDfu));
  }
}

TEST(FirmwareUpdateGate, NeverReleasesEmergencyAndAlwaysReportsIdleDuringMaintenance)
{
  EXPECT_FALSE(mh::firmware_update_may_release_emergency(true, true));
  EXPECT_FALSE(mh::firmware_update_may_release_emergency(true, false));
  EXPECT_TRUE(mh::firmware_update_may_release_emergency(false, true));
  EXPECT_EQ(mh::firmware_update_high_level_mode(true, 2u, 1u), 1u);
  EXPECT_EQ(mh::firmware_update_high_level_mode(false, 2u, 1u), 2u);
  EXPECT_DOUBLE_EQ(mh::firmware_update_motion_component(true, -0.25), 0.0);
  EXPECT_DOUBLE_EQ(mh::firmware_update_motion_component(false, -0.25), -0.25);
}
