// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_lowlevel_config.cpp
 * @brief The flexible-length LowLevel config packet and the hall string.
 */

#include <cstring>
#include <vector>

#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_openmower_bridge/lowlevel_config.hpp"
#include <gtest/gtest.h>

namespace ll = mowgli_openmower_bridge::lowlevel;

TEST(LowLevelConfig, WireSizeMatchesOpenMower)
{
  EXPECT_EQ(sizeof(ll::ConfigOptions), 2u);
  EXPECT_EQ(sizeof(ll::HallConfig), 1u);
  EXPECT_EQ(sizeof(ll::HighLevelConfig), 42u);
}

TEST(LowLevelConfig, DefaultsAreUnknownSoTheBoardKeepsItsOwn)
{
  const auto cfg = ll::DefaultConfig();
  EXPECT_FLOAT_EQ(cfg.v_charge_cutoff, -1.0f);
  EXPECT_EQ(cfg.lift_period, 0xFFFFu);
  EXPECT_EQ(cfg.volume, 0xFFu);
  for (const auto& hall : cfg.hall_configs)
  {
    EXPECT_EQ(hall.mode, ll::HallMode::UNDEFINED);
    EXPECT_FALSE(hall.active_low);
  }
}

TEST(LowLevelConfig, HallStringParsesModesAndActiveLow)
{
  auto cfg = ll::DefaultConfig();
  ll::ApplyHallConfigString(cfg, "!L,l,S,i,u");
  EXPECT_EQ(cfg.hall_configs[0].mode, ll::HallMode::LIFT_TILT);
  EXPECT_TRUE(cfg.hall_configs[0].active_low);
  EXPECT_EQ(cfg.hall_configs[1].mode, ll::HallMode::LIFT_TILT);
  EXPECT_FALSE(cfg.hall_configs[1].active_low);
  EXPECT_EQ(cfg.hall_configs[2].mode, ll::HallMode::STOP);
  EXPECT_EQ(cfg.hall_configs[3].mode, ll::HallMode::OFF);
  EXPECT_EQ(cfg.hall_configs[4].mode, ll::HallMode::UNDEFINED);
  EXPECT_EQ(cfg.hall_configs[5].mode, ll::HallMode::UNDEFINED);  // untouched
}

TEST(LowLevelConfig, HallStringIgnoresExtraEntries)
{
  auto cfg = ll::DefaultConfig();
  ll::ApplyHallConfigString(cfg, "S,S,S,S,S,S,S,S,S,S,S,S");
  for (const auto& hall : cfg.hall_configs)
  {
    EXPECT_EQ(hall.mode, ll::HallMode::STOP);
  }
}

TEST(LowLevelConfig, PayloadRoundTrips)
{
  auto cfg = ll::DefaultConfig();
  cfg.v_charge_cutoff = 29.4f;
  cfg.lift_period = 1000u;
  cfg.language[0] = 'd';
  cfg.language[1] = 'e';
  const auto payload =
      ll::BuildConfigPayload(cfg, mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
  ASSERT_EQ(payload.size(), 1u + sizeof(ll::HighLevelConfig));
  EXPECT_EQ(payload[0], mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);

  // Receiver sees payload + 2 CRC bytes.
  std::vector<uint8_t> wire = payload;
  wire.push_back(0u);
  wire.push_back(0u);
  const auto back = ll::ParseConfigPayload(wire.data(), wire.size(), ll::DefaultConfig());
  EXPECT_FLOAT_EQ(back.v_charge_cutoff, 29.4f);
  EXPECT_EQ(back.lift_period, 1000u);
  EXPECT_EQ(back.language[0], 'd');
  EXPECT_EQ(back.language[1], 'e');
}

TEST(LowLevelConfig, ShorterSenderKeepsDefaultsForMissingTail)
{
  auto sent = ll::DefaultConfig();
  sent.v_charge_cutoff = 28.0f;
  auto payload = ll::BuildConfigPayload(sent, mowgli_hardware::PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP);
  payload.resize(1u + 8u);  // an older board that only knows the first fields
  payload.push_back(0u);
  payload.push_back(0u);
  auto defaults = ll::DefaultConfig();
  defaults.lift_period = 777u;
  const auto back = ll::ParseConfigPayload(payload.data(), payload.size(), defaults);
  EXPECT_FLOAT_EQ(back.v_charge_cutoff, 28.0f);
  EXPECT_EQ(back.lift_period, 777u);
}

TEST(LowLevelConfig, TooShortPayloadReturnsDefaults)
{
  const uint8_t junk[2] = {0x12u, 0x00u};
  auto defaults = ll::DefaultConfig();
  defaults.volume = 42u;
  const auto back = ll::ParseConfigPayload(junk, sizeof(junk), defaults);
  EXPECT_EQ(back.volume, 42u);
}
