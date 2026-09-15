// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_xesc_2040_protocol.cpp
 * @brief Pins the xESC 2040 packed layouts and the link's status decoding
 *        (signed tick accumulation from tacho_absolute + direction).
 */

#include <chrono>
#include <cstddef>
#include <string>

#include "mowgli_openmower_bridge/xesc_2040_link.hpp"
#include "mowgli_openmower_bridge/xesc_2040_protocol.hpp"
#include <gtest/gtest.h>

namespace x = mowgli_openmower_bridge::xesc2040;
using mowgli_openmower_bridge::SteadyClock;
using mowgli_openmower_bridge::Xesc2040Link;
using mowgli_openmower_bridge::Xesc2040Settings;

TEST(Xesc2040Protocol, PacketSizesAndOffsets)
{
  EXPECT_EQ(sizeof(x::StatusPacket), 62u);
  EXPECT_EQ(sizeof(x::ControlPacket), 11u);
  EXPECT_EQ(sizeof(x::SettingsPacket), 36u);
  EXPECT_EQ(offsetof(x::StatusPacket, seq), 1u);
  EXPECT_EQ(offsetof(x::StatusPacket, voltage_input), 7u);
  EXPECT_EQ(offsetof(x::StatusPacket, tacho_absolute), 51u);
  EXPECT_EQ(offsetof(x::StatusPacket, direction), 55u);
  EXPECT_EQ(offsetof(x::StatusPacket, fault_code), 56u);
  EXPECT_EQ(offsetof(x::ControlPacket, duty_cycle), 1u);
  EXPECT_EQ(offsetof(x::SettingsPacket, motor_current_limit), 9u);
}

TEST(Xesc2040Link, AccumulatesSignedTicksFromDirection)
{
  Xesc2040Link link("/nonexistent/port", Xesc2040Settings{}, [](const std::string&) {});
  const auto t0 = SteadyClock::now();

  x::StatusPacket pkt{};
  pkt.message_type = x::kMsgTypeStatus;
  pkt.tacho_absolute = 100u;
  pkt.direction = false;
  link.HandleStatus(pkt, t0);  // primes
  EXPECT_EQ(link.telemetry().signed_ticks, 0);

  pkt.tacho_absolute = 130u;  // +30 forward
  link.HandleStatus(pkt, t0);
  EXPECT_EQ(link.telemetry().signed_ticks, 30);

  pkt.tacho_absolute = 150u;  // +20 while reversing
  pkt.direction = true;
  link.HandleStatus(pkt, t0);
  EXPECT_EQ(link.telemetry().signed_ticks, 10);
  EXPECT_TRUE(link.telemetry().has_status);
  EXPECT_TRUE(link.telemetry().connected);
}

TEST(Xesc2040Link, SurvivesTachoWraparound)
{
  Xesc2040Link link("/nonexistent/port", Xesc2040Settings{}, [](const std::string&) {});
  const auto t0 = SteadyClock::now();
  x::StatusPacket pkt{};
  pkt.message_type = x::kMsgTypeStatus;
  pkt.tacho_absolute = 0xFFFFFFF0u;
  link.HandleStatus(pkt, t0);
  pkt.tacho_absolute = 0x00000010u;  // wrapped: +32
  link.HandleStatus(pkt, t0);
  EXPECT_EQ(link.telemetry().signed_ticks, 32);
}

TEST(Xesc2040Link, CopiesTelemetryFields)
{
  Xesc2040Link link("/nonexistent/port", Xesc2040Settings{}, [](const std::string&) {});
  x::StatusPacket pkt{};
  pkt.message_type = x::kMsgTypeStatus;
  pkt.fw_version_major = 1u;
  pkt.fw_version_minor = 4u;
  pkt.voltage_input = 27.3;
  pkt.temperature_pcb = 41.0;
  pkt.temperature_motor = 39.5;
  pkt.current_input = 1.25;
  pkt.duty_cycle = -0.4;
  pkt.fault_code = static_cast<int32_t>(x::kFaultOvercurrent);
  link.HandleStatus(pkt, SteadyClock::now());
  const auto& t = link.telemetry();
  EXPECT_EQ(t.fw_major, 1u);
  EXPECT_EQ(t.fw_minor, 4u);
  EXPECT_DOUBLE_EQ(t.voltage_in, 27.3);
  EXPECT_DOUBLE_EQ(t.temp_pcb, 41.0);
  EXPECT_DOUBLE_EQ(t.temp_motor, 39.5);
  EXPECT_DOUBLE_EQ(t.current_in, 1.25);
  EXPECT_DOUBLE_EQ(t.duty, -0.4);
  EXPECT_EQ(t.fault_code, x::kFaultOvercurrent);
  EXPECT_DOUBLE_EQ(t.rpm, 0.0);
}

TEST(Xesc2040Link, SendDutyWithoutPortFailsCleanly)
{
  Xesc2040Link link("/nonexistent/port", Xesc2040Settings{}, [](const std::string&) {});
  EXPECT_FALSE(link.SendDuty(0.3));
  link.Poll(SteadyClock::now());
  EXPECT_FALSE(link.telemetry().connected);
}
