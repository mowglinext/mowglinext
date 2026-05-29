// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_protocol.cpp
 * @brief Unit tests for the wire-format protocol structs and COBS roundtrip.
 *
 * Tests cover:
 *  - Packed struct sizes match expected wire sizes (compile-time + runtime)
 *  - Odometry packet field layout: encode, decode, verify field ordering
 *  - Roundtrip: fill struct -> COBS encode -> COBS decode -> verify CRC -> compare
 *  - Status, IMU, UI event, cmd_vel, heartbeat, hl_state packet roundtrips
 *  - PacketId enum values match PKT_ID_* defines (ll_datatypes.hpp vs mowgli_protocol.h)
 */

#include <cstring>
#include <vector>

#include "mowgli_hardware/crc16.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include <gtest/gtest.h>

using namespace mowgli_hardware;

// ---------------------------------------------------------------------------
// Size checks — ensure packed structs match expected wire sizes
// ---------------------------------------------------------------------------

TEST(ProtocolSizes, StatusPacketSize)
{
  EXPECT_EQ(sizeof(LlStatus), 38u);
}

TEST(ProtocolSizes, ImuPacketSize)
{
  EXPECT_EQ(sizeof(LlImu), 41u);
}

TEST(ProtocolSizes, UiEventPacketSize)
{
  EXPECT_EQ(sizeof(LlUiEvent), 5u);
}

TEST(ProtocolSizes, OdometryPacketSize)
{
  // 1 (type) + 2 (dt_millis) + 4 (left_ticks) + 4 (right_ticks)
  // + 2 (left_velocity_mm_s) + 2 (right_velocity_mm_s) + 2 (crc) = 17.
  // The ll_datatypes.hpp header has a matching static_assert; this test
  // guards the wire layout against future field additions.
  EXPECT_EQ(sizeof(LlOdometry), 17u);
}

TEST(ProtocolSizes, HeartbeatPacketSize)
{
  EXPECT_EQ(sizeof(LlHeartbeat), 5u);
}

TEST(ProtocolSizes, HighLevelStatePacketSize)
{
  EXPECT_EQ(sizeof(LlHighLevelState), 5u);
}

TEST(ProtocolSizes, CmdVelPacketSize)
{
  EXPECT_EQ(sizeof(LlCmdVel), 11u);
}

TEST(ProtocolSizes, RebootPacketSize)
{
  EXPECT_EQ(sizeof(LlReboot), 4u);  // type(1) + magic(1) + crc(2)
}

// ---------------------------------------------------------------------------
// Packet ID consistency (ll_datatypes.hpp enum matches mowgli_protocol.h)
// ---------------------------------------------------------------------------

TEST(ProtocolIds, PacketIdValues)
{
  EXPECT_EQ(PACKET_ID_LL_STATUS, 0x01);
  EXPECT_EQ(PACKET_ID_LL_IMU, 0x02);
  EXPECT_EQ(PACKET_ID_LL_UI_EVENT, 0x03);
  EXPECT_EQ(PACKET_ID_LL_ODOMETRY, 0x04);
  EXPECT_EQ(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ, 0x11);
  EXPECT_EQ(PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP, 0x12);
  EXPECT_EQ(PACKET_ID_LL_HEARTBEAT, 0x42);
  EXPECT_EQ(PACKET_ID_LL_HIGH_LEVEL_STATE, 0x43);
  EXPECT_EQ(PACKET_ID_LL_CMD_VEL, 0x50);
  EXPECT_EQ(PACKET_ID_LL_CMD_BLADE, 0x51);
  EXPECT_EQ(PACKET_ID_LL_REBOOT, 0x52);
}

// ---------------------------------------------------------------------------
// Odometry struct field layout verification
// ---------------------------------------------------------------------------

TEST(OdometryPacket, FieldOffsetsAreCorrect)
{
  LlOdometry pkt{};
  pkt.type = 0x04;
  pkt.dt_millis = 0x1234;
  pkt.left_ticks = 0x11223344;
  pkt.right_ticks = -42;
  pkt.left_velocity_mm_s = 250;  // signed — positive == forward
  pkt.right_velocity_mm_s = -120;  // signed — negative == reverse
  pkt.crc = 0xABCD;

  // Verify offsets by examining raw bytes
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);

  // type at offset 0
  EXPECT_EQ(raw[0], 0x04);

  // dt_millis at offset 1 (little-endian)
  uint16_t dt;
  std::memcpy(&dt, raw + 1, sizeof(dt));
  EXPECT_EQ(dt, 0x1234);

  // left_ticks at offset 3
  int32_t lt;
  std::memcpy(&lt, raw + 3, sizeof(lt));
  EXPECT_EQ(lt, 0x11223344);

  // right_ticks at offset 7
  int32_t rt;
  std::memcpy(&rt, raw + 7, sizeof(rt));
  EXPECT_EQ(rt, -42);

  // left_velocity_mm_s at offset 11
  int16_t lv;
  std::memcpy(&lv, raw + 11, sizeof(lv));
  EXPECT_EQ(lv, 250);

  // right_velocity_mm_s at offset 13
  int16_t rv;
  std::memcpy(&rv, raw + 13, sizeof(rv));
  EXPECT_EQ(rv, -120);

  // crc at offset 15
  uint16_t crc;
  std::memcpy(&crc, raw + 15, sizeof(crc));
  EXPECT_EQ(crc, 0xABCD);
}

// ---------------------------------------------------------------------------
// COBS roundtrip helpers
// ---------------------------------------------------------------------------

/// Helper: build a raw payload from a packed struct (without CRC),
/// encode with PacketHandler, decode by feeding back, verify content.
template <typename T>
static void roundtrip_struct(const T& pkt)
{
  PacketHandler handler;

  std::vector<uint8_t> received;
  handler.set_callback(
      [&](const uint8_t* data, std::size_t len)
      {
        received.assign(data, data + len);
      });

  // Build raw payload: struct bytes excluding the CRC field
  constexpr std::size_t payload_len = sizeof(T) - sizeof(uint16_t);
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);

  const auto frame = handler.encode_packet(raw, payload_len);
  handler.feed(frame.data(), frame.size());

  // received = payload + 2 CRC bytes
  ASSERT_EQ(received.size(), payload_len + 2u);

  // Verify payload content matches original struct bytes
  EXPECT_EQ(std::memcmp(received.data(), raw, payload_len), 0);

  // Verify CRC is valid
  EXPECT_TRUE(PacketHandler::verify_crc(received.data(), received.size()));
}

// ---------------------------------------------------------------------------
// Roundtrip tests for each packet type
// ---------------------------------------------------------------------------

TEST(ProtocolRoundtrip, OdometryPacket)
{
  LlOdometry pkt{};
  pkt.type = PACKET_ID_LL_ODOMETRY;
  pkt.dt_millis = 20;
  pkt.left_ticks = 15000;
  pkt.right_ticks = 14900;
  pkt.left_velocity_mm_s = 250;
  pkt.right_velocity_mm_s = 240;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, StatusPacket)
{
  LlStatus pkt{};
  pkt.type = PACKET_ID_LL_STATUS;
  pkt.status_bitmask = STATUS_BIT_INITIALIZED | STATUS_BIT_RASPI_POWER;
  pkt.uss_ranges_m[0] = 1.5f;
  pkt.uss_ranges_m[1] = 2.0f;
  pkt.uss_ranges_m[2] = 0.0f;
  pkt.uss_ranges_m[3] = 0.0f;
  pkt.uss_ranges_m[4] = 0.0f;
  pkt.emergency_bitmask = 0;
  pkt.v_charge = 28.5f;
  pkt.v_system = 25.2f;
  pkt.charging_current = 1.5f;
  pkt.batt_percentage = 75;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, ImuPacket)
{
  LlImu pkt{};
  pkt.type = PACKET_ID_LL_IMU;
  pkt.dt_millis = 20;
  pkt.acceleration_mss[0] = 0.1f;
  pkt.acceleration_mss[1] = 0.2f;
  pkt.acceleration_mss[2] = 9.81f;
  pkt.gyro_rads[0] = 0.01f;
  pkt.gyro_rads[1] = -0.02f;
  pkt.gyro_rads[2] = 0.005f;
  pkt.mag_uT[0] = 30.0f;
  pkt.mag_uT[1] = -15.0f;
  pkt.mag_uT[2] = 45.0f;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, UiEventPacket)
{
  LlUiEvent pkt{};
  pkt.type = PACKET_ID_LL_UI_EVENT;
  pkt.button_id = 3;
  pkt.press_duration = 1;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, HeartbeatPacket)
{
  LlHeartbeat pkt{};
  pkt.type = PACKET_ID_LL_HEARTBEAT;
  pkt.emergency_requested = 0;
  pkt.emergency_release_requested = 1;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, HighLevelStatePacket)
{
  LlHighLevelState pkt{};
  pkt.type = PACKET_ID_LL_HIGH_LEVEL_STATE;
  pkt.current_mode = 1;
  pkt.gps_quality = 95;

  roundtrip_struct(pkt);
}

TEST(ProtocolRoundtrip, CmdVelPacket)
{
  LlCmdVel pkt{};
  pkt.type = PACKET_ID_LL_CMD_VEL;
  pkt.linear_x = 0.35f;
  pkt.angular_z = -0.15f;

  roundtrip_struct(pkt);
}

// ---------------------------------------------------------------------------
// Odometry: direction now encoded in the sign of the signed fields. Positive
// ticks/velocity = forward, negative = reverse, zero = stopped. Verifies the
// polymorphism works (signed arithmetic, no dedicated direction byte).
// ---------------------------------------------------------------------------

TEST(OdometryPacket, SignedDirectionEncoding)
{
  LlOdometry pkt{};
  pkt.type = PACKET_ID_LL_ODOMETRY;

  // Forward
  pkt.left_ticks = 100;
  pkt.right_ticks = 100;
  pkt.left_velocity_mm_s = 200;
  pkt.right_velocity_mm_s = 200;
  EXPECT_GT(pkt.left_ticks, 0);
  EXPECT_GT(pkt.left_velocity_mm_s, 0);

  // Reverse
  pkt.left_ticks = -100;
  pkt.right_ticks = -100;
  pkt.left_velocity_mm_s = -200;
  pkt.right_velocity_mm_s = -200;
  EXPECT_LT(pkt.left_ticks, 0);
  EXPECT_LT(pkt.left_velocity_mm_s, 0);

  // Stopped — ticks may be nonzero cumulative, but velocity must be zero
  pkt.left_velocity_mm_s = 0;
  pkt.right_velocity_mm_s = 0;
  EXPECT_EQ(pkt.left_velocity_mm_s, 0);
  EXPECT_EQ(pkt.right_velocity_mm_s, 0);
}

// ---------------------------------------------------------------------------
// Odometry: negative ticks (reverse direction)
// ---------------------------------------------------------------------------

TEST(OdometryPacket, NegativeTicksRoundtrip)
{
  LlOdometry pkt{};
  pkt.type = PACKET_ID_LL_ODOMETRY;
  pkt.dt_millis = 20;
  pkt.left_ticks = -5000;
  pkt.right_ticks = -5100;
  pkt.left_velocity_mm_s = -180;
  pkt.right_velocity_mm_s = -170;

  roundtrip_struct(pkt);
}

// ---------------------------------------------------------------------------
// Zero-value edge cases
// ---------------------------------------------------------------------------

TEST(OdometryPacket, ZeroValuesRoundtrip)
{
  LlOdometry pkt{};
  pkt.type = PACKET_ID_LL_ODOMETRY;
  // All other fields default to 0

  roundtrip_struct(pkt);
}

// ---------------------------------------------------------------------------
// Status bitmask constants
// ---------------------------------------------------------------------------

TEST(StatusBitmask, BitPositions)
{
  EXPECT_EQ(STATUS_BIT_INITIALIZED, 0x01);
  EXPECT_EQ(STATUS_BIT_RASPI_POWER, 0x02);
  EXPECT_EQ(STATUS_BIT_CHARGING, 0x04);
  EXPECT_EQ(STATUS_BIT_RAIN, 0x10);
  EXPECT_EQ(STATUS_BIT_SOUND_AVAIL, 0x20);
  EXPECT_EQ(STATUS_BIT_SOUND_BUSY, 0x40);
  EXPECT_EQ(STATUS_BIT_UI_AVAIL, 0x80);
}

TEST(EmergencyBitmask, BitPositions)
{
  EXPECT_EQ(EMERGENCY_BIT_LATCH, 0x01);
  EXPECT_EQ(EMERGENCY_BIT_STOP, 0x02);
  EXPECT_EQ(EMERGENCY_BIT_LIFT, 0x04);
}
