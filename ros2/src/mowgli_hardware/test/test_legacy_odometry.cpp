// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Previous-protocol (v8) odometry decoding — legacy_odometry.hpp.
// Field 2026-10-25: a v9 host dropped every 17-byte v8 packet, /wheel_odom
// went silent, and the update-first install rolled back on stale wheel
// telemetry, stranding the operator on the old GUI.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "mowgli_hardware/legacy_odometry.hpp"
#include "mowgli_hardware/packet_handler.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::board_sends_legacy_odometry;
using mowgli_hardware::classify_odometry_packet;
using mowgli_hardware::kMowgliProtocolVersion;
using mowgli_hardware::LegacyOdometryDecoder;
using mowgli_hardware::LlOdometry;
using mowgli_hardware::LlOdometryV8;
using mowgli_hardware::OdometryWireFormat;

namespace
{

LlOdometryV8 v8(uint16_t dt, int32_t left, int32_t right, int16_t vl = 0, int16_t vr = 0)
{
  LlOdometryV8 p{};
  p.type = mowgli_hardware::PACKET_ID_LL_ODOMETRY;
  p.dt_millis = dt;
  p.left_ticks = left;
  p.right_ticks = right;
  p.left_velocity_mm_s = vl;
  p.right_velocity_mm_s = vr;
  return p;
}

}  // namespace

TEST(LegacyOdometryTest, WireLayoutMatchesProtocolV8)
{
  // type(1) + dt_millis(2) + left(4) + right(4) + vl(2) + vr(2) + crc(2).
  EXPECT_EQ(sizeof(LlOdometryV8), 17u);
  EXPECT_EQ(offsetof(LlOdometryV8, dt_millis), 1u);
  EXPECT_EQ(offsetof(LlOdometryV8, left_ticks), 3u);
  EXPECT_EQ(offsetof(LlOdometryV8, right_ticks), 7u);
  EXPECT_EQ(offsetof(LlOdometryV8, left_velocity_mm_s), 11u);
  EXPECT_EQ(offsetof(LlOdometryV8, right_velocity_mm_s), 13u);
  EXPECT_EQ(offsetof(LlOdometryV8, crc), 15u);
}

TEST(LegacyOdometryTest, ClassifiesByExactLength)
{
  EXPECT_EQ(classify_odometry_packet(sizeof(LlOdometry)), OdometryWireFormat::kCurrent);
  EXPECT_EQ(classify_odometry_packet(sizeof(LlOdometry) + 4u), OdometryWireFormat::kCurrent);
  EXPECT_EQ(classify_odometry_packet(17u), OdometryWireFormat::kLegacyV8);
  EXPECT_EQ(classify_odometry_packet(18u), OdometryWireFormat::kInvalid);
  EXPECT_EQ(classify_odometry_packet(16u), OdometryWireFormat::kInvalid);
  EXPECT_EQ(classify_odometry_packet(3u), OdometryWireFormat::kInvalid);
  EXPECT_EQ(classify_odometry_packet(0u), OdometryWireFormat::kInvalid);
}

TEST(LegacyOdometryTest, AcceptsTheLegacyLayoutFromEveryPublishedPreviousProtocol)
{
  // v1.5.0 shipped protocol v7, whose odometry is byte-identical to v8. A
  // v1.5.0 robot updating straight to this release must keep /wheel_odom
  // alive, or the update-first readiness starves and the update rolls back.
  EXPECT_TRUE(board_sends_legacy_odometry(7u));
  EXPECT_TRUE(board_sends_legacy_odometry(8u));
  EXPECT_FALSE(board_sends_legacy_odometry(0u));  // no handshake yet
  EXPECT_FALSE(board_sends_legacy_odometry(6u));
  EXPECT_FALSE(board_sends_legacy_odometry(kMowgliProtocolVersion));
}

TEST(LegacyOdometryTest, AccumulatesDtIntoMonotonicAcquisitionClock)
{
  LegacyOdometryDecoder d;
  EXPECT_EQ(d.convert(v8(999, 0, 0)).acquisition_millis, 0u);  // seed: dt ignored
  EXPECT_EQ(d.convert(v8(20, 0, 0)).acquisition_millis, 20u);
  EXPECT_EQ(d.convert(v8(21, 0, 0)).acquisition_millis, 41u);
  EXPECT_EQ(d.convert(v8(0, 0, 0)).acquisition_millis, 41u);
  EXPECT_EQ(d.convert(v8(65535, 0, 0)).acquisition_millis, 41u + 65535u);
}

TEST(LegacyOdometryTest, StationaryStreamKeepsTicksAndVelocity)
{
  LegacyOdometryDecoder d;
  for (int i = 0; i < 10; ++i)
  {
    const LlOdometry out = d.convert(v8(20, 1234, -5678));
    EXPECT_EQ(out.left_ticks, 1234);
    EXPECT_EQ(out.right_ticks, -5678);
    EXPECT_EQ(out.left_velocity_mm_s, 0);
    EXPECT_EQ(out.right_velocity_mm_s, 0);
    EXPECT_EQ(out.type, mowgli_hardware::PACKET_ID_LL_ODOMETRY);
  }
}

TEST(LegacyOdometryTest, MovingStreamCarriesDeltasAndVelocities)
{
  LegacyOdometryDecoder d;
  (void)d.convert(v8(20, 100, 200));
  const LlOdometry a = d.convert(v8(20, 105, 193, 250, -350));
  EXPECT_EQ(a.left_ticks - 100, 5);
  EXPECT_EQ(a.right_ticks - 200, -7);
  EXPECT_EQ(a.left_velocity_mm_s, 250);
  EXPECT_EQ(a.right_velocity_mm_s, -350);
  const LlOdometry b = d.convert(v8(20, 115, 183));
  EXPECT_EQ(b.left_ticks - a.left_ticks, 10);
  EXPECT_EQ(b.right_ticks - a.right_ticks, -10);
}

TEST(LegacyOdometryTest, Unwraps16BitCounterJumpsLikeTheV8Host)
{
  LegacyOdometryDecoder d;
  const LlOdometry seed = d.convert(v8(20, 10, 10));
  // A raw ±65536 jump (+ a few real ticks) is the v8 motor-controller wrap.
  const LlOdometry out = d.convert(v8(20, 10 - 65533, 10 + 65531));
  EXPECT_EQ(out.left_ticks - seed.left_ticks, 3);
  EXPECT_EQ(out.right_ticks - seed.right_ticks, -5);
  EXPECT_EQ(mowgli_hardware::unwrap_legacy_16bit_delta(32768), 32768);
  EXPECT_EQ(mowgli_hardware::unwrap_legacy_16bit_delta(-32768), -32768);
  EXPECT_EQ(mowgli_hardware::unwrap_legacy_16bit_delta(32769), -32767);
  EXPECT_EQ(mowgli_hardware::unwrap_legacy_16bit_delta(-32769), 32767);
}

TEST(LegacyOdometryTest, Int32CounterWrapIsModularNotUndefined)
{
  LegacyOdometryDecoder d;
  const LlOdometry seed = d.convert(v8(20, INT32_MAX - 1, INT32_MIN + 1));
  const LlOdometry out = d.convert(v8(20, INT32_MIN + 1, INT32_MAX - 1));
  EXPECT_EQ(static_cast<int32_t>(static_cast<uint32_t>(out.left_ticks) -
                                 static_cast<uint32_t>(seed.left_ticks)),
            3);
  EXPECT_EQ(static_cast<int32_t>(static_cast<uint32_t>(out.right_ticks) -
                                 static_cast<uint32_t>(seed.right_ticks)),
            -3);
}

TEST(LegacyOdometryTest, ResetReseedsBaselines)
{
  LegacyOdometryDecoder d;
  (void)d.convert(v8(20, 0, 0));
  (void)d.convert(v8(20, 50, 50));
  d.reset();
  const LlOdometry out = d.convert(v8(20, 7, 9));
  EXPECT_EQ(out.acquisition_millis, 0u);
  EXPECT_EQ(out.left_ticks, 7);
  EXPECT_EQ(out.right_ticks, 9);
}

// The 17-byte frame rides the same CRC-verified deframer as every packet:
// a good v8 frame reaches the dispatcher with its exact length, a corrupted
// one never does.
TEST(LegacyOdometryTest, V8FrameUsesTheSharedCrcPath)
{
  LlOdometryV8 pkt = v8(20, 42, -42, 1, -1);
  std::vector<std::size_t> delivered;
  mowgli_hardware::PacketHandler handler;
  handler.set_callback(
      [&delivered](const uint8_t*, std::size_t len)
      {
        delivered.push_back(len);
      });

  const auto frame = handler.encode_packet(reinterpret_cast<const uint8_t*>(&pkt),
                                           sizeof(LlOdometryV8) - sizeof(uint16_t));
  handler.feed(frame.data(), frame.size());
  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_EQ(classify_odometry_packet(delivered.front()), OdometryWireFormat::kLegacyV8);

  auto corrupted = frame;
  corrupted[3] ^= 0x5Au;
  handler.feed(corrupted.data(), corrupted.size());
  EXPECT_EQ(delivered.size(), 1u);
}
