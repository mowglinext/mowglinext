// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_vesc_protocol.cpp
 * @brief Pins the xESC mini (VESC) frame bytes, CRC, GET_VALUES decoding and
 *        the deframer's resynchronisation behaviour.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include "mowgli_openmower_bridge/vesc_protocol.hpp"
#include <gtest/gtest.h>

namespace vesc = mowgli_openmower_bridge::vesc;

namespace
{

std::vector<uint8_t> ValuesPayload(int16_t temp_mos,
                                   int16_t temp_motor,
                                   int32_t current_in,
                                   int16_t duty,
                                   int32_t erpm,
                                   int16_t v_in,
                                   int32_t tacho,
                                   int32_t tacho_abs,
                                   uint8_t fault)
{
  std::vector<uint8_t> p(vesc::kValuesPayloadMinSize, 0u);
  p[0] = vesc::COMM_GET_VALUES;
  auto put16 = [&p](std::size_t at, int16_t v)
  {
    p[at] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8u) & 0xFFu);
    p[at + 1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFu);
  };
  auto put32 = [&p](std::size_t at, int32_t v)
  {
    const auto u = static_cast<uint32_t>(v);
    p[at] = static_cast<uint8_t>((u >> 24u) & 0xFFu);
    p[at + 1] = static_cast<uint8_t>((u >> 16u) & 0xFFu);
    p[at + 2] = static_cast<uint8_t>((u >> 8u) & 0xFFu);
    p[at + 3] = static_cast<uint8_t>(u & 0xFFu);
  };
  put16(vesc::kTempMos, temp_mos);
  put16(vesc::kTempMotor, temp_motor);
  put32(vesc::kCurrentIn, current_in);
  put16(vesc::kDutyNow, duty);
  put32(vesc::kErpm, erpm);
  put16(vesc::kVoltageIn, v_in);
  put32(vesc::kTachometer, tacho);
  put32(vesc::kTachometerAbs, tacho_abs);
  p[vesc::kFaultCode] = fault;
  return p;
}

}  // namespace

TEST(VescProtocol, Crc16XmodemMatchesKnownVector)
{
  // CRC-16/XMODEM check value for "123456789" is 0x31C3.
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(vesc::crc16_xmodem(data, sizeof(data)), 0x31C3u);
}

TEST(VescProtocol, GetValuesRequestFrameBytes)
{
  const auto frame = vesc::BuildGetValuesRequest();
  // [2][len=1][COMM_GET_VALUES=4][crc hi][crc lo][3]
  ASSERT_EQ(frame.size(), 6u);
  EXPECT_EQ(frame[0], 2u);
  EXPECT_EQ(frame[1], 1u);
  EXPECT_EQ(frame[2], 4u);
  const uint8_t id = 4u;
  const uint16_t crc = vesc::crc16_xmodem(&id, 1u);
  EXPECT_EQ(frame[3], static_cast<uint8_t>(crc >> 8u));
  EXPECT_EQ(frame[4], static_cast<uint8_t>(crc & 0xFFu));
  EXPECT_EQ(frame[5], 3u);
}

TEST(VescProtocol, SetDutyScalesAndClamps)
{
  const auto frame = vesc::BuildSetDuty(0.5);
  ASSERT_EQ(frame.size(), 10u);
  EXPECT_EQ(frame[2], vesc::COMM_SET_DUTY);
  // 0.5 * 100000 = 50000 = 0x0000C350 big-endian
  EXPECT_EQ(frame[3], 0x00u);
  EXPECT_EQ(frame[4], 0x00u);
  EXPECT_EQ(frame[5], 0xC3u);
  EXPECT_EQ(frame[6], 0x50u);

  const auto over = vesc::BuildSetDuty(7.0);
  // clamped to 1.0 → 100000 = 0x000186A0
  EXPECT_EQ(over[3], 0x00u);
  EXPECT_EQ(over[4], 0x01u);
  EXPECT_EQ(over[5], 0x86u);
  EXPECT_EQ(over[6], 0xA0u);

  const auto nan_frame = vesc::BuildSetDuty(std::nan(""));
  EXPECT_EQ(nan_frame[3], 0u);
  EXPECT_EQ(nan_frame[4], 0u);
  EXPECT_EQ(nan_frame[5], 0u);
  EXPECT_EQ(nan_frame[6], 0u);
}

TEST(VescProtocol, ParsesValuesWithFirmwareScaling)
{
  const auto p = ValuesPayload(312, 405, -150, -250, -3600, 267, -1234, 5678, 0u);
  const auto parsed = vesc::ParsePayload(p.data(), p.size());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->kind, vesc::PayloadKind::kValues);
  EXPECT_DOUBLE_EQ(parsed->values.temp_pcb, 31.2);
  EXPECT_DOUBLE_EQ(parsed->values.temp_motor, 40.5);
  EXPECT_DOUBLE_EQ(parsed->values.current_in, -1.5);
  EXPECT_DOUBLE_EQ(parsed->values.duty, -0.25);
  EXPECT_DOUBLE_EQ(parsed->values.erpm, -3600.0);
  EXPECT_DOUBLE_EQ(parsed->values.voltage_in, 26.7);
  EXPECT_EQ(parsed->values.tacho, -1234);
  EXPECT_EQ(parsed->values.tacho_abs, 5678);
  EXPECT_EQ(parsed->values.fault_code, 0u);
}

TEST(VescProtocol, ShortValuesPayloadIsRejected)
{
  std::vector<uint8_t> p(20u, 0u);
  p[0] = vesc::COMM_GET_VALUES;
  EXPECT_FALSE(vesc::ParsePayload(p.data(), p.size()).has_value());
}

TEST(VescProtocol, ParsesFirmwareVersion)
{
  const std::vector<uint8_t> p{vesc::COMM_FW_VERSION, 5u, 3u};
  const auto parsed = vesc::ParsePayload(p.data(), p.size());
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->kind, vesc::PayloadKind::kFwVersion);
  EXPECT_EQ(parsed->fw.major, 5u);
  EXPECT_EQ(parsed->fw.minor, 3u);
}

TEST(VescDeframer, RoundTripsAFrameFedByteByByte)
{
  const auto payload = ValuesPayload(1, 2, 3, 4, 5, 6, 7, 8, 9u);
  const auto frame = vesc::BuildFrame(payload);
  vesc::Deframer d;
  for (std::size_t i = 0; i + 1 < frame.size(); ++i)
  {
    d.Feed(&frame[i], 1u);
    EXPECT_FALSE(d.NextPayload().has_value());
  }
  d.Feed(&frame.back(), 1u);
  const auto out = d.NextPayload();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(*out, payload);
  EXPECT_FALSE(d.NextPayload().has_value());
  EXPECT_EQ(d.crc_errors(), 0u);
}

TEST(VescDeframer, ResynchronisesAfterGarbageAndDropsBadCrc)
{
  const auto good = vesc::BuildFrame({vesc::COMM_FW_VERSION, 5u, 3u});
  auto bad = good;
  bad[3] ^= 0xFFu;  // corrupt the payload → CRC mismatch
  std::vector<uint8_t> stream{0xAAu, 0x55u};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());

  vesc::Deframer d;
  d.Feed(stream.data(), stream.size());
  const auto out = d.NextPayload();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ((*out)[0], vesc::COMM_FW_VERSION);
  EXPECT_EQ((*out)[1], 5u);
  EXPECT_GE(d.crc_errors(), 1u);
  EXPECT_GE(d.resync_drops(), 2u);
  EXPECT_FALSE(d.NextPayload().has_value());
}

TEST(VescDeframer, PhantomLargeFrameHeaderDoesNotStallTheParser)
{
  // 0x03 0x00 0xFA looks like a large-frame header announcing 250 bytes; a
  // real 6-byte frame right behind it must still come out.
  const auto good = vesc::BuildFrame({vesc::COMM_FW_VERSION, 5u, 3u});
  std::vector<uint8_t> stream{0x03u, 0x00u, 0xFAu};
  stream.insert(stream.end(), good.begin(), good.end());
  vesc::Deframer d;
  d.Feed(stream.data(), stream.size());
  const auto out = d.NextPayload();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ((*out)[0], vesc::COMM_FW_VERSION);
  EXPECT_FALSE(d.NextPayload().has_value());
}

TEST(VescDeframer, IncompleteFrameIsKeptUntilItsTailArrives)
{
  const auto good = vesc::BuildFrame({vesc::COMM_FW_VERSION, 5u, 3u});
  vesc::Deframer d;
  d.Feed(good.data(), 4u);
  EXPECT_FALSE(d.NextPayload().has_value());
  d.Feed(good.data() + 4, good.size() - 4u);
  ASSERT_TRUE(d.NextPayload().has_value());
}
