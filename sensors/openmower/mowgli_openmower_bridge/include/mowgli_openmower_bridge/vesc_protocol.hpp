// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file vesc_protocol.hpp
 * @brief Wire format of the xESC mini (VESC-protocol STM32 ESC) as flashed on
 *        stock OpenMower drive and mow motors.
 *
 * Frame: `[SOF=2][len][payload…][crc16 BE][EOF=3]` (len < 256; SOF=3 + 16-bit
 * length for longer payloads). CRC-16 with polynomial 0x1021 and init 0
 * (CRC-16/XMODEM — NOT the CCITT-FALSE the COBS links use). payload[0] is the
 * COMM id. The GET_VALUES layout below is the xESC mini firmware's, mirrored
 * from OpenMower's vesc_driver data_map.h; offsets are relative to payload[0].
 *
 * Pure functions only — no I/O, no ROS — so the unit tests pin the exact
 * bytes on the wire.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mowgli_openmower_bridge::vesc
{

constexpr uint8_t kSofSmallFrame = 2u;
constexpr uint8_t kSofLargeFrame = 3u;
constexpr uint8_t kEof = 3u;
constexpr std::size_t kMaxPayloadSize = 1024u;

enum CommId : uint8_t
{
  COMM_FW_VERSION = 0u,
  COMM_GET_VALUES = 4u,
  COMM_SET_DUTY = 5u,
};

/// Byte offsets inside a COMM_GET_VALUES payload (payload[0] is the id).
enum ValuesOffset : std::size_t
{
  kTempMos = 1u,  ///< int16 / 10 [°C]
  kTempMotor = 3u,  ///< int16 / 10 [°C]
  kCurrentMotor = 5u,  ///< int32 / 100 [A]
  kCurrentIn = 9u,  ///< int32 / 100 [A]
  kDutyNow = 21u,  ///< int16 / 1000
  kErpm = 23u,  ///< int32
  kVoltageIn = 27u,  ///< int16 / 10 [V]
  kTachometer = 45u,  ///< int32, signed cumulative ticks
  kTachometerAbs = 49u,  ///< int32, unsigned cumulative ticks
  kFaultCode = 53u,  ///< uint8
};

/// Minimum GET_VALUES payload length that carries every field above.
constexpr std::size_t kValuesPayloadMinSize = kFaultCode + 1u;

/// CRC-16/XMODEM (poly 0x1021, init 0x0000, no reflection, no xor-out).
[[nodiscard]] uint16_t crc16_xmodem(const uint8_t* data, std::size_t len) noexcept;

/// Wrap a payload into a complete VESC frame.
[[nodiscard]] std::vector<uint8_t> BuildFrame(const std::vector<uint8_t>& payload);

[[nodiscard]] std::vector<uint8_t> BuildFwVersionRequest();
[[nodiscard]] std::vector<uint8_t> BuildGetValuesRequest();

/// COMM_SET_DUTY with the duty clamped to [-1, 1] and scaled by 100000.
[[nodiscard]] std::vector<uint8_t> BuildSetDuty(double duty);

struct Values
{
  double temp_pcb{0.0};
  double temp_motor{0.0};
  double current_motor{0.0};
  double current_in{0.0};
  double duty{0.0};
  double erpm{0.0};
  double voltage_in{0.0};
  int32_t tacho{0};
  int32_t tacho_abs{0};
  uint8_t fault_code{0u};
};

struct FwVersion
{
  uint8_t major{0u};
  uint8_t minor{0u};
};

enum class PayloadKind
{
  kValues,
  kFwVersion,
  kOther,
};

struct ParsedPayload
{
  PayloadKind kind{PayloadKind::kOther};
  Values values{};
  FwVersion fw{};
};

/// Decode a complete payload (id byte first). Returns nullopt when the id is
/// known but the payload is too short to be trusted.
[[nodiscard]] std::optional<ParsedPayload> ParsePayload(const uint8_t* payload, std::size_t len);

/**
 * @brief Incremental frame extractor over a raw byte stream.
 *
 * NextPayload() scans for the first COMPLETE, CRC-verified frame anywhere in
 * the buffer and drops everything before it. A garbage byte that merely looks
 * like a frame start (e.g. 0x03 followed by a huge "length") therefore cannot
 * stall the parser waiting for bytes that never come: as soon as a real frame
 * lands behind it, the phantom is discarded. Bytes are held only while some
 * candidate frame is still incomplete; the buffer is capped to keep a dead
 * stream from growing without bound.
 */
class Deframer
{
public:
  static constexpr std::size_t kMaxBufferedBytes = 4096u;

  void Feed(const uint8_t* data, std::size_t len);

  /// Pop the next complete, CRC-verified payload, or nullopt.
  [[nodiscard]] std::optional<std::vector<uint8_t>> NextPayload();

  [[nodiscard]] uint64_t crc_errors() const noexcept
  {
    return crc_errors_;
  }
  [[nodiscard]] uint64_t resync_drops() const noexcept
  {
    return resync_drops_;
  }

private:
  std::vector<uint8_t> buffer_;
  uint64_t crc_errors_{0u};
  uint64_t resync_drops_{0u};
};

}  // namespace mowgli_openmower_bridge::vesc
