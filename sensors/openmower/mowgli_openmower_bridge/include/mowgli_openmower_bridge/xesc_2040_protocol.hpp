// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file xesc_2040_protocol.hpp
 * @brief Wire format of the xESC 2040 (RP2040 ESC) used on OpenMower.
 *
 * Framing is identical to the LowLevel board and the Mowgli STM32:
 * `0x00 | COBS(payload + CRC-16/CCITT-FALSE LE) | 0x00`, handled by
 * mowgli_hardware::PacketHandler. Struct layouts mirror OpenMower's
 * xesc_2040_datatypes.h byte for byte (packed, little-endian, IEEE doubles).
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace mowgli_openmower_bridge::xesc2040
{

constexpr uint8_t kMsgTypeStatus = 1u;
constexpr uint8_t kMsgTypeControl = 2u;
constexpr uint8_t kMsgTypeSettings = 3u;

constexpr uint32_t kFaultUninitialized = 0b1u;
constexpr uint32_t kFaultWatchdog = 0b10u;
constexpr uint32_t kFaultUndervoltage = 0b100u;
constexpr uint32_t kFaultOvervoltage = 0b1000u;
constexpr uint32_t kFaultOvercurrent = 0b10000u;
constexpr uint32_t kFaultOvertempMotor = 0b100000u;
constexpr uint32_t kFaultOvertempPcb = 0b1000000u;
constexpr uint32_t kFaultInvalidHall = 0b10000000u;

constexpr std::size_t kHallTableSize = 8u;

#pragma pack(push, 1)

struct StatusPacket
{
  uint8_t message_type;
  uint32_t seq;
  uint8_t fw_version_major;
  uint8_t fw_version_minor;
  double voltage_input;  ///< [V]
  double temperature_pcb;  ///< [°C]
  double temperature_motor;  ///< [°C]
  double current_input;  ///< [A]
  double duty_cycle;  ///< [-1, 1]
  uint32_t tacho;
  uint32_t tacho_absolute;  ///< monotonic |ticks|
  bool direction;  ///< true = reverse (CCW)
  int32_t fault_code;
  uint16_t crc;
};

struct ControlPacket
{
  uint8_t message_type;
  double duty_cycle;  ///< [-1, 1]
  uint16_t crc;
};

struct SettingsPacket
{
  uint8_t message_type;
  uint8_t hall_table[kHallTableSize];
  float motor_current_limit;  ///< [A]
  float acceleration;  ///< duty/s
  bool has_motor_temp;
  float min_motor_temp;
  float max_motor_temp;
  float min_pcb_temp;
  float max_pcb_temp;
  uint16_t crc;
};

#pragma pack(pop)

static_assert(sizeof(StatusPacket) == 62u, "xESC 2040 status packet layout drifted");
static_assert(sizeof(ControlPacket) == 11u, "xESC 2040 control packet layout drifted");
static_assert(sizeof(SettingsPacket) == 36u, "xESC 2040 settings packet layout drifted");

}  // namespace mowgli_openmower_bridge::xesc2040
