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
 * @file ll_datatypes.hpp
 * @brief Wire-format structs for the STM32 ↔ Raspberry Pi serial protocol.
 *
 * All structs use #pragma pack(push,1) / #pragma pack(pop) to ensure
 * zero padding, matching the layout produced by the STM32 firmware.
 * Fields use fixed-width stdint types throughout.
 *
 * Ported from ll_datatypes.h in the OpenMower STM32 firmware.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace mowgli_hardware
{

// ---------------------------------------------------------------------------
// Packet type identifiers
// ---------------------------------------------------------------------------

/// Packet IDs shared by the STM32 firmware and this bridge node.
enum PacketId : uint8_t
{
  PACKET_ID_LL_STATUS = 0x01,  ///< STM32 → Pi: system status
  PACKET_ID_LL_IMU = 0x02,  ///< STM32 → Pi: IMU data
  PACKET_ID_LL_UI_EVENT = 0x03,  ///< STM32 → Pi: UI button event
  PACKET_ID_LL_ODOMETRY = 0x04,  ///< STM32 → Pi: wheel odometry
  PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ = 0x11,  ///< Bidirectional: config request
  PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP = 0x12,  ///< Bidirectional: config response
  PACKET_ID_LL_HEARTBEAT = 0x42,  ///< Pi → STM32: heartbeat
  PACKET_ID_LL_HIGH_LEVEL_STATE = 0x43,  ///< Pi → STM32: high-level state
  PACKET_ID_LL_CMD_VEL = 0x50,  ///< Pi → STM32: velocity command (extension)
  PACKET_ID_LL_BLADE_STATUS = 0x05,  ///< STM32 → Pi: blade motor status
  PACKET_ID_LL_CMD_BLADE = 0x51,  ///< Pi → STM32: blade motor control
  PACKET_ID_LL_REBOOT = 0x52,  ///< Pi → STM32: reboot the board (NVIC_SystemReset)
};

/// Magic byte in LlReboot — a dedicated reboot packet plus this confirmation
/// byte prevents a corrupt/misframed packet from accidentally rebooting the
/// board (the consequence is a full firmware restart).
static constexpr uint8_t kLlRebootMagic = 0xB0;

// ---------------------------------------------------------------------------
// Status bitmask constants (ll_status::status_bitmask)
// ---------------------------------------------------------------------------

constexpr uint8_t STATUS_BIT_INITIALIZED = (1u << 0u);
constexpr uint8_t STATUS_BIT_RASPI_POWER = (1u << 1u);
constexpr uint8_t STATUS_BIT_CHARGING = (1u << 2u);
// Bit 3 is reserved / free
constexpr uint8_t STATUS_BIT_RAIN = (1u << 4u);
constexpr uint8_t STATUS_BIT_SOUND_AVAIL = (1u << 5u);
constexpr uint8_t STATUS_BIT_SOUND_BUSY = (1u << 6u);
constexpr uint8_t STATUS_BIT_UI_AVAIL = (1u << 7u);

// ---------------------------------------------------------------------------
// Emergency bitmask constants (ll_status::emergency_bitmask)
// ---------------------------------------------------------------------------

constexpr uint8_t EMERGENCY_BIT_LATCH = (1u << 0u);
constexpr uint8_t EMERGENCY_BIT_STOP = (1u << 1u);
constexpr uint8_t EMERGENCY_BIT_LIFT = (1u << 2u);

// ---------------------------------------------------------------------------
// USS (ultrasonic) sensor count
// ---------------------------------------------------------------------------

constexpr std::size_t LL_USS_SENSOR_COUNT = 5u;

// ---------------------------------------------------------------------------
// Wire-format structs — all fields packed with no padding
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

/**
 * @brief System status packet sent by the STM32 (PACKET_ID_LL_STATUS = 0x01).
 */
struct LlStatus
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_STATUS
  uint8_t status_bitmask;  ///< See STATUS_BIT_* constants
  float uss_ranges_m[LL_USS_SENSOR_COUNT];  ///< Ultrasonic range readings [m]
  uint8_t emergency_bitmask;  ///< See EMERGENCY_BIT_* constants
  float v_charge;  ///< Charge voltage [V]
  float v_system;  ///< System/battery voltage [V]
  float charging_current;  ///< Charging current [A]
  uint8_t batt_percentage;  ///< Battery charge percentage [0-100]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief IMU data packet sent by the STM32 (PACKET_ID_LL_IMU = 0x02).
 */
struct LlImu
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_IMU
  uint16_t dt_millis;  ///< Time delta since last packet [ms]
  float acceleration_mss[3];  ///< Linear acceleration [m/s^2], order: x, y, z
  float gyro_rads[3];  ///< Angular velocity [rad/s], order: x, y, z
  float mag_uT[3];  ///< Magnetic field [uT], order: x, y, z
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief UI event packet sent by the STM32 (PACKET_ID_LL_UI_EVENT = 0x03).
 */
struct LlUiEvent
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_UI_EVENT
  uint8_t button_id;  ///< Identifier of the button that was pressed
  uint8_t press_duration;  ///< Duration category (short/long)
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Wheel odometry packet sent by the STM32 (PACKET_ID_LL_ODOMETRY = 0x04).
 *
 * Sent every 20 ms when the drive motor controller responds with encoder data.
 *
 * Signed end-to-end: left_ticks/right_ticks carry direction in their sign
 * (no separate direction byte). Per-wheel velocity is computed on the
 * firmware side using the hardware-timer dt, so the host consumes it
 * directly without dividing by a jittery packet-arrival interval.
 */
struct LlOdometry
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_ODOMETRY
  uint16_t dt_millis;  ///< Firmware-measured interval since last packet [ms]
  int32_t left_ticks;  ///< Signed cumulative left encoder ticks
  int32_t right_ticks;  ///< Signed cumulative right encoder ticks
  int16_t left_velocity_mm_s;  ///< Signed left wheel velocity [mm/s]
  int16_t right_velocity_mm_s;  ///< Signed right wheel velocity [mm/s]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Heartbeat packet sent by the Pi (PACKET_ID_LL_HEARTBEAT = 0x42).
 *
 * Must be sent at regular intervals (typically 250 ms). The STM32 will
 * trigger an emergency stop if no heartbeat arrives within its timeout window.
 */
struct LlHeartbeat
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_HEARTBEAT
  uint8_t emergency_requested;  ///< Non-zero → request emergency stop
  uint8_t emergency_release_requested;  ///< Non-zero → release latched emergency
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief High-level state packet sent by the Pi (PACKET_ID_LL_HIGH_LEVEL_STATE = 0x43).
 *
 * Informs the STM32 of the current operational mode and GPS fix quality.
 */
struct LlHighLevelState
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_HIGH_LEVEL_STATE
  uint8_t current_mode;  ///< Current high-level operating mode
  uint8_t gps_quality;  ///< GPS fix quality indicator [0-100]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Velocity command packet sent by the Pi (PACKET_ID_LL_CMD_VEL = 0x50).
 *
 * Extension packet not in the original firmware; bridges geometry_msgs/Twist
 * to differential-drive wheel velocities for the STM32 motion controller.
 */
struct LlCmdVel
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_CMD_VEL
  float linear_x;  ///< Forward velocity [m/s]
  float angular_z;  ///< Angular (yaw) velocity [rad/s]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Blade motor control packet sent by the Pi (PACKET_ID_LL_CMD_BLADE = 0x51).
 */
struct LlCmdBlade
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_CMD_BLADE
  uint8_t blade_on;  ///< 1=start, 0=stop
  uint8_t blade_dir;  ///< 0=normal, 1=reverse
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Reboot request sent by the Pi (PACKET_ID_LL_REBOOT = 0x52).
 * The firmware reboots (NVIC_SystemReset) only when magic == kLlRebootMagic.
 * Used to recover the board from a wedged state (e.g. the IMU emitting NaN)
 * without a manual power-cycle.
 */
struct LlReboot
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_REBOOT
  uint8_t magic;  ///< Must equal kLlRebootMagic (0xB0)
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Blade motor status packet from STM32 (PACKET_ID_LL_BLADE_STATUS = 0x05).
 */
struct LlBladeStatus
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_BLADE_STATUS
  uint8_t is_active;  ///< 1=running, 0=stopped
  uint16_t rpm;  ///< Blade motor RPM
  uint16_t power_watts;  ///< Power consumption [W]
  float temperature;  ///< Blade/motor temperature [C]
  uint32_t error_count;  ///< Cumulative error counter
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

#pragma pack(pop)

// ---------------------------------------------------------------------------
// Compile-time size sanity checks
// ---------------------------------------------------------------------------

static_assert(sizeof(LlStatus) == 38u, "LlStatus layout mismatch");
static_assert(sizeof(LlImu) == 41u, "LlImu layout mismatch");
static_assert(sizeof(LlUiEvent) == 5u, "LlUiEvent layout mismatch");
static_assert(sizeof(LlOdometry) == 17u, "LlOdometry layout mismatch");
static_assert(sizeof(LlHeartbeat) == 5u, "LlHeartbeat layout mismatch");
static_assert(sizeof(LlHighLevelState) == 5u, "LlHighLevelState layout mismatch");
static_assert(sizeof(LlCmdVel) == 11u, "LlCmdVel layout mismatch");
static_assert(sizeof(LlCmdBlade) == 5u, "LlCmdBlade layout mismatch");
static_assert(sizeof(LlBladeStatus) == 16u, "LlBladeStatus layout mismatch");

}  // namespace mowgli_hardware
