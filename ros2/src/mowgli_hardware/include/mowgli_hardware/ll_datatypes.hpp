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

// Protocol v2 moves runtime drive tuning to packet 0x54 and adds
// ticks_per_meter to the payload so legacy firmware safely ignores the packet
// rather than mis-parsing it as the old PID-only layout.
// Protocol v3 extends the reset-cause packet with last_stage_before_reset so
// the bridge can report which firmware main-loop section was active when the
// WWDG fired. The same v3 diagnostic stack also uses the config handshake
// flags byte so the GUI can toggle optional firmware diagnostics on demand.
// Protocol v7 replaces the four per-group runtime packets (0x54-0x57) with the
// generic SET_PARAM / GET_PARAM / PARAM_COMMIT protocol: every firmware
// parameter has a stable id (FirmwareParamId below, mirroring
// fw_param_catalog.h), the firmware reports the value it applied, and it
// persists the set in flash so it applies before the host connects.
//
// This is the COMPATIBILITY KEY: the bridge sends PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ
// on every (re)connect and the firmware answers with its own
// MOWGLI_PROTOCOL_VERSION in LlConfigRsp. If they differ — or the firmware is
// too old to answer at all — the image and firmware speak different wire
// formats and the operator must reflash. Bump this in lockstep with
// MOWGLI_PROTOCOL_VERSION in mowgli_protocol.h when an incompatible wire
// change requires a hard compatibility break.
static constexpr uint8_t kMowgliProtocolVersion = 7u;

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
  PACKET_ID_LL_RESET_CAUSE = 0x06,  ///< STM32 → Pi: current boot reset cause
  PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ = 0x11,  ///< Bidirectional: config request
  PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP = 0x12,  ///< Bidirectional: config response
  PACKET_ID_LL_PARAM_VALUE = 0x13,  ///< STM32 → Pi: one parameter's applied value
  PACKET_ID_LL_PARAM_STORE_STATUS = 0x14,  ///< STM32 → Pi: flash persistence state
  PACKET_ID_LL_HEARTBEAT = 0x42,  ///< Pi → STM32: heartbeat
  PACKET_ID_LL_HIGH_LEVEL_STATE = 0x43,  ///< Pi → STM32: high-level state
  PACKET_ID_LL_CMD_VEL = 0x50,  ///< Pi → STM32: velocity command (extension)
  PACKET_ID_LL_BLADE_STATUS = 0x05,  ///< STM32 → Pi: blade motor status
  PACKET_ID_LL_CMD_BLADE = 0x51,  ///< Pi → STM32: blade motor control
  PACKET_ID_LL_REBOOT = 0x52,  ///< Pi → STM32: reboot the board (NVIC_SystemReset)
  // 0x54-0x57 (per-group runtime packets) were retired in protocol v7.
  PACKET_ID_LL_SET_PARAM = 0x58,  ///< Pi → STM32: set one runtime parameter
  PACKET_ID_LL_GET_PARAM = 0x59,  ///< Pi → STM32: request parameter report(s)
  PACKET_ID_LL_PARAM_COMMIT = 0x5A,  ///< Pi → STM32: persist the parameter set to flash
};

/// Magic byte in LlParamCommit.
static constexpr uint8_t kLlParamCommitMagic = 0xC5;

/// Runtime firmware parameter ids — mirror of fw_param_catalog.h (pinned by
/// test_fw_param_catalog.cpp). Ids are also flash record keys: never renumber.
enum FirmwareParamId : uint16_t
{
  FW_PARAM_ID_TICKS_PER_METER = 1,
  FW_PARAM_ID_WHEEL_KP = 2,
  FW_PARAM_ID_WHEEL_KI = 3,
  FW_PARAM_ID_WHEEL_KD = 4,
  FW_PARAM_ID_WHEEL_INTEGRAL_LIMIT = 5,
  FW_PARAM_ID_PWM_PER_MPS = 6,
  FW_PARAM_ID_YAW_KP = 10,
  FW_PARAM_ID_YAW_KI = 11,
  FW_PARAM_ID_YAW_TRIM_LIMIT_MPS = 12,
  FW_PARAM_ID_YAW_LOOP_ENABLED = 13,
  FW_PARAM_ID_YAW_GYRO_SIGN = 14,
  FW_PARAM_ID_YAW_GYRO_BIAS_RADPS = 15,
  FW_PARAM_ID_MAX_MPS = 20,
  FW_PARAM_ID_WHEEL_BASE = 21,
  FW_PARAM_ID_MAX_CHARGE_VOLTAGE = 30,
  FW_PARAM_ID_MAX_CHARGE_CURRENT = 31,
  FW_PARAM_ID_ONE_WHEEL_LIFT_MS = 40,
  FW_PARAM_ID_BOTH_WHEELS_LIFT_MS = 41,
  FW_PARAM_ID_TILT_MS = 42,
  FW_PARAM_ID_STOP_BUTTON_MS = 43,
  FW_PARAM_ID_PLAY_CLEAR_MS = 44,
  FW_PARAM_ID_IMU_INCLINATION_THRESHOLD = 50,
  FW_PARAM_ALL =
      0xFFFF,  ///< GET_PARAM: every parameter + the store status (firmware FW_PARAM_ID_ALL)
};

/// LlParamValue::status (fw_param_status_t).
constexpr uint8_t FW_PARAM_STATUS_OK = 0u;
constexpr uint8_t FW_PARAM_STATUS_CLAMPED = 1u;
constexpr uint8_t FW_PARAM_STATUS_UNKNOWN_ID = 2u;
constexpr uint8_t FW_PARAM_STATUS_REJECTED = 3u;

/// LlParamValue::flags.
constexpr uint8_t PARAM_VALUE_FLAG_PERSISTED = 0x01u;
constexpr uint8_t PARAM_VALUE_FLAG_VOLATILE = 0x02u;

/// LlParamStoreStatus::boot_source.
constexpr uint8_t PARAM_BOOT_DEFAULTS = 0u;
constexpr uint8_t PARAM_BOOT_FLASH = 1u;
constexpr uint8_t PARAM_BOOT_FLASH_ERASED = 2u;

/// LlParamStoreStatus::last_commit.
constexpr uint8_t PARAM_COMMIT_NONE = 0u;
constexpr uint8_t PARAM_COMMIT_WRITTEN = 1u;
constexpr uint8_t PARAM_COMMIT_UNCHANGED = 2u;
constexpr uint8_t PARAM_COMMIT_PENDING = 3u;
constexpr uint8_t PARAM_COMMIT_LOG_FULL = 4u;
constexpr uint8_t PARAM_COMMIT_ERROR = 5u;

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
// Reset cause constants (ll_reset_cause::reset_cause)
// ---------------------------------------------------------------------------

constexpr uint8_t RESET_CAUSE_UNKNOWN = 0u;
constexpr uint8_t RESET_CAUSE_PIN = 1u;
constexpr uint8_t RESET_CAUSE_POR_PDR = 2u;
constexpr uint8_t RESET_CAUSE_BOR = 3u;
constexpr uint8_t RESET_CAUSE_SFTRST = 4u;
constexpr uint8_t RESET_CAUSE_IWDG = 5u;
constexpr uint8_t RESET_CAUSE_WWDG = 6u;
constexpr uint8_t RESET_CAUSE_LPWR = 7u;

// ---------------------------------------------------------------------------
// Watchdog breadcrumb constants
// (ll_reset_cause::last_stage_before_reset)
// ---------------------------------------------------------------------------

constexpr uint8_t WATCHDOG_STAGE_NONE = 0u;
constexpr uint8_t WATCHDOG_STAGE_CHATTER = 1u;
constexpr uint8_t WATCHDOG_STAGE_MOTORS = 2u;
constexpr uint8_t WATCHDOG_STAGE_PANEL = 3u;
constexpr uint8_t WATCHDOG_STAGE_ROS_SPIN = 4u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST = 5u;
constexpr uint8_t WATCHDOG_STAGE_DRIVEMOTOR_RX = 6u;
constexpr uint8_t WATCHDOG_STAGE_PERIMETER = 7u;
constexpr uint8_t WATCHDOG_STAGE_ADC = 8u;
constexpr uint8_t WATCHDOG_STAGE_CHARGER = 9u;
constexpr uint8_t WATCHDOG_STAGE_STATUS_LED = 10u;
constexpr uint8_t WATCHDOG_STAGE_ULTRASONIC_HANDLER = 11u;
constexpr uint8_t WATCHDOG_STAGE_ULTRASONIC_APP = 12u;
constexpr uint8_t WATCHDOG_STAGE_WATCHDOG_REFRESH = 13u;
constexpr uint8_t WATCHDOG_STAGE_DRIVEMOTOR_10MS = 14u;
constexpr uint8_t WATCHDOG_STAGE_BLADEMOTOR = 15u;
constexpr uint8_t WATCHDOG_STAGE_BUZZER = 16u;
constexpr uint8_t WATCHDOG_STAGE_EMERGENCY = 17u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_ENTER = 18u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_IMU_BUILD = 19u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_IMU_SEND = 20u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_RESET_SEND = 21u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_STATUS_SEND = 22u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_BLADE_SEND = 23u;
constexpr uint8_t WATCHDOG_STAGE_BROADCAST_EXIT = 24u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_ENTER = 25u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_QUEUE = 26u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_RESUME = 27u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_EXIT = 28u;
constexpr uint8_t WATCHDOG_STAGE_IMU_ACCEL = 29u;
constexpr uint8_t WATCHDOG_STAGE_IMU_GYRO = 30u;
constexpr uint8_t WATCHDOG_STAGE_IMU_MAG = 31u;
constexpr uint8_t WATCHDOG_STAGE_IMU_PACKET_FILL = 32u;
constexpr uint8_t WATCHDOG_STAGE_USB_IRQ_ENTER = 33u;
constexpr uint8_t WATCHDOG_STAGE_USB_IRQ_EXIT = 34u;
constexpr uint8_t WATCHDOG_STAGE_CDC_RX_ENTER = 35u;
constexpr uint8_t WATCHDOG_STAGE_CDC_RX_PROCESS = 36u;
constexpr uint8_t WATCHDOG_STAGE_CDC_RX_EXIT = 37u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_PACKET = 38u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_COMPLETE = 39u;
constexpr uint8_t WATCHDOG_STAGE_USB_RESET = 40u;
constexpr uint8_t WATCHDOG_STAGE_USB_SUSPEND = 41u;
constexpr uint8_t WATCHDOG_STAGE_USB_RESUME = 42u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_PACKET_FAIL = 43u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_BUSY_STUCK = 44u;
constexpr uint8_t WATCHDOG_STAGE_CDC_TX_QUEUE_FULL = 45u;
constexpr uint8_t WATCHDOG_STAGE_CDC_HOST_CLOSED = 46u;

// ---------------------------------------------------------------------------
// USS (ultrasonic) sensor count
// ---------------------------------------------------------------------------

constexpr std::size_t LL_USS_SENSOR_COUNT = 5u;

// ---------------------------------------------------------------------------
// Config flags
// ---------------------------------------------------------------------------

constexpr uint8_t CONFIG_FLAG_FIRMWARE_DEBUG = (1u << 0u);

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
  // Legacy/reserved: firmware has no verified SoC provider and emits 0.
  // The host must publish BatteryState.percentage as NaN (unknown), not 0 %.
  uint8_t batt_percentage;
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
 * @brief Boot reset cause packet sent by the STM32 (PACKET_ID_LL_RESET_CAUSE = 0x06).
 *
 * Sent periodically so the host can recover the current boot cause even if it
 * connects after the STM32 has already started streaming. When reset_cause is
 * WWDG, last_stage_before_reset carries the persisted main-loop breadcrumb
 * captured immediately before the watchdog reset.
 */
struct LlResetCause
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_RESET_CAUSE
  uint8_t reset_cause;  ///< RESET_CAUSE_* constant
  uint8_t last_stage_before_reset;  ///< WATCHDOG_STAGE_* constant
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
 * @brief Set one runtime parameter (PACKET_ID_LL_SET_PARAM = 0x58).
 *
 * The firmware coerces the value into the parameter's absolute envelope,
 * applies it from its main loop and answers with LlParamValue.
 */
struct LlSetParam
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_SET_PARAM
  uint16_t param_id;  ///< FirmwareParamId
  float value;  ///< Requested value
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};
static_assert(offsetof(LlSetParam, param_id) == 1u, "LlSetParam.param_id offset drifted");
static_assert(offsetof(LlSetParam, value) == 3u, "LlSetParam.value offset drifted");
static_assert(offsetof(LlSetParam, crc) == 7u, "LlSetParam.crc offset drifted");

/**
 * @brief Request one parameter's report, or all of them plus the store status
 * with FW_PARAM_ID_ALL (PACKET_ID_LL_GET_PARAM = 0x59).
 */
struct LlGetParam
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_GET_PARAM
  uint16_t param_id;  ///< FirmwareParamId or FW_PARAM_ID_ALL
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Persist the current parameter set (PACKET_ID_LL_PARAM_COMMIT = 0x5A).
 * An unchanged set is not rewritten, so committing after every burst is cheap.
 */
struct LlParamCommit
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_PARAM_COMMIT
  uint8_t magic;  ///< Must equal kLlParamCommitMagic (0xC5)
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief One parameter's state from the STM32 (PACKET_ID_LL_PARAM_VALUE = 0x13).
 */
struct LlParamValue
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_PARAM_VALUE
  uint16_t param_id;  ///< FirmwareParamId
  uint8_t status;  ///< FW_PARAM_STATUS_* of the last set since boot
  uint8_t flags;  ///< PARAM_VALUE_FLAG_*
  float value;  ///< Applied value
  float default_value;  ///< Compiled power-on default
  float min_value;  ///< Absolute envelope lower bound
  float max_value;  ///< Absolute envelope upper bound
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};
static_assert(offsetof(LlParamValue, status) == 3u, "LlParamValue.status offset drifted");
static_assert(offsetof(LlParamValue, flags) == 4u, "LlParamValue.flags offset drifted");
static_assert(offsetof(LlParamValue, value) == 5u, "LlParamValue.value offset drifted");
static_assert(offsetof(LlParamValue, default_value) == 9u,
              "LlParamValue.default_value offset drifted");
static_assert(offsetof(LlParamValue, min_value) == 13u, "LlParamValue.min_value offset drifted");
static_assert(offsetof(LlParamValue, max_value) == 17u, "LlParamValue.max_value offset drifted");
static_assert(offsetof(LlParamValue, crc) == 21u, "LlParamValue.crc offset drifted");

/**
 * @brief Flash persistence state from the STM32
 * (PACKET_ID_LL_PARAM_STORE_STATUS = 0x14).
 */
struct LlParamStoreStatus
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_PARAM_STORE_STATUS
  uint8_t boot_source;  ///< PARAM_BOOT_*
  uint8_t last_commit;  ///< PARAM_COMMIT_*
  uint16_t records_left;  ///< Full records that still fit before a boot-time erase
  uint16_t param_count;  ///< Parameters the firmware knows
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};
static_assert(offsetof(LlParamStoreStatus, records_left) == 3u,
              "LlParamStoreStatus.records_left offset drifted");
static_assert(offsetof(LlParamStoreStatus, param_count) == 5u,
              "LlParamStoreStatus.param_count offset drifted");

/**
 * @brief Blade motor status packet from STM32 (PACKET_ID_LL_BLADE_STATUS = 0x05).
 */
struct LlBladeStatus
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_BLADE_STATUS
  uint8_t is_active;  ///< 1=running, 0=stopped
  uint16_t rpm;  ///< Blade motor RPM
  uint16_t power_watts;  ///< ESC current [mA]; legacy field name, not watts
  float temperature;  ///< Blade/motor temperature [C]
  uint32_t error_count;  ///< Cumulative error counter
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Config request sent by the Pi (PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ = 0x11).
 *
 * Sent on every (re)connect and whenever the host needs to update runtime
 * config flags. The firmware replies with LlConfigRsp. Firmware that predates
 * the handshake ignores the unknown packet and never replies, which the bridge
 * reads (after a timeout) as an incompatible firmware that needs reflashing.
 */
struct LlConfigReq
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ
  uint8_t flags;  ///< Requested CONFIG_FLAG_* runtime bits
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

/**
 * @brief Config response from the STM32 (PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP = 0x12).
 *
 * Reports the firmware's wire-protocol version (the compatibility key checked
 * against kMowgliProtocolVersion), the currently-active runtime config flags,
 * and its human-readable firmware semver.
 */
struct LlConfigRsp
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP
  uint8_t protocol_version;  ///< MOWGLI_PROTOCOL_VERSION on the firmware
  uint8_t active_flags;  ///< Active CONFIG_FLAG_* runtime bits
  uint8_t fw_version_major;  ///< Firmware semver major
  uint8_t fw_version_minor;  ///< Firmware semver minor
  uint8_t fw_version_patch;  ///< Firmware semver patch
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
static_assert(sizeof(LlResetCause) == 5u, "LlResetCause layout mismatch");
static_assert(sizeof(LlHeartbeat) == 5u, "LlHeartbeat layout mismatch");
static_assert(sizeof(LlHighLevelState) == 5u, "LlHighLevelState layout mismatch");
static_assert(sizeof(LlCmdVel) == 11u, "LlCmdVel layout mismatch");
static_assert(sizeof(LlCmdBlade) == 5u, "LlCmdBlade layout mismatch");
static_assert(sizeof(LlBladeStatus) == 16u, "LlBladeStatus layout mismatch");
static_assert(sizeof(LlConfigReq) == 4u, "LlConfigReq layout mismatch");
static_assert(sizeof(LlConfigRsp) == 8u, "LlConfigRsp layout mismatch");
static_assert(sizeof(LlSetParam) == 9u, "LlSetParam layout mismatch");
static_assert(sizeof(LlGetParam) == 5u, "LlGetParam layout mismatch");
static_assert(sizeof(LlParamCommit) == 4u, "LlParamCommit layout mismatch");
static_assert(sizeof(LlParamValue) == 23u, "LlParamValue layout mismatch");
static_assert(sizeof(LlParamStoreStatus) == 9u, "LlParamStoreStatus layout mismatch");

}  // namespace mowgli_hardware
