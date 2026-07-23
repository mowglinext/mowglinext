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
//
// This is the COMPATIBILITY KEY: the bridge sends PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ
// on every (re)connect and the firmware answers with its own
// MOWGLI_PROTOCOL_VERSION in LlConfigRsp. If they differ — or the firmware is
// too old to answer at all — the image and firmware speak different wire
// formats and the operator must reflash. Bump this in lockstep with
// MOWGLI_PROTOCOL_VERSION in mowgli_protocol.h when an incompatible wire
// change requires a hard compatibility break.
static constexpr uint8_t kMowgliProtocolVersion = 6u;

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
  PACKET_ID_LL_HEARTBEAT = 0x42,  ///< Pi → STM32: heartbeat
  PACKET_ID_LL_HIGH_LEVEL_STATE = 0x43,  ///< Pi → STM32: high-level state
  PACKET_ID_LL_CMD_VEL = 0x50,  ///< Pi → STM32: velocity command (extension)
  PACKET_ID_LL_BLADE_STATUS = 0x05,  ///< STM32 → Pi: blade motor status
  PACKET_ID_LL_CMD_BLADE = 0x51,  ///< Pi → STM32: blade motor control
  PACKET_ID_LL_REBOOT = 0x52,  ///< Pi → STM32: reboot the board (NVIC_SystemReset)
  PACKET_ID_LL_SET_DRIVE_PID =
      0x54,  ///< Pi → STM32: drive-motor runtime tuning (PID/FF + ticks_per_meter)
  PACKET_ID_LL_SET_YAW_PID = 0x55,  ///< Pi → STM32: firmware yaw-rate loop tuning (Option C)
  PACKET_ID_LL_SET_KINEMATICS = 0x56,  ///< Pi → STM32: runtime max-speed cap + wheel base
  PACKET_ID_LL_SET_SAFETY_LIMITS = 0x57,  ///< Pi → STM32: runtime charge ceiling + e-stop timeouts
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
 * @brief Drive-motor runtime tuning packet sent by the Pi (PACKET_ID_LL_SET_DRIVE_PID = 0x54).
 *
 * Retunes the firmware's per-wheel velocity loop (both wheels share the gains)
 * and the runtime encoder scale without reflashing. The firmware
 * validates/clamps every field before applying and keeps its compile-time
 * defaults as the power-on fallback until the host reconnects.
 */
struct LlSetDrivePid
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_SET_DRIVE_PID
  float ticks_per_meter;  ///< Runtime encoder scale [ticks / m]
  float kp;  ///< Proportional gain [PWM per m/s]
  float ki;  ///< Integral gain [PWM per (m/s·s)]
  float kd;  ///< Derivative gain [PWM per (m/s²)]
  float integral_limit;  ///< Anti-windup clamp on the integral term [PWM]
  float pwm_per_mps;  ///< Open-loop feedforward velocity→PWM scale
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

static_assert(offsetof(LlSetDrivePid, type) == 0u, "LlSetDrivePid.type offset drifted");
static_assert(offsetof(LlSetDrivePid, ticks_per_meter) == 1u,
              "LlSetDrivePid.ticks_per_meter offset drifted");
static_assert(offsetof(LlSetDrivePid, kp) == 5u, "LlSetDrivePid.kp offset drifted");
static_assert(offsetof(LlSetDrivePid, ki) == 9u, "LlSetDrivePid.ki offset drifted");
static_assert(offsetof(LlSetDrivePid, kd) == 13u, "LlSetDrivePid.kd offset drifted");
static_assert(offsetof(LlSetDrivePid, integral_limit) == 17u,
              "LlSetDrivePid.integral_limit offset drifted");
static_assert(offsetof(LlSetDrivePid, pwm_per_mps) == 21u,
              "LlSetDrivePid.pwm_per_mps offset drifted");
static_assert(offsetof(LlSetDrivePid, crc) == 25u, "LlSetDrivePid.crc offset drifted");

/**
 * @brief Gyro yaw-rate loop tuning packet sent by the Pi (PACKET_ID_LL_SET_YAW_PID = 0x55).
 *
 * Retunes the firmware's closed yaw-rate loop (Option C, task #33/#34): at
 * the 50 Hz motor cadence the firmware regulates (commanded wz − measured
 * gyro wz) and injects a SYMMETRIC differential velocity trim (clamped to
 * ±trim_limit_mps) onto the per-wheel setpoints before the per-wheel PIs.
 * This is a SEPARATE packet from LlSetDrivePid — it does not extend it.
 * The firmware validates/clamps every field before applying.
 *
 * gyro_bias_radps (protocol v6): the host-measured mean at-rest gyro-Z offset
 * (imu_cal_offset_gz_, raw sensor frame — the same value the host subtracts
 * before its /imu publish). The firmware subtracts it before the gyro_sign
 * multiply so open-loop moves (BackUp: wz=0) hold a straight line instead of
 * tracing the bias as a constant-radius arc.
 */
struct LlSetYawPid
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_SET_YAW_PID
  float yaw_kp;  ///< P gain [m/s trim per rad/s yaw error]
  float yaw_ki;  ///< I gain [m/s trim per (rad/s·s)]
  float trim_limit_mps;  ///< Clamp on |differential trim| [m/s]
  uint8_t enabled;  ///< 1 = closed yaw loop on, 0 = open-diff passthrough
  int8_t gyro_sign;  ///< +1 / -1: gyro Z sign vs robot +yaw (CCW) — field sign-check remedy
  float gyro_bias_radps;  ///< mean at-rest gyro-Z bias [rad/s], raw sensor frame (v6)
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

static_assert(offsetof(LlSetYawPid, type) == 0u, "LlSetYawPid.type offset drifted");
static_assert(offsetof(LlSetYawPid, yaw_kp) == 1u, "LlSetYawPid.yaw_kp offset drifted");
static_assert(offsetof(LlSetYawPid, yaw_ki) == 5u, "LlSetYawPid.yaw_ki offset drifted");
static_assert(offsetof(LlSetYawPid, trim_limit_mps) == 9u,
              "LlSetYawPid.trim_limit_mps offset drifted");
static_assert(offsetof(LlSetYawPid, enabled) == 13u, "LlSetYawPid.enabled offset drifted");
static_assert(offsetof(LlSetYawPid, gyro_sign) == 14u, "LlSetYawPid.gyro_sign offset drifted");
static_assert(offsetof(LlSetYawPid, gyro_bias_radps) == 15u,
              "LlSetYawPid.gyro_bias_radps offset drifted");
static_assert(offsetof(LlSetYawPid, crc) == 19u, "LlSetYawPid.crc offset drifted");

/**
 * @brief Runtime kinematics packet sent by the Pi (PACKET_ID_LL_SET_KINEMATICS = 0x56).
 *
 * Retunes the runtime max wheel-speed cap and wheel base without a reflash. The
 * firmware clamps max_mps to at most its compile-time MAX_MPS (the wire can only
 * LOWER the motion cap, never raise it above the compiled safety ceiling) and
 * wheel_base to a sane range; the compile-time MAX_MPS/WHEEL_BASE remain the
 * power-on fallback. The firmware validates/clamps every field before applying.
 */
struct LlSetKinematics
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_SET_KINEMATICS
  float max_mps;  ///< Runtime max wheel speed cap [m/s]; clamped ≤ firmware MAX_MPS
  float wheel_base;  ///< Wheel track (centre-to-centre) [m]
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

static_assert(offsetof(LlSetKinematics, type) == 0u, "LlSetKinematics.type offset drifted");
static_assert(offsetof(LlSetKinematics, max_mps) == 1u, "LlSetKinematics.max_mps offset drifted");
static_assert(offsetof(LlSetKinematics, wheel_base) == 5u,
              "LlSetKinematics.wheel_base offset drifted");
static_assert(offsetof(LlSetKinematics, crc) == 9u, "LlSetKinematics.crc offset drifted");

/**
 * @brief Runtime safety-limits packet sent by the Pi (PACKET_ID_LL_SET_SAFETY_LIMITS = 0x57).
 *
 * Retunes the battery charge ceiling + emergency-sensor timeouts without a
 * reflash. The firmware clamps EVERY field so the wire can only make protection
 * STRONGER, never weaker: charge V/I clamped ≤ compiled ceiling; the four trip
 * timeouts clamped ≤ compiled (faster e-stop); play_clear_ms clamped ≥ compiled
 * (harder to un-latch). Compile-time board_defaults.h values stay the power-on
 * fallback. The firmware validates/clamps every field before applying.
 */
struct LlSetSafetyLimits
{
  uint8_t type;  ///< Must equal PACKET_ID_LL_SET_SAFETY_LIMITS
  float max_charge_voltage;  ///< Charge voltage ceiling [V]; clamped ≤ compiled
  float max_charge_current;  ///< Charge current ceiling [A]; clamped ≤ compiled
  uint16_t one_wheel_lift_ms;  ///< One-wheel-lift trip [ms]; clamped ≤ compiled
  uint16_t both_wheels_lift_ms;  ///< Both-wheels-lift trip [ms]; clamped ≤ compiled
  uint16_t tilt_ms;  ///< Tilt trip [ms]; clamped ≤ compiled
  uint16_t stop_button_ms;  ///< Stop-button trip [ms]; clamped ≤ compiled
  uint16_t play_clear_ms;  ///< Hold-to-clear-emergency [ms]; clamped ≥ compiled
  uint16_t crc;  ///< CRC-16 CCITT over all preceding bytes
};

static_assert(offsetof(LlSetSafetyLimits, type) == 0u, "LlSetSafetyLimits.type offset drifted");
static_assert(offsetof(LlSetSafetyLimits, max_charge_voltage) == 1u,
              "LlSetSafetyLimits.max_charge_voltage offset drifted");
static_assert(offsetof(LlSetSafetyLimits, max_charge_current) == 5u,
              "LlSetSafetyLimits.max_charge_current offset drifted");
static_assert(offsetof(LlSetSafetyLimits, one_wheel_lift_ms) == 9u,
              "LlSetSafetyLimits.one_wheel_lift_ms offset drifted");
static_assert(offsetof(LlSetSafetyLimits, both_wheels_lift_ms) == 11u,
              "LlSetSafetyLimits.both_wheels_lift_ms offset drifted");
static_assert(offsetof(LlSetSafetyLimits, tilt_ms) == 13u,
              "LlSetSafetyLimits.tilt_ms offset drifted");
static_assert(offsetof(LlSetSafetyLimits, stop_button_ms) == 15u,
              "LlSetSafetyLimits.stop_button_ms offset drifted");
static_assert(offsetof(LlSetSafetyLimits, play_clear_ms) == 17u,
              "LlSetSafetyLimits.play_clear_ms offset drifted");
static_assert(offsetof(LlSetSafetyLimits, crc) == 19u, "LlSetSafetyLimits.crc offset drifted");

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
static_assert(sizeof(LlSetDrivePid) == 27u, "LlSetDrivePid layout mismatch");
static_assert(sizeof(LlSetYawPid) == 21u, "LlSetYawPid layout mismatch");
static_assert(sizeof(LlSetKinematics) == 11u, "LlSetKinematics layout mismatch");
static_assert(sizeof(LlSetSafetyLimits) == 21u, "LlSetSafetyLimits layout mismatch");

}  // namespace mowgli_hardware
