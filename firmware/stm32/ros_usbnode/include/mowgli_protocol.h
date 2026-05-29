/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file mowgli_protocol.h
 * @brief Shared wire-protocol definitions for the Mowgli STM32 firmware.
 *
 * This header is the single source of truth for packet IDs, status/emergency
 * bitmasks, and packed wire-format structs. It is intentionally written in
 * plain C99 so it can be included by both the STM32 firmware (C) and the
 * ROS 2 bridge (C++ includes it via extern "C" guards or direct inclusion).
 *
 * Every struct is declared with #pragma pack(push,1) / #pragma pack(pop) to
 * guarantee no compiler-inserted padding. Fields are little-endian (native on
 * both STM32 Cortex-M3 and x86/ARM64 Linux).
 *
 * Wire frame format (per packet):
 *   [0x00] [COBS-encoded payload] [0x00]
 *   payload = packed struct bytes + CRC-16 CCITT (appended as last 2 bytes)
 *
 * IMPORTANT: Keep this header in sync with ll_datatypes.hpp on the ROS 2 side.
 * The struct layouts and packet IDs MUST be identical on both ends.
 */

#ifndef MOWGLI_PROTOCOL_H
#define MOWGLI_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Protocol version — increment when wire format changes incompatibly.
 * ---------------------------------------------------------------------------*/

#define MOWGLI_PROTOCOL_VERSION  1u

/* ---------------------------------------------------------------------------
 * Packet IDs
 * Firmware -> Host (STM32 -> Raspberry Pi)
 * ---------------------------------------------------------------------------*/

/** System status packet (LlStatus / pkt_status_t). */
#define PKT_ID_STATUS      0x01u

/** IMU data packet (LlImu / pkt_imu_t). */
#define PKT_ID_IMU         0x02u

/** UI button event packet (LlUiEvent / pkt_ui_event_t). */
#define PKT_ID_UI_EVENT    0x03u

/** Wheel odometry packet (LlOdometry / pkt_odometry_t). */
#define PKT_ID_ODOMETRY    0x04u

/** Blade motor status packet (pkt_blade_status_t). */
#define PKT_ID_BLADE_STATUS 0x05u

/** High-level config response packet. */
#define PKT_ID_CONFIG_RSP  0x12u

/* ---------------------------------------------------------------------------
 * Packet IDs
 * Host -> Firmware (Raspberry Pi -> STM32)
 * ---------------------------------------------------------------------------*/

/** High-level config request packet. */
#define PKT_ID_CONFIG_REQ  0x11u

/**
 * Heartbeat packet.
 * Must be sent at regular intervals (~250 ms). STM32 declares an emergency
 * stop if the heartbeat is absent for more than its timeout window.
 */
#define PKT_ID_HEARTBEAT   0x42u

/** High-level state packet (mode + GPS quality). */
#define PKT_ID_HL_STATE    0x43u

/** Velocity command packet (forward + angular velocity). */
#define PKT_ID_CMD_VEL     0x50u

/** Blade motor control packet (on/off + direction). */
#define PKT_ID_CMD_BLADE   0x51u

/** Reboot request (Host -> Firmware). Triggers NVIC_SystemReset when the
 *  payload magic byte matches PKT_REBOOT_MAGIC — recovers a wedged board
 *  (e.g. IMU emitting NaN) without a manual power-cycle. */
#define PKT_ID_REBOOT      0x52u
#define PKT_REBOOT_MAGIC   0xB0u

/* ---------------------------------------------------------------------------
 * status_bitmask bit definitions  (pkt_status_t::status_bitmask)
 * ---------------------------------------------------------------------------*/

/** Firmware has completed its initialisation sequence. */
#define STATUS_BIT_INITIALIZED  (1u << 0u)

/** Raspberry Pi power rail is active. */
#define STATUS_BIT_RASPI_POWER  (1u << 1u)

/** Charging is currently active. */
#define STATUS_BIT_CHARGING     (1u << 2u)

/* Bit 3 is reserved. */

/** Rain sensor has detected moisture. */
#define STATUS_BIT_RAIN         (1u << 4u)

/** Sound hardware is present and available. */
#define STATUS_BIT_SOUND_AVAIL  (1u << 5u)

/** Sound hardware is currently playing audio. */
#define STATUS_BIT_SOUND_BUSY   (1u << 6u)

/** UI panel hardware is present and available. */
#define STATUS_BIT_UI_AVAIL     (1u << 7u)

/* ---------------------------------------------------------------------------
 * emergency_bitmask bit definitions  (pkt_status_t::emergency_bitmask)
 * ---------------------------------------------------------------------------*/

/** Emergency condition is latched (requires explicit release). */
#define EMERGENCY_BIT_LATCH  (1u << 0u)

/** Stop-button emergency is active. */
#define EMERGENCY_BIT_STOP   (1u << 1u)

/** Wheel-lift emergency is active. */
#define EMERGENCY_BIT_LIFT   (1u << 2u)

/* ---------------------------------------------------------------------------
 * USS sensor count
 * ---------------------------------------------------------------------------*/

/** Number of ultrasonic range sensors reported in pkt_status_t. */
#define MOWGLI_USS_COUNT  5u

/* ---------------------------------------------------------------------------
 * Packed wire-format structs
 *
 * All structs begin with a uint8_t 'type' field that holds the PKT_ID_*
 * constant for that message, followed by payload fields, and end with a
 * uint16_t 'crc' field that holds the CRC-16 CCITT computed over all bytes
 * that precede it (i.e. from 'type' up to but not including 'crc').
 * ---------------------------------------------------------------------------*/

#pragma pack(push, 1)

/**
 * @brief System status packet — Firmware -> Host (PKT_ID_STATUS = 0x01).
 *
 * Sent periodically by the STM32 at approximately 25 Hz. Contains battery
 * voltages, ultrasonic ranges, charging state, and emergency flags.
 *
 * Wire size: 36 bytes (must match sizeof(LlStatus) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;                          /**< PKT_ID_STATUS */
    uint8_t  status_bitmask;                /**< See STATUS_BIT_* defines */
    float    uss_ranges_m[MOWGLI_USS_COUNT];/**< Ultrasonic ranges [m] */
    uint8_t  emergency_bitmask;             /**< See EMERGENCY_BIT_* defines */
    float    v_charge;                      /**< Charge input voltage [V] */
    float    v_system;                      /**< Battery / system voltage [V] */
    float    charging_current;              /**< Charging current [A] */
    uint8_t  batt_percentage;               /**< Battery state of charge [0-100] */
    uint16_t crc;                           /**< CRC-16 CCITT over preceding bytes */
} pkt_status_t;

/**
 * @brief IMU data packet — Firmware -> Host (PKT_ID_IMU = 0x02).
 *
 * Sent at the IMU sample rate (typically 50-100 Hz).
 *
 * Wire size: 40 bytes (must match sizeof(LlImu) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;                  /**< PKT_ID_IMU */
    uint16_t dt_millis;             /**< Time delta since previous packet [ms] */
    float    acceleration_mss[3];   /**< Linear acceleration x/y/z [m/s^2] */
    float    gyro_rads[3];          /**< Angular velocity x/y/z [rad/s] */
    float    mag_uT[3];             /**< Magnetic field x/y/z [uT] */
    uint16_t crc;                   /**< CRC-16 CCITT over preceding bytes */
} pkt_imu_t;

/**
 * @brief UI button event packet — Firmware -> Host (PKT_ID_UI_EVENT = 0x03).
 *
 * Sent once per user interaction on the panel.
 *
 * Wire size: 5 bytes (must match sizeof(LlUiEvent) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;           /**< PKT_ID_UI_EVENT */
    uint8_t  button_id;      /**< Panel button identifier */
    uint8_t  press_duration; /**< 0 = short, 1 = long, 2 = very long */
    uint16_t crc;            /**< CRC-16 CCITT over preceding bytes */
} pkt_ui_event_t;

/**
 * @brief Wheel odometry packet — Firmware -> Host (PKT_ID_ODOMETRY = 0x04).
 *
 * Sent every 20 ms when the drive motor controller responds with encoder data.
 *
 * Signed, self-contained representation: the encoder tick counters are
 * signed cumulative counts (polarity = direction), and per-wheel velocity
 * is computed firmware-side using the hardware-timer-accurate dt so the
 * host doesn't have to divide by a jittery USB-arrival interval.
 *
 * Wire size: 17 bytes (must match sizeof(LlOdometry) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;                 /**< PKT_ID_ODOMETRY */
    uint16_t dt_millis;            /**< Firmware-measured interval since last packet [ms] */
    int32_t  left_ticks;           /**< Signed cumulative left encoder ticks */
    int32_t  right_ticks;          /**< Signed cumulative right encoder ticks */
    int16_t  left_velocity_mm_s;   /**< Signed left wheel velocity [mm/s] */
    int16_t  right_velocity_mm_s;  /**< Signed right wheel velocity [mm/s] */
    uint16_t crc;                  /**< CRC-16 CCITT over preceding bytes */
} pkt_odometry_t;

/**
 * @brief Heartbeat packet — Host -> Firmware (PKT_ID_HEARTBEAT = 0x42).
 *
 * The host must send this at least once every ~500 ms or the STM32 will
 * declare an emergency stop. The emergency_requested / emergency_release
 * fields allow the host to assert or release the emergency state
 * independently of the timeout watchdog.
 *
 * Wire size: 5 bytes (must match sizeof(LlHeartbeat) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;                        /**< PKT_ID_HEARTBEAT */
    uint8_t  emergency_requested;         /**< Non-zero: assert emergency stop */
    uint8_t  emergency_release_requested; /**< Non-zero: release latched emergency */
    uint16_t crc;                         /**< CRC-16 CCITT over preceding bytes */
} pkt_heartbeat_t;

/**
 * @brief High-level state packet — Host -> Firmware (PKT_ID_HL_STATE = 0x43).
 *
 * Informs the STM32 of the current operating mode and GPS fix quality so it
 * can make contextual decisions (e.g. UI feedback, docking speed limits).
 *
 * Wire size: 5 bytes (must match sizeof(LlHighLevelState) in ll_datatypes.hpp).
 */
/**
 * High-level operating modes sent by the ROS 2 host.
 * These MUST match the constants in mowgli_interfaces/msg/HighLevelStatus.msg.
 */
#define HL_MODE_NULL             0u   /**< Emergency or transitional */
#define HL_MODE_IDLE             1u   /**< Idle, docked, charging */
#define HL_MODE_AUTONOMOUS       2u   /**< Autonomous mowing */
#define HL_MODE_RECORDING        3u   /**< Area boundary recording */
#define HL_MODE_MANUAL_MOWING    4u   /**< Manual teleop with blade */

typedef struct {
    uint8_t  type;         /**< PKT_ID_HL_STATE */
    uint8_t  current_mode; /**< High-level operating mode (HL_MODE_*) */
    uint8_t  gps_quality;  /**< GPS fix quality [0-100] */
    uint16_t crc;          /**< CRC-16 CCITT over preceding bytes */
} pkt_hl_state_t;

/**
 * @brief Velocity command packet — Host -> Firmware (PKT_ID_CMD_VEL = 0x50).
 *
 * Bridges geometry_msgs/Twist from the ROS 2 navigation stack into the
 * firmware's differential-drive motion controller.
 *
 * Wire size: 11 bytes (must match sizeof(LlCmdVel) in ll_datatypes.hpp).
 */
typedef struct {
    uint8_t  type;      /**< PKT_ID_CMD_VEL */
    float    linear_x;  /**< Forward velocity [m/s] */
    float    angular_z; /**< Yaw (angular) velocity [rad/s] */
    uint16_t crc;       /**< CRC-16 CCITT over preceding bytes */
} pkt_cmd_vel_t;

/**
 * @brief Blade motor control packet — Host -> Firmware (PKT_ID_CMD_BLADE = 0x51).
 *
 * Commands the blade motor on/off and direction.
 *
 * Wire size: 5 bytes.
 */
typedef struct {
    uint8_t  type;      /**< PKT_ID_CMD_BLADE */
    uint8_t  blade_on;  /**< 1=start blade, 0=stop blade */
    uint8_t  blade_dir; /**< 0=normal, 1=reverse */
    uint16_t crc;       /**< CRC-16 CCITT over preceding bytes */
} pkt_cmd_blade_t;

/**
 * @brief Reboot request packet — Host -> Firmware (PKT_ID_REBOOT = 0x52).
 *
 * Triggers NVIC_SystemReset, but only when magic == PKT_REBOOT_MAGIC, so a
 * corrupt/misframed packet cannot accidentally reset the board.
 *
 * Wire size: 4 bytes.
 */
typedef struct {
    uint8_t  type;   /**< PKT_ID_REBOOT */
    uint8_t  magic;  /**< Must equal PKT_REBOOT_MAGIC (0xB0) */
    uint16_t crc;    /**< CRC-16 CCITT over preceding bytes */
} pkt_reboot_t;

/**
 * @brief Blade motor status packet — Firmware -> Host (PKT_ID_BLADE_STATUS = 0x05).
 *
 * Sent periodically (~4 Hz) with blade motor telemetry.
 *
 * Wire size: 16 bytes.
 */
typedef struct {
    uint8_t  type;           /**< PKT_ID_BLADE_STATUS */
    uint8_t  is_active;      /**< 1=running, 0=stopped */
    uint16_t rpm;            /**< Blade motor RPM */
    uint16_t power_watts;    /**< Power consumption [W] */
    float    temperature;    /**< Blade/motor temperature [C] */
    uint32_t error_count;    /**< Cumulative error counter */
    uint16_t crc;            /**< CRC-16 CCITT over preceding bytes */
} pkt_blade_status_t;

#pragma pack(pop)

/* ---------------------------------------------------------------------------
 * Compile-time layout verification
 *
 * Computed sizes (all fields packed, no padding):
 *
 *   pkt_status_t:
 *     type(1) + status_bitmask(1) + uss_ranges_m[5](20) +
 *     emergency_bitmask(1) + v_charge(4) + v_system(4) +
 *     charging_current(4) + batt_percentage(1) + crc(2) = 38
 *
 *   pkt_imu_t:
 *     type(1) + dt_millis(2) + acceleration_mss[3](12) +
 *     gyro_rads[3](12) + mag_uT[3](12) + crc(2) = 41
 *
 *   pkt_ui_event_t:
 *     type(1) + button_id(1) + press_duration(1) + crc(2) = 5
 *
 *   pkt_odometry_t:
 *     type(1) + dt_millis(2) + left_ticks(4) + right_ticks(4) +
 *     left_velocity_mm_s(2) + right_velocity_mm_s(2) + crc(2) = 17
 *
 *   pkt_heartbeat_t:
 *     type(1) + emergency_requested(1) + emergency_release_requested(1) +
 *     crc(2) = 5
 *
 *   pkt_hl_state_t:
 *     type(1) + current_mode(1) + gps_quality(1) + crc(2) = 5
 *
 *   pkt_cmd_vel_t:
 *     type(1) + linear_x(4) + angular_z(4) + crc(2) = 11
 *
 * NOTE: The static_assert() values in ll_datatypes.hpp (36 and 40 for
 * LlStatus and LlImu respectively) appear to be incorrect — the field-by-
 * field sums above yield 38 and 41. The struct layouts here are the
 * authoritative wire-format definition; the ll_datatypes.hpp assert values
 * should be corrected to match.
 * ---------------------------------------------------------------------------*/

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(pkt_status_t)    == 38u, "pkt_status_t layout unexpected");
_Static_assert(sizeof(pkt_imu_t)       == 41u, "pkt_imu_t layout unexpected");
_Static_assert(sizeof(pkt_ui_event_t)  ==  5u, "pkt_ui_event_t layout unexpected");
_Static_assert(sizeof(pkt_odometry_t)  == 17u, "pkt_odometry_t layout unexpected");
_Static_assert(sizeof(pkt_heartbeat_t) ==  5u, "pkt_heartbeat_t layout unexpected");
_Static_assert(sizeof(pkt_hl_state_t)  ==  5u, "pkt_hl_state_t layout unexpected");
_Static_assert(sizeof(pkt_cmd_vel_t)   == 11u, "pkt_cmd_vel_t layout unexpected");
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MOWGLI_PROTOCOL_H */
