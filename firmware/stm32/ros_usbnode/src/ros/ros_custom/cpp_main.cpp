/**
 ******************************************************************************
 * @file    cpp_main.cpp
 * @author  Georg Swoboda <cn@warp.at>
 * @date    21/09/2022
 * @version 2.0.0
 * @brief   COBS protocol bridge — replaces rosserial with direct COBS packets
 ******************************************************************************
 *
 * Migration from rosserial to COBS:
 *   - All ROS topic publish/subscribe removed
 *   - All ROS service servers/clients removed
 *   - ros::NodeHandle replaced by mowgli_comms COBS layer
 *   - USB CDC RX feeds mowgli_comms_process_rx() instead of ringbuffer
 *   - Packet send uses mowgli_comms_send_*() convenience wrappers
 *   - cmd_vel timeout uses HAL_GetTick() instead of nh.now()
 *
 ******************************************************************************
 */

#include "board.h"
#include "main.h"
#include "adc.h"

#include <cpp_main.h>
#include "panel.h"
#include "charger.h"
#include "emergency.h"
#include "drivemotor.h"
#include "blademotor.h"
#include "ultrasonic_sensor.h"
#include "stm32f_board_hal.h"
#include "nbt.h"

// USB CDC
#include "usbd_cdc_if.h"

// COBS protocol (replaces rosserial)
#include "mowgli_protocol.h"
#include "mowgli_comms.h"

// Math
#include <cmath>

// IMU
#include "imu/imu.h"

#ifdef OPTION_PERIMETER
#include "perimeter.h"
#endif

/* ---------------------------------------------------------------------------
 * Timer intervals
 * ---------------------------------------------------------------------------*/
/*
 * 10 ms (100 Hz) IMU rate. The MEMS sensor (LSM6DS33 / WT901 / LIS3MDL)
 * I2C read takes <1 ms, leaving plenty of CPU headroom in the 10 ms
 * window. At 115200 baud the additional ~50-byte IMU packet costs
 * ~4 ms wire time per second, so total serial usage stays comfortably
 * under the 11.5 KB/s budget alongside the existing 47 Hz odom and
 * 4 Hz status streams.
 */
#define IMU_NBT_TIME_MS    10
#define MOTORS_NBT_TIME_MS 20
#define STATUS_NBT_TIME_MS 250
#define PANEL_NBT_TIME_MS  100
#define LED_NBT_TIME_MS    1000
#define BLADE_NBT_TIME_MS  250

/* ---------------------------------------------------------------------------
 * Drive motor control state
 * ---------------------------------------------------------------------------
 * As of protocol v2 the per-wheel velocity PI loop lives on the HOST (see
 * wheel_rate_controller.hpp in the ROS 2 bridge), not here. The firmware is a
 * dumb PWM passthrough: on_cmd_pwm() stores the host-computed signed PWM in
 * these globals (ISR context) and motors_handler() forwards it to the PAC5210
 * at MOTORS_NBT_TIME_MS cadence, subject only to the emergency / cmd-watchdog
 * overrides. The diff-drive split, feedforward and closed-loop correction that
 * used to run here are gone — moving them to the host makes them tunable,
 * observable and auto-calibrating without reflashing. */
static volatile int16_t left_pwm_signed  = 0;
static volatile int16_t right_pwm_signed = 0;

/* ---------------------------------------------------------------------------
 * Blade motor control state
 * ---------------------------------------------------------------------------*/
static volatile uint8_t target_blade_on_off = 0;
static uint8_t blade_on_off        = 0;
/* volatile: written from the on_cmd_blade ISR, read in motors_handler. */
static volatile uint8_t blade_direction = 0;

/* ---------------------------------------------------------------------------
 * cmd_vel timeout tracking (replaces ros::Time)
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_cmd_vel_tick = 0;

/* ---------------------------------------------------------------------------
 * High-level state received from host
 * ---------------------------------------------------------------------------*/
static uint8_t hl_current_mode = 0;
static uint8_t hl_gps_quality  = 0;

/* ---------------------------------------------------------------------------
 * Heartbeat watchdog
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_heartbeat_tick   = 0;
#define HEARTBEAT_TIMEOUT_MS 2000u

/* ---------------------------------------------------------------------------
 * Reboot flag
 * ---------------------------------------------------------------------------*/
static bool reboot_flag = false;

/* ---------------------------------------------------------------------------
 * Non-blocking timers
 * ---------------------------------------------------------------------------*/
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t imu_nbt;
static nbt_t status_nbt;
static nbt_t led_nbt;
static nbt_t blade_nbt;

/* ---------------------------------------------------------------------------
 * Odometry timing
 * ---------------------------------------------------------------------------*/
static uint32_t last_odom_tick = 0;

/* Forward declarations */
static void update_blade_led(void);

/* ---------------------------------------------------------------------------
 * COBS packet handlers (Host -> Firmware)
 * ---------------------------------------------------------------------------*/

static void on_heartbeat(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_heartbeat_t) - 2u) {
        return;
    }

    const pkt_heartbeat_t *pkt = (const pkt_heartbeat_t *)data;

    last_heartbeat_tick = HAL_GetTick();

    if (pkt->emergency_requested) {
        Emergency_SetState(1);
    }
    if (pkt->emergency_release_requested) {
        /* Only clear emergency if no physical sensor is still asserted.
         * Firmware is the sole safety authority — never bypass hardware. */
        if (!Emergency_StopButtonYellow() && !Emergency_StopButtonWhite() &&
            !Emergency_WheelLiftBlue() && !Emergency_WheelLiftRed() &&
            !Emergency_Tilt() && !Emergency_LowZAccelerometer()) {
            Emergency_SetState(0);
        } else {
            debug_printf("emergency release rejected: physical sensor still active\r\n");
        }
    }
}

/* Host -> Firmware raw per-wheel PWM command (protocol v2). The host has
 * already done the diff-drive split and run the closed-loop wheel-velocity PI,
 * so all we do is latch the signed PWM and stamp the watchdog. DRIVEMOTOR_-
 * SetSpeedSigned saturates to ±255 when motors_handler applies it. */
static void on_cmd_pwm(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_cmd_pwm_t) - 2u) {
        return;
    }

    const pkt_cmd_pwm_t *pkt = (const pkt_cmd_pwm_t *)data;

    last_cmd_vel_tick = HAL_GetTick();

    if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
        /* Firmware safety gate: never drive the wheels while idle/docked,
         * regardless of what the host commands. */
        left_pwm_signed  = 0;
        right_pwm_signed = 0;
        return;
    }

    left_pwm_signed  = pkt->left_pwm;
    right_pwm_signed = pkt->right_pwm;
}

static void on_hl_state(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_hl_state_t) - 2u) {
        return;
    }

    const pkt_hl_state_t *pkt = (const pkt_hl_state_t *)data;

    hl_current_mode = pkt->current_mode;
    hl_gps_quality  = pkt->gps_quality;

    // Update panel LEDs based on mode
    if (hl_gps_quality < 90) {
        PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_OFF);
    } else {
        PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_ON);
    }

    // Map host mode to internal status for motor safety.
    // Constants defined in mowgli_protocol.h — keep in sync with HighLevelStatus.msg.
    switch (hl_current_mode) {
    case HL_MODE_AUTONOMOUS:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_ON);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_OFF);
        main_eOpenmowerStatus = OPENMOWER_STATUS_MOWING;
        break;
    case HL_MODE_RECORDING:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_ON);
        main_eOpenmowerStatus = OPENMOWER_STATUS_RECORD;
        break;
    case HL_MODE_MANUAL_MOWING:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_ON);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_ON);
        main_eOpenmowerStatus = OPENMOWER_STATUS_MOWING;
        break;
    case HL_MODE_NULL:
    case HL_MODE_IDLE:
    default:
        PANEL_Set_LED(PANEL_LED_S1, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_S2, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_4H, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_6H, PANEL_LED_OFF);
        PANEL_Set_LED(PANEL_LED_8H, PANEL_LED_OFF);
        main_eOpenmowerStatus = OPENMOWER_STATUS_IDLE;
        left_pwm_signed = right_pwm_signed = 0;
        blade_on_off = target_blade_on_off = 0;
        break;
    }

    update_blade_led();
}

static void on_cmd_blade(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_cmd_blade_t) - 2u) {
        return;
    }

    const pkt_cmd_blade_t *pkt = (const pkt_cmd_blade_t *)data;
    target_blade_on_off = pkt->blade_on;
    blade_direction = pkt->blade_dir;
}

/* Host -> Firmware reboot request. Sets reboot_flag so chatter_handler issues
 * NVIC_SystemReset on its next tick (lets the current packet/ISR unwind first).
 * Gated on the magic byte so a corrupt/misframed packet can't reset the board. */
static void on_reboot(const uint8_t *data, size_t len)
{
    if (len < sizeof(pkt_reboot_t) - 2u) {
        return;
    }
    const pkt_reboot_t *pkt = (const pkt_reboot_t *)data;
    if (pkt->magic == PKT_REBOOT_MAGIC) {
        debug_printf("reboot requested by host\r\n");
        reboot_flag = true;
    }
}

/* on_hl_state blade LED feedback (moved out of on_hl_state for clarity) */
static void update_blade_led(void)
{
    if (target_blade_on_off) {
        #ifdef PANEL_LED_2H
        if (BLADEMOTOR_bActivated) {
            PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_FLASH_SLOW);
        } else {
            PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_ON);
        }
        #endif
    } else {
        #ifdef PANEL_LED_2H
        PANEL_Set_LED(PANEL_LED_2H, PANEL_LED_OFF);
        #endif
    }
}

/* ---------------------------------------------------------------------------
 * USB CDC receive callback — feeds COBS layer
 * ---------------------------------------------------------------------------*/
uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len)
{
    mowgli_comms_process_rx(Buf, (size_t)len);
    return CDC_RX_DATA_HANDLED;
}

/* ---------------------------------------------------------------------------
 * usb_cdc_transmit — required by mowgli_comms.c
 * ---------------------------------------------------------------------------*/
void usb_cdc_transmit(const uint8_t *buf, size_t len)
{
    CDC_Transmit(buf, (uint32_t)len);
}

/* ---------------------------------------------------------------------------
 * LED blink + reboot handler (replaces chatter_handler)
 * ---------------------------------------------------------------------------*/
extern "C" void chatter_handler()
{
    if (NBT_handler(&led_nbt)) {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

        if (reboot_flag) {
            NVIC_SystemReset();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Drive & blade motors handler
 * ---------------------------------------------------------------------------*/
extern "C" void motors_handler()
{
    if (NBT_handler(&motors_nbt)) {
        /* Snapshot ISR-written variables under interrupt lock */
        __disable_irq();
        int16_t  snap_left_pwm     = left_pwm_signed;
        int16_t  snap_right_pwm    = right_pwm_signed;
        uint8_t  snap_target_blade = target_blade_on_off;
        uint8_t  snap_blade_dir    = blade_direction;
        uint32_t snap_heartbeat    = last_heartbeat_tick;
        uint32_t snap_cmd_vel      = last_cmd_vel_tick;
        __enable_irq();

        blade_on_off = snap_target_blade;

        /* --- safety overrides ---
         * Emergency, IDLE mode, or cmd watchdog timeout force a hard stop.
         * Otherwise the host-computed PWM (already through the host-side
         * wheel-velocity PI) is forwarded to the PAC5210 verbatim.
         *
         * The IDLE check is re-asserted HERE — not only in on_cmd_pwm /
         * on_hl_state — so the "never drive the wheels while idle/docked"
         * guarantee holds in the one place that actually drives the motors,
         * independent of the relative arrival order of CMD_PWM and HL_STATE
         * packets. main_eOpenmowerStatus is volatile (see main.h). */
        bool hard_stop = false;
        if (Emergency_State()) {
            hard_stop = true;
            blade_on_off = 0;
        } else if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
            hard_stop = true;
        } else {
            const uint32_t cmd_vel_age_ms = HAL_GetTick() - snap_cmd_vel;
            if (cmd_vel_age_ms > 200u) {
                /* Command watchdog: zero motors if the host hasn't sent a PWM
                 * command in 200 ms (Pi hang, USB glitch, host crash). This is
                 * the firmware-side backstop for a wedged host. */
                hard_stop = true;
            }
            if (cmd_vel_age_ms > 25000u) {
                blade_on_off = 0;
            }
        }

        if (hard_stop) {
            DRIVEMOTOR_SetSpeedSigned(0, 0);
        } else {
            DRIVEMOTOR_SetSpeedSigned(snap_left_pwm, snap_right_pwm);
        }

        // Heartbeat watchdog: if no heartbeat for HEARTBEAT_TIMEOUT_MS, emergency stop
        if (snap_heartbeat != 0 &&
            (HAL_GetTick() - snap_heartbeat) > HEARTBEAT_TIMEOUT_MS) {
            Emergency_SetState(1);
        }

        BLADEMOTOR_Set(blade_on_off, snap_blade_dir);
    }
}

/* ---------------------------------------------------------------------------
 * Panel handler — button presses generate UI events over COBS
 * ---------------------------------------------------------------------------*/
extern "C" void panel_handler()
{
    if (NBT_handler(&panel_nbt)) {
        PANEL_Tick();

        if (buttonupdated == 1 && buttoncleared == 0) {
            pkt_ui_event_t evt;
            evt.type = PKT_ID_UI_EVENT;
            evt.press_duration = 0;  // short press

            // Map physical buttons to IDs
            if (buttonstate[PANEL_BUTTON_DEF_S1]) {
                evt.button_id = 1;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_S2]) {
                evt.button_id = 2;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_LOCK]) {
                evt.button_id = 3;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_START]) {
                evt.button_id = 4;
                mowgli_comms_send(&evt, sizeof(evt));
            }
            if (buttonstate[PANEL_BUTTON_DEF_HOME]) {
                evt.button_id = 5;
                mowgli_comms_send(&evt, sizeof(evt));
            }

            buttonupdated = 0;
        }
    }
}

#if OPTION_ULTRASONIC == 1
extern "C" void ultrasonic_handler(void)
{
    // USS data is included in the status packet — no separate packet needed.
    // This handler is kept for the main loop call in main.c.
}
#endif

/* ---------------------------------------------------------------------------
 * Wheel ticks handler — called from DRIVEMOTOR_App_Rx() every 20 ms.
 *
 * Builds the odometry packet from signed cumulative ticks. Per-wheel
 * velocity is computed here (not on the Pi) because the firmware has the
 * hardware-timer-accurate dt. All four quantities in the packet are
 * signed; the host doesn't need a direction byte or to re-sign anything.
 * ---------------------------------------------------------------------------*/
extern "C" void wheelTicks_handler(
    int32_t  p_s32LeftTicksSigned,
    int32_t  p_s32RightTicksSigned,
    int16_t  p_s16LeftSpeed,   /* currently unused — reserved for future telemetry */
    int16_t  p_s16RightSpeed)
{
    (void)p_s16LeftSpeed;
    (void)p_s16RightSpeed;

    static int32_t prev_left_ticks  = 0;
    static int32_t prev_right_ticks = 0;

    const uint32_t now_tick = HAL_GetTick();
    const uint16_t dt_ms    = (uint16_t)(now_tick - last_odom_tick);
    last_odom_tick = now_tick;

    const int32_t delta_left  = p_s32LeftTicksSigned  - prev_left_ticks;
    const int32_t delta_right = p_s32RightTicksSigned - prev_right_ticks;
    prev_left_ticks  = p_s32LeftTicksSigned;
    prev_right_ticks = p_s32RightTicksSigned;

    /* Velocity: mm/s = (delta_ticks / TICKS_PER_M) * (1000 / dt_ms) * 1000
     *                = delta_ticks * 1e6 / (TICKS_PER_M * dt_ms).
     * TICKS_PER_M is 300, so the constant numerator (300 * dt_ms) stays
     * comfortably inside int32 for any realistic dt. We still cast to
     * int64 for the mul to be safe on large tick deltas.                  */
    int16_t left_v_mm_s  = 0;
    int16_t right_v_mm_s = 0;
    if (dt_ms > 0)
    {
        const int64_t denom = (int64_t)TICKS_PER_M * (int64_t)dt_ms;
        int64_t v_l = ((int64_t)delta_left  * 1000000LL) / denom;
        int64_t v_r = ((int64_t)delta_right * 1000000LL) / denom;
        if (v_l >  32767) v_l =  32767;
        if (v_l < -32768) v_l = -32768;
        if (v_r >  32767) v_r =  32767;
        if (v_r < -32768) v_r = -32768;
        left_v_mm_s  = (int16_t)v_l;
        right_v_mm_s = (int16_t)v_r;
    }

    pkt_odometry_t odom;
    odom.type                 = PKT_ID_ODOMETRY;
    odom.dt_millis            = dt_ms;
    odom.left_ticks           = p_s32LeftTicksSigned;
    odom.right_ticks          = p_s32RightTicksSigned;
    odom.left_velocity_mm_s   = left_v_mm_s;
    odom.right_velocity_mm_s  = right_v_mm_s;

    mowgli_comms_send_odometry(&odom);
}

/* ---------------------------------------------------------------------------
 * IMU + status broadcast handler
 * ---------------------------------------------------------------------------*/
extern "C" void broadcast_handler()
{
    if (NBT_handler(&imu_nbt)) {
        pkt_imu_t imu_pkt;
        imu_pkt.type = PKT_ID_IMU;

        static uint32_t last_imu_tick = 0;
        uint32_t now_tick = HAL_GetTick();
        imu_pkt.dt_millis = (uint16_t)(now_tick - last_imu_tick);
        last_imu_tick = now_tick;

#ifdef EXTERNAL_IMU_ACCELERATION
        float ax, ay, az;
        IMU_ReadAccelerometer(&ax, &ay, &az);
        imu_pkt.acceleration_mss[0] = ax;
        imu_pkt.acceleration_mss[1] = ay;
        imu_pkt.acceleration_mss[2] = az;
#else
        imu_pkt.acceleration_mss[0] = 0.0f;
        imu_pkt.acceleration_mss[1] = 0.0f;
        imu_pkt.acceleration_mss[2] = 0.0f;
#endif

#ifdef EXTERNAL_IMU_ANGULAR
        float gx, gy, gz;
        IMU_ReadGyro(&gx, &gy, &gz);
        imu_pkt.gyro_rads[0] = gx;
        imu_pkt.gyro_rads[1] = gy;
        imu_pkt.gyro_rads[2] = gz;
#else
        imu_pkt.gyro_rads[0] = 0.0f;
        imu_pkt.gyro_rads[1] = 0.0f;
        imu_pkt.gyro_rads[2] = 0.0f;
#endif

        // Magnetometer — uses generic IMU_ReadMag (works with any IMU that has mag)
        IMU_ReadMag(&imu_pkt.mag_uT[0], &imu_pkt.mag_uT[1], &imu_pkt.mag_uT[2]);

        mowgli_comms_send_imu(&imu_pkt);
    }

    if (NBT_handler(&status_nbt)) {
        pkt_status_t status_pkt;
        status_pkt.type = PKT_ID_STATUS;

        // Build status bitmask
        uint8_t status_bits = STATUS_BIT_INITIALIZED | STATUS_BIT_RASPI_POWER;
        if (chargecontrol_is_charging) {
            status_bits |= STATUS_BIT_CHARGING;
        }
        if (RAIN_Sense()) {
            status_bits |= STATUS_BIT_RAIN;
        }
        // Sound and UI availability from panel
        status_bits |= STATUS_BIT_UI_AVAIL;
        status_pkt.status_bitmask = status_bits;

        // USS ranges — fill from ultrasonic sensors
        for (unsigned int i = 0; i < MOWGLI_USS_COUNT; i++) {
            status_pkt.uss_ranges_m[i] = 0.0f;
        }
#if OPTION_ULTRASONIC == 1
        status_pkt.uss_ranges_m[0] = (float)(ULTRASONICSENSOR_u32GetLeftDistance()) / 10000.0f;
        status_pkt.uss_ranges_m[1] = (float)(ULTRASONICSENSOR_u32GetRightDistance()) / 10000.0f;
#endif

        // Emergency bitmask
        uint8_t emergency_bits = 0u;
        if (Emergency_State()) {
            emergency_bits |= EMERGENCY_BIT_LATCH;
            if (Emergency_StopButtonYellow() || Emergency_StopButtonWhite()) {
                emergency_bits |= EMERGENCY_BIT_STOP;
            }
            if (Emergency_WheelLiftBlue() || Emergency_WheelLiftRed()) {
                emergency_bits |= EMERGENCY_BIT_LIFT;
            }
        }
        status_pkt.emergency_bitmask = emergency_bits;

        // Power
        status_pkt.v_charge         = charge_voltage;
        status_pkt.v_system         = battery_voltage;
        status_pkt.charging_current = current;
        status_pkt.batt_percentage  = 0;  // TODO: compute from voltage curve

        mowgli_comms_send_status(&status_pkt);
    }

    // Blade motor status (4 Hz) — only after system has initialized
    if (NBT_handler(&blade_nbt) && last_heartbeat_tick != 0u) {
        pkt_blade_status_t blade_pkt;
        memset(&blade_pkt, 0, sizeof(blade_pkt));
        blade_pkt.type        = PKT_ID_BLADE_STATUS;
        blade_pkt.is_active   = BLADEMOTOR_bActivated ? 1u : 0u;
        blade_pkt.rpm         = BLADEMOTOR_u16RPM;
        blade_pkt.power_watts = BLADEMOTOR_u16Power;
        blade_pkt.temperature = blade_temperature;
        blade_pkt.error_count = BLADEMOTOR_u32Error;
        mowgli_comms_send(&blade_pkt, sizeof(blade_pkt));
    }
}

/* ---------------------------------------------------------------------------
 * spinOnce — no-op (rosserial spin removed)
 * ---------------------------------------------------------------------------*/
extern "C" void spinOnce()
{
    // Nothing to do — COBS RX is handled in CDC_DataReceivedHandler().
    // This function is kept so main.c doesn't need modification.
}

/* ---------------------------------------------------------------------------
 * Initialisation (replaces init_ROS)
 * ---------------------------------------------------------------------------*/
extern "C" void init_ROS()
{
    // Initialise COBS comms layer
    mowgli_comms_init();

    // Register handlers for Host -> Firmware packets
    mowgli_comms_register_handler(PKT_ID_HEARTBEAT, on_heartbeat);
    mowgli_comms_register_handler(PKT_ID_CMD_PWM,   on_cmd_pwm);
    mowgli_comms_register_handler(PKT_ID_HL_STATE,  on_hl_state);
    mowgli_comms_register_handler(PKT_ID_CMD_BLADE, on_cmd_blade);
    mowgli_comms_register_handler(PKT_ID_REBOOT,    on_reboot);

    // Initialise timers
    NBT_init(&led_nbt,     LED_NBT_TIME_MS);
    NBT_init(&panel_nbt,   PANEL_NBT_TIME_MS);
    NBT_init(&status_nbt,  STATUS_NBT_TIME_MS);
    NBT_init(&imu_nbt,     IMU_NBT_TIME_MS);
    NBT_init(&motors_nbt,  MOTORS_NBT_TIME_MS);
    NBT_init(&blade_nbt,   BLADE_NBT_TIME_MS);

    last_odom_tick      = HAL_GetTick();
    last_heartbeat_tick = 0;
    last_cmd_vel_tick   = 0;
}
