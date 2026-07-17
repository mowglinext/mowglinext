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

#include "adc.h"
#include "board.h"
#include "main.h"

#include "blademotor.h"
#include "charger.h"
#include "drivemotor.h"
#include "emergency.h"
#include "nbt.h"
#include "panel.h"
#include "pid.hpp"
#include "stm32f_board_hal.h"
#include "ultrasonic_sensor.h"
#include <cpp_main.h>

// USB CDC
#include "usbd_cdc_if.h"

// COBS protocol (replaces rosserial)
#include "mowgli_comms.h"
#include "mowgli_protocol.h"

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
#define IMU_NBT_TIME_MS 10
#define MOTORS_NBT_TIME_MS 20
#define STATUS_NBT_TIME_MS 250
#define PANEL_NBT_TIME_MS 100
#define LED_NBT_TIME_MS 1000
#define BLADE_NBT_TIME_MS 250

/* ---------------------------------------------------------------------------
 * Drive motor control state
 * ---------------------------------------------------------------------------*/
/* Target wheel velocities written by on_cmd_vel (ISR context) and read by
 * motors_handler() at MOTORS_NBT_TIME_MS cadence. Replaces the previous
 * "ISR writes PWM directly" path so motors_handler can run a wheel-level
 * PI loop using encoder feedback instead of forwarding an open-loop
 * cmd_vel × PWM_PER_MPS mapping. */
static volatile float left_target_mps = 0.0f;
static volatile float right_target_mps = 0.0f;

/* PWM ultimately sent to the PAC5210. Output of the PI loop in motors_handler;
 * legacy globals kept under the same names so the snapshot+watchdog logic
 * downstream is unchanged. */
static int16_t left_pwm_signed = 0;
static int16_t right_pwm_signed = 0;

/* ---------------------------------------------------------------------------
 * Wheel-level PI controller
 * ---------------------------------------------------------------------------
 * Brushed-DC motors driven by the PAC5210 have a hard static-friction
 * deadband (~PWM 40). Open-loop cmd_vel × PWM_PER_MPS produces PWM=2 for
 * a 0.05 m/s target — well below deadband, motors buzz, the chassis
 * doesn't move. We can't fix the motor physics but we CAN bridge the
 * deadband with closed-loop feedback: while the target says "move" and
 * the encoder says "stalled", the PI integrator ramps PWM up until the
 * motor breaks free, then settles around whatever PWM keeps the wheel
 * at target speed.
 *
 * Run at MOTORS_NBT_TIME_MS = 20 ms (50 Hz). Encoder feedback comes from
 * left_ticks_signed / right_ticks_signed (signed cumulative ticks
 * maintained by drivemotor.c, already direction-aware).
 *
 * Output = feedforward (target × PWM_PER_MPS, preserves the open-loop
 * behaviour above deadband) + Kp × error + integral_pwm. The integral is
 * stored pre-multiplied by Ki for trivial anti-windup.
 *
 * Set USE_WHEEL_PI to 0 to fall back to open-loop forwarding for
 * debugging / hardware bring-up. */
#define USE_WHEEL_PI 1
#define WHEEL_PI_KP_PWM_PER_MPS 30.0f /* proportional gain */
#define WHEEL_PI_KI_PWM_PER_MPS_S                                              \
  5000.0f /* integral gain (50 PWM in ~0.2 s when err=0.05 m/s) */
#define WHEEL_PI_INT_MAX_PWM                                                   \
  100.0f /* anti-windup clamp on the integral term                             \
          */
#define WHEEL_PI_DT_S (MOTORS_NBT_TIME_MS / 1000.0f)
/* Per-wheel velocity PI — battle-tested PX4 PID core (pid.hpp). Gains/limits
 * set once in init_ROS(). The integrator (kept inside the PID object) is what
 * bridges the static-friction deadband; output is the closed-loop PWM trim
 * added to the open-loop feedforward below. */
static PID left_wheel_pid;
static PID right_wheel_pid;
static int32_t prev_left_ticks_signed_pi = 0;
static int32_t prev_right_ticks_signed_pi = 0;
static float prev_left_target_mps = 0.0f;
static float prev_right_target_mps = 0.0f;

/* ---------------------------------------------------------------------------
 * Firmware-level anti-dig (task #46). ALWAYS active, in every mode, and
 * un-bypassable by any ROS controller: motors_handler drives the wheels for
 * docking, mowing AND transit alike (opennav_docking's cmd_vel arrives on the
 * same PKT_ID_CMD_VEL path), so cutting power here catches the "stuck wheels
 * keep spinning and dig holes in the lawn" failure that the FTC-level
 * anti-wheelspin (host, coverage-only) never sees. Per wheel: if it is
 * COMMANDED to move (|target| > MIN_TARGET) and the loop is actually PUSHING
 * (|PWM| > MIN_ABS_PWM — the PI integrator has wound past the deadband) but the
 * encoder logs < MIN_PROGRESS_M of real travel over WINDOW_MS, latch the wheel
 * OFF (force PWM 0) until the command clears. Only ever REDUCES output. */
#define ANTIDIG_WINDOW_MS 2500u       /* persistent stall before cut [ms] */
#define ANTIDIG_MIN_TARGET_MPS 0.02f  /* below this the wheel isn't "commanded" */
#define ANTIDIG_MIN_ABS_PWM 60        /* below this it isn't really "pushing" (deadband ~40) */
#define ANTIDIG_MIN_PROGRESS_M 0.03f  /* real travel over the window that counts as "moving" */
static uint32_t l_dig_stall_ms = 0u;
static uint32_t r_dig_stall_ms = 0u;
static int32_t l_dig_stall_ticks = 0;
static int32_t r_dig_stall_ticks = 0;
static bool l_dig_latched = false;
static bool r_dig_latched = false;

/* Open-loop feedforward velocity->PWM scale. Runtime-tunable copy of the
 * board.h PWM_PER_MPS default so the ROS 2 host can retune the drive loop via
 * PKT_ID_SET_DRIVE_PID without a reflash. Seeded with the compile-time default,
 * which therefore remains the power-on fallback (this board has no config
 * persistence; the bridge re-sends the gains on every reconnect). */
static volatile float g_pwm_per_mps = (float)PWM_PER_MPS;

/* ---------------------------------------------------------------------------
 * Gyro-local closed yaw-rate loop (Option C).
 *
 * The per-wheel PIs below regulate each wheel's SPEED independently, so the
 * chassis yaw rate was only their emergent difference — on soft/uneven turf the
 * actual yaw lagged/overshot the commanded wz (the weave). This loop closes a
 * real regulator on (commanded wz − measured gyro wz) at the 50 Hz motor
 * cadence and injects the correction as a SYMMETRIC differential velocity trim
 * (+right / −left), i.e. it rotates the robot WITHOUT changing mean forward
 * speed. Consequences of "symmetric": it never fights the host-side
 * anti-wheelspin forward-speed easing (that only scales vx), and it only shifts
 * the per-wheel PI setpoints (their deadband-bridging integrators still run).
 *
 * SAFETY: firmware stays the sole blade authority; this loop only shapes drive.
 * Fail-safe by construction — on gyro read failure, hard_stop, or disable, the
 * trim is 0 and the yaw integrator is reset, degrading to today's open-diff
 * behaviour. The trim is hard-clamped to ±g_yaw_trim_limit_mps (also the PID
 * output limit), so even a wrong gyro_sign (positive feedback) can only add a
 * bounded differential the per-wheel loops still cap at ±MAX_MPS — a bounded
 * veer, never an unbounded spin. Gains/sign/enable are runtime-tunable via
 * PKT_ID_SET_YAW_PID. */
#define YAW_PI_KP_DEFAULT 0.30f          /* m/s trim per rad/s yaw error */
#define YAW_PI_KI_DEFAULT 0.40f          /* m/s trim per (rad/s·s) */
#define YAW_TRIM_LIMIT_MPS_DEFAULT 0.15f /* clamp on |differential trim| [m/s] */
/* Integral term is clamped TIGHTER than the total trim (leaves headroom for the
 * P term and limits integral-driven overshoot/hunting). */
#define YAW_INT_LIMIT_FRAC 0.60f
/* Turn-exit anti-windup (task #37): when |commanded wz| falls from clearly
 * turning (> HI) to nearly straight (< LO) in one step, dump the integrator so
 * the wind-up built up through a U-turn doesn't drive a lingering differential
 * on the straight that follows (the reported post-U-turn wiggle). */
#define YAW_TURN_EXIT_HI_RADPS 0.40f
#define YAW_TURN_EXIT_LO_RADPS 0.15f
/* Low-speed authority scaling (task #37): below REF forward speed the yaw
 * correction (trim + a per-cycle integral leak) is scaled toward FLOOR so the
 * loop doesn't hunt on gyro noise when nearly stopped at the dock. The base
 * rotation still comes from the IK feedforward (snap targets), so an in-place
 * PRE_ROTATE pivot — commanded vx≈0 — is unaffected; only the closed-loop
 * correction is de-rated. On straights (vx ≥ REF) the scale is 1.0, so the
 * working straight-line weave rejection is untouched. */
#define YAW_LOWSPEED_REF_MPS 0.15f
#define YAW_LOWSPEED_FLOOR 0.25f
#define YAW_INT_LEAK_AT_FLOOR 0.90f /* per-cycle integral retain factor at FLOOR authority */
/* Anti-self-oscillation (task #39): the loop was self-exciting a ~2-4 Hz yaw
 * limit-cycle against the wheel deadband/stiction while the command was smooth
 * (~0.15 Hz). Two damping terms, both only REDUCE aggressiveness:
 *  - 1st-order IIR low-pass on the gyro feedback: alpha = dt / (RC + dt). At the
 *    50 Hz (dt=20 ms) motor rate, alpha=0.30 → RC≈47 ms → cutoff ≈ 3.4 Hz: it
 *    attenuates the 2-4 Hz self-oscillation the loop was chasing while adding
 *    little phase lag at the ~0.15 Hz of a real turn.
 *  - Slew-rate limit on the injected trim: caps |Δtrim| per 20 ms cycle so the
 *    output can't snap across the wheel deadband and re-excite the cycle. At
 *    0.03 m/s/cycle, full-scale (±0.15) takes ~5 cycles (100 ms) — invisible to a
 *    0.15 Hz command, fatal to a 2-4 Hz one. */
#define YAW_GYRO_LP_ALPHA 0.30f
#define YAW_TRIM_SLEW_MPS_PER_CYCLE 0.03f
static PID yaw_pid;
static float prev_yaw_trim_mps = 0.0f;
static float prev_applied_yaw_trim_mps = 0.0f; /* last trim actually injected (slew-limit state) */
static float yaw_gyro_filt = 0.0f;             /* low-passed gyro yaw rate [rad/s] */
static uint8_t yaw_gyro_filt_valid = 0u;       /* 0 until the IIR is seeded after a reset */
static float prev_cmd_wz = 0.0f;
static volatile float cmd_wz = 0.0f; /* commanded yaw rate [rad/s], set by on_cmd_vel */
static volatile uint8_t g_yaw_loop_enabled = 1;   /* default on for A/B */
static volatile float g_yaw_gyro_sign = 1.0f;     /* gyro Z sign vs robot +yaw */
static volatile float g_yaw_trim_limit_mps = YAW_TRIM_LIMIT_MPS_DEFAULT;

/* ---------------------------------------------------------------------------
 * Blade motor control state
 * ---------------------------------------------------------------------------*/
static volatile uint8_t target_blade_on_off = 0;
static uint8_t blade_on_off = 0;
static uint8_t blade_direction = 0;

/* ---------------------------------------------------------------------------
 * cmd_vel timeout tracking (replaces ros::Time)
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_cmd_vel_tick = 0;

/* ---------------------------------------------------------------------------
 * High-level state received from host
 * ---------------------------------------------------------------------------*/
static uint8_t hl_current_mode = 0;
static uint8_t hl_gps_quality = 0;

/* ---------------------------------------------------------------------------
 * Heartbeat watchdog
 * ---------------------------------------------------------------------------*/
static volatile uint32_t last_heartbeat_tick = 0;
#define HEARTBEAT_TIMEOUT_MS 2000u

/* True when the CURRENTLY latched emergency was raised SOLELY by the heartbeat
 * watchdog (host comms lost), with no physical safety sensor asserted. A pure
 * comms-loss latch is a fail-safe stop, not a physical hazard, so it is
 * auto-cleared when heartbeats resume (and no sensor is asserted) instead of
 * stranding the robot until a manual play-button / GUI reset. A physical
 * trigger (e-stop button, lift, tilt) clears this flag so its latch still
 * requires an explicit operator release. The blade stays cut throughout — it
 * only re-arms on an explicit CMD_BLADE after the emergency clears. */
static volatile bool heartbeat_only_latch = false;

/* Any physical safety sensor currently asserted? Firmware is the sole safety
 * authority; this gates every automatic emergency clear. */
static inline bool any_physical_emergency(void) {
  return Emergency_StopButtonYellow() || Emergency_StopButtonWhite() ||
         Emergency_WheelLiftBlue() || Emergency_WheelLiftRed() ||
         Emergency_Tilt() || Emergency_LowZAccelerometer();
}

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
 * Broadcast backpressure helpers
 * ---------------------------------------------------------------------------*/
static inline bool nbt_due(const nbt_t *nbt, const uint32_t now_tick) {
  return (now_tick - nbt->previousMillis) > nbt->timeout;
}

static inline void nbt_consume(nbt_t *nbt, const uint32_t now_tick) {
  nbt->previousMillis = now_tick;
}

static inline uint32_t usb_cdc_framed_packet_size(const size_t raw_packet_size) {
  /* All current packets are far below 254 bytes, so COBS adds at most one
   * overhead byte. Framed wire size = leading 0x00 + encoded payload + trailing
   * 0x00 <= raw + 3. */
  return (uint32_t)raw_packet_size + 3u;
}

static inline bool usb_cdc_can_queue_bytes(const uint32_t wire_bytes) {
  return CDC_TXQueue_GetWriteAvailable() >= wire_bytes;
}

static inline bool usb_cdc_should_send_telemetry_packet(
    const uint32_t wire_bytes) {
  if (!CDC_ShouldSendTelemetry()) {
    return false;
  }

  if (!usb_cdc_can_queue_bytes(wire_bytes)) {
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CDC_TX_QUEUE_FULL);
    return false;
  }

  return true;
}

/* ---------------------------------------------------------------------------
 * COBS packet handlers (Host -> Firmware)
 * ---------------------------------------------------------------------------*/

static void on_heartbeat(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_heartbeat_t) - 2u) {
    return;
  }

  const pkt_heartbeat_t *pkt = reinterpret_cast<const pkt_heartbeat_t *>(data);

  last_heartbeat_tick = HAL_GetTick();

  /* Comms restored. If the active emergency was a PURE comms-loss watchdog latch
   * (no physical trigger) and no sensor is asserted now, auto-clear it so a brief
   * host/USB stall doesn't strand the robot until a manual reset. A physical
   * trigger that appeared in the meantime asserts a sensor (handled below) and
   * clears heartbeat_only_latch, so this never auto-clears a physical e-stop. */
  if (heartbeat_only_latch && Emergency_State()) {
    if (!any_physical_emergency()) {
      Emergency_SetState(0);
      heartbeat_only_latch = false;
      debug_printf("heartbeat resumed: comms-loss emergency auto-cleared\r\n");
    } else {
      /* A physical sensor is now asserted — this is no longer a pure comms
       * latch; require an explicit operator release. */
      heartbeat_only_latch = false;
    }
  }

  if (pkt->emergency_requested) {
    heartbeat_only_latch = false;  /* host-commanded e-stop is not comms-loss */
    Emergency_SetState(1);
  }
  if (pkt->emergency_release_requested) {
    /* Only clear emergency if no physical sensor is still asserted.
     * Firmware is the sole safety authority — never bypass hardware. */
    if (!any_physical_emergency()) {
      Emergency_SetState(0);
      heartbeat_only_latch = false;
    } else {
      debug_printf(
          "emergency release rejected: physical sensor still active\r\n");
    }
  }
}

static void on_cmd_vel(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_cmd_vel_t) - 2u) {
    return;
  }

  const pkt_cmd_vel_t *pkt = reinterpret_cast<const pkt_cmd_vel_t *>(data);

  last_cmd_vel_tick = HAL_GetTick();

  if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
    return;
  }

  const float vx = pkt->linear_x;
  const float wz = pkt->angular_z;

  /* Commanded yaw rate for the firmware yaw-rate loop (Option C), read in the
   * motor timebase by motors_handler. Stored raw (pre-IK) so the loop tracks
   * the operator/Nav2 intent, not the clamped per-wheel reconstruction. */
  cmd_wz = wz;

  /* Differential-drive inverse kinematics — per-wheel linear speed. */
  float left_mps = vx - wz * WHEEL_BASE * 0.5f;
  float right_mps = vx + wz * WHEEL_BASE * 0.5f;

  if (left_mps > MAX_MPS)
    left_mps = MAX_MPS;
  if (left_mps < -MAX_MPS)
    left_mps = -MAX_MPS;
  if (right_mps > MAX_MPS)
    right_mps = MAX_MPS;
  if (right_mps < -MAX_MPS)
    right_mps = -MAX_MPS;

  /* Hand the target wheel velocities to the PI loop in motors_handler.
   * The mapping to PWM (feedforward + closed-loop correction) lives
   * there now so the integrator can bridge the static-friction
   * deadband on sub-deadband commands. */
  left_target_mps = left_mps;
  right_target_mps = right_mps;
}

static void on_set_drive_pid(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_set_drive_pid_t) - 2u) {
    return;
  }

  const pkt_set_drive_pid_t *pkt =
      reinterpret_cast<const pkt_set_drive_pid_t *>(data);

  /* Drive behaviour is safety-relevant: reject the whole packet if any field
   * is non-finite, then clamp each field to a safe range before applying so a
   * bad host value can never make the wheel loop diverge. Both wheels share
   * the gains; the output limit stays fixed at 255 PWM (motor controller max).
   * pid_constrain() comes from pid.hpp. */
  if (!std::isfinite(pkt->ticks_per_meter) || !std::isfinite(pkt->kp) ||
      !std::isfinite(pkt->ki) || !std::isfinite(pkt->kd) ||
      !std::isfinite(pkt->integral_limit) || !std::isfinite(pkt->pwm_per_mps)) {
    debug_printf("set_drive_pid rejected: non-finite field\r\n");
    return;
  }

  const float ticks_per_meter =
      pid_constrain(pkt->ticks_per_meter, 50.0f, 5000.0f);
  const float kp = pid_constrain(pkt->kp, 0.0f, 200.0f);
  const float ki = pid_constrain(pkt->ki, 0.0f, 20000.0f);
  const float kd = pid_constrain(pkt->kd, 0.0f, 500.0f);
  const float ilim = pid_constrain(pkt->integral_limit, 0.0f, 255.0f);
  const float ff = pid_constrain(pkt->pwm_per_mps, 50.0f, 600.0f);

  /* Apply atomically w.r.t. motors_handler(), which reads these objects in the
   * main loop at 50 Hz: this handler runs in USB RX interrupt context, and a
   * half-applied update (e.g. new gains but the old integral limit) could let
   * the ki integrator wind up unbounded for one cycle. Same __disable_irq
   * guard motors_handler uses for its setpoint snapshot. setOutputLimit is
   * re-asserted here so the ±255 clamp never silently depends on init_ROS
   * having run first (the PX4 PID default-inits _limit_output to 0). */
  __disable_irq();
  left_wheel_pid.setGains(kp, ki, kd);
  left_wheel_pid.setIntegralLimit(ilim);
  left_wheel_pid.setOutputLimit(255.0f);
  right_wheel_pid.setGains(kp, ki, kd);
  right_wheel_pid.setIntegralLimit(ilim);
  right_wheel_pid.setOutputLimit(255.0f);
  DRIVEMOTOR_SetTicksPerMeter(ticks_per_meter);
  g_pwm_per_mps = ff;
  __enable_irq();

  /* Do not log successful drive-PID updates here: this handler runs in the USB
   * RX path and hardware_bridge intentionally re-sends the packet in bursts
   * after reconnect. Printing each success over the debug UART can stall long
   * enough to starve the main loop and re-trigger the watchdog reboot loop. */
}

static void on_set_yaw_pid(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_set_yaw_pid_t) - 2u) {
    return;
  }

  const pkt_set_yaw_pid_t *pkt =
      reinterpret_cast<const pkt_set_yaw_pid_t *>(data);

  /* Yaw regulation is safety-relevant (it steers the chassis): reject the whole
   * packet if any gain/limit is non-finite, then clamp before applying so a bad
   * host value can never make the yaw loop diverge. */
  if (!std::isfinite(pkt->yaw_kp) || !std::isfinite(pkt->yaw_ki) ||
      !std::isfinite(pkt->trim_limit_mps)) {
    debug_printf("set_yaw_pid rejected: non-finite field\r\n");
    return;
  }

  const float kp = pid_constrain(pkt->yaw_kp, 0.0f, 5.0f);
  const float ki = pid_constrain(pkt->yaw_ki, 0.0f, 20.0f);
  const float tl = pid_constrain(pkt->trim_limit_mps, 0.0f, (float)MAX_MPS);
  const uint8_t en = (pkt->enabled != 0) ? 1u : 0u;
  const float sign = (pkt->gyro_sign < 0) ? -1.0f : 1.0f;

  /* Apply atomically w.r.t. motors_handler() (reads these at 50 Hz); this
   * handler runs in USB RX interrupt context. The integral limit is pinned to
   * the trim limit so the integrator alone can never exceed the differential
   * clamp. Same __disable_irq guard as on_set_drive_pid. */
  __disable_irq();
  yaw_pid.setGains(kp, ki, 0.0f);
  yaw_pid.setIntegralLimit(tl * YAW_INT_LIMIT_FRAC);
  yaw_pid.setOutputLimit(tl);
  g_yaw_trim_limit_mps = tl;
  g_yaw_gyro_sign = sign;
  g_yaw_loop_enabled = en;
  __enable_irq();
}

static void on_hl_state(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_hl_state_t) - 2u) {
    return;
  }

  const pkt_hl_state_t *pkt = reinterpret_cast<const pkt_hl_state_t *>(data);

  hl_current_mode = pkt->current_mode;
  hl_gps_quality = pkt->gps_quality;

  // Update panel LEDs based on mode
  if (hl_gps_quality < 90) {
    PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_OFF);
  } else {
    PANEL_Set_LED(PANEL_LED_LOCK, PANEL_LED_ON);
  }

  // Map host mode to internal status for motor safety.
  // Constants defined in mowgli_protocol.h — keep in sync with
  // HighLevelStatus.msg.
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
    left_target_mps = right_target_mps = 0.0f;
    cmd_wz = 0.0f;
    blade_on_off = target_blade_on_off = 0;
    break;
  }

  update_blade_led();
}

static void on_cmd_blade(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_cmd_blade_t) - 2u) {
    return;
  }

  const pkt_cmd_blade_t *pkt = reinterpret_cast<const pkt_cmd_blade_t *>(data);
  /* Defense-in-depth: never arm the blade target while IDLE/docked. The
   * authoritative gate is in motors_handler (which zeroes blade_on_off in
   * IDLE every tick), but refusing to latch the target here keeps state
   * consistent and avoids an instantaneous spin-up on the IDLE→MOWING edge.
   * blade_dir is still accepted so direction is correct once mowing starts. */
  if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
    target_blade_on_off = 0;
  } else {
    target_blade_on_off = pkt->blade_on;
  }
  blade_direction = pkt->blade_dir;
}

/* Host -> Firmware reboot request. Sets reboot_flag so chatter_handler issues
 * NVIC_SystemReset on its next tick (lets the current packet/ISR unwind first).
 * Gated on the magic byte so a corrupt/misframed packet can't reset the board.
 */
static void on_reboot(const uint8_t *data, size_t len) {
  if (len < sizeof(pkt_reboot_t) - 2u) {
    return;
  }
  const pkt_reboot_t *pkt = reinterpret_cast<const pkt_reboot_t *>(data);
  if (pkt->magic == PKT_REBOOT_MAGIC) {
    debug_printf("reboot requested by host\r\n");
    reboot_flag = true;
  }
}

/* Host -> Firmware config/version request. Replies with this firmware's
 * wire-protocol version (the compatibility key the host checks) and its
 * human-readable semver, so the ROS 2 image can confirm it is talking to a
 * compatible firmware and warn the operator to reflash on mismatch. Firmware
 * older than this handshake never registers this handler, so it simply never
 * replies — which the host reads as "incompatible firmware". */
static void on_config_req(const uint8_t *data, size_t len) {
  if (len < 1u) {
    return;
  }
  const uint8_t flags = (len >= 2u) ? data[1] : 0u;
  g_firmware_debug_enabled = (flags & CONFIG_FLAG_FIRMWARE_DEBUG) != 0u ? 1u : 0u;

  pkt_config_rsp_t rsp;
  rsp.type = PKT_ID_CONFIG_RSP;
  rsp.protocol_version = MOWGLI_PROTOCOL_VERSION;
  rsp.active_flags = g_firmware_debug_enabled != 0u ? CONFIG_FLAG_FIRMWARE_DEBUG : 0u;
  rsp.fw_version_major = MOWGLI_FW_VERSION_MAJOR;
  rsp.fw_version_minor = MOWGLI_FW_VERSION_MINOR;
  rsp.fw_version_patch = MOWGLI_FW_VERSION_PATCH;
  mowgli_comms_send(&rsp, sizeof(rsp));
}

/* on_hl_state blade LED feedback (moved out of on_hl_state for clarity) */
static void update_blade_led(void) {
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
uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len) {
  WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CDC_RX_PROCESS);
  mowgli_comms_process_rx(Buf, (size_t)len);
  WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CDC_RX_EXIT);
  return CDC_RX_DATA_HANDLED;
}

/* ---------------------------------------------------------------------------
 * usb_cdc_transmit — required by mowgli_comms.c
 * ---------------------------------------------------------------------------*/
void usb_cdc_transmit(const uint8_t *buf, size_t len) {
  CDC_Transmit(buf, (uint32_t)len);
}

/* ---------------------------------------------------------------------------
 * LED blink + reboot handler (replaces chatter_handler)
 * ---------------------------------------------------------------------------*/
extern "C" void chatter_handler() {
  if (NBT_handler(&led_nbt)) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

    if (reboot_flag) {
      NVIC_SystemReset();
    }
  }
}

/* Anti-dig per-wheel step (task #46). Returns true when the wheel must be cut
 * (caller forces PWM 0 + resets that wheel's integrator). Accumulates real
 * travel over the window so a slow-but-progressing creep (normal dock approach)
 * is NOT flagged — only a commanded, pushing, non-progressing wheel latches.
 * Clears the latch the instant the command drops below MIN_TARGET. */
static bool antidig_step(uint32_t *stall_ms, int32_t *stall_ticks, bool *latched,
                         float target, int16_t pwm, int32_t dticks,
                         int32_t min_ticks) {
  const bool commanded = fabsf(target) > ANTIDIG_MIN_TARGET_MPS;
  if (!commanded) {
    *stall_ms = 0u;
    *stall_ticks = 0;
    *latched = false;
    return false;
  }
  if (*latched) {
    return true; /* hold until the command clears (handled above) */
  }
  const bool pushing = (pwm > ANTIDIG_MIN_ABS_PWM) || (pwm < -ANTIDIG_MIN_ABS_PWM);
  if (!pushing) {
    *stall_ms = 0u;
    *stall_ticks = 0;
    return false;
  }
  *stall_ms += MOTORS_NBT_TIME_MS;
  *stall_ticks += (dticks < 0) ? -dticks : dticks;
  if (*stall_ms >= ANTIDIG_WINDOW_MS) {
    if (*stall_ticks < min_ticks) {
      *latched = true; /* pushed hard for the whole window, barely moved → dig */
    } else {
      *stall_ms = 0u; /* actually progressing — reset the window */
      *stall_ticks = 0;
    }
  }
  return *latched;
}

/* ---------------------------------------------------------------------------
 * Drive & blade motors handler
 * ---------------------------------------------------------------------------*/
extern "C" void motors_handler() {
  if (NBT_handler(&motors_nbt)) {
    /* Snapshot ISR-written variables under interrupt lock */
    __disable_irq();
    float snap_left_target = left_target_mps;
    float snap_right_target = right_target_mps;
    float snap_cmd_wz = cmd_wz;
    uint8_t snap_target_blade = target_blade_on_off;
    uint32_t snap_heartbeat = last_heartbeat_tick;
    uint32_t snap_cmd_vel = last_cmd_vel_tick;
    float snap_ticks_per_meter = DRIVEMOTOR_GetTicksPerMeter();
    __enable_irq();

    blade_on_off = snap_target_blade;

    /* --- decide effective target ---
     * Emergency or cmd_vel watchdog timeout overrides to a hard stop.
     * Otherwise the snapshot value drives the PI loop below. */
    bool hard_stop = false;
    if (Emergency_State()) {
      hard_stop = true;
      blade_on_off = 0;
    } else if (main_eOpenmowerStatus == OPENMOWER_STATUS_IDLE) {
      /* Re-assert the IDLE gate HERE — in the one place that actually
       * drives the wheels AND the blade — so the "never move / never
       * spin the blade while idle/docked" guarantee holds regardless of
       * the relative arrival order of CMD_PWM, CMD_BLADE and HL_STATE.
       * on_hl_state zeroes the targets when the IDLE packet arrives, but
       * a CMD_BLADE(on=1) arriving AFTER it would otherwise re-arm the
       * blade with no gate (on_cmd_blade is fire-and-forget). Firmware is
       * the sole blade safety authority. */
      hard_stop = true;
      blade_on_off = 0;
    } else {
      const uint32_t cmd_vel_age_ms = HAL_GetTick() - snap_cmd_vel;
      if (cmd_vel_age_ms > 200u) {
        /* Command-vel watchdog: zero motors if the host hasn't
         * sent a twist in 200 ms (Pi hang, USB glitch, etc). */
        hard_stop = true;
      }
      if (cmd_vel_age_ms > 25000u) {
        blade_on_off = 0;
      }
    }

    /* --- Option C: gyro-local closed yaw-rate loop ---
     * Regulate (commanded wz − measured gyro wz) and fold the output in as a
     * symmetric differential velocity trim on the per-wheel setpoints (below).
     * Runs before the per-wheel PIs so their integrators track the trimmed
     * setpoint. See the block comment at the yaw-loop globals for rationale and
     * the bounded-failure argument. */
    float yaw_trim_mps = 0.0f;
    const bool yaw_loop_active = (g_yaw_loop_enabled != 0u) && !hard_stop;
    /* Reset the yaw integrator on stop / yaw-direction reversal (mirrors the
     * per-wheel resets) AND at turn-exit — a sharp drop in |commanded wz| from
     * turning to straight (task #37). Dumping the wind-up here is what kills the
     * reported post-U-turn wiggle: without it the integral accumulated while
     * holding the turn keeps driving a differential onto the following straight. */
    const bool yaw_turn_exit = (fabsf(prev_cmd_wz) > YAW_TURN_EXIT_HI_RADPS &&
                                fabsf(snap_cmd_wz) < YAW_TURN_EXIT_LO_RADPS);
    const bool yaw_reset = !yaw_loop_active ||
                           (snap_cmd_wz == 0.0f && prev_cmd_wz != 0.0f) ||
                           (snap_cmd_wz * prev_cmd_wz < 0.0f) || yaw_turn_exit;
    if (yaw_reset) {
      yaw_pid.resetIntegral();
      yaw_pid.resetDerivative();
      yaw_gyro_filt_valid = 0u; /* re-seed the gyro low-pass after a reset */
    }
    prev_cmd_wz = snap_cmd_wz;

    /* Low-speed authority scale from COMMANDED mean forward speed: 1.0 at/above
     * REF, ramping down to FLOOR as vx→0. Uses the commanded (not measured)
     * speed so a momentary stall doesn't collapse authority, and so an in-place
     * pivot (vx≈0) is de-rated to FLOOR rather than zeroed — the pivot itself
     * still comes from the IK feedforward. On straights this is 1.0, leaving the
     * (working) weave rejection untouched. */
    const float yaw_vx_cmd = 0.5f * (snap_left_target + snap_right_target);
    float yaw_speed_scale = fabsf(yaw_vx_cmd) / YAW_LOWSPEED_REF_MPS;
    if (yaw_speed_scale > 1.0f)
      yaw_speed_scale = 1.0f;
    yaw_speed_scale =
        YAW_LOWSPEED_FLOOR + (1.0f - YAW_LOWSPEED_FLOOR) * yaw_speed_scale;

    if (yaw_loop_active) {
      float gx = 0.0f, gy = 0.0f, gz = 0.0f;
      /* Fresh gyro read in the motor timebase. On I2C failure hold trim at 0
       * (degrade to open-diff) rather than injecting a stale/bogus correction.
       * Both this and the IMU broadcast read run in the cooperative main loop,
       * so there is no re-entrancy on the software-I2C bus. */
      if (IMU_TryReadGyro(&gx, &gy, &gz)) {
        const float meas_wz_raw =
            g_yaw_gyro_sign * gz; /* rad/s, robot +yaw = CCW */
        /* 1st-order IIR low-pass on the gyro feedback (seeded on the first
         * sample after a reset to avoid a startup transient) so the loop stops
         * chasing the 2-4 Hz self-oscillation. */
        if (!yaw_gyro_filt_valid) {
          yaw_gyro_filt = meas_wz_raw;
          yaw_gyro_filt_valid = 1u;
        } else {
          yaw_gyro_filt += YAW_GYRO_LP_ALPHA * (meas_wz_raw - yaw_gyro_filt);
        }
        const float meas_wz = yaw_gyro_filt;
        const float yaw_err = snap_cmd_wz - meas_wz;
        /* Conditional-integration anti-windup: freeze the integrator only in
         * the direction that would push |trim| further past the clamp (same
         * pattern as the per-wheel loop). Keyed on the RAW (unscaled) trim so
         * the low-speed scaling below doesn't defeat the saturation test. */
        const bool yaw_update_integral =
            !((prev_yaw_trim_mps >= g_yaw_trim_limit_mps && yaw_err > 0.0f) ||
              (prev_yaw_trim_mps <= -g_yaw_trim_limit_mps && yaw_err < 0.0f));
        yaw_pid.setSetpoint(snap_cmd_wz);
        float raw_trim =
            yaw_pid.update(meas_wz, WHEEL_PI_DT_S, yaw_update_integral);
        /* Hard clamp (belt-and-suspenders over the PID output limit) — the
         * differential correction is bounded so a wrong gyro_sign can only veer,
         * never spin unbounded. */
        if (raw_trim > g_yaw_trim_limit_mps)
          raw_trim = g_yaw_trim_limit_mps;
        if (raw_trim < -g_yaw_trim_limit_mps)
          raw_trim = -g_yaw_trim_limit_mps;
        prev_yaw_trim_mps = raw_trim; /* anti-windup uses the unscaled value */

        /* Low-speed integral leak: bleed the integrator toward 0 when authority
         * is de-rated (retain=1.0 at full speed → no leak; approaches
         * YAW_INT_LEAK_AT_FLOOR near the dock) so accumulated bias can't sit and
         * hunt while nearly stopped. */
        const float yaw_int_retain =
            YAW_INT_LEAK_AT_FLOOR +
            (1.0f - YAW_INT_LEAK_AT_FLOOR) * yaw_speed_scale;
        yaw_pid.setIntegral(yaw_pid.getIntegral() * yaw_int_retain);

        /* Apply the low-speed authority scale to the correction actually
         * injected (base rotation is unaffected — it rides the IK feedforward). */
        yaw_trim_mps = raw_trim * yaw_speed_scale;
      }
      /* Slew-rate limit the injected trim (runs even on a gyro-read failure,
       * where yaw_trim_mps is 0, so prev tracks the applied value): the output
       * can't snap across the wheel deadband in one cycle and re-excite the
       * limit-cycle. */
      const float yaw_dtrim = yaw_trim_mps - prev_applied_yaw_trim_mps;
      if (yaw_dtrim > YAW_TRIM_SLEW_MPS_PER_CYCLE)
        yaw_trim_mps = prev_applied_yaw_trim_mps + YAW_TRIM_SLEW_MPS_PER_CYCLE;
      else if (yaw_dtrim < -YAW_TRIM_SLEW_MPS_PER_CYCLE)
        yaw_trim_mps = prev_applied_yaw_trim_mps - YAW_TRIM_SLEW_MPS_PER_CYCLE;
      prev_applied_yaw_trim_mps = yaw_trim_mps;
    } else {
      prev_yaw_trim_mps = 0.0f;
      prev_applied_yaw_trim_mps = 0.0f;
    }

    /* Apply the symmetric differential trim to the per-wheel setpoints
     * (+right / −left increases yaw rate, matching the IK in on_cmd_vel), then
     * re-clamp to the physical wheel-speed limit. hard_stop forces 0. */
    float l_target = snap_left_target - yaw_trim_mps;
    float r_target = snap_right_target + yaw_trim_mps;
    if (hard_stop) {
      l_target = 0.0f;
      r_target = 0.0f;
    }
    if (l_target > MAX_MPS)
      l_target = MAX_MPS;
    if (l_target < -MAX_MPS)
      l_target = -MAX_MPS;
    if (r_target > MAX_MPS)
      r_target = MAX_MPS;
    if (r_target < -MAX_MPS)
      r_target = -MAX_MPS;

#if USE_WHEEL_PI
    /* Wheel-level PI loop.
     *
     * Reads the signed cumulative encoder count maintained by
     * drivemotor.c, derives actual_mps over the 20 ms loop
     * period, computes a feedforward + PI PWM. The integrator
     * is what bridges the static-friction deadband: while the
     * target says "move 0.05 m/s" and the encoder says "0",
     * Ki × error × dt accumulates until the PWM crosses the
     * deadband (~40), the motor breaks free, the wheel starts
     * counting ticks, error drops, and the integrator settles
     * at whatever PWM keeps that wheel at target speed.
     *
     * Read left_ticks_signed/right_ticks_signed directly (these are
     * 32-bit and updated from the drivemotor rx-decode path —
     * not strictly atomic, but a torn read here costs at most
     * one 20 ms loop of incorrect velocity, then converges). */
    const int32_t cur_left_ticks = left_ticks_signed;
    const int32_t cur_right_ticks = right_ticks_signed;
    const int32_t dleft_ticks = cur_left_ticks - prev_left_ticks_signed_pi;
    const int32_t dright_ticks = cur_right_ticks - prev_right_ticks_signed_pi;
    prev_left_ticks_signed_pi = cur_left_ticks;
    prev_right_ticks_signed_pi = cur_right_ticks;

    const float l_actual_mps =
        ((float)dleft_ticks) / snap_ticks_per_meter / WHEEL_PI_DT_S;
    const float r_actual_mps =
        ((float)dright_ticks) / snap_ticks_per_meter / WHEEL_PI_DT_S;

    /* Reset the integrator on direction reversal / stop-to-go / hard-stop.
     * Without this the integral built up while decelerating would drive the
     * motor backwards as soon as the chassis stopped (micro-oscillation). */
    const bool l_target_sign_changed =
        (l_target * prev_left_target_mps < 0.0f) ||
        (l_target == 0.0f && prev_left_target_mps != 0.0f) || hard_stop;
    const bool r_target_sign_changed =
        (r_target * prev_right_target_mps < 0.0f) ||
        (r_target == 0.0f && prev_right_target_mps != 0.0f) || hard_stop;
    if (l_target_sign_changed) {
      left_wheel_pid.resetIntegral();
      left_wheel_pid.resetDerivative();
    }
    if (r_target_sign_changed) {
      right_wheel_pid.resetIntegral();
      right_wheel_pid.resetDerivative();
    }
    prev_left_target_mps = l_target;
    prev_right_target_mps = r_target;

    /* Conditional-integration anti-windup, DIRECTION-AWARE: freeze the
     * integrator only in the direction that would worsen an already-
     * saturated output; always allow it to unwind OUT of saturation. Keying
     * on the error sign (not just the saturation bit) avoids a one-
     * directional integral latch — e.g. on overspeed (err < 0) while the
     * output is railed high, the integrator must still be able to wind down
     * to cut PWM. *_pwm_signed is the previous cycle's total (feedforward +
     * trim), saturated by DRIVEMOTOR_SetSpeedSigned at ±255. This sits on
     * top of the PID's own ±100 integral-magnitude clamp. */
    const float l_err = l_target - l_actual_mps;
    const float r_err = r_target - r_actual_mps;
    const bool l_update_integral = !((left_pwm_signed >= 255 && l_err > 0.0f) ||
                                     (left_pwm_signed <= -255 && l_err < 0.0f));
    const bool r_update_integral =
        !((right_pwm_signed >= 255 && r_err > 0.0f) ||
          (right_pwm_signed <= -255 && r_err < 0.0f));

    /* Closed-loop PI trim (Kp·err + integrator; D gain = 0). The PID
     * computes error = setpoint − feedback internally and integrates AFTER
     * forming the output (PX4 form), so a fresh integral increment reaches
     * the actuator one 50 Hz cycle later than the old integrate-before form
     * — steady-state identical, ~20 ms transient shift (immaterial here). */
    left_wheel_pid.setSetpoint(l_target);
    right_wheel_pid.setSetpoint(r_target);
    const float l_trim =
        left_wheel_pid.update(l_actual_mps, WHEEL_PI_DT_S, l_update_integral);
    const float r_trim =
        right_wheel_pid.update(r_actual_mps, WHEEL_PI_DT_S, r_update_integral);

    /* Open-loop feedforward (deadband-bridge, preserves the above-deadband
     * mapping) + closed-loop PI trim. Sign carried through. */
    const float l_pwm_f = l_target * g_pwm_per_mps + l_trim;
    const float r_pwm_f = r_target * g_pwm_per_mps + r_trim;

    /* When the target is exactly zero AND we're not braking from a
     * larger speed, force PWM to zero outright — avoids the residual
     * "hum" from a non-zero integral applied to a stopped wheel. */
    left_pwm_signed = (l_target == 0.0f && fabsf(l_actual_mps) < 0.02f)
                          ? 0
                          : (int16_t)l_pwm_f;
    right_pwm_signed = (r_target == 0.0f && fabsf(r_actual_mps) < 0.02f)
                           ? 0
                           : (int16_t)r_pwm_f;

    /* Anti-dig cutout (always active, all modes). min_ticks derived from the
     * live ticks_per_meter so the "is it progressing?" test tracks calibration.
     * dleft/dright_ticks and left/right_pwm_signed are this cycle's values. */
    const int32_t antidig_min_ticks =
        (int32_t)(ANTIDIG_MIN_PROGRESS_M * snap_ticks_per_meter);
    if (antidig_step(&l_dig_stall_ms, &l_dig_stall_ticks, &l_dig_latched, l_target,
                     left_pwm_signed, dleft_ticks, antidig_min_ticks)) {
      left_pwm_signed = 0;
      left_wheel_pid.resetIntegral();
    }
    if (antidig_step(&r_dig_stall_ms, &r_dig_stall_ticks, &r_dig_latched, r_target,
                     right_pwm_signed, dright_ticks, antidig_min_ticks)) {
      right_pwm_signed = 0;
      right_wheel_pid.resetIntegral();
    }
#else
    /* Open-loop fallback for bring-up / regression A/B. Replicates the
     * pre-PI mapping exactly: PWM = target × PWM_PER_MPS, no encoder
     * feedback, no integrator. */
    left_pwm_signed = (int16_t)(l_target * g_pwm_per_mps);
    right_pwm_signed = (int16_t)(r_target * g_pwm_per_mps);
#endif

    if (hard_stop) {
      DRIVEMOTOR_SetSpeedSigned(0, 0);
    } else {
      DRIVEMOTOR_SetSpeedSigned(left_pwm_signed, right_pwm_signed);
    }

    // Heartbeat watchdog: if no heartbeat for HEARTBEAT_TIMEOUT_MS, emergency
    // stop. Tag a PURE comms-loss latch (no physical sensor asserted) so it can
    // be auto-cleared when heartbeats resume (on_heartbeat), instead of
    // stranding the robot. If a physical sensor is asserted, leave the flag
    // cleared so the latch needs an explicit operator release.
    if (snap_heartbeat != 0 &&
        (HAL_GetTick() - snap_heartbeat) > HEARTBEAT_TIMEOUT_MS) {
      if (any_physical_emergency()) {
        heartbeat_only_latch = false;
      } else if (!Emergency_State()) {
        heartbeat_only_latch = true;
      }
      Emergency_SetState(1);
    }

    BLADEMOTOR_Set(blade_on_off, blade_direction);
  }
}

/* ---------------------------------------------------------------------------
 * Panel handler — button presses generate UI events over COBS
 * ---------------------------------------------------------------------------*/
extern "C" void panel_handler() {
  if (NBT_handler(&panel_nbt)) {
    PANEL_Tick();

    if (buttonupdated == 1 && buttoncleared == 0) {
      const uint32_t ui_event_wire_bytes =
          usb_cdc_framed_packet_size(sizeof(pkt_ui_event_t));
      pkt_ui_event_t evt;
      evt.type = PKT_ID_UI_EVENT;
      evt.press_duration = 0; // short press

      // Map physical buttons to IDs
      if (buttonstate[PANEL_BUTTON_DEF_S1]) {
        evt.button_id = 1;
        if (usb_cdc_should_send_telemetry_packet(ui_event_wire_bytes)) {
          mowgli_comms_send(&evt, sizeof(evt));
        }
      }
      if (buttonstate[PANEL_BUTTON_DEF_S2]) {
        evt.button_id = 2;
        if (usb_cdc_should_send_telemetry_packet(ui_event_wire_bytes)) {
          mowgli_comms_send(&evt, sizeof(evt));
        }
      }
      if (buttonstate[PANEL_BUTTON_DEF_LOCK]) {
        evt.button_id = 3;
        if (usb_cdc_should_send_telemetry_packet(ui_event_wire_bytes)) {
          mowgli_comms_send(&evt, sizeof(evt));
        }
      }
      if (buttonstate[PANEL_BUTTON_DEF_START]) {
        evt.button_id = 4;
        if (usb_cdc_should_send_telemetry_packet(ui_event_wire_bytes)) {
          mowgli_comms_send(&evt, sizeof(evt));
        }
      }
      if (buttonstate[PANEL_BUTTON_DEF_HOME]) {
        evt.button_id = 5;
        if (usb_cdc_should_send_telemetry_packet(ui_event_wire_bytes)) {
          mowgli_comms_send(&evt, sizeof(evt));
        }
      }

      buttonupdated = 0;
    }
  }
}

#if OPTION_ULTRASONIC == 1
extern "C" void ultrasonic_handler(void) {
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
extern "C" void
wheelTicks_handler(int32_t p_s32LeftTicksSigned, int32_t p_s32RightTicksSigned,
                   int16_t p_s16LeftSpeed, /* currently unused — reserved for
                                              future telemetry */
                   int16_t p_s16RightSpeed) {
  (void)p_s16LeftSpeed;
  (void)p_s16RightSpeed;

  static int32_t prev_left_ticks = 0;
  static int32_t prev_right_ticks = 0;

  const uint32_t now_tick = HAL_GetTick();
  const uint16_t dt_ms = (uint16_t)(now_tick - last_odom_tick);
  last_odom_tick = now_tick;

  const int32_t delta_left = p_s32LeftTicksSigned - prev_left_ticks;
  const int32_t delta_right = p_s32RightTicksSigned - prev_right_ticks;
  prev_left_ticks = p_s32LeftTicksSigned;
  prev_right_ticks = p_s32RightTicksSigned;

  /* Velocity: mm/s = (delta_ticks / ticks_per_meter) * (1000 / dt_ms) * 1000
   *                = delta_ticks * 1e6 / (ticks_per_meter * dt_ms).
   * ticks_per_meter starts from board.h TICKS_PER_M, then the ROS host can
   * override it at runtime. Use float math here so fractional tuning values
   * (e.g. 319.305) survive end-to-end. */
  int16_t left_v_mm_s = 0;
  int16_t right_v_mm_s = 0;
  if (dt_ms > 0) {
    const float ticks_per_meter = DRIVEMOTOR_GetTicksPerMeter();
    const float scale = 1000000.0f / (ticks_per_meter * (float)dt_ms);
    int32_t v_l = (int32_t)((float)delta_left * scale);
    int32_t v_r = (int32_t)((float)delta_right * scale);
    if (v_l > 32767)
      v_l = 32767;
    if (v_l < -32768)
      v_l = -32768;
    if (v_r > 32767)
      v_r = 32767;
    if (v_r < -32768)
      v_r = -32768;
    left_v_mm_s = (int16_t)v_l;
    right_v_mm_s = (int16_t)v_r;
  }

  pkt_odometry_t odom;
  odom.type = PKT_ID_ODOMETRY;
  odom.dt_millis = dt_ms;
  odom.left_ticks = p_s32LeftTicksSigned;
  odom.right_ticks = p_s32RightTicksSigned;
  odom.left_velocity_mm_s = left_v_mm_s;
  odom.right_velocity_mm_s = right_v_mm_s;

  if (usb_cdc_should_send_telemetry_packet(
          usb_cdc_framed_packet_size(sizeof(pkt_odometry_t)))) {
    mowgli_comms_send_odometry(&odom);
  }
}

/* ---------------------------------------------------------------------------
 * IMU + status broadcast handler
 * ---------------------------------------------------------------------------*/
extern "C" void broadcast_handler() {
  WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_ENTER);
  const uint32_t now_tick = HAL_GetTick();

  /* Bound this section to one broadcast group per pass. Without that bound a
   * delayed loop can emit IMU + reset/status + blade packets back-to-back in
   * the same WATCHDOG_STAGE_BROADCAST window. The USB CDC enqueue path is
   * non-blocking, but building and queueing several packets in one pass still
   * stretches the watchdog window unnecessarily when USB is backpressured. */
  if (nbt_due(&status_nbt, now_tick)) {
    const uint32_t status_group_bytes =
        usb_cdc_framed_packet_size(sizeof(pkt_reset_cause_t)) +
        usb_cdc_framed_packet_size(sizeof(pkt_status_t));
    if (!usb_cdc_should_send_telemetry_packet(status_group_bytes)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }

    nbt_consume(&status_nbt, now_tick);

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
    status_pkt.uss_ranges_m[0] =
        (float)(ULTRASONICSENSOR_u32GetLeftDistance()) / 10000.0f;
    status_pkt.uss_ranges_m[1] =
        (float)(ULTRASONICSENSOR_u32GetRightDistance()) / 10000.0f;
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
    status_pkt.v_charge = charge_voltage;
    status_pkt.v_system = battery_voltage;
    status_pkt.charging_current = current;
    status_pkt.batt_percentage = 0; // TODO: compute from voltage curve

    pkt_reset_cause_t reset_pkt;
    reset_pkt.type = PKT_ID_RESET_CAUSE;
    reset_pkt.reset_cause = g_boot_reset_cause_code;
    reset_pkt.last_stage_before_reset = g_boot_last_watchdog_stage_code;
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_RESET_SEND);
    mowgli_comms_send(&reset_pkt, sizeof(reset_pkt));
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_STATUS_SEND);
    mowgli_comms_send_status(&status_pkt);
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
    return;
  }

  if (nbt_due(&imu_nbt, now_tick)) {
    const uint32_t imu_wire_bytes = usb_cdc_framed_packet_size(sizeof(pkt_imu_t));
    if (!usb_cdc_should_send_telemetry_packet(imu_wire_bytes)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }

    nbt_consume(&imu_nbt, now_tick);
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_IMU_BUILD);

    static uint32_t last_imu_tick = 0;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

#ifdef EXTERNAL_IMU_ACCELERATION
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_IMU_ACCEL);
    if (!IMU_TryReadAccelerometer(&ax, &ay, &az)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }
#endif

#ifdef EXTERNAL_IMU_ANGULAR
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_IMU_GYRO);
    if (!IMU_TryReadGyro(&gx, &gy, &gz)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }
#endif

    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_IMU_MAG);
    if (!IMU_TryReadMag(&mx, &my, &mz)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }

    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_IMU_PACKET_FILL);

    pkt_imu_t imu_pkt = {};
    imu_pkt.type = PKT_ID_IMU;
    imu_pkt.dt_millis = (uint16_t)(now_tick - last_imu_tick);
    last_imu_tick = now_tick;
    imu_pkt.acceleration_mss[0] = ax;
    imu_pkt.acceleration_mss[1] = ay;
    imu_pkt.acceleration_mss[2] = az;
    imu_pkt.gyro_rads[0] = gx;
    imu_pkt.gyro_rads[1] = gy;
    imu_pkt.gyro_rads[2] = gz;
    imu_pkt.mag_uT[0] = mx;
    imu_pkt.mag_uT[1] = my;
    imu_pkt.mag_uT[2] = mz;

    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_IMU_SEND);
    mowgli_comms_send_imu(&imu_pkt);
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
    return;
  }

  // Blade motor status (4 Hz) — only after system has initialized
  if (last_heartbeat_tick != 0u && nbt_due(&blade_nbt, now_tick)) {
    const uint32_t blade_wire_bytes =
        usb_cdc_framed_packet_size(sizeof(pkt_blade_status_t));
    if (!usb_cdc_should_send_telemetry_packet(blade_wire_bytes)) {
      WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
      return;
    }

    nbt_consume(&blade_nbt, now_tick);

    pkt_blade_status_t blade_pkt = {};
    blade_pkt.type = PKT_ID_BLADE_STATUS;
    blade_pkt.is_active = BLADEMOTOR_bActivated ? 1u : 0u;
    blade_pkt.rpm = BLADEMOTOR_u16RPM;
    blade_pkt.power_watts = BLADEMOTOR_u16Power;
    blade_pkt.temperature = blade_temperature;
    blade_pkt.error_count = BLADEMOTOR_u32Error;
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_BLADE_SEND);
    mowgli_comms_send(&blade_pkt, sizeof(blade_pkt));
    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
    return;
  }

  WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_BROADCAST_EXIT);
}

/* ---------------------------------------------------------------------------
 * spinOnce — no-op (rosserial spin removed)
 * ---------------------------------------------------------------------------*/
extern "C" void spinOnce() {
  // Nothing to do — COBS RX is handled in CDC_DataReceivedHandler().
  // This function is kept so main.c doesn't need modification.
}

/* ---------------------------------------------------------------------------
 * Initialisation (replaces init_ROS)
 * ---------------------------------------------------------------------------*/
extern "C" void init_ROS() {
  // Initialise COBS comms layer
  mowgli_comms_init();

  // Register handlers for Host -> Firmware packets
  mowgli_comms_register_handler(PKT_ID_HEARTBEAT, on_heartbeat);
  mowgli_comms_register_handler(PKT_ID_CMD_VEL, on_cmd_vel);
  mowgli_comms_register_handler(PKT_ID_HL_STATE, on_hl_state);
  mowgli_comms_register_handler(PKT_ID_CMD_BLADE, on_cmd_blade);
  mowgli_comms_register_handler(PKT_ID_REBOOT, on_reboot);
  mowgli_comms_register_handler(PKT_ID_SET_DRIVE_PID, on_set_drive_pid);
  mowgli_comms_register_handler(PKT_ID_SET_YAW_PID, on_set_yaw_pid);
  mowgli_comms_register_handler(PKT_ID_CONFIG_REQ, on_config_req);

  // Initialise timers
  NBT_init(&led_nbt, LED_NBT_TIME_MS);
  NBT_init(&panel_nbt, PANEL_NBT_TIME_MS);
  NBT_init(&status_nbt, STATUS_NBT_TIME_MS);
  NBT_init(&imu_nbt, IMU_NBT_TIME_MS);
  NBT_init(&motors_nbt, MOTORS_NBT_TIME_MS);
  NBT_init(&blade_nbt, BLADE_NBT_TIME_MS);

#if USE_WHEEL_PI
  // Per-wheel velocity PI gains/limits (vendored PX4 PID, pid.hpp). D=0 — no
  // derivative on a velocity loop. Gains/limits are in PWM units, matching the
  // hand-rolled loop they replace (Kp·err + integrator, integral clamp ±100,
  // output clamp ±255). The PID adds derivative-on-measurement (unused at D=0)
  // and conditional-integration anti-windup.
  left_wheel_pid.setGains(WHEEL_PI_KP_PWM_PER_MPS, WHEEL_PI_KI_PWM_PER_MPS_S,
                          0.0f);
  left_wheel_pid.setIntegralLimit(WHEEL_PI_INT_MAX_PWM);
  left_wheel_pid.setOutputLimit(255.0f);
  right_wheel_pid.setGains(WHEEL_PI_KP_PWM_PER_MPS, WHEEL_PI_KI_PWM_PER_MPS_S,
                           0.0f);
  right_wheel_pid.setIntegralLimit(WHEEL_PI_INT_MAX_PWM);
  right_wheel_pid.setOutputLimit(255.0f);

  /* Gyro yaw-rate loop (Option C). Output/integral both clamped to the trim
   * limit so the differential correction is bounded regardless of gains. */
  yaw_pid.setGains(YAW_PI_KP_DEFAULT, YAW_PI_KI_DEFAULT, 0.0f);
  yaw_pid.setIntegralLimit(YAW_TRIM_LIMIT_MPS_DEFAULT * YAW_INT_LIMIT_FRAC);
  yaw_pid.setOutputLimit(YAW_TRIM_LIMIT_MPS_DEFAULT);
#endif

  last_odom_tick = HAL_GetTick();
  last_heartbeat_tick = 0;
  last_cmd_vel_tick = 0;
}

float clamp(float d, float min, float max) {
  const float t = d < min ? min : d;
  return t > max ? max : t;
}
