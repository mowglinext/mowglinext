/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_catalog.h
 * @brief Runtime firmware parameters: wire ids and the ABSOLUTE envelope.
 *
 * Pure C (no HAL), so the host test suite can compile it directly
 * (ros2/src/mowgli_hardware/test/test_fw_param_catalog.cpp pins the host
 * mirror in ll_datatypes.hpp against it).
 *
 * Every parameter the host can set at runtime (PKT_ID_SET_PARAM) has:
 *   - a STABLE wire id. Never renumber or reuse one: ids are also the keys of
 *     the records persisted in flash (fw_param_log.h), so a renumbered id would
 *     load an old value into the wrong parameter after a firmware update.
 *   - an absolute [min, max] envelope compiled here. The operator (GUI, yaml,
 *     custom board.h) can move a value anywhere inside it — tighter OR looser
 *     than the compiled default — but never outside it. The envelope is not
 *     configurable from the GUI flash form on purpose: it is the last line of
 *     defence against a bad or hostile value.
 *
 * SAFETY: every bound below changes what the host is allowed to do to the
 * physical robot (e-stop latency, speed, charging). Treat any change here as
 * safety-critical and robot-verify it.
 */
#ifndef FW_PARAM_CATALOG_H
#define FW_PARAM_CATALOG_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Wire ids (uint16). Grouped by tens; gaps are reserved for the group.
 * ---------------------------------------------------------------------------*/
#define FW_PARAM_TICKS_PER_METER 1u
#define FW_PARAM_WHEEL_KP 2u
#define FW_PARAM_WHEEL_KI 3u
#define FW_PARAM_WHEEL_KD 4u
#define FW_PARAM_WHEEL_INTEGRAL_LIMIT 5u
#define FW_PARAM_PWM_PER_MPS 6u

#define FW_PARAM_YAW_KP 10u
#define FW_PARAM_YAW_KI 11u
#define FW_PARAM_YAW_TRIM_LIMIT_MPS 12u
#define FW_PARAM_YAW_LOOP_ENABLED 13u
#define FW_PARAM_YAW_GYRO_SIGN 14u
#define FW_PARAM_YAW_GYRO_BIAS_RADPS 15u

#define FW_PARAM_MAX_MPS 20u
#define FW_PARAM_WHEEL_BASE 21u

#define FW_PARAM_MAX_CHARGE_VOLTAGE 30u
#define FW_PARAM_MAX_CHARGE_CURRENT 31u

#define FW_PARAM_ONE_WHEEL_LIFT_MS 40u
#define FW_PARAM_BOTH_WHEELS_LIFT_MS 41u
#define FW_PARAM_TILT_MS 42u
#define FW_PARAM_STOP_BUTTON_MS 43u
#define FW_PARAM_PLAY_CLEAR_MS 44u

#define FW_PARAM_IMU_INCLINATION_THRESHOLD 50u

/** GET_PARAM with this id asks for every parameter. Never a real id. */
#define FW_PARAM_ID_ALL 0xFFFFu

/* ---------------------------------------------------------------------------
 * Absolute envelope. Where a bound equals today's compiled default it is
 * because no verified hardware limit above it exists (charging), not an
 * oversight: raise it only with a hardware reason.
 * ---------------------------------------------------------------------------*/
/* Charge: 29.4 V is 7S Li-ion at 4.2 V/cell — never exceed. 1.2 A is the
 * shipped charge current; no higher value has been validated on the board. */
#define FW_ENVELOPE_CHARGE_VOLTAGE_MIN 25.2f /* = LOW_BAT_THRESHOLD, the end-voltage floor in charger.c */
#define FW_ENVELOPE_CHARGE_VOLTAGE_MAX 29.4f
#define FW_ENVELOPE_CHARGE_CURRENT_MIN 0.1f
#define FW_ENVELOPE_CHARGE_CURRENT_MAX 1.2f
/* Emergency trip delays [ms]: how long a fault must persist before the latch.
 * Shorter is stricter. The maxima allow loosening a noisy sensor, never
 * disabling it (I_DONT_NEED_MY_FINGERS stays a compile-time-only choice). */
#define FW_ENVELOPE_TRIP_MIN_MS 10.0f
#define FW_ENVELOPE_ONE_WHEEL_LIFT_MAX_MS 5000.0f
#define FW_ENVELOPE_BOTH_WHEELS_LIFT_MAX_MS 3000.0f
#define FW_ENVELOPE_TILT_MAX_MS 1000.0f
#define FW_ENVELOPE_STOP_BUTTON_MAX_MS 250.0f
/* Hold-PLAY-to-clear [ms]: longer is stricter. */
#define FW_ENVELOPE_PLAY_CLEAR_MIN_MS 500.0f
#define FW_ENVELOPE_PLAY_CLEAR_MAX_MS 10000.0f
/* Wheel-speed cap [m/s]. */
#define FW_ENVELOPE_MAX_MPS_MIN 0.1f
#define FW_ENVELOPE_MAX_MPS_MAX 0.6f
/* LIS3DH INT1 threshold (same window board_defaults.h enforces at compile
 * time; 0x2C is stock firmware, larger = more inclination before the trip). */
#define FW_ENVELOPE_INCLINATION_MIN 44.0f /* 0x2C */
#define FW_ENVELOPE_INCLINATION_MAX 64.0f /* 0x40 */

/* ---------------------------------------------------------------------------
 * Spec table
 * ---------------------------------------------------------------------------*/
typedef enum {
  FW_PARAM_GROUP_DRIVE = 0,
  FW_PARAM_GROUP_YAW = 1,
  FW_PARAM_GROUP_KINEMATICS = 2,
  FW_PARAM_GROUP_CHARGE = 3,
  FW_PARAM_GROUP_EMERGENCY = 4,
  FW_PARAM_GROUP_IMU = 5,
  FW_PARAM_GROUP_COUNT = 6
} fw_param_group_t;

typedef enum {
  FW_PARAM_KIND_REAL = 0, /* any finite value in [min, max] */
  FW_PARAM_KIND_INT = 1,  /* rounded to the nearest integer */
  FW_PARAM_KIND_BOOL = 2, /* 0 or 1 (non-zero -> 1) */
  FW_PARAM_KIND_SIGN = 3  /* -1 or +1 (negative -> -1, else +1) */
} fw_param_kind_t;

/** Not persisted in flash: re-measured by the host (gyro bias). */
#define FW_PARAM_FLAG_VOLATILE 0x01u

typedef struct {
  uint16_t id;
  uint8_t group; /* fw_param_group_t */
  uint8_t kind;  /* fw_param_kind_t */
  uint8_t flags; /* FW_PARAM_FLAG_* */
  float min;
  float max;
} fw_param_spec_t;

static const fw_param_spec_t FW_PARAM_SPECS[] = {
    {FW_PARAM_TICKS_PER_METER, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 50.0f, 5000.0f},
    {FW_PARAM_WHEEL_KP, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 0.0f, 200.0f},
    {FW_PARAM_WHEEL_KI, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 0.0f, 20000.0f},
    {FW_PARAM_WHEEL_KD, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 0.0f, 500.0f},
    {FW_PARAM_WHEEL_INTEGRAL_LIMIT, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 0.0f, 255.0f},
    {FW_PARAM_PWM_PER_MPS, FW_PARAM_GROUP_DRIVE, FW_PARAM_KIND_REAL, 0u, 50.0f, 600.0f},

    {FW_PARAM_YAW_KP, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_REAL, 0u, 0.0f, 5.0f},
    {FW_PARAM_YAW_KI, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_REAL, 0u, 0.0f, 20.0f},
    /* Also clamped to the live max_mps when applied (a trim above the speed
     * cap would be meaningless). */
    {FW_PARAM_YAW_TRIM_LIMIT_MPS, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_REAL, 0u, 0.0f, 0.5f},
    {FW_PARAM_YAW_LOOP_ENABLED, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_BOOL, 0u, 0.0f, 1.0f},
    {FW_PARAM_YAW_GYRO_SIGN, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_SIGN, 0u, -1.0f, 1.0f},
    /* A physical at-rest bias is a few deg/s: clamp hard so a bad value can
     * only nudge, not steer. */
    {FW_PARAM_YAW_GYRO_BIAS_RADPS, FW_PARAM_GROUP_YAW, FW_PARAM_KIND_REAL,
     FW_PARAM_FLAG_VOLATILE, -0.5f, 0.5f},

    {FW_PARAM_MAX_MPS, FW_PARAM_GROUP_KINEMATICS, FW_PARAM_KIND_REAL, 0u,
     FW_ENVELOPE_MAX_MPS_MIN, FW_ENVELOPE_MAX_MPS_MAX},
    {FW_PARAM_WHEEL_BASE, FW_PARAM_GROUP_KINEMATICS, FW_PARAM_KIND_REAL, 0u, 0.15f, 0.60f},

    {FW_PARAM_MAX_CHARGE_VOLTAGE, FW_PARAM_GROUP_CHARGE, FW_PARAM_KIND_REAL, 0u,
     FW_ENVELOPE_CHARGE_VOLTAGE_MIN, FW_ENVELOPE_CHARGE_VOLTAGE_MAX},
    {FW_PARAM_MAX_CHARGE_CURRENT, FW_PARAM_GROUP_CHARGE, FW_PARAM_KIND_REAL, 0u,
     FW_ENVELOPE_CHARGE_CURRENT_MIN, FW_ENVELOPE_CHARGE_CURRENT_MAX},

    {FW_PARAM_ONE_WHEEL_LIFT_MS, FW_PARAM_GROUP_EMERGENCY, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_TRIP_MIN_MS, FW_ENVELOPE_ONE_WHEEL_LIFT_MAX_MS},
    {FW_PARAM_BOTH_WHEELS_LIFT_MS, FW_PARAM_GROUP_EMERGENCY, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_TRIP_MIN_MS, FW_ENVELOPE_BOTH_WHEELS_LIFT_MAX_MS},
    {FW_PARAM_TILT_MS, FW_PARAM_GROUP_EMERGENCY, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_TRIP_MIN_MS, FW_ENVELOPE_TILT_MAX_MS},
    {FW_PARAM_STOP_BUTTON_MS, FW_PARAM_GROUP_EMERGENCY, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_TRIP_MIN_MS, FW_ENVELOPE_STOP_BUTTON_MAX_MS},
    {FW_PARAM_PLAY_CLEAR_MS, FW_PARAM_GROUP_EMERGENCY, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_PLAY_CLEAR_MIN_MS, FW_ENVELOPE_PLAY_CLEAR_MAX_MS},

    {FW_PARAM_IMU_INCLINATION_THRESHOLD, FW_PARAM_GROUP_IMU, FW_PARAM_KIND_INT, 0u,
     FW_ENVELOPE_INCLINATION_MIN, FW_ENVELOPE_INCLINATION_MAX},
};

#define FW_PARAM_COUNT (sizeof(FW_PARAM_SPECS) / sizeof(FW_PARAM_SPECS[0]))

/** Index of @p id in FW_PARAM_SPECS, or -1 when unknown. */
static inline int fw_param_index(uint16_t id) {
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    if (FW_PARAM_SPECS[i].id == id) {
      return (int)i;
    }
  }
  return -1;
}

typedef enum {
  FW_PARAM_STATUS_OK = 0,          /* applied exactly as requested */
  FW_PARAM_STATUS_CLAMPED = 1,     /* applied, moved into the envelope */
  FW_PARAM_STATUS_UNKNOWN_ID = 2,  /* no such parameter on this firmware */
  FW_PARAM_STATUS_REJECTED = 3     /* non-finite value: nothing changed */
} fw_param_status_t;

/**
 * Coerce @p requested into the spec's kind and envelope. Returns the value to
 * apply in @p out and the status. A non-finite request is REJECTED and leaves
 * @p out untouched: the caller keeps the current value.
 */
static inline fw_param_status_t fw_param_coerce(const fw_param_spec_t *spec, float requested,
                                                float *out) {
  if (!isfinite(requested)) {
    return FW_PARAM_STATUS_REJECTED;
  }
  float v = requested;
  switch (spec->kind) {
    case FW_PARAM_KIND_INT:
      v = floorf(v + 0.5f);
      break;
    case FW_PARAM_KIND_BOOL:
      v = (v != 0.0f) ? 1.0f : 0.0f;
      break;
    case FW_PARAM_KIND_SIGN:
      v = (v < 0.0f) ? -1.0f : 1.0f;
      break;
    default:
      break;
  }
  if (v < spec->min) {
    v = spec->min;
  } else if (v > spec->max) {
    v = spec->max;
  }
  *out = v;
  return (v == requested) ? FW_PARAM_STATUS_OK : FW_PARAM_STATUS_CLAMPED;
}

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAM_CATALOG_H */
