/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file drive_tuning_defaults.h
 * @brief Power-on defaults of the per-wheel velocity PI and the gyro yaw-rate
 *        loop.
 *
 * Shared by cpp_main.cpp (the loops) and fw_params.c (the runtime parameter
 * table seeds its defaults from here). A parameter persisted in flash, or sent
 * by the host, replaces these; they only run on a board that has neither.
 */
#ifndef DRIVE_TUNING_DEFAULTS_H
#define DRIVE_TUNING_DEFAULTS_H

/* Per-wheel velocity PI, PWM units. */
#define WHEEL_PI_KP_PWM_PER_MPS 30.0f /* proportional gain */
#define WHEEL_PI_KI_PWM_PER_MPS_S \
  5000.0f /* integral gain (50 PWM in ~0.2 s when err=0.05 m/s) */
#define WHEEL_PI_KD_DEFAULT 0.0f /* no derivative on a velocity loop */
#define WHEEL_PI_INT_MAX_PWM 100.0f /* anti-windup clamp on the integral term */

/* Gyro yaw-rate loop (Option C). */
#define YAW_PI_KP_DEFAULT 0.30f          /* m/s trim per rad/s yaw error */
#define YAW_PI_KI_DEFAULT 0.40f          /* m/s trim per (rad/s·s) */
#define YAW_TRIM_LIMIT_MPS_DEFAULT 0.15f /* clamp on |differential trim| [m/s] */
#define YAW_LOOP_ENABLED_DEFAULT 1.0f    /* closed yaw loop on */
#define YAW_GYRO_SIGN_DEFAULT 1.0f       /* gyro Z sign vs robot +yaw (CCW) */

#endif /* DRIVE_TUNING_DEFAULTS_H */
