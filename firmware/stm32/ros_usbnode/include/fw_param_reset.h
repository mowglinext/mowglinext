/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_reset.h
 * @brief Explicit, reboot-time reset of the runtime parameter flash log.
 *
 * RTC backup registers DR7/8 hold the one-shot reset marker. DR9/10 retain a
 * bounded duplicate-check ID after marker consumption; they are not durable
 * request history. An interrupted later arm can partially replace this value
 * before the marker is complete, and losing the backup domain loses it.
 */
#ifndef FW_PARAM_RESET_H
#define FW_PARAM_RESET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t firmware_idle;
  uint8_t drive_feedback_healthy;
  uint8_t blade_feedback_healthy;
  float left_target_mps;
  float right_target_mps;
  int16_t left_measured_speed;
  int16_t right_measured_speed;
  uint8_t blade_target_on;
  uint8_t blade_active;
  uint16_t blade_reported_rpm;
} fw_param_reset_safety_t;

/** True only when fresh feedback and all firmware-side stop predicates agree. */
uint8_t fw_param_reset_is_safe(const fw_param_reset_safety_t *safety);

/** Enable RTC backup-register access without resetting the backup domain. */
void fw_param_reset_backup_init(void);

/** Raw retained request ID in DR9/10 (zero on a blank domain; may be partial). */
uint32_t fw_param_reset_last_request_id(void);

/** True only for a complete marker pair with a nonzero stored request id. */
uint8_t fw_param_reset_is_pending(void);

/**
 * Arm the next-boot reset for @p request_id. Repeating the active id is
 * idempotent; a different active id or an ID matching the retained value is
 * rejected. The retained value is only a bounded duplicate check: a cut during
 * a later arm may partially replace it before a marker becomes valid.
 * Every register write is read back before the marker can be acknowledged.
 */
uint8_t fw_param_reset_arm(uint32_t request_id);

/** Clear the active marker after flash erase and verification; retain its id. */
uint8_t fw_param_reset_clear(uint32_t request_id);

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAM_RESET_H */
