/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_reset.c
 * @brief RTC backup marker and safety predicate for explicit parameter reset.
 */
#include "fw_param_reset.h"

#include "adc.h"
#include "board.h"
#include "stm32f_board_hal.h"

/* DR1-4 hold charger state, DR5 the watchdog breadcrumb, DR6 the charger
 * format marker. Keep DR7-10 reserved for this reset protocol on both MCUs. */
#define FW_PARAM_RESET_MARKER_REG RTC_BKP_DR7
#define FW_PARAM_RESET_MARKER_COMPLEMENT_REG RTC_BKP_DR8
#define FW_PARAM_RESET_REQUEST_ID_LOW_REG RTC_BKP_DR9
#define FW_PARAM_RESET_REQUEST_ID_HIGH_REG RTC_BKP_DR10
#define FW_PARAM_RESET_MARKER_MAGIC 0xA55Au
#define FW_PARAM_RESET_MARKER_COMPLEMENT 0x5AA5u

static uint8_t fw_param_reset_marker_pair_matches(void) {
  return (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_REG) ==
                       FW_PARAM_RESET_MARKER_MAGIC &&
                   HAL_RTCEx_BKUPRead(&hrtc,
                                      FW_PARAM_RESET_MARKER_COMPLEMENT_REG) ==
                       FW_PARAM_RESET_MARKER_COMPLEMENT);
}

uint8_t fw_param_reset_is_safe(const fw_param_reset_safety_t *safety) {
  if (safety == 0) {
    return 0u;
  }
  return (uint8_t)(safety->firmware_idle != 0u &&
                   safety->drive_feedback_healthy != 0u &&
                   safety->blade_feedback_healthy != 0u &&
                   safety->left_target_mps == 0.0f &&
                   safety->right_target_mps == 0.0f &&
                   safety->left_measured_speed == 0 &&
                   safety->right_measured_speed == 0 &&
                   safety->blade_target_on == 0u &&
                   safety->blade_active == 0u &&
                   safety->blade_reported_rpm == 0u);
}

void fw_param_reset_backup_init(void) {
  /* The F4 HAL's BKP accessors dereference hrtc.Instance. Do not initialize or
   * reset the RTC/backup domain: DR1-6 belong to charger/watchdog state. */
  hrtc.Instance = RTC;
  __HAL_RCC_PWR_CLK_ENABLE();
#if BOARD_YARDFORCE500_VARIANT_ORIG
  __HAL_RCC_BKP_CLK_ENABLE();
#endif
  HAL_PWR_EnableBkUpAccess();
  HAL_PWR_DisableBkUpAccess();
}

uint32_t fw_param_reset_last_request_id(void) {
  const uint32_t low = HAL_RTCEx_BKUPRead(&hrtc,
                                         FW_PARAM_RESET_REQUEST_ID_LOW_REG) &
                       0xFFFFu;
  const uint32_t high = HAL_RTCEx_BKUPRead(&hrtc,
                                          FW_PARAM_RESET_REQUEST_ID_HIGH_REG) &
                        0xFFFFu;
  return (high << 16) | low;
}

uint8_t fw_param_reset_is_pending(void) {
  return (uint8_t)(fw_param_reset_marker_pair_matches() &&
                   fw_param_reset_last_request_id() != 0u);
}

uint8_t fw_param_reset_arm(uint32_t request_id) {
  if (request_id == 0u) {
    return 0u;
  }

  const uint32_t stored_id = fw_param_reset_last_request_id();
  if (fw_param_reset_is_pending()) {
    /* The pending marker owns one request. A duplicate is safe and idempotent;
     * a different request must never replace it. */
    return (uint8_t)(stored_id == request_id);
  }
  if (stored_id == request_id) {
    return 0u; /* retained-ID match: reject a possible delayed duplicate */
  }

  /* Invalidate the complement first, so a cut during a re-arm cannot leave a
   * valid marker pair. The marker is not armed until the complement is written
   * last after both request-id halves have been verified. */
  uint8_t armed = 0u;
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG, 0u);
  if (HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG) != 0u) {
    goto done;
  }

  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_REQUEST_ID_LOW_REG,
                      request_id & 0xFFFFu);
  if (HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_REQUEST_ID_LOW_REG) !=
      (request_id & 0xFFFFu)) {
    goto done;
  }
  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_REQUEST_ID_HIGH_REG,
                      (request_id >> 16) & 0xFFFFu);
  if (HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_REQUEST_ID_HIGH_REG) !=
          ((request_id >> 16) & 0xFFFFu) ||
      fw_param_reset_last_request_id() != request_id) {
    goto done;
  }

  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_MARKER_REG,
                      FW_PARAM_RESET_MARKER_MAGIC);
  if (HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_REG) !=
          FW_PARAM_RESET_MARKER_MAGIC ||
      HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG) != 0u) {
    goto done;
  }

  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG,
                      FW_PARAM_RESET_MARKER_COMPLEMENT);
  armed = (uint8_t)(fw_param_reset_marker_pair_matches() &&
                    fw_param_reset_last_request_id() == request_id);
done:
  HAL_PWR_DisableBkUpAccess();
  return armed;
}

uint8_t fw_param_reset_clear(uint32_t request_id) {
  if (request_id == 0u || !fw_param_reset_is_pending() ||
      fw_param_reset_last_request_id() != request_id) {
    return 0u;
  }

  /* Clear the first marker half to invalidate the pair before touching the
   * complement. If power fails during clear, the verified flash erase already
   * happened; the retained ID remains available for the bounded duplicate
   * check. */
  uint8_t cleared = 0u;
  HAL_PWR_EnableBkUpAccess();
  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_MARKER_REG, 0u);
  if (HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_REG) != 0u ||
      fw_param_reset_is_pending()) {
    goto done;
  }
  HAL_RTCEx_BKUPWrite(&hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG, 0u);
  cleared = (uint8_t)(HAL_RTCEx_BKUPRead(&hrtc, FW_PARAM_RESET_MARKER_REG) ==
                          0u &&
                      HAL_RTCEx_BKUPRead(
                          &hrtc, FW_PARAM_RESET_MARKER_COMPLEMENT_REG) == 0u &&
                      fw_param_reset_last_request_id() == request_id &&
                      !fw_param_reset_is_pending());
done:
  HAL_PWR_DisableBkUpAccess();
  return cleared;
}
