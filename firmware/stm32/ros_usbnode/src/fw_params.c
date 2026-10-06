/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_params.c
 * @brief Runtime parameter table, host protocol v8 and flash persistence.
 *        See fw_params.h for the lifecycle.
 */
#include "fw_params.h"

#include <math.h>
#include <string.h>

#include "stm32f_board_hal.h"
#include "board.h"
#include "main.h"
#include "drive_tuning_defaults.h"
#include "fw_param_log.h"
#include "fw_param_reset.h"
#include "fw_param_store.h"
#include "mowgli_comms.h"

_Static_assert(FW_PARAM_COUNT <= 32u, "report/dirty bitmasks are 32-bit");
_Static_assert(FW_PARAM_COUNT <= FW_PARAM_LOG_MAX_ENTRIES, "record entry limit");

/* Minimum spacing between two reports: a GET_PARAM(ALL) answer is ~25 packets,
 * and the USB TX queue drops (never blocks) when full. */
#define FW_PARAMS_REPORT_SPACING_MS 5u
#define FW_PARAMS_RESET_STATUS_INTERVAL_MS 1000u

#define FW_PARAMS_RECORD_MAX_WORDS \
  (FW_PARAM_LOG_HEADER_WORDS + 2u * FW_PARAM_COUNT + FW_PARAM_LOG_TRAILER_WORDS)

static float s_values[FW_PARAM_COUNT];
static float s_defaults[FW_PARAM_COUNT];
static uint8_t s_last_status[FW_PARAM_COUNT];
/* What the newest flash record holds (raw, as decoded). */
static float s_stored[FW_PARAM_COUNT];
static uint8_t s_stored_valid[FW_PARAM_COUNT];

static volatile uint32_t s_dirty_groups;
static volatile uint32_t s_report_pending;
static volatile uint8_t s_store_report_pending;
static volatile uint8_t s_unknown_report_pending;
static volatile uint16_t s_unknown_report_id;
static volatile uint8_t s_commit_requested;
static volatile uint8_t s_reset_requested;
static volatile uint32_t s_reset_request_id;
static volatile uint8_t s_reset_armed;
static volatile uint8_t s_reset_blocked;
static volatile uint32_t s_last_reset_request_id;

static uint8_t s_boot_source = PARAM_BOOT_DEFAULTS;
static uint8_t s_last_commit = PARAM_COMMIT_NONE;
static size_t s_next_free; /* word offset of the append point */

/* Commit in progress: programmed one word per fw_params_service() call. */
static uint32_t s_pending_record[FW_PARAMS_RECORD_MAX_WORDS];
static float s_pending_values[FW_PARAM_COUNT];
static size_t s_pending_words;
static size_t s_pending_pos;
static uint8_t s_pending_active;

static uint32_t s_last_report_ms;
static uint32_t s_last_reset_status_ms;

static uint8_t fw_params_area_is_erased(void) {
  const uint32_t *area = fw_param_store_area();
  const size_t words = fw_param_store_area_words();
  for (size_t i = 0; i < words; ++i) {
    if (area[i] != FW_PARAM_LOG_ERASED) {
      return 0u;
    }
  }
  return 1u;
}

static float fw_params_compiled_default(uint16_t id) {
  switch (id) {
    case FW_PARAM_TICKS_PER_METER: return (float)TICKS_PER_M;
    case FW_PARAM_WHEEL_KP: return WHEEL_PI_KP_PWM_PER_MPS;
    case FW_PARAM_WHEEL_KI: return WHEEL_PI_KI_PWM_PER_MPS_S;
    case FW_PARAM_WHEEL_KD: return WHEEL_PI_KD_DEFAULT;
    case FW_PARAM_WHEEL_INTEGRAL_LIMIT: return WHEEL_PI_INT_MAX_PWM;
    case FW_PARAM_PWM_PER_MPS: return (float)PWM_PER_MPS;
    case FW_PARAM_YAW_KP: return YAW_PI_KP_DEFAULT;
    case FW_PARAM_YAW_KI: return YAW_PI_KI_DEFAULT;
    case FW_PARAM_YAW_TRIM_LIMIT_MPS: return YAW_TRIM_LIMIT_MPS_DEFAULT;
    case FW_PARAM_YAW_LOOP_ENABLED: return YAW_LOOP_ENABLED_DEFAULT;
    case FW_PARAM_YAW_GYRO_SIGN: return YAW_GYRO_SIGN_DEFAULT;
    case FW_PARAM_YAW_GYRO_BIAS_RADPS: return 0.0f;
    case FW_PARAM_MAX_MPS: return (float)MAX_MPS;
    case FW_PARAM_WHEEL_BASE: return (float)WHEEL_BASE;
    case FW_PARAM_MAX_CHARGE_VOLTAGE: return (float)MAX_CHARGE_VOLTAGE;
    case FW_PARAM_MAX_CHARGE_CURRENT: return (float)MAX_CHARGE_CURRENT;
    case FW_PARAM_ONE_WHEEL_LIFT_MS: return (float)ONE_WHEEL_LIFT_EMERGENCY_MILLIS;
    case FW_PARAM_BOTH_WHEELS_LIFT_MS: return (float)BOTH_WHEELS_LIFT_EMERGENCY_MILLIS;
    case FW_PARAM_TILT_MS: return (float)TILT_EMERGENCY_MILLIS;
    case FW_PARAM_STOP_BUTTON_MS: return (float)STOP_BUTTON_EMERGENCY_MILLIS;
    case FW_PARAM_PLAY_CLEAR_MS: return (float)PLAY_BUTTON_CLEAR_EMERGENCY_MILLIS;
    case FW_PARAM_IMU_INCLINATION_THRESHOLD: return (float)IMU_ONBOARD_INCLINATION_THRESHOLD;
    default: return NAN;
  }
}

static uint8_t fw_params_is_volatile(size_t index) {
  return (FW_PARAM_SPECS[index].flags & FW_PARAM_FLAG_VOLATILE) != 0u;
}

static size_t fw_params_persistent_count(void) {
  size_t count = 0u;
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    if (!fw_params_is_volatile(i)) {
      ++count;
    }
  }
  return count;
}

static size_t fw_params_record_words(void) {
  return fw_param_log_record_words(fw_params_persistent_count());
}

/* Encode the persistent subset of @p values; returns the record size. */
static size_t fw_params_encode(const float *values, uint32_t *out) {
  uint16_t ids[FW_PARAM_COUNT];
  float persisted[FW_PARAM_COUNT];
  size_t count = 0u;
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    if (!fw_params_is_volatile(i)) {
      ids[count] = FW_PARAM_SPECS[i].id;
      persisted[count] = values[i];
      ++count;
    }
  }
  return fw_param_log_encode(ids, persisted, count, out, FW_PARAMS_RECORD_MAX_WORDS);
}

static uint8_t fw_params_same_bits(float a, float b) {
  return fw_param_log_float_bits(a) == fw_param_log_float_bits(b);
}

static void fw_params_mark_stored(const float *values) {
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    if (!fw_params_is_volatile(i)) {
      s_stored[i] = values[i];
      s_stored_valid[i] = 1u;
    }
  }
}

static void fw_params_load_record(const uint32_t *record) {
  const size_t entries = fw_param_log_entry_count(record);
  for (size_t e = 0; e < entries; ++e) {
    uint16_t id;
    float stored;
    fw_param_log_entry(record, e, &id, &stored);
    const int index = fw_param_index(id);
    if (index < 0 || fw_params_is_volatile((size_t)index)) {
      continue; /* a parameter this firmware no longer has */
    }
    float value;
    if (fw_param_coerce(&FW_PARAM_SPECS[index], stored, &value) != FW_PARAM_STATUS_REJECTED) {
      s_values[index] = value;
    }
    /* Keep the raw stored value: if the envelope moved in a firmware update
     * the coerced value differs, and the next commit rewrites the record. */
    s_stored[index] = stored;
    s_stored_valid[index] = 1u;
  }
}

void fw_params_init(void) {
  s_boot_source = PARAM_BOOT_DEFAULTS;
  s_last_commit = PARAM_COMMIT_NONE;
  s_pending_active = 0u;
  s_commit_requested = 0u;
  s_reset_requested = 0u;
  s_reset_request_id = 0u;
  s_reset_armed = 0u;
  s_reset_blocked = 0u;
  s_last_reset_request_id = fw_param_reset_last_request_id();
  s_last_reset_status_ms = 0u;
  s_dirty_groups = 0u;
  s_report_pending = 0u;
  s_store_report_pending = 0u;
  s_unknown_report_pending = 0u;
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    float value = 0.0f;
    /* A custom build (GUI flash form) may compile a default outside the
     * envelope: it is coerced into it like any other value. */
    (void)fw_param_coerce(&FW_PARAM_SPECS[i],
                          fw_params_compiled_default(FW_PARAM_SPECS[i].id), &value);
    s_defaults[i] = value;
    s_values[i] = value;
    s_last_status[i] = FW_PARAM_STATUS_OK;
    s_stored_valid[i] = 0u;
  }

  const uint32_t *area = fw_param_store_area();
  const size_t area_words = fw_param_store_area_words();

  /* An explicit request is consumed before the watchdog is armed. Keep the
   * retained RTC ID if marker clearing fails; this blocks commits in this boot
   * and leaves the bounded duplicate check available. */
  if (fw_param_reset_is_pending()) {
    s_reset_request_id = s_last_reset_request_id;
    s_reset_armed = 1u;
    s_next_free = area_words;
    if (fw_param_store_erase() == 0 && fw_params_area_is_erased() &&
        fw_param_reset_clear(s_reset_request_id)) {
      s_boot_source = PARAM_BOOT_FLASH_ERASED;
      s_next_free = 0u;
      s_reset_armed = 0u;
      s_last_commit = PARAM_COMMIT_NONE;
      debug_printf(" * Explicit parameter reset %lu: flash erased\r\n",
                   (unsigned long)s_reset_request_id);
    } else {
      s_reset_blocked = 1u;
      s_last_commit = PARAM_COMMIT_RESET_PENDING;
      debug_printf(" * Explicit parameter reset %lu: retry required\r\n",
                   (unsigned long)s_reset_request_id);
    }
    return;
  }

  const fw_param_log_scan_t scan = fw_param_log_scan(area, area_words);
  uint8_t loaded = 0u;
  if (scan.last_valid >= 0) {
    fw_params_load_record(&area[scan.last_valid]);
    s_boot_source = PARAM_BOOT_FLASH;
    loaded = 1u;
  }
  s_next_free = scan.next_free;

  if (scan.needs_erase || s_next_free + fw_params_record_words() > area_words) {
    if (loaded) {
      /* No second persistent bank exists. Erasing here would destroy the
       * only committed set before its replacement could be secured. Keep
       * both full logs and unappendable tails intact across every reboot. */
      s_next_free = area_words;
      s_last_commit = PARAM_COMMIT_LOG_FULL;
      debug_printf(" * Parameter log %s: retaining committed values\r\n",
                   scan.needs_erase ? "invalid tail" : "full");
      return;
    }
    debug_printf(" * Parameter log %s: erasing\r\n", scan.needs_erase ? "invalid" : "full");
    if (fw_param_store_erase() != 0) {
      /* Leave nothing appendable: commits report LOG_FULL until a boot
       * manages to erase. The compiled defaults still apply. */
      s_next_free = area_words;
      s_last_commit = PARAM_COMMIT_ERROR;
      return;
    }
    s_next_free = 0u;
    s_boot_source = PARAM_BOOT_FLASH_ERASED;
    for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
      s_stored_valid[i] = 0u;
    }
  }
  debug_printf(" * Parameters: %s, %u valid record(s)\r\n",
               s_boot_source == PARAM_BOOT_DEFAULTS ? "compiled defaults" : "loaded from flash",
               (unsigned)scan.valid_records);
}

float fw_params_get(uint16_t id) {
  const int index = fw_param_index(id);
  return index < 0 ? NAN : s_values[index];
}

uint8_t fw_params_set(uint16_t id, float requested) {
  const int index = fw_param_index(id);
  if (index < 0) {
    s_unknown_report_id = id;
    s_unknown_report_pending = 1u;
    return FW_PARAM_STATUS_UNKNOWN_ID;
  }
  float value;
  const fw_param_status_t status = fw_param_coerce(&FW_PARAM_SPECS[index], requested, &value);
  s_last_status[index] = (uint8_t)status;
  if (status != FW_PARAM_STATUS_REJECTED) {
    s_values[index] = value;
    s_dirty_groups |= (1u << FW_PARAM_SPECS[index].group);
  }
  s_report_pending |= (1u << (uint32_t)index);
  return (uint8_t)status;
}

uint32_t fw_params_take_dirty_groups(void) {
  __disable_irq();
  const uint32_t groups = s_dirty_groups;
  s_dirty_groups = 0u;
  __enable_irq();
  return groups;
}

void fw_params_mark_all_dirty(void) {
  __disable_irq();
  s_dirty_groups = (1u << FW_PARAM_GROUP_COUNT) - 1u;
  __enable_irq();
}

void fw_params_request_report(uint16_t id) {
  if (id == FW_PARAM_ID_ALL) {
    s_report_pending = (FW_PARAM_COUNT >= 32u) ? 0xFFFFFFFFu : ((1u << FW_PARAM_COUNT) - 1u);
    s_store_report_pending = 1u;
    return;
  }
  const int index = fw_param_index(id);
  if (index < 0) {
    s_unknown_report_id = id;
    s_unknown_report_pending = 1u;
    return;
  }
  s_report_pending |= (1u << (uint32_t)index);
}

void fw_params_request_commit(void) {
  if (s_reset_armed || s_reset_blocked) {
    s_last_commit = s_reset_armed ? PARAM_COMMIT_RESET_PENDING
                                  : PARAM_COMMIT_ERROR;
    s_store_report_pending = 1u;
    return;
  }
  s_commit_requested = 1u;
}

uint8_t fw_params_request_reset(uint32_t request_id) {
  if (request_id == 0u) {
    return 0u;
  }
  if (s_reset_armed) {
    if (s_reset_request_id != request_id) {
      return 0u;
    }
    s_store_report_pending = 1u;
    return 1u;
  }
  if (s_reset_blocked) {
    return 0u;
  }
  if (s_reset_requested) {
    return (uint8_t)(s_reset_request_id == request_id);
  }
  if (s_last_reset_request_id == request_id) {
    return 0u;
  }
  s_reset_request_id = request_id;
  s_reset_requested = 1u;
  return 1u;
}

void fw_params_report_reset_rejected(void) {
  if (s_reset_armed) {
    s_last_commit = PARAM_COMMIT_RESET_PENDING;
  } else {
    s_last_commit = PARAM_COMMIT_ERROR;
  }
  s_store_report_pending = 1u;
}

static void fw_params_queue_all_reports(void) {
  __disable_irq();
  s_report_pending = (FW_PARAM_COUNT >= 32u) ? 0xFFFFFFFFu : ((1u << FW_PARAM_COUNT) - 1u);
  s_store_report_pending = 1u;
  __enable_irq();
}

static void fw_params_start_commit(void) {
  if (s_reset_armed || s_reset_blocked) {
    s_last_commit = s_reset_armed ? PARAM_COMMIT_RESET_PENDING
                                  : PARAM_COMMIT_ERROR;
    s_store_report_pending = 1u;
    return;
  }
  float snapshot[FW_PARAM_COUNT];
  __disable_irq();
  memcpy(snapshot, s_values, sizeof(snapshot));
  __enable_irq();

  uint8_t unchanged = 1u;
  for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
    if (!fw_params_is_volatile(i) &&
        (!s_stored_valid[i] || !fw_params_same_bits(s_stored[i], snapshot[i]))) {
      unchanged = 0u;
      break;
    }
  }
  if (unchanged) {
    s_last_commit = PARAM_COMMIT_UNCHANGED;
    s_store_report_pending = 1u;
    return;
  }
  const size_t words = fw_params_encode(snapshot, s_pending_record);
  if (words == 0u || s_next_free + words > fw_param_store_area_words()) {
    /* Preserve the last committed set. Without a second persistent bank a
     * reboot cannot safely reclaim a full log; live values still apply. */
    s_last_commit = PARAM_COMMIT_LOG_FULL;
    s_store_report_pending = 1u;
    return;
  }
  memcpy(s_pending_values, snapshot, sizeof(snapshot));
  s_pending_words = words;
  s_pending_pos = 0u;
  s_pending_active = 1u;
  s_last_commit = PARAM_COMMIT_PENDING;
}

static void fw_params_step_commit(void) {
  if (fw_param_store_program_word(s_next_free + s_pending_pos, s_pending_record[s_pending_pos]) !=
      0) {
    /* Most likely space that was not erased: stop appending for this boot
     * (the record's CRC/commit word make it invisible). A later boot may
     * append only if the scanner can locate safe erased space. */
    s_pending_active = 0u;
    s_next_free = fw_param_store_area_words();
    s_last_commit = PARAM_COMMIT_ERROR;
    s_store_report_pending = 1u;
    return;
  }
  if (++s_pending_pos < s_pending_words) {
    return;
  }
  s_pending_active = 0u;
  s_next_free += s_pending_words;
  fw_params_mark_stored(s_pending_values);
  s_last_commit = PARAM_COMMIT_WRITTEN;
  /* The PERSISTED flag of every parameter may have changed. */
  fw_params_queue_all_reports();
}

static void fw_params_send_value(size_t index) {
  pkt_param_value_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = PKT_ID_PARAM_VALUE;
  pkt.param_id = FW_PARAM_SPECS[index].id;
  pkt.status = s_last_status[index];
  const float value = s_values[index];
  if (fw_params_is_volatile(index)) {
    pkt.flags |= PARAM_VALUE_FLAG_VOLATILE;
  } else if (s_stored_valid[index] && fw_params_same_bits(s_stored[index], value)) {
    pkt.flags |= PARAM_VALUE_FLAG_PERSISTED;
  }
  pkt.value = value;
  pkt.default_value = s_defaults[index];
  pkt.min_value = FW_PARAM_SPECS[index].min;
  pkt.max_value = FW_PARAM_SPECS[index].max;
  mowgli_comms_send(&pkt, sizeof(pkt));
}

static void fw_params_send_unknown(uint16_t id) {
  pkt_param_value_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = PKT_ID_PARAM_VALUE;
  pkt.param_id = id;
  pkt.status = FW_PARAM_STATUS_UNKNOWN_ID;
  pkt.value = NAN;
  pkt.default_value = NAN;
  pkt.min_value = NAN;
  pkt.max_value = NAN;
  mowgli_comms_send(&pkt, sizeof(pkt));
}

static void fw_params_send_store_status(void) {
  pkt_param_store_status_t pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.type = PKT_ID_PARAM_STORE_STATUS;
  pkt.boot_source = s_boot_source;
  pkt.last_commit = s_last_commit;
  const size_t area_words = fw_param_store_area_words();
  const size_t free_words = (s_next_free < area_words) ? area_words - s_next_free : 0u;
  const size_t records_left = free_words / fw_params_record_words();
  pkt.records_left = (uint16_t)(records_left > 0xFFFFu ? 0xFFFFu : records_left);
  pkt.param_count = (uint16_t)FW_PARAM_COUNT;
  pkt.reset_request_id = s_last_reset_request_id;
  mowgli_comms_send(&pkt, sizeof(pkt));
}

/* Pop and send at most one queued report. */
static void fw_params_send_one_report(void) {
  if (s_unknown_report_pending) {
    __disable_irq();
    const uint16_t id = s_unknown_report_id;
    s_unknown_report_pending = 0u;
    __enable_irq();
    fw_params_send_unknown(id);
    return;
  }
  __disable_irq();
  const uint32_t pending = s_report_pending;
  uint32_t bit = pending & (0u - pending); /* lowest set bit */
  s_report_pending = pending & ~bit;
  __enable_irq();
  if (bit != 0u) {
    size_t index = 0u;
    while ((bit >>= 1u) != 0u) {
      ++index;
    }
    if (index < FW_PARAM_COUNT) {
      fw_params_send_value(index);
    }
    return;
  }
  if (s_store_report_pending) {
    s_store_report_pending = 0u;
    fw_params_send_store_status();
  }
}

void fw_params_service(uint32_t now_ms) {
  if (s_reset_requested) {
    const uint32_t request_id = s_reset_request_id;
    /* An older in-progress append may leave a harmless torn tail, but no word
     * is programmed after the reset intent has been accepted. Keep the queued
     * flag set while RTC writes are in flight: a USB interrupt may repeat this
     * ID, but cannot replace the owner with a different reset request. */
    s_pending_active = 0u;
    s_commit_requested = 0u;
    if (fw_param_reset_arm(request_id)) {
      s_reset_armed = 1u;
      s_last_reset_request_id = request_id;
      s_last_commit = PARAM_COMMIT_RESET_PENDING;
    } else {
      s_reset_blocked = 1u;
      if (fw_param_reset_is_pending() &&
          fw_param_reset_last_request_id() == request_id) {
        s_reset_armed = 1u;
        s_last_reset_request_id = request_id;
        s_last_commit = PARAM_COMMIT_RESET_PENDING;
      } else {
        s_last_commit = PARAM_COMMIT_ERROR;
      }
    }
    if (s_reset_armed) {
      s_last_reset_status_ms = now_ms;
    }
    /* Publish the armed/blocked state before releasing request ownership. */
    s_reset_requested = 0u;
    s_store_report_pending = 1u;
  }

  if (s_reset_armed || s_reset_blocked) {
    if (s_commit_requested) {
      s_commit_requested = 0u;
      s_last_commit = s_reset_armed ? PARAM_COMMIT_RESET_PENDING
                                    : PARAM_COMMIT_ERROR;
      s_store_report_pending = 1u;
    }
    s_pending_active = 0u;
  } else if (s_pending_active) {
    fw_params_step_commit();
  } else if (s_commit_requested) {
    s_commit_requested = 0u;
    fw_params_start_commit();
  }
  if (s_reset_armed &&
      (uint32_t)(now_ms - s_last_reset_status_ms) >=
          FW_PARAMS_RESET_STATUS_INTERVAL_MS) {
    s_last_reset_status_ms = now_ms;
    /* Queue the existing status packet only. Do not alter last_commit here:
     * failed, unarmed reset requests must remain ERROR. */
    s_store_report_pending = 1u;
  }
  if ((uint32_t)(now_ms - s_last_report_ms) >= FW_PARAMS_REPORT_SPACING_MS) {
    s_last_report_ms = now_ms;
    fw_params_send_one_report();
  }
}
