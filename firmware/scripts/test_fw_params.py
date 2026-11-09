#!/usr/bin/env python3
"""Exercise production runtime parameter/reset firmware code against
a simulated flash: boot load, envelope coercion, reports, commit dedupe, torn
writes, full log, foreign data and flash faults.

The flash model behaves like the STM32 parts: an erase sets every word to
0xFFFFFFFF, and programming a word that is not erased fails (the HAL returns an
error) instead of silently OR-ing bits. Both board variants are compiled.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'

SHIM = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "drive_tuning_defaults.h"
#include "fw_param_catalog.h"
#include "fw_param_log.h"
#include "fw_param_reset.h"
#include "fw_param_store.h"
#include "mowgli_protocol.h"
#include "fw_params.h"
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static void debug_printf(const char *fmt, ...) { (void)fmt; }

/* ---- simulated RTC backup registers ---- */
typedef struct { void *Instance; } RTC_HandleTypeDef;
static int fake_rtc_instance;
#define RTC ((void *)&fake_rtc_instance)
#define RTC_BKP_DR7 7u
#define RTC_BKP_DR8 8u
#define RTC_BKP_DR9 9u
#define RTC_BKP_DR10 10u
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define __HAL_RCC_BKP_CLK_ENABLE() ((void)0)
RTC_HandleTypeDef hrtc;
static uint32_t backup_regs[11];
static unsigned backup_writes, fail_backup_write_at;
static uint8_t backup_access;
static unsigned inject_reset_on_write;
static uint32_t inject_reset_duplicate_id, inject_reset_conflicting_id;
static uint8_t inject_reset_duplicate_result, inject_reset_conflicting_result;
void HAL_PWR_EnableBkUpAccess(void) { backup_access = 1u; }
void HAL_PWR_DisableBkUpAccess(void) { backup_access = 0u; }
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *rtc, uint32_t reg) {
    assert(rtc == &hrtc && rtc->Instance == RTC && reg < 11u);
    return backup_regs[reg];
}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *rtc, uint32_t reg, uint32_t value) {
    assert(backup_access && rtc == &hrtc && rtc->Instance == RTC && reg < 11u);
    ++backup_writes;
    if (!fail_backup_write_at || backup_writes != fail_backup_write_at)
        backup_regs[reg] = value & 0xFFFFu;
    if (inject_reset_on_write && backup_writes == inject_reset_on_write) {
        inject_reset_duplicate_result = fw_params_request_reset(inject_reset_duplicate_id);
        inject_reset_conflicting_result = fw_params_request_reset(inject_reset_conflicting_id);
    }
}

/* ---- simulated flash ---- */
#define MAX_WORDS 4096u
static uint32_t flash[MAX_WORDS];
static size_t area_words = MAX_WORDS;
static unsigned erases, programs, fail_erase, fail_program_at;
static uint8_t corrupt_after_erase;
const uint32_t *fw_param_store_area(void) { return flash; }
size_t fw_param_store_area_words(void) { return area_words; }
int fw_param_store_erase(void) {
    if (fail_erase) return -1;
    ++erases;
    for (size_t i = 0; i < area_words; ++i) flash[i] = 0xFFFFFFFFu;
    if (corrupt_after_erase && area_words) flash[0] = 0u;
    return 0;
}
int fw_param_store_program_word(size_t off, uint32_t value) {
    ++programs;
    if (fail_program_at && programs == fail_program_at) return -1;
    if (off >= area_words || flash[off] != 0xFFFFFFFFu) return -1;
    flash[off] = value;
    return 0;
}

/* ---- captured USB traffic ---- */
static unsigned sent_values, sent_store, sent_unknown;
static pkt_param_value_t last_value;
static pkt_param_store_status_t last_store;
static uint32_t last_send_ms, now_ms;
static int spacing_violations;
void mowgli_comms_send(const void *packet, size_t len) {
    const uint8_t type = ((const uint8_t *)packet)[0];
    if (sent_values + sent_store && now_ms - last_send_ms < 5u) ++spacing_violations;
    last_send_ms = now_ms;
    if (type == PKT_ID_PARAM_VALUE) {
        assert(len == sizeof(pkt_param_value_t));
        memcpy(&last_value, packet, len);
        if (last_value.status == FW_PARAM_STATUS_UNKNOWN_ID) ++sent_unknown; else ++sent_values;
    } else if (type == PKT_ID_PARAM_STORE_STATUS) {
        assert(len == sizeof(pkt_param_store_status_t));
        memcpy(&last_store, packet, len);
        ++sent_store;
    } else {
        assert(!"unexpected packet");
    }
}
'''

TEST = r'''
static void run(unsigned ms) { for (unsigned i = 0; i < ms; ++i) { ++now_ms; fw_params_service(now_ms); } }
static void boot(void) {
    erases = programs = 0;
    sent_values = sent_store = sent_unknown = 0;
    fw_param_reset_backup_init();
    fw_params_init();
}
static void wipe(size_t words) {
    area_words = words;
    for (size_t i = 0; i < MAX_WORDS; ++i) flash[i] = 0xFFFFFFFFu;
    fail_erase = fail_program_at = 0;
    corrupt_after_erase = 0u;
    memset(backup_regs, 0, sizeof(backup_regs));
    backup_writes = fail_backup_write_at = 0u;
    inject_reset_on_write = 0u;
    inject_reset_duplicate_id = inject_reset_conflicting_id = 0u;
    inject_reset_duplicate_result = inject_reset_conflicting_result = 0u;
    backup_access = 0u;
}
static uint8_t commit(void) {
    fw_params_request_commit();
    run(500);  /* one word per service call + reports */
    fw_params_request_report(FW_PARAM_ID_ALL);
    run(500);
    return last_store.last_commit;
}
static float stored_record_value(uint16_t id) {
    fw_param_log_scan_t scan = fw_param_log_scan(flash, area_words);
    assert(scan.last_valid >= 0);
    const uint32_t *rec = &flash[scan.last_valid];
    for (size_t e = 0; e < fw_param_log_entry_count(rec); ++e) {
        uint16_t rid; float v; fw_param_log_entry(rec, e, &rid, &v);
        if (rid == id) return v;
    }
    return NAN;
}

int main(void) {
    /* The reset predicate is closed unless fresh drive+blade feedback and
     * every command/measurement agree that the robot is stopped. */
    {
        fw_param_reset_safety_t safety = {1u, 1u, 1u, 0.0f, 0.0f,
                                           0, 0, 0u, 0u, 0u};
        assert(fw_param_reset_is_safe(&safety));
        safety.firmware_idle = 0u; assert(!fw_param_reset_is_safe(&safety));
        safety.firmware_idle = 1u;
        safety.drive_feedback_healthy = 0u; assert(!fw_param_reset_is_safe(&safety));
        safety.drive_feedback_healthy = 1u;
        safety.blade_feedback_healthy = 0u; assert(!fw_param_reset_is_safe(&safety));
        safety.blade_feedback_healthy = 1u;
        safety.left_target_mps = 0.01f; assert(!fw_param_reset_is_safe(&safety));
        safety.left_target_mps = 0.0f;
        safety.right_measured_speed = 1; assert(!fw_param_reset_is_safe(&safety));
        safety.right_measured_speed = 0;
        safety.blade_target_on = 1u; assert(!fw_param_reset_is_safe(&safety));
        safety.blade_target_on = 0u;
        safety.blade_active = 1u; assert(!fw_param_reset_is_safe(&safety));
        safety.blade_active = 0u;
        safety.blade_reported_rpm = 1u; assert(!fw_param_reset_is_safe(&safety));
    }

    /* 1. Blank flash: compiled defaults, nothing erased or written. */
    wipe(MAX_WORDS); boot();
    assert(erases == 0 && programs == 0);
    assert(fw_params_get(FW_PARAM_TILT_MS) == (float)TILT_EMERGENCY_MILLIS);
    assert(fw_params_get(FW_PARAM_MAX_CHARGE_VOLTAGE) == (float)MAX_CHARGE_VOLTAGE);
    assert(fw_params_get(FW_PARAM_WHEEL_KP) == WHEEL_PI_KP_PWM_PER_MPS);
    assert(isnan(fw_params_get(999)));
    fw_params_request_report(FW_PARAM_ID_ALL); run(400);
    assert(sent_values == FW_PARAM_COUNT && sent_store == 1 && spacing_violations == 0);
    assert(last_store.boot_source == PARAM_BOOT_DEFAULTS && last_store.param_count == FW_PARAM_COUNT);

    /* 2. Coercion into the envelope; non-finite rejected; groups marked dirty. */
    (void)fw_params_take_dirty_groups();
    assert(fw_params_set(FW_PARAM_TILT_MS, 800.0f) == FW_PARAM_STATUS_OK);
    assert(fw_params_get(FW_PARAM_TILT_MS) == 800.0f);
    assert(fw_params_take_dirty_groups() == (1u << FW_PARAM_GROUP_EMERGENCY));
    assert(fw_params_take_dirty_groups() == 0u);
    assert(fw_params_set(FW_PARAM_MAX_CHARGE_VOLTAGE, 42.0f) == FW_PARAM_STATUS_CLAMPED);
    assert(fw_params_get(FW_PARAM_MAX_CHARGE_VOLTAGE) == 29.4f);
    assert(fw_params_set(FW_PARAM_TILT_MS, NAN) == FW_PARAM_STATUS_REJECTED);
    assert(fw_params_get(FW_PARAM_TILT_MS) == 800.0f);
    assert(fw_params_take_dirty_groups() == (1u << FW_PARAM_GROUP_CHARGE));
    sent_values = 0; run(50);
    assert(sent_values >= 1 && last_value.param_id == FW_PARAM_TILT_MS &&
           last_value.status == FW_PARAM_STATUS_REJECTED && last_value.value == 800.0f &&
           !(last_value.flags & PARAM_VALUE_FLAG_PERSISTED));
    assert(fw_params_set(999, 1.0f) == FW_PARAM_STATUS_UNKNOWN_ID);
    run(20); assert(sent_unknown == 1);

    /* 3. Commit, then a reboot loads the stored values. */
    assert(commit() == PARAM_COMMIT_WRITTEN);
    assert(stored_record_value(FW_PARAM_TILT_MS) == 800.0f);
    assert(isnan(stored_record_value(FW_PARAM_YAW_GYRO_BIAS_RADPS)));  /* volatile */
    fw_params_request_report(FW_PARAM_TILT_MS); run(20);
    assert(last_value.param_id == FW_PARAM_TILT_MS && (last_value.flags & PARAM_VALUE_FLAG_PERSISTED));
    boot();
    assert(erases == 0 && fw_params_get(FW_PARAM_TILT_MS) == 800.0f);
    assert(fw_params_get(FW_PARAM_MAX_CHARGE_VOLTAGE) == 29.4f);
    fw_params_request_report(FW_PARAM_ID_ALL); run(400);
    assert(last_store.boot_source == PARAM_BOOT_FLASH);

    /* 4. An unchanged set is not rewritten; the volatile bias never triggers a write. */
    programs = 0;
    assert(commit() == PARAM_COMMIT_UNCHANGED && programs == 0);
    fw_params_set(FW_PARAM_YAW_GYRO_BIAS_RADPS, 0.1f);
    assert(commit() == PARAM_COMMIT_UNCHANGED && programs == 0);
    boot(); assert(fw_params_get(FW_PARAM_YAW_GYRO_BIAS_RADPS) == 0.0f);

    /* 5. Power lost mid-commit: the old set survives and the log still appends. */
    fw_params_set(FW_PARAM_TILT_MS, 300.0f);
    fw_params_request_commit();
    run(6);  /* a few words only */
    boot();
    assert(erases == 0 && fw_params_get(FW_PARAM_TILT_MS) == 800.0f);
    fw_params_set(FW_PARAM_TILT_MS, 300.0f);
    assert(commit() == PARAM_COMMIT_WRITTEN);
    boot(); assert(erases == 0 && fw_params_get(FW_PARAM_TILT_MS) == 300.0f);

    /* 6. Full log: no boot may erase the only committed parameter copy.
     *    Repeated boots and refused commits leave it byte-identical. */
    const size_t rec = fw_param_log_record_words(FW_PARAM_COUNT - 1u);
    wipe(rec * 3u); boot();
    for (int i = 0; i < 3; ++i) {
        fw_params_set(FW_PARAM_TILT_MS, 100.0f + (float)i);
        assert(commit() == PARAM_COMMIT_WRITTEN);
    }
    fw_params_set(FW_PARAM_TILT_MS, 200.0f);
    assert(commit() == PARAM_COMMIT_LOG_FULL);
    assert(last_store.records_left == 0);
    uint32_t saved_flash[MAX_WORDS];
    memcpy(saved_flash, flash, sizeof(flash));
    /* Including failure at each would-be replacement word: there must be no
     * erase or write to interrupt when a committed copy is retained. */
    for (size_t word = 0; word <= rec; ++word) {
        fail_program_at = (unsigned)word;
        boot();
        assert(erases == 0 && programs == 0);
        assert(fw_params_get(FW_PARAM_TILT_MS) == 102.0f);
        assert(stored_record_value(FW_PARAM_TILT_MS) == 102.0f);
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
        fw_params_request_report(FW_PARAM_ID_ALL); run(400);
        assert(last_store.boot_source == PARAM_BOOT_FLASH && last_store.records_left == 0);
        assert(last_store.last_commit == PARAM_COMMIT_LOG_FULL);
        fw_params_request_report(FW_PARAM_TILT_MS); run(20);
        assert(last_value.flags & PARAM_VALUE_FLAG_PERSISTED);
        assert(commit() == PARAM_COMMIT_UNCHANGED);
        fw_params_set(FW_PARAM_TILT_MS, 200.0f);
        assert(commit() == PARAM_COMMIT_LOG_FULL);
        fw_params_request_report(FW_PARAM_TILT_MS); run(20);
        assert(!(last_value.flags & PARAM_VALUE_FLAG_PERSISTED));
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
    }
    fail_program_at = 0;

    /* Reset after every append word, including the final commit marker.
     * Boot must retain the prior set until the new set is fully committed,
     * without modifying either a full log or a torn append. */
    for (size_t written = 0; written <= rec; ++written) {
        wipe(rec * 2u); boot();
        fw_params_set(FW_PARAM_TILT_MS, 321.0f);
        assert(commit() == PARAM_COMMIT_WRITTEN);
        fw_params_set(FW_PARAM_TILT_MS, 400.0f);
        fw_params_request_commit();
        run(1);  /* start the asynchronous append */
        run((unsigned)written);
        memcpy(saved_flash, flash, sizeof(flash));
        boot();
        assert(erases == 0 && programs == 0);
        assert(fw_params_get(FW_PARAM_TILT_MS) == (written == rec ? 400.0f : 321.0f));
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
    }

    /* F103 word programming can be interrupted after its low halfword. The
     * first two words have narrowly recognized tombstones; later words have a
     * known record length from the complete header. In every case the old
     * committed set survives, the append point stays after programmed bits,
     * and a later valid append remains visible after another reboot. */
    for (size_t partial_word = 0; partial_word < rec; ++partial_word) {
        wipe(rec * 4u); boot();
        fw_params_set(FW_PARAM_TILT_MS, 321.0f);
        assert(commit() == PARAM_COMMIT_WRITTEN);
        fw_params_set(FW_PARAM_TILT_MS, 400.0f);
        fw_params_request_commit(); run(1u + (unsigned)partial_word);
        const size_t partial_at = rec + partial_word;
        uint16_t ids[FW_PARAM_COUNT]; float values[FW_PARAM_COUNT];
        size_t count = 0u;
        for (size_t i = 0; i < FW_PARAM_COUNT; ++i) {
            if ((FW_PARAM_SPECS[i].flags & FW_PARAM_FLAG_VOLATILE) == 0u) {
                ids[count] = FW_PARAM_SPECS[i].id;
                values[count] = fw_params_get(ids[count]);
                ++count;
            }
        }
        uint32_t candidate[FW_PARAM_COUNT * 2u + 4u];
        const size_t candidate_words = fw_param_log_encode(ids, values, count,
                                                             candidate,
                                                             sizeof(candidate) / sizeof(candidate[0]));
        assert(candidate_words == rec);
        flash[partial_at] = 0xFFFF0000u | (candidate[partial_word] & 0xFFFFu);
        memcpy(saved_flash, flash, sizeof(flash));
        boot();
        assert(erases == 0u && programs == 0u);
        assert(fw_params_get(FW_PARAM_TILT_MS) == 321.0f);
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
        fw_params_set(FW_PARAM_TILT_MS, 450.0f);
        assert(commit() == PARAM_COMMIT_WRITTEN);
        boot();
        assert(erases == 0u && fw_params_get(FW_PARAM_TILT_MS) == 450.0f);
    }

    /* A header cut after its first F103 halfword and a magic cut after its
     * first halfword are both recoverable even when followed by later records.
     * The exact two-word prefix is retained forever and never programmed over. */
    {
        const uint32_t torn_magic[][2] = {
            {0xFFFF524Du, FW_PARAM_LOG_ERASED},
            {FW_PARAM_LOG_MAGIC, FW_PARAM_LOG_ERASED},
            {FW_PARAM_LOG_MAGIC, 0xFFFF0001u},
        };
        for (size_t t = 0; t < sizeof(torn_magic) / sizeof(torn_magic[0]); ++t) {
            wipe(MAX_WORDS);
            uint16_t id = FW_PARAM_TILT_MS; float old_value = 321.0f;
            const size_t old_words = fw_param_log_encode(&id, &old_value, 1u, flash, MAX_WORDS);
            flash[old_words] = torn_magic[t][0];
            flash[old_words + 1u] = torn_magic[t][1];
            fw_param_log_scan_t scan = fw_param_log_scan(flash, area_words);
            assert(scan.last_valid == 0 && scan.next_free == old_words + 2u && !scan.needs_erase);

            float new_value = 450.0f;
            const size_t new_words = fw_param_log_encode(&id, &new_value, 1u,
                                                          &flash[scan.next_free],
                                                          area_words - scan.next_free);
            assert(new_words == old_words);
            scan = fw_param_log_scan(flash, area_words);
            assert(scan.last_valid == (long)(old_words + 2u));
            assert(scan.next_free == old_words + 2u + new_words && !scan.needs_erase);
            assert(fw_param_log_entry_count(&flash[scan.last_valid]) == 1u);
            assert(stored_record_value(FW_PARAM_TILT_MS) == 450.0f);
        }
    }

    /* Tombstones may repeat, but foreign/non-erased suffixes still fail closed
     * even when a valid older record has already been located. */
    {
        wipe(MAX_WORDS);
        uint16_t id = FW_PARAM_TILT_MS; float value = 321.0f;
        const size_t old_words = fw_param_log_encode(&id, &value, 1u, flash, MAX_WORDS);
        flash[old_words] = 0xFFFF524Du;
        flash[old_words + 1u] = FW_PARAM_LOG_ERASED;
        flash[old_words + 2u] = FW_PARAM_LOG_MAGIC;
        flash[old_words + 3u] = 0xFFFF0001u;
        fw_param_log_scan_t scan = fw_param_log_scan(flash, area_words);
        assert(scan.last_valid == 0 && scan.next_free == old_words + 4u && !scan.needs_erase);
        flash[scan.next_free] = 0x20005000u;
        scan = fw_param_log_scan(flash, area_words);
        assert(scan.last_valid == 0 && scan.needs_erase && scan.next_free == area_words);
    }

    /* Unrecognized tail data after a valid record blocks appending, but must
     * never cause the preceding committed record to be erased. */
    wipe(rec * 3u); boot();
    fw_params_set(FW_PARAM_TILT_MS, 321.0f);
    assert(commit() == PARAM_COMMIT_WRITTEN);
    flash[rec] = 0x20005000u;  /* foreign tail: not one of the safe prefixes */
    memcpy(saved_flash, flash, sizeof(flash));
    for (int reboot = 0; reboot < 3; ++reboot) {
        boot();
        assert(erases == 0 && programs == 0 && fw_params_get(FW_PARAM_TILT_MS) == 321.0f);
        fw_params_request_report(FW_PARAM_ID_ALL); run(400);
        assert(last_store.boot_source == PARAM_BOOT_FLASH && last_store.records_left == 0);
        assert(last_store.last_commit == PARAM_COMMIT_LOG_FULL);
        fw_params_set(FW_PARAM_TILT_MS, 400.0f);
        assert(commit() == PARAM_COMMIT_LOG_FULL);
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
    }

    /* 7. Foreign data (e.g. stock firmware leftovers): erased, defaults apply. */
    wipe(MAX_WORDS); flash[0] = 0x20005000u; flash[1] = 0x08000131u; boot();
    assert(erases == 1 && fw_params_get(FW_PARAM_TILT_MS) == (float)TILT_EMERGENCY_MILLIS);
    assert(flash[0] == 0xFFFFFFFFu);

    /* 8. A stored value outside a (newer, narrower) envelope is coerced at boot
     *    and the record is rewritten on the next commit. */
    wipe(MAX_WORDS);
    { uint16_t id = FW_PARAM_TILT_MS; float v = 90000.0f;
      assert(fw_param_log_encode(&id, &v, 1, flash, MAX_WORDS) > 0); }
    boot();
    assert(fw_params_get(FW_PARAM_TILT_MS) == FW_ENVELOPE_TILT_MAX_MS);
    assert(commit() == PARAM_COMMIT_WRITTEN);
    assert(stored_record_value(FW_PARAM_TILT_MS) == FW_ENVELOPE_TILT_MAX_MS);

    /* 9. Flash faults never corrupt the running values. */
    wipe(MAX_WORDS); flash[0] = 1u; fail_erase = 1; boot();
    fw_params_set(FW_PARAM_TILT_MS, 400.0f);
    assert(commit() == PARAM_COMMIT_LOG_FULL && fw_params_get(FW_PARAM_TILT_MS) == 400.0f);
    wipe(MAX_WORDS); boot();
    fw_params_set(FW_PARAM_TILT_MS, 450.0f);
    fail_program_at = programs + 3u;
    assert(commit() == PARAM_COMMIT_ERROR && fw_params_get(FW_PARAM_TILT_MS) == 450.0f);
    fail_program_at = 0;
    assert(commit() == PARAM_COMMIT_LOG_FULL);  /* no appends in this boot */

    /* 10. RTC reset marker ordering, request-ID ownership and interrupted
     * writes. A marker is acknowledged only after the complete ID and both
     * complementary marker registers read back. */
    {
        const uint32_t request_id = 0x12345678u;
        for (unsigned failed_write = 1u; failed_write <= 5u; ++failed_write) {
            memset(backup_regs, 0, sizeof(backup_regs));
            backup_regs[RTC_BKP_DR7] = 0x1357u;
            backup_regs[RTC_BKP_DR8] = 0x2468u;
            backup_writes = 0u;
            fail_backup_write_at = failed_write;
            fw_param_reset_backup_init();
            assert(!fw_param_reset_arm(request_id));
            assert(!fw_param_reset_is_pending());
            assert(backup_access == 0u);
        }
        memset(backup_regs, 0, sizeof(backup_regs));
        backup_writes = fail_backup_write_at = 0u;
        fw_param_reset_backup_init();
        assert(fw_param_reset_last_request_id() == 0u);
        assert(!fw_param_reset_is_pending());
        assert(fw_param_reset_arm(request_id));
        assert(fw_param_reset_is_pending());
        assert(backup_regs[RTC_BKP_DR7] == 0xA55Au &&
               backup_regs[RTC_BKP_DR8] == 0x5AA5u &&
               backup_regs[RTC_BKP_DR9] == 0x5678u &&
               backup_regs[RTC_BKP_DR10] == 0x1234u);
        const unsigned writes_after_arm = backup_writes;
        assert(fw_param_reset_arm(request_id)); /* active duplicate is idempotent */
        assert(backup_writes == writes_after_arm);
        assert(!fw_param_reset_arm(0x87654321u)); /* another ID cannot take ownership */
        assert(backup_writes == writes_after_arm &&
               fw_param_reset_last_request_id() == request_id);

        backup_writes = 0u; fail_backup_write_at = 1u;
        assert(!fw_param_reset_clear(request_id));
        assert(fw_param_reset_is_pending()); /* failed invalidation leaves owner armed */
        backup_writes = 0u; fail_backup_write_at = 2u;
        assert(!fw_param_reset_clear(request_id));
        assert(!fw_param_reset_is_pending()); /* first clear half invalidated the pair */
        assert(fw_param_reset_last_request_id() == request_id);

        fail_backup_write_at = 0u; backup_writes = 0u;
        const uint32_t next_id = 0xA1B2C3D4u;
        assert(fw_param_reset_arm(next_id));
        assert(fw_param_reset_clear(next_id));
        const unsigned writes_after_clear = backup_writes;
        assert(!fw_param_reset_arm(next_id)); /* consumed duplicate cannot erase again */
        assert(backup_writes == writes_after_clear);
        assert(fw_param_reset_last_request_id() == next_id &&
               !fw_param_reset_is_pending());
    }

    /* Pending store status repeats once per second with wrap-safe monotonic
     * timing, then stops after the next boot consumes the reset. */
    {
        const uint32_t request_id = 0x55667788u;
        wipe(MAX_WORDS);
        now_ms = UINT32_MAX - 500u;
        boot();
        assert(fw_params_request_reset(request_id));
        run(1u);
        assert(fw_param_reset_is_pending());
        assert(sent_store == 1u &&
               last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == request_id);
        const unsigned reports_after_arm = sent_store;
        const unsigned backup_writes_after_arm = backup_writes;
        run(999u);
        assert(sent_store == reports_after_arm);
        run(1u);
        assert(sent_store == reports_after_arm + 1u &&
               last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == request_id);
        run(1000u);
        assert(sent_store == reports_after_arm + 2u &&
               last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == request_id);
        assert(backup_writes == backup_writes_after_arm); /* status is not reset retry */

        boot();
        assert(!fw_param_reset_is_pending() && erases == 1u);
        run(1500u);
        assert(sent_store == 0u); /* consumed reset is no longer re-reported */
    }

    /* A USB interrupt that arrives during any marker write cannot replace the
     * in-flight owner. Repeating that owner's ID is harmless at every cut. */
    for (unsigned write_at = 1u; write_at <= 5u; ++write_at) {
        wipe(MAX_WORDS); boot();
        const uint32_t owner_id = 0x22334455u + write_at;
        const uint32_t conflict_id = 0x66778899u + write_at;
        assert(fw_params_request_reset(owner_id));
        inject_reset_on_write = write_at;
        inject_reset_duplicate_id = owner_id;
        inject_reset_conflicting_id = conflict_id;
        run(1u);
        assert(inject_reset_duplicate_result && !inject_reset_conflicting_result);
        assert(fw_param_reset_is_pending() &&
               fw_param_reset_last_request_id() == owner_id);
        fw_params_request_report(FW_PARAM_ID_ALL); run(400u);
        assert(last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == owner_id);
    }

    /* A failed, unarmed request blocks writes in this boot but must report
     * ERROR with the previous owner ID; RESET_PENDING is reserved for an
     * actually armed marker. A reboot with no marker restores normal commits. */
    {
        wipe(MAX_WORDS); boot();
        const uint32_t failed_id = 0x33445566u;
        assert(fw_params_request_reset(failed_id));
        fail_backup_write_at = 2u; backup_writes = 0u;
        run(20u);
        assert(!fw_param_reset_is_pending());
        assert(sent_store == 1u && last_store.last_commit == PARAM_COMMIT_ERROR);
        const unsigned reports_after_failed_arm = sent_store;
        run(1500u);
        assert(sent_store == reports_after_failed_arm &&
               last_store.last_commit == PARAM_COMMIT_ERROR);
        fw_params_request_commit(); run(20u);
        fw_params_request_report(FW_PARAM_ID_ALL); run(400u);
        assert(last_store.last_commit == PARAM_COMMIT_ERROR &&
               last_store.reset_request_id == 0u);
        assert(!fw_params_request_reset(0x77889900u));
        fw_params_report_reset_rejected();
        fw_params_request_report(FW_PARAM_ID_ALL); run(400u);
        assert(last_store.last_commit == PARAM_COMMIT_ERROR &&
               last_store.reset_request_id == 0u);
        fail_backup_write_at = 0u;
        boot();
        assert(!fw_param_reset_is_pending() && erases == 0u);
        assert(commit() == PARAM_COMMIT_WRITTEN);
    }

    /* An accepted operator request arms RTC only from the main-loop service.
     * It never erases at runtime or allows a normal commit to persist after
     * the marker is armed. The next boot erases/verifies/clears, and a fresh
     * value can then be committed without a later duplicate erase. */
    {
        wipe(rec); boot();
        assert(commit() == PARAM_COMMIT_WRITTEN);
        fw_params_set(FW_PARAM_TILT_MS, 100.0f);
        assert(commit() == PARAM_COMMIT_LOG_FULL);
        uint32_t saved_flash[MAX_WORDS];
        memcpy(saved_flash, flash, sizeof(flash));
        programs = 0u;
        const uint32_t request_id = 0x0A0B0C0Du;
        assert(fw_params_request_reset(request_id));
        run(20u);
        assert(fw_param_reset_is_pending() &&
               fw_param_reset_last_request_id() == request_id);
        assert(erases == 0u && programs == 0u);
        assert(memcmp(saved_flash, flash, sizeof(flash)) == 0);
        fw_params_set(FW_PARAM_TILT_MS, 200.0f);
        fw_params_request_commit();
        run(20u);
        fw_params_request_report(FW_PARAM_ID_ALL); run(400u);
        assert(last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == request_id);
        assert(programs == 0u && memcmp(saved_flash, flash, sizeof(flash)) == 0);

        boot();
        assert(erases == 1u && programs == 0u && !fw_param_reset_is_pending());
        assert(fw_param_reset_last_request_id() == request_id);
        assert(fw_params_get(FW_PARAM_TILT_MS) == (float)TILT_EMERGENCY_MILLIS);
        for (size_t i = 0; i < area_words; ++i) assert(flash[i] == FW_PARAM_LOG_ERASED);
        fw_params_set(FW_PARAM_TILT_MS, 555.0f);
        assert(commit() == PARAM_COMMIT_WRITTEN);
        assert(!fw_params_request_reset(request_id));
        fw_params_report_reset_rejected();
        fw_params_request_report(FW_PARAM_ID_ALL); run(400u);
        assert(last_store.last_commit == PARAM_COMMIT_ERROR &&
               last_store.reset_request_id == request_id);
        boot();
        assert(erases == 0u && fw_params_get(FW_PARAM_TILT_MS) == 555.0f);
    }

    /* A failed erase, an erase that leaves residual bits, or a failed marker
     * clear blocks persistence. The marker is retained on erase failure; after
     * a clear failure the consumed ID still protects newly saved values. */
    {
        const uint32_t retry_id = 0x11223344u;
        assert(fw_params_request_reset(retry_id)); run(20u);
        assert(fw_param_reset_is_pending());
        fail_erase = 1u;
        boot();
        assert(fw_param_reset_is_pending() && erases == 0u);
        run(20u);
        assert(sent_store == 1u &&
               last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == retry_id);
        const unsigned reports_after_boot_retry = sent_store;
        run(1000u);
        assert(sent_store == reports_after_boot_retry + 1u &&
               last_store.last_commit == PARAM_COMMIT_RESET_PENDING &&
               last_store.reset_request_id == retry_id);
        fw_params_set(FW_PARAM_TILT_MS, 600.0f);
        assert(commit() == PARAM_COMMIT_RESET_PENDING && programs == 0u);

        fail_erase = 0u; corrupt_after_erase = 1u;
        boot();
        assert(fw_param_reset_is_pending() && erases == 1u);
        assert(flash[0] != FW_PARAM_LOG_ERASED);
        corrupt_after_erase = 0u;
        backup_writes = 0u; fail_backup_write_at = 2u;
        boot();
        assert(!fw_param_reset_is_pending() && erases == 1u);
        fw_params_set(FW_PARAM_TILT_MS, 600.0f);
        assert(commit() == PARAM_COMMIT_RESET_PENDING && programs == 0u);
        assert(fw_param_reset_last_request_id() == retry_id);

        fail_backup_write_at = 0u;
        boot(); /* invalidated marker does not erase again; runtime block is gone */
        assert(erases == 0u && !fw_param_reset_is_pending());
        fw_params_set(FW_PARAM_TILT_MS, 650.0f);
        assert(commit() == PARAM_COMMIT_WRITTEN);
        boot();
        assert(erases == 0u && fw_params_get(FW_PARAM_TILT_MS) == 650.0f);
    }

    assert(spacing_violations == 0);
    puts("PASS: fw_params persistence, torn-write recovery, explicit reset marker, request IDs, safety guard, and flash faults");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    source = re.sub(r'^#include .*$', '', (FW / 'src/fw_params.c').read_text(), flags=re.M)
    with tempfile.TemporaryDirectory(prefix='fw-params-') as directory:
        out = Path(directory)
        for name in ['board.h', 'board_defaults.h', 'drive_tuning_defaults.h', 'fw_param_catalog.h',
                     'fw_param_log.h', 'fw_param_reset.h', 'fw_param_store.h', 'fw_params.h',
                     'mowgli_protocol.h']:
            (out / name).write_text((FW / 'include' / name).read_text())
        reset_source = re.sub(r'^#include .*$', '', (FW / 'src/fw_param_reset.c').read_text(), flags=re.M)
        (out / 'test.c').write_text(SHIM + reset_source + source + TEST)
        for variant in ['BOARD_YARDFORCE500_VARIANT_ORIG', 'BOARD_YARDFORCE500_VARIANT_B']:
            binary = out / 'test'
            subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            f'-D{variant}=1', 'test.c', '-lm', '-o', str(binary)], cwd=out, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
