#!/usr/bin/env python3
"""Exercise the production fw_params.c (protocol v7 runtime parameters) against
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
#include "fw_param_store.h"
#include "mowgli_protocol.h"
#include "fw_params.h"
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static void debug_printf(const char *fmt, ...) { (void)fmt; }

/* ---- simulated flash ---- */
#define MAX_WORDS 4096u
static uint32_t flash[MAX_WORDS];
static size_t area_words = MAX_WORDS;
static unsigned erases, programs, fail_erase, fail_program_at;
const uint32_t *fw_param_store_area(void) { return flash; }
size_t fw_param_store_area_words(void) { return area_words; }
int fw_param_store_erase(void) {
    if (fail_erase) return -1;
    ++erases;
    for (size_t i = 0; i < area_words; ++i) flash[i] = 0xFFFFFFFFu;
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
    fw_params_init();
}
static void wipe(size_t words) {
    area_words = words;
    for (size_t i = 0; i < MAX_WORDS; ++i) flash[i] = 0xFFFFFFFFu;
    fail_erase = fail_program_at = 0;
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

    /* 6. Full log: commits report LOG_FULL; the next boot erases once and
     *    rewrites the loaded values, so nothing is lost. */
    const size_t rec = fw_param_log_record_words(FW_PARAM_COUNT - 1u);
    wipe(rec * 3u); boot();
    for (int i = 0; i < 3; ++i) {
        fw_params_set(FW_PARAM_TILT_MS, 100.0f + (float)i);
        assert(commit() == PARAM_COMMIT_WRITTEN);
    }
    fw_params_set(FW_PARAM_TILT_MS, 200.0f);
    assert(commit() == PARAM_COMMIT_LOG_FULL);
    assert(last_store.records_left == 0);
    boot();
    assert(erases == 1 && fw_params_get(FW_PARAM_TILT_MS) == 102.0f);
    assert(stored_record_value(FW_PARAM_TILT_MS) == 102.0f);
    fw_params_request_report(FW_PARAM_ID_ALL); run(400);
    assert(last_store.boot_source == PARAM_BOOT_FLASH_ERASED && last_store.records_left == 2);

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
    assert(commit() == PARAM_COMMIT_LOG_FULL);  /* no appends until a boot erase */

    assert(spacing_violations == 0);
    puts("PASS: fw_params boot load, envelope, reports, commit dedupe, torn write, full log, foreign data, flash faults");
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
                     'fw_param_log.h', 'fw_param_store.h', 'fw_params.h', 'mowgli_protocol.h']:
            (out / name).write_text((FW / 'include' / name).read_text())
        (out / 'test.c').write_text(SHIM + source + TEST)
        for variant in ['BOARD_YARDFORCE500_VARIANT_ORIG', 'BOARD_YARDFORCE500_VARIANT_B']:
            binary = out / 'test'
            subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            f'-D{variant}=1', 'test.c', '-lm', '-o', str(binary)], cwd=out, check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
