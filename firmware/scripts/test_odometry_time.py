#!/usr/bin/env python3
"""Run production odometry acquisition and send gating with clock/USB shims.

Compiles the real encoder folding, wheelTicks_handler and USB gate helpers.
The independent oracle checks acquisition time against delivered encoder deltas.
No hardware or USB transport behavior is inferred from this host-side harness.
"""
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import os
from pathlib import Path
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


SHIM = r'''
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <math.h>
#include "mowgli_protocol.h"
#include "fw_param_catalog.h"
#define TICKS_PER_M 277.0f
#define MAX_MPS 0.5f
#define DRIVEMOTOR_MIN_TICKS_PER_M 50.0f
#define DRIVEMOTOR_MAX_TICKS_PER_M 5000.0f
static volatile float g_ticks_per_meter = TICKS_PER_M;
static volatile float g_max_mps = MAX_MPS;
static uint32_t tick;
static bool telemetry_open = true;
static uint32_t queue_free = 1000;
static uint32_t watchdog_stage;
static pkt_odometry_t received;
static unsigned sends;
static uint32_t last_odom_tick;
static uint32_t HAL_GetTick() { return tick; }
static bool CDC_ShouldSendTelemetry() { return telemetry_open; }
static uint32_t CDC_TXQueue_GetWriteAvailable() { return queue_free; }
static void WATCHDOG_SetMainLoopStage(uint32_t stage) { watchdog_stage = stage; }
static float DRIVEMOTOR_GetTicksPerMeter() { return 277.0f; }
static void mowgli_comms_send_odometry(const pkt_odometry_t *pkt) {
    received = *pkt;
    ++sends;
}
'''

TEST = r'''
static void test_glitch_during_usb_gap() {
    DRIVEMOTOR_dirfilter_t left_filter{}, right_filter{};
    uint16_t left_prev = 1000, right_prev = 2000;
    uint32_t left_magnitude = 0, right_magnitude = 0;
    int32_t left = 0, right = 0;
    telemetry_open = true;
    queue_free = 1000;
    wheelTicks_handler(left, right, 0, 0);
    for (unsigned i = 1; i <= 1503; ++i) {
        tick += 20;
        // A corrupt near-65536 counter value arrives while USB is blocked.
        // Discard it and the unusable reset remainder; do not export a huge
        // accumulated jump when USB resumes after 30 seconds.
        const uint16_t left_raw = i == 50 ? 65535 : 1000 + i;
        const uint16_t right_raw = i == 50 ? 65535 : 2000 + 2 * i;
        DRIVEMOTOR_UpdateWheel(&left_filter, 1, 1, left_raw, &left_prev,
                              &left_magnitude, &left);
        DRIVEMOTOR_UpdateWheel(&right_filter, -1, 1, right_raw, &right_prev,
                              &right_magnitude, &right);
        const bool dropped = i >= 2 && i <= 1500;
        queue_free = dropped ? 0 : 1000;
        const unsigned before = sends;
        wheelTicks_handler(left, right, 0, 0);
        if (dropped) {
            assert(sends == before);
        } else {
            const int32_t accepted_samples = i > 50 ? i - 2 : i;
            assert(sends == before + 1);
            assert(received.left_ticks == accepted_samples);
            assert(received.right_ticks == -2 * accepted_samples);
        }
    }
}

static void test_driver_edges() {
    DRIVEMOTOR_dirfilter_t filter{};
    uint16_t prev = 65534;
    uint32_t magnitude = 0;
    int32_t total = 0;
    assert(DRIVEMOTOR_UpdateWheel(&filter, 1, 1, 1, &prev, &magnitude, &total) == 1);
    assert(total == 3 && magnitude == 3); // natural 16-bit wrap, not a reset

    prev = 100;
    DRIVEMOTOR_UpdateWheel(&filter, 1, 1, 2, &prev, &magnitude, &total);
    assert(total == 5); // ordinary controller reset retains its remainder
    for (const int direction : {-1, 1}) {
        filter = {};
        prev = 1;
        magnitude = 0;
        total = 0;
        DRIVEMOTOR_UpdateWheel(&filter, direction, 1, 65535, &prev, &magnitude, &total);
        assert(total == 0 && magnitude == 0); // huge positive raw glitch
        DRIVEMOTOR_UpdateWheel(&filter, direction, 1, 2, &prev, &magnitude, &total);
        assert(std::abs(total) <= static_cast<int32_t>(drivemotor_max_ticks_per_frame()));
        const int32_t recovered = total;
        DRIVEMOTOR_UpdateWheel(&filter, direction, 1, 3, &prev, &magnitude, &total);
        assert(total == recovered + direction); // normal motion resumes
    }

    filter = {1, 0};
    prev = 100;
    magnitude = 0;
    total = 0;
    assert(DRIVEMOTOR_UpdateWheel(&filter, -1, 0, 101, &prev, &magnitude, &total) == 1);
    assert(DRIVEMOTOR_UpdateWheel(&filter, -1, 0, 1, &prev, &magnitude, &total) == 1);
    assert(DRIVEMOTOR_UpdateWheel(&filter, -1, 0, 0, &prev, &magnitude, &total) == -1);
    assert(DRIVEMOTOR_UpdateWheel(&filter, -1, 1, 2, &prev, &magnitude, &total) == -1);
    assert(total == 0); // coast +2, then confirmed reverse -2
}

int main(int argc, char **argv) {
    test_driver_edges();
    assert(argc == 4);
    const unsigned drops = static_cast<unsigned>(std::stoul(argv[1]));
    const bool closed = std::stoul(argv[2]) != 0;
    tick = static_cast<uint32_t>(std::stoul(argv[3]));
    last_odom_tick = tick;
    int32_t left = 0, right = 0;
    DRIVEMOTOR_dirfilter_t left_filter{}, right_filter{};
    uint16_t left_raw = 65534, right_raw = 65533;
    uint16_t left_prev = left_raw, right_prev = right_raw;
    uint32_t left_magnitude = 0, right_magnitude = 0;
    uint32_t last_time = tick;
    int32_t last_left = 0, last_right = 0;
    wheelTicks_handler(left, right, 0, 0); // establish receiver baseline
    assert(sends == 1);
    double distance = 0;
    for (unsigned i = 1; i <= drops + 6; ++i) {
        tick += 20;
        left_raw += 1;
        right_raw += 2;
        DRIVEMOTOR_UpdateWheel(&left_filter, 1, 1, left_raw, &left_prev,
                              &left_magnitude, &left);
        DRIVEMOTOR_UpdateWheel(&right_filter, -1, 1, right_raw, &right_prev,
                              &right_magnitude, &right);
        assert(left == static_cast<int32_t>(i));
        assert(right == -2 * static_cast<int32_t>(i));
        const bool drop = i >= 2 && i < drops + 2;
        telemetry_open = !(closed && drop);
        queue_free = (!closed && drop) ? 0 : 1000;
        const unsigned before = sends;
        wheelTicks_handler(left, right, 0, 0);
        if (drop) {
            assert(sends == before);
            continue;
        }
        assert(sends == before + 1);
        assert(received.left_ticks == left && received.right_ticks == right);
        const uint32_t elapsed = tick - last_time;
        assert(received.acquisition_millis == tick);
        const uint32_t delivered_elapsed = received.acquisition_millis - last_time;
        assert(delivered_elapsed == elapsed);
        assert(received.left_velocity_mm_s == 180);
        assert(received.right_velocity_mm_s == -361);
        const double velocity = (left - last_left) * 1000.0 / (277.0 * elapsed);
        assert(std::abs(velocity - 1000.0 / (277.0 * 20)) < 1e-12);
        assert(std::abs((right - last_right) * 1000.0 / (277.0 * elapsed)
                        + 2000.0 / (277.0 * 20)) < 1e-12);
        distance += velocity * elapsed / 1000.0;
        last_time = tick;
        last_left = left;
        last_right = right;
    }
    assert(std::abs(distance - left / 277.0) < 1e-10);
    if (drops && !closed) assert(watchdog_stage == WATCHDOG_STAGE_CDC_TX_QUEUE_FULL);
    test_glitch_during_usb_gap();
    std::puts("PASS: production odometry acquisition and gate-drop intervals");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx', default=os.environ.get('CXX', 'c++'))
    args = parser.parse_args()
    source = (FW / 'src/ros/ros_custom/cpp_main.cpp').read_text()
    driver = (FW / 'src/drivemotor.c').read_text()
    type_end = driver.index('} DRIVEMOTOR_dirfilter_t;') + len('} DRIVEMOTOR_dirfilter_t;')
    driver_type = driver[driver.rindex('typedef struct {', 0, type_end):type_end]
    driver_signatures = ['static float drivemotor_clamp_ticks_per_meter(',
                         'static float drivemotor_clamp_max_mps(',
                         'static uint32_t drivemotor_max_ticks_per_frame(',
                         'static int8_t resolve_direction(',
                         'static int8_t DRIVEMOTOR_UpdateWheel(']
    signatures = ['static inline uint32_t usb_cdc_framed_packet_size(',
                  'static inline bool usb_cdc_can_queue_bytes(',
                  'static inline bool usb_cdc_should_send_telemetry_packet(',
                  'extern "C" void\nwheelTicks_handler(']
    production = driver_type + '\n' + '\n'.join(function(driver, sig) for sig in driver_signatures)
    production += '\n' + '\n'.join(function(source, sig) for sig in signatures)
    with tempfile.TemporaryDirectory(prefix='odometry-time-') as directory:
        out = Path(directory)
        test = out / 'test.cpp'
        test.write_text(SHIM + '\n#include <string>\n' + production + TEST)
        binary = out / 'test'
        subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-I', str(FW / 'include'),
                        str(test), '-o', str(binary)], check=True)
        for drops in [0, 1, 7, 3500, 35002, 70002]:
            for closed in [0, 1]:
                for start in [100, 0xfffffff0]:
                    print(f'Testing drops={drops}, closed={closed}, start={start}', flush=True)
                    subprocess.run([str(binary), str(drops), str(closed), str(start)],
                                   cwd=out, check=True)


if __name__ == '__main__':
    main()
