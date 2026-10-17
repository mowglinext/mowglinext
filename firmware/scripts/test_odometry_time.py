#!/usr/bin/env python3
"""Run production odometry acquisition and send gating with clock/USB shims.

Compiles the real wheelTicks_handler and its gate helpers, without MCU setup.
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
#include "mowgli_protocol.h"
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
int main(int argc, char **argv) {
    assert(argc == 4);
    const unsigned drops = static_cast<unsigned>(std::stoul(argv[1]));
    const bool closed = std::stoul(argv[2]) != 0;
    tick = static_cast<uint32_t>(std::stoul(argv[3]));
    last_odom_tick = tick;
    int32_t left = 0, right = 0;
    uint32_t last_time = tick;
    int32_t last_left = 0, last_right = 0;
    wheelTicks_handler(left, right, 0, 0); // establish receiver baseline
    assert(sends == 1);
    double distance = 0;
    for (unsigned i = 1; i <= drops + 6; ++i) {
        tick += 20;
        left += 1;
        right -= 2;
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
    std::puts("PASS: production odometry acquisition and gate-drop intervals");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx', default=os.environ.get('CXX', 'c++'))
    args = parser.parse_args()
    source = (FW / 'src/ros/ros_custom/cpp_main.cpp').read_text()
    signatures = ['static inline uint32_t usb_cdc_framed_packet_size(',
                  'static inline bool usb_cdc_can_queue_bytes(',
                  'static inline bool usb_cdc_should_send_telemetry_packet(',
                  'extern "C" void\nwheelTicks_handler(']
    production = '\n'.join(function(source, sig) for sig in signatures)
    with tempfile.TemporaryDirectory(prefix='odometry-time-') as directory:
        out = Path(directory)
        test = out / 'test.cpp'
        test.write_text(SHIM + '\n#include <string>\n' + production + TEST)
        binary = out / 'test'
        subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-I', str(FW / 'include'),
                        str(test), '-o', str(binary)], check=True)
        for drops in [0, 1, 7, 3500]:
            for closed in [0, 1]:
                for start in [100, 0xfffffff0]:
                    print(f'Testing drops={drops}, closed={closed}, start={start}', flush=True)
                    subprocess.run([str(binary), str(drops), str(closed), str(start)],
                                   cwd=out, check=True)


if __name__ == '__main__':
    main()
