#!/usr/bin/env python3
"""Execute production soft_i2c.c against a clock/data-line fault model.

Checks electrical sequencing, bounded failures, retry throttling, tick wrap,
and the public IMU read/write entry points. No hardware/debugger is needed.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'
SHIM = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <assert.h>
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_SPEED_HIGH 1
#define GPIO_MODE_OUTPUT_OD 1
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP 1
#define SOFT_I2C_GPIO_CLK_ENABLE() ((void)0)
#define SOFT_I2C_SCL_PORT 0
#define SOFT_I2C_SDA_PORT 0
#define SOFT_I2C_SCL_PIN 0
#define SOFT_I2C_SDA_PIN 1
typedef struct { unsigned Pin, Speed, Mode, Pull; } GPIO_InitTypeDef;
static unsigned odr[2], mode[2], pulses, stops, release_at, scl_reads;
static unsigned sda_stuck, scl_stuck, scl_fail_at, slave_ack, stretch_reads;
static uint32_t tick;
static uint32_t HAL_GetTick(void) { return tick; }
static void HAL_GPIO_Init(int port, GPIO_InitTypeDef *p) {
    (void)port; assert(p->Mode == GPIO_MODE_OUTPUT_OD || p->Mode == GPIO_MODE_INPUT);
    mode[p->Pin] = p->Mode;
}
static void HAL_GPIO_DeInit(int port, unsigned pin) { (void)port; (void)pin; }
static void HAL_GPIO_WritePin(int port, unsigned pin, unsigned high) {
    (void)port;
    if (pin == 0 && !odr[0] && high) {
        ++pulses;
        if (release_at && pulses >= release_at) sda_stuck = 0;
        if (scl_fail_at && pulses >= scl_fail_at) scl_stuck = 1;
    }
    if (pin == 1 && !odr[1] && high && odr[0] && !scl_stuck) ++stops;
    odr[pin] = high;
}
static uint8_t HAL_GPIO_ReadPin(int port, unsigned pin) {
    (void)port;
    if (pin == 0) {
        assert(++scl_reads < 1000); // catches accidentally unbounded waits
        if (stretch_reads) { --stretch_reads; return 0; }
        return (uint8_t)(odr[0] && !scl_stuck);
    }
    if (sda_stuck || (slave_ack && mode[1] == GPIO_MODE_INPUT)) return 0;
    return (uint8_t)(mode[1] == GPIO_MODE_INPUT || odr[1]);
}
'''
TEST = r'''
static void reset_model(void) {
    odr[0] = odr[1] = 1; mode[0] = mode[1] = GPIO_MODE_OUTPUT_OD;
    pulses = stops = release_at = scl_reads = 0;
    sda_stuck = scl_stuck = scl_fail_at = slave_ack = stretch_reads = 0;
    tick = 0; i2c_recovery_attempted = 0; i2c_recovery_last_tick = 0;
    sw_i2c_recovery_diag.attempts = sw_i2c_recovery_diag.successes =
        sw_i2c_recovery_diag.failures = 0;
}
int main(void) {
    reset_model();
    assert(i2c_prepare_transaction());
    assert(pulses == 0 && stops == 0 && sw_i2c_recovery_diag.attempts == 0);
    for (unsigned n = 1; n <= 9; ++n) {
        reset_model(); sda_stuck = 1; release_at = n;
        assert(i2c_prepare_transaction());
        assert(pulses == n + 1 && stops == 1); // n bus-clear clocks + STOP setup
        assert(odr[0] && odr[1] && sw_i2c_recovery_diag.successes == 1);
    }
    reset_model(); sda_stuck = 1;
    assert(!i2c_prepare_transaction());
    assert(pulses == 9 && odr[0] && odr[1]);
    assert(sw_i2c_recovery_diag.failures == 1);
    tick = 999; assert(!i2c_prepare_transaction()); assert(pulses == 9);
    tick = 1000; release_at = 10;
    assert(i2c_prepare_transaction()); assert(sw_i2c_recovery_diag.attempts == 2);
    // A naturally freed bus must be usable even during the recovery cooldown.
    reset_model(); sda_stuck = 1; assert(!i2c_prepare_transaction());
    sda_stuck = 0; tick = 1; assert(i2c_prepare_transaction()); assert(pulses == 9);
    reset_model(); scl_stuck = 1;
    assert(!i2c_prepare_transaction()); assert(pulses == 0 && odr[0] && odr[1]);
    assert(scl_reads <= I2C_RECOVERY_SCL_POLLS + 1);
    // SCL can become stuck in the middle of recovery or during STOP setup.
    reset_model(); sda_stuck = 1; scl_fail_at = 3;
    assert(!i2c_prepare_transaction()); assert(pulses == 3 && odr[0] && odr[1]);
    reset_model(); sda_stuck = 1; release_at = 2; scl_fail_at = 3;
    assert(!i2c_prepare_transaction()); assert(odr[0] && odr[1]);
    reset_model(); sda_stuck = 1; release_at = 2; stretch_reads = 4;
    assert(i2c_prepare_transaction()); assert(stops == 1);
    reset_model(); sda_stuck = 1; tick = UINT32_MAX - 499;
    assert(!i2c_prepare_transaction()); tick = 499;
    assert(!i2c_prepare_transaction()); assert(sw_i2c_recovery_diag.attempts == 1);
    tick = 500; release_at = 10; assert(i2c_prepare_transaction());
    assert(sw_i2c_recovery_diag.attempts == 2);
    // Public reads must fail without changing the output on a stuck bus.
    uint8_t data[6] = {42,42,42,42,42,42};
    reset_model(); sda_stuck = 1;
    assert(!SW_I2C_UTIL_Read_Multi(0x68, 0x3b, 6, data));
    for (unsigned n = 0; n < 6; ++n) assert(data[n] == 42);
    assert(!SW_I2C_UTIL_WRITE(0x68, 0x6b, 9));
    assert(SW_I2C_UTIL_Read(0x68, 0x75) == 0x80);
    assert(sw_i2c_recovery_diag.attempts == 1);
    reset_model(); sda_stuck = 1;
    assert(!SW_I2C_UTIL_Read_Multi(0x68, 0x3b, 0, data));
    assert(!SW_I2C_UTIL_Read_Multi(0x68, 0x3b, 6, NULL));
    assert(sw_i2c_recovery_diag.attempts == 0);
    // Retry the actual transaction after freeing SDA; successful bus clear
    // alone must not turn an absent slave's NACK into a successful read.
    reset_model(); sda_stuck = 1; release_at = 3;
    assert(!SW_I2C_UTIL_Read_Multi(0x68, 0x3b, 6, data));
    assert(sw_i2c_recovery_diag.successes == 1);
    reset_model(); sda_stuck = 1; release_at = 3; slave_ack = 1;
    assert(SW_I2C_UTIL_Read_Multi(0x68, 0x3b, 6, data));
    assert(sw_i2c_recovery_diag.successes == 1);
    for (unsigned n = 0; n < 6; ++n) assert(data[n] == 0);
    reset_model(); sda_stuck = 1; release_at = 9; slave_ack = 1;
    assert(SW_I2C_UTIL_WRITE(0x68, 0x6b, 9));
    assert(sw_i2c_recovery_diag.successes == 1);
    puts("PASS: bus clear, STOP, stuck/stretching SCL/SDA, cooldown/wrap, IMU transactions");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    source = (FW / 'src/soft_i2c.c').read_text(encoding='utf-8')
    source = re.sub(r'^#include .*$', '', source, flags=re.M)
    # This timing-only GCC attribute is not understood by MSVC.
    source = source.replace('__attribute__ ((optimize(0)))', '')
    header = (FW / 'include/soft_i2c.h').read_text(encoding='utf-8')
    with tempfile.TemporaryDirectory(prefix='i2c-recovery-') as directory:
        out = Path(directory)
        (out / 'test.c').write_text(SHIM + header + source + TEST, encoding='utf-8')
        binary = out / ('test.exe' if os.name == 'nt' else 'test')
        if Path(args.cc).stem.lower() == 'cl':
            cmd = [args.cc, '/nologo', '/std:c11', '/utf-8', '/W3', 'test.c', '/Fe:' + str(binary)]
        else:
            cmd = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', 'test.c', '-o', str(binary)]
        subprocess.run(cmd, cwd=out, check=True)
        subprocess.run([str(binary)], cwd=out, check=True)


if __name__ == '__main__':
    main()
