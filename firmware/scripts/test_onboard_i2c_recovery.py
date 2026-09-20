#!/usr/bin/env python3
"""Exercise production onboard I2C recovery and emergency latch against HAL faults."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'
SHIM = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "i2c_lis3dh.h"
#include "i2c.h"
#include "emergency.h"
#define debug_printf(...) ((void)0)
#define GPIO_PIN_6 64
#define GPIO_PIN_7 128
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_MODE_AF_OD 2
#define GPIO_MODE_OUTPUT_OD 1
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_HIGH 1
#define GPIO_AF4_I2C1 4
#define GPIOB 1
#define I2C1 1
#define I2C_DUTYCYCLE_2 0
#define I2C_ADDRESSINGMODE_7BIT 0
#define I2C_DUALADDRESS_DISABLE 0
#define I2C_GENERALCALL_DISABLE 0
#define I2C_NOSTRETCH_DISABLE 0
#define I2C_MEMADD_SIZE_8BIT 1
#define I2C_FLAG_BUSY 1
#define RESET 0
#define HAL_OK 0
#ifndef MS2_PER_G
#define MS2_PER_G 9.80665f
#endif
typedef struct { unsigned Pin, Mode, Speed, Pull, Alternate; } GPIO_InitTypeDef;
typedef struct {
    int Instance;
    struct { unsigned ClockSpeed, DutyCycle, AddressingMode, DualAddressMode,
                      GeneralCallMode, NoStretchMode; } Init;
} I2C_HandleTypeDef;
static uint32_t tick, ipsr, primask;
static unsigned mode, outputs, pulses, stops, stuck_sda, stuck_scl, release_after;
static unsigned busy, fail_io, bad_write, io_calls, hal_init_fail;
static uint8_t registers[128];
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_IPSR(void) { return ipsr; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1; }
static void __set_PRIMASK(uint32_t p) { primask = p; }
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_FORCE_RESET() (busy = 0)
#define __HAL_RCC_I2C1_RELEASE_RESET() ((void)0)
#define __HAL_I2C_GET_FLAG(h, f) ((void)(h), (void)(f), busy)
static void HAL_GPIO_Init(int port, GPIO_InitTypeDef *g) {
    assert(port == GPIOB && g->Pin == (GPIO_PIN_6 | GPIO_PIN_7));
    assert(g->Mode == GPIO_MODE_AF_OD || g->Mode == GPIO_MODE_OUTPUT_OD);
    mode = g->Mode;
}
static unsigned HAL_GPIO_ReadPin(int port, unsigned pin) {
    assert(port == GPIOB);
    if (pin == GPIO_PIN_6 && stuck_scl) return 0;
    if (pin == GPIO_PIN_7 && stuck_sda) return 0;
    return mode == GPIO_MODE_AF_OD || (outputs & pin) != 0;
}
static void HAL_GPIO_WritePin(int port, unsigned pins, unsigned high) {
    assert(port == GPIOB);
    unsigned old = outputs;
    if (high) outputs |= pins; else outputs &= ~pins;
    if (mode != GPIO_MODE_OUTPUT_OD) return;
    if (high && (pins & GPIO_PIN_6) && !(old & GPIO_PIN_6) && (outputs & GPIO_PIN_7)) {
        ++pulses;
        if (release_after && pulses == release_after) stuck_sda = 0;
    }
    if (high && (pins & GPIO_PIN_7) && !(old & GPIO_PIN_7) && (outputs & GPIO_PIN_6)) ++stops;
}
static int HAL_I2C_Init(I2C_HandleTypeDef *h) {
    assert(h->Init.ClockSpeed == 400000); return hal_init_fail;
}
static int transfer(void *h, unsigned address, unsigned reg, unsigned size,
                    uint8_t *p, unsigned n, unsigned timeout, unsigned write) {
    (void)h;
    assert(!ipsr && !busy && mode == GPIO_MODE_AF_OD);
    assert(address == LIS3DH_I2C_ADD_L && size == 1 && timeout == 2);
    ++io_calls;
    if (fail_io) return 1;
    for (unsigned i=0; i<n; ++i) {
        unsigned index = (reg & 127) + i;
        if (write) registers[index] = p[i] ^ (bad_write ? 1 : 0);
        else p[i] = registers[index];
    }
    return 0;
}
#define HAL_I2C_Mem_Read(h,a,r,s,p,n,t) transfer(h,a,r,s,p,n,t,0)
#define HAL_I2C_Mem_Write(h,a,r,s,p,n,t) transfer(h,a,r,s,p,n,t,1)
float lis3dh_from_fs2_hr_to_mg(int16_t n) { return n; }
float lis3dh_from_lsb_hr_to_celsius(int16_t n) { return n; }
static volatile uint8_t emergency_state;
'''

TEST = r'''
static void step(void) {
    unsigned before = io_calls;
    I2C_Onboard_Service();
    assert(io_calls - before <= 1); // no burst of setup/verification I/O
    ++tick;
}
static void advance(unsigned n) { while(n--) step(); }
static void reset(void) {
    memset(registers, 0, sizeof(registers)); registers[LIS3DH_WHO_AM_I] = LIS3DH_ID;
    tick=ipsr=primask=0; mode=GPIO_MODE_AF_OD; outputs=GPIO_PIN_6|GPIO_PIN_7;
    pulses=stops=stuck_sda=stuck_scl=release_after=busy=fail_io=bad_write=io_calls=hal_init_fail=0;
    emergency_state=0; memset((void *)&onboard_i2c_diag,0,sizeof(onboard_i2c_diag));
    I2C_Init();
}
static void ready(void) {
    reset(); assert(!I2C_OnboardHealthy() && Emergency_State());
    advance(80); assert(I2C_OnboardHealthy() && !I2C_TestZLowINT());
    assert(!Emergency_State());
    assert(registers[LIS3DH_CTRL_REG3] == 0x40);
    assert(registers[LIS3DH_INT1_THS] == IMU_ONBOARD_INCLINATION_THRESHOLD);
}
int main(void) {
    ready();
    registers[LIS3DH_INT1_SRC]=0x50; advance(12); assert(I2C_TestZLowINT());
    Emergency_SetState(1); Emergency_SetState(0); assert(Emergency_State());
    registers[LIS3DH_INT1_SRC]=0; advance(12); Emergency_SetState(0); assert(!Emergency_State());

    // IRQ consumers never transact or advance recovery, and preserve PRIMASK.
    unsigned calls=io_calls; ipsr=1; primask=1;
    uint8_t value=0; I2C_Onboard_Service(); (void)I2C_TestZLowINT();
    assert(I2C_platform_read(&I2C_Handle,15,&value,1)==-1);
    assert(io_calls==calls && primask==1); ipsr=primask=0;

    // Every possible slave release pulse, STOP, verified configuration and
    // successful fresh INT1_SRC required; recovery never clears the latch.
    for (unsigned release=1; release<=9; ++release) {
        ready(); stuck_sda=1; release_after=release; calls=io_calls;
        advance(12); assert(!I2C_OnboardHealthy() && io_calls==calls);
        assert(Emergency_State() & 0x40); Emergency_SetState(0); assert(Emergency_State());
        advance(1200); assert(I2C_OnboardHealthy()); assert(pulses==release && stops==1);
        assert(onboard_i2c_diag.recoveries==1 && Emergency_State()==0x40);
        Emergency_SetState(1); assert(Emergency_State()==0x41); // retain fault provenance
        Emergency_SetState(0); assert(!Emergency_State());
    }
    ready(); stuck_sda=1; advance(1100);
    assert(!I2C_OnboardHealthy() && pulses==9);
    calls=onboard_i2c_diag.attempts; advance(500); assert(onboard_i2c_diag.attempts==calls);
    assert(outputs==(GPIO_PIN_6|GPIO_PIN_7));
    ready(); stuck_scl=1; advance(1100); assert(!I2C_OnboardHealthy() && !pulses);
    assert(outputs==(GPIO_PIN_6|GPIO_PIN_7));

    // Busy peripheral without held pins, NACK/timeouts, identity and bad writes.
    ready(); busy=1; calls=io_calls; advance(12); assert(io_calls==calls && !I2C_OnboardHealthy());
    advance(1200); assert(I2C_OnboardHealthy());
    ready(); fail_io=1; advance(12); assert(!I2C_OnboardHealthy()); fail_io=0;
    advance(1200); assert(I2C_OnboardHealthy() && Emergency_State());
    reset(); registers[LIS3DH_WHO_AM_I]=0; advance(1200); assert(!I2C_OnboardHealthy());
    reset(); bad_write=1; advance(80); assert(!I2C_OnboardHealthy());
    reset(); hal_init_fail=1; I2C_Init(); advance(1200); assert(!I2C_OnboardHealthy());

    // A sensor reset cannot masquerade indefinitely as healthy zero tilt.
    ready(); registers[LIS3DH_CTRL_REG3]=0; advance(1200);
    assert(!I2C_OnboardHealthy() && Emergency_State());
    ready(); registers[LIS3DH_CTRL_REG3]=0;
    for (unsigned i=0; i<115; ++i) { tick+=10; step(); }
    assert(!I2C_OnboardHealthy()); // audit cannot starve behind always-due polling

    // Age does not advance on cached reads; stalled service inhibits release.
    ready(); tick+=101; calls=io_calls; assert(I2C_TestZLowINT());
    assert(!I2C_OnboardHealthy() && io_calls==calls); Emergency_SetState(0); assert(Emergency_State());
    step(); advance(1200); assert(I2C_OnboardHealthy() && Emergency_State());
    ready(); tick=UINT32_MAX-50; sensor_sample_tick=poll_tick=tick; stuck_sda=1;
    advance(20); stuck_sda=0; advance(1200); assert(I2C_OnboardHealthy() && Emergency_State());
    puts("PASS: onboard bus recovery, register verification, bounded service, IRQ isolation, stale/invalid data and emergency latch");
}
'''

HEARTBEAT_SHIM = r'''
#include "heartbeat_emergency_policy.hpp"
static bool heartbeat_only_latch;
static uint32_t last_heartbeat_tick;
typedef struct { uint8_t type, emergency_requested, emergency_release_requested; uint16_t crc; } pkt_heartbeat_t;
static bool any_physical_emergency(void) { return I2C_TestZLowINT(); }
'''
HEARTBEAT_TEST = r'''
static void beat(bool stop, bool release) {
    pkt_heartbeat_t pkt = {}; pkt.emergency_requested=stop; pkt.emergency_release_requested=release;
    ipsr=1; on_heartbeat(reinterpret_cast<const uint8_t *>(&pkt),sizeof(pkt)-2); ipsr=0;
}
int main() {
    registers[LIS3DH_WHO_AM_I]=LIS3DH_ID;
    outputs=GPIO_PIN_6|GPIO_PIN_7; I2C_Init();
    for(unsigned i=0;i<80;++i) { I2C_Onboard_Service(); ++tick; }
    assert(I2C_OnboardHealthy());
    Emergency_SetState(1); heartbeat_only_latch=true;
    beat(false,false); assert(!Emergency_State()); // genuine comms-only recovery
    Emergency_SetState(1); heartbeat_only_latch=true;
    Emergency_OnboardSensorFault(); // sensor recovered, but its fault remains latched
    unsigned before=io_calls;
    beat(false,false); assert(Emergency_State()==0x41 && io_calls==before);
    beat(true,false); assert(Emergency_State()==0x41);
    beat(false,true); assert(!Emergency_State()); // explicit release after health verification
    sensor_valid=0; Emergency_OnboardSensorFault();
    beat(false,true); assert(Emergency_State() & 0x40);
    assert(io_calls==before); // the real USB callback never performs an I2C transaction
    puts("PASS: real heartbeat callback preserves sensor faults across comms recovery and gates release");
}
'''


def extract(source, signature):
    start = source.index(signature)
    pos = source.index('{', start)
    depth, end = 1, pos + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    source = re.sub(r'^#include .*$', '', (FW / 'src/i2c.c').read_text(), flags=re.M)
    emergency = (FW / 'src/emergency.c').read_text()
    functions = ''.join(extract(emergency, signature) for signature in [
        'uint8_t Emergency_State(void)', 'void  Emergency_SetState(uint8_t',
        'static void emergency_set_bits(uint8_t', 'void Emergency_OnboardSensorFault(void)'])
    with tempfile.TemporaryDirectory(prefix='onboard-i2c-') as directory:
        out = Path(directory)
        for name in ['board.h', 'board_defaults.h', 'i2c_lis3dh.h', 'i2c.h', 'emergency.h']:
            (out / name).write_text((FW / 'include' / name).read_text())
        (out / 'test.c').write_text(SHIM + functions + source + TEST)
        for variant in ['BOARD_YARDFORCE500_VARIANT_ORIG', 'BOARD_YARDFORCE500_VARIANT_B']:
            binary = out / 'test'
            subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            f'-D{variant}=1', 'test.c', '-lm', '-o', str(binary)], cwd=out, check=True)
            print(variant, flush=True)
            subprocess.run([str(binary)], check=True)
        (out / 'heartbeat_emergency_policy.hpp').write_text(
            (FW / 'include/heartbeat_emergency_policy.hpp').read_text())
        heartbeat = extract((FW / 'src/ros/ros_custom/cpp_main.cpp').read_text(),
                            'static void on_heartbeat(const uint8_t *data, size_t len)')
        (out / 'heartbeat.cpp').write_text(SHIM + functions + source + HEARTBEAT_SHIM + heartbeat + HEARTBEAT_TEST)
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++14', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers',
                        '-DBOARD_YARDFORCE500_VARIANT_B=1', 'heartbeat.cpp', '-o', str(binary)], cwd=out, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
