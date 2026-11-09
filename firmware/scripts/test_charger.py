#!/usr/bin/env python3
"""Run production charger state-machine tests with clock, ADC and HAL stubs.

Only TIM1_Init is omitted. ChargeController and both runtime setters are compiled
unchanged against the real board defaults; no physical charging is performed.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'
TESTS = Path(__file__).resolve().parent / 'tests'

SHIM = r'''
#pragma once
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef int TIM_HandleTypeDef;
typedef int RTC_HandleTypeDef;
typedef int ADC_HandleTypeDef;
typedef int GPIO_TypeDef;
static GPIO_TypeDef gpio_c;
#define GPIOC (&gpio_c)
#define GPIO_PIN_5 (1u << 5)
typedef struct { uint32_t CCR1; } TestTimer;
static TestTimer timer;
#define TIM1 (&timer)
#define RTC_BKP_DR1 1u
#define RTC_BKP_DR2 2u
#define RTC_BKP_DR3 3u
#define RTC_BKP_DR4 4u
static uint32_t test_tick;
static unsigned tf4 = 1u, offset_writes;
static uint32_t HAL_GetTick(void) { return test_tick; }
static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, unsigned value) {
    assert(port == GPIOC && pin == GPIO_PIN_5);
    tf4 = value;
}
static void HAL_PWR_EnableBkUpAccess(void) {}
static void HAL_PWR_DisableBkUpAccess(void) {}
static void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *rtc, unsigned reg, uint32_t value) {
    (void)rtc; (void)value;
    if (reg == RTC_BKP_DR3 || reg == RTC_BKP_DR4) ++offset_writes;
}
#include "adc.h"
RTC_HandleTypeDef hrtc;
float battery_voltage, charge_voltage, current, current_without_offset;
float chargerInputVoltage;
union FtoU ampere_acc, charge_current_offset;
'''


def controller_source():
    source = (FW / 'src/charger.c').read_text(encoding='utf-8')
    start = source.index('void TIM1_Init(void)')
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[:start] + source[end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    tests = sorted(TESTS.glob('test_charger_*.c'))
    if not tests:
        parser.error('no charger tests found')
    with tempfile.TemporaryDirectory(prefix='charger-') as directory:
        out = Path(directory)
        for name in ['board.h', 'board_defaults.h', 'adc.h', 'charger.h', 'fw_param_catalog.h']:
            (out / name).write_text((FW / 'include' / name).read_text(encoding='utf-8'),
                                   encoding='utf-8')
        for name in ['stm32f1xx_hal.h', 'stm32f4xx_hal.h']:
            (out / name).write_text('', encoding='utf-8')
        (out / 'main.h').write_text(SHIM, encoding='utf-8')
        (out / 'charger_under_test.c').write_text(controller_source(), encoding='utf-8')
        binary = out / ('test.exe' if os.name == 'nt' else 'test')
        for test in tests:
            (out / 'test.c').write_text(test.read_text(encoding='utf-8'), encoding='utf-8')
            for name, defines in [
                ('Yardforce500', ['BOARD_YARDFORCE500_VARIANT_ORIG=1']),
                ('Yardforce500B', ['BOARD_YARDFORCE500_VARIANT_B=1']),
                ('BiltemaRM1000', ['BOARD_YARDFORCE500_VARIANT_B=1', 'BOARD_BILTEMA_RM1000=1']),
            ]:
                print(f'Testing {test.name}: {name}', flush=True)
                subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror',
                                *[f'-D{value}' for value in defines], 'test.c', '-o', str(binary)],
                               cwd=out, check=True)
                subprocess.run([str(binary)], cwd=out, check=True)


if __name__ == '__main__':
    main()
