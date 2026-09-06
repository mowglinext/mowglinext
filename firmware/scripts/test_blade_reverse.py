#!/usr/bin/env python3
"""Execute the production blade command/RX path with UART and clock stubs.

Tests frame bytes and reversal interlocks; cannot establish physical rotation.
Only GPIO/UART initialization is removed. The command builder, App, Set and RX
callback are the actual firmware implementations, including failed transmissions.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

FW = Path(__file__).resolve().parents[1] / 'stm32/ros_usbnode'

SHIM = r'''
#include <stdint.h>
#include <assert.h>
#include <string.h>
typedef struct { unsigned gState; } UART_HandleTypeDef;
typedef int DMA_HandleTypeDef;
#define HAL_UART_STATE_READY 0u
#define HAL_OK 0
#define HAL_BUSY 1
static uint32_t test_tick, test_primask;
static int test_tx_result;
static unsigned test_tx_count;
static uint8_t test_frame[22];
static uint32_t HAL_GetTick(void) { return test_tick; }
static uint32_t __get_PRIMASK(void) { return test_primask; }
static void __disable_irq(void) { test_primask = 1; }
static void __set_PRIMASK(uint32_t value) { test_primask = value; }
static int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, unsigned n) {
    (void)h;
    if (!test_tx_result) { memcpy(test_frame, p, n); ++test_tx_count; }
    return test_tx_result;
}
static int HAL_UART_Receive_DMA(UART_HandleTypeDef *h, uint8_t *p, unsigned n) {
    (void)h; (void)p; (void)n; return HAL_OK;
}
#define debug_printf(...) ((void)0)
'''

TEST = r'''
#include <stdio.h>
#include "blade_under_test.c"

static void reset(void) {
    test_tick = 100;
    test_primask = test_tx_count = test_tx_result = 0;
    BLADEMOTOR_USART_Handler.gState = HAL_UART_STATE_READY;
    blademotor_eState = BLADEMOTOR_RUN;
    blademotor_u8OnOff = blademotor_u8Direction = blademotor_u8RunDirection = 0;
    blademotor_reverse_pending = blademotor_off_sent = blademotor_zero_seen = false;
    blademotor_stop_since = blademotor_zero_since = blademotor_last_feedback_seq = 0;
    blademotor_zero_epoch = 0;
    blademotor_feedback = (blademotor_feedback_t){0};
    BLADEMOTOR_bActivated = false;
    BLADEMOTOR_u16RPM = BLADEMOTOR_u16Power = BLADEMOTOR_u32Error = 0;
    memset(blademotor_pu8ReceivedData, 0, sizeof(blademotor_pu8ReceivedData));
}
static void frame(uint8_t command) {
    assert(test_frame[5] == command);
    assert(test_frame[6] == crcCalc(test_frame, 6));
    assert(test_frame[6] == (command == 0xC0 ? 0x62 : command == 0x80 ? 0x22 : 0xA2));
}
/* valid: 1 good, 0 bad checksum, -1 bad preamble with otherwise valid checksum. */
static void feedback(unsigned advance, unsigned rpm, unsigned active, unsigned error, int valid) {
    test_tick += advance;
    memset(blademotor_pu8ReceivedData, 0, sizeof(blademotor_pu8ReceivedData));
    blademotor_pu8ReceivedData[0] = valid == -1 ? 0 : 0x55;
    blademotor_pu8ReceivedData[1] = 0xAA;
    blademotor_pu8ReceivedData[5] = active ? 0x80 : 0;
    blademotor_pu8ReceivedData[6] = error;
    blademotor_pu8ReceivedData[7] = rpm & 255;
    blademotor_pu8ReceivedData[8] = rpm >> 8;
    blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_RECEIVED_MSG-1] =
        crcCalc(blademotor_pu8ReceivedData, BLADEMOTOR_LENGTH_RECEIVED_MSG-1) ^ (valid == 0);
    BLADEMOTOR_ReceiveIT();
}
static void zero(unsigned advance) { feedback(advance, 0, 0, 0, 1); BLADEMOTOR_App(); }
static void reversing(void) {
    reset(); BLADEMOTOR_Set(1, 0); BLADEMOTOR_App(); frame(0x80);
    feedback(100, 3300, 1, 0, 1);
    BLADEMOTOR_Set(1, 1); BLADEMOTOR_App(); frame(0);
    assert(blademotor_off_sent);
}
int main(void) {
    reset(); BLADEMOTOR_App(); frame(0);
    BLADEMOTOR_Set(1, 0); BLADEMOTOR_App(); frame(0x80);
    BLADEMOTOR_Set(0, 1); BLADEMOTOR_App(); frame(0);
    // A reverse first start cannot use the zero-initialized public RPM.
    reset(); BLADEMOTOR_Set(1, 1); BLADEMOTOR_App(); frame(0);
    test_tick += 5000; BLADEMOTOR_App(); frame(0);

    reversing();
    for (unsigned i=0; i<9; ++i) { zero(100); frame(0); }
    zero(99); frame(0); // minimum OFF dwell not complete
    zero(1); frame(0xC0); // fresh zero confirmation + exact dwell boundary
    assert(blademotor_u8RunDirection == 1);
    BLADEMOTOR_Set(1, 0); BLADEMOTOR_App(); frame(0);
    for (unsigned i=0; i<9; ++i) { zero(100); frame(0); }
    zero(100); frame(0x80); // symmetric reverse-to-forward interlock

    reversing();
    feedback(1000, 0, 0, 0, 1); BLADEMOTOR_App(); frame(0);
    test_tick += 300; BLADEMOTOR_App(); frame(0); // one old zero is insufficient
    zero(1); frame(0); // feedback gap breaks the zero interval
    zero(299); frame(0);
    zero(1); frame(0xC0);

    // Spinning, active, error, bad CRC/preamble and missing feedback all inhibit.
    for (unsigned failure=0; failure<6; ++failure) {
        reversing();
        for (unsigned i=0; i<15; ++i) {
            if (failure == 5) test_tick += 100;
            else feedback(100, failure==0 ? 1:0, failure==1, failure==2,
                          failure==3 ? 0 : failure==4 ? -1 : 1);
            BLADEMOTOR_App(); frame(0);
        }
    }
    // A bad sample cannot disappear when a good reply arrives before App.
    reversing();
    for (unsigned i=0; i<9; ++i) zero(100);
    feedback(50, 0, 0, 0, 0);
    zero(50); frame(0);
    zero(299); frame(0);
    zero(1); frame(0xC0);

    // OFF cancels pending reversal; fresh enable must establish a new stop.
    reversing(); for (unsigned i=0; i<9; ++i) zero(100);
    BLADEMOTOR_Set(0, 1); zero(100); frame(0);
    for (unsigned i=0; i<20; ++i) zero(100);
    frame(0);
    BLADEMOTOR_Set(1, 1); BLADEMOTOR_App(); frame(0);
    for (unsigned i=0; i<9; ++i) { zero(100); frame(0); }
    zero(100); frame(0xC0);

    // Failed OFF transmissions cannot count as a stopped dwell.
    reset(); BLADEMOTOR_Set(1,0); BLADEMOTOR_App(); frame(0x80);
    BLADEMOTOR_Set(1,1); test_tx_result=HAL_BUSY;
    for (unsigned i=0; i<20; ++i) zero(100);
    assert(!blademotor_off_sent); frame(0x80);
    test_tx_result=HAL_OK; BLADEMOTOR_App(); frame(0);
    for (unsigned i=0; i<9; ++i) { zero(100); frame(0); }
    zero(100); frame(0xC0);

    // Set does not mutate DMA data; App does not rebuild while TX owns it.
    reset(); BLADEMOTOR_Set(1,0); BLADEMOTOR_App(); frame(0x80);
    BLADEMOTOR_USART_Handler.gState=1;
    BLADEMOTOR_Set(1,1); BLADEMOTOR_App();
    assert(blademotor_pu8RqstMessage[5]==0x80 && test_tx_count==1);
    BLADEMOTOR_USART_Handler.gState=HAL_UART_STATE_READY;
    BLADEMOTOR_App(); frame(0);

    // Repeated ON updates do not restart the dwell; timing survives tick wrap.
    reversing(); test_tick=UINT32_MAX-500;
    blademotor_stop_since=test_tick;
    blademotor_feedback.tick=test_tick;
    for (unsigned i=0; i<9; ++i) { BLADEMOTOR_Set(1,1); zero(100); frame(0); }
    test_primask=1; zero(100); frame(0xC0); assert(test_primask==1);
    puts("PASS: blade frames/checksums, bidirectional stop guard, stale/invalid feedback, cancellation, TX failure/busy, tick wrap");
}
'''


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return start, end


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    source = (FW / 'src/blademotor.c').read_text(encoding='utf-8')
    start, end = function(source, 'void BLADEMOTOR_Init(void)')
    source = source[:start] + source[end:]
    main_source = (FW / 'src/main.c').read_text(encoding='utf-8')
    start, end = function(main_source, 'uint8_t crcCalc(const uint8_t *msg, uint8_t msg_len)')
    with tempfile.TemporaryDirectory(prefix='blade-reverse-') as directory:
        out = Path(directory)
        (out / 'main.h').write_text('#pragma once\n' + SHIM + main_source[start:end], encoding='utf-8')
        for name in ['stm32f_board_hal.h', 'blademotor.h']:
            (out / name).write_text('', encoding='utf-8')
        (out / 'blade_under_test.c').write_text(source, encoding='utf-8')
        (out / 'test.c').write_text(TEST, encoding='utf-8')
        binary = out / ('test.exe' if os.name == 'nt' else 'test')
        # Compile against each real board selection, not a guessed response
        # length. Both supported 500 families use 16 bytes; 14 belongs to LUV.
        for name in ['board.h', 'board_defaults.h']:
            (out / name).write_text((FW / 'include' / name).read_text(encoding='utf-8'), encoding='utf-8')
        for name, variant in [('Yardforce500', 'BOARD_YARDFORCE500_VARIANT_ORIG'),
                              ('Yardforce500B', 'BOARD_YARDFORCE500_VARIANT_B')]:
            if Path(args.cc).stem.lower() == 'cl':
                cmd = [args.cc, '/nologo', '/std:c11', '/utf-8', '/W3', f'/D{variant}=1', 'test.c', '/Fe:' + str(binary)]
            else:
                cmd = [args.cc, '-std=c11', '-Wall', '-Wextra', '-Werror', f'-D{variant}=1', 'test.c', '-o', str(binary)]
            print(f'Testing {name} with production board.h', flush=True)
            subprocess.run(cmd, cwd=out, check=True)
            subprocess.run([str(binary)], cwd=out, check=True)


if __name__ == '__main__':
    main()
