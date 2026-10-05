#!/usr/bin/env python3
"""Exercise production CDC recovery with native USB/time/IRQ fault shims."""
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
#include "mowgli_comms.h"
#include "cobs.h"
#include "crc16.h"
#define BOARD_YARDFORCE500_VARIANT_B 1
#define __weak
#define UNUSED(x) ((void)(x))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define USBD_OK 0
#define USBD_BUSY 1
#define USBD_FAIL 2
#define USBD_STATE_DEFAULT 1
#define USBD_STATE_CONFIGURED 3
#define CDC_DATA_FS_MAX_PACKET_SIZE 64
#define CDC_SEND_ENCAPSULATED_COMMAND 0
#define CDC_GET_ENCAPSULATED_RESPONSE 1
#define CDC_SET_COMM_FEATURE 2
#define CDC_GET_COMM_FEATURE 3
#define CDC_CLEAR_COMM_FEATURE 4
#define CDC_SET_LINE_CODING 5
#define CDC_GET_LINE_CODING 6
#define CDC_SET_CONTROL_LINE_STATE 7
#define CDC_SEND_BREAK 8
#define OTG_FS_IRQn 10
#define WATCHDOG_SetMainLoopStage(x) ((void)0)
#define debug_printf(...) ((void)0)
typedef struct { uint32_t bitrate; uint8_t format, paritytype, datatype; } USBD_CDC_LineCodingTypeDef;
typedef struct { uint32_t TxState; uint8_t *TxBuffer; uint32_t TxLength; } USBD_CDC_HandleTypeDef;
typedef struct { void *pClassData; unsigned dev_state; } USBD_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void), (*DeInit)(void);
    int8_t (*Control)(uint8_t,uint8_t*,uint16_t);
    int8_t (*Receive)(uint8_t*,uint32_t*);
    int8_t (*TransmitCplt)(uint8_t*,uint32_t*,uint8_t);
} USBD_CDC_ItfTypeDef;
static USBD_CDC_HandleTypeDef klass;
USBD_HandleTypeDef hUsbDeviceFS = { &klass, USBD_STATE_CONFIGURED };
static uint32_t tick, primask, ipsr;
static unsigned irq_enabled=1, stops, starts, submits, clears, aborts;
static unsigned detached_pin;
static unsigned fail_start, fail_stop;
static uint8_t *hardware_buffer;
static uint32_t hardware_length;
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __set_PRIMASK(uint32_t p) { primask=p; }
static uint32_t __get_IPSR(void) { return ipsr; }
static void HAL_NVIC_DisableIRQ(int irq) { assert(irq==OTG_FS_IRQn); irq_enabled=0; }
static void HAL_NVIC_EnableIRQ(int irq) { assert(irq==OTG_FS_IRQn); irq_enabled=1; }
static void HAL_NVIC_ClearPendingIRQ(int irq) { assert(irq==OTG_FS_IRQn && !irq_enabled); ++clears; }
static uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *d, uint8_t *b, uint32_t n) {
    assert(d->pClassData); klass.TxBuffer=b; klass.TxLength=n; return USBD_OK;
}
static uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *d, uint8_t *b) {
    (void)d; (void)b; return USBD_OK;
}
static uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *d) { (void)d; return USBD_OK; }
static uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *d) {
    assert(irq_enabled && d->pClassData && klass.TxState==0);
    klass.TxState=1; hardware_buffer=klass.TxBuffer; hardware_length=klass.TxLength;
    ++submits; return USBD_OK;
}
static uint8_t USBD_Stop(USBD_HandleTypeDef *d);
static uint8_t USBD_Start(USBD_HandleTypeDef *d);
static uint8_t USBD_LL_Stop(USBD_HandleTypeDef *d) {
    (void)d; assert(!irq_enabled && !primask); return fail_stop ? USBD_FAIL : USBD_OK;
}
static void USB_DEVICE_Detach(void) { assert(!irq_enabled && !primask && !hUsbDeviceFS.pClassData); detached_pin=1; }
static void USB_DEVICE_Attach(void) { assert(!irq_enabled && !primask); detached_pin=0; }
'''

TEST = r'''
void usb_cdc_transmit(const uint8_t *b, size_t n) { CDC_Transmit(b,(uint32_t)n); }
static unsigned commands;
static void command(const uint8_t *b, size_t n) { assert(b[0]==PKT_ID_CMD_VEL && n==sizeof(pkt_cmd_vel_t)-2); ++commands; }
static uint8_t USBD_Stop(USBD_HandleTypeDef *d) {
    assert(!primask && !ipsr && !irq_enabled);
    ++stops; ++aborts; hardware_buffer=NULL; hardware_length=0;
    CDC_DeInit(); d->pClassData=NULL; klass.TxState=0;
    return USBD_OK;
}
static uint8_t USBD_Start(USBD_HandleTypeDef *d) {
    (void)d; assert(!primask && !ipsr && !irq_enabled && !detached_pin); ++starts;
    if (fail_start) { fail_start=0; return USBD_FAIL; }
    return USBD_OK;
}
static void receive(void) {
    uint8_t b=0; uint32_t n=1;
    ipsr=1; CDC_Receive(&b,&n); ipsr=0;
}
static void configure(void) {
    hUsbDeviceFS.pClassData=&klass; hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;
    klass.TxState=0; ipsr=1; CDC_Init(); ipsr=0;
}
static void complete(void) {
    uint8_t *b=hardware_buffer; uint32_t n=hardware_length;
    klass.TxState=0; hardware_buffer=NULL; hardware_length=0;
    ipsr=1; CDC_TransmitCplt(b,&n,1); ipsr=0;
}
int main(void) {
    const uint8_t old[]={1,2,3}, fresh[]={9,8,7,6};
    mowgli_comms_init(); mowgli_comms_register_handler(PKT_ID_CMD_VEL,command);
    tick=100; configure(); receive();
    assert(CDC_TransmitString("")==USBD_OK);
    assert(CDC_Transmit(old,sizeof(old))==USBD_OK);
    pkt_cmd_vel_t cmd={0}; cmd.type=PKT_ID_CMD_VEL;
    uint8_t encoded[64];
    cmd.crc=crc16_ccitt((uint8_t*)&cmd,sizeof(cmd)-2);
    size_t encoded_len=cobs_encode((uint8_t*)&cmd,sizeof(cmd),encoded);
    uint32_t encoded_size=(uint32_t)encoded_len;
    tick=600; CDC_Transmit(old,sizeof(old));
    assert(!stops && !s_txRecoveryHold); // exact threshold is still transient
    tick=601; receive();
    CDC_Receive(encoded,&encoded_size); // only delimiter is missing
    assert(!commands && s_rx_write==encoded_len);
    CDC_Transmit(old,sizeof(old));
    assert(s_txRecoveryHold && klass.TxState && s_usbRecoveryState==CDC_USB_RECOVERY_REQUESTED);
    unsigned prior_submits=submits;
    uint32_t oldtail=s_txtail, n=3;
    receive(); CDC_NotifyUsbResume(); CDC_NotifyUsbConnect();
    CDC_TransmitCplt((uint8_t*)old,&n,1);
    assert(s_txRecoveryHold && s_txtail==oldtail && submits==prior_submits);
    primask=1; CDC_ServiceRecovery(); primask=0;
    ipsr=1; CDC_ServiceRecovery(); ipsr=0;
    assert(!stops); // never stop from IRQ/global critical section
    CDC_Receive(encoded,&encoded_size);
    assert(!commands && s_rx_write==encoded_len); // pending RX was rejected
    fail_stop=1; CDC_ServiceRecovery();
    assert(!stops && irq_enabled && klass.TxState && s_rx_write==encoded_len);
    fail_stop=0;
    CDC_ServiceRecovery();
    assert(stops==1 && aborts==1 && !irq_enabled && !CDC_TXQueue_GetReadAvailable());
    assert(detached_pin);
    assert(hUsbDeviceFS.dev_state==USBD_STATE_DEFAULT && !CDC_ShouldSendTelemetry());
    assert(s_rx_write==0); // Stop itself discarded pre-detach command assembly
    oldtail=s_txtail;
    receive(); CDC_TransmitCplt((uint8_t*)old,&n,1);
    CDC_Receive(encoded,&encoded_size); assert(s_rx_write==0 && !commands);
    assert(s_txtail==oldtail && s_txRecoveryHold);
    tick=850; CDC_ServiceRecovery(); assert(!starts);
    tick=851; CDC_ServiceRecovery();
    assert(starts==1 && clears==1 && irq_enabled && s_txRecoveryHold);
    assert(!detached_pin);
    CDC_ServiceRecovery(); assert(stops==1 && starts==1);
    CDC_NotifyUsbReset(); receive(); assert(s_txRecoveryHold);
    configure(); assert(!s_txRecoveryHold && CDC_ShouldSendTelemetry());
    uint8_t delim=0; uint32_t delim_len=1;
    CDC_Receive(&delim,&delim_len); assert(!commands); // old command cannot revive
    CDC_Receive(encoded,&encoded_size); CDC_Receive(&delim,&delim_len);
    assert(commands==1); // handlers survive; fresh commands still decode
    tick=900; receive(); CDC_Transmit(fresh,sizeof(fresh));
    assert(hardware_length==sizeof(fresh) && !memcmp(hardware_buffer,fresh,sizeof(fresh)));
    complete(); assert(!CDC_TXQueue_GetReadAvailable());

    // A host that stopped reading must not cause disconnect/reconnect storms.
    tick=6000; configure(); CDC_Transmit(old,sizeof(old));
    tick=6501; CDC_Transmit(old,sizeof(old));
    assert(s_txRecoveryHold && klass.TxState && s_usbRecoveryState==CDC_USB_RUNNING);
    CDC_ServiceRecovery(); assert(stops==1);
    receive(); CDC_Transmit(old,sizeof(old)); CDC_ServiceRecovery();
    assert(stops==2); // renewed OUT permits safe recovery of that same transfer
    tick=6751; CDC_ServiceRecovery(); configure(); receive();
    CDC_Transmit(fresh,sizeof(fresh));
    tick=7252; receive(); CDC_Transmit(old,sizeof(old)); CDC_ServiceRecovery();
    assert(stops==2 && klass.TxState); // cooldown bounds repeated failures
    tick=11502; receive(); CDC_Transmit(old,sizeof(old)); CDC_ServiceRecovery();
    assert(stops==3);

    tick=11752; fail_start=1; CDC_ServiceRecovery();
    assert(s_usbRecoveryState==CDC_USB_DETACHED && detached_pin && !irq_enabled);
    unsigned attempts=starts;
    tick=12001; CDC_ServiceRecovery(); assert(starts==attempts);
    tick=12002; CDC_ServiceRecovery(); assert(starts==attempts+1 && irq_enabled && s_txRecoveryHold);

    // Tick wrap still honours the detach interval.
    tick=UINT32_MAX-100; s_usbDetachTick=tick; s_usbRecoveryState=CDC_USB_DETACHED;
    irq_enabled=0; attempts=starts; tick=148; CDC_ServiceRecovery(); assert(starts==attempts);
    tick=149; CDC_ServiceRecovery(); assert(starts==attempts+1);
    puts("PASS: production USB timeout quiesces before reuse; asynchronous re-enumeration, callback fencing, host liveness, cooldown and tick wrap");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    parser.add_argument('--cc-arg', action='append', default=[], help='Compiler prefix argument (e.g. cc for Zig)')
    args = parser.parse_args()
    header = re.sub(r'^#include .*$', '', (FW / 'include/usbd_cdc_if.h').read_text(), flags=re.M)
    source = re.sub(r'^#include "[^"]+".*$', '', (FW / 'src/usbd_cdc_if.c').read_text(), flags=re.M)
    with tempfile.TemporaryDirectory(prefix='usb-recovery-') as directory:
        out = Path(directory)
        # Use the real protocol receiver override and production COBS/CRC too.
        start = source.index('__weak uint8_t CDC_DataReceivedHandler')
        end = source.index('/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */', start)
        callback = (FW / 'src/ros/ros_custom/cpp_main.cpp').read_text()
        begin = callback.index('uint8_t CDC_DataReceivedHandler(')
        finish = callback.index('\n}', begin) + 2
        source = source[:start] + callback[begin:finish] + source[end:]
        comms = ''
        for name in ['cobs.c', 'crc16.c', 'mowgli_comms.c']:
            comms += re.sub(r'^#include "[^"]+".*$', '', (FW / 'src' / name).read_text(), flags=re.M)
        (out / 'test.c').write_text(SHIM + header + comms + source + TEST)
        binary = out / ('test.exe' if os.name == 'nt' else 'test')
        subprocess.run([args.cc] + args.cc_arg + ['-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(FW / 'include'), str(out / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
