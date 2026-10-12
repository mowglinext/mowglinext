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
#include <stdarg.h>
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
static unsigned irq_enabled=1, stops, ll_stops, starts, submits, clears, aborts;
static unsigned detached_pin;
static unsigned fail_start, fail_stop, fail_second_stop;
static unsigned logs;
static char last_log[200];
static uint8_t *hardware_buffer;
static uint32_t hardware_length;
static uint32_t HAL_GetTick(void) { return tick; }
static uint32_t __get_PRIMASK(void) { return primask; }
static void __set_PRIMASK(uint32_t p) { primask=p; }
static uint32_t __get_IPSR(void) { return ipsr; }
static void debug_printf(const char *fmt, ...) {
    assert(!primask && !ipsr && irq_enabled);
    va_list args; va_start(args,fmt);
    vsnprintf(last_log,sizeof(last_log),fmt,args); va_end(args);
    ++logs;
}
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
    (void)d; assert(!irq_enabled && !primask && !ipsr); ++ll_stops;
    if (fail_stop || (fail_second_stop && hardware_buffer==NULL)) { return USBD_FAIL; }
    ++aborts; hardware_buffer=NULL; hardware_length=0;
    return USBD_OK;
}
typedef struct { unsigned Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIOA 1
#define GPIO_PIN_12 4096
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_SPEED_FREQ_VERY_HIGH 3
#define GPIO_AF10_OTG_FS 10
#define USB_OTG_GCCFG_PWRDWN (1u<<16)
static struct { unsigned GCCFG; } usb_regs = { USB_OTG_GCCFG_PWRDWN };
#define USB_OTG_FS (&usb_regs)
static void HAL_GPIO_WritePin(unsigned p, unsigned pin, unsigned v) {
    assert(p==GPIOA && pin==GPIO_PIN_12 && v==GPIO_PIN_RESET && !(usb_regs.GCCFG&USB_OTG_GCCFG_PWRDWN));
}
static void HAL_GPIO_Init(unsigned p, GPIO_InitTypeDef *g) {
    assert(p==GPIOA && g->Pin==GPIO_PIN_12 && !g->Pull && !irq_enabled && !primask);
    if (g->Mode==GPIO_MODE_OUTPUT_PP) {
        assert(!hUsbDeviceFS.pClassData && !(usb_regs.GCCFG&USB_OTG_GCCFG_PWRDWN) && g->Speed==GPIO_SPEED_FREQ_LOW);
        detached_pin=1;
    } else {
        assert(g->Mode==GPIO_MODE_AF_PP && g->Alternate==GPIO_AF10_OTG_FS && g->Speed==GPIO_SPEED_FREQ_VERY_HIGH);
        detached_pin=0;
    }
}
static void USB_DEVICE_Detach(void);
static void USB_DEVICE_Attach(void);
'''

TEST = r'''
void usb_cdc_transmit(const uint8_t *b, size_t n) { CDC_Transmit(b,(uint32_t)n); }
static unsigned commands;
static void command(const uint8_t *b, size_t n) { assert(b[0]==PKT_ID_CMD_VEL && n==sizeof(pkt_cmd_vel_t)-2); ++commands; }
static uint8_t USBD_Stop(USBD_HandleTypeDef *d) {
    assert(!primask && !ipsr && !irq_enabled);
    // Cube's single-class USBD_Stop ignores a SECOND LL_Stop result, then
    // deinitializes/frees CDC regardless. Keep that ownership boundary real.
    ++stops; (void)USBD_LL_Stop(d);
    assert(hardware_buffer==NULL && hardware_length==0);
    CDC_DeInit(); d->pClassData=NULL; klass.TxState=0;
    return USBD_OK;
}
static uint8_t USBD_Start(USBD_HandleTypeDef *d) {
    (void)d; assert(!primask && !ipsr && !irq_enabled && !detached_pin && (usb_regs.GCCFG&USB_OTG_GCCFG_PWRDWN)); ++starts;
    if (fail_start) { --fail_start; return USBD_FAIL; }
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
    CDC_ServiceRecovery(); assert(!CDC_GetUsbRecoveryCount() && !logs);
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
    assert(ll_stops==1 && !stops && !aborts && irq_enabled && klass.TxState && s_rx_write==encoded_len);
    assert(hardware_buffer && !CDC_GetUsbRecoveryCount() && !logs);
    // Repeated stop failures neither free ownership nor spam diagnostics.
    for (unsigned i=0;i<100;++i) { CDC_ServiceRecovery(); }
    assert(ll_stops==101 && !stops && !aborts && hardware_buffer && !logs);
    fail_stop=0;
    CDC_ServiceRecovery();
    assert(ll_stops==103 && stops==1 && aborts==2 && !irq_enabled && !CDC_TXQueue_GetReadAvailable());
    assert(!CDC_GetUsbRecoveryCount() && !logs);
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
    assert(CDC_GetUsbRecoveryCount()==1 && !logs); // no printing from Init IRQ
    configure(); assert(CDC_GetUsbRecoveryCount()==1); // ordinary configuration excluded
    uint8_t delim=0; uint32_t delim_len=1;
    CDC_Receive(&delim,&delim_len); assert(!commands); // old command cannot revive
    CDC_Receive(encoded,&encoded_size); CDC_Receive(&delim,&delim_len);
    assert(commands==1); // handlers survive; fresh commands still decode
    tick=900; receive(); CDC_ServiceRecovery();
    assert(logs==1 && strstr(last_log,"USB recoveries=1 busy_stuck=1 missing_completion=1 tick=900"));
    for (unsigned i=0;i<100;++i) { CDC_ServiceRecovery(); }
    assert(logs==1); // unchanged counters do not repeat
    CDC_Transmit(fresh,sizeof(fresh));
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
    assert(CDC_GetUsbRecoveryCount()==2 && logs==1);
    CDC_Transmit(fresh,sizeof(fresh));
    tick=7252; receive(); CDC_Transmit(old,sizeof(old)); CDC_ServiceRecovery();
    assert(stops==2 && klass.TxState); // cooldown bounds repeated failures
    // If Cube's redundant stop fails, the checked first call has already
    // quiesced hardware; freeing CDC remains safe and retry can proceed.
    tick=11502; receive(); CDC_Transmit(old,sizeof(old));
    unsigned prior_ll_stops=ll_stops, prior_aborts=aborts;
    fail_second_stop=1; CDC_ServiceRecovery(); fail_second_stop=0;
    assert(stops==3);
    assert(ll_stops==prior_ll_stops+2 && aborts==prior_aborts+1);
    assert(!hardware_buffer && !hUsbDeviceFS.pClassData && s_txRecoveryHold);

    tick=11752; fail_start=2; CDC_ServiceRecovery();
    assert(s_usbRecoveryState==CDC_USB_DETACHED && detached_pin && !irq_enabled);
    assert(CDC_GetUsbRecoveryCount()==2 && logs==1);
    receive(); CDC_TransmitCplt((uint8_t*)old,&n,1);
    CDC_Receive(encoded,&encoded_size);
    assert(s_txRecoveryHold && s_rx_write==0 && commands==1);
    unsigned attempts=starts;
    tick=12001; CDC_ServiceRecovery(); assert(starts==attempts);
    tick=12002; CDC_ServiceRecovery();
    assert(starts==attempts+1 && !irq_enabled && detached_pin && s_txRecoveryHold);
    for (unsigned i=0;i<100;++i) { CDC_ServiceRecovery(); }
    assert(starts==attempts+1 && CDC_GetUsbRecoveryCount()==2 && logs==1);
    tick=12251; CDC_ServiceRecovery(); assert(starts==attempts+1);
    tick=12252; CDC_ServiceRecovery(); assert(starts==attempts+2 && irq_enabled && s_txRecoveryHold);
    // Start alone is not a successful recovery, and enumeration still fences
    // callbacks/commands until Init creates the new CDC session.
    receive(); CDC_TransmitCplt((uint8_t*)old,&n,1);
    CDC_Receive(encoded,&encoded_size);
    assert(CDC_GetUsbRecoveryCount()==2 && commands==1 && s_rx_write==0 && s_txRecoveryHold);
    configure(); assert(CDC_GetUsbRecoveryCount()==3 && !s_txRecoveryHold);
    CDC_Receive(&delim,&delim_len); assert(commands==1);
    CDC_Receive(encoded,&encoded_size); CDC_Receive(&delim,&delim_len);
    assert(commands==2);
    receive(); CDC_Transmit(fresh,sizeof(fresh));
    assert(hardware_length==sizeof(fresh) && !memcmp(hardware_buffer,fresh,sizeof(fresh)));
    complete(); assert(!CDC_TXQueue_GetReadAvailable());
    tick=30899; CDC_ServiceRecovery(); assert(logs==1);
    tick=30900; CDC_ServiceRecovery();
    assert(logs==2 && strstr(last_log,"USB recoveries=3 ")); // aggregate suppressed events

    // Diagnostic throttling, like detach timing, is wrap-safe.
    s_usbRecoveryLogTick=UINT32_MAX-100;
    s_usbRecoveryLoggedCount=2;
    tick=29898; CDC_ServiceRecovery(); assert(logs==2);
    tick=29899; CDC_ServiceRecovery(); assert(logs==3);

    // Tick wrap still honours the detach interval.
    tick=UINT32_MAX-100; s_usbDetachTick=tick; s_usbRecoveryState=CDC_USB_DETACHED;
    irq_enabled=0; attempts=starts; tick=148; CDC_ServiceRecovery(); assert(starts==attempts);
    tick=149; CDC_ServiceRecovery(); assert(starts==attempts+1);
    puts("PASS: checked/vendor double-stop ownership, stop/start faults, stale callbacks/commands, reconnect TX/RX, cooldown and throttled recovery diagnostics");
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
        usb = (FW / 'src/usb_device.c').read_text()
        helpers = usb[usb.index('void USB_DEVICE_Detach(void)'):usb.index('/* USER CODE END 0 */')]
        helpers = helpers.replace('void USB_DEVICE_', 'static void USB_DEVICE_')
        (out / 'test.c').write_text(SHIM + header + comms + helpers + source + TEST)
        binary = out / ('test.exe' if os.name == 'nt' else 'test')
        subprocess.run([args.cc] + args.cc_arg + ['-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(FW / 'include'), str(out / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
