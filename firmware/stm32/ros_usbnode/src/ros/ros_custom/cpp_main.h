/**
 * @file cpp_main.h
 * @brief Public C-linkage API for the COBS communication bridge.
 *
 * These functions are called from main.c (pure C context) via the NBT
 * timer system.
 */
#ifndef CPP_MAIN_H_
#define CPP_MAIN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_ROS(void);
void spinOnce(void);
void chatter_handler(void);
void DFU_Transition_Handler(void);
void motors_handler(void);
void panel_handler(void);
void broadcast_handler(void);
void ultrasonic_handler(void);
/**
 * Wheel-tick update — called from DRIVEMOTOR_App_Rx() every 20 ms.
 *
 * Ticks are SIGNED cumulative encoder counts (polarity encodes direction).
 * Raw motor-controller per-wheel speeds are passed through for telemetry;
 * the odometry packet's authoritative per-wheel velocity is computed here
 * from the signed-tick delta and the firmware-measured dt, so downstream
 * consumers don't have to divide by a jittery packet-arrival interval.
 */
void wheelTicks_handler(int32_t p_s32LeftTicksSigned,
                        int32_t p_s32RightTicksSigned,
                        int16_t p_s16LeftSpeed, int16_t p_s16RightSpeed);

uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len);

/* Required by mowgli_comms.c — implemented in cpp_main.cpp */
void usb_cdc_transmit(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CPP_MAIN_H_ */
