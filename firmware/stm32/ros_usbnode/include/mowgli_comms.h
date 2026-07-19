/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file mowgli_comms.h
 * @brief Firmware-side packet communication layer over USB CDC.
 *
 * This module owns the entire packet lifecycle on the STM32 side:
 *
 *   Receive path:
 *     1. The USB CDC interrupt delivers raw bytes to mowgli_comms_process_rx().
 *     2. Incoming bytes are appended to a static ring buffer.
 *     3. On each 0x00 delimiter the accumulated COBS payload is decoded.
 *     4. The CRC-16 is verified.
 *     5. The first byte (packet type) is used to dispatch to a registered
 *        packet_handler_t callback.
 *
 *   Transmit path:
 *     1. Caller fills a packet struct and calls mowgli_comms_send() (or one
 *        of the typed convenience wrappers).
 *     2. CRC-16 is computed and written into the struct's crc field.
 *     3. The struct is COBS-encoded into a local stack buffer.
 *     4. A leading 0x00 delimiter, the encoded payload, and a trailing 0x00
 *        delimiter are forwarded to usb_cdc_transmit() (extern).
 *
 * Design constraints (STM32F103, 64 KB RAM, 256 KB Flash):
 *   - Zero dynamic allocation. All buffers are statically sized.
 *   - RX accumulation buffer: MOWGLI_COMMS_RX_BUF_SIZE bytes (default 512).
 *   - Maximum registered handlers: MOWGLI_COMMS_MAX_HANDLERS (default 16).
 *   - The TX encode buffer is allocated on the call stack of
 *     mowgli_comms_send(). Worst-case size for the largest packet
 *     (pkt_imu_t = 40 bytes): COBS_MAX_ENCODED_SIZE(40) = 41 bytes.
 *
 * Thread / interrupt safety:
 *   mowgli_comms_process_rx() is intended to be called from the USB CDC
 *   receive interrupt (or a deferred handler). mowgli_comms_send() should
 *   only be called from the main task context. If the application requires
 *   calling send() from an interrupt, the caller must ensure that usb_cdc_
 *   transmit() is interrupt-safe.
 */

#ifndef MOWGLI_COMMS_H
#define MOWGLI_COMMS_H

#include <stddef.h>
#include <stdint.h>

#include "mowgli_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Tunable constants — override in your build system before including this
 * header if the defaults do not suit your application.
 * ---------------------------------------------------------------------------*/

/**
 * @brief Size of the static RX accumulation buffer in bytes.
 *
 * Must be larger than the maximum COBS-encoded packet size. The largest
 * defined packet is pkt_imu_t (40 raw bytes -> 41 COBS bytes + 2 delimiters).
 * 512 bytes provides ample margin for back-to-back packets queued by the host.
 */
#ifndef MOWGLI_COMMS_RX_BUF_SIZE
#define MOWGLI_COMMS_RX_BUF_SIZE  512u
#endif

/**
 * @brief Maximum number of simultaneously registered packet handlers.
 *
 * One slot per distinct PKT_ID_* that the firmware wishes to receive.
 *
 * MUST be >= the number of mowgli_comms_register_handler() calls in
 * cpp_main.cpp. Registrations past the cap are DROPPED, and the packet simply
 * goes unanswered at runtime — there is no link-time or boot-time error. This
 * bit us at 8: cpp_main.cpp registers 10 handlers, so SET_SAFETY_LIMITS and
 * CONFIG_REQ (the last two) were never dispatched. The firmware therefore never
 * answered the host's version handshake, and the GUI reported "Firmware
 * incompatible / vunknown" forever — reflashing could not fix it, because the
 * binary itself was fine. Keep headroom here (each slot is a few bytes of .bss).
 */
#ifndef MOWGLI_COMMS_MAX_HANDLERS
#define MOWGLI_COMMS_MAX_HANDLERS  16u
#endif

/* ---------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------------*/

/**
 * @brief Callback type for received packet handlers.
 *
 * The callback receives a pointer to the raw (COBS-decoded, CRC-stripped)
 * packet bytes and the byte count. The first byte is always the packet type
 * (PKT_ID_*). The CRC bytes at the end have been consumed by the comms layer
 * and are NOT included in @p len.
 *
 * The callback is invoked synchronously from mowgli_comms_process_rx(), which
 * is typically called from USB interrupt context. Keep handlers short.
 *
 * @param data Pointer to the decoded packet bytes (type byte first).
 * @param len  Number of bytes in @p data (excludes the 2-byte CRC).
 */
typedef void (*packet_handler_t)(const uint8_t *data, size_t len);

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

/**
 * @brief Initialise the comms layer.
 *
 * Zeroes all internal state: RX buffer, write index, and handler table.
 * Call once from your firmware's main initialisation sequence, before
 * enabling the USB CDC interrupt.
 */
void mowgli_comms_init(void);

/**
 * @brief Feed raw bytes received from USB CDC into the comms layer.
 *
 * Call this function from the USB CDC receive callback (CDC_DataReceivedHandler
 * or equivalent). The function appends the bytes to the internal ring buffer,
 * then scans for 0x00 frame delimiters. Each complete frame is COBS-decoded,
 * CRC-verified, and dispatched to the matching registered handler.
 *
 * Bytes that would overflow the RX buffer are silently dropped; the overflow
 * counter is incremented (see mowgli_comms_get_rx_overflow_count()).
 *
 * @param data Pointer to the received byte array. Must not be NULL.
 * @param len  Number of bytes received.
 */
void mowgli_comms_process_rx(const uint8_t *data, size_t len);

/**
 * @brief Encode and transmit a packet over USB CDC.
 *
 * The function performs the following steps:
 *   1. Compute CRC-16 CCITT over the first (len - 2) bytes of @p packet.
 *   2. Write the CRC into the last two bytes of a local copy.
 *   3. COBS-encode the complete struct into a stack buffer.
 *   4. Call usb_cdc_transmit() with: [0x00][encoded bytes][0x00].
 *
 * @p packet is treated as read-only; the CRC is never written back to the
 * caller's struct (a local copy is used).
 *
 * @param packet Pointer to the packed packet struct (e.g. pkt_status_t *).
 *               The last two bytes MUST be the uint16_t crc field (the
 *               struct layouts in mowgli_protocol.h guarantee this).
 * @param len    sizeof(*packet). Must be >= 3 (1 type + at least 0 payload
 *               + 2 CRC bytes).
 */
void mowgli_comms_send(const void *packet, size_t len);

/**
 * @brief Register a handler for a specific packet type.
 *
 * Replaces any existing handler for @p packet_id. Registering NULL removes
 * the handler (incoming packets of that type are silently discarded after
 * CRC verification).
 *
 * At most MOWGLI_COMMS_MAX_HANDLERS distinct packet IDs may be registered
 * simultaneously. Attempting to register a new ID when the table is full
 * silently discards the registration.
 *
 * @param packet_id PKT_ID_* constant for the packet type to handle.
 * @param handler   Callback function, or NULL to deregister.
 */
void mowgli_comms_register_handler(uint8_t packet_id,
                                   packet_handler_t handler);

/**
 * @brief Send a PKT_ID_STATUS packet.
 *
 * Convenience wrapper around mowgli_comms_send(). The CRC field in
 * @p status does not need to be pre-computed; it is filled in internally.
 *
 * @param status Pointer to the populated status struct. Must not be NULL.
 */
void mowgli_comms_send_status(const pkt_status_t *status);

/**
 * @brief Send a PKT_ID_IMU packet.
 *
 * Convenience wrapper around mowgli_comms_send(). The CRC field in
 * @p imu does not need to be pre-computed; it is filled in internally.
 *
 * @param imu Pointer to the populated IMU struct. Must not be NULL.
 */
void mowgli_comms_send_imu(const pkt_imu_t *imu);

/**
 * @brief Send a PKT_ID_ODOMETRY packet.
 *
 * Convenience wrapper around mowgli_comms_send(). The CRC field in
 * @p odom does not need to be pre-computed; it is filled in internally.
 *
 * @param odom Pointer to the populated odometry struct. Must not be NULL.
 */
void mowgli_comms_send_odometry(const pkt_odometry_t *odom);

/**
 * @brief Return the number of RX bytes dropped due to buffer overflow.
 *
 * Useful for diagnosing communication problems. The counter is not cleared
 * automatically; call mowgli_comms_init() to reset it.
 *
 * @return Cumulative count of dropped bytes since last init.
 */
uint32_t mowgli_comms_get_rx_overflow_count(void);

/**
 * @brief Return the number of packets discarded due to CRC errors.
 *
 * @return Cumulative count of CRC-error packets since last init.
 */
uint32_t mowgli_comms_get_crc_error_count(void);

/* ---------------------------------------------------------------------------
 * External dependency — provided by usbd_cdc_if.c in the firmware project.
 * ---------------------------------------------------------------------------*/

/**
 * @brief Transmit @p len bytes over USB CDC.
 *
 * This function must be provided by the firmware's USB CDC layer. It is
 * called by mowgli_comms_send() to write the framed, COBS-encoded packet
 * to the USB endpoint.
 *
 * The implementation in the existing firmware is CDC_Transmit() in
 * usbd_cdc_if.c. Wire it up by adding a thin wrapper or by mapping this
 * symbol at link time.
 *
 * @param buf Pointer to bytes to transmit.
 * @param len Number of bytes.
 */
extern void usb_cdc_transmit(const uint8_t *buf, size_t len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MOWGLI_COMMS_H */
