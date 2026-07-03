/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file mowgli_comms.c
 * @brief Firmware-side packet communication layer — implementation.
 *
 * See mowgli_comms.h for the full API description and design notes.
 *
 * Internal layout
 * ---------------
 * RX path uses a flat linear buffer (s_rx_buf) and a write index
 * (s_rx_write). The buffer is treated as a simple byte queue: bytes are
 * appended at s_rx_write and the scan for 0x00 delimiters always starts
 * from s_rx_scan (the last unprocessed position). When a delimiter is found:
 *
 *   frame   = s_rx_buf[frame_start .. delimiter - 1]   (COBS payload)
 *   decode  -> s_decode_buf[]
 *   verify CRC
 *   dispatch to handler
 *
 * After processing a frame the consumed bytes are compacted by memmove() so
 * that s_rx_write resets toward 0. This keeps the implementation simple and
 * avoids a circular-buffer pointer-arithmetic error at the cost of one memm-
 * ove per packet (~40 bytes on average). At the packet rates used here
 * (< 100 Hz) this is negligible on a 72 MHz Cortex-M3.
 *
 * TX path allocates the encode buffer on the stack. The largest packet is
 * pkt_imu_t (40 bytes raw) -> COBS_MAX_ENCODED_SIZE(40) = 41 bytes plus
 * 2 delimiter bytes = 43 bytes total. A 128-byte stack buffer covers every
 * current packet type with substantial margin.
 */

#include "mowgli_comms.h"
#include "cobs.h"
#include "crc16.h"
#include "main.h"

#include <string.h>  /* memmove, memcpy */

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Frame delimiter byte. */
#define FRAME_DELIM  0x00u

/**
 * Maximum raw (pre-COBS) packet size.
 * pkt_imu_t is the largest defined packet at 41 bytes. 64 bytes provides
 * a safe margin for future packet additions without recompilation.
 */
#define MAX_RAW_PKT_SIZE  64u

/**
 * Maximum COBS-encoded size for a packet of MAX_RAW_PKT_SIZE bytes.
 * = MAX_RAW_PKT_SIZE + (MAX_RAW_PKT_SIZE / 254) + 1
 */
#define MAX_ENC_PKT_SIZE  COBS_MAX_ENCODED_SIZE(MAX_RAW_PKT_SIZE)

/**
 * TX stack buffer. 2 delimiter bytes + COBS overhead on top of the largest
 * possible raw packet. 128 bytes is generous.
 */
#define TX_BUF_SIZE  128u

/* Minimum valid packet size: 1 type byte + 2 CRC bytes = 3 bytes raw.
 * After CRC stripping the handler sees >= 1 byte. */
#define MIN_RAW_PKT_SIZE  3u

/* ---------------------------------------------------------------------------
 * Handler table entry
 * ---------------------------------------------------------------------------*/

typedef struct {
    uint8_t          id;
    packet_handler_t handler;
} handler_entry_t;

/* ---------------------------------------------------------------------------
 * Module-level static state — all zero-initialised at startup by C runtime.
 * ---------------------------------------------------------------------------*/

/** RX byte accumulation buffer. */
static uint8_t s_rx_buf[MOWGLI_COMMS_RX_BUF_SIZE];

/** Number of valid bytes currently in s_rx_buf. */
static size_t  s_rx_write;

/** Decode scratch buffer (stack-sized equivalent on the heap). */
static uint8_t s_decode_buf[MAX_RAW_PKT_SIZE];

/** Registered packet handlers. */
static handler_entry_t s_handlers[MOWGLI_COMMS_MAX_HANDLERS];

/** Number of entries populated in s_handlers. */
static size_t s_handler_count;

/** Diagnostic counters. */
static uint32_t s_rx_overflow_count;
static uint32_t s_crc_error_count;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------------*/

/**
 * @brief Process a single raw COBS frame (the bytes between two delimiters).
 *
 * @param frame     Pointer to the first byte of the COBS-encoded payload.
 * @param frame_len Number of COBS-encoded bytes (delimiter bytes excluded).
 */
static void process_frame(const uint8_t *frame, size_t frame_len)
{
    /* Ignore empty frames (two consecutive delimiters). */
    if (frame_len == 0u) {
        return;
    }

    /*
     * Reject oversized frames BEFORE decoding. cobs_decode() takes no
     * output-capacity argument and writes up to frame_len bytes into the
     * 64-byte s_decode_buf; a corrupt or malicious stream of >MAX_ENC_PKT_SIZE
     * non-zero bytes would otherwise overrun the scratch buffer into adjacent
     * statics (the handler table). The largest legal packet is 41 bytes raw,
     * so anything that can't fit a MAX_RAW_PKT_SIZE packet is invalid.
     */
    if (frame_len > MAX_ENC_PKT_SIZE) {
        ++s_crc_error_count;
        return;
    }

    /* COBS-decode into scratch buffer. */
    size_t decoded_len = cobs_decode(frame, frame_len, s_decode_buf);
    if (decoded_len == 0u) {
        /* Malformed COBS — count as CRC error for simplicity. */
        ++s_crc_error_count;
        return;
    }

    /* Require at least type(1) + crc(2) bytes. */
    if (decoded_len < MIN_RAW_PKT_SIZE) {
        ++s_crc_error_count;
        return;
    }

    /*
     * Verify CRC-16. The CRC covers all bytes from the start of the decoded
     * buffer up to (but not including) the two CRC bytes at the end.
     */
    size_t payload_len = decoded_len - 2u;    /* bytes before the CRC */
    uint16_t computed_crc = crc16_ccitt(s_decode_buf, payload_len);

    /* CRC is stored little-endian (low byte first) at the end. */
    uint16_t received_crc =
        (uint16_t)s_decode_buf[payload_len] |
        ((uint16_t)s_decode_buf[payload_len + 1u] << 8u);

    if (computed_crc != received_crc) {
        ++s_crc_error_count;
        return;
    }

    /* Dispatch to handler. */
    uint8_t pkt_type = s_decode_buf[0u];
    for (size_t i = 0u; i < s_handler_count; ++i) {
        if (s_handlers[i].id == pkt_type && s_handlers[i].handler != NULL) {
            /* Pass payload_len so handler does not see the trailing CRC. */
            s_handlers[i].handler(s_decode_buf, payload_len);
            return;
        }
    }
    /* Unknown / unregistered packet type: silently discard. */
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

void mowgli_comms_init(void)
{
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_write = 0u;

    memset(s_handlers, 0, sizeof(s_handlers));
    s_handler_count = 0u;

    s_rx_overflow_count = 0u;
    s_crc_error_count   = 0u;
}

void mowgli_comms_process_rx(const uint8_t *data, size_t len)
{
    for (size_t i = 0u; i < len; ++i) {
        uint8_t byte = data[i];

        if (byte == FRAME_DELIM) {
            /*
             * Delimiter found. Everything in s_rx_buf[0 .. s_rx_write-1]
             * is the COBS payload of a complete frame (may be empty for a
             * leading delimiter at startup or after a previous frame).
             */
            process_frame(s_rx_buf, s_rx_write);

            /* Compact: reset the write pointer so the buffer is reused. */
            s_rx_write = 0u;
        } else {
            /* Non-delimiter byte: append to accumulation buffer. */
            if (s_rx_write < MOWGLI_COMMS_RX_BUF_SIZE) {
                s_rx_buf[s_rx_write++] = byte;
            } else {
                /* Buffer full — drop the byte and record the overflow. */
                ++s_rx_overflow_count;
            }
        }
    }
}

void mowgli_comms_send(const void *packet, size_t len)
{
    /*
     * Local mutable copy so we can fill in the CRC without touching the
     * caller's struct. MAX_RAW_PKT_SIZE is sufficient for all defined types.
     */
    uint8_t  raw[MAX_RAW_PKT_SIZE];
    uint8_t  encoded[TX_BUF_SIZE];

    if (packet == NULL || len < MIN_RAW_PKT_SIZE || len > MAX_RAW_PKT_SIZE) {
        return;
    }

    WATCHDOG_SetMainLoopStage(WATCHDOG_STAGE_CDC_TX_ENTER);

    /* Copy packet into mutable buffer. */
    memcpy(raw, packet, len);

    /*
     * Compute CRC over all bytes except the trailing 2-byte CRC field, then
     * store it little-endian in the last two bytes of the local copy.
     */
    size_t payload_len = len - 2u;
    uint16_t crc = crc16_ccitt(raw, payload_len);
    raw[payload_len]      = (uint8_t)(crc & 0xFFu);
    raw[payload_len + 1u] = (uint8_t)((crc >> 8u) & 0xFFu);

    /* COBS-encode the full packet (type + payload + CRC). */
    size_t encoded_len = cobs_encode(raw, len, encoded + 1u);
    /*
     * Shift: we left a gap at index 0 for the leading delimiter and will
     * append a trailing delimiter. encoded+1 is the COBS output start.
     * Rewrite into a contiguous buffer:
     *   encoded[0]                    = 0x00  (leading delimiter)
     *   encoded[1 .. encoded_len]     = COBS payload
     *   encoded[encoded_len + 1]      = 0x00  (trailing delimiter)
     */
    encoded[0]               = FRAME_DELIM;
    encoded[encoded_len + 1u] = FRAME_DELIM;

    usb_cdc_transmit(encoded, encoded_len + 2u);
}

void mowgli_comms_register_handler(uint8_t packet_id,
                                   packet_handler_t handler)
{
    /* Check if the ID is already registered — update in place. */
    for (size_t i = 0u; i < s_handler_count; ++i) {
        if (s_handlers[i].id == packet_id) {
            s_handlers[i].handler = handler;
            return;
        }
    }

    /* New ID — append if there is room. */
    if (s_handler_count < MOWGLI_COMMS_MAX_HANDLERS) {
        s_handlers[s_handler_count].id      = packet_id;
        s_handlers[s_handler_count].handler = handler;
        ++s_handler_count;
    }
    /* If the table is full, silently discard (caller can check capacity). */
}

void mowgli_comms_send_status(const pkt_status_t *status)
{
    if (status == NULL) {
        return;
    }
    mowgli_comms_send(status, sizeof(pkt_status_t));
}

void mowgli_comms_send_imu(const pkt_imu_t *imu)
{
    if (imu == NULL) {
        return;
    }
    mowgli_comms_send(imu, sizeof(pkt_imu_t));
}

void mowgli_comms_send_odometry(const pkt_odometry_t *odom)
{
    if (odom == NULL) {
        return;
    }
    mowgli_comms_send(odom, sizeof(pkt_odometry_t));
}

uint32_t mowgli_comms_get_rx_overflow_count(void)
{
    return s_rx_overflow_count;
}

uint32_t mowgli_comms_get_crc_error_count(void)
{
    return s_crc_error_count;
}
