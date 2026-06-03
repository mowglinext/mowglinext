/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file cobs.h
 * @brief Consistent Overhead Byte Stuffing (COBS) for STM32 firmware.
 *
 * COBS encodes an arbitrary byte buffer so that the output contains no 0x00
 * bytes. This makes 0x00 usable as an unambiguous packet frame delimiter over
 * any serial or USB-CDC link.
 *
 * Encoding overhead:
 *   At most 1 overhead byte per 254 input bytes, plus 1 trailing byte.
 *   Safe output buffer size: len + (len / 254) + 1 bytes.
 *
 * No dynamic allocation is used. Buffers are caller-supplied.
 *
 * Reference:
 *   S. Cheshire and M. Baker, "Consistent Overhead Byte Stuffing",
 *   IEEE/ACM Transactions on Networking, Vol. 7, No. 2, April 1999.
 */

#ifndef MOWGLI_COBS_H
#define MOWGLI_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute the worst-case size of a COBS-encoded buffer.
 *
 * Use this macro to size the output buffer before calling cobs_encode().
 *
 * @param raw_len Number of raw (pre-encoding) bytes.
 * @return Maximum number of bytes that cobs_encode() may write.
 */
#define COBS_MAX_ENCODED_SIZE(raw_len) \
    ((size_t)(raw_len) + ((size_t)(raw_len) / 254u) + 1u)

/**
 * @brief COBS-encode @p len bytes from @p input into @p output.
 *
 * The caller must ensure @p output is at least COBS_MAX_ENCODED_SIZE(len)
 * bytes. The function does NOT write frame delimiter bytes (0x00); framing
 * is the responsibility of the caller (see mowgli_comms_send()).
 *
 * @p input and @p output must not overlap.
 *
 * @param input   Pointer to the raw data to encode. Must not be NULL.
 * @param len     Number of bytes to encode. May be 0.
 * @param output  Destination buffer. Must not be NULL.
 * @return Number of bytes written to @p output.
 */
size_t cobs_encode(const uint8_t *input, size_t len, uint8_t *output);

/**
 * @brief COBS-decode @p len bytes from @p input into @p output.
 *
 * @p input must not contain any 0x00 bytes; the caller must strip frame
 * delimiters before calling this function.
 *
 * @p input and @p output must not overlap.
 *
 * @param input   COBS-encoded data without delimiters. Must not be NULL.
 * @param len     Number of encoded bytes. Must be >= 1.
 * @param output  Destination buffer. Must not be NULL.
 * @param out_cap Capacity of @p output in bytes. Decoding stops and returns 0
 *                if the decoded data would exceed this — a hard bound that
 *                prevents a long/garbage frame from overrunning @p output.
 * @return Number of decoded bytes written, or 0 if the input is malformed
 *         or would exceed @p out_cap.
 */
size_t cobs_decode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MOWGLI_COBS_H */
