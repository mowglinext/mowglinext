/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file cobs.c
 * @brief Consistent Overhead Byte Stuffing (COBS) implementation.
 *
 * Algorithm
 * ---------
 * Encoding:
 *   Walk the input and group bytes between zeros. For each group:
 *     - Write an overhead byte that is (distance to the next zero + 1), capped
 *       at 0xFF = 255 meaning "no zero within the next 254 bytes".
 *     - Copy the non-zero bytes of the group.
 *     - If a zero was found (not at the natural group boundary of 254 bytes),
 *       do NOT emit it; it is represented implicitly by the overhead byte.
 *   Write a final overhead byte for the last group.
 *
 * Decoding:
 *   Read the first overhead byte (code). Copy (code - 1) non-zero bytes
 *   verbatim, then:
 *     - If code != 0xFF, emit a decoded zero byte (unless this is the last
 *       group, i.e. we have consumed all input).
 *     - Advance by code bytes and read the next overhead byte.
 *
 * No dynamic allocation. No library calls beyond size_t arithmetic.
 *
 * Compatible with the C++ cobs_encode / cobs_decode in the ROS 2 bridge
 * (mowgli_hardware/cobs.hpp).
 */

#include "cobs.h"

/* --------------------------------------------------------------------------
 * cobs_encode
 * --------------------------------------------------------------------------*/

size_t cobs_encode(const uint8_t *input, size_t len, uint8_t *output)
{
    /*
     * 'code_ptr' points to the overhead byte slot we are currently filling.
     * 'write'    is the current write position in output.
     * 'code'     tracks how many bytes since the last zero (or start).
     */
    size_t write = 0;
    size_t code_pos = 0;   /* index of the current overhead byte in output */
    uint8_t code = 1u;     /* distance to next zero + 1, starts at 1 */

    /* Reserve space for the first overhead byte. */
    write = 1u;

    for (size_t i = 0u; i < len; ++i) {
        if (input[i] == 0x00u) {
            /* Found a zero: fill in the overhead byte and start a new group. */
            output[code_pos] = code;
            code_pos = write;
            output[write++] = 1u; /* placeholder for next overhead byte */
            code = 1u;
        } else {
            output[write++] = input[i];
            ++code;

            if (code == 0xFFu) {
                /*
                 * Group is full (254 non-zero bytes). Close this group with
                 * overhead byte 0xFF (meaning "no zero in next 254 bytes")
                 * and start a new group.
                 */
                output[code_pos] = code;
                code_pos = write;
                output[write++] = 1u; /* placeholder */
                code = 1u;
            }
        }
    }

    /* Close the final group. */
    output[code_pos] = code;

    return write;
}

/* --------------------------------------------------------------------------
 * cobs_decode
 * --------------------------------------------------------------------------*/

size_t cobs_decode(const uint8_t *input, size_t len, uint8_t *output, size_t out_cap)
{
    size_t read = 0u;
    size_t write = 0u;

    while (read < len) {
        uint8_t code = input[read++];

        if (code == 0x00u) {
            /* 0x00 must never appear in valid COBS-encoded data. */
            return 0u;
        }

        /* Copy (code - 1) data bytes. */
        size_t copy_count = (size_t)(code - 1u);
        if (read + copy_count > len) {
            /* Encoded data claims more bytes than are available. */
            return 0u;
        }

        /* Output-buffer bound: refuse to write past the caller's buffer. A
         * frame that decodes to more than out_cap bytes cannot be a valid
         * packet (the largest is far smaller), so treat it as malformed rather
         * than overrun the destination. This is the hard guard against a long
         * mis-framed/garbage run corrupting memory past `output`. */
        if (write + copy_count > out_cap) {
            return 0u;
        }

        for (size_t i = 0u; i < copy_count; ++i) {
            output[write++] = input[read++];
        }

        /*
         * If this group's code was not 0xFF (full group with no embedded
         * zero), and we have not consumed all input yet, emit a decoded zero.
         */
        if (code != 0xFFu && read < len) {
            if (write >= out_cap) {
                return 0u;
            }
            output[write++] = 0x00u;
        }
    }

    return write;
}
