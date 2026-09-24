/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_param_store.h
 * @brief Flash region reserved for the runtime parameter log (fw_param_log.h).
 *
 * The region sits at the END of flash, outside the firmware image. platformio.ini
 * caps board_upload.maximum_size just below it, so an image that grows into it
 * fails the build instead of being silently overwritten by a commit:
 *   - STM32F103VC (Yardforce500):  last 4 x 2 KB pages, 0x0803E000-0x0803FFFF.
 *   - STM32F401VC (Yardforce500B): sector 5 (128 KB), 0x08020000-0x0803FFFF.
 * Flashing a new image (ST-Link / OpenOCD / the GUI) only erases the pages or
 * sectors the image covers, so persisted parameters survive a firmware update.
 * A full-chip erase clears them: the board then boots on compiled defaults and
 * the host re-sends and re-commits its values on the next connect.
 */
#ifndef FW_PARAM_STORE_H
#define FW_PARAM_STORE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** First word of the region (memory-mapped, read-only through this pointer). */
const uint32_t *fw_param_store_area(void);

/** Region size in 32-bit words. */
size_t fw_param_store_area_words(void);

/**
 * Erase the whole region. BLOCKING for tens of ms (F103) up to seconds (F401):
 * call it at boot only, before the window watchdog is armed.
 * @return 0 on success.
 */
int fw_param_store_erase(void);

/**
 * Program one 32-bit word at @p word_offset into already-erased space. Stalls
 * the CPU for tens of microseconds; safe from the main loop.
 * @return 0 on success.
 */
int fw_param_store_program_word(size_t word_offset, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAM_STORE_H */
