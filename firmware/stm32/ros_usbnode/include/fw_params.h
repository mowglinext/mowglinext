/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file fw_params.h
 * @brief Runtime parameter table: values, host protocol (protocol v7), flash
 *        persistence.
 *
 * Lifecycle:
 *   1. fw_params_init() — at boot, BEFORE the window watchdog is armed: seeds
 *      every parameter with its compiled default, then overlays the newest
 *      valid record from the flash log. Erases (and rewrites) the log here if it
 *      is full or holds foreign data, because only here may erasing block.
 *   2. init_ROS() applies every group to the subsystems, so a board that never
 *      hears from the host still runs the persisted values.
 *   3. SET_PARAM (USB RX interrupt) -> fw_params_set(): coerce into the
 *      envelope, store, mark the group dirty and queue a report. The group is
 *      applied from the main loop (fw_params_take_dirty_groups), never from the
 *      interrupt.
 *   4. PARAM_COMMIT -> fw_params_request_commit(); fw_params_service() (main
 *      loop) encodes the persistent set and programs it one word per call.
 */
#ifndef FW_PARAMS_H
#define FW_PARAMS_H

#include <stdint.h>

#include "fw_param_catalog.h"
#include "mowgli_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void fw_params_init(void);

/** Current value of @p id; NaN when the id is unknown. */
float fw_params_get(uint16_t id);

/** Store a coerced value (interrupt-safe). @return fw_param_status_t. */
uint8_t fw_params_set(uint16_t id, float requested);

/** Bitmask (1 << fw_param_group_t) of groups changed since the last call. */
uint32_t fw_params_take_dirty_groups(void);

/** Mark every group dirty (init_ROS applies the boot values this way). */
void fw_params_mark_all_dirty(void);

/** Queue a PARAM_VALUE report for @p id, or for every parameter plus a
 *  PARAM_STORE_STATUS with FW_PARAM_ID_ALL (interrupt-safe). */
void fw_params_request_report(uint16_t id);

/** Ask for the current set to be persisted (interrupt-safe). */
void fw_params_request_commit(void);

/**
 * Main-loop service: runs a pending commit (encode + one flash word per call)
 * and sends at most one queued report, rate-limited so a GET_PARAM(ALL) burst
 * cannot overflow the USB TX queue.
 * @param now_ms HAL_GetTick().
 */
void fw_params_service(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAMS_H */
