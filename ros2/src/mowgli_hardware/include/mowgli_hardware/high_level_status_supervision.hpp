// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Behaviour-tree liveness supervision for the hardware bridge. Pure logic, no
// ROS, so it is unit-testable standalone (see test_high_level_status_supervision.cpp).
//
// ── Why ─────────────────────────────────────────────────────────────────────
// The bridge re-sends the last BT-reported high-level mode to the firmware on a
// timer. If behavior_tree_node dies while that mode is AUTONOMOUS (or any other
// mode in which the firmware acts on cmd_vel and the blade), nothing downstream
// notices: the bridge keeps announcing AUTONOMOUS, Nav2 finishes the controller
// goal the dead tree last sent while none of the tree's guards (boundary,
// localization, rain, battery) run, and the blade stays powered until the
// firmware's own blade timeout.
//
// The BT re-publishes HighLevelStatus at 1 Hz even while a long action runs, so
// silence for several seconds means the tree is gone (or its DDS reader is
// stuck, which looks identical from here and fails safe the same way).
//
// ── What it does ────────────────────────────────────────────────────────────
// A status that was received and then stayed silent for longer than the timeout,
// while the mode announced to the firmware is one it actuates in, counts as LOST.
// The bridge checks this on every cmd_vel and blade request and on its
// high-level timer, so an idle robot is stopped at most one timer tick late.
// The bridge then applies the existing firmware interlock (firmware_interlock.hpp):
// wheels zero, blade OFF, mode IDLE. It only ever reduces what reaches the wire,
// and releases on its own as soon as a fresh status arrives.
//
// A bridge that has never received a status is not supervised: before the BT has
// booted, the cmd_vel fallback in hardware_bridge_node.cpp owns that case.
// IDLE and NULL are not supervised either — the firmware already holds the
// wheels and blade stopped in them.

#ifndef MOWGLI_HARDWARE__HIGH_LEVEL_STATUS_SUPERVISION_HPP_
#define MOWGLI_HARDWARE__HIGH_LEVEL_STATUS_SUPERVISION_HPP_

#include <cstdint>

namespace mowgli_hardware
{

/// Default silence, in seconds, after which a BT status counts as lost. Five
/// missed 1 Hz republishes: short enough that Nav2 cannot drive the dead tree's
/// goal to its end. A host or tree stalled for longer than this is stopped too
/// (fails safe, and releases on the next status).
inline constexpr double kDefaultHighLevelStatusTimeoutS = 5.0;

/// True for every mode in which the firmware acts on cmd_vel or the blade.
/// HL_MODE_NULL (0) and HL_MODE_IDLE (1) are the firmware's stopped modes.
/// An unknown mode is treated as actuating: unsupervised is the unsafe default.
constexpr bool high_level_mode_is_supervised(uint8_t mode)
{
  return mode != 0u && mode != 1u;
}

/// True when the BT status has gone silent in a mode that needs a live BT.
/// `timeout_s <= 0` disables the supervision. A receipt that is not older than
/// `now_ns` (monotonic clock) is fresh.
constexpr bool high_level_status_lost(bool have_status,
                                      uint8_t reported_mode,
                                      std::int64_t now_ns,
                                      std::int64_t receipt_ns,
                                      double timeout_s)
{
  return timeout_s > 0.0 && have_status && high_level_mode_is_supervised(reported_mode) &&
         now_ns > receipt_ns && static_cast<double>(now_ns - receipt_ns) * 1.0e-9 > timeout_s;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__HIGH_LEVEL_STATUS_SUPERVISION_HPP_
