// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Blade intent re-assert. Pure logic, no ROS, so it is unit-testable standalone
// (see test_blade_reassert.cpp) — same shape as blade_gate.hpp.
//
// ── The failure this exists for ─────────────────────────────────────────────
// CMD_BLADE is sent ONCE per MowerControl service call; nothing repeats it.
// Since firmware 1fd01ae9 (2026-09-25) the STM32 accepts a blade ON only when a
// CMD_VEL arrived within its 200 ms watchdog window, and treats a stale ON as an
// OFF that also clears its retained request. FollowStrip sends its ON right
// BEFORE dispatching the coverage goal (after a 1.5 s spin-up wait with no
// controller publishing), so that ON is stale by construction: the firmware
// drops it silently, the bridge keeps mow_enabled = true, HighLevelStatus keeps
// saying MOWING, and the robot drives the swath with the blade stopped (field
// log 2026-10-04 12:33:40, first mow on firmware v1.12.21).
//
// ── What it does ────────────────────────────────────────────────────────────
// When the host's intent is ON but the firmware REPORTS the blade stopped, the
// ON is sent again — only at a moment the firmware can accept it.
//
// ── NOT A SAFETY BYPASS ─────────────────────────────────────────────────────
// The STM32 firmware stays the sole blade safety authority: a re-assert is the
// same fire-and-forget request as the original, and the firmware still refuses
// it under an emergency, in IDLE, or while a motor-link re-arm is required. The
// host-side conditions below only stop the bridge from ASKING when it must not:
//   * never across an emergency — the firmware deliberately forgets the blade
//     request there so that clearing the emergency cannot restart the blade
//     (#763). `intent_authorized` is dropped on any emergency and only an
//     explicit MowerControl ON sets it again;
//   * never while the lift handling owns the blade (`enable_allowed`);
//   * never send ON against a stale CMD_VEL: when stale, the bridge first sends
//     a zero-velocity stop packet and retries ON only if that packet was written
//     within the firmware window; a blind stale re-send would STOP a running
//     blade.

#ifndef MOWGLI_HARDWARE__BLADE_REASSERT_HPP_
#define MOWGLI_HARDWARE__BLADE_REASSERT_HPP_

namespace mowgli_hardware
{

struct BladeReassertConfig
{
  // A re-assert is only sent when the last CMD_VEL packet left this long ago at
  // most. Half the firmware's CMD_VEL_TIMEOUT_MS (200 ms), so serial latency
  // cannot turn a fresh request into a stale one on the wire.
  double max_cmd_vel_age_s{0.10};
  // Minimum spacing between two CMD_BLADE packets. The firmware reports blade
  // activity every 250 ms; asking faster than it can answer is noise.
  double min_interval_s{0.5};
  // Blade telemetry older than this says nothing about the blade right now.
  double max_blade_status_age_s{1.0};
  // How long host intent and firmware report may disagree before it is logged.
  double mismatch_warn_after_s{3.0};
};

struct BladeReassertInputs
{
  bool mow_enabled{false};  ///< host intent after the dry-run gate
  bool enable_allowed{false};  ///< no lift in progress, no delayed lift resume pending
  bool intent_authorized{false};  ///< an explicit ON arrived since the last emergency
  bool emergency_active{false};  ///< host-requested or firmware-latched
  bool blade_status_fresh{false};
  bool blade_active{false};  ///< firmware-reported blade activity
  double cmd_vel_age_s{1e9};  ///< since the last CMD_VEL packet was written
  double since_last_blade_cmd_s{1e9};  ///< since the last CMD_BLADE packet was written
};

// True when the host wants the blade on and the firmware says it is not.
constexpr bool blade_intent_mismatch(const BladeReassertInputs& in)
{
  return in.mow_enabled && in.enable_allowed && in.blade_status_fresh && !in.blade_active;
}

// True when an authorized retry is due. The bridge must send a zero CMD_VEL
// first when the velocity stream is stale, then send the blade ON only if that
// stop packet was written freshly enough for the firmware gate.
constexpr bool blade_reassert_due(const BladeReassertInputs& in,
                                  const BladeReassertConfig& cfg = {})
{
  return blade_intent_mismatch(in) && in.intent_authorized && !in.emergency_active &&
         in.since_last_blade_cmd_s >= cfg.min_interval_s;
}

// True when a blade ON should be sent again now.
constexpr bool should_reassert_blade_on(const BladeReassertInputs& in,
                                        const BladeReassertConfig& cfg = {})
{
  return blade_reassert_due(in, cfg) && in.cmd_vel_age_s <= cfg.max_cmd_vel_age_s;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__BLADE_REASSERT_HPP_
