// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// `mowing_enabled` blade gate. Pure logic, no ROS, so it is unit-testable
// standalone (see test_blade_gate.cpp) — same shape as dig_detector.hpp.
//
// ── NOT A SAFETY INTERLOCK ──────────────────────────────────────────────────
// The STM32 firmware is the SOLE blade safety authority; blade commands from
// ROS2 are fire-and-forget and firmware decides whether to execute them. This
// gate is a CONVENIENCE / DRY-RUN INHIBIT: it lets an operator drive a full
// mowing mission with the blade never spinning (commissioning, coverage-path
// tuning, indoor testing). It can never make the robot safer than the firmware
// already makes it, and no UI/doc copy may present it as a cut-out.
//
// ── Why it lives here and not in the BT ─────────────────────────────────────
// Every blade-ON request in the system converges on ONE service,
// /hardware_bridge/mower_control. Its callers are the BT's SetMowerEnabled,
// FollowStrip::setBladeEnabled, AND the GUI's own direct call — so a BT-side
// gate would be bypassed by the GUI's blade button. This is the same
// "gate at the merged chokepoint, not per-lane" argument CLAUDE.md Invariant 16
// makes for the dig detector on ~/cmd_vel.
//
// ── The one asymmetry that matters ──────────────────────────────────────────
// Only an ENABLE is ever suppressed. A DISABLE request MUST always pass through
// untouched, in either state — swallowing a stop would turn a convenience knob
// into a false safety promise, which is exactly the bug this exists to avoid.

#ifndef MOWGLI_HARDWARE__BLADE_GATE_HPP_
#define MOWGLI_HARDWARE__BLADE_GATE_HPP_

namespace mowgli_hardware
{

// Resolve a blade request against the `mowing_enabled` dry-run inhibit.
//
//   requested_enable  what the caller asked for (true = spin the blade)
//   mowing_enabled    mowgli_robot.yaml `mowing_enabled` (default true)
//
// Returns what should actually be commanded. Note the shape: a DISABLE
// (requested_enable == false) returns false regardless of `mowing_enabled`, so
// a stop is never swallowed.
constexpr bool blade_enable_allowed(bool requested_enable, bool mowing_enabled)
{
  return requested_enable && mowing_enabled;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__BLADE_GATE_HPP_
