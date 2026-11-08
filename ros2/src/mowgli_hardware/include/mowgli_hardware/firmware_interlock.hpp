// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Firmware-protocol actuation interlock. Pure logic, no ROS, so it is
// unit-testable standalone (see test_firmware_interlock.cpp).
//
// ── Why ─────────────────────────────────────────────────────────────────────
// The bridge learns the board's wire-protocol version from the CONFIG_REQ /
// CONFIG_RSP handshake (kMowgliProtocolVersion). Until that handshake has
// reported the host's own protocol, the bridge cannot know that the board
// parses a CMD_VEL / CMD_BLADE / HIGH_LEVEL_STATE packet the way the host
// built it. Packet layouts CAN coincide across versions (v8 and v9 CMD_VEL
// do), so an incompatible board does not merely ignore commands — it drives.
// The behaviour tree's PreFlightCheck only blocks AUTONOMOUS mowing; teleop,
// tuning, docking and the GUI's manual-mowing blade service all reach the
// bridge without it. This interlock is the one chokepoint they share.
//
// ── What it does ────────────────────────────────────────────────────────────
// While actuation is not allowed the bridge only ever REDUCES what reaches the
// wire: zero wheel velocity, blade OFF, high-level mode IDLE (the firmware's
// hard-stop mode). Everything needed to recover — heartbeat (emergency
// stop/release), the version handshake, telemetry, parameter reads, reboot —
// is untouched. The STM32 firmware stays the sole safety authority
// (Invariant 9); this never commands anything the firmware would not already
// accept from a stopped robot.

#ifndef MOWGLI_HARDWARE__FIRMWARE_INTERLOCK_HPP_
#define MOWGLI_HARDWARE__FIRMWARE_INTERLOCK_HPP_

#include <cstdint>

namespace mowgli_hardware
{

/// High-level mode forwarded while the interlock is engaged. Equals
/// HL_MODE_IDLE (mowgli_protocol.h / HighLevelStatus.msg): the firmware
/// zeroes both wheel targets and the blade on it.
constexpr uint8_t kInterlockHighLevelMode = 1u;

/// Actuation is allowed only once a handshake has COMPLETED and reported the
/// host's protocol. "No handshake yet", "handshake timed out" and "different
/// protocol" all deny.
constexpr bool firmware_actuation_allowed(bool handshake_done, bool protocol_compatible)
{
  return handshake_done && protocol_compatible;
}

struct InterlockedVelocity
{
  double vx;
  double wz;
};

/// Velocity actually written to the wire: the request when allowed, zero
/// otherwise. Zero (not "nothing") so the firmware's own watchdog sees an
/// explicit stop rather than having to time out.
constexpr InterlockedVelocity interlock_velocity(bool allowed, double vx, double wz)
{
  return allowed ? InterlockedVelocity{vx, wz} : InterlockedVelocity{0.0, 0.0};
}

/// Blade-on byte actually written. An OFF always passes; an ON is forced OFF.
constexpr uint8_t interlock_blade_on(bool allowed, uint8_t requested_on)
{
  return allowed ? requested_on : uint8_t{0u};
}

/// High-level mode actually written. Any mode becomes IDLE while denied.
constexpr uint8_t interlock_high_level_mode(bool allowed, uint8_t requested_mode)
{
  return allowed ? requested_mode : kInterlockHighLevelMode;
}

/// True when this request would have been changed by the interlock — used to
/// decide whether a refusal is worth logging (a zero / OFF / IDLE request is
/// not a refusal).
constexpr bool interlock_refuses_velocity(bool allowed, double vx, double wz)
{
  return !allowed && (vx != 0.0 || wz != 0.0);
}

constexpr bool interlock_refuses_blade(bool allowed, uint8_t requested_on)
{
  return !allowed && requested_on != 0u;
}

constexpr bool interlock_refuses_mode(bool allowed, uint8_t requested_mode)
{
  return !allowed && requested_mode != kInterlockHighLevelMode && requested_mode != 0u;
}

enum class InterlockTransition
{
  kNone,
  kEngaged,  ///< allowed -> denied: actively stop the board
  kReleased,  ///< denied -> allowed: re-send the real high-level mode
};

constexpr InterlockTransition interlock_transition(bool was_allowed, bool now_allowed)
{
  if (was_allowed == now_allowed)
    return InterlockTransition::kNone;
  return now_allowed ? InterlockTransition::kReleased : InterlockTransition::kEngaged;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__FIRMWARE_INTERLOCK_HPP_
