// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// Bounded open-loop escape from a start pose Nav2 refuses to plan from.
// Pure logic, no ROS, so it is unit-testable standalone (see
// test_start_blocked_escape.cpp) — same shape as mowgli_hardware/dig_detector.hpp
// and mowgli_nav2_plugins/ftc_stall.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// Issue #487, field 2026-08-24: the robot undocked to (4.47, 4.70), which is
// inside the inflated keepout of a 0.25 m obstacle circle. SmacPlanner2D has no
// start tolerance, so every one of 26 plan calls answered "Start occupied",
// every blade-off sub-path transit was refused, and a whole field was forfeited
// at 0 % coverage. PR #495 made that pass recoverable WITHOUT motion (classify
// the refusal, exempt the pass from the retirement budget, clear the costmaps,
// retry). But clearing a costmap does not clear a KEEPOUT — a keepout is a
// static costmap FILTER — so from an unchanged pose the retry fails again and
// the area is eventually retired. Something has to physically move the robot
// off the cell it is standing on.
//
// ── Why the direction comes from the last motion ────────────────────────────
// The safe direction is genuinely ambiguous, and getting it wrong makes things
// worse rather than merely not better:
//
//   * Immediately after an undock the robot REVERSED into the blocked spot
//     (the dock is at ~(6.2, 2.8) and undock backs out on a ~126° bearing,
//     straight at the obstacle). Reversing again drives it DEEPER into the
//     obstacle it just backed onto.
//   * Mid-mow the robot ARRIVED DRIVING FORWARD (a promoted dig keepout can
//     appear under a robot that is already standing still). Reversing there
//     retraces ground it physically occupied seconds ago, which is the least
//     likely direction to find anything new.
//
// One signal gets both right by construction: move OPPOSITE the last motion the
// robot was actually commanded to make. Post-undock that was reverse, so we
// drive forward, away from the obstacle. Mid-mow that was forward, so we
// reverse onto known-good ground. This is option B from the proposal on #495,
// which is the option the maintainer picked.
//
// ── Stand-downs (the reason this is a decision function, not a drive loop) ──
// The direction is only as good as the signal it came from, so this header
// REFUSES to move rather than guess whenever:
//
//   * the escape is disabled by configuration;
//   * the start-blocked arming token from #495 is absent — the escape may only
//     ever fire on a CONFIRMED START_OCCUPIED-with-zero-progress pass, never on
//     any other failure;
//   * the blade is not VERIFIED off (not merely requested off, and not merely
//     unknown because the hardware status went stale);
//   * no non-zero forward command has been observed, the last one is below a
//     deadband, or it is older than signal_max_age_s.
//
// Every stand-down degrades to exactly the pre-existing #495 behaviour: no
// motion, clear the costmaps, retry, and eventually retire the area and dock.
// That is the whole point — a false escape hard-drives a healthy robot.
//
// ── Bounds ──────────────────────────────────────────────────────────────────
// Bounded in BOTH distance and time, like the dig escape. Distance is the
// intent; the timeout is what ends the manoeuvre if the robot is not actually
// moving (wheels blocked, collision_monitor zeroing the command, a lane of
// higher twist_mux priority holding the wire). Distance is integrated from the
// COMMANDED speed, never from the encoders: over-counting means the robot
// travels LESS than the budget, which is the safe direction to be wrong in.
//
// The configured bounds are additionally clamped to compiled ceilings by
// SanitizeEscapeCfg, so a bad YAML edit cannot turn a 0.4 m nudge into a drive
// across the lawn.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Hard compiled ceilings
// ---------------------------------------------------------------------------
//
// Not tunable. These exist so that the worst case an operator (or a corrupted
// config file) can configure is still a slow, short nudge. They are checked in
// the unit tests.

/// Absolute cap on the escape command magnitude [m/s]. Below the 0.20 m/s
/// mowing speed and below the 0.15 m/s undock speed.
inline constexpr double kEscapeMaxSpeed = 0.15;
/// Absolute cap on the distance budget [m]. Roughly one chassis length.
inline constexpr double kEscapeMaxDistance = 0.60;
/// Absolute cap on the hard time bound [s].
inline constexpr double kEscapeMaxTimeout = 15.0;
/// Absolute cap on how stale the direction signal may be allowed to be [s].
inline constexpr double kEscapeMaxSignalAge = 300.0;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct StartBlockedEscapeCfg
{
  /// Master switch. False = the recovery stays exactly as PR #495 left it.
  bool enabled = true;
  /// Magnitude of the open-loop command [m/s].
  double speed = 0.10;
  /// Distance budget [m], integrated from the COMMANDED speed.
  double distance = 0.40;
  /// Hard time bound [s]. Ends the manoeuvre even if the distance budget was
  /// never spent (blocked wheels, collision_monitor stop, a higher-priority
  /// twist_mux lane holding the wire).
  double timeout_s = 6.0;
  /// Commands below this magnitude are "not moving" and do not update the
  /// direction signal [m/s].
  double min_signal_speed = 0.03;
  /// A direction signal older than this is not trusted [s].
  double signal_max_age_s = 90.0;
};

/// Clamp a config read from YAML into the compiled envelope. Always call this
/// before using a config that came from outside the process.
inline StartBlockedEscapeCfg SanitizeEscapeCfg(StartBlockedEscapeCfg cfg)
{
  cfg.speed = std::clamp(std::abs(cfg.speed), 0.0, kEscapeMaxSpeed);
  cfg.distance = std::clamp(std::abs(cfg.distance), 0.0, kEscapeMaxDistance);
  cfg.timeout_s = std::clamp(std::abs(cfg.timeout_s), 0.0, kEscapeMaxTimeout);
  cfg.min_signal_speed = std::max(0.0, cfg.min_signal_speed);
  cfg.signal_max_age_s = std::clamp(std::abs(cfg.signal_max_age_s), 0.0, kEscapeMaxSignalAge);
  return cfg;
}

// ---------------------------------------------------------------------------
// Direction
// ---------------------------------------------------------------------------

enum class EscapeDirection
{
  kUnknown,  ///< stand down — never guess
  kForward,
  kReverse
};

/// The last forward motion the robot was actually commanded to make, taken from
/// twist_mux's MERGED output (i.e. what reached the wheels), not from any one
/// controller's lane. `valid` is false until a command above the deadband has
/// been seen at least once this session.
struct LastMotionSignal
{
  bool valid = false;
  double vx = 0.0;  ///< last commanded forward velocity above the deadband [m/s]
  double age_s = 0.0;  ///< seconds since that command was observed
};

/// Direction to escape in: the OPPOSITE of the last commanded motion.
/// kUnknown whenever the signal cannot be trusted — the caller must then
/// command nothing.
inline EscapeDirection EscapeDirectionFromLastMotion(const StartBlockedEscapeCfg& cfg,
                                                     const LastMotionSignal& signal)
{
  if (!signal.valid)
  {
    return EscapeDirection::kUnknown;
  }
  // NaN-safe: a NaN magnitude or age fails these comparisons and stands down.
  if (!(std::abs(signal.vx) >= cfg.min_signal_speed))
  {
    return EscapeDirection::kUnknown;
  }
  if (!(signal.age_s <= cfg.signal_max_age_s))
  {
    return EscapeDirection::kUnknown;
  }
  // We came in driving forward -> back out. We came in reversing -> drive out.
  return signal.vx > 0.0 ? EscapeDirection::kReverse : EscapeDirection::kForward;
}

// ---------------------------------------------------------------------------
// Preconditions and the decision
// ---------------------------------------------------------------------------

/// Everything outside the direction signal that must hold before the robot is
/// allowed to move.
struct EscapePreconditions
{
  /// The arming token set when #495's IsCoverageStartBlocked consumed a
  /// CONFIRMED start-pose-blocked pass. Without it there is no escape, full
  /// stop — this is the gate that keeps the motion off every other failure.
  bool start_blocked_armed = false;
  /// A hardware status message recent enough to describe the blade NOW.
  bool blade_state_fresh = false;
  /// That status reports the blade off. Verified, not merely requested.
  bool blade_off = false;
};

enum class EscapeVerdict
{
  kDisabled,
  kNotStartBlocked,
  kBladeNotVerifiedOff,
  kNoDirectionSignal,
  kEscapeForward,
  kEscapeReverse
};

inline const char* EscapeVerdictName(EscapeVerdict v)
{
  switch (v)
  {
    case EscapeVerdict::kDisabled:
      return "DISABLED";
    case EscapeVerdict::kNotStartBlocked:
      return "NOT_START_BLOCKED";
    case EscapeVerdict::kBladeNotVerifiedOff:
      return "BLADE_NOT_VERIFIED_OFF";
    case EscapeVerdict::kNoDirectionSignal:
      return "NO_DIRECTION_SIGNAL";
    case EscapeVerdict::kEscapeForward:
      return "ESCAPE_FORWARD";
    case EscapeVerdict::kEscapeReverse:
      return "ESCAPE_REVERSE";
  }
  return "UNKNOWN";
}

/// True only for the two verdicts that command motion.
inline bool EscapeMoves(EscapeVerdict v)
{
  return v == EscapeVerdict::kEscapeForward || v == EscapeVerdict::kEscapeReverse;
}

inline EscapeDirection EscapeVerdictDirection(EscapeVerdict v)
{
  switch (v)
  {
    case EscapeVerdict::kEscapeForward:
      return EscapeDirection::kForward;
    case EscapeVerdict::kEscapeReverse:
      return EscapeDirection::kReverse;
    default:
      return EscapeDirection::kUnknown;
  }
}

/// The single place that decides whether the robot moves at all, and which way.
///
/// Checked in order of how bad it would be to get them wrong, so the log line
/// names the FIRST reason the escape stood down.
inline EscapeVerdict EscapeDecide(const StartBlockedEscapeCfg& cfg,
                                  const EscapePreconditions& pre,
                                  const LastMotionSignal& signal)
{
  if (!cfg.enabled)
  {
    return EscapeVerdict::kDisabled;
  }
  if (!pre.start_blocked_armed)
  {
    return EscapeVerdict::kNotStartBlocked;
  }
  if (!pre.blade_state_fresh || !pre.blade_off)
  {
    return EscapeVerdict::kBladeNotVerifiedOff;
  }
  // A zero budget is a configured "do nothing", not an escape.
  if (!(cfg.speed > 0.0) || !(cfg.distance > 0.0) || !(cfg.timeout_s > 0.0))
  {
    return EscapeVerdict::kDisabled;
  }

  switch (EscapeDirectionFromLastMotion(cfg, signal))
  {
    case EscapeDirection::kForward:
      return EscapeVerdict::kEscapeForward;
    case EscapeDirection::kReverse:
      return EscapeVerdict::kEscapeReverse;
    case EscapeDirection::kUnknown:
    default:
      return EscapeVerdict::kNoDirectionSignal;
  }
}

// ---------------------------------------------------------------------------
// The bounded manoeuvre itself
// ---------------------------------------------------------------------------

struct StartBlockedEscapeState
{
  double travelled = 0.0;  ///< commanded-integral distance so far [m]
  double elapsed = 0.0;  ///< time spent escaping [s]
};

/// Either bound ends the escape. Both are checked, never just one.
inline bool EscapeDone(const StartBlockedEscapeCfg& cfg, const StartBlockedEscapeState& st)
{
  return st.travelled >= cfg.distance || st.elapsed >= cfg.timeout_s;
}

/// Advance the escape by one tick. Returns the forward velocity to command:
/// signed per `dir` while escaping, exactly 0.0 once either budget is spent or
/// if the direction is unknown.
inline double EscapeStep(const StartBlockedEscapeCfg& cfg,
                         StartBlockedEscapeState& st,
                         EscapeDirection dir,
                         double dt)
{
  if (dir == EscapeDirection::kUnknown || dt <= 0.0 || EscapeDone(cfg, st))
  {
    return 0.0;
  }

  const double speed = std::clamp(std::abs(cfg.speed), 0.0, kEscapeMaxSpeed);

  st.elapsed += dt;
  st.travelled = std::min(cfg.distance, st.travelled + speed * dt);

  return dir == EscapeDirection::kForward ? speed : -speed;
}

}  // namespace mowgli_behavior
