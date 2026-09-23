// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// LD19 spin-motor PWM state machine. Pure logic, no ROS, no filesystem — unit
// -testable standalone (see test_pwm_controller.cpp), same shape as
// mowgli_hardware/dig_detector.hpp.
//
// ── Why this exists (issue #569) ────────────────────────────────────────────
// The LD19's spin motor runs continuously today: its PWM control pin is
// grounded, which per the datasheet forces fixed internal self-regulation at
// ~10 Hz regardless of whether the robot is mowing or parked on the dock for
// 24 hours. Continuous, needless mechanical wear and wasted heat.
//
// ── Why the default GPIO is 12 (physical pin 32), not what the issue proposed
// The issue assumed GPIO12/13 (pins 32/33) were free because "already used
// for UART to the mainboard" — they are not; the STM32 mainboard talks over
// USB CDC. GPIO12/13 are this repo's own DEFAULT LD19 scan-data UART (UART5,
// /dev/ttyAMA5 — install/lib/lidar.sh). The LD19 only ever TRANSMITS scan
// data (LiDAR TX -> Pi RXD5/GPIO13/pin33); it never receives anything on
// TXD5/GPIO12/pin32, so that pin is claimed by the uart5 overlay but
// functionally idle. Repurposing ONLY GPIO12 for hardware PWM (a second
// dtoverlay=pwm,pin=12,func=4 line after dtoverlay=uart5) frees it without
// touching the scan-data path on GPIO13. This is a manual, documented boot
// config step (see the lidar_pwm_gpio_pin setting's description) — NOT
// installer-automated.
//
// ── What this class does NOT do ─────────────────────────────────────────────
// It does not touch sysfs (see sysfs_pwm.hpp) and it does not decide overall
// mission safety. Once RUNNING is reached once, it stays RUNNING for as long
// as `active` stays true — it does NOT re-supervise scan health afterward.
// That is deliberate: the existing three-layer scan-staleness defense
// (mowgli_behavior/scan_pause.hpp, collision_monitor's source_timeout,
// SensorSafetyGuard's IsScanStale in main_tree.xml) already owns "was
// healthy, then went stale" and duplicating that logic here with a different
// threshold would be a second, inconsistent source of truth. This class's
// only job is getting the motor spinning and confirming it ONCE before the
// BT is allowed to leave IDLE — see mowgli_behavior's pre-undock precondition
// that consumes LidarMotorStatus.
//
// ── Duty-cycle numbers are placeholders ──────────────────────────────────────
// The enter-external-control handshake (30 kHz carrier, duty cycle in
// (45%, 55%) held >=100 ms) is the one number confirmed from the issue's own
// datasheet citation. `run_duty` and `stop_duty` are NOT — the LD19
// datasheet's duty-cycle-to-speed mapping must be read during hardware
// bring-up and these defaults tuned to match. Do not treat them as verified.

#ifndef MOWGLI_LIDAR_PWM__PWM_CONTROLLER_HPP_
#define MOWGLI_LIDAR_PWM__PWM_CONTROLLER_HPP_

#include <algorithm>

namespace mowgli_lidar_pwm
{

enum class PwmMotorState
{
  kIdle,
  kSpinningUp,
  kRunning,
  kSpinningDown,
  kFault
};

struct PwmControllerCfg
{
  /// Duty cycle (0-1) that takes the LD19 out of internal self-regulation
  /// into external control. Datasheet: (0.45, 0.55), midpoint by default.
  double handshake_duty = 0.50;
  /// How long handshake_duty must be held before switching to run_duty.
  /// Datasheet minimum is 100 ms; held a little longer for margin.
  double handshake_hold_s = 0.15;
  /// Target duty cycle once running. PLACEHOLDER — confirm against the LD19
  /// datasheet during hardware bring-up.
  double run_duty = 0.80;
  /// Duty cycle while idle/stopped. PLACEHOLDER — confirm against the LD19
  /// datasheet during hardware bring-up (may not be 0.0; the datasheet may
  /// define a specific "stop" duty distinct from simply de-energising).
  double stop_duty = 0.0;
  /// Bound on SPINNING_UP before declaring FAULT if no fresh scan has
  /// arrived yet. Must comfortably exceed the LD19's real spin-up time.
  double spinup_timeout_s = 8.0;
  /// SPINNING_DOWN is held this long before settling to IDLE, so a brief
  /// flicker of `active` doesn't chatter the motor on/off.
  double spindown_settle_s = 1.0;
};

struct PwmControllerState
{
  PwmMotorState state = PwmMotorState::kIdle;
  double time_in_state = 0.0;
};

struct PwmCommand
{
  PwmMotorState state = PwmMotorState::kIdle;
  double duty_cycle = 0.0;
};

namespace detail
{
inline void Enter(PwmControllerState & st, PwmMotorState next)
{
  if (st.state != next) {
    st.state = next;
    st.time_in_state = 0.0;
  }
}
}  // namespace detail

/// Advance the controller by one tick.
///
/// @param active     true whenever the robot is anywhere other than IDLE
///                   (mowgli_interfaces/HighLevelStatus.state != HIGH_LEVEL_STATE_IDLE)
/// @param scan_fresh true when the caller has observed a recent /scan —
///                   supplied by the node's own subscription, not owned here
/// @param dt         tick duration [s]
inline PwmCommand PwmControllerStep(
  const PwmControllerCfg & cfg,
  PwmControllerState & st,
  bool active,
  bool scan_fresh,
  double dt)
{
  if (dt > 0.0) {
    st.time_in_state += dt;
  }

  switch (st.state) {
    case PwmMotorState::kIdle:
      if (active) {
        detail::Enter(st, PwmMotorState::kSpinningUp);
      }
      break;

    case PwmMotorState::kSpinningUp:
      if (!active) {
        detail::Enter(st, PwmMotorState::kSpinningDown);
      } else if (scan_fresh) {
        detail::Enter(st, PwmMotorState::kRunning);
      } else if (st.time_in_state >= cfg.spinup_timeout_s) {
        detail::Enter(st, PwmMotorState::kFault);
      }
      break;

    case PwmMotorState::kRunning:
      // Deliberately does not re-check scan_fresh here — see the class
      // comment. Only `active` going false ends this state.
      if (!active) {
        detail::Enter(st, PwmMotorState::kSpinningDown);
      }
      break;

    case PwmMotorState::kSpinningDown:
      if (active) {
        // Changed its mind mid-spin-down — resume spinning up rather than
        // finish stopping and immediately having to start again.
        detail::Enter(st, PwmMotorState::kSpinningUp);
      } else if (st.time_in_state >= cfg.spindown_settle_s) {
        detail::Enter(st, PwmMotorState::kIdle);
      }
      break;

    case PwmMotorState::kFault:
      if (!active) {
        detail::Enter(st, PwmMotorState::kSpinningDown);
      } else if (scan_fresh) {
        // Recovered — a scan finally arrived (e.g. the vendor driver was
        // slow to come up, not a real motor fault).
        detail::Enter(st, PwmMotorState::kRunning);
      }
      break;
  }

  PwmCommand cmd;
  cmd.state = st.state;
  switch (st.state) {
    case PwmMotorState::kIdle:
      cmd.duty_cycle = cfg.stop_duty;
      break;
    case PwmMotorState::kSpinningUp:
      cmd.duty_cycle = st.time_in_state < cfg.handshake_hold_s ? cfg.handshake_duty : cfg.run_duty;
      break;
    case PwmMotorState::kRunning:
    case PwmMotorState::kFault:
      cmd.duty_cycle = cfg.run_duty;
      break;
    case PwmMotorState::kSpinningDown:
      cmd.duty_cycle = cfg.stop_duty;
      break;
  }
  cmd.duty_cycle = std::clamp(cmd.duty_cycle, 0.0, 1.0);
  return cmd;
}

}  // namespace mowgli_lidar_pwm

#endif  // MOWGLI_LIDAR_PWM__PWM_CONTROLLER_HPP_
