// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure heartbeat emergency policy.  Keep this independent of hardware so the
// host/firmware boundary's stop-over-release invariant is unit-testable.

#ifndef HEARTBEAT_EMERGENCY_POLICY_HPP
#define HEARTBEAT_EMERGENCY_POLICY_HPP

enum class HeartbeatEmergencyAction {
  Unchanged,
  Assert,
  Release,
  AutoClearWatchdog,
};

struct HeartbeatEmergencyDecision {
  HeartbeatEmergencyAction action;
  // This is consumed after every action by the firmware handler. It discards
  // watchdog-only provenance once the emergency is asserted, released,
  // auto-cleared, or found to have become physical.
  bool clear_heartbeat_only_latch;
};

// STOP ALWAYS DOMINATES RELEASE. A watchdog-only latch may still auto-clear on
// a normal heartbeat, but never when that heartbeat explicitly requests STOP.
constexpr HeartbeatEmergencyDecision decide_heartbeat_emergency(
    const bool emergency_requested, const bool emergency_release_requested,
    const bool physical_emergency, const bool watchdog_latch_active) {
  return emergency_requested
             ? HeartbeatEmergencyDecision{HeartbeatEmergencyAction::Assert,
                                          true}
         : watchdog_latch_active
             ? (physical_emergency
                    ? HeartbeatEmergencyDecision{HeartbeatEmergencyAction::
                                                     Unchanged,
                                                 true}
                    : HeartbeatEmergencyDecision{HeartbeatEmergencyAction::
                                                     AutoClearWatchdog,
                                                 true})
         : emergency_release_requested && !physical_emergency
             ? HeartbeatEmergencyDecision{HeartbeatEmergencyAction::Release,
                                          true}
             : HeartbeatEmergencyDecision{HeartbeatEmergencyAction::Unchanged,
                                          false};
}

#endif // HEARTBEAT_EMERGENCY_POLICY_HPP
