// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure blade-intent policy for the firmware. Emergency and IDLE gates clear
// retained intent as well as the effective output, so removing a gate cannot
// restore an older blade-enable request.

#ifndef BLADE_EMERGENCY_POLICY_HPP
#define BLADE_EMERGENCY_POLICY_HPP

#include <cstdint>

struct BladeIntentDecision {
  std::uint8_t retained_request;
  std::uint8_t effective_output;
  std::uint32_t request_generation;
};

constexpr BladeIntentDecision stop_blade_intent(
    const std::uint32_t current_generation) {
  return {0u, 0u, current_generation};
}

constexpr bool blade_on_command_is_fresh(
    const std::uint32_t now, const std::uint32_t last_cmd_vel,
    const bool cmd_vel_seen, const std::uint32_t cmd_vel_timeout,
    const std::uint32_t last_heartbeat, const bool heartbeat_seen,
    const std::uint32_t heartbeat_timeout) {
  return cmd_vel_seen && heartbeat_seen &&
         (now - last_cmd_vel) <= cmd_vel_timeout &&
         (now - last_heartbeat) <= heartbeat_timeout;
}

constexpr BladeIntentDecision decide_blade_intent(
    const std::uint8_t previous_request,
    const std::uint32_t previous_generation, const bool command_received,
    const std::uint8_t command_request,
    const std::uint32_t command_generation, const bool idle,
    const bool emergency_active, const bool motor_link_rearm_required,
    const std::uint32_t current_generation) {
  const std::uint8_t requested = command_received
                                     ? (command_request != 0u ? 1u : 0u)
                                     : previous_request;
  const std::uint32_t request_generation =
      command_received ? command_generation : previous_generation;
  const bool stale_request =
      requested != 0u && request_generation != current_generation;
  const std::uint8_t safe_request =
      (idle || emergency_active || motor_link_rearm_required || stale_request)
          ? 0u
          : requested;
  return {safe_request, safe_request,
          safe_request != 0u ? request_generation : current_generation};
}

#endif  // BLADE_EMERGENCY_POLICY_HPP
