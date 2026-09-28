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

constexpr BladeIntentDecision decide_blade_intent(
    const std::uint8_t previous_request,
    const std::uint32_t previous_generation, const bool command_received,
    const std::uint8_t command_request,
    const std::uint32_t command_generation, const bool idle,
    const bool emergency_active, const std::uint32_t current_generation) {
  const std::uint8_t requested = command_received
                                     ? (command_request != 0u ? 1u : 0u)
                                     : previous_request;
  const std::uint32_t request_generation =
      command_received ? command_generation : previous_generation;
  const bool stale_request =
      requested != 0u && request_generation != current_generation;
  const std::uint8_t safe_request =
      (idle || emergency_active || stale_request) ? 0u : requested;
  return {safe_request, safe_request,
          safe_request != 0u ? request_generation : current_generation};
}

#endif  // BLADE_EMERGENCY_POLICY_HPP
