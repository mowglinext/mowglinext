// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file blade_policy.hpp
 * @brief When may the mow motor actually spin on OpenMower hardware?
 *
 * On the Mowgli STM32 the firmware is the blade authority: it refuses the
 * blade in IDLE, under emergency, and without a live host. OpenMower's
 * controllers have no such policy — whoever sends duty spins the blade — so
 * the bridge has to hold the same line before a duty byte leaves the host.
 * Pure and constexpr; the tests pin every branch.
 */

#pragma once

#include <cstdint>

namespace mowgli_openmower_bridge
{

// High-level modes, as in HighLevelStatus.msg and mowgli_hardware.
constexpr uint8_t HL_MODE_NULL = 0u;
constexpr uint8_t HL_MODE_IDLE = 1u;
constexpr uint8_t HL_MODE_AUTONOMOUS = 2u;
constexpr uint8_t HL_MODE_RECORDING = 3u;
constexpr uint8_t HL_MODE_MANUAL_MOWING = 4u;

struct BladeInputs
{
  bool requested{false};  ///< /hardware_bridge/mower_control mow_enabled
  bool mowing_enabled{true};  ///< mowgli_robot.yaml dry-run inhibit
  bool emergency{false};  ///< board or host emergency
  bool ll_link_alive{false};  ///< LowLevel status packets are fresh
  uint8_t hl_mode{HL_MODE_NULL};
  /// A HighLevelStatus arrived within high_level_status_timeout_s. The mode
  /// alone is a value remembered from the last message; if behavior_tree_node
  /// dies while it last said AUTONOMOUS, nothing else would ever stop the blade
  /// (the Mowgli STM32 has its own watchdog for this, the OpenMower ESC does
  /// not). mower_comms_v1 covers the same hole with a 25 s cmd_vel cutoff.
  bool hl_status_fresh{false};
  bool update_maintenance{false};  ///< image update in progress
};

/// The blade spins only when EVERY gate opens; a stop is never suppressed.
[[nodiscard]] constexpr bool BladeMayRun(const BladeInputs& in) noexcept
{
  if (!in.requested || !in.mowing_enabled || in.emergency || !in.ll_link_alive ||
      !in.hl_status_fresh || in.update_maintenance)
  {
    return false;
  }
  return in.hl_mode == HL_MODE_AUTONOMOUS || in.hl_mode == HL_MODE_MANUAL_MOWING;
}

/// Wheels: the firmware parity is a hard stop under emergency and in IDLE.
[[nodiscard]] constexpr bool WheelsMayRun(bool emergency,
                                          bool ll_link_alive,
                                          uint8_t hl_mode,
                                          bool cmd_fresh) noexcept
{
  if (emergency || !ll_link_alive || !cmd_fresh)
  {
    return false;
  }
  return hl_mode != HL_MODE_IDLE;
}

}  // namespace mowgli_openmower_bridge
