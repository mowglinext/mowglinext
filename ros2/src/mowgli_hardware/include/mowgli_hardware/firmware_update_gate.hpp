// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
#pragma once

#include <cstdint>

namespace mowgli_hardware
{

/// Capabilities reported by the STM32 CONFIG_RSP active_flags byte.
constexpr std::uint32_t kFirmwareCapabilityUsbDfu = 1u << 0u;

/// Only these packet classes may cross the serial boundary while a firmware
/// update owns the board. Their callers additionally force motion/blade off,
/// IDLE mode, and suppress emergency-release bits before encoding.
constexpr bool firmware_update_packet_allowed(const std::uint8_t packet_id,
                                              const std::uint8_t heartbeat_id,
                                              const std::uint8_t high_level_id,
                                              const std::uint8_t cmd_vel_id,
                                              const std::uint8_t cmd_blade_id,
                                              const std::uint8_t config_req_id,
                                              const std::uint8_t enter_dfu_id)
{
  return packet_id == heartbeat_id || packet_id == high_level_id || packet_id == cmd_vel_id ||
         packet_id == cmd_blade_id || packet_id == config_req_id || packet_id == enter_dfu_id;
}

constexpr bool firmware_update_can_begin(const bool serial_open,
                                         const bool firmware_compatible,
                                         const bool usb_dfu_capable,
                                         const bool motion_commanded,
                                         const bool blade_commanded,
                                         const std::uint8_t current_mode,
                                         const std::uint8_t idle_mode)
{
  return serial_open && firmware_compatible && usb_dfu_capable && !motion_commanded &&
         !blade_commanded && current_mode == idle_mode;
}

constexpr bool firmware_update_may_release_emergency(const bool update_active,
                                                     const bool release_requested)
{
  return !update_active && release_requested;
}

constexpr std::uint8_t firmware_update_high_level_mode(const bool update_active,
                                                       const std::uint8_t current_mode,
                                                       const std::uint8_t idle_mode)
{
  return update_active ? idle_mode : current_mode;
}

constexpr double firmware_update_motion_component(const bool update_active, const double value)
{
  return update_active ? 0.0 : value;
}

}  // namespace mowgli_hardware
