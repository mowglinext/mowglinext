// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "mowgli_hardware/packet_handler.hpp"
#include "mowgli_hardware/serial_port.hpp"
#include "mowgli_openmower_bridge/motor_link.hpp"
#include "mowgli_openmower_bridge/xesc_2040_protocol.hpp"

namespace mowgli_openmower_bridge
{

struct Xesc2040Settings
{
  uint8_t hall_table[xesc2040::kHallTableSize]{255u, 5u, 3u, 4u, 1u, 6u, 2u, 255u};
  float motor_current_limit{0.5f};
  float acceleration{20.0f};
  bool has_motor_temp{false};
  float min_motor_temp{0.0f};
  float max_motor_temp{0.0f};
  float min_pcb_temp{0.0f};
  float max_pcb_temp{60.0f};
};

/**
 * @brief xESC 2040 over a UART: streams status packets on its own, needs a
 *        SETTINGS packet after boot (it reports FAULT_UNINITIALIZED until
 *        then) and a steady flow of CONTROL packets to keep its watchdog fed.
 */
class Xesc2040Link final : public MotorLink
{
public:
  using LogFn = std::function<void(const std::string&)>;

  Xesc2040Link(std::string port, Xesc2040Settings settings, LogFn warn);

  void Poll(SteadyClock::time_point now) override;
  bool SendDuty(double duty) override;

  [[nodiscard]] const MotorTelemetry& telemetry() const override
  {
    return telemetry_;
  }
  [[nodiscard]] const std::string& port() const override
  {
    return port_;
  }
  [[nodiscard]] const char* type_name() const override
  {
    return "xesc_2040";
  }

  /// Exposed for tests: decode one status payload into the telemetry.
  void HandleStatus(const xesc2040::StatusPacket& pkt, SteadyClock::time_point now);

private:
  void SendSettings(SteadyClock::time_point now);
  bool SendRaw(const uint8_t* payload, std::size_t len);
  void OnPacket(const uint8_t* data, std::size_t len, SteadyClock::time_point now);

  std::string port_;
  Xesc2040Settings settings_;
  LogFn warn_;
  mowgli_hardware::SerialPort serial_;
  mowgli_hardware::PacketHandler packets_;
  MotorTelemetry telemetry_{};
  SteadyClock::time_point last_reconnect_attempt_{};
  SteadyClock::time_point last_settings_sent_{};
  bool have_previous_status_{false};
  uint32_t previous_tacho_absolute_{0u};
};

}  // namespace mowgli_openmower_bridge
