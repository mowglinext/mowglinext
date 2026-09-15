// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "mowgli_hardware/serial_port.hpp"
#include "mowgli_openmower_bridge/motor_link.hpp"
#include "mowgli_openmower_bridge/vesc_protocol.hpp"

namespace mowgli_openmower_bridge
{

/**
 * @brief xESC mini (VESC protocol) over a UART: polled — the bridge asks for
 *        the firmware version until it answers, then requests GET_VALUES on
 *        every poll and streams SET_DUTY commands.
 *
 * `motor_pole_pairs` turns the reported electrical rpm into shaft rpm.
 */
class XescMiniLink final : public MotorLink
{
public:
  using LogFn = std::function<void(const std::string&)>;

  XescMiniLink(std::string port, int motor_pole_pairs, LogFn warn);

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
    return "xesc_mini";
  }

  /// Exposed for tests: fold one decoded payload into the telemetry.
  void HandlePayload(const vesc::ParsedPayload& parsed, SteadyClock::time_point now);

private:
  bool SendFrame(const std::vector<uint8_t>& frame);

  std::string port_;
  int pole_pairs_;
  LogFn warn_;
  mowgli_hardware::SerialPort serial_;
  vesc::Deframer deframer_;
  MotorTelemetry telemetry_{};
  bool fw_known_{false};
  SteadyClock::time_point last_reconnect_attempt_{};
  SteadyClock::time_point last_fw_request_{};
};

}  // namespace mowgli_openmower_bridge
