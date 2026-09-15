// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file motor_link.hpp
 * @brief Common interface over the two OpenMower motor-controller flavours.
 *
 * A link owns one serial port, is polled from the node's control tick (no
 * threads), reconnects on its own, and exposes the latest telemetry in the
 * units the bridge needs. `signed_ticks` is a monotonic SIGNED cumulative
 * tick counter in the controller's own direction convention; the bridge
 * applies per-motor inversion on top.
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace mowgli_openmower_bridge
{

using SteadyClock = std::chrono::steady_clock;

struct MotorTelemetry
{
  bool connected{false};  ///< port open AND a status packet within the timeout
  bool has_status{false};  ///< at least one status packet since (re)connect
  double voltage_in{0.0};
  double temp_pcb{0.0};
  double temp_motor{0.0};
  double current_in{0.0};
  double duty{0.0};
  double rpm{0.0};  ///< shaft rpm when the controller reports it, else 0
  uint32_t fault_code{0u};
  int64_t signed_ticks{0};
  uint8_t fw_major{0u};
  uint8_t fw_minor{0u};
  SteadyClock::time_point last_status{};
};

class MotorLink
{
public:
  virtual ~MotorLink() = default;

  /// Service the serial link: reconnect, read, parse, refresh telemetry.
  virtual void Poll(SteadyClock::time_point now) = 0;

  /// Send a duty command in [-1, 1]. Returns false when nothing was written.
  virtual bool SendDuty(double duty) = 0;

  [[nodiscard]] virtual const MotorTelemetry& telemetry() const = 0;
  [[nodiscard]] virtual const std::string& port() const = 0;
  [[nodiscard]] virtual const char* type_name() const = 0;
};

/// A controller is considered gone when it stays silent this long.
constexpr std::chrono::milliseconds kMotorStatusTimeout{1000};
/// Minimum spacing between reopen attempts of a dead port.
constexpr std::chrono::milliseconds kMotorReconnectPeriod{1000};

}  // namespace mowgli_openmower_bridge
