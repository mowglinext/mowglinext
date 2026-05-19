// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mowgli_localization/gnss_runtime_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_localization
{

enum class GnssBackendKind : uint8_t
{
  kUnknown = 0,
  kNmea = 1,
  kUblox = 2,
  kUnicore = 3,
};

struct GnssDiagnosticEntry
{
  uint8_t level{0};
  std::string message{};
  std::unordered_map<std::string, std::string> values{};
};

struct GnssDiagnosticSnapshot
{
  rclcpp::Time stamp{};
  std::unordered_map<std::string, GnssDiagnosticEntry> entries{};
};

GnssBackendKind ResolveGnssBackend(const std::string& gnss_backend, const std::string& gps_protocol);
GnssDiagnosticSnapshot BuildGnssDiagnosticSnapshot(const diagnostic_msgs::msg::DiagnosticArray& array);
GnssRuntimeState BuildGnssRuntimeStateFromFix(const sensor_msgs::msg::NavSatFix& fix,
                                              GnssBackendKind backend);
void EnrichGnssRuntimeStateFromDiagnostics(GnssRuntimeState& state,
                                           GnssBackendKind backend,
                                           const GnssDiagnosticSnapshot& snapshot,
                                           double diagnostics_timeout_sec);

}  // namespace mowgli_localization
