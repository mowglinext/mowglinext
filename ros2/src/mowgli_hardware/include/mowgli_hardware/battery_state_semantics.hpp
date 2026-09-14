// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <cstdint>
#include <limits>

namespace mowgli_hardware
{

/**
 * Convert the firmware status packet's legacy battery percentage field to the
 * sensor_msgs/BatteryState percentage convention.
 *
 * Firmware currently writes zero to this field because it has no state-of-
 * charge provider.  Zero is therefore not a measurement: it must not be
 * exposed as 0 %.  BatteryState specifies NaN for an unmeasured percentage.
 *
 * Keep the raw argument so a future firmware protocol which supplies a
 * verified SoC source has one explicit migration point.  Until then, every
 * raw value is intentionally unknown; this avoids inventing a voltage-to-SoC
 * estimate or treating the legacy zero as a legitimate empty battery.
 */
inline float battery_percentage_from_firmware(uint8_t /*raw_percentage*/)
{
  return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace mowgli_hardware
