// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <cmath>
#include <cstdint>

#include "mowgli_hardware/battery_state_semantics.hpp"
#include <gtest/gtest.h>

namespace mowgli_hardware
{

TEST(BatteryStateSemantics, LegacyFirmwarePercentageIsUnknown)
{
  // Firmware currently emits 0 while no SoC source exists.  BatteryState uses
  // NaN, rather than 0.0, for an unmeasured percentage.
  EXPECT_TRUE(std::isnan(battery_percentage_from_firmware(0)));
}

TEST(BatteryStateSemantics, UnimplementedFieldCannotMasqueradeAsMeasurement)
{
  // The status-packet field is reserved for a future verified SoC provider;
  // until then even an arbitrary byte is not a valid 0..1 measurement.
  EXPECT_TRUE(std::isnan(battery_percentage_from_firmware(UINT8_C(75))));
}

}  // namespace mowgli_hardware
