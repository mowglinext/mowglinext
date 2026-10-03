// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
#ifndef MOWGLI_MONITORING__BATTERY_PERCENTAGE_HPP_
#define MOWGLI_MONITORING__BATTERY_PERCENTAGE_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_monitoring
{

/// Voltage-derived estimate using the same configured pack endpoints as the BT.
inline double battery_percentage(double voltage, double empty_voltage, double full_voltage)
{
  const double range = full_voltage - empty_voltage;
  if (!std::isfinite(voltage) || !std::isfinite(range) || range <= 0.01)
  {
    return 0.0;
  }
  return 100.0 * (std::clamp(voltage, empty_voltage, full_voltage) - empty_voltage) / range;
}

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__BATTERY_PERCENTAGE_HPP_
