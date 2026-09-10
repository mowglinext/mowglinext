// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0

#ifndef MOWGLI_BEHAVIOR__BATTERY_FILTER_HPP_
#define MOWGLI_BEHAVIOR__BATTERY_FILTER_HPP_

#include <optional>

namespace mowgli_behavior
{

// ── Why this exists ────────────────────────────────────────────────────────
// battery_percent is a linear interpolation of v_battery between the empty and
// full endpoints, and every percent-gated BT condition (NeedsDocking at
// battery_low_percent, IsBatteryLow at battery_critical_percent, PreFlightCheck)
// reads it. Feeding the raw rail voltage in means motor transients decide when
// the robot docks: PWM startup and stiction-overcome pull the pack down
// 0.5–1 V for well under a second. A run reported from the field ended in
// LOW_BATTERY_DOCKING while the GUI still showed ~35 %, because the sag briefly
// crossed the 20 % voltage — and the moment the BT reacted (SetMowerEnabled
// false / StopMoving) the load collapsed, the rail recovered to ~25.6 V, and
// the percent snapped back. The trip was numerically correct and operationally
// wrong: it acted on the under-load voltage, not on the state of charge.
//
// So low-pass the voltage before deriving the percent.
//
// ── Why the time constant, not a fixed alpha ───────────────────────────────
// A fixed EWMA weight silently encodes the sample rate, and this topic's rate
// is not ours to assume. /hardware_bridge/power is published once per firmware
// status packet (hardware_bridge_node::handle_status), and the firmware emits
// those every STATUS_NBT_TIME_MS = 250 ms — 4 Hz, not the 10 Hz the sim's
// fake_hardware_bridge_node runs at. One alpha would therefore mean two
// different filters on the robot and in sim, and would drift again the day
// anyone retunes STATUS_NBT_TIME_MS. Deriving alpha per sample from the
// elapsed time (alpha = dt / (tau + dt)) fixes the behaviour to TAU_S
// regardless of the rate, and makes the tuning knob the quantity you can
// actually reason about against a transient's duration.
constexpr double kBatteryFilterTauS = 2.0;

/// Samples at or below this are treated as no reading at all — a disconnected
/// pack or a glitched ADC, not a flat battery. A 24 V pack that has genuinely
/// collapsed this far is long past any threshold the BT reacts to.
constexpr float kBatteryMinValidVoltage = 10.0f;

/// First-order low-pass over v_battery with a rate-independent time constant.
///
/// Pure logic, no ROS, so it is unit-testable standalone (see
/// test_battery_filter.cpp) — same shape as localization_health.hpp.
class BatteryVoltageFilter
{
public:
  BatteryVoltageFilter() = default;

  /// @param tau_s      time constant in seconds; <= 0 disables smoothing
  ///                   (every valid sample passes through unchanged).
  /// @param min_valid_v samples below this are ignored as invalid.
  BatteryVoltageFilter(double tau_s, float min_valid_v);

  /// Fold one sample in.
  ///
  /// @param v_raw   the reported rail voltage.
  /// @param now_sec a monotonically increasing clock, in seconds. The node
  ///                passes its ROS clock (so sim time works) rather than the
  ///                message stamp, which keeps the filter correct even for a
  ///                publisher that does not stamp its messages.
  /// @return the filtered voltage, or nullopt while no valid sample has ever
  ///         arrived. Callers must not derive a percent from nullopt: writing
  ///         a 0 % derived from a glitch reading is exactly the false reading
  ///         this filter exists to prevent.
  std::optional<float> update(float v_raw, double now_sec);

  /// Last filtered value, or nullopt before the first valid sample.
  std::optional<float> value() const
  {
    return value_;
  }

  /// Drop the state — the next valid sample bootstraps the filter again.
  void reset();

private:
  double tau_s_{kBatteryFilterTauS};
  float min_valid_v_{kBatteryMinValidVoltage};
  std::optional<float> value_;
  double last_sec_{0.0};
};

/// Linear voltage → percent interpolation against the configured endpoints,
/// clamped to [0, 100]. Returns 0 for a degenerate (non-positive) range.
float batteryPercentFromVoltage(float voltage, float v_empty, float v_full);

}  // namespace mowgli_behavior

#endif  // MOWGLI_BEHAVIOR__BATTERY_FILTER_HPP_
