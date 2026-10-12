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
//
// How IsChargingProgressing tells a dead charger from a healthy one, as a pure
// function so the 30-minute window can be tested without waiting for it.
//
// The rule is "battery_percent must rise by min_increase within each window",
// and battery_percent is derived from raw pack VOLTAGE. That rule cannot hold
// on a SATURATED pack: in the charger's CV phase the voltage is held at the
// charge setpoint, so battery_percent sits flat at its ceiling while the charge
// current is still tapering. Before #765 the charge loops left at
// battery_full_pct, so the rule never met a saturated pack. #765 made them also
// wait for the current taper (IsChargeCurrentBelow) — i.e. for exactly the
// stretch where the percentage is flat — and a CV tail longer than one window
// would then be judged a dead charger: CHARGER_FAILED, and the session ends
// with the robot on the dock instead of resuming the mow.
//
// So a pack at or above full_pct is kSaturated, not stalled. Below it the rule
// is unchanged, which is where a dead charger actually shows (the pack never
// reaches full). A taper that never completes is still bounded by the charge
// loop's RetryUntilSuccessful (8 h) in main_tree.xml.

#pragma once

namespace mowgli_behavior
{

/// full_pct value that disables the saturation exemption: no battery_percent
/// reaches it, so the classic rule applies everywhere (the behaviour of a tree
/// that does not pass full_pct).
constexpr float kNoChargeSaturationPct = 1000.0f;

enum class ChargeProgress
{
  kWithinWindow,  ///< window not elapsed yet — assume charging is fine
  kProgressing,  ///< rose by at least min_increase this window
  kSaturated,  ///< at or above full_pct: flat by construction, not a stall
  kStalled,  ///< a whole window below full with no meaningful rise
};

/// @param baseline_pct      battery_percent at the start of the window
/// @param current_pct       battery_percent now
/// @param elapsed_s         seconds since the window started
/// @param window_s          window length (IsChargingProgressing: 30 min)
/// @param min_increase_pct  rise required per window (IsChargingProgressing: 1 %)
/// @param full_pct          resume level; kNoChargeSaturationPct disables the exemption
inline ChargeProgress judgeChargeProgress(float baseline_pct,
                                          float current_pct,
                                          double elapsed_s,
                                          double window_s,
                                          float min_increase_pct,
                                          float full_pct)
{
  if (current_pct >= full_pct)
  {
    return ChargeProgress::kSaturated;
  }
  if (elapsed_s < window_s)
  {
    return ChargeProgress::kWithinWindow;
  }
  if (current_pct - baseline_pct >= min_increase_pct)
  {
    return ChargeProgress::kProgressing;
  }
  return ChargeProgress::kStalled;
}

}  // namespace mowgli_behavior
