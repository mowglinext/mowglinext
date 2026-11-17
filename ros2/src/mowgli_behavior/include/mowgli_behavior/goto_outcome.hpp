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
/**
 * @file goto_outcome.hpp
 * @brief The values SetGotoOutcome records in BTContext::goto_outcome.
 */

#pragma once

#include <array>
#include <string_view>

namespace mowgli_behavior
{

inline constexpr std::string_view kGotoOutcomeReached = "REACHED";
inline constexpr std::string_view kGotoOutcomeFailed = "FAILED";
inline constexpr std::string_view kGotoOutcomeLowBatteryDocked = "LOW_BATTERY_DOCKED";
inline constexpr std::string_view kGotoOutcomeLowBatteryDockFailed = "LOW_BATTERY_DOCK_FAILED";
inline constexpr std::string_view kGotoOutcomeCriticalBatteryDocked = "CRITICAL_BATTERY_DOCKED";

inline constexpr std::array<std::string_view, 5> kGotoOutcomes = {
    kGotoOutcomeReached,
    kGotoOutcomeFailed,
    kGotoOutcomeLowBatteryDocked,
    kGotoOutcomeLowBatteryDockFailed,
    kGotoOutcomeCriticalBatteryDocked,
};

inline bool isGotoOutcome(std::string_view value)
{
  for (const auto outcome : kGotoOutcomes)
  {
    if (outcome == value)
    {
      return true;
    }
  }
  return false;
}

}  // namespace mowgli_behavior
