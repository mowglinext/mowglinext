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
#pragma once

#include <string>
#include <utility>

#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

/// Stable machine codes of HighLevelStatus.stop_reason. The GUI maps each to a localized sentence
/// (gui/web/src/utils/stopReason.ts); add a code in both places.
namespace stop_reason
{
/// An area was given up on after its passes were refused from the robot's own pose.
inline constexpr const char* kAreaStartBlocked = "AREA_START_BLOCKED";
/// An area was given up on after repeated passes that mowed nothing new.
inline constexpr const char* kAreaNoProgress = "AREA_NO_PROGRESS";
/// The drive to the dock failed (Nav2 could not reach the staging pose, or the dock approach
/// failed).
inline constexpr const char* kDockNavFailed = "DOCK_NAV_FAILED";
/// The undock manoeuvre failed.
inline constexpr const char* kUndockFailed = "UNDOCK_FAILED";
/// The robot docked but never started charging.
inline constexpr const char* kChargerFailed = "CHARGER_FAILED";
}  // namespace stop_reason

/// Remember why the mower stopped. `overwrite` false keeps an earlier reason of the same session
/// (the first area given up on stays the headline); true replaces it (a failed return to the dock
/// is what leaves the robot standing in the field, so it wins over the area that caused it) and
/// carries the replaced detail along, so the cause is not lost.
inline void recordStopReason(BTContext& ctx,
                             const std::string& reason,
                             const std::string& detail,
                             bool overwrite)
{
  if (reason.empty() || (!overwrite && !ctx.stop_reason.empty()))
  {
    return;
  }
  std::string combined = detail;
  if (!ctx.stop_detail.empty() && ctx.stop_reason != reason)
  {
    combined += (combined.empty() ? "" : " ") + std::string("Earlier: ") + ctx.stop_detail;
  }
  ctx.stop_reason = reason;
  ctx.stop_detail = std::move(combined);
}

inline void clearStopReason(BTContext& ctx)
{
  ctx.stop_reason.clear();
  ctx.stop_detail.clear();
}

}  // namespace mowgli_behavior
