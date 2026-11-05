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

#include <cstdint>

#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

/// True for the operator commands that take the mower out of battery recovery:
/// HOME (2), RECORD_AREA / RECORD_FINISH / RECORD_CANCEL (3 / 5 / 6),
/// MANUAL_MOW (7) and STOP (8). START (1, and S2 normalised to it) is the one
/// command that keeps a pending charge: it asks to continue the mow, which must
/// still charge first.
inline bool commandReleasesBatteryCharge(uint8_t cmd)
{
  return cmd == 2 || cmd == 3 || cmd == 5 || cmd == 6 || cmd == 7 || cmd == 8;
}

/// Apply an accepted operator command to the battery-recovery latches.
/// Caller holds ctx.context_mutex. Returns true when persisted state changed.
///
/// - Any command except STOP clears the critical dock failure latch and the
///   critical charge STOP hold (a retry request, as before).
/// - STOP never re-arms a dock attempt. After a failed critical dock it turns
///   the failure latch into the critical charge STOP hold, which takes the
///   critical branch out of MainLogic (CriticalChargeStopNotLatched), so the
///   tree reaches StopHoldSequence instead of sitting in NAV_FAILED, even with
///   a still-critical battery. During the critical charge hold it latches the
///   same hold, as before.
/// - HOME / RECORD / MANUAL / STOP drop battery_charge_pending, so a pending
///   charge can no longer re-arm DockRobot behind the operator's back. A battery
///   that is still critical re-enters through the live IsBatteryLow check.
inline bool applyCommandToBatteryRecovery(BTContext& ctx, uint8_t cmd)
{
  constexpr uint8_t kCommandStop = 8;
  bool changed = false;
  if (cmd == kCommandStop)
  {
    if (ctx.critical_dock_failure_latched)
    {
      ctx.critical_dock_failure_latched = false;
      ctx.critical_charge_stop_latched = true;
      changed = true;
    }
    else if (ctx.last_high_level_status.state_name == "CRITICAL_BATTERY_CHARGING")
    {
      // Capture cancellation synchronously as well as in the tree condition,
      // so the persisted snapshot already represents this post-dock hold if
      // the process stops before its next tick.
      changed = changed || !ctx.critical_charge_stop_latched;
      ctx.critical_charge_stop_latched = true;
    }
  }
  else
  {
    changed = ctx.critical_dock_failure_latched || ctx.critical_charge_stop_latched;
    ctx.critical_charge_stop_latched = false;
    ctx.critical_dock_failure_latched = false;
  }
  if (commandReleasesBatteryCharge(cmd) && ctx.battery_charge_pending)
  {
    ctx.battery_charge_pending = false;
    changed = true;
  }
  return changed;
}

}  // namespace mowgli_behavior
