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

#include "mowgli_behavior/status_snapshot.hpp"

#include <cstdint>

namespace mowgli_behavior
{

mowgli_interfaces::msg::HighLevelStatus withLiveStatusFields(
    const mowgli_interfaces::msg::HighLevelStatus& base, const BTContext& ctx)
{
  mowgli_interfaces::msg::HighLevelStatus msg;

  // Tree-owned identity: only PublishHighLevelStatus knows which branch is
  // selected, so carry it through untouched before applying live overlays.
  msg.state = base.state;
  msg.state_name = base.state_name;
  msg.sub_state_name = base.sub_state_name;
  // EXCEPTION: sub_state_name also carries two live overrides that change
  // mid-invocation, between tree ticks, so only this per-tick/1 Hz-republish
  // projection can track them accurately. Overridden, not appended: nothing
  // else currently populates this field (PublishHighLevelStatus always
  // writes ""). TRANSIT wins if both happen to be true on the same tick (a
  // later area still mowing after an earlier one was flagged) — cosmetic
  // only, the warning reappears the next non-transiting tick.
  if (ctx.transiting)
  {
    msg.sub_state_name = "TRANSIT";
  }
  else if (ctx.coverage_plausibility_warning)
  {
    // Issue #680: FollowStrip::checkCoveragePlausibility (coverage_nodes.cpp)
    // can flip this mid-session; it must stay visible through to the final
    // report rather than only flash at the instant it was set.
    msg.sub_state_name = "COVERAGE_INCOMPLETE";
  }

  // A scan pause owns the blade. It therefore takes priority over a transit
  // snapshot: on the completion tick transit_active_ is cleared only after
  // onRunning() has sampled it, while sendFollowGoal() can synchronously find
  // a stale scan. Never let that one-tick stale TRANSIT snapshot hide the
  // active safety hold from the operator.
  if (ctx.coverage_scan_paused &&
      base.state == mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS &&
      base.state_name == "MOWING")
  {
    msg.sub_state_name = "SCAN_PAUSED";
  }

  msg.current_area = static_cast<int16_t>(ctx.current_area);
  // The GUI computes progress as current_path_index / current_path
  // (MowerStatus.tsx, MowgliNextPage.tsx), so current_path is the DENOMINATOR
  // (total swaths in the current area's plan) and current_path_index the
  // NUMERATOR (completed swaths). Both are tracked by PlanCoverageArea /
  // FollowStrip in coverage_nodes.
  msg.current_path = static_cast<int16_t>(ctx.total_swaths);
  msg.current_path_index = static_cast<int16_t>(ctx.completed_swaths);
  msg.total_swaths = static_cast<int16_t>(ctx.total_swaths);
  msg.completed_swaths = static_cast<int16_t>(ctx.completed_swaths);
  msg.skipped_swaths = static_cast<int16_t>(ctx.skipped_swaths);
  // Smooth pose-cursor-based progress for the current area (primary GUI %); the
  // swath counts above are the coarse secondary "sub-path X/Y" readout.
  msg.coverage_percent = ctx.coverage_percent;
  msg.gps_quality_percent = ctx.gps_quality;
  msg.battery_percent = ctx.battery_percent;
  msg.is_charging = ctx.latest_power.charger_enabled;
  msg.emergency = ctx.latest_emergency.active_emergency;

  return msg;
}

}  // namespace mowgli_behavior
