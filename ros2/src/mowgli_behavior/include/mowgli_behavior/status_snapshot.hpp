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

#ifndef MOWGLI_BEHAVIOR__STATUS_SNAPSHOT_HPP_
#define MOWGLI_BEHAVIOR__STATUS_SNAPSHOT_HPP_

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"

namespace mowgli_behavior
{

/// Return a copy of @p base with every context-derived field refreshed from
/// @p ctx, keeping the tree-owned state identity (state / state_name /
/// sub_state_name) exactly as cached.
///
/// HighLevelStatus mixes two kinds of field. The state identity is only
/// meaningful at a tree transition, so PublishHighLevelStatus (a SyncActionNode)
/// is its sole author. Everything else — battery, GPS quality, charging,
/// emergency, area + swath progress — is live sensor/plan state that the
/// subscriber callbacks and the coverage nodes keep current in the BTContext
/// between transitions.
///
/// behavior_tree_node's 1 Hz republish timer used to re-send the cached message
/// verbatim, which froze the live half for as long as the tree sat in a
/// transition-free branch. On the robot 2026-08-23 the low-battery charge hold
/// (a single "CHARGING" publish followed by a 30 s retry loop) pinned the GUI
/// battery gauge at the percent captured at dock contact for the entire charge;
/// a multi-minute FollowStrip froze the mowing progress the same way. Routing
/// both the transition publish and the periodic republish through this one
/// projection keeps the two paths from drifting apart again.
///
/// Pure: no locking, no ROS. The caller owns synchronisation — hold
/// BTContext::context_mutex across the call, since the fields read here are
/// written by subscriber callbacks on another thread.
mowgli_interfaces::msg::HighLevelStatus withLiveStatusFields(
    const mowgli_interfaces::msg::HighLevelStatus& base, const BTContext& ctx);

}  // namespace mowgli_behavior

#endif  // MOWGLI_BEHAVIOR__STATUS_SNAPSHOT_HPP_
