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
 * @file test_high_level_status_snapshot.cpp
 * @brief Regression test for the frozen GUI battery gauge while charging.
 *
 * PublishHighLevelStatus is a SyncActionNode that only fires on tree
 * transitions, and behavior_tree_node's 1 Hz timer used to re-publish the
 * cached message VERBATIM. Once the tree parked in the low-battery charge hold
 * (BatteryDockAndResume: PublishHighLevelStatus "CHARGING" once, then a 30 s
 * RetryUntilSuccessful loop with no further transitions), the topic kept
 * emitting the battery percent captured at the moment of docking — observed on
 * the robot 2026-08-23 frozen at 46.03 % for the whole charge while the pack
 * actually climbed from 25.84 V to 26.42 V. The same staleness hit the mowing
 * progress during a multi-minute FollowStrip.
 *
 * withLiveStatusFields refreshes every context-derived field from the live
 * BTContext while keeping the tree-owned state identity (state / state_name /
 * sub_state_name) from the cached snapshot. These tests pin that split with a
 * plain BTContext (no ROS, no publisher).
 */

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/status_snapshot.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::withLiveStatusFields;
using mowgli_interfaces::msg::HighLevelStatus;

namespace
{

/// The status cached when the tree published "CHARGING" at dock contact.
HighLevelStatus chargingSnapshot()
{
  HighLevelStatus cached;
  cached.state = HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
  cached.state_name = "CHARGING";
  cached.sub_state_name = "";
  cached.battery_percent = 46.03f;
  cached.gps_quality_percent = 1.0f;
  cached.is_charging = true;
  cached.coverage_percent = 80.57f;
  return cached;
}

}  // namespace

// The exact field the operator watches: the pack charges, so the re-published
// percent must track the live context, not the value latched at dock contact.
TEST(HighLevelStatusSnapshot, BatteryPercentTracksLiveContext)
{
  BTContext ctx;
  ctx.battery_percent = 60.4f;  // pack has climbed since docking

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_FLOAT_EQ(refreshed.battery_percent, 60.4f);
}

// The state identity is owned by the tree and is only valid at a transition, so
// the republish must carry it through from the cache untouched. Refreshing it
// from a default-constructed context would blank the GUI's state label.
TEST(HighLevelStatusSnapshot, StateIdentityIsCarriedFromCache)
{
  BTContext ctx;

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_EQ(refreshed.state, HighLevelStatus::HIGH_LEVEL_STATE_IDLE);
  EXPECT_EQ(refreshed.state_name, "CHARGING");
  EXPECT_EQ(refreshed.sub_state_name, "");
}

// Charger unplugged / emergency asserted while the tree sits in the charge hold:
// both are live sensor state and must reach the GUI without a tree transition.
TEST(HighLevelStatusSnapshot, ChargingAndEmergencyTrackLiveContext)
{
  BTContext ctx;
  ctx.latest_power.charger_enabled = false;
  ctx.latest_emergency.active_emergency = true;

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_FALSE(refreshed.is_charging);
  EXPECT_TRUE(refreshed.emergency);
}

// The other half of the same bug: during a multi-minute FollowStrip the tree
// never re-ticks PublishHighLevelStatus, so coverage progress and GPS quality
// were frozen too.
TEST(HighLevelStatusSnapshot, ProgressAndGpsTrackLiveContext)
{
  BTContext ctx;
  ctx.coverage_percent = 92.5f;
  ctx.gps_quality = 100.0f;
  ctx.current_area = 3;
  ctx.total_swaths = 8;
  ctx.completed_swaths = 6;
  ctx.skipped_swaths = 1;

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_FLOAT_EQ(refreshed.coverage_percent, 92.5f);
  EXPECT_FLOAT_EQ(refreshed.gps_quality_percent, 100.0f);
  EXPECT_EQ(refreshed.current_area, 3);
  EXPECT_EQ(refreshed.total_swaths, 8);
  EXPECT_EQ(refreshed.completed_swaths, 6);
  EXPECT_EQ(refreshed.skipped_swaths, 1);
}

// The GUI divides current_path_index by current_path to render the mowing
// percentage (see the PublishHighLevelStatus comment), so the republish must
// keep mirroring the swath counts into that pair rather than leaving the
// denominator at its cached value.
TEST(HighLevelStatusSnapshot, GuiRatioPairMirrorsSwathCounts)
{
  BTContext ctx;
  ctx.total_swaths = 8;
  ctx.completed_swaths = 6;

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_EQ(refreshed.current_path, 8);
  EXPECT_EQ(refreshed.current_path_index, 6);
}

// A default-constructed context must not fabricate progress: the helper is a
// pure projection of the context, so an untouched context yields zeros (and the
// 100 % battery default), never leftovers from the cached snapshot.
TEST(HighLevelStatusSnapshot, LiveFieldsNeverFallBackToCachedValues)
{
  BTContext ctx;

  const HighLevelStatus refreshed = withLiveStatusFields(chargingSnapshot(), ctx);

  EXPECT_FLOAT_EQ(refreshed.coverage_percent, 0.0f);
  EXPECT_FLOAT_EQ(refreshed.gps_quality_percent, 0.0f);
  EXPECT_FLOAT_EQ(refreshed.battery_percent, ctx.battery_percent);
  EXPECT_FALSE(refreshed.is_charging);
}
