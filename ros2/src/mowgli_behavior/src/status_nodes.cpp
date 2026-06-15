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

#include "mowgli_behavior/status_nodes.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// PublishHighLevelStatus
// ---------------------------------------------------------------------------

BT::NodeStatus PublishHighLevelStatus::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto state_res = getInput<uint8_t>("state");
  if (!state_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PublishHighLevelStatus: missing required port 'state': %s",
                 state_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto name_res = getInput<std::string>("state_name");
  if (!name_res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "PublishHighLevelStatus: missing required port 'state_name': %s",
                 name_res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!pub_)
  {
    pub_ =
        ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>("~/high_level_status",
                                                                             10);
  }

  mowgli_interfaces::msg::HighLevelStatus msg;
  msg.state = state_res.value();
  msg.state_name = name_res.value();
  msg.sub_state_name = "";
  msg.current_area = static_cast<int16_t>(ctx->current_area);
  msg.current_path = -1;
  msg.current_path_index = static_cast<int16_t>(ctx->coverage_percent);
  msg.total_swaths = static_cast<int16_t>(ctx->total_swaths);
  msg.completed_swaths = static_cast<int16_t>(ctx->completed_swaths);
  msg.skipped_swaths = static_cast<int16_t>(ctx->skipped_swaths);
  msg.gps_quality_percent = ctx->gps_quality;
  msg.battery_percent = ctx->battery_percent;
  msg.is_charging = ctx->latest_power.charger_enabled;
  msg.emergency = ctx->latest_emergency.active_emergency;

  pub_->publish(msg);

  RCLCPP_DEBUG(ctx->node->get_logger(),
               "PublishHighLevelStatus: state=%u name='%s'",
               msg.state,
               msg.state_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WasRainingAtStart
// ---------------------------------------------------------------------------

BT::NodeStatus WasRainingAtStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->raining_at_mow_start = ctx->latest_status.rain_detected;
  // Reset session-level counters at mowing start.
  ctx->resume_undock_failures = 0;
  RCLCPP_INFO(ctx->node->get_logger(),
              "WasRainingAtStart: rain_at_start=%s",
              ctx->raining_at_mow_start ? "true" : "false");
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

BT::NodeStatus ClearCommand::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  RCLCPP_INFO(ctx->node->get_logger(),
              "ClearCommand: resetting current_command from %u to 0",
              ctx->current_command);
  ctx->current_command = 0;
  // Note: session-scoped flags (yaw_seeded_this_session, skipped_swaths) are
  // intentionally NOT touched here. ClearCommand is invoked from mid-session
  // error handlers (UndockFailed, RainTimeout, ChargerFailed,
  // ResumeUndockOrAbort), and resetting yaw_seeded_this_session there caused
  // SeedYawFromMotion to re-drive 1 m forward on the next ReactiveSequence
  // re-tick of UndockOrSkip — even when the dock_yaw seed was already healthy.
  // Use EndSession at the real session boundaries instead.
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// EndSession
// ---------------------------------------------------------------------------

BT::NodeStatus EndSession::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  RCLCPP_INFO(ctx->node->get_logger(),
              "EndSession: clearing per-session flags "
              "(yaw_seeded=%s, skipped_swaths=%d, undock_recorded=%s, "
              "obstacle_backoffs=%d)",
              ctx->yaw_seeded_this_session ? "true" : "false",
              ctx->skipped_swaths,
              ctx->undock_start_recorded ? "true" : "false",
              ctx->obstacle_backoff_count);
  ctx->yaw_seeded_this_session = false;
  ctx->skipped_swaths = 0;
  ctx->undock_start_recorded = false;
  ctx->obstacle_backoff_count = 0;
  ctx->last_obstacle_backoff_time = std::chrono::steady_clock::time_point{};
  // Clear the per-session "already planned" set and attempt counters
  // so the next COMMAND_START can plan + mow each area afresh.
  ctx->attempted_areas.clear();
  ctx->area_attempt_count.clear();
  // Also clear the per-area coverage high-water mark (documented in
  // bt_context.hpp as "Cleared by EndSession"). Leaking it across sessions
  // makes the next session's first GetNextUnmowedArea dispatch compute
  // made_progress against last session's mark, so a freshly resumable area
  // (coverage reset / re-mow) is wrongly judged "no progress" and pushed
  // toward premature give-up at kMaxAreaAttempts.
  ctx->area_last_coverage.clear();
  // Swath-completion model (replaces the cell coverage grid): clear the
  // per-area completed-swath sets, swath counts, and the completed-area set so
  // the next COMMAND_START re-plans and re-mows every area from swath 0.
  // Leaking these across sessions would make the next session skip every
  // already-mowed swath (the grid used to "decay"; the swath model resets at
  // the session boundary instead).
  ctx->area_completed_swaths.clear();
  ctx->area_swath_count.clear();
  ctx->completed_areas.clear();
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// IncrementSkippedSwaths
// ---------------------------------------------------------------------------

BT::NodeStatus IncrementSkippedSwaths::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->skipped_swaths++;
  RCLCPP_WARN(ctx->node->get_logger(),
              "IncrementSkippedSwaths: skipped %d strips (unreachable)",
              ctx->skipped_swaths);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
