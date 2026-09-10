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

#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_behavior/status_snapshot.hpp"

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

  // Shared publisher owned by the context so the behavior_tree_node's periodic
  // timer can re-publish the last status while a long-running FollowStrip keeps
  // this SyncActionNode from ticking (see BTContext::last_high_level_status).
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    if (!ctx->high_level_status_pub)
    {
      ctx->high_level_status_pub =
          ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>(
              "~/high_level_status", 10);
    }
  }

  // Debounce transient IDLE. The requested state is recomputed from tree
  // traversal each tick; a single-tick reactive deselection of MowingSequence
  // (or a momentarily cleared current_command) makes the IdleSequence
  // fall-through request IDLE for one tick mid-mission. Only publish IDLE after
  // it has persisted for kIdleDebounceTicks ticks when coming FROM an active
  // state (AUTONOMOUS/RECORDING/MANUAL_MOWING). All other transitions —
  // including into motion states and into EMERGENCY/NULL — publish immediately.
  const uint8_t requested_state = state_res.value();
  uint8_t published_state = requested_state;
  const bool was_active =
      have_published_ && last_published_state_ != kStateIdle && last_published_state_ != kStateNull;
  if (requested_state == kStateIdle && was_active)
  {
    if (++pending_idle_ticks_ < kIdleDebounceTicks)
    {
      // Hold the previous active state until IDLE proves persistent.
      published_state = last_published_state_;
    }
    // else: IDLE has persisted long enough — accept it (published_state stays IDLE).
  }
  else
  {
    // Not a debounced IDLE transition — reset the counter and publish as-is.
    pending_idle_ticks_ = 0;
  }

  mowgli_interfaces::msg::HighLevelStatus identity;
  identity.state = published_state;
  identity.state_name = name_res.value();
  identity.sub_state_name = "";
  last_published_state_ = published_state;
  have_published_ = true;

  // Cache the message so the behavior_tree_node timer can re-publish it while
  // the tree is parked in a long-running action with no further transitions.
  // Only the state identity above is authored here; every live field is filled
  // by the same projection the republish timer uses, so the two paths cannot
  // drift apart (see status_snapshot.hpp).
  mowgli_interfaces::msg::HighLevelStatus msg;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    msg = withLiveStatusFields(identity, *ctx);
    ctx->last_high_level_status = msg;
    ctx->has_high_level_status = true;
    ctx->high_level_status_pub->publish(msg);
  }

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
  // Start-pose-blocked bookkeeping (issue #487) is per-session too: the next
  // COMMAND_START must get a fresh exemption budget, and a stale
  // start_blocked_area would make the first dispatch of the new session skip
  // the no-progress counter for a pass that never happened.
  ctx->coverage_start_blocked = false;
  ctx->start_blocked_area.reset();
  ctx->area_start_blocked_count.clear();
  // SAFETY (issue #487 escape motion): disarm the escape and forget the
  // last-motion direction at the session boundary. A token or a direction that
  // survived into the next session would describe a pose the robot may no
  // longer be standing in — the escape must re-derive both from the new
  // session's own evidence or stand down.
  ctx->start_blocked_escape_armed = false;
  ctx->last_motion_valid = false;
  ctx->last_motion_cmd_vx = 0.0;
  // Swath-completion model (replaces the cell coverage grid): clear the
  // per-area completed-swath sets, swath counts, and the completed-area set so
  // the next COMMAND_START re-plans and re-mows every area from swath 0.
  // Leaking these across sessions would make the next session skip every
  // already-mowed swath (the grid used to "decay"; the swath model resets at
  // the session boundary instead).
  ctx->area_completed_swaths.clear();
  ctx->area_swath_count.clear();
  ctx->area_resume_pose_index.clear();
  ctx->area_path_pose_count.clear();
  ctx->area_plan_fingerprint.clear();
  ctx->completed_areas.clear();
  // Drop any "mow only area N" constraint from a targeted run (~/start_in_area)
  // at the same boundary as every other per-session set, so the next plain
  // COMMAND_START mows the whole lawn again. The clip is session state (it must
  // survive GetNextUnmowedArea re-entering after the targeted area finishes, or
  // the run rolls over into the next area), so THIS is where it dies.
  clearSingleAreaMode(*ctx);
  // Remove the on-disk resume snapshot too: this is a real session boundary, so
  // the next COMMAND_START must start fresh rather than resume a finished (or
  // aborted-and-docked) session from the persisted cursor.
  clearCoverageResumeState(*ctx);
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
