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

#pragma once

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// PublishHighLevelStatus
// ---------------------------------------------------------------------------

/// Publishes a HighLevelStatus message to ~/high_level_status.
///
/// Input ports:
///   state      (uint8_t) – HIGH_LEVEL_STATE_* constant.
///   state_name (string)  – Human-readable sub_state_name string.
class PublishHighLevelStatus : public BT::SyncActionNode
{
public:
  PublishHighLevelStatus(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint8_t>("state", "HIGH_LEVEL_STATE_* value"),
            BT::InputPort<std::string>("state_name", "Human-readable state label")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
// WasRainingAtStart
// ---------------------------------------------------------------------------

/// Records whether it's currently raining into ctx->raining_at_mow_start.
/// Always returns SUCCESS. Called once at the start of a mowing session.
class WasRainingAtStart : public BT::SyncActionNode
{
public:
  WasRainingAtStart(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// ClearCommand
// ---------------------------------------------------------------------------

/// Resets the current_command in BTContext to 0 (no command), preventing the
/// BT from re-entering a completed sequence. Safe to call from mid-session
/// error handlers — does NOT touch session-scoped state.
class ClearCommand : public BT::SyncActionNode
{
public:
  ClearCommand(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// EndSession
// ---------------------------------------------------------------------------

/// Resets per-session BTContext flags so the next session starts with a clean
/// slate — yaw-seed flag, undock bookkeeping, skipped-swath counter. Call only
/// at confirmed session boundaries (IDLE_DOCKED after MOWING_COMPLETE,
/// RECORDING_COMPLETE, etc.). Mid-session error handlers must NOT call this:
/// resetting yaw_seeded_this_session there causes SeedYawFromMotion to re-fire
/// the 1 m forward drive on the next ReactiveSequence re-tick of UndockOrSkip,
/// even though the dock_yaw seed is already healthy.
class EndSession : public BT::SyncActionNode
{
public:
  EndSession(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

class IncrementSkippedSwaths : public BT::SyncActionNode
{
public:
  IncrementSkippedSwaths(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior
