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

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /dock_robot action to dock the robot.
///
/// Input ports:
///   dock_id   (string) – named dock instance (e.g. "home_dock")
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class DockRobot : public BT::StatefulActionNode
{
public:
  using DockAction = nav2_msgs::action::DockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<DockAction>;

  DockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id", "home_dock", "Named dock instance"),
            BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Logs where the robot actually came to rest, relative to the dock pose, at
  /// the moment opennav_docking claims contact (feedback state WAIT_FOR_CHARGE).
  ///
  /// Issue #486: the server logs "Made contact with dock" from
  /// SimpleChargingDock::isDocked(), which is a pure POSITION test against
  /// docking_threshold and ignores yaw entirely. A seat that is laterally
  /// offset but geometrically near therefore reports contact identically to a
  /// good one, and the only downstream evidence is v_charge staying 0.0.
  /// Reconstructing the 2026-08-24 failure took an encoder-tick reconstruction
  /// across two log files purely because this delta was never recorded; split
  /// into along-track / cross-track it distinguishes "stopped short" from
  /// "seated off to the side" in one line.
  ///
  /// Diagnostic only — nothing reads the result.
  void log_contact_delta(const std::shared_ptr<BTContext>& ctx, uint16_t num_retries) const;

  rclcpp_action::Client<DockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;

  /// Last docking feedback state seen, so the contact delta is logged once per
  /// entry into WAIT_FOR_CHARGE rather than at the feedback rate. Written from
  /// the action feedback callback, read from the BT tick thread.
  std::atomic<uint16_t> last_feedback_state_{DockAction::Feedback::NONE};
};

// ---------------------------------------------------------------------------
// UndockRobot
// ---------------------------------------------------------------------------

/// Calls the opennav_docking /undock_robot action to undock the robot.
///
/// Input ports:
///   dock_type (string) – dock plugin type (e.g. "simple_charging_dock")
class UndockRobot : public BT::StatefulActionNode
{
public:
  using UndockAction = nav2_msgs::action::UndockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<UndockAction>;

  UndockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_type", "simple_charging_dock", "Dock plugin type")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<UndockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
};

// ---------------------------------------------------------------------------
// RecordResumeUndockFailure
// ---------------------------------------------------------------------------

/// Increments the resume_undock_failures counter in BTContext.
/// Always returns SUCCESS so it can be placed inside any sequence.
class RecordResumeUndockFailure : public BT::SyncActionNode
{
public:
  RecordResumeUndockFailure(const std::string& name, const BT::NodeConfig& config)
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
