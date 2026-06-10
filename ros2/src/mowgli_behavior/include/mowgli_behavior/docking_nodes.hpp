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

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/undock_robot.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DockApproach
// ---------------------------------------------------------------------------

/// v2-style obstacle-aware dock approach that replaces opennav_docking's
/// graceful-controller arc (the oscillation source on this robot's
/// deadband-limited motors). It plans a single continuous path and follows it
/// with FTCController:
///
///   1. Smac (ComputePathToPose / `GridBased`) plans the robot's current pose
///      to a STAGING pose on the dock axis (`dock - lstage * û`, goal yaw =
///      dock yaw) — obstacle-aware, footprint-checked.
///   2. A straight dock-aligned TAIL is appended (staging → `dock + overshoot
///      * û`, every pose at dock yaw). The Smac/tail join is C¹-smooth because
///      Smac arrives at staging tangent to its goal yaw.
///   3. The concatenated path is handed to ONE FollowPath/FTC. The overshoot
///      keeps the robot moving as it noses into the cradle (open-loop motors
///      cannot break static friction from rest), so it seats with momentum;
///      FTC then ABORTs stalled against the dock — which, when the final pose
///      is within `arrival_tol` of the dock, is treated as SUCCESS. The BT's
///      IsCharging check confirms the electrical seat.
///
/// Dock pose comes from BTContext (dock_x/dock_y/dock_yaw, sourced from
/// mowgli_robot.yaml via the behavior node's dock_pose_x/y/yaw params). The
/// dock_id / dock_type ports are accepted but ignored, so this node is a
/// drop-in replacement for DockRobot in the tree.
class DockApproach : public BT::StatefulActionNode
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  DockApproach(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id",
                                       "home_dock",
                                       "Accepted for DockRobot compat (ignored)"),
            BT::InputPort<std::string>("dock_type",
                                       "simple_charging_dock",
                                       "Accepted for DockRobot compat (ignored)"),
            BT::InputPort<double>("lstage", 1.1, "Staging back-off along the dock axis (m)"),
            BT::InputPort<double>("overshoot",
                                  0.35,
                                  "Tail extension past the dock pose for seating momentum (m)"),
            BT::InputPort<double>("step", 0.05, "Straight-tail densification step (m)"),
            BT::InputPort<double>(
                "arrival_tol",
                0.30,
                "Max final-pose distance to dock for an FTC abort to count as seated (m)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    Planning,  // ComputePathToPose (Smac) → staging
    Following,  // FollowPath over the concatenated path
  };

  // Build the straight dock-aligned tail (staging → dock + overshoot*û), every
  // pose at dock yaw, densified to `step`, and append it to the Smac plan.
  nav_msgs::msg::Path buildConcatPath(const nav_msgs::msg::Path& smac_plan) const;

  // True when the robot's current map→base_footprint pose is within
  // `arrival_tol_` of the dock — used to accept an FTC abort as a seat.
  bool nearDock(const std::shared_ptr<BTContext>& ctx) const;

  rclcpp_action::Client<ComputePath>::SharedPtr plan_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  std::shared_future<ComputePathGoalHandle::SharedPtr> plan_goal_future_;
  ComputePathGoalHandle::SharedPtr plan_goal_handle_;
  std::shared_future<ComputePathGoalHandle::WrappedResult> plan_result_future_;
  bool plan_result_requested_{false};
  std::shared_future<FollowPathGoalHandle::SharedPtr> follow_goal_future_;
  FollowPathGoalHandle::SharedPtr follow_goal_handle_;

  Phase phase_{Phase::Planning};
  double dock_x_{0.0};
  double dock_y_{0.0};
  double dock_yaw_{0.0};
  double lstage_{1.1};
  double overshoot_{0.35};
  double step_{0.05};
  double arrival_tol_{0.30};
};

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
  rclcpp_action::Client<DockAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
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
