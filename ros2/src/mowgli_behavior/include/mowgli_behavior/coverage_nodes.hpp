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

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_coverage_status.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "mowgli_interfaces/srv/paint_swath.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// ComputeCoveragePath — call opennav_coverage's /compute_coverage_path action
// for a single mowing area. Result populates ctx->current_coverage_path with
// the full nav_msgs/Path (swaths + headland + transitions) ready for MPPI.
// ---------------------------------------------------------------------------

class ComputeCoveragePath : public BT::StatefulActionNode
{
public:
  using CoverageAction = opennav_coverage_msgs::action::ComputeCoveragePath;
  using CoverageGoalHandle = rclcpp_action::ClientGoalHandle<CoverageAction>;

  ComputeCoveragePath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<CoverageAction>::SharedPtr action_client_;
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr area_client_;
  std::shared_future<CoverageGoalHandle::SharedPtr> goal_future_;
  CoverageGoalHandle::SharedPtr goal_handle_;
  std::shared_future<CoverageGoalHandle::WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// FollowCoveragePath — drive ctx->current_coverage_path with MPPI (single
// FollowPath goal, blade ON for the whole path; MPPI deviates around dynamic
// obstacles natively via ObstaclesCritic). On SUCCESS, paints mow_progress
// along the *actual* driven track (sampled by TF lookup at every tick),
// not the planned path — otherwise an MPPI shortcut from start to a point
// near the goal-tolerance fires the goal_checker and we'd report 100%
// coverage despite the robot never having traversed the strips.
// ---------------------------------------------------------------------------

class FollowCoveragePath : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;

  FollowCoveragePath(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);
  void paintTrajectory(const nav_msgs::msg::Path& trajectory);
  void recordPose();

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  rclcpp::Client<mowgli_interfaces::srv::PaintSwath>::SharedPtr paint_client_;

  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  // Driven-track accumulator: every tick we look up the latest
  // map→base_footprint TF and append the robot's pose if it has moved
  // > min_pose_step_m_ from the last sample. This is what gets painted
  // into mow_progress on SUCCESS, instead of the planned path.
  nav_msgs::msg::Path driven_trajectory_;
  static constexpr double min_pose_step_m_ = 0.05;

  bool goal_sent_{false};
  bool blade_on_{false};
};

// ---------------------------------------------------------------------------
// GetNextUnmowedArea — find next area with coverage_percent < 99 %.
// `strips_remaining` in GetCoverageStatus is now a coverage-threshold shim
// (1 if work remains, 0 otherwise); existing semantics preserved.
// ---------------------------------------------------------------------------

class GetNextUnmowedArea : public BT::StatefulActionNode
{
public:
  GetNextUnmowedArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("max_areas", 20u, "Maximum number of areas to check"),
        BT::OutputPort<uint32_t>("area_index", "Index of the next unmowed area"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Process a completed service response. Returns SUCCESS if an unmowed area
  /// was found, FAILURE if all areas are done / no areas defined, or RUNNING
  /// if more areas need to be checked (launches next async call internally).
  BT::NodeStatus processResponse();

  rclcpp::Client<mowgli_interfaces::srv::GetCoverageStatus>::SharedPtr client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetCoverageStatus>::FutureAndRequestId>
      pending_future_;
  std::chrono::steady_clock::time_point call_start_;
  uint32_t current_area_idx_{0};
  uint32_t max_areas_{20};
  uint32_t areas_queried_{0};
  uint32_t areas_complete_{0};
};

}  // namespace mowgli_behavior
