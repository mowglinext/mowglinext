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
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_coverage_status.hpp"
#include "mowgli_interfaces/srv/get_next_segment.hpp"
#include "mowgli_interfaces/srv/get_next_strip.hpp"
#include "mowgli_interfaces/srv/get_remaining_area_polygon.hpp"
#include "mowgli_interfaces/srv/mark_segment_blocked.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// GetNextStrip — fetch next unmowed strip from map_server
// ---------------------------------------------------------------------------

class GetNextStrip : public BT::StatefulActionNode
{
public:
  GetNextStrip(const std::string& name, const BT::NodeConfig& config)
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
  rclcpp::Client<mowgli_interfaces::srv::GetNextStrip>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// FollowStrip — follow a strip path with FTCController, blade ON
// ---------------------------------------------------------------------------

class FollowStrip : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;

  FollowStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  // Blade spinup delay — wait before sending path goal
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  bool goal_sent_ = false;
};

// ---------------------------------------------------------------------------
// GetNextSegment — Path C: ask map_server for the next short dynamic
// segment from the robot's pose, ending at the first obstacle / dead
// cell / boundary / max_length. Replaces GetNextStrip in new BT trees;
// kept side-by-side during migration.
// ---------------------------------------------------------------------------

class GetNextSegment : public BT::StatefulActionNode
{
public:
  GetNextSegment(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index"),
        // Coverage row direction. NaN → server auto-computes from
        // polygon MBR (same as the strip planner does today). Operator
        // override goes through the GUI's mow_angle_offset_deg.
        BT::InputPort<double>("prefer_dir_yaw_rad",
                              std::numeric_limits<double>::quiet_NaN(),
                              "Preferred mowing row direction (rad). NaN = auto from polygon."),
        BT::InputPort<bool>("boustrophedon", true, "Alternate row direction for snake pattern"),
        BT::InputPort<double>("max_segment_length_m",
                              0.0,
                              "Max segment length (m). 0 = server default (3.0)."),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::GetNextSegment>::SharedPtr client_;
  rclcpp::Client<mowgli_interfaces::srv::GetCoverageStatus>::SharedPtr coverage_status_client_;
};

// ---------------------------------------------------------------------------
// IsShortSegment — condition: returns SUCCESS when the current segment
// can be mowed with the blade on (no transit). Reads
// ctx->current_segment_is_long_transit set by GetNextSegment.
// ---------------------------------------------------------------------------

class IsShortSegment : public BT::ConditionNode
{
public:
  IsShortSegment(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// MarkSegmentBlocked — call map_server's ~/mark_segment_blocked with
// the current segment path so cells along the failed approach get a
// fail_count tick. After N consecutive failures cells get promoted to
// LAWN_DEAD and skipped permanently (until decay or manual clear).
// Should be ticked when FollowSegment / FollowStrip returns FAILURE
// because of a non-trivial blocker. The node itself returns SUCCESS
// so the surrounding RetryUntilSuccessful resets cleanly and the
// next tick can pick a fresh segment.
// ---------------------------------------------------------------------------

class MarkSegmentBlocked : public BT::StatefulActionNode
{
public:
  MarkSegmentBlocked(const std::string& name, const BT::NodeConfig& config)
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
  rclcpp::Client<mowgli_interfaces::srv::MarkSegmentBlocked>::SharedPtr client_;
  std::shared_future<mowgli_interfaces::srv::MarkSegmentBlocked::Response::SharedPtr> future_;
};

// ---------------------------------------------------------------------------
// TransitToStrip — navigate to strip start using Nav2 navigate_to_pose
// ---------------------------------------------------------------------------

class TransitToStrip : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  TransitToStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

// ---------------------------------------------------------------------------
// DetourAroundObstacle — when FollowStrip aborts on a lookahead-collision,
// drive a short side-step path through the global planner so the robot
// physically gets out from in front of the obstacle (a person standing in
// the strip). The next strip iteration replans from the new pose; the
// `mow_progress` layer prevents re-cutting already-mowed cells.
//
// The detour is a NavigateToPose at (current_pose ⊕ forward·x̂_body
// + lateral·ŷ_body), routed via SmacPlanner over the local costmap which
// has the obstacle layer enabled — so the planner naturally curves around
// the obstruction rather than charging through it.
// ---------------------------------------------------------------------------

class DetourAroundObstacle : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  DetourAroundObstacle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("forward_m", 0.8, "Forward offset from current pose, body frame"),
        BT::InputPort<double>("lateral_m", 0.6, "Lateral offset (positive = left), body frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

// ---------------------------------------------------------------------------
// GetNextUnmowedArea — find next area with remaining strips
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

// ---------------------------------------------------------------------------
// PlanCoverageArea — opennav_coverage migration. Calls map_server's
// ~/get_remaining_area_polygon to get the unmowed portion of an area as a
// list of MapArea pieces (outer + holes), then asks opennav_coverage's
// /compute_coverage_path action to build a full F2C path (headland +
// boustrophedon swaths + Dubin turns) for each piece, and concatenates
// the resulting nav_msgs/Path segments into ctx->current_strip_path. The
// existing FollowStrip node then feeds that path to FTCController.
//
// Replaces the GetNextSegment/GetNextStrip per-strip loop. Plans the
// whole area in one shot at each (re)start; on resume the
// "remaining area" call already excludes the already-mowed cells.
// ---------------------------------------------------------------------------

class PlanCoverageArea : public BT::StatefulActionNode
{
public:
  using ComputeCoveragePath = opennav_coverage_msgs::action::ComputeCoveragePath;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputeCoveragePath>;

  PlanCoverageArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index"),
        BT::InputPort<double>("operation_width_m", 0.20,
                              "F2C operation width (mower cut width, m)"),
        BT::InputPort<double>("headland_width_m", 0.20,
                              "F2C constant-width headland (m)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    QueryRemaining,
    PlanNextPiece,
    WaitingForGoal,
    WaitingForResult,
    Done,
  };

  /// Build a ComputeCoveragePath::Goal from one MapArea piece. Polygons
  /// list is [outer, hole0, hole1, ...]. Modes: CONSTANT headland,
  /// BRUTE_FORCE/LENGTH swath, BOUSTROPHEDON route, DUBIN/DISCONTINUOUS path.
  ComputeCoveragePath::Goal buildGoal(
      const mowgli_interfaces::msg::MapArea& piece,
      double operation_width,
      double headland_width) const;

  rclcpp::Client<mowgli_interfaces::srv::GetRemainingAreaPolygon>::SharedPtr srv_client_;
  std::optional<rclcpp::Client<
      mowgli_interfaces::srv::GetRemainingAreaPolygon>::FutureAndRequestId> srv_future_;

  rclcpp_action::Client<ComputeCoveragePath>::SharedPtr action_client_;
  std::shared_future<ComputeGoalHandle::SharedPtr> goal_future_;
  ComputeGoalHandle::SharedPtr goal_handle_;
  std::shared_future<ComputeGoalHandle::WrappedResult> result_future_;

  std::vector<mowgli_interfaces::msg::MapArea> pieces_;
  size_t current_piece_{0};
  size_t pieces_succeeded_{0};
  size_t pieces_failed_{0};
  nav_msgs::msg::Path accumulated_path_;
  /// First pose of the FIRST piece's F2C swath path (the real mowing start,
  /// AFTER the robot→F2C straight transit prefix). Used as the obstacle-aware
  /// TransitToStrip goal so the robot drives to the coverage start AROUND
  /// obstacles instead of FTC dragging it there along the costmap-blind
  /// straight prefix.
  geometry_msgs::msg::PoseStamped first_coverage_pose_;
  bool have_first_coverage_pose_{false};
  Phase phase_{Phase::QueryRemaining};
  std::chrono::steady_clock::time_point phase_start_;
};

}  // namespace mowgli_behavior
