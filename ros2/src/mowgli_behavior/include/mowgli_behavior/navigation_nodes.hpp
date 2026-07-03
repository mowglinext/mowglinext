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
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_recovery_point.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// StopMoving
// ---------------------------------------------------------------------------

/// Publishes zero velocity to /cmd_vel_emergency (twist_mux priority 100) to
/// halt the robot. Publishes continuously for `duration_sec` so that twist_mux
/// keeps the emergency channel latched and actively overrides any /cmd_vel_nav
/// commands still coming from a RUNNING Nav2 action (e.g. when BT branches
/// from TRANSIT into SkipStrip without first cancelling FollowPath). One-shot
/// wasn't enough: field session showed the robot drifting 0.40 m after a
/// supposed stop because FollowPath kept commanding forward velocity.
///
/// Input ports:
///   duration_sec (double, default 0.5) — how long to stream zero velocity.
class StopMoving : public BT::StatefulActionNode
{
public:
  StopMoving(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec", 0.5, "Seconds to stream zero-velocity cmds (s)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Time start_time_;
  double duration_sec_{0.5};
  void publish_zero(const rclcpp::Node::SharedPtr& node);
};

// ---------------------------------------------------------------------------
// ClearCostmap
// ---------------------------------------------------------------------------

/// Calls the Nav2 clear_entirely service on both the global and local costmaps.
///
/// This is a synchronous fire-and-forget node: it sends both service requests
/// without waiting for responses (the node is already being spun by the main
/// executor), then returns SUCCESS immediately.  Useful after obstacle removal
/// to let the planner see a clean costmap before retrying coverage.
class ClearCostmap : public BT::SyncActionNode
{
public:
  ClearCostmap(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_client_;
};

// ---------------------------------------------------------------------------
// SetNav2Lifecycle
// ---------------------------------------------------------------------------

/// PAUSEs or RESUMEs the Nav2 lifecycle stack via
/// /lifecycle_manager_navigation/manage_nodes, to drop idle CPU/thermal
/// load while the robot sits docked. The whole Nav2 navigation group
/// (controller/planner/smoother/behaviors/bt_navigator/waypoint/docking/
/// coverage/collision_monitor) is otherwise kept fully active and looping
/// its costmaps even while parked — on the Pi 4 that is the dominant idle
/// thermal cost (triaged 2026-06-16).
///
/// SAFETY: pausing the navigation group also deactivates collision_monitor,
/// so PAUSE is gated on charger_enabled — it only fires when the robot is
/// physically on the dock, where it cannot move (motors coast at zero, no
/// cmd_vel is issued in IDLE) and the firmware remains the sole safety
/// authority. RESUME is wired as a root guard that runs before any motion
/// branch, and the existing Nav2ReadyPoll in MowingSequence blocks all
/// motion until lifecycle_manager reports active — so collision_monitor is
/// always active again before the robot can drive.
///
/// Behaviour is gated behind the idle_nav2_suspend blackboard flag
/// (default false): when disabled this node is a pure no-op (a single
/// blackboard read, no service call). The node tracks ctx->nav2_suspended
/// so a manage_nodes request is only sent on an actual transition — RESUME
/// when already active, or a redundant PAUSE, costs nothing.
///
/// Input ports:
///   - command: "PAUSE" or "RESUME"
class SetNav2Lifecycle : public BT::SyncActionNode
{
public:
  SetNav2Lifecycle(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("command", "PAUSE or RESUME"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// NavigateToPose
// ---------------------------------------------------------------------------

/// Stateful action that sends a goal to the Nav2 NavigateToPose action server
/// and waits for it to complete.
///
/// Input ports:
///   goal (string) – target pose encoded as "x;y;yaw" (metres / radians,
///                   frame_id = "map").
class NavigateToPose : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPose(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal", "Target pose as 'x;y;yaw'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;

  /// Lazily creates the action client once and reuses it across ticks.
  void ensureActionClient(const rclcpp::Node::SharedPtr& node);
};

// ---------------------------------------------------------------------------
// BackUp
// ---------------------------------------------------------------------------

/// Calls the Nav2 /backup action to reverse the robot by a given distance.
/// Used for undocking (reverse away from charger) and recovery (back away
/// from unseen obstacles).
///
/// Input ports:
///   backup_dist  (double, default "0.5") – distance to reverse in metres.
///   backup_speed (double, default "0.15") – reverse speed in m/s.
class BackUp : public BT::StatefulActionNode
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;
  using GoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;

  BackUp(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("backup_dist", 0.5, "Distance to reverse (m)"),
            BT::InputPort<double>("backup_speed", 0.15, "Reverse speed (m/s)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using WrappedResult = rclcpp_action::ClientGoalHandle<BackUpAction>::WrappedResult;

  rclcpp_action::Client<BackUpAction>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// SetNavMode
// ---------------------------------------------------------------------------

/// Dynamically adjusts Nav2 controller speed and costmap inflation based on
/// GPS quality.  "precise" = full speed, "degraded" = half speed + wider
/// inflation.
///
/// Input ports:
///   mode (string) – "precise" or "degraded"
class SetNavMode : public BT::SyncActionNode
{
public:
  SetNavMode(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "precise", "Navigation mode: precise or degraded")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// NavigateInsideBoundary
// ---------------------------------------------------------------------------

/// Recovery node used when the robot has drifted past a polygon edge.
///
/// Phases:
///   1. Call `/map_server_node/get_recovery_point` for a pose
///      ~boundary_recovery_offset_m inside the nearest polygon, facing inward.
///   2. Temporarily disable the global_costmap's `keepout_filter`. Without
///      this Smac refuses the plan with `"Start occupied"` because the
///      robot's current cell is in the keepout-lethal zone (that's exactly
///      what triggered the recovery — robot drifted past the line). Disable
///      → clear-costmap → plan-against-clean-costmap → re-enable on exit.
///   3. Hand the recovery pose to Nav2 `/navigate_to_pose`.
///   4. On any termination (success / abort / cancel / halt), re-enable
///      `keepout_filter` so the boundary safety is back as soon as the
///      robot is inside.
///
/// Returns FAILURE if the recovery service is unreachable / unhappy, or if
/// Nav2 aborts/cancels the recovery goal. In that case the BT escalates to
/// the lethal-boundary emergency path. The keepout toggle is best-effort:
/// failures to disable/re-enable are logged but do not themselves fail the
/// recovery — we still try the Nav2 leg, and the next costmap update with
/// `enabled=true` restores the filter.
class NavigateInsideBoundary : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;
  using BackUpAction = nav2_msgs::action::BackUp;
  using BackUpGoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;
  using RecoverySrv = mowgli_interfaces::srv::GetRecoveryPoint;
  using ClearSrv = nav2_msgs::srv::ClearEntireCostmap;
  using SetParamsResult = std::vector<rcl_interfaces::msg::SetParametersResult>;

  NavigateInsideBoundary(const std::string& name, const BT::NodeConfig& config)
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
  enum class Phase
  {
    WaitingForService,  // /map_server_node/get_recovery_point
    DisablingKeepout,  // global_costmap.set_parameters(keepout_filter.enabled=false)
    ClearingCostmap,  // global_costmap/clear_entirely_global_costmap
    WaitingForGoalHandle,  // /navigate_to_pose
    WaitingForResult,
    FallbackBackingUp,  // /backup (open-loop reverse when Smac aborts)
    ReEnablingKeepout,  // global_costmap.set_parameters(keepout_filter.enabled=true)
  };

  // Reset Phase + tracking state for a fresh recovery attempt.
  void ResetState();

  // Best-effort fire-and-forget re-enable of the keepout filter. Used by
  // onHalted to leave the costmap in its normal state even if we never
  // reach Phase::ReEnablingKeepout cleanly.
  void RequestKeepoutEnable(bool enabled);

  // Send the /navigate_to_pose goal using recovery_pose_, transition into
  // WaitingForGoalHandle, and return RUNNING. Falls into BeginReEnableKeepout
  // if the action server isn't reachable. Always returns a tickable status.
  BT::NodeStatus SendNav2Goal();

  // Kick off the keepout re-enable using AsyncParametersClient and transition
  // into ReEnablingKeepout. Short-circuits to the pending Nav2 result if
  // there is nothing to re-enable.
  BT::NodeStatus BeginReEnableKeepout();

  // Fire an open-loop BackUp action (~0.5 m reverse) as a fallback when
  // Smac aborts the recovery plan. Transitions into FallbackBackingUp.
  // Used at most once per onStart() to break boundary-edge deadlocks
  // where the planner refuses to plan from the robot's lethal cell even
  // after the keepout filter has been disabled.
  BT::NodeStatus BeginFallbackBackup();

  rclcpp::Client<RecoverySrv>::SharedPtr service_client_;
  rclcpp::Client<ClearSrv>::SharedPtr clear_client_;
  rclcpp::AsyncParametersClient::SharedPtr keepout_params_client_;
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  rclcpp_action::Client<BackUpAction>::SharedPtr backup_action_client_;

  std::shared_future<RecoverySrv::Response::SharedPtr> service_future_;
  std::shared_future<SetParamsResult> set_param_future_;
  std::shared_future<ClearSrv::Response::SharedPtr> clear_future_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<BackUpGoalHandle::SharedPtr> backup_goal_handle_future_;
  BackUpGoalHandle::SharedPtr backup_goal_handle_;
  std::shared_future<BackUpGoalHandle::WrappedResult> backup_result_future_;
  bool backup_result_requested_{false};

  // Cached recovery target from the map_server, held across the
  // disable-keepout / clear-costmap phases until we actually send the
  // Nav2 goal.
  geometry_msgs::msg::Pose recovery_pose_;
  double distance_outside_{0.0};

  // Latched Nav2 outcome carried through ReEnablingKeepout so we return
  // the right BT status after the re-enable round-trip.
  BT::NodeStatus pending_nav_result_{BT::NodeStatus::FAILURE};
  // Set true between DisablingKeepout and ReEnablingKeepout — onHalted
  // checks this to fire the safety re-enable.
  bool keepout_disabled_{false};
  // True after we've already used the BackUp fallback once this onStart()
  // cycle. The outer BT (RetryUntilSuccessful num_attempts="2") gives us a
  // second cycle if needed, so we don't try to chain multiple backups in
  // one cycle.
  bool fallback_attempted_{false};

  Phase phase_{Phase::WaitingForService};
};

}  // namespace mowgli_behavior
