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
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "opennav_coverage_msgs/action/navigate_complete_coverage.hpp"
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

  // ── Hole-filter geometry helpers (public for unit-test access) ───────────
  // These are pure shoelace / ray-casting / point-segment-distance routines
  // exposed here so test_coverage_nodes.cpp can verify the filter without
  // standing up an action server.
  static double polygonArea(const geometry_msgs::msg::Polygon& p);
  static bool pointInPolygon(double x, double y, const geometry_msgs::msg::Polygon& p);
  static double distanceToPolygonBoundary(double x, double y,
                                          const geometry_msgs::msg::Polygon& p);
  /// Returns true when @p obs is safe to send to F2C as a hole inside @p field:
  ///   - has ≥ 3 points and area ≥ @p min_area_m2
  ///   - every vertex lies inside @p field with ≥ @p min_clearance_m to its
  ///     boundary (so the hole survives F2C's headland inset)
  static bool isHoleSafeForF2C(const geometry_msgs::msg::Polygon& obs,
                               const geometry_msgs::msg::Polygon& field,
                               double min_area_m2,
                               double min_clearance_m);

private:
  rclcpp_action::Client<CoverageAction>::SharedPtr action_client_;
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr area_client_;
  std::shared_future<CoverageGoalHandle::SharedPtr> goal_future_;
  CoverageGoalHandle::SharedPtr goal_handle_;
  std::shared_future<CoverageGoalHandle::WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// NavigateToFirstStripPose — drives the robot to ctx->current_coverage_path
// .poses[0] (the first F2C strip endpoint) using Nav2's /navigate_to_pose
// action, BEFORE FollowCoveragePath runs. This matches the upstream
// GHANSHYAM-13/coverage-path-planning architecture and replaces the prior
// "prepend robot pose to F2C plan" hack — that hack inserted a non-strip-
// shaped lead-in segment which confused MPPI's PathHandler closest-point
// logic and produced the looping driven track captured 2026-05-05.
// With this node in place, when FollowCoveragePath starts the robot is
// already at path[0] and MPPI tracks pure strips end-to-end.
// ---------------------------------------------------------------------------

class NavigateToFirstStripPose : public BT::StatefulActionNode
{
public:
  using NavAction = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  NavigateToFirstStripPose(const std::string& name, const BT::NodeConfig& config)
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
  rclcpp_action::Client<NavAction>::SharedPtr action_client_;
  std::shared_future<NavGoalHandle::SharedPtr> goal_future_;
  NavGoalHandle::SharedPtr goal_handle_;
  std::shared_future<NavGoalHandle::WrappedResult> result_future_;
  bool result_requested_{false};
};

// ---------------------------------------------------------------------------
// MowHeadlandPerimeter — perimeter sweep along F2C's planning_field.
//
// F2C's coverage_server publishes /coverage_server/planning_field once per
// ComputeCoveragePath call: the inset polygon = field minus headland_width.
// behavior_tree_node subscribes (transient_local) and stashes the latest
// message in ctx->current_planning_field.
//
// This node:
//   1. Reads the planning_field polygon from BTContext.
//   2. Builds a closed-loop nav_msgs/Path tracing the polygon vertices
//      (with intermediate poses every ~0.1 m so MPPI's PathHandler has
//      dense waypoints), starting from whichever vertex is closest to
//      the robot's current pose.
//   3. Sends /follow_path with controller_id=FollowCoveragePath (MPPI).
//   4. Paints the driven track on success.
//
// Sequenced BEFORE FollowSwathsWithSpin so the perimeter band is mowed
// first. Strip-end transitions in FollowSwathsWithSpin then happen
// safely inside the already-mowed headland zone — no boundary risk.
// ---------------------------------------------------------------------------

class MowHeadlandPerimeter : public BT::StatefulActionNode
{
public:
  using FollowPathAction = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPathAction>;

  MowHeadlandPerimeter(const std::string& name, const BT::NodeConfig& config)
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

  rclcpp_action::Client<FollowPathAction>::SharedPtr follow_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  rclcpp::Client<mowgli_interfaces::srv::PaintSwath>::SharedPtr paint_client_;

  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
  std::shared_future<FollowGoalHandle::WrappedResult> result_future_;
  bool result_requested_{false};

  nav_msgs::msg::Path driven_trajectory_;
  static constexpr double min_pose_step_m_ = 0.05;
  bool blade_on_{false};

  // Per-launch wait for planning_field (subscription is transient_local
  // so it should arrive immediately after ComputeCoveragePath, but the
  // first time through there can be a small race).
  std::chrono::steady_clock::time_point wait_start_;
  bool waiting_for_planning_field_{false};
};

// ---------------------------------------------------------------------------
// FollowSwathsWithSpin — pivot-in-place at strip ends.
//
// Iterates ctx->current_swaths (populated by ComputeCoveragePath from
// F2C's coverage_path.swaths[]). For each strip:
//
//   1. /spin — pivot in place to align with the strip's heading (atan2
//      from start to end). The first strip's spin handles the post-
//      NavigateToFirstStripPose alignment; subsequent spins are 180°
//      flips between adjacent boustrophedon strips.
//   2. /follow_path — follow a 2-pose mini-path (start → end) along
//      the strip with FollowCoveragePath controller (MPPI). Since the
//      mini-path is a straight line, MPPI tracks it cleanly without
//      the wide-arc smoothing it does at strip-end transitions of
//      F2C's connected nav_path.
//
// On SUCCESS, paints the driven track into mow_progress.
//
// This node is the "option 2" implementation of the user's pivot-in-
// place request: F2C still plans the coverage layout, but we discard
// its inter-strip turn arcs and synthesise explicit pivots ourselves.
// Mirrors what main-branch FTC achieved before the opennav_coverage
// migration.
// ---------------------------------------------------------------------------

class FollowSwathsWithSpin : public BT::StatefulActionNode
{
public:
  using SpinAction = nav2_msgs::action::Spin;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<SpinAction>;
  using FollowPathAction = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPathAction>;
  using NavAction = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  FollowSwathsWithSpin(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index — for paint")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class State
  {
    SPIN,         // /spin pending (aligning to strip[strip_idx_] heading)
    FOLLOW,       // /follow_path pending (driving along strip[strip_idx_])
    NAV_AROUND,   // /navigate_to_pose pending — Option B obstacle bypass:
                  // when both RPP and MPPI fail on a strip, route around
                  // the obstacle to the strip's far end via Smac planner
                  // so we still mow the un-blocked portion of the strip.
                  // Permanent obstacles should be operator-flagged and
                  // fed into F2C as polygon holes; this state handles
                  // unflagged transient obstacles only.
    NEXT_STRIP,   // momentary; advance strip_idx_ then back to SPIN or DONE
    DONE_OK,
    DONE_FAIL,
  };

  void setBladeEnabled(bool enabled);
  void paintTrajectory(const nav_msgs::msg::Path& trajectory);
  void recordPose();

  /// Compute the heading the robot should face to drive strip[i].start →
  /// strip[i].end as a forward straight line.
  double swathHeadingRad(std::size_t idx) const;

  /// Build a 2-pose nav_msgs/Path for FollowPath: header = map / now,
  /// poses[0] = swath start at heading h, poses[1] = swath end at h.
  nav_msgs::msg::Path buildStripPath(std::size_t idx, double heading) const;

  /// Send /spin with target_yaw = delta to align robot with strip
  /// heading. Returns RUNNING (goal sent) or FAILURE (server unavailable).
  BT::NodeStatus startSpinTo(double target_heading);

  /// Send /follow_path for strip[strip_idx_] using @p controller_id
  /// ("FollowPath" = RPP — clean straight tracking, "FollowCoveragePath"
  /// = MPPI — obstacle-deviating). Returns RUNNING or FAILURE.
  BT::NodeStatus startFollow(const std::string& controller_id);

  /// Send /navigate_to_pose to strip[strip_idx_].end as the Option B
  /// obstacle bypass after both RPP and MPPI failed on this strip.
  /// Smac (the global planner) routes around any costmap-marked
  /// obstacle in between, MPPI tracks the plan so dynamic deviation
  /// is preserved. Returns RUNNING or FAILURE.
  BT::NodeStatus startNavAroundObstacle();

  rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
  rclcpp_action::Client<FollowPathAction>::SharedPtr follow_client_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  rclcpp::Client<mowgli_interfaces::srv::PaintSwath>::SharedPtr paint_client_;

  // /spin pending state
  std::shared_future<SpinGoalHandle::SharedPtr> spin_future_;
  SpinGoalHandle::SharedPtr spin_handle_;
  std::shared_future<SpinGoalHandle::WrappedResult> spin_result_future_;
  bool spin_result_requested_{false};

  // /follow_path pending state
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
  std::shared_future<FollowGoalHandle::WrappedResult> follow_result_future_;
  bool follow_result_requested_{false};

  // /navigate_to_pose pending state (NAV_AROUND, Option B obstacle bypass)
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  std::shared_future<NavGoalHandle::WrappedResult> nav_result_future_;
  bool nav_result_requested_{false};

  // Iteration state
  State state_{State::DONE_FAIL};
  std::size_t strip_idx_{0};
  double current_strip_heading_{0.0};
  // Hybrid controller fallback: when RPP fails (typically because its
  // collision-detection found a LiDAR-detected obstacle on the strip
  // path), retry the SAME strip with MPPI. MPPI's ObstaclesCritic
  // produces a deviation arc around the obstacle and continues to the
  // strip end, so the strip area on either side of the obstacle still
  // gets mowed instead of being skipped.
  bool mppi_fallback_used_{false};

  // Driven-track accumulator (same role as in MowAreaWithCoverage).
  nav_msgs::msg::Path driven_trajectory_;
  static constexpr double min_pose_step_m_ = 0.05;
  bool blade_on_{false};
  uint32_t area_idx_{0};
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
