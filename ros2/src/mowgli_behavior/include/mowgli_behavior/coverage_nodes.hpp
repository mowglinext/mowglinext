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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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
// FollowSwathsWithSpin — gyro-locked straight-line strip drive with
// Roborock-style reactive obstacle avoidance.
//
// For each strip from F2C's coverage_path.swaths[]:
//
//   1. /spin — pivot to align with the strip's heading (Nav2 behavior_server
//      Spin action). Sets the heading reference at strip start.
//   2. DRIVE — direct cmd_vel publishing in a tight loop (no Nav2 controller
//      in the path-tracking loop):
//        * Heading PID against the locked yaw_target_odom captured at
//          strip start. Uses /odom yaw which is drift-free local frame —
//          GPS jitter on /map can't kick the robot off course.
//        * LiDAR forward-arc clearance check at every tick. If an obstacle
//          is within obstacle_threshold_m, swerve to the side with more
//          clearance (left vs right arc) at reduced speed. When clear,
//          resume heading PID at full speed.
//        * Stops when the integrated forward distance from /odom reaches
//          strip_length, OR a watchdog timeout fires.
//      This mirrors how a Roborock mows: drive straight, route around
//      obstacles, continue. The strip ends slightly offset laterally
//      after avoidance, but the next strip (0.18 m away) covers any
//      missed area via overlap.
//
// On SUCCESS, paints the driven track into mow_progress.
// ---------------------------------------------------------------------------

class FollowSwathsWithSpin : public BT::StatefulActionNode
{
public:
  using SpinAction = nav2_msgs::action::Spin;
  using SpinGoalHandle = rclcpp_action::ClientGoalHandle<SpinAction>;

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
    SPIN,         // /spin pending — aligning to strip[strip_idx_] heading
    DRIVE,        // gyro-locked + LiDAR-avoid drive forward to strip end
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

  /// Compute the Euclidean length of strip[i].
  double swathLengthM(std::size_t idx) const;

  /// Send /spin with target_yaw = delta to align robot with strip
  /// heading. Returns RUNNING (goal sent) or FAILURE (server unavailable).
  BT::NodeStatus startSpinTo(double target_heading);

  /// Begin the DRIVE phase: capture initial yaw + position from latest /odom,
  /// reset distance accumulator. The control loop runs in onRunning while
  /// state_ == DRIVE.
  BT::NodeStatus startDrive();

  /// One tick of the DRIVE control loop. Reads latest /odom + /scan, computes
  /// cmd_vel (gyro PID + reactive avoidance), publishes to /cmd_vel_nav.
  /// Returns RUNNING, or transitions state_ on completion / failure.
  void tickDrive();

  /// Min /scan range within an angular arc relative to robot forward
  /// (positive = left). Returns +inf if no valid returns in the arc.
  double minRangeInArc(double arc_min_rad, double arc_max_rad) const;

  rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  rclcpp::Client<mowgli_interfaces::srv::PaintSwath>::SharedPtr paint_client_;

  // /cmd_vel publisher — feeds into the existing twist_mux pipeline at the
  // same topic the controller_server normally publishes on. Since we don't
  // dispatch to /follow_path, controller_server is dormant and our cmd_vel
  // is the sole publisher for the strip drive.
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;

  // /spin pending state
  std::shared_future<SpinGoalHandle::SharedPtr> spin_future_;
  SpinGoalHandle::SharedPtr spin_handle_;
  std::shared_future<SpinGoalHandle::WrappedResult> spin_result_future_;
  bool spin_result_requested_{false};

  // DRIVE state
  double drive_target_yaw_odom_{0.0};   // yaw at strip start (odom frame, gyro-locked)
  double drive_start_x_odom_{0.0};
  double drive_start_y_odom_{0.0};
  double drive_target_distance_m_{0.0}; // strip length
  rclcpp::Time drive_start_time_;

  // Iteration state
  State state_{State::DONE_FAIL};
  std::size_t strip_idx_{0};
  double current_strip_heading_{0.0};

  // Driven-track accumulator.
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
