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

#ifndef MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_
#define MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2/LinearMath/Quaternion.h>  // No .hpp equivalent for LinearMath
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

#include "mowgli_nav2_plugins/oscillation_detector.hpp"
#include <Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>

namespace mowgli_nav2_plugins
{

/**
 * @class FTCController
 * @brief Nav2 controller plugin implementing the Follow-The-Carrot (FTC) algorithm.
 *
 * The controller advances a virtual carrot point along the global path and drives
 * the robot towards it using three decoupled PID channels (longitudinal, lateral,
 * angular).  A five-state machine manages the full trajectory lifecycle:
 *
 *   PRE_ROTATE -> FOLLOWING -> WAITING_FOR_GOAL_APPROACH -> POST_ROTATE -> FINISHED
 *
 * Ported from ftc_local_planner (mbf_costmap_core::CostmapController, ROS1).
 */
class FTCController : public nav2_core::Controller
{
public:
  FTCController() = default;
  ~FTCController() override = default;

  // ── nav2_core::Controller interface ──────────────────────────────────────

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path& path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
  // ── State machine ─────────────────────────────────────────────────────────

  enum class PlannerState
  {
    PRE_ROTATE,
    FOLLOWING,
    WAITING_FOR_GOAL_APPROACH,
    POST_ROTATE,
    FINISHED
  };

  PlannerState current_state_{PlannerState::PRE_ROTATE};
  rclcpp::Time state_entered_time_;
  bool is_crashed_{false};

  /// Seconds elapsed since the current state was entered.
  double time_in_current_state() const;

  PlannerState update_planner_state();

  // ── Control point / path tracking ─────────────────────────────────────────

  /// Advance the virtual carrot and project it into base_link.
  void update_control_point(double dt);

  /// Compute the look-ahead distance along the remaining straight path.
  double distanceLookahead() const;

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  Eigen::Affine3d current_control_point_;  ///< Carrot pose in map frame.
  Eigen::Affine3d local_control_point_;  ///< Carrot pose in base_link frame.

  uint32_t current_index_{0};
  double current_progress_{0.0};
  double current_movement_speed_{0.0};

  // ── PID state ────────────────────────────────────────────────────────────

  void calculate_velocity_commands(double dt, geometry_msgs::msg::TwistStamped& cmd_vel);

  double lat_error_{0.0};
  double lon_error_{0.0};
  /// Heading error of the carrot in robot frame, **unwrapped**: kept
  /// continuous across the ±π discontinuity by tracking the previous
  /// raw atan2 result and adding the smallest 2π-multiple that
  /// minimises the per-tick change. This prevents a robot whose
  /// instantaneous heading offset is ~±π from seeing the proportional
  /// PID flip sign on every tick (issue #200). The PID still operates
  /// in the same proportional regime — only the discontinuity is
  /// removed — so behaviour for small heading errors is unchanged.
  double angle_error_{0.0};
  /// Previous raw atan2 result, used to detect 2π wraps. NaN means
  /// "no previous sample" (first tick after setPlan).
  double angle_error_raw_prev_{std::numeric_limits<double>::quiet_NaN()};
  double last_lat_error_{0.0};
  double last_lon_error_{0.0};
  double last_angle_error_{0.0};
  double i_lat_error_{0.0};
  double i_lon_error_{0.0};
  double i_angle_error_{0.0};

  rclcpp::Time last_time_;

  // ── Collision checking ────────────────────────────────────────────────────

  bool checkCollision(int max_points);
  void debugObstacle(visualization_msgs::msg::Marker& obstacle_points,
                     double x,
                     double y,
                     unsigned char cost,
                     int max_ids);

  // ── Obstacle deviation ────────────────────────────────────────────────────
  //
  // When checkCollision() reports a lethal cell in the lookahead window,
  // instead of throwing we laterally offset the carrot to skirt the obstacle.
  // Implementation in mowgli_nav2_plugins/obstacle_deviation.{hpp,cpp}.
  //
  //   target_lateral_deviation_ — the offset the algorithm wants right now
  //                                (positive = left of path heading).
  //   lateral_deviation_         — the smoothed value actually applied to
  //                                the carrot, slewed toward the target at
  //                                config_.deviation_blend_rate m/s.
  //   is_avoiding_               — true while the deviation is non-zero, used
  //                                to bias chooseDeviationSide toward the
  //                                already-chosen side (no zigzag).

  /// Update target / smoothed lateral deviation based on current costmap
  /// state. Throws ControllerException if the algorithm needs more than
  /// max_lateral_deviation to find clearance.
  void updateLateralDeviation(double dt);

  /// Apply lateral_deviation_ to current_control_point_ in-place.
  void applyLateralDeviationToCarrot();

  /// Wait-before-abort gate for the AVOIDANCE-out-of-headroom path. Sets
  /// obstacle_waiting_=true (caller halts) until obstacle_wait_timeout_s
  /// has elapsed, then throws ControllerException. Returns true while
  /// still waiting, never returns false on the throw path (the throw
  /// unwinds the stack instead).
  bool waitOrThrowForObstacle(const std::string& reason);

  bool is_avoiding_{false};
  double target_lateral_deviation_{0.0};
  double lateral_deviation_{0.0};
  /// Sign (+1 = left, -1 = right) of the side chosen when AVOIDANCE was
  /// entered. Held for the whole avoidance episode so the min-deviation
  /// floor can restore the correct side even on a tick where a transient
  /// clear_at_zero zeroed target_lateral_deviation_ while is_avoiding_ is
  /// still true — without it, the floor's `(dev_init >= 0) ? +1 : -1` rule
  /// would flip a right-side skirt to the left (toward the blocked side)
  /// and could steer the chassis into the obstacle it was skirting.
  double avoid_sign_{1.0};

  // Wait-before-abort window for the two "no path" cases inside
  // updateLateralDeviation: (a) both sides blocked at the obstacle pose,
  // (b) the deviation needed to clear exceeds max_lateral_deviation. In
  // both cases, before this change, FTC threw a ControllerException
  // immediately — the action server aborted, the BT incremented
  // RetryUntilSuccessful, and after 5 retries the area was declared
  // unreachable. Often the blocker was transient (LIDAR noise during
  // post-undock costmap warmup, a passing person, inflation around the
  // dock body that hadn't been cleared yet); the abort burned 5 cheap
  // retries for nothing. We now hold zero velocity for up to
  // obstacle_wait_timeout_s seconds; if the costmap clears in that
  // window the controller resumes, otherwise it throws as before.
  std::optional<rclcpp::Time> obstacle_wait_start_;
  bool obstacle_waiting_{false};

  // ── Oscillation detection ─────────────────────────────────────────────────

  bool checkOscillation(const geometry_msgs::msg::TwistStamped& cmd_vel);

  FailureDetector failure_detector_;
  // Guards failure_detector_ against the param-callback thread reallocating its
  // ring buffer (setBufferLength) concurrently with the control thread's
  // update()/isOscillating() reads.
  std::mutex failure_detector_mutex_;
  rclcpp::Time time_last_oscillation_;
  bool oscillation_detected_{false};
  bool oscillation_warning_{false};

  // ── ROS2 infrastructure ───────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FTCController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_map_{nullptr};

  std::string plugin_name_;

  // Publishers (lifecycle-aware)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      global_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      obstacle_marker_pub_;

  // ── Parameters ────────────────────────────────────────────────────────────

  /// Declare all ROS2 parameters and populate the local config struct.
  void declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

  /// Parameter-change callback registered with the node.
  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  struct Config
  {
    // Control point speed
    double speed_fast{0.5};
    double speed_fast_threshold{1.5};
    double speed_fast_threshold_angle{5.0};
    double speed_slow{0.2};
    double speed_angular{20.0};
    double acceleration{1.0};

    // PID longitudinal
    double kp_lon{1.0};
    double ki_lon{0.0};
    double ki_lon_max{10.0};
    double kd_lon{0.0};

    // PID lateral
    double kp_lat{1.0};
    double ki_lat{0.0};
    double ki_lat_max{10.0};
    double kd_lat{0.0};

    // PID angular
    double kp_ang{1.0};
    double ki_ang{0.0};
    double ki_ang_max{10.0};
    double kd_ang{0.0};

    // Robot limits
    double max_cmd_vel_speed{2.0};
    double max_cmd_vel_ang{2.0};
    double max_goal_distance_error{1.0};
    double max_goal_angle_error{10.0};
    double goal_timeout{5.0};
    double max_follow_distance{1.0};

    // Options
    bool forward_only{true};
    bool debug_pid{false};
    bool debug_obstacle{false};

    // Recovery
    bool oscillation_recovery{true};
    double oscillation_v_eps{0.05};
    double oscillation_omega_eps{0.05};
    double oscillation_recovery_min_duration{5.0};

    // Obstacles
    bool check_obstacles{true};
    int obstacle_lookahead{5};
    bool obstacle_footprint{true};

    // Obstacle deviation (FTC's "skirt the obstacle" behaviour). When
    // disabled, hitting an obstacle in lookahead throws ControllerException
    // (the legacy behaviour). When enabled, the carrot is laterally offset
    // until the path is clear, then blended back once the obstacle is past.
    bool enable_obstacle_deviation{true};
    double max_lateral_deviation{1.5};   // m, abort if needed offset exceeds this
    double deviation_step{0.05};         // m, search increment
    double deviation_blend_rate{0.5};    // m/s, slew rate for lateral_deviation_
    /// Minimum committed offset magnitude (m) once AVOIDANCE is entered.
    /// growDeviationUntilClear() only checks the path CENTERLINE sample
    /// per pose, so a tiny offset (e.g. one deviation_step) can clear the
    /// centerline while the robot's body — half_width ≈ chassis_width/2 —
    /// still overlaps the lethal obstacle cell. Flooring the committed
    /// deviation to ~half_width + margin guarantees the chassis skirts
    /// the obstacle, not just the path point. 0 disables the floor.
    double min_lateral_deviation{0.30};
    /// Wait-before-abort timeout when the AVOIDANCE search can't fit
    /// inside max_lateral_deviation (both sides blocked or the needed
    /// offset exceeds the cap). During the wait the robot stops; if the
    /// costmap clears before the timeout, the controller resumes. After
    /// the timeout we throw a ControllerException as before.
    double obstacle_wait_timeout_s{5.0};
  };

  Config config_;

  /// Speed limit applied via setSpeedLimit(). -1.0 means "no external limit".
  double speed_limit_{-1.0};
  bool speed_limit_is_percentage_{false};
  // Configured max linear speed, captured at configure() and on every
  // max_cmd_vel_speed parameter change. setSpeedLimit() restores
  // config_.max_cmd_vel_speed to this value when the limit is cleared
  // (speed_limit < 0); without it a once-applied limit stuck forever.
  double base_max_cmd_vel_speed_{2.0};
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__FTC_CONTROLLER_HPP_
