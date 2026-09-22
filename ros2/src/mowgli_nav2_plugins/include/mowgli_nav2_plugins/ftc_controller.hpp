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

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_nav2_plugins/ftc_blade_load.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_reverse_escape.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"
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
 * angular).  A state machine manages the full trajectory lifecycle:
 *
 *   PRE_ROTATE -> FOLLOWING -> WAITING_FOR_GOAL_APPROACH -> POST_ROTATE -> FINISHED
 *                   ^    |
 *                   +----+  PIVOT at each explicit planner corner (ftc_pivot.hpp)
 *
 * Ported from ftc_local_planner (mbf_costmap_core::CostmapController, ROS1).
 */
class FTCController : public nav2_core::Controller
{
public:
  FTCController() = default;
  ~FTCController() override = default;

  // ── nav2_core::Controller interface ──────────────────────────────────────

  void configure(const nav2::LifecycleNode::WeakPtr& parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void newPathReceived(const nav_msgs::msg::Path& path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker,
      const nav_msgs::msg::Path& transformed_global_plan,
      const geometry_msgs::msg::PoseStamped& global_goal) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
  // ── State machine ─────────────────────────────────────────────────────────

  enum class PlannerState
  {
    PRE_ROTATE,
    FOLLOWING,
    WAITING_FOR_GOAL_APPROACH,
    POST_ROTATE,
    FINISHED,
    /// In-place rotation at a pivot corner the coverage planner encoded in the
    /// path (mowgli_interfaces/coverage_geometry.hpp, "PIVOT CORNER CONTRACT").
    /// Entered from FOLLOWING once base_link reaches the corner the carrot is
    /// capped on; rotates with PRE_ROTATE's angular control and limits, no
    /// linear motion and no lateral offset; returns to FOLLOWING from the
    /// corner's outgoing pose. Appended last so the logged state numbers of the
    /// other states are unchanged.
    PIVOT
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

  // ── Pivot corners (ftc_pivot.hpp) ─────────────────────────────────────────

  /// Plan indices of the pivot corners' FIRST (incoming-heading) poses, found
  /// once per plan in newPathReceived. Empty for plans without corners (every
  /// transit plan, every plan from a planner without pivot joins).
  std::vector<std::size_t> pivot_corners_;

  /// The corner the carrot must stop at (first corner at/after current_index_).
  std::optional<std::size_t> nextPivotCorner() const;

  /// FOLLOWING -> PIVOT: stop at the corner, retarget the carrot to the
  /// corner's outgoing pose, drop any lateral offset, reset the PID history.
  void enterPivot();

  /// PIVOT -> FOLLOWING from the corner's outgoing pose.
  void leavePivot();

  /// True when the chassis footprint, rotated in place from the current
  /// heading to the pivot target, overlaps a TRUE-lethal local-costmap cell.
  bool pivotSweepBlocked();

  /// Obstacle gate of a pivot: holds (and eventually aborts, like every other
  /// obstacle hold) while pivotSweepBlocked(); returns true when the pivot may
  /// rotate this tick.
  bool pivotSweepGate(double dt);

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  Eigen::Affine3d current_control_point_;  ///< Carrot pose in map frame.
  Eigen::Affine3d local_control_point_;  ///< Carrot pose in base_link frame.

  uint32_t current_index_{0};
  double current_progress_{0.0};
  double current_movement_speed_{0.0};
  // Seconds the robot has been stalling (actual forward speed << commanded).
  // Drives the anti-wheelspin crawl (see Config::stall_*).
  double stall_time_{0.0};
  // True once stall_time_ passes the grace period: set in update_control_point
  // and read in calculate_velocity_commands to cap the commanded velocity at
  // the crawl speed (bypassing the min_speed_mps floor) so a blocked robot
  // pushes gently instead of digging in. See ftc_stall.hpp.
  bool is_stalled_{false};
  // Latest measured forward speed (odom feedback), cached from
  // computeVelocityCommands so update_control_point can detect a stall.
  double last_measured_fwd_speed_{0.0};
  // True while the blade-load slowdown is holding the carrot's target speed
  // below the path speed: set in update_control_point, read in
  // calculate_velocity_commands to let the commanded speed follow the slowed
  // carrot under the min_speed_mps floor. See ftc_blade_load.hpp.
  bool is_blade_limited_{false};
  // Last applied blade-load scale (1.0 = not limiting); kept for the
  // limit-engaged/released log lines.
  double blade_load_scale_{1.0};

  /// Apply the blade-load slowdown (Config::blade_load_*) to a carrot target
  /// speed, reading the latest blade telemetry under blade_mutex_.
  double applyBladeLoad(double target_speed);

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
  /// Low-pass-filtered PID derivative state (used only when
  /// config_.derivative_filter_tau > 0). Reset alongside last_*_error_.
  double d_lat_filt_{0.0};
  double d_lon_filt_{0.0};
  double d_angle_filt_{0.0};
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

  /// SAFETY (SAFETY_REVIEW_2026-07-23 F-C1): is the robot's ACTUAL current
  /// footprint overlapping a TRUE-LETHAL costmap cell right now? The
  /// deviation-enabled path only ever samples path poses AHEAD of the carrot,
  /// so during PRE_ROTATE pivots, stall-crawl pushes, and lateral blends
  /// nothing checked the body itself — collision_monitor was the sole guard.
  /// Samples the chassis polygon (costmap_ros_->getRobotFootprint()) at the
  /// live robot pose against the local costmap at kLethalOnlyThreshold (254):
  /// a scan return INSIDE the chassis outline means actual/imminent contact.
  /// True-lethal only — inflation halos the robot legitimately hugs must not
  /// trip it. Returns false when pose/footprint/costmap are unavailable
  /// (cannot assert either way; the forward checks still run).
  bool currentBodyInLethal();

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

  /// Whole-profile avoidance (ftc_offset_lattice.hpp): plans the lateral offset
  /// over the horizon and sets target_lateral_deviation_. Returns false when it
  /// engaged the reverse-escape / wait fallback and the caller must return.
  bool planOffsetLattice(std::size_t carrot_idx,
                         const BoundaryGuard& guard,
                         const std::vector<geometry_msgs::msg::Point>& footprint,
                         double dt);

  /// Plan poses [first, last) expressed in the local costmap frame.
  bool planWindowInCostmapFrame(std::size_t first,
                                std::size_t last,
                                std::vector<geometry_msgs::msg::PoseStamped>& out);

  /// The lattice solver parameters planOffsetLattice uses, from config_.
  LatticeSolverCfg latticeSolverCfg() const;

  /// Zone guard (boundary costmap -> local costmap frame) for the deviation
  /// checks. Call with boundary_mutex_ held. False (logged, throttled) when the
  /// global costmap or its transform is missing: deviation is skipped that tick.
  bool buildBoundaryGuard(BoundaryGuard& guard);

  // ── Turn fallback (ftc_turn_fallback.hpp) ─────────────────────────────────
  //
  // When the offset lattice finds no profile and the blockage lies in a TURN
  // of the plan (field 2026-09-22: 8 of 8 WEDGED), FTC improvises the turn
  // instead of aborting: (bounded straight reverse) -> pivot -> straight
  // connector -> pivot -> FOLLOWING from a rejoin pose past the turn. The
  // pivots and the connector are spliced into global_plan_ as runtime pivot
  // corners (mowgli_interfaces/coverage_geometry.hpp contract) so the existing
  // PIVOT state, its per-tick sweep gate, the lattice and currentBodyInLethal
  // execute and re-check them every cycle. Anything not safely possible falls
  // back to the unchanged WEDGED -> reverse-escape -> hold -> abort path.

  enum class TurnFallbackPhase
  {
    kIdle,
    /// Backing straight up (bounded, rear footprint probed every tick) before
    /// the first rotation; the spliced plan is built when it ends.
    kReverse,
    /// The spliced pivots / connector are being driven; ends at the rejoin.
    kRunning,
  };

  struct TurnFallbackState
  {
    TurnFallbackPhase phase{TurnFallbackPhase::kIdle};
    double reverse_target_m{0.0};
    double reverse_done_m{0.0};
    double reverse_time_s{0.0};
    rclcpp::Time started{0, 0, RCL_ROS_TIME};
    /// Working-plan index of the rejoin pose once spliced.
    std::size_t rejoin_idx{0};
    double skipped_arc_m{0.0};
    /// No new fallback before current_index_ reaches this (progress past the
    /// last rejoin), so a turn that stays blocked cannot loop.
    std::optional<std::size_t> rearm_idx;
    std::size_t count{0};
  };

  TurnFallbackState turn_fallback_;
  /// Set on the tick a fallback engages: that tick only stops.
  bool turn_fallback_engaged_now_{false};
  /// Header of the plan as received (frame + stamp), for republishing.
  std_msgs::msg::Header plan_header_;

  bool turnFallbackReversing() const
  {
    return turn_fallback_.phase == TurnFallbackPhase::kReverse;
  }

  /// Called where the lattice would go WEDGED. True when a fallback engaged
  /// (the caller returns; this tick emits zero velocity), false to take the
  /// unchanged WEDGED path. Needs the costmap lock and boundary_mutex_ held
  /// (both are, inside updateLateralDeviation).
  bool tryTurnFallback(std::size_t carrot_idx,
                       const BoundaryGuard& guard,
                       const ObstacleDeviation::Footprint& body);

  /// Plan a fallback from the robot's current pose, reversing at most
  /// `max_reverse_m`. `window_first` receives the plan index of the window's
  /// first pose. Locks held as for tryTurnFallback.
  TurnFallbackPlan planTurnFallback(std::size_t carrot_idx,
                                    const BoundaryGuard& guard,
                                    const ObstacleDeviation::Footprint& body,
                                    double max_reverse_m,
                                    std::size_t& window_first,
                                    TurnFallbackProblem* problem_out = nullptr);

  /// Replace the skipped part of global_plan_ by the fallback's runtime pivot
  /// corners and connector, and start driving them. False when the robot pose
  /// is unavailable (nothing changed).
  bool spliceTurnFallback(const TurnFallbackPlan& plan, std::size_t window_first);

  /// One tick of the fallback's straight reverse; emits into cmd_vel.
  void turnFallbackReverseTick(double dt, geometry_msgs::msg::TwistStamped& cmd_vel);

  /// Completion (rejoin reached) and timeout of a running fallback.
  void turnFallbackProgress();

  /// Log line + hold/abort after a fallback that could not continue.
  void failTurnFallback(const std::string& why);

  /// Apply lateral_deviation_ to current_control_point_ in-place.
  void applyLateralDeviationToCarrot();

  /// Wait-before-abort gate for the AVOIDANCE-out-of-headroom path. Sets
  /// obstacle_waiting_=true (caller halts) until obstacle_wait_timeout_s
  /// has elapsed, then throws ControllerException. Returns true while
  /// still waiting, never returns false on the throw path (the throw
  /// unwinds the stack instead).
  bool waitOrThrowForObstacle(const std::string& reason);

  /// Freeze the virtual carrot and reset PID history while an obstacle hold
  /// owns the output. This prevents the controller from accumulating a large
  /// catch-up error and derivative kick behind a zero-velocity command.
  void holdObstacleMotion();

  /// Bounded reverse-escape gate for the WEDGED case. Called from
  /// updateLateralDeviation instead of waitOrThrowForObstacle when the skirt
  /// search runs out of headroom. If reverse-escape is enabled, a footprint is
  /// available, budget remains, AND the rear footprint is clear of lethal cells,
  /// it engages a straight reverse (sets reverse_escape_active_) and returns
  /// true (caller returns; computeVelocityCommands emits the reverse). Otherwise
  /// it clears the reverse state and forwards to waitOrThrowForObstacle(reason).
  /// SAFETY-CRITICAL: probes the rear footprint at the ACTUAL robot pose
  /// (costmap_ros_->getRobotPose) and never reverses when it would hit lethal or
  /// the pose is unavailable.
  /// One tick of the escape itself: integrate the distance reversed, probe the
  /// rear footprint, decide. No side effects on the wait/abort state.
  ReverseEscapeAction reverseEscapeStep(const ObstacleDeviation::Footprint& footprint, double dt);
  /// Path arc length from the index where the reverse budget was first touched
  /// to the current index (0 when the robot has not advanced).
  double progressSinceReverseEngaged() const;
  bool reverseEscapeOrWait(const std::string& reason,
                           const ObstacleDeviation::Footprint& footprint,
                           double dt);

  /// True while backing straight up as an obstacle-escape maneuver. Read in
  /// computeVelocityCommands to emit the reverse command (bypassing the PID).
  bool reverse_escape_active_{false};
  /// Odom-integrated reversed distance for the current escape (m), hard-capped
  /// at config_.obstacle_reverse_max_dist_m. Reset when the wedge clears.
  double reverse_distance_done_{0.0};
  /// Time the planner has kept a usable profile since the last infeasible tick,
  /// while a reverse-escape is engaged (ReverseEscapeShouldRelease).
  double reverse_followable_time_{0.0};
  /// Path index at which the current reverse budget started being spent; the
  /// budget is only refilled after real progress past it.
  std::size_t reverse_engaged_index_{0};
  bool reverse_budget_touched_{false};

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
  // IMPORTANT: only clear obstacle_wait_start_/obstacle_waiting_ once a
  // candidate has remained workable continuously for obstacle_clear_hold_s.
  // Clearing it on one good scan creates a stop/go loop; clearing it when a
  // side is merely found lets the two wait call-sites hand each other fresh
  // timeout windows near a marginal gap.
  std::optional<rclcpp::Time> obstacle_wait_start_;
  bool obstacle_waiting_{false};
  /// Continuous time for which a valid skirt has existed while
  /// obstacle_waiting_ is active. A blocked tick resets it to zero.
  double obstacle_followable_time_{0.0};
  /// True from the first hard hold until the path has remained followable
  /// while moving for obstacle_clear_hold_s. During this probation the angular
  /// command is slew-limited and a reappearing obstacle keeps the original
  /// wait timeout instead of opening a fresh stop/restart episode.
  bool obstacle_recovery_active_{false};
  /// Last angular command emitted while recovering, used by ClampCommandSlew.
  double last_recovery_angular_cmd_{0.0};
  /// When the nominal path first read CLEAR during an active AVOIDANCE
  /// episode. The skirt is held until the nominal path has stayed clear
  /// continuously for obstacle_clear_hold_s. Obstacle-wait recovery has its
  /// own followable-duration counter because a valid offset path may exist
  /// while the nominal path remains blocked.
  std::optional<rclcpp::Time> avoidance_clear_start_;
  /// Since when the lattice has been asking for a SMALLER offset than applied.
  std::optional<rclcpp::Time> lattice_return_start_;
  /// Since when the free plan has been asking for the side opposite to the committed one.
  std::optional<rclcpp::Time> lattice_switch_start_;

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

  nav2::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FTCController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_map_{nullptr};

  // ── Zone-boundary guard (confine_deviation_to_zone) ───────────────────────
  //
  // The LOCAL costmap (costmap_map_, odom frame) is deliberately boundary-free
  // (obstacle layer only) so near-edge coverage swaths don't read as blocked.
  // The mowing-zone boundary lives as LETHAL cells in the GLOBAL costmap (map
  // frame, keepout / lethal_outside_areas filter). We subscribe to it (latched)
  // and rebuild boundary_costmap_ from each OccupancyGrid; updateLateralDeviation
  // feeds it as an ObstacleDeviation::BoundaryGuard to the lateral-OFFSET checks
  // ONLY, so a skirt never leaves the zone. (The offset samples are in the LOCAL
  // costmap / odom frame here, so the guard affine is boundary(map) <- odom.)
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr boundary_costmap_sub_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> boundary_costmap_;
  std::string boundary_frame_;  ///< frame_id of the global costmap (e.g. "map").
  std::mutex boundary_mutex_;

  // ── Blade-load slowdown telemetry (blade_load_slowdown_enabled) ───────────
  //
  // hardware_bridge republishes the STM32's 4 Hz blade-controller report in
  // /hardware_bridge/status (mower_esc_status = is_active, mower_motor_rpm,
  // blade_status_stamp = time of the last DIRECT blade report — NOT the
  // message stamp, which refreshes on every mainboard packet even when the
  // blade stream has died). update_control_point reads the trio under
  // blade_mutex_ and scales the carrot speed by the RPM sag; a stale stamp
  // fails OPEN (no slowdown). Written by the subscription callback on the
  // controller_server executor, read on the control-loop thread.
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr blade_status_sub_;
  std::mutex blade_mutex_;
  bool blade_active_{false};
  double blade_rpm_{0.0};
  rclcpp::Time blade_status_time_{0, 0, RCL_ROS_TIME};

  std::string plugin_name_;

  // Publishers (lifecycle-aware)
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      global_point_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  /// The plan the coverage progress trackers follow: PathProgressGoalChecker's
  /// plan_topic and FollowStrip's cursor, "~/<plugin>/global_plan" =
  /// /controller_server/FollowCoveragePath/global_plan (FollowStrip publishes
  /// each dispatched unit there). global_plan_pub_ above resolves to
  /// /<plugin>/global_plan instead and only feeds visualisation. FTC publishes
  /// here only when a turn fallback rejoins the plan: the remainder from the
  /// rejoin pose, so neither tracker is left behind at the skipped turn.
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr progress_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
      obstacle_marker_pub_;

  // ── Parameters ────────────────────────────────────────────────────────────

  /// Declare all ROS2 parameters and populate the local config struct.
  void declareParameters(const nav2::LifecycleNode::SharedPtr& node);

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
    /// Angular acceleration limit (rad/s^2) after an obstacle hard hold. This
    /// applies until the resumed path has remained stable for
    /// obstacle_clear_hold_s. 0 disables the limiter.
    double obstacle_restart_angular_acceleration{1.0};
    double min_speed_mps{0.15};

    // Anti-wheelspin / traction control. When the carrot commands a forward
    // speed but the robot's ACTUAL forward speed (odom feedback) stays below
    // stall_speed_ratio * commanded for longer than stall_grace_s, the wheels
    // are slipping or the chassis is blocked. Ease the carrot speed down to
    // stall_crawl_speed instead of ramping to speed_fast and flooring it —
    // which spins the wheels and digs holes in soft turf. Set
    // stall_speed_ratio <= 0 to disable.
    double stall_speed_ratio{0.35};
    double stall_grace_s{0.6};
    double stall_crawl_speed{0.08};

    // Blade-load slowdown. When the blade motor's reported RPM sags under
    // load (thick / wet grass), scale the carrot's target speed down on a
    // linear ramp from 1.0 at blade_load_rpm_full to blade_load_min_speed_ratio
    // at blade_load_rpm_min, floored at stall_crawl_speed, so the blade gets
    // time to chew through instead of stalling or leaving an uncut strip.
    // Fail-open: an inactive blade or telemetry older than
    // blade_load_telemetry_max_age_s never slows the robot. Operator knobs
    // flow from mowgli_robot.yaml (GUI Mowing section) via
    // navigation.launch.py; OFF by default because the no-load RPM differs per
    // blade motor and the thresholds must be read off the Diagnostics page
    // first. See ftc_blade_load.hpp.
    bool blade_load_slowdown_enabled{false};
    double blade_load_rpm_full{2500.0};
    double blade_load_rpm_min{1800.0};
    double blade_load_min_speed_ratio{0.4};
    double blade_load_telemetry_max_age_s{1.0};

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
    // Heading P-gain used ONLY in the FOLLOWING state (straight-swath
    // tracking). kp_ang itself is kept high enough to clear the deadband in a
    // PRE_ROTATE pivot, but that high gain limit-cycles on a straight (the
    // left-right weave). Defaults to kp_ang (no change unless set lower).
    double kp_ang_following{1.0};

    // First-order low-pass time constant (s) for the PID derivative terms
    // (d_lat / d_lon / d_angle). The raw finite-difference derivative amplifies
    // the jitter in the 10 Hz fused-pose feedback, which kd_lat then pumps into
    // the angular command as a ~1.5 Hz steering limit cycle ("hunting"). Filtering
    // the derivative lets kd_lat stay high for tight cross-track tracking without
    // the chatter. 0 disables (raw derivative). From PR #290 (64dce368).
    double derivative_filter_tau{0.0};

    // Robot limits
    double max_cmd_vel_speed{2.0};
    double max_cmd_vel_ang{2.0};
    double max_goal_distance_error{1.0};
    double max_goal_angle_error{10.0};
    /// Heading tolerance (deg) that ends an in-place PIVOT at a planner corner.
    /// Tighter than max_goal_angle_error (PRE_ROTATE's, 30° shipped): the next
    /// straight after a corner is one swath spacing long, too short for the
    /// lateral loop to absorb a large heading error before the next pivot.
    double pivot_angle_tolerance_deg{10.0};
    double goal_timeout{5.0};
    double max_follow_distance{1.0};
    /// Max longitudinal carrot lead (m); <= 0 derives it (ftc_carrot_lead.hpp).
    double carrot_max_lead{-1.0};

    // Options
    bool forward_only{true};
    /// Legacy: snap to the nearest plan point in setPlan instead of starting at
    /// index 0. OFF by default — on a CLOSED headland ring (start == end) the
    /// snap is ambiguous and could skip the whole ring. See setPlan.
    bool snap_to_nearest_on_set_plan{false};
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
    /// Robot body half-width (m, perpendicular to heading) used to make BOTH
    /// obstacle DETECTION and the deviation-clearance search sample the full
    /// chassis sweep instead of only the path centerline. The local costmap's
    /// inscribed-inflation radius (~0.10–0.14 m, the footprint rear edge) is far
    /// smaller than the real half-width (chassis_width/2 ≈ 0.20 m), so a
    /// centerline-only sample misses obstacles in the ~0.14–0.25 m lateral band
    /// that the body still hits — they never trigger avoidance and the robot
    /// drives into them. Sampling across ±half_width (spacing ≤ costmap
    /// resolution) closes that gap. 0 = legacy centerline-only sampling.
    double obstacle_body_half_width{0.20};

    /// Extra lateral clearance (m) demanded when SKIRTING an obstacle, on top
    /// of obstacle_body_half_width. Applied ONLY to the clearance search
    /// (chooseDeviationSide / growDeviationUntilClear) — never to detection
    /// (findFirstObstacleIndex / the clear_at_zero test), which stays at the
    /// bare body half-width.
    ///
    /// Why a separate knob: obstacle_body_half_width does double duty as
    /// "how far ahead do I notice an obstacle" AND "how wide must the gap be
    /// to pass". Widening it to buy pass-by margin also widens detection,
    /// which is exactly the over-reach that caused the 15x
    /// "deviation > max -> hold 5 s" stalls per mow (see the 2026-06-25 note
    /// on obstacle_body_half_width in nav2_params_base.yaml). Splitting the
    /// two lets margin grow without touching detection reach.
    ///
    /// Note this is NOT obtainable via costmap inflation: the deviation checks
    /// threshold at 253 (INSCRIBED_INFLATED_OBSTACLE), a band whose width is
    /// set by the footprint's inscribed radius, not by inflation_radius — so
    /// raising inflation_radius grows only the 1..252 decay region, which
    /// these checks ignore entirely.
    double obstacle_clearance_margin{0.0};

    // Obstacle deviation (FTC's "skirt the obstacle" behaviour). When
    // disabled, hitting an obstacle in lookahead throws ControllerException
    // (the legacy behaviour). When enabled, the carrot is laterally offset
    // until the path is clear, then blended back once the obstacle is past.
    bool enable_obstacle_deviation{true};
    double max_lateral_deviation{1.5};  // m, abort if needed offset exceeds this
    double deviation_step{0.05};  // m, search increment
    double deviation_blend_rate{0.5};  // m/s, slew rate for lateral_deviation_
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
    double obstacle_wait_timeout_s{2.5};
    /// Hysteresis hold (s) before declaring AVOIDANCE complete. Once
    /// avoiding, the nominal path must read CLEAR continuously for this long
    /// before the skirt is blended back to the line. Without it, the
    /// obstacle sitting at the lookahead-window edge (and the
    /// observation_persistence:0 costmap re-marking it each scan) makes the
    /// clear/blocked test flicker, so the old code blended the skirt back at
    /// the first clear tick and re-entered on the other side — the ±step
    /// left-right flap that never grows a deviation big enough to go around.
    double obstacle_clear_hold_s{1.5};
    /// Confine lateral obstacle-avoidance deviation to the mowing zone. When
    /// true, the lateral-OFFSET checks also treat out-of-zone cells (lethal in
    /// the global keepout costmap) as blocked, so a skirt that would leave the
    /// zone is rejected and the wait-or-abort path stops the robot instead.
    /// Applies ONLY to the offset checks, never to the nominal-path check
    /// (near-edge swaths legitimately run inside the keepout margin). When
    /// false, behaves exactly as before.
    bool confine_deviation_to_zone{true};

    /// Model the robot as its actual rectangular chassis FOOTPRINT (from
    /// costmap_ros_->getRobotFootprint()) for obstacle detection and the
    /// deviation-clearance search, instead of a swept ±half_width line. The
    /// footprint is rasterised against the local costmap at ≤ resolution spacing
    /// and thresholds on TRUE lethal (254) — it models the body explicitly, so
    /// FTC no longer leans on the costmap's inscribed-inflation band (253) as a
    /// body-width proxy. When false (or no footprint available), falls back to
    /// the obstacle_body_half_width line model at threshold 253.
    bool use_footprint_clearance{true};

    /// Front-clip length (m) applied to the footprint used for the SKIRT
    /// clearance search when use_footprint_clearance is on (spec Part A middle
    /// ground). Only the leading `obstacle_footprint_front_length_m` of the
    /// chassis is probed for a clear side, instead of the full 0.60 m body — the
    /// full-length footprint proved too conservative live (found NO clear side
    /// even at max_lateral_deviation = 3.0 m for skirtable obstacles). Detection
    /// still uses the FULL footprint. 0 (or a length ≥ the body length) disables
    /// the clip (full-length clearance probe, the prior behaviour).
    double obstacle_footprint_front_length_m{0.30};

    /// Cul-de-sac guard (spec Part A, highest leverage). Before committing to a
    /// lateral skirt, require the obstacle's FAR edge to be visible inside the
    /// lookahead window (a clear nominal-path pose exists past the first blocked
    /// pose). When true, an obstacle that stays blocked to the end of the
    /// lookahead — a wall or a pocket — is NOT skirted sideways (which is how the
    /// robot boxes itself in); instead FTC reverse-escapes / waits / aborts, and
    /// the coverage detour-and-continue safety net takes over. Model-independent
    /// (works with the half-width line model AND the footprint model). Default
    /// true. Set false to restore the prior skirt-anything behaviour.
    bool require_clear_exit{true};
    /// Whole-profile avoidance planner instead of the single-offset search.
    bool use_offset_lattice{false};
    /// How far ahead of the carrot the offset profile is planned (m).
    double avoidance_horizon_m{2.5};
    /// Steepest lateral change per metre of path the profile may ask for.
    double avoidance_max_slope{1.0};
    /// Path length by which a skirt must be in place BEFORE the obstacle (m).
    double avoidance_reaction_m{0.5};
    /// Only a blockage closer than this makes the robot WEDGED (m).
    double avoidance_min_horizon_m{1.0};

    /// Bounded reverse-escape for the WEDGED case (both sides of an obstacle
    /// blocked, or the skirt needed exceeds max_lateral_deviation). Before
    /// holding/aborting, FTC backs STRAIGHT up (no rotation) a bounded distance
    /// and re-attempts the deviation search from the new pose. SAFETY-CRITICAL
    /// (blades + reversing): the rear footprint is checked against lethal cells
    /// before every reverse tick and the maneuver fails safe (stops) on any
    /// ambiguity. This is a distinct escape sub-state — it does NOT relax the
    /// forward_only semantics of normal path following. OPT-IN (default false):
    /// this drives the bladed robot BACKWARDS and has not been field-validated —
    /// enable only after a supervised field test. When false the wedged case
    /// holds/aborts exactly as before.
    bool obstacle_reverse_enabled{false};
    /// Hard cap on total reversed distance (m) per escape. Never exceeded.
    double obstacle_reverse_max_dist_m{0.30};
    /// Straight reverse speed (m/s). Must clear the firmware deadband (~0.05).
    double obstacle_reverse_speed_mps{0.10};

    /// Turn fallback (ftc_turn_fallback.hpp): improvise a turn of the plan the
    /// lattice cannot drive instead of aborting. Lattice planner only
    /// (use_offset_lattice), and only where obstacle deviation runs (never in
    /// the no-LiDAR overlay).
    bool turn_fallback_enabled{true};
    /// Longest straight reverse before the first pivot (m). Also 0 whenever
    /// obstacle_reverse_enabled is false.
    double turn_fallback_max_reverse_m{0.40};
    /// Longest stretch of plan a rejoin may skip (m).
    double turn_fallback_max_rejoin_arc_m{3.0};
    /// A blockage is in a turn when the plan heading sweeps this much (deg).
    double turn_fallback_min_turn_deg{45.0};
    /// The whole fallback (reverse, pivots, connector) must reach its rejoin
    /// within this time (s), else the goal is aborted.
    double turn_fallback_timeout_s{45.0};
  };

  Config config_;

  /// Half-width used by the CLEARANCE checks (which side to skirt, and how far
  /// out the offset path must sit) — the body half-width plus the operator's
  /// clearance margin. Detection deliberately does NOT use this; see
  /// Config::obstacle_clearance_margin.
  double clearanceHalfWidth() const
  {
    return config_.obstacle_body_half_width + std::max(0.0, config_.obstacle_clearance_margin);
  }

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
