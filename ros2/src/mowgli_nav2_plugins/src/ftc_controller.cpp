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

#include "mowgli_nav2_plugins/ftc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <nav2_ros_common/node_utils.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "mowgli_interfaces/ftc_abort_reason.hpp"
#include "mowgli_nav2_plugins/ftc_carrot_lead.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_obstacle_wait.hpp"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include "mowgli_nav2_plugins/ftc_resync.hpp"
#include "mowgli_nav2_plugins/ftc_stall.hpp"
#include "mowgli_nav2_plugins/ftc_start_index.hpp"
#include "mowgli_nav2_plugins/ftc_zone_grid.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

namespace mowgli_nav2_plugins
{

namespace
{
// Command horizon when controller_server's controller_frequency cannot be
// read (unit tests that configure FTC on a bare node): the shipped 10 Hz.
constexpr double kDefaultCommandHorizonSec = 0.1;
constexpr double kMaxControlIntervalSec = 0.5;  // existing supported PID interval
constexpr double kStoppedTranslationMps = 0.01;
// Once a PRE_ROTATE/PIVOT episode has seen the translation settled, only motion
// above this re-inhibits the rotation. The in-place rotation itself makes the
// wheel odometry read +-0.027 m/s (one tick of difference between the wheels
// per 66 ms sample); against kStoppedTranslationMps every such sample zeroed
// the command, the robot stuttered and 6 of 13 pivots hit the 10 s timeout
// (field 2026-11-09, bag 113449Z). Real coasting (>= 2 ticks) still holds it.
constexpr double kRotationTranslationNoiseMps = 0.05;
}  // namespace

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void FTCController::configure(const nav2::LifecycleNode::WeakPtr& parent,
                              std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_map_ = costmap_ros_->getCostmap();

  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error("FTCController: failed to lock lifecycle node during configure");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declareParameters(node);
  // The pivot stopping budgets reserve whole command intervals: derive them
  // from the rate controller_server actually ticks at, not a literal 10 Hz.
  command_horizon_s_ = kDefaultCommandHorizonSec;
  if (node->has_parameter("controller_frequency"))
  {
    const rclcpp::Parameter frequency = node->get_parameter("controller_frequency");
    const double hz = frequency.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                          ? static_cast<double>(frequency.as_int())
                          : (frequency.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                                 ? frequency.as_double()
                                 : 0.0);
    if (std::isfinite(hz) && hz > 0.0)
    {
      command_horizon_s_ = 1.0 / hz;
    }
  }
  RCLCPP_INFO(logger_, "FTCController: command horizon %.3f s.", command_horizon_s_);
  corner_geometry_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/map_server_node/transit_geometry",
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
      {
        auto parsed = CornerGeometry::parse(*message);
        parsed.entry_tolerance = config_.coverage_entry_tolerance_m;
        auto geometry = std::make_shared<const CornerGeometry>(std::move(parsed));
        std::lock_guard lock(corner_geometry_mutex_);
        corner_geometry_ = std::move(geometry);
      },
      rclcpp::QoS(1).transient_local());

  // Publishers (created as lifecycle-aware, activated/deactivated with the node).
  global_point_pub_ =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(plugin_name_ + "/global_point",
                                                              rclcpp::QoS(1));
  global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/global_plan",
                                                                 rclcpp::QoS(1).transient_local());
  // Same QoS as FollowStrip's publisher on that topic and the goal checker's
  // subscription (reliable + transient_local, depth 1).
  progress_plan_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("~/" + plugin_name_ + "/global_plan",
                                                  rclcpp::QoS(1).transient_local());
  obstacle_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::Marker>(plugin_name_ + "/costmap_marker",
                                                              rclcpp::QoS(10));

  // Subscribe to the GLOBAL costmap (map frame, latched). It carries the
  // mowing-zone boundary as lethal cells (keepout / lethal_outside_areas
  // filter), binarised into boundary_costmap_ so the lateral-OFFSET deviation
  // checks can refuse to skirt out of the zone. The full grid only arrives on
  // connect and on a resize; every later change comes as a delta on
  // costmap_updates and must be applied too, or the zone stays frozen at the
  // first snapshot (occupancy_grid_update.hpp).
  boundary_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr og)
      {
        auto cm = ZoneCostmapFromGrid(*og);
        std::lock_guard<std::mutex> lock(boundary_mutex_);
        boundary_costmap_ = std::move(cm);
        boundary_frame_ = og->header.frame_id;
      },
      rclcpp::QoS(1).transient_local());
  boundary_update_sub_ = node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/global_costmap/costmap_updates",
      [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr update)
      {
        std::lock_guard<std::mutex> lock(boundary_mutex_);
        if (boundary_costmap_ == nullptr || !ApplyZoneGridUpdate(*boundary_costmap_, *update))
        {
          // No base grid yet, or a delta for a resized grid: the full grid
          // that a resize republishes replaces it.
          RCLCPP_DEBUG(logger_,
                       "FTCController: dropped a global costmap update that does not fit.");
        }
      },
      rclcpp::QoS(10));

  // Blade telemetry for the blade-load slowdown (ftc_blade_load.hpp). Always
  // subscribed — the feature can be switched on live via the
  // blade_load_slowdown_enabled parameter — and cheap: one struct copy per
  // status message. Reliable QoS matches the bridge's publisher (QoS(10)).
  blade_status_sub_ = node->create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      [this](const mowgli_interfaces::msg::Status::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(blade_mutex_);
        blade_active_ = msg->mower_esc_status != 0u;
        blade_rpm_ = static_cast<double>(msg->mower_motor_rpm);
        blade_status_time_ = rclcpp::Time(msg->blade_status_stamp, RCL_ROS_TIME);
      },
      rclcpp::QoS(1));

  // collision_monitor's verdict (its state_topic), published on change only:
  // the last value is the current one. Lets FTC tell "collision_monitor holds
  // the wheels" from "the robot is slow" (ftc_collision_monitor_hold.hpp).
  cm_state_sub_ = node->create_subscription<nav2_msgs::msg::CollisionMonitorState>(
      "/collision_monitor_state",
      [this](const nav2_msgs::msg::CollisionMonitorState::SharedPtr msg)
      {
        cm_action_.store(msg->action_type);
      },
      rclcpp::QoS(10));

  // The robot is outside every recorded area (map_server, undebounced). Reverse refuses while true.
  outside_area_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/map_server_node/outside_area",
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        outside_area_.store(msg->data);
      },
      rclcpp::QoS(1));

  current_state_ = PlannerState::PRE_ROTATE;
  last_time_ = clock_->now();
  time_last_oscillation_ = clock_->now();

  failure_detector_.setBufferLength(
      static_cast<int>(std::round(config_.oscillation_recovery_min_duration * 10.0)));

  RCLCPP_INFO(logger_, "FTCController: configured as '%s'.", plugin_name_.c_str());
}

void FTCController::cleanup()
{
  RCLCPP_INFO(logger_, "FTCController: cleanup.");
  global_point_pub_.reset();
  global_plan_pub_.reset();
  progress_plan_pub_.reset();
  obstacle_marker_pub_.reset();
  corner_geometry_sub_.reset();
  {
    std::lock_guard lock(corner_geometry_mutex_);
    corner_geometry_.reset();
    checked_corner_geometry_.reset();
    tick_corner_geometry_.reset();
  }
  pivot_pose_epoch_.reset();
  boundary_costmap_sub_.reset();
  boundary_update_sub_.reset();
  outside_area_sub_.reset();
  blade_status_sub_.reset();
  cm_state_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(boundary_mutex_);
    boundary_costmap_.reset();
  }
}

void FTCController::activate()
{
  RCLCPP_INFO(logger_, "FTCController: activate.");
  global_point_pub_->on_activate();
  global_plan_pub_->on_activate();
  progress_plan_pub_->on_activate();
  obstacle_marker_pub_->on_activate();
}

void FTCController::deactivate()
{
  RCLCPP_INFO(logger_, "FTCController: deactivate.");
  global_point_pub_->on_deactivate();
  global_plan_pub_->on_deactivate();
  progress_plan_pub_->on_deactivate();
  obstacle_marker_pub_->on_deactivate();
}

// ── setPlan ───────────────────────────────────────────────────────────────────

void FTCController::newPathReceived(const nav_msgs::msg::Path& path)
{
  applyPendingConfig();
  pivot_approach_start_.reset();
  current_state_ = PlannerState::PRE_ROTATE;
  state_entered_time_ = clock_->now();
  ++state_entry_seq_;
  is_crashed_ = false;
  rotation_hold_reason_.clear();

  // Reset deviation state with the new path so a previous strip's avoidance
  // doesn't leak into the new one.
  is_avoiding_ = false;
  target_lateral_deviation_ = 0.0;
  lateral_deviation_ = 0.0;
  obstacle_wait_start_.reset();
  obstacle_waiting_ = false;
  obstacle_followable_time_ = 0.0;
  obstacle_recovery_active_ = false;
  last_recovery_angular_cmd_ = 0.0;
  avoidance_clear_start_.reset();

  // Reset reverse-escape sub-state — a new strip must never inherit a
  // mid-reverse budget from the previous one.
  reverse_escape_active_ = false;
  reverse_distance_done_ = 0.0;
  reverse_elapsed_s_ = 0.0;
  reverse_followable_time_ = 0.0;
  reverse_budget_touched_ = false;
  reverse_engaged_index_ = 0;

  // A new plan never inherits a fallback of the previous one.
  turn_fallback_ = TurnFallbackState{};
  turn_fallback_engaged_now_ = false;
  plan_header_ = path.header;

  // Reset angle unwrapping state — the new path's first pose orientation
  // is the new reference; nothing prior to setPlan informs continuity.
  angle_error_raw_prev_ = std::numeric_limits<double>::quiet_NaN();
  resetOscillation();

  global_plan_ = path.poses;
  current_index_ = 0;
  current_progress_ = 0.0;

  // Pivot corners the coverage planner encoded in this plan (two poses at the
  // same position, incoming then outgoing heading). Found on the plan AS
  // RECEIVED: the tail duplication below appends and re-orients only the last
  // pose, which FindPivotCorners never counts as a corner's outgoing pose.
  {
    std::vector<PlanPose2D> poses;
    poses.reserve(path.poses.size());
    for (const auto& ps : path.poses)
    {
      poses.push_back({ps.pose.position.x, ps.pose.position.y, tf2::getYaw(ps.pose.orientation)});
    }
    pivot_corners_ = FindPivotCorners(poses);
  }

  // Start at the BEGINNING of a freshly dispatched plan.
  //
  // This used to run an UNBOUNDED nearest-point search over the whole plan and
  // begin tracking wherever that landed. On coverage headland rings — which are
  // CLOSED, start == end to the millimetre — index 0 and index N-1 are the SAME
  // POINT, so the search is genuinely ambiguous and floating-point noise decides
  // which end wins. When the last index won, FTC drove three poses, reported the
  // goal reached, and FollowStrip recorded the ring as MOWED. Observed on the
  // robot on both 2026-08-24 runs:
  //
  //     new path with 436 poses, start=(1.95,9.50), end=(1.95,9.50)
  //     setPlan with 436 points, starting at idx=432   -> 99 % of the ring skipped
  //     setPlan with 805 points, starting at idx=372   -> 46 % skipped
  //
  // and both were then logged "reached 99-100 % of path - treating as MOWED".
  // Un-mowed ground marked done, with skipped_swaths still reading 0.
  //
  // Resume is not this function's job and never was: FollowStrip already owns it
  // by trimming the path at its resume cursor BEFORE dispatch, and it decides
  // whether to transit blade-off first by measuring to poses.front() — index 0.
  // Snapping to a different index here silently disagreed with that decision.
  // Starting at 0 makes the two consistent again.
  //
  // snap_to_nearest_on_set_plan restores the old behaviour if a site needs it.
  //
  // NOTE: this block chooses the START INDEX ONLY. Everything after it —
  // PID/derivative/stall reset, the tail-duplication the state machine's
  // `size() - 2` arithmetic depends on, the latched plan publish, and the
  // <3-pose termination guard — is SHARED and must run on BOTH paths. An
  // earlier revision returned early from the default branch and skipped all of
  // it, which leaked integrator windup from an aborted strip into the next
  // strip's first tick and transitioned the state machine one segment early.
  // Do not reintroduce an early return here.
  if (!config_.snap_to_nearest_on_set_plan)
  {
    // current_index_ is already 0 from the reset above.
    RCLCPP_INFO(logger_,
                "FTCController: setPlan with %zu points, starting at idx=0.",
                global_plan_.size());
  }
  else
  {
    try
    {
      const auto base_to_map = tf_buffer_->lookupTransform("map",
                                                           "base_link",
                                                           tf2::TimePointZero,
                                                           tf2::durationFromSec(0.5));
      const double rx = base_to_map.transform.translation.x;
      const double ry = base_to_map.transform.translation.y;

      std::vector<std::pair<double, double>> xy;
      xy.reserve(global_plan_.size());
      for (const auto& p : global_plan_)
      {
        xy.emplace_back(p.pose.position.x, p.pose.position.y);
      }
      current_index_ = static_cast<uint32_t>(ChooseStartIndex(true, xy, rx, ry));

      const double bdx = global_plan_[current_index_].pose.position.x - rx;
      const double bdy = global_plan_[current_index_].pose.position.y - ry;
      const double best_dist = std::hypot(bdx, bdy);
      RCLCPP_INFO(logger_,
                  "FTCController: setPlan with %zu points, starting at idx=%u (%.2fm from robot "
                  "at %.2f,%.2f).",
                  global_plan_.size(),
                  current_index_,
                  best_dist,
                  rx,
                  ry);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(logger_,
                  "FTCController: TF lookup in setPlan failed (%s), starting from idx=0.",
                  ex.what());
      current_index_ = 0;
    }
  }
  // A plan that opens ON a pivot corner (FollowStrip trimmed it there) starts
  // at the corner's outgoing pose, so PRE_ROTATE turns straight to it.
  current_index_ = static_cast<uint32_t>(PivotAwareStartIndex(pivot_corners_, current_index_));
  if (!pivot_corners_.empty())
  {
    RCLCPP_INFO(logger_,
                "FTCController: plan has %zu pivot corner(s) — rotating in place at each.",
                pivot_corners_.size());
  }

  last_time_ = clock_->now();
  current_movement_speed_ = config_.speed_slow;
  stall_time_ = 0.0;
  is_stalled_ = false;
  last_cmd_fwd_speed_ = 0.0;
  cm_held_s_ = 0.0;
  cm_stalled_s_ = 0.0;
  last_cm_hold_arc_.reset();
  is_blade_limited_ = false;
  blade_load_scale_ = 1.0;

  lat_error_ = 0.0;
  lon_error_ = 0.0;
  angle_error_ = 0.0;
  i_lon_error_ = 0.0;
  i_lat_error_ = 0.0;
  i_angle_error_ = 0.0;
  last_lat_error_ = 0.0;
  last_lon_error_ = 0.0;
  last_angle_error_ = 0.0;
  d_lat_filt_ = 0.0;
  d_lon_filt_ = 0.0;
  d_angle_filt_ = 0.0;

  nav_msgs::msg::Path pub_path;

  if (global_plan_.size() > 2)
  {
    // Duplicate last point so the carrot can exactly reach goal.
    global_plan_.push_back(global_plan_.back());
    // Give the second-to-last point the same orientation as the one before it,
    // so the final segment has a well-defined heading.
    global_plan_[global_plan_.size() - 2].pose.orientation =
        global_plan_[global_plan_.size() - 3].pose.orientation;

    pub_path.header = path.header;
    pub_path.poses = global_plan_;
  }
  else
  {
    RCLCPP_WARN(logger_,
                "FTCController: global plan has fewer than 3 poses (%zu) - cancelling.",
                global_plan_.size());
    current_state_ = PlannerState::FINISHED;
    state_entered_time_ = clock_->now();
    ++state_entry_seq_;
  }

  global_plan_pub_->publish(pub_path);

  RCLCPP_INFO(logger_,
              "FTCController: received new global plan with %zu points.",
              path.poses.size());
}

// ── applyBladeLoad ────────────────────────────────────────────────────────────

double FTCController::applyBladeLoad(double target_speed)
{
  bool active = false;
  double rpm = 0.0;
  double age_s = -1.0;  // never received
  {
    std::lock_guard<std::mutex> lock(blade_mutex_);
    active = blade_active_;
    rpm = blade_rpm_;
    if (blade_status_time_.nanoseconds() > 0)
    {
      age_s = (clock_->now() - blade_status_time_).seconds();
    }
  }

  const FtcBladeLoadCfg cfg{config_.blade_load_slowdown_enabled,
                            config_.blade_load_rpm_full,
                            config_.blade_load_rpm_min,
                            config_.blade_load_min_speed_ratio,
                            config_.blade_load_telemetry_max_age_s};
  const bool fresh = age_s >= 0.0 && age_s <= cfg.telemetry_max_age_s;
  const bool armed =
      BladeLoadArmed(blade_spin_up_, active, fresh, rpm, clock_->now().seconds(), cfg);
  const FtcBladeLoadResult out =
      armed ? BladeLoadDecision(target_speed, active, rpm, age_s, config_.stall_crawl_speed, cfg)
            : FtcBladeLoadResult{target_speed, 1.0, false};

  if (out.is_limited != is_blade_limited_)
  {
    if (out.is_limited)
    {
      RCLCPP_INFO(logger_,
                  "FTCController: blade load slowdown ENGAGED — blade %.0f rpm (ramp %.0f..%.0f), "
                  "speed scale %.2f → %.3f m/s.",
                  rpm,
                  cfg.rpm_min,
                  cfg.rpm_full,
                  out.scale,
                  out.target_speed);
    }
    else
    {
      RCLCPP_INFO(logger_,
                  "FTCController: blade load slowdown released — blade %.0f rpm, full speed.",
                  rpm);
    }
  }
  is_blade_limited_ = out.is_limited;
  blade_load_scale_ = out.scale;
  return out.target_speed;
}

// ── setSpeedLimit ─────────────────────────────────────────────────────────────

void FTCController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  if (!std::isfinite(speed_limit))
  {
    RCLCPP_WARN(logger_, "FTCController: ignoring non-finite speed limit.");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    // Zero is Nav2's no-limit sentinel. Preserve legacy negative clears too.
    speed_limit_ = std::max(nav2_costmap_2d::NO_SPEED_LIMIT, speed_limit);
    speed_limit_is_percentage_ = percentage;
  }

  RCLCPP_INFO(logger_,
              "FTCController: speed limit set to %.3f (percentage=%s).",
              speed_limit,
              percentage ? "true" : "false");
}

double FTCController::maxLinearSpeed() const
{
  std::lock_guard<std::mutex> lock(speed_limit_mutex_);
  if (speed_limit_ == nav2_costmap_2d::NO_SPEED_LIMIT)
  {
    return base_max_cmd_vel_speed_;
  }
  const double external_cap =
      speed_limit_is_percentage_
          ? base_max_cmd_vel_speed_ * std::clamp(speed_limit_, 0.0, 100.0) / 100.0
          : speed_limit_;
  return std::min(base_max_cmd_vel_speed_, external_cap);
}

// ── computeVelocityCommands ───────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped FTCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& /*pose*/,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker,
    const nav_msgs::msg::Path& /*transformed_global_plan*/,
    const geometry_msgs::msg::PoseStamped& /*global_goal*/)
{
  // FTC owns the complete coverage path and its monotonic carrot cursor.
  // Do not replace it with the path handler's pruned local path each tick.
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = clock_->now();

  // The mowing geometry this tick's fallback splices are checked against
  // (finalizeSplice). Snapshotted BEFORE departure_lock may be taken: a
  // permitted rotation handoff keeps corner_geometry_mutex_ for the rest of
  // the tick, and the same tick may already plan a fallback in FOLLOWING.
  {
    std::lock_guard lock(corner_geometry_mutex_);
    tick_corner_geometry_ = corner_geometry_;
  }

  // A successful rotation handoff and its output selection are ordered before
  // complete geometry replacement. Refused handoffs release this lock and keep
  // the rotation state/deadline so a later publication can be re-evaluated.
  std::unique_lock departure_lock(corner_geometry_mutex_, std::defer_lock);

  // Apply the hard linear cap at every output boundary, after movement floors
  // and including reverse escape / turn-fallback commands that bypass the PID.
  const auto bounded_command = [this, &cmd_vel, &departure_lock]()
  {
    const double max_speed = maxLinearSpeed();
    cmd_vel.twist.linear.x = std::clamp(cmd_vel.twist.linear.x, -max_speed, max_speed);
    const auto approach = stopCornerApproach();
    if (cmd_vel.twist.linear.x > 0.0 && approach)
    {
      // Obstacles can release ownership after errors were projected. Wait one
      // tick for a coherent point/cap instead of emitting an uncapped restart.
      if (!pivot_approach_speed_limit_ || !pivot_approach_response_limit_)
        cmd_vel.twist = geometry_msgs::msg::Twist{};
      else
      {
        double speed_limit = *pivot_approach_speed_limit_;
        double response_limit = *pivot_approach_response_limit_;
        const auto pose = pivotMapPose();
        const auto& tip = global_plan_[*approach].pose.position;
        const double radius = std::hypot(tip.x - pose.position.x, tip.y - pose.position.y);
        transit::Ring body;
        for (const auto& p : costmap_ros_->getRobotFootprint())
          body.push_back({p.x, p.y});
        if (!departure_lock.owns_lock())
          departure_lock.lock();
        // A certified disk can provide more stopping travel than either
        // outgoing coordinate plane. Keep B's cached bounds and exact fallback.
        if (corner_geometry_)
        {
          const auto budget =
              corner_geometry_->stoppingBudget({pose.position.x, pose.position.y}, radius, body);
          if (budget)
          {
            const double certified_speed =
                *budget / (config_.pivot_coast_time_bound_s + 3 * command_horizon_s_);
            const double certified_response =
                *budget / (config_.pivot_coast_time_bound_s + 2 * command_horizon_s_);
            if (certified_speed > speed_limit && certified_response > response_limit)
            {
              speed_limit = certified_speed;
              response_limit = certified_response;
            }
          }
        }
        if (!(std::abs(last_measured_fwd_speed_) <= response_limit))
          cmd_vel.twist = geometry_msgs::msg::Twist{};
        else
          cmd_vel.twist.linear.x = std::min(cmd_vel.twist.linear.x, speed_limit);
      }
    }
    // Rotation outputs stay inside the checked direction and arc even with
    // adverse I/D, oscillation override or recovery slew. T is the declared
    // intended interval, never the preceding callback's measured dt.
    if ((current_state_ == PlannerState::PRE_ROTATE || current_state_ == PlannerState::PIVOT) &&
        cmd_vel.twist.angular.z != 0)
    {
      std::unique_lock lock(corner_geometry_mutex_, std::defer_lock);
      if (!departure_lock.owns_lock())
        lock.lock();
      const double turn = checked_corner_turn_.value_or(0.0);
      if (!checked_corner_geometry_ || checked_corner_geometry_ != corner_geometry_ ||
          !(cmd_vel.twist.angular.z * turn > 0))
        cmd_vel.twist = geometry_msgs::msg::Twist{};
      else
        cmd_vel.twist.angular.z = std::copysign(std::min(std::abs(cmd_vel.twist.angular.z),
                                                         std::abs(turn) / command_horizon_s_),
                                                turn);
    }
    last_cmd_fwd_speed_ = cmd_vel.twist.linear.x;
    return cmd_vel;
  };

  applyPendingConfig();
  const rclcpp::Time now = clock_->now();
  const double dt = (now - last_time_).seconds();
  last_time_ = now;

  // Guard against pathological dt:
  //   * Upper bound 0.5 s — prevents an integration jump after a pause
  //     (e.g. preempted action, then resumed seconds later).
  //   * Lower bound 0.01 s — `setPlan` resets `last_time_ = clock_->now()`
  //     so the very first computeVelocityCommands call after a new plan
  //     sees dt ≈ 0. Without a floor, the PID derivative becomes
  //     `(error - 0) / 0 = ±inf`, the resulting cmd_vel is NaN, and
  //     controller_server logs `Velocity message contains NaNs or Infs!`
  //     and silently drops it — leaving the strip un-driven for the
  //     entire goal_timeout window.
  const double safe_dt = std::clamp(dt, 0.01, kMaxControlIntervalSec);

  // Cache the measured forward speed (odom feedback) for update_control_point's
  // anti-wheelspin stall detection.
  last_measured_fwd_speed_ = velocity.linear.x;

  // Remember the ground the robot has just driven (odom frame): a reverse may only undo that.
  if (config_.reverse_boundary_safe && !reverse_escape_active_ && !turnFallbackReversing())
  {
    geometry_msgs::msg::PoseStamped crumb_pose;
    if (costmap_ros_->getRobotPose(crumb_pose))
    {
      reverse_crumbs_.add({crumb_pose.pose.position.x,
                           crumb_pose.pose.position.y,
                           tf2::getYaw(crumb_pose.pose.orientation)});
    }
  }

  if (is_crashed_)
  {
    throw nav2_core::ControllerException("FTCController: robot has crashed / collision detected.");
  }

  if (current_state_ == PlannerState::FINISHED)
  {
    // Zero velocity — goal reached.
    return bounded_command();
  }

  // NOTE: do NOT reset the goal checker here. controller_server already
  // resets it once per new goal. Resetting every tick zeroed
  // PathProgressGoalChecker::max_reached_index_ each cycle, and isGoalReached
  // can only re-advance it by max_idx_advance_per_call_ (10) poses per call —
  // so live progress was pinned at <=10/(n-1), never reaching the 0.95
  // threshold on any real coverage path. The coverage goal checker could
  // therefore never report success during FOLLOWING; strips only terminated
  // via FTC's own state-machine timeout/abort. The stateless SimpleGoalChecker
  // (transit FollowPath) is unaffected either way. (void) the unused arg.
  (void)goal_checker;

  pivot_pose_epoch_.reset();
  checked_corner_geometry_.reset();
  checked_corner_turn_.reset();

  // 1. Advance the carrot; compute lat/lon/angle errors in base_link.
  update_control_point(safe_dt);

  RCLCPP_INFO_THROTTLE(logger_,
                       *clock_,
                       2000,
                       "FTCController: state=%d idx=%u/%zu dt=%.4f "
                       "lat=%.3f lon=%.3f ang=%.3f(deg=%.1f) pos=(%.2f,%.2f)",
                       static_cast<int>(current_state_),
                       current_index_,
                       global_plan_.size(),
                       safe_dt,
                       lat_error_,
                       lon_error_,
                       angle_error_,
                       angle_error_ * 180.0 / M_PI,
                       local_control_point_.translation().x(),
                       local_control_point_.translation().y());

  // 2. Update the state machine.
  PlannerState new_state = update_planner_state();
  // Timeouts are terminal before any recovery path can emit another command.
  if (is_crashed_)
  {
    throw nav2_core::ControllerException(
        "FTCController: terminal failure in planner state machine.");
  }
  const bool rotating_state =
      current_state_ == PlannerState::PRE_ROTATE || current_state_ == PlannerState::PIVOT;
  const double measured_translation = std::hypot(velocity.linear.x, velocity.linear.y);
  if (rotating_state && measured_translation <= kStoppedTranslationMps)
    rotation_settled_entry_ = state_entry_seq_;
  const bool settled_this_episode =
      rotation_settled_entry_ && *rotation_settled_entry_ == state_entry_seq_;
  const bool settling_rotation =
      rotating_state &&
      !(measured_translation <=
        (settled_this_episode ? kRotationTranslationNoiseMps : kStoppedTranslationMps));
  if ((current_state_ == PlannerState::PRE_ROTATE || current_state_ == PlannerState::PIVOT) &&
      new_state == PlannerState::FOLLOWING)
  {
    departure_lock.lock();
    const char* hold = settling_rotation ? nullptr
                       : !corner_geometry_ || !corner_geometry_->valid
                           ? "no valid mowing geometry to depart against"
                       : !rotationDeparturePermitted(*corner_geometry_)
                           ? "the straight departure after the rotation leaves the mowing area or "
                             "is blocked"
                           : nullptr;
    if (settling_rotation || hold != nullptr)
    {
      if (hold != nullptr)
      {
        noteRotationHold(hold);
      }
      new_state = current_state_;
      departure_lock.unlock();
    }
  }
  const bool entering_pivot =
      new_state == PlannerState::PIVOT && current_state_ != PlannerState::PIVOT;
  if (new_state != current_state_)
  {
    RCLCPP_INFO(logger_,
                "FTCController: state transition %d -> %d (idx=%u, angle_err=%.3f deg).",
                static_cast<int>(current_state_),
                static_cast<int>(new_state),
                current_index_,
                angle_error_ * 180.0 / M_PI);
    if (new_state == PlannerState::PIVOT)
    {
      enterPivot();
    }
    else if (current_state_ == PlannerState::PIVOT && new_state == PlannerState::FOLLOWING)
    {
      leavePivot();
    }
    state_entered_time_ = clock_->now();
    ++state_entry_seq_;
    current_state_ = new_state;
    rotation_hold_reason_.clear();
  }

  // A running turn fallback ends once FOLLOWING has reached its rejoin pose,
  // and is bounded in time (throws past turn_fallback_timeout_s).
  if (turn_fallback_.phase != TurnFallbackPhase::kIdle)
  {
    turnFallbackProgress();
  }

  // The tick that enters a pivot only STOPS. Its heading error was measured
  // against the corner's incoming pose (update_control_point ran before the
  // transition); the rotation, and the sweep check that must precede it, start
  // next tick from the outgoing pose enterPivot() retargeted to.
  if (entering_pivot)
  {
    return bounded_command();  // zero velocity
  }

  // Retain the original PRE/PIVOT timer while measured translation settles.
  // This applies to both obstacle overlays and never enters FOLLOWING to wait.
  if (settling_rotation)
    return bounded_command();

  // Rotation/departure also applies with the shipped no-LiDAR overlay. Do
  // not enable deviation, its lattice or its sensors to reach this gate.
  if (current_state_ == PlannerState::PRE_ROTATE || current_state_ == PlannerState::PIVOT)
  {
    if (!cornerRotationGate(safe_dt))
      return bounded_command();
    calculate_velocity_commands(safe_dt, cmd_vel);
    return bounded_command();
  }

  // 3. Collision check + lateral-deviation update.
  // When enable_obstacle_deviation is true, we never throw on a lookahead
  // collision: instead the carrot is laterally offset until the obstacle
  // clears (see updateLateralDeviation). Throws are reserved for the
  // hard-fail case where the offset would exceed max_lateral_deviation —
  // BT then aborts the strip and requests the next one.
  if (config_.enable_obstacle_deviation)
  {
    // SAFETY (SAFETY_REVIEW_2026-07-23 F-C1): before anything else, check the
    // robot's ACTUAL current footprint. The deviation machinery below only
    // samples path poses AHEAD — during PRE_ROTATE pivots, stall-crawl pushes
    // and lateral blends nothing ever asked "is my body in a lethal cell right
    // now?", leaving collision_monitor as the sole guard (and it misses thin /
    // sub-scan-plane obstacles). A true-lethal cell INSIDE the chassis outline
    // means actual or imminent contact: hold zero velocity via the standard
    // wait gate (transient scan noise clears within obstacle_wait_timeout_s;
    // a real contact throws → the BT aborts the strip and the detour net
    // routes around). Skipped while reverse-escaping — backing OUT of the
    // contact is exactly what we want then.
    if (!reverse_escape_active_ && !turnFallbackReversing() && currentBodyInLethal())
    {
      waitOrThrowForObstacle("chassis footprint overlaps a lethal obstacle cell");
      return bounded_command();  // zero-velocity hold (waitOrThrow throws after timeout)
    }
    // Turn fallback, straight reverse before its first pivot: a pure straight
    // reverse (like the reverse-escape), rear footprint probed every tick,
    // bounded in distance and time; then the pivots are spliced in.
    if (turnFallbackReversing())
    {
      turnFallbackReverseTick(safe_dt, cmd_vel);
      return bounded_command();
    }
    updateLateralDeviation(safe_dt);
    // The lattice found no profile and a turn fallback engaged instead of the
    // WEDGED path: this tick only stops; its reverse or first pivot starts
    // next tick.
    if (turn_fallback_engaged_now_)
    {
      turn_fallback_engaged_now_ = false;
      return bounded_command();  // zero velocity
    }
    // updateLateralDeviation engaged the bounded reverse-escape sub-state
    // (both sides of an obstacle blocked / skirt over cap, rear footprint
    // clear). Emit a PURE STRAIGHT reverse (no rotation, distance hard-capped
    // in updateLateralDeviation) bypassing the PID, then return. This is the
    // ONLY place FTC drives backwards and it is a distinct escape sub-state —
    // normal following stays forward_only.
    if (reverse_escape_active_)
    {
      cmd_vel.twist.linear.x = -config_.obstacle_reverse_speed_mps;
      cmd_vel.twist.angular.z = 0.0;
      return bounded_command();
    }
    // updateLateralDeviation flipped on the wait-before-abort gate (the
    // costmap is blocked beyond max_lateral_deviation and we're holding
    // for obstacle_wait_timeout_s). Hold zero velocity until either the
    // costmap clears or the helper throws on timeout.
    if (obstacle_waiting_)
    {
      return bounded_command();
    }
    // Every check above reads the local costmap and says "clear", yet
    // collision_monitor (raw /scan_collision points, downstream of us) can
    // hold the wheels at zero. Following a stopped robot only ends on the
    // progress checker, an abort FollowStrip cannot tell from a non-obstacle
    // one (field 2026-10-10, the dock spike of the boundary). Hand it to the
    // WEDGED handling instead: bounded reverse-escape, then hold, then an
    // obstacle-marked abort.
    if (turn_fallback_.phase == TurnFallbackPhase::kIdle && collisionMonitorHoldConfirmed(safe_dt))
    {
      const double arc = planArcLength(current_index_);
      if (CmHoldRepeats(last_cm_hold_arc_, arc))
      {
        is_crashed_ = true;
        throw nav2_core::ControllerException(
            std::string(mowgli_interfaces::ftc_abort_reason::kObstacleAbortMarker) +
            "FTCController: collision_monitor held the robot again within " +
            std::to_string(kCmHoldRepeatProgressM) + " m of path, aborting strip.");
      }
      last_cm_hold_arc_ = arc;
      reverseEscapeOrWait("collision_monitor holds the robot while the path reads clear",
                          costmap_ros_->getRobotFootprint(),
                          safe_dt);
      if (reverse_escape_active_)
      {
        cmd_vel.twist.linear.x = -config_.obstacle_reverse_speed_mps;
        cmd_vel.twist.angular.z = 0.0;
      }
      return bounded_command();
    }
    applyLateralDeviationToCarrot();
    // Re-derive the base_link PID errors from the NOW-deviated carrot.
    // update_control_point() (called above) computed lat/lon/angle from the
    // un-deviated carrot; applyLateralDeviationToCarrot() then shifted
    // current_control_point_, but calculate_velocity_commands consumes the
    // already-extracted error members. Without re-projecting, the lateral
    // offset never reaches the PID and obstacle deviation was a viz-only
    // no-op (the robot drove the nominal path straight at the obstacle).
    // The offset is a pure translation, so heading (angle_error_) is
    // unchanged — only lon/lat translation errors move.
    if (lateral_deviation_ != 0.0)
    {
      try
      {
        const auto map_to_base = tf_buffer_->lookupTransform("base_link",
                                                             "map",
                                                             tf2::TimePointZero,
                                                             tf2::durationFromSec(1.0));
        tf2::doTransform(current_control_point_, local_control_point_, map_to_base);
        lat_error_ = local_control_point_.translation().y();
        lon_error_ = local_control_point_.translation().x();
      }
      catch (const tf2::TransformException& ex)
      {
        throw nav2_core::ControllerException(
            std::string("FTCController: TF lookup failed (deviation reproject): ") + ex.what());
      }
    }
  }
  else if (checkCollision(config_.obstacle_lookahead))
  {
    is_crashed_ = true;
    // Issue #743: marked as obstacle-caused (mowgli_interfaces/ftc_abort_
    // reason.hpp) so FollowStrip's detour confirmation does not have to
    // re-derive "was this an obstacle" from a different costmap/body model.
    throw nav2_core::ControllerException(
        std::string(mowgli_interfaces::ftc_abort_reason::kObstacleAbortMarker) +
        "FTCController: collision detected along lookahead path.");
  }

  // 4. PID velocity computation.
  calculate_velocity_commands(safe_dt, cmd_vel);

  if (is_crashed_)
  {
    throw nav2_core::ControllerException(
        "FTCController: collision detected during velocity computation.");
  }

  return bounded_command();
}

// ── State machine ─────────────────────────────────────────────────────────────

double FTCController::time_in_current_state() const
{
  return (clock_->now() - state_entered_time_).seconds();
}

FTCController::PlannerState FTCController::update_planner_state()
{
  switch (current_state_)
  {
    case PlannerState::PRE_ROTATE:
    {
      if (time_in_current_state() > config_.goal_timeout)
      {
        RCLCPP_ERROR(logger_,
                     "FTCController: timeout (%.1fs) in PRE_ROTATE%s%s.",
                     config_.goal_timeout,
                     rotation_hold_reason_.empty() ? "" : " — held: ",
                     rotation_hold_reason_.c_str());
        is_crashed_ = true;
        return PlannerState::FINISHED;
      }
      // Use the GEOMETRIC (wrapped to (-π, π]) angle, not the unwrap
      // accumulator. The accumulator can drift to ±2π+ if the robot
      // overshoots and oscillates during PRE_ROTATE — staying gated on
      // it would keep PRE_ROTATE alive forever even after the robot is
      // physically aligned with the carrot.
      const double angle_wrapped = std::atan2(std::sin(angle_error_), std::cos(angle_error_));
      if (std::abs(angle_wrapped) * (180.0 / M_PI) < config_.max_goal_angle_error)
      {
        RCLCPP_INFO(logger_, "FTCController: PRE_ROTATE done, starting FOLLOWING.");
        return PlannerState::FOLLOWING;
      }
    }
    break;

    case PlannerState::FOLLOWING:
    {
      const auto corner = nextPivotCorner();
      const auto approach = stopCornerApproach();
      if (pivot_approach_start_ && (!corner || pivot_approach_start_->first != *corner))
        pivot_approach_start_.reset();
      if (corner && !pivot_approach_start_)
      {
        // The carrot can enter the approach window well before the body.
        // local_control_point_ is the corner transformed at the same supported
        // pose epoch used for the approach speed and response limits.
        const double window =
            maxLinearSpeed() * (config_.pivot_coast_time_bound_s + 3 * command_horizon_s_);
        const double distance = local_control_point_.translation().norm();
        if (plan_header_.frame_id != "map" ||
            (approach && (!std::isfinite(distance) || distance <= window)))
          pivot_approach_start_ = std::pair{*corner, clock_->now()};
      }
      // Once armed, leaving the window or temporarily losing approach
      // ownership cannot extend the deadline for the same corner.
      if (pivot_approach_start_)
      {
        if ((clock_->now() - pivot_approach_start_->second).seconds() > config_.goal_timeout)
          throw nav2_core::ControllerException("FTCController: timeout approaching pivot corner.");
      }
      const double distance = local_control_point_.translation().norm();
      if (distance > config_.max_follow_distance)
      {
        // Instead of aborting, try nearest-point recovery: find the closest
        // path point to the robot and resync the carrot there.
        try
        {
          const auto base_to_map = tf_buffer_->lookupTransform("map",
                                                               "base_link",
                                                               tf2::TimePointZero,
                                                               tf2::durationFromSec(0.5));
          const double rx = base_to_map.transform.translation.x;
          const double ry = base_to_map.transform.translation.y;

          // Bounded to a window of PATH LENGTH around the carrot: the nearest
          // pose of the whole plan is, on concentric rings 0.16 m apart, a pose
          // on a NEIGHBOURING ring tens of metres further on (ftc_resync.hpp).
          // And bounded to the current LEG between pivot corners: re-anchoring
          // past a corner would skip its pivot (the robot would then chase a
          // carrot 90° off its heading), re-anchoring before the last one would
          // pivot twice.
          const auto [leg_first, leg_last] =
              PivotLeg(pivot_corners_, current_index_, global_plan_.size());
          std::vector<std::pair<double, double>> plan_xy;
          plan_xy.reserve(leg_last - leg_first + 1);
          for (std::size_t i = leg_first; i <= leg_last; ++i)
          {
            plan_xy.emplace_back(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y);
          }
          const ResyncResult resync =
              FindResyncIndex(plan_xy,
                              current_index_ - leg_first,
                              rx,
                              ry,
                              kResyncWindowFactor * config_.max_follow_distance);
          const double best_dist = resync.distance_m;
          // The carrot interpolates towards index + 1, so never anchor on the
          // very last pose.
          const uint32_t best_idx = static_cast<uint32_t>(
              std::min(leg_first + resync.index,
                       global_plan_.size() >= 2 ? global_plan_.size() - 2 : 0));
          if (best_dist < config_.max_follow_distance)
          {
            RCLCPP_WARN(logger_,
                        "FTCController: resyncing carrot idx %u->%u (%.3fm away, was %.3fm).",
                        static_cast<unsigned>(current_index_),
                        best_idx,
                        best_dist,
                        distance);
            current_index_ = best_idx;
            current_progress_ = 0.0;
            tf2::fromMsg(global_plan_[current_index_].pose, current_control_point_);
          }
          else
          {
            RCLCPP_ERROR(logger_,
                         "FTCController: robot too far from plan (%.3f > %.3f). Aborting.",
                         best_dist,
                         config_.max_follow_distance);
            is_crashed_ = true;
            return PlannerState::FINISHED;
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_ERROR(logger_, "FTCController: TF lookup failed during resync: %s", ex.what());
          is_crashed_ = true;
          return PlannerState::FINISHED;
        }
      }
      // Pivot corner: the carrot is capped on its incoming pose (see
      // update_control_point), so the robot drives straight at it; once
      // base_link is there, stop and rotate. Never mid reverse-escape or
      // mid obstacle hold — those own the output until they release.
      if (corner.has_value() && current_index_ == *corner && !reverse_escape_active_ &&
          !obstacle_waiting_ && !turnFallbackReversing() &&
          PivotArrived(local_control_point_.translation().x(),
                       local_control_point_.translation().y(),
                       kPivotArrivalToleranceM))
      {
        return PlannerState::PIVOT;
      }
      if (current_index_ == global_plan_.size() - 2)
      {
        RCLCPP_INFO(logger_, "FTCController: switching to WAITING_FOR_GOAL_APPROACH.");
        return PlannerState::WAITING_FOR_GOAL_APPROACH;
      }
    }
    break;

    case PlannerState::PIVOT:
    {
      if (time_in_current_state() > config_.goal_timeout)
      {
        // Same bound and same escalation as PRE_ROTATE: the BT sees an aborted
        // goal and runs its detour / resume logic.
        RCLCPP_ERROR(logger_,
                     "FTCController: timeout (%.1fs) pivoting at corner idx=%u%s%s.",
                     config_.goal_timeout,
                     current_index_,
                     rotation_hold_reason_.empty() ? "" : " — held: ",
                     rotation_hold_reason_.c_str());
        is_crashed_ = true;
        return PlannerState::FINISHED;
      }
      if (PivotAligned(angle_error_, config_.pivot_angle_tolerance_deg * (M_PI / 180.0)))
      {
        return PlannerState::FOLLOWING;
      }
    }
    break;

    case PlannerState::WAITING_FOR_GOAL_APPROACH:
    {
      const double distance = local_control_point_.translation().norm();
      if (time_in_current_state() > config_.goal_timeout)
      {
        // Approach timed out — robot didn't reach within max_goal_distance_error.
        // Mark the controller as crashed so the next computeVelocityCommands
        // throws a ControllerException, which the action server reports as a
        // failure. This unblocks the BT (FollowStrip → aborted → SKIP_SEGMENT
        // → next strip). Without it the FTC would silently sit in FINISHED
        // emitting zero velocity, leaving the action open while the goal
        // checker waits for a tolerance the robot will never meet.
        RCLCPP_WARN(logger_,
                    "FTCController: timeout in WAITING_FOR_GOAL_APPROACH (dist=%.3fm > "
                    "max_goal_distance_error=%.3fm); aborting strip.",
                    distance,
                    config_.max_goal_distance_error);
        is_crashed_ = true;
        return PlannerState::FINISHED;
      }
      if (distance < config_.max_goal_distance_error)
      {
        RCLCPP_INFO(logger_, "FTCController: goal position reached, entering POST_ROTATE.");
        return PlannerState::POST_ROTATE;
      }
    }
    break;

    case PlannerState::POST_ROTATE:
    {
      if (time_in_current_state() > config_.goal_timeout)
      {
        // Same rationale as the WAITING_FOR_GOAL_APPROACH timeout: bubble the
        // failure up through ControllerException so the BT sees an aborted
        // action and progresses. Otherwise FTC would idle in FINISHED with
        // the goal-checker still unhappy about the angle tolerance.
        RCLCPP_WARN(logger_, "FTCController: timeout in POST_ROTATE; aborting strip.");
        is_crashed_ = true;
        return PlannerState::FINISHED;
      }
      // Use the GEOMETRIC (wrapped to (-π, π]) angle, not the unwrap
      // accumulator — same fix as PRE_ROTATE above. The accumulator can
      // drift past ±π if the robot oscillates while settling on the final
      // heading, which would keep POST_ROTATE alive past the tolerance even
      // when the robot is physically aligned with the goal pose.
      const double angle_wrapped = std::atan2(std::sin(angle_error_), std::cos(angle_error_));
      if (std::abs(angle_wrapped) * (180.0 / M_PI) < config_.max_goal_angle_error)
      {
        RCLCPP_INFO(logger_, "FTCController: POST_ROTATE done.");
        return PlannerState::FINISHED;
      }
    }
    break;

    case PlannerState::FINISHED:
      break;
  }

  return current_state_;
}

// ── Control point advancement (carrot) ───────────────────────────────────────

double FTCController::distanceLookahead() const
{
  if (global_plan_.size() < 2)
  {
    return 0.0;
  }

  const Eigen::Quaternion<double> current_rot(current_control_point_.linear());
  double lookahead_distance = 0.0;
  Eigen::Affine3d last_straight_point = current_control_point_;

  for (uint32_t i = current_index_ + 1; i < global_plan_.size(); ++i)
  {
    Eigen::Affine3d current_point;
    tf2::fromMsg(global_plan_[i].pose, current_point);

    const Eigen::Quaternion<double> rot2(current_point.linear());

    if (lookahead_distance > config_.speed_fast_threshold ||
        std::abs(rot2.angularDistance(current_rot)) >
            config_.speed_fast_threshold_angle * (M_PI / 180.0))
    {
      break;
    }

    lookahead_distance += (current_point.translation() - last_straight_point.translation()).norm();
    last_straight_point = current_point;
  }

  return lookahead_distance;
}

void FTCController::update_control_point(double dt)
{
  if (!pivot_corners_.empty() && plan_header_.frame_id != "map")
    throw nav2_core::ControllerException("FTCController: pivot requires a map-frame path.");
  switch (current_state_)
  {
    case PlannerState::PRE_ROTATE:
    case PlannerState::PIVOT:
      // Rotate in place towards the pose under the cursor: the plan start, or
      // a pivot corner's outgoing-heading pose (enterPivot moved the cursor).
      tf2::fromMsg(global_plan_[current_index_].pose, current_control_point_);
      break;

    case PlannerState::FOLLOWING:
    {
      // Rebuild the carrot from the plan (SLERP between the two bounding poses).
      // EVERY exit of this case must go through it: applyLateralDeviationToCarrot
      // shifts current_control_point_ in place, so a tick that leaves the
      // previous value standing re-applies the skirt offset on top of itself.
      const auto interpolate_carrot = [this]()
      {
        Eigen::Affine3d currentPose, nextPose;
        tf2::fromMsg(global_plan_[current_index_].pose, currentPose);
        tf2::fromMsg(global_plan_[current_index_ + 1].pose, nextPose);

        const Eigen::Quaternion<double> rot1(currentPose.linear());
        const Eigen::Quaternion<double> rot2(nextPose.linear());
        const Eigen::Vector3d trans1 = currentPose.translation();
        const Eigen::Vector3d trans2 = nextPose.translation();

        Eigen::Affine3d result;
        result.translation() = (1.0 - current_progress_) * trans1 + current_progress_ * trans2;
        result.linear() = rot1.slerp(current_progress_, rot2).toRotationMatrix();
        current_control_point_ = result;
      };

      // A zero-velocity obstacle hold owns the output. Keep the virtual carrot
      // fixed as well; otherwise it walks away from the stationary robot and
      // creates a catch-up surge as soon as one scan looks clear. The same
      // while a turn fallback backs up before its first pivot.
      if (obstacle_waiting_ || turnFallbackReversing())
      {
        current_movement_speed_ = 0.0;
        stall_time_ = 0.0;
        is_stalled_ = false;
        interpolate_carrot();
        break;
      }

      // Don't advance the carrot once it leads the robot by more than the
      // longitudinal loop needs. Something the controller cannot see may be
      // holding the chassis (dig hard-stop + reverse, collision_monitor
      // slowdown); a far carrot is then steered at along a chord that cuts the
      // INSIDE of a curved path. The cap is derived, see ftc_carrot_lead.hpp.
      //
      // HOLDING the carrot must not skip the interpolation at the bottom of this
      // case: current_control_point_ still carries LAST tick's lateral deviation
      // (applyLateralDeviationToCarrot shifts it in place), so leaving it
      // untouched made the skirt offset accumulate tick after tick — 0.45 m per
      // 100 ms — until the carrot sat > max_follow_distance away and the resync
      // re-anchored on a neighbouring ring (2026-09-17: 96 m of plan skipped).
      const double carrot_max_lead =
          CarrotMaxLead(config_.carrot_max_lead, config_.speed_fast, config_.kp_lon);
      const bool hold_carrot =
          CarrotLeadExceeded(local_control_point_.translation().x(), carrot_max_lead);

      // Compute target speed based on how much straight path lies ahead.
      const double straight_dist = distanceLookahead();
      double target_speed =
          (straight_dist >= config_.speed_fast_threshold) ? config_.speed_fast : config_.speed_slow;
      // A mower obeying a low external cap is not slipping. Keep the carrot
      // and stall detector's expected speed within the same hard limit.
      const double max_speed = maxLinearSpeed();
      target_speed = std::min(target_speed, max_speed);
      current_movement_speed_ = std::min(current_movement_speed_, max_speed);

      // Anti-wheelspin traction control. If the carrot is already commanding a
      // meaningful forward speed but the robot's ACTUAL forward speed (odom
      // feedback) stays well below it, the wheels are slipping or the chassis is
      // blocked. Rather than ramp to speed_fast and floor it — which spins the
      // wheels and digs holes in soft turf (operator report) — ease the target
      // down to a slow crawl until traction returns. Easing the carrot's target
      // speed alone is not enough: the lon PID would still be floored up to
      // min_speed_mps and push into the obstruction, so while `in_stall` we also
      // freeze the carrot (below) and cap the commanded velocity at the crawl
      // speed (calculate_velocity_commands). See ftc_stall.hpp for the pure
      // decision function + unit tests (test_ftc_stall.cpp).
      const FtcStallCfg stall_cfg{config_.stall_speed_ratio,
                                  config_.stall_grace_s,
                                  config_.stall_crawl_speed};
      const FtcStallResult stall = StallDecision(target_speed,
                                                 current_movement_speed_,
                                                 last_measured_fwd_speed_,
                                                 dt,
                                                 stall_cfg,
                                                 stall_time_);
      target_speed = stall.target_speed;
      // calculate_velocity_commands reads this to cap the commanded velocity
      // at the crawl speed (bypassing the min_speed_mps floor) while blocked.
      is_stalled_ = stall.in_stall;

      // Blade-load slowdown: scale the (possibly stall-eased) target by the
      // blade motor's RPM sag so a bogged blade gets fed slower. Sets
      // is_blade_limited_ for calculate_velocity_commands. Pure decision +
      // unit tests in ftc_blade_load.hpp / test_ftc_blade_load.cpp.
      target_speed = std::min(applyBladeLoad(target_speed), max_speed);

      // Smooth speed ramp (acceleration / deceleration).
      if (target_speed > current_movement_speed_)
      {
        current_movement_speed_ += dt * config_.acceleration;
        if (current_movement_speed_ > target_speed)
        {
          current_movement_speed_ = target_speed;
        }
      }
      else if (target_speed < current_movement_speed_)
      {
        current_movement_speed_ -= dt * config_.acceleration;
        if (current_movement_speed_ < target_speed)
        {
          current_movement_speed_ = target_speed;
        }
      }

      double distance_to_move = dt * current_movement_speed_;
      double angle_to_move = dt * config_.speed_angular * (M_PI / 180.0);

      // While stalled (blocked or slipping) freeze the carrot in place. The
      // robot isn't moving, so advancing the carrot would only grow lon_error
      // ahead of the chassis and make the lon PID push harder into the
      // obstruction — the runaway that dug holes.
      if (is_stalled_ || hold_carrot)
      {
        distance_to_move = 0.0;
        angle_to_move = 0.0;
      }

      // Advance the carrot along path segments — but never past the next pivot
      // corner: the carrot stops ON the corner's incoming pose so the robot
      // drives straight to it and pivots there (update_planner_state). A
      // carrot allowed round a zero-radius corner is what made the heading
      // error change side and the angular command flip at its clamp
      // (2026-09-09: 29 sign flips in 3.3 s).
      const std::size_t carrot_cap =
          std::min<std::size_t>(global_plan_.size() - 2,
                                nextPivotCorner().value_or(global_plan_.size() - 2));
      Eigen::Affine3d nextPose, currentPose;
      while (angle_to_move > 0.0 && distance_to_move > 0.0 && current_index_ < carrot_cap)
      {
        tf2::fromMsg(global_plan_[current_index_].pose, currentPose);
        tf2::fromMsg(global_plan_[current_index_ + 1].pose, nextPose);

        const double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

        const Eigen::Quaternion<double> current_rot(currentPose.linear());
        const Eigen::Quaternion<double> next_rot(nextPose.linear());
        const double pose_distance_angular = current_rot.angularDistance(next_rot);

        if (pose_distance <= 0.0)
        {
          RCLCPP_WARN(logger_, "FTCController: skipping duplicate path point.");
          ++current_index_;
          continue;
        }

        const double remaining_dist = pose_distance * (1.0 - current_progress_);
        const double remaining_ang = pose_distance_angular * (1.0 - current_progress_);

        if (remaining_dist < distance_to_move && remaining_ang < angle_to_move)
        {
          // Consume this segment completely and move to the next.
          current_progress_ = 0.0;
          ++current_index_;
          distance_to_move -= remaining_dist;
          angle_to_move -= remaining_ang;
        }
        else
        {
          // Partial advancement within this segment.
          const double progress_distance =
              (pose_distance * current_progress_ + distance_to_move) / pose_distance;
          const double progress_angle =
              (pose_distance_angular * current_progress_ + angle_to_move) / pose_distance_angular;

          current_progress_ = std::min(progress_distance, progress_angle);
          if (current_progress_ > 1.0)
          {
            RCLCPP_WARN(logger_, "FTCController: carrot progress > 1.0 (%.4f).", current_progress_);
          }
          distance_to_move = 0.0;
          angle_to_move = 0.0;
        }
      }

      interpolate_carrot();
    }
    break;

    case PlannerState::POST_ROTATE:
      tf2::fromMsg(global_plan_.back().pose, current_control_point_);
      break;

    case PlannerState::WAITING_FOR_GOAL_APPROACH:
      // Carrot stays at the last interpolated position.
      break;

    case PlannerState::FINISHED:
      break;
  }

  // Visualise the carrot in the map frame.
  {
    geometry_msgs::msg::PoseStamped viz;
    viz.header.frame_id = global_plan_[current_index_].header.frame_id;
    viz.header.stamp = clock_->now();
    viz.pose = tf2::toMsg(current_control_point_);
    global_point_pub_->publish(viz);
  }

  pivot_approach_speed_limit_.reset();
  pivot_approach_response_limit_.reset();
  const auto approach = stopCornerApproach(current_state_ == PlannerState::PRE_ROTATE);
  const auto corner = nextPivotCorner();
  const bool corner_pose = approach || current_state_ == PlannerState::PRE_ROTATE ||
                           current_state_ == PlannerState::PIVOT ||
                           (corner && current_index_ == *corner);
  Eigen::Affine3d target = current_control_point_;
  if (approach)
    tf2::fromMsg(global_plan_[*approach].pose, target);
  try
  {
    geometry_msgs::msg::TransformStamped map_to_base;
    if (corner_pose)
    {
      const auto latest = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
      const auto now = clock_->now();
      const rclcpp::Time stamp(latest.header.stamp, now.get_clock_type());
      const auto epoch = stamp.nanoseconds() == 0 ? now : std::min(now, stamp);
      if ((now - epoch).seconds() > kMaxControlIntervalSec)
        throw nav2_core::ControllerException("FTCController: stale pivot pose.");
      map_to_base = tf_buffer_->lookupTransform("base_link", "map", epoch);
      pivot_pose_epoch_ = epoch;
    }
    else
      map_to_base = tf_buffer_->lookupTransform("base_link",
                                                "map",
                                                tf2::TimePointZero,
                                                tf2::durationFromSec(1.0));
    tf2::doTransform(target, local_control_point_, map_to_base);
    if (approach)
    {
      // Invert this same transform: arrival, point bearing and closing travel
      // must describe one actual pose epoch, never a forecast or map/odom mix.
      tf2::Transform base_to_map;
      tf2::fromMsg(map_to_base.transform, base_to_map);
      base_to_map = base_to_map.inverse();
      const double travel = stopCornerTravel(*approach,
                                             base_to_map.getOrigin().x(),
                                             base_to_map.getOrigin().y(),
                                             tf2::getYaw(base_to_map.getRotation()));
      // One future sample beyond the 2*T+C reserve avoids alternating STOP
      // pulses as a slowing request becomes the next tick's measured speed.
      pivot_approach_speed_limit_ =
          travel / (config_.pivot_coast_time_bound_s + 3 * command_horizon_s_);
      // Reserve one pending and one next intended interval before the
      // declared residual STOP travel; measured speed must fit this reserve.
      pivot_approach_response_limit_ =
          travel / (config_.pivot_coast_time_bound_s + 2 * command_horizon_s_);
    }
  }
  catch (const tf2::TransformException& ex)
  {
    throw nav2_core::ControllerException(std::string("FTCController: TF lookup failed: ") +
                                         ex.what());
  }

  lat_error_ = local_control_point_.translation().y();
  lon_error_ = local_control_point_.translation().x();
  const Eigen::Matrix3d& rot = local_control_point_.rotation();
  // On an interrupted incoming approach PRE_ROTATE aligns to the point too;
  // at an already reached tip retain the plan heading instead of atan2(0,0).
  const bool point_bearing =
      approach && std::hypot(lon_error_, lat_error_) > kPivotArrivalToleranceM;
  const double angle_error_raw =
      point_bearing ? std::atan2(lat_error_, lon_error_) : std::atan2(rot(1, 0), rot(0, 0));

  // Unwrap angle_error_ across the ±π discontinuity (issue #200).
  //
  // atan2 returns values in (-π, π]. When the carrot's relative yaw is
  // near ±π, infinitesimal pose changes can flip the raw result from
  // +π−ε to −π+ε. The proportional PID then commands ω with the
  // opposite sign, the robot reverses direction, and the next tick
  // flips back. Result: a robot whose target heading is near opposite
  // dithers in place instead of converging.
  //
  // Fix: maintain a continuous angle_error_ by detecting the wrap and
  // adding the appropriate 2π-multiple. After this, the PID sees a
  // smooth signal that increases or decreases monotonically as the
  // robot rotates toward the target, with no spurious sign flips.
  //
  // For small heading errors (well away from ±π) this is a no-op —
  // angle_error_raw - angle_error_raw_prev_ is small, no wrap detected.
  if (std::isnan(angle_error_raw_prev_))
  {
    // First tick after setPlan: nothing to unwrap against.
    angle_error_ = angle_error_raw;
  }
  else
  {
    double delta = angle_error_raw - angle_error_raw_prev_;
    if (delta > M_PI)
    {
      delta -= 2.0 * M_PI;
    }
    else if (delta < -M_PI)
    {
      delta += 2.0 * M_PI;
    }
    angle_error_ += delta;
  }
  angle_error_raw_prev_ = angle_error_raw;
}

std::optional<std::size_t> FTCController::stopCornerApproach(bool starting) const
{
  const auto corner = nextPivotCorner();
  const bool approach_state = current_state_ == PlannerState::FOLLOWING ||
                              (starting && current_state_ == PlannerState::PRE_ROTATE);
  if (!approach_state || !corner || plan_header_.frame_id != "map" || is_avoiding_ ||
      lateral_deviation_ != 0.0 || reverse_escape_active_ || obstacle_waiting_ ||
      turn_fallback_.phase != TurnFallbackPhase::kIdle)
    return std::nullopt;
  const auto& end = global_plan_[*corner].pose.position;
  double px = current_control_point_.translation().x();
  double py = current_control_point_.translation().y();
  const double ax = end.x - px, ay = end.y - py;
  if (!std::isfinite(ax) || !std::isfinite(ay))
    return std::nullopt;
  const double window =
      maxLinearSpeed() * (config_.pivot_coast_time_bound_s + 3 * command_horizon_s_);
  double arc = 0.0;
  for (std::size_t i = current_index_ + 1; i <= *corner; ++i)
  {
    const auto& point = global_plan_[i].pose.position;
    const double dx = point.x - px, dy = point.y - py;
    const double yaw = tf2::getYaw(global_plan_[i - 1].pose.orientation);
    arc += std::hypot(dx, dy);
    if (!std::isfinite(arc) || arc > window ||
        !((dx == 0 && dy == 0) ||
          (dx * ax + dy * ay > 0 && dx * std::cos(yaw) + dy * std::sin(yaw) > 0)))
      return std::nullopt;
    px = point.x;
    py = point.y;
  }
  return corner;
}

double FTCController::stopCornerTravel(std::size_t corner, double x, double y, double yaw) const
{
  const auto& goal = global_plan_[corner].pose.position;
  const double dx = goal.x - x, dy = goal.y - y;
  const double outgoing = tf2::getYaw(global_plan_[corner + 1].pose.orientation);
  yaw -= outgoing;
  // Bound radial travel and closing components in the outgoing leg's
  // tangent/normal basis for the actual point approach.
  double travel = std::hypot(dx, dy);
  for (const auto& component :
       {std::pair{dx * std::cos(outgoing) + dy * std::sin(outgoing), std::cos(yaw)},
        std::pair{-dx * std::sin(outgoing) + dy * std::cos(outgoing), std::sin(yaw)}})
    if (component.first * component.second > 0.0)
      travel = std::min(travel, component.first / component.second);
  return std::max(0.0, travel);
}

// ── PID velocity computation ──────────────────────────────────────────────────

void FTCController::calculate_velocity_commands(double dt,
                                                geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (current_state_ == PlannerState::FINISHED || is_crashed_)
  {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return;
  }

  // Integrate errors (with windup clamping).
  i_lon_error_ += lon_error_ * dt;
  i_lat_error_ += lat_error_ * dt;
  i_angle_error_ += angle_error_ * dt;

  i_lon_error_ = std::clamp(i_lon_error_, -config_.ki_lon_max, config_.ki_lon_max);
  i_lat_error_ = std::clamp(i_lat_error_, -config_.ki_lat_max, config_.ki_lat_max);
  i_angle_error_ = std::clamp(i_angle_error_, -config_.ki_ang_max, config_.ki_ang_max);

  // Derivative terms (raw backward finite difference).
  double d_lat = (lat_error_ - last_lat_error_) / dt;
  double d_lon = (lon_error_ - last_lon_error_) / dt;
  double d_angle = (angle_error_ - last_angle_error_) / dt;

  last_lat_error_ = lat_error_;
  last_lon_error_ = lon_error_;
  last_angle_error_ = angle_error_;

  // Optional first-order low-pass on the derivative (derivative-on-measurement
  // filtering). The raw finite difference amplifies the high-frequency jitter
  // in the 10 Hz fused-pose feedback; kd_lat then pumps it into the angular
  // command as a ~1.5 Hz steering limit cycle ("hunting"). Filtering the
  // derivative lets kd_lat stay high enough for tight cross-track tracking
  // without the chatter. tau = 0 keeps the raw derivative (prior behaviour);
  // alpha = dt / (tau + dt) is the standard discrete one-pole coefficient.
  // From PR #290 (64dce368).
  if (config_.derivative_filter_tau > 0.0)
  {
    const double alpha = dt / (config_.derivative_filter_tau + dt);
    d_lat_filt_ += alpha * (d_lat - d_lat_filt_);
    d_lon_filt_ += alpha * (d_lon - d_lon_filt_);
    d_angle_filt_ += alpha * (d_angle - d_angle_filt_);
    d_lat = d_lat_filt_;
    d_lon = d_lon_filt_;
    d_angle = d_angle_filt_;
  }

  // ── Linear velocity (FOLLOWING only) ──────────────────────────────────────

  if (current_state_ == PlannerState::FOLLOWING)
  {
    double lin_speed =
        lon_error_ * config_.kp_lon + i_lon_error_ * config_.ki_lon + d_lon * config_.kd_lon;

    if (lin_speed < 0.0 && config_.forward_only)
    {
      lin_speed = 0.0;
    }
    else
    {
      const double max_speed = maxLinearSpeed();
      lin_speed = std::clamp(lin_speed, -max_speed, max_speed);
    }

    if (is_stalled_)
    {
      // Blocked or slipping: bound the OUTPUT to the crawl speed and skip the
      // min_speed_mps floor. The floor keeps normal driving smooth, but here
      // it would command 0.15-0.30 m/s straight into the obstruction (the lon
      // PID stays positive because the carrot is ahead), digging holes in soft
      // turf. A negative lin_speed (reversing away, forward_only off) passes
      // through unclamped.
      cmd_vel.twist.linear.x = std::min(lin_speed, config_.stall_crawl_speed);
    }
    else
    {
      // Cap forward catch-up at the ramped target speed. lin_speed is a pure-P
      // carrot-follow term (kp_lon·lon_error), and the carrot advances OPEN-LOOP
      // at current_movement_speed_ (see update_control_point). Whenever the
      // robot's real motion transiently lags that open-loop carrot — exiting a
      // turn as the target ramps speed_slow→speed_fast, or a downstream
      // collision_monitor slowdown the controller can't see — lon_error grows and
      // the raw term surges toward max_cmd_vel_speed (0.30), well above the mowing
      // speed (~0.20). That is the "slows a little, then accelerates a lot, then
      // settles" the operator sees. Clamping the forward output to the
      // acceleration-limited current_movement_speed_ closes the lag at the ramp
      // rate instead of leaping; max_cmd_vel_speed stays as the absolute cap only.
      // ClampForwardToMovementRamp bounds the PID output by the acceleration-
      // limited carrot speed and floors at min(min_speed_mps, movement speed).
      // That floor is also what the blade-load slowdown needs: while the blade
      // load is limiting, current_movement_speed_ is the slowed carrot speed,
      // so the min_speed_mps floor cannot undo a slowdown below 0.15 m/s
      // (0.20 x 0.4 = 0.08); BladeLoadDecision never goes below
      // stall_crawl_speed, which still clears the firmware wheel deadband.
      cmd_vel.twist.linear.x =
          ClampForwardToMovementRamp(lin_speed, current_movement_speed_, config_.min_speed_mps);
      // Point-bearing corner approach must be able to rotate toward an aside
      // target without the normal driving floor forcing a pursuit orbit.
      if (cmd_vel.twist.linear.x > 0.0 && stopCornerApproach())
      {
        cmd_vel.twist.linear.x = std::min(cmd_vel.twist.linear.x, lin_speed);
      }
    }
  }
  else
  {
    cmd_vel.twist.linear.x = 0.0;
  }

  // ── Angular velocity ───────────────────────────────────────────────────────

  // When reversing, steering must correct in the opposite lateral direction.
  // Use a local adjusted copy so lat_error_ (the member) stays unflipped for
  // the next cycle's derivative computation.
  const double lat_error_for_steering = (cmd_vel.twist.linear.x < 0.0) ? -lat_error_ : lat_error_;

  if (current_state_ == PlannerState::FOLLOWING)
  {
    // Combined angle + lateral PID during path following. NOTE: the heading
    // gain here is kp_ang_following, NOT kp_ang. On a straight swath the full
    // kp_ang=1.5 (needed to clear the deadband during a PRE_ROTATE pivot)
    // makes the kp_ang*angle_error term saturate max_cmd_vel_ang and limit-
    // cycle at ~0.5 Hz — the left-right swath weave (2026-06-19). A lower
    // FOLLOWING gain kills the weave; the pivot path below keeps full kp_ang.
    double ang_speed = angle_error_ * config_.kp_ang_following + i_angle_error_ * config_.ki_ang +
                       d_angle * config_.kd_ang + lat_error_for_steering * config_.kp_lat +
                       i_lat_error_ * config_.ki_lat + d_lat * config_.kd_lat;

    ang_speed = std::clamp(ang_speed, -config_.max_cmd_vel_ang, config_.max_cmd_vel_ang);
    cmd_vel.twist.angular.z = ang_speed;
  }
  else
  {
    // Pure angle PID during rotation states (no lateral contribution).
    // P follows this tick's checked directed arc, which may exceed pi. Keep
    // the original geometric error for I/D and handoff tolerances; its unwrap
    // accumulator must never become an unbounded proportional turn target.
    const bool directed_long = checked_corner_turn_ && std::abs(*checked_corner_turn_) > M_PI;
    const double angle_for_pid =
        directed_long ? *checked_corner_turn_ : WrapPivotAngle(angle_error_);
    double ang_speed =
        angle_for_pid * config_.kp_ang + i_angle_error_ * config_.ki_ang + d_angle * config_.kd_ang;

    ang_speed = std::clamp(ang_speed, -config_.max_cmd_vel_ang, config_.max_cmd_vel_ang);
    cmd_vel.twist.angular.z = ang_speed;

    // Oscillation override in rotation states. When checkOscillation
    // detects the command is flapping, saturate the magnitude to escape
    // the dither — but PRESERVE the sign of the underlying angle_error_
    // so we rotate the right way. The previous unconditional `+max`
    // forced CCW even when the robot needed CW (negative angle_error_),
    // which made the oscillation worse rather than escape it. Sign comes
    // from angle_error_ (the target the PID is trying to close), not
    // from the (already noisy) PID output ang_speed — so the override
    // doesn't get fooled by zero-crossings in the proportional term.
    // See issue #202.
    const bool is_oscillating = checkOscillation(cmd_vel);
    if (is_oscillating)
    {
      const double sign = ((directed_long ? angle_for_pid : angle_error_) >= 0.0) ? 1.0 : -1.0;
      cmd_vel.twist.angular.z = sign * config_.max_cmd_vel_ang;
    }
  }

  if (obstacle_recovery_active_)
  {
    cmd_vel.twist.angular.z = ClampCommandSlew(last_recovery_angular_cmd_,
                                               cmd_vel.twist.angular.z,
                                               config_.obstacle_restart_angular_acceleration,
                                               dt);
    last_recovery_angular_cmd_ = cmd_vel.twist.angular.z;
  }

  if (config_.debug_pid)
  {
    RCLCPP_DEBUG(logger_,
                 "FTCController PID | lon_err=%.4f lat_err=%.4f ang_err=%.4f "
                 "lin=%.4f ang=%.4f",
                 lon_error_,
                 lat_error_,
                 angle_error_,
                 cmd_vel.twist.linear.x,
                 cmd_vel.twist.angular.z);
  }
}

// ── Pivot corners (ftc_pivot.hpp) ─────────────────────────────────────────────

std::optional<std::size_t> FTCController::nextPivotCorner() const
{
  return NextPivotCorner(pivot_corners_, current_index_);
}

void FTCController::enterPivot()
{
  const std::size_t corner = current_index_;
  const auto& in = global_plan_[corner].pose;
  const double turn_deg = WrapPivotAngle(tf2::getYaw(global_plan_[corner + 1].pose.orientation) -
                                         tf2::getYaw(in.orientation)) *
                          180.0 / M_PI;
  RCLCPP_INFO(logger_,
              "FTCController: PIVOT at corner idx=%zu (%.2f, %.2f), turning %.0f deg in place "
              "(lateral error %.3f m, residual offset %.3f m).",
              corner,
              in.position.x,
              in.position.y,
              turn_deg,
              lat_error_,
              lateral_deviation_);
  // Aim at the corner's outgoing-heading twin: update_control_point's rotation
  // case keeps the carrot there until the heading matches.
  current_index_ = static_cast<uint32_t>(corner + 1);
  current_progress_ = 0.0;
  tf2::fromMsg(global_plan_[current_index_].pose, current_control_point_);

  // No lateral offset through a pivot: the offset is defined against the path
  // heading, which is discontinuous here. The next leg's avoidance starts
  // fresh, from the line, once the robot is aligned with it.
  is_avoiding_ = false;
  target_lateral_deviation_ = 0.0;
  lateral_deviation_ = 0.0;
  avoidance_clear_start_.reset();
  lattice_return_start_.reset();
  lattice_switch_start_.reset();

  // Stop: rotation states emit no linear velocity. The speed ramp restarts in
  // leavePivot; the stall detector must not read the stop as a stall.
  current_movement_speed_ = 0.0;
  stall_time_ = 0.0;
  is_stalled_ = false;

  // The PID history belongs to the leg just finished; the heading error jumps
  // by the corner's turn when the carrot switches twin. Same reset as a fresh
  // plan, so the rotation starts exactly like PRE_ROTATE's.
  i_lon_error_ = 0.0;
  i_lat_error_ = 0.0;
  i_angle_error_ = 0.0;
  last_lat_error_ = 0.0;
  last_lon_error_ = 0.0;
  last_angle_error_ = 0.0;
  d_lat_filt_ = 0.0;
  d_lon_filt_ = 0.0;
  d_angle_filt_ = 0.0;
  angle_error_raw_prev_ = std::numeric_limits<double>::quiet_NaN();
  resetOscillation();
}

void FTCController::leavePivot()
{
  RCLCPP_INFO(logger_,
              "FTCController: pivot done at idx=%u (heading error %.1f deg), following.",
              current_index_,
              WrapPivotAngle(angle_error_) * 180.0 / M_PI);
  // Resume FOLLOWING from the corner's outgoing pose, exactly as PRE_ROTATE
  // hands a fresh plan over: carrot on the cursor, slow speed, no stall debt.
  current_progress_ = 0.0;
  current_movement_speed_ = config_.speed_slow;
  stall_time_ = 0.0;
  is_stalled_ = false;
  i_lat_error_ = 0.0;
  i_lon_error_ = 0.0;
  i_angle_error_ = 0.0;
}

geometry_msgs::msg::Pose FTCController::pivotMapPose() const
{
  if (!pivot_pose_epoch_ || plan_header_.frame_id != "map")
    throw nav2_core::ControllerException("FTCController: no supported rotation pose");
  const auto tf = tf_buffer_->lookupTransform("map", "base_link", *pivot_pose_epoch_);
  const auto& t = tf.transform;
  const auto& q = t.rotation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(t.translation.x) || !std::isfinite(t.translation.y) ||
      !std::isfinite(t.translation.z) || !std::isfinite(norm) || std::abs(norm - 1) > 1e-3)
    throw nav2_core::ControllerException("FTCController: invalid rotation pose");
  geometry_msgs::msg::Pose pose;
  pose.position.x = t.translation.x;
  pose.position.y = t.translation.y;
  pose.orientation = q;
  return pose;
}

bool FTCController::rotationRestartPermitted(const geometry_msgs::msg::Pose& pose,
                                             const CornerGeometry& geometry) const
{
  const transit::Point start{pose.position.x, pose.position.y};
  if (!geometry.valid || !geometry.mowing.authorizedWithin(start, geometry.entry_tolerance))
    return false;
  const double yaw = tf2::getYaw(pose.orientation);
  if (const auto corner = stopCornerApproach(true))
  {
    // An interrupted approach is still capped by B's actual closing travel.
    // At zero travel this is a stopped handoff to PIVOT, not a cruise restart.
    const double travel = stopCornerTravel(*corner, start.x, start.y, yaw);
    if (travel == 0)
      return std::abs(last_measured_fwd_speed_) <= kStoppedTranslationMps;
    return geometry.mowing.segmentAuthorized(start,
                                             {start.x + travel * std::cos(yaw),
                                              start.y + travel * std::sin(yaw)});
  }
  // PRE_ROTATE also owns the initial rendezvous with a distant path. That
  // ordinary FOLLOWING correction is outside the local corner contract. Apply
  // the outgoing-prefix rule once the axle is on this leg (the same existing
  // radial arrival band), or when B owns an interrupted corner approach above.
  if (current_state_ == PlannerState::PRE_ROTATE && current_index_ + 1 < global_plan_.size())
  {
    const auto& a = global_plan_[current_index_].pose.position;
    const auto& b = global_plan_[current_index_ + 1].pose.position;
    if (transit::edgeDistance(start, {a.x, a.y}, {b.x, b.y}) > kPivotArrivalToleranceM)
      return true;
  }
  const auto& origin = global_plan_[current_index_].pose.position;
  const double heading = tf2::getYaw(global_plan_[current_index_].pose.orientation);
  const auto next_stop = nextPivotCorner();
  double straight = 0;
  for (std::size_t i = current_index_ + 1; i < global_plan_.size(); ++i)
  {
    const auto& next = global_plan_[i].pose;
    const double dx = next.position.x - origin.x, dy = next.position.y - origin.y;
    const double along = dx * std::cos(heading) + dy * std::sin(heading);
    const double lateral = -dx * std::sin(heading) + dy * std::cos(heading);
    // Same construction-noise allowance as IsStraightForwardPlan.
    if (!std::isfinite(along) || !std::isfinite(lateral) || std::abs(lateral) > 1e-6 ||
        along + 1e-6 < straight ||
        std::abs(WrapPivotAngle(tf2::getYaw(next.orientation) - heading)) > 1e-6)
      break;
    straight = along;
    if (next_stop && i == *next_stop)
      break;
  }
  const double direction = std::cos(yaw - heading);
  if (!(direction > 0))
    return false;
  const double advanced =
      (start.x - origin.x) * std::cos(heading) + (start.y - origin.y) * std::sin(heading);
  // The actual axle can be ahead of the outgoing twin. Never restart a whole
  // leg at that pose, or demand a cruise response across a millimetre bevel.
  const double remaining =
      std::min(config_.avoidance_horizon_m, std::max(0.0, straight - advanced) / direction);
  return geometry.mowing.segmentAuthorized(start,
                                           {start.x + remaining * std::cos(yaw),
                                            start.y + remaining * std::sin(yaw)});
}

bool FTCController::rotationDeparturePermitted(const CornerGeometry& geometry)
{
  // Caller retains corner_geometry_mutex_ through the state/output handoff.
  const auto pose = pivotMapPose();
  transit::Ring body;
  for (const auto& p : costmap_ros_->getRobotFootprint())
    body.push_back({p.x, p.y});
  return rotationRestartPermitted(pose, geometry) &&
         geometry.rotationClear({pose.position.x, pose.position.y},
                                tf2::getYaw(pose.orientation),
                                0,
                                body) &&
         !localRotationBlocked(0);
}

bool FTCController::localRotationBlocked(double turn)
{
  if (!costmap_ros_ || !costmap_map_ || !pivot_pose_epoch_)
    return true;
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3)
    return true;
  // Same supported epoch as map readiness, rather than getRobotPose's latest
  // (possibly future) transform. Keep the existing filled-footprint raster.
  const auto tf = tf_buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                              "base_link",
                                              *pivot_pose_epoch_);
  geometry_msgs::msg::PoseStamped probe;
  probe.header = tf.header;
  probe.pose.position.x = tf.transform.translation.x;
  probe.pose.position.y = tf.transform.translation.y;
  double reach = 0;
  for (const auto& p : footprint)
    reach = std::max(reach, std::hypot(p.x, p.y));
  const double yaw_now = tf2::getYaw(tf.transform.rotation);
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_map_->getMutex());
  const double step = costmap_map_->getResolution() / reach;
  const auto yaws = std::abs(turn) > M_PI ? DirectedPivotSweepYaws(yaw_now, turn, step)
                                          : PivotSweepYaws(yaw_now, yaw_now + turn, step);
  if (yaws.empty())
    return true;
  for (double yaw : yaws)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    probe.pose.orientation = tf2::toMsg(q);
    if (ObstacleDeviation::footprintBlocked(
            *costmap_map_, probe, 0, footprint, {}, ObstacleDeviation::kLethalOnlyThreshold))
      return true;
  }
  return false;
}

bool FTCController::cornerRotationGate(double dt)
{
  std::shared_ptr<const CornerGeometry> geometry;
  {
    std::lock_guard lock(corner_geometry_mutex_);
    geometry = corner_geometry_;
  }
  if (!geometry || !geometry->valid)
  {
    // Original PRE/PIVOT deadline remains authoritative; say why it runs.
    noteRotationHold(!geometry ? "no mowing geometry received on /map_server_node/transit_geometry"
                               : "the mowing geometry on /map_server_node/transit_geometry is "
                                 "invalid");
    return false;
  }
  const auto pose = pivotMapPose();
  // Missing positional mowing authority is not an observed obstacle. Keep
  // zero under the original PRE/PIVOT deadline without arming obstacle wait.
  // A previously observed, still-active obstacle retains its own deadline.
  if (!geometry->mowing.authorizedWithin({pose.position.x, pose.position.y},
                                         geometry->entry_tolerance))
  {
    noteRotationHold("the robot is outside the mowing area it may rotate in");
    if (obstacle_waiting_)
      waitOrThrowForObstacle("waiting for mowing authority during an obstacle hold");
    return false;
  }
  const double yaw = tf2::getYaw(pose.orientation);
  transit::Ring body;
  for (const auto& p : costmap_ros_->getRobotFootprint())
    body.push_back({p.x, p.y});
  const auto clear = [&](double turn)
  {
    return geometry->rotationClear({pose.position.x, pose.position.y}, yaw, turn, body) &&
           !localRotationBlocked(turn);
  };
  const double original = WrapPivotAngle(angle_error_);
  const double tolerance =
      (current_state_ == PlannerState::PIVOT ? config_.pivot_angle_tolerance_deg
                                             : config_.max_goal_angle_error) *
      M_PI / 180;
  double turn = original;
  if (std::abs(original) < tolerance && !rotationRestartPermitted(pose, *geometry))
  {
    bool heading_exists = false, selected = false;
    for (double offset : {-tolerance / 2, tolerance / 2})
    {
      const double candidate = WrapPivotAngle(original + offset);
      auto target = pose;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw + candidate);
      target.orientation = tf2::toMsg(q);
      if (!rotationRestartPermitted(target, *geometry))
        continue;
      heading_exists = true;
      if (!clear(candidate))
        continue;
      turn = candidate;
      selected = true;
      break;
    }
    if (heading_exists && !selected)
    {
      waitOrThrowForObstacle("blocked finite corner rotation");
      return false;
    }
  }
  if (!clear(turn))
  {
    // A retained sharp corner must stop even where its full disc does not fit.
    // A goal resumed at an opening twin enters PRE_ROTATE. Both stopped
    // rotation states need the same checked alternative when the short arc
    // is blocked; no new state, tolerance or deadline is granted.
    const double opposite = turn - std::copysign(2 * M_PI, turn);
    if (!clear(opposite))
    {
      waitOrThrowForObstacle("obstacle inside the pivot sweep");
      return false;
    }
    turn = opposite;
  }
  if (obstacle_waiting_)
  {
    if (!ObstacleWaitReadyToResume(
            true, dt, config_.obstacle_clear_hold_s, obstacle_followable_time_))
    {
      holdObstacleMotion();
      return false;
    }
    obstacle_waiting_ = false;
    obstacle_followable_time_ = 0;
  }
  // Preserve the established short/bias arithmetic and I/D history exactly.
  // A checked opposite arc instead needs a directed P target separate from the
  // original geometric error used by completion and derivative bookkeeping.
  if (std::abs(turn) <= M_PI)
  {
    angle_error_ = turn;
    angle_error_raw_prev_ = std::numeric_limits<double>::quiet_NaN();
    checked_corner_turn_ = WrapPivotAngle(turn);  // old output-bound arithmetic
  }
  else
  {
    checked_corner_turn_ = turn;
  }
  checked_corner_geometry_ = std::move(geometry);
  return true;
}

// ── Oscillation detection ─────────────────────────────────────────────────────

void FTCController::noteRotationHold(const std::string& reason)
{
  if (reason != rotation_hold_reason_)
  {
    RCLCPP_WARN(logger_, "FTCController: rotation held — %s.", reason.c_str());
  }
  rotation_hold_reason_ = reason;
}

void FTCController::resetOscillation()
{
  {
    std::lock_guard<std::mutex> fd_lock(failure_detector_mutex_);
    failure_detector_.clear();
  }
  oscillation_detected_ = false;
  oscillation_warning_ = false;
  time_last_oscillation_ = clock_->now();
}

bool FTCController::checkOscillation(const geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!config_.oscillation_recovery)
  {
    return false;
  }

  const double max_vel_theta = config_.max_cmd_vel_ang;
  const double max_vel_speed = config_.max_cmd_vel_speed;

  bool oscillating;
  {
    std::lock_guard<std::mutex> fd_lock(failure_detector_mutex_);
    failure_detector_.update(cmd_vel.twist.linear.x,
                             cmd_vel.twist.angular.z,
                             max_vel_speed,
                             max_vel_speed,
                             max_vel_theta,
                             config_.oscillation_v_eps,
                             config_.oscillation_omega_eps);

    oscillating = failure_detector_.isOscillating();
  }

  if (oscillating)
  {
    if (!oscillation_detected_)
    {
      time_last_oscillation_ = clock_->now();
      oscillation_detected_ = true;
    }

    const double oscillation_duration = (clock_->now() - time_last_oscillation_).seconds();
    const bool timeout = oscillation_duration >= config_.oscillation_recovery_min_duration;

    if (timeout)
    {
      if (!oscillation_warning_)
      {
        RCLCPP_WARN(logger_,
                    "FTCController: oscillation detected for %.1fs. "
                    "Activating recovery (preferring current turn direction).",
                    oscillation_duration);
        oscillation_warning_ = true;
      }
      return true;
    }

    return false;  // Oscillating but recovery timeout not yet reached.
  }

  // Not oscillating — reset tracking state.
  time_last_oscillation_ = clock_->now();
  oscillation_detected_ = false;
  oscillation_warning_ = false;

  return false;
}

}  // namespace mowgli_nav2_plugins
