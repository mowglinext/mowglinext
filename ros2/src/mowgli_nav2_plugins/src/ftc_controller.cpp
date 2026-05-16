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
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

namespace mowgli_nav2_plugins
{

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void FTCController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
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

  // Publishers (created as lifecycle-aware, activated/deactivated with the node).
  global_point_pub_ =
      node->create_publisher<geometry_msgs::msg::PoseStamped>(plugin_name_ + "/global_point", 1);
  global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/global_plan",
                                                                 rclcpp::QoS(1).transient_local());
  obstacle_marker_pub_ =
      node->create_publisher<visualization_msgs::msg::Marker>(plugin_name_ + "/costmap_marker", 10);

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
  obstacle_marker_pub_.reset();
}

void FTCController::activate()
{
  RCLCPP_INFO(logger_, "FTCController: activate.");
  global_point_pub_->on_activate();
  global_plan_pub_->on_activate();
  obstacle_marker_pub_->on_activate();
}

void FTCController::deactivate()
{
  RCLCPP_INFO(logger_, "FTCController: deactivate.");
  global_point_pub_->on_deactivate();
  global_plan_pub_->on_deactivate();
  obstacle_marker_pub_->on_deactivate();
}

// ── Parameter handling ────────────────────────────────────────────────────────

void FTCController::declareParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
  auto declare_double = [&](const std::string& key, double default_val)
  {
    nav2_util::declare_parameter_if_not_declared(node,
                                                 plugin_name_ + "." + key,
                                                 rclcpp::ParameterValue(default_val));
    return node->get_parameter(plugin_name_ + "." + key).as_double();
  };

  auto declare_int = [&](const std::string& key, int default_val)
  {
    nav2_util::declare_parameter_if_not_declared(node,
                                                 plugin_name_ + "." + key,
                                                 rclcpp::ParameterValue(default_val));
    return static_cast<int>(node->get_parameter(plugin_name_ + "." + key).as_int());
  };

  auto declare_bool = [&](const std::string& key, bool default_val)
  {
    nav2_util::declare_parameter_if_not_declared(node,
                                                 plugin_name_ + "." + key,
                                                 rclcpp::ParameterValue(default_val));
    return node->get_parameter(plugin_name_ + "." + key).as_bool();
  };

  // Control point speed
  config_.speed_fast = declare_double("speed_fast", 0.5);
  config_.speed_fast_threshold = declare_double("speed_fast_threshold", 1.5);
  config_.speed_fast_threshold_angle = declare_double("speed_fast_threshold_angle", 5.0);
  config_.speed_slow = declare_double("speed_slow", 0.2);
  config_.speed_angular = declare_double("speed_angular", 20.0);
  config_.acceleration = declare_double("acceleration", 1.0);

  // PID longitudinal
  config_.kp_lon = declare_double("kp_lon", 1.0);
  config_.ki_lon = declare_double("ki_lon", 0.0);
  config_.ki_lon_max = declare_double("ki_lon_max", 10.0);
  config_.kd_lon = declare_double("kd_lon", 0.0);

  // PID lateral
  config_.kp_lat = declare_double("kp_lat", 1.0);
  config_.ki_lat = declare_double("ki_lat", 0.0);
  config_.ki_lat_max = declare_double("ki_lat_max", 10.0);
  config_.kd_lat = declare_double("kd_lat", 0.0);

  // PID angular
  config_.kp_ang = declare_double("kp_ang", 1.0);
  config_.ki_ang = declare_double("ki_ang", 0.0);
  config_.ki_ang_max = declare_double("ki_ang_max", 10.0);
  config_.kd_ang = declare_double("kd_ang", 0.0);

  // Robot limits
  config_.max_cmd_vel_speed = declare_double("max_cmd_vel_speed", 2.0);
  config_.max_cmd_vel_ang = declare_double("max_cmd_vel_ang", 2.0);
  config_.max_goal_distance_error = declare_double("max_goal_distance_error", 1.0);
  config_.max_goal_angle_error = declare_double("max_goal_angle_error", 10.0);
  config_.goal_timeout = declare_double("goal_timeout", 5.0);
  config_.max_follow_distance = declare_double("max_follow_distance", 1.0);

  // Options
  config_.forward_only = declare_bool("forward_only", true);
  config_.debug_pid = declare_bool("debug_pid", false);
  config_.debug_obstacle = declare_bool("debug_obstacle", false);

  // Recovery
  config_.oscillation_recovery = declare_bool("oscillation_recovery", true);
  // 0.05 m/s and 0.05 rad/s match the ftc_controller.hpp Config defaults.
  // The previous 5.0 m/s default was a typo — at 5 m/s eps the oscillation
  // detector treats almost any motion as "stopped" and never fires its
  // recovery override. With 0.05 the detector fires only when the robot is
  // truly idle (below the firmware ~0.12 m/s deadband).
  config_.oscillation_v_eps = declare_double("oscillation_v_eps", 0.05);
  config_.oscillation_omega_eps = declare_double("oscillation_omega_eps", 0.05);
  config_.oscillation_recovery_min_duration =
      declare_double("oscillation_recovery_min_duration", 5.0);

  // Obstacles
  config_.check_obstacles = declare_bool("check_obstacles", true);
  config_.obstacle_lookahead = declare_int("obstacle_lookahead", 5);
  config_.obstacle_footprint = declare_bool("obstacle_footprint", true);

  // Obstacle deviation
  config_.enable_obstacle_deviation = declare_bool("enable_obstacle_deviation", true);
  config_.max_lateral_deviation = declare_double("max_lateral_deviation", 1.5);
  config_.deviation_step = declare_double("deviation_step", 0.05);
  config_.deviation_blend_rate = declare_double("deviation_blend_rate", 0.5);
  config_.obstacle_wait_timeout_s = declare_double("obstacle_wait_timeout_s", 5.0);

  // Register parameter-change callback.
  param_cb_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
      {
        return onParameterChange(params);
      });
}

rcl_interfaces::msg::SetParametersResult FTCController::onParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params)
  {
    // Strip the plugin namespace prefix before comparing.
    const std::string prefix = plugin_name_ + ".";
    std::string key = p.get_name();
    if (key.rfind(prefix, 0) == 0)
    {
      key = key.substr(prefix.size());
    }

    if (key == "speed_fast")
    {
      config_.speed_fast = p.as_double();
    }
    else if (key == "speed_fast_threshold")
    {
      config_.speed_fast_threshold = p.as_double();
    }
    else if (key == "speed_fast_threshold_angle")
    {
      config_.speed_fast_threshold_angle = p.as_double();
    }
    else if (key == "speed_slow")
    {
      config_.speed_slow = p.as_double();
    }
    else if (key == "speed_angular")
    {
      config_.speed_angular = p.as_double();
    }
    else if (key == "acceleration")
    {
      config_.acceleration = p.as_double();
    }
    else if (key == "kp_lon")
    {
      config_.kp_lon = p.as_double();
    }
    else if (key == "ki_lon")
    {
      config_.ki_lon = p.as_double();
    }
    else if (key == "ki_lon_max")
    {
      config_.ki_lon_max = p.as_double();
    }
    else if (key == "kd_lon")
    {
      config_.kd_lon = p.as_double();
    }
    else if (key == "kp_lat")
    {
      config_.kp_lat = p.as_double();
    }
    else if (key == "ki_lat")
    {
      config_.ki_lat = p.as_double();
    }
    else if (key == "ki_lat_max")
    {
      config_.ki_lat_max = p.as_double();
    }
    else if (key == "kd_lat")
    {
      config_.kd_lat = p.as_double();
    }
    else if (key == "kp_ang")
    {
      config_.kp_ang = p.as_double();
    }
    else if (key == "ki_ang")
    {
      config_.ki_ang = p.as_double();
    }
    else if (key == "ki_ang_max")
    {
      config_.ki_ang_max = p.as_double();
    }
    else if (key == "kd_ang")
    {
      config_.kd_ang = p.as_double();
    }
    else if (key == "max_cmd_vel_speed")
    {
      config_.max_cmd_vel_speed = p.as_double();
    }
    else if (key == "max_cmd_vel_ang")
    {
      config_.max_cmd_vel_ang = p.as_double();
    }
    else if (key == "max_goal_distance_error")
    {
      config_.max_goal_distance_error = p.as_double();
    }
    else if (key == "max_goal_angle_error")
    {
      config_.max_goal_angle_error = p.as_double();
    }
    else if (key == "goal_timeout")
    {
      config_.goal_timeout = p.as_double();
    }
    else if (key == "max_follow_distance")
    {
      config_.max_follow_distance = p.as_double();
    }
    else if (key == "forward_only")
    {
      config_.forward_only = p.as_bool();
    }
    else if (key == "debug_pid")
    {
      config_.debug_pid = p.as_bool();
    }
    else if (key == "debug_obstacle")
    {
      config_.debug_obstacle = p.as_bool();
    }
    else if (key == "oscillation_recovery")
    {
      config_.oscillation_recovery = p.as_bool();
    }
    else if (key == "oscillation_v_eps")
    {
      config_.oscillation_v_eps = p.as_double();
    }
    else if (key == "oscillation_omega_eps")
    {
      config_.oscillation_omega_eps = p.as_double();
    }
    else if (key == "oscillation_recovery_min_duration")
    {
      config_.oscillation_recovery_min_duration = p.as_double();
      failure_detector_.setBufferLength(
          static_cast<int>(std::round(config_.oscillation_recovery_min_duration * 10.0)));
    }
    else if (key == "check_obstacles")
    {
      config_.check_obstacles = p.as_bool();
    }
    else if (key == "obstacle_lookahead")
    {
      config_.obstacle_lookahead = static_cast<int>(p.as_int());
    }
    else if (key == "obstacle_footprint")
    {
      config_.obstacle_footprint = p.as_bool();
    }
    else if (key == "enable_obstacle_deviation")
    {
      config_.enable_obstacle_deviation = p.as_bool();
    }
    else if (key == "max_lateral_deviation")
    {
      config_.max_lateral_deviation = p.as_double();
    }
    else if (key == "deviation_step")
    {
      config_.deviation_step = p.as_double();
    }
    else if (key == "deviation_blend_rate")
    {
      config_.deviation_blend_rate = p.as_double();
    }
    else if (key == "obstacle_wait_timeout_s")
    {
      config_.obstacle_wait_timeout_s = p.as_double();
    }
  }

  // Ensure slow speed is always the safe baseline when parameters change.
  current_movement_speed_ = config_.speed_slow;

  return result;
}

// ── setPlan ───────────────────────────────────────────────────────────────────

void FTCController::setPlan(const nav_msgs::msg::Path& path)
{
  current_state_ = PlannerState::PRE_ROTATE;
  state_entered_time_ = clock_->now();
  is_crashed_ = false;

  // Reset deviation state with the new path so a previous strip's avoidance
  // doesn't leak into the new one.
  is_avoiding_ = false;
  target_lateral_deviation_ = 0.0;
  lateral_deviation_ = 0.0;

  // Reset angle unwrapping state — the new path's first pose orientation
  // is the new reference; nothing prior to setPlan informs continuity.
  angle_error_raw_prev_ = std::numeric_limits<double>::quiet_NaN();

  global_plan_ = path.poses;
  current_index_ = 0;
  current_progress_ = 0.0;

  // Find the nearest path point to the robot's current position so we don't
  // start from index 0 when the robot is far from the path start.
  try
  {
    const auto base_to_map = tf_buffer_->lookupTransform("map",
                                                         "base_link",
                                                         tf2::TimePointZero,
                                                         tf2::durationFromSec(0.5));
    const double rx = base_to_map.transform.translation.x;
    const double ry = base_to_map.transform.translation.y;
    double best_dist = std::numeric_limits<double>::max();
    for (uint32_t i = 0; i < global_plan_.size(); ++i)
    {
      const double dx = global_plan_[i].pose.position.x - rx;
      const double dy = global_plan_[i].pose.position.y - ry;
      const double d = dx * dx + dy * dy;
      if (d < best_dist)
      {
        best_dist = d;
        current_index_ = i;
      }
    }
    best_dist = std::sqrt(best_dist);
    RCLCPP_INFO(logger_,
                "FTCController: setPlan with %zu points, starting at idx=%u (%.2fm from robot at "
                "%.2f,%.2f).",
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

  last_time_ = clock_->now();
  current_movement_speed_ = config_.speed_slow;

  lat_error_ = 0.0;
  lon_error_ = 0.0;
  angle_error_ = 0.0;
  i_lon_error_ = 0.0;
  i_lat_error_ = 0.0;
  i_angle_error_ = 0.0;
  last_lat_error_ = 0.0;
  last_lon_error_ = 0.0;
  last_angle_error_ = 0.0;

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
  }

  global_plan_pub_->publish(pub_path);

  RCLCPP_INFO(logger_,
              "FTCController: received new global plan with %zu points.",
              path.poses.size());
}

// ── setSpeedLimit ─────────────────────────────────────────────────────────────

void FTCController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;

  if (speed_limit_ < 0.0)
  {
    // Negative means "no limit" — restore original parameter value.
    return;
  }

  if (speed_limit_is_percentage_)
  {
    // Treat limit as a fraction [0, 1] of the configured max speed.
    config_.max_cmd_vel_speed = config_.speed_fast * std::clamp(speed_limit_, 0.0, 1.0);
  }
  else
  {
    config_.max_cmd_vel_speed = speed_limit_;
  }

  RCLCPP_INFO(logger_,
              "FTCController: speed limit set to %.3f (percentage=%s).",
              speed_limit_,
              percentage ? "true" : "false");
}

// ── computeVelocityCommands ───────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped FTCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& /*pose*/,
    const geometry_msgs::msg::Twist& /*velocity*/,
    nav2_core::GoalChecker* goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = clock_->now();

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
  const double safe_dt = std::clamp(dt, 0.01, 0.5);

  if (is_crashed_)
  {
    throw nav2_core::ControllerException("FTCController: robot has crashed / collision detected.");
  }

  if (current_state_ == PlannerState::FINISHED)
  {
    // Zero velocity — goal reached.
    return cmd_vel;
  }

  // Reset the Nav2 goal checker so it re-evaluates from scratch each cycle.
  // This prevents stateful goal checkers from latching a spurious "reached"
  // before the FTC state machine has finished following the path.
  if (goal_checker)
  {
    goal_checker->reset();
  }

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
  const PlannerState new_state = update_planner_state();
  if (new_state != current_state_)
  {
    RCLCPP_INFO(logger_,
                "FTCController: state transition %d -> %d (idx=%u, angle_err=%.3f deg).",
                static_cast<int>(current_state_),
                static_cast<int>(new_state),
                current_index_,
                angle_error_ * 180.0 / M_PI);
    state_entered_time_ = clock_->now();
    current_state_ = new_state;
  }

  // 3. Collision check + lateral-deviation update.
  // When enable_obstacle_deviation is true, we never throw on a lookahead
  // collision: instead the carrot is laterally offset until the obstacle
  // clears (see updateLateralDeviation). Throws are reserved for the
  // hard-fail case where the offset would exceed max_lateral_deviation —
  // BT then aborts the strip and requests the next one.
  if (config_.enable_obstacle_deviation)
  {
    updateLateralDeviation(safe_dt);
    // updateLateralDeviation flipped on the wait-before-abort gate (the
    // costmap is blocked beyond max_lateral_deviation and we're holding
    // for obstacle_wait_timeout_s). Hold zero velocity until either the
    // costmap clears or the helper throws on timeout.
    if (obstacle_waiting_)
    {
      return cmd_vel;
    }
    applyLateralDeviationToCarrot();
  }
  else if (checkCollision(config_.obstacle_lookahead))
  {
    is_crashed_ = true;
    throw nav2_core::ControllerException("FTCController: collision detected along lookahead path.");
  }

  // 4. PID velocity computation.
  calculate_velocity_commands(safe_dt, cmd_vel);

  if (is_crashed_)
  {
    throw nav2_core::ControllerException(
        "FTCController: collision detected during velocity computation.");
  }

  return cmd_vel;
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
                     "FTCController: timeout (%.1fs) in PRE_ROTATE.",
                     config_.goal_timeout);
        is_crashed_ = true;
        return PlannerState::FINISHED;
      }
      // Use the GEOMETRIC (wrapped to (-π, π]) angle, not the unwrap
      // accumulator. The accumulator can drift to ±2π+ if the robot
      // overshoots and oscillates during PRE_ROTATE — staying gated on
      // it would keep PRE_ROTATE alive forever even after the robot is
      // physically aligned with the carrot.
      const double angle_wrapped =
          std::atan2(std::sin(angle_error_), std::cos(angle_error_));
      if (std::abs(angle_wrapped) * (180.0 / M_PI) < config_.max_goal_angle_error)
      {
        RCLCPP_INFO(logger_, "FTCController: PRE_ROTATE done, starting FOLLOWING.");
        return PlannerState::FOLLOWING;
      }
    }
    break;

    case PlannerState::FOLLOWING:
    {
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

          double best_dist = std::numeric_limits<double>::max();
          uint32_t best_idx = current_index_;
          for (uint32_t i = 0; i < global_plan_.size(); ++i)
          {
            const double dx = global_plan_[i].pose.position.x - rx;
            const double dy = global_plan_[i].pose.position.y - ry;
            const double d = std::sqrt(dx * dx + dy * dy);
            if (d < best_dist)
            {
              best_dist = d;
              best_idx = i;
            }
          }
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
      if (current_index_ == global_plan_.size() - 2)
      {
        RCLCPP_INFO(logger_, "FTCController: switching to WAITING_FOR_GOAL_APPROACH.");
        return PlannerState::WAITING_FOR_GOAL_APPROACH;
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
        RCLCPP_WARN(
            logger_,
            "FTCController: timeout in WAITING_FOR_GOAL_APPROACH (dist=%.3fm > "
            "max_goal_distance_error=%.3fm); aborting strip.",
            distance, config_.max_goal_distance_error);
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
      if (std::abs(angle_error_) * (180.0 / M_PI) < config_.max_goal_angle_error)
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
  switch (current_state_)
  {
    case PlannerState::PRE_ROTATE:
      tf2::fromMsg(global_plan_[current_index_].pose, current_control_point_);
      break;

    case PlannerState::FOLLOWING:
    {
      // Don't advance the carrot if it's already too far ahead of the robot.
      // This prevents the carrot from running away when an external component
      // (e.g. collision_monitor) slows the robot below the carrot's speed.
      const double carrot_dist = local_control_point_.translation().norm();
      const double carrot_max_lead = 1.0;  // max metres the carrot may lead
      if (carrot_dist > carrot_max_lead)
      {
        break;  // skip advancement, let robot catch up
      }

      // Compute target speed based on how much straight path lies ahead.
      const double straight_dist = distanceLookahead();
      const double target_speed =
          (straight_dist >= config_.speed_fast_threshold) ? config_.speed_fast : config_.speed_slow;

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

      // Advance the carrot along path segments.
      Eigen::Affine3d nextPose, currentPose;
      while (angle_to_move > 0.0 && distance_to_move > 0.0 &&
             current_index_ < global_plan_.size() - 2)
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

      // SLERP interpolation between the two bounding path points.
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

  // Transform carrot from map into base_link to get the PID errors.
  try
  {
    const auto map_to_base = tf_buffer_->lookupTransform("base_link",
                                                         "map",
                                                         tf2::TimePointZero,
                                                         tf2::durationFromSec(1.0));

    tf2::doTransform(current_control_point_, local_control_point_, map_to_base);
  }
  catch (const tf2::TransformException& ex)
  {
    throw nav2_core::ControllerException(std::string("FTCController: TF lookup failed: ") +
                                         ex.what());
  }

  lat_error_ = local_control_point_.translation().y();
  lon_error_ = local_control_point_.translation().x();
  // Extract yaw from rotation matrix using atan2 (reliable for 2D, unlike
  // Eigen::eulerAngles which can give ambiguous results near singularities).
  const Eigen::Matrix3d& rot = local_control_point_.rotation();
  const double angle_error_raw = std::atan2(rot(1, 0), rot(0, 0));

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

  // Derivative terms.
  const double d_lat = (lat_error_ - last_lat_error_) / dt;
  const double d_lon = (lon_error_ - last_lon_error_) / dt;
  const double d_angle = (angle_error_ - last_angle_error_) / dt;

  last_lat_error_ = lat_error_;
  last_lon_error_ = lon_error_;
  last_angle_error_ = angle_error_;

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
      lin_speed = std::clamp(lin_speed, -config_.max_cmd_vel_speed, config_.max_cmd_vel_speed);
    }

    cmd_vel.twist.linear.x = lin_speed;
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
    // Combined angle + lateral PID during path following.
    double ang_speed = angle_error_ * config_.kp_ang + i_angle_error_ * config_.ki_ang +
                       d_angle * config_.kd_ang + lat_error_for_steering * config_.kp_lat +
                       i_lat_error_ * config_.ki_lat + d_lat * config_.kd_lat;

    ang_speed = std::clamp(ang_speed, -config_.max_cmd_vel_ang, config_.max_cmd_vel_ang);
    cmd_vel.twist.angular.z = ang_speed;
  }
  else
  {
    // Pure angle PID during rotation states (no lateral contribution).
    // Use the GEOMETRIC angle wrapped to (-π, π] for the proportional
    // term. angle_error_ is the unwrap accumulator — useful for
    // derivative continuity around the ±π boundary (issue #200), but
    // catastrophic for the P term if the robot's net rotation since
    // setPlan exceeds π: kp_ang × (-3π) saturates angular cmd at the
    // wrong sign, so the robot keeps spinning the wrong way and the
    // accumulator drifts further away from zero each tick.
    const double angle_for_pid =
        std::atan2(std::sin(angle_error_), std::cos(angle_error_));
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
      const double sign = (angle_error_ >= 0.0) ? 1.0 : -1.0;
      cmd_vel.twist.angular.z = sign * config_.max_cmd_vel_ang;
    }
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

// ── Collision checking ────────────────────────────────────────────────────────

bool FTCController::checkCollision(int max_points)
{
  if (!config_.check_obstacles)
  {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;

  visualization_msgs::msg::Marker obstacle_marker;

  // Clamp lookahead to the available plan length.
  if (static_cast<std::size_t>(max_points) > global_plan_.size())
  {
    max_points = static_cast<int>(global_plan_.size());
  }

  // Check robot footprint at current pose.
  if (config_.obstacle_footprint)
  {
    std::vector<geometry_msgs::msg::Point> footprint;
    costmap_ros_->getOrientedFootprint(footprint);

    for (const auto& fp_pt : footprint)
    {
      if (costmap_map_->worldToMap(fp_pt.x, fp_pt.y, mx, my))
      {
        const unsigned char cost = costmap_map_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          RCLCPP_WARN(logger_, "FTCController: lethal footprint collision at current pose.");
          return true;
        }
      }
    }
  }

  // Check costmap cells along the lookahead path segments.
  for (int i = 0; i < max_points; ++i)
  {
    std::size_t index = current_index_ + static_cast<std::size_t>(i);
    if (index >= global_plan_.size())
    {
      index = global_plan_.size() - 1;
    }

    const auto& pose = global_plan_[index];

    if (costmap_map_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
    {
      const unsigned char cost = costmap_map_->getCost(mx, my);

      if (config_.debug_obstacle)
      {
        debugObstacle(
            obstacle_marker, static_cast<double>(mx), static_cast<double>(my), cost, max_points);
      }

      // Only abort on lethal-or-inscribed cells: the robot footprint will hit
      // the obstacle. Inflation-gradient cells (128..252) are routinely
      // crossed by coverage strips that intentionally pass near obstacles —
      // collision_monitor's PolygonStop is the runtime guard for those (see
      // CLAUDE.md invariant 5).
      if (cost >= 253u)
      {
        RCLCPP_WARN(logger_, "FTCController: lethal obstacle on path (cost=%u).", cost);
        return true;
      }
    }
  }

  return false;
}

// ── Lateral deviation (skirt obstacles) ───────────────────────────────────────

// Decide whether to stall the controller for `obstacle_wait_timeout_s` or
// throw and let the BT escalate. Called from updateLateralDeviation when
// the AVOIDANCE search runs out of headroom inside max_lateral_deviation
// (both sides blocked at the obstacle pose, OR growDeviationUntilClear
// exceeded the cap). Returns true if the caller should bail out of
// updateLateralDeviation for this tick (keeps lateral_deviation_ at its
// current value, output is zeroed in computeVelocityCommands). Returns
// false when the timeout has elapsed — caller proceeds as if the throw
// were direct, EXCEPT we still throw from inside here to consolidate the
// log message + is_crashed_ latch.
bool FTCController::waitOrThrowForObstacle(const std::string& reason)
{
  if (!obstacle_wait_start_.has_value())
  {
    obstacle_wait_start_ = clock_->now();
    RCLCPP_INFO(logger_,
                "FTCController: %s — holding zero velocity up to %.1fs for the costmap to clear.",
                reason.c_str(), config_.obstacle_wait_timeout_s);
  }
  const double elapsed = (clock_->now() - obstacle_wait_start_.value()).seconds();
  if (elapsed > config_.obstacle_wait_timeout_s)
  {
    is_crashed_ = true;
    throw nav2_core::ControllerException(
        std::string("FTCController: ") + reason + ", aborting strip after " +
        std::to_string(static_cast<int>(elapsed)) + "s wait.");
  }
  obstacle_waiting_ = true;
  return true;
}

void FTCController::updateLateralDeviation(double dt)
{
  // Bail if no costmap or path — tests sometimes run without one of either.
  if (costmap_map_ == nullptr || global_plan_.empty())
  {
    return;
  }

  const std::size_t start_idx = std::min(static_cast<std::size_t>(current_index_),
                                         global_plan_.size() - 1);

  // Step 1: figure out whether the current applied deviation still keeps the
  // path clear in the lookahead window.
  const bool currently_clear = ObstacleDeviation::isPathClearWithDeviation(
      *costmap_map_, global_plan_, start_idx, config_.obstacle_lookahead, lateral_deviation_);

  if (!currently_clear)
  {
    // Need to grow the deviation. If we weren't already avoiding, choose a
    // side from the first obstacle pose; otherwise keep the current sign.
    if (!is_avoiding_)
    {
      const int obs_idx = ObstacleDeviation::findFirstObstacleIndex(
          *costmap_map_, global_plan_, start_idx, config_.obstacle_lookahead);
      if (obs_idx < 0)
      {
        // Footprint collision but no path-pose hit (e.g. inflated cell next
        // to robot from a transient scan return) — nothing to deviate around.
        return;
      }
      target_lateral_deviation_ = ObstacleDeviation::chooseDeviationSide(
          *costmap_map_,
          global_plan_[static_cast<std::size_t>(obs_idx)],
          config_.max_lateral_deviation,
          config_.deviation_step);
      if (target_lateral_deviation_ == 0.0)
      {
        // Both sides blocked at the obstacle pose. Before bailing, hold a
        // wait window so transient costmap state (LIDAR noise, a person
        // crossing the path, inflation around the dock not yet cleared
        // by post-undock observations) can clear without burning a BT
        // retry.
        if (waitOrThrowForObstacle("obstacle blocks both sides, cannot skirt"))
        {
          return;
        }
      }
      is_avoiding_ = true;
      obstacle_wait_start_.reset();
      obstacle_waiting_ = false;
      RCLCPP_INFO(logger_,
                  "FTCController: entering AVOIDANCE (target_dev=%.2fm at idx=%d)",
                  target_lateral_deviation_,
                  obs_idx);
    }

    // Grow the deviation until clear or past the cap.
    target_lateral_deviation_ = ObstacleDeviation::growDeviationUntilClear(
        *costmap_map_,
        global_plan_,
        start_idx,
        config_.obstacle_lookahead,
        target_lateral_deviation_,
        config_.max_lateral_deviation,
        config_.deviation_step);

    if (std::abs(target_lateral_deviation_) > config_.max_lateral_deviation)
    {
      // Same wait-before-abort as the both-sides-blocked case. If the
      // obstacle is transient, the next tick will pull target_dev back
      // under the cap and we resume cleanly.
      if (waitOrThrowForObstacle("lateral deviation needed > max_lateral_deviation"))
      {
        return;
      }
    }
  }
  else if (is_avoiding_)
  {
    // Path is clear at the current deviation — start blending back toward
    // the original path.
    target_lateral_deviation_ = 0.0;
    if (std::abs(lateral_deviation_) < 0.01)
    {
      is_avoiding_ = false;
      RCLCPP_INFO(logger_, "FTCController: AVOIDANCE complete, back on path.");
    }
  }

  // Path is now followable inside the deviation cap. Clear any pending
  // wait state so the next blockage starts its own fresh wait window.
  if (obstacle_waiting_)
  {
    RCLCPP_INFO(logger_, "FTCController: obstacle cleared, resuming after wait.");
    obstacle_waiting_ = false;
    obstacle_wait_start_.reset();
  }

  // Step 2: slew lateral_deviation_ toward target_lateral_deviation_ at the
  // configured blend rate (m/s of lateral shift).
  const double max_step = config_.deviation_blend_rate * dt;
  const double delta = target_lateral_deviation_ - lateral_deviation_;
  lateral_deviation_ += std::clamp(delta, -max_step, max_step);
}

void FTCController::applyLateralDeviationToCarrot()
{
  if (lateral_deviation_ == 0.0)
  {
    return;
  }
  // Shift the carrot's translation in its own y-axis (left of heading).
  const Eigen::Vector3d lateral(0.0, lateral_deviation_, 0.0);
  current_control_point_.translation() += current_control_point_.linear() * lateral;
}

void FTCController::debugObstacle(
    visualization_msgs::msg::Marker& marker, double x, double y, unsigned char cost, int max_ids)
{
  if (marker.points.empty())
  {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.header.stamp = clock_->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
  }

  marker.id = static_cast<int>(marker.points.size()) + 1;

  if (cost < 127u)
  {
    marker.color.g = 1.0f;
    marker.color.r = 0.0f;
  }
  else if (cost < 255u)
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
  }
  marker.color.a = 1.0f;

  geometry_msgs::msg::Point p;
  costmap_map_->mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y), p.x, p.y);
  p.z = 0.0;
  marker.points.push_back(p);

  if (static_cast<int>(marker.points.size()) >= max_ids || cost > 0u)
  {
    obstacle_marker_pub_->publish(marker);
    marker.points.clear();
  }
}

// ── Oscillation detection ─────────────────────────────────────────────────────

bool FTCController::checkOscillation(const geometry_msgs::msg::TwistStamped& cmd_vel)
{
  if (!config_.oscillation_recovery)
  {
    return false;
  }

  const double max_vel_theta = config_.max_cmd_vel_ang;
  const double max_vel_speed = config_.max_cmd_vel_speed;

  failure_detector_.update(cmd_vel.twist.linear.x,
                           cmd_vel.twist.angular.z,
                           max_vel_speed,
                           max_vel_speed,
                           max_vel_theta,
                           config_.oscillation_v_eps,
                           config_.oscillation_omega_eps);

  const bool oscillating = failure_detector_.isOscillating();

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
