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
//
// FTCController's parameters: declaration (the compiled defaults) and the
// dynamic-parameter callback, which validates a whole batch into a staged copy
// that the control thread adopts at its next cycle (applyPendingConfig). Kept
// out of ftc_controller.cpp, which only reads config_.

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
#include "mowgli_nav2_plugins/ftc_controller.hpp"
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

// ── Parameter handling ────────────────────────────────────────────────────────

void FTCController::declareParameters(const nav2::LifecycleNode::SharedPtr& node)
{
  auto declare_double = [&](const std::string& key, double default_val)
  {
    nav2::declare_parameter_if_not_declared(node,
                                            plugin_name_ + "." + key,
                                            rclcpp::ParameterValue(default_val));
    return node->get_parameter(plugin_name_ + "." + key).as_double();
  };

  auto declare_int = [&](const std::string& key, int default_val)
  {
    nav2::declare_parameter_if_not_declared(node,
                                            plugin_name_ + "." + key,
                                            rclcpp::ParameterValue(default_val));
    return static_cast<int>(node->get_parameter(plugin_name_ + "." + key).as_int());
  };

  auto declare_bool = [&](const std::string& key, bool default_val)
  {
    nav2::declare_parameter_if_not_declared(node,
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
  config_.obstacle_restart_angular_acceleration =
      declare_double("obstacle_restart_angular_acceleration", 1.0);
  config_.min_speed_mps = declare_double("min_speed_mps", 0.15);
  config_.stall_speed_ratio = declare_double("stall_speed_ratio", 0.35);
  config_.stall_grace_s = declare_double("stall_grace_s", 0.6);
  config_.stall_crawl_speed = declare_double("stall_crawl_speed", 0.08);

  // Blade-load slowdown (ftc_blade_load.hpp). Operator knobs are injected from
  // mowgli_robot.yaml by navigation.launch.py; the telemetry age is a static
  // nav2_params_base.yaml value.
  config_.blade_load_slowdown_enabled = declare_bool("blade_load_slowdown_enabled", false);
  config_.blade_load_rpm_full = declare_double("blade_load_rpm_full", 2500.0);
  config_.blade_load_rpm_min = declare_double("blade_load_rpm_min", 1800.0);
  config_.blade_load_min_speed_ratio = declare_double("blade_load_min_speed_ratio", 0.4);
  config_.blade_load_telemetry_max_age_s = declare_double("blade_load_telemetry_max_age_s", 1.0);

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
  // FOLLOWING-only heading gain; defaults to kp_ang so behaviour is unchanged
  // unless explicitly lowered (it is, in nav2_params_base.yaml, to kill the
  // straight-swath weave without softening the PRE_ROTATE pivot).
  config_.kp_ang_following = declare_double("kp_ang_following", config_.kp_ang);

  // Derivative low-pass time constant (s); 0 = raw derivative (prior behaviour).
  config_.derivative_filter_tau = declare_double("derivative_filter_tau", 0.0);

  // Robot limits
  config_.max_cmd_vel_speed = declare_double("max_cmd_vel_speed", 2.0);
  {
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    base_max_cmd_vel_speed_ = config_.max_cmd_vel_speed;
    speed_limit_ = nav2_costmap_2d::NO_SPEED_LIMIT;
  }
  config_.max_cmd_vel_ang = declare_double("max_cmd_vel_ang", 2.0);
  config_.max_goal_distance_error = declare_double("max_goal_distance_error", 1.0);
  config_.max_goal_angle_error = declare_double("max_goal_angle_error", 10.0);
  config_.pivot_angle_tolerance_deg = declare_double("pivot_angle_tolerance_deg", 10.0);
  // Injected by navigation.launch.py (robot_config_util.coverage_entry_tolerance);
  // also read by the transit tree's coverage_entry_goal_checker. 0 = exact line.
  config_.coverage_entry_tolerance_m = declare_double("coverage_entry_tolerance_m", 0.0);
  if (!std::isfinite(config_.coverage_entry_tolerance_m) ||
      config_.coverage_entry_tolerance_m < 0.0 || config_.coverage_entry_tolerance_m > 0.5)
  {
    throw nav2_core::ControllerException(
        plugin_name_ + ".coverage_entry_tolerance_m must be finite and in [0, 0.5]");
  }
  config_.pivot_coast_time_bound_s = declare_double("pivot_coast_time_bound_s", 0.5);
  if (!std::isfinite(config_.pivot_coast_time_bound_s) || config_.pivot_coast_time_bound_s < 0.01 ||
      config_.pivot_coast_time_bound_s > 2.0)
    throw nav2_core::ControllerException(
        plugin_name_ + ".pivot_coast_time_bound_s must be finite and in [0.01, 2.0]");
  config_.goal_timeout = declare_double("goal_timeout", 5.0);
  config_.max_follow_distance = declare_double("max_follow_distance", 1.0);
  // <= 0 derives the cap from speed_fast / kp_lon (ftc_carrot_lead.hpp).
  config_.carrot_max_lead = declare_double("carrot_max_lead", -1.0);

  // Options
  config_.forward_only = declare_bool("forward_only", true);
  // Legacy nearest-point snap in setPlan. OFF by default: on closed headland
  // rings it could skip the entire ring (see setPlan).
  config_.snap_to_nearest_on_set_plan = declare_bool("snap_to_nearest_on_set_plan", false);
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
  config_.obstacle_body_half_width = declare_double("obstacle_body_half_width", 0.20);
  config_.obstacle_clearance_margin = declare_double("obstacle_clearance_margin", 0.0);

  // Obstacle deviation
  config_.enable_obstacle_deviation = declare_bool("enable_obstacle_deviation", true);
  config_.max_lateral_deviation = declare_double("max_lateral_deviation", 1.5);
  config_.deviation_step = declare_double("deviation_step", 0.05);
  config_.deviation_blend_rate = declare_double("deviation_blend_rate", 0.5);
  config_.min_lateral_deviation = declare_double("min_lateral_deviation", 0.30);
  config_.obstacle_wait_timeout_s = declare_double("obstacle_wait_timeout_s", 2.5);
  config_.obstacle_clear_hold_s = declare_double("obstacle_clear_hold_s", 1.5);
  config_.cm_hold_s = declare_double("cm_hold_s", 3.0);
  config_.cm_stall_only_s = declare_double("cm_stall_only_s", 10.0);
  config_.confine_deviation_to_zone = declare_bool("confine_deviation_to_zone", true);

  // Footprint-polygon clearance + bounded reverse-escape.
  config_.use_footprint_clearance = declare_bool("use_footprint_clearance", false);
  config_.obstacle_footprint_front_length_m =
      declare_double("obstacle_footprint_front_length_m", 0.30);
  config_.require_clear_exit = declare_bool("require_clear_exit", true);
  config_.use_offset_lattice = declare_bool("use_offset_lattice", false);
  config_.avoidance_horizon_m = declare_double("avoidance_horizon_m", 2.5);
  config_.avoidance_max_slope = declare_double("avoidance_max_slope", 1.0);
  config_.avoidance_reaction_m = declare_double("avoidance_reaction_m", 0.5);
  config_.avoidance_min_horizon_m = declare_double("avoidance_min_horizon_m", 1.0);
  config_.obstacle_reverse_enabled = declare_bool("obstacle_reverse_enabled", false);
  config_.reverse_boundary_safe = declare_bool("reverse_boundary_safe", false);
  config_.obstacle_reverse_max_dist_m = declare_double("obstacle_reverse_max_dist_m", 0.30);
  config_.obstacle_reverse_speed_mps = declare_double("obstacle_reverse_speed_mps", 0.10);

  // Turn fallback (ftc_turn_fallback.hpp). Declared defaults = the shipped
  // nav2_params_base.yaml values.
  config_.turn_fallback_enabled = declare_bool("turn_fallback_enabled", true);
  config_.turn_fallback_max_reverse_m =
      std::clamp(declare_double("turn_fallback_max_reverse_m", 0.40),
                 0.0,
                 kTurnFallbackMaxReverseCapM);
  config_.turn_fallback_max_rejoin_arc_m =
      std::clamp(declare_double("turn_fallback_max_rejoin_arc_m", 3.0),
                 kTurnFallbackMinRejoinArcM,
                 kTurnFallbackMaxRejoinArcCapM);
  config_.turn_fallback_min_turn_deg =
      std::clamp(declare_double("turn_fallback_min_turn_deg", 45.0), 10.0, 180.0);
  config_.turn_fallback_timeout_s =
      std::clamp(declare_double("turn_fallback_timeout_s", 45.0), 5.0, 300.0);

  // Contour bypass (ftc_contour_bypass.hpp). Declared defaults = the shipped
  // nav2_params_base.yaml values.
  config_.contour_bypass_enabled = declare_bool("contour_bypass_enabled", true);
  config_.contour_bypass_max_contour_m =
      std::clamp(declare_double("contour_bypass_max_contour_m", 8.0),
                 kContourBypassMinContourM,
                 kContourBypassMaxContourCapM);
  config_.contour_bypass_max_rejoin_arc_m =
      std::clamp(declare_double("contour_bypass_max_rejoin_arc_m", 8.0),
                 kContourBypassMinRejoinArcM,
                 kContourBypassMaxRejoinArcCapM);
  config_.contour_bypass_follow_distance_m =
      std::clamp(declare_double("contour_bypass_follow_distance_m", 0.65),
                 kContourBypassMinFollowM,
                 kContourBypassMaxFollowM);
  config_.contour_bypass_timeout_s = std::clamp(declare_double("contour_bypass_timeout_s", 90.0),
                                                kContourBypassMinTimeoutS,
                                                kContourBypassMaxTimeoutS);

  {
    std::lock_guard<std::mutex> lock(staged_config_mutex_);
    staged_config_ = config_;
    staged_config_dirty_ = false;
  }

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

  auto reject_invalid =
      [&result](const std::string& name, double value, double min, double max) -> bool
  {
    if (!std::isfinite(value) || value < min || value > max)
    {
      result.successful = false;
      result.reason = name + " must be finite and within [" + std::to_string(min) + ", " +
                      std::to_string(max) + "]";
      return true;
    }
    return false;
  };

  // Validate the WHOLE batch into a copy and adopt it only if every value is
  // accepted: a rejected batch used to leave the keys before the bad one
  // applied, so config_ and the node's parameters disagreed. The copy is
  // staged for the control thread (applyPendingConfig), the only writer of
  // config_ — this callback runs on the executor thread.
  Config next;
  {
    std::lock_guard<std::mutex> lock(staged_config_mutex_);
    next = staged_config_;
  }
  bool touched = false;
  const std::string prefix = plugin_name_ + ".";
  for (const auto& p : params)
  {
    // Only this plugin's keys: controller_server hosts other plugins too.
    std::string key = p.get_name();
    if (key.rfind(prefix, 0) != 0)
    {
      continue;
    }
    key = key.substr(prefix.size());
    touched = true;

    if (key == "speed_fast")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.speed_fast = p.as_double();
    }
    else if (key == "speed_fast_threshold")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 10.0))
        break;
      next.speed_fast_threshold = p.as_double();
    }
    else if (key == "speed_fast_threshold_angle")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 180.0))
        break;
      next.speed_fast_threshold_angle = p.as_double();
    }
    else if (key == "speed_slow")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.speed_slow = p.as_double();
    }
    else if (key == "speed_angular")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 180.0))
        break;
      next.speed_angular = p.as_double();
    }
    else if (key == "acceleration")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 10.0))
        break;
      next.acceleration = p.as_double();
    }
    else if (key == "obstacle_restart_angular_acceleration")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 10.0))
        break;
      next.obstacle_restart_angular_acceleration = p.as_double();
    }
    else if (key == "min_speed_mps")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.min_speed_mps = p.as_double();
    }
    else if (key == "stall_speed_ratio")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 1.0))
        break;
      next.stall_speed_ratio = p.as_double();
    }
    else if (key == "stall_grace_s")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 30.0))
        break;
      next.stall_grace_s = p.as_double();
    }
    else if (key == "stall_crawl_speed")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.stall_crawl_speed = p.as_double();
    }
    else if (key == "blade_load_slowdown_enabled")
    {
      next.blade_load_slowdown_enabled = p.as_bool();
    }
    else if (key == "blade_load_rpm_full")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 20000.0))
        break;
      next.blade_load_rpm_full = p.as_double();
    }
    else if (key == "blade_load_rpm_min")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 20000.0))
        break;
      next.blade_load_rpm_min = p.as_double();
    }
    else if (key == "blade_load_min_speed_ratio")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 1.0))
        break;
      next.blade_load_min_speed_ratio = p.as_double();
    }
    else if (key == "blade_load_telemetry_max_age_s")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 60.0))
        break;
      next.blade_load_telemetry_max_age_s = p.as_double();
    }
    else if (key == "kp_lon")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kp_lon = p.as_double();
    }
    else if (key == "ki_lon")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_lon = p.as_double();
    }
    else if (key == "ki_lon_max")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_lon_max = p.as_double();
    }
    else if (key == "kd_lon")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kd_lon = p.as_double();
    }
    else if (key == "kp_lat")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kp_lat = p.as_double();
    }
    else if (key == "ki_lat")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_lat = p.as_double();
    }
    else if (key == "ki_lat_max")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_lat_max = p.as_double();
    }
    else if (key == "kd_lat")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kd_lat = p.as_double();
    }
    else if (key == "kp_ang")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kp_ang = p.as_double();
    }
    else if (key == "kp_ang_following")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kp_ang_following = p.as_double();
    }
    else if (key == "ki_ang")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_ang = p.as_double();
    }
    else if (key == "ki_ang_max")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.ki_ang_max = p.as_double();
    }
    else if (key == "kd_ang")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 100.0))
        break;
      next.kd_ang = p.as_double();
    }
    else if (key == "derivative_filter_tau")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 5.0))
        break;
      next.derivative_filter_tau = p.as_double();
    }
    else if (key == "max_cmd_vel_speed")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 10.0))
        break;
      next.max_cmd_vel_speed = p.as_double();
    }
    else if (key == "max_cmd_vel_ang")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 10.0))
        break;
      next.max_cmd_vel_ang = p.as_double();
    }
    else if (key == "max_goal_distance_error")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 10.0))
        break;
      next.max_goal_distance_error = p.as_double();
    }
    else if (key == "max_goal_angle_error")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 180.0))
        break;
      next.max_goal_angle_error = p.as_double();
    }
    else if (key == "pivot_angle_tolerance_deg")
    {
      if (reject_invalid(key, p.as_double(), 1.0, 45.0))
        break;
      next.pivot_angle_tolerance_deg = p.as_double();
    }
    else if (key == "pivot_coast_time_bound_s")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 2.0))
        break;
      next.pivot_coast_time_bound_s = p.as_double();
    }
    else if (key == "goal_timeout")
    {
      if (reject_invalid(key, p.as_double(), 0.1, 300.0))
        break;
      next.goal_timeout = p.as_double();
    }
    else if (key == "max_follow_distance")
    {
      if (reject_invalid(key, p.as_double(), 0.01, 50.0))
        break;
      next.max_follow_distance = p.as_double();
    }
    else if (key == "carrot_max_lead")
    {
      if (reject_invalid(key, p.as_double(), -1.0, 5.0))
        break;
      next.carrot_max_lead = p.as_double();
    }
    else if (key == "forward_only")
    {
      next.forward_only = p.as_bool();
    }
    else if (key == "debug_pid")
    {
      next.debug_pid = p.as_bool();
    }
    else if (key == "debug_obstacle")
    {
      next.debug_obstacle = p.as_bool();
    }
    else if (key == "oscillation_recovery")
    {
      next.oscillation_recovery = p.as_bool();
    }
    else if (key == "oscillation_v_eps")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 1.0))
        break;
      next.oscillation_v_eps = p.as_double();
    }
    else if (key == "oscillation_omega_eps")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 1.0))
        break;
      next.oscillation_omega_eps = p.as_double();
    }
    else if (key == "oscillation_recovery_min_duration")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 300.0))
        break;
      next.oscillation_recovery_min_duration = p.as_double();
    }
    else if (key == "check_obstacles")
    {
      next.check_obstacles = p.as_bool();
    }
    else if (key == "obstacle_lookahead")
    {
      next.obstacle_lookahead = static_cast<int>(p.as_int());
    }
    else if (key == "obstacle_footprint")
    {
      next.obstacle_footprint = p.as_bool();
    }
    else if (key == "obstacle_body_half_width")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.obstacle_body_half_width = p.as_double();
    }
    else if (key == "obstacle_clearance_margin")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.obstacle_clearance_margin = p.as_double();
    }
    else if (key == "enable_obstacle_deviation")
    {
      next.enable_obstacle_deviation = p.as_bool();
    }
    else if (key == "max_lateral_deviation")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 10.0))
        break;
      next.max_lateral_deviation = p.as_double();
    }
    else if (key == "deviation_step")
    {
      if (reject_invalid(key, p.as_double(), 0.001, 1.0))
        break;
      next.deviation_step = p.as_double();
    }
    else if (key == "deviation_blend_rate")
    {
      next.deviation_blend_rate = p.as_double();
    }
    else if (key == "min_lateral_deviation")
    {
      next.min_lateral_deviation = p.as_double();
    }
    else if (key == "obstacle_wait_timeout_s")
    {
      next.obstacle_wait_timeout_s = p.as_double();
    }
    else if (key == "cm_hold_s")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 25.0))
        break;
      next.cm_hold_s = p.as_double();
    }
    else if (key == "cm_stall_only_s")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 25.0))
        break;
      next.cm_stall_only_s = p.as_double();
    }
    else if (key == "obstacle_clear_hold_s")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 30.0))
        break;
      next.obstacle_clear_hold_s = p.as_double();
    }
    else if (key == "confine_deviation_to_zone")
    {
      next.confine_deviation_to_zone = p.as_bool();
    }
    else if (key == "use_footprint_clearance")
    {
      next.use_footprint_clearance = p.as_bool();
    }
    else if (key == "obstacle_footprint_front_length_m")
    {
      next.obstacle_footprint_front_length_m = std::max(0.0, p.as_double());
    }
    else if (key == "require_clear_exit")
    {
      next.require_clear_exit = p.as_bool();
    }
    else if (key == "use_offset_lattice")
    {
      next.use_offset_lattice = p.as_bool();
    }
    else if (key == "avoidance_horizon_m")
    {
      if (reject_invalid(key, p.as_double(), 0.3, 5.0))
        break;
      next.avoidance_horizon_m = p.as_double();
    }
    else if (key == "avoidance_min_horizon_m")
    {
      if (reject_invalid(key, p.as_double(), 0.2, 5.0))
        break;
      next.avoidance_min_horizon_m = p.as_double();
    }
    else if (key == "avoidance_reaction_m")
    {
      if (reject_invalid(key, p.as_double(), 0.0, 2.0))
        break;
      next.avoidance_reaction_m = p.as_double();
    }
    else if (key == "avoidance_max_slope")
    {
      if (reject_invalid(key, p.as_double(), 0.1, 3.0))
        break;
      next.avoidance_max_slope = p.as_double();
    }
    else if (key == "obstacle_reverse_enabled")
    {
      next.obstacle_reverse_enabled = p.as_bool();
    }
    else if (key == "reverse_boundary_safe")
    {
      next.reverse_boundary_safe = p.as_bool();
    }
    else if (key == "obstacle_reverse_max_dist_m")
    {
      next.obstacle_reverse_max_dist_m = std::clamp(p.as_double(), 0.0, 2.0);
    }
    else if (key == "obstacle_reverse_speed_mps")
    {
      next.obstacle_reverse_speed_mps = std::clamp(p.as_double(), 0.0, 1.0);
    }
    else if (key == "turn_fallback_enabled")
    {
      next.turn_fallback_enabled = p.as_bool();
    }
    else if (key == "turn_fallback_max_reverse_m")
    {
      if (reject_invalid(key, p.as_double(), 0.0, kTurnFallbackMaxReverseCapM))
        break;
      next.turn_fallback_max_reverse_m = p.as_double();
    }
    else if (key == "turn_fallback_max_rejoin_arc_m")
    {
      if (reject_invalid(
              key, p.as_double(), kTurnFallbackMinRejoinArcM, kTurnFallbackMaxRejoinArcCapM))
        break;
      next.turn_fallback_max_rejoin_arc_m = p.as_double();
    }
    else if (key == "turn_fallback_min_turn_deg")
    {
      if (reject_invalid(key, p.as_double(), 10.0, 180.0))
        break;
      next.turn_fallback_min_turn_deg = p.as_double();
    }
    else if (key == "turn_fallback_timeout_s")
    {
      if (reject_invalid(key, p.as_double(), 5.0, 300.0))
        break;
      next.turn_fallback_timeout_s = p.as_double();
    }
    else if (key == "contour_bypass_enabled")
    {
      next.contour_bypass_enabled = p.as_bool();
    }
    else if (key == "contour_bypass_max_contour_m")
    {
      if (reject_invalid(
              key, p.as_double(), kContourBypassMinContourM, kContourBypassMaxContourCapM))
        break;
      next.contour_bypass_max_contour_m = p.as_double();
    }
    else if (key == "contour_bypass_max_rejoin_arc_m")
    {
      if (reject_invalid(
              key, p.as_double(), kContourBypassMinRejoinArcM, kContourBypassMaxRejoinArcCapM))
        break;
      next.contour_bypass_max_rejoin_arc_m = p.as_double();
    }
    else if (key == "contour_bypass_follow_distance_m")
    {
      if (reject_invalid(key, p.as_double(), kContourBypassMinFollowM, kContourBypassMaxFollowM))
        break;
      next.contour_bypass_follow_distance_m = p.as_double();
    }
    else if (key == "contour_bypass_timeout_s")
    {
      if (reject_invalid(key, p.as_double(), kContourBypassMinTimeoutS, kContourBypassMaxTimeoutS))
        break;
      next.contour_bypass_timeout_s = p.as_double();
    }
  }

  if (!result.successful || !touched)
  {
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(staged_config_mutex_);
    staged_config_ = next;
    staged_config_dirty_ = true;
  }
  // Read outside the control loop's config_ (own mutexes): keep them current
  // at once, as before.
  {
    std::lock_guard<std::mutex> lock(speed_limit_mutex_);
    base_max_cmd_vel_speed_ = next.max_cmd_vel_speed;
  }
  {
    std::lock_guard<std::mutex> fd_lock(failure_detector_mutex_);
    failure_detector_.setBufferLength(
        static_cast<int>(std::round(next.oscillation_recovery_min_duration * 10.0)));
  }
  return result;
}

void FTCController::applyPendingConfig()
{
  std::lock_guard<std::mutex> lock(staged_config_mutex_);
  if (!staged_config_dirty_)
  {
    return;
  }
  config_ = staged_config_;
  staged_config_dirty_ = false;
  // A parameter change may LOWER the carrot speed to the safe baseline, never
  // raise it: assigning speed_slow outright jumped a stopped or slowed robot
  // (obstacle hold, blade-load slowdown) past the acceleration limit, and did
  // so for any parameter set on controller_server, other plugins' included.
  current_movement_speed_ = std::min(current_movement_speed_, config_.speed_slow);
}

}  // namespace mowgli_nav2_plugins
