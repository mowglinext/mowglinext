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

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// IsEmergency
// ---------------------------------------------------------------------------

/// Returns SUCCESS when an active emergency is flagged in the hardware bridge.
class IsEmergency : public BT::ConditionNode
{
public:
  IsEmergency(const std::string& name, const BT::NodeConfig& config)
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
// IsCharging
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the charger relay is enabled (robot is on the dock).
class IsCharging : public BT::ConditionNode
{
public:
  IsCharging(const std::string& name, const BT::NodeConfig& config)
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
// IsBatteryLow
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent falls below the given percent
/// threshold OR (when voltage_threshold > 0) when the raw battery voltage
/// falls below that voltage threshold. Either condition is sufficient —
/// voltage trips first on a sagging pack even if percent is still nominal.
///
/// Input ports:
///   threshold (float, default "22.0") – low-battery threshold in percent.
///   voltage_threshold (float, default "0.0") – low-battery threshold in
///     volts. 0 disables the voltage check (percent only).
class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<float>("threshold", 22.0f, "Low-battery percent threshold"),
        BT::InputPort<float>("voltage_threshold",
                             0.0f,
                             "Low-battery voltage threshold (0 = disabled)"),
    };
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsRainDetected
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the rain sensor reports rain.
class IsRainDetected : public BT::ConditionNode
{
public:
  IsRainDetected(const std::string& name, const BT::NodeConfig& config)
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
// NeedsDocking
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent is at or below the docking threshold.
///
/// Input ports:
///   threshold (float, default "20.0") – return-to-dock battery level in percent.
class NeedsDocking : public BT::ConditionNode
{
public:
  NeedsDocking(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("threshold", 20.0f, "Docking threshold in percent")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsBatteryAbove
// ---------------------------------------------------------------------------

/// Returns SUCCESS when battery_percent is at or above the given threshold.
/// Used to wait until the battery is sufficiently charged before resuming.
///
/// Input ports:
///   threshold (float, default "95.0") – battery percent to consider "charged".
class IsBatteryAbove : public BT::ConditionNode
{
public:
  IsBatteryAbove(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("threshold", 95.0f, "Battery percent threshold")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsCommand
// ---------------------------------------------------------------------------

/// Returns SUCCESS when context.current_command equals the requested command.
///
/// Input ports:
///   command (uint8_t) – expected command value (see HighLevelControl.srv).
class IsCommand : public BT::ConditionNode
{
public:
  IsCommand(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<uint8_t>("command", "Expected HighLevelControl command value")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsGPSFixed
// ---------------------------------------------------------------------------

/// Returns SUCCESS when GPS has RTK fixed quality (high precision).
class IsGPSFixed : public BT::ConditionNode
{
public:
  IsGPSFixed(const std::string& name, const BT::NodeConfig& config)
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
// ReplanNeeded
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the obstacle map has changed and a coverage replan
/// is required.  The flag is set by the main node from the
/// /map_server_node/replan_needed topic.
class ReplanNeeded : public BT::ConditionNode
{
public:
  ReplanNeeded(const std::string& name, const BT::NodeConfig& config)
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
// IsBoundaryViolation
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the robot is detected outside the allowed mowing area.
class IsBoundaryViolation : public BT::ConditionNode
{
public:
  IsBoundaryViolation(const std::string& name, const BT::NodeConfig& config)
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
// IsLethalBoundaryViolation
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the robot is outside all allowed areas by more than
/// the configured lethal margin. Used to escalate BoundaryGuard from
/// "try to navigate back inside" to "emergency stop + wait for operator"
/// — blade/motors operating this far outside the authorised zone can
/// cause real damage. State is fed from
/// /map_server_node/lethal_boundary_violation (std_msgs/Bool).
class IsLethalBoundaryViolation : public BT::ConditionNode
{
public:
  IsLethalBoundaryViolation(const std::string& name, const BT::NodeConfig& config)
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
// IsNewRain
// ---------------------------------------------------------------------------

/// Returns SUCCESS when rain is currently detected AND it was NOT raining
/// when mowing started (i.e., rain is new since mow start) AND rain has
/// been continuous for at least rain_debounce_sec (read from blackboard,
/// 0 disables debounce). Returns FAILURE unconditionally when rain_mode
/// is 0 (rain handling disabled).
class IsNewRain : public BT::ConditionNode
{
public:
  IsNewRain(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsRainModeAtLeast
// ---------------------------------------------------------------------------

/// Returns SUCCESS when the configured rain_mode (read from blackboard) is
/// at least the requested level.
///
/// Modes: 0 = off, 1 = pause-in-place, 2 = dock-and-pause.
///
/// Input ports:
///   mode (int) – minimum rain_mode level required for SUCCESS.
class IsRainModeAtLeast : public BT::ConditionNode
{
public:
  IsRainModeAtLeast(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("mode", "Minimum rain_mode level (0/1/2)")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsResumeUndockAllowed
// ---------------------------------------------------------------------------

/// Returns SUCCESS if the number of resume-undock failures this session is
/// below the configured maximum.  Prevents infinite dock/charge/undock loops
/// when undocking is mechanically broken.
///
/// Input ports:
///   max_attempts (int, default "3") – maximum resume-undock attempts per session.
class IsResumeUndockAllowed : public BT::ConditionNode
{
public:
  IsResumeUndockAllowed(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("max_attempts", 3, "Max resume-undock attempts per session")};
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// IsChargingProgressing
// ---------------------------------------------------------------------------

/// Returns SUCCESS if battery has increased by at least 1% in the last
/// 30 minutes of charging.  Returns FAILURE if charging appears stalled
/// (broken charger, bad connection, etc.).  On first call it records the
/// baseline and always returns SUCCESS.
class IsChargingProgressing : public BT::ConditionNode
{
public:
  IsChargingProgressing(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  bool baseline_set_{false};
  bool charger_failed_{false};
  float baseline_battery_{0.0f};
  std::chrono::steady_clock::time_point baseline_time_{};

  static constexpr double check_interval_sec_{1800.0};  // 30 minutes
  static constexpr float min_increase_{1.0f};  // 1% minimum
};

// ---------------------------------------------------------------------------
// PreFlightCheck
// ---------------------------------------------------------------------------

/// Comprehensive readiness gate before undocking to start mowing.
///
/// Returns SUCCESS only when ALL the following are true at tick time:
///   - No emergency asserted
///   - Battery >= min_battery
///   - GPS fix type >= min_gps_fix_type
///   - TF chain map -> base_footprint is resolvable within tf_timeout_sec
///     (implicitly confirms ekf_odom_node is publishing odom→base_footprint
///      AND ekf_map_node is publishing map→odom)
///   - At least one mowing area is defined in map_server (service call
///     to /map_server_node/get_coverage_status with area_index=0)
///
/// Returns FAILURE on any missing condition, with a single-line summary log
/// so the operator knows exactly which check blocked undocking. Meant to be
/// wrapped in a RetryUntilSuccessful so transient issues (GPS not yet RTK,
/// etc.) have a grace window.
///
/// Input ports:
///   min_battery       (float,   default 20.0) — require at least this %.
///   min_gps_fix_type  (int,     default 2)    — quality-monotonic encoding
///                                               (see behavior_tree_node.cpp):
///                                               0=no fix, 2=DGPS, 3=RTK float,
///                                               4=RTK fixed (1=autonomous, unused).
///                                               Higher = better, so require-RTK-Fixed
///                                               is min=4. Default 2 accepts DGPS+.
///   tf_timeout_sec    (double,  default 0.5)  — how long to wait for TF.
class PreFlightCheck : public BT::ConditionNode
{
public:
  PreFlightCheck(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<float>("min_battery", 20.0f, "Minimum battery percent to start mowing"),
        BT::InputPort<int>("min_gps_fix_type",
                           2,
                           "Min GPS fix type (monotonic: 0=no,2=DGPS,3=RTKfloat,4=RTKfix)"),
        BT::InputPort<double>("tf_timeout_sec", 0.5, "Max wait for map→base_footprint TF"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr coverage_client_;
};

// ---------------------------------------------------------------------------
// Nav2Active
// ---------------------------------------------------------------------------

/// Returns SUCCESS when Nav2's lifecycle_manager reports all managed nodes
/// as active (bt_navigator, controller_server, planner_server,
/// behavior_server). Used as a pre-undock / pre-transit gate so the BT
/// doesn't issue goals into a half-activated Nav2 stack — one observed
/// failure mode had bt_navigator stuck in inactive state while the rest of
/// Nav2 was up, causing every NavigateToPose to be instantly rejected and
/// the strip loop to skip-cascade through the area.
///
/// Calls /lifecycle_manager_navigation/is_active (std_srvs/srv/Trigger).
///
/// Input ports:
///   timeout_sec (double, default 0.5) – max time to wait for the service
///                                       call; on timeout returns FAILURE.
class Nav2Active : public BT::ConditionNode
{
public:
  Nav2Active(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("timeout_sec", 0.5, "Max service call wait in seconds"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// IsObstacleStuck
// ---------------------------------------------------------------------------

/// Returns SUCCESS when collision_monitor's PolygonStop has been
/// continuously active for at least min_duration_sec AND we are still
/// under max_count obstacle-backoffs for this session AND the cooldown
/// since the previous backoff has elapsed. On success, increments
/// ctx->obstacle_backoff_count and stamps ctx->last_obstacle_backoff_time
/// — that is the side effect the StuckBackoff sequence relies on for its
/// per-session cap and cooldown.
///
/// Field problem this solves: collision_monitor's front-stop polygon can
/// wedge the robot motionless for minutes when LiDAR sees an obstacle.
/// /cmd_vel_nav stays nonzero (Nav2 wants to move) but /cmd_vel reaches
/// the motors as zero, so progress_checker eventually escalates to
/// MarkBlockedAndSkip — which then re-plants the robot on the same
/// obstacle. This node lets the BT trigger a 0.40 m reverse + costmap
/// clear when collision_monitor has stopped us for ≥5 s, capped at
/// max_count attempts/session with a cooldown between firings.
///
/// "Stuck" detection is sourced from /collision_monitor_state
/// (nav2_msgs/CollisionMonitorState). The behavior_tree_node subscribes
/// and updates ctx->collision_action_type + ctx->collision_stop_since;
/// this node only consumes those fields. Note that we deliberately do
/// NOT read /cmd_vel{,_nav} as a fallback heuristic — collision_monitor
/// publishes its state directly, so the heuristic would just be a noisier
/// proxy for the same signal.
///
/// Input ports:
///   min_duration_sec (double, default 5.0) — STOP must be active this
///                                            long before we trip.
///   max_count        (int,    default 3)   — per-session backoff cap.
///   cooldown_sec     (double, default 8.0) — min gap between firings.
class IsObstacleStuck : public BT::ConditionNode
{
public:
  IsObstacleStuck(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("min_duration_sec",
                              5.0,
                              "Seconds collision_monitor must be in STOP before tripping"),
        BT::InputPort<int>("max_count", 3, "Per-session cap on obstacle-backoff firings"),
        BT::InputPort<double>("cooldown_sec",
                              8.0,
                              "Minimum gap between obstacle-backoff firings"),
    };
  }

  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// WasRecentlyInCollisionStop
// ---------------------------------------------------------------------------

/// Returns SUCCESS if collision_monitor is currently in STOP OR the most
/// recent STOP→non-STOP transition happened within the last `max_age_sec`
/// seconds. FAILURE otherwise.
///
/// Field problem this solves: a dynamic obstacle (person, animal) wedges
/// the robot just long enough for the controller to abort the FollowStrip
/// goal, then walks off before IsObstacleStuck's check fires. Without this
/// guard the BT falls through to MarkBlockedAndSkip and DEAD-marks cells
/// the robot was perfectly capable of mowing once the obstacle moved.
/// Inserted as a guard before MarkBlockedAndSkip: if we WERE recently
/// stopped, treat the failure as a transient dynamic-obstacle event,
/// clear the costmap, and let SegmentLoop fetch a fresh segment without
/// permanently penalizing the cells.
///
/// Source signal: ctx->collision_action_type and ctx->last_collision_stop_end,
/// both maintained by the /collision_monitor_state subscriber in
/// behavior_tree_node. Pure read — no side effects on the context.
///
/// Input ports:
///   max_age_sec (double, default 10.0) — how recently STOP must have ended
///                                        for this guard to fire.
class WasRecentlyInCollisionStop : public BT::ConditionNode
{
public:
  WasRecentlyInCollisionStop(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("max_age_sec",
                              10.0,
                              "Seconds since last STOP exit to still count as recent"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace mowgli_behavior
