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
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mowgli_behavior
{

/// Shared context passed to all BehaviorTree nodes via the blackboard.
///
/// The main node keeps this struct alive and updates it from ROS2 topic
/// callbacks before each tree tick.  BT nodes retrieve a shared_ptr to
/// this struct with:
///
///   auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
struct BTContext
{
  /// ROS2 node used by action/service nodes to create clients.
  rclcpp::Node::SharedPtr node;

  // -----------------------------------------------------------------------
  // Latest sensor state (updated by topic subscribers in the main node)
  // -----------------------------------------------------------------------

  mowgli_interfaces::msg::Status latest_status;
  mowgli_interfaces::msg::Emergency latest_emergency;
  mowgli_interfaces::msg::Power latest_power;

  /// Timestamp of the last emergency message received.
  std::chrono::steady_clock::time_point last_emergency_time{std::chrono::steady_clock::now()};

  // -----------------------------------------------------------------------
  // Thread safety
  // -----------------------------------------------------------------------

  /// Mutex protecting fields written by subscriber callbacks and read by
  /// BT condition/action nodes.  Use std::lock_guard for RAII locking.
  mutable std::mutex context_mutex;

  // -----------------------------------------------------------------------
  // Command state (set by HighLevelControl service handler)
  // -----------------------------------------------------------------------

  /// Last command received via the ~/high_level_control service.
  /// Constants match HighLevelControl.srv (COMMAND_START=1, COMMAND_HOME=2,
  /// COMMAND_S1=3, COMMAND_S2=4, COMMAND_MANUAL_MOW=7,
  /// COMMAND_RESET_EMERGENCY=254, …).
  uint8_t current_command{0};

  /// Set by ~/start_in_area service to request mowing a single, specific
  /// area instead of iterating all areas. Consumed (and reset) by
  /// GetNextUnmowedArea on its first call within a mowing run; once the
  /// requested area is complete, the BT exits MowingSequence and docks
  /// rather than rolling over to other areas.
  std::optional<int> target_area_index;

  /// Areas already dispatched to PlanCoverageArea+FollowStrip in the
  /// current session. GetNextUnmowedArea skips any index in this set
  /// when iterating. An area is added here only after it is genuinely
  /// exhausted — either because all swaths were mowed, or because the
  /// per-area attempt counter (area_attempt_count) hit kMaxAreaAttempts.
  /// Cleared by EndSession.
  std::set<uint32_t> attempted_areas;

  /// Per-area count of CONSECUTIVE GetNextUnmowedArea dispatches that
  /// made NO coverage progress. Reset to 0 whenever a dispatch shows the
  /// area's coverage_percent advanced beyond the last dispatch (see
  /// area_last_coverage). Only a genuinely stuck area — one that cannot
  /// add any coverage across kMaxAreaAttempts successive passes — is
  /// promoted into attempted_areas and skipped. Previously this counted
  /// EVERY dispatch, so a progressing-but-stuttering area (each
  /// FollowStrip abort at a hard obstacle costs a dispatch) gave up while
  /// still climbing — observed 2026-05-29 as area 0 abandoned at 18.6 %
  /// despite advancing 0.3 → 18.6 % across the 5 dispatches. Cleared by
  /// EndSession.
  std::map<uint32_t, uint32_t> area_attempt_count;
  /// Best coverage_percent seen for an area so far this session, used to
  /// decide whether a dispatch made progress (and thus resets the
  /// no-progress attempt counter). Cleared by EndSession.
  std::map<uint32_t, float> area_last_coverage;
  static constexpr uint32_t kMaxAreaAttempts = 5;
  /// Minimum coverage_percent gain that counts as progress (resets the
  /// no-progress counter). Below this, a dispatch is treated as stuck.
  static constexpr float kAreaProgressEpsilonPct = 0.5f;

  // -----------------------------------------------------------------------
  // Derived / convenience fields (computed from latest_* messages)
  // -----------------------------------------------------------------------

  float battery_percent{100.0f};
  float gps_quality{0.0f};

  /// Latest GPS position in map frame (from /gps/absolute_pose)
  double gps_x{0.0};
  double gps_y{0.0};

  // -----------------------------------------------------------------------
  // GPS quality classification (derived from gps_quality / fix_type)
  // -----------------------------------------------------------------------

  /// GPS fix type: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float
  uint8_t gps_fix_type{0};

  /// true when RTK fixed (fix_type >= 4 and gps_quality > 80%)
  bool gps_is_fixed{false};

  // -----------------------------------------------------------------------
  // Localization quality flags (set by boundary/replan monitors)
  // -----------------------------------------------------------------------

  /// Set to true when ObstacleTracker publishes updated obstacles that
  /// differ from the last coverage plan.
  bool replan_needed{false};

  /// Set to true when the robot is outside all allowed polygons.
  bool boundary_violation{false};

  /// Set to true when the robot is outside all allowed polygons by more
  /// than lethal_boundary_margin_m. Escalates the BoundaryGuard from
  /// "try to navigate back inside" to "emergency stop + wait for
  /// operator" — blade/motors past this margin can do real damage.
  bool lethal_boundary_violation{false};

  /// Current navigation mode: "precise" or "degraded"
  std::string current_nav_mode{"precise"};

  /// Operator-configured drive speeds (m/s), sourced from mowgli_robot.yaml
  /// by behavior_tree_node and applied to the live controllers by SetNavMode:
  /// transit_speed → FollowPath.desired_linear_vel (RPP transit), mowing_speed
  /// → FollowCoveragePath.speed_fast (FTC coverage). Defaults match the shipped
  /// template; SetNavMode halves them in "degraded" mode (floored at the host
  /// min-drive clamp).
  double transit_speed{0.25};
  double mowing_speed{0.2};

  /// True if it was raining when the current mowing session started.
  /// Set by WasRainingAtStart, checked by IsNewRain.
  bool raining_at_mow_start{false};

  /// First time we observed continuous rain since the last dry sample.
  /// Used by IsNewRain to debounce short rain pulses (rain_debounce_sec).
  /// Default-constructed time_point flags "no rain currently observed".
  std::chrono::steady_clock::time_point rain_first_detected_time{};

  // -----------------------------------------------------------------------
  // Session-level counters (reset at mowing session start)
  // -----------------------------------------------------------------------

  /// Number of resume-undock failures this mowing session.  Prevents
  /// infinite dock/charge/undock cycles when undocking is mechanically broken.
  int resume_undock_failures{0};

  // -----------------------------------------------------------------------
  // GPS snapshot for heading calibration during undock
  // -----------------------------------------------------------------------
  double undock_start_x{0.0};
  double undock_start_y{0.0};
  bool undock_start_recorded{false};

  /// GPS samples (map-frame x, y) accumulated by the GPS subscriber while
  /// undock_start_recorded is true. CalibrateHeadingFromUndock fits a line
  /// through these to derive yaw with ~3× the precision of just using the
  /// start/end endpoints, then persists the result into mowgli_robot.yaml
  /// via an angular EMA on dock_pose_yaw.
  ///
  /// The subscriber dedups by minimum spacing (0.05 m) so a stationary
  /// chassis doesn't bloat the buffer. Capacity is capped — entries past
  /// the cap drop the oldest sample.
  std::vector<std::pair<double, double>> undock_gps_samples;
  static constexpr size_t kUndockGpsSamplesCap = 200;

  // -----------------------------------------------------------------------
  // Obstacle-stuck recovery (collision_monitor wedging)
  // -----------------------------------------------------------------------

  /// Latest action_type from /collision_monitor_state
  /// (nav2_msgs/CollisionMonitorState). 0 = DO_NOTHING, 1 = STOP,
  /// 2 = SLOWDOWN, 3 = APPROACH, 4 = LIMIT.
  uint8_t collision_action_type{0};

  /// Time at which collision_monitor first transitioned into STOP and
  /// has remained in STOP continuously since. Default-constructed value
  /// flags "not currently in STOP".
  std::chrono::steady_clock::time_point collision_stop_since{};

  /// Time of the most recent STOP→non-STOP transition. Default-constructed
  /// = no STOP has ever ended this session. Used by WasRecentlyInCollisionStop
  /// so transient obstacles that clear between FollowStrip retry attempts
  /// don't fall through to MarkBlockedAndSkip and get permanently DEAD-marked.
  std::chrono::steady_clock::time_point last_collision_stop_end{};

  /// Number of obstacle-backoff recoveries already attempted in the
  /// current session. Reset by EndSession.
  int obstacle_backoff_count{0};

  /// Time of the most recent obstacle-backoff success-tick. Used to
  /// enforce a cooldown so we don't re-fire on the same wedge while
  /// the BackUp + costmap clear is still settling.
  std::chrono::steady_clock::time_point last_obstacle_backoff_time{};

  // -----------------------------------------------------------------------
  // Per-session flags reset by ClearCommand at session end
  // -----------------------------------------------------------------------

  /// True after any seeding node (CalibrateHeadingFromUndock or
  /// SeedYawFromMotion) has successfully published a set_pose to ekf_map
  /// during the current autonomous session. Prevents the forward-drive
  /// SeedYawFromMotion from re-triggering when the root ReactiveSequence
  /// halts MowingSequence (e.g., BoundaryGuard or GpsMode transition) and
  /// later re-enters it from the top.
  bool yaw_seeded_this_session{false};

  // -----------------------------------------------------------------------
  // Docking point (set from parameter or service call)
  // -----------------------------------------------------------------------

  double dock_x{0.0};
  double dock_y{0.0};
  double dock_yaw{0.0};

  // -----------------------------------------------------------------------
  // Legacy coverage path components (retained for potential future use).
  // -----------------------------------------------------------------------

  struct Swath
  {
    geometry_msgs::msg::Point32 start;
    geometry_msgs::msg::Point32 end;
  };

  struct CoveragePlan
  {
    std::vector<Swath> swaths;
    std::vector<nav_msgs::msg::Path> turns;  // N-1 turns for N swaths
    nav_msgs::msg::Path full_path;  // Full F2C discretized path (swaths + turns)
  };

  std::optional<CoveragePlan> coverage_plan;

  /// Already-traveled waypoints from the current plan (legacy).
  std::vector<geometry_msgs::msg::Point> visited_waypoints;

  // -----------------------------------------------------------------------
  // Cell-based strip coverage state
  // -----------------------------------------------------------------------

  /// Current strip / segment path to mow. Populated by GetNextStrip
  /// (legacy) or GetNextSegment (Path C cell-based coverage), consumed
  /// by FollowStrip and MarkSegmentBlocked.
  nav_msgs::msg::Path current_strip_path;

  /// Transit goal to reach strip / segment start (populated by
  /// GetNextStrip or GetNextSegment, consumed by TransitToStrip).
  geometry_msgs::msg::PoseStamped current_transit_goal;

  /// Path C: true when the current segment requires transit
  /// (>~0.5 m gap or large turn) so the BT must disengage the blade
  /// before the move and re-engage at the start of the next FollowStrip.
  /// false → blade stays on for a continuous mowing flow between
  /// adjacent segments. Updated by GetNextSegment; read by the
  /// IsShortSegment condition node in the BT XML.
  bool current_segment_is_long_transit{false};

  /// Path C: free-form tag set by GetNextSegment for diagnostics
  /// ("interior" / "transit" / "complete").
  std::string current_segment_phase{};

  /// Path C: termination reason returned by the segment selector for
  /// the current segment ("boundary" / "obstacle" / "dead_zone" /
  /// "max_length" / "row_end"). Used by MarkSegmentBlocked decisions —
  /// e.g. don't bump fail_count when the segment ended at a known
  /// "obstacle" because that's not a robot failure.
  std::string current_segment_termination_reason{};

  /// Latest coverage percentage.
  float coverage_percent{0.0f};

  /// Progress tracking across charge cycles.
  size_t next_swath_index{0};

  /// Coverage progress (read by PublishHighLevelStatus).
  int current_area{-1};
  int total_swaths{0};
  int completed_swaths{0};
  int skipped_swaths{0};

  // -----------------------------------------------------------------------
  // TF buffer (shared across all BT nodes)
  // -----------------------------------------------------------------------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // -----------------------------------------------------------------------
  // Shared helper node for service calls (avoids creating/destroying DDS
  // participants on every call — the main node is in rclcpp::spin so it
  // cannot be used directly with spin_until_future_complete).
  // -----------------------------------------------------------------------
  rclcpp::Node::SharedPtr helper_node;
};

}  // namespace mowgli_behavior
