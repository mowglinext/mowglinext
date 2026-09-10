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
#include "mowgli_behavior/start_blocked_escape.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
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
  /// Arrival time of the most recent /hardware_bridge/status message.
  /// Default-constructed = none has ever arrived, so latest_status is all
  /// zeroes and describes nothing. EscapeStartBlocked (issue #487) needs this:
  /// `mow_enabled == false` on a message that stopped arriving minutes ago is
  /// NOT a verified blade-off, and the escape must stand down rather than drive
  /// on an unverified blade state.
  std::chrono::steady_clock::time_point last_status_time{};
  mowgli_interfaces::msg::Emergency latest_emergency;
  mowgli_interfaces::msg::Power latest_power;

  /// Timestamp of the last emergency message received.
  std::chrono::steady_clock::time_point last_emergency_time{std::chrono::steady_clock::now()};

  // -----------------------------------------------------------------------
  // Thread safety
  // -----------------------------------------------------------------------

  /// Mutex protecting fields written by subscriber callbacks and read by
  /// BT condition/action nodes.  Use std::lock_guard for RAII locking.
  ///
  /// Does NOT cover the coverage-tracking fields below (command state +
  /// swath-completion model: target_area_index, single_area_target,
  /// attempted_areas, area_attempt_count, area_last_coverage,
  /// area_completed_swaths,
  /// area_swath_count, area_resume_pose_index, area_path_pose_count,
  /// area_plan_fingerprint, completed_areas, coverage_all_complete). Those
  /// are mutated ONLY from this node's own BT action-node callbacks
  /// (FollowStrip, GetNextUnmowedArea, EndSession) and the deferred
  /// ~/clear_coverage_resume handling in tickTree() — every callback of
  /// behavior_tree_node shares its default MutuallyExclusive callback group,
  /// so the tick thread and every service/timer callback are already
  /// serialized against each other even under the MultiThreadedExecutor (see
  /// behavior_tree_node.cpp's ~/clear_coverage_resume comment for the full
  /// rationale). Locking context_mutex around them is therefore redundant —
  /// and 2026-07-17 (task #15) was actively HARMFUL: GetNextUnmowedArea was
  /// the one place still taking it inconsistently, which is what produced
  /// the non-recursive-mutex deadlock documented at
  /// GetNextUnmowedArea::processResponse (advanceAndProbe() re-locking from
  /// inside an already-held lock). Do not add locking back here without
  /// first re-deriving why the MutuallyExclusive guarantee no longer holds
  /// (e.g. a future Reentrant-group conversion) — see the atomic-future-
  /// proofing note on behavior_tree_node's clear_resume_requested_ for the
  /// pattern to follow instead of a mutex if that ever happens.
  mutable std::mutex context_mutex;

  // -----------------------------------------------------------------------
  // Command state (set by HighLevelControl service handler)
  // -----------------------------------------------------------------------

  /// Last command received via the ~/high_level_control service.
  /// Constants match HighLevelControl.srv (COMMAND_START=1, COMMAND_HOME=2,
  /// COMMAND_S1=3, COMMAND_S2=4, COMMAND_MANUAL_MOW=7,
  /// COMMAND_RESET_EMERGENCY=254, …).
  uint8_t current_command{0};

  /// Set by the ~/start_in_area service to REQUEST mowing a single, specific
  /// area instead of iterating all areas. This is the one-shot *request*:
  /// GetNextUnmowedArea consumes it on the next onStart() and latches the
  /// index into single_area_target (below), which is what actually constrains
  /// the run. Consuming it exactly once matters — it is also what triggers
  /// the completed/attempted erase for an explicit re-mow.
  std::optional<int> target_area_index;

  /// SESSION-scoped "single-area mode": the area index a targeted run
  /// (~/start_in_area) is clipped to. Every GetNextUnmowedArea::onStart()
  /// honours it, so the constraint survives the BT re-entering
  /// MowingSequence after the targeted area finishes.
  ///
  /// It exists because the clip used to live ONLY in GetNextUnmowedArea's
  /// own members (current_area_idx_ / max_areas_), which onStart() resets on
  /// every entry, while target_area_index was consumed on the FIRST entry —
  /// so the second entry had no memory of the request and iterated from area
  /// 0. Field log 2026-08-24: "targeted run — mowing only area 1", ~56 min
  /// later "area 0 selected" with no targeted-run line, i.e. the robot rolled
  /// over into an area the operator never asked for.
  ///
  /// Cleared at the session boundary by EndSession, and by a plain
  /// COMMAND_START (see clearSingleAreaMode) so the next full-lawn run
  /// iterates normally.
  std::optional<uint32_t> single_area_target;

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
  // Start-pose-blocked passes (issue #487)
  // -----------------------------------------------------------------------
  /// Set by FollowStrip when a whole pass ended with EVERY sub-path skipped
  /// because Nav2's planner refused to plan from the ROBOT'S OWN pose
  /// (ComputePathToPose START_OCCUPIED) and ZERO swaths were mowed. That is a
  /// property of where the robot is STANDING, not of the field: the area is
  /// perfectly mowable from one metre away. Observed 2026-08-24 — the robot
  /// undocked into the inflated keepout of a 0.25 m obstacle circle and
  /// forfeited a whole field at 0 % coverage.
  ///
  /// Two consumers, deliberately split:
  ///   * IsCoverageStartBlocked (condition node) CONSUMES this bool, so the
  ///     coverage subtree can run a non-motion recovery (stop, clear costmaps,
  ///     wait) and re-tick FollowStrip;
  ///   * GetNextUnmowedArea reads `start_blocked_area` to keep the pass from
  ///     burning the no-progress retirement budget (see below).
  /// Cleared by EndSession.
  bool coverage_start_blocked{false};
  /// Area index whose most recent FollowStrip pass ended start-pose-blocked.
  /// Consumed (reset) by the next GetNextUnmowedArea dispatch of that area.
  std::optional<uint32_t> start_blocked_area;
  /// Per-area count of dispatches that were EXEMPTED from the no-progress
  /// retirement counter because the previous pass was start-pose-blocked.
  /// Bounded by kMaxStartBlockedAttempts so a robot that is genuinely parked on
  /// a lethal cell forever still retires the area and docks — the exemption buys
  /// a real second chance, it does not create an infinite loop. Cleared by
  /// EndSession.
  std::map<uint32_t, uint32_t> area_start_blocked_count;
  /// Maximum start-pose-blocked dispatches exempted from area_attempt_count per
  /// area. Worst case an area gets kMaxStartBlockedAttempts + kMaxAreaAttempts
  /// dispatches before retirement.
  static constexpr uint32_t kMaxStartBlockedAttempts = 3;

  // -----------------------------------------------------------------------
  // Start-pose escape motion (issue #487, follow-up to the above)
  // -----------------------------------------------------------------------
  //
  // SAFETY: these three fields are the entire gate on the only new PHYSICAL
  // MOTION in the #487 workstream. Read mowgli_behavior/start_blocked_escape.hpp
  // before touching any of them.

  /// Arming token for EscapeStartBlocked, set by IsCoverageStartBlocked at the
  /// moment it CONSUMES a confirmed start-pose-blocked pass, and consumed in
  /// turn by EscapeStartBlocked.
  ///
  /// A separate token rather than a second read of coverage_start_blocked
  /// because that bool is already consumed by the condition node one step
  /// earlier in the same sequence. Keeping the arming explicit means the escape
  /// node can refuse to move when it is ticked from anywhere else in the tree,
  /// so the XML placement is not the only thing standing between a tree edit
  /// and an unexpected drive command.
  bool start_blocked_escape_armed{false};
  /// When the token above was armed. A token older than
  /// kStartBlockedEscapeArmMaxAgeSec is refused: the escape follows the blocked
  /// pass immediately in the same branch, so a stale arming means the tree took
  /// a path nobody modelled.
  std::chrono::steady_clock::time_point start_blocked_escape_armed_time{};
  static constexpr double kStartBlockedEscapeArmMaxAgeSec = 30.0;

  /// Escape bounds + stand-down thresholds, loaded from ROS parameters at
  /// startup and already passed through SanitizeEscapeCfg.
  StartBlockedEscapeCfg start_blocked_escape_cfg{};

  // -----------------------------------------------------------------------
  // Last commanded motion (direction source for the escape above)
  // -----------------------------------------------------------------------
  //
  // Updated from twist_mux's MERGED output (/cmd_vel) — i.e. what actually
  // reached the wheels, across every motion lane (coverage, transit, docking,
  // undock BackUp, teleop). A single controller's lane would miss the undock
  // case, which is exactly the case #487 reported.

  /// Last commanded forward velocity whose magnitude exceeded the escape's
  /// min_signal_speed deadband [m/s]. Sign is what matters: it says which way
  /// the robot was travelling when it last actually moved, and therefore which
  /// way it arrived at wherever it is standing now.
  double last_motion_cmd_vx{0.0};
  /// False until such a command has been seen at least once. Cleared by
  /// EndSession so a signal from a previous session can never steer an escape.
  bool last_motion_valid{false};
  /// Arrival time of that command.
  std::chrono::steady_clock::time_point last_motion_time{};
  /// While now() is before this instant the tracker IGNORES /cmd_vel samples.
  /// EscapeStartBlocked keeps it bumped for the duration of its own manoeuvre
  /// plus a short hold-off, because the escape's own commands are not an
  /// ARRIVAL: letting them become "the last motion" would make a second escape
  /// on the next blocked pass drive straight back into the cell the first one
  /// left. The hold-off also covers the round trip through collision_monitor
  /// and twist_mux, so a command published on the final escape tick cannot be
  /// recorded after the node has finished.
  std::chrono::steady_clock::time_point last_motion_suppress_until{};

  // -----------------------------------------------------------------------
  // Swath-completion model (replaces the mow_progress cell grid)
  // -----------------------------------------------------------------------
  /// Indices of swaths already mowed for each area, in the deterministic F2C
  /// swath order. FollowStrip inserts a swath index once its FollowPath goal
  /// succeeds; on a re-plan (resume after recharge / preempt) it skips any
  /// index already present. F2C is deterministic for a fixed area+params, so
  /// indices are stable across re-plans within a session. Persisted with the
  /// area set so resume survives a restart. Cleared per area by EndSession /
  /// a coverage reset.
  std::map<uint32_t, std::set<std::size_t>> area_completed_swaths;
  /// Total swath count for each area, set by FollowStrip after segmenting the
  /// planned path. 0 until the area has been planned at least once.
  std::map<uint32_t, std::size_t> area_swath_count;
  /// Resume cursor: the furthest pose index reached along the area's CONTINUOUS
  /// full_path. FollowStrip drives the plan as one continuous path, so the
  /// per-segment "completed swath index" model can only ever record index 0 (on
  /// full completion). Without this cursor, an interruption mid-path (recharge,
  /// preempt, controller abort) restarts the WHOLE path from the beginning, an
  /// area needing >1 charge never finishes, and a single abort that made real
  /// progress used to fail/abandon the area. FollowStrip persists the furthest
  /// reached index here on abort/halt and, on re-dispatch, trims the already-
  /// driven prefix so it resumes near where it stopped. F2C is deterministic for
  /// a fixed area+params, so the re-planned path is identical and the index is
  /// stable. Cleared (erased) when the area completes; reset by EndSession.
  std::map<uint32_t, std::size_t> area_resume_pose_index;
  /// Total pose count of the area's continuous full_path (the denominator for
  /// the resume-cursor coverage_percent). Set by FollowStrip at dispatch.
  std::map<uint32_t, std::size_t> area_path_pose_count;
  /// Plan-GEOMETRY fingerprint of the area's freshly-planned drivable units
  /// (hash of every unit's quantized pose positions + per-unit counts). This is
  /// the resume-cursor STALENESS key: the pose COUNT alone is not sufficient
  /// because the AUTO mow-angle tie-break (longest-edge, coverage_planning.cpp)
  /// or the sub-path split can yield a geometrically DIFFERENT concatenation with
  /// the SAME pose count — resuming a cursor against that different geometry would
  /// re-enable the blade at the wrong location. On re-plan, FollowStrip discards
  /// the persisted resume state when this fingerprint no longer matches. Set at
  /// dispatch; persisted with the area row; cleared with the other resume maps.
  std::map<uint32_t, uint64_t> area_plan_fingerprint;
  /// Areas whose every swath is completed-or-skipped this session. Skipped by
  /// GetNextUnmowedArea. Cleared by EndSession.
  std::set<uint32_t> completed_areas;
  /// Filesystem path the coverage RESUME state (the four maps above +
  /// completed_areas + current_area) is persisted to, so an interrupted session
  /// survives a full process/container restart — not just the in-RAM BT
  /// halt/resume. Set from the `coverage_resume_path` parameter at startup;
  /// empty disables disk persistence. Written on every interruption / swath
  /// completion, loaded once at node startup, and removed by EndSession. See
  /// coverage_persistence.{hpp,cpp}.
  std::string coverage_resume_path;
  /// True when GetNextUnmowedArea exhausted the area list because every area is
  /// genuinely DONE (not because of a transient service error / timeout / no
  /// areas defined). The coverage subtree reads this (IsCoverageComplete) to
  /// route a normal finish to the MOWING_COMPLETE terminal instead of the
  /// COVERAGE_FAILED_DOCKING path. Reset to false at the start of each
  /// GetNextUnmowedArea run so a transient failure never masquerades as done.
  bool coverage_all_complete{false};

  // -----------------------------------------------------------------------
  // Derived / convenience fields (computed from latest_* messages)
  // -----------------------------------------------------------------------

  float battery_percent{100.0f};

  /// Low-pass-filtered v_battery, in volts, from which battery_percent above is
  /// derived. 0 means no valid reading has arrived yet — check that before
  /// comparing against a voltage threshold. Raw latest_power.v_battery swings
  /// 0.5-1 V on motor transients; see battery_filter.hpp.
  float battery_voltage_filtered{0.0f};

  float gps_quality{0.0f};

  /// Latest GPS position in map frame (from /gps/absolute_pose)
  double gps_x{0.0};
  double gps_y{0.0};

  // -----------------------------------------------------------------------
  // GPS quality classification (derived from gps_quality / fix_type)
  // -----------------------------------------------------------------------

  /// Quality-monotonic GPS fix type derived from /gps/status:
  /// 0=no fix, 2=generic GNSS fix, 3=RTK float, 4=RTK fixed.
  uint8_t gps_fix_type{0};

  /// True when the authoritative /gps/status contract reports a stable RTK-fixed
  /// state at high confidence. SeedYawFromMotion and preflight RTK gates read
  /// this instead of inferring fix quality from /gps/absolute_pose covariance.
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

  /// Set (with hysteresis, see behavior_tree_node's updateLocalizationHealthLocked)
  /// while ABSOLUTE POSITION is untrustworthy. The LocalizationGuard pauses
  /// blade-on mowing while set: field incident 2026-08-02 — RTK dropped to
  /// plain GPS (σ ≈ 1.5 m) for >60 s and FTC kept steering on the drifting
  /// estimate until the robot physically left the area and BoundaryGuard
  /// tripped.
  ///
  /// Driven by GNSS solution quality from /gps/status, NOT by the fused
  /// marginal covariance — fusion_graph inflates that deliberately on every
  /// pivot, which livelocked mowing at 0 % on 2026-08-20. σ_xy survives only
  /// as a generous divergence backstop. See localization_health.hpp.
  bool localization_degraded{false};

  /// Current navigation mode: "precise" or "degraded"
  std::string current_nav_mode{"precise"};

  /// Whether SetNav2Lifecycle has suspended (PAUSEd) the Nav2 lifecycle
  /// stack to save CPU/thermal budget while idle on the dock. Tracked here
  /// (rather than re-querying lifecycle_manager every tick) so the
  /// SetNav2Lifecycle RESUME/PAUSE nodes only issue a manage_nodes service
  /// call on an actual state transition — the BT is the sole pause/resume
  /// authority. Only meaningful when the idle_nav2_suspend feature flag is
  /// enabled; stays false otherwise. Protected by context_mutex.
  bool nav2_suspended{false};

  /// Operator-configured drive speeds (m/s), sourced from mowgli_robot.yaml
  /// by behavior_tree_node and applied to the live controllers by SetNavMode:
  /// transit_speed → FollowPath.desired_linear_vel (RPP transit), mowing_speed
  /// → FollowCoveragePath.vx_max (MPPI coverage). Defaults match the shipped
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

  /// Arrival time of the most recent /collision_monitor_state message —
  /// ANY action_type. collision_monitor only processes (and republishes
  /// state) while cmd_vel_nav flows; once the tree halts, the stream goes
  /// silent and collision_action_type is a STALE LATCH, not live state.
  /// Field 2026-07-23: the first SensorSafetyGuard deployment deadlocked on
  /// exactly this — guard halts tree → Nav2 stops publishing → monitor stops
  /// publishing → STOP latched forever → guard never releases (268 s observed).
  /// Consumers MUST treat a stale latch as "unknown", not "still stopped".
  std::chrono::steady_clock::time_point last_collision_state_time{};

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

  /// Scan-stream liveness (SAFETY_REVIEW_2026-07-23 A-C2). Stamped by the
  /// /scan_collision subscriber in behavior_tree_node on every message.
  /// Default-constructed = no scan EVER received this session — IsScanStale
  /// treats that as "no LiDAR install" and stays inert, so no lidar_enabled
  /// plumbing is needed: the guard only arms once a real scan stream has
  /// existed and then died (LiDAR container crash, filter-chain death).
  std::chrono::steady_clock::time_point last_scan_time{};

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
  // Docking transit lifecycle (owned by the DockRobot action node)
  // -----------------------------------------------------------------------

  /// True while a DockRobot action is actively running (between onStart's
  /// RUNNING return and its terminal SUCCESS/FAILURE or a parent halt).
  /// Maintained SOLELY by DockRobot (onStart sets true, onRunning clears on
  /// any terminal status, onHalted clears unconditionally) so the flag can
  /// never stick true — BehaviorTree.CPP guarantees onHalted() is invoked
  /// whenever a RUNNING StatefulActionNode is halted by a parent.
  ///
  /// Consumed by IsDocking, which BoundaryGuard uses to EXEMPT the blade-off
  /// dock transit (command 1) from the SoftBoundaryHandler: every DockRobot
  /// is entered only after SetMowerEnabled(false) and can never overlap the
  /// blade-on FollowStrip subtree, so "docking_active under command 1" is
  /// provably a blade-OFF transit — the boundary handler must NOT cancel it.
  /// Blade-ON mowing (FollowStrip) keeps full boundary protection because
  /// docking_active is false there. See main_tree.xml BoundaryGuard.
  bool docking_active{false};

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
  // Swath-segmented coverage state
  // -----------------------------------------------------------------------

  /// Full-area coverage path — the concatenation of all segments, kept for
  /// the GUI/Foxglove full-plan view and empty-checks. Populated by
  /// PlanCoverageArea. Execution uses current_strip_segments, NOT this.
  nav_msgs::msg::Path current_strip_path;

  /// EXPLICIT ordered coverage segments from the coverage server (headland
  /// rings first, then straight serpentine swaths). Populated by
  /// PlanCoverageArea; FollowStrip dispatches ONE segment per
  /// FollowCoveragePath goal (RotationShim pivots in place at each segment
  /// start, MPPI tracks the straight swath / smooth ring). Replaces the
  /// heading-jump re-segmentation heuristic, which silently failed on smooth
  /// turn arcs (field 2026-06-12: one 3982-pose "swath").
  std::vector<nav_msgs::msg::Path> current_strip_segments;

  /// Hole-free continuous SUB-PATHS from the coverage server (issue #333), in
  /// drive order. A forward turn-around connector can't route around a large
  /// interior obstacle, so the continuous path is split where it would cross a
  /// hole; FollowStrip drives each sub-path with MPPI and bridges the gap
  /// between consecutive sub-paths with a blade-off, costmap-aware Nav2 transit
  /// (its existing >kSegmentTransitGap behaviour) that routes around the
  /// obstacle. Exactly ONE entry for a hole-free field (== current_strip_path).
  /// When present, FollowStrip drives THESE (one FollowCoveragePath goal per
  /// sub-path) instead of the single current_strip_path.
  std::vector<nav_msgs::msg::Path> current_strip_subpaths;

  /// Transit goal to reach the coverage path start (populated by
  /// PlanCoverageArea, consumed by TransitToStrip).
  geometry_msgs::msg::PoseStamped current_transit_goal;

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
  // High-level status publishing (shared publisher + last-published cache)
  // -----------------------------------------------------------------------
  // PublishHighLevelStatus is a SyncActionNode that only ticks on tree
  // transitions, so during a multi-minute FollowStrip (which sits RUNNING with
  // no transitions) the topic goes SILENT for the whole traversal. The
  // publisher is volatile and the GUI frontend starts from an empty status and
  // only updates on live messages, so a dashboard opened/refreshed mid-mow
  // receives nothing and renders "idle". To keep the topic fresh, the publisher
  // is owned here and the behavior_tree_node re-publishes last_high_level_status
  // on a periodic timer. Protected by context_mutex.
  rclcpp::Publisher<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr high_level_status_pub;
  mowgli_interfaces::msg::HighLevelStatus last_high_level_status;
  bool has_high_level_status{false};

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

/// Drop any "mow only this area" constraint, so the next GetNextUnmowedArea
/// run iterates every area normally.
///
/// Two callers, two reasons, and BOTH are needed:
///   * EndSession — the real session boundary, alongside every other
///     per-session set (attempted_areas, completed_areas, …). This is the
///     normal path: a targeted run finishes its area, docks, EndSession runs.
///   * the ~/high_level_control handler on a plain COMMAND_START — a mowing
///     session does NOT always end with EndSession (a low-battery dock or an
///     emergency deliberately keeps the session alive so mowing auto-resumes),
///     so a stale single-area clip could otherwise still be latched when the
///     operator presses the ordinary "Start" button and expects the whole
///     lawn. The GUI's "mow this area" button calls ~/start_in_area, which
///     sets current_command itself and never goes through that handler, so
///     clearing there cannot cancel a targeted request.
/// Also drops an unconsumed target_area_index: a request that was never
/// picked up (e.g. start_in_area during an emergency) must not silently
/// hijack a later plain start.
inline void clearSingleAreaMode(BTContext& ctx)
{
  ctx.single_area_target.reset();
  ctx.target_area_index.reset();
}

}  // namespace mowgli_behavior
