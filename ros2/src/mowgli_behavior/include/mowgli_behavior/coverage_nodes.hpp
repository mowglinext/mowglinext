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
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/action_outcome.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/detour_resume.hpp"
#include "mowgli_behavior/dig_skip.hpp"
#include "mowgli_behavior/scan_pause.hpp"
#include "mowgli_behavior/transit_failure.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
#include "mowgli_interfaces/path_tracking_stats.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Swath (mow) angle sentinel. Any negative value = AUTO: the coverage server
// picks the swath-count-minimising angle (F2C NSwath). 0..179 selects a fixed
// swath angle in degrees. Operator-tunable end to end via mowgli_robot.yaml
// (mow_angle_deg) → behavior_tree_node blackboard → PlanCoverageArea goal.
// ---------------------------------------------------------------------------
inline constexpr double kMowAngleAutoDeg = -1.0;

// Every boundary between planner-produced sub-paths is intentional: it means
// the planner could not build one continuous blade-on path. A later unit must
// therefore go through NavigateToPose even when its start is nearby (typically
// the same swath-end pose with the opposite heading). The distance threshold
// still protects the first unit and legacy single-path fallback.
inline bool coverageTransitRequired(double start_gap_m, bool previous_unit_dispatched)
{
  return previous_unit_dispatched ||
         start_gap_m > mowgli_interfaces::coverage_geometry::kSegmentTransitGapM;
}

/// Hard time bound on ONE blade-off coverage transit [s], from the straight-line
/// gap to its goal. Field 2026-09-12: a 0.40 m transit with no time bound turned
/// on the spot for 164 s (RPP could not meet the ±0.10 rad yaw goal, the tree
/// replanned at 1 Hz) until the operator intervened. Generous on purpose —
/// Smac detours around obstacles are much longer than the gap — this only has
/// to end a transit that is clearly not going anywhere.
constexpr double kTransitTimeoutMinSec = 20.0;
constexpr double kTransitTimeoutSlackSec = 15.0;
constexpr double kTransitTimeoutSpeedMps = 0.10;  // half the transit speed
inline double transitDeadlineSec(double gap_m)
{
  const double g = gap_m > 0.0 ? gap_m : 0.0;
  const double t = g / kTransitTimeoutSpeedMps + kTransitTimeoutSlackSec;
  return t > kTransitTimeoutMinSec ? t : kTransitTimeoutMinSec;
}

/// Two transit goals this close are the SAME destination. FollowStrip's first
/// unit starts where TransitToStrip was sent (both come from the plan start or
/// the resume cursor), give or take the dig-zone front trim.
constexpr double kSameTransitTargetM = 0.30;
inline bool sameTransitTarget(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy <= kSameTransitTargetM * kSameTransitTargetM;
}

/// TransitToStrip's time bound when the robot's position is unknown: the
/// deadline of a 30 m transit (transitDeadlineSec) — bounded, and generous.
constexpr double kTransitToStripUnknownGapM = 30.0;

/// Latest NavigateToPose result, filled by the result_callback registered at
/// dispatch (FollowStrip's sub-path transits, TransitToStrip). Held behind a
/// shared_ptr so the callback never touches a destroyed node, and behind a
/// mutex so a future Reentrant callback group cannot race the BT tick (today
/// they are serialized — see bt_context.hpp).
struct TransitResultSlot
{
  std::mutex mutex;
  bool ready = false;
  uint16_t error_code = 0;
  std::string error_msg;
};
/// The status topic reports a transit's abort before its result (and nav2's
/// error code) lands. How long a caller waits for it before classifying the
/// failure as UNKNOWN [s].
constexpr double kTransitResultWaitSec = 2.0;

/// Whether FollowStrip should spin the blade up on start, BEFORE the first
/// unit is dispatched. Only when that unit will be mowed directly from where
/// the robot stands. A first unit that must be reached by a blade-off transit
/// gets its blade from sendFollowGoal once the transit has succeeded. Field
/// 2026-09-10: while every transit was refused with START_OCCUPIED, each retry
/// of the pass spun the blade up for 1.5 s and cut it again — dozens of
/// on/off cycles on a robot that was not going anywhere.
inline bool bladeSpinupBeforeFirstUnit(double first_unit_gap_m)
{
  return !coverageTransitRequired(first_unit_gap_m, /*previous_unit_dispatched=*/false);
}

// ---------------------------------------------------------------------------
// Resume-cursor resolution — shared between FollowStrip (which trims the driven
// prefix and marks fully-driven sub-paths done) and PlanCoverageArea (which aims
// the blade-off transit at the resume point instead of the ring start). Both MUST
// agree on WHERE a resume begins; if they don't, the robot arrives at one place
// and then re-transits to another (the "arrive, wait, drive off elsewhere" bug).
// Keeping the mapping in ONE function is what guarantees they can't drift apart.
// ---------------------------------------------------------------------------
struct ResumeLocation
{
  bool valid = false;  ///< false → no resumable cursor; mow fresh from pose 0.
  std::size_t unit = 0;  ///< index of the sub-path the cursor lands in (units 0..unit-1 are done).
  std::size_t local = 0;  ///< local offset to trim to; 0 → resume at the unit's front pose.
};

/// Map an absolute resume cursor (index into the sub-path concatenation) to the
/// sub-path unit and local offset at which mowing resumes. Applies the guards
/// FollowStrip uses: a cursor of 0 or past the end is not resumable. A cursor
/// near the end replays a short suffix rather than inferring completion; a
/// landing at a unit boundary resumes at that next unit's front.
/// `total_poses` is the sum of unit sizes.
ResumeLocation resolveResumeLocation(const std::vector<nav_msgs::msg::Path>& units,
                                     std::size_t cursor,
                                     std::size_t total_poses);

// ---------------------------------------------------------------------------
// refreshSwathProgress — publish the GUI-facing live swath progress for the
// area currently being mown.
//
// Sets ctx.total_swaths (the number of drivable UNITS in the current plan — the
// denominator behind HighLevelStatus.current_path) and ctx.completed_swaths
// (how many of those units are recorded mowed so far — the numerator behind
// current_path_index). Called at pass START and on EVERY swath boundary, not
// only when a whole area pass finishes, so current_path is > 0 throughout the
// mow and the GUI %-readout (current_path_index / current_path) renders and
// climbs live. Previously these scalars were written only at the terminal
// branch, so they stayed 0 during mowing and the GUI showed no percentage.
// Display-only: touches no blade/motion state.
// ---------------------------------------------------------------------------
void refreshSwathProgress(BTContext& ctx, uint32_t area_idx, std::size_t unit_count);

// ---------------------------------------------------------------------------
// coveragePercentFromCursor — smooth mowing progress (0..100) from the pose
// cursor: 100 * absolute_cursor / total_poses, clamped to [0, 100]. absolute is
// the index into the concatenation of all drivable units, so it is monotonic as
// the robot drives forward across sub-paths within an area — giving a smooth
// live percentage (vs the coarse unit-count ratio in refreshSwathProgress).
// total_poses == 0 yields 0. Pure/free so it is unit-testable without ROS.
// ---------------------------------------------------------------------------
float coveragePercentFromCursor(std::size_t absolute_cursor, std::size_t total_poses);

// Record progress when a coverage execution is interrupted. This deliberately
// does NOT alter completed swaths or completed areas; completion is decided by
// the terminal path-handling logic. Kept separate from disk I/O so the
// invariant is regression-testable without ROS action servers.
void recordInterruptedCoverageProgress(BTContext& ctx,
                                       uint32_t area_idx,
                                       std::size_t absolute_cursor,
                                       std::size_t total_poses);

// ---------------------------------------------------------------------------
// forwardSkipIndex — smallest index > `from` whose cumulative path arc-length
// from poses[from] is at least `skip_dist_m`, bounded to poses.size()-1. Returns
// `from` unchanged when the path is too short, `from` is out of range, or
// skip_dist_m <= 0. Pure/free so it is unit-testable without ROS.
//
// Used to step the resume cursor PAST a non-obstacle FTC abort (issue #389): a
// re-dispatch that resumes ON the identical abort pose re-aborts there forever
// (deterministic ~20 % stall). Skipping a bounded arc-length ahead lands the
// resume beyond the offending path feature (curvature spike / connector arc /
// goal-checker quirk) and guarantees the cursor advances monotonically.
// ---------------------------------------------------------------------------
std::size_t forwardSkipIndex(const std::vector<geometry_msgs::msg::PoseStamped>& poses,
                             std::size_t from,
                             double skip_dist_m);

// ---------------------------------------------------------------------------
// FollowStrip — execute the planned coverage path, blade ON.
//
// Consumes ctx->current_strip_subpaths (the hole-free, continuous drivable
// SUB-PATHS from the coverage server — already joined with forward turn-
// around connector arcs; issue #333) and drives each as ONE FollowCoveragePath
// goal end-to-end via mowgli_nav2_plugins/FTCController: one PRE_ROTATE pivot
// to the sub-path-start heading, then FTC tracks the continuous path tightly
// (following the connector arcs at swath U-turns) using a pose cursor
// (path_progress_idx_ / total_path_poses_), not per-segment dispatch. Falls
// back to ctx->current_strip_path or the raw ctx->current_strip_segments
// (joined) only if current_strip_subpaths is empty — see swaths_ construction
// in onStart(). ctx->current_strip_segments itself is GUI/resume bookkeeping,
// not what this node normally drives.
//
// When the next unit's start is far from the robot (resume mid-list, a
// skipped unit, or a concave field whose sub-paths hop across a hole), the
// node first runs a NavigateToPose transit to the unit start — boundary-aware
// (global costmap keepout) instead of letting FTC cut cross-country.
// ---------------------------------------------------------------------------

class FollowStrip : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  // Distance gate for the first/legacy unit. Every subsequent planner-produced
  // sub-path transits regardless of distance; see coverageTransitRequired().
  // Public so the single-source regression test can assert the value directly.
  static constexpr double kSegmentTransitGap =
      mowgli_interfaces::coverage_geometry::kSegmentTransitGapM;

  FollowStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        // Per-SEGMENT detour budget (issue: FTC blocked by an un-skirtable
        // obstacle). Each blade-off detour around an obstacle increments a
        // counter; once it reaches this many, FollowStrip stops detouring and
        // falls back to the abort-to-next-segment behaviour so it can never loop
        // forever. Reset per segment (unit).
        BT::InputPort<int>("max_detours_per_segment",
                           5,
                           "Max obstacle detours attempted per coverage segment before giving up"),
        // Robot footprint radius (disc) used to test whether a candidate resume
        // pose is clear of lethal costmap cells. Conservative chassis half-width.
        BT::InputPort<double>("detour_footprint_radius_m",
                              0.25,
                              "Footprint disc radius for the resume-pose clearance test (m)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);
  // Cancel every in-flight follow / transit goal, reset the transit state
  // machine and switch the blade off. Shared by onHalted() and yieldToFleet().
  void abortActiveGoals(const std::shared_ptr<BTContext>& ctx);
  // Fleet coordination: the area this pass is mowing was handed to another
  // fleet member (BTContext::fleet_excluded_areas). Save the resume cursor when
  // a path is in flight, stop everything, record the yield so the next
  // dispatch is not charged to the no-progress budget, and end the pass.
  BT::NodeStatus yieldToFleet(const std::shared_ptr<BTContext>& ctx, bool mid_pass);
  // Detour-and-continue: on a FollowCoveragePath obstacle-abort, try to salvage
  // the REST of the current segment instead of abandoning it. Confirms (via the
  // latest global costmap) that a lethal cell really lies ahead, searches FORWARD
  // for the first footprint-clear pose past the obstacle (>= min skip distance),
  // trims the current unit to that pose, and dispatches the EXISTING blade-off
  // NavigateToPose transit toward it (the Nav2 global planner routes around the
  // obstacle). transit_active_ then re-dispatches FollowCoveragePath (blade on)
  // for the remainder. Returns true when a detour was started (caller returns
  // RUNNING); false when it should fall back to the abort-to-next path (no
  // costmap, abort not obstacle-related, no clear resume, or budget exhausted).
  bool tryStartDetour(const std::shared_ptr<BTContext>& ctx);
  /// Trim the current unit to [idx, end) and persist the moved resume cursor.
  void trimUnitAt(const std::shared_ptr<BTContext>& ctx, std::size_t idx);

  /// Coverage-completion plausibility cross-check (issue #680), run once a
  /// pass reports every swath done. Compares ctx->latest_mow_progress
  /// against ctx->current_area_polygon/current_area_obstacles
  /// (mow_coverage_plausibility.hpp) and sets
  /// ctx->coverage_plausibility_warning when the actually-stamped fraction
  /// falls below kMinPlausibleMowedFraction. Never fails or blocks the pass
  /// — a low-confidence signal (no mow_progress sample yet) is not treated
  /// as implausible, only a genuinely low one is.
  void checkCoveragePlausibility(const std::shared_ptr<BTContext>& ctx) const;

  // --- Dig skip zones + dig recovery (dig_skip.hpp) --------------------------
  // The session's dig points never reach a costmap (a keepout under the robot
  // refused every plan from its own pose, 2026-09-10 / 2026-09-17). What keeps
  // the robot out of a hole it dug is decided HERE, on the path it follows.
  struct DigSnapshot
  {
    std::vector<DigPoint> points;
    std::uint64_t event_count{0};
    double radius_m{0.0};
  };
  /// Copy of the session dig state, taken under ctx->context_mutex (it is
  /// written by a subscriber callback).
  static DigSnapshot snapshotDigs(const std::shared_ptr<BTContext>& ctx);
  /// Trim the front of the current unit to the first drivable run outside
  /// every dig zone. Returns false when nothing drivable is left in the unit.
  bool skipUnitFrontPastDigZones(const std::shared_ptr<BTContext>& ctx);
  enum class DigRecoveryStep
  {
    kIdle,  ///< no dig being handled — run the normal handlers
    kBusy,  ///< cancelling / waiting for the reverse / re-dispatched: return RUNNING
    kUnitGivenUp,  ///< the no-progress budget is spent: caller skips the unit
  };
  /// Notice a new dig event while a goal of ours is active, cancel that goal,
  /// wait for the bridge's bounded reverse to settle, then resume the SAME unit
  /// past the dig zone through the existing blade-off transit.
  DigRecoveryStep stepDigRecovery(const std::shared_ptr<BTContext>& ctx);
  /// Book the current unit as mowed (shared by the success paths).
  void markCurrentUnitMowed(const std::shared_ptr<BTContext>& ctx);
  // Dispatch swaths_[swath_idx_]. The first unit transits when it is farther
  // than kSegmentTransitGap; every later sub-path always transits blade-off so
  // a planned discontinuity is reoriented safely before FollowPath starts.
  bool sendCurrentSwath(const std::shared_ptr<BTContext>& ctx);
  // Send the FollowPath goal for the current segment (no gap check).
  bool sendFollowGoal(const std::shared_ptr<BTContext>& ctx);
  /// Log the path-tracking summary of the segment that just finished, then arm
  /// a fresh episode. `outcome` names why it finished ("completed", "aborted").
  void logSegmentTracking(const std::shared_ptr<BTContext>& ctx, const char* outcome);
  // Robot distance to the current segment's first pose (TF map→base_footprint);
  // returns a large value if TF is unavailable (forces the safe transit path).
  double distanceToSegmentStart(const std::shared_ptr<BTContext>& ctx) const;
  // Advance path_progress_idx_ to the furthest pose of the continuous path the
  // robot has reached (monotonic, bounded forward nearest-pose search from the
  // current cursor). Cheap to call every tick.
  void updateProgress(const std::shared_ptr<BTContext>& ctx);
  // Persist the resume cursor + partial coverage_percent for the area, so a
  // re-dispatch after an abort/halt resumes near where it stopped instead of
  // restarting the whole path (and so GetNextUnmowedArea sees the progress and
  // does not abandon the area).
  void persistResumeCursor(const std::shared_ptr<BTContext>& ctx);
  // Smooth live coverage percent (0..100) from the current pose cursor
  // (swath_base_[swath_idx_] + resume_start_idx_ + path_progress_idx_) over
  // total_path_poses_. Monotonic within an area; recomputed every following tick
  // so the GUI %-readout climbs smoothly rather than jumping per sub-path.
  float livePercent() const;

  // --- Transit-failure classification (issue #487) ---------------------------
  /// Result of the blade-off NavigateToPose transit that just finished, plus
  /// the raw nav2 fields it was derived from (kept for the log line).
  struct TransitOutcome
  {
    TransitFailure kind = TransitFailure::kUnknown;
    uint16_t error_code = 0;
    std::string error_msg;
  };
  /// Arm a fresh result slot for a transit about to be dispatched.
  void resetTransitResult();
  /// Classify the transit whose STATUS already reported abort/cancel. The nav2
  /// result arrives on its own callback, slightly after the status topic, so
  /// this returns nullopt while still waiting (bounded by
  /// kTransitResultWaitSec, after which it classifies as UNKNOWN and the caller
  /// proceeds exactly as it did before this change).
  std::optional<TransitOutcome> classifyFinishedTransit();

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  // Mirrors the active segment onto the coverage controller's global_plan
  // topic so the PathProgressGoalChecker (coverage_goal_checker) can track
  // per-pose progress (FTC/RotationShim does not republish the plan).
  // Latched (transient_local) so a late-subscribing goal checker still
  // receives the current segment.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr coverage_plan_pub_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;

  /// Terminal verdicts from the result callbacks. Polling a goal handle is not
  /// enough: a goal the server finishes in the same instant it accepts it can
  /// lose its status message and leave the poll stuck (action_outcome.hpp).
  std::shared_ptr<ActionOutcomeSlot> transit_outcome_ = std::make_shared<ActionOutcomeSlot>();
  std::shared_ptr<ActionOutcomeSlot> follow_outcome_ = std::make_shared<ActionOutcomeSlot>();
  // Path-tracking error of the segment currently being driven, reduced from the
  // FollowPath action feedback that ROS 2 Lyrical's controller_server fills in
  // for whichever controller runs (FTC here). Logged once per segment so a field
  // bag carries the mowing-quality number next to the segment it belongs to.
  //
  // shared_ptr-owned for the same reason as transit_result_ below: a feedback
  // callback that lands after this node was destroyed must not write through a
  // dangling `this`. The mutex guards it against the action client's thread.
  struct TrackingFeedbackSlot
  {
    std::mutex mutex;
    mowgli_interfaces::path_tracking::Summary summary;
    /// Signed error and index of the last sample, for the log line.
    double last_error_m{0.0};
    std::uint32_t last_index{0};
  };
  std::shared_ptr<TrackingFeedbackSlot> tracking_slot_;
  // Inter-segment transit (NavigateToPose) state.
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  bool transit_active_ = false;
  // A blade-off transit is REQUIRED for the current swath (its start is
  // >kSegmentTransitGap away) but navigate_to_pose was not ready when we tried to
  // dispatch it. The blade is held OFF and the dispatch is retried each tick;
  // this flag prevents ever falling through to a blade-on FollowPath across the
  // gap. Bounded by kTransitServerWaitSec (from transit_wait_start_), after which
  // the swath is skipped rather than mowed cross-country.
  bool transit_pending_ = false;
  std::chrono::steady_clock::time_point transit_wait_start_;
  // Result slot for the in-flight transit + the bounded wait that lets it
  // arrive after the status topic reports the abort (issue #487).
  std::shared_ptr<TransitResultSlot> transit_result_;
  bool transit_abort_seen_ = false;
  std::chrono::steady_clock::time_point transit_abort_time_;
  // Max wait for the NavigateToPose RESULT after get_status() says the goal
  // terminated. Past this the failure is logged as UNKNOWN and the swath is
  // skipped — i.e. exactly the pre-#487 behaviour, never a hang.

  // The drivable units being executed: the hole-free continuous SUB-PATHS
  // (ctx->current_strip_subpaths, issue #333), or a single continuous path when
  // the field has no holes. FollowStrip drives one FollowCoveragePath goal per
  // unit and bridges gaps between units with a blade-off Nav2 transit.
  std::vector<nav_msgs::msg::Path> swaths_;
  std::size_t swath_idx_ = 0;
  std::size_t swaths_skipped_ = 0;
  // Of swaths_skipped_, how many were skipped because the blade-off transit was
  // refused with START_OCCUPIED — i.e. because of where the ROBOT stands, not
  // anything about the swath. Paired with swaths_mowed_this_pass_ to recognise
  // the "zero progress and every failure was our own pose" case that must not
  // retire the area (issue #487).
  std::size_t swaths_skipped_start_occupied_ = 0;
  // Sub-paths recorded MOWED during THIS pass. Distinct from
  // ctx->area_completed_swaths[area].size(), which is cumulative across passes.
  std::size_t swaths_mowed_this_pass_ = 0;
  bool swath_goal_sent_ = false;
  // Absolute start index of each swaths_ unit within the CONCATENATION of all
  // units (== full_path). swath_base_[k] = sum of the ORIGINAL (untrimmed) pose
  // counts of units 0..k-1, so the resume cursor persisted in
  // BTContext::area_resume_pose_index is an index into the concatenation and
  // stays comparable to area_path_pose_count. For a single unit this is {0} and
  // the bookkeeping reduces to the original single-path behaviour.
  std::vector<std::size_t> swath_base_;
  // Resume-cursor bookkeeping. resume_start_idx_ = trim offset WITHIN the
  // currently-driven unit (swaths_[swath_idx_]) where this run begins (non-zero
  // only for the one unit a mid-unit resume trimmed); path_progress_idx_ =
  // furthest pose reached within the currently-driven (trimmed) unit;
  // total_path_poses_ = concatenation length (percent denominator). Both reset
  // to 0 on advance() to the next unit.
  std::size_t resume_start_idx_ = 0;
  std::size_t path_progress_idx_ = 0;
  std::size_t total_path_poses_ = 0;
  // Area being mowed (from ctx->current_area) — keys the swath-completion
  // tracking in BTContext so a resume/re-plan skips already-mowed segments.
  uint32_t area_idx_ = 0;

  // --- Detour-and-continue state (obstacle blocking an un-skirtable segment) ---
  // Latest global costmap, used to (a) confirm an abort is obstacle-related and
  // (b) find a footprint-clear resume pose past the obstacle. Latched
  // (transient_local) subscription; updated on the node's single MutuallyExclusive
  // callback group so it is serialized against the BT tick (no extra mutex — see
  // bt_context.hpp). Null until the first costmap arrives → detour falls back.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  // --- Coverage controller rejoin (FTC turn fallback) ------------------------
  // FTC republishes the rest of the unit on the goal checker's plan topic when
  // its turn fallback rejoins the plan past a blocked turn; the front pose is an
  // exact pose of the unit, and path_progress_idx_ jumps to it
  // (strip_progress.hpp findControllerRejoin). Same topic as coverage_plan_pub_,
  // so this node also hears its own dispatches: only a message STAMPED after the
  // goal in flight was sent counts (FTC stamps its republish with now(); a
  // dispatched unit carries the coverage plan's older stamp). Serialized
  // against the tick like latest_costmap_ (one MutuallyExclusive group).
  struct ControllerRejoin
  {
    rclcpp::Time stamp;
    geometry_msgs::msg::Pose pose;
  };
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr controller_plan_sub_;
  std::optional<ControllerRejoin> controller_rejoin_;
  rclcpp::Time follow_goal_sent_stamp_{0, 0, RCL_ROS_TIME};
  // Blade-off detours taken on the CURRENT segment (unit). Reset to 0 per unit
  // (onStart and on advance() to the next unit). Bounded by max_detours_per_segment_.
  std::size_t detours_used_ = 0;
  /// Consecutive same-unit resumes that made no real progress (unit_resume.hpp).
  std::size_t unit_resumes_without_progress_ = 0;

  /// The FollowCoveragePath goal in flight was cut short at this index of the
  /// current unit because a dig zone starts there. When it succeeds the unit
  /// is NOT done: it is trimmed here and re-dispatched past the zone.
  std::optional<std::size_t> truncated_at_;
  /// sendCurrentSwath found nothing drivable left in the unit (its remainder
  /// lies inside dig zones). Consumed at the top of the next onRunning tick,
  /// which books the unit and advances — sendCurrentSwath itself cannot.
  bool unit_exhausted_by_dig_{false};
  /// The first unit's blade-off transit would repeat one TransitToStrip has
  /// just FAILED (same destination). Booked as skipped by the next onRunning
  /// tick instead of sending the identical transit a second time.
  bool unit_transit_already_failed_{false};
  /// dig_event_count value already handled.
  std::uint64_t dig_events_seen_{0};
  bool dig_recovery_active_{false};
  /// The interrupted goal was a FollowCoveragePath (vs a blade-off transit).
  bool dig_recovery_was_following_{false};
  bool dig_cancel_sent_{false};
  DigSettleState dig_settle_;
  std::chrono::steady_clock::time_point dig_settle_last_tick_{};
  // Ports read once in onStart.
  std::size_t max_detours_per_segment_ = 5;
  double detour_footprint_radius_m_ = 0.25;

  // Resume pose must be at least this far (euclidean) past the stuck pose so the
  // robot clears the obstacle. MUST exceed kSegmentTransitGap (0.6 m) so reaching
  // the resume pose always triggers the structural blade-off transit rather than a
  // blade-on drive-through (see sendCurrentSwath's gap guard, DetourResumeCfg).
  static constexpr double kDetourMinSkipM = 0.8;
  // Bounded forward search for a clear resume pose. Wider blockage → no resume →
  // fall back (skip the segment) instead of scanning the whole field.
  static constexpr double kDetourMaxSearchM = 8.0;
  // OccupancyGrid cost at/above which a cell is lethal for the clearance test.
  // MUST be 100 (TRUE lethal only): the published /global_costmap/costmap maps
  // LETHAL(254)->100 and INSCRIBED(253)->99. (Historical: until 2026-09-17 the
  // keepout wall was inflated and its inscribed band reached ~0.20 m inward;
  // the global costmap now inflates BEFORE keepout_filter, so only LiDAR marks
  // carry a 99 band. The threshold stays at TRUE lethal either way.) The outer
  // headland ring rides ON the recorded line (chassis_safety_inset 0), i.e.
  // permanently within 0.20 m of the boundary band — a 90 threshold counted
  // those 99-cells as lethal, so EVERY outer-ring abort was "obstacle
  // confirmed" (wedge) and NO ring pose was ever footprint-clear, which made
  // decideDetour return no-resume and FollowStrip skip the ENTIRE sub-path
  // (the whole field on a hole-free area). Field regression 2026-07-2x.
  static constexpr int8_t kDetourLethalCost = 100;
  // Radius of the "stalled beside an obstacle" wedge check around the stuck pose
  // (spec Part B). Fires the detour when lethal cells hug the robot even with no
  // dead-ahead blockage. Chassis half-width (~0.20 m) + margin. Safe against the
  // boundary band only because kDetourLethalCost is 100 and the TRUE lethal wall
  // sits enforce_boundary_margin_m (0.40 m) outside the outer ring (> 0.35).
  static constexpr double kDetourWedgeRadiusM = 0.35;

  // Arc-length the resume cursor is stepped FORWARD past a NON-obstacle FTC abort
  // (issue #389). When tryStartDetour declines (no lethal cell ahead / budget
  // spent / no costmap) the abort is not a skirtable obstacle, so persisting the
  // cursor AT the abort pose makes the next dispatch resume on the identical pose
  // and re-abort there forever (deterministic ~20 % stall, capping every
  // sub-path). Skipping this far ahead lands the re-dispatch beyond the offending
  // path feature and guarantees the cursor advances monotonically on every such
  // abort. MUST exceed kSegmentTransitGap (0.6 m) so the resume reaches the
  // skipped span as a blade-off transit (see sendCurrentSwath's gap guard) rather
  // than driving through it blade-on. Matches kDetourMinSkipM for the same reason.
  static constexpr double kNonObstacleAbortSkipM = 0.8;

  // Max time to hold (blade off) waiting for navigate_to_pose to become ready to
  // run a required inter-swath transit. If the server never comes up in this
  // window the swath is skipped (rolls to the next pass) — the robot never drives
  // to a >kSegmentTransitGap segment start blade-on.
  static constexpr double kTransitServerWaitSec = 5.0;

  // Blade spinup delay — wait before sending the FIRST segment goal
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  /// False when the first unit needs a blade-off transit: the blade stays off
  /// on start and the spin-up wait is skipped (bladeSpinupBeforeFirstUnit).
  bool blade_spinup_pending_{true};
  /// Transit watchdog (transitDeadlineSec): started when a blade-off transit
  /// goal is sent; on expiry the goal is cancelled ONCE and the existing
  /// aborted/cancelled path skips the unit.
  std::chrono::steady_clock::time_point transit_start_time_{};
  double transit_deadline_s_{0.0};
  bool transit_timeout_requested_{false};
  void armTransitWatchdog(double gap_m);

  /// Blade pause across a short LiDAR dropout (scan_pause.hpp): the coverage
  /// goal stays alive, only the blade is cut and later restored.
  ScanPauseState scan_pause_;
  std::chrono::steady_clock::time_point last_scan_pause_tick_{};
  /// Per-tick scan-pause step; returns true while the blade is paused. A
  /// blade-off transit may call this with allow_resume=false to notice a
  /// dropout at its completion without treating the unobserved transit time as
  /// proof of continuously fresh scans.
  bool stepScanPause(const std::shared_ptr<BTContext>& ctx, bool allow_resume = true);
  bool goal_sent_ = false;
  bool follow_goal_ever_sent_ = false;

  // This threshold applies only while handling a dig-truncated goal: reaching
  // the truncation point lets the unit continue past that dig zone. It is never
  // evidence that an interrupted coverage unit or area is complete.
  static constexpr double kPathCompleteFraction = 0.95;
};

// ---------------------------------------------------------------------------
// TransitToStrip — navigate to strip start using Nav2 navigate_to_pose
// ---------------------------------------------------------------------------

class TransitToStrip : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  TransitToStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_sec",
                                  -1.0,
                                  "Hard bound on the transit [s]; <= 0 derives it from the "
                                  "distance to the strip (transitDeadlineSec)")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// The transit ended ABORTED/CANCELED. Waits (bounded) for nav2's error code,
  /// then records the destination in BTContext::transit_to_strip_failed_at so
  /// FollowStrip does not send the identical transit again — unless the planner
  /// refused the robot's OWN pose (START_OCCUPIED): that refusal is instant and
  /// FollowStrip's own transit must see it to arm the escape (issue #487).
  BT::NodeStatus onFailed(const std::shared_ptr<BTContext>& ctx);
  /// Cancels a transit still running past its bound, once.
  void enforceDeadline(const std::shared_ptr<BTContext>& ctx);

  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  /// Watchdog: TransitToStrip used to have NO time bound — field 2026-09-21 it
  /// ran 53 s against a path its controller kept refusing.
  std::chrono::steady_clock::time_point start_time_{};
  double deadline_s_{0.0};
  bool timeout_requested_{false};
  /// nav2's error code for the failure classification (issue #487).
  std::shared_ptr<TransitResultSlot> nav_result_ = std::make_shared<TransitResultSlot>();
  bool failure_seen_{false};
  std::chrono::steady_clock::time_point failure_time_{};

  /// Terminal verdict from the result callback: a goal finished in the same
  /// instant it is accepted can lose its status message (action_outcome.hpp).
  std::shared_ptr<ActionOutcomeSlot> nav_outcome_ = std::make_shared<ActionOutcomeSlot>();
};

// ---------------------------------------------------------------------------
// DetourAroundObstacle — when FollowStrip aborts on a lookahead-collision,
// drive a short side-step path through the global planner so the robot
// physically gets out from in front of the obstacle (a person standing in
// the strip). The next strip iteration replans from the new pose; the
// `mow_progress` layer prevents re-cutting already-mowed cells.
//
// The detour is a NavigateToPose at (current_pose ⊕ forward·x̂_body
// + lateral·ŷ_body), routed via SmacPlanner over the local costmap which
// has the obstacle layer enabled — so the planner naturally curves around
// the obstruction rather than charging through it.
// ---------------------------------------------------------------------------

class DetourAroundObstacle : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  DetourAroundObstacle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("forward_m", 0.8, "Forward offset from current pose, body frame"),
        BT::InputPort<double>("lateral_m", 0.6, "Lateral offset (positive = left), body frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;

  /// Terminal verdict from the result callback: a goal finished in the same
  /// instant it is accepted can lose its status message (action_outcome.hpp).
  std::shared_ptr<ActionOutcomeSlot> nav_outcome_ = std::make_shared<ActionOutcomeSlot>();
};

// ---------------------------------------------------------------------------
// GetNextUnmowedArea — find next area with remaining strips
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

  /// Advance current_area_idx_ past completed/attempted areas and fire the
  /// next existence probe. Returns RUNNING (probe in flight) or FAILURE.
  BT::NodeStatus advanceAndProbe();

  // Existence probe: GetMowingArea(index).success is false once index passes
  // the last defined area. Area completion is tracked in-memory via the
  // swath-completion model (ctx->completed_areas), not the removed cell grid.
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::FutureAndRequestId>
      pending_future_;
  std::chrono::steady_clock::time_point call_start_;
  uint32_t current_area_idx_{0};
  uint32_t max_areas_{20};
  uint32_t areas_queried_{0};
  uint32_t areas_complete_{0};
  // Bounded retries for a transient get_mowing_area timeout — a momentary
  // service blip must NOT abort the whole run (and must not be mistaken for
  // "all areas complete"). Re-probe up to kMaxProbeRetries before failing.
  uint32_t probe_retries_{0};
  static constexpr uint32_t kMaxProbeRetries = 3;
  // Fleet rotation (BTContext::fleet_preferred_start): the scan starts at the
  // preferred index and, once the upper range is exhausted (probe returned
  // success=false), wraps ONCE to [0, preferred). Reset per onStart().
  bool fleet_wrap_pending_{false};
  uint32_t fleet_wrap_limit_{0};
  // Indices the skip loops passed over before the first probe of a range.
  // "success=false with nothing queried" is a CONFIG error (no areas at all)
  // only when nothing was skipped; with skips it is genuine completion (every
  // defined area is done, retired or assigned to another fleet member).
  uint32_t skipped_before_probe_{0};
};

// ---------------------------------------------------------------------------
// PlanCoverageArea — calls map_server's ~/get_mowing_area for the area's
// polygon (outer + obstacle holes), then asks mowgli_coverage's
// /plan_coverage action for the EXPLICIT segment list (headland rings +
// straight serpentine swaths — no turn geometry). Stores the segments in
// ctx->current_strip_segments (and the concatenated path in
// ctx->current_strip_path for the GUI); FollowStrip executes them one
// FollowCoveragePath goal at a time.
//
// Plans the whole area in one shot at each (re)start; resume is swath-based
// (FollowStrip skips segment indices already in ctx->area_completed_swaths —
// the plan is deterministic for a fixed polygon + params).
// ---------------------------------------------------------------------------

class PlanCoverageArea : public BT::StatefulActionNode
{
public:
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<PlanCoverage>;

  PlanCoverageArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    QueryRemaining,
    Dispatch,
    WaitingForGoal,
    WaitingForResult,
  };

  /// Build a PlanCoverage::Goal from the area polygon. The coverage
  /// geometry (operation_width, headland, insets) lives in the coverage
  /// server's parameters (injected at launch from mowgli_robot.yaml).
  PlanCoverage::Goal buildGoal(const mowgli_interfaces::msg::MapArea& area) const;

  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr srv_client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::FutureAndRequestId>
      srv_future_;

  rclcpp_action::Client<PlanCoverage>::SharedPtr action_client_;
  std::shared_future<PlanGoalHandle::SharedPtr> goal_future_;
  PlanGoalHandle::SharedPtr goal_handle_;
  std::shared_future<PlanGoalHandle::WrappedResult> result_future_;

  mowgli_interfaces::msg::MapArea area_;
  // Publishes the FULL plan (concatenation of all segments) for visualisation.
  // FollowStrip feeds the coverage controller one segment at a time (and
  // republishes only that segment on FollowCoveragePath/global_plan for the
  // goal checker), so without this the operator/GUI could only ever see a
  // single segment. Latched (transient_local) so a late GUI subscriber still
  // gets the whole plan for the current area.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_plan_pub_;
  Phase phase_{Phase::QueryRemaining};
  std::chrono::steady_clock::time_point phase_start_;
};

}  // namespace mowgli_behavior
