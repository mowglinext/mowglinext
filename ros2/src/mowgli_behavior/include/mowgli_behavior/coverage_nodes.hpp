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
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/detour_resume.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
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
  bool valid = false;    ///< false → no resumable cursor; mow fresh from pose 0.
  std::size_t unit = 0;  ///< index of the sub-path the cursor lands in (units 0..unit-1 are done).
  std::size_t local = 0;  ///< local offset to trim to; 0 → resume at the unit's front pose.
};

/// Map an absolute resume cursor (index into the sub-path concatenation) to the
/// sub-path unit and local offset at which mowing resumes. Applies the guards
/// FollowStrip uses: a cursor of 0 (or within 2 poses of the very end) is not
/// resumable, and a landing offset is only trimmed mid-unit when it is strictly
/// interior (local > 0 and at least 2 poses before the unit end) — otherwise the
/// resume snaps to the unit's front. `total_poses` is the sum of unit sizes.
ResumeLocation resolveResumeLocation(const std::vector<nav_msgs::msg::Path>& units,
                                     std::size_t cursor, std::size_t total_poses);

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

  // Start an explicit transit when the segment start is farther than this.
  // Below it, RotationShim+MPPI close the gap themselves (adjacent swaths are
  // one op_width ≈ 0.16 m apart). Single-sourced from mowgli_interfaces so
  // this matches mowgli_coverage's planning-side split threshold (the server
  // decides which gaps become a separate drivable_subpaths entry using the
  // exact same value) — see coverage_geometry.hpp for why the two sides must
  // agree. Public (unlike the rest of this class's tuning constants) so the
  // single-source regression test can assert the equality directly.
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
  // Dispatch swaths_[swath_idx_]: if the robot is farther than
  // kSegmentTransitGap from the segment start, first run a NavigateToPose
  // transit (sets transit_active_); otherwise send the FollowPath goal
  // directly. Returns false only if a client is missing.
  bool sendCurrentSwath(const std::shared_ptr<BTContext>& ctx);
  // Send the FollowPath goal for the current segment (no gap check).
  bool sendFollowGoal(const std::shared_ptr<BTContext>& ctx);
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

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  // Mirrors the active segment onto the coverage controller's global_plan
  // topic so the PathProgressGoalChecker (coverage_goal_checker) can track
  // per-pose progress (MPPI/RotationShim does not republish the plan).
  // Latched (transient_local) so a late-subscribing goal checker still
  // receives the current segment.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr coverage_plan_pub_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
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

  // The drivable units being executed: the hole-free continuous SUB-PATHS
  // (ctx->current_strip_subpaths, issue #333), or a single continuous path when
  // the field has no holes. FollowStrip drives one FollowCoveragePath goal per
  // unit and bridges gaps between units with a blade-off Nav2 transit.
  std::vector<nav_msgs::msg::Path> swaths_;
  std::size_t swath_idx_ = 0;
  std::size_t swaths_skipped_ = 0;
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
  // Blade-off detours taken on the CURRENT segment (unit). Reset to 0 per unit
  // (onStart and on advance() to the next unit). Bounded by max_detours_per_segment_.
  std::size_t detours_used_ = 0;
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
  // LETHAL(254)->100 and INSCRIBED(253)->99, and the inscribed band extends
  // inflation_radius (0.20 m) from every keepout wall BY DESIGN. The outer
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

  // Max time to hold (blade off) waiting for navigate_to_pose to become ready to
  // run a required inter-swath transit. If the server never comes up in this
  // window the swath is skipped (rolls to the next pass) — the robot never drives
  // to a >kSegmentTransitGap segment start blade-on.
  static constexpr double kTransitServerWaitSec = 5.0;

  // Blade spinup delay — wait before sending the FIRST segment goal
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  bool goal_sent_ = false;

  // A FollowCoveragePath goal that ABORTS at or beyond this fraction of the
  // path is treated as COMPLETE rather than skipped. FTC zeroes linear.x once
  // it leaves FOLLOWING and parks up to max_goal_distance_error (~0.5 m) short
  // of the final pose; the PathProgressGoalChecker then can't fire (robot
  // stopped just outside xy tolerance) and the progress_checker aborts the goal
  // with err 105 at ~100 % tracked. Without this, that abort was scored as a
  // skip, the near-100 % resume cursor was discarded (resume+2 >= size), the
  // area was never marked complete, and GetNextUnmowedArea re-mowed it from
  // scratch — an endless re-mow loop. Matches the goal-checker progress_threshold
  // (0.95): reaching >=95 % of poses means the area is mowed.
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
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
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
