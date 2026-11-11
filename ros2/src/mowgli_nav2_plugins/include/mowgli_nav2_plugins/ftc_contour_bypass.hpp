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
// CONTOUR BYPASS (prototype, NOT wired into FTCController): a Bug2-style
// obstacle bypass on the coverage plan, evaluated offline against the turn
// fallback (ftc_turn_fallback.hpp) and FollowStrip's blade-off detour.
//
// Bug2 (Lumelsky & Stepanov) drives the straight "M-line" from start to goal;
// on contact it follows the obstacle boundary and leaves it where it meets the
// M-line again CLOSER to the goal. Here the M-line is the coverage plan itself:
//
//   1. HIT: the first plan pose ahead whose lattice body (footprint +
//      clearance margin) covers a TRUE-lethal local cell. The bypass leaves
//      the plan at the last pose before it that is clear with the planning
//      margin (or, when the robot already stands too close to turn, at the
//      robot backed STRAIGHT up along a clear rear strip, like the fallback).
//   2. CONTOUR: from there the walk drives straight on until it engages the
//      obstacle, then follows an iso-distance line of a distance field built
//      from the TRUE-lethal local cells AND the zone band (the recorded
//      boundary is a virtual wall), keeping the obstacle on one side — a Bug
//      with a range sensor rather than a contact sensor. Each 1-cell step takes
//      the desired heading (contour tangent + proportional pull onto
//      follow_distance_m), or the nearest heading to it, that keeps the body
//      clear, its axis out of the zone band (the lattice's off-line rule) and
//      whose rotation in place is clear for the real footprint. Both sides are
//      walked.
//   3. LEAVE: the first step that comes back within leave_tol_m of a plan pose
//      PAST the hit (Bug2's "closer to the goal" = more plan arc), where the
//      body is clear on the line, the robot can rotate to the plan heading and
//      FTC can follow from there (caller predicate, as for the fallback).
//   4. The walk is then SIMPLIFIED into straight legs joined by pivots — the
//      motion FTC can execute (PIVOT at runtime corners + straight connectors,
//      exactly like the turn fallback): greedy farthest-reach, every leg
//      re-checked with StraightSweepClear and every corner with
//      RotationSweepClear, using the planning margin.
//
// Bug2's failure rules are kept: the walk fails when it comes back to its leave
// point (the plan ahead is unreachable round this side), when no heading is
// clear, when it leaves the local costmap or exceeds max_contour_m.
//
// Unlike textbook Bug2 the side is not fixed: both are walked and the shorter
// (time) contour wins. The result is a DECISION, made on one costmap snapshot;
// the executing controller would re-check every leg every cycle, as it does for
// the turn fallback.
//
// Pure (no ROS node), unit-tested in test_ftc_contour_bypass.cpp and replayed on
// field costmaps by lattice_replay --contour.

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_nav2_plugins
{

/// Default ContourBypassCfg::max_evaluations. A 5 m walk round a 3 m bed costs
/// ~15 000; the synthetic worst case (lawn cut in two, both sides walked to a
/// 15 m contour and failing) ~71 000 (test_ftc_contour_bypass). The budget sits
/// above the first and caps anything past the second.
inline constexpr std::size_t kContourBypassDefaultMaxEvaluations = 100000;

struct ContourBypassCfg
{
  /// Wall-follower step (m); one local costmap cell.
  double step_m{0.05};
  /// Heading resolution of the candidate search around the desired heading.
  double heading_step_rad{10.0 * 3.14159265358979323846 / 180.0};
  /// Furthest a step may deviate from the desired heading (rad).
  double max_deviation_rad{120.0 * 3.14159265358979323846 / 180.0};
  /// Contour followed: base_link distance to the nearest obstacle / zone cell
  /// (m). The planning body reaches 0.285 m + clearance margin + plan margin
  /// sideways; the slack above that is what lets the simplification replace
  /// the curved contour by long straight chords (short legs after a pivot
  /// leave FTC no room to settle its heading before the next corner).
  double follow_distance_m{0.65};
  /// Proportional pull back onto the contour (1/m).
  double follow_gain{3.0};
  /// Before contact the walk drives straight on (Bug2's motion-to-goal along
  /// the M-line). It engages the contour when the distance drops to
  /// follow_distance_m + engage_slack_m, or EARLIER — at the reach of the
  /// footprint grown by the plan margin + two cells — when the obstacle is
  /// ahead (gradient against the
  /// heading): base_link is the pivot, and a robot that reaches a frontal wall
  /// can no longer turn away from it.
  double engage_slack_m{0.05};
  /// Longest contour walked per side (m).
  double max_contour_m{8.0};
  /// Furthest plan arc past the hit the leave point may be (m).
  double max_rejoin_arc_m{8.0};
  /// A walk position this close to a plan pose is ON the M-line (m).
  double leave_tol_m{0.10};
  /// The leave pose must lie at least this far (plan arc) past the hit (m).
  double min_leave_past_hit_m{0.05};
  /// Most the walk heading may differ from the plan heading at the leave (rad):
  /// Bug2 leaves when it CROSSES the M-line heading on, not against it.
  double max_leave_heading_rad{100.0 * 3.14159265358979323846 / 180.0};
  /// The walk has closed on its own start (Bug2 "unreachable") once it is this
  /// close again after at least loop_min_walk_m of contour (m).
  double loop_tol_m{0.10};
  double loop_min_walk_m{1.0};
  /// Longest straight reverse before the walk, when it cannot start where the
  /// robot stands (m); 0 = never reverse.
  double max_reverse_m{0.40};
  double reverse_step_m{0.10};
  /// Extra margin around the footprint and the body while PLANNING (m), as the
  /// fallback's plan_margin_m.
  double plan_margin_m{0.05};
  /// How far ahead of the robot the hit is looked for (m).
  double blockage_scan_m{2.5};
  /// Nominal speeds, for the time estimate that picks the side.
  double drive_speed_mps{0.25};
  double turn_speed_rps{0.6};
  /// Footprint-test budget for the whole plan (both sides, every start,
  /// simplification included), in the units of ContourBypassPlan::evaluations.
  /// The planner runs inside one control tick under the costmap lock: once the
  /// budget is spent every further test reads "blocked" and the walk fails.
  std::size_t max_evaluations{kContourBypassDefaultMaxEvaluations};
  /// The robot is ON the plan (within kContourStartOnPlanM of a window pose).
  /// When it is not — wedged mid-skirt, beside its line — a start further
  /// along the plan would first drive the plan back onto the line beside the
  /// obstacle, a stretch nothing here checked: only the robot's own pose (and
  /// reverses from it) may start the walk.
  bool start_on_plan{true};
};

enum class ContourBypassVerdict
{
  kPlanned,
  kNoBlockage,
  /// Neither side reached a leave point (see ContourSide::why).
  kNoLeave,
  kBadInput,
};

const char* ToString(ContourBypassVerdict verdict);

/// One side's walk.
struct ContourSide
{
  /// +1: obstacle kept on the RIGHT (the walk turns left round it);
  /// -1: obstacle on the LEFT.
  int side{0};
  bool reached{false};
  /// Why the walk failed (empty when reached).
  std::string why;
  /// The raw walk, one pose per step, from the leave-the-plan pose to the
  /// leave pose (headings = direction of travel).
  std::vector<FallbackPose> walk;
  /// The executable motion: vertices of straight legs; the robot pivots at
  /// each vertex to the next leg's heading. Starts at the leave-the-plan pose
  /// (heading = robot heading), ends at the rejoin plan pose (plan heading).
  std::vector<FallbackPose> legs;
  double reverse_m{0.0};
  /// Window index of the plan pose the walk starts from (0 when it starts from
  /// the backed-up robot).
  std::size_t start_idx{0};
  /// Window index of the rejoin plan pose.
  std::size_t rejoin{0};
  double walk_m{0.0};
  double legs_m{0.0};
  /// Sum of |pivot| over the legs (rad) and the number of pivots > 1 deg.
  double rotation_rad{0.0};
  std::size_t pivots{0};
  /// Plan arc between the leave-the-plan pose and the rejoin: what the plan
  /// would have mowed there (m).
  double skipped_arc_m{0.0};
  /// Smallest real-footprint clearance to a TRUE-lethal cell along the legs
  /// (m, capped at 0.5).
  double min_clearance_m{0.0};
  /// legs_m / drive_speed + rotation / turn_speed + reverse / drive_speed (s).
  double est_time_s{0.0};
};

struct ContourBypassPlan
{
  ContourBypassVerdict verdict{ContourBypassVerdict::kBadInput};
  /// Window index of the first blocked plan pose and its arc from index 0.
  std::size_t blocked{0};
  double blocked_arc_m{0.0};
  /// Window index of the last clear plan pose before it, where the walk
  /// leaves the plan (robot pose when the walk starts from a reverse).
  std::size_t leave_plan{0};
  ContourSide left;  // side = +1
  ContourSide right;  // side = -1
  /// The chosen side (+1 / -1), 0 when none reached.
  int chosen{0};
  /// Footprint tests performed (both sides, incl. simplification).
  std::size_t evaluations{0};

  const ContourSide* Best() const
  {
    if (chosen > 0)
      return &left;
    if (chosen < 0)
      return &right;
    return nullptr;
  }
};

// ── Bounds FTCController enforces on its contour_bypass_* parameters ────────

/// How close the robot must be to a window pose for a walk to start further
/// along the plan (ContourBypassCfg::start_on_plan).
inline constexpr double kContourStartOnPlanM = 0.10;

inline constexpr double kContourBypassMinContourM = 1.0;
inline constexpr double kContourBypassMaxContourCapM = 15.0;
inline constexpr double kContourBypassMinRejoinArcM = 0.5;
inline constexpr double kContourBypassMaxRejoinArcCapM = 10.0;
/// The contour must keep the body (half-width 0.285 m + margins) off the
/// obstacle, and stay a skirt, not a detour across the lawn.
inline constexpr double kContourBypassMinFollowM = 0.35;
inline constexpr double kContourBypassMaxFollowM = 1.0;
/// Planning margin of the contour legs (m): twice the turn fallback's. The
/// legs are skirts driven blade-on along an obstacle the LiDAR sees, and the
/// simplification cuts every corner down to this margin.
inline constexpr double kContourBypassPlanMarginM = 0.10;
inline constexpr double kContourBypassMinTimeoutS = 5.0;
inline constexpr double kContourBypassMaxTimeoutS = 300.0;

/// Same inputs as the turn fallback (local costmap frame).
using ContourBypassProblem = TurnFallbackProblem;

ContourBypassPlan PlanContourBypass(const ContourBypassProblem& problem,
                                    const ContourBypassCfg& cfg);

}  // namespace mowgli_nav2_plugins
