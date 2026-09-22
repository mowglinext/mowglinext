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
// TURN FALLBACK: improvise a blocked turn of the coverage plan instead of
// aborting the goal.
//
// Field 2026-09-22 (110-minute mow, bag mow-20260922): all 8 FTC WEDGED events
// ("no collision-free offset profile within the lattice") happened in TURNS —
// U-turns, omega loops, 70-107 deg kinks. At each, real lethal cells of the
// local costmap sat under the FRONT of the body at the turn poses: 7 times the
// hedge just past the recorded boundary, where the planner deliberately lets
// the body overhang the line at a turn (Invariant 5), once an unmapped object
// beside a drawn obstacle. The offset lattice can only move the line sideways,
// which cannot shorten a turn, so FTC wedged, reversed 0.30 m, held and aborted,
// and FollowStrip stepped past the abort, losing the turn and more.
//
// What this module decides, once, at the moment the lattice finds no profile:
//
//   1. Is the blockage IN A TURN? The first plan pose ahead of the robot whose
//      body (the lattice's: footprint + clearance margin, zero offset) covers a
//      lethal cell must lie where the plan heading sweeps at least
//      min_turn_rad. An obstacle on a straight is NOT handled here: the old
//      WEDGED -> reverse-escape -> hold -> abort path and FollowStrip's detour
//      stay in charge of it.
//   2. A REJOIN pose on the plan past the blockage, searched forward pose by
//      pose (least skipped path first) up to max_rejoin_arc_m, such that
//        - the body at the rejoin pose, at zero offset, covers no lethal cell;
//        - FTC can follow from there (a caller predicate: the controller runs
//          its own lattice solve from the rejoin);
//        - from a START pose — the robot, possibly backed STRAIGHT up by up to
//          max_reverse_m along a rear sweep that is itself clear — a rotation
//          in place to the straight CONNECTOR heading, the connector itself and
//          a rotation in place at the rejoin to the plan heading are all clear.
//   3. The motion that results: (reverse) -> rotate -> straight -> rotate ->
//      follow from the rejoin. FTC executes it with its existing machinery (a
//      bounded straight reverse, then its PIVOT state at runtime corner pairs
//      spliced into its working plan — the planner's pivot-corner contract of
//      mowgli_interfaces/coverage_geometry.hpp), and every check below runs
//      again every control cycle while the motion executes.
//
// The checks are the ones FTC already runs, so what is planned here is what the
// executing controller will accept:
//   * rotations and the reverse: the real footprint (getRobotFootprint(), incl.
//     Nav2 footprint_padding) against TRUE-lethal local cells — pivotSweepBlocked's
//     test, same probe spacing (the farthest vertex moves at most one cell);
//   * plan poses and the connector: the lattice body (footprint widened by
//     obstacle_clearance_margin) against TRUE-lethal local cells — the lattice's
//     zero-offset node test, since the lattice keeps checking the connector
//     every tick;
//   * zone: every motion OFF the plan (rotations, connector) also tests the body
//     AXIS (rear, base_link, front) against the global zone band (>= 99), the
//     lattice's off-line zone rule — the band already contains the body, so the
//     axis counts it exactly once (CLAUDE.md "count the body once"). The reverse
//     retraces the path just driven and, like the existing reverse-escape, is
//     not zone-tested; poses ON the plan never are (the plan is authoritative).
//
// Pure (no ROS node): Costmap2D + geometry_msgs only, unit-tested in
// test_ftc_turn_fallback.cpp and replayed on field costmaps by
// test/turn_fallback_replay.cpp.

#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_nav2_plugins/obstacle_deviation.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mowgli_nav2_plugins
{

/// A planar pose (costmap frame). `yaw` is the heading of base_link.
struct FallbackPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TurnFallbackCfg
{
  /// Longest straight reverse before the first rotation (m). 0 = rotate where
  /// the robot stands or not at all.
  double max_reverse_m{0.40};
  /// Resolution of the reverse search (m).
  double reverse_step_m{0.05};
  /// Longest stretch of plan the rejoin may skip, measured from the plan pose
  /// nearest the robot (m).
  double max_rejoin_arc_m{3.0};
  /// A blockage is "in a turn" when the plan heading sweeps at least this much
  /// (rad) over [blocked - turn_behind_m, blocked + turn_ahead_m] of arc.
  double min_turn_rad{45.0 * 3.14159265358979323846 / 180.0};
  double turn_behind_m{0.30};
  /// Covers the body front (0.54 m ahead of base_link) reaching the turn
  /// before base_link does.
  double turn_ahead_m{0.75};
  /// How far ahead of the robot the blockage is looked for (m).
  double blockage_scan_m{2.5};
  /// The executing PIVOT turns the SHORT way from the heading the robot
  /// actually has when it gets there, which is off the planned one by up to
  /// its pivot tolerance and tracking error. Up to this rotation (rad) that is
  /// certainly the planned direction and only that sweep is checked; a larger
  /// one (up to 180 deg) may go either way, so the FULL turn must be clear.
  double unambiguous_rotation_rad{160.0 * 3.14159265358979323846 / 180.0};
  /// Extra margin (m, every direction) around the footprint and the body when
  /// PLANNING the rotations, the reverse and the connector. The executing
  /// checks (pivot sweep gate, lattice, rear probe) test the bare shapes every
  /// cycle; a plan with no slack would be stopped by the first noisy scan,
  /// half-way through, next to the obstacle.
  double plan_margin_m{0.05};
  /// A connector shorter than this is not driven: the start pose rotates
  /// straight to the rejoin heading (m).
  double min_connector_m{0.05};
  /// Most (rejoin, reverse) combinations examined before giving up: bounds the
  /// one control tick the search runs in. The 2026-09-22 wedges needed 37-454
  /// (3-36 ms on an Apple-silicon dev container).
  std::size_t max_evaluations{1000};
};

enum class TurnFallbackVerdict
{
  /// A fallback motion was found (TurnFallbackPlan fields are valid).
  kPlanned,
  /// Nothing lethal under the body on the plan ahead: not this module's case.
  kNoBlockage,
  /// The blockage is not in a turn (obstacle on a straight).
  kNotATurn,
  /// In a turn, but no rejoin is reachable safely within the bounds.
  kNoRejoin,
  /// Missing costmap / footprint / plan.
  kBadInput,
};

const char* ToString(TurnFallbackVerdict verdict);

struct TurnFallbackPlan
{
  TurnFallbackVerdict verdict{TurnFallbackVerdict::kBadInput};
  /// Window index of the first plan pose whose body covers a lethal cell.
  std::size_t blocked{0};
  /// Its arc length from the plan pose nearest the robot (window index 0).
  double blocked_arc_m{0.0};
  /// Heading sweep of the plan around it (rad).
  double turn_rad{0.0};
  /// How far the rear sweep behind the robot is clear (m, <= max_reverse_m).
  double reverse_limit_m{0.0};

  // --- valid when verdict == kPlanned -----------------------------------------
  double reverse_m{0.0};
  /// Window index of the rejoin pose.
  std::size_t rejoin{0};
  /// Plan arc from window index 0 to the rejoin: the path this fallback skips.
  double skipped_arc_m{0.0};
  double connector_m{0.0};
  /// Signed rotations (rad, CCW positive, shortest direction).
  double rotate_start_rad{0.0};
  double rotate_rejoin_rad{0.0};
  /// Where the first rotation happens (the robot backed up by reverse_m).
  FallbackPose start;
  /// The rejoin plan pose.
  FallbackPose rejoin_pose;

  /// (rejoin, reverse) combinations examined.
  std::size_t evaluated{0};
  /// Why the last examined candidate failed (kNoRejoin), for the log.
  std::string why;
};

/// Everything the planner reads, in the LOCAL COSTMAP frame.
struct TurnFallbackProblem
{
  const nav2_costmap_2d::Costmap2D* costmap{nullptr};
  /// Zone band (global costmap >= 99), for motion off the plan only. Leave
  /// `costmap` null to skip (confine_deviation_to_zone off).
  BoundaryGuard guard;
  /// The real footprint (getRobotFootprint()): rotations and the reverse.
  ObstacleDeviation::Footprint footprint;
  /// The lattice body (footprint + clearance margin): plan poses, connector.
  ObstacleDeviation::Footprint body;
  /// Plan window: plan[0] is the plan pose nearest the robot, then forward.
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  /// The robot's base_link pose.
  FallbackPose robot;
  /// FTC can resume FOLLOWING from window index `i` (the controller solves its
  /// offset lattice there). Empty = only the body at the rejoin pose is tested.
  std::function<bool(std::size_t)> followable;
};

/// Decide the fallback motion (see the file comment). Never throws; the
/// verdict says whether a motion was found and, if not, why.
TurnFallbackPlan PlanTurnFallback(const TurnFallbackProblem& problem, const TurnFallbackCfg& cfg);

// ── The individual checks. Shared by the planner, the per-tick execution in
//    FTCController, and the tests. ──────────────────────────────────────────

/// Heading sweep (rad) of `plan` over [arc(center) - behind, arc(center) + ahead]:
/// the range of the unwrapped pose headings, so an omega loop reads > 180 deg.
double PlanHeadingSweep(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                        std::size_t center,
                        double behind_m,
                        double ahead_m);

/// Rotation in place about (x, y) from `from_yaw` to `to_yaw` (shortest
/// direction): the footprint at every probe heading covers no TRUE-lethal
/// cell, and (guard set) the footprint axis stays out of the zone band.
bool RotationSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                        const BoundaryGuard& guard,
                        const ObstacleDeviation::Footprint& footprint,
                        double x,
                        double y,
                        double from_yaw,
                        double to_yaw);

/// A full turn in place about (x, y) is clear (RotationSweepClear over 360 deg).
bool FullTurnClear(const nav2_costmap_2d::Costmap2D& costmap,
                   const BoundaryGuard& guard,
                   const ObstacleDeviation::Footprint& footprint,
                   double x,
                   double y,
                   double yaw);

/// Straight translation from `from` to (to_x, to_y) at heading `from.yaw`: the
/// body at every probe (one cell apart, both ends included) covers no
/// TRUE-lethal cell and (guard set) its axis stays out of the zone band.
bool StraightSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                        const BoundaryGuard& guard,
                        const ObstacleDeviation::Footprint& body,
                        const FallbackPose& from,
                        double to_x,
                        double to_y);

/// The strip a straight REVERSE of `distance_m` sweeps behind the footprint:
/// from its rear edge back by `distance_m` + `margin_m`, as wide as the
/// footprint + `margin_m` each side (base frame). The body's own current area
/// is not part of it: backing straight up never brings the rest of the body
/// onto anything new.
ObstacleDeviation::Footprint RearStrip(const ObstacleDeviation::Footprint& footprint,
                                       double distance_m,
                                       double margin_m);

/// Straight REVERSE of `distance_m` from `robot` (along -heading): the rear
/// strip covers no TRUE-lethal cell. No zone test (it retraces the path just
/// driven, like the existing reverse-escape). A distance <= 0 is clear.
bool ReverseSweepClear(const nav2_costmap_2d::Costmap2D& costmap,
                       const ObstacleDeviation::Footprint& footprint,
                       const FallbackPose& robot,
                       double distance_m,
                       double margin_m = 0.0);

/// `pose` backed straight up by `distance_m`.
FallbackPose BackedUp(const FallbackPose& pose, double distance_m);

/// Distance (m) from the footprint polygon placed at `pose` to the nearest
/// TRUE-lethal cell centre, 0 when one is inside, capped at `max_m`.
double FootprintClearance(const nav2_costmap_2d::Costmap2D& costmap,
                          const ObstacleDeviation::Footprint& footprint,
                          const FallbackPose& pose,
                          double max_m);

/// Clearances (m, capped at `max_m`) of every leg of a planned fallback: the
/// smallest distance from the swept body to a TRUE-lethal cell.
struct TurnFallbackClearances
{
  double reverse_m{std::numeric_limits<double>::infinity()};
  double rotate_start_m{std::numeric_limits<double>::infinity()};
  double connector_m{std::numeric_limits<double>::infinity()};
  double rotate_rejoin_m{std::numeric_limits<double>::infinity()};
};

TurnFallbackClearances MeasureTurnFallbackClearances(const TurnFallbackProblem& problem,
                                                     const TurnFallbackPlan& plan,
                                                     double max_m);

// ── Bounds FTCController enforces on its turn_fallback_* parameters ─────────

inline constexpr double kTurnFallbackMaxReverseCapM = 1.0;
inline constexpr double kTurnFallbackMinRejoinArcM = 0.2;
inline constexpr double kTurnFallbackMaxRejoinArcCapM = 10.0;
/// Plan the robot must drive past a rejoin before another fallback may engage:
/// a turn that is still blocked right after the rejoin takes the old WEDGED
/// path instead of looping.
inline constexpr double kTurnFallbackRearmM = 1.0;
/// The fallback reverse gives up (and re-plans from where it stands) after
/// kTurnFallbackReverseTimeFactor x the nominal time plus this slack (s).
inline constexpr double kTurnFallbackReverseTimeFactor = 2.0;
inline constexpr double kTurnFallbackReverseTimeSlackS = 2.0;
/// Floor of the reverse speed used for that time cap (m/s).
inline constexpr double kTurnFallbackMinReverseSpeedMps = 0.02;
/// A reverse this close to its target is done (m).
inline constexpr double kTurnFallbackReverseDoneM = 0.005;
/// Look-behind of the per-tick rear probe while reversing (m): the
/// reverse-escape's.
inline constexpr double kTurnFallbackRearProbeM = 0.20;
/// Pose spacing of a spliced connector (m): the coverage planner's sampling.
inline constexpr double kTurnFallbackConnectorStepM = 0.05;
/// Clearances are measured (and logged) up to this distance (m).
inline constexpr double kTurnFallbackClearanceLogCapM = 0.5;

// ── Plan-side helpers shared by FTCController and the offline replay ─────────

/// How far behind the carrot the plan pose nearest the robot is looked for (m).
/// The robot trails the carrot by at most the carrot lead cap (0.30 m shipped);
/// the margin covers a lateral skirt and a reverse-escape in progress.
inline constexpr double kFallbackRobotSearchBackM = 1.0;

/// Plan window of a fallback, as plan indices [first, last). `first` is the
/// plan pose nearest (robot_x, robot_y) among those at most `back_m` of arc
/// behind `carrot_idx` (ties to the later pose); `last` stops once the arc
/// from `first` exceeds `ahead_m`. The plan is in the frame of the robot
/// position (map for FTC).
std::pair<std::size_t, std::size_t> FallbackWindow(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    std::size_t carrot_idx,
    double robot_x,
    double robot_y,
    double back_m,
    double ahead_m);

}  // namespace mowgli_nav2_plugins
