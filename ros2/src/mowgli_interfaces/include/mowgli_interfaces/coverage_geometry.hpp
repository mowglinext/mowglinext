#pragma once

namespace mowgli_interfaces::coverage_geometry
{

// Max gap (metres) between the end of one drivable sub-path segment and the
// start of the next before it must be treated as a RELOCATION (blade-off Nav2
// transit) instead of a continuous, blade-on connector join.
//
// This threshold is used on BOTH sides of the plan/execute split and MUST
// match, or the two sides disagree about which gaps are "close enough to
// drive through":
//   - mowgli_coverage (coverage_planning.cpp, buildContinuousSubPaths): the
//     PLANNING side. A gap above this is a lobe change / relocation across a
//     concave bite — an in-bounds Dubins connector for it "works" but mows a
//     long diagonal across the middle of the lawn (user report) — so the
//     server splits the path here into a separate drivable_subpaths entry
//     instead of bridging it with a blade-on connector.
//   - mowgli_behavior (coverage_nodes.hpp/cpp, FollowStrip): the EXECUTION
//     side. FollowStrip re-decides transit-vs-drive-through at runtime from
//     the segment start distance using the same threshold.
// If the two values diverge: the BT could drive blade-on across a gap the
// server already deemed a relocation (diagonal crossing hazard), or the BT
// could insert a redundant blade-off transit for a gap the server already
// left as one continuous sub-path.
//
// The planner ALSO uses it as the length bound of a PIVOT JOIN (below): a
// straight connector longer than this is a relocation, never a turn-around
// between adjacent passes, so it still splits the sub-path.
constexpr double kSegmentTransitGapM = 0.6;

// ── PIVOT CORNER CONTRACT (mowgli_coverage planner -> FTCController) ─────────
//
// When no forward turn-around arc fits between two ADJACENT passes (a thin
// headland apron: num_headland_passes auto/none), the planner may keep the join
// inside the sub-path as a short straight connector whose zero-radius corners
// the robot drives as IN-PLACE PIVOTS instead of splitting the sub-path into a
// blade-off Nav2 transit per swath end (field 2026-09-21: 128 sub-paths on a
// 152 m² lawn).
//
// Encoding (no message change): a pivot corner is TWO CONSECUTIVE POSES AT THE
// SAME POSITION — the first carries the INCOMING heading, the second the
// OUTGOING heading. That is how an in-place rotation is naturally expressed in
// a nav_msgs/Path. The planner guarantees:
//   * two consecutive poses closer than kPivotCornerMaxStepM appear ONLY at a
//     pivot corner (every other near-coincident pose is collapsed), and
//   * a pivot corner turns by MORE than kPivotCornerMinTurnRad (a smaller kink
//     is left as an ordinary vertex FTC tracks while moving, the same 15° the
//     planner already accepts for an aligned straight connector).
// FTC therefore treats ONLY such a pair as a pivot: ordinary curvature (the
// forward turn-around arcs, corner fillets, filleted ring corners) never has a
// zero-length step and can never trigger one. FTC detects at half the planner's
// turn threshold so float noise in the pose yaws cannot hide a real corner.
constexpr double kPivotCornerMaxStepM = 1e-4;
constexpr double kPivotCornerMinTurnRad = 15.0 * 3.14159265358979323846 / 180.0;

}  // namespace mowgli_interfaces::coverage_geometry
