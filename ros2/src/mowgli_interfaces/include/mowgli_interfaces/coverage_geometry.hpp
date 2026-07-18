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
constexpr double kSegmentTransitGapM = 0.6;

}  // namespace mowgli_interfaces::coverage_geometry
