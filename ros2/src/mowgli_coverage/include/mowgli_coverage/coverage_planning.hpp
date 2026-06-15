// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Simple boustrophedon coverage planner on Fields2Cover v3 — separated from
// coverage_server.cpp so it can be unit-tested against the real F2C library
// without standing up the ROS action server (test/test_coverage_planning).
//
// Design (2026-06-12 re-implementation, after two days of fighting turn
// planners): the robot is a diff-drive that PIVOTS IN PLACE, so the coverage
// plan contains NO turn geometry at all. F2C contributes exactly three things:
//   1. ConstHL::generateHeadlandSwaths — N concentric headland rings (mowed,
//      outermost first),
//   2. ConstHL::generateHeadlands     — the mainland left inside the rings,
//   3. BruteForce + BoustrophedonOrder — straight serpentine swaths over the
//      mainland (each disjoint clip of a sweep line is its OWN swath, so
//      concave boundaries and interior holes are handled without
//      decomposition — verified in F2C v3 Swaths::append).
// The output is a list of explicit, ordered, individually-drivable segments.
// Turns between segments are the navigation stack's job (RotationShim in-place
// pivot + MPPI straight tracking), NOT F2C's: every prior design that let F2C
// plan turns (Dubins Ω-loops, CC-Dubins arcs, Reeds-Shepp cusps) produced
// geometry this chassis could not track, and the downstream heading-jump
// re-segmentation heuristic silently failed on the smooth arcs.

#ifndef MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_
#define MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_

#include <cstddef>
#include <utility>
#include <vector>

// Fields2Cover v3 umbrella header — f2c::types::*, f2c::hg::ConstHL,
// f2c::sg::BruteForce, f2c::rp::BoustrophedonOrder.
#include "fields2cover.h"

namespace mowgli_coverage
{

// One planned coverage, as explicit drivable segments.
struct BoustrophedonPlan
{
  // Densified headland ring polylines (points ~0.10 m apart), outermost ring
  // first. Each entry is one closed drivable loop (a ring of a field with
  // holes contributes one entry per disjoint loop). Driven continuously.
  std::vector<std::vector<std::pair<double, double>>> rings;
  // Straight mainland swaths in serpentine order: {start, end} per swath,
  // direction already alternated by BoustrophedonOrder. The robot pivots in
  // place at each swath start.
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> swaths;
  // Swath heading actually used (rad, map frame) — for logging.
  double swath_angle_rad = 0.0;
};

// Plan boustrophedon coverage of `field_cell` (outer ring + optional holes).
//
//   op_width             swath spacing = F2C cov_width (m)
//   headland_width       desired headland band width (m); the ring count is
//                        ceil(headland_width / op_width), min 1, unless
//                        num_headland_passes_override > 0 forces a count
//   chassis_safety_inset polygon pull-back applied before everything (m)
//   mow_angle_rad        fixed swath heading; < 0 → auto (minimise swath count)
//   min_swath_length     drop straight swaths shorter than this (m)
//
// Geometry: safe = inset(field, chassis_safety_inset); rings are n_rings
// concentric loops spaced op_width inside safe; mainland = inset(safe,
// n_rings * op_width) so the swaths butt against the innermost ring's cut.
//
// Returns a plan whose rings/swaths may BOTH be empty (field too small after
// insets — the caller reports failure). Throws on internal F2C errors.
BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length);

// Flatten a BoustrophedonPlan into ONE continuous, cusp-free, in-bounds
// polyline so an MPPI-class sampling controller can track it without the
// bimodal dither/spin it does at sharp ~180° reversals.
//
// The robot drives the plan as: all rings (densified closed loops, outermost
// first) then all swaths (straight start→end, serpentine order). Reversals
// occur (a) between consecutive concentric rings, (b) at every swath U-turn,
// and as a big jump between the last ring and the first swath. At each such
// transition this inserts a smooth FORWARD turn-around connector: a forward-
// only Dubins path (fixed `turn_radius`, no reversing) tangent to the previous
// segment's exit heading and the next segment's entry heading, so there is NO
// cusp at the junction. For a ~180° reversal at ~op_width spacing this yields
// the expected teardrop / Ω loop into the already-mowed (headland) side. Of the
// six forward Dubins words (LSL/RSR/LSR/RSL/RLR/LRL) the SHORTEST whose sampled
// arc stays inside `boundary` (via pointInRing) is chosen, so the loop curls
// toward the mowed side that keeps it in-bounds. Re-mowing on the connectors is
// expected and accepted.
//
// If no full-radius connector fits in-bounds, `turn_radius` is shrunk for that
// connector (down to ~op_width/2); if a connector still cannot be made
// in-bounds, a straight connector is used as a last resort (an out-of-bounds
// straight join is then a real, reportable gap — not silently clipped).
//
//   plan        the rings + swaths from planBoustrophedon
//   boundary    the recorded-area polygon (open or closed), for the in-bounds
//               test that picks the turn direction
//   turn_radius nominal connector arc radius (m); shrunk per-connector if needed
//   step        densification step along the whole path (m, ~0.03)
//
// Returns one densified polyline starting at the first ring's first point. Pure
// function (no ROS deps) — unit-testable. Empty when the plan has no segments.
std::vector<std::pair<double, double>> buildContinuousPath(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double step);

// 2-D point-in-polygon (ray casting) against `ring`, a list of (x, y)
// vertices. Open or closed ring; winding-independent. Used by the server to
// VERIFY the plan stays inside the recorded boundary (log-only — the planner
// generates from insets of the boundary, so a violation is a bug, not an
// expected condition to silently clip).
bool pointInRing(double x, double y, const std::vector<std::pair<double, double>>& ring);

// Shortest distance from (x, y) to the nearest edge of `ring` (segment
// distance, not vertex distance).
double distanceToRing(double x, double y, const std::vector<std::pair<double, double>>& ring);

// Drop consecutive-duplicate vertices from an F2C ring and return it closed
// (first == last). A zero-length edge — e.g. a doubled leading vertex
// (points[0] == points[1]), as OpenMower exports and hand-drawn GUI polygons
// routinely carry — makes the ring non-simple, and boost::geometry (under F2C)
// rejects it, silently dropping that area from coverage planning. This is the
// last gate before a polygon becomes an f2c::types::Cell, so map areas reaching
// the server from any source (importer, saved areas file, GUI editor) are
// normalised here. Pure function (no ROS deps) — unit-testable.
f2c::types::LinearRing dedupClosedRing(const f2c::types::LinearRing& in);

}  // namespace mowgli_coverage

#endif  // MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_
