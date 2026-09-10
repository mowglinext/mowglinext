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
#include <string>
#include <utility>
#include <vector>

// Fields2Cover v3 umbrella header — f2c::types::*, f2c::hg::ConstHL,
// f2c::sg::BruteForce, f2c::rp::BoustrophedonOrder.
#include "fields2cover.h"

namespace mowgli_coverage
{

// Visibility into what planBoustrophedon dropped and how much of the polygon
// it ended up planning. Coverage gaps were previously silent (slivers, tiny
// rings, micro-cells, and whole-area F2C failures all just vanished from the
// plan); these counters/strings let the next iteration SEE the loss without
// changing any drop threshold. Pure accounting — cheap to populate.
struct PlanDiagnostics
{
  // One human-readable line per dropped piece, with its measured dimension and
  // the threshold it failed (e.g. "dropped swath len=0.1200<0.1500",
  // "dropped ring perim=0.7000<1.0000"). The server logs these verbatim.
  std::vector<std::string> drops;
  // Non-drop informational lines (e.g. the AUTO swath-angle large-field
  // fallback). Logged like drops but they do NOT indicate lost coverage.
  std::vector<std::string> notes;
  // Planned-coverage fraction in [0, 1]: (sum of swath-strip areas at op_width +
  // ring-strip areas at op_width) / polygon area. A coarse estimate (strips can
  // overlap at corners and the rings/swaths butt rather than overlap), so it can
  // slightly exceed the true mowed fraction — it is a visibility metric, not a
  // guarantee. Stays 0 when the field area is non-positive.
  double planned_fraction = 0.0;
  double field_area = 0.0;  // m² — the polygon area the fraction is taken over
  double planned_area = 0.0;  // m² — strip-area numerator (rings + swaths)
};

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
  // Closed outer ring of the chassis-safety-inset field (the SAME inset the
  // rings/swaths are planned against, == generateHeadlands(field, inset)). The
  // continuous-path connectors and corner fillets MUST stay inside THIS ring,
  // not the raw operator polygon, or a turn-around loop/fillet near a field
  // edge can push the spinning blade across the operator boundary (the discrete
  // segments are inset but the connectors that join them were not). Empty only
  // when no inset was applied (chassis_safety_inset <= 0) — the caller then
  // falls back to the raw boundary. (x, y) pairs, first == last.
  std::vector<std::pair<double, double>> safe_boundary;
  // Closed outer ring the turn-around CONNECTORS/FILLETS must stay inside — the
  // outermost headland RING's centerline (== safe_boundary eroded inward by
  // op_width/2, == the recorded line eroded by chassis_safety_inset). This is
  // TIGHTER than safe_boundary by op_width/2 and exists to close a spinning-blade
  // safety gap: allInside() only tests the path CENTERLINE, so bounding it to
  // safe_boundary let a turn-around arc's centerline reach op_width/2 FURTHER out
  // than the outermost ring's, pushing the chassis (± robot_width/2) and blade
  // that much past the operator boundary — and the excursion grew with the turn
  // radius (buildConnector accepts the largest radius whose centerline still
  // fits). Bounding connectors to the outermost-ring centerline instead makes a
  // turn's footprint no worse than the perimeter ring the robot already drives.
  // Deliberately op_width/2 (NOT robot_width/2): eroding by the chassis
  // half-width would keep turns robot_width/2 − op_width/2 TIGHTER than the
  // perimeter ring, forcing every edge turn-around below min_turning_radius →
  // straight fallback → sub-path fragmentation. Empty when the erosion degenerates
  // (tiny field) — the caller then falls back to safe_boundary. (x, y), first==last.
  //
  // The invariant, uniform across both branches: a connector centerline may go
  // NO FURTHER OUT than the outermost DRIVEN pass. WITH THE RING STAGE DISABLED
  // (num_headland_passes < 0 → zero rings, issue #429) there is no ring 0 to
  // erode to — the SWATH ENDS are then the outermost driven geometry and they
  // lie exactly ON safe_boundary, so this ring is safe_boundary EXACTLY (no
  // expansion, no erosion). Ends sitting on the ring are accepted by
  // allInside()'s 1 mm on-edge tolerance, not by moving the polygon outward.
  // Consequence to expect: with no mowed apron beyond the swath ends, a U-turn
  // arc usually does NOT fit, so buildConnector falls back to a straight join —
  // a pivot-through corner (which roundSharpCorners fillets where a
  // min_turning_radius arc fits). That fallback still passes allInside, so
  // conn_safe stays true and the sub-path does NOT fragment.
  std::vector<std::pair<double, double>> connector_clearance_boundary;
  // Inset ("grown") interior hole rings the continuous-path connectors and
  // corner fillets must stay OUT of, mirroring how safe_boundary is the ring
  // they must stay INSIDE. When the chassis-safety inset is applied these are
  // the holes of the inset field (grown outward by the inset, so the blade
  // clears an obstacle by the same margin it clears the outer boundary); with
  // no inset they are the raw operator holes. Rings/swaths are already clipped
  // around holes by F2C, but the turn-around connectors that JOIN them were
  // only validated against the outer boundary — a loop or straight fallback
  // could cut through a hole (issue #333). Each entry is one closed hole ring
  // (x, y). Empty when the field has no holes.
  std::vector<std::vector<std::pair<double, double>>> safe_holes;
  // Drop reasons + planned-coverage fraction (instrumentation only — see above).
  PlanDiagnostics diagnostics;
};

// Plan boustrophedon coverage of `field_cell` (outer ring + optional holes).
//
//   op_width             swath spacing = F2C cov_width (m)
//   headland_width       desired headland band width (m); used only by the AUTO
//                        ring count (see num_headland_passes_override)
//   num_headland_passes_override
//                        THREE-WAY contract (issue #429):
//                          < 0  NONE   — zero perimeter rings; the serpentine
//                                        swaths are the outermost driven pass.
//                                        They cut to the SAME line the outermost
//                                        ring would have (chassis_safety_inset
//                                        inside the recorded boundary) — this is
//                                        NOT "closer to the edge", it only drops
//                                        the perimeter loop. With no mowed apron
//                                        beyond the swath ends, row-end U-turns
//                                        degrade to straight pivot-through
//                                        joins.
//                          == 0 AUTO   — ceil(headland_width / op_width),
//                                        floored at 1.
//                          > 0  FORCED — exactly that many rings.
//   chassis_safety_inset polygon pull-back applied before everything (m)
//   mow_angle_rad        fixed swath heading; < 0 → auto (minimise swath count)
//   min_swath_length     drop straight swaths shorter than this (m)
//   ring_direction       perimeter/headland travel winding: 0 = planner default
//                        (F2C natural), 1 = clockwise, 2 = counter-clockwise.
//                        Flips which side of the robot faces the boundary — set
//                        it to keep a side-mounted blade on the cut side
//                        (issue #335). Swaths/connectors follow the rings.
//
// Geometry: safe = inset(field, chassis_safety_inset); rings are n_rings
// concentric loops spaced op_width inside safe; mainland = inset(safe,
// n_rings * op_width) so the swaths butt against the innermost ring's cut.
// With n_rings == 0 the whole ring stage is skipped and mainland == safe (never
// inset(safe, 0.0) — upstream that is a real GEOS buffer round-trip, not a
// no-op), so the swaths run edge to edge of the inset field.
//
// Returns a plan whose rings/swaths may BOTH be empty (field too small after
// insets — the caller reports failure). Throws on internal F2C errors. The
// returned plan's `diagnostics` records every dropped piece (slivers, tiny
// rings, micro-cells) and the planned-coverage fraction — instrumentation the
// caller logs; no drop threshold is altered.
//
// chassis_safety_inset is taken as-is here: the caller clamps it only at 0.0 (the
// default rides the outermost ring ON the recorded line, chassis straddling by
// design). The returned plan also carries connector_clearance_boundary (the
// outermost-ring centerline) so the turn-around connectors can be bounded to the
// perimeter ring rather than to safe_boundary — otherwise a turn arc's centerline
// (and its swept footprint) rides op_width/2 past the rings toward the boundary.
BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length,
                                    int ring_direction = 0,
                                    double min_turn_radius = 0.15);

// Per-plan accounting of how every segment-to-segment join was resolved by
// buildConnector's radius-shrink search. Pure visibility — populating it
// changes no decision.
//
// WHY THIS EXISTS (issue #499): the swath-end turn radius was believed to be
// the lever on the "violent turns dig the lawn" failure, and raising
// `min_turning_radius` was believed to be a TRADE rather than a free win — it
// is the FLOOR of buildConnector's shrink loop, so raising it should make the
// search give up sooner and fall through to the straight blind connector.
// Nobody could tell a radius change that fixed the turns from one that quietly
// replaced arcs with straight joins, so this counter shipped first as the
// prerequisite measurement.
//
// WHAT IT MEASURED (2026-08-24, the first plan it ever counted): on the SHIPPED
// geometry — num_headland_passes 2, op_width = tool_width - swath_overlap,
// connector_turn_radius 0.18, min_turning_radius 0.15 — only 1 join in 32 gets
// an arc. The other 31 are already straight blind connectors, and raising the
// floor to 0.25 m moves exactly one more. So the fallback TRADE above is not
// what constrains the radius, and the swath-end turns being carved are NOT
// too-tight Dubins arcs: there is no arc there to be tight.
//
// The binding constraint is the mowed HEADLAND APRON beyond the swath ends. At
// swath spacing d < 2R the turn-around must be an omega (RLR/LRL) loop whose
// forward extent past the swath end is ~sqrt(4R^2 - (R + d/2)^2) + R (~0.43 m
// at R = 0.18, d = 0.16), while the apron is num_headland_passes * op_width
// (0.32 m as shipped). Nothing in [min_turn_radius, turn_radius] fits, so
// buildConnector falls through; roundSharpCorners then cannot fillet the
// resulting 90 degree corners either, because the fillet's tangent length is
// floored at min_turn_radius (0.15 m) while the connector it must be trimmed
// into is only op_width (0.16 m) long. The path handed to FTC at a swath end is
// therefore a sharp corner, not an arc. See the CoverageConnectorStats tests.
//
// The three outcomes are mutually exclusive and sum to `attempted`:
struct ConnectorStats
{
  // Segment-to-segment joins where a connector was attempted (== segments - 1
  // in the common single-sub-path case).
  std::size_t attempted = 0;
  // A real forward Dubins turn-around arc fitted at some radius in
  // [min_turn_radius, turn_radius] and stayed in-bounds + clear of holes. This
  // is the healthy outcome.
  std::size_t arc = 0;
  // The shrink loop found NO in-bounds arc, so buildConnector returned its
  // straight blind connector — but that straight join happened to stay inside
  // the boundary and clear of every hole, so it is DRIVEN BLADE-ON as part of
  // the sub-path. Cheap in coverage terms, but it hands the controller a
  // heading discontinuity instead of a tangent arc.
  std::size_t straight_kept = 0;
  // No drivable connector at all (empty, or a straight fallback that left the
  // boundary / crossed a hole). The sub-path is BROKEN here and FollowStrip
  // bridges the gap with a blade-off Nav2 transit. This is the expensive
  // outcome — un-mowed transit time. A too-high min_turn_radius was expected to
  // inflate it; measured on a hole-free field it stays at zero, because the
  // straight fallbacks all verify in-bounds and are driven blade-on.
  std::size_t split = 0;
};

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
// connector — but NEVER below `min_turn_radius`. A loop tighter than the robot's
// minimum trackable turning radius is untrackable (wz≈vx/r exceeds the
// controller's authority), so MPPI loops/hesitates instead of driving it. If no
// arc of radius >= min_turn_radius fits in-bounds, a straight connector is used
// as a last resort (an out-of-bounds straight join is then a real, reportable
// gap — not silently clipped). The same floor caps the corner-fillet radius, so
// the whole path is free of sub-min_turn_radius arcs.
//
//   plan            the rings + swaths from planBoustrophedon
//   boundary        the recorded-area polygon (open or closed), for the
//                   in-bounds test that picks the turn direction
//   turn_radius     nominal connector arc radius (m); shrunk per-connector toward
//                   min_turn_radius if the nominal arc leaves the boundary
//   min_turn_radius hard floor on every connector/fillet arc (m) — the robot's
//                   minimum MPPI-trackable turning radius (mowgli_robot.yaml)
//   step            densification step along the whole path (m, ~0.03)
//
// Returns one densified polyline starting at the first ring's first point. Pure
// function (no ROS deps) — unit-testable. Empty when the plan has no segments.
//
// This is the concatenation of buildContinuousSubPaths() (below) — it stays a
// single polyline for the GUI full_path and the no-hole common case. When the
// field has holes it may contain a straight join across a hole; the DRIVER uses
// buildContinuousSubPaths so those joins become blade-off Nav2 transits instead.
std::vector<std::pair<double, double>> buildContinuousPath(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step);

// Like buildContinuousPath, but SPLIT into one or more hole-free continuous
// sub-paths (issue #333). A forward turn-around connector is a local Dubins arc;
// it cannot route around a large interior obstacle, so the path is BROKEN
// instead of joined blade-on when:
//   * the only join between two segments would cross a hole or leave the
//     boundary (a straight fallback that isn't safe), OR
//   * the join gap exceeds ~0.6 m (kMaxMowJoinGapM) — that is a RELOCATION
//     (lobe change across a concave bite, innermost ring → far first swath),
//     not a turn-around; an in-bounds Dubins for it would mow a long diagonal
//     across the middle of the lawn.
// Swath pieces are nearest-endpoint chained before joining (identical to the
// plain serpentine on a convex field; mows each lobe of a concave/hole-split
// field contiguously so a lobe change costs ONE split, not one per column).
// Each returned sub-path is internally continuous and cusp-free (an
// MPPI-trackable run); the caller (FollowStrip) drives them in order, bridging
// the gap between consecutive sub-paths with a blade-off, costmap-aware Nav2
// transit that routes around the obstacle. With a convex hole-free field this
// returns exactly one sub-path identical to buildContinuousPath. Sub-paths
// with < 2 points are dropped.
std::vector<std::vector<std::pair<double, double>>> buildContinuousSubPaths(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step,
    ConnectorStats* stats = nullptr);

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

// Grow a drawn-obstacle ring outward by `margin` metres (GDAL Buffer, rounded
// joins) and return it dedup-closed. Applied at goal-ingestion time so every
// downstream consumer — headland growth, safe_holes, sub-path splitting,
// connector routing — inherits the operator's obstacle_margin automatically.
// margin < 1e-3 or a degenerate ring falls back to dedupClosedRing(in): the
// obstacle is never dropped, only the extra margin. Pure function — testable.
f2c::types::LinearRing bufferRingOutward(const f2c::types::LinearRing& in, double margin);

}  // namespace mowgli_coverage

#endif  // MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_
