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
// pivot + FTC straight tracking), NOT F2C's: every prior design that let F2C
// plan turns (Dubins Ω-loops, CC-Dubins arcs, Reeds-Shepp cusps) produced
// geometry this chassis could not track, and the downstream heading-jump
// re-segmentation heuristic silently failed on the smooth arcs.

#ifndef MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_
#define MOWGLI_COVERAGE__COVERAGE_PLANNING_HPP_

#include <cstddef>
#include <optional>
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
  // Headland ring COUNT actually planned (the resolved n_rings — see
  // planBoustrophedon's num_headland_passes_override three-way contract), not
  // the raw config value: in AUTO (override == 0) this is the DERIVED count
  // (ceil(headland_width / op_width), floored at 1), which
  // connector_max_headland_passes still limits against even though the caller
  // never configured a concrete pass count. 0 with the ring stage disabled
  // (override < 0). Logging-only.
  int n_headland_passes = 0;
  // Closed outer ring of the chassis-safety-inset field (the SAME inset the
  // rings/swaths are planned against, == generateHeadlands(field, inset)). The
  // continuous-path connectors and corner fillets MUST stay inside THIS ring,
  // not the raw operator polygon, or a turn-around loop/fillet near a field
  // edge can push the spinning blade across the operator boundary (the discrete
  // segments are inset but the connectors that join them were not). Empty only
  // when no inset was applied (chassis_safety_inset <= 0) — the caller then
  // falls back to the raw boundary. (x, y) pairs, first == last.
  std::vector<std::pair<double, double>> safe_boundary;
  // Closed outer ring the turn-around CONNECTORS/FILLETS must stay inside —
  // ALWAYS the outermost headland RING's centerline (== safe_boundary eroded
  // inward by op_width/2, == the recorded line eroded by
  // chassis_safety_inset), regardless of `connector_max_headland_passes`
  // (issue #497 — see swath_turn_envelope below for the knob that actually
  // limits turn-arounds). This is TIGHTER than safe_boundary by op_width/2
  // and exists to close a spinning-blade safety gap: allInside() only tests
  // the path CENTERLINE, so bounding it to safe_boundary let a turn-around
  // arc's centerline reach op_width/2 FURTHER out than the outermost ring's,
  // pushing the chassis (± robot_width/2) and blade that much past the
  // operator boundary — and the excursion grew with the turn radius
  // (buildConnector accepts the largest radius whose centerline still fits).
  // Bounding connectors to the outermost-ring centerline instead makes a
  // turn's footprint no worse than the perimeter ring the robot already
  // drives. Deliberately op_width/2 (NOT robot_width/2): eroding by the
  // chassis half-width would keep turns robot_width/2 − op_width/2 TIGHTER
  // than the perimeter ring, forcing every edge turn-around below
  // min_turning_radius → straight fallback → sub-path fragmentation. Empty
  // when the erosion degenerates (tiny field) — the caller then falls back
  // to safe_boundary. (x, y), first==last.
  //
  // Also the bound for buildContinuousSubPaths' out-of-bounds clamp (#388,
  // clampInsideRing — applied to EVERY pose of EVERY sub-path, rings
  // included), the server's post-plan verify, and every ring-to-ring /
  // ring-to-swath connector — deliberately NEVER the tighter
  // swath_turn_envelope: clamping or verifying a headland RING pose against a
  // boundary narrower than ring 0 would silently project that ring's own
  // poses onto a DIFFERENT ring's centerline (issue #497 review: with 3
  // rings configured and turns limited to 2, ring 0 clamped against ring 1's
  // envelope drove ring 1 twice and never drove ring 0 at all).
  //
  // WITH THE RING STAGE DISABLED (num_headland_passes < 0 → zero rings, issue
  // #429) there is no ring to erode to — the SWATH ENDS are then the
  // outermost driven geometry and they lie exactly ON safe_boundary, so this
  // ring is safe_boundary EXACTLY (no expansion, no erosion). Ends sitting on
  // the ring are accepted by allInside()'s 1 mm on-edge tolerance, not by
  // moving the polygon outward. Consequence to expect: with no mowed apron
  // beyond the swath ends, a U-turn arc usually does NOT fit, so
  // buildConnector falls back to a straight join; since 40d0c30b a straight
  // fallback is only kept blade-on when straightFallbackIsContinuous (its
  // heading is within 15° of both segments) — a ~180° swath-to-swath reversal
  // fails that test, so it becomes a PIVOT JOIN (explicit in-place pivot
  // corners, see PivotJoinLimits) where the pivot sweep fits, and otherwise the
  // sub-path SPLITS there (a blade-off Nav2 transit); it is never driven as a
  // silent zero-radius corner. A tight
  // `swath_turn_envelope` produces the exact same starved-apron split pattern
  // one ring set further out — that is the intended trade-off of asking turns
  // to stay off the outer band.
  std::vector<std::pair<double, double>> connector_clearance_boundary;
  // A P-pass-deep turn envelope for MAINLAND SWATH-TO-SWATH connectors ONLY
  // (`connector_max_headland_passes`, issue #497): with N total headland
  // rings and a limit of P < N passes, this sits on ring (N − P)'s
  // centerline instead of ring 0's — leaving the outermost (N − P) rings'
  // band untouched by any row-end U-turn. Passed to buildContinuousSubPaths
  // as its optional `swath_turn_boundary` argument, which uses it ONLY for
  // buildConnector's allInside test on joins between two mainland swath
  // segments. Every other use in that function — the ring-to-ring joins, the
  // ring-to-first-swath transition, the #388 out-of-bounds clamp, and the
  // whole-path corner fillet pass — stays bound to connector_clearance_boundary
  // (ring 0) regardless of this field, so limiting how far a swath U-turn may
  // swing can never relocate a headland ring pass onto a different ring's
  // line. Empty when connector_max_headland_passes has no effect (<= 0,
  // >= n_rings, or rings disabled) — the caller then uses
  // connector_clearance_boundary for every join, identical to pre-#497
  // behaviour. (x, y), first==last.
  std::vector<std::pair<double, double>> swath_turn_envelope;
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
//   connector_max_headland_passes (issue #497)
//                        How many of the n_rings headland passes, counted from
//                        the mainland edge OUTWARD, a MAINLAND SWATH-TO-SWATH
//                        turn-around connector is permitted to cross. <= 0 or
//                        >= n_rings: UNLIMITED — swath U-turns may use the
//                        whole apron out to ring 0's centerline (the pre-#497
//                        default, unchanged; plan.swath_turn_envelope stays
//                        empty). In [1, n_rings): plan.swath_turn_envelope is
//                        populated at ring (n_rings − value)'s centerline,
//                        leaving the outermost (n_rings − value) rings' band a
//                        no-turn zone for swath U-turns ONLY — e.g. 3 rings
//                        configured with a limit of 2 keeps every mainland
//                        U-turn off the outermost ring, trading a higher
//                        straight-connector/split fallback rate (see
//                        ConnectorStats) for staying further from the recorded
//                        boundary during a turn. connector_clearance_boundary
//                        itself is UNAFFECTED by this parameter — it always
//                        stays ring 0's centerline, so ring-to-ring joins, the
//                        #388 clamp and the server's verify never move. Has no
//                        effect with rings disabled (n_rings == 0): there is
//                        no ring to bound to, so swath_turn_envelope stays
//                        empty.
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
// design). The returned plan also carries connector_clearance_boundary (always
// the outermost-ring centerline) so the turn-around connectors can be bounded
// to a perimeter ring rather than to safe_boundary — otherwise a turn arc's
// centerline (and its swept footprint) rides op_width/2 past the rings toward
// the boundary — plus swath_turn_envelope, a tighter ring-bound
// connector_max_headland_passes caps for mainland swath U-turns only (see the
// BoustrophedonPlan field docs).
// Deterministic Auto heading; degenerate clips never define an angle.
std::optional<double> longestValidSwathAngle(const f2c::types::Swaths& swaths);

BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length,
                                    int ring_direction = 0,
                                    double min_turn_radius = 0.20,
                                    bool perpendicular = false,
                                    int connector_max_headland_passes = 0);

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
// WHAT IT MEASURED (2026-08-24, the first plan it ever counted): on the old
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
// (0.32 m in that configuration). Nothing in [min_turn_radius, turn_radius] fits, so
// buildConnector falls through; roundSharpCorners then cannot fillet the
// resulting 90 degree corners either, because the fillet's tangent length is
// floored at min_turn_radius (0.15 m) while the connector it must be trimmed
// into is only op_width (0.16 m) long. The path handed to FTC at a swath end is
// therefore a sharp corner, not an arc. The 2026-09-09 field bag confirmed that
// FTC alternates its saturated angular command at those corners. Production now
// provides a five-pass apron for 0.20 m arcs and splits any residual
// discontinuous fallback. See the CoverageConnectorStats tests.
//
// The four outcomes are mutually exclusive and sum to `attempted`:
struct ConnectorStats
{
  // Segment-to-segment joins where a connector was attempted (== segments - 1
  // in the common single-sub-path case).
  std::size_t attempted = 0;
  // A real forward Dubins turn-around arc fitted at some radius in
  // [min_turn_radius, turn_radius] and stayed in-bounds + clear of holes. This
  // is the healthy outcome.
  std::size_t arc = 0;
  // The shrink loop found no in-bounds arc, but its straight fallback stayed
  // inside the boundary, clear of holes, and aligned with both segment headings.
  // Only this tangent-enough fallback is kept blade-on without a pivot.
  std::size_t straight_kept = 0;
  // No arc fitted and the straight fallback is NOT aligned, but it is a short
  // join between adjacent passes that passed every pivot-join check (see
  // PivotJoinLimits): kept blade-on inside the sub-path, its corners encoded as
  // explicit pivot corners FTC rotates in place at.
  std::size_t pivot = 0;
  // No continuous blade-on connector: empty, outside the boundary, through a
  // hole, or a discontinuous straight fallback that is not an acceptable pivot
  // join. The sub-path is broken here and FollowStrip repositions and reorients
  // blade-off.
  std::size_t split = 0;
};

// The server reports connector outcomes at WARN when at least this share of
// attempted joins split into blade-off transits. This is a visibility threshold,
// not a planner rejection rule: a small number of splits is normal on concave
// geometry, while a high share makes the resulting transits easy to miss.
constexpr double kConnectorSplitWarnPct = 25.0;

// True when ConnectorStats should be reported at WARN. The zero-attempt case is
// intentionally not a warning: no connector was attempted, so there is no
// split rate to report.
inline bool connectorSplitRateWarns(const ConnectorStats& stats)
{
  return stats.attempted > 0 &&
         100.0 * static_cast<double>(stats.split) / static_cast<double>(stats.attempted) >=
             kConnectorSplitWarnPct;
}

// When (and where) a join that fits no forward arc may still stay inside the
// sub-path as a PIVOT JOIN — a straight connector whose zero-radius corners FTC
// drives as in-place pivots (corner encoding: mowgli_interfaces/
// coverage_geometry.hpp, "PIVOT CORNER CONTRACT").
//
// Why it exists: with a thin headland apron (num_headland_passes auto on a
// narrow tool, or none) no arc of radius >= min_turning_radius fits between the
// swath ends and the clearance ring, so every row end split into a blade-off
// Nav2 transit + a PRE_ROTATE — 128 sub-paths / 127 transits on a 152 m² lawn
// (field 2026-09-21, 6.7 % mowed in 6 minutes).
//
// A join becomes a pivot join only when ALL of the following hold, otherwise it
// splits exactly as before:
//   * pivot joins are enabled (sweep_radius > 0);
//   * the straight connector is at most kSegmentTransitGapM long — a turn-around
//     between ADJACENT passes, never a relocation;
//   * the connector centreline stays inside the connector boundary and clear of
//     the (margin-grown) holes — the same allInside / clearOfHoles tests every
//     other blade-on join passes;
//   * at every corner the robot pivots at, the disc the chassis sweeps rotating
//     in place about base_link (rear wheel axis, radius sweep_radius) stays
//     inside the RECORDED boundary grown by boundary_margin (map_server's
//     non-lethal soft band) and clear of every DRAWN obstacle.
// The blade stays ON through a pivot join: it lies inside the mowing area
// between two adjacent passes, exactly like the blade-on arcs it replaces.
struct PivotJoinLimits
{
  // Radius (m) of the disc the chassis sweeps pivoting in place about
  // base_link: robot_config_util.chassis_circumscribed_radius (0.597 m shipped).
  // <= 0 disables pivot joins — the pre-pivot behaviour (and the default, so an
  // uninjected server never pivots).
  double sweep_radius = 0.0;
  // How far past the recorded boundary the body may reach (m): map_server's
  // non-lethal soft band, enforce_boundary_margin_m floored at the chassis
  // circumscribed radius (robot_config_util.boundary_soft_margin).
  double boundary_margin = 0.0;
  // The RECORDED outer boundary — the operator polygon before any inset or
  // outward expansion (open or closed ring).
  std::vector<std::pair<double, double>> recorded_boundary;
  // The DRAWN obstacles — operator polygons before obstacle_margin. The sweep
  // must clear these by sweep_radius; the margin-grown safe_holes only bound
  // the connector centreline.
  std::vector<std::vector<std::pair<double, double>>> recorded_obstacles;
};

// True iff a robot pivoting in place with base_link at (x, y) sweeps only
// ground inside `limits.recorded_boundary` grown by `limits.boundary_margin`
// and clear of every `limits.recorded_obstacles` polygon. Conservative: the
// whole disc of radius sweep_radius is tested, whatever the rotation. False
// when pivot joins are disabled or the recorded boundary is missing.
bool pivotSweepFits(double x, double y, const PivotJoinLimits& limits);

// Yaw of every pose of a drivable sub-path, honouring the pivot corner
// contract: a pose followed by a pose at the SAME position (a pivot corner)
// takes the INCOMING heading, its twin the outgoing one; every other pose takes
// the heading of the step to its successor, the last pose that of the step
// into it. coverage_server stamps these into the result poses.
std::vector<double> pathHeadings(const std::vector<std::pair<double, double>>& pts);

// Flatten a BoustrophedonPlan into continuous, in-bounds polylines that FTC can
// track without crossing a zero-radius segment join.
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
// controller's authority), so FTC saturates and oscillates instead of driving it. If no
// arc of radius >= min_turn_radius fits in-bounds, a straight connector is kept
// only when it is aligned with both segments. Otherwise the drivable path is
// split there. The same floor caps the corner-fillet radius.
//
//   plan            the rings + swaths from planBoustrophedon
//   boundary        the recorded-area polygon (open or closed), for the
//                   in-bounds test that picks the turn direction
//   turn_radius     nominal connector arc radius (m); shrunk per-connector toward
//                   min_turn_radius if the nominal arc leaves the boundary
//   min_turn_radius hard floor on every connector/fillet arc (m) — the robot's
//                   minimum FTC-trackable turning radius (mowgli_robot.yaml)
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

// Like buildContinuousPath, but split into one or more continuous sub-paths.
// The path is broken when no in-bounds, hole-free and heading-continuous
// connector exists. This includes straight fallbacks between antiparallel
// swaths: keeping them would hand FTC a zero-radius corner and cause alternating
// saturated steering commands.
// Swath pieces are nearest-endpoint chained before joining (identical to the
// plain serpentine on a convex field; mows each lobe of a concave/hole-split
// field contiguously so a lobe change costs ONE split, not one per column).
// The caller (FollowStrip) drives them in order, bridging every sub-path boundary
// with a blade-off, costmap-aware Nav2 reposition/reorientation. Sub-paths with
// fewer than two points are dropped.
//
//   swath_turn_boundary (issue #497) — optional, defaults to empty (meaning
//                        "no override, use `boundary` for every join": every
//                        existing call site is unaffected). When populated
//                        (size >= 3, normally plan.swath_turn_envelope), it
//                        REPLACES `boundary` inside buildConnector's
//                        allInside/clearOfHoles test, but ONLY for joins
//                        where both the previous and the current segment are
//                        mainland swaths (a row-end U-turn) — ring-to-ring
//                        joins, the ring-to-first-swath transition, the #388
//                        out-of-bounds clamp, and the whole-path corner
//                        fillet pass all stay bound to `boundary` regardless.
//                        This is what lets connector_max_headland_passes
//                        keep swath U-turns off the outer rings WITHOUT ever
//                        clamping a ring's own poses onto a different ring's
//                        centerline (see BoustrophedonPlan::swath_turn_envelope).
//   pivot_limits       — optional, defaults to disabled (every existing call
//                        site is unaffected). When enabled, a join that fits
//                        no arc and is not an aligned straight connector is
//                        kept as a PIVOT JOIN instead of splitting, where the
//                        PivotJoinLimits checks allow it. Its corners are
//                        emitted per the pivot corner contract
//                        (mowgli_interfaces/coverage_geometry.hpp): the corner
//                        position twice in a row. Every OTHER near-coincident
//                        pose pair is collapsed, enabled or not, so a
//                        zero-length step in a sub-path always means a pivot.
std::vector<std::vector<std::pair<double, double>>> buildContinuousSubPaths(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step,
    ConnectorStats* stats = nullptr,
    const std::vector<std::pair<double, double>>& swath_turn_boundary = {},
    const PivotJoinLimits& pivot_limits = {});

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
