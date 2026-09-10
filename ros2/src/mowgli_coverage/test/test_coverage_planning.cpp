// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Geometry tests for the boustrophedon coverage planner. These run under
// colcon test against the REAL Fields2Cover v3 library, so they exercise the
// actual ConstHL / BruteForce / BoustrophedonOrder output. The point is to
// catch a broken plan (empty / out-of-bounds / non-serpentine / hole-crossing)
// in CI instead of on the robot.

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_coverage::BoustrophedonPlan;
using mowgli_coverage::buildContinuousPath;
using mowgli_coverage::buildContinuousSubPaths;
using mowgli_coverage::dedupClosedRing;
using mowgli_coverage::distanceToRing;
using mowgli_coverage::planBoustrophedon;
using mowgli_coverage::pointInRing;

// A closed square Cell with corner (0,0) and side `size`.
f2c::types::Cell makeSquare(double size)
{
  f2c::types::LinearRing ring;
  ring.addPoint(f2c::types::Point(0.0, 0.0));
  ring.addPoint(f2c::types::Point(size, 0.0));
  ring.addPoint(f2c::types::Point(size, size));
  ring.addPoint(f2c::types::Point(0.0, size));
  ring.addPoint(f2c::types::Point(0.0, 0.0));  // F2C wants a closed ring
  return f2c::types::Cell(ring);
}

// A concave L-shape: a `size` square with a `notch`x`notch` bite removed from
// the top-right corner. Hole-free but NON-convex (re-entrant corner).
f2c::types::Cell makeLShape(double size, double notch)
{
  f2c::types::LinearRing ring;
  ring.addPoint(f2c::types::Point(0.0, 0.0));
  ring.addPoint(f2c::types::Point(size, 0.0));
  ring.addPoint(f2c::types::Point(size, size - notch));
  ring.addPoint(f2c::types::Point(size - notch, size - notch));  // re-entrant vertex
  ring.addPoint(f2c::types::Point(size - notch, size));
  ring.addPoint(f2c::types::Point(0.0, size));
  ring.addPoint(f2c::types::Point(0.0, 0.0));  // closed
  return f2c::types::Cell(ring);
}

std::vector<std::pair<double, double>> squareRing(double size)
{
  return {{0.0, 0.0}, {size, 0.0}, {size, size}, {0.0, size}};
}

double swathLen(const std::pair<std::pair<double, double>, std::pair<double, double>>& s)
{
  return std::hypot(s.second.first - s.first.first, s.second.second - s.first.second);
}

double swathYaw(const std::pair<std::pair<double, double>, std::pair<double, double>>& s)
{
  return std::atan2(s.second.second - s.first.second, s.second.first - s.first.first);
}

double wrapAngle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

// Mowgli production-ish knobs: 0.16 m spacing, 0.18 m headland band,
// auto ring count, 0.08 m chassis inset, auto angle, 0.15 m min swath.
BoustrophedonPlan planDefault(const f2c::types::Cell& cell)
{
  return planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15);
}

// Rectangle Cell centred on the origin, matching the sim's area_polygons.
f2c::types::Cell makeRectCentered(double w, double h)
{
  f2c::types::LinearRing ring;
  ring.addPoint(f2c::types::Point(-w / 2, -h / 2));
  ring.addPoint(f2c::types::Point(w / 2, -h / 2));
  ring.addPoint(f2c::types::Point(w / 2, h / 2));
  ring.addPoint(f2c::types::Point(-w / 2, h / 2));
  ring.addPoint(f2c::types::Point(-w / 2, -h / 2));  // closed
  return f2c::types::Cell(ring);
}

// Reproduces the SIM coverage_server pipeline that SEGFAULTS on result delivery
// (exit -11, heap-corruption signature). The distinguishing knob vs planDefault
// is the chassis inset = robot_width/2 = 0.20 m (the sim clamps
// chassis_safety_inset up to that), with connector turn_radius = 0.18 m, on the
// exact rectangle sizes that crashed. Built under ASAN this pinpoints the
// out-of-bounds write in the plan / continuous-path construction.
TEST(CoverageRepro, SimGeometryDoesNotCorruptHeap)
{
  std::vector<std::pair<double, double>> sizes = {{9.0, 6.0}, {5.0, 4.0}, {3.0, 2.0}};
  for (const auto& wh : sizes)
  {
    const auto cell = makeRectCentered(wh.first, wh.second);
    const auto plan = planBoustrophedon(cell, 0.16, 0.18, 0, 0.20, -1.0, 0.15);
    const auto subs = buildContinuousSubPaths(plan, plan.safe_boundary, 0.18, 0.18, 0.05);
    const auto full = buildContinuousPath(plan, plan.safe_boundary, 0.18, 0.18, 0.05);
    (void)subs;
    (void)full;
  }
}

// ---------------------------------------------------------------------------
// Integration fixture: the operator's REAL recorded area (areas.dat,
// recorded_area_1) — an elongated, concave ~26 m² garden. Coordinates copied
// verbatim from the persisted file so we exercise the planner on true field
// geometry, not a synthetic shape.
// ---------------------------------------------------------------------------
const std::vector<std::pair<double, double>>& recordedArea1Pts()
{
  static const std::vector<std::pair<double, double>> pts = {{0.69736, 0.542974},
                                                             {0.333294, 0.901446},
                                                             {0.0692125, 0.84469},
                                                             {-1.83674, 3.2762},
                                                             {-1.82738, 3.67492},
                                                             {-0.580451, 4.72558},
                                                             {-0.34252, 5.71537},
                                                             {-2.52072, 8.45872},
                                                             {-1.85113, 9.08351},
                                                             {-0.752371, 8.3454},
                                                             {3.55742, 2.91514},
                                                             {4.20313, 1.36811},
                                                             {2.98118, 0.11797},
                                                             {1.29082, 0.260783},
                                                             {0.664285, 0.365982}};
  return pts;
}

f2c::types::Cell makeRecordedArea1()
{
  f2c::types::LinearRing ring;
  for (const auto& p : recordedArea1Pts())
  {
    ring.addPoint(f2c::types::Point(p.first, p.second));
  }
  const auto& first = recordedArea1Pts().front();
  ring.addPoint(f2c::types::Point(first.first, first.second));  // close
  return f2c::types::Cell(ring);
}

// nav2_mppi_controller's path-inversion test (utils::findFirstPathInversion):
// the first index where the path turns by > 90° (dot of consecutive segment
// vectors < 0). enforce_path_inversion CROPS the plan handed to MPPI here.
// Returns pts.size() when the polyline never doubles back.
std::size_t firstInversion(const std::vector<std::pair<double, double>>& pts)
{
  if (pts.size() < 3)
  {
    return pts.size();
  }
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    const double ax = pts[i].first - pts[i - 1].first;
    const double ay = pts[i].second - pts[i - 1].second;
    const double bx = pts[i + 1].first - pts[i].first;
    const double by = pts[i + 1].second - pts[i].second;
    if (ax * bx + ay * by < 0.0)
    {
      return i + 1;
    }
  }
  return pts.size();
}

// Largest local turn angle (deg) over the polyline — purely for diagnostics.
double maxTurnDeg(const std::vector<std::pair<double, double>>& pts)
{
  double worst = 0.0;
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    const double ax = pts[i].first - pts[i - 1].first;
    const double ay = pts[i].second - pts[i - 1].second;
    const double bx = pts[i + 1].first - pts[i].first;
    const double by = pts[i + 1].second - pts[i].second;
    const double na = std::hypot(ax, ay), nb = std::hypot(bx, by);
    if (na < 1e-9 || nb < 1e-9)
    {
      continue;
    }
    double c = (ax * bx + ay * by) / (na * nb);
    c = std::max(-1.0, std::min(1.0, c));
    worst = std::max(worst, std::acos(c) * 180.0 / M_PI);
  }
  return worst;
}

double pathLength(const std::vector<std::pair<double, double>>& pts)
{
  double len = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i)
  {
    len += std::hypot(pts[i].first - pts[i - 1].first, pts[i].second - pts[i - 1].second);
  }
  return len;
}

// Longest run of consecutive over-curved steps — the signature of an arc/loop
// tighter than `min_radius`. On a densified arc of radius r at step `step`, each
// step turns by ≈ step/r, so a step is "over-curved" when its turn angle exceeds
// step/min_radius. A lone sharp polygon corner (a pivot vertex the rings inherit)
// is ONE over-curved step; a sub-min_radius loop is MANY consecutive ones. Used
// to assert buildContinuousPath never emits a turn-around / fillet tighter than
// the robot can track. Returns the max consecutive count (0 = none).
std::size_t maxTightArcRun(const std::vector<std::pair<double, double>>& pts,
                           double step,
                           double min_radius)
{
  // 10% slack so discretization noise at a clean min_radius arc doesn't trip it.
  const double max_step_turn = step / (min_radius * 0.9);
  std::size_t worst = 0, run = 0;
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    const double ax = pts[i].first - pts[i - 1].first;
    const double ay = pts[i].second - pts[i - 1].second;
    const double bx = pts[i + 1].first - pts[i].first;
    const double by = pts[i + 1].second - pts[i].second;
    const double na = std::hypot(ax, ay), nb = std::hypot(bx, by);
    if (na < 1e-9 || nb < 1e-9)
    {
      run = 0;
      continue;
    }
    double c = (ax * bx + ay * by) / (na * nb);
    c = std::max(-1.0, std::min(1.0, c));
    if (std::acos(c) > max_step_turn)
    {
      worst = std::max(worst, ++run);
    }
    else
    {
      run = 0;
    }
  }
  return worst;
}

}  // namespace

// 3x3 m square: at least one headland ring + several straight swaths spanning
// the interior, everything inside the boundary.
TEST(CoveragePlanning, SquareCoversField)
{
  const auto plan = planDefault(makeSquare(3.0));

  ASSERT_GE(plan.rings.size(), 1u);
  ASSERT_GE(plan.swaths.size(), 5u);

  // Outermost ring: a closed loop, inset ~ chassis(0.08) + op_w/2 (0.08) from
  // the boundary → every point well inside the square, none deeper than ~0.5m.
  const auto& ring = plan.rings.front();
  ASSERT_GE(ring.size(), 8u);
  EXPECT_NEAR(ring.front().first, ring.back().first, 1e-6);
  EXPECT_NEAR(ring.front().second, ring.back().second, 1e-6);
  const auto boundary = squareRing(3.0);
  for (const auto& p : ring)
  {
    EXPECT_TRUE(pointInRing(p.first, p.second, boundary));
    EXPECT_LT(distanceToRing(p.first, p.second, boundary), 0.5);
  }

  // Swaths: all inside, spanning >2 m of the field in at least one axis.
  double max_len = 0.0;
  for (const auto& s : plan.swaths)
  {
    EXPECT_TRUE(pointInRing(s.first.first, s.first.second, boundary));
    EXPECT_TRUE(pointInRing(s.second.first, s.second.second, boundary));
    max_len = std::max(max_len, swathLen(s));
  }
  EXPECT_GT(max_len, 1.5);
}

// Consecutive swaths drive in ALTERNATING directions (serpentine) and are
// spaced ~op_width apart — the boustrophedon contract the BT's per-segment
// pivot model relies on.
TEST(CoveragePlanning, SquareSwathsAreSerpentine)
{
  const auto plan = planDefault(makeSquare(3.0));
  ASSERT_GE(plan.swaths.size(), 4u);

  for (std::size_t i = 1; i < plan.swaths.size(); ++i)
  {
    const double dyaw =
        std::fabs(wrapAngle(swathYaw(plan.swaths[i]) - swathYaw(plan.swaths[i - 1])));
    EXPECT_GT(dyaw, M_PI - 0.1) << "swaths " << i - 1 << "→" << i << " do not alternate direction";
    // End of swath i-1 to start of swath i: the adjacent-row hop, about one
    // op_width (allow 3x for edge effects).
    const double hop = std::hypot(plan.swaths[i].first.first - plan.swaths[i - 1].second.first,
                                  plan.swaths[i].first.second - plan.swaths[i - 1].second.second);
    EXPECT_LT(hop, 3 * 0.16) << "swaths " << i - 1 << "→" << i << " hop too far";
  }
}

// A fixed mow angle is honoured and the plan is deterministic across calls
// (swath-index resume relies on this).
TEST(CoveragePlanning, FixedAngleIsHonouredAndDeterministic)
{
  const auto cell = makeSquare(3.0);
  const auto a = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, 0.0, 0.15);
  const auto b = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, 0.0, 0.15);

  ASSERT_GE(a.swaths.size(), 4u);
  ASSERT_EQ(a.swaths.size(), b.swaths.size());
  for (std::size_t i = 0; i < a.swaths.size(); ++i)
  {
    EXPECT_NEAR(a.swaths[i].first.first, b.swaths[i].first.first, 1e-9);
    EXPECT_NEAR(a.swaths[i].first.second, b.swaths[i].first.second, 1e-9);
    // angle 0 → swaths run along X (possibly reversed by the serpentine).
    const double yaw = swathYaw(a.swaths[i]);
    EXPECT_LT(std::min(std::fabs(wrapAngle(yaw)), std::fabs(wrapAngle(yaw - M_PI))), 0.05);
  }
}

// 5x5 m square with a 1.5x1.5 m central hole: no swath may cross the hole —
// each sweep line is clipped into per-side swaths (F2C v3 makes every disjoint
// clip its own swath; that property replaces decomposition).
TEST(CoveragePlanning, HoleIsNotCrossed)
{
  f2c::types::Cell cell = makeSquare(5.0);
  f2c::types::LinearRing hole;
  hole.addPoint(f2c::types::Point(1.75, 1.75));
  hole.addPoint(f2c::types::Point(3.25, 1.75));
  hole.addPoint(f2c::types::Point(3.25, 3.25));
  hole.addPoint(f2c::types::Point(1.75, 3.25));
  hole.addPoint(f2c::types::Point(1.75, 1.75));
  cell.addRing(hole);

  const auto plan = planDefault(cell);
  ASSERT_GE(plan.swaths.size(), 10u);

  const std::vector<std::pair<double, double>> hole_ring = {{1.75, 1.75},
                                                            {3.25, 1.75},
                                                            {3.25, 3.25},
                                                            {1.75, 3.25}};
  for (const auto& s : plan.swaths)
  {
    // Sample along the swath: no point inside the hole (small margin for the
    // hole's own headland inset keeps this robust).
    const int n = std::max(2, static_cast<int>(swathLen(s) / 0.05));
    for (int k = 0; k <= n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      const double x = s.first.first + t * (s.second.first - s.first.first);
      const double y = s.first.second + t * (s.second.second - s.first.second);
      EXPECT_FALSE(pointInRing(x, y, hole_ring))
          << "swath crosses the hole at (" << x << ", " << y << ")";
    }
  }
}

// #333: the CONTINUOUS path's turn-around connectors AND corner fillets — not
// just the swaths — must stay out of an obstacle hole. Before the fix the
// connectors were validated only against the outer boundary, so a turn-around
// loop or a straight fallback join between two hole-split segments could cut
// straight through the obstacle. Flatten the plan to the one continuous path and
// assert no pose lands inside the hole. Also confirms the inset hole ring is
// exposed on the plan (the data the avoidance uses).
TEST(CoverageContinuousPath, ContinuousPathAvoidsHole)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kInset = 0.15;
  constexpr double kMinSwath = 0.15;
  constexpr double kTurnRadius = 0.18;  // deployed connector_turn_radius default
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;

  // 6 m square with a 1.5 m central hole — big enough that hole-free connectors
  // exist on the mowed side, so the fix can route around rather than fall back.
  f2c::types::Cell cell = makeSquare(6.0);
  f2c::types::LinearRing hole;
  hole.addPoint(f2c::types::Point(2.25, 2.25));
  hole.addPoint(f2c::types::Point(3.75, 2.25));
  hole.addPoint(f2c::types::Point(3.75, 3.75));
  hole.addPoint(f2c::types::Point(2.25, 3.75));
  hole.addPoint(f2c::types::Point(2.25, 2.25));
  cell.addRing(hole);

  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty()) << "no rings on the hole field";
  ASSERT_GE(plan.safe_boundary.size(), 3u);
  ASSERT_FALSE(plan.safe_holes.empty())
      << "inset hole ring not exposed on the plan — connectors have nothing to avoid";

  // The DRIVER follows the split sub-paths; each must be internally hole-free.
  // The gap between consecutive sub-paths is bridged by a blade-off Nav2 transit
  // (routes around the hole), so it is intentionally NOT continuous here.
  const auto subs =
      buildContinuousSubPaths(plan, plan.safe_boundary, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_GE(subs.size(), 2u)
      << "a central hole must split the path into >=2 sub-paths (else a connector "
         "cut through the hole)";
  // Upper bound: a single hole must NOT over-fragment. Sub-paths split ONLY at
  // real obstacle-gap relocations (Split A) — not at every serpentine U-turn cusp
  // (the removed MPPI-era Split B), and not at the per-pass outer↔hole ring
  // ping-pong (rings are grouped outer-first). One central hole ⇒ ~2 sub-paths.
  EXPECT_LE(subs.size(), 3u) << subs.size()
                             << " sub-paths for ONE hole — over-fragmentation regression "
                                "(residual-cusp split or interleaved rings)";

  // The RAW hole ring: sub-paths are validated against the GROWN (inset) hole,
  // so the raw hole is cleared with margin. Any sub-path pose inside it is a real
  // #333 breach.
  const std::vector<std::pair<double, double>> hole_ring = {{2.25, 2.25},
                                                            {3.75, 2.25},
                                                            {3.75, 3.75},
                                                            {2.25, 3.75}};
  std::size_t total_pts = 0, in_hole = 0;
  for (const auto& sp : subs)
  {
    ASSERT_GE(sp.size(), 2u) << "degenerate sub-path emitted";
    for (const auto& p : sp)
    {
      ++total_pts;
      if (pointInRing(p.first, p.second, hole_ring))
      {
        ++in_hole;
      }
    }
  }
  EXPECT_EQ(in_hole, 0u) << in_hole << "/" << total_pts
                         << " sub-path poses fall inside the obstacle hole";
}

// A hole-free field must yield EXACTLY ONE continuous sub-path — nothing splits it
// (no obstacle gap to route around, so Split A never fires), and no interior
// serpentine U-turn cusp splits it either (the removed MPPI-era Split B). One
// sub-path means the whole field is driven blade-on end-to-end with zero blade-off
// transits, which is the point of the continuous full_path.
TEST(CoverageContinuousPath, HoleFreeFieldIsOneSubPath)
{
  // Deployed connector knobs (turn 0.18, min_turn 0.15) on a hole-free rectangle.
  const auto cell = makeRectCentered(9.0, 6.0);
  const auto plan = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15);
  ASSERT_FALSE(plan.rings.empty());
  ASSERT_TRUE(plan.safe_holes.empty()) << "test field must be hole-free";
  const auto subs = buildContinuousSubPaths(plan, plan.safe_boundary, 0.18, 0.15, 0.05);
  EXPECT_EQ(subs.size(), 1u) << subs.size()
                             << " sub-paths on a HOLE-FREE field — a spurious split "
                                "(residual-cusp or ring-ordering artifact)";
}

// Repro of the field-reported over-fragmentation: a ~10.5 m square with a single
// ~1 m central hole over a 72 m² field split into 18 sub-paths / 17 blade-off
// transits. With Split B removed and rings grouped outer-first it must collapse to
// ~2 (one blade-off transit around the single obstacle), bounded here at 3.
TEST(CoverageContinuousPath, SingleCentralHoleDoesNotOverFragment)
{
  constexpr double kSide = 10.5;
  f2c::types::Cell cell = makeSquare(kSide);
  const double c = kSide / 2.0;  // centre; ~1 m square hole about it
  f2c::types::LinearRing hole;
  hole.addPoint(f2c::types::Point(c - 0.5, c - 0.5));
  hole.addPoint(f2c::types::Point(c + 0.5, c - 0.5));
  hole.addPoint(f2c::types::Point(c + 0.5, c + 0.5));
  hole.addPoint(f2c::types::Point(c - 0.5, c + 0.5));
  hole.addPoint(f2c::types::Point(c - 0.5, c - 0.5));
  cell.addRing(hole);

  const auto plan = planBoustrophedon(cell, 0.16, 0.18, 0, 0.0, -1.0, 0.15);
  ASSERT_FALSE(plan.rings.empty());
  // inset 0.0 leaves safe_boundary empty → fall back to the raw field ring.
  const auto boundary = plan.safe_boundary.empty() ? squareRing(kSide) : plan.safe_boundary;
  const auto subs = buildContinuousSubPaths(plan, boundary, 0.18, 0.15, 0.05);
  EXPECT_GE(subs.size(), 2u) << "the central hole must still split the path (safety)";
  EXPECT_LE(subs.size(), 3u) << subs.size()
                             << " sub-paths — over-fragmentation regression (was 18)";
}

// Sub-path DRIVE ORDER: on a multi-lobe field the sub-paths are reordered by
// nearest-neighbour (entering each at whichever end is nearer) to cut the
// blade-off Nav2 transit between them — measured 77 → 47 m on the recorded
// 4-hole garden. Reversing a FINISHED sub-path polyline is safe (identical
// points → every turn-around stays in-bounds), so the reorder must preserve two
// safety-critical properties: (1) every sub-path is still hole-free, and (2) the
// order is DETERMINISTIC — the BT resumes coverage by sub-path index, so a
// re-plan of the same area must reproduce the identical sequence.
TEST(CoverageContinuousPath, SubPathReorderStaysHoleFreeAndDeterministic)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kInset = 0.15;
  constexpr double kMinSwath = 0.15;
  constexpr double kTurnRadius = 0.18;
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;

  // 10 m square with two separated holes. Sub-paths split ONLY where a blade-on
  // connector cannot clear a hole (obstacle-driven, issue #333) — a lobe change
  // that passes to the SIDE of a hole stays blade-on, so this field yields a
  // small number of hole-free lobes (>= 2), enough to exercise the NN reorder.
  // The reorder's SAFETY contract (every sub-path stays hole-free; the order is
  // deterministic for BT resume-by-index) is what this test guards, not a
  // specific lobe count.
  f2c::types::Cell cell = makeSquare(10.0);
  auto add_hole = [&cell](double cx, double cy, double h)
  {
    f2c::types::LinearRing r;
    r.addPoint(f2c::types::Point(cx - h, cy - h));
    r.addPoint(f2c::types::Point(cx + h, cy - h));
    r.addPoint(f2c::types::Point(cx + h, cy + h));
    r.addPoint(f2c::types::Point(cx - h, cy + h));
    r.addPoint(f2c::types::Point(cx - h, cy - h));
    cell.addRing(r);
  };
  add_hole(3.0, 3.0, 0.9);
  add_hole(7.0, 7.0, 0.9);

  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);
  ASSERT_GE(plan.safe_holes.size(), 2u) << "both holes must reach the plan as safe_holes";
  const auto subs =
      buildContinuousSubPaths(plan, plan.safe_boundary, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_GE(subs.size(), 2u) << "holes must split the plan into >=2 hole-free lobes to reorder";

  // (1) Hole-free preserved after reorder/reversal.
  std::size_t in_hole = 0, total = 0;
  for (const auto& sp : subs)
  {
    ASSERT_GE(sp.size(), 2u) << "degenerate sub-path after reorder";
    for (const auto& p : sp)
    {
      ++total;
      for (const auto& hole : plan.safe_holes)
      {
        if (pointInRing(p.first, p.second, hole))
        {
          ++in_hole;
          break;
        }
      }
    }
  }
  EXPECT_EQ(in_hole, 0u) << in_hole << "/" << total << " poses inside a hole after reorder";

  // (2) Deterministic across re-plans (BT resume-by-index stability).
  const auto subs2 =
      buildContinuousSubPaths(plan, plan.safe_boundary, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_EQ(subs.size(), subs2.size()) << "sub-path count not reproducible";
  for (std::size_t i = 0; i < subs.size(); ++i)
  {
    ASSERT_EQ(subs[i].size(), subs2[i].size())
        << "sub-path " << i << " length differs across re-plans";
    EXPECT_NEAR(subs[i].front().first, subs2[i].front().first, 1e-9)
        << "sub-path " << i << " start x";
    EXPECT_NEAR(subs[i].front().second, subs2[i].front().second, 1e-9)
        << "sub-path " << i << " start y";
    EXPECT_NEAR(subs[i].back().first, subs2[i].back().first, 1e-9) << "sub-path " << i << " end x";
  }

  // (3) The realized inter-sub-path transit is finite and sane (a runaway order
  // would criss-cross far more than a few field diagonals).
  double transit = 0.0;
  for (std::size_t i = 1; i < subs.size(); ++i)
  {
    transit += std::hypot(subs[i].front().first - subs[i - 1].back().first,
                          subs[i].front().second - subs[i - 1].back().second);
  }
  const double field_diag = 10.0 * std::sqrt(2.0);
  EXPECT_LT(transit, field_diag * static_cast<double>(subs.size()))
      << "relocation transit " << transit << " m implausibly long for " << subs.size()
      << " sub-paths — the NN order is not sequencing lobes locally";
}

// #335: ring_direction controls the perimeter/headland travel winding (blade
// side). 1 = clockwise (negative shoelace area), 2 = counter-clockwise
// (positive). The two produce the same ring geometry driven the opposite way.
TEST(CoveragePlanning, RingDirectionControlsWinding)
{
  auto signedArea = [](const std::vector<std::pair<double, double>>& loop)
  {
    double a = 0.0;
    for (std::size_t i = 0; i + 1 < loop.size(); ++i)
    {
      a += loop[i].first * loop[i + 1].second - loop[i + 1].first * loop[i].second;
    }
    return a;
  };
  const auto cell = makeSquare(4.0);
  const auto cw = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15, /*ring_direction=*/1);
  const auto ccw = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15, /*ring_direction=*/2);
  ASSERT_FALSE(cw.rings.empty());
  ASSERT_FALSE(ccw.rings.empty());
  EXPECT_LT(signedArea(cw.rings.front()), 0.0) << "CW ring must have negative signed area";
  EXPECT_GT(signedArea(ccw.rings.front()), 0.0) << "CCW ring must have positive signed area";
  // Default (0) leaves F2C's natural winding untouched.
  const auto def = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15, /*ring_direction=*/0);
  ASSERT_FALSE(def.rings.empty());
}

// Field report (2026-07): a tall field with a small concave bite on one edge.
// BoustrophedonOrder interleaves the swath pieces the bite splits
// (below,above,below,above…), which used to force one blade-on field-crossing
// join per column — long diagonals mowed across the middle of the lawn, plus a
// straight fallback THROUGH the bite (out of bounds, boundary-guard trip).
// The nearest-endpoint chaining + the 0.6 m join-gap split must yield: each lobe
// mowed contiguously, a handful of sub-paths, zero out-of-bounds poses, and NO
// long blade-on connector run away from the planned swaths/rings.
TEST(CoverageContinuousPath, NotchFieldLobeChainedNoMidFieldJoins)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kInset = 0.15;
  constexpr double kMinSwath = 0.15;
  constexpr double kTurnRadius = 0.18;  // deployed connector_turn_radius default
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;

  // ~8.7 x 29 m (252 m²) with a bite on the left edge at y ≈ 21.3–23.0.
  const std::vector<std::pair<double, double>> outer = {{0.5, 0.0},
                                                        {8.2, 0.0},
                                                        {8.7, 0.5},
                                                        {8.7, 28.5},
                                                        {8.2, 29.0},
                                                        {0.5, 29.0},
                                                        {0.0, 28.5},
                                                        {0.0, 23.0},
                                                        {0.8, 22.6},
                                                        {0.9, 22.0},
                                                        {0.6, 21.6},
                                                        {0.0, 21.3},
                                                        {0.0, 0.5}};
  f2c::types::LinearRing ring;
  for (const auto& p : outer)
  {
    ring.addPoint(f2c::types::Point(p.first, p.second));
  }
  ring.addPoint(f2c::types::Point(outer.front().first, outer.front().second));
  const f2c::types::Cell cell{ring};

  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty());
  ASSERT_GE(plan.swaths.size(), 40u);
  ASSERT_GE(plan.safe_boundary.size(), 3u);

  const auto subs =
      buildContinuousSubPaths(plan, plan.safe_boundary, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_GE(subs.size(), 2u) << "the bite must split at least one lobe change";
  // With the outermost ring now on the recorded line (chassis_safety_inset is
  // the ring-centerline inset, so the planning field is only shrunk by
  // inset − op_width/2 = 0.07 m here instead of the old 0.15 m), the left bite
  // stays sharper and the notch chains into one extra lobe. Still a handful of
  // relocations, not a per-column explosion.
  EXPECT_LE(subs.size(), 6u) << subs.size()
                             << " sub-paths — one split per column instead of per lobe";

  // Every plan primitive as a segment list (swaths + densified ring edges), to
  // classify poses as on-primitive vs connector.
  std::vector<std::array<double, 4>> prim;
  for (const auto& s : plan.swaths)
  {
    prim.push_back({s.first.first, s.first.second, s.second.first, s.second.second});
  }
  for (const auto& loop : plan.rings)
  {
    for (std::size_t i = 0; i + 1 < loop.size(); ++i)
    {
      prim.push_back({loop[i].first, loop[i].second, loop[i + 1].first, loop[i + 1].second});
    }
  }
  auto distToPrims = [&prim](double x, double y)
  {
    double best = std::numeric_limits<double>::max();
    for (const auto& s : prim)
    {
      const double dx = s[2] - s[0], dy = s[3] - s[1];
      const double l2 = dx * dx + dy * dy;
      double t = l2 > 1e-12 ? ((x - s[0]) * dx + (y - s[1]) * dy) / l2 : 0.0;
      t = std::max(0.0, std::min(1.0, t));
      best = std::min(best, std::hypot(x - (s[0] + t * dx), y - (s[1] + t * dy)));
    }
    return best;
  };

  std::size_t oob = 0;
  double worst_conn_run = 0.0;
  for (const auto& path : subs)
  {
    double run = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
      if (!pointInRing(path[i].first, path[i].second, plan.safe_boundary))
      {
        ++oob;
      }
      const bool on_conn = distToPrims(path[i].first, path[i].second) > 0.30;
      if (on_conn && i > 0)
      {
        run += std::hypot(path[i].first - path[i - 1].first, path[i].second - path[i - 1].second);
      }
      else
      {
        worst_conn_run = std::max(worst_conn_run, run);
        run = 0.0;
      }
    }
    worst_conn_run = std::max(worst_conn_run, run);
  }
  // No blade-on pose out of bounds (the straight-through-the-bite fallback).
  EXPECT_EQ(oob, 0u) << oob << " blade-on poses outside the safety inset";
  // Field report 2026-07 ("robot stalls after every headland ring"): F2C closed
  // every ring on the same polygon corner, so the ring CLOSURE vertex and the
  // ring→ring junctions carried ~112° near-cusps FTC fought the drivetrain
  // deadband through. Pin the fixed geometry directly on plan.rings: every
  // closure must sit on a straight (mid-longest-edge rotation) and every
  // ring→ring junction must be a near-parallel sideways shift.
  auto stepHeading = [](const std::pair<double, double>& a, const std::pair<double, double>& b)
  {
    return std::atan2(b.second - a.second, b.first - a.first);
  };
  auto turnDeg = [](double h_in, double h_out)
  {
    double d = h_out - h_in;
    while (d > M_PI)
      d -= 2.0 * M_PI;
    while (d < -M_PI)
      d += 2.0 * M_PI;
    return std::fabs(d) * 180.0 / M_PI;
  };
  for (std::size_t i = 0; i < plan.rings.size(); ++i)
  {
    const auto& L = plan.rings[i];
    ASSERT_GE(L.size(), 4u);
    // Closure smoothness: heading into the closure vs heading out of the start.
    const double closure_turn =
        turnDeg(stepHeading(L[L.size() - 2], L[L.size() - 1]), stepHeading(L[0], L[1]));
    EXPECT_LT(closure_turn, 30.0) << "ring " << i << " closes with a " << closure_turn
                                  << "° corner — closure not on a straight edge";
    if (i + 1 < plan.rings.size())
    {
      const auto& N = plan.rings[i + 1];
      const double junction_turn =
          turnDeg(stepHeading(L[L.size() - 2], L[L.size() - 1]), stepHeading(N[0], N[1]));
      EXPECT_LT(junction_turn, 30.0)
          << "ring " << i << "→" << i + 1 << " junction turns " << junction_turn
          << "° — rings no longer chain on parallel edges (the FTC stall geometry)";
    }
  }
  // Turn-around teardrops are ~1.5 m of connector; the old mid-field diagonals
  // were 10–25 m. Anything beyond 3.5 m is a relocation being mowed.
  EXPECT_LT(worst_conn_run, 3.5)
      << "a " << worst_conn_run << " m blade-on connector run crosses the field — a "
      << "relocation is being mowed instead of split into a Nav2 transit";
}

// Concave L-shape: covered without decomposition — swaths exist in BOTH lobes
// and none crosses the notch.
TEST(CoveragePlanning, ConcaveFieldIsCovered)
{
  const double size = 4.0, notch = 2.0;
  const auto plan = planDefault(makeLShape(size, notch));
  ASSERT_GE(plan.swaths.size(), 6u);

  // The notch square (top-right bite) must not be crossed.
  const std::vector<std::pair<double, double>> notch_ring = {{size - notch, size - notch},
                                                             {size, size - notch},
                                                             {size, size},
                                                             {size - notch, size}};
  bool has_left_lobe = false, has_bottom_lobe = false;
  for (const auto& s : plan.swaths)
  {
    const int n = std::max(2, static_cast<int>(swathLen(s) / 0.05));
    for (int k = 0; k <= n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      const double x = s.first.first + t * (s.second.first - s.first.first);
      const double y = s.first.second + t * (s.second.second - s.first.second);
      EXPECT_FALSE(pointInRing(x, y, notch_ring))
          << "swath crosses the notch at (" << x << ", " << y << ")";
      if (x < size - notch && y > size - notch)
      {
        has_left_lobe = true;  // upper-left lobe
      }
      if (y < size - notch)
      {
        has_bottom_lobe = true;  // bottom band
      }
    }
  }
  EXPECT_TRUE(has_left_lobe) << "no swath in the upper-left lobe";
  EXPECT_TRUE(has_bottom_lobe) << "no swath in the bottom band";
}

// Tiny field: insets consume everything → empty plan (the server reports
// failure instead of retry-storming F2C).
TEST(CoveragePlanning, TooSmallFieldGivesEmptyPlan)
{
  const auto plan = planDefault(makeSquare(0.4));
  EXPECT_TRUE(plan.rings.empty());
  EXPECT_TRUE(plan.swaths.empty());
}

// num_headland_passes is a THREE-WAY sentinel (#429). This pins the two LEGACY
// meanings — they must survive the "disable the rings" change untouched:
//   > 0  forces exactly that ring count
//   == 0 AUTO = ceil(headland_width / op_width), floored at 1
TEST(CoveragePlanning, HeadlandPassOverride)
{
  const auto forced = planBoustrophedon(makeSquare(4.0),
                                        0.16,
                                        0.18,
                                        /*passes=*/3,
                                        0.08,
                                        -1.0,
                                        0.15);
  EXPECT_EQ(forced.rings.size(), 3u);

  // AUTO (0) still floors at 1 ring: headland 0.18 / op_width 0.16 → ceil = 2,
  // and even a sub-op_width headland must yield at least one perimeter ring.
  const auto automatic = planBoustrophedon(makeSquare(4.0),
                                           0.16,
                                           0.18,
                                           /*passes=*/0,
                                           0.08,
                                           -1.0,
                                           0.15);
  EXPECT_GE(automatic.rings.size(), 1u) << "passes=0 must keep meaning AUTO, floored at 1 ring";
}

// #429: a NEGATIVE num_headland_passes disables the ring stage entirely — the
// serpentine swaths become the outermost driven pass. Regression guard for the
// FLOOR on the rings-off field offset: field_offset is
// max(0, chassis_safety_inset - op_width/2) with the rings off, so at inset 0
// it is 0 and the planning cell is the recorded polygon. Removing the floor
// (i.e. letting the expression go negative the way the rings-on branch
// deliberately does, so ring 0 can ride ON the line) would EXPAND the planning
// cell, and since F2C clips swaths exactly to it every swath END would land
// ~0.08 m PAST the recorded line — where the robot meets the boundary head-on,
// a direction chassis_safety_inset has never modelled.
TEST(CoveragePlanning, HeadlandPassesDisabledYieldsNoRings)
{
  constexpr double kSize = 4.0;
  constexpr double kOpWidth = 0.16;
  const auto plan = planBoustrophedon(makeSquare(kSize),
                                      kOpWidth,
                                      0.18,
                                      /*passes=*/-1,
                                      /*chassis_safety_inset=*/0.0,
                                      -1.0,
                                      0.15);
  EXPECT_TRUE(plan.rings.empty()) << "negative num_headland_passes must plan ZERO rings";
  ASSERT_GE(plan.swaths.size(), 5u) << "no serpentine swaths planned with the rings off";

  const auto boundary = squareRing(kSize);
  // Swath ends ride ON the line by design (inset 0); allow only densification /
  // buffer noise outside it. 0.01 m << the 0.08 m (op_width/2) breach the bug
  // would produce.
  constexpr double kOnLineTolM = 0.01;
  for (const auto& s : plan.swaths)
  {
    for (const auto& pt : {s.first, s.second})
    {
      if (!pointInRing(pt.first, pt.second, boundary))
      {
        EXPECT_LE(distanceToRing(pt.first, pt.second, boundary), kOnLineTolM)
            << "swath endpoint (" << pt.first << ", " << pt.second
            << ") sits past the recorded boundary — the ring-0 field offset was applied "
               "with no ring 0 (#429)";
      }
    }
  }
}

// Smallest distance from the recorded ring to the MIDPOINT of any swath. A
// swath midpoint is far from both swath ends, so its nearest ring edge is the
// one PARALLEL to the sweep — i.e. this measures the LATERAL placement of the
// outermost swath centerline, independently of where the ends are clipped.
double minSwathMidDistance(
    const std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>>& swaths,
    const std::vector<std::pair<double, double>>& ring)
{
  double best = std::numeric_limits<double>::max();
  for (const auto& s : swaths)
  {
    const double mx = 0.5 * (s.first.first + s.second.first);
    const double my = 0.5 * (s.first.second + s.second.second);
    best = std::min(best, distanceToRing(mx, my, ring));
  }
  return best;
}

// Smallest distance from the recorded ring to any swath ENDPOINT (the
// LONGITUDINAL contact: F2C clips every sweep line to the planning cell, so the
// ends land on the cell edge).
double minSwathEndDistance(
    const std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>>& swaths,
    const std::vector<std::pair<double, double>>& ring)
{
  double best = std::numeric_limits<double>::max();
  for (const auto& s : swaths)
  {
    for (const auto& pt : {s.first, s.second})
    {
      best = std::min(best, distanceToRing(pt.first, pt.second, ring));
    }
  }
  return best;
}

// EXPERIMENT PIN (settles the swath-placement question in CI, against the REAL
// Fields2Cover v3 these tests link): BruteForce places the OUTERMOST swath
// centerline exactly op_width/2 inside the planning cell — upstream
// `SwathGeneratorBase::getSwathsDistribution` does `sx[0] = 0.5 * cov_width`
// measured from the rotated cell's bbox edge, the SAME convention
// `ConstHL::generateHeadlandSwaths` uses for ring 0 (`buffer(-w * (i + 0.5))`).
// The entire field_offset derivation rests on this being true for BOTH
// generators. If an F2C bump ever changed it (a flush-to-edge distribution, or
// an END_OVERLAP swath appended in front of sx[0]), this fails loudly instead of
// silently biasing every swath by op_width/2.
//
// Measured directly: chassis_safety_inset == op_width/2 makes field_offset 0, so
// the planning cell IS the recorded square.
TEST(CoveragePlanning, F2cPlacesOutermostSwathHalfOpWidthInsideThePlanningCell)
{
  constexpr double kSize = 6.0;
  constexpr double kOpWidth = 0.16;

  const auto plan = planBoustrophedon(makeSquare(kSize),
                                      kOpWidth,
                                      0.18,
                                      /*passes=*/-1,
                                      /*chassis_safety_inset=*/kOpWidth / 2.0,
                                      -1.0,
                                      0.15);
  ASSERT_TRUE(plan.rings.empty());
  ASSERT_GE(plan.swaths.size(), 5u) << "no serpentine swaths planned";

  const auto boundary = squareRing(kSize);
  EXPECT_NEAR(minSwathMidDistance(plan.swaths, boundary), kOpWidth / 2.0, 0.01)
      << "F2C no longer places the outermost swath centerline op_width/2 inside the planning "
         "cell — the field_offset derivation in planBoustrophedon is invalidated";
}

// #429 GEOMETRY CONTRACT with the rings OFF. chassis_safety_inset means "how far
// inside the recorded line the OUTERMOST DRIVEN PASS's centerline sits". With no
// rings the outermost driven pass is the outermost SWATH, and F2C insets it by
// op_width/2 exactly as it insets ring 0 (see the experiment pin above), so the
// −op_width/2 field offset applies in BOTH branches:
//   outermost swath CENTERLINE  → kInset from the recorded line (lateral)
//   swath ENDS (clipped to the planning cell) → kInset − op_width/2 (longitudinal)
// The pre-fix code dropped the term with the rings off, which pushed BOTH out by
// op_width/2 — so "no perimeter rings" mowed 8 cm FURTHER from the edge than 2
// rings did, the opposite of what the setting promises.
TEST(CoveragePlanning, HeadlandDisabledPlacesOutermostSwathAtChassisInset)
{
  constexpr double kSize = 6.0;
  constexpr double kOpWidth = 0.16;
  constexpr double kInset = 0.20;  // = robot_width/2 for the 0.40 m chassis
  constexpr double kSlack = 0.02;  // densification / GEOS buffer noise

  const auto plan = planBoustrophedon(makeSquare(kSize),
                                      kOpWidth,
                                      0.18,
                                      /*passes=*/-1,
                                      kInset,
                                      -1.0,
                                      0.15);
  EXPECT_TRUE(plan.rings.empty());
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths planned with rings off + a 0.20 m inset";

  const auto boundary = squareRing(kSize);
  for (const auto& s : plan.swaths)
  {
    for (const auto& pt : {s.first, s.second})
    {
      ASSERT_TRUE(pointInRing(pt.first, pt.second, boundary))
          << "swath endpoint (" << pt.first << ", " << pt.second << ") is outside the boundary";
    }
  }

  EXPECT_NEAR(minSwathMidDistance(plan.swaths, boundary), kInset, kSlack)
      << "the outermost swath centerline must sit chassis_safety_inset inside the recorded line "
         "with the rings off (pre-fix it sat kInset + op_width/2 = 0.28 m in)";
  EXPECT_NEAR(minSwathEndDistance(plan.swaths, boundary), kInset - kOpWidth / 2.0, kSlack)
      << "swath ENDS are clipped to the planning cell, which is eroded by "
         "(chassis_safety_inset - op_width/2)";
}

// #429 the operator-visible claim, pinned: choosing "None" must not leave a
// WIDER uncut border than the rings it replaces. Both modes are measured by the
// same yardstick — the distance from the recorded line to the outermost DRIVEN
// centerline (ring 0 with the rings on, the outermost swath with them off),
// which is what the cut edge is offset from by the same op_width/2 either way.
// Pre-fix: 0.20 m (rings on) vs 0.28 m (rings off) → the feature made it worse.
TEST(CoveragePlanning, HeadlandDisabledBorderNoWorseThanRingsOn)
{
  constexpr double kSize = 6.0;
  constexpr double kOpWidth = 0.16;
  constexpr double kInset = 0.20;
  const auto boundary = squareRing(kSize);

  const auto with_rings = planBoustrophedon(makeSquare(kSize),
                                            kOpWidth,
                                            0.18,
                                            /*passes=*/2,
                                            kInset,
                                            -1.0,
                                            0.15);
  ASSERT_FALSE(with_rings.rings.empty()) << "no rings planned with passes=2";
  double rings_on_outermost = std::numeric_limits<double>::max();
  for (const auto& ring : with_rings.rings)
  {
    for (const auto& p : ring)
    {
      rings_on_outermost =
          std::min(rings_on_outermost, distanceToRing(p.first, p.second, boundary));
    }
  }

  // Sanity-pin the yardstick itself so a failure below names the right cause:
  // with the rings ON, ring 0's centerline sits chassis_safety_inset in.
  EXPECT_NEAR(rings_on_outermost, kInset, 0.02)
      << "ring 0 no longer sits chassis_safety_inset inside the recorded line — the "
         "rings-off comparison below is measured against a moved baseline";

  const auto no_rings = planBoustrophedon(makeSquare(kSize),
                                          kOpWidth,
                                          0.18,
                                          /*passes=*/-1,
                                          kInset,
                                          -1.0,
                                          0.15);
  ASSERT_TRUE(no_rings.rings.empty());
  ASSERT_FALSE(no_rings.swaths.empty());
  const double rings_off_outermost = minSwathMidDistance(no_rings.swaths, boundary);

  EXPECT_LE(rings_off_outermost, rings_on_outermost + 0.01)
      << "with the rings OFF the outermost driven pass sits " << rings_off_outermost
      << " m inside the recorded line vs " << rings_on_outermost
      << " m with 2 rings — choosing \"None\" would leave a WIDER uncut border than the "
         "perimeter loop it removes (#429)";
}

// #429: with the rings off the connector clearance ring is safe_boundary
// EXACTLY — no outward expansion. An earlier revision expanded it by 0.03 m,
// which let a turn-around connector centerline ride 3 cm past the recorded line
// while staying invisible to the server's 0.05 m verify slack, eroding the
// keep-inside contract that is the SHIPPED mode (chassis_safety_inset 0.20 ==
// robot_width/2). On-edge swath ends are accepted by allInside()'s 1 mm
// tolerance instead — see HeadlandDisabledIsOneSubPath for the no-fragmentation
// half of that argument.
TEST(CoveragePlanning, HeadlandDisabledClearanceRingIsExactlySafeBoundary)
{
  constexpr double kSize = 6.0;
  constexpr double kOpWidth = 0.16;
  constexpr double kInset = 0.20;

  const auto plan = planBoustrophedon(makeSquare(kSize),
                                      kOpWidth,
                                      0.18,
                                      /*passes=*/-1,
                                      kInset,
                                      -1.0,
                                      0.15);
  ASSERT_TRUE(plan.rings.empty());
  ASSERT_GE(plan.safe_boundary.size(), 3u);
  ASSERT_EQ(plan.connector_clearance_boundary.size(), plan.safe_boundary.size())
      << "the clearance ring must be safe_boundary itself with the rings off";
  for (std::size_t i = 0; i < plan.safe_boundary.size(); ++i)
  {
    EXPECT_NEAR(plan.connector_clearance_boundary[i].first, plan.safe_boundary[i].first, 1e-9);
    EXPECT_NEAR(plan.connector_clearance_boundary[i].second, plan.safe_boundary[i].second, 1e-9);
  }

  // And it never pokes outside the recorded polygon.
  const auto boundary = squareRing(kSize);
  for (const auto& p : plan.connector_clearance_boundary)
  {
    EXPECT_TRUE(pointInRing(p.first, p.second, boundary))
        << "clearance-ring vertex (" << p.first << ", " << p.second
        << ") is outside the recorded boundary — the outward expansion is back";
  }
}

// #429 companion to TooSmallFieldGivesEmptyPlan: disabling the rings must not
// let a too-small field through the too-small guard, nor emit geometry outside
// the recorded boundary.
TEST(CoveragePlanning, HeadlandDisabledTinyFieldStillGivesEmptyPlan)
{
  // (a) An inset that leaves a field narrower than one swath must yield no
  // geometry at all — the server then reports failure. (0.4 m square, inset
  // 0.25 → planning cell 0.4 − 2*(0.25 − 0.08) = 0.06 m, well under the 0.16 m
  // swath spacing.)
  const auto consumed = planBoustrophedon(makeSquare(0.4),
                                          0.16,
                                          0.18,
                                          /*passes=*/-1,
                                          /*chassis_safety_inset=*/0.25,
                                          -1.0,
                                          0.15);
  EXPECT_TRUE(consumed.rings.empty()) << "rings are disabled, so there can never be any";
  EXPECT_TRUE(consumed.swaths.empty()) << "a fully-consumed field must yield no swaths";

  // (b) A field the inset merely SHRINKS: with the rings off, a short swath is
  // legitimate coverage rather than a degenerate plan — but nothing it emits
  // may escape the recorded boundary. At inset == op_width/2 the field offset
  // floors to 0, so the swath ENDS land exactly ON the recorded line; allow
  // that (pointInRing resolves on-edge poses inconsistently) but nothing past
  // it. This is the case that would break if the rings-off offset were allowed
  // to go NEGATIVE: the planning field would be EXPANDED and the ends would sit
  // ~0.08 m outside the polygon.
  constexpr double kSize = 0.4;
  constexpr double kOnLineTolM = 0.01;
  const auto tiny = planBoustrophedon(makeSquare(kSize),
                                      0.16,
                                      0.18,
                                      /*passes=*/-1,
                                      /*chassis_safety_inset=*/0.08,
                                      -1.0,
                                      0.15);
  EXPECT_TRUE(tiny.rings.empty());
  const auto boundary = squareRing(kSize);
  for (const auto& s : tiny.swaths)
  {
    for (const auto& pt : {s.first, s.second})
    {
      if (!pointInRing(pt.first, pt.second, boundary))
      {
        EXPECT_LE(distanceToRing(pt.first, pt.second, boundary), kOnLineTolM)
            << "swath endpoint (" << pt.first << ", " << pt.second
            << ") escaped the recorded boundary on a shrunken field (#429)";
      }
    }
  }
}

// OPT-IN keep-inside: the coverage server does NOT floor the inset (it clamps
// only at 0.0 — the default rides the outermost ring ON the recorded line, see
// RingRidesOnLineWhenInsetZero); an operator near hard fences opts the WHOLE
// chassis inside by setting chassis_safety_inset = chassis_width/2 explicitly.
// This test passes that value and asserts the GEOMETRIC GUARANTEE it provides on
// the STRAIGHT passes: no planned ring point or swath endpoint sits closer than
// robot_width/2 to the boundary. The turn-around CONNECTORS (which used to bulge
// op_width/2 further out than the rings) are covered separately by
// TurnArcFootprintStaysInsideRecordedBoundary — this test only exercises the
// discrete ring/swath geometry.
TEST(CoveragePlanning, ChassisInsetOptInKeepsBladesInsideHalfWidth)
{
  constexpr double kRobotWidth = 0.40;  // 0.40 m chassis (the field excursion case)
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  // Explicit opt-in to keep the whole chassis inside: chassis_safety_inset =
  // chassis_width/2. The outermost ring centerline then sits robot_width/2
  // inside the raw line, so the chassis edge just touches it.
  const double chassis_inset = kRobotWidth * 0.5;
  ASSERT_NEAR(chassis_inset, 0.20, 1e-9);

  const auto cell = makeSquare(6.0);  // big enough to still plan after a 0.20 m inset
  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, chassis_inset, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty()) << "no rings after the floored inset";
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths after the floored inset";

  const auto boundary = squareRing(6.0);
  // Densification (kDensifyStep ~0.10 m) and floating point can place a sampled
  // ring point a few cm shy of the analytic centerline; allow a small slack but
  // keep it well under the inset so a real breach (which would be ~0.20 m) fails.
  constexpr double kSlack = 0.03;
  const double min_clearance = kRobotWidth * 0.5 - kSlack;

  for (const auto& loop : plan.rings)
  {
    for (const auto& p : loop)
    {
      ASSERT_TRUE(pointInRing(p.first, p.second, boundary))
          << "ring point (" << p.first << ", " << p.second << ") is outside the boundary";
      EXPECT_GE(distanceToRing(p.first, p.second, boundary), min_clearance)
          << "ring point (" << p.first << ", " << p.second
          << ") is closer than robot_width/2 to the boundary";
    }
  }
  for (const auto& s : plan.swaths)
  {
    for (const auto& pt : {s.first, s.second})
    {
      ASSERT_TRUE(pointInRing(pt.first, pt.second, boundary))
          << "swath endpoint (" << pt.first << ", " << pt.second << ") is outside the boundary";
      EXPECT_GE(distanceToRing(pt.first, pt.second, boundary), min_clearance)
          << "swath endpoint (" << pt.first << ", " << pt.second
          << ") is closer than robot_width/2 to the boundary";
    }
  }
}

// SAFETY REGRESSION (turn-arc footprint): the pre-fix connectors were validated
// (allInside) only on the path CENTERLINE against plan.safe_boundary, which sits
// op_width/2 OUTSIDE the outermost ring's centerline. So a forward turn-around
// arc's centerline could reach op_width/2 further out than any ring/swath, and
// buildConnector picks the LARGEST radius whose centerline still fits — pushing
// the swept CHASSIS footprint (± robot_width/2) that much past the operator
// boundary, an excursion that GROWS with the turn radius. The fix bounds
// connectors to plan.connector_clearance_boundary (the outermost-ring centerline
// == recorded eroded by chassis_safety_inset), so a turn's footprint is never
// worse than the perimeter ring the robot already drives.
//
// With chassis_safety_inset = robot_width/2 the straight passes keep the whole
// chassis inside the recorded line; this test asserts the TURN arcs do too, via
// the centreline↔footprint identity (footprint_outer = centreline + robot_width/2,
// clearance ring = perimeter-ring centreline, so centreline ⊂ clearance ⟺ the
// swept footprint is no worse than the perimeter pass). It (1) HARD-asserts the
// pre-fix safe_boundary bound drives a connector centreline > slack (≈ op_width/2)
// past the clearance ring (bug reachable — a vacuous green is impossible), (2)
// requires every centreline of the clearance-bounded plan to stay within that
// ring, and (3) checks the plan is not fragmented into disconnected segments
// (which would satisfy (2) vacuously).
TEST(CoveragePlanning, TurnArcFootprintStaysInsideRecordedBoundary)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  constexpr double kRobotWidth = 0.40;  // chassis width (footprint is ± half this)
  constexpr double kInset = kRobotWidth * 0.5;  // opt-in: keep whole chassis inside
  constexpr double kTurnRadius = 0.18;
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;
  // Densification / discrete-normal estimate slack. Well under the ~op_width/2
  // (0.08 m) breach the bug produces.
  constexpr double kSlack = 0.05;

  // The operator's REAL recorded garden (recorded_area_1) — the elongated concave
  // field where the maintainer observes turns crossing the boundary. Its many
  // edge U-turns ride the connector bound: RecordedArea1NoCuspInBounds proves the
  // pre-fix connectors reach within (inset − op_width/2) of the raw line, i.e.
  // op_width/2 (0.08 m) PAST the outermost-ring clearance ring — so the RED guard
  // below (≥ 0.05 m) is provable, not incidental to a synthetic shape.
  const auto cell = makeRecordedArea1();

  const auto plan =
      planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath, 0, kMinTurnRadius);
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths to turn between";
  ASSERT_GE(plan.safe_boundary.size(), 3u);

  auto poseCount = [](const std::vector<std::vector<std::pair<double, double>>>& subs)
  {
    std::size_t n = 0;
    for (const auto& sub : subs)
    {
      n += sub.size();
    }
    return n;
  };

  // fix (a) must have populated the outermost-ring clearance ring.
  ASSERT_GE(plan.connector_clearance_boundary.size(), 3u)
      << "connector_clearance_boundary not populated — fix (a) missing";
  const std::vector<std::pair<double, double>>& clearance = plan.connector_clearance_boundary;

  // Max distance any CENTRELINE pose lies OUTSIDE the outermost-ring clearance
  // ring — the fix's actual contract (connectors must not push the path past the
  // perimeter ring the robot already drives). Ring/swath poses sit on/inside it by
  // construction, so this measures the connector/fillet outward bulge directly.
  auto centrelineExcursion = [&](const std::vector<std::vector<std::pair<double, double>>>& subs)
  {
    double m = 0.0;
    for (const auto& sub : subs)
    {
      for (const auto& p : sub)
      {
        m = std::max(m,
                     pointInRing(p.first, p.second, clearance)
                         ? 0.0
                         : distanceToRing(p.first, p.second, clearance));
      }
    }
    return m;
  };

  const auto unsafe =
      buildContinuousSubPaths(plan, plan.safe_boundary, kTurnRadius, kMinTurnRadius, kStep);
  const auto fixed = buildContinuousSubPaths(plan, clearance, kTurnRadius, kMinTurnRadius, kStep);

  // (1) Bug reachable — HARD guard (ASSERT, not EXPECT) so a vacuous green is
  // impossible: bounding connectors to safe_boundary lets a centreline ride up to
  // op_width/2 (0.08 m) past the outermost-ring envelope — the extra outward bulge
  // that (per buildConnector's accept-largest-radius) grows with the turn radius.
  // RecordedArea1NoCuspInBounds independently measures this: its connectors reach
  // within (inset − op_width/2) of the raw line, i.e. op_width/2 past the
  // clearance ring, so this excursion is ~0.08 m here — comfortably over kSlack.
  // If this ever fails, the pre-fix path did NOT exceed the clearance ring on this
  // field and the regression is untested — fail loudly rather than pass hollowly.
  const double unsafe_centreline_ex = centrelineExcursion(unsafe);
  ASSERT_GT(unsafe_centreline_ex, kSlack)
      << "pre-fix connectors stayed within the clearance ring (excursion " << unsafe_centreline_ex
      << " m ≤ " << kSlack
      << ") — the bug is not exercised on this field; choose one whose turns reach the boundary";

  // (2) Fix contract: every connector centreline stays within the clearance ring.
  // This is the precise chassis guarantee — centreline ⊂ clearance ⟺ the swept
  // footprint is no worse than the perimeter ring the robot already drives, which
  // (at chassis_safety_inset = robot_width/2) is the whole chassis inside the
  // recorded line. We do NOT assert an ABSOLUTE "every footprint inside recorded"
  // here: a ±robot_width/2 footprint can cross the raw line near a sharp CONCAVE
  // vertex of the real garden no matter how the path is routed — that is a field
  // property, not the turn-arc bug, and it appears equally in both builds.
  EXPECT_LE(centrelineExcursion(fixed), kSlack)
      << "a connector centreline still exceeds the outermost-ring clearance ring";
  // (3) The fix must not reach clearance by shredding the plan into disconnected
  // segments (dropping connectors would satisfy (2) vacuously). The op_width/2
  // tightening should shrink a few edge turns, not fragment the plan.
  const std::size_t unsafe_poses = poseCount(unsafe);
  const std::size_t fixed_poses = poseCount(fixed);
  EXPECT_GE(fixed_poses, static_cast<std::size_t>(0.75 * unsafe_poses))
      << "clearance bound over-fragmented the plan (" << fixed_poses << " vs " << unsafe_poses
      << " poses) — edge turns forced below min_turning_radius into straight-fallback splits";
}

// SAFETY REGRESSION (#388): outermost-ring connector start outside the clearance
// ring. With the DEFAULT chassis_safety_inset = 0 the outermost headland ring
// rides ON the recorded line, and F2C rounds the ring (generateHeadlandSwaths)
// and the clearance boundary (generateHeadlands) differently at convex corners —
// so a few outermost-ring vertices poke a few cm PAST the clearance ring. The
// first such pose is the seg-1 start TransitToStrip drives to; sitting outside
// the recorded line (in the keepout mask's lethal band) it made the blade-off
// transit un-drivable and the whole sub-path was skipped, collapsing coverage to
// the outer rings. buildContinuousSubPaths now projects every out-of-bounds pose
// back just inside the clearance ring. This asserts the continuous path — every
// pose, on the operator's REAL boundary-hugging recorded garden — stays inside
// the clearance ring the connectors are bounded to.
TEST(CoverageContinuousPath, OutermostRingStaysInsideClearanceRingOnTheLine)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  constexpr double kInset = 0.0;  // DEFAULT: outermost ring rides ON the recorded line
  constexpr double kTurnRadius = 0.18;
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;
  // Matches the server's kBoundarySlackM verify slack (coverage_server.cpp): a
  // pose within this of the ring from outside is on-the-line densification noise,
  // not a real excursion. The clamp places poses strictly INSIDE, so the check
  // below (pointInRing OR within slack) is met with room to spare.
  constexpr double kBoundarySlack = 0.05;

  const auto cell = makeRecordedArea1();
  const auto plan =
      planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath, 0, kMinTurnRadius);
  ASSERT_FALSE(plan.rings.empty()) << "no rings planned";
  ASSERT_GE(plan.connector_clearance_boundary.size(), 3u)
      << "connector_clearance_boundary not populated";
  const std::vector<std::pair<double, double>>& clearance = plan.connector_clearance_boundary;

  // The server bounds/verifies against connector_clearance_boundary — mirror that.
  const auto subs = buildContinuousSubPaths(plan, clearance, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_FALSE(subs.empty()) << "no continuous sub-paths built";

  std::size_t out_of_bounds = 0;
  double worst = 0.0;
  for (const auto& sub : subs)
  {
    for (const auto& p : sub)
    {
      if (!pointInRing(p.first, p.second, clearance))
      {
        const double d = distanceToRing(p.first, p.second, clearance);
        worst = std::max(worst, d);
        if (d > kBoundarySlack)
        {
          ++out_of_bounds;
        }
      }
    }
  }
  EXPECT_EQ(out_of_bounds, 0u)
      << out_of_bounds
      << " continuous-path pose(s) outside the outermost-ring clearance ring (worst " << worst
      << " m past it) — the connector/ring geometry escaped the safety inset (#388)";
}

// #429 REGRESSION: with the headland rings DISABLED the swath ENDS are the
// outermost driven geometry and sit exactly ON safe_boundary, which is also the
// connector clearance ring. If connector_clearance_boundary were still built by
// eroding op_width/2 inward, buildConnector's allInside() would reject every
// U-turn arc AND the straight fallback → conn_safe false → the sub-path SPLIT at
// every U-turn, i.e. one blade-off Nav2 transit per swath on a hole-free field.
// The clearance ring is safe_boundary EXACTLY (no outward expansion — that would
// let a connector centerline cross the recorded line); what keeps the on-edge
// ends acceptable is allInside()'s 1 mm kOnEdgeTolM tolerance, since pointInRing
// is a strict-`>` crossing test that resolves on-edge poses inconsistently.
//
// With no mowed apron beyond the swath ends most U-turn ARCS legitimately do not
// fit, so buildConnector falls back to a straight pivot-through join — that is
// the intended, documented behaviour of "None" and it still passes allInside, so
// this asserts the hole-free field yields exactly ONE sub-path and that no pose
// escapes the clearance ring by more than the server's verify slack.
TEST(CoverageContinuousPath, HeadlandDisabledIsOneSubPath)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  constexpr double kInset = 0.0;
  constexpr double kTurnRadius = 0.18;
  constexpr double kMinTurnRadius = 0.15;
  constexpr double kStep = 0.03;
  constexpr double kBoundarySlack = 0.05;  // == coverage_server.cpp kBoundarySlackM

  const auto plan = planBoustrophedon(makeSquare(6.0),
                                      kOpWidth,
                                      kHeadland,
                                      /*passes=*/-1,
                                      kInset,
                                      -1.0,
                                      kMinSwath,
                                      0,
                                      kMinTurnRadius);
  ASSERT_TRUE(plan.rings.empty());
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths planned";
  ASSERT_GE(plan.connector_clearance_boundary.size(), 3u)
      << "connector_clearance_boundary not populated with the rings off";
  const std::vector<std::pair<double, double>>& clearance = plan.connector_clearance_boundary;

  const auto subs = buildContinuousSubPaths(plan, clearance, kTurnRadius, kMinTurnRadius, kStep);
  EXPECT_EQ(subs.size(), 1u)
      << "a hole-free field must stay ONE sub-path with the headland rings off — " << subs.size()
      << " sub-paths means a blade-off Nav2 transit per U-turn (#429)";

  std::size_t out_of_bounds = 0;
  double worst = 0.0;
  for (const auto& sub : subs)
  {
    for (const auto& p : sub)
    {
      if (!pointInRing(p.first, p.second, clearance))
      {
        const double d = distanceToRing(p.first, p.second, clearance);
        worst = std::max(worst, d);
        if (d > kBoundarySlack)
        {
          ++out_of_bounds;
        }
      }
    }
  }
  EXPECT_EQ(out_of_bounds, 0u) << out_of_bounds << " pose(s) more than " << kBoundarySlack
                               << " m outside the clearance ring (worst " << worst << " m)";
}

// Headland-on-the-line: with chassis_safety_inset = 0 (the new default) the
// OUTERMOST ring rides ON the recorded boundary — the perimeter the operator
// drove — so its centerline comes within a densify step of the line instead of
// the ~op_width/2 (or old robot_width/2) inset that left an uncut perimeter band.
TEST(CoveragePlanning, RingRidesOnLineWhenInsetZero)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  const auto cell = makeSquare(6.0);
  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, 0.0, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty());
  const auto boundary = squareRing(6.0);
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& loop : plan.rings)
  {
    for (const auto& p : loop)
    {
      min_dist = std::min(min_dist, distanceToRing(p.first, p.second, boundary));
    }
  }
  // Straight ring edges land on the line; corner fillets pull in slightly. The
  // OLD behaviour put every ring point >= op_width/2 (0.08 m) — with the removed
  // floor >= 0.20 m — inside, so this bound both proves on-the-line and guards
  // against the inset silently regressing.
  EXPECT_LT(min_dist, 0.05) << "outermost ring is " << min_dist
                            << " m inside the recorded line, not on it";
}

// INSTRUMENTATION (Bug A): planBoustrophedon reports a planned-coverage fraction
// and a non-empty plan over a healthy field. The fraction is a coarse strip-area
// estimate (visibility, not a guarantee), so we only bound it loosely.
TEST(CoveragePlanning, DiagnosticsReportPlannedFraction)
{
  const auto cell = makeSquare(6.0);
  const auto plan = planDefault(cell);
  ASSERT_FALSE(plan.swaths.empty());
  EXPECT_NEAR(plan.diagnostics.field_area, 36.0, 1e-6);
  EXPECT_GT(plan.diagnostics.planned_area, 0.0);
  // A 6 m square at 0.16 m spacing tiles densely — most of the field is planned.
  EXPECT_GT(plan.diagnostics.planned_fraction, 0.5);
}

// pointInRing / distanceToRing handle a concave notch (kept from the previous
// suite — the in-bounds verification depends on them).
TEST(BoundaryClipGeometry, PointInRingHandlesConcaveNotch)
{
  const std::vector<std::pair<double, double>> ring = {
      {0, 0}, {4, 0}, {4, 4}, {2.5, 4}, {2.5, 2}, {1.5, 2}, {1.5, 4}, {0, 4}};

  EXPECT_TRUE(pointInRing(0.5, 0.5, ring));
  EXPECT_TRUE(pointInRing(3.5, 3.5, ring));
  EXPECT_FALSE(pointInRing(2.0, 3.0, ring));  // inside the notch = outside
  EXPECT_FALSE(pointInRing(5.0, 2.0, ring));
  EXPECT_FALSE(pointInRing(-1.0, 2.0, ring));

  EXPECT_NEAR(distanceToRing(2.0, 1.0, ring), 1.0, 1e-6);
  EXPECT_NEAR(distanceToRing(0.5, 0.0, ring), 0.0, 1e-6);
}

// ===========================================================================
// INTEGRATION: plan on the operator's REAL recorded area and analyse the trace
// the way the robot drives it — ordered segments → continuous runs (split at a
// gap > kSegmentTransitGap, FollowStrip's logic) → the MPPI path-inversion crop
// per run (enforce_path_inversion). This reproduces, offline, the "lost at the
// end of the headland" class of bug: it pinpoints where each run would be
// cropped on the real concave geometry, and verifies the plan stays in-bounds.
// ===========================================================================
TEST(CoverageIntegration, RecordedArea1FullTraceAnalysis)
{
  constexpr double kOpWidth = 0.16;  // tool_width 0.18 − swath_overlap 0.02
  constexpr double kHeadland = 0.18;  // deployed headland_width
  constexpr double kInset = 0.15;  // deployed chassis_safety_inset
  constexpr double kMinSwath = 0.15;
  constexpr double kTransitGap = 0.6;  // FollowStrip kSegmentTransitGap
  constexpr double kDensify = 0.10;  // swath densify step (≈ ring step)

  const auto cell = makeRecordedArea1();
  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);

  ASSERT_FALSE(plan.rings.empty()) << "no headland rings produced on the real area";
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths produced on the real area";
  const auto& boundary = recordedArea1Pts();

  // Build the ordered segments exactly as the robot drives them: rings
  // (densified loops) first, then serpentine swaths (densified start→end).
  struct Seg
  {
    std::vector<std::pair<double, double>> pts;
    bool is_ring;
  };
  std::vector<Seg> segs;
  for (const auto& loop : plan.rings)
  {
    segs.push_back({loop, true});
  }
  for (const auto& sw : plan.swaths)
  {
    std::vector<std::pair<double, double>> pts;
    const double dx = sw.second.first - sw.first.first;
    const double dy = sw.second.second - sw.first.second;
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / kDensify)));
    for (int k = 0; k <= n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      pts.push_back({sw.first.first + t * dx, sw.first.second + t * dy});
    }
    segs.push_back({pts, false});
  }

  // In-bounds check (the plan is built from insets — any pose outside is a bug).
  std::size_t oob = 0, total_pts = 0;
  for (const auto& s : segs)
  {
    for (const auto& p : s.pts)
    {
      ++total_pts;
      if (!pointInRing(p.first, p.second, boundary))
      {
        ++oob;
      }
    }
  }

  // Split into continuous runs at gaps > kTransitGap (FollowStrip's run logic).
  struct Run
  {
    std::vector<std::pair<double, double>> pts;
    std::size_t first_seg, last_seg;
    bool has_ring = false, has_swath = false;
    double entry_gap = 0.0;
  };
  std::vector<Run> runs;
  for (std::size_t i = 0; i < segs.size(); ++i)
  {
    double gap = 0.0;
    if (!runs.empty())
    {
      const auto& a = runs.back().pts.back();
      const auto& b = segs[i].pts.front();
      gap = std::hypot(b.first - a.first, b.second - a.second);
    }
    if (runs.empty() || gap > kTransitGap)
    {
      runs.push_back(Run{});
      runs.back().first_seg = i;
      runs.back().entry_gap = gap;
    }
    runs.back().pts.insert(runs.back().pts.end(), segs[i].pts.begin(), segs[i].pts.end());
    runs.back().last_seg = i;
    runs.back().has_ring = runs.back().has_ring || segs[i].is_ring;
    runs.back().has_swath = runs.back().has_swath || !segs[i].is_ring;
  }

  std::cout << "\n=== recorded_area_1 trace analysis ===\n"
            << "rings(loops)=" << plan.rings.size() << "  swaths=" << plan.swaths.size()
            << "  angle=" << plan.swath_angle_rad * 180.0 / M_PI << " deg\n"
            << "segments=" << segs.size() << "  poses=" << total_pts << "  out_of_bounds=" << oob
            << "\n"
            << "continuous runs=" << runs.size() << "  (split at gap > " << kTransitGap << " m)\n";
  for (std::size_t r = 0; r < runs.size(); ++r)
  {
    const std::size_t inv = firstInversion(runs[r].pts);
    std::cout << "  run " << r << ": segs[" << runs[r].first_seg << ".." << runs[r].last_seg << "] "
              << (runs[r].has_ring ? "RING " : "") << (runs[r].has_swath ? "SWATH" : "")
              << "  poses=" << runs[r].pts.size() << "  entry_gap=" << runs[r].entry_gap << "m"
              << "  first_inversion="
              << (inv < runs[r].pts.size()
                      ? std::to_string(inv) + "/" + std::to_string(runs[r].pts.size())
                      : std::string("none"))
              << "\n";
  }
  std::cout << std::flush;

  // SINGLE-CONTINUOUS-PATH safety: if we drop the per-run split and feed ONE
  // joined path (the operator's request, now that the arc-length MPPI fix lets
  // it track reversals), the run boundaries become straight blind connectors.
  // The arc-length fix does NOT make a connector in-bounds — it's a real gap.
  // Sample each connector and verify it stays inside the boundary, so we know
  // BEFORE deploying whether a naive single join is safe on this concave area.
  std::size_t connector_oob = 0;
  double max_connector_gap = 0.0;
  for (std::size_t r = 1; r < runs.size(); ++r)
  {
    const auto& a = runs[r - 1].pts.back();  // previous run end
    const auto& b = runs[r].pts.front();  // this run start
    max_connector_gap = std::max(max_connector_gap, runs[r].entry_gap);
    const int n = std::max(1, static_cast<int>(std::ceil(runs[r].entry_gap / 0.10)));
    for (int k = 0; k <= n; ++k)
    {
      const double t = static_cast<double>(k) / n;
      const double x = a.first + t * (b.first - a.first);
      const double y = a.second + t * (b.second - a.second);
      if (!pointInRing(x, y, boundary))
      {
        ++connector_oob;
      }
    }
  }
  std::cout << "single-join connectors: max_gap=" << max_connector_gap
            << "m  out_of_bounds_samples=" << connector_oob
            << (connector_oob == 0 ? "  -> single join is IN-BOUNDS"
                                   : "  -> single join CROSSES BOUNDARY (needs gap minimisation)")
            << "\n"
            << std::flush;

  // The plan must stay inside the recorded boundary, and produce ≥1 drivable run.
  EXPECT_EQ(oob, 0u) << oob << "/" << total_pts << " poses fall outside the boundary";
  EXPECT_GE(runs.size(), 1u);
}

// ===========================================================================
// CONTINUOUS PATH: flatten the plan into ONE cusp-free, in-bounds polyline via
// forward turn-around connectors at every reversal, so MPPI tracks it without
// the bimodal dither/spin at sharp 180° reversals. Re-mowing on the turns is
// accepted. Asserts on the REAL operator area with the deployed knobs.
// ===========================================================================
TEST(CoverageContinuousPath, RecordedArea1NoCuspInBounds)
{
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kInset = 0.15;
  constexpr double kMinSwath = 0.15;
  constexpr double kTurnRadius = 0.18;  // deployed connector_turn_radius default
  constexpr double kMinTurnRadius = 0.15;  // robot's min MPPI-trackable radius
  constexpr double kStep = 0.03;

  const auto cell = makeRecordedArea1();
  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty()) << "no headland rings on the real area";
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths on the real area";

  const auto& boundary = recordedArea1Pts();
  // SAFETY: the server bounds the connectors/fillets by the chassis-safety-inset
  // ring (plan.safe_boundary), NOT the raw operator polygon — otherwise a
  // turn-around loop or fillet near a field edge pushes the spinning blade across
  // the operator boundary. Mirror the server exactly so this test guards the real
  // behaviour.
  ASSERT_GE(plan.safe_boundary.size(), 3u)
      << "plan.safe_boundary not populated for a deployed inset — connectors would "
         "fall back to the raw boundary (the safety bug this guards)";
  const auto& connector_boundary = plan.safe_boundary;
  // The driven units are the SUB-PATHS: each is an independently continuous,
  // cusp-free, in-bounds MPPI run; the gap BETWEEN sub-paths is a blade-off Nav2
  // transit (a relocation, not a driven cusp), so the per-path invariants below
  // apply PER SUB-PATH, not across the viz-only concatenation.
  const auto subs =
      buildContinuousSubPaths(plan, connector_boundary, kTurnRadius, kMinTurnRadius, kStep);
  ASSERT_GE(subs.size(), 1u);
  // A concave garden may split at lobe changes, but the nearest-endpoint
  // chaining must keep it to a handful of relocations — a per-column explosion
  // means the ordering regressed.
  EXPECT_LE(subs.size(), 6u) << subs.size()
                             << " sub-paths — lobe chaining regressed (one split per column?)";

  std::size_t total_poses = 0;
  double total_len = 0.0, worst_turn = 0.0;
  std::size_t oob = 0, tight_run = 0;
  double min_clearance = std::numeric_limits<double>::max();
  for (const auto& path : subs)
  {
    ASSERT_GE(path.size(), 2u);
    total_poses += path.size();
    total_len += pathLength(path);
    // (1) NO near-180° REVERSAL cusp within a driven run. Forward turn-around
    // connectors remove every swath U-turn; what can remain are pivot-able ~90°
    // corners the rings inherit from a rectangular boundary. Bound the WORST
    // turn well below a reversal (120°).
    worst_turn = std::max(worst_turn, maxTurnDeg(path));
    // (2) Every point inside the planning boundary ring (safe_boundary) AND no
    // closer than (effective inset − one densify step) to the RAW operator
    // boundary. chassis_safety_inset is now the OUTERMOST-RING-CENTERLINE inset,
    // so generateHeadlandSwaths' built-in op_width/2 offset means the planning
    // field is shrunk by only (inset − op_width/2); the connectors/fillets ride
    // that much inside the raw boundary.
    for (const auto& p : path)
    {
      if (!pointInRing(p.first, p.second, connector_boundary))
      {
        ++oob;
      }
      min_clearance = std::min(min_clearance, distanceToRing(p.first, p.second, boundary));
    }
    // (3) NO sustained arc tighter than the robot's min turning radius (an
    // untrackable loop). A lone sharp ring corner is ONE over-curved step.
    tight_run = std::max(tight_run, maxTightArcRun(path, kStep, kMinTurnRadius));
  }
  ASSERT_GE(total_poses, 100u) << "continuous path is implausibly short";
  std::cout << "\n=== recorded_area_1 continuous-path analysis ===\n"
            << "rings=" << plan.rings.size() << "  swaths=" << plan.swaths.size()
            << "  sub-paths=" << subs.size() << "  turn_radius=" << kTurnRadius
            << "  step=" << kStep << "\n"
            << "points=" << total_poses << "  length=" << total_len << " m\n"
            << "max_local_turn=" << worst_turn << " deg" << "  out_of_bounds=" << oob << "/"
            << total_poses << "\n"
            << "min clearance to raw operator boundary=" << min_clearance << " m (inset " << kInset
            << ")\n"
            << "max_tight_arc_run=" << tight_run << " steps (floor " << kMinTurnRadius << " m)\n"
            << std::flush;

  EXPECT_LT(worst_turn, 120.0) << "a sub-path has a " << worst_turn
                               << "° turn — a near-reversal cusp MPPI will dither at";
  EXPECT_EQ(oob, 0u) << oob << "/" << total_poses
                     << " continuous-path points are outside the safety-inset ring";
  // Effective planning inset = chassis_safety_inset − op_width/2 (the outermost
  // ring centerline sits `kInset` inside the raw line; the field it is planned
  // in is shrunk by that much less). Connectors/fillets ride the planning
  // boundary, so they stay at least (kEffInset − one densify step) inside.
  constexpr double kEffInset = kInset - kOpWidth / 2.0;
  EXPECT_GE(min_clearance, kEffInset - kStep - 1e-6)
      << "a continuous-path pose sits " << min_clearance
      << " m from the raw operator boundary — closer than the effective planning inset ("
      << kEffInset << " m); a connector/fillet can push the blade past the planning boundary";
  EXPECT_LT(tight_run, 3u) << "found a run of " << tight_run
                           << " consecutive steps tighter than min_turning_radius ("
                           << kMinTurnRadius << " m) — an untrackable loop";
  // (4) Non-trivial, and starts at the first ring's first point.
  EXPECT_LT(total_poses, 200000u) << "path is implausibly large";
  EXPECT_NEAR(subs.front().front().first, plan.rings.front().front().first, 1e-9);
  EXPECT_NEAR(subs.front().front().second, plan.rings.front().front().second, 1e-9);
}

// ===========================================================================
// RING DEDUP: a doubled leading vertex (points[0] == points[1]) is the common
// OpenMower-export / hand-drawn-GUI-polygon defect. The zero-length edge makes
// the ring non-simple, and boost::geometry (under F2C) rejects it, silently
// dropping that area from coverage — which is exactly why the last working
// areas of an imported map failed to plan. dedupClosedRing is the server's
// last gate before a polygon becomes an f2c::types::Cell.
// ===========================================================================

// Count edges (i, i+1) of `ring` whose endpoints coincide. A clean closed F2C
// ring [A,B,C,A] has none (the only coincident pair, ring[0]==ring[n-1], is
// not consecutive).
std::size_t zeroLengthEdges(const f2c::types::LinearRing& ring)
{
  std::size_t z = 0;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i)
  {
    const auto a = ring.getGeometry(i);
    const auto b = ring.getGeometry(i + 1);
    if (std::fabs(a.getX() - b.getX()) < 1e-9 && std::fabs(a.getY() - b.getY()) < 1e-9)
    {
      ++z;
    }
  }
  return z;
}

TEST(RingDedup, DropsDoubledLeadingVertex)
{
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(0.0, 0.0));
  in.addPoint(f2c::types::Point(0.0, 0.0));  // the defect: doubled leading vertex
  in.addPoint(f2c::types::Point(4.0, 0.0));
  in.addPoint(f2c::types::Point(4.0, 3.0));
  in.addPoint(f2c::types::Point(0.0, 3.0));

  const auto out = dedupClosedRing(in);

  EXPECT_EQ(zeroLengthEdges(out), 0u) << "doubled leading vertex left a zero-length edge";
  // 4 distinct corners + explicit closing vertex.
  EXPECT_EQ(out.size(), 5u);
  EXPECT_NEAR(out.getGeometry(0).getX(), out.getGeometry(out.size() - 1).getX(), 1e-9);
  EXPECT_NEAR(out.getGeometry(0).getY(), out.getGeometry(out.size() - 1).getY(), 1e-9);
}

TEST(RingDedup, AlreadyCleanRingIsUnchanged)
{
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(0.0, 0.0));
  in.addPoint(f2c::types::Point(4.0, 0.0));
  in.addPoint(f2c::types::Point(4.0, 3.0));
  in.addPoint(f2c::types::Point(0.0, 3.0));
  in.addPoint(f2c::types::Point(0.0, 0.0));  // already closed

  const auto out = dedupClosedRing(in);

  EXPECT_EQ(zeroLengthEdges(out), 0u);
  EXPECT_EQ(out.size(), in.size()) << "a clean closed ring must be left intact";
}

// End-to-end: a square whose outer ring carries the doubled leading vertex must
// still produce a non-empty coverage plan once normalised — the regression that
// the importer + this server gate jointly fix.
TEST(RingDedup, DedupedDegenerateRingStillPlans)
{
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(0.0, 0.0));
  in.addPoint(f2c::types::Point(0.0, 0.0));  // the defect
  in.addPoint(f2c::types::Point(3.0, 0.0));
  in.addPoint(f2c::types::Point(3.0, 3.0));
  in.addPoint(f2c::types::Point(0.0, 3.0));
  f2c::types::Cell cell(dedupClosedRing(in));

  const auto plan = planDefault(cell);
  EXPECT_FALSE(plan.rings.empty() && plan.swaths.empty())
      << "deduped degenerate ring produced an empty plan";
}

// ---------------------------------------------------------------------------
// bufferRingOutward — drawn-obstacle margin (mowgli_robot.yaml.obstacle_margin)
// ---------------------------------------------------------------------------

TEST(ObstacleMargin, GrowsRingOutwardByMargin)
{
  // 1×1 m square obstacle centred at (2, 2).
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(1.5, 1.5));
  in.addPoint(f2c::types::Point(2.5, 1.5));
  in.addPoint(f2c::types::Point(2.5, 2.5));
  in.addPoint(f2c::types::Point(1.5, 2.5));

  const double margin = 0.3;
  const auto out = mowgli_coverage::bufferRingOutward(in, margin);

  ASSERT_GE(out.size(), 4u);
  // Every grown vertex sits at least `margin` from the ORIGINAL ring (rounded
  // joins mean edge midpoints sit exactly margin out, corners on the arc).
  std::vector<std::pair<double, double>> orig = {
      {1.5, 1.5}, {2.5, 1.5}, {2.5, 2.5}, {1.5, 2.5}, {1.5, 1.5}};
  for (std::size_t i = 0; i < out.size(); ++i)
  {
    const auto p = out.getGeometry(i);
    const double d = mowgli_coverage::distanceToRing(p.getX(), p.getY(), orig);
    EXPECT_GE(d, margin - 1e-6) << "grown vertex " << i << " at (" << p.getX() << ", " << p.getY()
                                << ") is only " << d << " m from the original obstacle ring";
    EXPECT_LE(d, margin + 1e-6) << "grown vertex " << i << " overshoots the requested margin";
  }
  // Closed ring (F2C requirement).
  EXPECT_NEAR(out.getGeometry(0).getX(), out.getGeometry(out.size() - 1).getX(), 1e-9);
  EXPECT_NEAR(out.getGeometry(0).getY(), out.getGeometry(out.size() - 1).getY(), 1e-9);
}

TEST(ObstacleMargin, ZeroMarginIsPassthrough)
{
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(0.0, 0.0));
  in.addPoint(f2c::types::Point(1.0, 0.0));
  in.addPoint(f2c::types::Point(1.0, 1.0));
  in.addPoint(f2c::types::Point(0.0, 1.0));

  const auto out = mowgli_coverage::bufferRingOutward(in, 0.0);
  // Same vertices, dedup-closed — identical to dedupClosedRing(in).
  const auto expected = dedupClosedRing(in);
  ASSERT_EQ(out.size(), expected.size());
  for (std::size_t i = 0; i < out.size(); ++i)
  {
    EXPECT_NEAR(out.getGeometry(i).getX(), expected.getGeometry(i).getX(), 1e-9);
    EXPECT_NEAR(out.getGeometry(i).getY(), expected.getGeometry(i).getY(), 1e-9);
  }
}

TEST(ObstacleMargin, DegenerateRingFallsBackToInput)
{
  // Two-point "ring" cannot be buffered — must fall back, never drop.
  f2c::types::LinearRing in;
  in.addPoint(f2c::types::Point(0.0, 0.0));
  in.addPoint(f2c::types::Point(1.0, 0.0));

  const auto out = mowgli_coverage::bufferRingOutward(in, 0.3);
  EXPECT_GE(out.size(), 2u) << "degenerate obstacle ring was dropped";
}

// End-to-end: with a margin, no planned ring point or swath sample may come
// closer than the margin to the DRAWN obstacle ring (the planner's chassis
// inset adds more on top; this pins the floor the margin itself guarantees).
TEST(ObstacleMargin, PlanKeepsMarginOffDrawnObstacle)
{
  f2c::types::Cell cell = makeSquare(8.0);
  f2c::types::LinearRing hole;
  hole.addPoint(f2c::types::Point(3.5, 3.5));
  hole.addPoint(f2c::types::Point(4.5, 3.5));
  hole.addPoint(f2c::types::Point(4.5, 4.5));
  hole.addPoint(f2c::types::Point(3.5, 4.5));
  const double margin = 0.3;
  cell.addRing(mowgli_coverage::bufferRingOutward(dedupClosedRing(hole), margin));

  const auto plan = planDefault(cell);
  ASSERT_FALSE(plan.swaths.empty()) << "field with grown hole produced no swaths";

  const std::vector<std::pair<double, double>> obstacle = {
      {3.5, 3.5}, {4.5, 3.5}, {4.5, 4.5}, {3.5, 4.5}, {3.5, 3.5}};
  auto check_point = [&](double x, double y, const char* what)
  {
    const double d = mowgli_coverage::distanceToRing(x, y, obstacle);
    EXPECT_GE(d, margin - 1e-3) << what << " point (" << x << ", " << y << ") is " << d
                                << " m from the drawn obstacle — margin " << margin << " violated";
  };
  for (const auto& ring : plan.rings)
  {
    for (const auto& p : ring)
    {
      check_point(p.first, p.second, "ring");
    }
  }
  // Swaths are straight — sample along each segment, not just endpoints.
  for (const auto& s : plan.swaths)
  {
    const double len = swathLen(s);
    const int n = std::max(1, static_cast<int>(len / 0.05));
    for (int i = 0; i <= n; ++i)
    {
      const double t = static_cast<double>(i) / n;
      check_point(s.first.first + t * (s.second.first - s.first.first),
                  s.first.second + t * (s.second.second - s.first.second),
                  "swath");
    }
  }
  // The grown hole must surface in safe_holes so the continuous-path
  // connectors inherit the margin too.
  EXPECT_FALSE(plan.safe_holes.empty()) << "grown hole missing from safe_holes";
}

// LARGE-FIELD AUTO-ANGLE FALLBACK: f2c::sg::BruteForce::generateBestSwaths
// sweeps 180 candidate angles and regenerates the full swath set at each, so on
// a big field it blows past the action/BT planning timeouts and the coverage
// action fails outright ("huge maps don't plan"). planBoustrophedon must instead
// fall back to the boundary's longest-edge angle above kAutoAngleMaxAreaM2,
// recording a diagnostics note, while still producing a full plan.
TEST(CoveragePlanning, LargeFieldUsesLongestEdgeAngleFallback)
{
  const auto plan = planDefault(makeSquare(30.0));  // 900 m² > 400 m² gate
  EXPECT_FALSE(plan.swaths.empty()) << "large field produced no swaths";
  // The always-on "timing:" note means notes is never empty and the auto-angle
  // note is not guaranteed to be first — search for it explicitly.
  const bool has_auto_angle = std::any_of(plan.diagnostics.notes.begin(),
                                          plan.diagnostics.notes.end(),
                                          [](const std::string& n)
                                          {
                                            return n.find("auto-angle") != std::string::npos;
                                          });
  EXPECT_TRUE(has_auto_angle) << "expected an auto-angle fallback note on a large field";
}

// A field under the threshold keeps the exhaustive best-angle search, so small
// lawns get the optimum swath orientation as before. planBoustrophedon always
// records a per-stage "timing:" diagnostics note (see coverage_planning.cpp), so
// the contract is specifically that NO "auto-angle" fallback note is present —
// not that notes is empty.
TEST(CoveragePlanning, SmallFieldKeepsExhaustiveAngleSearch)
{
  const auto plan = planDefault(makeSquare(18.0));  // 324 m² < 400 m² gate
  EXPECT_FALSE(plan.swaths.empty());
  for (const auto& note : plan.diagnostics.notes)
  {
    EXPECT_EQ(note.find("auto-angle"), std::string::npos)
        << "small field should not trigger the large-field angle fallback: " << note;
  }
}

// The fallback is deterministic (longest-edge angle of a fixed polygon), so the
// swath-index resume contract (CLAUDE.md invariant #7) still holds across
// re-plans of a large field.
TEST(CoveragePlanning, LargeFieldFallbackIsDeterministic)
{
  const auto a = planDefault(makeSquare(30.0));
  const auto b = planDefault(makeSquare(30.0));
  ASSERT_EQ(a.swaths.size(), b.swaths.size());
  EXPECT_NEAR(a.swath_angle_rad, b.swath_angle_rad, 1e-9);
}

// GEOMETRY SANITIZATION (field-reported partial coverage): an operator-recorded
// 252 m² boundary whose ring is geometrically DEGENERATE — a ~1.2 mm near-
// duplicate closure vertex plus several 1–10 mm sliver edges. The 68 vertices are
// verbatim from map (4).json → working_area[0].area.points. These points are
// EXACT enough that boost::geometry (F2C's backend) accepts them, yet degenerate
// enough that generateHeadlands / BruteForce silently produced PARTIAL coverage
// on a fresh plan. dedupClosedRing must (a) strip the mm degeneracies and (b)
// yield a cell the planner covers in full again.
TEST(CoveragePlanning, DegenerateRecordedRingSanitizedToFullCoverage)
{
  // NOT closed on purpose — the last vertex is ~1.2 mm from the first (the
  // recorded closure seam), and several consecutive vertices are 1–10 mm apart.
  const std::vector<std::pair<double, double>> pts = {
      {-0.1468, 1.8471},   {-6.5998, 23.0608},  {-6.6046, 23.0691},  {-6.6104, 23.0731},
      {-6.6178, 23.0743},  {-18.0979, 19.4606}, {-18.1731, 19.4307}, {-18.2238, 19.3894},
      {-18.2655, 19.3284}, {-18.3162, 19.2429}, {-18.3168, 19.1411}, {-17.1843, 16.0394},
      {-17.0688, 15.99},   {-16.9131, 15.9514}, {-16.8322, 15.9453}, {-16.7189, 15.952},
      {-16.6073, 15.9903}, {-16.4939, 16.0335}, {-16.3954, 16.0579}, {-16.2647, 16.0626},
      {-16.1277, 16.021},  {-16.0434, 15.9435}, {-15.9782, 15.8541}, {-15.9184, 15.7451},
      {-15.8716, 15.6532}, {-15.8313, 15.5422}, {-15.7997, 15.4433}, {-15.7889, 15.3582},
      {-15.8092, 15.2467}, {-15.8614, 15.1469}, {-15.9637, 15.0745}, {-16.0734, 15.0063},
      {-16.1273, 14.9349}, {-16.1976, 14.8194}, {-16.219, 14.7384},  {-16.237, 14.6354},
      {-16.2368, 14.5039}, {-16.2014, 14.3504}, {-16.0401, 13.7975}, {-11.8444, -0.8705},
      {-11.8026, -0.9601}, {-11.7139, -1.0434}, {-11.6045, -1.0228}, {-11.4932, -1.0051},
      {-11.3939, -1.024},  {-11.29, -1.0601},   {-11.1989, -1.1268}, {-11.1094, -1.1825},
      {-10.9994, -1.2351}, {-10.8877, -1.2878}, {-10.8178, -1.2811}, {-10.7607, -1.2553},
      {-10.7413, -1.2112}, {-10.14, -0.1519},   {-10.0367, -0.0724}, {-4.9679, 1.5848},
      {-4.8472, 1.6154},   {-4.7325, 1.6342},   {-2.335, 2.2454},    {-2.1199, 2.2914},
      {-1.9402, 2.2562},   {-1.8679, 2.1138},   {-1.7507, 1.9035},   {-1.6768, 1.7029},
      {-1.5999, 1.5751},   {-1.4961, 1.4756},   {-1.361, 1.4318},    {-0.148, 1.8469}};

  f2c::types::LinearRing raw;
  for (const auto& p : pts)
  {
    raw.addPoint(f2c::types::Point(p.first, p.second));
  }

  // Shortest cyclic edge, skipping the exact zero-length wrap of an already-closed
  // ring (first == last) so the metric works on both the OPEN raw ring — where the
  // wrap IS the 1.2 mm closure seam we want to catch — and the CLOSED clean ring.
  auto minCyclicEdge = [](const f2c::types::LinearRing& r)
  {
    double m = std::numeric_limits<double>::max();
    const std::size_t n = r.size();
    for (std::size_t i = 0; i < n; ++i)
    {
      const auto a = r.getGeometry(i);
      const auto b = r.getGeometry((i + 1) % n);
      const double d = std::hypot(b.getX() - a.getX(), b.getY() - a.getY());
      if (d > 1e-9)
      {
        m = std::min(m, d);
      }
    }
    return m;
  };

  // (a) The raw ring is degenerate: its shortest cyclic edge is the ~1.2 mm
  // closure seam — the very degeneracy boost accepts but F2C mis-clips. Guards
  // the regression: if the fixture ever loses its slivers the test is vacuous.
  EXPECT_LT(minCyclicEdge(raw), 0.002) << "fixture is not degenerate — regression untestable";

  const auto clean = dedupClosedRing(raw);
  // Sanitized ring is closed, has strictly fewer vertices (slivers removed), and
  // carries no sub-cm edge left for F2C to choke on.
  ASSERT_GE(clean.size(), 4u);
  EXPECT_NEAR(clean.getGeometry(0).getX(), clean.getGeometry(clean.size() - 1).getX(), 1e-9);
  EXPECT_NEAR(clean.getGeometry(0).getY(), clean.getGeometry(clean.size() - 1).getY(), 1e-9);
  EXPECT_LT(clean.size(), raw.size()) << "no degenerate vertices were removed";
  EXPECT_GT(minCyclicEdge(clean), 0.008) << "a mm-scale sliver survived sanitization";

  // (b) Planning the SANITIZED cell covers the whole field again. planned_fraction
  // is a coarse strip estimate, so a healthy full plan scores ~0.9+ while the
  // degenerate/partial plan scored far lower.
  const f2c::types::Cell cell(clean);
  const auto plan = planBoustrophedon(cell, 0.16, 0.18, 0, 0.08, -1.0, 0.15);
  ASSERT_FALSE(plan.rings.empty());
  ASSERT_GE(plan.swaths.size(), 20u);
  EXPECT_GT(plan.diagnostics.planned_fraction, 0.85)
      << "sanitized field still plans as PARTIAL coverage (" << plan.diagnostics.planned_fraction
      << ")";

  // Swaths must tile the field's full x-extent — partial coverage left a whole
  // sub-region unmowed, collapsing the swath x-span.
  double field_min_x = std::numeric_limits<double>::max();
  double field_max_x = std::numeric_limits<double>::lowest();
  for (std::size_t i = 0; i < clean.size(); ++i)
  {
    const double x = clean.getGeometry(i).getX();
    field_min_x = std::min(field_min_x, x);
    field_max_x = std::max(field_max_x, x);
  }
  double swath_min_x = std::numeric_limits<double>::max();
  double swath_max_x = std::numeric_limits<double>::lowest();
  for (const auto& s : plan.swaths)
  {
    swath_min_x = std::min({swath_min_x, s.first.first, s.second.first});
    swath_max_x = std::max({swath_max_x, s.first.first, s.second.first});
  }
  const double field_span = field_max_x - field_min_x;
  const double swath_span = swath_max_x - swath_min_x;
  EXPECT_GT(swath_span, 0.7 * field_span)
      << "swaths span only " << swath_span << " m of the field's " << field_span
      << " m x-extent — coverage is still partial";
}

// ── ConnectorStats (issue #499) ───────────────────────────────────────────────
//
// The connector-outcome counter is the prerequisite measurement for ever
// changing min_turning_radius / connector_turn_radius: raising the floor makes
// buildConnector's shrink search give up sooner and fall through to a straight
// blind connector, and nothing else in the plan or the logs distinguishes that
// from a healthy turn-around arc.
//
// WHAT THE COUNTER MEASURED, once it existed (2026-08-24, this PR): on the
// SHIPPED coverage geometry — num_headland_passes = 2, op_width = tool_width −
// swath_overlap, connector_turn_radius 0.18, min_turning_radius 0.15 — only
// 1 join in 32 gets a real turn-around arc. The other 31 are straight blind
// connectors. That is not a regression and not an artefact of the synthetic
// field; it is the baseline, and it is geometric (see the apron test below).
//
// These tests therefore pin the MECHANISM that decides an arc from a straight
// join — the width of the mowed headland apron beyond the swath ends, measured
// against the forward extent of the turn-around — rather than any one radius or
// any one fallback percentage. A radius change is expected to MOVE the raw
// counts, which is the entire point of having them.

// Shared plan geometry for the connector-outcome tests. Everything except the
// ring count and the radii is held at the SHIPPED configuration so the numbers
// these tests report are the numbers the robot actually plans with:
// mowgli_robot.yaml tool_width 0.18 − swath_overlap 0.02 = op_width 0.16,
// chassis_safety_inset 0.20, num_headland_passes 2.
namespace
{
constexpr double kStatsOpWidth = 0.16;
constexpr double kStatsHeadland = 0.18;
constexpr double kStatsInset = 0.20;
constexpr double kStatsMinSwath = 0.15;
constexpr double kStatsTurnRadius = 0.18;
constexpr double kStatsMinTurnRadius = 0.15;
constexpr double kStatsStep = 0.03;
constexpr int kShippedHeadlandPasses = 2;  // mowgli_robot.yaml num_headland_passes

// Plan a 6 m square with `rings` perimeter passes, then join it exactly the way
// coverage_server does — bounded by connector_clearance_boundary, not the looser
// safe_boundary — and return how every join resolved.
mowgli_coverage::ConnectorStats connectorOutcomes(
    int rings, double op_width, double headland, double turn_radius, double min_turn_radius)
{
  const f2c::types::Cell cell = makeSquare(6.0);
  const auto plan =
      planBoustrophedon(cell, op_width, headland, rings, kStatsInset, -1.0, kStatsMinSwath);
  EXPECT_FALSE(plan.rings.empty()) << "test geometry planned no perimeter rings";
  const auto& connector_boundary = plan.connector_clearance_boundary.size() >= 3
                                       ? plan.connector_clearance_boundary
                                       : plan.safe_boundary;
  mowgli_coverage::ConnectorStats stats;
  (void)buildContinuousSubPaths(
      plan, connector_boundary, turn_radius, min_turn_radius, kStatsStep, &stats);
  return stats;
}
}  // namespace

// The three outcomes partition every attempted join. If this ever fails, the
// fallback rate derived from them is meaningless.
TEST(CoverageConnectorStats, OutcomesPartitionAttemptedJoins)
{
  const auto stats = connectorOutcomes(
      kShippedHeadlandPasses, kStatsOpWidth, kStatsHeadland, kStatsTurnRadius, kStatsMinTurnRadius);

  EXPECT_GT(stats.attempted, 0u) << "a multi-segment plan must attempt connectors";
  EXPECT_EQ(stats.attempted, stats.arc + stats.straight_kept + stats.split)
      << "arc/straight_kept/split must partition attempted — the fallback rate "
         "derived from them is otherwise nonsense";
}

// Counter sanity on a case whose outcome is hand-checkable, so a stuck
// classifier cannot masquerade as a planner result. With the swath spacing at
// or above twice the turn radius (0.50 >= 2 * 0.18) the turn-around is a plain
// RSR U-turn: quarter circle, straight, quarter circle. Its forward extent past
// the swath end is exactly the turn radius, 0.18 m, and the single 0.50 m
// headland pass leaves 0.50 m of mowed apron there. An arc MUST fit at every
// join, so anything other than a clean sweep is a counting bug.
TEST(CoverageConnectorStats, EveryJoinIsAnArcWhenSpacingExceedsTwiceTheTurnRadius)
{
  constexpr double kWideSpacing = 0.50;  // >= 2 * kStatsTurnRadius
  const auto stats =
      connectorOutcomes(1, kWideSpacing, kWideSpacing, kStatsTurnRadius, kStatsMinTurnRadius);

  ASSERT_GT(stats.attempted, 5u) << "need several joins for this to mean anything";
  EXPECT_EQ(stats.arc, stats.attempted)
      << "only " << stats.arc << "/" << stats.attempted
      << " joins got an arc where a plain U-turn provably fits (spacing " << kWideSpacing
      << " m >= 2 x turn radius " << kStatsTurnRadius << " m, apron " << kWideSpacing
      << " m > extent " << kStatsTurnRadius
      << " m) — the classifier or the connector search is broken, not the geometry";
}

// THE #499 FINDING. What decides an arc from a straight blind connector at a
// swath end is the width of the mowed HEADLAND APRON beyond the swath ends, not
// the turn radius.
//
// When the swath spacing d is below 2R the turn-around cannot be a half circle;
// Dubins has to emit an omega (RLR/LRL) loop, whose forward extent past the
// swath end is roughly sqrt(4R^2 - (R + d/2)^2) + R — about 0.43 m at R = 0.18,
// d = 0.16. The apron available is num_headland_passes * op_width, so the
// shipped two passes give 0.32 m and the omega does not fit: buildConnector
// exhausts its shrink search and falls through to the straight connector. Widen
// the apron and the same search, at the same radii, succeeds almost everywhere.
//
// If this test starts failing on the narrow arm, swath-end joins have started
// getting real arcs — that is the desired outcome, not a regression: re-measure
// and update issue #499 rather than relaxing the bound.
TEST(CoverageConnectorStats, SwathEndArcsAreDecidedByTheHeadlandApronNotTheRadius)
{
  constexpr int kWideHeadlandPasses = 8;  // 8 * 0.16 = 1.28 m of apron

  const auto narrow = connectorOutcomes(
      kShippedHeadlandPasses, kStatsOpWidth, kStatsHeadland, kStatsTurnRadius, kStatsMinTurnRadius);
  const auto wide = connectorOutcomes(
      kWideHeadlandPasses, kStatsOpWidth, kStatsHeadland, kStatsTurnRadius, kStatsMinTurnRadius);

  ASSERT_GT(narrow.attempted, 10u);
  ASSERT_GT(wide.attempted, 10u);
  const double narrow_arc_share =
      static_cast<double>(narrow.arc) / static_cast<double>(narrow.attempted);
  const double wide_arc_share = static_cast<double>(wide.arc) / static_cast<double>(wide.attempted);

  // Shipped apron: the omega does not fit, so almost every join is straight.
  EXPECT_LT(narrow_arc_share, 0.2)
      << "shipped geometry (" << kShippedHeadlandPasses << " headland passes, " << kStatsOpWidth
      << " m apron/pass) now gets arcs at " << narrow.arc << "/" << narrow.attempted
      << " joins. Swath-end turns are no longer straight blind connectors — "
         "re-measure and update issue #499 instead of relaxing this bound";
  // Wide apron: the SAME search at the SAME radii fits an arc nearly everywhere,
  // which is what proves the radius is not the binding constraint.
  EXPECT_GT(wide_arc_share, 0.8) << "with a " << kWideHeadlandPasses * kStatsOpWidth
                                 << " m apron only " << wide.arc << "/" << wide.attempted
                                 << " joins got an arc — the connector search itself has regressed";

  // The straight fallbacks stay inside the connector-clearance envelope, so they
  // are driven blade-on: the shipped plan is one continuous sub-path, NOT a
  // fragmented one. This separates "no arc" (quality) from "no drivable
  // connector" (extra blade-off transits), which are very different costs.
  EXPECT_EQ(narrow.split, 0u)
      << narrow.split << " join(s) had no drivable connector at all on a hole-free convex field";
}

// The trade issue #499 feared — that raising min_turning_radius buys fewer
// carving arcs at the price of many more straight blind connectors — is not
// present at the shipped headland apron, because there are almost no arcs left
// to lose. Raising the floor from 0.15 m (below the 0.1625 m half-track) to
// 0.25 m, with the ceiling raised alongside it so the shrink search still has
// somewhere to start, moves at most one join and creates no sub-path split.
//
// This does not say what the radius SHOULD be — that decision is the
// maintainer's. It says the fallback rate is not the argument against changing
// it, which is the opposite of what #499 assumed.
TEST(CoverageConnectorStats, RaisingTheTurnRadiusFloorBarelyMovesTheFallbackRate)
{
  constexpr double kCeiling = 0.30;  // >= both floors, so the search starts identically
  constexpr double kRaisedFloor = 0.25;  // clears the 0.1625 m half-track

  const auto low = connectorOutcomes(
      kShippedHeadlandPasses, kStatsOpWidth, kStatsHeadland, kCeiling, kStatsMinTurnRadius);
  const auto high = connectorOutcomes(
      kShippedHeadlandPasses, kStatsOpWidth, kStatsHeadland, kCeiling, kRaisedFloor);

  ASSERT_EQ(low.attempted, high.attempted) << "the two arms must plan the same joins";
  ASSERT_GT(low.attempted, 10u);

  const std::size_t low_fallbacks = low.straight_kept + low.split;
  const std::size_t high_fallbacks = high.straight_kept + high.split;
  ASSERT_GE(high_fallbacks, low_fallbacks) << "a higher floor cannot fit MORE arcs";
  EXPECT_LE(high_fallbacks - low_fallbacks, 2u)
      << "raising min_turning_radius " << kStatsMinTurnRadius << " -> " << kRaisedFloor
      << " m turned " << (high_fallbacks - low_fallbacks) << " of " << low.attempted
      << " joins into straight fallbacks (" << low.arc << " -> " << high.arc
      << " arcs). The fallback trade #499 assumed is now real — re-open that "
         "trade-off before choosing a radius";
  EXPECT_EQ(high.split, 0u) << "a higher floor must not fragment a hole-free field into "
                            << high.split << " blade-off transit(s)";
}

// The counter is opt-in: every existing caller passes no stats pointer, so the
// nullptr path must stay safe and must not change the plan.
TEST(CoverageConnectorStats, NullStatsPointerIsSafeAndChangesNothing)
{
  const f2c::types::Cell cell = makeSquare(6.0);
  const auto plan = planBoustrophedon(cell,
                                      kStatsOpWidth,
                                      kStatsHeadland,
                                      kShippedHeadlandPasses,
                                      kStatsInset,
                                      -1.0,
                                      kStatsMinSwath);
  ASSERT_FALSE(plan.rings.empty());

  const auto without = buildContinuousSubPaths(
      plan, plan.safe_boundary, kStatsTurnRadius, kStatsMinTurnRadius, kStatsStep);
  mowgli_coverage::ConnectorStats stats;
  const auto with = buildContinuousSubPaths(
      plan, plan.safe_boundary, kStatsTurnRadius, kStatsMinTurnRadius, kStatsStep, &stats);

  ASSERT_EQ(without.size(), with.size()) << "collecting stats changed the sub-path split";
  for (std::size_t i = 0; i < without.size(); ++i)
  {
    EXPECT_EQ(without[i].size(), with[i].size())
        << "collecting stats changed sub-path " << i << "'s pose count";
  }
}
