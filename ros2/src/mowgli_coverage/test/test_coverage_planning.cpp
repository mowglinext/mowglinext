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

  // 10 m square with two separated holes → several lobes, so the driver relocates
  // between them and the NN reorder is exercised (> 2 sub-paths).
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
  ASSERT_GE(subs.size(), 3u) << "two holes should split the plan into >=3 lobes to reorder";

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
  EXPECT_LE(subs.size(), 5u) << subs.size()
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

// num_headland_passes override forces the ring count.
TEST(CoveragePlanning, HeadlandPassOverride)
{
  const auto plan = planBoustrophedon(makeSquare(4.0),
                                      0.16,
                                      0.18,
                                      /*passes=*/3,
                                      0.08,
                                      -1.0,
                                      0.15);
  EXPECT_EQ(plan.rings.size(), 3u);
}

// SAFETY FLOOR (Bug C1): the coverage server floors the planning inset at
// robot_width/2 before calling planBoustrophedon, so a too-small configured
// chassis_safety_inset can't let a blade cross the boundary. The floor itself
// lives in coverage_server.cpp; here we assert the GEOMETRIC GUARANTEE it
// provides — given the floored inset, NO planned ring point or swath endpoint
// sits closer than robot_width/2 to the boundary. We mirror the server's floor
// expression (max(configured, robot_width/2)) so the planner is exercised with
// exactly the value the server would pass.
TEST(CoveragePlanning, InsetFloorKeepsBladesInsideHalfWidth)
{
  constexpr double kRobotWidth = 0.40;  // 0.40 m chassis (the field excursion case)
  constexpr double kOpWidth = 0.16;
  constexpr double kHeadland = 0.18;
  constexpr double kMinSwath = 0.15;
  // The deployed-drift value that caused the 0.32-0.39 m boundary excursion:
  // configured well below the chassis half-width.
  constexpr double kConfiguredInset = 0.0;
  // Server's floor: max(configured, robot_width/2).
  const double effective_inset = std::max(kConfiguredInset, kRobotWidth * 0.5);
  ASSERT_NEAR(effective_inset, 0.20, 1e-9) << "floor must be robot_width/2 = 0.20 m";

  const auto cell = makeSquare(6.0);  // big enough to still plan after a 0.20 m inset
  const auto plan =
      planBoustrophedon(cell, kOpWidth, kHeadland, 0, effective_inset, -1.0, kMinSwath);
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
    // (2) Every point inside the chassis-safety-inset ring AND no closer than
    // (inset − one densify step) to the RAW operator boundary, so the spinning
    // blade can never cross it even on the turn-around connectors/fillets.
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
  EXPECT_GE(min_clearance, kInset - kStep - 1e-6)
      << "a continuous-path pose sits " << min_clearance
      << " m from the raw operator boundary — closer than the chassis-safety inset (" << kInset
      << " m); a connector/fillet can push the blade across the boundary";
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
  ASSERT_FALSE(plan.diagnostics.notes.empty())
      << "expected an auto-angle fallback note on a large field";
  EXPECT_NE(plan.diagnostics.notes[0].find("auto-angle"), std::string::npos);
}

// A field under the threshold keeps the exhaustive best-angle search (no note),
// so small lawns get the optimum swath orientation as before.
TEST(CoveragePlanning, SmallFieldKeepsExhaustiveAngleSearch)
{
  const auto plan = planDefault(makeSquare(18.0));  // 324 m² < 400 m² gate
  EXPECT_FALSE(plan.swaths.empty());
  EXPECT_TRUE(plan.diagnostics.notes.empty())
      << "small field should not trigger the large-field angle fallback";
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
