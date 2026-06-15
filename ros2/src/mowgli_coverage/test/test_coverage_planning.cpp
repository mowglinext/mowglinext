// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Geometry tests for the boustrophedon coverage planner. These run under
// colcon test against the REAL Fields2Cover v3 library, so they exercise the
// actual ConstHL / BruteForce / BoustrophedonOrder output. The point is to
// catch a broken plan (empty / out-of-bounds / non-serpentine / hole-crossing)
// in CI instead of on the robot.

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
  constexpr double kTurnRadius = 0.30;
  constexpr double kStep = 0.03;

  const auto cell = makeRecordedArea1();
  const auto plan = planBoustrophedon(cell, kOpWidth, kHeadland, 0, kInset, -1.0, kMinSwath);
  ASSERT_FALSE(plan.rings.empty()) << "no headland rings on the real area";
  ASSERT_FALSE(plan.swaths.empty()) << "no swaths on the real area";

  const auto& boundary = recordedArea1Pts();
  const auto path = buildContinuousPath(plan, boundary, kTurnRadius, kStep);

  ASSERT_GE(path.size(), 100u) << "continuous path is implausibly short";

  // Diagnostics.
  const std::size_t inv = firstInversion(path);
  std::size_t oob = 0;
  for (const auto& p : path)
  {
    if (!pointInRing(p.first, p.second, boundary))
    {
      ++oob;
    }
  }
  std::cout << "\n=== recorded_area_1 continuous-path analysis ===\n"
            << "rings=" << plan.rings.size() << "  swaths=" << plan.swaths.size()
            << "  turn_radius=" << kTurnRadius << "  step=" << kStep << "\n"
            << "points=" << path.size() << "  length=" << pathLength(path) << " m\n"
            << "max_local_turn=" << maxTurnDeg(path) << " deg\n"
            << "first_inversion="
            << (inv < path.size() ? std::to_string(inv) + "/" + std::to_string(path.size())
                                  : std::string("none"))
            << "  out_of_bounds=" << oob << "/" << path.size() << "\n"
            << std::flush;

  // (1) NO >90° turn anywhere → no cusp → no MPPI bimodal dither.
  EXPECT_EQ(inv, path.size()) << "path inverts at index " << inv << " (cusp → MPPI dither)";
  // (2) Every point inside the recorded boundary.
  EXPECT_EQ(oob, 0u) << oob << "/" << path.size() << " continuous-path points are out of bounds";
  // (3) Non-trivial, and starts at the first ring's first point.
  EXPECT_LT(path.size(), 200000u) << "path is implausibly large";
  EXPECT_NEAR(path.front().first, plan.rings.front().front().first, 1e-9);
  EXPECT_NEAR(path.front().second, plan.rings.front().front().second, 1e-9);
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
