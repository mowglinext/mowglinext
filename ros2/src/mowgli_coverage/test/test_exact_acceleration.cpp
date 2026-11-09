// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// The two planner accelerations must not change a single bit of the plan:
// RingIndex::contains must equal pointInRing, and leastCostNearestFirst must make
// leastCostInScanOrder's choice. Both are checked here against their reference.
// test/bench/ab_compare.py checks whole plans against the previous implementation.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include <gtest/gtest.h>

namespace
{
using mowgli_coverage::leastCostInScanOrder;
using mowgli_coverage::leastCostNearestFirst;
using mowgli_coverage::pointInRing;
using mowgli_coverage::RingIndex;
using mowgli_coverage::ScanCandidate;
using Ring = std::vector<std::pair<double, double>>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Every query point that sits on a decision boundary of either implementation:
// each vertex, its y with nearby x, edge midpoints, band-ish y levels across the
// whole extent, the extent's ends, plus random points and non-finite values.
std::vector<std::pair<double, double>> queries(const Ring& ring, std::mt19937_64& rng)
{
  std::vector<std::pair<double, double>> q;
  double x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf;
  for (const auto& [x, y] : ring)
  {
    if (!std::isfinite(x) || !std::isfinite(y))
      continue;
    x0 = std::min(x0, x);
    x1 = std::max(x1, x);
    y0 = std::min(y0, y);
    y1 = std::max(y1, y);
  }
  if (!(x0 <= x1))
    x0 = y0 = -1.0, x1 = y1 = 1.0;
  const double w = std::max(x1 - x0, 1e-12), h = std::max(y1 - y0, 1e-12);
  for (std::size_t i = 0; i < ring.size(); ++i)
  {
    const auto& [x, y] = ring[i];
    const auto& [nx, ny] = ring[(i + 1) % ring.size()];
    q.emplace_back(x, y);
    q.emplace_back(std::nextafter(x, -kInf), y);
    q.emplace_back(std::nextafter(x, kInf), y);
    q.emplace_back(x, std::nextafter(y, kInf));
    q.emplace_back(x, std::nextafter(y, -kInf));
    q.emplace_back(0.5 * (x + nx), 0.5 * (y + ny));
    q.emplace_back(x0 + 0.5 * w, y);
  }
  // Band boundaries: RingIndex starts with one band per vertex, so y_min + k·h/n
  // (and its neighbouring doubles) sit on, or next to, the edges of its bands.
  if (std::isfinite(h))
  {
    const std::size_t n = std::max<std::size_t>(ring.size(), 1);
    for (std::size_t k = 0; k <= n; ++k)
    {
      const double y = y0 + h * static_cast<double>(k) / static_cast<double>(n);
      const double x = x0 + w * ((k * 0.618034) - std::floor(k * 0.618034));
      for (double v : {std::nextafter(y, -kInf), y, std::nextafter(y, kInf)})
        q.emplace_back(x, v);
    }
  }
  // Random points need a representable range (uniform_real_distribution's
  // precondition); a ring spanning +-1e308 gets the explicit values below instead.
  if (std::isfinite(w) && std::isfinite(h))
  {
    std::uniform_real_distribution<double> ux(x0 - 0.1 * w, x1 + 0.1 * w);
    std::uniform_real_distribution<double> uy(y0 - 0.1 * h, y1 + 0.1 * h);
    for (int k = 0; k < 20000; ++k)
      q.emplace_back(ux(rng), uy(rng));
  }
  // Explicit extremes and midpoints (0.5 * a + 0.5 * b cannot overflow).
  const double max = std::numeric_limits<double>::max();
  const double tiny = std::numeric_limits<double>::denorm_min();
  for (double vx : {x0, x1, 0.5 * x0 + 0.5 * x1, 0.0, -tiny, tiny, -max, max, kNaN, kInf, -kInf})
  {
    for (double vy : {y0,
                      y1,
                      0.5 * y0 + 0.5 * y1,
                      0.0,
                      -tiny,
                      tiny,
                      -max,
                      max,
                      kNaN,
                      kInf,
                      -kInf,
                      std::nextafter(y0, kInf),
                      std::nextafter(y1, -kInf)})
      q.emplace_back(vx, vy);
  }
  return q;
}

// `has_inside`: the ring has interior the queries must hit (not a degenerate one).
void expectSameAsPointInRing(const Ring& ring,
                             std::uint64_t seed,
                             bool has_inside = true,
                             std::size_t max_queries = std::numeric_limits<std::size_t>::max())
{
  std::mt19937_64 rng(seed);
  const RingIndex index(ring);
  std::size_t inside = 0;
  auto points = queries(ring, rng);
  if (points.size() > max_queries)
  {
    std::shuffle(points.begin(), points.end(), rng);
    points.resize(max_queries);
  }
  for (const auto& [x, y] : points)
  {
    const bool expected = pointInRing(x, y, ring);
    ASSERT_EQ(index.contains(x, y), expected) << "x=" << x << " y=" << y;
    inside += expected ? 1 : 0;
  }
  if (has_inside)
  {
    EXPECT_GT(inside, 0u);
  }
}

Ring star(std::size_t n, double r_in, double r_out, double cx, double cy, std::mt19937_64& rng)
{
  std::uniform_real_distribution<double> u(r_in, r_out);
  Ring ring;
  for (std::size_t i = 0; i < n; ++i)
  {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    const double r = u(rng);
    ring.emplace_back(cx + r * std::cos(a), cy + r * std::sin(a));
  }
  return ring;
}

TEST(RingIndex, RandomConcaveRingsMatchPointInRing)
{
  std::mt19937_64 rng(42);
  for (std::size_t n : {3u, 4u, 7u, 40u, 400u, 3000u})
    expectSameAsPointInRing(star(n, 2.0, 25.0, 13.7, -8.2, rng), n);
}

TEST(RingIndex, SpikyRingWhoseEdgesSpanManyBandsMatches)
{
  // Long edges across the whole extent force the band count down.
  std::mt19937_64 rng(7);
  expectSameAsPointInRing(star(500, 0.01, 30.0, 0.0, 0.0, rng), 7);
}

TEST(RingIndex, RingWithMoreVerticesThanTheBandCapMatches)
{
  // 70 000 vertices: more than the 65 536-band cap, so bands hold several vertices.
  std::mt19937_64 rng(13);
  expectSameAsPointInRing(star(70000, 20.0, 21.0, -3.0, 4.0, rng), 13, true, 30000);
}

TEST(RingIndex, AxisAlignedClosedAndDegenerateRingsMatch)
{
  // Horizontal and vertical edges, an explicit closing vertex, a doubled vertex,
  // collinear runs: the shapes the densified rings and rectangles really have.
  expectSameAsPointInRing({{0, 0}, {6, 0}, {6, 6}, {0, 6}}, 1);
  expectSameAsPointInRing({{0, 0}, {6, 0}, {6, 6}, {0, 6}, {0, 0}}, 2);
  expectSameAsPointInRing({{0, 0}, {0, 0}, {3, 0}, {6, 0}, {6, 3}, {6, 6}, {3, 6}, {0, 6}}, 3);
  expectSameAsPointInRing({{0, 0},
                           {4, 0},
                           {4, 2.7},
                           {7, 2.7},
                           {7, 0},
                           {11, 0},
                           {11, 6},
                           {7, 6},
                           {7, 3.3},
                           {4, 3.3},
                           {4, 6},
                           {0, 6}},
                          4);
  // Map-frame float32 values far from the datum, and a sliver 1e-9 m tall.
  Ring far;
  for (const auto& [x, y] : Ring{{0, 0}, {30, 0.5}, {29, 20}, {14, 9}, {1, 21}})
    far.emplace_back(static_cast<float>(4.0e5 + x), static_cast<float>(-6.2e5 + y));
  expectSameAsPointInRing(far, 5);
  expectSameAsPointInRing({{0, 0}, {10, 1e-9}, {20, 0}, {10, 2e-9}}, 6);
}

TEST(RingIndex, RingsTheBandArithmeticCannotRepresentFallBackToTheScan)
{
  expectSameAsPointInRing({{0, -1e308}, {1, 1e308}, {-1, 1e308}}, 1);  // extent overflows
  expectSameAsPointInRing({{0, 0}, {1, 5e-324}, {2, 0}, {1, 1e-323}}, 2, false);  // subnormal
  expectSameAsPointInRing({{0, 0}, {kNaN, 3}, {4, 4}, {5, 0}}, 3, false);  // non-finite vertex
  expectSameAsPointInRing({{0, 0}, {1, 1}}, 4, false);  // fewer than three vertices
  expectSameAsPointInRing({{0, 1}, {2, 1}, {4, 1}}, 5, false);  // flat
}

// ── leastCostNearestFirst vs leastCostInScanOrder ───────────────────────────

struct Choice
{
  std::optional<std::size_t> order;
  std::size_t evaluated = 0;
};

Choice choose(bool nearest_first,
              const std::vector<ScanCandidate>& candidates,
              const std::vector<double>& costs)  // by order
{
  Choice c;
  const auto cost = [&](const ScanCandidate& s)
  {
    ++c.evaluated;
    return costs[s.order];
  };
  c.order = nearest_first ? leastCostNearestFirst(candidates, cost)
                          : leastCostInScanOrder(candidates, cost);
  return c;
}

TEST(LeastCost, NearestFirstMakesTheScanOrderChoiceWithFewerEvaluations)
{
  // Costs >= gaps, the property every join cost has (straight line, or a polyline
  // between the same ends, or the straight line plus a turn penalty). Coarse
  // values make exact ties in both gap and cost common.
  std::mt19937_64 rng(3);
  std::size_t scan_evaluations = 0, nearest_evaluations = 0;
  for (int trial = 0; trial < 20000; ++trial)
  {
    const std::size_t n = 1 + rng() % 60;
    std::vector<ScanCandidate> candidates;
    std::vector<double> costs(2 * n + 1, kNaN);
    for (std::size_t i = 0; i < n; ++i)
    {
      const std::size_t order = 2 * i + rng() % 2;  // gaps in the order, like unused swaths
      const double gap = static_cast<double>(rng() % 40) * 0.25;
      const int kind = static_cast<int>(rng() % 4);
      costs[order] = kind == 0   ? gap
                     : kind == 1 ? gap + static_cast<double>(rng() % 8) * 0.25
                                 : gap + static_cast<double>(rng() % 1000) * 1e-3;  // arc length
      candidates.push_back({gap, order});
    }
    const Choice scan = choose(false, candidates, costs);
    const Choice nearest = choose(true, candidates, costs);
    ASSERT_EQ(nearest.order, scan.order) << "trial " << trial;
    scan_evaluations += scan.evaluated;
    nearest_evaluations += nearest.evaluated;
  }
  EXPECT_LT(nearest_evaluations, scan_evaluations);
}

TEST(LeastCost, RoundingBelowTheGapWithinTheSlackKeepsTheChoice)
{
  // A summed polyline can come out a few ulps under the straight distance.
  std::mt19937_64 rng(11);
  for (int trial = 0; trial < 20000; ++trial)
  {
    const std::size_t n = 2 + rng() % 30;
    std::vector<ScanCandidate> candidates;
    std::vector<double> costs(n, kNaN);
    for (std::size_t i = 0; i < n; ++i)
    {
      const double gap = 1.0 + static_cast<double>(rng() % 6) * 0.5;
      const double ulps = static_cast<double>(rng() % 9) - 4.0;
      costs[i] = std::max(0.0,
                          gap + ulps * std::ldexp(gap, -52) +
                              (rng() % 3 == 0 ? static_cast<double>(rng() % 4) * 0.5 : 0.0));
      candidates.push_back({gap, i});
    }
    ASSERT_EQ(choose(true, candidates, costs).order, choose(false, candidates, costs).order)
        << "trial " << trial;
  }
}

TEST(LeastCost, AChosenCostBelowItsGapRunsTheScanOrderFallback)
{
  // Equal gaps: the scan evaluates order 0 (cost 1.0) and then SKIPS order 1, its
  // gap not being below 1.0. Nearest-first evaluates both and would take order 1
  // (cost 0.5 < its gap 1.0, impossible for a real join). The guard sees the
  // chosen cost under its gap and returns the scan's choice instead.
  const std::vector<ScanCandidate> candidates{{1.0, 0}, {1.0, 1}};
  const std::vector<double> costs{1.0, 0.5};
  const Choice scan = choose(false, candidates, costs);
  const Choice nearest = choose(true, candidates, costs);
  ASSERT_EQ(scan.order, std::optional<std::size_t>(0));
  EXPECT_EQ(nearest.order, scan.order);
  // 2 nearest-first evaluations, then the scan's own 1: proof the fallback ran.
  EXPECT_EQ(nearest.evaluated, 3u);

  // The fallback is also taken when the cheaper candidate comes first in the scan.
  const std::vector<ScanCandidate> later{{2.0, 0}, {1.0, 1}};
  const std::vector<double> later_costs{1.5, 0.9};
  EXPECT_EQ(choose(true, later, later_costs).order, choose(false, later, later_costs).order);
  EXPECT_EQ(choose(true, later, later_costs).evaluated, 1u + 2u);
}

TEST(LeastCost, NonFiniteInputsFallBackToTheScan)
{
  const std::vector<ScanCandidate> nan_gap{{kNaN, 0}, {1.0, 1}, {0.5, 2}};
  const std::vector<double> costs{3.0, 1.0, 0.7};
  EXPECT_EQ(choose(true, nan_gap, costs).order, choose(false, nan_gap, costs).order);
  const std::vector<ScanCandidate> nan_cost{{1.0, 0}, {2.0, 1}};
  EXPECT_EQ(choose(true, nan_cost, {kNaN, kNaN}).order, std::nullopt);
  EXPECT_EQ(choose(false, nan_cost, {kNaN, kNaN}).order, std::nullopt);
  EXPECT_EQ(choose(true, {}, {}).order, std::nullopt);
}

TEST(LeastCost, AdversarialBoundariesAroundTheBestAndTheStopKeepTheChoice)
{
  // Gaps and costs one ulp either side of the best cost and of the stop threshold
  // best * (1 + 1e-6), equal costs behind different gaps, and costs that are other
  // candidates' gaps: every comparison the two choices make is exercised on its edge.
  std::mt19937_64 rng(17);
  std::size_t fallbacks_possible = 0;
  for (int trial = 0; trial < 50000; ++trial)
  {
    const double m = 0.25 * static_cast<double>(1 + rng() % 8);
    const double stop = m * (1.0 + 1e-6);
    const double values[] = {m,
                             std::nextafter(m, 0.0),
                             std::nextafter(m, kInf),
                             stop,
                             std::nextafter(stop, 0.0),
                             std::nextafter(stop, kInf),
                             0.5 * m,
                             2.0 * m,
                             0.0};
    const std::size_t n = 2 + rng() % 12;
    std::vector<ScanCandidate> candidates;
    std::vector<double> costs(n);
    for (std::size_t i = 0; i < n; ++i)
    {
      const double gap = values[rng() % std::size(values)];
      double cost = std::max(gap, values[rng() % std::size(values)]);
      if (rng() % 5 == 0)
      {
        cost = std::nextafter(gap, 0.0);  // a summed polyline a ulp under the gap
        ++fallbacks_possible;
      }
      costs[i] = cost;
      candidates.push_back({gap, i});
    }
    ASSERT_EQ(choose(true, candidates, costs).order, choose(false, candidates, costs).order)
        << "trial " << trial;
  }
  EXPECT_GT(fallbacks_possible, 0u);
}

TEST(LeastCost, EqualCostsBehindDifferentGapsKeepTheScanOrderFirst)
{
  // Order 1 comes first in the scan; order 3 is nearer. Both cost 1.0.
  const std::vector<ScanCandidate> candidates{{0.9, 1}, {0.5, 3}};
  const std::vector<double> costs{0.0, 1.0, 0.0, 1.0};
  EXPECT_EQ(choose(true, candidates, costs).order, std::optional<std::size_t>(1));
  EXPECT_EQ(choose(false, candidates, costs).order, std::optional<std::size_t>(1));
}

TEST(LeastCost, CoincidentZeroGapEntriesStillPrune)
{
  // Swath ends that coincide with the robot's position and a zero-cost join: the
  // first is taken and the stop (gap >= best · (1 + slack)) holds at best == 0.
  std::vector<ScanCandidate> candidates;
  for (std::size_t i = 0; i < 50; ++i)
    candidates.push_back({0.0, i});
  const std::vector<double> costs(50, 0.0);
  const Choice scan = choose(false, candidates, costs);
  const Choice nearest = choose(true, candidates, costs);
  EXPECT_EQ(nearest.order, std::optional<std::size_t>(0));
  EXPECT_EQ(nearest.order, scan.order);
  EXPECT_LE(nearest.evaluated, scan.evaluated);
}

TEST(LeastCost, ACostBelowItsGapOnAPrunedCandidateIsTheCallersContract)
{
  // Why buildContinuousSubPaths must keep every join cost >= its gap: here the
  // FARTHER candidate (scan order 0) costs less than its own gap. The scan takes it;
  // nearest-first stops before evaluating it, and the guard (which sees only the
  // candidate it chose) cannot tell. Pinned so the precondition stays visible.
  const std::vector<ScanCandidate> candidates{{2.0, 0}, {1.0, 1}};
  const std::vector<double> costs{0.5, 1.0};
  EXPECT_EQ(choose(false, candidates, costs).order, std::optional<std::size_t>(0));
  EXPECT_EQ(choose(true, candidates, costs).order, std::optional<std::size_t>(1));
}

}  // namespace
