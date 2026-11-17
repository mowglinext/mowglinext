// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// AreaRingIndex must answer exactly what the plain even-odd walk the
// LiDAR-ignore corridor filter used per beam answered — for every ring shape
// and every query, including points exactly on vertex rows, band boundaries
// and the ring's extent.

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_localization/area_ring_index.hpp"

namespace
{
using mowgli_localization::AreaRingIndex;
using mowgli_localization::Point2D;

// Frozen copy of costmap_scan_filter_node's point_in_polygon before the index,
// so an edit to the shared header cannot move the reference too.
bool ReferencePointInPolygon(double px, double py, const std::vector<Point2D>& ring)
{
  if (ring.size() < 3)
    return false;
  bool inside = false;
  for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
  {
    const Point2D& pi = ring[i];
    const Point2D& pj = ring[j];
    if ((pi.y > py) == (pj.y > py))
      continue;
    const double x_at_py = pi.x + (py - pi.y) * (pj.x - pi.x) / (pj.y - pi.y);
    if (px < x_at_py)
      inside = !inside;
  }
  return inside;
}

// Frozen copy of the node's point_in_any_area before the index.
bool ReferencePointInAnyArea(double px, double py, const std::vector<std::vector<Point2D>>& areas)
{
  for (const auto& ring : areas)
  {
    if (ReferencePointInPolygon(px, py, ring))
      return true;
  }
  return false;
}

// Queries on and around every vertex coordinate plus random ones.
std::vector<Point2D> Queries(const std::vector<Point2D>& ring, std::mt19937& rng, int random)
{
  std::vector<Point2D> q;
  const auto on_rows = [&q](const Point2D& a, const Point2D& b)
  {
    q.push_back({a.x, b.y});
    q.push_back({std::nextafter(a.x, -1e9), std::nextafter(b.y, 1e9)});
    q.push_back({std::nextafter(a.x, 1e9), std::nextafter(b.y, -1e9)});
  };
  if (ring.size() <= 60)
  {
    for (const auto& a : ring)
      for (const auto& b : ring)
        on_rows(a, b);
  }
  else if (!ring.empty())
  {
    // Every vertex row, with x taken from a few other vertices.
    std::uniform_int_distribution<std::size_t> pick(0, ring.size() - 1);
    for (const auto& b : ring)
      for (int k = 0; k < 4; ++k)
        on_rows(ring[pick(rng)], b);
  }
  double lo = -1.0, hi = 1.0;
  for (const auto& p : ring)
  {
    if (std::isfinite(p.x) && std::isfinite(p.y))
    {
      lo = std::min({lo, p.x, p.y});
      hi = std::max({hi, p.x, p.y});
    }
  }
  std::uniform_real_distribution<double> u(lo - 1.0, hi + 1.0);
  for (int i = 0; i < random; ++i)
    q.push_back({u(rng), u(rng)});
  // Rows on and a few ulp around the band boundaries (same sizing as the index).
  double y_min = std::numeric_limits<double>::infinity(), y_max = -y_min;
  for (const auto& p : ring)
  {
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }
  if (ring.size() >= 3 && std::isfinite(y_max - y_min) && y_max > y_min)
  {
    const double bands = std::ceil(2.0 * std::sqrt(static_cast<double>(ring.size())));
    const double per_m = bands / (y_max - y_min);
    for (double k = 0; k <= bands; ++k)
      for (double y : {y_min + k / per_m, y_min + k * ((y_max - y_min) / bands)})
      {
        for (int step = 0; step < 3; ++step)
        {
          y = std::nextafter(y, -std::numeric_limits<double>::infinity());
        }
        for (int step = 0; step < 7; ++step)
        {
          q.push_back({u(rng), y});
          y = std::nextafter(y, std::numeric_limits<double>::infinity());
        }
      }
  }
  return q;
}

void ExpectSameAsReference(const std::vector<Point2D>& ring, std::mt19937& rng, int random = 2000)
{
  const AreaRingIndex index(ring);
  for (const auto& p : Queries(ring, rng, random))
    ASSERT_EQ(index.contains(p.x, p.y), ReferencePointInPolygon(p.x, p.y, ring))
        << "query (" << p.x << ", " << p.y << ") ring size " << ring.size();
}

}  // namespace

TEST(AreaRingIndex, MatchesPointInRingOnRandomRings)
{
  std::mt19937 rng(20261109);
  std::uniform_int_distribution<int> size(3, 200);
  std::uniform_real_distribution<double> coord(-30.0, 30.0);
  for (int r = 0; r < 300; ++r)
  {
    // Arbitrary vertex soup: self-intersecting, repeated and collinear points.
    std::vector<Point2D> ring(static_cast<std::size_t>(size(rng)));
    for (auto& p : ring)
      p = {coord(rng), coord(rng)};
    ExpectSameAsReference(ring, rng, 300);
  }
}

TEST(AreaRingIndex, MatchesPointInRingOnGridSnappedRings)
{
  // Vertices on a 0.5 m grid: many horizontal edges, shared vertex rows and
  // queries exactly on them.
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> size(3, 60);
  std::uniform_int_distribution<int> cell(-10, 10);
  for (int r = 0; r < 300; ++r)
  {
    std::vector<Point2D> ring(static_cast<std::size_t>(size(rng)));
    for (auto& p : ring)
      p = {0.5 * cell(rng), 0.5 * cell(rng)};
    ExpectSameAsReference(ring, rng, 200);
  }
}

TEST(AreaRingIndex, MatchesPointInRingWithVerticesOnBandBoundaries)
{
  // Vertex rows exactly on (and a few ulp around) the index's band
  // boundaries: the band an edge is filed under and the band a query on that
  // row looks in come from the same arithmetic, or the edge is missed.
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> coord(-20.0, 20.0);
  std::uniform_int_distribution<int> ulps(-3, 3);
  for (const double extent : {7.3, 1e-3 * M_PI, 123.456, 1e-250, 3e250})
    for (const std::size_t n : {16u, 37u, 200u})
      for (int r = 0; r < 20; ++r)
      {
        const double bands = std::ceil(2.0 * std::sqrt(static_cast<double>(n)));
        std::uniform_int_distribution<int> k(0, static_cast<int>(bands));
        std::vector<Point2D> ring(n);
        for (auto& p : ring)
        {
          double y = (r % 2) ? k(rng) / (bands / extent) : k(rng) * (extent / bands);
          for (int u = ulps(rng); u != 0; u += (u > 0 ? -1 : 1))
            y = std::nextafter(y, u > 0 ? 1e300 : -1e300);
          p = {coord(rng), std::clamp(y, 0.0, extent)};
        }
        ring[0].y = 0.0;
        ring[1].y = extent;
        ExpectSameAsReference(ring, rng, 100);
      }
}

TEST(AreaRingIndex, MatchesPointInRingOnARecordedLawnBoundary)
{
  // A dense, wobbly recorded perimeter like a driven hedge line, with an
  // explicit closing vertex.
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> wobble(-0.15, 0.15);
  std::vector<Point2D> ring;
  for (int i = 0; i < 1500; ++i)
  {
    const double a = 2.0 * M_PI * i / 1500;
    const double c = std::cos(a), s = std::sin(a);
    ring.push_back({17.8 * std::copysign(std::pow(std::abs(c), 0.3), c) + wobble(rng),
                    13.8 * std::copysign(std::pow(std::abs(s), 0.3), s) + wobble(rng)});
  }
  ring.push_back(ring.front());
  ExpectSameAsReference(ring, rng, 20000);
}

TEST(AreaRingIndex, MatchesPointInRingOnZigzagsOffsetsAndFloatCoordinates)
{
  std::mt19937 rng(11);
  // Every edge spans the whole y-extent: the index falls back to the walk.
  std::vector<Point2D> zigzag;
  for (int i = 0; i < 400; ++i)
    zigzag.push_back({0.05 * i, (i % 2) ? 10.0 : -10.0});
  ExpectSameAsReference(zigzag, rng, 2000);

  // A small lawn far from the origin, and coordinates rounded through float
  // as the geometry_msgs/Polygon message delivers them.
  std::uniform_real_distribution<double> wobble(-0.2, 0.2);
  for (const double offset : {1e6, -1e6, 1e15})
  {
    std::vector<Point2D> ring;
    for (int i = 0; i < 120; ++i)
    {
      const double a = 2.0 * M_PI * i / 120;
      ring.push_back(
          {offset + 5.0 * std::cos(a) + wobble(rng), offset + 3.0 * std::sin(a) + wobble(rng)});
    }
    ExpectSameAsReference(ring, rng, 2000);
  }
  std::vector<Point2D> as_float;
  for (int i = 0; i < 300; ++i)
  {
    const double a = 2.0 * M_PI * i / 300;
    as_float.push_back({static_cast<float>(17.8 * std::cos(a) + wobble(rng)),
                        static_cast<float>(13.8 * std::sin(a) + wobble(rng))});
  }
  ExpectSameAsReference(as_float, rng, 5000);
}

TEST(AreaRingIndex, HorizontalEdgesAndPointsOnEdgesAndVertices)
{
  // Explicit horizontal edges (top, bottom and a step) and queries exactly on
  // edges, on vertices and on edge midpoints / interpolated points.
  std::mt19937 rng(23);
  const std::vector<Point2D> stairs{{0, 0}, {6, 0}, {6, 2}, {4, 2}, {4, 4}, {2, 4}, {2, 6}, {0, 6}};
  ExpectSameAsReference(stairs, rng, 2000);
  const AreaRingIndex index(stairs);
  for (std::size_t i = 0, j = stairs.size() - 1; i < stairs.size(); j = i++)
    for (double t = 0.0; t <= 1.0; t += 0.125)
    {
      const double x = stairs[j].x + t * (stairs[i].x - stairs[j].x);
      const double y = stairs[j].y + t * (stairs[i].y - stairs[j].y);
      ASSERT_EQ(index.contains(x, y), ReferencePointInPolygon(x, y, stairs)) << x << ", " << y;
    }
  // A ring that is mostly horizontal runs, and a slanted one queried on its
  // edges' own arithmetic points.
  const std::vector<Point2D> comb{{0, 0},
                                  {10, 0},
                                  {10, 1},
                                  {8, 1},
                                  {8, 0.5},
                                  {6, 0.5},
                                  {6, 1},
                                  {4, 1},
                                  {4, 0.5},
                                  {2, 0.5},
                                  {2, 1},
                                  {0, 1}};
  ExpectSameAsReference(comb, rng, 2000);
  const std::vector<Point2D> slant{{0.1, 0.3}, {7.7, 1.9}, {3.3, 8.1}};
  const AreaRingIndex slanted(slant);
  for (std::size_t i = 0, j = slant.size() - 1; i < slant.size(); j = i++)
    for (double t = 0.0; t <= 1.0; t += 1.0 / 64)
    {
      const double x = slant[j].x + t * (slant[i].x - slant[j].x);
      const double y = slant[j].y + t * (slant[i].y - slant[j].y);
      ASSERT_EQ(slanted.contains(x, y), ReferencePointInPolygon(x, y, slant)) << x << ", " << y;
    }
}

TEST(AreaRingIndex, AnyRingMatchesTheVectorWalkIncludingAnEmptyList)
{
  // The corridor filter's area-side test. An EMPTY list (none received yet,
  // or none recorded) contains nothing, exactly as before the index; the
  // filter's far-side rule then applies to every beam. This pins current
  // behaviour; it does not endorse it.
  std::mt19937 rng(29);
  std::uniform_real_distribution<double> u(-40.0, 40.0);
  const std::vector<std::vector<Point2D>> none;
  const std::vector<std::vector<Point2D>> some{{{0, 0}, {4, 0}, {4, 4}, {0, 4}},
                                               {{1, 1}},
                                               {{10, 10}, {20, 10}, {15, 18}},
                                               {}};
  for (const auto* areas : {&none, &some})
  {
    std::vector<AreaRingIndex> indexed;
    for (const auto& ring : *areas)
      indexed.emplace_back(ring);
    for (int i = 0; i < 20000; ++i)
    {
      const double x = u(rng), y = u(rng);
      ASSERT_EQ(mowgli_localization::point_in_any_ring(x, y, indexed),
                ReferencePointInAnyArea(x, y, *areas));
    }
  }
  EXPECT_FALSE(mowgli_localization::point_in_any_ring(2.0, 2.0, {}));
}

TEST(AreaRingIndex, DegenerateAndNonFiniteRings)
{
  std::mt19937 rng(1);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ExpectSameAsReference({}, rng, 50);
  ExpectSameAsReference({{0, 0}, {1, 1}}, rng, 50);
  ExpectSameAsReference({{0, 1}, {2, 1}, {5, 1}}, rng, 50);  // flat
  ExpectSameAsReference({{0, 0}, {0, 0}, {0, 0}}, rng, 50);
  ExpectSameAsReference({{nan, 0}, {4, 0}, {4, 4}, {0, 4}}, rng, 200);
  ExpectSameAsReference({{0, nan}, {4, 0}, {4, 4}, {0, 4}}, rng, 200);
  ExpectSameAsReference({{0, 0}, {4, 0}, {4, inf}, {0, 4}}, rng, 200);
  ExpectSameAsReference({{0, -1e300}, {4, 0}, {4, 1e300}, {0, 4}}, rng, 200);
  ExpectSameAsReference({{0, 0}, {1e-310, 0}, {0, 1e-310}}, rng, 200);  // subnormal extent

  const AreaRingIndex square({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
  EXPECT_TRUE(square.contains(2.0, 2.0));
  EXPECT_FALSE(square.contains(nan, 2.0));
  EXPECT_FALSE(square.contains(2.0, nan));
  EXPECT_FALSE(square.contains(2.0, 4.0));  // y_max row, as point_in_ring
  EXPECT_TRUE(square.contains(2.0, 0.0));  // y_min row, as point_in_ring
}
