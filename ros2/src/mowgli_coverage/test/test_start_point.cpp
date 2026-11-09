// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// The operator-chosen start point: where a headland ring closes, the guarantee that
// the route starts on the OUTERMOST ring, and the pinned first sub-path. The pure
// closure choice is checked on synthetic loops; the rest against the real planner
// (Fields2Cover v3).

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_coverage::BoustrophedonPlan;
using mowgli_coverage::buildContinuousSubPaths;
using mowgli_coverage::pickRingClosure;
using mowgli_coverage::planBoustrophedon;
using mowgli_coverage::RingClosure;

using Point = std::pair<double, double>;
using Ring = std::vector<Point>;
using Hint = std::optional<Point>;

double dist(const Point& a, const Point& b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

// Shoelace area: > 0 counter-clockwise.
double signedArea(const Ring& ring)
{
  double twice = 0.0;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i)
    twice += ring[i].first * ring[i + 1].second - ring[i + 1].first * ring[i].second;
  return 0.5 * twice;
}

f2c::types::LinearRing makeRing(const Ring& pts)
{
  f2c::types::LinearRing ring;
  for (const Point& p : pts)
    ring.addPoint(f2c::types::Point(p.first, p.second));
  ring.addPoint(f2c::types::Point(pts.front().first, pts.front().second));
  return ring;
}

f2c::types::Cell square(double size)
{
  return f2c::types::Cell(makeRing({{0, 0}, {size, 0}, {size, size}, {0, size}}));
}

// A size x size square with a square hole [h0, h1] x [h0, h1].
f2c::types::Cell squareWithHole(double size, double h0, double h1)
{
  f2c::types::Cell cell = square(size);
  cell.addRing(makeRing({{h0, h0}, {h0, h1}, {h1, h1}, {h1, h0}}));
  return cell;
}

BoustrophedonPlan plan(const f2c::types::Cell& cell,
                       const Hint& hint,
                       int ring_direction = 0,
                       int passes = 0)
{
  return planBoustrophedon(
      cell, 0.18, 0.30, passes, 0.0, 0.0, 0.15, ring_direction, 0.20, false, 0, hint);
}

// A 10 x 4 rectangle's corners, counter-clockwise.
const Ring kRect = {{0, 0}, {10, 0}, {10, 4}, {0, 4}};

}  // namespace

// ---- pickRingClosure (pure) -------------------------------------------------

TEST(PickRingClosure, WithoutAHintItIsTheMidpointOfTheLongestSide)
{
  const RingClosure c = pickRingClosure(kRect, std::nullopt, 0.2);
  EXPECT_EQ(c.edge, 0u);  // (0,0)-(10,0) is the first of the two longest sides
  EXPECT_DOUBLE_EQ(c.point.first, 5.0);
  EXPECT_DOUBLE_EQ(c.point.second, 0.0);
}

TEST(PickRingClosure, ALongestSideTieGoesToTheFirstOne)
{
  const Ring square4 = {{0, 0}, {4, 0}, {4, 4}, {0, 4}};
  EXPECT_EQ(pickRingClosure(square4, std::nullopt, 0.2).edge, 0u);
}

TEST(PickRingClosure, AHintPicksTheNearestSide)
{
  // Near the right-hand short side (10,0)-(10,4), not the longest.
  const RingClosure c = pickRingClosure(kRect, Hint{{11.0, 1.7}}, 0.2);
  EXPECT_EQ(c.edge, 1u);
  EXPECT_DOUBLE_EQ(c.point.first, 10.0);
  EXPECT_NEAR(c.point.second, 1.7, 1e-9);
}

TEST(PickRingClosure, TheClosureLandsOnThePointNearestTheHint)
{
  const RingClosure c = pickRingClosure(kRect, Hint{{3.3, -2.0}}, 0.2);
  EXPECT_EQ(c.edge, 0u);
  EXPECT_NEAR(c.point.first, 3.3, 1e-9);
  EXPECT_DOUBLE_EQ(c.point.second, 0.0);
}

TEST(PickRingClosure, ItNeverLandsOnACorner)
{
  // A hint right at the corner (10, 0) is pulled back onto the straight part.
  for (const Point& h : {Point{10.0, 0.0}, Point{10.4, -0.4}, Point{0.0, 0.0}, Point{9.9, 0.1}})
  {
    const RingClosure c = pickRingClosure(kRect, Hint{h}, 0.2);
    for (const Point& corner : kRect)
      EXPECT_GE(dist(c.point, corner), 0.5 - 1e-9) << "hint " << h.first << "," << h.second;
  }
}

TEST(PickRingClosure, TheClearanceGrowsWithTheTurningRadius)
{
  // 2.5 * 0.4 = 1.0 m off the corner.
  const RingClosure c = pickRingClosure(kRect, Hint{{10.0, 0.0}}, 0.4);
  for (const Point& corner : kRect)
    EXPECT_GE(dist(c.point, corner), 1.0 - 1e-9);
}

TEST(PickRingClosure, ASideTooShortToBeARealSideIsSkippedForALongOne)
{
  // The 0.8 m end of a thin strip is not a place to close a ring on: the nearest REAL side
  // (the 10 m one) takes it, kept clear of the corner.
  const Ring thin = {{0, 0}, {10, 0}, {10, 0.8}, {0, 0.8}};
  const RingClosure c = pickRingClosure(thin, Hint{{10.2, 0.1}}, 0.2);
  EXPECT_EQ(c.edge, 0u);
  EXPECT_NEAR(c.point.first, 9.5, 1e-9);
  EXPECT_DOUBLE_EQ(c.point.second, 0.0);
}

TEST(PickRingClosure, ASideOfAtLeastAMetreIsRealButTooShortToLeaveRoomClosesAtItsMidpoint)
{
  const Ring pad = {{0, 0}, {1.0, 0}, {1.0, 1.0}, {0, 1.0}};  // every side exactly 1 m
  const RingClosure c = pickRingClosure(pad, Hint{{1.2, 0.2}}, 0.2);
  EXPECT_EQ(c.edge, 1u);
  EXPECT_DOUBLE_EQ(c.point.first, 1.0);
  EXPECT_NEAR(c.point.second, 0.5, 1e-9);
}

TEST(PickRingClosure, ARoundedCornerIsNotASideToCloseOn)
{
  // F2C's outermost ring has rounded corners: tiny sides round each corner. A hint right at
  // the corner must still close on a long side, well clear of the corner.
  const Ring rounded = {{0.1, 0.0},
                        {19.9, 0.0},
                        {20.0, 0.1},
                        {20.0, 19.9},
                        {19.9, 20.0},
                        {0.1, 20.0},
                        {0.0, 19.9},
                        {0.0, 0.1}};
  const RingClosure c = pickRingClosure(rounded, Hint{{20.0, 20.0}}, 0.2);
  EXPECT_TRUE(c.edge == 2u || c.edge == 4u) << "closed on side " << c.edge;
  EXPECT_GE(dist(c.point, {20.0, 20.0}), 0.5);
  for (const Point& v : rounded)
    EXPECT_GE(dist(c.point, v), 0.4) << "the closure must be clear of every corner vertex";
}

TEST(PickRingClosure, ASmoothCurveOfShortSidesStillClosesNearTheHint)
{
  // A hand-drawn curve made of 0.3 m sides has no real straight side: every side
  // qualifies, and the closure lands next to the hint.
  Ring circle;
  for (int i = 0; i < 100; ++i)
  {
    const double a = 2.0 * M_PI * i / 100.0;
    circle.emplace_back(5.0 * std::cos(a), 5.0 * std::sin(a));
  }
  const RingClosure c = pickRingClosure(circle, Hint{{6.0, 0.0}}, 0.2);
  EXPECT_LT(dist(c.point, {5.0, 0.0}), 0.4);
}

TEST(PickRingClosure, AHintEquidistantFromTwoSidesTakesTheLowerIndex)
{
  // (-5,-5) is nearest the corner (0,0): equally near side 3 (0,4)-(0,0) end and side 0 start.
  const RingClosure a = pickRingClosure(kRect, Hint{{-5.0, -5.0}}, 0.2);
  const RingClosure b = pickRingClosure(kRect, Hint{{-5.0, -5.0}}, 0.2);
  EXPECT_EQ(a.edge, b.edge);
  EXPECT_EQ(a.edge, 0u);
}

TEST(PickRingClosure, IsDeterministicAndHandlesDegenerateLoops)
{
  EXPECT_EQ(pickRingClosure({}, Hint{{1, 1}}, 0.2).edge, 0u);
  const RingClosure one = pickRingClosure({{2.0, 3.0}}, Hint{{9, 9}}, 0.2);
  EXPECT_DOUBLE_EQ(one.point.first, 2.0);
  const RingClosure c1 = pickRingClosure(kRect, Hint{{4.2, 7.0}}, 0.2);
  const RingClosure c2 = pickRingClosure(kRect, Hint{{4.2, 7.0}}, 0.2);
  EXPECT_EQ(c1.edge, c2.edge);
  EXPECT_EQ(c1.point, c2.point);
}

// ---- the real planner ---------------------------------------------------------

TEST(StartPoint, TheRouteStartsOnTheSideNearestTheHint)
{
  // 20 m square; a hint on the east side at y = 6.
  const auto p = plan(square(20.0), Hint{{21.0, 6.0}});
  ASSERT_FALSE(p.rings.empty());
  const Point start = p.rings[0].front();
  EXPECT_GT(start.first, 19.5);  // on the east side of the outer ring
  EXPECT_NEAR(start.second, 6.0, 0.01);
  EXPECT_EQ(p.rings[0].front(), p.rings[0].back()) << "the ring still closes where it starts";
}

TEST(StartPoint, EveryRingClosesOnTheSameSideSoTheJunctionsStack)
{
  const auto p = plan(square(20.0), Hint{{21.0, 6.0}});
  ASSERT_GE(p.rings.size(), 2u);
  for (const Ring& ring : p.rings)
    EXPECT_NEAR(ring.front().second, 6.0, 0.01);
}

TEST(StartPoint, MovingTheHintMovesTheStart)
{
  const auto a = plan(square(20.0), Hint{{21.0, 6.0}});
  const auto b = plan(square(20.0), Hint{{9.0, -1.0}});  // the south side
  EXPECT_GT(a.rings[0].front().first, 19.5);
  EXPECT_LT(b.rings[0].front().second, 0.5);
  EXPECT_NEAR(b.rings[0].front().first, 9.0, 0.01);
}

TEST(StartPoint, AHintDeepInsideTheFieldStillStartsOnTheOuterRing)
{
  // The start is always on the outermost ring: an interior point snaps to the nearest side.
  const auto p = plan(square(20.0), Hint{{10.0, 2.0}});
  ASSERT_FALSE(p.rings.empty());
  EXPECT_LT(p.rings[0].front().second, 0.5);
  EXPECT_NEAR(p.rings[0].front().first, 10.0, 0.01);
}

TEST(StartPoint, NeverStartsOnACornerEvenWhenTheHintIsOne)
{
  const auto p = plan(square(20.0), Hint{{20.0, 20.0}});
  ASSERT_FALSE(p.rings.empty());
  const Point start = p.rings[0].front();
  for (const Point& corner : Ring{{0, 0}, {20, 0}, {20, 20}, {0, 20}})
    EXPECT_GT(dist(start, corner), 0.4);
}

TEST(StartPoint, TheWindingIsUntouched)
{
  for (const int direction : {1, 2})
  {
    const auto p = plan(square(20.0), Hint{{21.0, 6.0}}, direction);
    ASSERT_FALSE(p.rings.empty());
    for (const Ring& ring : p.rings)
    {
      if (direction == 1)
        EXPECT_LT(signedArea(ring), 0.0);
      else
        EXPECT_GT(signedArea(ring), 0.0);
    }
  }
}

TEST(StartPoint, WithoutAHintThePlanIsExactlyWhatItWas)
{
  const auto with_default = planBoustrophedon(square(20.0), 0.18, 0.30, 0, 0.0, 0.0, 0.15);
  const auto with_nullopt = plan(square(20.0), std::nullopt);
  EXPECT_EQ(with_default.rings, with_nullopt.rings);
  EXPECT_EQ(with_default.swaths, with_nullopt.swaths);
  // ...and it starts at the midpoint of the longest side (the square: the first side).
  const Point start = with_nullopt.rings[0].front();
  EXPECT_NEAR(std::min(std::abs(start.first - 10.0), std::abs(start.second - 10.0)), 0.0, 0.3);
}

TEST(StartPoint, WithTheRingsOffTheHintDoesNothing)
{
  const auto without = plan(square(20.0), std::nullopt, 0, /*passes=*/-1);
  const auto with = plan(square(20.0), Hint{{21.0, 6.0}}, 0, /*passes=*/-1);
  EXPECT_TRUE(with.rings.empty());
  EXPECT_EQ(without.swaths, with.swaths);
}

TEST(StartPoint, AFieldWithAHoleStillStartsOnThePerimeterNotTheObstacleRing)
{
  // A hint right beside the hole: the route must still start on the OUTER ring.
  const f2c::types::Cell cell = squareWithHole(20.0, 8.0, 12.0);
  const auto p = plan(cell, Hint{{9.0, 7.0}});
  ASSERT_GE(p.rings.size(), 2u);
  // rings[0] is the perimeter: it reaches the field edge, a ring round the hole never does.
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  for (const Point& pt : p.rings[0])
  {
    min_x = std::min(min_x, pt.first);
    max_x = std::max(max_x, pt.first);
  }
  EXPECT_LT(min_x, 1.0);
  EXPECT_GT(max_x, 19.0);
}

TEST(StartPoint, ThePinnedFirstSubPathBeginsAtTheStart)
{
  const f2c::types::Cell cell = squareWithHole(20.0, 8.0, 12.0);
  const Hint hint{{21.0, 6.0}};
  const auto p = plan(cell, hint);
  ASSERT_FALSE(p.rings.empty());
  const Point start = p.rings[0].front();

  const auto subs = buildContinuousSubPaths(p,
                                            p.connector_clearance_boundary,
                                            0.20,
                                            0.20,
                                            0.03,
                                            nullptr,
                                            {},
                                            {},
                                            /*pin_first_subpath=*/true);
  ASSERT_FALSE(subs.empty());
  EXPECT_LT(dist(subs.front().front(), start), 0.25) << "the route must start at the chosen point";
}
