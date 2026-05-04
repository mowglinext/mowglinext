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

// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for ComputeCoveragePath's polygon-hole filter helpers.
// These are pure geometry routines (shoelace area, ray-cast PIP,
// point-segment distance) plus the composite isHoleSafeForF2C predicate
// that decides which obstacles get sent to F2C as polygon holes vs.
// dropped onto the costmap-only keepout layer.

#include <gtest/gtest.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "mowgli_behavior/coverage_nodes.hpp"

using mowgli_behavior::ComputeCoveragePath;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Point32;

namespace
{
Polygon make_square(double cx, double cy, double half)
{
  Polygon p;
  Point32 a, b, c, d;
  a.x = cx - half; a.y = cy - half;
  b.x = cx + half; b.y = cy - half;
  c.x = cx + half; c.y = cy + half;
  d.x = cx - half; d.y = cy + half;
  p.points = {a, b, c, d};  // CCW, open ring (first != last)
  return p;
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// polygonArea
// ─────────────────────────────────────────────────────────────────────────────

TEST(ComputeCoveragePathArea, EmptyPolygonHasZeroArea)
{
  Polygon p;
  EXPECT_DOUBLE_EQ(0.0, ComputeCoveragePath::polygonArea(p));
}

TEST(ComputeCoveragePathArea, TwoPointPolygonHasZeroArea)
{
  Polygon p;
  Point32 a, b;
  a.x = 0; a.y = 0;
  b.x = 1; b.y = 0;
  p.points = {a, b};
  EXPECT_DOUBLE_EQ(0.0, ComputeCoveragePath::polygonArea(p));
}

TEST(ComputeCoveragePathArea, UnitSquareIsOne)
{
  EXPECT_NEAR(1.0, ComputeCoveragePath::polygonArea(make_square(0, 0, 0.5)), 1e-9);
}

TEST(ComputeCoveragePathArea, AreaInvariantUnderTranslation)
{
  EXPECT_NEAR(4.0, ComputeCoveragePath::polygonArea(make_square(100, -50, 1.0)), 1e-9);
}

TEST(ComputeCoveragePathArea, AreaIsAlwaysPositive)
{
  // CW square should still produce positive area (we use |signed_area|).
  Polygon cw;
  Point32 a, b, c, d;
  a.x = 0; a.y = 0;
  b.x = 0; b.y = 1;
  c.x = 1; c.y = 1;
  d.x = 1; d.y = 0;
  cw.points = {a, b, c, d};
  EXPECT_NEAR(1.0, ComputeCoveragePath::polygonArea(cw), 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// pointInPolygon
// ─────────────────────────────────────────────────────────────────────────────

TEST(ComputeCoveragePathPIP, EmptyPolygonRejectsAllPoints)
{
  Polygon p;
  EXPECT_FALSE(ComputeCoveragePath::pointInPolygon(0.0, 0.0, p));
}

TEST(ComputeCoveragePathPIP, CenterOfSquareIsInside)
{
  Polygon sq = make_square(0, 0, 5.0);
  EXPECT_TRUE(ComputeCoveragePath::pointInPolygon(0.0, 0.0, sq));
  EXPECT_TRUE(ComputeCoveragePath::pointInPolygon(2.0, -3.0, sq));
}

TEST(ComputeCoveragePathPIP, OutsideSquareIsOutside)
{
  Polygon sq = make_square(0, 0, 5.0);
  EXPECT_FALSE(ComputeCoveragePath::pointInPolygon(10.0, 0.0, sq));
  EXPECT_FALSE(ComputeCoveragePath::pointInPolygon(-6.0, 0.0, sq));
  EXPECT_FALSE(ComputeCoveragePath::pointInPolygon(0.0, 5.5, sq));
}

TEST(ComputeCoveragePathPIP, ConcavePolygon)
{
  // L-shape: (0,0)→(4,0)→(4,2)→(2,2)→(2,4)→(0,4)
  Polygon l;
  auto pt = [](double x, double y) {
    Point32 p; p.x = x; p.y = y; return p;
  };
  l.points = {pt(0, 0), pt(4, 0), pt(4, 2), pt(2, 2), pt(2, 4), pt(0, 4)};
  EXPECT_TRUE(ComputeCoveragePath::pointInPolygon(1.0, 1.0, l));   // inside left arm
  EXPECT_TRUE(ComputeCoveragePath::pointInPolygon(3.0, 1.0, l));   // inside bottom arm
  EXPECT_FALSE(ComputeCoveragePath::pointInPolygon(3.0, 3.0, l));  // notch
}

// ─────────────────────────────────────────────────────────────────────────────
// distanceToPolygonBoundary
// ─────────────────────────────────────────────────────────────────────────────

TEST(ComputeCoveragePathDist, CenterOfUnitSquareIsHalfFromBoundary)
{
  Polygon sq = make_square(0, 0, 0.5);
  EXPECT_NEAR(0.5, ComputeCoveragePath::distanceToPolygonBoundary(0, 0, sq), 1e-9);
}

TEST(ComputeCoveragePathDist, NearEdgeIsCloseToBoundary)
{
  Polygon sq = make_square(0, 0, 5.0);
  EXPECT_NEAR(0.1, ComputeCoveragePath::distanceToPolygonBoundary(4.9, 0, sq), 1e-9);
  EXPECT_NEAR(0.0, ComputeCoveragePath::distanceToPolygonBoundary(5.0, 0, sq), 1e-9);
}

TEST(ComputeCoveragePathDist, OutsidePointStillReturnsBoundaryDistance)
{
  Polygon sq = make_square(0, 0, 1.0);
  // Point 2 m outside the right edge — distance to the right edge segment = 1 m.
  EXPECT_NEAR(1.0, ComputeCoveragePath::distanceToPolygonBoundary(2.0, 0, sq), 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// isHoleSafeForF2C
// ─────────────────────────────────────────────────────────────────────────────

TEST(ComputeCoveragePathHole, RejectsTooFewPoints)
{
  Polygon field = make_square(0, 0, 10.0);
  Polygon obs;
  Point32 a, b;
  a.x = 0; a.y = 0;
  b.x = 1; b.y = 0;
  obs.points = {a, b};
  EXPECT_FALSE(ComputeCoveragePath::isHoleSafeForF2C(obs, field, 0.04, 0.7));
}

TEST(ComputeCoveragePathHole, RejectsTooSmall)
{
  Polygon field = make_square(0, 0, 10.0);
  // 5 cm × 5 cm = 0.0025 m² < 0.04 m² floor.
  Polygon tiny = make_square(0, 0, 0.025);
  EXPECT_FALSE(ComputeCoveragePath::isHoleSafeForF2C(tiny, field, 0.04, 0.7));
}

TEST(ComputeCoveragePathHole, RejectsHoleVertexOutsideField)
{
  Polygon field = make_square(0, 0, 5.0);
  // Hole centered at (4.6, 0) with half=0.5: rightmost x = 5.1 (outside).
  Polygon obs = make_square(4.6, 0, 0.5);
  EXPECT_FALSE(ComputeCoveragePath::isHoleSafeForF2C(obs, field, 0.04, 0.0));
}

TEST(ComputeCoveragePathHole, RejectsHoleTooCloseToBoundary)
{
  Polygon field = make_square(0, 0, 5.0);
  // Hole inside but rightmost vertex is 0.4 m from boundary (clearance 0.7 fails).
  Polygon obs = make_square(4.4, 0, 0.2);
  EXPECT_FALSE(ComputeCoveragePath::isHoleSafeForF2C(obs, field, 0.04, 0.7));
}

TEST(ComputeCoveragePathHole, AcceptsHoleWellInsideField)
{
  Polygon field = make_square(0, 0, 5.0);
  Polygon obs = make_square(0, 0, 1.0);
  EXPECT_TRUE(ComputeCoveragePath::isHoleSafeForF2C(obs, field, 0.04, 0.7));
}

TEST(ComputeCoveragePathHole, ZeroClearanceAcceptsTouchingHole)
{
  // Sanity: clearance=0 is the "any hole strictly inside" mode.
  // Hole vertex on the boundary should still fail because we require
  // pointInPolygon, which is false for boundary points (PIP is open).
  Polygon field = make_square(0, 0, 5.0);
  Polygon obs = make_square(0, 0, 5.0);  // identical to field: vertices on boundary
  EXPECT_FALSE(ComputeCoveragePath::isHoleSafeForF2C(obs, field, 0.04, 0.0));
}
