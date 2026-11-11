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
// A turn-fallback / contour-bypass splice is only adopted when the PIVOT state
// could execute it: SpliceAuthorityViolation applies the mowing-authority,
// sweep and departure checks the rotation gate will apply.

#include <cmath>
#include <vector>

#include "mowgli_nav2_plugins/ftc_corner_geometry.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include <gtest/gtest.h>

namespace
{

namespace mnp = mowgli_nav2_plugins;
using mnp::PlanPose2D;

constexpr double kDepartureCap = 2.5;  // avoidance_horizon_m

mnp::transit::Ring Body()
{
  return {{0.53, 0.275}, {0.53, -0.275}, {-0.17, -0.275}, {-0.17, 0.275}};
}

mnp::CornerGeometry Lawn()
{
  mnp::CornerGeometry g;
  g.valid = true;
  g.mowing.allowed = {{{-5, -5}, {15, -5}, {15, 15}, {-5, 15}}};
  return g;
}

/// East along y = 0 to (2, 0), pivot to north, then 1 m north.
std::vector<PlanPose2D> SplicedPivotAt(double x, double y)
{
  std::vector<PlanPose2D> p;
  for (double t = 0.0; t <= x + 1e-9; t += 0.05)
  {
    p.push_back({t, y, 0.0});
  }
  p.push_back({x, y, M_PI / 2});
  for (double t = 0.05; t <= 1.0 + 1e-9; t += 0.05)
  {
    p.push_back({x, y + t, M_PI / 2});
  }
  return p;
}

TEST(SpliceAuthority, APivotInsideTheLawnIsExecutable)
{
  const auto p = SplicedPivotAt(2.0, 2.0);
  EXPECT_EQ(mnp::SpliceAuthorityViolation(Lawn(), p, p.size() - 1, Body(), kDepartureCap), "");
}

TEST(SpliceAuthority, NoGeometryMeansThePivotCanNeverRun)
{
  const auto p = SplicedPivotAt(2.0, 2.0);
  EXPECT_NE(
      mnp::SpliceAuthorityViolation(mnp::CornerGeometry{}, p, p.size() - 1, Body(), kDepartureCap),
      "");
}

TEST(SpliceAuthority, APivotOutsideTheMowingAreaIsRefused)
{
  // The fallback planners see lethal cells and the global band only; this
  // pivot sits past the recorded line, where the rotation gate never rotates.
  mnp::CornerGeometry g = Lawn();
  g.mowing.allowed = {{{-5, -5}, {1.5, -5}, {1.5, 15}, {-5, 15}}};
  const auto p = SplicedPivotAt(2.0, 2.0);
  EXPECT_NE(mnp::SpliceAuthorityViolation(g, p, p.size() - 1, Body(), kDepartureCap), "");
}

TEST(SpliceAuthority, APivotSweepingADrawnObstacleIsRefused)
{
  // Drawn obstacles 0.45 m from the axle, north-east AND south-west: the body
  // sweeps one on the short arc (east -> north) and the other on the long one,
  // and neither is under the chassis at the start or the end of the turn. The
  // LiDAR may not see a drawn obstacle at all.
  mnp::CornerGeometry g = Lawn();
  const auto square = [](double cx, double cy)
  {
    return mnp::transit::Ring{{cx - 0.03, cy - 0.03},
                              {cx + 0.03, cy - 0.03},
                              {cx + 0.03, cy + 0.03},
                              {cx - 0.03, cy + 0.03}};
  };
  g.mowing.blocked = {square(2.32, 2.32), square(1.68, 1.68)};
  g.mowing.blocked_margins = {0.0, 0.0};
  const auto p = SplicedPivotAt(2.0, 2.0);
  EXPECT_NE(mnp::SpliceAuthorityViolation(g, p, p.size() - 1, Body(), kDepartureCap), "");
}

TEST(SpliceAuthority, ADepartureLeavingTheMowingAreaIsRefused)
{
  mnp::CornerGeometry g = Lawn();
  g.mowing.allowed = {{{-5, -5}, {15, -5}, {15, 2.5}, {-5, 2.5}}};
  const auto p = SplicedPivotAt(2.0, 2.0);  // drives 1 m north, past y = 2.5
  EXPECT_NE(mnp::SpliceAuthorityViolation(g, p, p.size() - 1, Body(), kDepartureCap), "");
}

TEST(SpliceAuthority, CornersPastTheRejoinAreTheirOwnBusiness)
{
  mnp::CornerGeometry g = Lawn();
  g.mowing.allowed = {{{-5, -5}, {1.5, -5}, {1.5, 15}, {-5, 15}}};
  const auto p = SplicedPivotAt(2.0, 2.0);
  // Up to a pose before the corner nothing of the splice is outside.
  EXPECT_EQ(mnp::SpliceAuthorityViolation(g, p, 10, Body(), kDepartureCap), "");
}

}  // namespace
