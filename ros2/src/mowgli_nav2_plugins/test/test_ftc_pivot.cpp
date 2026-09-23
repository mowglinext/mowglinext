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
// FTC in-place pivots at the coverage planner's explicit corners
// (ftc_pivot.hpp). The central property: ONLY an explicit corner — two poses at
// the same position — can trigger a pivot; ordinary curvature never does.

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "mowgli_interfaces/coverage_geometry.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include <gtest/gtest.h>

namespace mowgli_nav2_plugins
{
namespace
{

namespace cg = mowgli_interfaces::coverage_geometry;
using Leg = std::pair<std::size_t, std::size_t>;

// Densified straight from (x0, y0) along `yaw` for `length` metres at `step`,
// yaws as coverage_server stamps them (next-step heading).
void appendStraight(
    std::vector<PlanPose2D>& out, double x0, double y0, double yaw, double length, double step)
{
  const int n = static_cast<int>(std::lround(length / step));
  for (int i = 0; i <= n; ++i)
  {
    out.push_back({x0 + std::cos(yaw) * step * i, y0 + std::sin(yaw) * step * i, yaw});
  }
}

// A row-end U-turn as the planner emits it: north up x = 0, pivot corner,
// 0.13 m east, pivot corner, south down x = 0.13.
std::vector<PlanPose2D> uTurnWithPivots()
{
  std::vector<PlanPose2D> p;
  appendStraight(p, 0.0, 0.0, M_PI / 2, 1.0, 0.03);
  p.back().yaw = M_PI / 2;  // incoming twin
  p.push_back({p.back().x, p.back().y, 0.0});  // outgoing twin
  const double cx = p.back().x;
  const double cy = p.back().y;
  p.push_back({cx + 0.065, cy, 0.0});
  p.push_back({cx + 0.13, cy, 0.0});  // incoming twin of corner 2
  p.push_back({cx + 0.13, cy, -M_PI / 2});  // outgoing twin
  appendStraight(p, cx + 0.13, cy - 0.03, -M_PI / 2, 0.9, 0.03);
  return p;
}

// A forward turn-around arc of radius r (the pre-pivot connector shape): a
// half circle sampled every 0.03 m, heading tangent — the tightest ordinary
// curvature a coverage plan contains.
std::vector<PlanPose2D> turnAroundArc(double r)
{
  std::vector<PlanPose2D> p;
  appendStraight(p, 0.0, 0.0, M_PI / 2, 1.0, 0.03);
  const double cx = r;
  const double cy = p.back().y;
  const int n = static_cast<int>(std::ceil(M_PI * r / 0.03));
  for (int i = 1; i <= n; ++i)
  {
    const double a = M_PI - M_PI * i / n;  // from the left end over the top
    p.push_back({cx + r * std::cos(a), cy + r * std::sin(a), a - M_PI / 2});
  }
  appendStraight(p, 2 * r, cy - 0.03, -M_PI / 2, 1.0, 0.03);
  return p;
}

TEST(FtcPivot, FindsExactlyThePlannersCorners)
{
  const auto plan = uTurnWithPivots();
  const auto corners = FindPivotCorners(plan);
  ASSERT_EQ(corners.size(), 2u);
  EXPECT_NEAR(plan[corners[0]].yaw, M_PI / 2, 1e-12);
  EXPECT_NEAR(plan[corners[0] + 1].yaw, 0.0, 1e-12);
  EXPECT_NEAR(plan[corners[1]].yaw, 0.0, 1e-12);
  EXPECT_NEAR(plan[corners[1] + 1].yaw, -M_PI / 2, 1e-12);
}

TEST(FtcPivot, OrdinaryCurvatureNeverTriggersAPivot)
{
  // Turn-around arcs down to the tightest trackable radius, a filleted ring
  // corner, and a plain straight: consecutive poses are always a real step
  // apart, whatever the heading change between them.
  for (const double r : {0.10, 0.15, 0.20, 0.30})
  {
    EXPECT_TRUE(FindPivotCorners(turnAroundArc(r)).empty()) << "r = " << r;
  }
  std::vector<PlanPose2D> straight;
  appendStraight(straight, 0.0, 0.0, 0.3, 2.0, 0.03);
  EXPECT_TRUE(FindPivotCorners(straight).empty());
  // A zero-radius corner WITHOUT the twin (what a straight fallback used to
  // look like) is not a pivot either — it has no zero-length step.
  std::vector<PlanPose2D> sharp;
  appendStraight(sharp, 0.0, 0.0, 0.0, 1.0, 0.03);
  appendStraight(sharp, 1.0, 0.03, M_PI / 2, 1.0, 0.03);
  EXPECT_TRUE(FindPivotCorners(sharp).empty());
}

TEST(FtcPivot, DuplicateWithoutATurnIsNotACorner)
{
  // FTC's own tail duplication (same pose twice, same heading) and a small
  // kink under the planner's threshold are not pivots.
  std::vector<PlanPose2D> p;
  appendStraight(p, 0.0, 0.0, 0.0, 1.0, 0.03);
  p.push_back(p.back());
  appendStraight(p, p.back().x + 0.03, 0.0, 0.0, 0.3, 0.03);
  EXPECT_TRUE(FindPivotCorners(p).empty());
  std::vector<PlanPose2D> kink;
  appendStraight(kink, 0.0, 0.0, 0.0, 1.0, 0.03);
  kink.push_back({kink.back().x, kink.back().y, 0.5 * kPivotCornerDetectTurnRad});
  appendStraight(kink, kink.back().x + 0.03, 0.0, 0.5 * kPivotCornerDetectTurnRad, 0.3, 0.03);
  EXPECT_TRUE(FindPivotCorners(kink).empty());
}

TEST(FtcPivot, CornerAtTheEndOfThePlanIsIgnored)
{
  // Nothing follows the outgoing twin: POST_ROTATE owns the final heading.
  std::vector<PlanPose2D> p;
  appendStraight(p, 0.0, 0.0, 0.0, 1.0, 0.03);
  p.push_back({p.back().x, p.back().y, M_PI / 2});
  EXPECT_TRUE(FindPivotCorners(p).empty());
}

TEST(FtcPivot, DetectionThresholdIsBelowThePlannersGuarantee)
{
  EXPECT_LT(kPivotCornerDetectTurnRad, cg::kPivotCornerMinTurnRad);
  EXPECT_GT(kPivotCornerDetectTurnRad, 0.0);
}

TEST(FtcPivot, NextCornerAndLegBounds)
{
  const std::vector<std::size_t> corners = {10, 20};
  EXPECT_EQ(NextPivotCorner(corners, 0), 10u);
  EXPECT_EQ(NextPivotCorner(corners, 10), 10u);
  EXPECT_EQ(NextPivotCorner(corners, 11), 20u);
  EXPECT_FALSE(NextPivotCorner(corners, 21).has_value());

  EXPECT_EQ(PivotLeg(corners, 5, 40), Leg(0, 10));
  EXPECT_EQ(PivotLeg(corners, 10, 40), Leg(0, 10));
  EXPECT_EQ(PivotLeg(corners, 11, 40), Leg(11, 20));
  EXPECT_EQ(PivotLeg(corners, 25, 40), Leg(21, 39));
  EXPECT_EQ(PivotLeg({}, 25, 40), Leg(0, 39));
}

TEST(FtcPivot, ObstacleWindowStopsAtTheCorner)
{
  EXPECT_EQ(ClipWindowToCorner(5, 35, std::size_t{10}), 11u) << "corner pose included, no further";
  EXPECT_EQ(ClipWindowToCorner(5, 8, std::size_t{10}), 8u) << "short window untouched";
  EXPECT_EQ(ClipWindowToCorner(5, 35, std::nullopt), 35u);
  EXPECT_EQ(ClipWindowToCorner(12, 35, std::size_t{10}), 35u) << "corner behind the window";
}

TEST(FtcPivot, PlanOpeningOnACornerStartsAtItsOutgoingPose)
{
  EXPECT_EQ(PivotAwareStartIndex({0, 30}, 0), 1u);
  EXPECT_EQ(PivotAwareStartIndex({0, 30}, 30), 31u) << "legacy nearest snap onto a corner";
  EXPECT_EQ(PivotAwareStartIndex({12}, 0), 0u);
  EXPECT_EQ(PivotAwareStartIndex({}, 0), 0u);
}

TEST(FtcPivot, ArrivalAndAlignment)
{
  EXPECT_FALSE(PivotArrived(0.10, kPivotArrivalToleranceM));
  EXPECT_TRUE(PivotArrived(0.015, kPivotArrivalToleranceM));
  EXPECT_TRUE(PivotArrived(-0.03, kPivotArrivalToleranceM)) << "overshoot still pivots";
  const double tol = 10.0 * M_PI / 180.0;
  EXPECT_FALSE(PivotAligned(0.5, tol));
  EXPECT_TRUE(PivotAligned(0.1, tol));
  EXPECT_TRUE(PivotAligned(2.0 * M_PI + 0.05, tol)) << "wrapped, like PRE_ROTATE";
}

TEST(FtcPivot, SweepYawsCoverTheShortestRotation)
{
  const auto yaws = PivotSweepYaws(0.0, M_PI / 2, 0.1);
  ASSERT_GE(yaws.size(), 17u);
  EXPECT_DOUBLE_EQ(yaws.front(), 0.0);
  EXPECT_NEAR(yaws.back(), M_PI / 2, 1e-12);
  for (std::size_t i = 1; i < yaws.size(); ++i)
  {
    EXPECT_LE(yaws[i] - yaws[i - 1], 0.1 + 1e-12);
    EXPECT_GT(yaws[i] - yaws[i - 1], 0.0);
  }
  // Across ±π the rotation goes the short way (0.2 rad), not round the circle.
  const auto wrap = PivotSweepYaws(M_PI - 0.1, -M_PI + 0.1, 0.05);
  EXPECT_LE(wrap.size(), 6u);
  EXPECT_NEAR(wrap.back(), M_PI + 0.1, 1e-12);
  EXPECT_EQ(PivotSweepYaws(0.3, 0.3, 0.1).size(), 2u);
}

}  // namespace
}  // namespace mowgli_nav2_plugins
