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
// Tests for summarisePlanForPreview, the reduction behind the GUI's map-editor
// "line preview": rings simplified, swaths kept in drive order, heading folded
// into [0, 180). The last group runs the REAL planner (Fields2Cover v3) so the
// preview is checked against what a plan actually produces.

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_coverage::BoustrophedonPlan;
using mowgli_coverage::CoveragePreview;
using mowgli_coverage::planBoustrophedon;
using mowgli_coverage::summarisePlanForPreview;

using Point = std::pair<double, double>;
using Ring = std::vector<Point>;

// Shoelace area: > 0 counter-clockwise, < 0 clockwise.
double signedArea(const Ring& ring)
{
  double twice = 0.0;
  for (std::size_t i = 0; i < ring.size(); ++i)
  {
    const Point& a = ring[i];
    const Point& b = ring[(i + 1) % ring.size()];
    twice += a.first * b.second - b.first * a.second;
  }
  return 0.5 * twice;
}

// Distance from p to the segment a-b.
double distToSegment(const Point& p, const Point& a, const Point& b)
{
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0.0 ? ((p.first - a.first) * dx + (p.second - a.second) * dy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.first - (a.first + t * dx), p.second - (a.second + t * dy));
}

// A closed square loop of side `size`, points `step` apart, like the planner's
// densified headland rings (first point NOT repeated at the end).
Ring densifiedSquare(double size, double step, bool clockwise)
{
  Ring ring;
  const int n = static_cast<int>(std::lround(size / step));
  for (int i = 0; i < n; ++i)
    ring.emplace_back(i * step, 0.0);
  for (int i = 0; i < n; ++i)
    ring.emplace_back(size, i * step);
  for (int i = 0; i < n; ++i)
    ring.emplace_back(size - i * step, size);
  for (int i = 0; i < n; ++i)
    ring.emplace_back(0.0, size - i * step);
  if (clockwise)
    std::reverse(ring.begin(), ring.end());
  return ring;
}

f2c::types::Cell makeSquare(double size)
{
  f2c::types::LinearRing ring;
  ring.addPoint(f2c::types::Point(0.0, 0.0));
  ring.addPoint(f2c::types::Point(size, 0.0));
  ring.addPoint(f2c::types::Point(size, size));
  ring.addPoint(f2c::types::Point(0.0, size));
  ring.addPoint(f2c::types::Point(0.0, 0.0));
  return f2c::types::Cell(ring);
}

// Smallest absolute difference between two headings modulo 180 degrees.
double headingDiffDeg(double a, double b)
{
  double d = std::fmod(std::fabs(a - b), 180.0);
  return std::min(d, 180.0 - d);
}

}  // namespace

TEST(CoveragePreview, DensifiedRingCollapsesToItsCorners)
{
  BoustrophedonPlan plan;
  plan.rings.push_back(densifiedSquare(10.0, 0.1, false));

  const CoveragePreview preview = summarisePlanForPreview(plan);

  ASSERT_EQ(preview.rings.size(), 1u);
  // 400 densified points -> the four corners (plus the kept first/last vertex).
  EXPECT_LE(preview.rings[0].size(), 6u);
  EXPECT_GE(preview.rings[0].size(), 4u);
  // First and last vertex survive, so the start of the loop is never lost.
  EXPECT_EQ(preview.rings[0].front(), plan.rings[0].front());
  EXPECT_EQ(preview.rings[0].back(), plan.rings[0].back());
}

TEST(CoveragePreview, SimplifiedRingStaysWithinTolerance)
{
  BoustrophedonPlan plan;
  // A gentle arc, densified: the simplification must not cut it by more than the
  // tolerance, however many vertices it drops.
  Ring arc;
  for (int i = 0; i <= 200; ++i)
  {
    const double a = M_PI * i / 200.0;
    arc.emplace_back(5.0 * std::cos(a), 5.0 * std::sin(a));
  }
  plan.rings.push_back(arc);

  const double tol = 0.02;
  const CoveragePreview preview = summarisePlanForPreview(plan, tol);

  ASSERT_EQ(preview.rings.size(), 1u);
  const Ring& simple = preview.rings[0];
  EXPECT_LT(simple.size(), arc.size());
  for (const Point& p : arc)
  {
    double best = 1e9;
    for (std::size_t i = 0; i + 1 < simple.size(); ++i)
      best = std::min(best, distToSegment(p, simple[i], simple[i + 1]));
    EXPECT_LE(best, tol + 1e-9);
  }
}

TEST(CoveragePreview, ARingThatDoublesBackKeepsItsFarVertex)
{
  // Out along the x axis and back again: the far vertex is collinear with the
  // chord from the first to the last vertex, yet 10 m beyond its end. Measuring
  // to the chord's infinite line would drop it and cut the ring short.
  BoustrophedonPlan plan;
  plan.rings.push_back({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}, {7.0, 0.0}, {5.0, 0.0}});

  const CoveragePreview preview = summarisePlanForPreview(plan);

  ASSERT_EQ(preview.rings.size(), 1u);
  double max_x = 0.0;
  for (const Point& p : preview.rings[0])
    max_x = std::max(max_x, p.first);
  EXPECT_NEAR(max_x, 10.0, 1e-9);
}

TEST(CoveragePreview, SimplificationKeepsTheDriveDirection)
{
  BoustrophedonPlan plan;
  plan.rings.push_back(densifiedSquare(10.0, 0.1, false));
  plan.rings.push_back(densifiedSquare(9.0, 0.1, true));

  const CoveragePreview preview = summarisePlanForPreview(plan);

  ASSERT_EQ(preview.rings.size(), 2u);
  EXPECT_GT(signedArea(preview.rings[0]), 0.0);  // CCW stays CCW
  EXPECT_LT(signedArea(preview.rings[1]), 0.0);  // CW stays CW
  EXPECT_NEAR(std::fabs(signedArea(preview.rings[0])), 100.0, 0.5);
}

TEST(CoveragePreview, SwathsAreCopiedInDriveOrder)
{
  BoustrophedonPlan plan;
  plan.swaths = {{{0.0, 0.0}, {10.0, 0.0}}, {{10.0, 1.0}, {0.0, 1.0}}, {{0.0, 2.0}, {10.0, 2.0}}};

  const CoveragePreview preview = summarisePlanForPreview(plan);

  EXPECT_EQ(preview.swaths, plan.swaths);
}

TEST(CoveragePreview, HeadingIsFoldedIntoZeroToOneEighty)
{
  struct Case
  {
    double rad;
    double expected_deg;
  };
  const Case cases[] = {{0.0, 0.0},
                        {M_PI / 6.0, 30.0},
                        {-M_PI / 2.0, 90.0},  // a negative heading
                        {-M_PI / 6.0, 150.0},  // -30 deg is the same line as 150
                        {M_PI, 0.0},  // 180 folds to 0
                        {1.5 * M_PI, 90.0}};
  for (const Case& c : cases)
  {
    BoustrophedonPlan plan;
    plan.swath_angle_rad = c.rad;
    const CoveragePreview preview = summarisePlanForPreview(plan);
    EXPECT_GE(preview.swath_angle_deg, 0.0);
    EXPECT_LT(preview.swath_angle_deg, 180.0);
    EXPECT_LT(headingDiffDeg(preview.swath_angle_deg, c.expected_deg), 1e-6)
        << "rad=" << c.rad << " got " << preview.swath_angle_deg;
  }
}

TEST(CoveragePreview, CarriesTheCountsThrough)
{
  BoustrophedonPlan plan;
  plan.n_headland_passes = 3;
  plan.diagnostics.planned_fraction = 0.93;
  plan.diagnostics.field_area = 412.5;

  const CoveragePreview preview = summarisePlanForPreview(plan);

  EXPECT_EQ(preview.headland_passes, 3);
  EXPECT_DOUBLE_EQ(preview.planned_fraction, 0.93);
  EXPECT_DOUBLE_EQ(preview.field_area_m2, 412.5);
  EXPECT_EQ(preview.dropped_pieces, plan.diagnostics.drops.size());
}

// ---------------------------------------------------------------------------
// Against the real planner
// ---------------------------------------------------------------------------

class CoveragePreviewRealPlanner : public ::testing::Test
{
protected:
  static BoustrophedonPlan plan(double mow_angle_deg,
                                int ring_direction,
                                bool perpendicular = false)
  {
    const double rad = mow_angle_deg < 0.0 ? -1.0 : mow_angle_deg * M_PI / 180.0;
    return planBoustrophedon(
        makeSquare(20.0), 0.18, 0.30, 0, 0.0, rad, 0.15, ring_direction, 0.20, perpendicular, 0);
  }
};

TEST_F(CoveragePreviewRealPlanner, ReportsTheRequestedHeading)
{
  for (const double angle : {0.0, 30.0, 90.0, 135.0})
  {
    const CoveragePreview preview = summarisePlanForPreview(plan(angle, 0));
    EXPECT_FALSE(preview.swaths.empty()) << "angle " << angle;
    EXPECT_LT(headingDiffDeg(preview.swath_angle_deg, angle), 0.5) << "angle " << angle;
  }
}

TEST_F(CoveragePreviewRealPlanner, AutoHeadingResolvesToAConcreteAngle)
{
  const CoveragePreview preview = summarisePlanForPreview(plan(-1.0, 0));
  EXPECT_FALSE(preview.swaths.empty());
  EXPECT_GE(preview.swath_angle_deg, 0.0);
  EXPECT_LT(preview.swath_angle_deg, 180.0);
}

TEST_F(CoveragePreviewRealPlanner, PerpendicularTurnsTheHeadingNinetyDegrees)
{
  const CoveragePreview straight = summarisePlanForPreview(plan(30.0, 0, false));
  const CoveragePreview crossed = summarisePlanForPreview(plan(30.0, 0, true));
  EXPECT_LT(headingDiffDeg(crossed.swath_angle_deg, straight.swath_angle_deg + 90.0), 0.5);
}

TEST_F(CoveragePreviewRealPlanner, RingDirectionDecidesTheWinding)
{
  const CoveragePreview cw = summarisePlanForPreview(plan(0.0, 1));
  const CoveragePreview ccw = summarisePlanForPreview(plan(0.0, 2));
  ASSERT_FALSE(cw.rings.empty());
  ASSERT_FALSE(ccw.rings.empty());
  for (const Ring& ring : cw.rings)
    EXPECT_LT(signedArea(ring), 0.0) << "ring_direction 1 must drive clockwise";
  for (const Ring& ring : ccw.rings)
    EXPECT_GT(signedArea(ring), 0.0) << "ring_direction 2 must drive counter-clockwise";
}

TEST_F(CoveragePreviewRealPlanner, SwathsAlternateDirection)
{
  const CoveragePreview preview = summarisePlanForPreview(plan(0.0, 0));
  ASSERT_GE(preview.swaths.size(), 3u);
  for (std::size_t i = 1; i < preview.swaths.size(); ++i)
  {
    const auto& prev = preview.swaths[i - 1];
    const auto& cur = preview.swaths[i];
    const double prev_dx = prev.second.first - prev.first.first;
    const double cur_dx = cur.second.first - cur.first.first;
    EXPECT_LT(prev_dx * cur_dx, 0.0) << "swath " << i << " runs the same way as the one before";
  }
}

TEST_F(CoveragePreviewRealPlanner, PreviewIsMuchSmallerThanThePlan)
{
  const BoustrophedonPlan full = plan(0.0, 0);
  const CoveragePreview preview = summarisePlanForPreview(full);
  std::size_t full_points = 0;
  std::size_t preview_points = 0;
  for (const Ring& r : full.rings)
    full_points += r.size();
  for (const Ring& r : preview.rings)
    preview_points += r.size();
  ASSERT_GT(full_points, 0u);
  EXPECT_LT(preview_points * 5, full_points);
}
