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
// PIVOT JOINS (field 2026-09-21): with the headland ring count on AUTO the
// robot's narrow tool (op_width 0.13 m) left a 0.195 m apron past the swath
// ends, no 0.20 m turn-around arc fitted, and the planner split a 152 m² lawn
// into 128 sub-paths — one blade-off Nav2 transit + PRE_ROTATE per row end,
// 6.7 % mowed in 6 minutes. A join that fits no arc may now stay in the
// sub-path as a short straight whose corners FTC pivots at in place, where the
// pivot sweep fits the designed envelope. These tests run the REAL Fields2Cover
// v3 pipeline (planBoustrophedon) and the join builder.

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "mowgli_coverage/coverage_planning.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
#include "ogr_geometry.h"
#include <gtest/gtest.h>

namespace
{

using mowgli_coverage::BoustrophedonPlan;
using mowgli_coverage::buildContinuousSubPaths;
using mowgli_coverage::ConnectorStats;
using mowgli_coverage::distanceToRing;
using mowgli_coverage::pathHeadings;
using mowgli_coverage::PivotJoinLimits;
using mowgli_coverage::pivotSweepFits;
using mowgli_coverage::planBoustrophedon;
using mowgli_coverage::pointInRing;
namespace cg = mowgli_interfaces::coverage_geometry;

using Pts = std::vector<std::pair<double, double>>;
using SubPaths = std::vector<Pts>;

// The robot of the 2026-09-21 field bag: tool_width 0.15 − swath_overlap 0.02,
// headland_width 0.15 (AUTO → 2 rings), 0.20 m arcs, template chassis.
constexpr double kOpWidth = 0.13;
constexpr double kHeadland = 0.15;
constexpr double kTurnRadius = 0.20;
constexpr double kMinTurnRadius = 0.20;
constexpr double kStep = 0.03;
constexpr double kMinSwath = 0.15;
constexpr double kInset = 0.0;
// robot_config_util.chassis_circumscribed_radius at the template chassis:
// hypot(chassis_center_x + length/2 + 0.05, width/2 + 0.05).
const double kSweepRadius = std::hypot(0.18 + 0.30 + 0.05, 0.225 + 0.05);
// == coverage_server.cpp kBoundarySlackM, the server's own verify slack.
constexpr double kBoundarySlack = 0.05;

Pts rectRing(double w, double h)
{
  return {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}};
}

f2c::types::LinearRing toRing(const Pts& pts)
{
  f2c::types::LinearRing ring;
  for (const auto& p : pts)
  {
    ring.addPoint(f2c::types::Point(p.first, p.second));
  }
  ring.addPoint(f2c::types::Point(pts.front().first, pts.front().second));
  return ring;
}

PivotJoinLimits shippedLimits(const Pts& recorded, const std::vector<Pts>& obstacles = {})
{
  PivotJoinLimits limits;
  limits.sweep_radius = kSweepRadius;
  limits.boundary_margin = kSweepRadius;  // enforce_boundary_margin_m floored at it
  limits.recorded_boundary = recorded;
  limits.recorded_obstacles = obstacles;
  return limits;
}

const Pts& clearanceOf(const BoustrophedonPlan& plan)
{
  return plan.connector_clearance_boundary.size() >= 3 ? plan.connector_clearance_boundary
                                                       : plan.safe_boundary;
}

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

bool coincident(const std::pair<double, double>& a, const std::pair<double, double>& b)
{
  return std::hypot(a.first - b.first, a.second - b.second) < cg::kPivotCornerMaxStepM;
}

// Every pivot corner of every sub-path, as the index of its FIRST pose.
std::vector<std::pair<std::size_t, std::size_t>> cornersOf(const SubPaths& subs)
{
  std::vector<std::pair<std::size_t, std::size_t>> out;  // (sub-path, index)
  for (std::size_t s = 0; s < subs.size(); ++s)
  {
    for (std::size_t i = 0; i + 1 < subs[s].size(); ++i)
    {
      if (coincident(subs[s][i], subs[s][i + 1]))
      {
        out.emplace_back(s, i);
      }
    }
  }
  return out;
}

// The pivot corner contract (coverage_geometry.hpp), checked on real output:
// a near-zero step appears only as a bit-exact twin, never at either end of a
// sub-path, never three in a row, and always turns by more than the threshold
// — so FTC can never mistake ordinary curvature for a pivot.
void expectCornerContract(const SubPaths& subs)
{
  for (const auto& [s, i] : cornersOf(subs))
  {
    const Pts& sub = subs[s];
    EXPECT_EQ(sub[i], sub[i + 1]) << "a corner twin must be bit-exact";
    EXPECT_GT(i, 0u) << "a corner never opens a sub-path";
    EXPECT_LT(i + 2, sub.size()) << "a corner never closes a sub-path";
    if (i + 2 < sub.size())
    {
      EXPECT_FALSE(coincident(sub[i + 1], sub[i + 2])) << "never three coincident poses";
    }
    const auto yaw = pathHeadings(sub);
    EXPECT_GT(std::abs(wrap(yaw[i + 1] - yaw[i])), cg::kPivotCornerMinTurnRad)
        << "a corner must turn by more than the aligned-kink threshold";
  }
}

// Strictly inside a hole. A swath END clipped by F2C lies ON the hole ring
// (rings off), where ray casting is ambiguous — same 1 mm tolerance as the
// planner's allInside().
bool insideHole(const std::pair<double, double>& p, const Pts& hole)
{
  return pointInRing(p.first, p.second, hole) && distanceToRing(p.first, p.second, hole) > 1e-3;
}

std::size_t posesOutside(const SubPaths& subs, const Pts& ring)
{
  std::size_t out = 0;
  for (const auto& sub : subs)
  {
    for (const auto& p : sub)
    {
      if (!pointInRing(p.first, p.second, ring) &&
          distanceToRing(p.first, p.second, ring) > kBoundarySlack)
      {
        ++out;
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Synthetic two-swath joins: one row-end turn, hand-checkable geometry.
// Swath A runs north up x = 1.00 to the top edge (y = 3), swath B comes back
// south down x = 1.13 — one op_width over, exactly the robot's spacing. The
// swath ends sit ON the clearance ring, so no forward arc fits (it would have
// to loop north of y = 3): today that join splits.
// ---------------------------------------------------------------------------
class PivotJoinFixture : public ::testing::Test
{
protected:
  BoustrophedonPlan plan;
  Pts clearance = rectRing(4.0, 3.0);
  PivotJoinLimits limits = shippedLimits(rectRing(4.0, 3.0));
  ConnectorStats stats;

  void SetUp() override
  {
    plan.swaths = {{{1.00, 1.0}, {1.00, 3.0}}, {{1.13, 3.0}, {1.13, 1.0}}};
  }

  SubPaths build()
  {
    stats = ConnectorStats{};
    auto subs = buildContinuousSubPaths(
        plan, clearance, kTurnRadius, kMinTurnRadius, kStep, &stats, {}, limits);
    EXPECT_EQ(stats.attempted, 1u);
    EXPECT_EQ(stats.attempted, stats.arc + stats.straight_kept + stats.pivot + stats.split);
    return subs;
  }
};

TEST_F(PivotJoinFixture, DisabledLimitsSplitAsBefore)
{
  limits = PivotJoinLimits{};
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.split, 1u);
  EXPECT_EQ(stats.pivot, 0u);
  EXPECT_TRUE(cornersOf(subs).empty());
}

TEST_F(PivotJoinFixture, AdjacentPassesBecomeOnePivotJoin)
{
  const auto subs = build();
  ASSERT_EQ(subs.size(), 1u) << "the row-end turn must stay inside the sub-path";
  EXPECT_EQ(stats.pivot, 1u);
  EXPECT_EQ(stats.split, 0u);
  const auto corners = cornersOf(subs);
  ASSERT_EQ(corners.size(), 2u) << "a 90°-straight-90° U-turn has two pivot corners";
  const Pts& sub = subs.front();
  const auto yaw = pathHeadings(sub);
  // Corner 1 at the end of A: arrive northbound, leave eastbound.
  const std::size_t a = corners[0].second;
  EXPECT_NEAR(sub[a].first, 1.00, 1e-9);
  EXPECT_NEAR(sub[a].second, 3.00, 1e-9);
  EXPECT_NEAR(wrap(yaw[a] - M_PI / 2), 0.0, 1e-9);
  EXPECT_NEAR(wrap(yaw[a + 1] - 0.0), 0.0, 1e-9);
  // Corner 2 at the start of B: arrive eastbound, leave southbound.
  const std::size_t b = corners[1].second;
  EXPECT_NEAR(sub[b].first, 1.13, 1e-9);
  EXPECT_NEAR(sub[b].second, 3.00, 1e-9);
  EXPECT_NEAR(wrap(yaw[b] - 0.0), 0.0, 1e-9);
  EXPECT_NEAR(wrap(yaw[b + 1] + M_PI / 2), 0.0, 1e-9);
  expectCornerContract(subs);
  EXPECT_EQ(posesOutside(subs, clearance), 0u);
  // The straight between the two corners is the only connector: one op_width.
  EXPECT_NEAR(std::hypot(sub[b].first - sub[a].first, sub[b].second - sub[a].second), 0.13, 1e-9);
}

TEST_F(PivotJoinFixture, RelocationStillSplits)
{
  // 0.70 m > kSegmentTransitGapM: a lobe change, never a turn-around.
  plan.swaths.back() = {{1.70, 3.0}, {1.70, 1.0}};
  ASSERT_GT(0.70, cg::kSegmentTransitGapM);
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.split, 1u);
  EXPECT_EQ(stats.pivot, 0u);
}

TEST_F(PivotJoinFixture, ConnectorThroughAHoleStillSplits)
{
  plan.safe_holes = {{{1.04, 2.95}, {1.09, 2.95}, {1.09, 3.05}, {1.04, 3.05}}};
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.split, 1u);
  EXPECT_EQ(stats.pivot, 0u);
}

TEST_F(PivotJoinFixture, ConnectorLeavingTheClearanceRingStillSplits)
{
  // Same ends, but a notch in the clearance ring between them: the straight
  // would cut outside it.
  clearance = {
      {0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {1.10, 3.0}, {1.065, 2.9}, {1.03, 3.0}, {0.0, 3.0}};
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.pivot, 0u);
}

TEST_F(PivotJoinFixture, SweepPastTheSoftBandSplits)
{
  // The ends sit ON the recorded line; a band narrower than the body's reach
  // cannot hold the pivot sweep.
  limits.boundary_margin = kSweepRadius - 0.05;
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.pivot, 0u);
  EXPECT_EQ(stats.split, 1u);
}

TEST_F(PivotJoinFixture, SweepDeepEnoughInsideFitsANarrowerBand)
{
  // Same narrow band, but the recorded line is 0.20 m past the ends (a
  // headland apron): the sweep only reaches radius − 0.20 beyond it.
  limits.boundary_margin = kSweepRadius - 0.05;
  limits.recorded_boundary = rectRing(4.0, 3.2);
  const auto subs = build();
  EXPECT_EQ(subs.size(), 1u);
  EXPECT_EQ(stats.pivot, 1u);
}

TEST_F(PivotJoinFixture, DrawnObstacleInsideTheSweepSplits)
{
  // 0.10 m post 0.33 m from the B corner: the front of the pivoting body would
  // hit it.
  limits.recorded_obstacles = {{{1.35, 3.25}, {1.45, 3.25}, {1.45, 3.35}, {1.35, 3.35}}};
  ASSERT_LT(distanceToRing(1.13, 3.0, limits.recorded_obstacles.front()), kSweepRadius);
  const auto subs = build();
  EXPECT_EQ(subs.size(), 2u);
  EXPECT_EQ(stats.pivot, 0u);
}

TEST_F(PivotJoinFixture, DrawnObstacleJustOutsideTheSweepKeepsThePivot)
{
  const double x = 1.13 + kSweepRadius + 0.02;
  limits.recorded_obstacles = {{{x, 2.9}, {x + 0.1, 2.9}, {x + 0.1, 3.1}, {x, 3.1}}};
  const auto subs = build();
  EXPECT_EQ(subs.size(), 1u);
  EXPECT_EQ(stats.pivot, 1u);
}

// ---------------------------------------------------------------------------
// Pure helpers.
// ---------------------------------------------------------------------------
TEST(PivotSweep, FitsOnlyInsideTheBandAndClearOfObstacles)
{
  const auto limits = shippedLimits(rectRing(4.0, 3.0), {{{2.0, 2.0}, {2.2, 2.0}, {2.2, 2.2}}});
  EXPECT_TRUE(pivotSweepFits(1.0, 1.0, limits)) << "deep inside";
  EXPECT_TRUE(pivotSweepFits(1.0, 3.0, limits)) << "ON the line, band == reach";
  EXPECT_FALSE(pivotSweepFits(1.0, 3.01, limits)) << "1 cm past the line";
  EXPECT_FALSE(pivotSweepFits(2.0, 1.6, limits)) << "0.4 m from a drawn obstacle";
  EXPECT_FALSE(pivotSweepFits(2.1, 2.05, limits)) << "inside a drawn obstacle";
  PivotJoinLimits disabled = limits;
  disabled.sweep_radius = 0.0;
  EXPECT_FALSE(pivotSweepFits(1.0, 1.0, disabled)) << "disabled never pivots";
  PivotJoinLimits no_boundary = limits;
  no_boundary.recorded_boundary.clear();
  EXPECT_FALSE(pivotSweepFits(1.0, 1.0, no_boundary)) << "unknown boundary never pivots";
}

TEST(PivotCorner, PathHeadingsGiveEachTwinItsOwnHeading)
{
  const Pts pts = {{0.0, 0.0}, {0.0, 1.0}, {0.0, 1.0}, {0.5, 1.0}, {0.5, 1.0}, {0.5, 0.0}};
  const auto yaw = pathHeadings(pts);
  ASSERT_EQ(yaw.size(), pts.size());
  EXPECT_NEAR(yaw[0], M_PI / 2, 1e-12);
  EXPECT_NEAR(yaw[1], M_PI / 2, 1e-12) << "first twin: incoming heading";
  EXPECT_NEAR(yaw[2], 0.0, 1e-12) << "second twin: outgoing heading";
  EXPECT_NEAR(yaw[3], 0.0, 1e-12);
  EXPECT_NEAR(yaw[4], -M_PI / 2, 1e-12);
  EXPECT_NEAR(yaw[5], -M_PI / 2, 1e-12) << "last pose: incoming heading";
}

TEST(PivotCorner, PathHeadingsMatchTheOldRuleWithoutCorners)
{
  // No coincident poses: the next-step rule coverage_server always used.
  const Pts pts = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 2.0}};
  const auto yaw = pathHeadings(pts);
  EXPECT_NEAR(yaw[0], 0.0, 1e-12);
  EXPECT_NEAR(yaw[1], M_PI / 2, 1e-12);
  EXPECT_NEAR(yaw[2], std::atan2(1.0, -1.0), 1e-12);
  EXPECT_NEAR(yaw[3], std::atan2(1.0, -1.0), 1e-12);
}

// ---------------------------------------------------------------------------
// Whole-field plans with the robot's settings.
// ---------------------------------------------------------------------------
class PivotJoinField : public ::testing::TestWithParam<int>
{
};

struct FieldRun
{
  BoustrophedonPlan plan;
  SubPaths subs;
  ConnectorStats stats;
};

FieldRun planField(const f2c::types::Cell& cell,
                   int passes,
                   const PivotJoinLimits& limits,
                   double mow_angle = 0.0)
{
  FieldRun run;
  run.plan = planBoustrophedon(
      cell, kOpWidth, kHeadland, passes, kInset, mow_angle, kMinSwath, 0, kMinTurnRadius);
  run.subs = buildContinuousSubPaths(run.plan,
                                     clearanceOf(run.plan),
                                     kTurnRadius,
                                     kMinTurnRadius,
                                     kStep,
                                     &run.stats,
                                     run.plan.swath_turn_envelope,
                                     limits);
  return run;
}

// A plain 6 x 4 m lawn: with 0 rings, AUTO (2) or 1 ring, every row end used to
// be a blade-off transit. With pivot joins the plan is O(1) sub-paths — at most
// the one ring→first-swath relocation remains.
TEST_P(PivotJoinField, ThinApronRectangleIsOneOrTwoSubPaths)
{
  const int passes = GetParam();
  const Pts recorded = rectRing(6.0, 4.0);
  const f2c::types::Cell cell(toRing(recorded));

  const auto before = planField(cell, passes, PivotJoinLimits{});
  ASSERT_FALSE(before.plan.swaths.empty());
  EXPECT_GT(before.subs.size(), 10u) << "precondition: the thin apron fragments the plan";

  const auto after = planField(cell, passes, shippedLimits(recorded));
  EXPECT_LE(after.subs.size(), 2u) << "splits " << after.stats.split << " pivots "
                                   << after.stats.pivot << " arcs " << after.stats.arc;
  EXPECT_LE(after.stats.split, 1u);
  EXPECT_GT(after.stats.pivot, 10u);
  expectCornerContract(after.subs);
  EXPECT_EQ(posesOutside(after.subs, clearanceOf(after.plan)), 0u);
  for (const auto& [s, i] : cornersOf(after.subs))
  {
    const auto& p = after.subs[s][i];
    EXPECT_TRUE(pivotSweepFits(p.first, p.second, shippedLimits(recorded)))
        << "pivot at (" << p.first << ", " << p.second << ") sweeps past the band";
  }
  // Determinism (the resume cursor indexes into this concatenation).
  EXPECT_EQ(planField(cell, passes, shippedLimits(recorded)).subs, after.subs);
}

INSTANTIATE_TEST_SUITE_P(HeadlandPasses, PivotJoinField, ::testing::Values(-1, 0, 1));

// A drawn obstacle in the lawn: no pivot may sweep over it, the joins beside it
// split as before, the rest of the lawn still pivots.
TEST(PivotJoinFieldObstacle, NoPivotSweepsOverADrawnObstacle)
{
  const Pts recorded = rectRing(8.0, 5.0);
  const Pts obstacle = {{3.8, 2.3}, {4.3, 2.3}, {4.3, 2.8}, {3.8, 2.8}};
  f2c::types::Cell cell(toRing(recorded));
  cell.addRing(mowgli_coverage::bufferRingOutward(toRing(obstacle), 0.389));

  const auto before = planField(cell, 0, PivotJoinLimits{});
  const auto after = planField(cell, 0, shippedLimits(recorded, {obstacle}));
  EXPECT_LT(after.subs.size(), before.subs.size() / 4);
  EXPECT_GT(after.stats.pivot, 10u);
  expectCornerContract(after.subs);
  for (const auto& [s, i] : cornersOf(after.subs))
  {
    const auto& p = after.subs[s][i];
    EXPECT_FALSE(pointInRing(p.first, p.second, obstacle));
    EXPECT_GE(distanceToRing(p.first, p.second, obstacle), kSweepRadius)
        << "pivot at (" << p.first << ", " << p.second << ") sweeps over the obstacle";
  }
  for (const auto& sub : after.subs)
  {
    for (const auto& p : sub)
    {
      for (const auto& hole : after.plan.safe_holes)
      {
        EXPECT_FALSE(insideHole(p, hole)) << "(" << p.first << ", " << p.second << ")";
      }
    }
  }
}

// Rings off hands F2C the goal cell itself. Two drawn obstacles 0.30 m apart,
// each grown by the 0.389 m obstacle margin, overlap: an INVALID polygon that
// crashed F2C's swath clip (GEOS TopologyException, then SIGSEGV) on the
// 2026-09-21 lawn geometry. It must be repaired and planned.
TEST(PivotJoinFieldObstacle, RingsOffWithOverlappingGrownObstaclesStillPlans)
{
  const Pts recorded = rectRing(8.0, 5.0);
  f2c::types::Cell cell(toRing(recorded));
  cell.addRing(
      mowgli_coverage::bufferRingOutward(toRing({{3.0, 2.0}, {3.5, 2.0}, {3.5, 2.5}, {3.0, 2.5}}),
                                         0.389));
  cell.addRing(
      mowgli_coverage::bufferRingOutward(toRing({{3.8, 2.0}, {4.3, 2.0}, {4.3, 2.5}, {3.8, 2.5}}),
                                         0.389));
  ASSERT_FALSE(cell.get()->IsValid()) << "precondition: the grown obstacles overlap";

  const auto run = planField(cell, -1, shippedLimits(recorded));
  EXPECT_FALSE(run.plan.swaths.empty());
  EXPECT_FALSE(run.subs.empty());
  bool noted = false;
  for (const auto& note : run.plan.diagnostics.notes)
  {
    noted = noted || note.find("repaired") != std::string::npos;
  }
  EXPECT_TRUE(noted) << "the repair must be visible in the plan diagnostics";
  for (const auto& sub : run.subs)
  {
    for (const auto& p : sub)
    {
      for (const auto& hole : run.plan.safe_holes)
      {
        EXPECT_FALSE(insideHole(p, hole)) << "(" << p.first << ", " << p.second << ")";
      }
    }
  }
}

}  // namespace
