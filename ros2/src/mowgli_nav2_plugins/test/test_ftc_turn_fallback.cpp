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
// Tests for the turn fallback's decision (ftc_turn_fallback.hpp): when FTC's
// offset lattice is WEDGED, is the blockage in a turn, and which reverse /
// pivot / straight / pivot / rejoin gets round it safely. ROS-free: synthetic
// Costmap2D grids, plus one case recorded on the robot (2026-09-22 08:08:25).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

namespace mn = mowgli_nav2_plugins;
using mn::FallbackPose;
using mn::ObstacleDeviation;
using mn::TurnFallbackCfg;
using mn::TurnFallbackPlan;
using mn::TurnFallbackProblem;
using mn::TurnFallbackVerdict;
using Pose = geometry_msgs::msg::PoseStamped;

constexpr double kRes = 0.05;
constexpr double kStep = 0.05;  // the coverage planner's pose spacing

Pose MakePose(double x, double y, double yaw)
{
  Pose p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

double YawOf(const Pose& p)
{
  const auto& q = p.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// getRobotFootprint() on the robot: the 0.60 x 0.45 chassis centred 0.18 m
/// ahead of base_link, + the 0.05 m chassis_footprint margin + Nav2's
/// footprint_padding 0.01.
ObstacleDeviation::Footprint Footprint()
{
  ObstacleDeviation::Footprint fp;
  for (const auto& [x, y] : std::vector<std::pair<double, double>>{
           {0.54, 0.285}, {0.54, -0.285}, {-0.18, -0.285}, {-0.18, 0.285}})
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    fp.push_back(p);
  }
  return fp;
}

/// The lattice body: Footprint() widened by obstacle_clearance_margin 0.05.
ObstacleDeviation::Footprint Body()
{
  return ObstacleDeviation::expandFootprintLateral(Footprint(), 0.05);
}

/// Swath A along +x (y = 0) to x = x_turn, a U-turn of radius r to the LEFT,
/// swath B back along -x at y = 2r. Poses one kStep apart, headings along the
/// path — the shape of a row end.
std::vector<Pose> UTurnPlan(double x0, double x_turn, double r, double x_end)
{
  std::vector<Pose> plan;
  for (double x = x0; x < x_turn - 1e-9; x += kStep)
  {
    plan.push_back(MakePose(x, 0.0, 0.0));
  }
  const int n = static_cast<int>(std::ceil(M_PI * r / kStep));
  for (int k = 0; k <= n; ++k)
  {
    const double a = -M_PI / 2.0 + M_PI * k / n;
    plan.push_back(MakePose(x_turn + r * std::cos(a), r + r * std::sin(a), a + M_PI / 2.0));
  }
  for (double x = x_turn - kStep; x >= x_end - 1e-9; x -= kStep)
  {
    plan.push_back(MakePose(x, 2.0 * r, M_PI));
  }
  return plan;
}

std::vector<Pose> StraightPlan(double x0, double x1)
{
  std::vector<Pose> plan;
  for (double x = x0; x <= x1 + 1e-9; x += kStep)
  {
    plan.push_back(MakePose(x, 0.0, 0.0));
  }
  return plan;
}

void MarkLethal(nav2_costmap_2d::Costmap2D& costmap, double x, double y)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/// A wall of lethal cells along x = `x` (a hedge beyond the row end).
void MarkWallX(nav2_costmap_2d::Costmap2D& costmap, double x, double y0, double y1)
{
  for (double y = y0; y <= y1 + 1e-9; y += kRes)
  {
    MarkLethal(costmap, x, y);
  }
}

void MarkWallY(nav2_costmap_2d::Costmap2D& costmap, double y, double x0, double x1)
{
  for (double x = x0; x <= x1 + 1e-9; x += kRes)
  {
    MarkLethal(costmap, x, y);
  }
}

/// 10 x 10 m grid, origin (-3, -5).
nav2_costmap_2d::Costmap2D EmptyCostmap()
{
  return nav2_costmap_2d::Costmap2D(200, 200, kRes, -3.0, -5.0);
}

/// The plan window from the pose nearest the robot, as FTCController builds it.
TurnFallbackProblem Problem(const nav2_costmap_2d::Costmap2D& costmap,
                            const std::vector<Pose>& plan,
                            const FallbackPose& robot)
{
  TurnFallbackProblem p;
  p.costmap = &costmap;
  p.footprint = Footprint();
  p.body = Body();
  p.robot = robot;
  std::size_t carrot = 0;
  double best = 1e9;
  for (std::size_t i = 0; i < plan.size(); ++i)
  {
    const double d = std::hypot(plan[i].pose.position.x - robot.x - 0.3 * std::cos(robot.yaw),
                                plan[i].pose.position.y - robot.y - 0.3 * std::sin(robot.yaw));
    if (d < best)
    {
      best = d;
      carrot = i;  // the carrot leads the robot by the 0.30 m lead cap
    }
  }
  const auto [first, last] =
      mn::FallbackWindow(plan, carrot, robot.x, robot.y, mn::kFallbackRobotSearchBackM, 5.5);
  p.plan.assign(plan.begin() + static_cast<std::ptrdiff_t>(first),
                plan.begin() + static_cast<std::ptrdiff_t>(last));
  return p;
}

/// Independent re-check of a planned fallback with the BARE shapes the
/// executing controller tests every tick (no planning margin).
void ExpectEveryLegClear(const TurnFallbackProblem& p, const TurnFallbackPlan& plan)
{
  ASSERT_EQ(plan.verdict, TurnFallbackVerdict::kPlanned);
  const auto& cm = *p.costmap;
  EXPECT_TRUE(mn::ReverseSweepClear(cm, p.footprint, p.robot, plan.reverse_m));
  const double heading = plan.start.yaw + plan.rotate_start_rad;
  EXPECT_TRUE(mn::RotationSweepClear(
      cm, p.guard, p.footprint, plan.start.x, plan.start.y, plan.start.yaw, heading));
  if (plan.connector_m > 0.0)
  {
    EXPECT_TRUE(mn::StraightSweepClear(cm,
                                       p.guard,
                                       p.body,
                                       {plan.start.x, plan.start.y, heading},
                                       plan.rejoin_pose.x,
                                       plan.rejoin_pose.y));
    EXPECT_TRUE(mn::RotationSweepClear(cm,
                                       p.guard,
                                       p.footprint,
                                       plan.rejoin_pose.x,
                                       plan.rejoin_pose.y,
                                       heading,
                                       plan.rejoin_pose.yaw));
  }
  // The rejoin pose itself is on the line and clear for the lattice body.
  EXPECT_FALSE(ObstacleDeviation::footprintBlocked(
      cm, p.plan[plan.rejoin], 0.0, p.body, {}, ObstacleDeviation::kLethalOnlyThreshold));
  // What the log reports: every leg driven keeps the planning margin.
  const double margin = TurnFallbackCfg{}.plan_margin_m - 1e-3;
  const mn::TurnFallbackClearances cl = mn::MeasureTurnFallbackClearances(p, plan, 0.5);
  EXPECT_GE(cl.rotate_start_m, margin);
  if (plan.reverse_m > 0.0)
  {
    EXPECT_GE(cl.reverse_m, margin);
  }
  if (plan.connector_m > 0.0)
  {
    EXPECT_GE(cl.connector_m, margin);
    EXPECT_GE(cl.rotate_rejoin_m, margin);
  }
}

// ── Plan geometry helpers ────────────────────────────────────────────────────

TEST(FtcTurnFallback, HeadingSweepMeasuresTurnsNotStraights)
{
  const auto uturn = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const auto straight = StraightPlan(0.0, 5.0);
  // Just before the U-turn: the turn lies within the 0.75 m looked ahead.
  const std::size_t before_turn = 57;  // x = 2.85
  EXPECT_GT(mn::PlanHeadingSweep(uturn, before_turn, 0.3, 0.75), M_PI / 2.0);
  // The whole U-turn reads as a half turn, and an omega loop as more.
  EXPECT_NEAR(mn::PlanHeadingSweep(uturn, 60, 0.3, 1.0), M_PI, 1e-6);
  EXPECT_NEAR(mn::PlanHeadingSweep(straight, 40, 0.3, 0.75), 0.0, 1e-9);
  // Far from the turn, on the same plan, it reads straight.
  EXPECT_NEAR(mn::PlanHeadingSweep(uturn, 10, 0.3, 0.75), 0.0, 1e-9);
}

TEST(FtcTurnFallback, WindowStartsAtThePoseNearestTheRobotBehindTheCarrot)
{
  const auto plan = StraightPlan(0.0, 5.0);
  // Carrot at x = 2.0, robot 0.3 m behind it.
  const auto [first, last] = mn::FallbackWindow(plan, 40, 1.7, 0.02, 1.0, 2.0);
  EXPECT_EQ(first, 34u);
  EXPECT_NEAR(plan[last - 1].pose.position.x - plan[first].pose.position.x, 2.0, kStep + 1e-9);
  // Never further back than the search distance.
  const auto [far_first, far_last] = mn::FallbackWindow(plan, 40, 0.0, 0.0, 0.52, 2.0);
  (void)far_last;
  EXPECT_EQ(far_first, 30u);
}

// ── The decision ─────────────────────────────────────────────────────────────

TEST(FtcTurnFallback, BlockedUTurnBesideAHedgePlansAFallback)
{
  // Row end: swath A runs into a U-turn whose apex overhangs x = 3.3; the
  // hedge stands at x = 3.45, so the body front (0.54 m ahead of base_link)
  // hits it from x = 2.9 on — the lattice, which can only offset sideways, is
  // wedged there (08:08-09:39 on 2026-09-22).
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  const TurnFallbackPlan fb = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  ASSERT_EQ(fb.verdict, TurnFallbackVerdict::kPlanned) << fb.why;
  EXPECT_GE(fb.turn_rad, TurnFallbackCfg{}.min_turn_rad);
  // Rejoins swath B, heading back along -x, past the U-turn.
  EXPECT_NEAR(fb.rejoin_pose.y, 0.6, 1e-6);
  EXPECT_NEAR(std::abs(mn::WrapPivotAngle(fb.rejoin_pose.yaw)), M_PI, 1e-3);
  EXPECT_GT(fb.skipped_arc_m, 0.0);
  EXPECT_LE(fb.skipped_arc_m, TurnFallbackCfg{}.max_rejoin_arc_m);
  EXPECT_GT(fb.connector_m, 0.0);
  ExpectEveryLegClear(p, fb);
}

TEST(FtcTurnFallback, ObstacleOnAStraightIsNotATurnFallback)
{
  // A post in the middle of a long straight swath: the WEDGED path (reverse,
  // hold, abort -> FollowStrip's detour) stays in charge.
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.0, -0.6, 0.6);
  const auto plan = StraightPlan(0.0, 6.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {2.0, 0.0, 0.0});
  const TurnFallbackPlan fb = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  EXPECT_EQ(fb.verdict, TurnFallbackVerdict::kNotATurn);
  EXPECT_LT(fb.turn_rad, TurnFallbackCfg{}.min_turn_rad);
}

TEST(FtcTurnFallback, NothingOnTheLineIsNotAFallback)
{
  auto costmap = EmptyCostmap();
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  EXPECT_EQ(mn::PlanTurnFallback(p, TurnFallbackCfg{}).verdict, TurnFallbackVerdict::kNoBlockage);
}

TEST(FtcTurnFallback, UnreachableRejoinIsNoFallback)
{
  // The return swath is blocked along its whole length (a second hedge ON
  // it): no rejoin whose body is clear exists within the search.
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  MarkWallY(costmap, 0.6, -1.0, 3.4);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  const TurnFallbackPlan fb = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  EXPECT_EQ(fb.verdict, TurnFallbackVerdict::kNoRejoin);
}

TEST(FtcTurnFallback, RejoinSearchIsBoundedInArc)
{
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  TurnFallbackCfg cfg;
  cfg.max_rejoin_arc_m = 1.0;  // not even past the U-turn
  EXPECT_EQ(mn::PlanTurnFallback(p, cfg).verdict, TurnFallbackVerdict::kNoRejoin);

  // ...and in compute: the search gives up after max_evaluations candidates.
  TurnFallbackCfg budget;
  budget.max_evaluations = 1;
  const TurnFallbackPlan starved = mn::PlanTurnFallback(p, budget);
  EXPECT_EQ(starved.verdict, TurnFallbackVerdict::kNoRejoin);
  EXPECT_LE(starved.evaluated, 1u);
}

TEST(FtcTurnFallback, RotationNeedsAReverseAndTheReverseIsBounded)
{
  // The robot has driven close to the hedge: every pivot towards the return
  // swath sweeps its front-right corner into it where it stands, so it must
  // back up first — never more than max_reverse_m, and only as far as its rear
  // sweep is clear.
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const FallbackPose robot{2.90, 0.0, 0.0};
  const TurnFallbackProblem p = Problem(costmap, plan, robot);
  EXPECT_FALSE(mn::RotationSweepClear(costmap, {}, Footprint(), robot.x, robot.y, 0.0, M_PI / 2));

  TurnFallbackCfg cfg;
  cfg.max_reverse_m = 0.0;  // obstacle_reverse_enabled false
  EXPECT_EQ(mn::PlanTurnFallback(p, cfg).verdict, TurnFallbackVerdict::kNoRejoin);

  cfg.max_reverse_m = 0.40;
  const TurnFallbackPlan fb = mn::PlanTurnFallback(p, cfg);
  ASSERT_EQ(fb.verdict, TurnFallbackVerdict::kPlanned) << fb.why;
  EXPECT_GE(fb.reverse_m, 0.10 - 1e-9);
  EXPECT_LE(fb.reverse_m, cfg.max_reverse_m + 1e-9);
  EXPECT_NEAR(fb.start.x, robot.x - fb.reverse_m, 1e-9);
  ExpectEveryLegClear(p, fb);

  // Something behind the robot (right of swath A, y <= 0) caps the reverse
  // at what its rear strip allows (rear edge 0.18 m + planning margin 0.05 m
  // behind base_link; cell centre at x = 2.325: 0.30 m of reverse left).
  auto boxed = costmap;
  MarkWallX(boxed, 2.31, -1.0, 0.0);
  const TurnFallbackProblem q = Problem(boxed, plan, robot);
  const TurnFallbackPlan capped = mn::PlanTurnFallback(q, cfg);
  EXPECT_NEAR(capped.reverse_limit_m, 0.30, 1e-9);
  ASSERT_EQ(capped.verdict, TurnFallbackVerdict::kPlanned) << capped.why;
  EXPECT_LE(capped.reverse_m, capped.reverse_limit_m + 1e-9);
  ExpectEveryLegClear(q, capped);

  // Boxed in tighter (wall centre at x = 2.475): 0.15 m of reverse is allowed,
  // but then the REAR corners sweep into the wall behind when pivoting — no
  // fallback, the WEDGED path takes over.
  auto tight = costmap;
  MarkWallX(tight, 2.47, -1.0, 0.0);
  const TurnFallbackProblem t = Problem(tight, plan, robot);
  const TurnFallbackPlan none = mn::PlanTurnFallback(t, cfg);
  EXPECT_NEAR(none.reverse_limit_m, 0.15, 1e-9);
  EXPECT_EQ(none.verdict, TurnFallbackVerdict::kNoRejoin);
}

TEST(FtcTurnFallback, RejoinPoseMustBeFollowable)
{
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  const TurnFallbackPlan first = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  ASSERT_EQ(first.verdict, TurnFallbackVerdict::kPlanned);
  // The controller's lattice says it cannot follow from that rejoin: the next
  // followable one is taken instead, never the refused one.
  const std::size_t refused = first.rejoin;
  p.followable = [refused](std::size_t i)
  {
    return i != refused;
  };
  const TurnFallbackPlan second = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  ASSERT_EQ(second.verdict, TurnFallbackVerdict::kPlanned);
  EXPECT_GT(second.rejoin, refused);
  p.followable = [](std::size_t)
  {
    return false;
  };
  EXPECT_EQ(mn::PlanTurnFallback(p, TurnFallbackCfg{}).verdict, TurnFallbackVerdict::kNoRejoin);
}

TEST(FtcTurnFallback, NearHalfTurnRotationNeedsTheWholeTurnClear)
{
  // A rotation close to 180 deg may be executed either way round (the robot's
  // real heading on arrival decides): a lethal cell only on the LONG way round
  // must still refuse it.
  auto costmap = EmptyCostmap();
  const double x = 1.0;
  const double y = 0.0;
  const double from = 0.0;
  const double to = M_PI - 0.12;  // 173 deg counter-clockwise
  EXPECT_TRUE(mn::RotationSweepClear(costmap, {}, Footprint(), x, y, from, to));
  EXPECT_TRUE(mn::FullTurnClear(costmap, {}, Footprint(), x, y, from));
  // Behind-right of the robot: swept only when turning clockwise.
  MarkLethal(costmap, x + 0.05, y - 0.55);
  EXPECT_TRUE(mn::RotationSweepClear(costmap, {}, Footprint(), x, y, from, to));
  EXPECT_FALSE(mn::FullTurnClear(costmap, {}, Footprint(), x, y, from));
}

TEST(FtcTurnFallback, ANewObstacleOnAPlannedLegIsSeenByThePerTickChecks)
{
  // What FTC re-checks every cycle while executing: an obstacle that appears
  // after planning, on the connector or in a pivot sweep, reads blocked.
  auto costmap = EmptyCostmap();
  MarkWallX(costmap, 3.45, -1.5, 2.0);
  const auto plan = UTurnPlan(0.0, 3.0, 0.3, 0.0);
  const TurnFallbackProblem p = Problem(costmap, plan, {1.9, 0.0, 0.0});
  const TurnFallbackPlan fb = mn::PlanTurnFallback(p, TurnFallbackCfg{});
  ASSERT_EQ(fb.verdict, TurnFallbackVerdict::kPlanned);
  const double heading = fb.start.yaw + fb.rotate_start_rad;
  auto later = costmap;
  const double mid_x = 0.5 * (fb.start.x + fb.rejoin_pose.x);
  const double mid_y = 0.5 * (fb.start.y + fb.rejoin_pose.y);
  MarkLethal(later, mid_x, mid_y);
  EXPECT_FALSE(mn::StraightSweepClear(
      later, {}, Body(), {fb.start.x, fb.start.y, heading}, fb.rejoin_pose.x, fb.rejoin_pose.y));
  auto later2 = costmap;
  MarkLethal(later2, fb.start.x + 0.4 * std::cos(heading), fb.start.y + 0.4 * std::sin(heading));
  EXPECT_FALSE(mn::RotationSweepClear(
      later2, {}, Footprint(), fb.start.x, fb.start.y, fb.start.yaw, heading));
}

TEST(FtcTurnFallback, ZoneBandRefusesMotionOffThePlan)
{
  // The whole area beyond y = 0.9 is out of the zone. Any rotation whose body
  // AXIS would reach it is refused, while the plan itself is never zone-tested.
  auto costmap = EmptyCostmap();
  nav2_costmap_2d::Costmap2D zone(200, 200, kRes, -3.0, -5.0);
  for (double yy = 0.9; yy < 4.9; yy += kRes)
  {
    for (double xx = -2.9; xx < 6.9; xx += kRes)
    {
      MarkLethal(zone, xx, yy);
    }
  }
  mn::BoundaryGuard guard;
  guard.costmap = &zone;
  // At y = 0.6, a quarter turn to face +y puts the axis front at y = 1.14.
  EXPECT_FALSE(mn::RotationSweepClear(costmap, guard, Footprint(), 1.0, 0.6, M_PI, M_PI / 2));
  // At y = 0.0 the same turn keeps the axis front at y <= 0.54: inside.
  EXPECT_TRUE(mn::RotationSweepClear(costmap, guard, Footprint(), 1.0, 0.0, 0.0, M_PI / 2));
  // A straight whose axis would cross into the band is refused too.
  EXPECT_FALSE(mn::StraightSweepClear(costmap, guard, Body(), {1.0, 0.0, M_PI / 2}, 1.0, 0.5));
  EXPECT_TRUE(mn::StraightSweepClear(costmap, guard, Body(), {1.0, 0.0, 0.0}, 2.0, 0.0));
}

TEST(FtcTurnFallback, BadInputIsRefused)
{
  const auto costmap = EmptyCostmap();
  TurnFallbackProblem p;
  EXPECT_EQ(mn::PlanTurnFallback(p, TurnFallbackCfg{}).verdict, TurnFallbackVerdict::kBadInput);
  p.costmap = &costmap;
  p.plan = StraightPlan(0.0, 1.0);
  EXPECT_EQ(mn::PlanTurnFallback(p, TurnFallbackCfg{}).verdict, TurnFallbackVerdict::kBadInput);
}

// --- Recorded case: 2026-09-22 08:08:25 UTC (bag mow-20260922) -------------
//
// The first WEDGED of that mow: a 107 deg kink of the headland (idx 447 ->
// 448, no pivot twin) whose body front, at the corner pose, reaches three
// hedge cells 0.2 m past the recorded boundary (see test_ftc_lattice_solver's
// RecordedCornerIntoTheHedgeIsWedged, same /local_costmap/costmap sample,
// 1790064505.81). On the robot: reverse-escape 0.30 m, 2.5 s hold, strip
// aborted, FollowStrip stepped 0.8 m past it and transited blade-off.
// Replayed with FTC's own code (lattice_replay --fallback), the turn fallback
// pivots -64 deg where the robot stands, drives 0.98 m straight past the
// kink, pivots -45 deg and rejoins at idx 455, skipping 1.40 m of path.
// Data: the local costmap's lethal cells within 3 m of the robot, the zone
// band (global costmap >= 99) within 3 m as row runs, and the plan slice.

// local 240 x 240, 0.05 m, origin (-2.50000, -3.45000) odom
const std::vector<std::pair<unsigned int, unsigned int>> kLethal = {
    {126, 76},  {127, 76},  {128, 76},  {139, 79},  {131, 80},  {131, 81},  {132, 81},  {133, 81},
    {134, 81},  {148, 85},  {149, 85},  {132, 92},  {133, 92},  {133, 93},  {145, 95},  {145, 96},
    {137, 101}, {137, 102}, {138, 102}, {150, 103}, {151, 103}, {154, 105}, {153, 106}, {154, 106},
    {154, 107}, {154, 108}, {155, 109}, {156, 110}, {156, 111}, {155, 112}, {155, 113}, {158, 115},
    {157, 116}, {158, 116}, {157, 117}, {158, 117}, {161, 118}, {162, 119}, {162, 120}, {162, 121},
    {163, 121}, {164, 122}, {165, 123}, {166, 124}, {167, 125}, {167, 126}, {167, 127}, {169, 128},
    {168, 129}, {170, 129}, {169, 130}, {170, 130}, {169, 131}, {170, 131},
};  // 54 cells
// global 875 x 875, 0.08 m, origin (-38.64000, -17.92000) map; window i 400..475 j 400..475
constexpr unsigned int kZoneI0 = 400;
constexpr unsigned int kZoneJ0 = 400;
constexpr unsigned int kZoneW = 76;
constexpr unsigned int kZoneH = 76;
// Runs of zone cells (>= 99) per row: {row, first col, last col}, window-relative.
const std::vector<std::array<unsigned int, 3>> kZoneRuns = {
    {17, 66, 69}, {18, 65, 70}, {19, 65, 70}, {20, 65, 70}, {21, 65, 70}, {21, 75, 75},
    {22, 65, 71}, {22, 74, 75}, {23, 65, 71}, {23, 73, 75}, {24, 66, 71}, {24, 73, 75},
    {25, 66, 75}, {26, 67, 75}, {27, 67, 75}, {28, 68, 75}, {29, 68, 75}, {30, 68, 75},
    {31, 64, 75}, {32, 63, 75}, {33, 63, 75}, {34, 63, 75}, {35, 0, 0},   {35, 62, 75},
    {36, 0, 0},   {36, 61, 75}, {37, 0, 1},   {37, 61, 75}, {38, 0, 2},   {38, 28, 31},
    {38, 55, 58}, {38, 61, 75}, {39, 0, 3},   {39, 26, 33}, {39, 54, 59}, {39, 61, 75},
    {40, 0, 3},   {40, 26, 33}, {40, 54, 59}, {40, 61, 75}, {41, 0, 4},   {41, 24, 35},
    {41, 54, 75}, {42, 0, 4},   {42, 24, 35}, {42, 54, 75}, {43, 0, 5},   {43, 24, 35},
    {43, 50, 53}, {43, 55, 75}, {44, 0, 6},   {44, 24, 35}, {44, 49, 54}, {44, 58, 75},
    {45, 0, 7},   {45, 24, 40}, {45, 49, 54}, {45, 57, 75}, {46, 0, 7},   {46, 24, 41},
    {46, 49, 54}, {46, 56, 75}, {47, 0, 8},   {47, 25, 42}, {47, 50, 53}, {47, 56, 75},
    {48, 0, 8},   {48, 26, 42}, {48, 54, 75}, {49, 0, 9},   {49, 28, 43}, {49, 53, 75},
    {50, 0, 10},  {50, 32, 43}, {50, 52, 75}, {51, 0, 12},  {51, 32, 43}, {51, 51, 75},
    {52, 0, 12},  {52, 32, 43}, {52, 51, 75}, {53, 0, 14},  {53, 33, 42}, {53, 51, 75},
    {54, 0, 16},  {54, 33, 42}, {54, 50, 75}, {55, 0, 17},  {55, 34, 41}, {55, 47, 75},
    {56, 0, 19},  {56, 36, 39}, {56, 46, 75}, {57, 0, 20},  {57, 45, 75}, {58, 0, 22},
    {58, 44, 75}, {59, 0, 24},  {59, 43, 75}, {60, 0, 25},  {60, 43, 75}, {61, 0, 27},
    {61, 43, 75}, {62, 0, 28},  {62, 42, 75}, {63, 0, 30},  {63, 42, 75}, {64, 0, 32},
    {64, 41, 75}, {65, 0, 33},  {65, 40, 75}, {66, 0, 35},  {66, 40, 75}, {67, 0, 36},
    {67, 38, 75}, {68, 0, 75},  {69, 0, 75},  {70, 0, 75},  {71, 0, 75},  {72, 0, 75},
    {73, 0, 75},  {74, 0, 75},  {75, 0, 75},
};  // 123 runs
// FTC's plan idx 436..530 (map): x, y, yaw
const std::vector<std::array<double, 3>> kPlan = {
    {-3.52122, 17.05347, 0.883839},  {-3.48184, 17.10148, 0.883839},
    {-3.44246, 17.14949, 1.080113},  {-3.41470, 17.20146, 1.080113},
    {-3.38693, 17.25343, 0.294493},  {-3.32239, 17.27301, 0.294493},
    {-3.25785, 17.29259, 0.490821},  {-3.19600, 17.32564, 0.490821},
    {-3.13415, 17.35870, 0.687149},  {-3.08613, 17.39810, 0.687149},
    {-3.03812, 17.43749, 0.785314},  {-2.99723, 17.47838, 0.785314},
    {-2.95634, 17.51926, -1.088769}, {-2.91776, 17.44552, -1.088769},
    {-2.87919, 17.37179, -1.088769}, {-2.84061, 17.29805, -0.939588},
    {-2.78237, 17.21839, -0.939588}, {-2.72414, 17.13872, -0.939588},
    {-2.66590, 17.05905, -0.939588}, {-2.60767, 16.97938, -0.939588},
    {-2.54943, 16.89971, -0.939588}, {-2.49120, 16.82005, -0.939588},
    {-2.43296, 16.74038, -0.939588}, {-2.37473, 16.66071, -0.939588},
    {-2.31650, 16.58104, -0.939588}, {-2.25826, 16.50137, -0.939588},
    {-2.20003, 16.42171, -0.939588}, {-2.14179, 16.34204, -0.939588},
    {-2.08356, 16.26237, -0.939588}, {-2.02532, 16.18270, -0.939588},
    {-1.96709, 16.10303, -0.939588}, {-1.90885, 16.02337, -0.939588},
    {-1.85062, 15.94370, -0.939588}, {-1.79238, 15.86403, -0.939588},
    {-1.73415, 15.78436, -0.939588}, {-1.67591, 15.70469, -0.939588},
    {-1.61768, 15.62503, -0.852530}, {-1.55819, 15.55696, -0.852530},
    {-1.49869, 15.48889, -0.852530}, {-1.43920, 15.42082, -0.852530},
    {-1.37971, 15.35275, -0.852530}, {-1.32022, 15.28468, -0.852530},
    {-1.26072, 15.21661, -0.852530}, {-1.20123, 15.14854, -0.852530},
    {-1.14174, 15.08047, -0.736066}, {-1.07048, 15.01592, -0.736066},
    {-0.99923, 14.95137, -0.736066}, {-0.92797, 14.88682, -0.736066},
    {-0.85672, 14.82227, -0.736066}, {-0.78546, 14.75772, -0.736066},
    {-0.71421, 14.69317, -0.736066}, {-0.64295, 14.62862, -0.736066},
    {-0.57170, 14.56407, -0.736066}, {-0.50044, 14.49952, -0.736066},
    {-0.42919, 14.43497, -0.736066}, {-0.35793, 14.37042, -1.428451},
    {-0.34505, 14.28053, -1.428451}, {-0.33216, 14.19064, -1.428451},
    {-0.31928, 14.10074, -1.428451}, {-0.30640, 14.01085, -1.144441},
    {-0.26944, 13.92948, -1.144441}, {-0.23248, 13.84811, -1.144441},
    {-0.19552, 13.76674, -1.144441}, {-0.15856, 13.68537, -1.144441},
    {-0.12160, 13.60400, -1.144441}, {-0.08464, 13.52263, -0.510915},
    {-0.00240, 13.47653, -0.510915}, {0.07985, 13.43043, -0.510915},
    {0.16209, 13.38432, -0.510915},  {0.24433, 13.33822, -0.510915},
    {0.32657, 13.29212, -0.510915},  {0.40882, 13.24602, -0.510915},
    {0.49106, 13.19992, -0.801926},  {0.55708, 13.13167, -0.801926},
    {0.62311, 13.06343, -0.801926},  {0.68913, 12.99518, -0.801926},
    {0.75516, 12.92694, -0.801926},  {0.82118, 12.85870, -0.801926},
    {0.88721, 12.79045, -0.801926},  {0.95323, 12.72221, -0.801926},
    {1.01926, 12.65396, -0.801926},  {1.08528, 12.58572, -0.801926},
    {1.15130, 12.51748, -0.801926},  {1.21733, 12.44923, -0.801926},
    {1.28335, 12.38099, -0.801926},  {1.34938, 12.31274, -0.801926},
    {1.41540, 12.24450, -0.801926},  {1.48143, 12.17626, -0.801926},
    {1.54745, 12.10801, -0.980472},  {1.55121, 12.10240, -2.847052},
    {1.54989, 12.10200, -2.650691},  {1.48803, 12.06893, -2.650691},
    {1.42617, 12.03586, -2.454331},  {1.37815, 11.99645, -2.454331},
    {1.33013, 11.95704, -2.356151},
};
constexpr double kMapOdomX = -2.05628;
constexpr double kMapOdomY = 13.08677;
constexpr double kMapOdomYaw = 1.302257;
constexpr double kRobotX = 3.48912;  // odom -> base_footprint
constexpr double kRobotY = 2.53327;
constexpr double kRobotYaw = -0.339831;
constexpr double kLocalOriginX = -2.50000;
constexpr double kLocalOriginY = -3.45000;
constexpr double kGlobalOriginX = -38.64000;
constexpr double kGlobalOriginY = -17.92000;
constexpr double kGlobalRes = 0.08000;

nav2_costmap_2d::Costmap2D RecordedLocal()
{
  nav2_costmap_2d::Costmap2D costmap(240, 240, kRes, kLocalOriginX, kLocalOriginY);
  for (const auto& [mx, my] : kLethal)
  {
    costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  return costmap;
}

nav2_costmap_2d::Costmap2D RecordedZone()
{
  nav2_costmap_2d::Costmap2D zone(kZoneW,
                                  kZoneH,
                                  kGlobalRes,
                                  kGlobalOriginX + kZoneI0 * kGlobalRes,
                                  kGlobalOriginY + kZoneJ0 * kGlobalRes);
  for (const auto& [row, c0, c1] : kZoneRuns)
  {
    for (unsigned int c = c0; c <= c1; ++c)
    {
      zone.setCost(c, row, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  return zone;
}

Pose RecordedInOdom(const std::array<double, 3>& p)
{
  const double c = std::cos(kMapOdomYaw);
  const double s = std::sin(kMapOdomYaw);
  const double dx = p[0] - kMapOdomX;
  const double dy = p[1] - kMapOdomY;
  return MakePose(c * dx + s * dy, -s * dx + c * dy, p[2] - kMapOdomYaw);
}

TEST(FtcTurnFallback, RecordedKinkIntoTheHedgeIsImprovised)
{
  const auto local = RecordedLocal();
  const auto zone = RecordedZone();
  mn::BoundaryGuard guard;  // zone (map) <- costmap (odom) = map -> odom
  guard.costmap = &zone;
  guard.tx = kMapOdomX;
  guard.ty = kMapOdomY;
  guard.cos_yaw = std::cos(kMapOdomYaw);
  guard.sin_yaw = std::sin(kMapOdomYaw);

  std::vector<Pose> plan_map;
  std::vector<mn::PlanPose2D> plan2d;
  for (const auto& p : kPlan)
  {
    plan_map.push_back(MakePose(p[0], p[1], p[2]));
    plan2d.push_back({p[0], p[1], p[2]});
  }
  const auto corners = mn::FindPivotCorners(plan2d);

  // The window FTC builds: from the plan pose nearest the robot (idx 436,
  // the carrot was at 439) forward.
  const double c = std::cos(kMapOdomYaw);
  const double s = std::sin(kMapOdomYaw);
  const double robot_map_x = kMapOdomX + c * kRobotX - s * kRobotY;
  const double robot_map_y = kMapOdomY + s * kRobotX + c * kRobotY;
  const auto [first, last] =
      mn::FallbackWindow(plan_map, 3, robot_map_x, robot_map_y, mn::kFallbackRobotSearchBackM, 5.5);
  ASSERT_EQ(first, 0u);

  TurnFallbackProblem problem;
  problem.costmap = &local;
  problem.guard = guard;
  problem.footprint = Footprint();
  problem.body = Body();
  problem.robot = {kRobotX, kRobotY, kRobotYaw};
  for (std::size_t i = first; i < last; ++i)
  {
    problem.plan.push_back(RecordedInOdom(kPlan[i]));
  }
  // FTC's own followability test: its lattice, solved from the rejoin.
  mn::LatticeSolverCfg lattice;
  lattice.lattice.offset_step = 0.05;
  lattice.lattice.max_offset = 1.0;
  lattice.station_spacing_m = mn::OffsetLatticeStationSpacing(0.05, 1.0);
  lattice.lead_m = 0.30;
  lattice.reaction_m = 0.5;
  lattice.min_horizon_m = 1.0;
  problem.followable = [&](std::size_t w)
  {
    const std::size_t j = first + w;
    const auto [leg_first, leg_last] = mn::PivotLeg(corners, j, plan_map.size());
    return mn::LatticeFeasibleFrom(
        local,
        guard,
        problem.body,
        plan_map,
        j,
        leg_first,
        leg_last,
        mn::NextPivotCorner(corners, j),
        [&](const Pose& p)
        {
          return RecordedInOdom({p.pose.position.x, p.pose.position.y, YawOf(p)});
        },
        lattice,
        2.5);
  };

  const TurnFallbackPlan fb = mn::PlanTurnFallback(problem, TurnFallbackCfg{});
  ASSERT_EQ(fb.verdict, TurnFallbackVerdict::kPlanned) << fb.why;
  EXPECT_EQ(first + fb.blocked, 447u - 436u);  // the kink's corner pose
  EXPECT_NEAR(fb.turn_rad * 180.0 / M_PI, 107.0, 2.0);
  EXPECT_DOUBLE_EQ(fb.reverse_m, 0.0);
  EXPECT_EQ(first + fb.rejoin, 455u - 436u);
  EXPECT_NEAR(fb.skipped_arc_m, 1.40, 0.02);
  EXPECT_NEAR(fb.connector_m, 0.98, 0.02);
  EXPECT_NEAR(fb.rotate_start_rad * 180.0 / M_PI, -64.0, 2.0);
  EXPECT_NEAR(fb.rotate_rejoin_rad * 180.0 / M_PI, -45.0, 2.0);
  ExpectEveryLegClear(problem, fb);
  // Clear of the zone too: motion off the plan never enters the band.
  EXPECT_TRUE(mn::StraightSweepClear(local,
                                     guard,
                                     problem.body,
                                     {fb.start.x, fb.start.y, fb.start.yaw + fb.rotate_start_rad},
                                     fb.rejoin_pose.x,
                                     fb.rejoin_pose.y));
}

}  // namespace
