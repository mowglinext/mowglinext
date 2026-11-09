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
// Tests for the Bug2-style contour bypass prototype (ftc_contour_bypass.hpp).
// ROS-free: synthetic Costmap2D grids, the robot's real footprint and lattice
// body. The scenarios are the ones the turn fallback does NOT handle: an
// obstacle on a straight wider than the lattice's lateral reach, an obstacle
// leaning on the recorded boundary, a lawn cut in two.

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_contour_bypass.hpp"
#include "mowgli_nav2_plugins/ftc_turn_fallback.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

namespace mn = mowgli_nav2_plugins;
using mn::ContourBypassCfg;
using mn::ContourBypassPlan;
using mn::ContourBypassProblem;
using mn::ContourBypassVerdict;
using mn::FallbackPose;
using mn::ObstacleDeviation;
using Pose = geometry_msgs::msg::PoseStamped;

constexpr double kRes = 0.05;
constexpr double kStep = 0.05;

Pose MakePose(double x, double y, double yaw)
{
  Pose p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

/// getRobotFootprint(): 0.60 x 0.45 chassis 0.18 m ahead of base_link + 0.06.
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

ObstacleDeviation::Footprint Body()
{
  return ObstacleDeviation::expandFootprintLateral(Footprint(), 0.05);
}

/// 10 x 10 m grid centred on the origin.
struct World
{
  nav2_costmap_2d::Costmap2D local{200, 200, kRes, -5.0, -5.0};
  nav2_costmap_2d::Costmap2D zone{200, 200, kRes, -5.0, -5.0};

  static void Fill(nav2_costmap_2d::Costmap2D& cm, double x0, double y0, double x1, double y1)
  {
    for (double x = x0; x <= x1 + 1e-9; x += kRes / 2.0)
    {
      for (double y = y0; y <= y1 + 1e-9; y += kRes / 2.0)
      {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (cm.worldToMap(x, y, mx, my))
        {
          cm.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  void Obstacle(double x0, double y0, double x1, double y1)
  {
    Fill(local, x0, y0, x1, y1);
  }
  void Zone(double x0, double y0, double x1, double y1)
  {
    Fill(zone, x0, y0, x1, y1);
  }
};

std::vector<Pose> StraightPlan(double x0, double x1)
{
  std::vector<Pose> plan;
  for (double x = x0; x <= x1 + 1e-9; x += kStep)
  {
    plan.push_back(MakePose(x, 0.0, 0.0));
  }
  return plan;
}

ContourBypassProblem Problem(World& w, bool with_zone)
{
  ContourBypassProblem p;
  p.costmap = &w.local;
  if (with_zone)
  {
    p.guard.costmap = &w.zone;  // identity transform: zone grid in the same frame
  }
  p.footprint = Footprint();
  p.body = Body();
  p.plan = StraightPlan(-3.0, 4.5);
  p.robot = {-3.0, 0.0, 0.0};
  return p;
}

/// Every leg of the chosen motion is clear for the BARE shapes (what the
/// executing controller would check each cycle).
void ExpectLegsClear(const World& w, const ContourBypassProblem& p, const mn::ContourSide& s)
{
  ASSERT_GE(s.legs.size(), 2u);
  for (std::size_t k = 0; k + 1 < s.legs.size(); ++k)
  {
    const FallbackPose& a = s.legs[k];
    const FallbackPose& b = s.legs[k + 1];
    EXPECT_TRUE(mn::RotationSweepClear(w.local, p.guard, p.footprint, a.x, a.y, a.yaw, b.yaw))
        << "pivot " << k;
    EXPECT_TRUE(mn::StraightSweepClear(w.local, p.guard, p.body, {a.x, a.y, b.yaw}, b.x, b.y))
        << "leg " << k;
  }
  EXPECT_GT(s.min_clearance_m, 0.0);
}

TEST(ContourBypass, NoObstacleIsNotItsCase)
{
  World w;
  const auto plan = mn::PlanContourBypass(Problem(w, false), ContourBypassCfg{});
  EXPECT_EQ(plan.verdict, ContourBypassVerdict::kNoBlockage);
}

TEST(ContourBypass, MissingInputsAreRejected)
{
  ContourBypassProblem p;
  EXPECT_EQ(mn::PlanContourBypass(p, ContourBypassCfg{}).verdict, ContourBypassVerdict::kBadInput);
}

// A 3.0 m wide bed across a straight swath, 1.0 m deep: wider than the
// lattice's lateral reach (max_lateral_deviation 1.0-1.5 m each side of an
// off-centre obstacle) — today WEDGED -> abort -> blade-off detour.
TEST(ContourBypass, WideObstacleOnAStraightIsWalkedRoundAndRejoined)
{
  World w;
  w.Obstacle(-1.0, -1.0, 0.0, 2.0);
  const ContourBypassProblem p = Problem(w, false);
  const ContourBypassPlan plan = mn::PlanContourBypass(p, ContourBypassCfg{});

  ASSERT_EQ(plan.verdict, ContourBypassVerdict::kPlanned)
      << plan.left.why << " / " << plan.right.why;
  const mn::ContourSide* best = plan.Best();
  ASSERT_NE(best, nullptr);
  // The short way round is the side where the bed ends sooner (right, y < 0).
  EXPECT_EQ(plan.chosen, -1);
  EXPECT_TRUE(plan.left.reached);
  // Rejoins the swath past the bed, on the line.
  const auto& rejoin = p.plan[best->rejoin].pose.position;
  EXPECT_GT(rejoin.x, 0.0);
  EXPECT_NEAR(best->legs.back().y, 0.0, 1e-9);
  EXPECT_LT(best->skipped_arc_m, 3.5);
  EXPECT_LE(best->pivots, 8u);
  ExpectLegsClear(w, p, *best);

  // The turn fallback does not take this case (obstacle on a straight): today
  // it goes WEDGED -> reverse -> hold -> abort -> FollowStrip blade-off detour.
  ContourBypassProblem from_robot = p;
  from_robot.robot = mn::FallbackPose{-2.0, 0.0, 0.0};
  from_robot.plan = StraightPlan(-2.0, 4.5);
  EXPECT_EQ(mn::PlanTurnFallback(from_robot, mn::TurnFallbackCfg{}).verdict,
            mn::TurnFallbackVerdict::kNotATurn);
  std::printf(
      "[ contour  ] wide bed: walk %.2f m, %zu legs %.2f m, %zu pivots %.0f deg, "
      "skipped %.2f m, clearance %.2f m, ~%.1f s\n",
      best->walk_m,
      best->legs.size() - 1,
      best->legs_m,
      best->pivots,
      best->rotation_rad * 180.0 / M_PI,
      best->skipped_arc_m,
      best->min_clearance_m,
      best->est_time_s);
}

// The same bed leaning on the recorded boundary (zone band y >= 1.2): only the
// open side may be walked; the boundary is a virtual wall, never crossed.
TEST(ContourBypass, BoundaryIsAVirtualWall)
{
  World w;
  w.Obstacle(-1.0, -0.6, 0.0, 1.2);
  w.Zone(-5.0, 1.2, 5.0, 5.0);
  const ContourBypassProblem p = Problem(w, true);
  const ContourBypassPlan plan = mn::PlanContourBypass(p, ContourBypassCfg{});

  ASSERT_EQ(plan.verdict, ContourBypassVerdict::kPlanned) << plan.right.why;
  EXPECT_FALSE(plan.left.reached);
  EXPECT_EQ(plan.chosen, -1);
  ExpectLegsClear(w, p, plan.right);
  for (const auto& q : plan.right.walk)
  {
    EXPECT_FALSE(p.guard.isLethalAt(q.x, q.y));
  }
}

// A wall across the whole lawn, closed on both sides by the boundary: Bug2's
// "unreachable" — both walks fail, nothing is planned.
TEST(ContourBypass, LawnCutInTwoIsUnreachable)
{
  World w;
  w.Obstacle(-0.5, -5.0, 0.0, 5.0);
  w.Zone(-5.0, 2.0, 5.0, 5.0);
  w.Zone(-5.0, -5.0, 5.0, -2.0);
  ContourBypassCfg cfg;
  cfg.max_contour_m = 20.0;
  const ContourBypassPlan plan = mn::PlanContourBypass(Problem(w, true), cfg);

  EXPECT_EQ(plan.verdict, ContourBypassVerdict::kNoLeave);
  EXPECT_FALSE(plan.left.why.empty());
  EXPECT_FALSE(plan.right.why.empty());
}

// The caller can veto a rejoin (FTC could not follow from there): the walk
// keeps going round until the next acceptable pose.
TEST(ContourBypass, FollowablePredicateMovesTheRejoin)
{
  World w;
  w.Obstacle(-1.0, -0.4, -0.4, 0.4);
  ContourBypassProblem p = Problem(w, false);
  const ContourBypassPlan free_plan = mn::PlanContourBypass(p, ContourBypassCfg{});
  ASSERT_EQ(free_plan.verdict, ContourBypassVerdict::kPlanned);
  const std::size_t first = free_plan.Best()->rejoin;

  p.followable = [first](std::size_t j)
  {
    return j > first + 10;
  };
  const ContourBypassPlan vetoed = mn::PlanContourBypass(p, ContourBypassCfg{});
  if (vetoed.verdict == ContourBypassVerdict::kPlanned)
  {
    EXPECT_GT(vetoed.Best()->rejoin, first + 10);
  }
}

}  // namespace
