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
// Tests for the costmap side of FTC's offset lattice (ftc_lattice_solver.hpp):
// the node tests (real footprint vs raw lethal LOCAL cells; the zone guard only
// off the planned line) and the degrading solve FTCController runs every tick.
// ROS-free: synthetic Costmap2D grids plus one case recorded on the robot.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{

using mowgli_nav2_plugins::BoundaryGuard;
using mowgli_nav2_plugins::LatticeBlock;
using mowgli_nav2_plugins::LatticeSolution;
using mowgli_nav2_plugins::LatticeSolver;
using mowgli_nav2_plugins::LatticeSolverCfg;
using mowgli_nav2_plugins::LatticeWindow;
using mowgli_nav2_plugins::ObstacleDeviation;
using mowgli_nav2_plugins::OffsetLatticeStationSpacing;
using mowgli_nav2_plugins::ResampleLatticeWindow;
using Pose = geometry_msgs::msg::PoseStamped;

constexpr double kRes = 0.05;
constexpr double kLead = 0.30;  // CarrotMaxLead(0, speed_fast 0.20, kp_lon 1.0)
constexpr double kHorizon = 2.5;

Pose MakePose(double x, double y, double yaw)
{
  Pose p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

/// FTC's lattice body on the robot: getRobotFootprint() (chassis 0.60 x 0.45 m
/// centred 0.18 m ahead of base_link, + the 0.05 m chassis_footprint margin,
/// + Nav2's default footprint_padding 0.01), widened by obstacle_clearance_margin
/// 0.05 exactly as updateLateralDeviation() does.
ObstacleDeviation::Footprint RobotBody()
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
  return ObstacleDeviation::expandFootprintLateral(fp, 0.05);
}

/// Production parameters (nav2_params_base.yaml + the template injections).
LatticeSolverCfg RobotCfg()
{
  LatticeSolverCfg cfg;
  cfg.lattice.offset_step = 0.05;
  cfg.lattice.max_offset = 1.0;  // max_obstacle_avoidance_distance
  cfg.station_spacing_m = OffsetLatticeStationSpacing(0.05, 1.0);
  cfg.lead_m = kLead;
  cfg.reaction_m = 0.5;
  cfg.min_horizon_m = 1.0;
  return cfg;
}

/// Straight path along +x from x0 to x1 at y, one pose per `step` (> the 0.05 m
/// station spacing, so every pose is a station, as on the robot's plans).
constexpr double kStep = 0.06;
std::vector<Pose> StraightPath(double x0, double x1, double y, double step = kStep)
{
  std::vector<Pose> path;
  for (double x = x0; x <= x1 + 1e-9; x += step)
  {
    path.push_back(MakePose(x, y, 0.0));
  }
  return path;
}

void MarkLethal(nav2_costmap_2d::Costmap2D& costmap, double x, double y)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(x, y, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
}

/// What FTCController::planOffsetLattice feeds the solver, for a plan already
/// in the costmap frame and no pivot corner.
LatticeSolver SolverAt(const nav2_costmap_2d::Costmap2D& costmap,
                       const std::vector<Pose>& plan,
                       std::size_t carrot_idx,
                       const BoundaryGuard& guard = {})
{
  const LatticeSolverCfg cfg = RobotCfg();
  const LatticeWindow w = ResampleLatticeWindow(
      plan, carrot_idx, 0, plan.size() - 1, std::nullopt, cfg.station_spacing_m, kLead, kHorizon);
  std::vector<Pose> poses;
  for (const std::size_t i : w.pose_idx)
  {
    poses.push_back(plan[i]);
  }
  return LatticeSolver(
      costmap, guard, RobotBody(), std::move(poses), w.carrot_pos, w.corner_is_last_station, cfg);
}

TEST(FtcLatticeSolver, WindowSpansTheLeadBehindAndTheHorizonAhead)
{
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  const LatticeWindow w =
      ResampleLatticeWindow(plan, 20, 0, plan.size() - 1, std::nullopt, 0.05, kLead, kHorizon);
  ASSERT_FALSE(w.pose_idx.empty());
  EXPECT_EQ(w.pose_idx[w.carrot_pos], 20u);
  // lead 0.30 m behind the carrot, horizon 2.5 m ahead (to within one pose).
  EXPECT_NEAR(plan[20].pose.position.x - plan[w.pose_idx.front()].pose.position.x, kLead, kStep);
  EXPECT_NEAR(plan[w.pose_idx.back()].pose.position.x - plan[20].pose.position.x, kHorizon, kStep);
  EXPECT_FALSE(w.corner_is_last_station);
}

TEST(FtcLatticeSolver, WindowStopsAtAPivotCornerWhichBecomesTheLastStation)
{
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  const std::size_t corner = 40;  // 1.2 m past the carrot
  const LatticeWindow w = ResampleLatticeWindow(plan, 20, 0, corner, corner, 0.05, kLead, kHorizon);
  EXPECT_EQ(w.pose_idx.back(), corner);
  EXPECT_TRUE(w.corner_is_last_station);
}

TEST(FtcLatticeSolver, ClearPathKeepsTheLine)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 20);
  const LatticeSolution sol = solver.Solve(0.0, 0, 0);
  ASSERT_TRUE(sol.plan.feasible);
  EXPECT_EQ(sol.level, 1);
  EXPECT_EQ(sol.planned_stations, solver.Stations().size());
  EXPECT_DOUBLE_EQ(sol.plan.MaxAbsOffset(), 0.0);
}

TEST(FtcLatticeSolver, SkirtsAPostOnTheLine)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  MarkLethal(costmap, 3.3, 0.0);  // 1.5 m ahead of the carrot at x = 1.8
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 30);
  const LatticeSolution sol = solver.Solve(0.0, 0, 0);
  ASSERT_TRUE(sol.plan.feasible);
  // The body (0.335 m half-width) has to clear the post.
  EXPECT_GE(sol.plan.MaxAbsOffset(), 0.335);
}

TEST(FtcLatticeSolver, AWallFarAheadOnlyCutsTheHorizon)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  for (double y = -1.6; y <= 1.6; y += kRes)
  {
    MarkLethal(costmap, 4.0, y);  // 2.2 m ahead of the carrot, the whole lattice wide
  }
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 30);
  const LatticeSolution sol = solver.Solve(0.0, 0, 0);
  ASSERT_TRUE(sol.plan.feasible);
  EXPECT_LT(sol.planned_stations, solver.Stations().size());
  EXPECT_DOUBLE_EQ(sol.plan.MaxAbsOffset(), 0.0);
}

TEST(FtcLatticeSolver, AWallInsideTheMinimumHorizonIsWedged)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  for (double y = -1.6; y <= 1.6; y += kRes)
  {
    MarkLethal(costmap, 2.6, y);  // 0.8 m ahead of the carrot: inside the 1 m minimum horizon
  }
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 30);
  EXPECT_FALSE(solver.Solve(0.0, 0, 0).plan.feasible);
}

TEST(FtcLatticeSolver, ZoneIsNeverTestedOnThePlannedLine)
{
  // The whole plan lies inside a zone band (e.g. the soft edge of a drawn
  // obstacle): the plan is authoritative, so the line stays feasible, while
  // leaving it into the band is refused.
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  nav2_costmap_2d::Costmap2D zone(200, 200, kRes, -2.0, -5.0);
  for (unsigned int mx = 0; mx < 200; ++mx)
  {
    for (unsigned int my = 0; my < 200; ++my)
    {
      zone.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  BoundaryGuard guard;
  guard.costmap = &zone;
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 20, guard);
  EXPECT_EQ(solver.PoseBlock(solver.CarrotPos() + 5, 0.0), LatticeBlock::kFree);
  EXPECT_EQ(solver.PoseBlock(solver.CarrotPos() + 5, 0.05), LatticeBlock::kZone);
  const LatticeSolution sol = solver.Solve(0.0, 0, 0);
  ASSERT_TRUE(sol.plan.feasible);
  EXPECT_DOUBLE_EQ(sol.plan.MaxAbsOffset(), 0.0);
}

TEST(FtcLatticeSolver, ObstacleTestWinsOverTheZoneAndNamesTheCause)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, kRes, -2.0, -5.0);
  MarkLethal(costmap, 2.0, 0.0);
  const auto plan = StraightPath(0.0, 5.0, 0.0);
  LatticeSolver solver = SolverAt(costmap, plan, 30);
  // The carrot pose, 0.2 m before the post: the body front (0.54 m) covers it.
  const std::size_t pose = solver.CarrotPos();
  ASSERT_NEAR(solver.Poses()[pose].pose.position.x, 1.8, 1e-6);
  EXPECT_EQ(solver.PoseBlock(pose, 0.0), LatticeBlock::kObstacle);
  EXPECT_EQ(solver.PoseBlock(pose, 0.40), LatticeBlock::kFree);
}

// --- Recorded case: the first of the 2026-09-22 WEDGED events ----------------
//
// 08:08:25 UTC (bag mow-20260922, split 0): "WEDGED (no collision-free offset
// profile within the lattice)", reverse-escape, 2.5 s hold, strip aborted. The
// published costmaps were thought to show nothing on the planned line. They
// do: this plan turns 107 deg between two poses 0.06 m apart (idx 447 -> 448),
// and at the corner pose the body front reaches 0.54 m along the incoming
// heading, onto three lethal cells that the LiDAR saw in 10 of 11 frames over
// +-3 s, 0.2 m beyond the recorded boundary (keepout soft band). Past the
// corner the left side is lethal too. Replaying the solver on the recorded
// /local_costmap/costmap reproduces the WEDGED, and the zone guard is not
// needed for it (it is left out here). Pinned so the lattice keeps refusing to
// drive its body into real lethal cells at a turn.

// /local_costmap/costmap at 1790064505.81: 240 x 240, 0.05 m, origin
// (-2.50, -3.45) in odom; the lethal cells within 1.6 m of the carrot.
const std::vector<std::pair<unsigned int, unsigned int>> kRecordedLethal = {
    {132, 92},  {133, 92},  {133, 93},  {145, 95},  {145, 96},  {137, 101},  // }
    {137, 102},  // } the cells the corner pose's body front covers
    {138, 102},  // }
    {150, 103}, {151, 103}, {154, 105}, {153, 106}, {154, 106}, {154, 107},
    {154, 108}, {155, 109}, {156, 110}, {156, 111}, {155, 112}, {155, 113},
};

// FTC's plan (controller_server/FollowCoveragePath/global_plan), idx 430..482,
// map frame: x, y, yaw. The carrot sat between idx 439 and 440.
const std::vector<std::array<double, 3>> kRecordedPlan = {
    {-3.879, 16.771, 0.4911},  {-3.817, 16.804, 0.4911},  {-3.755, 16.837, 0.6875},
    {-3.707, 16.876, 0.6875},  {-3.659, 16.916, 0.7857},  {-3.590, 16.985, 0.7857},
    {-3.521, 17.053, 0.8838},  {-3.482, 17.101, 0.8838},  {-3.442, 17.149, 1.0801},
    {-3.415, 17.201, 1.0801},  {-3.387, 17.253, 0.2945},  {-3.322, 17.273, 0.2945},
    {-3.258, 17.293, 0.4908},  {-3.196, 17.326, 0.4908},  {-3.134, 17.359, 0.6871},
    {-3.086, 17.398, 0.6871},  {-3.038, 17.437, 0.7853},  {-2.997, 17.478, 0.7853},
    {-2.956, 17.519, -1.0888}, {-2.918, 17.446, -1.0888}, {-2.879, 17.372, -1.0888},
    {-2.841, 17.298, -0.9396}, {-2.782, 17.218, -0.9396}, {-2.724, 17.139, -0.9396},
    {-2.666, 17.059, -0.9396}, {-2.608, 16.979, -0.9396}, {-2.549, 16.900, -0.9396},
    {-2.491, 16.820, -0.9396}, {-2.433, 16.740, -0.9396}, {-2.375, 16.661, -0.9396},
    {-2.317, 16.581, -0.9396}, {-2.258, 16.501, -0.9396}, {-2.200, 16.422, -0.9396},
    {-2.142, 16.342, -0.9396}, {-2.084, 16.262, -0.9396}, {-2.025, 16.183, -0.9396},
    {-1.967, 16.103, -0.9396}, {-1.909, 16.023, -0.9396}, {-1.851, 15.944, -0.9396},
    {-1.792, 15.864, -0.9396}, {-1.734, 15.784, -0.9396}, {-1.676, 15.705, -0.9396},
    {-1.618, 15.625, -0.8525}, {-1.558, 15.557, -0.8525}, {-1.499, 15.489, -0.8525},
    {-1.439, 15.421, -0.8525}, {-1.380, 15.353, -0.8525}, {-1.320, 15.285, -0.8525},
    {-1.261, 15.217, -0.8525}, {-1.201, 15.149, -0.8525}, {-1.142, 15.080, -0.7361},
    {-1.070, 15.016, -0.7361}, {-0.999, 14.951, -0.7361},
};
constexpr std::size_t kRecordedCarrot = 9;  // idx 439
// map -> odom at that tick (fusion_graph): x, y, yaw.
constexpr double kMapOdomX = -2.05628;
constexpr double kMapOdomY = 13.08677;
constexpr double kMapOdomYaw = 1.302257;

/// The recorded local costmap, optionally without some of its lethal cells.
nav2_costmap_2d::Costmap2D RecordedCostmap(
    const std::vector<std::pair<unsigned int, unsigned int>>& drop = {})
{
  nav2_costmap_2d::Costmap2D costmap(240, 240, kRes, -2.50, -3.45);
  for (const auto& cell : kRecordedLethal)
  {
    if (std::find(drop.begin(), drop.end(), cell) == drop.end())
    {
      costmap.setCost(cell.first, cell.second, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  return costmap;
}

/// The recorded plan expressed in the costmap (odom) frame, as
/// planWindowInCostmapFrame() does with the latest map -> odom.
std::vector<Pose> RecordedPlanInOdom()
{
  const double c = std::cos(kMapOdomYaw);
  const double s = std::sin(kMapOdomYaw);
  std::vector<Pose> plan;
  for (const auto& p : kRecordedPlan)
  {
    const double dx = p[0] - kMapOdomX;
    const double dy = p[1] - kMapOdomY;
    plan.push_back(MakePose(c * dx + s * dy, -s * dx + c * dy, p[2] - kMapOdomYaw));
  }
  return plan;
}

TEST(FtcLatticeSolver, RecordedCornerIntoTheHedgeIsWedged)
{
  const auto costmap = RecordedCostmap();
  const auto plan = RecordedPlanInOdom();
  LatticeSolver solver = SolverAt(costmap, plan, kRecordedCarrot);
  // The corner pose (idx 447) itself: its body covers a lethal cell.
  std::size_t corner = 0;
  for (std::size_t i = 0; i < solver.Poses().size(); ++i)
  {
    if (std::hypot(solver.Poses()[i].pose.position.x - plan[17].pose.position.x,
                   solver.Poses()[i].pose.position.y - plan[17].pose.position.y) < 1e-9)
    {
      corner = i;
    }
  }
  ASSERT_GT(corner, solver.CarrotPos());
  EXPECT_EQ(solver.PoseBlock(corner, 0.0), LatticeBlock::kObstacle);
  // FTC applied +0.05 m, committed to the left.
  EXPECT_FALSE(solver.Solve(0.05, 1, 0).plan.feasible);
  EXPECT_FALSE(solver.Solve(0.0, 0, 0).plan.feasible);
}

TEST(FtcLatticeSolver, RecordedCornerWithoutTheHedgeCellsIsDriven)
{
  const auto costmap = RecordedCostmap({{137, 101}, {137, 102}, {138, 102}});
  const auto plan = RecordedPlanInOdom();
  LatticeSolver solver = SolverAt(costmap, plan, kRecordedCarrot);
  const LatticeSolution sol = solver.Solve(0.05, 1, 0);
  ASSERT_TRUE(sol.plan.feasible);
  EXPECT_LE(sol.plan.MaxAbsOffset(), 0.10);
}

}  // namespace
