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
/**
 * @file test_detour_resume.cpp
 * @brief Unit tests for the pure detour-and-continue resume-pose search
 *        (detour_resume.hpp), the core of FollowStrip's "don't abandon the whole
 *        swath when FTC is blocked by an un-skirtable obstacle" behaviour.
 *
 * Covers: obstacle confirmation, resume-pose-found → detour, no-clear-resume →
 * fallback, no-obstacle → fallback, minimum skip distance respected, the
 * footprint clearance test (lethal / unknown / out-of-grid), and the structural
 * invariant that the resume pose is always far enough to force the blade-off
 * transit (min_skip > kSegmentTransitGap). Fully ROS-free: DetourCostmap is a
 * plain grid and poses are plain PoseStamped.
 */

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/detour_resume.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
#include <gtest/gtest.h>

using geometry_msgs::msg::PoseStamped;
using mowgli_behavior::decideDetour;
using mowgli_behavior::DetourCostmap;
using mowgli_behavior::DetourDecision;
using mowgli_behavior::DetourResumeCfg;
using mowgli_behavior::footprintClear;

namespace
{

// A 40x10 grid at 0.1 m/cell covering x∈[0,4], y∈[0,1], all-free by default.
DetourCostmap makeGrid()
{
  DetourCostmap cm;
  cm.origin_x = 0.0;
  cm.origin_y = 0.0;
  cm.resolution = 0.1;
  cm.width = 40;
  cm.height = 10;
  cm.data.assign(static_cast<std::size_t>(cm.width) * cm.height, 0);
  return cm;
}

// Mark every cell in the column range [col_lo, col_hi] (all rows) lethal — a
// wall spanning the field in Y at those X columns.
void addWall(DetourCostmap& cm, uint32_t col_lo, uint32_t col_hi, int8_t cost = 100)
{
  for (uint32_t row = 0; row < cm.height; ++row)
  {
    for (uint32_t col = col_lo; col <= col_hi && col < cm.width; ++col)
    {
      cm.data[static_cast<std::size_t>(row) * cm.width + col] = cost;
    }
  }
}

// A straight path along y=0.5, x = i*step for i in [0, n).
std::vector<PoseStamped> straightPath(std::size_t n, double step = 0.1, double y = 0.5)
{
  std::vector<PoseStamped> poses(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    poses[i].pose.position.x = static_cast<double>(i) * step;
    poses[i].pose.position.y = y;
  }
  return poses;
}

DetourResumeCfg cfg(double min_skip = 0.8, double radius = 0.25, double max_search = 8.0)
{
  DetourResumeCfg c;
  c.min_skip_dist_m = min_skip;
  c.footprint_radius_m = radius;
  c.max_search_dist_m = max_search;
  c.lethal_cost = 90;
  return c;
}

}  // namespace

// A wall midway → obstacle confirmed AND a clear resume pose past it is returned.
TEST(DetourResume, ResumeFoundPastObstacle)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 15, 17);  // x ≈ 1.5..1.75
  const auto poses = straightPath(40);

  const DetourDecision d = decideDetour(cm, poses, /*stuck=*/0, cfg());

  ASSERT_TRUE(d.obstacle_confirmed);
  ASSERT_TRUE(d.resume_idx.has_value());
  // Resume is PAST the wall and >= min_skip from the stuck pose (x=0).
  const double rx = poses[*d.resume_idx].pose.position.x;
  EXPECT_GT(rx, 1.75);
  EXPECT_GE(rx, 0.8);
}

// The clear approach BEFORE the obstacle is never chosen as the resume pose —
// resume must come AFTER a confirmed blockage.
TEST(DetourResume, ResumeIsAfterBlockageNotBefore)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 20, 22);  // x ≈ 2.0..2.25
  const auto poses = straightPath(40);

  const DetourDecision d = decideDetour(cm, poses, 0, cfg());

  ASSERT_TRUE(d.resume_idx.has_value());
  // Poses at x < 2.0 are clear and >= min_skip (e.g. x=1.0), but must be rejected
  // because the blockage has not been passed yet.
  EXPECT_GT(poses[*d.resume_idx].pose.position.x, 2.25);
}

// Obstacle extends to the end of the segment → no clear resume → fallback, but
// the obstacle is still CONFIRMED (so the caller knows it was obstacle-related).
TEST(DetourResume, NoClearResumeWhenObstacleRunsToEnd)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 15, 39);  // wall from x≈1.5 to the end
  const auto poses = straightPath(40);

  const DetourDecision d = decideDetour(cm, poses, 0, cfg());

  EXPECT_TRUE(d.obstacle_confirmed);
  EXPECT_FALSE(d.resume_idx.has_value());  // caller falls back to skip
}

// No lethal cell anywhere ahead → NOT obstacle-related → no detour (fallback).
TEST(DetourResume, NoObstacleMeansNoDetour)
{
  DetourCostmap cm = makeGrid();  // all free
  const auto poses = straightPath(40);

  const DetourDecision d = decideDetour(cm, poses, 0, cfg());

  EXPECT_FALSE(d.obstacle_confirmed);
  EXPECT_FALSE(d.resume_idx.has_value());
}

// The minimum skip distance is respected even when a clear pose exists just past
// a nearby obstacle: the resume pose is never closer than min_skip to the stuck
// pose (so the robot actually clears the obstacle instead of re-hitting it).
TEST(DetourResume, MinSkipDistanceRespected)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 2, 3);  // obstacle very close to the start (x≈0.2..0.3)
  const auto poses = straightPath(40);

  // A clear pose exists at x≈0.55 right after the wall, but min_skip=0.8 forbids
  // resuming before x=0.8.
  const DetourDecision d = decideDetour(cm, poses, 0, cfg(/*min_skip=*/0.8));

  ASSERT_TRUE(d.resume_idx.has_value());
  EXPECT_GE(poses[*d.resume_idx].pose.position.x, 0.8);
}

// An invalid/empty costmap yields no confirmation and no resume → fallback
// (never detour blind).
TEST(DetourResume, InvalidCostmapFallsBack)
{
  DetourCostmap cm;  // default: width=0 → invalid
  const auto poses = straightPath(40);

  const DetourDecision d = decideDetour(cm, poses, 0, cfg());

  EXPECT_FALSE(d.obstacle_confirmed);
  EXPECT_FALSE(d.resume_idx.has_value());
}

// A blockage beyond the bounded search window is not reached → treated as no
// obstacle (fallback), never an unbounded scan.
TEST(DetourResume, SearchWindowIsBounded)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 38, 39);  // wall at x≈3.8, far from stuck pose
  const auto poses = straightPath(40);

  // Cap the search at 1.0 m: the wall at 3.8 m is never scanned.
  const DetourDecision d = decideDetour(cm,
                                        poses,
                                        0,
                                        cfg(/*min_skip=*/0.8,
                                            /*radius=*/0.25,
                                            /*max_search=*/1.0));

  EXPECT_FALSE(d.obstacle_confirmed);
  EXPECT_FALSE(d.resume_idx.has_value());
}

// ---------------------------------------------------------------------------
// footprintClear
// ---------------------------------------------------------------------------

TEST(FootprintClear, LethalCellBlocks)
{
  DetourCostmap cm = makeGrid();
  addWall(cm, 20, 20);  // single lethal column at x≈2.05
  // A pose centred on the lethal cell with a footprint covering it is NOT clear.
  EXPECT_FALSE(footprintClear(cm, 2.05, 0.5, 0.25, 90));
  // A pose far from any lethal cell IS clear.
  EXPECT_TRUE(footprintClear(cm, 0.5, 0.5, 0.25, 90));
}

TEST(FootprintClear, UnknownAndSubLethalAreClear)
{
  DetourCostmap cm = makeGrid();
  // Unknown (-1) and a sub-lethal inflation cost (50) must NOT block.
  cm.data[static_cast<std::size_t>(5) * cm.width + 5] = -1;
  cm.data[static_cast<std::size_t>(5) * cm.width + 6] = 50;
  EXPECT_TRUE(footprintClear(cm, 0.55, 0.55, 0.1, 90));
}

TEST(FootprintClear, OutOfGridIsClear)
{
  DetourCostmap cm = makeGrid();
  // A pose well outside the grid bounds has no lethal cells → clear (unmapped).
  EXPECT_TRUE(footprintClear(cm, 100.0, 100.0, 0.25, 90));
}

// ---------------------------------------------------------------------------
// Structural invariant: the resume pose is always far enough to force the
// blade-off transit (min_skip > kSegmentTransitGap), so the detour crossing is
// provably blade-OFF (it goes through sendCurrentSwath's gap guard).
// ---------------------------------------------------------------------------
TEST(DetourResume, MinSkipExceedsTransitGap)
{
  EXPECT_GT(DetourResumeCfg{}.min_skip_dist_m,
            mowgli_interfaces::coverage_geometry::kSegmentTransitGapM);
}
