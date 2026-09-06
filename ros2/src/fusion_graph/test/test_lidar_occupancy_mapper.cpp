// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>

#include "fusion_graph/lidar_occupancy_mapper.hpp"
#include <gtest/gtest.h>

using fusion_graph::LidarOccupancyMapper;
using fusion_graph::LidarOccupancyMapperParams;

namespace
{
LidarOccupancyMapperParams SmallParams()
{
  LidarOccupancyMapperParams p;
  p.resolution_m = 0.1;
  p.half_extent_m = 5.0;
  return p;
}
}  // namespace

TEST(LidarOccupancyMapper, GridCoversTheConfiguredExtent)
{
  LidarOccupancyMapper m(SmallParams());
  EXPECT_EQ(m.size(), 100u);  // 10 m / 0.1 m
  const auto g = m.Export();
  EXPECT_EQ(g.width, 100u);
  EXPECT_NEAR(g.origin_x, -5.0, 1e-9);
  EXPECT_EQ(g.occupied, 0u);
  EXPECT_EQ(g.free, 0u);
}

// A wall 2 m ahead, seen from the origin looking +x: the wall cells become
// occupied, the cells between robot and wall become free, and cells beyond
// the wall stay unknown — the ray must not carve through what it hit.
TEST(LidarOccupancyMapper, WallAheadBecomesOccupiedAndSpaceBeforeItFree)
{
  LidarOccupancyMapper m(SmallParams());
  std::vector<std::pair<double, double>> pts;
  for (double y = -0.5; y <= 0.5; y += 0.05)
    pts.emplace_back(2.0, y);
  for (int i = 0; i < 3; ++i)  // three scans: enough to cross the thresholds
    m.Insert(0.0, 0.0, 0.0, pts);
  const auto g = m.Export();
  EXPECT_GT(g.occupied, 0u);
  EXPECT_GT(g.free, 0u);
  EXPECT_GT(m.LogOddsAt(2.0, 0.0), 0.0) << "wall cell must be occupied";
  EXPECT_LT(m.LogOddsAt(1.0, 0.0), 0.0) << "free space before the wall";
  EXPECT_DOUBLE_EQ(m.LogOddsAt(3.0, 0.0), 0.0) << "beyond the wall must remain untouched";
}

// The pose's yaw rotates body-frame points into the map frame.
TEST(LidarOccupancyMapper, InsertRespectsRobotYaw)
{
  LidarOccupancyMapper m(SmallParams());
  std::vector<std::pair<double, double>> pts{{2.0, 0.0}};
  for (int i = 0; i < 3; ++i)
    m.Insert(0.0, 0.0, M_PI / 2.0, pts);  // facing +y: the hit lands at (0, 2)
  EXPECT_GT(m.LogOddsAt(0.0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(m.LogOddsAt(2.0, 0.0), 0.0);
}

// A transient obstacle (a person) must not become permanent: after it leaves,
// free rays through the same cell pull it back below the occupied threshold.
TEST(LidarOccupancyMapper, CellsRecoverAfterATransientHit)
{
  LidarOccupancyMapper m(SmallParams());
  std::vector<std::pair<double, double>> person{{1.0, 0.0}};
  std::vector<std::pair<double, double>> wall{{3.0, 0.0}};
  for (int i = 0; i < 2; ++i)
    m.Insert(0.0, 0.0, 0.0, person);
  EXPECT_GT(m.LogOddsAt(1.0, 0.0), 0.0);
  for (int i = 0; i < 12; ++i)
    m.Insert(0.0, 0.0, 0.0, wall);  // rays now pass through (1,0)
  EXPECT_LT(m.LogOddsAt(1.0, 0.0), 0.0) << "the person's cell must recover to free";
  EXPECT_GT(m.LogOddsAt(3.0, 0.0), 0.0);
}

// Beams beyond max_range carve free space up to max_range and place no hit.
TEST(LidarOccupancyMapper, BeamBeyondMaxRangeIsMissOnly)
{
  auto p = SmallParams();
  p.max_range_m = 2.0;
  LidarOccupancyMapper m(p);
  std::vector<std::pair<double, double>> far{{4.0, 0.0}};
  for (int i = 0; i < 3; ++i)
    m.Insert(0.0, 0.0, 0.0, far);
  EXPECT_LT(m.LogOddsAt(1.0, 0.0), 0.0);
  EXPECT_LE(m.LogOddsAt(2.0, 0.0), 0.0) << "no hit may be placed at or beyond max_range";
  EXPECT_DOUBLE_EQ(m.LogOddsAt(4.0, 0.0), 0.0);
}

// Poses outside the grid are ignored rather than indexing out of bounds.
TEST(LidarOccupancyMapper, PoseOutsideGridIsIgnored)
{
  LidarOccupancyMapper m(SmallParams());
  std::vector<std::pair<double, double>> pts{{1.0, 0.0}};
  m.Insert(50.0, 50.0, 0.0, pts);
  EXPECT_EQ(m.inserted_scans(), 0u);
  EXPECT_EQ(m.Export().occupied, 0u);
}

// A scan placed at the true pose scores ~1; the same scan half a metre off
// scores low — this is the witness that distinguishes a lost filter.
TEST(LidarOccupancyMapper, ScoreScanSeparatesTruePoseFromShiftedPose)
{
  LidarOccupancyMapperParams p;
  p.resolution_m = 0.10;
  p.half_extent_m = 10.0;
  LidarOccupancyMapper m(p);
  // A straight wall 3 m ahead (x = 3, y in [-2, 2]) seen from the origin, 3 scans.
  std::vector<std::pair<double, double>> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05)
    wall.emplace_back(3.0, y);
  for (int i = 0; i < 3; ++i)
    m.Insert(0.0, 0.0, 0.0, wall);
  const auto at_truth = m.ScoreScan(0.0, 0.0, 0.0, wall);
  EXPECT_EQ(at_truth.total, static_cast<int>(wall.size()));
  EXPECT_GT(static_cast<double>(at_truth.hits) / at_truth.total, 0.95);
  const auto shifted = m.ScoreScan(0.5, 0.0, 0.0, wall);  // wall would be at x = 3.5
  EXPECT_LT(static_cast<double>(shifted.hits) / shifted.total, 0.2);
  const auto empty = m.ScoreScan(0.0, 0.0, 0.0, {});
  EXPECT_EQ(empty.total, 0);
}
