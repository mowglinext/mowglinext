// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_nav2_plugins/obstacle_deviation.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mowgli_nav2_plugins
{

// ── Fixtures ──────────────────────────────────────────────────────────────────

class ObstacleDeviationTest : public ::testing::Test
{
protected:
  // 20 m × 20 m costmap centred at origin, 0.05 m resolution → 400×400 cells.
  // Origin at (-10, -10) so world (0,0) is the costmap centre.
  static constexpr unsigned int kSize = 400;
  static constexpr double kResolution = 0.05;
  static constexpr double kOriginX = -10.0;
  static constexpr double kOriginY = -10.0;

  nav2_costmap_2d::Costmap2D costmap_{kSize, kSize, kResolution, kOriginX, kOriginY,
                                      nav2_costmap_2d::FREE_SPACE};

  /// Stamp a square block of LETHAL cells centred on (cx, cy) with half-side
  /// `half` metres.
  void stampBlock(double cx, double cy, double half)
  {
    unsigned int mx0 = 0;
    unsigned int my0 = 0;
    unsigned int mx1 = 0;
    unsigned int my1 = 0;
    ASSERT_TRUE(costmap_.worldToMap(cx - half, cy - half, mx0, my0));
    ASSERT_TRUE(costmap_.worldToMap(cx + half, cy + half, mx1, my1));
    for (unsigned int x = mx0; x <= mx1; ++x)
    {
      for (unsigned int y = my0; y <= my1; ++y)
      {
        costmap_.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  /// Build a straight horizontal path (along +X) from (start_x, y) for n
  /// poses spaced `step` apart. All poses face +X (yaw=0).
  std::vector<geometry_msgs::msg::PoseStamped>
  makeStraightPath(double start_x, double y, std::size_t n, double step)
  {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    path.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = start_x + static_cast<double>(i) * step;
      p.pose.position.y = y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      p.pose.orientation = tf2::toMsg(q);
      path.push_back(p);
    }
    return path;
  }
};

// ── findFirstObstacleIndex ────────────────────────────────────────────────────

TEST_F(ObstacleDeviationTest, FindFirstObstacle_NoObstacle_ReturnsMinusOne)
{
  // Empty costmap, straight path. Should find no obstacle.
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const int idx = ObstacleDeviation::findFirstObstacleIndex(costmap_, path, 0, 10);
  EXPECT_EQ(idx, -1);
}

TEST_F(ObstacleDeviationTest, FindFirstObstacle_BlockOnPath_ReturnsCorrectIndex)
{
  // Block centred at (0.5, 0.0), half-side 0.03 m → covers cells from
  // x≈0.47..0.53 (after FP rounding inside Costmap2D::worldToMap). Path
  // poses sit at x = 0, 0.1, 0.2, ..., so only pose idx=5 (x=0.5) lands
  // inside the block; idx=4 (x=0.4) and idx=6 (x=0.6) are clear.
  stampBlock(0.5, 0.0, 0.03);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const int idx = ObstacleDeviation::findFirstObstacleIndex(costmap_, path, 0, 10);
  EXPECT_EQ(idx, 5);
}

TEST_F(ObstacleDeviationTest, FindFirstObstacle_RespectsStartIndex)
{
  // Two blocks on path: at idx 3 and idx 7. Starting at idx 5 should find idx 7.
  stampBlock(0.3, 0.0, 0.04);
  stampBlock(0.7, 0.0, 0.04);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const int idx = ObstacleDeviation::findFirstObstacleIndex(costmap_, path, 5, 5);
  EXPECT_EQ(idx, 7);
}

TEST_F(ObstacleDeviationTest, FindFirstObstacle_RespectsLookahead)
{
  // Block at idx 8 but lookahead only covers [0..3]. Should NOT find it.
  stampBlock(0.8, 0.0, 0.04);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const int idx = ObstacleDeviation::findFirstObstacleIndex(costmap_, path, 0, 4);
  EXPECT_EQ(idx, -1);
}

// ── chooseDeviationSide ───────────────────────────────────────────────────────

TEST_F(ObstacleDeviationTest, ChooseSide_FreeBothSides_BiasLeft)
{
  // Empty costmap → both sides clear at the first sample → bias left.
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);  // facing +X → left = +Y
  p.pose.orientation = tf2::toMsg(q);
  const double dev = ObstacleDeviation::chooseDeviationSide(costmap_, p, 1.0, 0.1);
  EXPECT_GT(dev, 0.0);
  EXPECT_NEAR(dev, 0.1, 1e-9);  // first step
}

TEST_F(ObstacleDeviationTest, ChooseSide_BlockedLeft_PicksRight)
{
  // Block fills left side (positive Y) at the obstacle pose.
  // Pose at origin facing +X → left is +Y, right is -Y.
  stampBlock(0.0, 0.5, 0.5);  // big block on left covering Y=[0.0..1.0]
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  p.pose.orientation = tf2::toMsg(q);
  const double dev = ObstacleDeviation::chooseDeviationSide(costmap_, p, 2.0, 0.1);
  EXPECT_LT(dev, 0.0);  // right side
}

TEST_F(ObstacleDeviationTest, ChooseSide_BlockedBoth_ReturnsZero)
{
  // Block both sides within the search radius.
  stampBlock(0.0, 0.4, 0.4);   // left
  stampBlock(0.0, -0.4, 0.4);  // right
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  p.pose.orientation = tf2::toMsg(q);
  const double dev = ObstacleDeviation::chooseDeviationSide(costmap_, p, 0.5, 0.1);
  EXPECT_DOUBLE_EQ(dev, 0.0);
}

TEST_F(ObstacleDeviationTest, ChooseSide_RespectsHeading)
{
  // Pose facing +Y (yaw = π/2) → "left" rotates to -X.
  // Block at -X should be detected as left-blocked, choose +X (right).
  stampBlock(-0.3, 0.0, 0.2);
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_2);
  p.pose.orientation = tf2::toMsg(q);
  const double dev = ObstacleDeviation::chooseDeviationSide(costmap_, p, 0.6, 0.1);
  EXPECT_LT(dev, 0.0);  // pose-frame right (which is +X in world)
}

// ── isPathClearWithDeviation ──────────────────────────────────────────────────

TEST_F(ObstacleDeviationTest, IsPathClear_NoObstacle_ZeroDeviation_True)
{
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  EXPECT_TRUE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.0));
}

TEST_F(ObstacleDeviationTest, IsPathClear_ObstacleOnPath_ZeroDeviation_False)
{
  // Block on the path itself.
  stampBlock(0.5, 0.0, 0.05);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  EXPECT_FALSE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.0));
}

TEST_F(ObstacleDeviationTest, IsPathClear_DeviationSkipsObstacle)
{
  // Block centred on path at (0.5, 0). Deviating left by 0.5 m should clear it
  // (block is only 0.1 m wide).
  stampBlock(0.5, 0.0, 0.05);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  EXPECT_TRUE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.5));
}

TEST_F(ObstacleDeviationTest, IsPathClear_DeviationStillBlocked_False)
{
  // Wide block: covers Y=[-0.6, 0.6] at x=0.5. No deviation up to 0.5 m clears.
  stampBlock(0.5, 0.0, 0.6);  // block centred at (0.5, 0), half-side 0.6
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  EXPECT_FALSE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.4));
  // But 1.0 m deviation should clear.
  EXPECT_TRUE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 1.0));
}

// ── growDeviationUntilClear ───────────────────────────────────────────────────

TEST_F(ObstacleDeviationTest, GrowDeviation_StartsClear_KeepsInitial)
{
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  // No obstacle, should keep initial value (or step minimum if 0).
  const double dev = ObstacleDeviation::growDeviationUntilClear(
      costmap_, path, 0, 10, 0.0, 1.5, 0.05);
  EXPECT_LE(std::abs(dev), 0.05 + 1e-9);
}

TEST_F(ObstacleDeviationTest, GrowDeviation_FindsClearance)
{
  // Block of half-side 0.2 m → needs ≥ 0.20 m + costmap-resolution buffer.
  stampBlock(0.5, 0.0, 0.2);  // covers Y=[-0.2, 0.2]
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  // Initial sign = positive (left), grow until clear.
  const double dev = ObstacleDeviation::growDeviationUntilClear(
      costmap_, path, 0, 10, 0.05, 1.5, 0.05);
  EXPECT_GT(dev, 0.20);          // must clear block edge
  EXPECT_LE(dev, 0.30);          // doesn't grow more than necessary
  // And the resulting path should now be clear.
  EXPECT_TRUE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, dev));
}

TEST_F(ObstacleDeviationTest, GrowDeviation_NoClearanceWithinCap_ReturnsOverCap)
{
  // Block too wide — even max_dev cannot clear it.
  stampBlock(0.5, 0.0, 2.0);  // covers Y=[-2.0, 2.0]
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const double max_dev = 1.5;
  const double dev = ObstacleDeviation::growDeviationUntilClear(
      costmap_, path, 0, 10, 0.05, max_dev, 0.05);
  EXPECT_GT(std::abs(dev), max_dev);  // Caller will see this and abort.
}

TEST_F(ObstacleDeviationTest, GrowDeviation_PreservesSign)
{
  // Block on path. Negative initial deviation should grow in negative direction.
  stampBlock(0.5, 0.0, 0.2);
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const double dev = ObstacleDeviation::growDeviationUntilClear(
      costmap_, path, 0, 10, -0.05, 1.5, 0.05);
  EXPECT_LT(dev, -0.20);  // negative side
}

// ── BoundaryGuard (confine deviation to zone) ─────────────────────────────────
//
// A second synthetic costmap acts as the zone boundary (lethal = out-of-zone).
// The two test costmaps share a frame, so the guard transform is identity.

class BoundaryGuardTest : public ObstacleDeviationTest
{
protected:
  // Boundary costmap, same geometry/frame as costmap_ → identity transform.
  nav2_costmap_2d::Costmap2D boundary_{kSize, kSize, kResolution, kOriginX, kOriginY,
                                       nav2_costmap_2d::FREE_SPACE};

  /// Stamp a square block of LETHAL cells into the boundary costmap.
  void stampBoundaryBlock(double cx, double cy, double half)
  {
    unsigned int mx0 = 0;
    unsigned int my0 = 0;
    unsigned int mx1 = 0;
    unsigned int my1 = 0;
    ASSERT_TRUE(boundary_.worldToMap(cx - half, cy - half, mx0, my0));
    ASSERT_TRUE(boundary_.worldToMap(cx + half, cy + half, mx1, my1));
    for (unsigned int x = mx0; x <= mx1; ++x)
    {
      for (unsigned int y = my0; y <= my1; ++y)
      {
        boundary_.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  /// Identity guard pointing at boundary_ (test costmaps share a frame).
  ObstacleDeviation::BoundaryGuard guard()
  {
    ObstacleDeviation::BoundaryGuard g;
    g.costmap = &boundary_;
    return g;  // tx/ty = 0, cos_yaw = 1, sin_yaw = 0 (identity)
  }
};

TEST_F(BoundaryGuardTest, IsPathClear_OffsetIntoBoundary_Rejected)
{
  // LOCAL costmap is empty (free everywhere), so a +0.5 m left offset is
  // locally clear. But the boundary marks the left side out-of-zone, so the
  // offset must be reported BLOCKED.
  stampBoundaryBlock(0.5, 0.5, 0.5);  // boundary-lethal on the left of the path
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  // Without the guard the offset path is clear (local costmap free).
  EXPECT_TRUE(ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.5));
  // With the guard the same offset lands out-of-zone → blocked.
  EXPECT_FALSE(
      ObstacleDeviation::isPathClearWithDeviation(costmap_, path, 0, 10, 0.5, guard()));
}

TEST_F(BoundaryGuardTest, GrowDeviation_OnlyClearSideOutOfZone_ReturnsOverCap)
{
  // Local obstacle on the path forces a deviation. The right side (negative Y)
  // is locally clear, but the boundary marks ALL negative Y out-of-zone, so the
  // only locally-clear side is boundary-blocked → grow can't clear → over cap
  // (caller waits/aborts instead of leaving the zone).
  stampBlock(0.5, 0.0, 0.2);              // local obstacle on the path
  stampBoundaryBlock(0.5, -2.0, 2.0);     // boundary: all of -Y near x=0.5 lethal
  const auto path = makeStraightPath(0.0, 0.0, 10, 0.1);
  const double max_dev = 1.5;
  const double dev = ObstacleDeviation::growDeviationUntilClear(
      costmap_, path, 0, 10, -0.05, max_dev, 0.05, guard());
  EXPECT_GT(std::abs(dev), max_dev);  // no in-zone clearance on the chosen side
}

TEST_F(BoundaryGuardTest, ChooseSide_OtherSideOutOfZone_PicksInZoneSide)
{
  // Pose facing +X → left = +Y, right = -Y. The local obstacle is on the left
  // AND the right is out-of-zone per the boundary... so flip it: make the LEFT
  // out-of-zone and the right in-zone+free. chooseDeviationSide must pick the
  // in-zone (right) side.
  stampBoundaryBlock(0.0, 0.5, 0.5);  // boundary-lethal on the LEFT (+Y)
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  p.pose.orientation = tf2::toMsg(q);
  const double dev = ObstacleDeviation::chooseDeviationSide(costmap_, p, 2.0, 0.1, guard());
  EXPECT_LT(dev, 0.0);  // right side (in-zone), even though left is locally free
}

}  // namespace mowgli_nav2_plugins
