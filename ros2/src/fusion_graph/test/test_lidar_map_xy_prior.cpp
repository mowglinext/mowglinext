// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager-level tests for the LiDAR map anchor's unary factor: a queued
// QueueLidarMapXy(xy, cov, robust) must become an XY-ONLY PoseTranslationPrior
// on the target node (pulls position, never heading), with its covariance floored
// at GraphParams::lidar_anchor_sigma_floor_m. Scan-time offset is removed in map
// coordinates before the factor is added; the consumer is graph_manager_node.cpp.

#include <cmath>

#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/lidar_scan_history.hpp"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose2.h>

namespace fg = fusion_graph;

namespace
{
constexpr double kFloorSigmaM = 0.05;

fg::GraphParams TickParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_node_period_s = 0.0;  // a node every tick
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 0.0;
  gp.lidar_anchor_sigma_floor_m = kFloorSigmaM;
  return gp;
}

// Drive straight 5 ticks @ 0.5 m/s (a node each tick) and return the
// dead-reckoned x of node 5.
double DriveFiveNodes(fg::GraphManager& gm, const gtsam::Pose2& initial = gtsam::Pose2(0, 0, 0))
{
  gm.Initialize(initial, 0.0);
  for (int i = 1; i <= 5; ++i)
  {
    gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);
    gm.Tick(0.1 * i);
  }
  auto snap = gm.LatestSnapshot();
  EXPECT_TRUE(snap.has_value());
  return snap ? snap->pose.x() : 0.0;
}

// One more dead-reckoned step (0.05 m) with an XY prior queued `pull_x` metres
// ahead of the pure DR position under the given isotropic covariance. Returns
// the 6th node's pose and the DR x it would have had without the prior.
struct PullResult
{
  gtsam::Pose2 pose;
  double dr6_x;
};

PullResult TickWithXyPrior(double pull_x,
                           double pull_y,
                           double sigma_m,
                           const gtsam::Vector2& node_to_scan_map = gtsam::Vector2::Zero())
{
  fg::GraphManager gm(TickParams());
  const double x_dr = DriveFiveNodes(gm);
  const double dr6_x = x_dr + 0.05;
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  gm.AddGyroDelta(0.0, 0.1);
  gm.QueueLidarMapXy(gtsam::Vector2(dr6_x + pull_x, pull_y) + node_to_scan_map,
                     Eigen::Matrix2d::Identity() * (sigma_m * sigma_m),
                     /*robust=*/true,
                     std::nullopt,
                     node_to_scan_map);
  auto out = gm.Tick(0.1 * 6);
  EXPECT_TRUE(out.has_value());
  EXPECT_EQ(gm.LidarAnchorFactorCount(), 1u);
  return PullResult{out ? out->pose : gtsam::Pose2(), dr6_x};
}
}  // namespace

// The prior pulls the new node clearly past pure dead reckoning toward the
// target, without overshooting it.
TEST(LidarMapXyPrior, PullsNextNodeXyTowardTarget)
{
  const double pull = 0.10;
  const auto r = TickWithXyPrior(pull, 0.0, kFloorSigmaM);
  const double target_x = r.dr6_x + pull;

  EXPECT_GT(r.pose.x(), r.dr6_x + 0.02)
      << "prior did not pull the node: x=" << r.pose.x() << " dr6=" << r.dr6_x;
  EXPECT_LT(r.pose.x(), target_x + 0.03) << "node overshot the prior";
}

// XY-ONLY: even a tight lateral target must not rotate the heading — yaw
// stays owned by the gyro / wheel between-factors (2026-07-22 yaw-flip
// incident is why no LiDAR heading ever enters the graph).
TEST(LidarMapXyPrior, LeavesYawToGyro)
{
  // A 10 cm scan-time translation is a realistic non-zero offset. The
  // lateral target would rotate this offset to reduce XY error if the factor
  // incorrectly used body-frame lever-arm geometry.
  const gtsam::Vector2 node_to_scan_map(0.10, 0.0);
  const auto forward = TickWithXyPrior(0.10, 0.0, kFloorSigmaM, node_to_scan_map);
  EXPECT_LT(std::abs(forward.pose.theta()), 0.02)
      << "forward xy prior moved heading: theta=" << forward.pose.theta();

  const auto lateral_without_offset = TickWithXyPrior(0.0, 0.10, 1.0e-3);
  const auto lateral = TickWithXyPrior(0.0, 0.10, 1.0e-3, node_to_scan_map);
  EXPECT_LT(std::abs(lateral_without_offset.pose.theta()), 0.02)
      << "lateral xy prior moved heading without a scan offset: theta="
      << lateral_without_offset.pose.theta();
  EXPECT_LT(std::abs(lateral.pose.theta()), 0.02)
      << "lateral xy prior moved heading: theta=" << lateral.pose.theta();
  EXPECT_NEAR(lateral.pose.theta(), lateral_without_offset.pose.theta(), 1.0e-7)
      << "a non-zero node-to-scan offset changed the graph yaw";
  EXPECT_NEAR(lateral.pose.y(), lateral_without_offset.pose.y(), 1.0e-6)
      << "retiming the scan observation changed its XY constraint";
  EXPECT_GT(lateral.pose.y(), 0.02)
      << "translation-only prior did not constrain lateral position: y=" << lateral.pose.y();
}

// A 1 mm covariance must not pin harder than lidar_anchor_sigma_floor_m: the
// pull it produces is identical to one queued AT the floor, and a covariance
// far above the floor pulls much less.
TEST(LidarMapXyPrior, CovarianceIsFlooredAtSigmaFloor)
{
  const double pull = 0.10;
  const auto tiny = TickWithXyPrior(pull, 0.0, 1.0e-3);  // 1 mm, below the floor
  const auto at_floor = TickWithXyPrior(pull, 0.0, kFloorSigmaM);
  const auto huge = TickWithXyPrior(pull, 0.0, 1.0);  // 1 m, far above the floor

  const double moved_tiny = tiny.pose.x() - tiny.dr6_x;
  const double moved_floor = at_floor.pose.x() - at_floor.dr6_x;
  const double moved_huge = huge.pose.x() - huge.dr6_x;

  EXPECT_NEAR(moved_tiny, moved_floor, 1.0e-6)
      << "1 mm covariance pulled differently from the floor: tiny=" << moved_tiny
      << " floor=" << moved_floor;
  EXPECT_GT(moved_floor, 0.02) << "floored prior did not pull: " << moved_floor;
  EXPECT_LT(moved_huge, 0.5 * moved_floor)
      << "1 m covariance pulled almost as hard as the floor: huge=" << moved_huge
      << " floor=" << moved_floor;
}

TEST(LidarMapXyPrior, DelayedObservationUsesHistoricalNodeAndMotionOffset)
{
  fg::GraphManager gm(TickParams());
  DriveFiveNodes(gm);
  auto before = gm.LatestSnapshot();
  ASSERT_TRUE(before);
  const auto offset = gtsam::Vector2(0.02, 0.0);
  gm.QueueLidarMapXy(before->pose.translation() + offset,
                     Eigen::Matrix2d::Identity() * 0.0025,
                     false,
                     before->node_index,
                     offset);
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  gm.AddGyroDelta(0.0, 0.1);
  auto out = gm.Tick(0.7);
  ASSERT_TRUE(out);
  EXPECT_EQ(gm.LidarAnchorFactorCount(), 1u);
  EXPECT_NEAR(out->pose.x(), before->pose.x() + 0.05, 1e-5);
}

TEST(LidarMapXyPrior, RotatedHistoricalOffsetRetimesToSameNodeTranslation)
{
  const gtsam::Pose2 initial(1.0, -0.5, 0.4);
  fg::GraphManager scan_time_graph(TickParams());
  fg::GraphManager node_time_graph(TickParams());
  DriveFiveNodes(scan_time_graph, initial);
  DriveFiveNodes(node_time_graph, initial);
  const auto scan_node = scan_time_graph.LatestSnapshot();
  const auto target_node = node_time_graph.LatestSnapshot();
  ASSERT_TRUE(scan_node);
  ASSERT_TRUE(target_node);
  ASSERT_NEAR(scan_node->pose.theta(), 0.4, 1e-5);

  // The scan offset is expressed in the node's local odometry frame, then
  // rotated once into map coordinates just as LidarMapAnchorStep does.
  const gtsam::Pose2 node_to_scan_local(0.02, 0.0, 0.0);
  const auto scan_pose = scan_node->pose.compose(node_to_scan_local);
  const gtsam::Vector2 node_to_scan_map =
      fg::LidarNodeToScanMapOffset(scan_node->pose, node_to_scan_local);
  const gtsam::Vector2 expected_offset(0.02 * std::cos(scan_node->pose.theta()),
                                       0.02 * std::sin(scan_node->pose.theta()));
  EXPECT_NEAR(node_to_scan_map.x(), expected_offset.x(), 1e-12);
  EXPECT_NEAR(node_to_scan_map.y(), expected_offset.y(), 1e-12);
  EXPECT_NEAR((scan_pose.translation() - scan_node->pose.translation() - node_to_scan_map).norm(),
              0.0,
              1e-12);
  const gtsam::Vector2 target_xy = target_node->pose.translation() + gtsam::Vector2(0.08, -0.04);
  const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.0025;

  scan_time_graph.QueueLidarMapXy(
      target_xy + node_to_scan_map, cov, false, scan_node->node_index, node_to_scan_map);
  node_time_graph.QueueLidarMapXy(
      target_xy, cov, false, target_node->node_index, gtsam::Vector2::Zero());
  for (auto* gm : {&scan_time_graph, &node_time_graph})
  {
    gm->AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    gm->AddGyroDelta(0.0, 0.1);
  }
  const auto scan_time_out = scan_time_graph.Tick(0.7);
  const auto node_time_out = node_time_graph.Tick(0.7);
  ASSERT_TRUE(scan_time_out);
  ASSERT_TRUE(node_time_out);
  EXPECT_EQ(scan_time_graph.LidarAnchorFactorCount(), 1u);
  EXPECT_EQ(node_time_graph.LidarAnchorFactorCount(), 1u);
  EXPECT_NEAR(scan_time_out->pose.x(), node_time_out->pose.x(), 1e-6);
  EXPECT_NEAR(scan_time_out->pose.y(), node_time_out->pose.y(), 1e-6);
  EXPECT_NEAR(scan_time_out->pose.theta(), node_time_out->pose.theta(), 1e-7);
}

TEST(LidarMapXyPrior, MissingHistoricalNodeDoesNotFallForward)
{
  fg::GraphManager gm(TickParams());
  DriveFiveNodes(gm);
  gm.QueueLidarMapXy(gtsam::Vector2(10, 10), Eigen::Matrix2d::Identity() * 0.0025, false, 999999);
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  ASSERT_TRUE(gm.Tick(0.7));
  EXPECT_EQ(gm.LidarAnchorFactorCount(), 0u);
}

TEST(LidarMapXyPrior, ExpiredOrCancelledObservationDoesNotApply)
{
  fg::GraphManager gm(TickParams());
  DriveFiveNodes(gm);
  gm.QueueLidarMapXy(gtsam::Vector2(10, 10),
                     Eigen::Matrix2d::Identity() * 0.0025,
                     false,
                     std::nullopt,
                     gtsam::Vector2::Zero(),
                     0.55);
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  ASSERT_TRUE(gm.Tick(0.7));
  EXPECT_EQ(gm.LidarAnchorFactorCount(), 0u);
  gm.QueueLidarMapXy(gtsam::Vector2(10, 10), Eigen::Matrix2d::Identity() * 0.0025);
  gm.ClearLidarObservations();
  ASSERT_TRUE(gm.Tick(0.9));
  EXPECT_EQ(gm.LidarAnchorFactorCount(), 0u);
}
