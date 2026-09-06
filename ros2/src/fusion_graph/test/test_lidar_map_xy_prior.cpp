// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager-level tests for the LiDAR map anchor's unary factor: a queued
// QueueLidarMapXy(xy, cov, robust) must become an XY-ONLY PoseTranslationPrior
// on the NEXT node (pulls position, never heading), with its covariance floored
// at GraphParams::lidar_anchor_sigma_floor_m. Modelled on the removed
// scan-to-keyframe apply tests; the consumer is graph_manager_node.cpp.

#include <cmath>

#include "fusion_graph/graph_manager.hpp"
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
double DriveFiveNodes(fg::GraphManager& gm)
{
  gm.Initialize(gtsam::Pose2(0, 0, 0), 0.0);
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

PullResult TickWithXyPrior(double pull_x, double pull_y, double sigma_m)
{
  fg::GraphManager gm(TickParams());
  const double x_dr = DriveFiveNodes(gm);
  const double dr6_x = x_dr + 0.05;
  gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
  gm.AddGyroDelta(0.0, 0.1);
  gm.QueueLidarMapXy(gtsam::Vector2(dr6_x + pull_x, pull_y),
                     Eigen::Matrix2d::Identity() * (sigma_m * sigma_m),
                     /*robust=*/true);
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
  const auto forward = TickWithXyPrior(0.10, 0.0, kFloorSigmaM);
  EXPECT_LT(std::abs(forward.pose.theta()), 0.02)
      << "forward xy prior moved heading: theta=" << forward.pose.theta();

  const auto lateral = TickWithXyPrior(0.0, 0.10, 1.0e-3);
  EXPECT_LT(std::abs(lateral.pose.theta()), 0.02)
      << "lateral xy prior moved heading: theta=" << lateral.pose.theta();
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
