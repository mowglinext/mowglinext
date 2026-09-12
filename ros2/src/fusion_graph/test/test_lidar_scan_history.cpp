// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#include <limits>

#include "fusion_graph/lidar_covariance.hpp"
#include "fusion_graph/lidar_scan_history.hpp"
#include <gtest/gtest.h>
namespace fg = fusion_graph;
TEST(LidarScanHistory, InterpolatesAcquisitionAndReturnsHistoricalNode)
{
  fg::LidarScanHistory h;
  h.Push(10, gtsam::Pose2(0, 0, 0));
  h.AddNode(10, 7, gtsam::Pose2(0, 0, 0));
  h.Push(10.2, gtsam::Pose2(0.2, 0, 0));
  h.AddNode(10.2, 8, gtsam::Pose2(0.2, 0, 0));
  auto scan = h.At(10.1, 10.2, 0.5);
  ASSERT_TRUE(scan);
  EXPECT_EQ(scan->index, 7u);
  EXPECT_NEAR(scan->offset.x(), 0.1, 1e-9);
  EXPECT_FALSE(h.At(9.9, 10.2, 0.5));
  EXPECT_FALSE(h.At(10.3, 10.2, 0.5));
  EXPECT_FALSE(h.At(10.1, 12.0, 0.5));
}
TEST(LidarScanHistory, HandlesAngleWrapAndClockRewind)
{
  fg::LidarScanHistory h;
  h.Push(10, gtsam::Pose2(0, 0, 3.1));
  h.AddNode(10, 1, gtsam::Pose2(0, 0, 3.1));
  h.Push(10.2, gtsam::Pose2(0, 0, -3.1));
  auto scan = h.At(10.1, 10.2, 0.5);
  ASSERT_TRUE(scan);
  EXPECT_NEAR(std::abs(scan->odom.theta()), M_PI, 1e-9);
  EXPECT_TRUE(h.Push(1, gtsam::Pose2()));
  EXPECT_FALSE(h.At(1, 1, 0.5));
}
TEST(LidarCovariance, FloorsPrincipalVarianceWithoutErasingWeakDirection)
{
  Eigen::Matrix2d cov;
  cov << 0.04, 0.039, 0.039, 0.04;
  const auto result = fg::FloorLidarCovariance(cov, 0.05);
  ASSERT_TRUE(result);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(*result);
  EXPECT_NEAR(es.eigenvalues()[0], 0.0025, 1e-12);
  EXPECT_NEAR(es.eigenvalues()[1], 0.079, 1e-12);
  cov << 1, 2, 2, 1;
  EXPECT_FALSE(fg::FloorLidarCovariance(cov, 0.05));
  cov(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fg::FloorLidarCovariance(cov, 0.05));
}

TEST(LidarScanHistory, DoesNotBridgeMissingImuCoverage)
{
  fg::LidarScanHistory h;
  h.Push(10, gtsam::Pose2());
  h.AddNode(10, 1, gtsam::Pose2());
  h.Push(11, gtsam::Pose2(1, 0, 0));
  EXPECT_FALSE(h.At(10.9, 11, 0.5));
}
