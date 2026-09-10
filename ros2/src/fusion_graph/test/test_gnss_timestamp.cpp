// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for delayed GNSS measurement association. A receiver or
// bridge may deliver a fix after the robot has advanced through several graph
// nodes; the factor must constrain the pose at header.stamp, not callback time.

#include <cmath>
#include <limits>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

fg::GraphParams MakeParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.wheel_sigma_x_per_sqrt_m = 0.01;
  gp.wheel_sigma_y_per_sqrt_m = 0.01;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.01;
  gp.adaptive_noise_enabled_gain = 0.0;
  gp.prior_sigma_xy = 0.001;
  gp.prior_sigma_theta = 0.001;
  gp.lever_arm_x = 0.0;
  gp.lever_arm_y = 0.0;
  return gp;
}

void DriveOneMetre(fg::GraphManager& gm, double timestamp_s)
{
  gm.AddWheelTwist(1.0, 0.0, 0.0, 1.0);
  gm.AddGyroDelta(0.0, 1.0);
  ASSERT_TRUE(gm.Tick(timestamp_s).has_value());
}

}  // namespace

TEST(GnssTimestamp, FindsNewestLiveNodeAtOrBeforeEpoch)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 10.0);
  DriveOneMetre(gm, 11.0);
  DriveOneMetre(gm, 12.0);

  EXPECT_EQ(gm.FindNodeAtOrBefore(10.0), std::optional<uint64_t>(0));
  EXPECT_EQ(gm.FindNodeAtOrBefore(11.0), std::optional<uint64_t>(1));
  EXPECT_EQ(gm.FindNodeAtOrBefore(11.5), std::optional<uint64_t>(1));
  EXPECT_EQ(gm.FindNodeAtOrBefore(12.0), std::optional<uint64_t>(2));
  EXPECT_EQ(gm.FindNodeAtOrBefore(20.0), std::optional<uint64_t>(2));
  EXPECT_FALSE(gm.FindNodeAtOrBefore(9.99).has_value());
  EXPECT_FALSE(gm.FindNodeAtOrBefore(std::numeric_limits<double>::quiet_NaN()).has_value());
}

TEST(GnssTimestamp, DelayedFixConstrainsMeasurementEpoch)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  DriveOneMetre(gm, 1.0);
  DriveOneMetre(gm, 2.0);

  const auto measurement_node = gm.FindNodeAtOrBefore(1.0);
  ASSERT_EQ(measurement_node, std::optional<uint64_t>(1));
  ASSERT_TRUE(gm.QueueGnss(1.0, 0.0, 0.001, /*robust=*/false, measurement_node));

  DriveOneMetre(gm, 3.0);
  const auto historical = gm.GetPose(1);
  const auto current = gm.GetPose(3);
  ASSERT_TRUE(historical.has_value());
  ASSERT_TRUE(current.has_value());

  EXPECT_NEAR(historical->x(), 1.0, 0.03);
  EXPECT_NEAR(current->x(), 3.0, 0.08);
  EXPECT_NEAR(current->y(), 0.0, 0.03);
}

TEST(GnssTimestamp, RejectsTargetThatIsNotInLiveGraph)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  EXPECT_FALSE(gm.QueueGnss(0.0,
                            0.0,
                            0.01,
                            /*robust=*/false,
                            std::optional<uint64_t>(999)));
}

TEST(GnssTimestamp, UnstampedFixRetainsNextNodeFallback)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  DriveOneMetre(gm, 1.0);
  DriveOneMetre(gm, 2.0);

  // A zero/unset ROS header stamp cannot be associated historically. Keep the
  // legacy next-node behavior so unstamped sensor sources remain compatible.
  ASSERT_TRUE(gm.QueueGnss(1.0, 0.0, 0.001));
  DriveOneMetre(gm, 3.0);

  const auto current = gm.GetPose(3);
  ASSERT_TRUE(current.has_value());
  EXPECT_LT(current->x(), 1.2);
}

TEST(GnssTimestamp, HistoryIsBoundedByGraphWindow)
{
  auto params = MakeParams();
  params.max_graph_nodes = 2;
  fg::GraphManager gm(params);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  DriveOneMetre(gm, 1.0);
  DriveOneMetre(gm, 2.0);

  EXPECT_FALSE(gm.FindNodeAtOrBefore(0.5).has_value());
  EXPECT_EQ(gm.FindNodeAtOrBefore(1.0), std::optional<uint64_t>(1));
  EXPECT_EQ(gm.FindNodeAtOrBefore(2.0), std::optional<uint64_t>(2));
}

TEST(GnssTimestamp, ResetAndClockRewindDiscardStaleEpochs)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 100.0);
  DriveOneMetre(gm, 101.0);

  // Simulated-time rewind: Tick self-heals the cadence and rebuilds the time
  // index from the latest live pose in the new clock epoch.
  EXPECT_FALSE(gm.Tick(5.0).has_value());
  EXPECT_FALSE(gm.FindNodeAtOrBefore(4.99).has_value());
  EXPECT_EQ(gm.FindNodeAtOrBefore(5.0), std::optional<uint64_t>(1));

  gm.Reset();
  EXPECT_FALSE(gm.FindNodeAtOrBefore(5.0).has_value());
}
