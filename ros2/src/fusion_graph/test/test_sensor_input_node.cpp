// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <limits>
#include <memory>

#include "fusion_graph/fusion_graph_node.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace fusion_graph
{
class FusionGraphNodeInputTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"autoload_graph", false},
                                 {"auto_save_enabled", false},
                                 {"primary_mode", false},
                                 {"tf_broadcast_rate_hz", 0.0},
                                 {"use_lidar_map_anchor", false},
                                 {"periodic_save_period_s", 0.0},
                                 {"fast_pose_publish_rate_hz", 0.0}});
    node_ = std::make_shared<FusionGraphNode>(options);
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    Clock(10.0);
    Wheel(10.0);
    Imu(10.0);
    Clock(10.02);
    Wheel(10.02);
    Imu(10.02);
  }
  void TearDown() override
  {
    node_.reset();
  }
  void Clock(double seconds)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(),
                                        std::llround(seconds * 1e9)),
              RCL_RET_OK);
  }
  void Wheel(double seconds, double vx = .15, double vy = 0.0, double wz = 0.0)
  {
    auto message = WheelMessage(seconds, vx, vy, wz);
    node_->OnWheelOdom(message);
  }
  static nav_msgs::msg::Odometry::SharedPtr WheelMessage(double seconds,
                                                         double vx = .15,
                                                         double vy = 0.0,
                                                         double wz = 0.0)
  {
    auto message = std::make_shared<nav_msgs::msg::Odometry>();
    message->header.stamp = rclcpp::Time(std::llround(seconds * 1e9), RCL_ROS_TIME);
    message->twist.twist.linear.x = vx;
    message->twist.twist.linear.y = vy;
    message->twist.twist.angular.z = wz;
    return message;
  }
  void Imu(double seconds, double wz = 0.0)
  {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(std::llround(seconds * 1e9), RCL_ROS_TIME);
    message->angular_velocity.z = wz;
    node_->OnImu(message);
  }
  void ExpectPose(double x)
  {
    EXPECT_NEAR(node_->dr_x_, x, 1e-15);
    EXPECT_DOUBLE_EQ(node_->dr_y_, 0.0);
    EXPECT_DOUBLE_EQ(node_->dr_yaw_, 0.0);
  }
  void ExpectAcceptedSources(int64_t wheel_ns, int64_t imu_ns)
  {
    ASSERT_TRUE(node_->last_wheel_stamp_);
    ASSERT_TRUE(node_->last_imu_stamp_);
    EXPECT_EQ(node_->last_wheel_stamp_->nanoseconds(), wheel_ns);
    EXPECT_EQ(node_->last_imu_stamp_->nanoseconds(), imu_ns);
    EXPECT_DOUBLE_EQ(node_->wheel_vx_, .15);
    EXPECT_DOUBLE_EQ(node_->wheel_wz_, 0.0);
  }
  void Next()
  {
    Clock(10.04);
    // Consume the cached wheel rate first: a rejected input must not replace it.
    Imu(10.04);
    Wheel(10.04);
  }
  void InvalidImuStamp(int32_t sec, uint32_t nanosec)
  {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp.sec = sec;
    message->header.stamp.nanosec = nanosec;
    node_->OnImu(message);
  }
  void InvalidWheelStamp(int32_t sec, uint32_t nanosec)
  {
    auto message = WheelMessage(10.03, -.3);
    message->header.stamp.sec = sec;
    message->header.stamp.nanosec = nanosec;
    node_->OnWheelOdom(message);
  }
  std::shared_ptr<FusionGraphNode> node_;
};

TEST_F(FusionGraphNodeInputTest, OrderedIntervalsRetainIntegration)
{
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ReorderedImuDoesNotReopenIntegratedInterval)
{
  Clock(10.03);
  Imu(10.01);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ReorderedWheelDoesNotReplaceCachedRateOrBaseline)
{
  Clock(10.03);
  Wheel(9.99, -.3);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, AlteredDuplicateWheelCannotReplaceCachedRate)
{
  Clock(10.03);
  Wheel(10.02, -.3);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, AlteredDuplicateImuCannotIntegrateRotation)
{
  Clock(10.03);
  Imu(10.02, 1.0);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, NonfiniteGyroCannotPoisonPoseOrAcceptedSource)
{
  Clock(10.03);
  Imu(10.03, std::numeric_limits<double>::quiet_NaN());
  Imu(10.03, std::numeric_limits<double>::infinity());
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, NonfiniteWheelCannotPoisonCachedRateOrAcceptedSource)
{
  Clock(10.03);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  Wheel(10.03, nan);
  Wheel(10.03, inf);
  Wheel(10.03, .15, nan);
  Wheel(10.03, .15, 0.0, inf);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, InvalidStampDoesNotBecomeAnIntegrationBaseline)
{
  Clock(10.03);
  InvalidImuStamp(10, 1000000000u);
  InvalidWheelStamp(10, 1000000000u);
  InvalidImuStamp(-1, 0);
  InvalidWheelStamp(-1, 0);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ClockRewindAdmitsNewEpochWithoutIntegratingAcrossIt)
{
  Clock(1.0);
  Wheel(1.0);
  Imu(1.0);
  ExpectPose(.003);
  Clock(1.02);
  Wheel(1.02);
  Imu(1.02);
  ExpectPose(.006);
  ExpectAcceptedSources(1020000000, 1020000000);
}

TEST_F(FusionGraphNodeInputTest, InvalidFirstSampleAfterRewindDoesNotOwnNewBaseline)
{
  Clock(1.0);
  Wheel(1.0, std::numeric_limits<double>::quiet_NaN());
  Imu(1.0, std::numeric_limits<double>::quiet_NaN());
  Clock(1.02);
  Wheel(1.02);
  Imu(1.02);
  ExpectPose(.003);
  Clock(1.04);
  Wheel(1.04);
  Imu(1.04);
  ExpectPose(.006);
}
TEST_F(FusionGraphNodeInputTest, ImuFirstAfterRewindCannotUsePreviousEpochWheelRate)
{
  Clock(1.0);
  Imu(1.0);
  Clock(1.02);
  Imu(1.02);
  ExpectPose(.003);
  Wheel(1.02);
  Clock(1.04);
  Imu(1.04);
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, WheelFirstAfterRewindKeepsNewEpochWheelRate)
{
  Clock(1.0);
  Wheel(1.0);
  Imu(1.0);
  Clock(1.02);
  Imu(1.02);
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ZeroSourceTimeCanBootstrapSimulationEpoch)
{
  Clock(0.0);
  Wheel(0.0);
  Imu(0.0);
  Clock(.02);
  Wheel(.02);
  Imu(.02);
  ExpectPose(.006);
}

}  // namespace fusion_graph
