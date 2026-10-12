// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <vector>

#include "mowgli_hardware/odometry_publisher.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

class OdometryPublisherTest : public ::testing::Test
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
};

TEST_F(OdometryPublisherTest, ReconnectStartsNewClockFitWithNormalPacketDeltas)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_test",
                                             rclcpp::NodeOptions().use_intra_process_comms(true));
  auto clock = node->get_clock();
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  int64_t host_ns = 10'000'000'000;
  std::vector<int64_t> stamps;
  auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
      "~/wheel_odom",
      rclcpp::QoS(10),
      [&stamps](const nav_msgs::msg::Odometry& msg)
      {
        stamps.push_back(rclcpp::Time(msg.header.stamp).nanoseconds());
      });
  mowgli_hardware::OdometryPublisher publisher(*node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  mowgli_hardware::LlOdometry packet{};
  packet.dt_millis = 60;  // Always below the fitter's automatic reset threshold.
  auto deliver = [&]()
  {
    host_ns += 60'000'000;
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), host_ns), RCL_RET_OK);
    ++packet.left_ticks;
    ++packet.right_ticks;
    publisher.handle_packet(packet, 300.0, 0.3, false);
    executor.spin_some();
  };

  // The first packet seeds the encoders; the next 100 converge the clock fit.
  for (int i = 0; i < 101; ++i)
  {
    deliver();
  }
  ASSERT_EQ(stamps.size(), 100u);
  EXPECT_EQ(stamps.back(), host_ns);

  // Repeat to cover both a settled fit and a fit in the new serial session.
  for (int session = 0; session < 2; ++session)
  {
    const int64_t previous_stamp = stamps.back();
    publisher.reset();
    host_ns += 2'000'000'000;  // Host-side gap, absent from firmware dt_millis.
    const size_t before = stamps.size();
    packet.left_ticks = 0;
    packet.right_ticks = 0;
    deliver();
    ASSERT_EQ(stamps.size(), before);  // Encoder baseline emits no odometry.
    deliver();
    ASSERT_EQ(stamps.size(), before + 1);
    EXPECT_EQ(stamps.back(), host_ns);  // Bootstrap at the new host epoch.
    EXPECT_GT(stamps.back(), previous_stamp);
    for (int i = 0; i < 100; ++i)
    {
      const int64_t last = stamps.back();
      deliver();
      ASSERT_EQ(stamps.size(), before + 2 + i);
      EXPECT_GT(stamps.back(), last);
      EXPECT_NEAR(static_cast<double>(stamps.back() - host_ns), 0.0, 1'000.0);
    }
  }
}
