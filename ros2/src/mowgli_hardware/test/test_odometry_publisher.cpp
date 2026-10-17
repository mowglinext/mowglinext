// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <limits>
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
  packet.acquisition_millis = 0;
  auto deliver = [&]()
  {
    host_ns += 60'000'000;
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), host_ns), RCL_RET_OK);
    packet.acquisition_millis += 60;  // Below the fitter's automatic reset threshold.
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
    host_ns += 2'000'000'000;  // Host-side gap, absent from firmware acquisition time.
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

TEST_F(OdometryPublisherTest, DroppedSamplesPreserveDistanceVelocityAndAcquisitionTime)
{
  // Same constant wheel motion at 20 ms acquisition cadence. The host sees
  // only delivered cumulative samples; arrival jitter must not set velocity.
  for (const unsigned drops : {0u, 1u, 7u, 100u, 3502u, 35002u})
  {
    for (const uint32_t start : {100u, std::numeric_limits<uint32_t>::max() - 10u})
    {
      SCOPED_TRACE(::testing::Message() << "drops=" << drops << " start=" << start);
      auto node =
          std::make_shared<rclcpp::Node>("odometry_drop_test",
                                         rclcpp::NodeOptions().use_intra_process_comms(true));
      auto clock = node->get_clock();
      ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
      constexpr int64_t epoch = 100'000'000'000;
      unsigned acquisition_index = 0, previous_publish_index = 0, publishes = 0;
      double distance = 0;
      auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
          "~/wheel_odom",
          rclcpp::QoS(10),
          [&](const nav_msgs::msg::Odometry& msg)
          {
            // Left +1 tick, right -2 ticks per acquisition => reverse centre
            // motion and a turn. Check both signed linear and angular velocity.
            EXPECT_NEAR(msg.twist.twist.linear.x, -0.5 / 277.0 / 0.020, 1e-12);
            EXPECT_NEAR(msg.twist.twist.angular.z, -3.0 / 277.0 / 0.325 / 0.020, 1e-12);
            const double dt = (acquisition_index - previous_publish_index) * 0.020;
            distance += msg.twist.twist.linear.x * dt;
            previous_publish_index = acquisition_index;
            ++publishes;
          });
      mowgli_hardware::OdometryPublisher publisher(*node);
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      mowgli_hardware::LlOdometry packet{};
      packet.acquisition_millis = start;
      ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), epoch), RCL_RET_OK);
      publisher.handle_packet(packet, 277.0, 0.325, false);
      // These drop counts resume on an aggregation boundary; finish on one too
      // so the two streams integrate exactly the same full acquisition range.
      const unsigned samples = std::max(12u, ((drops + 10u) / 3u) * 3u);
      for (acquisition_index = 1; acquisition_index <= samples; ++acquisition_index)
      {
        packet.acquisition_millis += 20;
        ++packet.left_ticks;
        packet.right_ticks -= 2;
        if (acquisition_index >= 2 && acquisition_index < drops + 2)
          continue;
        // Deliberately vary delivery latency. It affects stamp fitting only.
        const int64_t host_ns =
            epoch + acquisition_index * 20'000'000LL + (acquisition_index % 2) * 4'000'000LL;
        ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), host_ns), RCL_RET_OK);
        publisher.handle_packet(packet, 277.0, 0.325, false);
        executor.spin_some();
      }
      EXPECT_GT(publishes, 0u);
      EXPECT_EQ(previous_publish_index, samples);
      EXPECT_NEAR(distance, -0.5 * samples / 277.0, 1e-10);
      EXPECT_NEAR(publisher.tyre_travelled(), 2.0 * samples / 277.0, 1e-10);
    }
  }
}

TEST_F(OdometryPublisherTest, SameMillisecondSampleRetainsTickBaseline)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_same_time_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  mowgli_hardware::LlOdometry packet{};
  packet.acquisition_millis = 100;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.left_ticks = packet.right_ticks = 1;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 60;
  packet.left_ticks = packet.right_ticks = 3;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
}

TEST_F(OdometryPublisherTest, ShortIntervalStillRecoversControllerWrapAndRejectsSpikes)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_spike_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  mowgli_hardware::LlOdometry packet{};
  packet.acquisition_millis = 100;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  packet.left_ticks = 65537;  // Raw delta +65536 + 1 => one valid tick.
  packet.right_ticks = -65535;  // Raw delta -65536 + 1 => one valid tick.
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  packet.left_ticks += 101;  // Still rejected at the normal 20 ms cadence.
  packet.right_ticks += 101;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  ++packet.left_ticks;
  ++packet.right_ticks;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 2.0 / 277.0, 1e-12);
}

TEST_F(OdometryPublisherTest, FirmwareClockRestartReseedsTimeAndTicksTogether)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_clock_restart_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  mowgli_hardware::LlOdometry packet{};
  packet.acquisition_millis = 1000;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 60;
  packet.left_ticks = packet.right_ticks = 3;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  // A firmware restart may precede the host's serial reconnect handling.
  packet.acquisition_millis = 20;
  packet.left_ticks = packet.right_ticks = 0;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  packet.acquisition_millis += 60;
  packet.left_ticks = packet.right_ticks = 3;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 6.0 / 277.0, 1e-12);
}
