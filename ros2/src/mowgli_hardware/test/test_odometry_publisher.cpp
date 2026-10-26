// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
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
  for (const unsigned drops : {0u, 1u, 7u, 100u, 3502u, 35002u, 70000u})
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

TEST_F(OdometryPublisherTest, ShortIntervalRejectsMalformedCumulativeJumpsAndSpikes)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_spike_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  mowgli_hardware::LlOdometry packet{};
  packet.acquisition_millis = 100;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  // A malformed jump is rejected, never reinterpreted as a raw counter wrap.
  // Firmware already handles those before constructing cumulative totals.
  packet.left_ticks = 65537;
  packet.right_ticks = -65535;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  packet.left_ticks += 101;  // Still rejected at the normal 20 ms cadence.
  packet.right_ticks += 101;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  packet.acquisition_millis += 20;
  ++packet.left_ticks;
  ++packet.right_ticks;
  publisher.handle_packet(packet, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 1.0 / 277.0, 1e-12);
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

// ---------------------------------------------------------------------------
// Previous-protocol (v8) packets through the publisher (update-first window,
// docs/UPDATES.md). Field 2026-10-25: without this the v9 host published no
// /wheel_odom against v8 firmware and the update rolled back.
// ---------------------------------------------------------------------------

namespace
{

struct WheelOdomCapture
{
  std::shared_ptr<rclcpp::Node> node;
  std::vector<nav_msgs::msg::Odometry> msgs;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor executor;

  explicit WheelOdomCapture(const std::string& name)
      : node(std::make_shared<rclcpp::Node>(name,
                                            rclcpp::NodeOptions().use_intra_process_comms(true)))
  {
    sub = node->create_subscription<nav_msgs::msg::Odometry>("~/wheel_odom",
                                                             rclcpp::QoS(100),
                                                             [this](
                                                                 const nav_msgs::msg::Odometry& msg)
                                                             {
                                                               msgs.push_back(msg);
                                                             });
    executor.add_node(node);
  }
};

mowgli_hardware::LlOdometryV8 legacy_packet(uint16_t dt, int32_t left, int32_t right)
{
  mowgli_hardware::LlOdometryV8 p{};
  p.type = mowgli_hardware::PACKET_ID_LL_ODOMETRY;
  p.dt_millis = dt;
  p.left_ticks = left;
  p.right_ticks = right;
  return p;
}

}  // namespace

TEST_F(OdometryPublisherTest, LegacyV8StationaryStreamPublishesZeroVelocity)
{
  WheelOdomCapture cap("odometry_legacy_stationary_test");
  mowgli_hardware::OdometryPublisher publisher(*cap.node);
  for (int i = 0; i < 20; ++i)
  {
    publisher.handle_legacy_v8_packet(legacy_packet(20, 500, -500), 277.0, 0.325, false);
    cap.executor.spin_some();
  }
  // 19 intervals of 20 ms aggregate into 60 ms windows -> 6 publications.
  ASSERT_GE(cap.msgs.size(), 5u);
  for (const auto& msg : cap.msgs)
  {
    EXPECT_EQ(msg.twist.twist.linear.x, 0.0);
    EXPECT_EQ(msg.twist.twist.angular.z, 0.0);
  }
  EXPECT_TRUE(publisher.wheels_stationary());
  EXPECT_EQ(publisher.tyre_travelled(), 0.0);
}

TEST_F(OdometryPublisherTest, LegacyV8MovingStreamUsesFirmwareDtForVelocity)
{
  WheelOdomCapture cap("odometry_legacy_moving_test");
  mowgli_hardware::OdometryPublisher publisher(*cap.node);
  int32_t left = 1000, right = 1000;
  publisher.handle_legacy_v8_packet(legacy_packet(20, left, right), 277.0, 0.325, false);
  for (int i = 0; i < 30; ++i)
  {
    left += 2;
    right += 4;
    publisher.handle_legacy_v8_packet(legacy_packet(20, left, right), 277.0, 0.325, false);
    cap.executor.spin_some();
  }
  ASSERT_FALSE(cap.msgs.empty());
  for (const auto& msg : cap.msgs)
  {
    EXPECT_NEAR(msg.twist.twist.linear.x, 3.0 / 277.0 / 0.020, 1e-9);
    EXPECT_NEAR(msg.twist.twist.angular.z, 2.0 / 277.0 / 0.325 / 0.020, 1e-9);
  }
  EXPECT_FALSE(publisher.wheels_stationary());
  EXPECT_NEAR(publisher.tyre_travelled(), 30.0 * 4.0 / 277.0, 1e-9);
}

TEST_F(OdometryPublisherTest, LegacyV8CounterWrapIsRecoveredAndSpikeDropped)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_legacy_wrap_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  publisher.handle_legacy_v8_packet(legacy_packet(60, 10, 10), 277.0, 0.325, false);
  // 60 ms per packet: each one closes a 50 ms aggregation window.
  // 16-bit wrap carrying +3 / -3 real ticks: recovered, as the v8 host did.
  publisher.handle_legacy_v8_packet(legacy_packet(60, 10 - 65533, 10 + 65533), 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  // A residual glitch beyond the spike limit is dropped, not integrated.
  publisher.handle_legacy_v8_packet(legacy_packet(60, 10 - 65533 + 500, 10 + 65533),
                                    277.0,
                                    0.325,
                                    false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  publisher.handle_legacy_v8_packet(legacy_packet(60, 10 - 65533 + 501, 10 + 65533),
                                    277.0,
                                    0.325,
                                    false);
  EXPECT_NEAR(publisher.tyre_travelled(), 4.0 / 277.0, 1e-12);
}

TEST_F(OdometryPublisherTest, SwitchingWireFormatReseedsInsteadOfInventingMotion)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_format_switch_test");
  mowgli_hardware::OdometryPublisher publisher(*node);
  publisher.handle_legacy_v8_packet(legacy_packet(60, 0, 0), 277.0, 0.325, false);
  publisher.handle_legacy_v8_packet(legacy_packet(60, 3, 3), 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  // Board reflashed to v9 without a serial reconnect: unrelated baselines.
  mowgli_hardware::LlOdometry current{};
  current.acquisition_millis = 5;
  current.left_ticks = 90000;
  current.right_ticks = -90000;
  publisher.handle_packet(current, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 3.0 / 277.0, 1e-12);
  current.acquisition_millis += 60;
  current.left_ticks += 2;
  current.right_ticks += 2;
  publisher.handle_packet(current, 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 5.0 / 277.0, 1e-12);
  // And back (e.g. a v8 board re-attached): re-seeded again.
  publisher.handle_legacy_v8_packet(legacy_packet(60, 7, 7), 277.0, 0.325, false);
  EXPECT_NEAR(publisher.tyre_travelled(), 5.0 / 277.0, 1e-12);
}
