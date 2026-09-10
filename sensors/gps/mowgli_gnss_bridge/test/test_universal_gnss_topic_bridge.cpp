// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// Behaviour-parity test for the C++ Universal GNSS topic bridge. Drives the
// node end-to-end (publish Universal messages, assert the republished public
// contract) so the port stays field-exact with the reference Python node.

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "mowgli_gnss_bridge/universal_gnss_topic_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using mowgli_gnss_bridge::PublicGnssStatus;
using mowgli_gnss_bridge::PublicRtcmMessage;
using mowgli_gnss_bridge::RtcmFrame;
using mowgli_gnss_bridge::UniversalGnssStatus;

namespace
{

std::shared_ptr<mowgli_gnss_bridge::UniversalGnssTopicBridge> makeBridge()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"receiver_family", std::string("ublox")},
    {"frame_id", std::string("gps_link")},
    {"input_status_topic", std::string("/test/universal/status")},
    {"output_status_topic", std::string("/test/gps/status")},
    {"input_diagnostics_topic", std::string("/test/diagnostics")},
    {"input_rtcm_topic", std::string("/test/universal/rtcm")},
    {"output_rtcm_topic", std::string("/test/rtcm")},
  });
  return std::make_shared<mowgli_gnss_bridge::UniversalGnssTopicBridge>(options);
}

template <typename PredT>
bool spinUntil(rclcpp::executors::SingleThreadedExecutor & exec, PredT pred)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

}  // namespace

class BridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(BridgeTest, StatusIsMappedToPublicContract)
{
  auto bridge = makeBridge();
  auto helper = std::make_shared<rclcpp::Node>("bridge_test_helper");

  const rclcpp::QoS qos = rclcpp::QoS(10).reliable().durability_volatile();
  auto status_pub =
    helper->create_publisher<UniversalGnssStatus>("/test/universal/status", qos);

  PublicGnssStatus received;
  bool got = false;
  auto status_sub = helper->create_subscription<PublicGnssStatus>(
    "/test/gps/status", qos,
    [&](const PublicGnssStatus & msg) {
      received = msg;
      got = true;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(helper);

  UniversalGnssStatus in;
  in.fix_type = UniversalGnssStatus::FIX_TYPE_RTK_FIXED;
  in.fix_valid = true;
  in.rtk_mode = UniversalGnssStatus::RTK_MODE_FIXED;
  in.capability_flags = UniversalGnssStatus::CAP_RTK_MODE;
  in.value_flags = UniversalGnssStatus::CAP_RTK_MODE;
  in.horizontal_accuracy_m = 0.014F;

  // Republish until the reliable link is established.
  spinUntil(exec, [&]() {
    status_pub->publish(in);
    return got;
  });

  ASSERT_TRUE(got);
  EXPECT_EQ(received.fix_type, PublicGnssStatus::FIX_TYPE_RTK_FIXED);
  EXPECT_TRUE(received.fix_valid);
  EXPECT_EQ(received.rtk_mode, PublicGnssStatus::RTK_MODE_FIXED);
  EXPECT_FLOAT_EQ(received.quality_percent, 100.0F);
  EXPECT_EQ(received.capability_flags & PublicGnssStatus::CAP_RTK_MODE,
    PublicGnssStatus::CAP_RTK_MODE);
  EXPECT_EQ(received.backend, "universal");
  EXPECT_EQ(received.receiver_vendor, "u-blox");
  EXPECT_EQ(received.header.frame_id, "gps_link");
  EXPECT_FLOAT_EQ(received.horizontal_accuracy_m, 0.014F);
  // Unset upstream stays 0, which selects the receipt-stamp compatibility mode
  // in mowgli_interfaces/gnss_observation_freshness.hpp.
  EXPECT_EQ(received.position_observation_sequence, 0U);

  exec.remove_node(helper);
  exec.remove_node(bridge);
}

TEST_F(BridgeTest, RtcmFrameIsForwardedByteExact)
{
  auto bridge = makeBridge();
  auto helper = std::make_shared<rclcpp::Node>("bridge_test_helper_rtcm");

  const rclcpp::QoS qos = rclcpp::QoS(50).reliable().durability_volatile();
  auto rtcm_pub = helper->create_publisher<RtcmFrame>("/test/universal/rtcm", qos);

  PublicRtcmMessage received;
  bool got = false;
  auto rtcm_sub = helper->create_subscription<PublicRtcmMessage>(
    "/test/rtcm", qos,
    [&](const PublicRtcmMessage & msg) {
      received = msg;
      got = true;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(helper);

  RtcmFrame frame;
  frame.message_type = 1077;
  frame.data = std::vector<uint8_t>{0xD3, 0x00, 0x13, 0x3E, 0xD7};

  spinUntil(exec, [&]() {
    rtcm_pub->publish(frame);
    return got;
  });

  ASSERT_TRUE(got);
  EXPECT_EQ(received.message, frame.data);

  exec.remove_node(helper);
  exec.remove_node(bridge);
}

// The observation sequence is the strongest identity a consumer can use to tell
// a genuinely new receiver observation from the timer-driven republication of a
// cached one. The bridge must forward it verbatim and must never latch it: a
// stale value here would make a frozen receiver look live to every consumer of
// the typed contract.
TEST_F(BridgeTest, PositionObservationSequenceIsForwardedVerbatim)
{
  auto bridge = makeBridge();
  auto helper = std::make_shared<rclcpp::Node>("bridge_test_helper_sequence");

  const rclcpp::QoS qos = rclcpp::QoS(10).reliable().durability_volatile();
  auto status_pub =
    helper->create_publisher<UniversalGnssStatus>("/test/universal/status", qos);

  PublicGnssStatus received;
  bool got = false;
  auto status_sub = helper->create_subscription<PublicGnssStatus>(
    "/test/gps/status", qos,
    [&](const PublicGnssStatus & msg) {
      received = msg;
      got = true;
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(helper);

  UniversalGnssStatus in;
  in.fix_type = UniversalGnssStatus::FIX_TYPE_RTK_FIXED;
  in.fix_valid = true;
  in.position_observation_sequence = 4242U;

  spinUntil(exec, [&]() {
    status_pub->publish(in);
    return got;
  });
  ASSERT_TRUE(got);
  EXPECT_EQ(received.position_observation_sequence, 4242U);

  // A numerically identical observation with a new upstream sequence must come
  // through as the NEW sequence — the payload is deliberately not the identity.
  in.position_observation_sequence = 4243U;
  const bool advanced = spinUntil(exec, [&]() {
    status_pub->publish(in);
    return received.position_observation_sequence == 4243U;
  });
  EXPECT_TRUE(advanced) << "bridge latched the observation sequence instead of forwarding it";

  exec.remove_node(helper);
  exec.remove_node(bridge);
}
