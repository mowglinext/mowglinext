// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include "mowgli_openmower_bridge/openmower_bridge_node.hpp"
#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

namespace mowgli_openmower_bridge
{
struct OpenMowerPrecisionTestPeer
{
  static void report(OpenMowerBridgeNode& node, uint8_t mode)
  {
    auto msg = std::make_shared<mowgli_interfaces::msg::HighLevelStatus>();
    msg->state = mode;
    node.on_high_level_status(msg);
  }
  static double input(OpenMowerBridgeNode& node, double vx)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->twist.linear.x = vx;
    node.on_cmd_vel(msg);
    return node.cmd_vx_;
  }
  static uint8_t mode(const OpenMowerBridgeNode& node)
  {
    return node.current_mode_;
  }
  static void stale(OpenMowerBridgeNode& node)
  {
    node.bt_status_receipt_ns_ -= 6'000'000'000;
  }
  static void attach(OpenMowerBridgeNode& node, const char* port)
  {
    node.ll_serial_port_ = port;
    node.lowlevel_read_tick();
  }
  static void read(OpenMowerBridgeNode& node)
  {
    node.ll_last_rx_ = node.now() - rclcpp::Duration::from_seconds(3.0);
    node.lowlevel_read_tick();
  }
};

class OpenMowerPrecision : public ::testing::Test
{
protected:
  using Peer = OpenMowerPrecisionTestPeer;
  std::unique_ptr<OpenMowerBridgeNode> node;
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
    node = std::make_unique<OpenMowerBridgeNode>();
  }
};

TEST_F(OpenMowerPrecision, FreshAutonomyPassesLowTargetsAndExactStops)
{
  Peer::report(*node, HL_MODE_AUTONOMOUS);
  for (double v : {-0.03, 0.0, .025 / .8, .030 / .8, .040 / .8, .3})
    EXPECT_DOUBLE_EQ(Peer::input(*node, v), v);
}

TEST_F(OpenMowerPrecision, ManualModesRetainLegacyInputPolicy)
{
  for (uint8_t mode : {HL_MODE_IDLE, HL_MODE_RECORDING, HL_MODE_MANUAL_MOWING, HL_MODE_NULL})
  {
    Peer::report(*node, mode);
    for (double v : {-0.3, -0.03, -0.001, 0.0, 0.001, 0.03, 0.05, 0.3})
    {
      const double legacy = std::abs(v) > 1e-3 && std::abs(v) < .05 ? 0.0 : v;
      EXPECT_DOUBLE_EQ(Peer::input(*node, v), legacy);
    }
  }
}

TEST_F(OpenMowerPrecision, FallbackBeforeBtAndAfterBtNullCannotGrantPrecision)
{
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
  EXPECT_EQ(Peer::mode(*node), HL_MODE_AUTONOMOUS);
  Peer::report(*node, HL_MODE_NULL);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
  EXPECT_EQ(Peer::mode(*node), HL_MODE_AUTONOMOUS);
}

TEST_F(OpenMowerPrecision, ExpiryAndModeHandoverRestoreClamp)
{
  Peer::report(*node, HL_MODE_AUTONOMOUS);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), .03);
  Peer::stale(*node);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
  Peer::report(*node, HL_MODE_AUTONOMOUS);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), .03);
  Peer::report(*node, HL_MODE_MANUAL_MOWING);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
}

TEST_F(OpenMowerPrecision, LowlevelReconnectRequiresNewBtEvidence)
{
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  Peer::report(*node, HL_MODE_AUTONOMOUS);
  Peer::attach(*node, ptsname(master));
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
  Peer::report(*node, HL_MODE_AUTONOMOUS);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), .03);
  close(master);
  Peer::read(*node);
  EXPECT_DOUBLE_EQ(Peer::input(*node, .03), 0.0);
}
}  // namespace mowgli_openmower_bridge
