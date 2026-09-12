// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <rclcpp/rclcpp.hpp>

#include "mowgli_hardware/cobs.hpp"
#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

// The bridge is intentionally a single executable. Rename its main while
// including the implementation so this test exercises the real parameter
// descriptors instead of duplicating their behaviour in a helper.
#define main hardware_bridge_node_main_for_test
#include "../src/hardware_bridge_node.cpp"
#undef main

namespace mowgli_hardware
{

struct HardwareBridgeBladeStatusTestPeer
{
  static std::string requested(const HardwareBridgeNode& node)
  {
    return node.blade_requested_direction_;
  }
  static void send(HardwareBridgeNode& node, uint8_t on, uint8_t dir)
  {
    node.send_blade_command(on, dir);
  }
  static void control(HardwareBridgeNode& node, uint8_t on, uint8_t direction)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
    req->mow_enabled = on;
    req->mow_direction = direction;
    auto res = std::make_shared<mowgli_interfaces::srv::MowerControl::Response>();
    node.on_mower_control(req, res);
  }
  static void status(HardwareBridgeNode& node, uint8_t emergency)
  {
    LlStatus packet{};
    packet.type = PACKET_ID_LL_STATUS;
    packet.emergency_bitmask = emergency;
    packet.v_system = 24.0;
    node.handle_status(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  }
  static void expireDelay(HardwareBridgeNode& node)
  {
    node.lift_cleared_time_ = node.now() - rclcpp::Duration::from_seconds(2);
  }
  static bool pending(const HardwareBridgeNode& node)
  {
    return node.waiting_blade_resume_;
  }
  static void dryRun(HardwareBridgeNode& node)
  {
    node.mowing_enabled_ = false;
  }
  static void emergency(HardwareBridgeNode& node)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::EmergencyStop::Request>();
    req->emergency = 1;
    node.on_emergency_stop(req,
                           std::make_shared<mowgli_interfaces::srv::EmergencyStop::Response>());
  }
  static void disconnect(HardwareBridgeNode& node)
  {
    node.close_serial_for_reconnect();
  }
};

class HardwareBridgeParametersTest : public ::testing::Test
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

TEST_F(HardwareBridgeParametersTest, PublishRateRejectsRuntimeUpdate)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device")});
  auto node = std::make_shared<HardwareBridgeNode>(options);

  const auto result = node->set_parameter(rclcpp::Parameter("publish_rate", 5.0));

  EXPECT_FALSE(result.successful);
}

TEST_F(HardwareBridgeParametersTest, UsableOutOfRangePublishRateStarts)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device"),
                               rclcpp::Parameter("publish_rate", 5.0)});

  EXPECT_NO_THROW(std::make_shared<HardwareBridgeNode>(options));
}

TEST_F(HardwareBridgeParametersTest, BladeDirectionTracksWrittenCommandsAndClearsOnDisconnect)
{
  // Real serial writes into a pseudo-terminal, without physical hardware.
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);
  struct CloseFd
  {
    int fd;
    ~CloseFd()
    {
      close(fd);
    }
  } close_fd{master};
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  ASSERT_NE(ptsname(master), nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", ptsname(master))});
  auto node = std::make_shared<HardwareBridgeNode>(options);
  using Peer = HardwareBridgeBladeStatusTestPeer;
  EXPECT_EQ(Peer::requested(*node), "unknown");
  Peer::send(*node, 1, 1);
  EXPECT_EQ(Peer::requested(*node), "reverse");
  Peer::send(*node, 0, 0);
  EXPECT_EQ(Peer::requested(*node), "off");
  Peer::send(*node, 1, 0);
  EXPECT_EQ(Peer::requested(*node), "forward");
  Peer::disconnect(*node);
  EXPECT_EQ(Peer::requested(*node), "unknown");
  Peer::send(*node, 1, 1);
  EXPECT_EQ(Peer::requested(*node), "unknown");
}

class BladeLiftRecovery : public HardwareBridgeParametersTest
{
protected:
  using Peer = HardwareBridgeBladeStatusTestPeer;
  int master{-1};
  std::shared_ptr<HardwareBridgeNode> node;
  void SetUp() override
  {
    master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    ASSERT_GE(master, 0);
    ASSERT_EQ(grantpt(master), 0);
    ASSERT_EQ(unlockpt(master), 0);
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("serial_port", ptsname(master)),
                                 rclcpp::Parameter("lift_recovery_mode", true),
                                 rclcpp::Parameter("lift_blade_resume_delay_sec", 1.0)});
    node = std::make_shared<HardwareBridgeNode>(options);
    drain();
  }
  void TearDown() override
  {
    node.reset();
    if (master >= 0)
      close(master);
  }
  std::vector<std::pair<uint8_t, uint8_t>> drain()
  {
    std::vector<uint8_t> bytes;
    uint8_t buffer[4096];
    ssize_t n;
    while ((n = read(master, buffer, sizeof(buffer))) > 0)
      bytes.insert(bytes.end(), buffer, buffer + n);
    std::vector<std::pair<uint8_t, uint8_t>> commands;
    std::vector<uint8_t> encoded;
    for (auto byte : bytes)
    {
      if (byte != 0)
      {
        encoded.push_back(byte);
        continue;
      }
      if (encoded.empty())
        continue;
      uint8_t decoded[4096];
      auto size = cobs_decode(encoded.data(), encoded.size(), decoded);
      if (size == sizeof(LlCmdBlade) && decoded[0] == PACKET_ID_LL_CMD_BLADE)
      {
        LlCmdBlade command{};
        std::memcpy(&command, decoded, sizeof(command));
        commands.emplace_back(command.blade_on, command.blade_dir);
      }
      encoded.clear();
    }
    return commands;
  }
  void liftAndClear(uint8_t direction = 1)
  {
    Peer::control(*node, 1, direction);
    Peer::status(*node, EMERGENCY_BIT_LIFT);
    Peer::status(*node, 0);
    ASSERT_TRUE(Peer::pending(*node));
  }
};

TEST_F(BladeLiftRecovery, ResumesLatestDirectionAndRepeatedOnCannotBypassDelay)
{
  for (const auto direction : {0u, 1u})
  {
    liftAndClear(direction);
    drain();
    Peer::control(*node, 1, direction);
    auto held = drain();
    ASSERT_EQ(held.size(), 1u);
    EXPECT_EQ(held[0].first, 0u);
    Peer::expireDelay(*node);
    Peer::status(*node, 0);
    auto resumed = drain();
    ASSERT_EQ(resumed.size(), 1u);
    EXPECT_EQ(resumed[0].first, 1u);
    EXPECT_EQ(resumed[0].second, direction);
  }
  liftAndClear(0);
  drain();
  Peer::control(*node, 1, 1);
  drain();
  Peer::expireDelay(*node);
  Peer::status(*node, 0);
  auto changed = drain();
  ASSERT_EQ(changed.size(), 1u);
  EXPECT_EQ(changed[0], std::make_pair(uint8_t(1), uint8_t(1)));
}

TEST_F(BladeLiftRecovery, OffDuringLiftOrDelayCancelsRecovery)
{
  for (const bool during_lift : {false, true})
  {
    Peer::control(*node, 1, 1);
    Peer::status(*node, EMERGENCY_BIT_LIFT);
    if (!during_lift)
      Peer::status(*node, 0);
    Peer::control(*node, 0, 255);
    drain();
    Peer::status(*node, 0);
    Peer::expireDelay(*node);
    Peer::status(*node, 0);
    EXPECT_FALSE(Peer::pending(*node));
    EXPECT_TRUE(drain().empty());
  }
}

TEST_F(BladeLiftRecovery, NewLiftRestartsDelayAndStopCancelsIt)
{
  liftAndClear();
  Peer::expireDelay(*node);
  drain();
  Peer::status(*node, EMERGENCY_BIT_LIFT);
  EXPECT_FALSE(Peer::pending(*node));
  for (const auto& command : drain())
    EXPECT_EQ(command.first, 0u);
  Peer::status(*node, 0);
  drain();
  Peer::status(*node, EMERGENCY_BIT_STOP);
  Peer::expireDelay(*node);
  Peer::status(*node, 0);
  EXPECT_FALSE(Peer::pending(*node));
  EXPECT_TRUE(drain().empty());
}

TEST_F(BladeLiftRecovery, EmergencyAndDisconnectInvalidateDelayedEnable)
{
  liftAndClear();
  Peer::emergency(*node);
  drain();
  Peer::expireDelay(*node);
  Peer::status(*node, 0);
  EXPECT_FALSE(Peer::pending(*node));
  EXPECT_TRUE(drain().empty());
  liftAndClear();
  drain();
  Peer::disconnect(*node);
  Peer::expireDelay(*node);
  Peer::status(*node, 0);
  EXPECT_FALSE(Peer::pending(*node));
  EXPECT_TRUE(drain().empty());
  EXPECT_EQ(Peer::requested(*node), "unknown");
}

TEST_F(BladeLiftRecovery, DryRunAndFailedSendCannotResume)
{
  liftAndClear();
  drain();
  Peer::dryRun(*node);
  Peer::control(*node, 1, 1);
  Peer::expireDelay(*node);
  Peer::status(*node, 0);
  EXPECT_FALSE(Peer::pending(*node));
  for (const auto& command : drain())
    EXPECT_EQ(command.first, 0u);
  Peer::disconnect(*node);
  Peer::send(*node, 1, 1);
  EXPECT_FALSE(Peer::pending(*node));
  EXPECT_EQ(Peer::requested(*node), "unknown");
}

}  // namespace mowgli_hardware
