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
  struct CmdVelState
  {
    bool known;
    std::int64_t sent_ns;
    std::int64_t target_ns;
    float vx;
    float wz;
  };

  static std::string requested(const HardwareBridgeNode& node)
  {
    return node.blade_requested_direction_;
  }
  static void sendCmdVel(HardwareBridgeNode& node, double vx, double wz)
  {
    node.send_cmd_vel_packet(vx, wz);
  }
  static CmdVelState cmdVelState(const HardwareBridgeNode& node)
  {
    return {node.have_host_velocity_target_,
            node.last_cmd_vel_packet_ns_,
            node.last_host_velocity_target_steady_ns_,
            node.last_host_velocity_target_vx_,
            node.last_host_velocity_target_wz_};
  }
  static std::int64_t steadyNowNs()
  {
    return HardwareBridgeNode::steadyNowNs();
  }
  static bool bladeIntentAuthorized(const HardwareBridgeNode& node)
  {
    return node.blade_intent_authorized_;
  }
  static bool haveBladeStatus(const HardwareBridgeNode& node)
  {
    return node.have_blade_status_;
  }
  static void send(HardwareBridgeNode& node, uint8_t on, uint8_t dir)
  {
    node.send_blade_command(on, dir);
  }
  static void bladeStatus(HardwareBridgeNode& node, uint8_t active)
  {
    LlBladeStatus packet{};
    packet.type = PACKET_ID_LL_BLADE_STATUS;
    packet.is_active = active;
    node.handle_blade_status(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  }
  static void reassertTick(HardwareBridgeNode& node)
  {
    node.blade_reassert_tick();
  }
  static void setBladeStatusReceipt(HardwareBridgeNode& node, std::int64_t receipt_ns)
  {
    node.have_blade_status_ = true;
    node.blade_active_ = false;
    node.blade_status_time_ = node.now();
    node.last_blade_status_steady_ns_ = receipt_ns;
  }
  static void ageBladeCommand(HardwareBridgeNode& node, std::int64_t age_ns)
  {
    node.last_blade_cmd_ns_ = steadyNowNs() - age_ns;
  }
  static void makeCmdVelFresh(HardwareBridgeNode& node)
  {
    node.last_cmd_vel_packet_ns_ = steadyNowNs() - 10'000'000;
  }
  static void prepareReassertAfterDisconnect(HardwareBridgeNode& node)
  {
    node.mow_enabled_ = true;
    node.blade_intent_authorized_ = true;
    node.last_cmd_vel_packet_ns_ = steadyNowNs() - 10'000'000;
    node.blade_status_time_ = node.now();
    node.blade_active_ = false;
    node.last_blade_cmd_ns_ = steadyNowNs() - 1'000'000'000;
  }
  static void setRequestedForTest(HardwareBridgeNode& node, std::string requested)
  {
    node.blade_requested_direction_ = std::move(requested);
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
  static void emergency(HardwareBridgeNode& node, uint8_t emergency = 1)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::EmergencyStop::Request>();
    req->emergency = emergency;
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
    std::vector<std::pair<uint8_t, uint8_t>> commands;
    for (const auto& decoded : drainPackets())
    {
      if (decoded.size() == sizeof(LlCmdBlade) && decoded[0] == PACKET_ID_LL_CMD_BLADE)
      {
        LlCmdBlade command{};
        std::memcpy(&command, decoded.data(), sizeof(command));
        commands.emplace_back(command.blade_on, command.blade_dir);
      }
    }
    return commands;
  }
  std::vector<std::vector<uint8_t>> drainPackets()
  {
    std::vector<uint8_t> bytes;
    uint8_t buffer[4096];
    ssize_t n;
    while ((n = read(master, buffer, sizeof(buffer))) > 0)
      bytes.insert(bytes.end(), buffer, buffer + n);
    std::vector<std::vector<uint8_t>> packets;
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
      std::vector<uint8_t> decoded(encoded.size());
      const auto size = cobs_decode(encoded.data(), encoded.size(), decoded.data());
      if (size > 0)
      {
        decoded.resize(size);
        packets.push_back(std::move(decoded));
      }
      encoded.clear();
    }
    return packets;
  }
  void closeMaster()
  {
    if (master >= 0)
    {
      close(master);
      master = -1;
    }
  }
  void liftAndClear(uint8_t direction = 1)
  {
    Peer::control(*node, 1, direction);
    Peer::status(*node, EMERGENCY_BIT_LIFT | EMERGENCY_BIT_LATCH);
    Peer::status(*node, 0);
    ASSERT_TRUE(Peer::pending(*node));
  }
};

TEST_F(BladeLiftRecovery, CmdVelSendTracksExactWireTargetOnSuccessfulWrite)
{
  const double vx = 0.123456789;
  const double wz = -0.876543219;
  Peer::sendCmdVel(*node, vx, wz);

  const auto packets = drainPackets();
  ASSERT_EQ(packets.size(), 1u);
  ASSERT_EQ(packets[0].size(), sizeof(LlCmdVel));
  LlCmdVel sent{};
  std::memcpy(&sent, packets[0].data(), sizeof(sent));
  EXPECT_EQ(sent.type, PACKET_ID_LL_CMD_VEL);
  EXPECT_EQ(sent.linear_x, static_cast<float>(vx));
  EXPECT_EQ(sent.angular_z, static_cast<float>(wz));

  const auto state = Peer::cmdVelState(*node);
  EXPECT_TRUE(state.known);
  EXPECT_GT(state.sent_ns, 0);
  EXPECT_EQ(state.sent_ns, state.target_ns);
  EXPECT_EQ(state.vx, sent.linear_x);
  EXPECT_EQ(state.wz, sent.angular_z);
}

TEST_F(BladeLiftRecovery, FailedCmdVelWriteDoesNotRefreshOrEstablishTarget)
{
  ASSERT_FALSE(Peer::cmdVelState(*node).known);
  closeMaster();
  Peer::sendCmdVel(*node, 0.25, -0.5);

  const auto state = Peer::cmdVelState(*node);
  EXPECT_FALSE(state.known);
  EXPECT_EQ(state.sent_ns, 0);
  EXPECT_EQ(state.target_ns, 0);
  EXPECT_TRUE(drain().empty());
}

TEST_F(BladeLiftRecovery, FailedCmdVelWriteDoesNotRefreshPriorTarget)
{
  Peer::sendCmdVel(*node, 0.25, -0.5);
  ASSERT_EQ(drainPackets().size(), 1u);
  const auto before = Peer::cmdVelState(*node);
  ASSERT_TRUE(before.known);

  closeMaster();
  Peer::sendCmdVel(*node, -0.75, 1.25);

  const auto after = Peer::cmdVelState(*node);
  EXPECT_FALSE(after.known);
  EXPECT_EQ(after.sent_ns, before.sent_ns);
  EXPECT_EQ(after.target_ns, before.target_ns);
  EXPECT_EQ(after.vx, before.vx);
  EXPECT_EQ(after.wz, before.wz);
  EXPECT_TRUE(drain().empty());
}

TEST_F(BladeLiftRecovery, FreshSendAndTelemetryReassertOneCorrectBladeOn)
{
  Peer::control(*node, 1, 1);
  EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));

  Peer::sendCmdVel(*node, 0.2, 0.1);
  ASSERT_EQ(drainPackets().size(), 1u);
  Peer::bladeStatus(*node, 0);
  Peer::ageBladeCommand(*node, 1'000'000'000);
  Peer::makeCmdVelFresh(*node);

  Peer::reassertTick(*node);
  EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
  Peer::reassertTick(*node);
  EXPECT_TRUE(drain().empty());
}

TEST_F(BladeLiftRecovery, ServiceStopAndReleaseBetweenTicksNeedsNewExplicitOn)
{
  Peer::control(*node, 1, 1);
  drain();
  Peer::sendCmdVel(*node, 0.2, 0.0);
  drainPackets();
  Peer::bladeStatus(*node, 0);

  Peer::emergency(*node, 1);
  EXPECT_FALSE(Peer::bladeIntentAuthorized(*node));
  Peer::emergency(*node, 0);
  drainPackets();

  Peer::sendCmdVel(*node, 0.2, 0.0);
  drainPackets();
  Peer::bladeStatus(*node, 0);
  Peer::ageBladeCommand(*node, 1'000'000'000);
  Peer::makeCmdVelFresh(*node);
  Peer::reassertTick(*node);
  EXPECT_TRUE(drain().empty());

  Peer::control(*node, 1, 1);
  EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
  EXPECT_TRUE(Peer::bladeIntentAuthorized(*node));
  Peer::sendCmdVel(*node, 0.2, 0.0);
  drainPackets();
  Peer::bladeStatus(*node, 0);
  Peer::ageBladeCommand(*node, 1'000'000'000);
  Peer::makeCmdVelFresh(*node);
  Peer::reassertTick(*node);
  EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
}

TEST_F(BladeLiftRecovery, FirmwareStopAndLatchBetweenTicksNeedNewExplicitOn)
{
  for (const uint8_t emergency : {EMERGENCY_BIT_STOP, EMERGENCY_BIT_LATCH})
  {
    Peer::control(*node, 1, 1);
    drain();
    Peer::sendCmdVel(*node, 0.2, 0.0);
    drainPackets();
    Peer::bladeStatus(*node, 0);

    Peer::status(*node, emergency);
    EXPECT_FALSE(Peer::bladeIntentAuthorized(*node));
    Peer::status(*node, 0);

    Peer::sendCmdVel(*node, 0.2, 0.0);
    drainPackets();
    Peer::bladeStatus(*node, 0);
    Peer::ageBladeCommand(*node, 1'000'000'000);
    Peer::makeCmdVelFresh(*node);
    Peer::reassertTick(*node);
    EXPECT_TRUE(drain().empty());

    Peer::control(*node, 1, 1);
    EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
    EXPECT_TRUE(Peer::bladeIntentAuthorized(*node));
    Peer::sendCmdVel(*node, 0.2, 0.0);
    drainPackets();
    Peer::bladeStatus(*node, 0);
    Peer::ageBladeCommand(*node, 1'000'000'000);
    Peer::makeCmdVelFresh(*node);
    Peer::reassertTick(*node);
    EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
  }
}

TEST_F(BladeLiftRecovery, BladeReassertRequiresFreshSteadyReceiptAndConnectedStatus)
{
  for (const std::int64_t receipt_offset_ns :
       {std::int64_t{0}, std::int64_t{-2'000'000'000}, std::int64_t{1'000'000'000}})
  {
    Peer::control(*node, 1, 1);
    drain();
    Peer::sendCmdVel(*node, 0.2, 0.0);
    drainPackets();
    Peer::bladeStatus(*node, 0);
    const auto receipt_ns = receipt_offset_ns == 0 ? 0 : Peer::steadyNowNs() + receipt_offset_ns;
    Peer::setBladeStatusReceipt(*node, receipt_ns);
    Peer::ageBladeCommand(*node, 1'000'000'000);
    Peer::makeCmdVelFresh(*node);

    Peer::reassertTick(*node);
    EXPECT_TRUE(drain().empty());
  }

  Peer::control(*node, 1, 1);
  drain();
  Peer::sendCmdVel(*node, 0.2, 0.0);
  drainPackets();
  Peer::bladeStatus(*node, 0);
  Peer::disconnect(*node);
  ASSERT_FALSE(Peer::haveBladeStatus(*node));
  Peer::prepareReassertAfterDisconnect(*node);
  Peer::setRequestedForTest(*node, "sentinel");
  Peer::reassertTick(*node);
  EXPECT_EQ(Peer::requested(*node), "sentinel");
  EXPECT_TRUE(drain().empty());
}

TEST_F(BladeLiftRecovery, ResumesLatestDirectionAndRepeatedOnCannotBypassDelay)
{
  for (const auto direction : {0u, 1u})
  {
    liftAndClear(direction);
    EXPECT_TRUE(Peer::bladeIntentAuthorized(*node));
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
  EXPECT_TRUE(Peer::bladeIntentAuthorized(*node));
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

TEST_F(HardwareBridgeParametersTest, CmdVelSlewLimitsRejectRuntimeUpdate)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device")});
  auto node = std::make_shared<HardwareBridgeNode>(options);

  for (const auto* name : {"cmd_vel_linear_accel_limit",
                           "cmd_vel_linear_decel_limit",
                           "cmd_vel_angular_accel_limit",
                           "cmd_vel_angular_decel_limit"})
  {
    const auto result = node->set_parameter(rclcpp::Parameter(name, 3.0));
    EXPECT_FALSE(result.successful) << name;
  }
}

TEST_F(HardwareBridgeParametersTest, NonPositiveCmdVelSlewLimitFailsStartup)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_port", "/definitely/not/a/serial/device"),
                               rclcpp::Parameter("cmd_vel_angular_accel_limit", 0.0)});

  EXPECT_THROW(std::make_shared<HardwareBridgeNode>(options), std::invalid_argument);
}

}  // namespace mowgli_hardware
