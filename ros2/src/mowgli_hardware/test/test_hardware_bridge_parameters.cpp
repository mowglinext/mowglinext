// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <cstring>

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
  static void ageCmdVel(HardwareBridgeNode& node, std::int64_t age_ns)
  {
    node.last_cmd_vel_packet_ns_ = steadyNowNs() - age_ns;
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
  // Feed a CONFIG_RSP as if the board had answered the version handshake.
  static void handshake(HardwareBridgeNode& node, uint8_t protocol)
  {
    LlConfigRsp packet{};
    packet.type = PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP;
    packet.protocol_version = protocol;
    packet.fw_version_major = 1;
    node.handle_config_rsp(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  }
  static void rearmHandshake(HardwareBridgeNode& node)
  {
    node.rearm_firmware_handshake();
  }
  static bool actuationAllowed(const HardwareBridgeNode& node)
  {
    return node.actuation_allowed();
  }
  static bool controlResult(HardwareBridgeNode& node, uint8_t on, uint8_t direction)
  {
    auto req = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
    req->mow_enabled = on;
    req->mow_direction = direction;
    auto res = std::make_shared<mowgli_interfaces::srv::MowerControl::Response>();
    node.on_mower_control(req, res);
    return res->success;
  }
  static void cmdVel(HardwareBridgeNode& node, double vx, double wz)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->twist.linear.x = vx;
    msg->twist.angular.z = wz;
    node.on_cmd_vel(msg);
  }
  static void setMode(HardwareBridgeNode& node, uint8_t mode)
  {
    node.current_mode_ = mode;
    node.send_high_level_state();
  }
  static uint8_t currentMode(const HardwareBridgeNode& node)
  {
    return node.current_mode_;
  }
  static bool mowEnabled(const HardwareBridgeNode& node)
  {
    return node.mow_enabled_;
  }
  static bool emergencyActive(const HardwareBridgeNode& node)
  {
    return node.emergency_active_;
  }
  static bool digEscapeAllowed(const HardwareBridgeNode& node)
  {
    return node.dig_escape_allowed();
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
  Peer::handshake(*node, kMowgliProtocolVersion);
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
    // The board under test speaks this image's protocol; the firmware
    // interlock is exercised separately (FirmwareInterlock* below).
    Peer::handshake(*node, kMowgliProtocolVersion);
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

TEST_F(BladeLiftRecovery, StaleVelocityGetsAZeroPacketBeforeExplicitBladeOn)
{
  Peer::control(*node, 1, 1);

  const auto packets = drainPackets();
  ASSERT_EQ(packets.size(), 2u);
  ASSERT_EQ(packets[0].size(), sizeof(LlCmdVel));
  ASSERT_EQ(packets[1].size(), sizeof(LlCmdBlade));
  EXPECT_EQ(packets[0][0], PACKET_ID_LL_CMD_VEL);
  EXPECT_EQ(packets[1][0], PACKET_ID_LL_CMD_BLADE);
  LlCmdVel velocity{};
  std::memcpy(&velocity, packets[0].data(), sizeof(velocity));
  EXPECT_FLOAT_EQ(velocity.linear_x, 0.0f);
  EXPECT_FLOAT_EQ(velocity.angular_z, 0.0f);
  LlCmdBlade blade{};
  std::memcpy(&blade, packets[1].data(), sizeof(blade));
  EXPECT_EQ(blade.blade_on, 1u);
  EXPECT_EQ(blade.blade_dir, 1u);
}

TEST_F(BladeLiftRecovery, ReassertAfterIdleRequiresTheVelocityProducerToResume)
{
  Peer::control(*node, 1, 1);
  drainPackets();
  Peer::bladeStatus(*node, 0);
  Peer::ageBladeCommand(*node, 1'000'000'000);
  // Include the firmware's 25s blade-intent expiry: fresh stopped telemetry
  // and cached ON intent must not manufacture a zero CMD_VEL or restart it.
  for (const std::int64_t age_ns : {1'000'000'000LL, 26'000'000'000LL})
  {
    Peer::ageCmdVel(*node, age_ns);
    const auto before = Peer::cmdVelState(*node);
    Peer::reassertTick(*node);
    EXPECT_TRUE(drainPackets().empty());
    EXPECT_EQ(Peer::cmdVelState(*node).sent_ns, before.sent_ns);
  }

  // A real producer's fresh velocity restores the existing retry eligibility.
  Peer::sendCmdVel(*node, 0.2, 0.0);
  ASSERT_EQ(drainPackets().size(), 1u);
  Peer::reassertTick(*node);
  EXPECT_EQ(drain(), (std::vector<std::pair<uint8_t, uint8_t>>{{1, 1}}));
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

// ---------------------------------------------------------------------------
// Firmware actuation interlock (firmware_interlock.hpp), on the real node.
// ---------------------------------------------------------------------------

class BridgeFirmwareInterlock : public BladeLiftRecovery
{
protected:
  void SetUp() override
  {
    BladeLiftRecovery::SetUp();
    // Start every case from an un-handshaken board, like a fresh boot.
    Peer::rearmHandshake(*node);
    drainPackets();
  }

  std::vector<LlCmdVel> cmdVels(const std::vector<std::vector<uint8_t>>& packets)
  {
    std::vector<LlCmdVel> out;
    for (const auto& p : packets)
    {
      if (p.size() == sizeof(LlCmdVel) && p[0] == PACKET_ID_LL_CMD_VEL)
      {
        LlCmdVel v{};
        std::memcpy(&v, p.data(), sizeof(v));
        out.push_back(v);
      }
    }
    return out;
  }
  std::vector<uint8_t> blades(const std::vector<std::vector<uint8_t>>& packets)
  {
    std::vector<uint8_t> out;
    for (const auto& p : packets)
      if (p.size() == sizeof(LlCmdBlade) && p[0] == PACKET_ID_LL_CMD_BLADE)
        out.push_back(p[1]);
    return out;
  }
  std::vector<uint8_t> modes(const std::vector<std::vector<uint8_t>>& packets)
  {
    std::vector<uint8_t> out;
    for (const auto& p : packets)
      if (p.size() == sizeof(LlHighLevelState) && p[0] == PACKET_ID_LL_HIGH_LEVEL_STATE)
        out.push_back(p[1]);
    return out;
  }
  std::vector<LlHeartbeat> heartbeats(const std::vector<std::vector<uint8_t>>& packets)
  {
    std::vector<LlHeartbeat> out;
    for (const auto& p : packets)
    {
      if (p.size() == sizeof(LlHeartbeat) && p[0] == PACKET_ID_LL_HEARTBEAT)
      {
        LlHeartbeat h{};
        std::memcpy(&h, p.data(), sizeof(h));
        out.push_back(h);
      }
    }
    return out;
  }
};

TEST_F(BridgeFirmwareInterlock, NoHandshakeYetSendsZeroForEveryNonZeroCmdVel)
{
  ASSERT_FALSE(Peer::actuationAllowed(*node));
  Peer::cmdVel(*node, 0.3, 0.5);
  Peer::sendCmdVel(*node, -0.2, 0.0);  // the dig-escape / internal path

  const auto sent = cmdVels(drainPackets());
  ASSERT_EQ(sent.size(), 2u);
  for (const auto& v : sent)
  {
    EXPECT_EQ(v.linear_x, 0.0f);
    EXPECT_EQ(v.angular_z, 0.0f);
  }
}

TEST_F(BridgeFirmwareInterlock, CmdVelCannotForceAModeChangeWhileIncompatible)
{
  ASSERT_EQ(Peer::currentMode(*node), HL_MODE_NULL);
  Peer::cmdVel(*node, 0.3, 0.0);
  EXPECT_EQ(Peer::currentMode(*node), HL_MODE_NULL);
  EXPECT_TRUE(modes(drainPackets()).empty());
}

TEST_F(BridgeFirmwareInterlock, WrongProtocolRefusesBladeOnAndReportsFailure)
{
  Peer::handshake(*node, static_cast<uint8_t>(kMowgliProtocolVersion - 1u));
  drainPackets();
  ASSERT_FALSE(Peer::actuationAllowed(*node));

  EXPECT_FALSE(Peer::controlResult(*node, 1, 0));
  EXPECT_FALSE(Peer::mowEnabled(*node));
  const auto packets = drainPackets();
  EXPECT_EQ(blades(packets), (std::vector<uint8_t>{0u}));
  for (const auto& v : cmdVels(packets))
    EXPECT_EQ(v.linear_x, 0.0f);

  // A direct ON (re-assert / lift-resume path) is forced OFF too.
  Peer::send(*node, 1, 0);
  EXPECT_EQ(blades(drainPackets()), (std::vector<uint8_t>{0u}));
  // An OFF request still succeeds.
  EXPECT_TRUE(Peer::controlResult(*node, 0, 0));
}

TEST_F(BridgeFirmwareInterlock, NonIdleModeIsForwardedAsIdleWhileIncompatible)
{
  Peer::setMode(*node, HL_MODE_MANUAL_MOWING);
  Peer::setMode(*node, HL_MODE_AUTONOMOUS);
  EXPECT_EQ(modes(drainPackets()), (std::vector<uint8_t>{HL_MODE_IDLE, HL_MODE_IDLE}));
}

TEST_F(BridgeFirmwareInterlock, EmergencyStopAndReleaseStillReachTheFirmware)
{
  Peer::emergency(*node, 1);
  Peer::emergency(*node, 0);
  const auto beats = heartbeats(drainPackets());
  ASSERT_EQ(beats.size(), 2u);
  EXPECT_EQ(beats[0].emergency_requested, 1u);
  EXPECT_EQ(beats[1].emergency_requested, 0u);
  EXPECT_EQ(beats[1].emergency_release_requested, 1u);
}

TEST_F(BridgeFirmwareInterlock, DigEscapeIsNotAllowedWhileIncompatible)
{
  EXPECT_FALSE(Peer::digEscapeAllowed(*node));
  Peer::handshake(*node, kMowgliProtocolVersion);
  EXPECT_TRUE(Peer::digEscapeAllowed(*node));
}

TEST_F(BridgeFirmwareInterlock, MatchingHandshakeRestoresUnchangedBehaviour)
{
  Peer::setMode(*node, HL_MODE_AUTONOMOUS);
  drainPackets();
  Peer::handshake(*node, kMowgliProtocolVersion);
  ASSERT_TRUE(Peer::actuationAllowed(*node));
  // Release re-sends the BT's real mode immediately.
  EXPECT_EQ(modes(drainPackets()), (std::vector<uint8_t>{HL_MODE_AUTONOMOUS}));

  Peer::sendCmdVel(*node, 0.25, -0.5);
  const auto sent = cmdVels(drainPackets());
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_EQ(sent[0].linear_x, 0.25f);
  EXPECT_EQ(sent[0].angular_z, -0.5f);

  EXPECT_TRUE(Peer::controlResult(*node, 1, 1));
  EXPECT_EQ(blades(drainPackets()), (std::vector<uint8_t>{1u}));
}

TEST_F(BridgeFirmwareInterlock, CompatibleToIncompatibleForcesBladeOffZeroAndIdle)
{
  Peer::handshake(*node, kMowgliProtocolVersion);
  Peer::setMode(*node, HL_MODE_AUTONOMOUS);
  ASSERT_TRUE(Peer::controlResult(*node, 1, 0));
  Peer::sendCmdVel(*node, 0.3, 0.0);
  drainPackets();

  // The board re-handshakes with a different protocol (e.g. reflashed to an
  // older image without a USB re-enumeration being noticed).
  Peer::handshake(*node, static_cast<uint8_t>(kMowgliProtocolVersion + 1u));
  ASSERT_FALSE(Peer::actuationAllowed(*node));
  const auto packets = drainPackets();
  EXPECT_EQ(blades(packets), (std::vector<uint8_t>{0u}));
  const auto sent = cmdVels(packets);
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_EQ(sent[0].linear_x, 0.0f);
  EXPECT_EQ(modes(packets), (std::vector<uint8_t>{HL_MODE_IDLE}));
  EXPECT_FALSE(Peer::mowEnabled(*node));

  // The re-assert path cannot spin it back up on its own.
  Peer::bladeStatus(*node, 0);
  Peer::ageBladeCommand(*node, 1'000'000'000);
  Peer::makeCmdVelFresh(*node);
  Peer::reassertTick(*node);
  EXPECT_TRUE(blades(drainPackets()).empty());
}

TEST_F(BridgeFirmwareInterlock, RearmOnReconnectEngagesTheInterlock)
{
  Peer::handshake(*node, kMowgliProtocolVersion);
  ASSERT_TRUE(Peer::actuationAllowed(*node));
  drainPackets();
  Peer::rearmHandshake(*node);
  EXPECT_FALSE(Peer::actuationAllowed(*node));
  EXPECT_EQ(blades(drainPackets()), (std::vector<uint8_t>{0u}));
}

}  // namespace mowgli_hardware
