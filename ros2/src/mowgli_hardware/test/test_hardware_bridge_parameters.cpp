// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

// The bridge is intentionally a single executable. Rename its main while
// including the implementation so this test exercises the real parameter
// descriptors instead of duplicating their behaviour in a helper.
#define main hardware_bridge_node_main_for_test
#include "../src/hardware_bridge_node.cpp"
#undef main

namespace mowgli_hardware
{

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
