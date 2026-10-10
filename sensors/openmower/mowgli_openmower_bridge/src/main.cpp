// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_openmower_bridge/openmower_bridge_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try
  {
    auto node = std::make_shared<mowgli_openmower_bridge::OpenMowerBridgeNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("hardware_bridge"), "OpenMower bridge failed: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
