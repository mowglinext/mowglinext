// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#include <memory>

#include "mowgli_gnss_bridge/universal_gnss_topic_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_gnss_bridge::UniversalGnssTopicBridge>());
  rclcpp::shutdown();
  return 0;
}
