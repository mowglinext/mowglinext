// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Entry point for the led_ring_node executable.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_leds/led_ring_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_leds::LedRingNode>());
  rclcpp::shutdown();
  return 0;
}
