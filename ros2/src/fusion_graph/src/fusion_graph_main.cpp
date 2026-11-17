// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/fusion_graph_node.hpp"

// ── Entry point ──────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fusion_graph::FusionGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
