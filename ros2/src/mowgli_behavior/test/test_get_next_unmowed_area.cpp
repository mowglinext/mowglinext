// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_get_next_unmowed_area.cpp
 * @brief Regression test: the mow-selection path SKIPS navigation-only areas.
 *
 * GetNextUnmowedArea iterates the map's areas via get_mowing_area. A
 * navigation-only zone (is_navigation_area=true) is a transit corridor, not a
 * mowing target — the blades must never run inside it. map_server still returns
 * these areas (success=true) because the obstacle tracker needs their geometry,
 * so the skip lives on the BT selection side. These tests stand up a REAL
 * in-process get_mowing_area service (no robot, no mocked interfaces) and tick
 * the StatefulActionNode to verify a nav-only area is never selected.
 */

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::GetNextUnmowedArea;
using GetMowingArea = mowgli_interfaces::srv::GetMowingArea;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

// ---------------------------------------------------------------------------
// One entry the fake map_server returns per index: name + nav-only flag.
// index past the last entry → success=false (matches real map_server).
// ---------------------------------------------------------------------------
struct AreaEntry
{
  std::string name;
  bool is_navigation_area;
};

class GetNextUnmowedAreaTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::Service<GetMowingArea>::SharedPtr service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::map<uint32_t, AreaEntry> areas;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_get_next_unmowed_area");
    ctx->helper_node = rclcpp::Node::make_shared("test_get_next_unmowed_area_helper");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<GetNextUnmowedArea>("GetNextUnmowedArea");

    server_node = rclcpp::Node::make_shared("fake_map_server");
    service = server_node->create_service<GetMowingArea>(
        "/map_server_node/get_mowing_area",
        [this](const std::shared_ptr<GetMowingArea::Request> req,
               std::shared_ptr<GetMowingArea::Response> resp)
        {
          auto it = areas.find(req->index);
          if (it == areas.end())
          {
            resp->success = false;  // index past the last defined area
            return;
          }
          resp->area.name = it->second.name;
          resp->area.is_navigation_area = it->second.is_navigation_area;
          resp->success = true;
        });

    executor.add_node(ctx->helper_node);
    executor.add_node(server_node);
  }

  /// Wait for the helper-side client to discover the fake service.
  void waitForService()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      auto client =
          ctx->helper_node->create_client<GetMowingArea>("/map_server_node/get_mowing_area");
      if (client->service_is_ready())
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL() << "fake get_mowing_area service was never discovered";
  }

  /// Tick the node to completion, spinning the executor between ticks so the
  /// async service round-trips complete. Returns the terminal status.
  BT::NodeStatus tickToCompletion(BT::Tree& tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (std::chrono::steady_clock::now() < deadline)
    {
      status = tree.tickOnce();
      if (status != BT::NodeStatus::RUNNING)
      {
        break;
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return status;
  }

  BT::Tree makeTree(uint32_t max_areas)
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<GetNextUnmowedArea max_areas=\"" +
        std::to_string(max_areas) +
        "\" area_index=\"{area_index}\"/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// A navigation-only area at index 0 followed by a mowing area at index 1: the
// selection must SKIP index 0 and pick index 1.
TEST_F(GetNextUnmowedAreaTest, SkipsNavigationOnlyAreaAndSelectsMowingArea)
{
  areas[0] = {"front_path", /*is_navigation_area=*/true};
  areas[1] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);

  uint32_t selected = 99;
  ASSERT_TRUE(blackboard->get("area_index", selected));
  EXPECT_EQ(selected, 1u) << "must skip the nav-only area 0 and select mowing area 1";
  EXPECT_EQ(ctx->current_area, 1);
  // The skipped nav area is marked attempted so it is not re-evaluated.
  EXPECT_GT(ctx->attempted_areas.count(0u), 0u);
}

// The ONLY area is navigation-only: nothing is mowable → FAILURE, and the nav
// area is never selected.
TEST_F(GetNextUnmowedAreaTest, NavigationOnlyAreaIsNeverMowed)
{
  areas[0] = {"perimeter_corridor", /*is_navigation_area=*/true};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->current_area, -1) << "no area should have been selected for mowing";
  EXPECT_GT(ctx->attempted_areas.count(0u), 0u);
}

// Control: an ordinary mowing area at index 0 is selected as before (the skip
// must not regress normal selection).
TEST_F(GetNextUnmowedAreaTest, SelectsMowingAreaAtIndexZero)
{
  areas[0] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);

  uint32_t selected = 99;
  ASSERT_TRUE(blackboard->get("area_index", selected));
  EXPECT_EQ(selected, 0u);
  EXPECT_EQ(ctx->current_area, 0);
}
