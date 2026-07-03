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
 * @file test_set_nav2_lifecycle.cpp
 * @brief Unit tests for the SetNav2Lifecycle action node.
 *
 * Verifies the idle-Nav2-suspend gating logic that decides whether to issue a
 * lifecycle_manager manage_nodes PAUSE/RESUME. The safety-critical guarantees
 * are exercised without any real lifecycle_manager:
 *   - disabled flag  → pure no-op (default field behaviour is unchanged)
 *   - PAUSE off-dock → no-op (collision_monitor is never deactivated unless
 *     the robot is physically charging on the dock)
 *   - RESUME when already active / PAUSE when already paused → idempotent
 * A separate test stands up a fake manage_nodes service to confirm the actual
 * suspend/resume transition flips ctx->nav2_suspended and reaches the server.
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/navigation_nodes.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::SetNav2Lifecycle;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

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
// Fixture — builds a BTContext + blackboard + a SetNav2Lifecycle tree
// ---------------------------------------------------------------------------

class SetNav2LifecycleTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_set_nav2_lifecycle");
    ctx->helper_node = rclcpp::Node::make_shared("test_set_nav2_lifecycle_helper");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<SetNav2Lifecycle>("SetNav2Lifecycle");
  }

  /// Build a one-node tree with the given command ("PAUSE" / "RESUME").
  BT::Tree makeTree(const std::string& command)
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<SetNav2Lifecycle command=\"" +
        command +
        "\"/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }

  void setEnabled(bool enabled)
  {
    blackboard->set("idle_nav2_suspend", enabled);
  }
  void setCharging(bool charging)
  {
    ctx->latest_power.charger_enabled = charging;
  }
};

// ---------------------------------------------------------------------------
// Gating logic (no service required — deterministic)
// ---------------------------------------------------------------------------

TEST_F(SetNav2LifecycleTest, DisabledFlagIsNoOp)
{
  // Feature off (the default): even PAUSE while charging must do nothing.
  setEnabled(false);
  setCharging(true);
  auto tree = makeTree("PAUSE");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->nav2_suspended);
}

TEST_F(SetNav2LifecycleTest, MissingFlagIsNoOp)
{
  // Blackboard key never set → treated as disabled, no transition.
  setCharging(true);
  auto tree = makeTree("PAUSE");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->nav2_suspended);
}

TEST_F(SetNav2LifecycleTest, PauseOffDockIsRefused)
{
  // SAFETY: never PAUSE (which deactivates collision_monitor) unless the
  // robot is physically on the dock charging.
  setEnabled(true);
  setCharging(false);
  auto tree = makeTree("PAUSE");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->nav2_suspended);
}

TEST_F(SetNav2LifecycleTest, ResumeWhenAlreadyActiveIsNoOp)
{
  setEnabled(true);
  ctx->nav2_suspended = false;  // already active
  auto tree = makeTree("RESUME");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->nav2_suspended);
}

TEST_F(SetNav2LifecycleTest, PauseWhenAlreadyPausedIsIdempotent)
{
  setEnabled(true);
  setCharging(true);
  ctx->nav2_suspended = true;  // already paused
  auto tree = makeTree("PAUSE");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->nav2_suspended);
}

TEST_F(SetNav2LifecycleTest, BadCommandFails)
{
  setEnabled(true);
  auto tree = makeTree("NONSENSE");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// Transition with a fake manage_nodes server
// ---------------------------------------------------------------------------

TEST_F(SetNav2LifecycleTest, PauseAndResumeTransitionReachesServer)
{
  uint8_t last_command = 255;
  int call_count = 0;
  auto server_node = rclcpp::Node::make_shared("fake_lifecycle_manager");
  auto service = server_node->create_service<ManageLifecycleNodes>(
      "/lifecycle_manager_navigation/manage_nodes",
      [&](const std::shared_ptr<ManageLifecycleNodes::Request> req,
          std::shared_ptr<ManageLifecycleNodes::Response> resp)
      {
        last_command = req->command;
        ++call_count;
        resp->success = true;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ctx->helper_node);
  executor.add_node(server_node);

  // Let discovery settle so the helper client sees the service.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  // First tick creates the client (lazy); spin until it's discovered.
  setEnabled(true);
  setCharging(true);
  auto pause_tree = makeTree("PAUSE");
  bool ready = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    auto client = ctx->helper_node->create_client<ManageLifecycleNodes>(
        "/lifecycle_manager_navigation/manage_nodes");
    if (client->service_is_ready())
    {
      ready = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(ready) << "fake manage_nodes service was never discovered";

  // PAUSE: charging + enabled + not-yet-suspended → sends PAUSE, flips state.
  EXPECT_EQ(pause_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->nav2_suspended);
  for (int i = 0; i < 50 && call_count < 1; ++i)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(call_count, 1);
  EXPECT_EQ(last_command, ManageLifecycleNodes::Request::PAUSE);

  // RESUME: suspended → sends RESUME, clears state.
  auto resume_tree = makeTree("RESUME");
  EXPECT_EQ(resume_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->nav2_suspended);
  for (int i = 0; i < 50 && call_count < 2; ++i)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(call_count, 2);
  EXPECT_EQ(last_command, ManageLifecycleNodes::Request::RESUME);
}
