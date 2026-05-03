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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_obstacle_recovery.cpp
 * @brief Unit tests for IsObstacleStuck condition node.
 *
 * Drives the condition node tick() across the time window, the per-session
 * cap, and the cooldown. BTContext is built directly in-process and the
 * collision_monitor fields are set as a callback would set them.
 */

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsObstacleStuck;
using CollisionMonitorState = nav2_msgs::msg::CollisionMonitorState;

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
// Fixture — builds a BTContext + blackboard + IsObstacleStuck instance
// ---------------------------------------------------------------------------

class IsObstacleStuckTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_is_obstacle_stuck");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<IsObstacleStuck>("IsObstacleStuck");

    // Wrap in a minimal tree so port defaults are honoured by BT.CPP.
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsObstacleStuck min_duration_sec="5.0" max_count="3" cooldown_sec="8.0"/>
        </BehaviorTree>
      </root>
    )";
    tree = factory.createTreeFromText(xml, blackboard);
  }

  /// Set collision_monitor_state to STOP, with collision_stop_since
  /// at `seconds_ago` in the past. Mirrors what the subscriber callback
  /// in behavior_tree_node.cpp does on a STOP transition.
  void setStuckFor(double seconds_ago)
  {
    ctx->collision_action_type = CollisionMonitorState::STOP;
    ctx->collision_stop_since =
        std::chrono::steady_clock::now() -
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds_ago));
  }

  void clearStop()
  {
    ctx->collision_action_type = CollisionMonitorState::DO_NOTHING;
    ctx->collision_stop_since = std::chrono::steady_clock::time_point{};
  }

  BT::NodeStatus tick()
  {
    return tree.tickOnce();
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_F(IsObstacleStuckTest, FailsWhenNotInStop)
{
  clearStop();
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

TEST_F(IsObstacleStuckTest, FailsWhenStopBelowMinDuration)
{
  setStuckFor(2.0);  // <5 s
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

TEST_F(IsObstacleStuckTest, SucceedsWhenStopExceedsMinDuration)
{
  setStuckFor(6.0);  // >5 s
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);
  // last_obstacle_backoff_time must have been stamped on the SUCCESS path.
  EXPECT_NE(ctx->last_obstacle_backoff_time.time_since_epoch().count(), 0);
}

TEST_F(IsObstacleStuckTest, IncrementsCounterOnlyOnSuccessPath)
{
  // Several FAILURE-path ticks must not bump the counter.
  clearStop();
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);

  setStuckFor(2.0);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);

  // Now cross the threshold — exactly one SUCCESS, then cooldown blocks
  // further fires.
  setStuckFor(6.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);
}

TEST_F(IsObstacleStuckTest, FailsWhileInCooldown)
{
  // First fire succeeds.
  setStuckFor(6.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);

  // Immediately re-tick with stop still active — cooldown (8 s) blocks.
  setStuckFor(6.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);
}

TEST_F(IsObstacleStuckTest, SucceedsAgainAfterCooldownElapsed)
{
  // Pretend the previous backoff was 9 s ago (>cooldown=8 s).
  ctx->obstacle_backoff_count = 1;
  ctx->last_obstacle_backoff_time =
      std::chrono::steady_clock::now() - std::chrono::seconds(9);

  setStuckFor(6.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 2);
}

TEST_F(IsObstacleStuckTest, FailsWhenAtMaxCount)
{
  // Already hit cap — and pretend cooldown long expired so only the cap
  // matters.
  ctx->obstacle_backoff_count = 3;
  ctx->last_obstacle_backoff_time =
      std::chrono::steady_clock::now() - std::chrono::seconds(60);

  setStuckFor(60.0);  // wedged for a minute, doesn't matter
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 3);  // unchanged
}

TEST_F(IsObstacleStuckTest, GuardsAgainstStopTypeWithUnsetTimestamp)
{
  // Defensive: if action_type=STOP but collision_stop_since was never
  // set (subscriber missed the transition), the node should not fire.
  ctx->collision_action_type = CollisionMonitorState::STOP;
  ctx->collision_stop_since = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}
