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
 * @file test_wheel_slip_stuck.cpp
 * @brief Unit tests for the IsWheelSlipStuck condition node.
 *
 * Builds a BTContext in-process and fills motion_window the way
 * behavior_tree_node's 5 Hz snapshot timer would, then ticks the node:
 * slip (wheel odom advances, map pose doesn't), stall (cmd integral
 * advances, wheels blocked), normal mowing, disabled toggle, stale/invalid
 * TF, shared cap + cooldown with IsObstacleStuck.
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
using mowgli_behavior::IsWheelSlipStuck;
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
// Fixture
// ---------------------------------------------------------------------------

class IsWheelSlipStuckTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_is_wheel_slip_stuck");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<IsWheelSlipStuck>("IsWheelSlipStuck");

    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsWheelSlipStuck enabled="true" window_sec="4.0"
                            min_commanded_m="0.15" max_displacement_m="0.05"
                            max_count="3" cooldown_sec="8.0"/>
        </BehaviorTree>
      </root>
    )";
    tree = factory.createTreeFromText(xml, blackboard);
  }

  /// Fill motion_window with two endpoint samples the way the 5 Hz
  /// snapshot timer would: `span_sec` apart, ending `end_age_sec` ago,
  /// with the given per-window deltas. Absolute integrals are arbitrary —
  /// only the deltas matter to the detector.
  void fillWindow(double span_sec,
                  double cmd_delta,
                  double wheel_delta,
                  double map_dx,
                  double map_dy,
                  bool oldest_valid = true,
                  bool newest_valid = true,
                  double end_age_sec = 0.0)
  {
    ctx->motion_window.clear();
    const auto now = std::chrono::steady_clock::now();
    const auto newest_t = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(end_age_sec));
    const auto oldest_t =
        newest_t - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(span_sec));
    BTContext::MotionSample oldest;
    oldest.t = oldest_t;
    oldest.cmd_dist = 10.0;
    oldest.wheel_dist = 10.0;
    oldest.map_x = 5.0;
    oldest.map_y = 5.0;
    oldest.map_valid = oldest_valid;
    BTContext::MotionSample newest;
    newest.t = newest_t;
    newest.cmd_dist = 10.0 + cmd_delta;
    newest.wheel_dist = 10.0 + wheel_delta;
    newest.map_x = 5.0 + map_dx;
    newest.map_y = 5.0 + map_dy;
    newest.map_valid = newest_valid;
    ctx->motion_window.push_back(oldest);
    ctx->motion_window.push_back(newest);
  }

  BT::NodeStatus tick()
  {
    return tree.tickOnce();
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Wheels spinning in place (digging on a root): wheel integral advances,
// map pose static → SUCCESS.
TEST_F(IsWheelSlipStuckTest, FiresOnWheelSlip)
{
  fillWindow(4.0, /*cmd*/ 0.6, /*wheel*/ 0.6, /*dx*/ 0.02, /*dy*/ 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);
}

// Wheels BLOCKED by a root (firmware PID stalled): wheel integral ~0 but
// the commanded integral advances → SUCCESS. This is the case a
// wheel-odom-only detector misses.
TEST_F(IsWheelSlipStuckTest, FiresOnStall)
{
  fillWindow(4.0, /*cmd*/ 0.6, /*wheel*/ 0.0, /*dx*/ 0.0, /*dy*/ 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);
}

// Normal mowing: commanded travel matched by real displacement → FAILURE.
TEST_F(IsWheelSlipStuckTest, FailsWhenActuallyMoving)
{
  fillWindow(4.0, 0.6, 0.6, 0.55, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Not commanded to move (idle, pivot, FTC deviation hold): below the
// min-commanded arm threshold → FAILURE even with zero displacement.
TEST_F(IsWheelSlipStuckTest, FailsBelowMinCommanded)
{
  fillWindow(4.0, 0.05, 0.05, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// map→base_footprint TF unavailable at a window endpoint: refuse to judge.
TEST_F(IsWheelSlipStuckTest, FailsWithoutValidMapPose)
{
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0, /*oldest_valid*/ false);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0, true, /*newest_valid*/ false);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Newest sample stale (snapshot timer dead / node just started) → FAILURE.
TEST_F(IsWheelSlipStuckTest, FailsOnStaleWindow)
{
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0, true, true, /*end_age_sec*/ 3.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Window that does not span (~80 % of) window_sec yet — right after startup
// or a purge — must not fire.
TEST_F(IsWheelSlipStuckTest, FailsOnShortSpan)
{
  fillWindow(/*span*/ 1.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Window whose surviving endpoints span FAR more than window_sec (a stalled
// snapshot timer bridging a gap) must not fire either — the commanded
// integral would cover motion the displacement check never saw.
TEST_F(IsWheelSlipStuckTest, FailsOnOverlongSpan)
{
  fillWindow(/*span*/ 8.0, 1.2, 1.2, 0.0, 0.0);  // 8 s >> 1.5 × 4 s window
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Empty / single-sample window → FAILURE.
TEST_F(IsWheelSlipStuckTest, FailsOnEmptyWindow)
{
  ctx->motion_window.clear();
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
}

// enabled=false master toggle.
TEST_F(IsWheelSlipStuckTest, DisabledNeverFires)
{
  static const char* xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <IsWheelSlipStuck enabled="false" window_sec="4.0"
                          min_commanded_m="0.15" max_displacement_m="0.05"
                          max_count="3" cooldown_sec="8.0"/>
      </BehaviorTree>
    </root>
  )";
  BT::BehaviorTreeFactory f2;
  f2.registerNodeType<IsWheelSlipStuck>("IsWheelSlipStuck");
  auto disabled_tree = f2.createTreeFromText(xml, blackboard);
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(disabled_tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 0);
}

// Cooldown: a second wedge within cooldown_sec of the first firing (from
// EITHER trigger — the timestamps are shared with IsObstacleStuck) must wait.
TEST_F(IsWheelSlipStuckTest, RespectsSharedCooldown)
{
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);

  // Same wedge immediately re-judged: cooldown blocks.
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 1);

  // Simulate the cooldown having elapsed.
  ctx->last_obstacle_backoff_time = std::chrono::steady_clock::now() - std::chrono::seconds(9);
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->obstacle_backoff_count, 2);
}

// Per-session cap shared with IsObstacleStuck: firings by the collision
// trigger count against this node's budget too.
TEST_F(IsWheelSlipStuckTest, RespectsSharedSessionCap)
{
  ctx->obstacle_backoff_count = 3;  // as if IsObstacleStuck fired 3× already
  fillWindow(4.0, 0.6, 0.6, 0.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->obstacle_backoff_count, 3);
}
