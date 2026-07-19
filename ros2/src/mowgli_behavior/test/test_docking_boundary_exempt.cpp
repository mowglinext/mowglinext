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
 * @file test_docking_boundary_exempt.cpp
 * @brief Unit tests for IsDocking and the BoundaryGuard dock-transit exemption.
 *
 * Regression for the "Mowing Complete → robot stops short of the dock" bug:
 * DockRobot runs while current_command is still 1, the dock sits outside the
 * perimeter so /boundary_violation asserts, and the SoftBoundaryHandler used to
 * halt the in-flight dock. The fix exempts the blade-off dock transit
 * (command 1 AND IsDocking) from BoundaryGuard while keeping FULL boundary
 * protection during blade-on mowing (command 1 AND NOT IsDocking).
 *
 * These tests exercise (a) the IsDocking condition directly and (b) the exact
 * whitelist logic of the BoundaryGuard ReactiveFallback, with a stand-in
 * (AlwaysFailure) for the boundary handlers so we can assert whether the guard
 * short-circuits (exempt) or falls through to the handler.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsBoundaryViolation;
using mowgli_behavior::IsCommand;
using mowgli_behavior::IsDocking;

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
// IsDocking condition — pure read of ctx->docking_active
// ---------------------------------------------------------------------------

class IsDockingTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_is_docking");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<IsDocking>("IsDocking");

    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsDocking/>
        </BehaviorTree>
      </root>
    )";
    tree = factory.createTreeFromText(xml, blackboard);
  }

  BT::NodeStatus tick()
  {
    return tree.tickOnce();
  }
};

TEST_F(IsDockingTest, FailsWhenNotDocking)
{
  ctx->docking_active = false;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
}

TEST_F(IsDockingTest, SucceedsWhenDocking)
{
  ctx->docking_active = true;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
}

TEST_F(IsDockingTest, NoSideEffectsOnContext)
{
  ctx->docking_active = true;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  // Pure read — the flag is owned by DockRobot, never mutated by IsDocking.
  EXPECT_TRUE(ctx->docking_active);
}

// ---------------------------------------------------------------------------
// BoundaryGuard whitelist logic — mirrors the ReactiveFallback ordering from
// main_tree.xml (charging omitted; the command-1 dock-transit exemption + the
// boundary-violation inverter + a stand-in handler are what we assert on).
// ---------------------------------------------------------------------------

class BoundaryGuardExemptTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_boundary_guard_exempt");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<IsCommand>("IsCommand");
    factory.registerNodeType<IsBoundaryViolation>("IsBoundaryViolation");
    factory.registerNodeType<IsDocking>("IsDocking");

    // A ReactiveFallback returns SUCCESS when a whitelist child short-circuits
    // (i.e. BoundaryGuard is exempt / handler skipped), and FAILURE only when
    // it falls through to BoundaryHandler (the AlwaysFailure stand-in). So:
    //   tick() == SUCCESS  ->  handler SKIPPED (robot keeps driving)
    //   tick() == FAILURE  ->  handler ENGAGED (robot stopped / recovered)
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <ReactiveFallback name="BoundaryGuard">
            <IsCommand command="2"/>
            <Sequence name="DockTransitExempt">
              <IsCommand command="1"/>
              <IsDocking/>
            </Sequence>
            <Inverter><IsBoundaryViolation/></Inverter>
            <AlwaysFailure/>
          </ReactiveFallback>
        </BehaviorTree>
      </root>
    )";
    tree = factory.createTreeFromText(xml, blackboard);
  }

  BT::NodeStatus tick()
  {
    return tree.tickOnce();
  }
};

// The exact bug: mowing-complete dock transit (command 1, docking, outside the
// perimeter) MUST short-circuit so DockRobot is not cancelled.
TEST_F(BoundaryGuardExemptTest, ExemptsDockTransitUnderCommand1)
{
  ctx->current_command = 1;
  ctx->docking_active = true;
  ctx->boundary_violation = true;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);  // handler skipped
}

// SAFETY: blade-on mowing (command 1, NOT docking) with a real boundary
// violation MUST still engage the handler — protection is unchanged.
TEST_F(BoundaryGuardExemptTest, EngagesHandlerForBladeOnMowingViolation)
{
  ctx->current_command = 1;
  ctx->docking_active = false;
  ctx->boundary_violation = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);  // handler engaged
}

// Normal blade-on mowing INSIDE the polygon: no violation, guard passes via the
// boundary-violation inverter regardless of docking state.
TEST_F(BoundaryGuardExemptTest, PassesWhenNoViolation)
{
  ctx->current_command = 1;
  ctx->docking_active = false;
  ctx->boundary_violation = false;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
}

// HOME (command 2) remains whitelisted wholesale — unchanged by this fix.
TEST_F(BoundaryGuardExemptTest, HomeCommandStillWhitelisted)
{
  ctx->current_command = 2;
  ctx->docking_active = false;
  ctx->boundary_violation = true;
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);  // handler skipped
}

// docking_active must NOT leak the exemption to an unrelated command that is
// not whitelisted: with a violation and no docking, the handler engages.
TEST_F(BoundaryGuardExemptTest, NoExemptionForUnrelatedCommandWithoutDocking)
{
  ctx->current_command = 4;  // COMMAND_S2 — not whitelisted
  ctx->docking_active = false;
  ctx->boundary_violation = true;
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);  // handler engaged
}
