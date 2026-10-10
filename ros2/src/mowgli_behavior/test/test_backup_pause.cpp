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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/navigation_nodes.hpp"
#include <gtest/gtest.h>

namespace mowgli_behavior
{

// Server acceptance alone does not prove that the client's response arrived,
// or that a BT tick harvested its goal handle. This peer only reads state.
struct BackUpTestAccess
{
  static bool GoalResponseReady(const BackUp& node)
  {
    return node.goal_handle_future_.valid() &&
           node.goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  static bool HasHarvestedHandle(const BackUp& node)
  {
    return static_cast<bool>(node.goal_handle_);
  }
};

}  // namespace mowgli_behavior

namespace
{

using Action = nav2_msgs::action::BackUp;
using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const ros_environment =
    ::testing::AddGlobalTestEnvironment(new RosEnvironment());

class BackupPauseTest : public ::testing::Test
{
protected:
  std::shared_ptr<mowgli_behavior::BTContext> context;
  rclcpp::Node::SharedPtr server_node;
  rclcpp_action::Server<Action>::SharedPtr server;
  rclcpp::executors::SingleThreadedExecutor client_executor;
  rclcpp::executors::SingleThreadedExecutor server_executor;
  std::vector<std::shared_ptr<ServerHandle>> goals;
  bool reject_goals = false;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;

  void SetUp() override
  {
    context = std::make_shared<mowgli_behavior::BTContext>();
    context->node = rclcpp::Node::make_shared("backup_pause_client");
    server_node = rclcpp::Node::make_shared("backup_pause_server");
    server = rclcpp_action::create_server<Action>(
        server_node,
        "/backup",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>)
        {
          return reject_goals ? rclcpp_action::GoalResponse::REJECT
                              : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<ServerHandle>&)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<ServerHandle>& handle)
        {
          goals.push_back(handle);
        });
    client_executor.add_node(context->node);
    server_executor.add_node(server_node);
    blackboard = BT::Blackboard::create();
    blackboard->set("context", context);
    blackboard->set("distance", 1.0);
    factory.registerNodeType<mowgli_behavior::BackUp>("BackUp");
    tree = std::make_unique<BT::Tree>(
        factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">"
                                   "<BackUp backup_dist=\"{distance}\" backup_speed=\"0.15\"/>"
                                   "</BehaviorTree></root>",
                                   blackboard));
  }

  void TearDown() override
  {
    tree->haltTree();
    tree.reset();
    client_executor.remove_node(context->node);
    server_executor.remove_node(server_node);
    server.reset();
    goals.clear();
  }

  bool spinUntil(const std::function<bool()>& predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do
    {
      server_executor.spin_some();
      client_executor.spin_some();
      if (predicate())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  bool canceling(double distance) const
  {
    for (const auto& handle : goals)
      if (std::abs(handle->get_goal()->target.x + distance) < 1e-6)
        return handle->is_canceling();
    return false;
  }

  mowgli_behavior::BackUp& backup()
  {
    return dynamic_cast<mowgli_behavior::BackUp&>(*tree->rootNode());
  }

  bool responseReady()
  {
    return mowgli_behavior::BackUpTestAccess::GoalResponseReady(backup());
  }
};

TEST_F(BackupPauseTest, HaltBeforeServerAcceptanceCancelsTheLateAcceptedGoal)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  // Neither executor has run: the server cannot have accepted the request,
  // and the BT cannot have harvested a client goal handle.
  ASSERT_TRUE(goals.empty());
  ASSERT_FALSE(responseReady());
  tree->haltTree();
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return canceling(1.0);
      }));
  EXPECT_EQ(goals.size(), 1u);
}

TEST_F(BackupPauseTest, HaltAfterAcceptanceBeforeTheNextBtTickStillCancels)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return goals.size() == 1 && responseReady();
      }));
  // The BT's onRunning has deliberately never run. Acceptance can arrive
  // between ticks, so cancellation cannot rely on its polled member handle.
  ASSERT_FALSE(mowgli_behavior::BackUpTestAccess::HasHarvestedHandle(backup()));
  tree->haltTree();
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return canceling(1.0);
      }));
}

TEST_F(BackupPauseTest, HaltWithAHarvestedHandleStillCancels)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return goals.size() == 1 && responseReady();
      }));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(mowgli_behavior::BackUpTestAccess::HasHarvestedHandle(backup()));
  tree->haltTree();
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return canceling(1.0);
      }));
}

TEST_F(BackupPauseTest, LateAcceptanceFromAHaltedAttemptDoesNotCancelItsReplacement)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(responseReady());
  tree->haltTree();
  blackboard->set("distance", 2.0);
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(responseReady());
  // Both responses are delivered only after the replacement is running.
  // Cancellation must retain the identity of the halted attempt.
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return goals.size() == 2 && canceling(1.0);
      }));
  EXPECT_FALSE(canceling(2.0));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tree->haltTree();
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return canceling(2.0);
      }));
}

TEST_F(BackupPauseTest, RejectedGoalStillReturnsFailure)
{
  reject_goals = true;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return responseReady();
      }));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(goals.empty());
}

TEST_F(BackupPauseTest, SuccessfulGoalStillReturnsSuccess)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return goals.size() == 1 && responseReady();
      }));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  goals.front()->succeed(std::make_shared<Action::Result>());
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  ASSERT_TRUE(spinUntil(
      [this, &status]
      {
        status = tree->tickOnce();
        return status != BT::NodeStatus::RUNNING;
      }));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BackupPauseTest, AbortedGoalStillReturnsFailure)
{
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(spinUntil(
      [this]
      {
        return goals.size() == 1 && responseReady();
      }));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  goals.front()->abort(std::make_shared<Action::Result>());
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  ASSERT_TRUE(spinUntil(
      [this, &status]
      {
        status = tree->tickOnce();
        return status != BT::NodeStatus::RUNNING;
      }));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

}  // namespace
