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
/**
 * @file test_cancel_on_halt.cpp
 * @brief Halting a node whose action goal already finished must not throw.
 *
 * Field 2026-09-21/22: the robot reached the charger in the same moment
 * DockRobot's goal finished. HomeOrAlreadyDocked (a ReactiveFallback) saw the
 * robot docked and halted DockRobot, whose onHalted called async_cancel_goal on
 * a handle the client had already forgotten. rclcpp_action threw "Goal handle
 * is not known to this client" before the handle was reset, the node stayed
 * RUNNING, and the tree threw again on every tick — 73 840 times in 10 h,
 * frozen in RETURNING_HOME on the dock.
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/cancel_goal.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::cancelGoalQuietly;
using mowgli_behavior::DockRobot;
using Dock = nav2_msgs::action::DockRobot;

namespace
{

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

/// Fake docking_server: accepts every goal, records it, lets the test end it.
class FakeDockServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Dock>;

  explicit FakeDockServer(const rclcpp::Node::SharedPtr& node)
  {
    server_ = rclcpp_action::create_server<Dock>(
        node,
        "/dock_robot",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Dock::Goal>)
        {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<Handle>)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<Handle> handle)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          handles_.push_back(handle);
        });
  }

  std::size_t goalCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.size();
  }

  bool isCanceling(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.at(i)->is_canceling();
  }

  void succeed(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->succeed(std::make_shared<Dock::Result>());
  }

private:
  rclcpp_action::Server<Dock>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> handles_;
};

bool waitFor(const std::function<bool()>& done, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (done())
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return done();
}

}  // namespace

class CancelOnHaltTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_cancel_on_halt");
    server_node = rclcpp::Node::make_shared("fake_docking_server");
    dock = std::make_unique<FakeDockServer>(server_node);
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(ctx->node);
    executor->add_node(server_node);
    spinner = std::thread(
        [this]()
        {
          executor->spin();
        });
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<DockRobot>("DockRobot");
  }

  void TearDown() override
  {
    tree.reset();
    executor->cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    dock.reset();
  }

  /// A goal on the fake server, with the result already delivered to `client`
  /// (the state in which rclcpp_action has forgotten the handle).
  rclcpp_action::ClientGoalHandle<Dock>::SharedPtr finishedGoal(
      const rclcpp_action::Client<Dock>::SharedPtr& client)
  {
    auto result_seen = std::make_shared<std::atomic<bool>>(false);
    rclcpp_action::Client<Dock>::SendGoalOptions opts;
    opts.result_callback = [result_seen](const auto&)
    {
      result_seen->store(true);
    };
    auto future = client->async_send_goal(Dock::Goal{}, opts);
    EXPECT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    auto handle = future.get();
    EXPECT_TRUE(handle);
    EXPECT_TRUE(waitFor(
        [&]()
        {
          return dock->goalCount() == 1;
        },
        5.0));
    dock->succeed(0);
    EXPECT_TRUE(waitFor(
        [&]()
        {
          return result_seen->load();
        },
        5.0));
    return handle;
  }

  std::shared_ptr<BTContext> ctx;
  rclcpp::Node::SharedPtr server_node;
  std::unique_ptr<FakeDockServer> dock;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
};

TEST_F(CancelOnHaltTest, RclcppThrowsOnAFinishedGoalButTheHelperDoesNot)
{
  // Arrange
  auto client = rclcpp_action::create_client<Dock>(ctx->node, "/dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  auto handle = finishedGoal(client);
  ASSERT_TRUE(handle);

  // Act + Assert: the raw call is what wedged the tree...
  EXPECT_THROW(client->async_cancel_goal(handle),
               rclcpp_action::exceptions::UnknownGoalHandleError);
  // ...the helper reports "nothing to cancel" instead.
  bool sent = true;
  EXPECT_NO_THROW(sent = cancelGoalQuietly(client, handle, ctx->node->get_logger(), "test"));
  EXPECT_FALSE(sent);
}

TEST_F(CancelOnHaltTest, TheHelperStillCancelsAnActiveGoal)
{
  // Arrange
  auto client = rclcpp_action::create_client<Dock>(ctx->node, "/dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  auto future = client->async_send_goal(Dock::Goal{});
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto handle = future.get();
  ASSERT_TRUE(handle);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));

  // Act
  const bool sent = cancelGoalQuietly(client, handle, ctx->node->get_logger(), "test");

  // Assert
  EXPECT_TRUE(sent);
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return dock->isCanceling(0);
      },
      5.0));
}

TEST_F(CancelOnHaltTest, DockRobotHaltedAfterItsGoalFinishedDoesNotWedgeTheTree)
{
  // Arrange: DockRobot running with a confirmed goal handle.
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>",
      blackboard));
  for (int i = 0; i < 100 && dock->goalCount() == 0; ++i)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(dock->goalCount(), 1u);
  for (int i = 0; i < 5; ++i)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);  // takes the goal handle
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // The docking server finishes, and its result reaches the client, while the
  // tree is not ticking DockRobot (the charger was detected first).
  dock->succeed(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Act + Assert: the parent halts DockRobot. It must not throw...
  EXPECT_NO_THROW(tree->haltTree());
  EXPECT_FALSE(ctx->docking_active);
  // ...and the node must be reusable: the next tick starts a fresh goal
  // instead of throwing again.
  EXPECT_NO_THROW(tree->tickOnce());
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 2;
      },
      5.0))
      << "DockRobot did not start again after the halt";
}
