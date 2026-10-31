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

// Exercise production docking nodes with real ROS action transport, including
// acceptance after their BT lifetime and restart before cancellation completes.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "mowgli_behavior/docking_nodes.hpp"
#include <gtest/gtest.h>

namespace
{
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

bool WaitFor(const std::function<bool()>& predicate, double seconds = 5.0)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  do
  {
    if (predicate())
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < deadline);
  return predicate();
}

struct DockCase
{
  using Node = mowgli_behavior::DockRobot;
  using Action = nav2_msgs::action::DockRobot;
  static constexpr const char* action = "/dock_robot";
};
struct UndockCase
{
  using Node = mowgli_behavior::UndockRobot;
  using Action = nav2_msgs::action::UndockRobot;
  static constexpr const char* action = "/undock_robot";
};

template <typename Case>
class DockingDispatchTest : public ::testing::Test
{
protected:
  using Action = typename Case::Action;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;

  void SetUp() override
  {
    context = std::make_shared<mowgli_behavior::BTContext>();
    context->node = rclcpp::Node::make_shared("docking_dispatch_client");
    server_node = rclcpp::Node::make_shared("docking_dispatch_server");
    server = rclcpp_action::create_server<Action>(
        server_node,
        Case::action,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>)
        {
          std::unique_lock<std::mutex> lock(mutex);
          request_waiting = true;
          changed.notify_all();
          changed.wait(lock,
                       [this]()
                       {
                         return !hold_accept;
                       });
          return reject_goal ? rclcpp_action::GoalResponse::REJECT
                             : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<Handle>)
        {
          return reject_cancel ? rclcpp_action::CancelResponse::REJECT
                               : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<Handle> handle)
        {
          std::lock_guard<std::mutex> lock(mutex);
          handles.push_back(handle);
          if (finish_on_accept)
          {
            handle->succeed(std::make_shared<typename Action::Result>());
          }
        });
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(context->node);
    executor->add_node(server_node);
    spinner = std::thread(
        [this]()
        {
          executor->spin();
        });
    BT::BehaviorTreeFactory factory;
    factory.template registerNodeType<typename Case::Node>("DispatchNode");
    auto blackboard = BT::Blackboard::create();
    blackboard->set("context", context);
    tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DispatchNode/></BehaviorTree></root>",
        blackboard));
  }

  void TearDown() override
  {
    releaseAcceptance();
    if (tree)
    {
      tree->haltTree();
      tree.reset();
    }
    executor->cancel();
    spinner.join();
  }

  void releaseAcceptance()
  {
    std::lock_guard<std::mutex> lock(mutex);
    hold_accept = false;
    changed.notify_all();
  }
  size_t count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return handles.size();
  }
  std::shared_ptr<Handle> handle(size_t index)
  {
    std::lock_guard<std::mutex> lock(mutex);
    return handles.at(index);
  }
  bool waiting()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return request_waiting;
  }
  void start()
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(WaitFor(
        [this]()
        {
          return count() == 1;
        }));
  }

  std::shared_ptr<mowgli_behavior::BTContext> context;
  rclcpp::Node::SharedPtr server_node;
  typename rclcpp_action::Server<Action>::SharedPtr server;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  std::unique_ptr<BT::Tree> tree;
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::shared_ptr<Handle>> handles;
  bool request_waiting = false;
  std::atomic<bool> hold_accept{false};
  std::atomic<bool> reject_goal{false};
  std::atomic<bool> reject_cancel{false};
  std::atomic<bool> finish_on_accept{false};
};

using Cases = ::testing::Types<DockCase, UndockCase>;
TYPED_TEST_SUITE(DockingDispatchTest, Cases);

TYPED_TEST(DockingDispatchTest, HaltBeforeFirstRunningTickCancelsAcceptedGoal)
{
  this->start();
  // The server accepted, but the BT has never polled its acceptance future.
  this->tree->haltTree();
  EXPECT_FALSE(this->context->docking_active);
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->handle(0)->is_canceling();
      }));
  this->handle(0)->canceled(std::make_shared<typename TestFixture::Action::Result>());
}

TYPED_TEST(DockingDispatchTest, AcceptanceAfterTreeDestructionStillCancels)
{
  this->hold_accept = true;
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->waiting();
      }));
  this->tree->haltTree();
  this->tree.reset();
  this->releaseAcceptance();
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->count() == 1;
      }));
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->handle(0)->is_canceling();
      }));
  this->handle(0)->canceled(std::make_shared<typename TestFixture::Action::Result>());
}

TYPED_TEST(DockingDispatchTest, RestartWaitsForOldResultAndKeepsNewOutcomeIndependent)
{
  this->start();
  this->tree->haltTree();
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->handle(0)->is_canceling();
      }));
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(this->context->docking_active);
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(this->count(), 1u);  // no server preemption while cancellation drains
  }
  this->handle(0)->canceled(std::make_shared<typename TestFixture::Action::Result>());
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        EXPECT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
        return this->count() == 2;
      }));
  EXPECT_FALSE(this->handle(1)->is_canceling());
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  this->handle(1)->succeed(std::make_shared<typename TestFixture::Action::Result>());
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->tree->tickOnce() == BT::NodeStatus::SUCCESS;
      }));
}

TYPED_TEST(DockingDispatchTest, RejectedCancellationCannotStartCompetingGoal)
{
  this->reject_cancel = true;
  this->start();
  this->tree->haltTree();
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->tree->tickOnce() == BT::NodeStatus::FAILURE;
      },
      7.0));
  EXPECT_EQ(this->count(), 1u);
  EXPECT_FALSE(this->context->docking_active);
  // Another entry must retain the old goal identity, even after the timeout.
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(this->count(), 1u);
  this->handle(0)->abort(std::make_shared<typename TestFixture::Action::Result>());
}

TYPED_TEST(DockingDispatchTest, ImmediateTerminalResultStillSucceeds)
{
  this->finish_on_accept = true;
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->tree->tickOnce() == BT::NodeStatus::SUCCESS;
      }));
  EXPECT_FALSE(this->context->docking_active);
}

TYPED_TEST(DockingDispatchTest, ConcurrentResultAndHaltCompleteWithoutWedge)
{
  // Both directions of this race acquire the real rclcpp goal-handle mutex.
  // A result callback can run while halt inspects that handle's terminal state.
  for (size_t index = 0; index < 20; ++index)
  {
    ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(WaitFor(
        [this, index]()
        {
          return this->count() == index + 1;
        }));
    std::atomic<bool> release{false};
    auto handle = this->handle(index);
    std::thread complete(
        [&]()
        {
          while (!release.load())
          {
            std::this_thread::yield();
          }
          handle->succeed(std::make_shared<typename TestFixture::Action::Result>());
        });
    release.store(true);
    this->tree->haltTree();
    complete.join();
    EXPECT_FALSE(this->context->docking_active);
    // Drain the previous result before the next iteration. Restart is allowed
    // to wait, but it must eventually submit exactly one successor goal.
    if (index + 1 < 20)
    {
      ASSERT_TRUE(WaitFor(
          [this, index]()
          {
            EXPECT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
            return this->count() == index + 2;
          }));
      // The successor is already RUNNING; the next iteration's tick continues
      // it rather than restarting the action.
    }
  }
}

TYPED_TEST(DockingDispatchTest, RejectedGoalStillFails)
{
  this->reject_goal = true;
  ASSERT_EQ(this->tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(WaitFor(
      [this]()
      {
        return this->tree->tickOnce() == BT::NodeStatus::FAILURE;
      }));
  EXPECT_FALSE(this->context->docking_active);
}
}  // namespace
