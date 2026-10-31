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
 * @file test_goto_navigate_to_pose.cpp
 * @brief The NavigateToPose BT node as used by COMMAND_GOTO, against a fake
 *        /navigate_to_pose server.
 *
 * NavigateToPose must not leave an orphaned goal (same class as #943 / #972: a
 * transit goal sent, then abandoned before the server answered, was accepted
 * with no owner and kept bt_navigator busy for seven minutes). These tests pin
 * that a goal halted before the server's answer is cancelled on acceptance,
 * and that the GOTO target is sent through the default bt_navigator tree
 * (empty behavior_tree → AuthorizedTransitPlanner).
 */

#include <chrono>
#include <cmath>
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
#include "mowgli_behavior/navigation_nodes.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::NavigateToPose;
using Nav = nav2_msgs::action::NavigateToPose;

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

/// Fake bt_navigator: accepts every goal after `accept_delay`, records it, lets the test end it.
class FakeNavServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Nav>;

  explicit FakeNavServer(const rclcpp::Node::SharedPtr& node)
  {
    server_ = rclcpp_action::create_server<Nav>(
        node,
        "/navigate_to_pose",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Nav::Goal>)
        {
          std::this_thread::sleep_for(accept_delay);
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

  std::chrono::milliseconds accept_delay{0};

  std::size_t goalCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.size();
  }

  std::shared_ptr<const Nav::Goal> goal(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.at(i)->get_goal();
  }

  bool isCanceling(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.at(i)->is_canceling();
  }

  void succeed(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->succeed(std::make_shared<Nav::Result>());
  }

  /// Completes a cancel request the way bt_navigator does.
  void confirmCancel(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->canceled(std::make_shared<Nav::Result>());
  }

  void abort(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->abort(std::make_shared<Nav::Result>());
  }

private:
  rclcpp_action::Server<Nav>::SharedPtr server_;
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

class GotoNavigateToPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_goto_navigate_to_pose");
    server_node = rclcpp::Node::make_shared("fake_bt_navigator");
    nav = std::make_unique<FakeNavServer>(server_node);
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
    factory.registerNodeType<NavigateToPose>("NavigateToPose");
  }

  void TearDown() override
  {
    tree.reset();
    executor->cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    nav.reset();
  }

  void setGotoTarget(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    ctx->goto_target_pose = pose;
  }

  void loadGotoNode()
  {
    tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><NavigateToPose/></BehaviorTree></root>",
        blackboard));
  }

  /// Tick until the node has the goal handle from the server.
  void tickUntilAccepted()
  {
    ASSERT_TRUE(waitFor(
        [&]()
        {
          return nav->goalCount() == 1;
        },
        5.0));
    for (int i = 0; i < 5; ++i)
    {
      ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::shared_ptr<BTContext> ctx;
  rclcpp::Node::SharedPtr server_node;
  std::unique_ptr<FakeNavServer> nav;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
};

TEST_F(GotoNavigateToPoseTest, SendsGotoTargetThroughTheDefaultNavigatorTree)
{
  setGotoTarget(4.0, -1.5, 0.7);
  loadGotoNode();

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->goalCount() == 1;
      },
      5.0));
  const auto goal = nav->goal(0);
  EXPECT_EQ(goal->pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(goal->pose.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(goal->pose.pose.position.y, -1.5);
  EXPECT_NEAR(goal->pose.pose.orientation.z, std::sin(0.35), 1e-9);
  // Empty → bt_navigator's default tree → AuthorizedTransitPlanner (never BoundaryRecovery).
  EXPECT_TRUE(goal->behavior_tree.empty());
}

TEST_F(GotoNavigateToPoseTest, FailsWithoutGoalPortOrGotoTarget)
{
  loadGotoNode();

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(nav->goalCount(), 0u);
}

TEST_F(GotoNavigateToPoseTest, SucceedsWhenNavigatorSucceeds)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();

  nav->succeed(0);

  EXPECT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::SUCCESS;
      },
      5.0));
}

TEST_F(GotoNavigateToPoseTest, FailsWhenNavigatorAborts)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();

  nav->abort(0);

  EXPECT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::FAILURE;
      },
      5.0));
}

TEST_F(GotoNavigateToPoseTest, HaltAfterAcceptanceCancelsTheGoal)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();

  EXPECT_NO_THROW(tree->haltTree());

  EXPECT_TRUE(waitFor(
      [&]()
      {
        return nav->isCanceling(0);
      },
      5.0));
}

// The PR #972 case: the goal is sent, the node is halted (STOP, low battery…)
// before the server has answered, and the server accepts it a moment later.
// The goal must be cancelled on acceptance instead of staying active with no owner.
TEST_F(GotoNavigateToPoseTest, HaltBeforeServerAnswersCancelsTheGoalOnAcceptance)
{
  nav->accept_delay = std::chrono::milliseconds(500);
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();

  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);  // goal sent, not answered yet
  EXPECT_NO_THROW(tree->haltTree());
  EXPECT_EQ(nav->goalCount(), 0u) << "test precondition: server must not have answered yet";

  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->goalCount() == 1;
      },
      5.0));
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return nav->isCanceling(0);
      },
      5.0))
      << "orphaned GOTO goal left running on the navigator";
}

// After a halt the node is reusable: the next start sends a fresh goal.
TEST_F(GotoNavigateToPoseTest, RestartsWithAFreshGoalAfterHalt)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();
  tree->haltTree();

  setGotoTarget(2.0, 3.0, 0.0);
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->goalCount() == 2;
      },
      5.0));
  EXPECT_DOUBLE_EQ(nav->goal(1)->pose.pose.position.x, 2.0);
  EXPECT_FALSE(nav->isCanceling(1));
}

// timeout_sec bounds one run: a goal bt_navigator keeps working on (here: never
// finished) is cancelled and the node fails, so the GOTO ends on GOTO_FAILED.
TEST_F(GotoNavigateToPoseTest, TimeoutCancelsTheGoalAndFails)
{
  setGotoTarget(1.0, 1.0, 0.0);
  tree = std::make_unique<BT::Tree>(
      factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
                                 "<NavigateToPose timeout_sec=\"0.6\"/></BehaviorTree></root>",
                                 blackboard));
  const auto start = std::chrono::steady_clock::now();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();

  EXPECT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::FAILURE;
      },
      5.0));
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GE(elapsed, 0.6);
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return nav->isCanceling(0);
      },
      5.0))
      << "timed-out GOTO goal left running on the navigator";
}

// The deadline also covers a server that has not answered yet: the goal is
// cancelled as soon as it is accepted (same GoalAbandonment path as a halt).
TEST_F(GotoNavigateToPoseTest, TimeoutBeforeServerAnswersCancelsOnAcceptance)
{
  nav->accept_delay = std::chrono::milliseconds(1200);
  setGotoTarget(1.0, 1.0, 0.0);
  tree = std::make_unique<BT::Tree>(
      factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
                                 "<NavigateToPose timeout_sec=\"0.3\"/></BehaviorTree></root>",
                                 blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  EXPECT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::FAILURE;
      },
      5.0));
  EXPECT_EQ(nav->goalCount(), 0u) << "test precondition: server must not have answered yet";
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->goalCount() == 1;
      },
      5.0));
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return nav->isCanceling(0);
      },
      5.0));
}

TEST_F(GotoNavigateToPoseTest, NoTimeoutByDefault)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();
  for (int i = 0; i < 20; ++i)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_FALSE(nav->isCanceling(0));
}

// A negative or non-finite timeout is a configuration error, not "no limit":
// the node fails before sending anything.
TEST_F(GotoNavigateToPoseTest, InvalidTimeoutFailsWithoutSendingAGoal)
{
  setGotoTarget(1.0, 1.0, 0.0);
  for (const char* value : {"-1", "nan", "inf"})
  {
    SCOPED_TRACE(value);
    tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
        std::string(
            "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><NavigateToPose timeout_sec=\"") +
            value + "\"/></BehaviorTree></root>",
        blackboard));
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(nav->goalCount(), 0u);
}

// A goal halted and cancelled (STOP) must not poison the next GOTO: its late
// CANCELED result must not be read by the fresh goal, which is kept and
// succeeds.
TEST_F(GotoNavigateToPoseTest, LateCancelResultOfPreviousGoalDoesNotFailTheNextOne)
{
  setGotoTarget(1.0, 1.0, 0.0);
  loadGotoNode();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tickUntilAccepted();
  tree->haltTree();
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->isCanceling(0);
      },
      5.0));

  setGotoTarget(2.0, 3.0, 0.0);
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);  // new goal sent
  nav->confirmCancel(0);  // old result arrives late
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return nav->goalCount() == 2;
      },
      5.0));
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING) << "tick " << i;
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  nav->succeed(1);
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::SUCCESS;
      },
      5.0));
}
