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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/cancel_goal.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "nav2_msgs/action/dock_robot.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::abandonGoal;
using mowgli_behavior::BTContext;
using mowgli_behavior::cancelGoalQuietly;
using mowgli_behavior::cancelWhenAbandoned;
using mowgli_behavior::DockRobot;
using mowgli_behavior::GoalAbandonment;
using mowgli_behavior::IsCommand;
using mowgli_behavior::IsDocking;
using mowgli_behavior::UndockRobot;
using Dock = nav2_msgs::action::DockRobot;
using Undock = nav2_msgs::action::UndockRobot;

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
template <typename Action>
class FakeActionServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Action>;

  FakeActionServer(const rclcpp::Node::SharedPtr& node, const std::string& name)
  {
    server_ = rclcpp_action::create_server<Action>(
        node,
        name,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>)
        {
          std::unique_lock<std::mutex> lock(mutex_);
          acceptance_requested_ = true;
          acceptance_cv_.wait(lock,
                              [this]()
                              {
                                return !hold_acceptance_;
                              });
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

  void holdAcceptance(bool hold)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_acceptance_ = hold;
    acceptance_cv_.notify_all();
  }

  bool acceptanceRequested()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return acceptance_requested_;
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
    handles_.at(i)->succeed(std::make_shared<typename Action::Result>());
  }

  void canceled(std::size_t i, uint16_t code = 0, const std::string& message = {})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::make_shared<typename Action::Result>();
    result->error_code = code;
    result->error_msg = message;
    handles_.at(i)->canceled(result);
  }

  void abort(std::size_t i, uint16_t code, const std::string& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::make_shared<typename Action::Result>();
    result->error_code = code;
    result->error_msg = message;
    handles_.at(i)->abort(result);
  }

  std::string goalId(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return rclcpp_action::to_string(handles_.at(i)->get_goal_id());
  }

private:
  typename rclcpp_action::Server<Action>::SharedPtr server_;
  std::mutex mutex_;
  std::condition_variable acceptance_cv_;
  bool hold_acceptance_ = false;
  bool acceptance_requested_ = false;
  std::vector<std::shared_ptr<Handle>> handles_;
};

using FakeDockServer = FakeActionServer<Dock>;
using FakeUndockServer = FakeActionServer<Undock>;

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
    dock = std::make_unique<FakeDockServer>(server_node, "/dock_robot");
    undock = std::make_unique<FakeUndockServer>(server_node, "/undock_robot");
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
    factory.registerNodeType<UndockRobot>("UndockRobot");
  }

  void TearDown() override
  {
    // Release a blocked server callback even when an assertion failed.
    dock->holdAcceptance(false);
    undock->holdAcceptance(false);
    tree.reset();
    executor->cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    dock.reset();
  }

  void captureDockResults()
  {
    result_logs = server_node->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        rclcpp::RosoutQoS(),
        [this](const rcl_interfaces::msg::Log& log)
        {
          if (log.name == ctx->node->get_logger().get_name() &&
              log.msg.starts_with("DockRobot: result "))
          {
            std::lock_guard<std::mutex> lock(log_mutex);
            logs.push_back(log);
          }
        });
    ASSERT_TRUE(waitFor(
        [this]()
        {
          return result_logs->get_publisher_count() > 0;
        },
        5.0));
  }

  bool hasDockResult(const std::string& goal_id,
                     const std::string& status,
                     uint16_t code,
                     const std::string& message,
                     uint8_t level)
  {
    std::lock_guard<std::mutex> lock(log_mutex);
    return std::any_of(logs.begin(),
                       logs.end(),
                       [&](const auto& log)
                       {
                         return log.level == level &&
                                log.msg.find("status=" + status + " ") != std::string::npos &&
                                log.msg.find("goal_id=" + goal_id + " ") != std::string::npos &&
                                log.msg.find("error_code=" + std::to_string(code) + " ") !=
                                    std::string::npos &&
                                log.msg.find("error_msg='" + message + "'") != std::string::npos &&
                                log.msg.find("dock_id='home_dock'") != std::string::npos &&
                                log.msg.find("dock_type='simple_charging_dock'") !=
                                    std::string::npos;
                       });
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
  std::unique_ptr<FakeUndockServer> undock;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
  std::mutex log_mutex;
  std::vector<rcl_interfaces::msg::Log> logs;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr result_logs;
};

TEST_F(CancelOnHaltTest, DockAbortReportsServerFailureAndGoalIdentity)
{
  captureDockResults();
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>",
      blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  const auto goal_id = dock->goalId(0);
  const std::string message = "Navigation request to staging pose failed: 100% blocked";
  dock->abort(0, 903, message);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return hasDockResult(goal_id, "ABORTED", 903, message, rcl_interfaces::msg::Log::WARN);
      },
      5.0));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(ctx->docking_active);
  EXPECT_FALSE(ctx->last_dock_succeeded);
}

TEST_F(CancelOnHaltTest, DockImmediateSuccessIsReportedBeforeTheNextTick)
{
  captureDockResults();
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>",
      blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  const auto goal_id = dock->goalId(0);
  dock->succeed(0);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return hasDockResult(goal_id, "SUCCEEDED", 0, "", rcl_interfaces::msg::Log::INFO);
      },
      5.0));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->last_dock_succeeded);
}

TEST_F(CancelOnHaltTest, LateDockResultKeepsItsOwnIdentityAfterRestart)
{
  captureDockResults();
  const std::string xml =
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>";
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(xml, blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  const auto old_id = dock->goalId(0);
  tree->haltTree();
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->isCanceling(0);
      },
      5.0));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 2;
      },
      5.0));
  const auto new_id = dock->goalId(1);
  ASSERT_NE(old_id, new_id);
  dock->canceled(0, 905, "old attempt canceled");
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return hasDockResult(
            old_id, "CANCELED", 905, "old attempt canceled", rcl_interfaces::msg::Log::INFO);
      },
      5.0));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->docking_active);
  dock->abort(1, 907, "new attempt failed");
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return hasDockResult(
            new_id, "ABORTED", 907, "new attempt failed", rcl_interfaces::msg::Log::WARN);
      },
      5.0));
  EXPECT_FALSE(hasDockResult(
      new_id, "CANCELED", 905, "old attempt canceled", rcl_interfaces::msg::Log::INFO));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
}

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

// A goal that was SENT but not yet ANSWERED has no handle, so a plain cancel has nothing to
// cancel; the server then accepts it for a sender that has already moved on. Field 2026-10-09:
// a FollowStrip transit orphaned this way kept bt_navigator busy for seven minutes.
TEST_F(CancelOnHaltTest, AGoalAbandonedBeforeTheServerAnswersIsCanceledWhenAccepted)
{
  // Arrange
  auto client = rclcpp_action::create_client<Dock>(ctx->node, "/dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  auto abandonment = std::make_shared<GoalAbandonment>();
  rclcpp_action::Client<Dock>::SendGoalOptions opts;
  opts.goal_response_callback = cancelWhenAbandoned<Dock>(client, abandonment);

  // Act: send the goal and give it up in the same breath.
  auto future = client->async_send_goal(Dock::Goal{}, opts);
  abandonGoal<Dock>(client, nullptr, future, abandonment, ctx->node->get_logger(), "test");

  // Assert: the server accepts it a moment later and it is cancelled at once.
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  EXPECT_TRUE(waitFor(
      [&]()
      {
        return dock->isCanceling(0);
      },
      5.0));
}

TEST_F(CancelOnHaltTest, AGoalThatIsStillWantedIsNotCanceledWhenAccepted)
{
  // Arrange
  auto client = rclcpp_action::create_client<Dock>(ctx->node, "/dock_robot");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  auto abandonment = std::make_shared<GoalAbandonment>();
  rclcpp_action::Client<Dock>::SendGoalOptions opts;
  opts.goal_response_callback = cancelWhenAbandoned<Dock>(client, abandonment);

  // Act
  auto future = client->async_send_goal(Dock::Goal{}, opts);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(future.get());
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Assert
  EXPECT_FALSE(dock->isCanceling(0));
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

TEST_F(CancelOnHaltTest, DockRobotHaltedBeforeAcceptanceCancelsTheLateGoal)
{
  dock->holdAcceptance(true);
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>",
      blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->acceptanceRequested();
      },
      5.0));
  tree->haltTree();
  EXPECT_FALSE(ctx->docking_active);

  dock->holdAcceptance(false);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->isCanceling(0);
      },
      5.0));
  dock->canceled(0);
}

TEST_F(CancelOnHaltTest, DockRobotStillWantedAfterDelayedAcceptanceIsNotCanceled)
{
  dock->holdAcceptance(true);
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><DockRobot/></BehaviorTree></root>",
      blackboard));
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->acceptanceRequested();
      },
      5.0));
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->docking_active);

  dock->holdAcceptance(false);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));
  EXPECT_FALSE(dock->isCanceling(0));
  dock->succeed(0);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::SUCCESS;
      },
      5.0));
  EXPECT_TRUE(ctx->last_dock_succeeded);
  EXPECT_FALSE(ctx->docking_active);
}

// BoundaryGuard's GOTO dock-transit exemption, as shipped in main_tree.xml,
// follows the real DockRobot lifecycle: it holds only while a dock goal runs
// under COMMAND_GOTO (9), and never again once that DockRobot is halted
// (docking_active is cleared in onHalted). Outside the area, the stand-in
// boundary handler then engages and halts the dock, as the guard would.
TEST_F(CancelOnHaltTest, GotoDockTransitExemptionFollowsDockRobotLifecycle)
{
  std::ifstream input(MOWGLI_MAIN_TREE_PATH);
  std::ostringstream contents;
  contents << input.rdbuf();
  const std::string xml = std::regex_replace(contents.str(), std::regex("<!--[\\s\\S]*?-->"), "");
  const auto begin = xml.find("<Sequence name=\"GotoDockTransitExempt\">");
  ASSERT_NE(begin, std::string::npos);
  const std::string close = "</Sequence>";
  const std::string exempt = xml.substr(begin, xml.find(close, begin) + close.size() - begin);

  bool inside_area = true;
  int handler_ticks = 0;
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<IsDocking>("IsDocking");
  factory.registerSimpleCondition("InsideArea",
                                  [&](BT::TreeNode&)
                                  {
                                    return inside_area ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
                                  });
  factory.registerSimpleAction("BoundaryHandler",
                               [&](BT::TreeNode&)
                               {
                                 ++handler_ticks;
                                 return BT::NodeStatus::FAILURE;
                               });
  tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><ReactiveSequence>"
      "<ReactiveFallback>" +
          exempt +
          "<InsideArea/><BoundaryHandler/></ReactiveFallback>"
          "<DockRobot/></ReactiveSequence></BehaviorTree></root>",
      blackboard));

  ctx->current_command = 9;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->docking_active);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->goalCount() == 1;
      },
      5.0));

  // The dock approach leaves the area: exempt while the dock goal runs.
  inside_area = false;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(handler_ticks, 0);
  EXPECT_FALSE(dock->isCanceling(0));

  // Another command: the exemption fails, the handler engages, DockRobot is
  // halted and its goal canceled.
  ctx->current_command = 0;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(handler_ticks, 1);
  EXPECT_FALSE(ctx->docking_active);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return dock->isCanceling(0);
      },
      5.0));
  dock->canceled(0);

  // Command 9 again, no dock running: no exemption is left behind.
  ctx->current_command = 9;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(handler_ticks, 2);
  EXPECT_FALSE(ctx->docking_active);
  EXPECT_EQ(dock->goalCount(), 1u) << "the halted dock must not restart behind the guard";
}

template <typename Action>
class DockingAttemptTest : public CancelOnHaltTest
{
protected:
  FakeActionServer<Action>& server()
  {
    if constexpr (std::is_same_v<Action, Dock>)
    {
      return *dock;
    }
    else
    {
      return *undock;
    }
  }

  void createTree()
  {
    const std::string node = std::is_same_v<Action, Dock> ? "DockRobot" : "UndockRobot";
    tree = std::make_unique<BT::Tree>(
        factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><" + node +
                                       "/></BehaviorTree></root>",
                                   blackboard));
  }
};

using DockingActions = ::testing::Types<Dock, Undock>;
TYPED_TEST_SUITE(DockingAttemptTest, DockingActions);

TYPED_TEST(DockingAttemptTest, DelayedCancellationOfHaltedAttemptDoesNotFailRestartedAttempt)
{
  this->createTree();
  auto& server = this->server();
  auto& tree = this->tree;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return server.goalCount() == 1;
      },
      5.0));
  // Make the BT take the accepted handle before halting (also for UndockRobot,
  // whose existing halt path cancels only a handle already taken by the BT).
  for (int i = 0; i < 5; ++i)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  tree->haltTree();
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return server.isCanceling(0);
      },
      5.0));

  // Restart the SAME BT node while the old server goal is still CANCELING.
  // This orders the old result strictly after onStart resets its outcome.
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return server.goalCount() == 2;
      },
      5.0));
  server.canceled(0);
  // Allow delivery of the late result while repeatedly ticking the new goal.
  // It must stay active until its OWN result arrives, then complete normally.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline)
  {
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_FALSE(server.isCanceling(1));
  if constexpr (std::is_same_v<TypeParam, Dock>)
  {
    EXPECT_TRUE(this->ctx->docking_active);
    EXPECT_FALSE(this->ctx->last_dock_succeeded);
  }
  server.succeed(1);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::SUCCESS;
      },
      5.0));
  if constexpr (std::is_same_v<TypeParam, Dock>)
  {
    EXPECT_FALSE(this->ctx->docking_active);
    EXPECT_TRUE(this->ctx->last_dock_succeeded);
  }
}

TYPED_TEST(DockingAttemptTest, CurrentAttemptCancellationStillFails)
{
  this->createTree();
  auto& server = this->server();
  auto& tree = this->tree;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return server.goalCount() == 1;
      },
      5.0));
  // Cancel as a remote client would, without halting the BT node.
  const std::string name = std::is_same_v<TypeParam, Dock> ? "/dock_robot" : "/undock_robot";
  auto client = rclcpp_action::create_client<TypeParam>(this->ctx->node, name);
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  client->async_cancel_all_goals();
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return server.isCanceling(0);
      },
      5.0));
  server.canceled(0);
  ASSERT_TRUE(waitFor(
      [&]()
      {
        return tree->tickOnce() == BT::NodeStatus::FAILURE;
      },
      5.0));
  if constexpr (std::is_same_v<TypeParam, Dock>)
  {
    EXPECT_FALSE(this->ctx->docking_active);
    EXPECT_FALSE(this->ctx->last_dock_succeeded);
  }
}
