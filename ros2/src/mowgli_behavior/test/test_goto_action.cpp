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
 * @file test_goto_action.cpp
 * @brief The ~/goto action server: when a goal ends, with which result, exactly once.
 *
 * evaluateGotoMission is tested as a pure function. The server tests run a real
 * GotoActionServer and a real action client on one node; the tree is replaced by
 * writes to BTContext followed by afterTick(), as the node does after each tick.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/goto_action.hpp"
#include "mowgli_behavior/goto_request.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::evaluateGotoMission;
using mowgli_behavior::GotoActionServer;
using mowgli_behavior::GotoMissionInput;
using mowgli_behavior::GotoMissionStep;
using mowgli_behavior::GotoRequestDecision;
using mowgli_behavior::kCommandGoto;
using Goto = mowgli_interfaces::action::Goto;
using GoalHandle = rclcpp_action::ClientGoalHandle<Goto>;
using namespace std::chrono_literals;

namespace
{

GotoMissionInput running()
{
  GotoMissionInput in;
  in.command = kCommandGoto;
  in.state_name = "GOTO_ACTIVE";
  in.elapsed_s = 10.0;
  in.deadline_s = 1800.0;
  return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// evaluateGotoMission
// ---------------------------------------------------------------------------

TEST(GotoMissionTest, RunningGotoContinues)
{
  EXPECT_EQ(evaluateGotoMission(running()).step, GotoMissionStep::kContinue);
}

TEST(GotoMissionTest, PausedByAGuardContinues)
{
  auto in = running();
  for (const char* state : {"EMERGENCY", "WAITING_FOR_RTK", "BOUNDARY_RECOVERY", "DIG_OBSTRUCTION"})
  {
    in.state_name = state;
    EXPECT_EQ(evaluateGotoMission(in).step, GotoMissionStep::kContinue) << state;
  }
}

TEST(GotoMissionTest, TreeOutcomesMapToResults)
{
  auto in = running();
  in.command = 0;
  in.outcome = "REACHED";
  EXPECT_EQ(evaluateGotoMission(in).step, GotoMissionStep::kSettle);
  EXPECT_EQ(evaluateGotoMission(in).error_code, Goto::Result::NONE);

  const std::pair<const char*, uint16_t> finished[] = {
      {"FAILED", Goto::Result::FAILED},
      {"LOW_BATTERY_DOCKED", Goto::Result::LOW_BATTERY_DOCKED},
      {"LOW_BATTERY_DOCK_FAILED", Goto::Result::LOW_BATTERY_DOCK_FAILED},
      {"CRITICAL_BATTERY_DOCKED", Goto::Result::CRITICAL_BATTERY},
  };
  for (const auto& [outcome, code] : finished)
  {
    in.outcome = outcome;
    const auto decision = evaluateGotoMission(in);
    EXPECT_EQ(decision.step, GotoMissionStep::kFinish) << outcome;
    EXPECT_EQ(decision.error_code, code) << outcome;
  }
}

// The tree's own outcome is the most precise account: it wins over a command
// written in the same tick and over a pending cancellation.
TEST(GotoMissionTest, TreeOutcomeWinsOverCommandChangeAndCancel)
{
  auto in = running();
  in.outcome = "FAILED";
  in.command = 8;
  in.cancel_requested = true;
  EXPECT_EQ(evaluateGotoMission(in).error_code, Goto::Result::FAILED);
}

TEST(GotoMissionTest, CancelStopsARunningGoto)
{
  auto in = running();
  in.cancel_requested = true;
  auto decision = evaluateGotoMission(in);
  EXPECT_EQ(decision.step, GotoMissionStep::kStopAndFinish);
  EXPECT_EQ(decision.error_code, Goto::Result::CANCELED);

  in.command = 2;  // already ended by HOME: nothing left to stop
  decision = evaluateGotoMission(in);
  EXPECT_EQ(decision.step, GotoMissionStep::kFinish);
  EXPECT_EQ(decision.error_code, Goto::Result::CANCELED);
}

TEST(GotoMissionTest, CommandChangeWithoutOutcomeIsInterrupted)
{
  auto in = running();
  in.command = 2;
  in.state_name = "RETURNING_HOME";
  const auto decision = evaluateGotoMission(in);
  EXPECT_EQ(decision.step, GotoMissionStep::kFinish);
  EXPECT_EQ(decision.error_code, Goto::Result::INTERRUPTED);
  EXPECT_EQ(decision.error_msg, "command=2 state=RETURNING_HOME");
}

// Critical-battery handling keeps command 9 for hours (docking, charging, a
// latched failure): the goal ends at once, without touching the command.
TEST(GotoMissionTest, CriticalBatteryTakeoverEndsTheGoal)
{
  auto in = running();
  for (const char* state : {"CRITICAL_BATTERY_DOCKING", "CRITICAL_BATTERY_NAV_FAILED"})
  {
    in.state_name = state;
    const auto decision = evaluateGotoMission(in);
    EXPECT_EQ(decision.step, GotoMissionStep::kFinish) << state;
    EXPECT_EQ(decision.error_code, Goto::Result::CRITICAL_BATTERY) << state;
    EXPECT_EQ(decision.error_msg, state);
  }
}

TEST(GotoMissionTest, DeadlineStopsTheGoto)
{
  auto in = running();
  in.state_name = "EMERGENCY";
  in.elapsed_s = 1800.0;
  const auto decision = evaluateGotoMission(in);
  EXPECT_EQ(decision.step, GotoMissionStep::kStopAndFinish);
  EXPECT_EQ(decision.error_code, Goto::Result::DEADLINE);

  in.deadline_s = 0.0;  // disabled
  EXPECT_EQ(evaluateGotoMission(in).step, GotoMissionStep::kContinue);
}

// ---------------------------------------------------------------------------
// GotoActionServer with a real action client
// ---------------------------------------------------------------------------

class GotoActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("goto_action_test");
    ctx = std::make_shared<BTContext>();
    ctx->node = node;
    makeServer(1800.0);
    client = rclcpp_action::create_client<Goto>(node, "goto_action_test/goto");
    wheel_pub = node->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);
    executor.add_node(node);
    ASSERT_TRUE(client->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    executor.remove_node(node);
  }

  void makeServer(double deadline_s)
  {
    server.reset();
    server = std::make_unique<GotoActionServer>(
        *node,
        ctx,
        [this](double, double, double)
        {
          ++admissions;
          if (!refusal.empty())
          {
            return GotoRequestDecision{false, refusal};
          }
          std::lock_guard<std::mutex> lock(ctx->context_mutex);
          ctx->goto_outcome.clear();
          ctx->current_command = kCommandGoto;
          return GotoRequestDecision{true, ""};
        },
        deadline_s);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end)
    {
      executor.spin_some(10ms);
    }
  }

  /// One "tick": the test writes the context, then the server observes it.
  void tick(bool goto_running = true)
  {
    server->afterTick(goto_running);
    executor.spin_some(10ms);
  }

  GoalHandle::SharedPtr sendGoal(rclcpp_action::Client<Goto>::SendGoalOptions options = {})
  {
    Goto::Goal goal;
    goal.x = 1.0;
    goal.y = 2.0;
    goal.yaw = 0.5;
    auto future = client->async_send_goal(goal, options);
    if (executor.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return nullptr;
    }
    auto handle = future.get();
    spinFor(50ms);  // let handle_accepted run
    return handle;
  }

  GoalHandle::WrappedResult waitResult(const GoalHandle::SharedPtr& handle)
  {
    auto future = client->async_get_result(handle);
    EXPECT_EQ(executor.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  void publishWheels(double linear, double angular)
  {
    nav_msgs::msg::Odometry msg;
    msg.twist.twist.linear.x = linear;
    msg.twist.twist.angular.z = angular;
    wheel_pub->publish(msg);
  }

  /// Ticks every 50 ms while publishing wheel speeds, until the goal ends.
  void tickUntilDone(double linear, std::chrono::milliseconds max)
  {
    const auto end = std::chrono::steady_clock::now() + max;
    while (std::chrono::steady_clock::now() < end && server->active())
    {
      publishWheels(linear, 0.0);
      spinFor(20ms);
      tick();
      spinFor(30ms);
    }
  }

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<BTContext> ctx;
  std::unique_ptr<GotoActionServer> server;
  rclcpp_action::Client<Goto>::SharedPtr client;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::string refusal;
  int admissions{0};
};

TEST_F(GotoActionServerTest, RefusedGoalIsAbortedWithTheReason)
{
  refusal = "docked";
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, Goto::Result::REFUSED);
  EXPECT_EQ(result.result->error_msg, "docked");
  EXPECT_FALSE(server->active());
}

TEST_F(GotoActionServerTest, ReachedTargetSucceedsOnceTheWheelsStand)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  ASSERT_TRUE(server->active());
  tick();
  EXPECT_TRUE(server->active());

  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->goto_outcome = "REACHED";
    ctx->current_command = 0;
  }
  tick();
  EXPECT_TRUE(server->active()) << "success waits for standstill";
  tickUntilDone(0.0, 2000ms);

  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->error_code, Goto::Result::NONE);
  EXPECT_TRUE(result.result->settled);
  EXPECT_FALSE(result.result->final_pose_valid) << "no map->base_link in this test";
}

TEST_F(GotoActionServerTest, ReachedTargetWithoutStandstillSucceedsUnsettled)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->goto_outcome = "REACHED";
    ctx->current_command = 0;
  }
  tickUntilDone(0.2, 5000ms);

  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_FALSE(result.result->settled);
}

TEST_F(GotoActionServerTest, SecondGoalWhileOneIsActiveIsRefusedBusy)
{
  auto first = sendGoal();
  ASSERT_TRUE(first);
  auto second = sendGoal();
  ASSERT_TRUE(second);
  const auto result = waitResult(second);
  EXPECT_EQ(result.result->error_code, Goto::Result::REFUSED);
  EXPECT_EQ(result.result->error_msg, "busy");
  EXPECT_EQ(admissions, 1) << "a busy refusal must not reach the admission";
  EXPECT_TRUE(server->active());
}

TEST_F(GotoActionServerTest, CancelStopsTheGotoAndEndsCanceled)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  auto cancel = client->async_cancel_goal(handle);
  ASSERT_EQ(executor.spin_until_future_complete(cancel, 5s), rclcpp::FutureReturnCode::SUCCESS);
  tick();

  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->error_code, Goto::Result::CANCELED);
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  EXPECT_EQ(ctx->current_command, 8) << "a canceled GOTO is stopped like COMMAND_STOP";
}

TEST_F(GotoActionServerTest, AnotherCommandEndsTheGoalInterrupted)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->current_command = 2;
    ctx->last_high_level_status.state_name = "RETURNING_HOME";
  }
  tick();
  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, Goto::Result::INTERRUPTED);
  EXPECT_EQ(result.result->error_msg, "command=2 state=RETURNING_HOME");
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  EXPECT_EQ(ctx->current_command, 2) << "the operator's command is left alone";
}

TEST_F(GotoActionServerTest, CriticalBatteryTakeoverEndsTheGoalWithoutStopping)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_high_level_status.state_name = "CRITICAL_BATTERY_DOCKING";
  }
  tick(false);
  const auto result = waitResult(handle);
  EXPECT_EQ(result.result->error_code, Goto::Result::CRITICAL_BATTERY);
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  EXPECT_EQ(ctx->current_command, kCommandGoto) << "the battery logic keeps the robot";
}

TEST_F(GotoActionServerTest, PausedGoalStaysActiveAndReportsPaused)
{
  rclcpp_action::Client<Goto>::SendGoalOptions options;
  std::vector<uint8_t> phases;
  options.feedback_callback =
      [&phases](GoalHandle::SharedPtr, const std::shared_ptr<const Goto::Feedback> feedback)
  {
    phases.push_back(feedback->phase);
  };
  auto handle = sendGoal(options);
  ASSERT_TRUE(handle);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_high_level_status.state_name = "EMERGENCY";
  }
  tick(false);
  spinFor(100ms);
  EXPECT_TRUE(server->active());
  ASSERT_FALSE(phases.empty());
  EXPECT_EQ(phases.back(), Goto::Feedback::PHASE_PAUSED);
}

TEST_F(GotoActionServerTest, DeadlineStopsTheGotoAndEndsTheGoal)
{
  makeServer(0.2);
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  tick(false);
  EXPECT_TRUE(server->active());
  spinFor(300ms);
  tick(false);

  const auto result = waitResult(handle);
  EXPECT_EQ(result.result->error_code, Goto::Result::DEADLINE);
  std::lock_guard<std::mutex> lock(ctx->context_mutex);
  EXPECT_EQ(ctx->current_command, 8);
}

// One result per goal: once ended, later ticks change nothing, and the result
// can still be fetched by the goal ID after the mission ended.
TEST_F(GotoActionServerTest, ResultIsReportedOnceAndFetchableAfterTheEnd)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->goto_outcome = "FAILED";
    ctx->current_command = 0;
  }
  tick();
  EXPECT_FALSE(server->active());
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->current_command = 8;
  }
  tick();
  spinFor(500ms);

  const auto result = waitResult(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->error_code, Goto::Result::FAILED);
}

TEST_F(GotoActionServerTest, ShutdownEndsAnActiveGoal)
{
  auto handle = sendGoal();
  ASSERT_TRUE(handle);
  server->shutdown();
  const auto result = waitResult(handle);
  EXPECT_EQ(result.result->error_code, Goto::Result::SHUTDOWN);
  EXPECT_FALSE(server->active());
}
