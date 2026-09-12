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

#include <chrono>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/blade_control_service.hpp"
#include "mowgli_behavior/blade_direction.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "mowgli_behavior/utility_nodes.hpp"
#include "mowgli_interfaces/srv/blade_control.hpp"
#include <gtest/gtest.h>

using namespace mowgli_behavior;
using MowerControl = mowgli_interfaces::srv::MowerControl;
using BladeControl = mowgli_interfaces::srv::BladeControl;

TEST(BladeDirection, DisabledAlwaysRequestsDefaultAcrossSessions)
{
  BladeDirection direction(42);
  for (int session = 0; session < 100; ++session)
  {
    EXPECT_EQ(direction.forCommand(false, false), 0u);
    EXPECT_EQ(direction.forCommand(true, false), 0u);
    direction.endSession();
  }
}

TEST(BladeDirection, OffNeverSelectsAndRepeatedCommandsNeverReroll)
{
  BladeDirection direction(42);
  BladeDirection reference(42);
  for (int session = 0; session < 100; ++session)
  {
    for (int repeat = 0; repeat < 10; ++repeat)
      EXPECT_EQ(direction.forCommand(false, true), 0u);
    const auto chosen = reference.forCommand(true, true);
    EXPECT_EQ(direction.forCommand(true, true), chosen);
    for (int repeat = 0; repeat < 10; ++repeat)
    {
      EXPECT_EQ(direction.forCommand(false, true), chosen);
      EXPECT_EQ(direction.forCommand(true, true), chosen);
    }
    direction.endSession();
    reference.endSession();
  }
}

TEST(BladeDirection, BothDirectionsAreSelectedAcrossSessions)
{
  BladeDirection direction(42);
  bool seen[2] = {false, false};
  for (int session = 0; session < 100; ++session)
  {
    const auto chosen = direction.forCommand(true, true);
    ASSERT_LE(chosen, 1u);
    seen[chosen] = true;
    direction.endSession();
  }
  EXPECT_TRUE(seen[0]);
  EXPECT_TRUE(seen[1]);
}

class BladeDirectionNodes : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<MowerControl>::SharedPtr blade_service;
  rclcpp_action::Server<FollowStrip::Nav2FollowPath>::SharedPtr follow_server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<MowerControl::Request> requests;
  std::unique_ptr<BladeControlService> operator_service;
  rclcpp::Client<BladeControl>::SharedPtr operator_client;

  void operatorCommand(uint8_t enabled, uint8_t direction, bool accepted = true)
  {
    auto req = std::make_shared<BladeControl::Request>();
    req->mow_enabled = enabled;
    req->mow_direction = direction;
    auto future = operator_client->async_send_request(req);
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(future.get()->success, accepted);
  }

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
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("blade_direction_test");
    ctx->blade_auto_reverse = true;
    ctx->blade_direction = BladeDirection(42);
    // Start in a reverse session so a silently-defaulted direction field fails.
    for (int attempt = 0; attempt < 100 && ctx->blade_direction.forCommand(true, true) != 1u;
         ++attempt)
      ctx->blade_direction.endSession();
    ASSERT_EQ(ctx->blade_direction.forCommand(false, true), 1u);
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
    factory.registerNodeType<FollowStrip>("FollowStrip");
    factory.registerNodeType<EndSession>("EndSession");
    factory.registerNodeType<ClearCommand>("ClearCommand");
    server = rclcpp::Node::make_shared("fake_blade_hardware");
    blade_service = server->create_service<MowerControl>(
        "/hardware_bridge/mower_control",
        [this](const std::shared_ptr<MowerControl::Request> req,
               std::shared_ptr<MowerControl::Response> resp)
        {
          requests.push_back(*req);
          resp->success = true;
        });
    follow_server = rclcpp_action::create_server<FollowStrip::Nav2FollowPath>(
        server,
        "/follow_path",
        [](const auto&, const auto&)
        {
          return rclcpp_action::GoalResponse::REJECT;
        },
        [](const auto&)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](const auto&) {});
    operator_service = std::make_unique<BladeControlService>(*ctx->node, ctx);
    operator_client = server->create_client<BladeControl>("/blade_direction_test/blade_control");
    ASSERT_TRUE(operator_client->wait_for_service(std::chrono::seconds(5)));
    executor.add_node(server);
    executor.add_node(ctx->node);
    auto probe = ctx->node->create_client<MowerControl>("/hardware_bridge/mower_control");
    ASSERT_TRUE(probe->wait_for_service(std::chrono::seconds(5)));
  }

  BT::Tree tree(const std::string& body)
  {
    return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + body +
                                          "</BehaviorTree></root>",
                                      blackboard);
  }

  void expectRequest(uint8_t enabled, uint8_t direction)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (requests.empty() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_EQ(requests.size(), 1u);
    EXPECT_EQ(requests[0].mow_enabled, enabled);
    EXPECT_EQ(requests[0].mow_direction, direction);
    requests.clear();
  }
};

TEST_F(BladeDirectionNodes, ManualCoverageAndStopRequestsShareDirection)
{
  auto manual = tree("<SetMowerEnabled enabled=\"true\"/>");
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(1u, 1u);
  auto stop = tree("<SetMowerEnabled enabled=\"false\"/>");
  EXPECT_EQ(stop.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(0u, 1u);

  ctx->current_strip_path.poses.resize(2);
  ctx->current_strip_path.poses[1].pose.position.x = 1.0;
  auto coverage = tree("<FollowStrip/>");
  EXPECT_EQ(coverage.tickOnce(), BT::NodeStatus::RUNNING);
  expectRequest(1u, 1u);
  coverage.haltTree();
  expectRequest(0u, 1u);
  // Re-entering after a pause/recharge must retain direction.
  EXPECT_EQ(coverage.tickOnce(), BT::NodeStatus::RUNNING);
  expectRequest(1u, 1u);
  coverage.haltTree();
  expectRequest(0u, 1u);
}

TEST_F(BladeDirectionNodes, OnlyEndSessionClearsSelection)
{
  auto clear = tree("<ClearCommand/>");
  EXPECT_EQ(clear.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->blade_direction.forCommand(false, true), 1u);
  auto end = tree("<EndSession/>");
  EXPECT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->blade_direction.forCommand(false, true), 0u);
  // Reset itself never sends a blade command or enables the motor.
  EXPECT_TRUE(requests.empty());
}

TEST_F(BladeDirectionNodes, MenuOverridesRandomChoiceAndOffSurvivesTicksAndCoverage)
{
  auto manual = tree("<SetMowerEnabled enabled=\"true\"/>");
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(1u, 1u);
  operatorCommand(1u, 0u);
  expectRequest(1u, 0u);
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(1u, 0u);
  operatorCommand(0u, 0u);
  expectRequest(0u, 0u);
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(0u, 0u);

  ctx->current_strip_path.poses.resize(2);
  ctx->current_strip_path.poses[1].pose.position.x = 1.0;
  auto coverage = tree("<FollowStrip/>");
  EXPECT_EQ(coverage.tickOnce(), BT::NodeStatus::RUNNING);
  expectRequest(0u, 0u);
  operatorCommand(1u, 1u);
  expectRequest(1u, 1u);
  coverage.haltTree();
  expectRequest(0u, 1u);
  // An explicit ON cannot override the tree's transit/guard OFF.
  operatorCommand(1u, 0u);
  expectRequest(0u, 0u);
  EXPECT_EQ(coverage.tickOnce(), BT::NodeStatus::RUNNING);
  expectRequest(1u, 0u);
  coverage.haltTree();
  expectRequest(0u, 0u);
}

TEST_F(BladeDirectionNodes, MenuCannotStartIdleBladeAndEndSessionClearsOverride)
{
  operatorCommand(1u, 1u);
  expectRequest(0u, 1u);
  operatorCommand(1u, 2u, false);
  EXPECT_TRUE(requests.empty());
  operatorCommand(0u, 0u);
  expectRequest(0u, 1u);
  auto end = tree("<EndSession/>");
  EXPECT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
  ctx->blade_auto_reverse = false;
  auto manual = tree("<SetMowerEnabled enabled=\"true\"/>");
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(1u, 0u);
}

TEST(BladeDirection, ExplicitStartClearsOnlyInhibition)
{
  BladeDirection policy(42);
  policy.forOperatorCommand(true, 1);
  policy.forOperatorCommand(false, 255);
  EXPECT_EQ(policy.forMowerCommand(true, true).enabled, 0u);
  policy.clearOperatorInhibit();
  const auto command = policy.forMowerCommand(true, true);
  EXPECT_EQ(command.enabled, 1u);
  EXPECT_EQ(command.direction, 1u);
}

TEST_F(BladeDirectionNodes, OffIgnoresDirectionAndSurvivesMissingHardware)
{
  auto client = server->create_client<mowgli_interfaces::srv::BladeControl>(
      "/blade_direction_test/blade_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  for (const auto direction : {2u, 255u})
  {
    auto req = std::make_shared<mowgli_interfaces::srv::BladeControl::Request>();
    req->mow_enabled = 0;
    req->mow_direction = direction;
    auto future = client->async_send_request(req);
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto result = future.get();
    EXPECT_TRUE(result->success);
    EXPECT_FALSE(result->message.empty());
    expectRequest(0, 1);
  }
  blade_service.reset();
  auto hardware_client = ctx->bladeClient();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (hardware_client->service_is_ready() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_FALSE(hardware_client->service_is_ready());
  for (const auto enabled : {0u, 1u})
  {
    auto req = std::make_shared<mowgli_interfaces::srv::BladeControl::Request>();
    req->mow_enabled = enabled;
    req->mow_direction = 0;
    auto future = client->async_send_request(req);
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto result = future.get();
    EXPECT_EQ(result->success, enabled == 0);
    EXPECT_FALSE(result->forwarded);
    EXPECT_FALSE(result->message.empty());
  }
  EXPECT_EQ(ctx->blade_direction.forMowerCommand(true, true).enabled, 0u);
}

TEST_F(BladeDirectionNodes, DelayedHardwareReadsReceiveTreeOnBeforeOperatorOff)
{
  // Queue ON without spinning the hardware. The operator callback and the
  // tree must use one client/writer so the later OFF cannot overtake this ON.
  auto manual = tree("<SetMowerEnabled enabled=\"true\"/>");
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  operatorCommand(0, 0);
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (requests.size() < 2 && std::chrono::steady_clock::now() < deadline)
    executor.spin_some();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_EQ(requests[0].mow_enabled, 1u);
  EXPECT_EQ(requests[1].mow_enabled, 0u);
  requests.clear();
  EXPECT_EQ(manual.tickOnce(), BT::NodeStatus::SUCCESS);
  expectRequest(0, 1);
}
