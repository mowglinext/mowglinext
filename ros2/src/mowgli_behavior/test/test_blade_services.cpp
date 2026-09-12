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

#include <future>
#include <thread>

#include <gtest/gtest.h>
#define main blade_behavior_main_for_test
// Include this package's implementation to exercise actual service/timer wiring.
#include "../src/behavior_tree_node.cpp"  // NOLINT(build/include)
#undef main

namespace mowgli_behavior
{
struct BladeServiceTestPeer
{
  static void setup(const std::shared_ptr<BehaviorTreeNode>& node,
                    const std::function<BT::NodeStatus(BT::TreeNode&)>& tick)
  {
    node->context_->node = node;
    node->setupServiceServer();
    node->factory_.registerSimpleAction("TestTick", tick);
    node->tree_ = node->factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><TestTick/></BehaviorTree></root>");
    node->declare_parameter("tick_rate", 10.0);
    node->setupTimer();
    node->tick_timer_->cancel();
  }
  static void startTimer(BehaviorTreeNode& node)
  {
    node.tick_timer_->reset();
  }
  static void stopTimer(BehaviorTreeNode& node)
  {
    node.tick_timer_->cancel();
  }
  static bool sharedGroup(BehaviorTreeNode& node)
  {
    auto group = node.get_node_base_interface()->get_default_callback_group();
    return group->type() == rclcpp::CallbackGroupType::MutuallyExclusive &&
           group->find_timer_ptrs_if(
               [&](const auto& timer)
               {
                 return timer == node.tick_timer_;
               }) &&
           group->find_service_ptrs_if(
               [&](const auto& service)
               {
                 return service == node.high_level_control_srv_;
               }) &&
           group->find_service_ptrs_if(
               [&](const auto& service)
               {
                 return service == node.start_in_area_srv_;
               }) &&
           group->find_service_ptrs_if(
               [](const auto& service)
               {
                 return std::string(service->get_service_name()) ==
                        "/mowgli_behavior_node/blade_control";
               });
  }
};

class BladeServices : public ::testing::Test
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
};

TEST_F(BladeServices, EveryExplicitStartClearsOffWithoutChangingDirection)
{
  auto node = std::make_shared<BehaviorTreeNode>();
  BladeServiceTestPeer::setup(node,
                              [](auto&)
                              {
                                return BT::NodeStatus::SUCCESS;
                              });
  ASSERT_TRUE(BladeServiceTestPeer::sharedGroup(*node));
  auto client_node = rclcpp::Node::make_shared("blade_start_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  using Start = mowgli_interfaces::srv::HighLevelControl;
  auto client = client_node->create_client<Start>("/mowgli_behavior_node/high_level_control");
  ASSERT_TRUE(client->wait_for_service(5s));
  for (auto cmd : {Start::Request::COMMAND_START,
                   Start::Request::COMMAND_S2,
                   Start::Request::COMMAND_MANUAL_MOW})
  {
    node->context()->blade_direction.forOperatorCommand(true, 1);
    node->context()->blade_direction.forOperatorCommand(false, 0);
    auto request = std::make_shared<Start::Request>();
    request->command = cmd;
    auto future = client->async_send_request(request);
    ASSERT_EQ(executor.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
    auto command = node->context()->blade_direction.forMowerCommand(true, true);
    EXPECT_EQ(command.enabled, 1u);
    EXPECT_EQ(command.direction, 1u);
  }
  using Area = mowgli_interfaces::srv::StartInArea;
  auto area = client_node->create_client<Area>("/mowgli_behavior_node/start_in_area");
  ASSERT_TRUE(area->wait_for_service(5s));
  node->context()->blade_direction.forOperatorCommand(false, 0);
  auto request = std::make_shared<Area::Request>();
  request->area = 2;
  auto future = area->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);
  auto command = node->context()->blade_direction.forMowerCommand(true, true);
  EXPECT_EQ(command.enabled, 1u);
  EXPECT_EQ(command.direction, 1u);
  EXPECT_EQ(node->context()->target_area_index, 2);
  node->context()->node.reset();
}

TEST_F(BladeServices, RealTickExcludesOperatorCallbackUnderMultithreadedExecutor)
{
  auto node = std::make_shared<BehaviorTreeNode>();
  std::promise<void> entered, release;
  auto entered_future = entered.get_future();
  auto release_future = release.get_future().share();
  std::atomic<bool> first{true};
  BladeServiceTestPeer::setup(node,
                              [&](auto&)
                              {
                                if (first.exchange(false))
                                {
                                  entered.set_value();
                                  release_future.wait_for(5s);
                                }
                                return BT::NodeStatus::SUCCESS;
                              });
  ASSERT_TRUE(BladeServiceTestPeer::sharedGroup(*node));
  auto client_node = rclcpp::Node::make_shared("blade_overlap_client");
  auto client = client_node->create_client<mowgli_interfaces::srv::BladeControl>(
      "/mowgli_behavior_node/blade_control");
  ASSERT_TRUE(client->wait_for_service(5s));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.add_node(client_node);
  BladeServiceTestPeer::startTimer(*node);
  std::thread worker(
      [&]
      {
        executor.spin();
      });
  EXPECT_EQ(entered_future.wait_for(3s), std::future_status::ready);
  auto request = std::make_shared<mowgli_interfaces::srv::BladeControl::Request>();
  request->mow_enabled = 0;
  auto response = client->async_send_request(request);
  EXPECT_EQ(response.wait_for(150ms), std::future_status::timeout);
  release.set_value();
  EXPECT_EQ(response.wait_for(3s), std::future_status::ready);
  executor.cancel();
  worker.join();
  BladeServiceTestPeer::stopTimer(*node);
  EXPECT_EQ(node->context()->blade_direction.forMowerCommand(true, true).enabled, 0u);
  node->context()->node.reset();
}
}  // namespace mowgli_behavior
