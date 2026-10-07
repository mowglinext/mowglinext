// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// #924: arming, direction and blade evidence do not grant a recovery region.
// This observes the real node's /cmd_vel_nav output, before collision_monitor.
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "mowgli_behavior/escape_nodes.hpp"
#include <gtest/gtest.h>

namespace
{
using Command = geometry_msgs::msg::TwistStamped;

class EscapeAuthorization : public ::testing::Test
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
    ctx_ = std::make_shared<mowgli_behavior::BTContext>();
    ctx_->node = rclcpp::Node::make_shared("escape_authorization_test");
    const auto now = std::chrono::steady_clock::now();
    ctx_->start_blocked_escape_cfg.enabled = true;
    ctx_->start_blocked_escape_armed = true;
    ctx_->start_blocked_escape_armed_time = now;
    ctx_->last_status_time = now;
    ctx_->latest_status.mow_enabled = false;
    ctx_->latest_status.mower_esc_status = 0;
    ctx_->last_motion_valid = true;
    ctx_->last_motion_cmd_vx = 0.20;
    ctx_->last_motion_time = now;
    auto blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx_);
    BT::NodeConfig config;
    config.blackboard = blackboard;
    escape_ = std::make_unique<mowgli_behavior::EscapeStartBlocked>("escape", config);
    observer_ = rclcpp::Node::make_shared("escape_command_observer");
    subscription_ = observer_->create_subscription<Command>("/cmd_vel_nav",
                                                            10,
                                                            [this](Command::ConstSharedPtr cmd)
                                                            {
                                                              commands_.push_back(*cmd);
                                                            });
    executor_.add_node(ctx_->node);
    executor_.add_node(observer_);
  }

  void Pump()
  {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::shared_ptr<mowgli_behavior::BTContext> ctx_;
  std::unique_ptr<mowgli_behavior::EscapeStartBlocked> escape_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<Command>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<Command> commands_;
};

TEST_F(EscapeAuthorization, MissingRecoveryGeometryMustNotPublishNonzeroMotion)
{
  // No map/pose/TF or named bounded region is supplied. All EXISTING escape
  // prerequisites are satisfied, isolating the missing authorization check.
  const auto status = escape_->executeTick();
  if (status == BT::NodeStatus::RUNNING)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (subscription_->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
    {
      Pump();
    }
    ASSERT_GT(subscription_->get_publisher_count(), 0u) << "Command observer not connected";
    // Discovery delay must not turn this into a stale-blade stand-down.
    auto running_status = status;
    const auto command_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (commands_.empty() && running_status == BT::NodeStatus::RUNNING &&
           std::chrono::steady_clock::now() < command_deadline)
    {
      ctx_->last_status_time = std::chrono::steady_clock::now();
      running_status = escape_->executeTick();
      Pump();
    }
    if (running_status == BT::NodeStatus::RUNNING)
    {
      EXPECT_FALSE(commands_.empty()) << "No running command reached the observer before halt";
    }
    escape_->haltNode();
    for (int i = 0; i < 10; ++i)
    {
      Pump();
    }
    if (!commands_.empty())
    {
      EXPECT_DOUBLE_EQ(commands_.back().twist.linear.x, 0.0) << "Halt must publish zero";
    }
  }
  for (const auto& cmd : commands_)
  {
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0) << "No authorized recovery envelope";
    EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
  }
}
}  // namespace
