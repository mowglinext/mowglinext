// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <gtest/gtest.h>

#define main sim_actuation_main_for_test
#include "../src/sim_actuation_node.cpp"
#undef main

namespace mowgli_simulation
{
struct SimActuationTestPeer
{
  static void report(SimActuationNode& node, uint8_t mode)
  {
    auto status = std::make_shared<mowgli_interfaces::msg::HighLevelStatus>();
    status->state = mode;
    node.on_high_level_status(status);
  }
  static void input(SimActuationNode& node, double vx)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->twist.linear.x = vx;
    node.on_cmd_vel(msg);
  }
  static void staleStatus(SimActuationNode& node)
  {
    node.bt_status_receipt_ns_ -= 3'000'000'000;
  }
  static void staleCommand(SimActuationNode& node)
  {
    node.last_cmd_stamp_ = node.now() - rclcpp::Duration::from_seconds(1.0);
  }
  static void tick(SimActuationNode& node)
  {
    node.tick();
  }
  static void cancelTimer(SimActuationNode& node)
  {
    node.timer_->cancel();
  }
};

class SimPrecision : public ::testing::Test
{
protected:
  using Peer = SimActuationTestPeer;
  using Status = mowgli_interfaces::msg::HighLevelStatus;
  std::shared_ptr<SimActuationNode> node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
  double output{NAN};
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
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.parameter_overrides({rclcpp::Parameter("deadband_enabled", false)});
    node = std::make_shared<SimActuationNode>(options);
    Peer::cancelTimer(*node);
    sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_wheels",
        rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
        {
          output = msg->twist.linear.x;
        });
    executor.add_node(node);
  }
  void TearDown() override
  {
    executor.remove_node(node);
  }
  double tick()
  {
    output = NAN;
    Peer::tick(*node);
    executor.spin_some();
    EXPECT_TRUE(std::isfinite(output));
    return output;
  }
  double input(double vx)
  {
    Peer::input(*node, vx);
    return tick();
  }
};

TEST_F(SimPrecision, FreshBtAutonomyPassesLowTargetsAndExplicitStops)
{
  Peer::report(*node, Status::HIGH_LEVEL_STATE_AUTONOMOUS);
  for (double v : {-0.03, 0.0, .025 / .8, .030 / .8, .040 / .8, .3})
    EXPECT_DOUBLE_EQ(input(v), v);
}

TEST_F(SimPrecision, NoStatusNullAndManualModesRetainOriginalClamp)
{
  EXPECT_DOUBLE_EQ(input(.03), 0.0);
  for (uint8_t mode : {Status::HIGH_LEVEL_STATE_NULL,
                       Status::HIGH_LEVEL_STATE_IDLE,
                       Status::HIGH_LEVEL_STATE_RECORDING,
                       Status::HIGH_LEVEL_STATE_MANUAL_MOWING})
  {
    Peer::report(*node, mode);
    for (double v : {-0.03, -.0005, 0.0, .0005, .03, .05, .3})
      EXPECT_DOUBLE_EQ(input(v), std::abs(v) < .05 ? 0.0 : v);
  }
}

TEST_F(SimPrecision, ExpiredStatusAndCommandReduceOutputToExactZero)
{
  Peer::report(*node, Status::HIGH_LEVEL_STATE_AUTONOMOUS);
  EXPECT_DOUBLE_EQ(input(.03), .03);
  Peer::staleStatus(*node);
  EXPECT_DOUBLE_EQ(input(.03), 0.0);
  Peer::report(*node, Status::HIGH_LEVEL_STATE_AUTONOMOUS);
  EXPECT_DOUBLE_EQ(input(.03), .03);
  Peer::staleCommand(*node);
  EXPECT_DOUBLE_EQ(tick(), 0.0);
}
}  // namespace mowgli_simulation
