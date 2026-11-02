// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

// Exercise the actual subscription, wall timer and publisher with an explicitly
// controlled ROS clock. No Webots, application underlay or robot is involved.
#ifndef MOWGLI_SIM_ACTUATION_SOURCE
#define MOWGLI_SIM_ACTUATION_SOURCE "../src/sim_actuation_node.cpp"
#endif
#define main embedded_sim_actuation_main
#include MOWGLI_SIM_ACTUATION_SOURCE
#undef main

#include <iterator>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rcl/time.h>

namespace
{
using Twist = geometry_msgs::msg::TwistStamped;
using namespace std::chrono_literals;
constexpr int64_t kStart = 20000000000LL;
constexpr int64_t kStep = 20000000LL;

class SimActuationClock : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const char* args[] = {"test_sim_actuation_clock",
                          "--ros-args",
                          "-p",
                          "use_sim_time:=true",
                          "-r",
                          "/cmd_vel:=/test_sim_actuation_clock/cmd_vel",
                          "-r",
                          "/cmd_vel_wheels:=/test_sim_actuation_clock/cmd_vel_wheels",
                          "-r",
                          "/clock:=/test_sim_actuation_clock/clock"};
    rclcpp::init(static_cast<int>(std::size(args)), args);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    act_ = std::make_shared<mowgli_simulation::SimActuationNode>();
    peer_ = std::make_shared<rclcpp::Node>("clock_test_peer");
    executor_->add_node(act_);
    executor_->add_node(peer_);
    pub_ = peer_->create_publisher<Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
    sub_ = peer_->create_subscription<Twist>("/cmd_vel_wheels",
                                             rclcpp::SystemDefaultsQoS(),
                                             [this](Twist::ConstSharedPtr msg)
                                             {
                                               rows_.push_back(*msg);
                                             });
    ASSERT_TRUE(act_->get_parameter("use_sim_time").as_bool());
    set_clock(kStart);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ((pub_->get_subscription_count() == 0 || sub_->get_publisher_count() == 0) &&
           std::chrono::steady_clock::now() < deadline)
    {
      pump(1ms);
    }
    ASSERT_EQ(pub_->get_subscription_count(), 1u);
    ASSERT_EQ(sub_->get_publisher_count(), 1u);
    pump(40ms);
    rows_.clear();
  }

  void TearDown() override
  {
    executor_->remove_node(act_);
    executor_->remove_node(peer_);
    sub_.reset();
    pub_.reset();
    peer_.reset();
    act_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void set_clock(int64_t ns)
  {
    ASSERT_EQ(rcl_set_ros_time_override(act_->get_clock()->get_clock_handle(), ns), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(peer_->get_clock()->get_clock_handle(), ns), RCL_RET_OK);
  }

  void pump(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    do
    {
      executor_->spin_some();
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < end);
    executor_->spin_some();
  }

  void command(double vx, double wz = 0.0)
  {
    Twist msg;
    msg.header.stamp = act_->now();
    msg.twist.linear.x = vx;
    msg.twist.angular.z = wz;
    pub_->publish(msg);
    pump(30ms);  // Transport servicing; ROS time remains fixed.
  }

  Twist sample(int64_t ns, std::chrono::milliseconds wall = 30ms)
  {
    rows_.clear();
    set_clock(ns);
    pump(wall);
    EXPECT_FALSE(rows_.empty()) << "no actual-node output at " << ns;
    return rows_.empty() ? Twist{} : rows_.back();
  }

  void expect_zero(const Twist& row)
  {
    EXPECT_DOUBLE_EQ(row.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(row.twist.angular.z, 0.0);
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<mowgli_simulation::SimActuationNode> act_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Twist>::SharedPtr sub_;
  std::vector<Twist> rows_;
};

TEST_F(SimActuationClock, PauseDoesNotAdvanceOrRepublishAndResumeUsesRosTime)
{
  command(0.15);
  const auto first = sample(kStart + kStep);
  EXPECT_GT(first.twist.linear.x, 0.0);
  rows_.clear();
  pump(650ms);  // Longer than the timeout in wall time, with no ROS time elapsed.
  EXPECT_TRUE(rows_.empty()) << "paused ROS time must not generate new wheel setpoints";

  mowgli_simulation::FirmwareWheelState state;
  mowgli_simulation::FirmwareWheelModelParams params;
  double vx, wz;
  mowgli_simulation::step_firmware_wheel_model(0.15, 0.0, 0.02, params, state, vx, wz);
  mowgli_simulation::step_firmware_wheel_model(0.15, 0.0, 0.02, params, state, vx, wz);
  const auto resumed = sample(kStart + 2 * kStep);
  EXPECT_DOUBLE_EQ(resumed.twist.linear.x, vx);
  EXPECT_DOUBLE_EQ(resumed.twist.angular.z, wz);
}

TEST_F(SimActuationClock, SameVirtualTraceIgnoresExtraWallCallbacksAndCoalescing)
{
  auto run = [this](std::chrono::milliseconds wall, bool coalesce)
  {
    sample(kStart - kStep, wall);  // New clock epoch, discard previous command/model.
    sample(kStart, wall);
    command(0.15);
    std::vector<Twist> result;
    for (int block = 1; block <= 4; ++block)
    {
      for (int step = 1; step <= 3; ++step)
      {
        const auto ns = kStart + ((block - 1) * 3 + step) * kStep;
        if (coalesce && step < 3)
        {
          set_clock(ns);  // Same clock trace, executor not serviced at this epoch.
        }
        else
        {
          auto row = sample(ns, wall);
          if (step == 3)
          {
            result.push_back(row);
          }
        }
      }
      command(block % 2 ? 0.0 : 0.15, block % 2 ? 1.0 : 0.0);
    }
    return result;
  };
  const auto regular = run(30ms, false);
  const auto delayed = run(90ms, true);
  ASSERT_EQ(regular.size(), delayed.size());
  for (size_t i = 0; i < regular.size(); ++i)
  {
    EXPECT_EQ(regular[i].header.stamp, delayed[i].header.stamp);
    EXPECT_DOUBLE_EQ(regular[i].twist.linear.x, delayed[i].twist.linear.x) << i;
    EXPECT_DOUBLE_EQ(regular[i].twist.angular.z, delayed[i].twist.angular.z) << i;
  }
}

TEST_F(SimActuationClock, FractionalPeriodsAreRetainedWithoutInventingTicks)
{
  command(0.15);
  rows_.clear();
  set_clock(kStart + 9000000LL);
  pump(30ms);
  EXPECT_TRUE(rows_.empty());
  set_clock(kStart + 19000000LL);
  pump(30ms);
  EXPECT_TRUE(rows_.empty());
  EXPECT_DOUBLE_EQ(sample(kStart + kStep).twist.linear.x, 0.215);
  rows_.clear();
  set_clock(kStart + 39000000LL);
  pump(30ms);
  EXPECT_TRUE(rows_.empty());
  EXPECT_DOUBLE_EQ(sample(kStart + 2 * kStep).twist.linear.x, 0.17183333333333334);
}

TEST_F(SimActuationClock, NormalStraightTurnStopAndStaleCommand)
{
  command(0.2);
  int step = 0;
  for (; step < 20;)
  {
    auto row = sample(kStart + ++step * kStep);
    EXPECT_GE(row.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(row.twist.angular.z, 0.0);
    EXPECT_LE(std::abs(row.twist.linear.x), 255.0 / 300.0);
  }
  command(0.0, 1.0);
  Twist row;
  for (int i = 0; i < 20; ++i)
  {
    row = sample(kStart + ++step * kStep);
    const double left = row.twist.linear.x - row.twist.angular.z * 0.325 / 2.0;
    const double right = row.twist.linear.x + row.twist.angular.z * 0.325 / 2.0;
    EXPECT_LE(std::abs(left), 255.0 / 300.0);
    EXPECT_LE(std::abs(right), 255.0 / 300.0);
  }
  EXPECT_NEAR(row.twist.linear.x, 0.0, 0.01);
  EXPECT_NEAR(row.twist.angular.z, 1.0, 0.05);
  command(0.0);
  for (int i = 0; i < 10; ++i)
  {
    row = sample(kStart + ++step * kStep);
  }
  expect_zero(row);
  command(0.2);
  for (int i = 0; i < 25; ++i)
  {
    row = sample(kStart + ++step * kStep);
  }
  EXPECT_GT(row.twist.linear.x, 0.0) << "command remains eligible at exactly 0.5 ROS seconds";
  for (int i = 0; i < 10; ++i)
  {
    row = sample(kStart + ++step * kStep);
  }
  expect_zero(row);
}

TEST_F(SimActuationClock, ForwardGapExpiresCommandWithoutInventingOneNominalStep)
{
  command(0.15);
  EXPECT_GT(sample(kStart + kStep).twist.linear.x, 0.0);
  expect_zero(sample(kStart + 2000000000LL));
  command(0.15);
  EXPECT_GT(sample(kStart + 2000000000LL + kStep).twist.linear.x, 0.0);
}

TEST_F(SimActuationClock, HighCommandRetainsWheelTargetAndPwmLimits)
{
  command(5.0);
  Twist row;
  for (int step = 1; step <= 25; ++step)
  {
    row = sample(kStart + step * kStep);
    EXPECT_LE(std::abs(row.twist.linear.x), 255.0 / 300.0);
    EXPECT_DOUBLE_EQ(row.twist.angular.z, 0.0);
  }
  EXPECT_NEAR(row.twist.linear.x, 0.5, 0.001);
}

TEST_F(SimActuationClock, RewindClearsModelAndOldCommandUntilNewCommand)
{
  command(0.15);
  EXPECT_GT(sample(kStart + kStep).twist.linear.x, 0.0);
  expect_zero(sample(kStart - 1000000000LL));
  expect_zero(sample(kStart - 1000000000LL + kStep));
  command(0.15);
  const auto restarted = sample(kStart - 1000000000LL + 2 * kStep);
  EXPECT_DOUBLE_EQ(restarted.twist.linear.x, 0.215);
  EXPECT_DOUBLE_EQ(restarted.twist.angular.z, 0.0);
}

TEST_F(SimActuationClock, RewindBetweenWallCallbacksStillInvalidatesOldEpoch)
{
  command(0.15);
  EXPECT_GT(sample(kStart + kStep).twist.linear.x, 0.0);
  set_clock(kStart - 1000000000LL);
  // No node callback runs at the rewind epoch. The next observed timestamp
  // alone would appear to be a normal forward step.
  expect_zero(sample(kStart + 2 * kStep));
  expect_zero(sample(kStart + 3 * kStep));
  command(0.15);
  EXPECT_DOUBLE_EQ(sample(kStart + 4 * kStep).twist.linear.x, 0.215);
}
}  // namespace
