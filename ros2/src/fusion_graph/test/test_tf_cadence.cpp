// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Source-clock cadence and lifecycle of the production dedicated TF writer.

#include <iostream>

#include <tf2_msgs/msg/tf_message.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include <gtest/gtest.h>
#include <rosgraph_msgs/msg/clock.hpp>

namespace fusion_graph
{
struct FusionGraphNodeTestAccess
{
  static void Seed(FusionGraphNode& n)
  {
    auto wheel = std::make_shared<nav_msgs::msg::Odometry>();
    wheel->header.stamp = n.now();
    n.OnWheelOdom(wheel);
    auto imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu->header.stamp = n.now();
    n.OnImu(imu);
    n.graph_->Initialize(gtsam::Pose2(1., 2., .2), 226.7);
    n.OnTimer();
  }
  static void StoppedLoop(FusionGraphNode& n)
  {
    n.tf_thread_stop_.store(true, std::memory_order_release);
    n.tf_broadcast_rate_hz_ = 20.;
    n.TfBroadcastLoop();
  }
  static bool Thread(FusionGraphNode& n)
  {
    return n.tf_thread_.joinable();
  }
  static std::unique_lock<std::mutex> HoldWriter(FusionGraphNode& n)
  {
    return std::unique_lock<std::mutex>(n.tf_state_mu_);
  }
};
}  // namespace fusion_graph
using Steady = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fg = fusion_graph;

class Cadence : public testing::Test
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
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    driver = std::make_shared<rclcpp::Node>("cadence_driver");
    probe = driver->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        tf2_ros::DynamicListenerQoS(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message)
        {
          ++tf_messages;
          last_tf_legs = message->transforms.size();
        });
    probe_executor.add_node(driver);
    clock_pub = driver->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({{"use_sim_time", true},
                              {"autoload_graph", false},
                              {"auto_save_enabled", false},
                              {"use_lidar_map_anchor", false},
                              {"tf_broadcast_rate_hz", Rate()},
                              {"primary_mode", Primary()},
                              {"tf_publish_lead_s", .1}});
    node = std::make_shared<fg::FusionGraphNode>(opts);
    consumer = std::make_shared<rclcpp::Node>("cadence_consumer");
    buffer = std::make_shared<tf2_ros::Buffer>(consumer->get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, consumer, true);
    const auto end = Steady::now() + 3s;
    while (clock_pub->get_subscription_count() == 0 && Steady::now() < end)
      std::this_thread::sleep_for(1ms);
    ASSERT_GT(clock_pub->get_subscription_count(), 0u);
    Clock(226700000000);
    ASSERT_TRUE(WaitClock(226700000000));
    if (Primary() && Rate() > 0.)
    {
      ASSERT_TRUE(WaitForDedicatedTimer());
    }
    fg::FusionGraphNodeTestAccess::Seed(*node);
    // Discovery and graph setup precede the measured window. No timer of the
    // production default callback group is spun by this fixture.
    Clock(226820000000);
    ASSERT_TRUE(WaitClock(226820000000));
    if (Primary() && Rate() > 0.)
    {
      const auto ready = Steady::now() + 2s;
      while (Latest() != 226920000000 && Steady::now() < ready)
        std::this_thread::sleep_for(100us);
      ASSERT_EQ(Latest(), 226920000000);
    }
  }
  virtual double Rate() const
  {
    return 20.;
  }
  virtual bool Primary() const
  {
    return true;
  }
  void TearDown() override
  {
    node.reset();
    probe_executor.remove_node(driver);
    probe.reset();
    listener.reset();
    buffer.reset();
    consumer.reset();
    clock_pub.reset();
    driver.reset();
  }
  void Clock(int64_t ns)
  {
    rosgraph_msgs::msg::Clock message;
    message.clock = rclcpp::Time(ns, RCL_ROS_TIME);
    clock_pub->publish(message);
  }
  bool WaitClock(int64_t ns)
  {
    const auto end = Steady::now() + 1s;
    while (node->now().nanoseconds() != ns && Steady::now() < end)
      std::this_thread::sleep_for(100us);
    return node->now().nanoseconds() == ns;
  }
  bool WaitForDedicatedTimer()
  {
    const auto deadline = Steady::now() + 2s;
    bool ready = false;
    while (!ready && Steady::now() < deadline)
    {
      node->get_node_base_interface()->for_each_callback_group(
          [&ready](rclcpp::CallbackGroup::SharedPtr group)
          {
            if (!group->automatically_add_to_executor_with_node() && group->find_timer_ptrs_if(
                                                                         [](const auto&)
                                                                         {
                                                                           return true;
                                                                         }))
              ready = true;
          });
      if (!ready)
        std::this_thread::sleep_for(1ms);
    }
    return ready;
  }
  bool WaitLatest(int64_t expected)
  {
    const auto deadline = Steady::now() + 2s;
    while (Latest() != expected && Steady::now() < deadline)
      std::this_thread::sleep_for(1ms);
    return Latest() == expected;
  }
  int64_t Latest()
  {
    try
    {
      return rclcpp::Time(
                 buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero).header.stamp)
          .nanoseconds();
    }
    catch (const tf2::TransformException&)
    {
      return -1;
    }
  }
  bool Exact(int64_t support, int64_t query)
  {
    const bool at_support = buffer->canTransform("map",
                                                 "base_footprint",
                                                 rclcpp::Time(support, RCL_ROS_TIME),
                                                 rclcpp::Duration::from_seconds(0.));
    const bool at_fixed_query_time = buffer->canTransform("map",
                                                          "base_footprint",
                                                          rclcpp::Time(query, RCL_ROS_TIME),
                                                          rclcpp::Duration::from_seconds(0.));
    std::cout << "PREDICATE support=" << support << " query=" << query << " latest=" << Latest()
              << " at_support=" << at_support << " at_fixed_query_time=" << at_fixed_query_time
              << std::endl;
    return at_support && at_fixed_query_time;
  }
  void ServiceProbe(std::chrono::milliseconds duration)
  {
    const auto deadline = Steady::now() + duration;
    do
    {
      probe_executor.spin_some();
      std::this_thread::sleep_for(1ms);
    } while (Steady::now() < deadline);
  }
  size_t tf_messages = 0;
  size_t last_tf_legs = 0;
  rclcpp::executors::SingleThreadedExecutor probe_executor;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr probe;
  std::shared_ptr<rclcpp::Node> driver, consumer;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  std::shared_ptr<fg::FusionGraphNode> node;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
};

TEST_F(Cadence, SourceClockPeriodControlsPublication)
{
  EXPECT_EQ(driver->count_publishers("/tf"), 1u);
  ServiceProbe(150ms);
  const auto paused_count = tf_messages;
  ServiceProbe(200ms);
  EXPECT_EQ(tf_messages, paused_count);
  EXPECT_EQ(Latest(), 226920000000);

  // The next timer deadline is 226.850. Ample wall servicing at 226.840
  // must not produce a TF; this catches a wall-sleep reversion without a
  // millisecond-scale CI scheduling requirement.
  Clock(226840000000);
  ASSERT_TRUE(WaitClock(226840000000));
  ServiceProbe(150ms);
  EXPECT_EQ(tf_messages, paused_count);
  EXPECT_EQ(Latest(), 226920000000);

  Clock(226860000000);
  ASSERT_TRUE(WaitClock(226860000000));
  const auto deadline = Steady::now() + 2s;
  while (tf_messages == paused_count && Steady::now() < deadline)
    ServiceProbe(1ms);
  EXPECT_EQ(tf_messages, paused_count + 1);
  EXPECT_EQ(last_tf_legs, 2u);
  const auto delivery = Steady::now() + 2s;
  while (Latest() != 226960000000 && Steady::now() < delivery)
    std::this_thread::sleep_for(1ms);
  EXPECT_EQ(Latest(), 226960000000);
  EXPECT_TRUE(Exact(226920000000, 226940000000));
}

TEST_F(Cadence, PauseRewindAndBoundedLiveContextDestruction)
{
  const auto before = Latest();
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(Latest(), before);
  Clock(100000000000);
  ASSERT_TRUE(WaitClock(100000000000));
  std::this_thread::sleep_for(10ms);
  // Clear old epoch cache just as a ROS-clock TF buffer does on a rewind.
  // The consumer here deliberately uses system time, so clear explicitly.
  buffer->clear();
  Clock(100060000000);
  ASSERT_TRUE(WaitClock(100060000000));
  ServiceProbe(100ms);
  EXPECT_EQ(Latest(), -1);  // Established motion support requires restart after rewind.
  const auto start = Steady::now();
  node.reset();
  const auto elapsed = Steady::now() - start;
  std::cout << "PAUSED_DESTROY_NS=" << elapsed.count() << std::endl;
  EXPECT_LT(elapsed, 150ms);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Cadence, ClockBeforeWriterCounterexample)
{
  auto blocked_writer = fg::FusionGraphNodeTestAccess::HoldWriter(*node);
  Clock(227100000000);
  ASSERT_TRUE(WaitClock(227100000000));
  // Scheduler adversary: a clock callback reaches the consumer before the
  // independent producer writer. This must stay false, even with ROS timers.
  EXPECT_FALSE(Exact(226920000000, 227100000000));
  blocked_writer.unlock();
}

TEST_F(Cadence, SourceSwitchLatchesUntilRestart)
{
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", false)).successful);
  buffer->clear();
  ServiceProbe(100ms);
  EXPECT_EQ(Latest(), -1);
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful);
  Clock(50000000000);
  ASSERT_TRUE(WaitClock(50000000000));
  Clock(50060000000);
  ASSERT_TRUE(WaitClock(50060000000));
  ServiceProbe(100ms);
  EXPECT_EQ(Latest(), -1);
}

TEST_F(Cadence, ContextShutdownThenDestruction)
{
  rclcpp::shutdown();
  const auto start = Steady::now();
  node.reset();
  EXPECT_LT(Steady::now() - start, 150ms);
}

class Observer : public Cadence
{
  bool Primary() const override
  {
    return false;
  }
};
TEST_F(Observer, NoDedicatedWriter)
{
  EXPECT_FALSE(fg::FusionGraphNodeTestAccess::Thread(*node));
  EXPECT_EQ(Latest(), -1);
}
class Inline : public Cadence
{
  double Rate() const override
  {
    return 0.;
  }
};
TEST_F(Inline, NoDedicatedWriter)
{
  EXPECT_FALSE(fg::FusionGraphNodeTestAccess::Thread(*node));
  EXPECT_EQ(Latest(), 226800000000);
}

TEST_F(Inline, ShutdownBeforeDedicatedLoopInitialization)
{
  rclcpp::shutdown();
  EXPECT_NO_THROW(fg::FusionGraphNodeTestAccess::StoppedLoop(*node));
}
