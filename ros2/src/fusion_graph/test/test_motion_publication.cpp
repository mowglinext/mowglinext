// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <iostream>

#include <tf2_msgs/msg/tf_message.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace fusion_graph
{
struct FusionGraphNodeTestAccess
{
  static void Clock(FusionGraphNode& n, int64_t t)
  {
    ASSERT_EQ(rcl_set_ros_time_override(n.get_clock()->get_clock_handle(), t), RCL_RET_OK);
  }
  static void Seed(FusionGraphNode& n, double t)
  {
    n.graph_->Initialize(gtsam::Pose2(), t);
  }
  static void Wheel(FusionGraphNode& n, int64_t t, double vx = -.1488)
  {
    auto m = std::make_shared<nav_msgs::msg::Odometry>();
    m->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
    m->twist.twist.linear.x = vx;
    n.OnWheelOdom(m);
  }
  static void Imu(FusionGraphNode& n, int64_t t)
  {
    auto m = std::make_shared<sensor_msgs::msg::Imu>();
    m->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
    n.OnImu(m);
  }
  static void Gnss(FusionGraphNode& n, int64_t t)
  {
    auto m = std::make_shared<sensor_msgs::msg::NavSatFix>();
    m->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
    m->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    m->latitude = 48.1;
    m->longitude = 11.5;
    m->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    m->position_covariance[0] = m->position_covariance[4] = .000009;
    n.OnGnss(m);
  }
  static void Timer(FusionGraphNode& n)
  {
    n.OnTimer();
  }
  static void Fast(FusionGraphNode& n)
  {
    if (auto data = n.fast_pose_timer_->call())
      n.fast_pose_timer_->execute_callback(data);
  }
  static double X(FusionGraphNode& n)
  {
    std::lock_guard<std::mutex> l(n.tf_state_mu_);
    return n.dr_x_;
  }
  static uint64_t Applied(FusionGraphNode& n)
  {
    return n.graph_->LatestSnapshot()->applied_gnss_token;
  }
};
}  // namespace fusion_graph
using namespace std::chrono_literals;
namespace fg = fusion_graph;
constexpr int64_t ns(double t)
{
  return static_cast<int64_t>(t * 1000000000. + .5);
}
class MotionPublication : public testing::TestWithParam<bool>
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
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"autoload_graph", false},
                           {"auto_save_enabled", false},
                           {"use_lidar_map_anchor", false},
                           {"tf_broadcast_rate_hz", GetParam() ? 20. : 0.},
                           {"tf_publish_lead_s", .1},
                           {"node_period_s", .04},
                           {"datum_lat", 48.1},
                           {"datum_lon", 11.5},
                           {"lever_arm_x", 0.},
                           {"lever_arm_y", 0.},
                           {"gyro_bias_estimation_enabled", false},
                           {"fast_pose_publish_rate_hz", 100.}});
    options = o;
    node = std::make_shared<fg::FusionGraphNode>(o);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    Clock(14.90);
    listener = std::make_shared<rclcpp::Node>("motion_publication_listener");
    odom = listener->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered_map",
        100,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
        {
          ++map_count;
          map_stamp = rclcpp::Time(m->header.stamp).nanoseconds();
        });
    local = listener->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered",
        100,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr)
        {
          ++local_count;
        });
    fast = listener->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered_map_fast",
        rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr)
        {
          ++fast_count;
        });
    yaw = listener->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/fg_yaw",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr)
        {
          ++yaw_count;
        });
    tf = listener->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        tf2_ros::DynamicListenerQoS(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m)
        {
          for (const auto& t : m->transforms)
            if (t.child_frame_id == "base_footprint")
            {
              ++tf_count;
              tf_stamp = rclcpp::Time(t.header.stamp).nanoseconds();
            }
        });
    update = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
        "/fusion_graph/gnss_update",
        rclcpp::QoS(100).reliable().transient_local(),
        [this](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr m)
        {
          facts.push_back(*m);
          if (m->pose_published)
            ++positive_count;
        });
    executor.add_node(listener);
    const auto end = std::chrono::steady_clock::now() + 3s;
    while ((odom->get_publisher_count() == 0 || tf->get_publisher_count() == 0) &&
           std::chrono::steady_clock::now() < end)
      Pump(5ms);
    ASSERT_GT(odom->get_publisher_count(), 0u);
    fg::FusionGraphNodeTestAccess::Seed(*node, 14.90);
    Both(14.90);
    Both(14.92);
    Tick();
    Pump(30ms);
    Clock(14.98);
    Both(14.98);
    Gnss();
    Tick();
    Pump(30ms);
    Clock(15.04);
    Gnss();
    Tick();
    Pump(30ms);
    ASSERT_GT(map_count, 0u);
    ASSERT_GT(tf_count, 0u);
    ASSERT_GT(positive_count, 0u);
  }
  void TearDown() override
  {
    node.reset();
    executor.remove_node(listener);
    listener.reset();
  }
  void Pump(std::chrono::milliseconds d = 30ms)
  {
    auto end = std::chrono::steady_clock::now() + d;
    do
    {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < end);
  }
  void Clock(double t)
  {
    now = t;
    fg::FusionGraphNodeTestAccess::Clock(*node, ns(t));
  }
  void Wheel(double t, double vx = -.1488)
  {
    fg::FusionGraphNodeTestAccess::Wheel(*node, ns(t), vx);
  }
  void Imu(double t)
  {
    fg::FusionGraphNodeTestAccess::Imu(*node, ns(t));
  }
  void Both(double t, double vx = -.1488)
  {
    Clock(t);
    Wheel(t, vx);
    Imu(t);
  }
  void Gnss()
  {
    fg::FusionGraphNodeTestAccess::Gnss(*node, ns(now));
  }
  void Tick()
  {
    fg::FusionGraphNodeTestAccess::Timer(*node);
    fg::FusionGraphNodeTestAccess::Fast(*node);
  }
  void NoMoreAuthority(int m, int l, int f, int y, int t, int p)
  {
    Pump();
    EXPECT_EQ(map_count, m);
    EXPECT_EQ(local_count, l);
    EXPECT_EQ(fast_count, f);
    EXPECT_EQ(yaw_count, y);
    EXPECT_EQ(tf_count, t);
    EXPECT_EQ(positive_count, p);
  }
  void FreshInstanceAfterLoss()
  {
    const auto m = map_count, t = tf_count;
    node.reset();
    node = std::make_shared<fg::FusionGraphNode>(options);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    Clock(20.);
    fg::FusionGraphNodeTestAccess::Seed(*node, 20.);
    Both(20., 0.);
    Tick();
    Pump();
    Clock(20.10);
    Tick();
    Pump();
    EXPECT_GT(map_count, m);
    EXPECT_GT(tf_count, t);
  }
  rclcpp::NodeOptions options;
  std::shared_ptr<fg::FusionGraphNode> node;
  std::shared_ptr<rclcpp::Node> listener;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom, local, fast;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr yaw;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssFusionUpdate>::SharedPtr update;
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  double now = 0.;
  int map_count = 0, local_count = 0, fast_count = 0, yaw_count = 0, tf_count = 0,
      positive_count = 0;
  int64_t map_stamp = 0, tf_stamp = 0;
};
TEST_P(MotionPublication, WheelLossDeniesAllCurrentAuthorityAndFirstReturn)
{
  // Preserve the exact incident's final wheel identity and rate.
  [[maybe_unused]] const double x = fg::FusionGraphNodeTestAccess::X(*node);
  for (int i = 1; i <= 70; ++i)
  {
    Clock(15.04 + i * .02);
    Imu(now);
    Gnss();
    Tick();
    Pump(1ms);
  }
  Pump();
#ifdef EXPECT_DEFECT
  EXPECT_GT(map_stamp, ns(15.98));
  EXPECT_GT(tf_stamp, ns(15.98));
  EXPECT_GT(positive_count, 1);
  EXPECT_LT(fg::FusionGraphNodeTestAccess::X(*node), x - .15);
  std::cout << "RED_WHEEL latest_map=" << map_stamp << " latest_tf=" << tf_stamp
            << " positives=" << positive_count
            << " dr_delta=" << fg::FusionGraphNodeTestAccess::X(*node) - x << std::endl;
#else
  EXPECT_LT(map_stamp, ns(15.98));
  EXPECT_LT(tf_stamp, ns(15.98));
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  const int m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
            p = positive_count;
  Both(16.46);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
#endif
}
TEST_P(MotionPublication, ImuLossDeniesAllCurrentAuthorityAndFirstReturn)
{
  for (int i = 1; i <= 70; ++i)
  {
    Clock(15.04 + i * .02);
    Wheel(now);
    Gnss();
    Tick();
    Pump(1ms);
  }
  Pump();
#ifdef EXPECT_DEFECT
  EXPECT_GT(map_stamp, ns(15.98));
  EXPECT_GT(tf_stamp, ns(15.98));
  EXPECT_GT(positive_count, 1);
  std::cout << "RED_IMU latest_map=" << map_stamp << " latest_tf=" << tf_stamp
            << " positives=" << positive_count << std::endl;
#else
  EXPECT_LT(map_stamp, ns(15.98));
  EXPECT_LT(tf_stamp, ns(15.98));
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  const int m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
            p = positive_count;
  Both(16.46);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
#endif
}
#if !defined(EXPECT_DEFECT) && !defined(ONLY_LOSS_TESTS)
TEST_P(MotionPublication, FreshZeroSlowCadenceAndPauseRemainSupported)
{
  const auto m = map_count, t = tf_count;
  Both(15.60, 0.);
  Tick();
  Pump();
  Both(16.40, 0.);
  Tick();
  Pump();
  EXPECT_GT(map_count, m);
  EXPECT_GT(tf_count, t);
  // Wall time does not consume the ROS acquisition support interval.
  const auto before = map_count;
  Pump(100ms);
  Tick();
  Pump();
  EXPECT_GT(map_count, before);
  const auto p = positive_count;
  Clock(16.80);
  Gnss();
  Tick();
  Pump();
  EXPECT_GT(positive_count, p);
}
TEST_P(MotionPublication, ExactPredictionBoundaryDeniesWithoutPrematureLatch)
{
  Clock(15.879999999);
  Tick();
  Pump();
  const auto m = map_count, l = local_count, y = yaw_count;
  Clock(15.88);
  Tick();
  Pump();
  EXPECT_EQ(map_count, m);
  EXPECT_EQ(local_count, l);
  EXPECT_EQ(yaw_count, y);
  EXPECT_LT(tf_stamp, ns(15.98));
  // Refusing a lead outside the horizon is not yet physical stream loss.
  Both(15.90, 0.);
  Tick();
  Pump();
  EXPECT_GT(map_count, m);
}
TEST_P(MotionPublication, FirstReturningSampleCannotHideAnUnobservedGap)
{
  // No timer/writer receives an intermediate expiry opportunity on inline path.
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Both(16.10);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
}
TEST_P(MotionPublication, DuplicateReorderedAndInvalidWheelCannotRenew)
{
  Clock(15.97);
  Imu(now);
  Wheel(14.98);
  Wheel(14.97);
  Wheel(15.97, std::numeric_limits<double>::quiet_NaN());
  Tick();
  Pump();
  Clock(15.98);
  Imu(now);
  Wheel(14.98);
  Tick();
  Pump();
  EXPECT_LT(map_stamp, ns(15.98));
  EXPECT_LT(tf_stamp, ns(15.98));
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Both(16.);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
}
TEST_P(MotionPublication, FutureAcquisitionWaitsForOwnClock)
{
  // A small real phase lead may wait for source clock catchup, without a grace
  // window or discarding its identity. It cannot authorize publication early.
  Wheel(15.20, 0.);
  Imu(15.20);
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  Clock(15.20);
  Tick();
  Pump();
  EXPECT_GT(map_count, m);
}
TEST_P(MotionPublication, RewindAfterSupportLatchesWithEitherStreamFirst)
{
  Clock(10.);
  if (GetParam())
  {
    Imu(10.);
    Wheel(10.);
  }
  else
  {
    Wheel(10.);
    Imu(10.);
  }
  Tick();
  Pump();
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Both(10.20);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  FreshInstanceAfterLoss();
}
TEST_P(MotionPublication, ClockSourceChangeLatchesWithoutClockLockDeadlock)
{
  ASSERT_EQ(rcl_disable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  Tick();
  Pump();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Both(16.);
  Gnss();
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  FreshInstanceAfterLoss();
}
TEST_P(MotionPublication, DedicatedWriterDetectsLossWithoutMainExecutor)
{
  if (!GetParam())
    GTEST_SKIP();
  Clock(15.50);
  Imu(now);
  Pump();
  const auto t = tf_count;
  // Main timer, fast publisher and input callbacks do not run at expiry.
  Clock(16.10);
  Pump(100ms);
  EXPECT_EQ(tf_count, t);
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  const auto p = positive_count;
  Both(16.20);
  Gnss();
  Tick();
  Pump();
  EXPECT_EQ(positive_count, p);
}
TEST_P(MotionPublication, RestartWaitsForBothRealStationaryBaselines)
{
  Clock(16.10);
  Tick();
  Pump();
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  node.reset();
  Pump();
  node = std::make_shared<fg::FusionGraphNode>(options);
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  Clock(5.);
  fg::FusionGraphNodeTestAccess::Seed(*node, 5.);
  Pump();
  const auto m = map_count, l = local_count, f = fast_count, y = yaw_count, t = tf_count,
             p = positive_count;
  Tick();
  Pump();
  Clock(5.10);
  Wheel(5.10, 0.);
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  // Rewind before first support clears startup identities rather than latching.
  Clock(4.);
  Imu(4.);
  Tick();
  NoMoreAuthority(m, l, f, y, t, p);
  Clock(4.10);
  Wheel(4.10, 0.);
  Imu(4.10);
  Tick();
  Pump();
  Clock(4.20);
  Tick();
  Pump();
  EXPECT_GT(map_count, m);
  EXPECT_GT(local_count, l);
  EXPECT_GT(tf_count, t);
}
#endif
INSTANTIATE_TEST_SUITE_P(ActualNode, MotionPublication, testing::Values(false, true));
