// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the actual callbacks and publisher; only the program entry is renamed.
#define main cog_to_imu_program_entry
#include "../src/cog_to_imu_node.cpp"
#undef main

#include <thread>

#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_localization
{
class CogToImuInputTest : public ::testing::Test
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
    node_ = std::make_shared<CogToImuNode>();
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    sink_ = std::make_shared<rclcpp::Node>("cog_source_motion_capture");
    subscription_ = sink_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/cog_heading",
        sensor_qos(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message)
        {
          output_.push_back(*message);
        });
    executor_.add_node(sink_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (node_->pub_->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_GT(node_->pub_->get_subscription_count(), 0u);
  }
  void TearDown() override
  {
    executor_.remove_node(sink_);
    subscription_.reset();
    sink_.reset();
    node_.reset();
  }
  void Clock(double seconds)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(),
                                        std::llround(seconds * 1e9)),
              RCL_RET_OK);
  }
  void Wheel(double source, double receipt, double vx, double omega = 0)
  {
    Clock(receipt);
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(std::llround(source * 1e9), RCL_ROS_TIME);
    message.twist.twist.linear.x = vx;
    message.twist.twist.angular.z = omega;
    node_->on_wheel(message);
  }
  void Gyro(double source, double receipt, double omega)
  {
    Clock(receipt);
    sensor_msgs::msg::Imu message;
    message.header.stamp = rclcpp::Time(std::llround(source * 1e9), RCL_ROS_TIME);
    message.angular_velocity.z = omega;
    node_->on_imu(message);
  }
  void Fix(double source, double receipt, double east)
  {
    Clock(receipt);
    sensor_msgs::msg::NavSatFix message;
    message.header.stamp = rclcpp::Time(std::llround(source * 1e9), RCL_ROS_TIME);
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    message.latitude = 59;
    message.longitude = 18 + east / (std::cos(59 * kDegToRad) * kMetersPerDeg);
    message.position_covariance[0] = message.position_covariance[4] = .003 * .003;
    node_->on_fix(message);
  }
  void Collect(std::size_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    do
    {
      executor_.spin_some();
      if (output_.size() >= count)
        return;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
  }
  void Warm(double vx)
  {
    for (double source : {99.8, 100.0})
    {
      Wheel(source, source, vx);
      Gyro(source, source, 0);
      Fix(source, source, .3 + vx * (source - 100));
    }
  }
  void ExpectStraightHeadings(bool check_reverse_fixture_noise = true)
  {
    Collect(3);
    ASSERT_EQ(output_.size(), 3u);
    for (std::size_t i = 0; i < output_.size(); ++i)
    {
      EXPECT_EQ(stamp_to_nanoseconds(output_[i].header.stamp),
                100600000000LL + static_cast<int64_t>(i) * 400000000LL);
      EXPECT_NEAR(2 * std::atan2(output_[i].orientation.z, output_[i].orientation.w), 0, 1e-12);
      if (check_reverse_fixture_noise)
      {
        EXPECT_NEAR(output_[i].orientation_covariance[8], .01538339696333997, 1e-12);
      }
    }
  }
  void StationaryTimer()
  {
    node_->republish_latched_when_stationary();
  }
  int PublishedReverse() const
  {
    return node_->published_rev_;
  }
  int PublishedForward() const
  {
    return node_->published_fwd_;
  }
  uint64_t MotionRejects() const
  {
    return node_->rejected_motion_source_;
  }
  auto MotionAt(int64_t source) const
  {
    return node_->motion_history_.At(source);
  }
  bool HasAnchor() const
  {
    return node_->have_anchor_;
  }
  int QuietCount() const
  {
    return node_->rotation_quiet_count_;
  }
  bool HasLatch() const
  {
    return node_->latched_yaw_.has_value();
  }
  std::shared_ptr<CogToImuNode> node_;
  std::shared_ptr<rclcpp::Node> sink_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<sensor_msgs::msg::Imu> output_;
};

TEST_F(CogToImuInputTest, DelayedReverseFixesRemainBodyHeadingAfterForwardTransition)
{
  Warm(-.3);
  Wheel(101.5, 101.5, .3);
  Gyro(101.5, 101.5, .7);  // Current rotation must not rewrite the old straight baseline.
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Fix(source, 101.5, .3 - .3 * (source - 100));
  }
  ExpectStraightHeadings();
  EXPECT_EQ(PublishedReverse(), 3);
  EXPECT_EQ(PublishedForward(), 0);
  EXPECT_EQ(MotionRejects(), 0u);
}

TEST_F(CogToImuInputTest, AlignedReverseRetainsExistingHeadingAndNoise)
{
  Warm(-.3);
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Wheel(source, source, -.3);
    Gyro(source, source, 0);
    Fix(source, source, .3 - .3 * (source - 100));
  }
  ExpectStraightHeadings();
}

TEST_F(CogToImuInputTest, DelayedConstantForwardMotionRetainsHeadingAndNoise)
{
  Warm(.3);
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Wheel(source, source, .3);
    Gyro(source, source, 0);
    Fix(source, source, .3 + .3 * (source - 100));
  }
  ExpectStraightHeadings(false);
  const auto aligned = output_;
  TearDown();
  output_.clear();
  SetUp();
  Warm(.3);
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Wheel(source, source, .3);
    Gyro(source, source, 0);
  }
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Fix(source, 101.5, .3 + .3 * (source - 100));
  }
  ExpectStraightHeadings(false);
  ASSERT_EQ(output_.size(), aligned.size());
  for (std::size_t i = 0; i < output_.size(); ++i)
    EXPECT_DOUBLE_EQ(output_[i].orientation_covariance[8], aligned[i].orientation_covariance[8]);
}

TEST_F(CogToImuInputTest, LateStraightHeadingCannotRefreshLatchAfterCompletedPivot)
{
  Warm(.3);
  Wheel(101.5, 101.5, 0);
  for (double source : {101.5, 101.6, 101.7, 101.8, 101.9})
    Gyro(source, source, .7);
  Gyro(101.99, 101.99, 0);
  Wheel(101.99, 101.99, 0);
  for (int i = 1; i <= 7; ++i)
  {
    const double source = 100 + .2 * i;
    Fix(source, 102, .3 + .3 * (source - 100));
  }
  Collect(3);
  ASSERT_EQ(output_.size(), 3u);  // Historical publications remain valid.
  EXPECT_FALSE(HasLatch());
  Clock(102);
  StationaryTimer();
  executor_.spin_some();
  EXPECT_EQ(output_.size(), 3u);  // None is relabeled as current heading.
}

TEST_F(CogToImuInputTest, MovingTimerObservesClockEvenWithoutLatchBeforeRewind)
{
  Warm(.3);
  Wheel(100.6, 100.6, .3);
  Gyro(100.6, 100.6, 0);
  Fix(100.6, 100.6, .48);
  ASSERT_TRUE(HasAnchor());
  ASSERT_FALSE(HasLatch());
  Clock(100.8);
  StationaryTimer();  // Moving and no latch, but this clock advance is observed.
  Wheel(100.7, 100.7, .3);
  Gyro(100.7, 100.7, 0);
  EXPECT_FALSE(HasAnchor());
  EXPECT_EQ(QuietCount(), 0);
  Fix(100.7, 100.7, .51);
  EXPECT_FALSE(HasAnchor());  // Existing quiet-sample debounce starts anew.
  EXPECT_TRUE(output_.empty());
}

TEST_F(CogToImuInputTest, CarriedRotationDoesNotOverlapNextReceiptInterval)
{
  Warm(.3);
  Gyro(100.65, 100.65, .2);
  Gyro(101.4, 101.4, .2);
  for (double source : {100.2, 100.4, 100.6})
    Fix(source, 101.6, .3 + .3 * (source - 100));
  Collect(1);
  ASSERT_EQ(output_.size(), 1u);
  ASSERT_TRUE(HasLatch());
  Gyro(101.8, 101.8, .2);
  Gyro(101.81, 101.81, 0);
  Wheel(101.81, 101.81, 0);
  Clock(101.82);
  StationaryTimer();
  Collect(2);
  EXPECT_TRUE(HasLatch());
  ASSERT_EQ(output_.size(), 2u);
  EXPECT_EQ(stamp_to_nanoseconds(output_.back().header.stamp), 101820000000LL);
}

TEST_F(CogToImuInputTest, MotionReorderingCannotChangeSourceDirectionAndRewindClearsBaseline)
{
  Warm(.3);
  Wheel(100.0, 100.1, -.3);  // Duplicate source with changed payload.
  Wheel(99.9, 100.1, -.3);
  EXPECT_DOUBLE_EQ(MotionAt(100000000000)->vx, .3);
  Gyro(99.9, 100.1, .8);
  EXPECT_DOUBLE_EQ(MotionAt(100000000000)->gyro_z, 0);
  Fix(100.1, 100.1, .33);
  ASSERT_TRUE(HasAnchor());
  Wheel(1.0, 1.0, -.3);
  EXPECT_FALSE(HasAnchor());
  EXPECT_EQ(QuietCount(), 0);
  EXPECT_FALSE(HasLatch());
  EXPECT_DOUBLE_EQ(MotionAt(1000000000)->vx, -.3);
}
}  // namespace mowgli_localization
