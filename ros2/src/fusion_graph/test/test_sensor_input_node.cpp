// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

#include "fusion_graph/fusion_graph_node.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <unistd.h>

namespace fusion_graph
{
class FusionGraphNodeInputTest : public ::testing::Test
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
  void CreateNode(double period = .1, double lever_x = 0.0, double lever_y = 0.0)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {{"autoload_graph", false},
         {"auto_save_enabled", false},
         {"primary_mode", false},
         {"tf_broadcast_rate_hz", 0.0},
         {"use_lidar_map_anchor", false},
         {"periodic_save_period_s", 0.0},
         {"fast_pose_publish_rate_hz", 0.0},
         {"node_period_s", period},
         {"lever_arm_x", lever_x},
         {"lever_arm_y", lever_y},
         {"graph_save_prefix", "/tmp/fusion_graph_node_input_" + std::to_string(getpid())}});
    node_ = std::make_shared<FusionGraphNode>(options);
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
  }
  void SetUp() override
  {
    CreateNode();
    Clock(10.0);
    Wheel(10.0);
    Imu(10.0);
    Clock(10.02);
    Wheel(10.02);
    Imu(10.02);
  }
  void TearDown() override
  {
    node_.reset();
  }
  void Clock(double seconds)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(),
                                        std::llround(seconds * 1e9)),
              RCL_RET_OK);
  }
  void Wheel(double seconds, double vx = .15, double vy = 0.0, double wz = 0.0)
  {
    auto message = WheelMessage(seconds, vx, vy, wz);
    node_->OnWheelOdom(message);
  }
  static nav_msgs::msg::Odometry::SharedPtr WheelMessage(double seconds,
                                                         double vx = .15,
                                                         double vy = 0.0,
                                                         double wz = 0.0)
  {
    auto message = std::make_shared<nav_msgs::msg::Odometry>();
    message->header.stamp = rclcpp::Time(std::llround(seconds * 1e9), RCL_ROS_TIME);
    message->twist.twist.linear.x = vx;
    message->twist.twist.linear.y = vy;
    message->twist.twist.angular.z = wz;
    return message;
  }
  void Imu(double seconds, double wz = 0.0)
  {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(std::llround(seconds * 1e9), RCL_ROS_TIME);
    message->angular_velocity.z = wz;
    node_->OnImu(message);
  }
  void ExpectPose(double x)
  {
    EXPECT_NEAR(node_->dr_x_, x, 1e-15);
    EXPECT_DOUBLE_EQ(node_->dr_y_, 0.0);
    EXPECT_DOUBLE_EQ(node_->dr_yaw_, 0.0);
  }
  void ExpectAcceptedSources(int64_t wheel_ns, int64_t imu_ns)
  {
    ASSERT_TRUE(node_->last_wheel_stamp_);
    ASSERT_TRUE(node_->last_imu_stamp_);
    EXPECT_EQ(node_->last_wheel_stamp_->nanoseconds(), wheel_ns);
    EXPECT_EQ(node_->last_imu_stamp_->nanoseconds(), imu_ns);
    EXPECT_DOUBLE_EQ(node_->wheel_vx_, .15);
    EXPECT_DOUBLE_EQ(node_->wheel_wz_, 0.0);
  }
  void Next()
  {
    Clock(10.04);
    // Consume the cached wheel rate first: a rejected input must not replace it.
    Imu(10.04);
    Wheel(10.04);
  }
  void InvalidImuStamp(int32_t sec, uint32_t nanosec)
  {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp.sec = sec;
    message->header.stamp.nanosec = nanosec;
    node_->OnImu(message);
  }
  void InvalidWheelStamp(int32_t sec, uint32_t nanosec)
  {
    auto message = WheelMessage(10.03, -.3);
    message->header.stamp.sec = sec;
    message->header.stamp.nanosec = nanosec;
    node_->OnWheelOdom(message);
  }
  void ResetSources()
  {
    node_->graph_->Reset();
    node_->ResetMotionSourceEpoch();
    node_->dr_x_ = node_->dr_y_ = node_->dr_yaw_ = 0.0;
    node_->dr_continuous_from_raw_ = gtsam::Pose2();
  }
  auto SourcePose(double seconds)
  {
    return node_->DrPoseAtSource(node_->integration_epoch_, std::llround(seconds * 1e9));
  }
  auto OutputState()
  {
    return node_->CaptureMotionPublishState();
  }
  auto Snapshot()
  {
    return node_->graph_->LatestSnapshot();
  }
  uint64_t Epoch()
  {
    return node_->integration_epoch_;
  }
  size_t ConsumedSteps()
  {
    return node_->dr_source_history_.size();
  }
  double Lead()
  {
    return node_->tf_publish_lead_s_;
  }
  void SeedLatest(double yaw = 0.0)
  {
    ASSERT_TRUE(node_->integrated_source_ns_);
    ASSERT_TRUE(
        node_->InitializeAtSource(gtsam::Pose2(0.0, 0.0, yaw), *node_->integrated_source_ns_));
  }
  void Timer()
  {
    node_->OnTimer();
  }
  void SupplySeedYaw(double yaw)
  {
    ASSERT_TRUE(node_->last_imu_stamp_);
    Heading(node_->last_imu_stamp_->seconds(), yaw);
  }
  void SetRecenterDistance(double distance)
  {
    node_->odom_rebase_dist_m_ = distance;
  }
  void Gnss(double seconds, double east, double sigma = .01, double north = 0.0)
  {
    node_->datum_lat_ = 57.0;
    node_->datum_lon_ = 12.0;
    node_->datum_cos_lat_ = std::cos(57.0 * M_PI / 180.0);
    auto message = std::make_shared<sensor_msgs::msg::NavSatFix>();
    message->header.stamp = rclcpp::Time(std::llround(seconds * 1e9), RCL_ROS_TIME);
    message->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    message->latitude = 57.0 + north / 6378137.0 * 180.0 / M_PI;
    message->longitude = 12.0 + east / (6378137.0 * node_->datum_cos_lat_) * 180.0 / M_PI;
    message->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    message->position_covariance[0] = message->position_covariance[4] = sigma * sigma;
    node_->OnGnss(message);
  }
  void ExpectRawPositionZero()
  {
    EXPECT_DOUBLE_EQ(node_->dr_x_, 0.0);
    EXPECT_DOUBLE_EQ(node_->dr_y_, 0.0);
  }
  uint64_t LastLidarSourceIndex() const
  {
    return node_->last_lidar_source_node_index_;
  }
  void PrimeExtrapolator()
  {
    node_->pose_extrap_.OnFusionPose(node_->now().seconds(), gtsam::Pose2());
  }
  bool HasExtrapolatedPose() const
  {
    return node_->pose_extrap_.HasFusionPose();
  }
  void SimulateCoreReset()
  {
    node_->graph_->Reset();
  }
  bool FastEligible(double seconds) const
  {
    return node_->FastPoseSourceIsCurrent(seconds);
  }
  double FastGyro() const
  {
    return node_->pose_extrap_.LastGyroWz();
  }
  double IntegratedGyro() const
  {
    return node_->dr_last_gz_;
  }
  void RememberSeededDockSession()
  {
    node_->gps_seen_once_ = true;
    node_->dock_seeded_this_session_ = true;
    node_->dock_seed_fallback_done_ = true;
  }
  void SeedDock()
  {
    node_->SeedFromDockPose();
  }
  void SetDocked()
  {
    node_->last_is_charging_valid_ = true;
    node_->last_is_charging_ = true;
  }
  auto UsableGnssReceipt() const
  {
    return node_->last_usable_gnss_stamp_;
  }
  auto FixedGnssReceipt() const
  {
    return node_->last_rtk_fixed_stamp_;
  }
  void Heading(double source_seconds, double yaw, bool cog = false)
  {
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.stamp = rclcpp::Time(std::llround(source_seconds * 1e9), RCL_ROS_TIME);
    message->orientation.z = std::sin(yaw / 2.0);
    message->orientation.w = std::cos(yaw / 2.0);
    message->orientation_covariance[8] = .01;
    if (cog)
      node_->OnCogHeading(message);
    else
      node_->OnMagYaw(message);
  }
  void SetOffDock()
  {
    node_->last_is_charging_valid_ = true;
    node_->last_is_charging_ = false;
  }
  bool ExplicitClear()
  {
    node_->last_hl_state_valid_ = true;
    node_->last_hl_state_ = mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
    node_->last_hl_state_name_ = "IDLE";
    auto client = node_->create_client<std_srvs::srv::Trigger>("~/clear_graph");
    if (!client->wait_for_service(std::chrono::seconds(1)))
      return false;
    auto pending = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    if (executor.spin_until_future_complete(pending, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return false;
    return pending.get()->success;
  }
  void StartCogScenario(double yaw)
  {
    node_.reset();
    CreateNode(.04);
    Clock(100.0);
    Wheel(100.0, 0.0);
    Imu(100.0);
    EXPECT_TRUE(node_->InitializeAtSource(gtsam::Pose2(0.0, 0.0, yaw), 100000000000));
    SetOffDock();
  }
  uint64_t CogRecoveries() const
  {
    return node_->cog_flip_recoveries_;
  }
  int CogDebounce() const
  {
    return node_->cog_flip_count_;
  }
  size_t PendingCogChecks() const
  {
    return node_->pending_cog_flip_checks_.size();
  }
  uint64_t CogSourceRejects() const
  {
    return node_->cog_flip_source_rejects_;
  }
  void RememberRecoveryCooldown()
  {
    node_->last_flip_recovery_stamp_ = node_->now();
  }
  bool HasRecoveryCooldown() const
  {
    return node_->last_flip_recovery_stamp_.has_value();
  }
  uint64_t AppliedYawFactors() const
  {
    return node_->graph_->SourceYawFactorCount();
  }
  void PrepareFutureCogChecks()
  {
    StartCogScenario(M_PI);
    Clock(100.4);
    Imu(100.4);
    Clock(100.65);
    Wheel(100.65, .15);
    Gnss(100.4, 0.0);
    for (const double source : {100.6, 100.62, 100.64})
      Heading(source, 0.0, true);
  }
  double RunTrueHistoricalCogFlip(bool use_recovery)
  {
    StartCogScenario(M_PI);
    // Build a stationary edge long enough to create an actual interior GNSS
    // acquisition state before the subsequent turn and delayed COG recovery.
    for (int i = 1; i <= 280; ++i)
    {
      const double time = 100.0 + .02 * i;
      Clock(time);
      const double gyro = i > 260 ? .75 : 0.0;
      Wheel(time, 0.0, 0.0, gyro);
      Imu(time, gyro);
      if (i == 10)
        Gnss(100.1, 0.0, .003);
      Timer();
    }
    EXPECT_EQ(node_->graph_->SourceGnssFactorCount(), 1u);
    EXPECT_EQ(node_->graph_->LiveNodeCount(), node_->graph_->GetAllPoses().size() + 1u);
    Clock(105.62);
    Wheel(105.62, .15);
    Gnss(105.6, 0.0, .003);
    node_->cog_flip_recovery_enabled_ = use_recovery;
    for (const double source : {100.1, 100.2, 100.3})
      Heading(source, 0.0, true);
    if (!use_recovery)
    {
      // Independent control: retain the same existing finite-strength recovery
      // prior, targeted with the known physical offset -pi at the latest state.
      const auto snapshot = Snapshot();
      EXPECT_TRUE(snapshot);
      if (!snapshot)
        return NAN;
      const auto target =
          gtsam::Pose2(snapshot->pose.x(), snapshot->pose.y(), snapshot->pose.theta() - M_PI);
      node_->graph_->ForceAnchor(snapshot->node_index, target, .005, M_PI / 180.0);
    }
    else
      EXPECT_EQ(CogRecoveries(), 1u);
    const auto snapshot = Snapshot();
    EXPECT_TRUE(snapshot);
    if (!snapshot)
      return NAN;
    const auto physical = node_->DrPoseAtSource(snapshot->source_epoch, snapshot->source_stamp_ns);
    EXPECT_TRUE(physical);
    if (physical)
    {
      const double error = snapshot->pose.theta() - physical->theta();
      std::cout << std::setprecision(17) << "true_flip_" << (use_recovery ? "actual" : "control")
                << " source_ns=" << snapshot->source_stamp_ns
                << " posterior_yaw=" << snapshot->pose.theta()
                << " physical_yaw=" << physical->theta() << " absolute_wrapped_error="
                << std::abs(std::atan2(std::sin(error), std::cos(error))) << '\n';
    }
    return snapshot->pose.theta();
  }
  struct PhaseResult
  {
    double x;
    double y;
    double yaw;
    uint64_t admitted;
    uint64_t applied;
  };
  PhaseResult RunGnssPhase(double receipt, double epoch)
  {
    node_.reset();
    CreateNode(.02);
    Clock(0.0);
    Wheel(0.0, 0.0);
    Imu(0.0);
    EXPECT_TRUE(node_->InitializeAtSource(gtsam::Pose2(), 0));
    node_->last_is_charging_valid_ = true;
    node_->last_is_charging_ = false;
    uint64_t admitted = 0;
    const int receipt_step = std::llround(receipt / .02);
    for (int i = 1; i <= 110; ++i)
    {
      const double time = i * .02;
      Clock(time);
      Wheel(time, i > 50 ? .15 : 0.0);
      Imu(time);
      if (i == receipt_step && epoch >= 0.0)
      {
        Gnss(epoch, std::max(0.0, epoch - 1.0) * .15, .003);
        if (node_->last_usable_gnss_stamp_ &&
            node_->last_usable_gnss_stamp_->nanoseconds() == std::llround(epoch * 1e9))
          ++admitted;
      }
      Timer();
    }
    const auto state = OutputState();
    EXPECT_TRUE(state);
    EXPECT_TRUE(state && state->anchor_valid);
    if (!state || !state->anchor_valid)
      return {NAN, NAN, NAN, admitted, 0};
    // Remove the configured future lead to compare at the consumed 2.2 s epoch.
    const auto map = state->anchor.compose(node_->ContinuousDrPoseLocked());
    return {map.x(), map.y(), map.theta(), admitted, node_->graph_->SourceGnssFactorCount()};
  }
  std::shared_ptr<FusionGraphNode> node_;
};

TEST_F(FusionGraphNodeInputTest, OrderedIntervalsRetainIntegration)
{
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ReorderedImuDoesNotReopenIntegratedInterval)
{
  Clock(10.03);
  Imu(10.01);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ReorderedWheelDoesNotReplaceCachedRateOrBaseline)
{
  Clock(10.03);
  Wheel(9.99, -.3);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, AlteredDuplicateWheelCannotReplaceCachedRate)
{
  Clock(10.03);
  Wheel(10.02, -.3);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, AlteredDuplicateImuCannotIntegrateRotation)
{
  Clock(10.03);
  Imu(10.02, 1.0);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, NonfiniteGyroCannotPoisonPoseOrAcceptedSource)
{
  Clock(10.03);
  Imu(10.03, std::numeric_limits<double>::quiet_NaN());
  Imu(10.03, std::numeric_limits<double>::infinity());
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, NonfiniteWheelCannotPoisonCachedRateOrAcceptedSource)
{
  Clock(10.03);
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  Wheel(10.03, nan);
  Wheel(10.03, inf);
  Wheel(10.03, .15, nan);
  Wheel(10.03, .15, 0.0, inf);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, InvalidStampDoesNotBecomeAnIntegrationBaseline)
{
  Clock(10.03);
  InvalidImuStamp(10, 1000000000u);
  InvalidWheelStamp(10, 1000000000u);
  InvalidImuStamp(-1, 0);
  InvalidWheelStamp(-1, 0);
  ExpectAcceptedSources(10020000000, 10020000000);
  Next();
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ClockRewindAdmitsNewEpochWithoutIntegratingAcrossIt)
{
  Clock(1.0);
  Wheel(1.0);
  Imu(1.0);
  ExpectPose(.003);
  Clock(1.02);
  Wheel(1.02);
  Imu(1.02);
  ExpectPose(.006);
  ExpectAcceptedSources(1020000000, 1020000000);
}

TEST_F(FusionGraphNodeInputTest, InvalidFirstSampleAfterRewindDoesNotOwnNewBaseline)
{
  Clock(1.0);
  Wheel(1.0, std::numeric_limits<double>::quiet_NaN());
  Imu(1.0, std::numeric_limits<double>::quiet_NaN());
  Clock(1.02);
  Wheel(1.02);
  Imu(1.02);
  ExpectPose(.003);
  Clock(1.04);
  Wheel(1.04);
  Imu(1.04);
  ExpectPose(.006);
}
TEST_F(FusionGraphNodeInputTest, ImuFirstAfterRewindCannotUsePreviousEpochWheelRate)
{
  Clock(1.0);
  Imu(1.0);
  Clock(1.02);
  Imu(1.02);
  ExpectPose(.003);
  Wheel(1.02);
  Clock(1.04);
  Imu(1.04);
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, WheelFirstAfterRewindKeepsNewEpochWheelRate)
{
  Clock(1.0);
  Wheel(1.0);
  Imu(1.0);
  Clock(1.02);
  Imu(1.02);
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, ZeroSourceTimeCanBootstrapSimulationEpoch)
{
  Clock(0.0);
  Wheel(0.0);
  Imu(0.0);
  Clock(.02);
  Wheel(.02);
  Imu(.02);
  ExpectPose(.006);
}

TEST_F(FusionGraphNodeInputTest, PublicationEpochIsConsumedSourceAndNotWallNow)
{
  Clock(10.8);
  const auto output = OutputState();
  ASSERT_TRUE(output);
  EXPECT_EQ(output->source_ns, 10020000000);
  EXPECT_NEAR(output->predicted_pose.x(), .003 + .15 * Lead(), 1e-14);
  // No timer cadence can manufacture another integration boundary.
  Timer();
  ASSERT_TRUE(OutputState());
  EXPECT_EQ(OutputState()->source_ns, 10020000000);
}

TEST_F(FusionGraphNodeInputTest, FirstSampleAndGapAreBaselinesWithoutPublishableMotion)
{
  ResetSources();
  Clock(20.0);
  Wheel(20.0);
  Imu(20.0);
  EXPECT_FALSE(OutputState());
  EXPECT_EQ(ConsumedSteps(), 0u);
  Clock(20.02);
  Imu(20.02);
  ASSERT_TRUE(OutputState());
  const auto epoch = Epoch();
  Clock(21.5);
  Imu(21.5);
  EXPECT_GT(Epoch(), epoch);
  EXPECT_FALSE(OutputState());
  EXPECT_EQ(ConsumedSteps(), 0u);
  // The gap never contributes a displacement; new wheel evidence is needed.
  Clock(21.52);
  Imu(21.52);
  ASSERT_TRUE(OutputState());
  EXPECT_NEAR(OutputState()->predicted_pose.x(), .003, 1e-14);
}

TEST_F(FusionGraphNodeInputTest, DelayedWheelCannotRewriteConsumedTrajectory)
{
  Clock(10.04);
  Imu(10.04);
  const auto original = SourcePose(10.03);
  ASSERT_TRUE(original);
  Wheel(10.03, .4);  // accepted source order, received after the interval was consumed
  ASSERT_TRUE(SourcePose(10.03));
  EXPECT_NEAR(SourcePose(10.03)->x(), original->x(), 1e-15);
  Clock(10.06);
  Imu(10.06);
  ASSERT_TRUE(SourcePose(10.05));
  EXPECT_NEAR(SourcePose(10.05)->x(), .006 + .004, 1e-14);
}

TEST_F(FusionGraphNodeInputTest, InteriorSourcePoseUsesSameExpmapAsIntegratedTurn)
{
  ResetSources();
  Clock(20.0);
  Wheel(20.0, .3, 0.0, .4);
  Imu(20.0, .4);
  Clock(20.2);
  Imu(20.2, .4);
  const auto expected = gtsam::Pose2::Expmap(gtsam::Vector3(.03, 0.0, .04));
  const auto interior = SourcePose(20.1);
  ASSERT_TRUE(interior);
  EXPECT_NEAR(interior->x(), expected.x(), 1e-13);
  EXPECT_NEAR(interior->y(), expected.y(), 1e-13);
  EXPECT_NEAR(interior->theta(), expected.theta(), 1e-13);
  const auto endpoint = SourcePose(20.2);
  ASSERT_TRUE(endpoint);
  EXPECT_NEAR(endpoint->x(), expected.compose(expected).x(), 1e-13);
  EXPECT_NEAR(endpoint->y(), expected.compose(expected).y(), 1e-13);
}

TEST_F(FusionGraphNodeInputTest, HeldControlTrajectoryIsIndependentOfImuPartitionRate)
{
  const auto run = [this](double start, int count)
  {
    ResetSources();
    Clock(start);
    Wheel(start, .3, 0.0, .4);
    Imu(start, .4);
    for (int i = 1; i <= count; ++i)
    {
      const double time = start + .6 * i / count;
      Clock(time);
      Imu(time, .4);
    }
    return SourcePose(start + .6);
  };
  const auto slow = run(20.0, 12);
  const auto fast = run(30.0, 60);
  ASSERT_TRUE(slow);
  ASSERT_TRUE(fast);
  EXPECT_NEAR(slow->x(), fast->x(), 1e-13);
  EXPECT_NEAR(slow->y(), fast->y(), 1e-13);
  EXPECT_NEAR(slow->theta(), fast->theta(), 1e-13);
}

TEST_F(FusionGraphNodeInputTest, RewindRemovesPublishableOldEpochBeforeNewIntegration)
{
  const auto old_epoch = Epoch();
  Clock(1.0);
  Imu(1.0);
  EXPECT_GT(Epoch(), old_epoch);
  EXPECT_FALSE(OutputState());
  EXPECT_FALSE(SourcePose(10.02));
  Clock(1.02);
  Imu(1.02);
  ASSERT_TRUE(OutputState());
  EXPECT_EQ(OutputState()->source_ns, 1020000000);
  EXPECT_NEAR(OutputState()->predicted_pose.x(), .003, 1e-14);
}

TEST_F(FusionGraphNodeInputTest, NumericRecenterKeepsPublishedOdomAndSourceHistoryContinuous)
{
  SeedLatest();
  SetRecenterDistance(.001);
  const auto before = OutputState();
  const auto source_before = SourcePose(10.01);
  ASSERT_TRUE(before);
  ASSERT_TRUE(source_before);
  Timer();
  const auto after = OutputState();
  ASSERT_TRUE(after);
  ExpectRawPositionZero();
  EXPECT_NEAR(after->predicted_pose.x(), before->predicted_pose.x(), 1e-15);
  EXPECT_NEAR(after->predicted_pose.y(), before->predicted_pose.y(), 1e-15);
  ASSERT_TRUE(SourcePose(10.01));
  EXPECT_NEAR(SourcePose(10.01)->x(), source_before->x(), 1e-15);
  Clock(10.04);
  Imu(10.04);
  ASSERT_TRUE(OutputState());
  EXPECT_NEAR(OutputState()->predicted_pose.x(), before->predicted_pose.x() + .003, 1e-14);
}

TEST_F(FusionGraphNodeInputTest, AnchoredStationaryCadenceRetainsOnlyNeededDrSupport)
{
  SeedLatest();
  Timer();
  Wheel(10.03, 0.0);
  for (int i = 1; i <= 1600; ++i)
  {
    const double time = 10.02 + .01 * i;
    Clock(time);
    Imu(time);
    if (i % 20 == 0)
      Timer();
    // A stationary graph node can span five seconds, but historical graph
    // retention does not force the frontend to retain every earlier DR step.
    EXPECT_LT(ConsumedSteps(), 600u);
  }
  ASSERT_TRUE(OutputState());
  EXPECT_TRUE(OutputState()->anchor_valid);
}

TEST_F(FusionGraphNodeInputTest, SeedPreservesGnssEpochUntilConsumedMotionBracketsIt)
{
  SupplySeedYaw(0.0);
  Clock(10.06);
  Gnss(10.04, .02);
  EXPECT_FALSE(Snapshot());
  Imu(10.06);
  Timer();
  const auto snapshot = Snapshot();
  ASSERT_TRUE(snapshot);
  // Initialization uses the accepted observation epoch, never receipt now.
  EXPECT_EQ(snapshot->source_stamp_ns, 10040000000);
  ASSERT_TRUE(SourcePose(10.04));
  ASSERT_TRUE(OutputState());
  EXPECT_EQ(OutputState()->source_ns, 10060000000);
}

TEST_F(FusionGraphNodeInputTest, GnssOnlyRevisionRefreshesAnchorAtSnapshotSource)
{
  SeedLatest();
  Timer();
  const auto before = Snapshot();
  ASSERT_TRUE(before);
  const auto lidar_index = LastLidarSourceIndex();
  Clock(10.04);
  Imu(10.04);
  Gnss(10.02, .02);
  Timer();
  EXPECT_EQ(LastLidarSourceIndex(), lidar_index);
  const auto after = Snapshot();
  const auto output = OutputState();
  ASSERT_TRUE(after);
  ASSERT_TRUE(output);
  EXPECT_EQ(after->node_index, before->node_index);
  EXPECT_GT(after->revision, before->revision);
  ASSERT_TRUE(output->anchor_valid);
  const auto source_pose = SourcePose(static_cast<double>(after->source_stamp_ns) * 1e-9);
  ASSERT_TRUE(source_pose);
  const auto represented = output->anchor.compose(*source_pose);
  EXPECT_NEAR(represented.x(), after->pose.x(), 1e-12);
  EXPECT_NEAR(represented.y(), after->pose.y(), 1e-12);
}

TEST_F(FusionGraphNodeInputTest, SourceEpochResetInvalidatesFastPublicationCache)
{
  PrimeExtrapolator();
  ASSERT_TRUE(HasExtrapolatedPose());
  Clock(1.0);
  Imu(1.0);
  EXPECT_FALSE(HasExtrapolatedPose());
}

TEST_F(FusionGraphNodeInputTest,
       FastPublisherRejectsExpiredAndFutureSourceButLiveStationaryContinues)
{
  SeedLatest();
  Timer();
  EXPECT_TRUE(FastEligible(10.1));
  EXPECT_FALSE(FastEligible(10.3));
  EXPECT_FALSE(FastEligible(10.01));
  Clock(10.22);
  Wheel(10.22, 0.0);
  Imu(10.22, .02);
  Timer();
  EXPECT_TRUE(FastEligible(10.3));
  EXPECT_DOUBLE_EQ(FastGyro(), IntegratedGyro());
  Clock(10.3);
  Imu(10.5);  // An accepted source in the future cannot authorize now prediction.
  Timer();
  EXPECT_FALSE(FastEligible(10.3));
}

TEST_F(FusionGraphNodeInputTest, CoreSnapshotLossInvalidatesFastPublicationCache)
{
  SeedLatest();
  PrimeExtrapolator();
  ASSERT_TRUE(HasExtrapolatedPose());
  SimulateCoreReset();
  Timer();
  EXPECT_FALSE(HasExtrapolatedPose());
  ASSERT_TRUE(OutputState());
  EXPECT_FALSE(OutputState()->anchor_valid);
}

TEST_F(FusionGraphNodeInputTest, DockingLatchesRejectRetiredButAdmitUnbracketedSourceEpoch)
{
  SeedLatest();
  SetDocked();
  Clock(10.03);
  Gnss(10.01, .02);  // Before the initialized source state at 10.02.
  EXPECT_FALSE(UsableGnssReceipt());
  EXPECT_FALSE(FixedGnssReceipt());
  Clock(10.05);
  Gnss(10.04, .02);  // Newer than consumed IMU, still valid current evidence.
  ASSERT_TRUE(UsableGnssReceipt());
  ASSERT_TRUE(FixedGnssReceipt());
  EXPECT_EQ(UsableGnssReceipt()->nanoseconds(), 10040000000);
  EXPECT_EQ(FixedGnssReceipt()->nanoseconds(), 10040000000);
}

TEST_F(FusionGraphNodeInputTest, DockSeedSurvivesClearUntilNewConsumedSourceIsAvailable)
{
  SeedLatest();
  SetDocked();
  RememberSeededDockSession();
  ResetSources();
  SeedDock();  // The clear operation has no consumed source interval yet.
  EXPECT_FALSE(Snapshot());
  Clock(20.0);
  Wheel(20.0, 0.0);
  Imu(20.0);
  Clock(20.02);
  Wheel(20.02, 0.0);
  Imu(20.02);
  Timer();
  ASSERT_TRUE(Snapshot());
  ASSERT_TRUE(OutputState());
  EXPECT_TRUE(OutputState()->anchor_valid);
  EXPECT_EQ(Snapshot()->source_stamp_ns, 20020000000);
}

TEST_F(FusionGraphNodeInputTest, AcquisitionEpochPhaseAndReceiptDelayDoNotBiasPerfectMotion)
{
  // Historical discriminator: each case drives the real callbacks, cadence
  // gate, conditional acquisition state and iSAM2 optimizer. In the old path
  // an off-node perfect fix produced a 14.814 mm final mean error.
  for (const double epoch : {-.1, .98, 1.0, 1.02, 1.08, 1.12, 1.14, 1.16, 1.20, 1.24})
  {
    SCOPED_TRACE(epoch);
    const auto immediate = RunGnssPhase(1.24, epoch);
    const auto delayed = RunGnssPhase(1.44, epoch);
    const uint64_t expected = epoch < 0.0 ? 0u : 1u;
    EXPECT_EQ(immediate.admitted, expected);
    EXPECT_EQ(delayed.admitted, expected);
    EXPECT_EQ(immediate.applied, expected);
    EXPECT_EQ(delayed.applied, expected);
    EXPECT_NEAR(immediate.x, .18, 1e-6);
    EXPECT_NEAR(delayed.x, .18, 1e-6);
    EXPECT_NEAR(immediate.y, 0.0, 1e-9);
    EXPECT_NEAR(delayed.y, 0.0, 1e-9);
    EXPECT_NEAR(immediate.yaw, 0.0, 1e-9);
    EXPECT_NEAR(delayed.yaw, 0.0, 1e-9);
    EXPECT_NEAR(immediate.x, delayed.x, 1e-9);
  }
}

TEST_F(FusionGraphNodeInputTest, BootstrapRetimesAbsoluteHeadingToGnssSourceBeforeReplay)
{
  for (const bool cog : {false, true})
  {
    SCOPED_TRACE(cog);
    ResetSources();
    const double start = cog ? 30.0 : 20.0;
    Clock(start);
    Wheel(start, 0.0);
    Imu(start, .5);
    Clock(start + .2);
    Imu(start + .2, .5);
    Clock(start + .6);
    Imu(start + .6, .5);
    Gnss(start + .2, 0.0);
    Heading(start + .6, .3, cog);
    const auto seeded = Snapshot();
    ASSERT_TRUE(seeded);
    EXPECT_EQ(seeded->source_stamp_ns, std::llround((start + .2) * 1e9));
    EXPECT_NEAR(seeded->pose.theta(), .1, 1e-12);
    Timer();
    const auto output = OutputState();
    ASSERT_TRUE(output);
    ASSERT_TRUE(output->anchor_valid);
    EXPECT_NEAR(output->anchor.compose(output->source_pose).theta(), .3, 1e-12);
  }
}

TEST_F(FusionGraphNodeInputTest, BootstrapWaitsUntilHeadingAndGnssHaveCommonSourceSupport)
{
  ResetSources();
  Clock(20.0);
  Wheel(20.0, 0.0);
  Imu(20.0, .5);
  Clock(20.2);
  Imu(20.2, .5);
  Clock(20.6);
  Gnss(20.2, 0.0);
  Heading(20.6, .3);
  EXPECT_FALSE(Snapshot());
  Imu(20.6, .5);
  Timer();
  ASSERT_TRUE(Snapshot());
  EXPECT_NEAR(Snapshot()->pose.theta(), .1, 1e-12);
}

TEST_F(FusionGraphNodeInputTest, BootstrapDoesNotReuseHeadingAcrossIntegrationEpochs)
{
  Heading(10.02, .7);
  Clock(1.0);
  Wheel(1.0, 0.0);
  Imu(1.0);
  Clock(1.2);
  Imu(1.2);
  Gnss(1.2, 0.0);
  EXPECT_FALSE(Snapshot());
  Heading(1.2, .1);
  ASSERT_TRUE(Snapshot());
  EXPECT_NEAR(Snapshot()->pose.theta(), .1, 1e-12);
}

TEST_F(FusionGraphNodeInputTest, BootstrapConvertsAntennaSeedToBaseAtSourceHeading)
{
  node_.reset();
  CreateNode(.1, .2, -.1);
  Clock(20.0);
  Wheel(20.0, 0.0);
  Imu(20.0);
  Clock(20.2);
  Imu(20.2);
  const double yaw = .7;
  const double antenna_x = 1.0 + .2 * std::cos(yaw) + .1 * std::sin(yaw);
  const double antenna_y = 2.0 + .2 * std::sin(yaw) - .1 * std::cos(yaw);
  Gnss(20.2, antenna_x, .01, antenna_y);
  Heading(20.2, yaw);
  const auto seeded = Snapshot();
  ASSERT_TRUE(seeded);
  EXPECT_NEAR(seeded->pose.x(), 1.0, 1e-8);
  EXPECT_NEAR(seeded->pose.y(), 2.0, 1e-8);
  EXPECT_NEAR(seeded->pose.theta(), yaw, 1e-12);
}

TEST_F(FusionGraphNodeInputTest, ExplicitClearRetainsSupportedHeadingForStationaryFreshGnss)
{
  Clock(10.04);
  Wheel(10.04, 0.0);
  Imu(10.04);
  SeedLatest(.7);
  SetOffDock();
  ASSERT_TRUE(ExplicitClear());
  EXPECT_FALSE(Snapshot());
  ASSERT_TRUE(OutputState());
  EXPECT_EQ(OutputState()->source_ns, 10040000000);
  Clock(10.06);
  Wheel(10.06, 0.0);
  Imu(10.06);
  Gnss(10.06, .1);
  Timer();
  ASSERT_TRUE(Snapshot());
  EXPECT_NEAR(Snapshot()->pose.theta(), .7, 1e-12);
  EXPECT_NEAR(Snapshot()->pose.x(), .1, 1e-8);
}

TEST_F(FusionGraphNodeInputTest, ExplicitClearRetimesSnapshotHeadingThroughConsumedTurn)
{
  ResetSources();
  Clock(20.0);
  Wheel(20.0, 0.0);
  Imu(20.0, .5);
  Clock(20.2);
  Imu(20.2, .5);
  SeedLatest(.1);
  Clock(20.6);
  Imu(20.6, .5);
  SetOffDock();
  ASSERT_TRUE(ExplicitClear());
  Clock(20.62);
  Wheel(20.62, 0.0);
  Imu(20.62);
  Gnss(20.62, 0.0);
  ASSERT_TRUE(Snapshot());
  EXPECT_NEAR(Snapshot()->pose.theta(), .3, 1e-12);
}

TEST_F(FusionGraphNodeInputTest, ExplicitClearPreservesGnssDedupAgainstCachedPosition)
{
  Clock(10.04);
  Wheel(10.04, 0.0);
  Imu(10.04);
  SeedLatest(.7);
  SetOffDock();
  Gnss(10.04, 0.0);
  ASSERT_TRUE(UsableGnssReceipt());
  ASSERT_TRUE(ExplicitClear());
  Gnss(10.04, 0.0);
  EXPECT_FALSE(UsableGnssReceipt());
  EXPECT_FALSE(Snapshot());
  Clock(10.06);
  Wheel(10.06, 0.0);
  Imu(10.06);
  Gnss(10.06, .1);
  ASSERT_TRUE(Snapshot());
  EXPECT_NEAR(Snapshot()->pose.theta(), .7, 1e-12);
}

TEST_F(FusionGraphNodeInputTest, ExplicitClearHeadingPriorDoesNotCrossGapOrRewind)
{
  for (const double restart : {1.0, 20.0})
  {
    SCOPED_TRACE(restart);
    node_.reset();
    CreateNode();
    Clock(10.0);
    Wheel(10.0, 0.0);
    Imu(10.0);
    Clock(10.02);
    Wheel(10.02, 0.0);
    Imu(10.02);
    SeedLatest(.7);
    SetOffDock();
    ASSERT_TRUE(ExplicitClear());
    Clock(restart);
    Imu(restart);
    Clock(restart + .02);
    Wheel(restart + .02, 0.0);
    Imu(restart + .02);
    Gnss(restart + .02, 0.0);
    EXPECT_FALSE(Snapshot());
    Heading(restart + .02, .1);
    ASSERT_TRUE(Snapshot());
    EXPECT_NEAR(Snapshot()->pose.theta(), .1, 1e-12);
  }
}

TEST_F(FusionGraphNodeInputTest, DeferredDockSeedCannotApplyAfterUndock)
{
  ResetSources();
  SetDocked();
  RememberSeededDockSession();
  SeedDock();
  SetOffDock();
  Clock(20.0);
  Wheel(20.0, 0.0);
  Imu(20.0);
  Clock(20.02);
  Imu(20.02);
  Timer();
  EXPECT_FALSE(Snapshot());
}

TEST_F(FusionGraphNodeInputTest, DockSeedRecoversCoreResetWithoutAnotherHardwareStatusEdge)
{
  SeedLatest();
  SetDocked();
  RememberSeededDockSession();
  SimulateCoreReset();
  Timer();
  for (int i = 2; i <= 4; ++i)
  {
    const double time = 10.0 + .02 * i;
    Clock(time);
    Wheel(time, 0.0);
    Imu(time);
    Timer();
  }
  ASSERT_TRUE(Snapshot());
  ASSERT_TRUE(OutputState());
  EXPECT_TRUE(OutputState()->anchor_valid);
}

TEST_F(FusionGraphNodeInputTest, FutureCogCannotBlockReadyChecksBeforeTheirFactors)
{
  StartCogScenario(M_PI);
  Clock(100.4);
  Imu(100.4);
  Clock(100.65);
  Wheel(100.65, .15);
  Gnss(100.4, 0.0);
  Heading(100.64, 0.0, true);  // Not consumed yet; delivery order is deliberate.
  Heading(100.1, 0.0, true);
  Heading(100.2, 0.0, true);
  EXPECT_EQ(CogDebounce(), 2);
  EXPECT_EQ(CogRecoveries(), 0u);
  EXPECT_EQ(PendingCogChecks(), 1u);
  EXPECT_EQ(AppliedYawFactors(), 0u);
  Timer();
  EXPECT_EQ(AppliedYawFactors(), 2u);
  EXPECT_EQ(PendingCogChecks(), 1u);
}

TEST_F(FusionGraphNodeInputTest, RewindBeforeExplicitClearCannotCarryOldSourceHeading)
{
  node_.reset();
  CreateNode(100.0);  // Keep the service as the first callback after rewind.
  Clock(10.0);
  Wheel(10.0, 0.0);
  Imu(10.0);
  Clock(10.02);
  Imu(10.02);
  SeedLatest(.7);
  SetOffDock();
  Clock(1.0);
  ASSERT_TRUE(ExplicitClear());
  EXPECT_FALSE(OutputState());
  EXPECT_FALSE(Snapshot());
}

TEST_F(FusionGraphNodeInputTest, RecoveryCooldownBelongsToItsRosClockEpoch)
{
  StartCogScenario(M_PI);
  RememberRecoveryCooldown();
  ResetSources();  // Same-clock resets keep the existing minimum interval.
  EXPECT_TRUE(HasRecoveryCooldown());
  Clock(1.0);
  Wheel(1.0, 0.0);
  Imu(1.0);
  EXPECT_FALSE(HasRecoveryCooldown());
  Clock(1.2);
  Wheel(1.2, .15);
  Imu(1.2);
  SeedLatest(M_PI);
  SetOffDock();
  for (int i = 1; i <= 3; ++i)
  {
    const double time = 1.2 + .8 * i;
    Clock(time);
    Wheel(time, .15);
    Imu(time);
    Gnss(time, .12 * i);
    Heading(time, 0.0, true);
  }
  EXPECT_EQ(CogRecoveries(), 1u);
}

TEST_F(FusionGraphNodeInputTest, HistoricalCogBeforePivotCannotFalselyRecoverCurrentHeading)
{
  StartCogScenario(0.0);
  for (int i = 1; i <= 351; ++i)
  {
    const double time = 100.0 + .02 * i;
    Clock(time);
    const bool pivot = i > 150 && i <= 350;
    Wheel(time, pivot ? 0.0 : .15, 0.0, pivot ? M_PI / 4.0 : 0.0);
    Imu(time, pivot ? M_PI / 4.0 : 0.0);
    Timer();
  }
  Gnss(107.02, .447, .003);
  for (const double source : {100.8, 101.6, 102.4})
    Heading(source, 0.0, true);
  ASSERT_TRUE(Snapshot());
  EXPECT_EQ(CogRecoveries(), 0u);
  EXPECT_EQ(CogDebounce(), 0);
  EXPECT_NEAR(std::atan2(std::sin(Snapshot()->pose.theta() - M_PI),
                         std::cos(Snapshot()->pose.theta() - M_PI)),
              0.0,
              1e-10);
}

TEST_F(FusionGraphNodeInputTest, TrueHistoricalCogFlipPreservesInterveningTurnWithGnssAcquisition)
{
  const double recovered = RunTrueHistoricalCogFlip(true);
  const double reference = RunTrueHistoricalCogFlip(false);
  EXPECT_NEAR(std::atan2(std::sin(recovered - reference), std::cos(recovered - reference)),
              0.0,
              1e-10);
}

TEST_F(FusionGraphNodeInputTest, FutureCogChecksRecoverOnlyAfterWholeSourceIntervalIsConsumed)
{
  PrepareFutureCogChecks();
  EXPECT_EQ(CogRecoveries(), 0u);
  EXPECT_EQ(CogDebounce(), 0);
  EXPECT_EQ(PendingCogChecks(), 3u);
  Imu(100.64);
  Timer();
  EXPECT_EQ(CogRecoveries(), 1u);
  EXPECT_EQ(PendingCogChecks(), 0u);
  ASSERT_TRUE(Snapshot());
}

TEST_F(FusionGraphNodeInputTest, DuplicateCogCannotAdvanceRecoveryAndRetiredSourceResetsDebounce)
{
  StartCogScenario(M_PI);
  Clock(100.4);
  Imu(100.4);
  Clock(100.5);
  Wheel(100.5, .15);
  Gnss(100.4, 0.0);
  for (int i = 0; i < 3; ++i)
    Heading(100.2, 0.0, true);
  EXPECT_EQ(CogDebounce(), 1);
  EXPECT_EQ(CogRecoveries(), 0u);
  Heading(99.9, 0.0, true);
  EXPECT_EQ(CogDebounce(), 0);
  EXPECT_EQ(CogSourceRejects(), 1u);
}

TEST_F(FusionGraphNodeInputTest, SourceResetAndExplicitClearInvalidatePendingCogChecks)
{
  PrepareFutureCogChecks();
  ASSERT_EQ(PendingCogChecks(), 3u);
  ResetSources();
  EXPECT_EQ(PendingCogChecks(), 0u);
  EXPECT_EQ(CogDebounce(), 0);
  PrepareFutureCogChecks();
  Clock(100.66);
  Wheel(100.66, 0.0);
  ASSERT_TRUE(ExplicitClear());
  EXPECT_EQ(PendingCogChecks(), 0u);
  EXPECT_EQ(CogDebounce(), 0);
}

TEST_F(FusionGraphNodeInputTest, PendingCogCapacityFailureIsBoundedAndObservable)
{
  StartCogScenario(M_PI);
  Clock(100.4);
  Imu(100.4);
  Clock(101.9);
  Wheel(101.9, .15);
  Gnss(100.4, 0.0);
  for (int i = 0; i <= 1024; ++i)
    Heading(100.41 + .00001 * i, 0.0, true);
  EXPECT_EQ(PendingCogChecks(), 1024u);
  EXPECT_EQ(CogSourceRejects(), 1u);
  EXPECT_EQ(CogRecoveries(), 0u);
}

}  // namespace fusion_graph
