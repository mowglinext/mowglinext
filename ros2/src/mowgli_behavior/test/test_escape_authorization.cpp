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
#include <visualization_msgs/msg/marker_array.hpp>

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

  visualization_msgs::msg::MarkerArray Geometry(bool hole = false)
  {
    visualization_msgs::msg::MarkerArray result;
    visualization_msgs::msg::Marker area;
    area.header.frame_id = "map";
    area.ns = "area";
    area.text = "mowing";
    area.type = area.LINE_STRIP;
    area.action = area.ADD;
    for (const auto& xy :
         std::vector<std::pair<double, double>>{{-3, -3}, {3, -3}, {3, 3}, {-3, 3}, {-3, -3}})
    {
      geometry_msgs::msg::Point point;
      point.x = xy.first;
      point.y = xy.second;
      area.points.push_back(point);
    }
    result.markers.push_back(area);
    if (hole)
    {
      auto obstacle = area;
      obstacle.ns = "obstacle";
      obstacle.text.clear();
      obstacle.points.clear();
      // The axle misses this hole; the swept rear corner hits it in reverse.
      for (const auto& xy : std::vector<std::pair<double, double>>{
               {-0.50, 0.20}, {-0.49, 0.20}, {-0.49, 0.21}, {-0.50, 0.21}, {-0.50, 0.20}})
      {
        geometry_msgs::msg::Point point;
        point.x = xy.first;
        point.y = xy.second;
        obstacle.points.push_back(point);
      }
      result.markers.push_back(obstacle);
    }
    return result;
  }

  void Pose(double x = 0.0, double y = 0.0, double yaw = 0.0, double age = 0.0)
  {
    if (!ctx_->tf_buffer)
    {
      ctx_->tf_buffer = std::make_shared<tf2_ros::Buffer>(ctx_->node->get_clock());
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.header.stamp = ctx_->node->now() - rclcpp::Duration::from_seconds(age);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
    ASSERT_TRUE(ctx_->tf_buffer->setTransform(transform, "test", false));
  }

  void PublishGeometry(const visualization_msgs::msg::MarkerArray& geometry)
  {
    if (!geometry_pub_)
    {
      geometry_pub_ = observer_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local().reliable());
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (geometry_pub_->get_subscription_count() == 0 &&
             std::chrono::steady_clock::now() < deadline)
      {
        Pump();
      }
      ASSERT_GT(geometry_pub_->get_subscription_count(), 0u);
    }
    geometry_pub_->publish(geometry);
    for (int i = 0; i < 10; ++i)
    {
      Pump();
    }
  }

  void Authorize(bool hole = false)
  {
    ctx_->node->declare_parameter<std::vector<double>>(
        "motion_footprint", {0.53, 0.275, 0.53, -0.275, -0.17, -0.275, -0.17, 0.275});
    PublishGeometry(Geometry(hole));
    Pose();
    ctx_->last_status_time = std::chrono::steady_clock::now();
    ctx_->last_motion_time = ctx_->last_status_time;
  }

  void ObserveMotion(double x = 0.0)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (commands_.empty() && std::chrono::steady_clock::now() < deadline)
    {
      Pump();
      Pose(x);
      ctx_->last_status_time = std::chrono::steady_clock::now();
      ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
      Pump();
    }
    ASSERT_FALSE(commands_.empty());
    ASSERT_NE(commands_.back().twist.linear.x, 0.0);
  }

  void ExpectStopped()
  {
    EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::SUCCESS);
    for (int i = 0; i < 10; ++i)
    {
      Pump();
    }
    ASSERT_FALSE(commands_.empty());
    EXPECT_DOUBLE_EQ(commands_.back().twist.linear.x, 0.0);
    const auto count = commands_.size();
    EXPECT_EQ(escape_->onRunning(), BT::NodeStatus::SUCCESS);
    Pump();
    EXPECT_EQ(commands_.size(), count) << "Ended action must not replay a nonzero command";
  }

  std::shared_ptr<mowgli_behavior::BTContext> ctx_;
  std::unique_ptr<mowgli_behavior::EscapeStartBlocked> escape_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<Command>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometry_pub_;
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

TEST_F(EscapeAuthorization, AuthorizedReversePublishesExpectedNonzeroMotionAndHaltZero)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  EXPECT_LT(commands_.back().twist.linear.x, 0.0);
  escape_->haltNode();
  for (int i = 0; i < 10; ++i)
  {
    Pump();
  }
  EXPECT_DOUBLE_EQ(commands_.back().twist.linear.x, 0.0);
  EXPECT_EQ(escape_->onRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(EscapeAuthorization, AuthorizedForwardPreservesOuterBoundaryBodyOverhang)
{
  Authorize();
  // Front chassis reaches outside the recorded area, while the rear axle's
  // entire intended path stays inside. This is explicitly permitted overhang.
  ctx_->last_motion_cmd_vx = -0.20;
  Pose(2.50);
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion(2.50);
  EXPECT_GT(commands_.back().twist.linear.x, 0.0);
  escape_->haltNode();
}

TEST_F(EscapeAuthorization, FullBodySweepRefusesRawHoleEvenWhenAxlePathIsClear)
{
  Authorize(true);
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(EscapeAuthorization, CompleteDistanceMustRemainInAuthorizedReferenceRegion)
{
  Authorize();
  Pose(-2.8);
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(EscapeAuthorization, GeometryWithoutConfiguredChassisStandsDown)
{
  PublishGeometry(Geometry());
  Pose();
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(EscapeAuthorization, GeometryWithoutFreshPoseStandsDown)
{
  Authorize();
  ctx_->tf_buffer.reset();
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(EscapeAuthorization, RevocationStopsActiveEscape)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  PublishGeometry({});
  Pose();
  ExpectStopped();
}

TEST_F(EscapeAuthorization, GeometryChangeRequiresNewArmedOperation)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  auto changed = Geometry();
  changed.markers.front().points[1].x = 3.1;
  PublishGeometry(changed);
  Pose();
  ExpectStopped();
}

TEST_F(EscapeAuthorization, RevocationAndIdenticalRestorationCannotReuseOldPermission)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  PublishGeometry({});
  PublishGeometry(Geometry());
  Pose();
  ExpectStopped();
}

TEST_F(EscapeAuthorization, IdenticalGeometryRepublicationPreservesNormalRecovery)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  PublishGeometry(Geometry());
  Pose(-0.05);
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  escape_->haltNode();
}

TEST_F(EscapeAuthorization, MeasuredDriftStopsStraightEscape)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  Pose(0.0, 0.06);
  ExpectStopped();
}

TEST_F(EscapeAuthorization, MeasuredDistanceEndsEscapeBeforeFurtherMotion)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  Pose(-ctx_->start_blocked_escape_cfg.distance);
  ExpectStopped();
}
TEST_F(EscapeAuthorization, HeldCommandTapersToRemainingDistanceAndStillMakesProgress)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  Pose(-0.30);
  EXPECT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  for (int i = 0; i < 10; ++i)
  {
    Pump();
  }
  ASSERT_FALSE(commands_.empty());
  EXPECT_LT(commands_.back().twist.linear.x, 0.0);
  EXPECT_LE(std::abs(commands_.back().twist.linear.x) *
                mowgli_behavior::EscapeStartBlocked::kCommandHoldEnvelopeSec,
            0.10 + 1e-12);
  escape_->haltNode();
}

TEST_F(EscapeAuthorization, StalePoseStopsActiveEscape)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  ctx_->tf_buffer = std::make_shared<tf2_ros::Buffer>(ctx_->node->get_clock());
  Pose(0.0, 0.0, 0.0, 1.0);
  ExpectStopped();
}

TEST_F(EscapeAuthorization, AuthorizedEscapeMakesSustainedKinematicProgress)
{
  Authorize();
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  double x = 0;
  auto previous = std::chrono::steady_clock::now();
  for (int i = 0; i < 30; ++i)
  {
    const double command = commands_.back().twist.linear.x;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto now = std::chrono::steady_clock::now();
    x += command * std::chrono::duration<double>(now - previous).count();
    previous = now;
    Pose(x);
    ctx_->last_status_time = now;
    const auto status = escape_->executeTick();
    Pump();
    ASSERT_EQ(status, BT::NodeStatus::RUNNING);
    ASSERT_GE(x, -ctx_->start_blocked_escape_cfg.distance);
  }
  EXPECT_LT(x, -0.15) << "Authorized recovery must make sustained progress";
  escape_->haltNode();
  for (int i = 0; i < 10; ++i)
    Pump();
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().twist.linear.x, 0);
}

TEST_F(EscapeAuthorization, WallClockTimeoutCannotBeRefundedByTickClamping)
{
  Authorize();
  ctx_->start_blocked_escape_cfg.timeout_s = 1.0;
  ASSERT_EQ(escape_->executeTick(), BT::NodeStatus::RUNNING);
  ObserveMotion();
  std::this_thread::sleep_for(std::chrono::milliseconds(1010));
  Pose();
  ExpectStopped();
}
}  // namespace
