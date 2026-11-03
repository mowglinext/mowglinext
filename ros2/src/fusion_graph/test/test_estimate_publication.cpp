// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the production OnTimer + PublishOutputs, including the real ROS
// publisher. Access only supplies deterministic graph/DR inputs and clock.

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace fusion_graph
{
struct FusionGraphNodeTestAccess
{
  static void Sync(FusionGraphNode& n)
  {
    n.SynchronizeSupportEpoch();
  }
  static void Cache(FusionGraphNode& n, uint64_t token, const sensor_msgs::msg::NavSatFix& fix)
  {
    n.RememberSupportFix(token, fix);
  }
  static void Odom(FusionGraphNode& n, const TickOutput& snapshot, int sec)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    n.RecordSupportOdom(snapshot, stamp);
  }
  static void TfSent(FusionGraphNode& n, uint64_t epoch, uint64_t revision, int sec)
  {
    std::lock_guard<std::mutex> lock(n.tf_state_mu_);
    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    n.support_tf_ = FusionGraphNode::SupportTf{epoch, revision, stamp};
    n.PublishSupportLocked();
  }
  static void Gnss(FusionGraphNode& n, const sensor_msgs::msg::NavSatFix& fix)
  {
    n.OnGnss(std::make_shared<const sensor_msgs::msg::NavSatFix>(fix));
  }
  static void ObserverMode(FusionGraphNode& n)
  {
    n.primary_mode_ = false;
  }
  static void StartTfThread(FusionGraphNode& n)
  {
    n.tf_broadcast_rate_hz_ = 20.;
    n.tf_thread_stop_.store(false);
    n.tf_thread_ = std::thread(
        [&n]()
        {
          n.TfBroadcastLoop();
        });
  }
  static GraphManager& Graph(FusionGraphNode& n)
  {
    return *n.graph_;
  }
  static void SetDr(FusionGraphNode& n, const gtsam::Pose2& dr)
  {
    std::lock_guard<std::mutex> lock(n.tf_state_mu_);
    n.dr_x_ = dr.x();
    n.dr_y_ = dr.y();
    n.dr_yaw_ = dr.theta();
  }
  static gtsam::Pose2 Dr(FusionGraphNode& n)
  {
    return gtsam::Pose2(n.dr_x_, n.dr_y_, n.dr_yaw_);
  }
  static void RebaseDistance(FusionGraphNode& n, double distance)
  {
    n.odom_rebase_dist_m_ = distance;
  }
  static void Timer(FusionGraphNode& n, double t)
  {
    auto clock = n.get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, int64_t(t * 1e9)), RCL_RET_OK);
    n.OnTimer();
  }
};
}  // namespace fusion_graph

namespace fg = fusion_graph;
namespace
{
class PublicationTest : public ::testing::Test
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
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({{"use_sim_time", true},
                              {"autoload_graph", false},
                              {"auto_save_enabled", false},
                              {"use_lidar_map_anchor", false},
                              {"tf_broadcast_rate_hz", 0.},
                              {"node_period_s", .04},
                              {"lever_arm_x", .3},
                              {"stationary_node_period_s", 5.},
                              {"gyro_bias_estimation_enabled", false},
                              {"adaptive_noise_enabled_gain", 0.}});
    node = std::make_shared<fg::FusionGraphNode>(opts);
    listener = std::make_shared<rclcpp::Node>("estimate_publication_test");
    sub = listener->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered_map",
        10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message)
        {
          const auto& p = message->pose.pose;
          received.emplace_back(p.position.x, p.position.y, fg::YawFromQuat(p.orientation));
        });
    executor.add_node(listener);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_GT(sub->get_publisher_count(), 0u);
  }
  void TearDown() override
  {
    executor.remove_node(listener);
    sub.reset();
    listener.reset();
    node.reset();
  }
  gtsam::Pose2 Publish(double t)
  {
    const auto count = received.size();
    fg::FusionGraphNodeTestAccess::Timer(*node, t);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received.size() == count && std::chrono::steady_clock::now() < deadline)
      executor.spin_some(std::chrono::milliseconds(10));
    EXPECT_GT(received.size(), count);
    return received.empty() ? gtsam::Pose2() : received.back();
  }
  std::shared_ptr<fg::FusionGraphNode> node;
  std::shared_ptr<rclcpp::Node> listener;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<gtsam::Pose2> received;
};

TEST_F(PublicationTest, SameIndexCorrectionRetainsSe2TailAcrossNewNodeAndOdomRebase)
{
  auto& graph = fg::FusionGraphNodeTestAccess::Graph(*node);
  const gtsam::Pose2 head(1., -2., .7), dr_head(.4, -.3, -.5), tail(.017, -.004, .08);
  graph.Initialize(head, 10.);
  fg::FusionGraphNodeTestAccess::SetDr(*node, dr_head);
  EXPECT_TRUE(Publish(10.05).equals(head, 1e-10));
  fg::FusionGraphNodeTestAccess::SetDr(*node, dr_head.compose(tail));
  const auto before = Publish(10.1);
  EXPECT_TRUE(before.equals(head.compose(tail), 1e-10));
  graph.ForceAnchor(0, gtsam::Pose2(1.02, -2.03, .9), .001, .001);
  const auto corrected = *graph.LatestSnapshot();
  const auto after = Publish(10.15);
  EXPECT_EQ(corrected.node_index, 0u);
  EXPECT_TRUE(after.equals(corrected.pose.compose(tail), 1e-9));
  EXPECT_TRUE(corrected.pose.between(after).equals(tail, 1e-9));
  EXPECT_FALSE(after.equals(corrected.pose, 1e-3));  // catches recapture-current-DR

  // The coordinated odom rebase changes DR and anchor, preserving map→base.
  fg::FusionGraphNodeTestAccess::RebaseDistance(*node, .1);
  EXPECT_TRUE(Publish(10.2).equals(after, 1e-9));
  EXPECT_NEAR(fg::FusionGraphNodeTestAccess::Dr(*node).x(), 0., 1e-12);
  graph.ForceAnchor(0, gtsam::Pose2(.98, -1.96, .8), .001, .001);
  const auto revised = *graph.LatestSnapshot();
  const auto rebased = Publish(10.25);
  EXPECT_TRUE(revised.pose.between(rebased).equals(tail, 1e-9));

  // Maintenance is another same-index revision. It preserves this tail too,
  // and gives the next process edge an independent, coherent snapshot prior.
  // This avoids conflating tail consumption with iSAM2's incremental nonlinear
  // relinearization of the two competing ForceAnchor priors above.
  graph.RebaseISAM2();
  EXPECT_TRUE(Publish(10.3).equals(rebased, 1e-9));

  // Consume the exact same process interval once in an ordinary node. The
  // optimized node catches up to the published pose; there is no second tail.
  graph.AddWheelTwist(tail.x(), tail.y(), tail.theta(), 1.);
  graph.AddGyroDelta(tail.theta(), 1.);
  const auto next = Publish(10.4);
  ASSERT_EQ(graph.LatestSnapshot()->node_index, 1u);
  EXPECT_TRUE(next.equals(graph.LatestSnapshot()->pose, 1e-9));
  // iSAM2 may also refine the predecessor when this update relinearizes.
  // Compare against that optimized predecessor, rather than assuming its
  // earlier nonlinear estimate remains bit-identical.
  const auto predecessor = graph.GetPose(0);
  ASSERT_TRUE(predecessor);
  EXPECT_TRUE(predecessor->between(next).equals(tail, 1e-8))
      << "predecessor " << *predecessor << " next " << next << " relative "
      << predecessor->between(next) << " expected " << tail;
  EXPECT_TRUE(Publish(10.45).equals(next, 1e-9));
}

TEST_F(PublicationTest, GnssOnlyRevisionIsPublishedWithResidualTail)
{
  auto& graph = fg::FusionGraphNodeTestAccess::Graph(*node);
  const gtsam::Pose2 head(.2, -.1, .8), dr_head(.1, .2, -.4), tail(.017, 0., .06);
  graph.Initialize(head, 10.);
  fg::FusionGraphNodeTestAccess::SetDr(*node, dr_head);
  Publish(10.05);
  fg::FusionGraphNodeTestAccess::SetDr(*node, dr_head.compose(tail));
  const auto before = Publish(10.1);
  const gtsam::Vector2 antenna =
      head.transformFrom(gtsam::Point2(.3, 0.)) + gtsam::Vector2(.02, -.01);
  ASSERT_TRUE(graph.QueueGnss(antenna.x(), antenna.y(), .003, false, 0));
  const auto after = Publish(10.2);
  const auto revised = *graph.LatestSnapshot();
  EXPECT_EQ(revised.node_index, 0u);
  EXPECT_GT(revised.pose.between(head).translation().norm(), .005);
  EXPECT_TRUE(after.equals(revised.pose.compose(tail), 1e-9));
  EXPECT_GT(before.between(after).translation().norm(), .005);
  EXPECT_TRUE(Publish(10.24).equals(after, 1e-9));
}
}  // namespace

TEST_F(PublicationTest, SupportRequiresMatchingSentTfAndRetainsExactAppliedFix)
{
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  auto facts_sub = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
      "/fusion_graph/gnss_update",
      rclcpp::QoS(10).reliable().transient_local(),
      [&facts](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr msg)
      {
        facts.push_back(*msg);
      });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (facts_sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto& g = fg::FusionGraphNodeTestAccess::Graph(*node);
  g.Initialize(gtsam::Pose2(), 1.0);
  fg::FusionGraphNodeTestAccess::Sync(*node);
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 1;
  fix.latitude = 48.0;
  fix.position_covariance[0] = .09;
  fg::FusionGraphNodeTestAccess::Cache(*node, 7, fix);
  ASSERT_TRUE(g.QueueGnss(.3, 0., .3, true, 0, 7));
  EXPECT_FALSE(g.Tick(1.04));
  auto snap = *g.LatestSnapshot();
  fg::FusionGraphNodeTestAccess::Odom(*node, snap, 2);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision - 1, 3);
  executor.spin_some(std::chrono::milliseconds(50));
  for (const auto& fact : facts)
    EXPECT_FALSE(fact.pose_published);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision, 4);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  ASSERT_FALSE(facts.empty());
  const auto fact = facts.back();
  EXPECT_TRUE(fact.pose_published);
  EXPECT_TRUE(fact.has_applied_fix);
  EXPECT_EQ(fact.header.stamp.sec, 2);
  EXPECT_EQ(fact.tf_support_stamp.sec, 4);
  EXPECT_EQ(fact.applied_fix.header.stamp.sec, 1);
  EXPECT_EQ(fact.applied_fix.latitude, 48.0);
  EXPECT_EQ(fact.applied_revision, snap.estimate_revision);
  // Pending eviction cannot erase the separately pinned committed observation.
  for (uint64_t token = 8; token < 50; ++token)
    fg::FusionGraphNodeTestAccess::Cache(*node, token, fix);
  fg::FusionGraphNodeTestAccess::Odom(*node, snap, 5);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision, 6);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  EXPECT_TRUE(facts.back().has_applied_fix);
  EXPECT_EQ(facts.back().applied_fix.header.stamp.sec, 1);
  g.Reset();
  fg::FusionGraphNodeTestAccess::Sync(*node);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision, 5);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  EXPECT_FALSE(facts.back().has_applied_fix);
  EXPECT_GT(facts.back().graph_epoch, snap.graph_epoch);
}

TEST_F(PublicationTest, MissingRawTokenCannotClaimPublishedSupport)
{
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  auto sub = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
      "/fusion_graph/gnss_update",
      rclcpp::QoS(10).reliable().transient_local(),
      [&facts](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr msg)
      {
        facts.push_back(*msg);
      });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto& g = fg::FusionGraphNodeTestAccess::Graph(*node);
  g.Initialize(gtsam::Pose2(), 1.0);
  fg::FusionGraphNodeTestAccess::Sync(*node);
  ASSERT_TRUE(g.QueueGnss(.3, 0., .3, true, 0, 88));
  EXPECT_FALSE(g.Tick(1.04));
  auto snap = *g.LatestSnapshot();
  fg::FusionGraphNodeTestAccess::Odom(*node, snap, 2);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision, 3);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  ASSERT_FALSE(facts.empty());
  EXPECT_FALSE(facts.back().pose_published);
  EXPECT_FALSE(facts.back().has_applied_fix);
}

TEST_F(PublicationTest, DedicatedTfThreadPublishesOnlyAfterActualSend)
{
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  auto sub = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
      "/fusion_graph/gnss_update",
      rclcpp::QoS(10).reliable().transient_local(),
      [&facts](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr msg)
      {
        facts.push_back(*msg);
      });
  const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < discovery)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto& g = fg::FusionGraphNodeTestAccess::Graph(*node);
  g.Initialize(gtsam::Pose2(), 1.0);
  fg::FusionGraphNodeTestAccess::Sync(*node);
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 1;
  fix.longitude = 11.5;
  fg::FusionGraphNodeTestAccess::Cache(*node, 99, fix);
  ASSERT_TRUE(g.QueueGnss(.3, 0., .3, true, 0, 99));
  fg::FusionGraphNodeTestAccess::StartTfThread(*node);
  fg::FusionGraphNodeTestAccess::Timer(*node, 1.04);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
    if (!facts.empty() && facts.back().pose_published)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_FALSE(facts.empty());
  EXPECT_TRUE(facts.back().pose_published);
  EXPECT_TRUE(facts.back().has_applied_fix);
  EXPECT_EQ(facts.back().applied_fix.longitude, 11.5);
  EXPECT_EQ(facts.back().estimate_revision, g.LatestSnapshot()->estimate_revision);
  EXPECT_GE(rclcpp::Time(facts.back().tf_support_stamp).nanoseconds(),
            rclcpp::Time(facts.back().header.stamp).nanoseconds());
}

TEST_F(PublicationTest, RawFixIsAcknowledgedOnlyAfterNodeCommitsAndPublishes)
{
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  auto sub = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
      "/fusion_graph/gnss_update",
      rclcpp::QoS(10).reliable().transient_local(),
      [&facts](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr msg)
      {
        facts.push_back(*msg);
      });
  const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < discovery)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto& g = fg::FusionGraphNodeTestAccess::Graph(*node);
  g.Initialize(gtsam::Pose2(), 1.0);
  fg::FusionGraphNodeTestAccess::Sync(*node);
  fg::FusionGraphNodeTestAccess::Timer(*node, 1.04);
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp.sec = 1;
  fix.header.stamp.nanosec = 40000000;
  fix.header.frame_id = "gps_link";
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  fix.latitude = 48.1;
  fix.longitude = 11.5;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = fix.position_covariance[4] = .000009;
  fg::FusionGraphNodeTestAccess::Gnss(*node, fix);
  EXPECT_EQ(g.LatestSnapshot()->applied_gnss_token, 0u);
  fg::FusionGraphNodeTestAccess::Timer(*node, 1.08);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  ASSERT_FALSE(facts.empty());
  ASSERT_TRUE(facts.back().pose_published);
  ASSERT_TRUE(facts.back().has_applied_fix);
  EXPECT_EQ(facts.back().applied_fix.header.stamp, fix.header.stamp);
  EXPECT_EQ(facts.back().applied_fix.header.frame_id, fix.header.frame_id);
  EXPECT_EQ(facts.back().applied_fix.latitude, fix.latitude);
  EXPECT_EQ(facts.back().applied_fix.position_covariance, fix.position_covariance);
}

TEST_F(PublicationTest, ObserverNeverClaimsOwnedTf)
{
  std::vector<mowgli_interfaces::msg::GnssFusionUpdate> facts;
  auto sub = listener->create_subscription<mowgli_interfaces::msg::GnssFusionUpdate>(
      "/fusion_graph/gnss_update",
      rclcpp::QoS(10).reliable().transient_local(),
      [&facts](mowgli_interfaces::msg::GnssFusionUpdate::ConstSharedPtr msg)
      {
        facts.push_back(*msg);
      });
  const auto discovery = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < discovery)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto& g = fg::FusionGraphNodeTestAccess::Graph(*node);
  g.Initialize(gtsam::Pose2(), 1.0);
  fg::FusionGraphNodeTestAccess::Sync(*node);
  sensor_msgs::msg::NavSatFix fix;
  fg::FusionGraphNodeTestAccess::Cache(*node, 1, fix);
  ASSERT_TRUE(g.QueueGnss(.3, 0., .3, true, 0, 1));
  EXPECT_FALSE(g.Tick(1.04));
  const auto snap = *g.LatestSnapshot();
  fg::FusionGraphNodeTestAccess::ObserverMode(*node);
  fg::FusionGraphNodeTestAccess::Odom(*node, snap, 2);
  fg::FusionGraphNodeTestAccess::TfSent(*node, snap.graph_epoch, snap.estimate_revision, 3);
  for (int i = 0; i < 10; ++i)
    executor.spin_some(std::chrono::milliseconds(10));
  ASSERT_FALSE(facts.empty());
  for (const auto& fact : facts)
    EXPECT_FALSE(fact.pose_published);
}
