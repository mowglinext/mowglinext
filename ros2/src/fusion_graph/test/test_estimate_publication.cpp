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
