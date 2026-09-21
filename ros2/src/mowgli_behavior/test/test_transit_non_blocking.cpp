// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
/**
 * @file test_transit_non_blocking.cpp
 * @brief A transit that cannot reach the coverage start must not block the mow.
 *
 * Field 2026-09-21: the first unit's start sat 0.2 m from a hedge the LiDAR
 * saw. RPP refused the path ("collision ahead") for 53 s in TransitToStrip,
 * then FollowStrip sent the IDENTICAL transit and it failed again for 43 s
 * more before the unit was skipped. TransitToStrip now has a time bound, and
 * FollowStrip does not repeat a transit TransitToStrip has just failed — it
 * moves on to the next unit, leaving the unreachable one for a later pass.
 * A START_OCCUPIED refusal (the robot's OWN pose) is the exception: it is
 * still left to FollowStrip, whose transit is what arms the escape (#487).
 *
 * Real action servers (fake Nav2) + a real TF buffer; the nodes are ticked in
 * the shape main_tree.xml uses them.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "tf2_ros/buffer.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::FollowStrip;
using mowgli_behavior::sameTransitTarget;
using mowgli_behavior::TransitToStrip;
using Follow = FollowStrip::Nav2FollowPath;
using Navigate = FollowStrip::Nav2Navigate;

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

constexpr double kStep = 0.05;
// A controller failure (the field case): not a planner refusal of the start.
constexpr uint16_t kControllerFailed =
    nav2_msgs::action::FollowPath::Result::FAILED_TO_MAKE_PROGRESS;
constexpr uint16_t kStartOccupied = nav2_msgs::action::ComputePathToPose::Result::START_OCCUPIED;

nav_msgs::msg::Path straightUnit(double x0, double x1, double y)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int n = static_cast<int>(std::round((x1 - x0) / kStep));
  for (int i = 0; i <= n; ++i)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x0 + kStep * i;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

/// Fake Nav2: accepts every goal, records it, and lets the test end it.
template <typename ActionT>
class FakeActionServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

  FakeActionServer(const rclcpp::Node::SharedPtr& node, const std::string& name)
  {
    server_ = rclcpp_action::create_server<ActionT>(
        node,
        name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionT::Goal>)
        {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<Handle>)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<Handle> handle)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          handles_.push_back(handle);
        });
  }

  std::size_t goalCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.size();
  }

  std::shared_ptr<const typename ActionT::Goal> goal(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.at(i)->get_goal();
  }

  bool isCanceling(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return handles_.at(i)->is_canceling();
  }

  void finishCanceled(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->canceled(std::make_shared<typename ActionT::Result>());
  }

  void succeed(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->succeed(std::make_shared<typename ActionT::Result>());
  }

  void abort(std::size_t i, uint16_t error_code)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    result->error_code = error_code;
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->abort(result);
  }

private:
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> handles_;
};

}  // namespace

class TransitNonBlockingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_transit_non_blocking");
    ctx->helper_node = rclcpp::Node::make_shared("test_transit_non_blocking_helper");
    ctx->tf_buffer = std::make_shared<tf2_ros::Buffer>(ctx->node->get_clock());
    ctx->current_area = 0;
    setRobot(0.0, 0.0);

    server_node = rclcpp::Node::make_shared("fake_nav2");
    follow = std::make_unique<FakeActionServer<Follow>>(server_node, "/follow_path");
    navigate = std::make_unique<FakeActionServer<Navigate>>(server_node, "/navigate_to_pose");

    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(ctx->node);
    executor->add_node(server_node);
    spinner = std::thread(
        [this]()
        {
          executor->spin();
        });

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<FollowStrip>("FollowStrip");
    factory.registerNodeType<TransitToStrip>("TransitToStrip");
  }

  void TearDown() override
  {
    tree.reset();
    executor->cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    follow.reset();
    navigate.reset();
  }

  void setRobot(double x, double y)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.w = 1.0;
    ctx->tf_buffer->setTransform(tf, "test", /*is_static=*/true);
  }

  /// What PlanCoverageArea leaves on the context: the sub-paths, their
  /// concatenation, and a transit goal at the start of the first one.
  void planArea(const std::vector<nav_msgs::msg::Path>& units)
  {
    ctx->current_strip_subpaths = units;
    ctx->current_strip_path = units.front();
    ctx->current_transit_goal = units.front().poses.front();
    ctx->current_transit_goal.header.frame_id = "map";
  }

  void build(const std::string& body)
  {
    tree = std::make_unique<BT::Tree>(
        factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + body +
                                       "</BehaviorTree></root>",
                                   blackboard));
  }

  /// main_tree.xml's shape: a soft TransitToStrip, then FollowStrip.
  void buildTransitThenFollow()
  {
    build(
        "<Sequence><Fallback><TransitToStrip/><AlwaysSuccess/></Fallback>"
        "<FollowStrip/></Sequence>");
  }

  /// Tick at 20 Hz until `done` or the timeout. Returns the last tree status.
  BT::NodeStatus tickUntil(const std::function<bool()>& done, double timeout_s)
  {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (std::chrono::steady_clock::now() < deadline)
    {
      status = tree->tickOnce();
      if (status != BT::NodeStatus::RUNNING || done())
      {
        return status;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return status;
  }

  std::shared_ptr<BTContext> ctx;
  rclcpp::Node::SharedPtr server_node;
  std::unique_ptr<FakeActionServer<Follow>> follow;
  std::unique_ptr<FakeActionServer<Navigate>> navigate;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
};

TEST(SameTransitTarget, OnlyNearbyGoalsAreTheSameDestination)
{
  EXPECT_TRUE(sameTransitTarget(1.0, 2.0, 1.0, 2.0));
  EXPECT_TRUE(sameTransitTarget(1.0, 2.0, 1.29, 2.0));
  EXPECT_FALSE(sameTransitTarget(1.0, 2.0, 1.31, 2.0));
  EXPECT_FALSE(sameTransitTarget(1.0, 2.0, 1.25, 2.25));
}

TEST_F(TransitNonBlockingTest, AFailedTransitIsNotRepeatedTheMowMovesOnToTheNextUnit)
{
  // Arrange: unit A starts where TransitToStrip is sent; unit B is elsewhere.
  planArea({straightUnit(5.0, 8.0, 0.0), straightUnit(5.0, 8.0, 3.0)});
  buildTransitThenFollow();
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  EXPECT_NEAR(navigate->goal(0)->pose.pose.position.x, 5.0, 1e-6);

  // Act: the controller cannot follow the path to A's start (field case).
  navigate->abort(0, kControllerFailed);

  // Assert: the next transit goes to B's start — A's transit is not sent
  // again, and nothing is mowed blade-on without a transit.
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 2;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  const auto next = navigate->goal(1)->pose.pose.position;
  EXPECT_NEAR(next.x, 5.0, 1e-6);
  EXPECT_NEAR(next.y, 3.0, 1e-6) << "FollowStrip repeated the transit TransitToStrip just failed";
  EXPECT_EQ(follow->goalCount(), 0u);
  EXPECT_FALSE(ctx->transit_to_strip_failed_at.has_value()) << "the failure applies once";
}

TEST_F(TransitNonBlockingTest, AStartOccupiedRefusalIsStillLeftToFollowStrip)
{
  // Arrange
  planArea({straightUnit(5.0, 8.0, 0.0), straightUnit(5.0, 8.0, 3.0)});
  buildTransitThenFollow();
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  // Act: the planner refuses the ROBOT'S pose.
  navigate->abort(0, kStartOccupied);

  // Assert: FollowStrip sends its own transit to A — its START_OCCUPIED
  // refusal is what arms the start-pose escape (issue #487).
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 2;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  EXPECT_NEAR(navigate->goal(1)->pose.pose.position.y, 0.0, 1e-6);
  EXPECT_FALSE(ctx->transit_to_strip_failed_at.has_value());
}

TEST_F(TransitNonBlockingTest, AFailureElsewhereDoesNotSkipTheFirstUnit)
{
  // Arrange: a failure recorded for a different destination.
  planArea({straightUnit(5.0, 8.0, 0.0), straightUnit(5.0, 8.0, 3.0)});
  geometry_msgs::msg::Point elsewhere;
  elsewhere.x = -4.0;
  elsewhere.y = 2.0;
  ctx->transit_to_strip_failed_at = elsewhere;
  build("<FollowStrip/>");

  // Act
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  // Assert
  EXPECT_NEAR(navigate->goal(0)->pose.pose.position.y, 0.0, 1e-6);
  EXPECT_FALSE(ctx->transit_to_strip_failed_at.has_value());
}

TEST_F(TransitNonBlockingTest, TheWatchdogCancelsATransitThatIsNotGoingAnywhere)
{
  // Arrange: a transit that never finishes on its own.
  planArea({straightUnit(5.0, 8.0, 0.0)});
  build("<TransitToStrip timeout_sec=\"1.0\"/>");
  const auto start = std::chrono::steady_clock::now();

  // Act
  tickUntil(
      [&]()
      {
        return navigate->goalCount() == 1 && navigate->isCanceling(0);
      },
      10.0);

  // Assert: cancelled at the bound, then the node fails and records where.
  ASSERT_EQ(navigate->goalCount(), 1u);
  ASSERT_TRUE(navigate->isCanceling(0)) << "the stuck transit was never cancelled";
  const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GE(waited, 1.0) << "cancelled before its bound";
  navigate->finishCanceled(0);
  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                10.0),
            BT::NodeStatus::FAILURE);
  ASSERT_TRUE(ctx->transit_to_strip_failed_at.has_value());
  EXPECT_NEAR(ctx->transit_to_strip_failed_at->x, 5.0, 1e-6);
  EXPECT_NEAR(ctx->transit_to_strip_failed_at->y, 0.0, 1e-6);
}

TEST_F(TransitNonBlockingTest, ADerivedBoundDoesNotCutANormalTransitShort)
{
  // Arrange: 5 m away → bound transitDeadlineSec(5) = 65 s.
  planArea({straightUnit(5.0, 8.0, 0.0)});
  build("<TransitToStrip/>");
  tickUntil(
      [&]()
      {
        return navigate->goalCount() == 1;
      },
      10.0);
  ASSERT_EQ(navigate->goalCount(), 1u);

  // Act: a couple of seconds of normal driving, then arrival.
  tickUntil(
      []()
      {
        return false;
      },
      2.0);
  EXPECT_FALSE(navigate->isCanceling(0));
  navigate->succeed(0);

  // Assert
  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                10.0),
            BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->transit_to_strip_failed_at.has_value());
}
