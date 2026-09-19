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
 * @file test_follow_strip_dig.cpp
 * @brief Tick-level tests of FollowStrip's reaction to a wheel-slip dig.
 *
 * Field 2026-09-17 (bag part2-1920): after a dig at (-3.23, 11.01) the bridge
 * stopped and reversed correctly, but FTC kept following the ACTIVE coverage
 * path straight back through the hole for 35 s, and the pending keepout
 * map_server had stamped around the robot then refused every transit with
 * START_OCCUPIED until the area was given up. The keepout is gone (a dig is an
 * operator proposal only); FollowStrip now
 *   - cancels the coverage goal the moment a dig is reported,
 *   - waits for the bridge's bounded reverse to settle,
 *   - resumes the SAME unit at the first pose past the dig skip zone, reached
 *     by the existing blade-off transit,
 *   - and skips the zone of every recorded dig on every later unit.
 *
 * Real action servers (fake Nav2) + a real TF buffer; FollowStrip is ticked
 * exactly as the tree ticks it.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_behavior/dig_skip.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "tf2_ros/buffer.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::DigPoint;
using mowgli_behavior::FollowStrip;
using mowgli_behavior::insideDigZone;
using mowgli_behavior::recordDigPoint;
using Follow = FollowStrip::Nav2FollowPath;
using Navigate = FollowStrip::Nav2Navigate;
using FollowHandle = rclcpp_action::ServerGoalHandle<Follow>;
using NavigateHandle = rclcpp_action::ServerGoalHandle<Navigate>;

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

constexpr double kRadius = 0.60;
constexpr double kStep = 0.05;

nav_msgs::msg::Path straightUnit(double x0, double x1)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const int n = static_cast<int>(std::round((x1 - x0) / kStep));
  for (int i = 0; i <= n; ++i)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x0 + kStep * i;
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

  void abort(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handles_.at(i)->abort(std::make_shared<typename ActionT::Result>());
  }

private:
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> handles_;
};

}  // namespace

class FollowStripDigTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_follow_strip_dig");
    ctx->helper_node = rclcpp::Node::make_shared("test_follow_strip_dig_helper");
    ctx->tf_buffer = std::make_shared<tf2_ros::Buffer>(ctx->node->get_clock());
    ctx->current_area = 0;
    ctx->dig_skip_radius_m = kRadius;
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
    factory.registerNodeType<mowgli_behavior::EndSession>("EndSession");
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

  /// What behavior_tree_node's /hardware_bridge/dig_event callback does.
  void reportDig(double x, double y)
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->session_dig_points = recordDigPoint(ctx->session_dig_points, DigPoint{x, y});
    ++ctx->dig_event_count;
  }

  void startFollowStrip(const std::vector<nav_msgs::msg::Path>& units)
  {
    ctx->current_strip_subpaths = units;
    tree = std::make_unique<BT::Tree>(factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><FollowStrip/></BehaviorTree></root>",
        blackboard));
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

  static bool pathTouchesZone(const nav_msgs::msg::Path& path, const DigPoint& dig)
  {
    for (const auto& p : path.poses)
    {
      if (insideDigZone(p.pose.position.x, p.pose.position.y, {dig}, kRadius))
      {
        return true;
      }
    }
    return false;
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

TEST_F(FollowStripDigTest, DigWhileMowingCancelsTheGoalWaitsForTheReverseAndResumesPastTheHole)
{
  // Arrange: one 10 m unit, mowing from its start.
  const DigPoint dig{3.2, 0.0};
  startFollowStrip({straightUnit(0.0, 10.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow->goalCount(), 1u);
  EXPECT_EQ(follow->goal(0)->path.poses.size(), 201u) << "no dig yet: the whole unit is sent";
  setRobot(3.0, 0.0);  // FTC drove 3 m...
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  // Act: ...and the bridge reports a dig just ahead.
  reportDig(dig.x, dig.y);
  const auto dig_time = std::chrono::steady_clock::now();

  // Assert 1: the coverage goal is cancelled — FTC must not keep pushing along
  // the old path through the hole (it did, for 35 s, on 2026-09-17).
  tickUntil(
      [&]()
      {
        return follow->isCanceling(0);
      },
      3.0);
  ASSERT_TRUE(follow->isCanceling(0)) << "the active FollowCoveragePath goal was not cancelled";
  follow->finishCanceled(0);
  EXPECT_EQ(navigate->goalCount(), 0u) << "nothing may be dispatched while the bridge reverses";

  // The bridge reverses the robot ~0.25 m, then it stands still.
  setRobot(2.95, 0.0);

  // Assert 2: only once the robot has settled does a blade-off transit go out,
  // to the first pose PAST the skip zone — never into it, never back to the
  // start of the unit.
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(navigate->goalCount(), 1u) << "the same unit was not resumed";
  const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - dig_time).count();
  EXPECT_GE(waited, 1.0) << "re-dispatched before the bridge's reverse could have settled";
  EXPECT_EQ(follow->goalCount(), 1u) << "no blade-on goal before the transit has succeeded";
  const auto resume = navigate->goal(0)->pose.pose.position;
  EXPECT_FALSE(insideDigZone(resume.x, resume.y, {dig}, kRadius));
  EXPECT_GT(resume.x, dig.x) << "the resume pose must be PAST the hole";
  EXPECT_NEAR(resume.x, dig.x + kRadius, 2 * kStep) << "skip no more than the zone";

  // Assert 3: after the transit, the REST of the same unit is mowed.
  navigate->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 2;
                },
                5.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow->goalCount(), 2u);
  const auto rest = follow->goal(1)->path;
  ASSERT_FALSE(rest.poses.empty());
  EXPECT_NEAR(rest.poses.front().pose.position.x, resume.x, 1e-6);
  EXPECT_NEAR(rest.poses.back().pose.position.x, 10.0, 1e-6);
  EXPECT_FALSE(pathTouchesZone(rest, dig));

  follow->succeed(1);
  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                5.0),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->area_completed_swaths[0].count(0), 1u);
}

TEST_F(FollowStripDigTest, LaterUnitIsCutAtARecordedDigZoneAndContinuedPastIt)
{
  // Arrange: the dig happened earlier in the session (previous ring / swath).
  const DigPoint dig{5.0, 0.0};
  reportDig(dig.x, dig.y);
  startFollowStrip({straightUnit(0.0, 10.0)});

  // Act + Assert 1: FTC only gets the run BEFORE the zone.
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow->goalCount(), 1u);
  const auto before = follow->goal(0)->path;
  EXPECT_NEAR(before.poses.front().pose.position.x, 0.0, 1e-6);
  EXPECT_LT(before.poses.back().pose.position.x, dig.x - kRadius + kStep);
  EXPECT_FALSE(pathTouchesZone(before, dig)) << "a recorded dig zone was sent to the controller";
  EXPECT_EQ(navigate->goalCount(), 0u) << "a dig recorded BEFORE the goal must not cancel it";

  // Assert 2: reaching the cut is NOT the end of the unit — blade-off transit
  // past the zone, then the rest.
  setRobot(4.3, 0.0);
  follow->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                5.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(navigate->goalCount(), 1u);
  EXPECT_TRUE(ctx->area_completed_swaths[0].empty()) << "the unit was booked mowed at the cut";
  const auto resume = navigate->goal(0)->pose.pose.position;
  EXPECT_GT(resume.x, dig.x);
  EXPECT_FALSE(insideDigZone(resume.x, resume.y, {dig}, kRadius));

  setRobot(resume.x, 0.0);
  navigate->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 2;
                },
                5.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow->goalCount(), 2u);
  const auto after = follow->goal(1)->path;
  EXPECT_FALSE(pathTouchesZone(after, dig));
  EXPECT_NEAR(after.poses.back().pose.position.x, 10.0, 1e-6);

  follow->succeed(1);
  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                5.0),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->area_completed_swaths[0].count(0), 1u);
}

TEST_F(FollowStripDigTest, UnitLyingEntirelyInsideADigZoneIsBookedWithoutAnyGoal)
{
  // Arrange: a 1 m unit, dig in its middle: every pose is inside the zone.
  reportDig(0.5, 0.0);
  startFollowStrip({straightUnit(0.0, 1.0)});

  // Act
  const BT::NodeStatus status = tickUntil(
      []()
      {
        return false;
      },
      5.0);

  // Assert
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(follow->goalCount(), 0u) << "the controller was sent into a dig zone";
  EXPECT_EQ(navigate->goalCount(), 0u);
  EXPECT_EQ(ctx->area_completed_swaths[0].count(0), 1u)
      << "an un-mowable unit must not keep the area open forever";
}

// A controller ABORT is not evidence that the final unverified tail was mowed.
// In particular, the 95%-progress goal checker must not make this terminal
// result look like SUCCESS. The existing forward skip leaves a cursor at the
// final pose; resolveResumeLocation replays its small tail on the next pass.
TEST_F(FollowStripDigTest, NearEndAbortPreservesResumeWithoutCompleting)
{
  startFollowStrip({straightUnit(0.0, 10.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  // 9.5 m is 190/200 path intervals (95%). The non-obstacle abort skip reaches
  // the final pose but must still leave the unit and area uncompleted.
  setRobot(9.5, 0.0);
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  follow->abort(0);

  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                5.0),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 200u);
  EXPECT_LT(ctx->coverage_percent, 100.0f);
  EXPECT_TRUE(ctx->area_completed_swaths[0].empty());
  EXPECT_TRUE(ctx->completed_areas.empty());
}

// The contrasting terminal result remains the only ordinary completion path:
// it clears an old resume cursor, records the unit, and retires a one-unit area.
TEST_F(FollowStripDigTest, SuccessClearsResumeAndCompletesArea)
{
  startFollowStrip({straightUnit(0.0, 10.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  ctx->area_resume_pose_index[0] = 42;
  follow->succeed(0);

  EXPECT_EQ(tickUntil(
                []()
                {
                  return false;
                },
                5.0),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->area_resume_pose_index.count(0), 0u);
  EXPECT_EQ(ctx->area_completed_swaths[0], (std::set<std::size_t>{0}));
  EXPECT_EQ(ctx->completed_areas.count(0), 1u);
}

TEST_F(FollowStripDigTest, EndSessionForgetsTheDigPointsButKeepsTheEventCounter)
{
  // Arrange
  reportDig(1.0, 1.0);
  reportDig(4.0, 1.0);
  auto end = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"E\"><EndSession/></BehaviorTree></root>",
      blackboard);

  // Act
  EXPECT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);

  // Assert: skip zones are session state; the counter is an edge detector.
  EXPECT_TRUE(ctx->session_dig_points.empty());
  EXPECT_EQ(ctx->dig_event_count, 2u);
}
