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
 * @brief Tick-level tests of FollowStrip's dig and short scan-dropout reactions.
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
 * The scan case additionally records real mower-control requests: stale scans
 * cut the blade while preserving the active follow goal; fresh scans restore it
 * without a transit or progress mutation.
 *
 * Also: FollowStrip's progress cursor following the coverage controller's
 * rejoin after an FTC turn fallback (ControllerRejoin, strip_progress.hpp).
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
#include "mowgli_behavior/status_snapshot.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
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
using MowerControl = mowgli_interfaces::srv::MowerControl;

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
    blade_service = server_node->create_service<MowerControl>(
        "/hardware_bridge/mower_control",
        [this](const std::shared_ptr<MowerControl::Request> request,
               std::shared_ptr<MowerControl::Response> response)
        {
          std::lock_guard<std::mutex> lock(blade_mutex);
          blade_requests.push_back(*request);
          response->success = true;
        });

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

  /// `yaw` matters: the progress cursor only accepts a path pose oriented with
  /// the robot (strip_progress.hpp, issue #742), so a test driving DOWN a
  /// return swath must say so or the cursor correctly refuses to follow it.
  void setRobot(double x, double y, double yaw = 0.0)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);
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
  rclcpp::Service<MowerControl>::SharedPtr blade_service;
  std::mutex blade_mutex;
  std::vector<MowerControl::Request> blade_requests;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread spinner;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;

  std::size_t bladeRequestCount()
  {
    std::lock_guard<std::mutex> lock(blade_mutex);
    return blade_requests.size();
  }

  MowerControl::Request bladeRequest(std::size_t i)
  {
    std::lock_guard<std::mutex> lock(blade_mutex);
    return blade_requests.at(i);
  }
};

TEST_F(FollowStripDigTest, ShortScanDropoutCutsAndRestoresBladeWithoutReplacingCoverageGoal)
{
  std::mutex status_mutex;
  std::vector<mowgli_interfaces::msg::HighLevelStatus> statuses;
  auto status_subscription =
      server_node->create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
          "/test_follow_strip_dig/high_level_status",
          10,
          [&](const mowgli_interfaces::msg::HighLevelStatus::SharedPtr msg)
          {
            std::lock_guard<std::mutex> lock(status_mutex);
            statuses.push_back(*msg);
          });
  ctx->high_level_status_pub =
      ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>("~/high_level_status",
                                                                           10);
  ctx->last_high_level_status.state =
      mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
  ctx->last_high_level_status.state_name = "MOWING";
  ctx->has_high_level_status = true;

  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (ctx->high_level_status_pub->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(ctx->high_level_status_pub->get_subscription_count(), 0u);

  auto sawStatus = [&](const std::string& sub_state)
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return std::any_of(statuses.begin(),
                       statuses.end(),
                       [&](const auto& status)
                       {
                         return status.sub_state_name == sub_state;
                       });
  };
  auto statusCount = [&]()
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return statuses.size();
  };

  startFollowStrip({straightUnit(0.0, 10.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow->goalCount(), 1u);
  ASSERT_GT(bladeRequestCount(), 0u);
  EXPECT_EQ(bladeRequest(0).mow_enabled, 1u);

  const std::size_t requests_before_pause = bladeRequestCount();
  const float progress_before_pause = ctx->coverage_percent;
  const auto completed_before_pause = ctx->area_completed_swaths;
  const auto resume_before_pause = ctx->area_resume_pose_index;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  }
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return ctx->coverage_scan_paused && bladeRequestCount() > requests_before_pause &&
                         sawStatus("SCAN_PAUSED");
                },
                3.0),
            BT::NodeStatus::RUNNING);
  EXPECT_EQ(bladeRequest(bladeRequestCount() - 1).mow_enabled, 0u);
  EXPECT_EQ(follow->goalCount(), 1u);
  EXPECT_FALSE(follow->isCanceling(0));
  EXPECT_EQ(navigate->goalCount(), 0u);
  EXPECT_FLOAT_EQ(ctx->coverage_percent, progress_before_pause);
  EXPECT_EQ(ctx->area_resume_pose_index, resume_before_pause);
  EXPECT_EQ(ctx->area_completed_swaths, completed_before_pause);

  const std::size_t requests_before_resume = bladeRequestCount();
  const std::size_t statuses_before_resume = statusCount();
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now();
  }
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  if (ctx->coverage_scan_paused || bladeRequestCount() <= requests_before_resume)
                  {
                    return false;
                  }
                  std::lock_guard<std::mutex> lock(status_mutex);
                  return statuses.size() > statuses_before_resume &&
                         statuses.back().sub_state_name.empty();
                },
                3.0),
            BT::NodeStatus::RUNNING);
  EXPECT_EQ(bladeRequest(bladeRequestCount() - 1).mow_enabled, 1u);
  EXPECT_EQ(follow->goalCount(), 1u);
  EXPECT_FALSE(follow->isCanceling(0));
  EXPECT_EQ(navigate->goalCount(), 0u);
  EXPECT_FLOAT_EQ(ctx->coverage_percent, progress_before_pause);
  EXPECT_EQ(ctx->area_resume_pose_index, resume_before_pause);
  EXPECT_EQ(ctx->area_completed_swaths, completed_before_pause);
}

TEST_F(FollowStripDigTest, ScanPauseSurvivesABladeOffTransitToTheNextUnit)
{
  startFollowStrip({straightUnit(0.0, 10.0), straightUnit(10.0, 20.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  }
  const std::size_t requests_before_pause = bladeRequestCount();
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return ctx->coverage_scan_paused && bladeRequestCount() > requests_before_pause;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  const std::size_t first_blade_off = bladeRequestCount() - 1;
  ASSERT_EQ(bladeRequest(first_blade_off).mow_enabled, 0u);

  // Finish unit one, then let scans become fresh only DURING the structural
  // transit. That unobserved interval must not shorten the full fresh-scan
  // window required after the second follow goal is active.
  follow->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now();
  }
  EXPECT_TRUE(ctx->coverage_scan_paused);
  navigate->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 2;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->coverage_scan_paused);
  EXPECT_FALSE(follow->isCanceling(1));
  for (std::size_t i = first_blade_off; i < bladeRequestCount(); ++i)
  {
    EXPECT_EQ(bladeRequest(i).mow_enabled, 0u)
        << "fresh scans observed only during transit must not re-enable the blade";
  }

  const std::size_t requests_before_resume = bladeRequestCount();
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(ctx->coverage_scan_paused);
  EXPECT_EQ(bladeRequestCount(), requests_before_resume);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return !ctx->coverage_scan_paused && bladeRequestCount() > requests_before_resume;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  EXPECT_EQ(bladeRequest(bladeRequestCount() - 1).mow_enabled, 1u);
  EXPECT_EQ(follow->goalCount(), 2u);
}

TEST_F(FollowStripDigTest, StaleScanDuringTransitKeepsBladeOffUntilFreshWindowAfterTransit)
{
  mowgli_interfaces::msg::HighLevelStatus mowing_status;
  mowing_status.state = mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
  mowing_status.state_name = "MOWING";
  std::mutex status_mutex;
  std::vector<mowgli_interfaces::msg::HighLevelStatus> statuses;
  auto status_subscription =
      server_node->create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
          "/test_follow_strip_dig/high_level_status",
          10,
          [&](const mowgli_interfaces::msg::HighLevelStatus::SharedPtr msg)
          {
            std::lock_guard<std::mutex> lock(status_mutex);
            statuses.push_back(*msg);
          });
  ctx->high_level_status_pub =
      ctx->node->create_publisher<mowgli_interfaces::msg::HighLevelStatus>("~/high_level_status",
                                                                           10);
  ctx->last_high_level_status = mowing_status;
  ctx->has_high_level_status = true;
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (ctx->high_level_status_pub->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(ctx->high_level_status_pub->get_subscription_count(), 0u);

  auto waitForStatus = [&](const std::string& sub_state, std::size_t after)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(status_mutex);
        if (statuses.size() > after && statuses.back().sub_state_name == sub_state)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  };

  startFollowStrip({straightUnit(0.0, 10.0), straightUnit(10.0, 20.0)});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);

  // Finish the first unit while scans are fresh, then let the structural
  // blade-off transit begin. The scan stream drops only DURING that transit.
  follow->succeed(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->transiting);
  EXPECT_EQ(mowgli_behavior::withLiveStatusFields(mowing_status, *ctx).sub_state_name, "TRANSIT");
  const std::size_t requests_before_stale_transit = bladeRequestCount();
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  }

  // The transit completion immediately dispatches FollowCoveragePath. That
  // hand-off must sample stale scan data before it can issue mower-on.
  navigate->succeed(0);
  const std::size_t statuses_before_pause = [&]()
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return statuses.size();
  }();
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return ctx->coverage_scan_paused;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  // onRunning snapshots transit state at the start of this completion tick.
  // The scan pause must still win the live status projection immediately.
  EXPECT_TRUE(ctx->transiting);
  EXPECT_EQ(mowgli_behavior::withLiveStatusFields(mowing_status, *ctx).sub_state_name,
            "SCAN_PAUSED");
  EXPECT_TRUE(waitForStatus("SCAN_PAUSED", statuses_before_pause));
  const auto follow_goal_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (follow->goalCount() < 2 && std::chrono::steady_clock::now() < follow_goal_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(follow->goalCount(), 2u);
  for (std::size_t i = requests_before_stale_transit; i < bladeRequestCount(); ++i)
  {
    EXPECT_EQ(bladeRequest(i).mow_enabled, 0u)
        << "a stale scan detected at transit completion must not briefly enable the blade";
  }

  // A newly fresh scan alone is insufficient. The ordinary coverage ticks,
  // rather than the unobserved transit duration, must establish the existing
  // continuous fresh-scan requirement before mower-on.
  const std::size_t requests_before_fresh_window = bladeRequestCount();
  const std::size_t statuses_before_resume = [&]()
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    return statuses.size();
  }();
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    ctx->last_scan_time = std::chrono::steady_clock::now();
  }
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(ctx->coverage_scan_paused);
  EXPECT_EQ(bladeRequestCount(), requests_before_fresh_window);

  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return !ctx->coverage_scan_paused &&
                         bladeRequestCount() > requests_before_fresh_window;
                },
                3.0),
            BT::NodeStatus::RUNNING);
  EXPECT_EQ(bladeRequest(bladeRequestCount() - 1).mow_enabled, 1u);
  EXPECT_EQ(follow->goalCount(), 2u);
  EXPECT_FALSE(ctx->transiting);
  EXPECT_EQ(mowgli_behavior::withLiveStatusFields(mowing_status, *ctx).sub_state_name, "");
  EXPECT_TRUE(waitForStatus("", statuses_before_resume));
}

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
  // FTC drives 3 m... The progress cursor follows the robot a bounded stretch
  // of path per tick (strip_progress.hpp), so drive there rather than teleport:
  // 0.25 m per tick is still ~8x a real 10 Hz tick at mowing speed.
  for (double x = 0.25; x <= 3.0 + 1e-9; x += 0.25)
  {
    setRobot(x, 0.0);
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  }

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
  // The progress cursor only creeps a bounded stretch of path per tick
  // (strip_progress.hpp), so drive there rather than teleport.
  for (double x = 0.25; x <= 9.5 + 1e-9; x += 0.25)
  {
    setRobot(x, 0.0);
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  }
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

// --- Coverage controller rejoin (FTC turn fallback) --------------------------
//
// Not a dig, but the same tick-level harness: FTC's turn fallback improvises a
// blocked U-turn and rejoins the unit on the return swath, then republishes the
// rest of the unit on the goal checker's plan topic. FollowStrip's progress
// cursor, which can only creep 1 m of path per tick and at a U-turn stays on
// the pose abeam on the other swath, must jump to the rejoin — or the live
// percent lags and an abort later on resumes back at the turn.

namespace
{

nav_msgs::msg::Path uTurnUnit(double length, double spacing)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const auto add = [&path](double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw / 2.0);
    p.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(p);
  };
  for (double x = 0.0; x < length - 1e-9; x += kStep)
  {
    add(x, 0.0, 0.0);
  }
  const double r = spacing / 2.0;
  for (int k = 0; k <= 4; ++k)
  {
    const double a = -M_PI / 2.0 + M_PI * k / 4.0;
    add(length + r * std::cos(a), r + r * std::sin(a), a + M_PI / 2.0);
  }
  for (double x = length - kStep; x >= -1e-9; x -= kStep)
  {
    add(x, spacing, M_PI);
  }
  return path;
}

std::size_t nearestPose(const nav_msgs::msg::Path& path, double x, double y)
{
  std::size_t best = 0;
  for (std::size_t i = 0; i < path.poses.size(); ++i)
  {
    const auto& p = path.poses[i].pose.position;
    const auto& b = path.poses[best].pose.position;
    if (std::hypot(p.x - x, p.y - y) < std::hypot(b.x - x, b.y - y))
    {
      best = i;
    }
  }
  return best;
}

}  // namespace

TEST_F(FollowStripDigTest, ControllerRejoinMovesTheProgressCursorPastTheSkippedTurn)
{
  // Arrange: one U-turn unit (swath A 6 m, 0.13 m spacing, swath B back).
  const auto unit = uTurnUnit(6.0, 0.13);
  startFollowStrip({unit});
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return follow->goalCount() == 1;
                },
                10.0),
            BT::NodeStatus::RUNNING);
  for (double x = 0.25; x <= 4.5 + 1e-9; x += 0.25)
  {
    setRobot(x, 0.0);
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  }
  const float before = ctx->coverage_percent;
  const std::size_t rejoin = nearestPose(unit, 4.5, 0.13);
  ASSERT_GT(rejoin, nearestPose(unit, 4.5, 0.0) + 40);

  auto pub = server_node->create_publisher<nav_msgs::msg::Path>(
      "/controller_server/FollowCoveragePath/global_plan", rclcpp::QoS(1).transient_local());
  nav_msgs::msg::Path remainder;
  remainder.header.frame_id = "map";
  remainder.poses.assign(unit.poses.begin() + static_cast<std::ptrdiff_t>(rejoin),
                         unit.poses.end());

  // A message stamped before the goal went out (a latched leftover) is ignored.
  remainder.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  pub->publish(remainder);
  setRobot(4.5, 0.13, M_PI);
  tickUntil(
      []()
      {
        return false;
      },
      0.5);
  EXPECT_NEAR(ctx->coverage_percent, before, 1.0) << "a stale republish moved the cursor";

  // Act: FTC's rejoin republish (stamped now), the robot on swath B.
  remainder.header.stamp = ctx->node->now();
  pub->publish(remainder);
  const float expected =
      100.0f * static_cast<float>(rejoin) / static_cast<float>(unit.poses.size());
  tickUntil(
      [&]()
      {
        return ctx->coverage_percent >= expected - 1.0f;
      },
      3.0);

  // Assert 1: the live percent follows the robot past the skipped turn.
  EXPECT_GE(ctx->coverage_percent, expected - 1.0f);

  // Assert 2: an abort further along B resumes from B, not back at the turn.
  for (double x = 4.25; x >= 3.5 - 1e-9; x -= 0.25)
  {
    setRobot(x, 0.13, M_PI);  // driving DOWN swath B
    ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  }
  follow->abort(0);
  ASSERT_EQ(tickUntil(
                [&]()
                {
                  return navigate->goalCount() == 1 || follow->goalCount() == 2;
                },
                5.0),
            BT::NodeStatus::RUNNING);
  const auto resume = navigate->goalCount() == 1
                          ? navigate->goal(0)->pose.pose.position
                          : follow->goal(1)->path.poses.front().pose.position;
  EXPECT_NEAR(resume.y, 0.13, 1e-6) << "resumed on swath A, back before the skipped turn";
  EXPECT_LT(resume.x, 3.5) << "did not resume past the robot on swath B";
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
