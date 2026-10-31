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

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_behavior/detour_costmap_snapshot.hpp"
#include "mowgli_interfaces/ftc_abort_reason.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <gtest/gtest.h>

namespace
{

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};
::testing::Environment* const environment =
    ::testing::AddGlobalTestEnvironment(new RosEnvironment());

template <typename Action>
class ActionServer
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  ActionServer(const rclcpp::Node::SharedPtr& node, const std::string& name)
  {
    server_ = rclcpp_action::create_server<Action>(
        node,
        name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal>)
        {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<Handle>)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Handle> handle)
        {
          std::lock_guard lock(mutex_);
          handles_.push_back(std::move(handle));
        });
  }
  std::size_t count()
  {
    std::lock_guard lock(mutex_);
    return handles_.size();
  }
  std::shared_ptr<Handle> handle(std::size_t index)
  {
    std::lock_guard lock(mutex_);
    return handles_.at(index);
  }

private:
  typename rclcpp_action::Server<Action>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::shared_ptr<Handle>> handles_;
};

nav2_msgs::msg::Costmap clearMap()
{
  nav2_msgs::msg::Costmap map;
  map.header.frame_id = "map";
  map.metadata.resolution = 0.1;
  map.metadata.size_x = 120;
  map.metadata.size_y = 40;
  map.metadata.origin.position.x = -1.0;
  map.metadata.origin.position.y = -2.0;
  map.metadata.origin.orientation.w = 1.0;
  map.data.resize(4800, 0);
  return map;
}

}  // namespace

class FollowStripDetourCostmap : public ::testing::Test
{
protected:
  using Follow = mowgli_behavior::FollowStrip::Nav2FollowPath;
  using Navigate = mowgli_behavior::FollowStrip::Nav2Navigate;
  using GetCostmap = nav2_msgs::srv::GetCostmap;

  void SetUp() override
  {
    ctx_ = std::make_shared<mowgli_behavior::BTContext>();
    ctx_->node = rclcpp::Node::make_shared("detour_costmap_behavior");
    ctx_->helper_node = rclcpp::Node::make_shared("detour_costmap_helper");
    ctx_->tf_buffer = std::make_shared<tf2_ros::Buffer>(ctx_->node->get_clock());
    ctx_->current_area = 0;
    setRobot(0.0);
    server_ = rclcpp::Node::make_shared("detour_costmap_servers");
    follow_ = std::make_unique<ActionServer<Follow>>(server_, "/follow_path");
    navigate_ = std::make_unique<ActionServer<Navigate>>(server_, "/navigate_to_pose");
    blade_ = server_->create_service<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control",
        [this](std::shared_ptr<mowgli_interfaces::srv::MowerControl::Request> request,
               std::shared_ptr<mowgli_interfaces::srv::MowerControl::Response> response)
        {
          blade_enabled_.store(request->mow_enabled);
          response->success = true;
        });
    costmap_group_ = server_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    costmap_ = server_->create_service<GetCostmap>(
        "/global_costmap/get_costmap",
        [this](std::shared_ptr<GetCostmap::Request>, std::shared_ptr<GetCostmap::Response> response)
        {
          std::unique_lock lock(map_mutex_);
          response->map = current_map_;
          ++requests_;
          map_cv_.wait(lock,
                       [this]
                       {
                         return !defer_reply_;
                       });
        },
        rclcpp::ServicesQoS(),
        costmap_group_);
    latched_ = server_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", rclcpp::QoS(1).reliable().transient_local());
    executor_ =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    executor_->add_node(ctx_->node);
    executor_->add_node(server_);
    spinner_ = std::thread(
        [this]
        {
          executor_->spin();
        });
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", ctx_);
    factory_.registerNodeType<mowgli_behavior::FollowStrip>("FollowStrip");
  }
  void TearDown() override
  {
    releaseReplies();
    if (tree_)
      tree_->haltTree();
    tree_.reset();
    executor_->cancel();
    spinner_.join();
  }
  void setRobot(double x)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x;
    tf.transform.rotation.w = 1.0;
    ctx_->tf_buffer->setTransform(tf, "test", true);
  }
  void start(bool reverse_path = false)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 200; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = i * (reverse_path ? -0.05 : 0.05);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    ctx_->current_strip_subpaths = {path};
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><FollowStrip/></BehaviorTree></root>",
        blackboard_));
    nav_msgs::msg::OccupancyGrid old;
    old.header.frame_id = "map";
    old.info.resolution = 0.1;
    old.info.width = 120;
    old.info.height = 40;
    old.info.origin = current_map_.metadata.origin;
    old.data.resize(4800, 0);
    latched_->publish(old);
  }
  BT::NodeStatus pump(const std::function<bool()>& done, double seconds = 5.0)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    do
    {
      {
        std::lock_guard lock(ctx_->context_mutex);
        ctx_->last_scan_time = std::chrono::steady_clock::now();
      }
      status = tree_->tickOnce();
      if (done() || status != BT::NodeStatus::RUNNING)
        return status;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return status;
  }
  void obstacleAbort(std::size_t index = 0)
  {
    auto result = std::make_shared<Follow::Result>();
    result->error_code = 100;
    result->error_msg = "FTC_OBSTACLE_ABORT: captured response sweep blocked";
    follow_->handle(index)->abort(result);
  }
  void markObstacle()
  {
    std::lock_guard lock(map_mutex_);
    // Changed after the retained all-clear full publication. This target is
    // therefore blocked in the current raw map, but clear in the old cache.
    for (unsigned y = 19; y <= 20; ++y)
      for (unsigned x = 17; x <= 22; ++x)
        current_map_.data[y * 120 + x] = 254;
  }
  void releaseReplies()
  {
    {
      std::lock_guard lock(map_mutex_);
      defer_reply_ = false;
    }
    map_cv_.notify_all();
  }

  void expectPlannerEligibleResume(uint8_t raw_cost)
  {
    start();
    pump(
        [this]
        {
          return follow_->count() == 1;
        });
    ASSERT_EQ(follow_->count(), 1u);
    {
      std::lock_guard lock(map_mutex_);
      for (unsigned y = 19; y <= 20; ++y)
        for (unsigned x = 17; x <= 22; ++x)
          current_map_.data[y * 120 + x] = raw_cost;
    }
    obstacleAbort();
    pump(
        [this]
        {
          return navigate_->count() == 1;
        });
    ASSERT_EQ(navigate_->count(), 1u);
    const auto& target = navigate_->handle(0)->get_goal()->pose.pose.position;
    // The producer uses double resolution; GetCostmap transports float32.
    // At (0.8,0), these deliberately select (18,20) and (17,19).
    nav2_costmap_2d::Costmap2D actual(120, 40, 0.1, -1.0, -2.0, 0);
    std::copy(current_map_.data.begin(), current_map_.data.end(), actual.getCharMap());
    unsigned x, y;
    ASSERT_TRUE(actual.worldToMap(target.x, target.y, x, y));
    EXPECT_LT(actual.getCost(x, y), 253);
    EXPECT_GE(target.x, 1.3);
    EXPECT_LE(target.x, 8.0);
    EXPECT_FALSE(blade_enabled_.load());
    EXPECT_EQ(requests_.load(), 1u);
    EXPECT_TRUE(ctx_->completed_areas.empty());
  }

  std::shared_ptr<mowgli_behavior::BTContext> ctx_;
  rclcpp::Node::SharedPtr server_;
  std::unique_ptr<ActionServer<Follow>> follow_;
  std::unique_ptr<ActionServer<Navigate>> navigate_;
  rclcpp::Service<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_;
  rclcpp::Service<GetCostmap>::SharedPtr costmap_;
  rclcpp::CallbackGroup::SharedPtr costmap_group_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr latched_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
  std::mutex map_mutex_;
  std::condition_variable map_cv_;
  nav2_msgs::msg::Costmap current_map_ = clearMap();
  bool defer_reply_ = false;
  std::atomic<unsigned> requests_{0};
  std::atomic<bool> blade_enabled_{false};
};

TEST_F(FollowStripDetourCostmap, FreshLethalSnapshotExcludesRetainedClearTarget)
{
  start();
  ASSERT_EQ(pump(
                [this]
                {
                  return follow_->count() == 1;
                }),
            BT::NodeStatus::RUNNING);
  ASSERT_EQ(follow_->count(), 1u);
  markObstacle();
  obstacleAbort();
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  EXPECT_GE(navigate_->handle(0)->get_goal()->pose.pose.position.x, 1.5);
  EXPECT_FALSE(blade_enabled_.load());
  EXPECT_EQ(requests_.load(), 1u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
}

TEST_F(FollowStripDetourCostmap, FreshClearSnapshotRestoresBoundedResumeAndCoverage)
{
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  const double target = navigate_->handle(0)->get_goal()->pose.pose.position.x;
  EXPECT_NEAR(target, 0.8, 0.051);
  setRobot(target);
  navigate_->handle(0)->succeed(std::make_shared<Navigate::Result>());
  pump(
      [this]
      {
        return follow_->count() == 2;
      });
  ASSERT_EQ(follow_->count(), 2u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  follow_->handle(1)->succeed(std::make_shared<Follow::Result>());
  EXPECT_EQ(pump(
                []
                {
                  return false;
                }),
            BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx_->completed_areas.count(0), 1u);
}

TEST_F(FollowStripDetourCostmap, InscribedTargetCenterIsSkippedBeforeTransitDispatch)
{
  expectPlannerEligibleResume(253);
}

TEST_F(FollowStripDetourCostmap, UnknownTargetCenterIsSkippedBeforeTransitDispatch)
{
  expectPlannerEligibleResume(255);
}

TEST_F(FollowStripDetourCostmap, InvalidFreshResponseNeverUsesRetainedClearGrid)
{
  current_map_.header.frame_id = "odom";
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  EXPECT_EQ(pump(
                []
                {
                  return false;
                }),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  EXPECT_EQ(ctx_->area_resume_pose_index[0], 0u);
  EXPECT_FALSE(blade_enabled_.load());
}

class InvalidDetourSnapshot : public FollowStripDetourCostmap,
                              public ::testing::WithParamInterface<int>
{
};

TEST_P(InvalidDetourSnapshot, MalformedFullSnapshotCannotDispatchOrAdvanceProgress)
{
  switch (GetParam())
  {
    case 0:
      current_map_.data.pop_back();
      break;
    case 1:
      current_map_.metadata.size_x = 0;
      break;
    case 2:
      current_map_.metadata.size_y = std::numeric_limits<uint32_t>::max();
      break;
    case 3:
      current_map_.metadata.resolution = std::numeric_limits<double>::infinity();
      break;
    case 4:
      current_map_.metadata.origin.orientation.w = 0.0;
      break;
    case 5:
      current_map_.metadata.origin.position.x = std::numeric_limits<double>::max();
      break;
  }
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  EXPECT_EQ(pump(
                []
                {
                  return false;
                }),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  EXPECT_EQ(ctx_->area_resume_pose_index[0], 0u);
  EXPECT_FALSE(blade_enabled_.load());
}

INSTANTIATE_TEST_SUITE_P(InvalidMetadata, InvalidDetourSnapshot, ::testing::Range(0, 6));

TEST_F(FollowStripDetourCostmap, EachDetourAcquiresNewSnapshotAfterObstacleClears)
{
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  markObstacle();
  obstacleAbort();
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  const double first_target = navigate_->handle(0)->get_goal()->pose.pose.position.x;
  setRobot(first_target);
  navigate_->handle(0)->succeed(std::make_shared<Navigate::Result>());
  pump(
      [this]
      {
        return follow_->count() == 2;
      });
  ASSERT_EQ(follow_->count(), 2u);
  {
    std::lock_guard lock(map_mutex_);
    current_map_ = clearMap();
  }
  obstacleAbort(1);
  pump(
      [this]
      {
        return navigate_->count() == 2;
      });
  ASSERT_EQ(navigate_->count(), 2u);
  EXPECT_EQ(requests_.load(), 2u);
  EXPECT_NEAR(navigate_->handle(1)->get_goal()->pose.pose.position.x, first_target + 0.8, 0.051);
  EXPECT_FALSE(blade_enabled_.load());
  EXPECT_TRUE(ctx_->completed_areas.empty());
}

TEST_F(FollowStripDetourCostmap, MissingServiceFailsBoundedlyWithoutCursorJump)
{
  costmap_.reset();
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_EQ(pump(
                []
                {
                  return false;
                },
                4.0),
            BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(), 4.0);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_EQ(ctx_->area_resume_pose_index[0], 0u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
}

TEST_F(FollowStripDetourCostmap, ReciprocalRoundingAtCellIndexLimitCannotDispatch)
{
  // Exact float32 resolution: division remains below 2^63, while the reciprocal
  // multiplication used by footprintClear rounds the positive bound to 2^63.
  current_map_.metadata.resolution = 1.422032e-12f;
  current_map_.metadata.origin.position.x = -13115929.649999999;
  const double bound = 0.35 - current_map_.metadata.origin.position.x;
  const double resolution = current_map_.metadata.resolution;
  ASSERT_LT(bound / resolution, std::ldexp(1.0, 63));
  ASSERT_EQ(bound * (1.0 / resolution), std::ldexp(1.0, 63));
  start(/*reverse_path=*/true);
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  EXPECT_EQ(pump(
                []
                {
                  return false;
                }),
            BT::NodeStatus::FAILURE);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_EQ(ctx_->area_resume_pose_index[0], 0u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  EXPECT_FALSE(blade_enabled_.load());
}

TEST_F(FollowStripDetourCostmap, DelayedReplyCannotDispatchAfterHaltOrNewPass)
{
  {
    std::lock_guard lock(map_mutex_);
    defer_reply_ = true;
  }
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  pump(
      [this]
      {
        return requests_.load() == 1 && !blade_enabled_.load();
      });
  ASSERT_EQ(requests_.load(), 1u);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_FALSE(blade_enabled_.load());
  tree_->haltTree();
  // A new pass owns a new action episode, not the old pending response.
  pump(
      [this]
      {
        return follow_->count() == 2;
      });
  ASSERT_EQ(follow_->count(), 2u);
  releaseReplies();
  pump(
      []
      {
        return false;
      },
      0.2);
  EXPECT_EQ(navigate_->count(), 0u);
  markObstacle();
  obstacleAbort(1);
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  EXPECT_EQ(requests_.load(), 2u);
  EXPECT_GE(navigate_->handle(0)->get_goal()->pose.pose.position.x, 1.5);
}

TEST_F(FollowStripDetourCostmap, DelayedReplyAfterTimeoutCannotDispatchOrAdvanceCursor)
{
  {
    std::lock_guard lock(map_mutex_);
    defer_reply_ = true;
  }
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  pump(
      [this]
      {
        return requests_.load() == 1 && !blade_enabled_.load();
      });
  ASSERT_EQ(requests_.load(), 1u);
  EXPECT_FALSE(blade_enabled_.load());
  EXPECT_EQ(pump(
                []
                {
                  return false;
                },
                4.0),
            BT::NodeStatus::FAILURE);
  releaseReplies();
  // Only ROS callbacks run after the timed-out episode. An old service reply
  // must not dispatch by itself, even while the owning FollowStrip still exists.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_EQ(ctx_->area_resume_pose_index[0], 0u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  EXPECT_FALSE(blade_enabled_.load());
}

TEST_F(FollowStripDetourCostmap, FailedDetourTransitWaitsForItsOwnFreshSnapshot)
{
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  markObstacle();
  obstacleAbort();
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  {
    std::lock_guard lock(map_mutex_);
    current_map_ = clearMap();
    // The next prospective target is now blocked. Keep the response pending
    // to exercise the transit-failure owner state across multiple BT ticks.
    for (unsigned y = 19; y <= 20; ++y)
      for (unsigned x = 32; x <= 36; ++x)
        current_map_.data[y * 120 + x] = 254;
    defer_reply_ = true;
  }
  auto result = std::make_shared<Navigate::Result>();
  result->error_code = 206;
  result->error_msg = "Transit goal occupied by an obstacle";
  navigate_->handle(0)->abort(result);
  pump(
      [this]
      {
        return requests_.load() == 2 && !blade_enabled_.load();
      });
  ASSERT_EQ(requests_.load(), 2u);
  EXPECT_EQ(navigate_->count(), 1u);
  EXPECT_FALSE(blade_enabled_.load());
  releaseReplies();
  pump(
      [this]
      {
        return navigate_->count() == 2;
      });
  ASSERT_EQ(navigate_->count(), 2u);
  EXPECT_GE(navigate_->handle(1)->get_goal()->pose.pose.position.x, 2.9);
  EXPECT_TRUE(ctx_->completed_areas.empty());
}

TEST_F(FollowStripDetourCostmap, FleetYieldDiscardsPendingSnapshot)
{
  {
    std::lock_guard lock(map_mutex_);
    defer_reply_ = true;
  }
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  obstacleAbort();
  pump(
      [this]
      {
        return requests_.load() == 1 && !blade_enabled_.load();
      });
  ASSERT_EQ(requests_.load(), 1u);
  {
    std::lock_guard lock(ctx_->context_mutex);
    ctx_->fleet_excluded_areas.insert(0);
  }
  EXPECT_EQ(pump(
                []
                {
                  return false;
                }),
            BT::NodeStatus::SUCCESS);
  releaseReplies();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_EQ(ctx_->fleet_yielded_areas.count(0), 1u);
  EXPECT_TRUE(ctx_->completed_areas.empty());
  EXPECT_FALSE(blade_enabled_.load());
}

TEST_F(FollowStripDetourCostmap, FreshScanCannotReenableBladeWhileSnapshotPending)
{
  {
    std::lock_guard lock(map_mutex_);
    defer_reply_ = true;
  }
  start();
  pump(
      [this]
      {
        return follow_->count() == 1;
      });
  ASSERT_EQ(follow_->count(), 1u);
  {
    std::lock_guard lock(ctx_->context_mutex);
    ctx_->last_scan_time = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  }
  ASSERT_EQ(tree_->tickOnce(), BT::NodeStatus::RUNNING);
  obstacleAbort();
  pump(
      [this]
      {
        return requests_.load() == 1 && !blade_enabled_.load();
      });
  ASSERT_EQ(requests_.load(), 1u);
  // pump supplies continuous fresh scans for longer than the normal 0.5 s
  // scan-resume window; the pending recovery decision still owns blade OFF.
  EXPECT_EQ(pump(
                []
                {
                  return false;
                },
                1.0),
            BT::NodeStatus::RUNNING);
  EXPECT_EQ(navigate_->count(), 0u);
  EXPECT_FALSE(blade_enabled_.load());
  releaseReplies();
  pump(
      [this]
      {
        return navigate_->count() == 1;
      });
  ASSERT_EQ(navigate_->count(), 1u);
  EXPECT_FALSE(blade_enabled_.load());
}

TEST(DetourCostmapSnapshot, PreservesRawOccupancyConversionAndRejectsMalformedMetadata)
{
  auto map = clearMap();
  map.data[0] = 0;
  map.data[1] = 1;
  map.data[2] = 252;
  map.data[3] = 253;
  map.data[4] = 254;
  map.data[5] = 255;
  const auto valid = mowgli_behavior::detourCostmapSnapshot(map);
  ASSERT_TRUE(valid.has_value());
  EXPECT_EQ(std::vector<int8_t>(valid->data.begin(), valid->data.begin() + 6),
            (std::vector<int8_t>{0, 1, 98, 99, 100, -1}));
  const auto reject = [](auto change)
  {
    auto invalid = clearMap();
    change(invalid);
    EXPECT_FALSE(mowgli_behavior::detourCostmapSnapshot(invalid).has_value());
  };
  reject(
      [](auto& m)
      {
        m.data.pop_back();
      });
  reject(
      [](auto& m)
      {
        m.metadata.size_x = 0;
      });
  reject(
      [](auto& m)
      {
        m.metadata.size_x = std::numeric_limits<uint32_t>::max();
      });
  reject(
      [](auto& m)
      {
        m.metadata.resolution = std::numeric_limits<double>::infinity();
      });
  reject(
      [](auto& m)
      {
        m.metadata.resolution = 0.0;
      });
  reject(
      [](auto& m)
      {
        m.metadata.origin.position.x = std::numeric_limits<double>::quiet_NaN();
      });
  reject(
      [](auto& m)
      {
        m.metadata.origin.orientation.w = 0.0;
      });
  reject(
      [](auto& m)
      {
        m.metadata.origin.orientation.z = 0.1;
      });
  reject(
      [](auto& m)
      {
        m.header.frame_id = "odom";
      });
}

TEST(DetourTransitTargetCenter, FloatBoundaryIncludesTheRealCostmapCell)
{
  auto map = clearMap();
  nav2_costmap_2d::Costmap2D actual(120, 40, 0.1, -1.0, -2.0, 0);
  unsigned x, y;
  ASSERT_TRUE(actual.worldToMap(0.8, 0.0, x, y));
  ASSERT_EQ(x, 18u);
  ASSERT_EQ(y, 20u);
  for (const uint8_t cost : {253, 254, 255})
  {
    map.data[y * 120 + x] = cost;
    const auto snapshot = mowgli_behavior::detourCostmapSnapshot(map);
    ASSERT_TRUE(snapshot);
    EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.8, 0.0));
  }
  map.data[y * 120 + x] = 252;
  auto snapshot = mowgli_behavior::detourCostmapSnapshot(map);
  ASSERT_TRUE(snapshot);
  EXPECT_TRUE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.8, 0.0));
  // A different consistent producer resolution selects the neighbouring cell.
  // Rejecting that ambiguous boundary is deliberate even when actual is clear.
  map.data[19 * 120 + 17] = 253;
  snapshot = mowgli_behavior::detourCostmapSnapshot(map);
  ASSERT_TRUE(snapshot);
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.8, 0.0));
  // Body-only inflation must not become a centre-eligibility rejection.
  EXPECT_TRUE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.85, 0.05));
}

TEST(DetourTransitTargetCenter, ActualCostmapCellsAtTranslatedGridLinesCannotBeMissed)
{
  for (const double resolution : {0.05, 0.08, 0.1, 0.125})
    for (const auto origin :
         {std::pair{-1.0, -2.0}, std::pair{0.0, 0.0}, std::pair{100000.0, -100000.0}})
    {
      auto map = clearMap();
      map.metadata.resolution = resolution;
      map.metadata.origin.position.x = origin.first;
      map.metadata.origin.position.y = origin.second;
      nav2_costmap_2d::Costmap2D actual(120, 40, resolution, origin.first, origin.second, 0);
      const double bx = origin.first + 18 * resolution;
      const double by = origin.second + 20 * resolution;
      for (double px : {std::nextafter(bx, -INFINITY), bx, std::nextafter(bx, INFINITY)})
        for (double py : {std::nextafter(by, -INFINITY), by, std::nextafter(by, INFINITY)})
        {
          unsigned x, y;
          ASSERT_TRUE(actual.worldToMap(px, py, x, y));
          for (const uint8_t cost : {253, 254, 255})
          {
            map.data[y * 120 + x] = cost;
            const auto snapshot = mowgli_behavior::detourCostmapSnapshot(map);
            ASSERT_TRUE(snapshot);
            EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, px, py));
            map.data[y * 120 + x] = 0;
          }
          const auto clear = mowgli_behavior::detourCostmapSnapshot(map);
          ASSERT_TRUE(clear);
          EXPECT_TRUE(mowgli_behavior::detourTransitTargetCenterClear(*clear, px, py));
        }
    }
}

TEST(DetourTransitTargetCenter, OriginEdgesAndInvalidArithmeticFailSafely)
{
  auto snapshot = mowgli_behavior::detourCostmapSnapshot(clearMap());
  ASSERT_TRUE(snapshot);
  EXPECT_TRUE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, -1.0, -2.0));
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, -1.00001, 0.0));
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 11.0, 0.0));
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, INFINITY, 0.0));
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, NAN, 0.0));
  snapshot->resolution = 0.1;  // Not a float32 wire value.
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.8, 0.0));
  snapshot->resolution = std::numeric_limits<float>::max();
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.8, 0.0));
  snapshot->resolution = std::numeric_limits<float>::denorm_min();
  snapshot->origin_x = snapshot->origin_y = 0.0;
  EXPECT_TRUE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot, 0.0, 0.0));
  EXPECT_FALSE(mowgli_behavior::detourTransitTargetCenterClear(*snapshot,
                                                               std::numeric_limits<double>::max(),
                                                               0.0));
}
