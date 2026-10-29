// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rcl/time.h"
#include <gtest/gtest.h>

namespace mowgli_nav2_plugins
{

static_assert(!std::is_abstract_v<FTCController>);
static_assert(!std::is_abstract_v<PathProgressGoalChecker>);

class PathProgressGoalCheckerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<nav2::LifecycleNode>("lyrical_goal_checker_test");
    checker_.initialize(node_, "coverage", nullptr);
    checker_.query_frame_ = "odom";
    checker_.tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.translation.x = 100.0;
    transform.transform.rotation.w = 1.0;
    checker_.tf_buffer_->setTransform(transform, "test", true);
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";
    for (int i = 0; i < 101; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = 100.0 + 0.1 * i;
      pose.pose.orientation.w = 1.0;
      path->poses.push_back(pose);
    }
    setPath(path);
    goal_.position.x = 10.0;
    goal_.orientation.w = 1.0;
  }

  void advance()
  {
    for (int i = 0; i < 100; ++i)
    {
      auto pose = goal_;
      pose.position.x = 0.1 * i;
      checker_.isGoalReached(pose, goal_, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{});
    }
  }

  void setPath(const nav_msgs::msg::Path::SharedPtr& path)
  {
    checker_.onPath(path);
  }

  // Friendship is the fixture's, not a TEST_F body's.
  double xyGoalTolerance() const
  {
    return checker_.xy_goal_tolerance_;
  }

  std::size_t maxReachedIndex() const
  {
    return checker_.max_reached_index_;
  }

  void setEpochTime(double time)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(),
                                        static_cast<int64_t>(std::llround(time * 1e9))),
              RCL_RET_OK);
  }

  void clearEpochTransforms()
  {
    checker_.tf_buffer_->clear();
  }

  nav_msgs::msg::Path::SharedPtr forgetEpochPath()
  {
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = checker_.path_frame_;
    path->poses = checker_.path_poses_;
    checker_.reset();
    checker_.path_poses_.clear();
    checker_.path_arc_m_.clear();
    checker_.path_frame_.clear();
    return path;
  }

  void epochTransform(double time,
                      const std::string& parent,
                      const std::string& child,
                      double x,
                      double y = 0.0,
                      double yaw = 0.0,
                      bool fixed = false)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.header.stamp =
        rclcpp::Time(static_cast<int64_t>(std::llround(time * 1e9)), RCL_ROS_TIME);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
    ASSERT_TRUE(checker_.tf_buffer_->setTransform(transform, "epoch_test", fixed));
  }

  geometry_msgs::msg::Pose epochQuery(double time = 0.0)
  {
    geometry_msgs::msg::PoseStamped pose;
    EXPECT_TRUE(
        nav2_util::getCurrentPose(pose,
                                  *checker_.tf_buffer_,
                                  "odom",
                                  "base_footprint",
                                  0.0,
                                  rclcpp::Time(static_cast<int64_t>(std::llround(time * 1e9)),
                                               RCL_ROS_TIME)));
    return pose.pose;
  }

  geometry_msgs::msg::Pose epochGoal(double time)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(time * 1e9)), RCL_ROS_TIME);
    goal.pose = canonical_epoch_goal_;
    return checker_.tf_buffer_->transform(goal, "odom").pose;
  }

  nav_msgs::msg::Path epochLocalPlan(double time)
  {
    nav_msgs::msg::Path local;
    local.header.frame_id = "odom";
    local.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(time * 1e9)), RCL_ROS_TIME);
    return local;
  }

  void prepareEpochCase(bool moving_anchor = false,
                        bool past_latest = false,
                        double position_offset = 0.0,
                        bool lagging_anchor = false)
  {
    // Set the clock before loading TF, then move it only forwards: a clock
    // reset correctly clears Buffer and would invalidate this fixture.
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    setEpochTime(95.0);
    checker_.tf_buffer_->clear();
    checker_.base_frame_ = "base_footprint";
    checker_.xy_goal_tolerance_ = 0.03;
    constexpr double yaw = 0.6;
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";
    for (int i = 0; i <= 80; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = 100.0 + 0.01 * i * std::cos(yaw);
      pose.pose.position.y = 0.01 * i * std::sin(yaw);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path->poses.push_back(pose);
    }
    setPath(path);
    canonical_epoch_goal_ = path->poses.back().pose;
    if (!moving_anchor)
      epochTransform(0.0, "map", "odom", 100.0, 0.0, yaw, true);
    for (int i = 0; i <= 78; ++i)
    {
      const double time = 96.2 + 0.01 * i / 0.2 - (past_latest ? 0.15 : 0.0);
      epochTransform(time, "odom", "base_footprint", 0.01 * i + position_offset);
      if (moving_anchor && (!lagging_anchor || time <= 99.95 + 1e-9))
      {
        const double shift = std::max(0.0, (time - 100.0) * 0.2);
        epochTransform(
            time, "map", "odom", 100.0 + shift * std::cos(yaw), shift * std::sin(yaw), yaw);
      }
    }
    for (int i = 0; i <= 74; ++i)
    {
      const double time = 96.2 + 0.01 * i / 0.2 - (past_latest ? 0.15 : 0.0);
      setEpochTime(time);
      EXPECT_FALSE(checker_.isGoalReached(
          epochQuery(time), epochGoal(time), geometry_msgs::msg::Twist{}, epochLocalPlan(time)));
    }
    setEpochTime(100.0);
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  PathProgressGoalChecker checker_;
  geometry_msgs::msg::Pose goal_;
  geometry_msgs::msg::Pose canonical_epoch_goal_;
};

TEST_F(PathProgressGoalCheckerTest, GoalProximityDoesNotSkipFullPath)
{
  // A pruned one-pose local plan must not erase the complete-path progress gate.
  nav_msgs::msg::Path local;
  local.poses.resize(1);
  EXPECT_FALSE(checker_.isGoalReached(goal_, goal_, geometry_msgs::msg::Twist{}, local));
  EXPECT_FALSE(checker_.isGoalXYReached(goal_, goal_, geometry_msgs::msg::Twist{}, local));
}

TEST_F(PathProgressGoalCheckerTest, NearEndReplayNeedsProgressBeforeProximityCompletes)
{
  // The shared replay length exceeds the proximity-only exception and keeps
  // the first bounded progress scan below the 95% threshold at the goal.
  auto replay = std::make_shared<nav_msgs::msg::Path>();
  replay->header.frame_id = "map";
  for (std::size_t i = 0; i < mowgli_interfaces::kCoverageResumeReplayPoses; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 100.0 + 0.1 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    replay->poses.push_back(pose);
  }
  setPath(replay);

  auto replay_goal = goal_;
  replay_goal.position.x = 1.1;
  for (int tick = 0; tick < 20; ++tick)
  {
    EXPECT_FALSE(checker_.isGoalReached(
        replay_goal, replay_goal, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}))
        << "tick " << tick;
  }
  for (int tick = 0; tick < 20; ++tick)
  {
    auto jittered_goal = replay_goal;
    jittered_goal.position.x += (tick % 2 == 0) ? 0.006 : -0.006;
    EXPECT_FALSE(checker_.isGoalReached(
        jittered_goal, replay_goal, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}))
        << "endpoint correction " << tick;
  }

  // This bounded sequence has positive motion from the first query but less
  // than the required accumulated forward displacement. Per-tick thresholds
  // would incorrectly release the search boundary on the final sample.
  checker_.reset();
  setPath(replay);
  for (const double x : {1.1, 1.088, 1.112, 1.125})
  {
    auto query = replay_goal;
    query.position.x = x;
    EXPECT_FALSE(checker_.isGoalReached(
        query, replay_goal, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  }
  EXPECT_EQ(maxReachedIndex(), 0u);

  // On a fresh replay, actual forward motion is still required before normal
  // progress-gated completion can occur — right up to the deliberate
  // end-approach rule, which completes the goal once the path still ahead of
  // the monotonic cursor is shorter than xy_goal_tolerance (FTC parks that far
  // short of the last pose; see path_progress_goal_checker.hpp). Only the span
  // before that final band can be asserted incomplete.
  checker_.reset();
  setPath(replay);
  constexpr double kReplayStepM = 0.1;
  const auto end_approach_poses =
      static_cast<std::size_t>(std::ceil(xyGoalTolerance() / kReplayStepM)) + 1;
  ASSERT_LT(end_approach_poses, replay->poses.size());
  for (std::size_t i = 0; i + end_approach_poses < replay->poses.size(); ++i)
  {
    auto replay_pose = replay_goal;
    replay_pose.position.x = kReplayStepM * static_cast<double>(i);
    EXPECT_FALSE(checker_.isGoalReached(
        replay_pose, replay_goal, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}))
        << "pose " << i;
  }
  EXPECT_TRUE(checker_.isGoalReached(
      replay_goal, replay_goal, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
}

TEST_F(PathProgressGoalCheckerTest, ProgressUsesMapToOdomTransform)
{
  advance();
  EXPECT_TRUE(
      checker_.isGoalReached(goal_, goal_, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
}

TEST_F(PathProgressGoalCheckerTest, XYCheckIgnoresOnlyYaw)
{
  advance();
  auto rotated = goal_;
  rotated.orientation.w = 0.0;
  rotated.orientation.z = 1.0;
  EXPECT_FALSE(
      checker_.isGoalReached(rotated, goal_, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
  EXPECT_TRUE(
      checker_.isGoalXYReached(rotated, goal_, geometry_msgs::msg::Twist{}, nav_msgs::msg::Path{}));
}

TEST_F(PathProgressGoalCheckerTest, FutureLatestDoesNotCompleteBeforeCurrentPose)
{
  prepareEpochCase();
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
  EXPECT_EQ(maxReachedIndex(), 76u);
  setEpochTime(100.1);
  EXPECT_TRUE(checker_.isGoalReached(
      epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
}

TEST_F(PathProgressGoalCheckerTest, PastLatestDoesNotNeedCurrentTimeBracket)
{
  prepareEpochCase(false, true);
  EXPECT_TRUE(checker_.isGoalReached(
      epochQuery(), epochGoal(99.95), geometry_msgs::msg::Twist{}, epochLocalPlan(99.95)));
}

TEST_F(PathProgressGoalCheckerTest, FutureGoalUsesItsOriginalTransformEpoch)
{
  prepareEpochCase(true);
  // In odom, the future goal is .78 and current robot is .76: correcting only
  // the robot would falsely pass. Their common map-frame error is still .04.
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
  EXPECT_EQ(maxReachedIndex(), 76u);
}

TEST_F(PathProgressGoalCheckerTest, FutureQueryNeedsGoalEpochWithoutAdvancingProgress)
{
  prepareEpochCase(true);
  const auto before = maxReachedIndex();
  for (auto local : {nav_msgs::msg::Path{}, epochLocalPlan(100.1), epochLocalPlan(90.0)})
  {
    if (local.header.stamp.sec == 100)
      local.header.frame_id = "wrong_frame";
    EXPECT_FALSE(
        checker_.isGoalReached(epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, local));
    EXPECT_EQ(maxReachedIndex(), before);
  }
}

TEST_F(PathProgressGoalCheckerTest, FutureQueryUsesCompleteChainsSupportedPastEpoch)
{
  prepareEpochCase(true, false, 0.0, true);
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(99.95), geometry_msgs::msg::Twist{}, epochLocalPlan(99.95)));
  EXPECT_EQ(maxReachedIndex(), 75u);
}

TEST_F(PathProgressGoalCheckerTest, FutureQueryWithoutCurrentBracketDoesNotAdvanceProgress)
{
  prepareEpochCase();
  const auto before = maxReachedIndex();
  clearEpochTransforms();
  epochTransform(0.0, "map", "odom", 100.0, 0.0, 0.6, true);
  epochTransform(100.1, "odom", "base_footprint", 0.78);
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
  EXPECT_EQ(maxReachedIndex(), before);
}

TEST_F(PathProgressGoalCheckerTest, FutureXYCheckIgnoresYawAfterEpochConversion)
{
  prepareEpochCase(false, false, 0.02);
  auto goal = epochGoal(100.1);
  goal.orientation.z = std::sin(0.5);
  goal.orientation.w = std::cos(0.5);
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), goal, geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
  EXPECT_TRUE(checker_.isGoalXYReached(
      epochQuery(), goal, geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
}

TEST_F(PathProgressGoalCheckerTest, FutureQueryRetainsTheSuppliedGoal)
{
  prepareEpochCase(false, false, 0.02);
  auto supplied_goal = epochGoal(100.1);
  supplied_goal.position.x = 0.70;
  // The robot is within tolerance of our topic endpoint, but the public API
  // supplies a different goal. Epoch correction must not substitute the topic.
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), supplied_goal, geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));
}

TEST_F(PathProgressGoalCheckerTest, MissingPlanWithFutureTFHoldsUntilItsFixedFrameArrives)
{
  prepareEpochCase();
  auto path = forgetEpochPath();
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(100.1), geometry_msgs::msg::Twist{}, epochLocalPlan(100.1)));

  setEpochTime(106.0);
  epochTransform(106.0, "odom", "base_footprint", 0.78);
  epochTransform(106.1, "odom", "base_footprint", 0.78);
  // Even at a valid current position, elapsed time cannot identify the goal's
  // original fixed frame. A healthy forecast stream must not bypass that gap.
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(106.1), geometry_msgs::msg::Twist{}, epochLocalPlan(106.1)));
  EXPECT_EQ(maxReachedIndex(), 0u);

  // A fresh short terminal path supplies the missing fixed frame; its normal
  // proximity rule can complete at the supported current pose.
  path->poses.erase(path->poses.begin(), path->poses.end() - 3);
  setPath(path);
  EXPECT_TRUE(checker_.isGoalReached(
      epochQuery(), epochGoal(106.1), geometry_msgs::msg::Twist{}, epochLocalPlan(106.1)));
}

TEST_F(PathProgressGoalCheckerTest, MissingPlanWithPastLatestRetainsFallbackDeadline)
{
  prepareEpochCase(false, true);
  forgetEpochPath();
  EXPECT_FALSE(checker_.isGoalReached(
      epochQuery(), epochGoal(99.95), geometry_msgs::msg::Twist{}, epochLocalPlan(99.95)));
  setEpochTime(106.0);
  epochTransform(105.95, "odom", "base_footprint", 0.78);
  EXPECT_TRUE(checker_.isGoalReached(
      epochQuery(), epochGoal(105.95), geometry_msgs::msg::Twist{}, epochLocalPlan(105.95)));
}

}  // namespace mowgli_nav2_plugins
