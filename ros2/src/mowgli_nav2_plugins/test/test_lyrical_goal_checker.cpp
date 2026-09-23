// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstddef>
#include <type_traits>

#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"
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
      checker_.isGoalReached(pose, goal_, {}, {});
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

  std::shared_ptr<nav2::LifecycleNode> node_;
  PathProgressGoalChecker checker_;
  geometry_msgs::msg::Pose goal_;
};

TEST_F(PathProgressGoalCheckerTest, GoalProximityDoesNotSkipFullPath)
{
  // A pruned one-pose local plan must not erase the complete-path progress gate.
  nav_msgs::msg::Path local;
  local.poses.resize(1);
  EXPECT_FALSE(checker_.isGoalReached(goal_, goal_, {}, local));
  EXPECT_FALSE(checker_.isGoalXYReached(goal_, goal_, {}, local));
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
    EXPECT_FALSE(checker_.isGoalReached(replay_goal, replay_goal, {}, {})) << "tick " << tick;
  }
  for (int tick = 0; tick < 20; ++tick)
  {
    auto jittered_goal = replay_goal;
    jittered_goal.position.x += (tick % 2 == 0) ? 0.006 : -0.006;
    EXPECT_FALSE(checker_.isGoalReached(jittered_goal, replay_goal, {}, {}))
        << "endpoint correction " << tick;
  }

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
    EXPECT_FALSE(checker_.isGoalReached(replay_pose, replay_goal, {}, {})) << "pose " << i;
  }
  EXPECT_TRUE(checker_.isGoalReached(replay_goal, replay_goal, {}, {}));
}

TEST_F(PathProgressGoalCheckerTest, ProgressUsesMapToOdomTransform)
{
  advance();
  EXPECT_TRUE(checker_.isGoalReached(goal_, goal_, {}, {}));
}

TEST_F(PathProgressGoalCheckerTest, XYCheckIgnoresOnlyYaw)
{
  advance();
  auto rotated = goal_;
  rotated.orientation.w = 0.0;
  rotated.orientation.z = 1.0;
  EXPECT_FALSE(checker_.isGoalReached(rotated, goal_, {}, {}));
  EXPECT_TRUE(checker_.isGoalXYReached(rotated, goal_, {}, {}));
}

}  // namespace mowgli_nav2_plugins
