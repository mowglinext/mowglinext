// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
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
    checker_.onPath(path);
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
