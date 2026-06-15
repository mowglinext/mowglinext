// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// PathProgressGoalChecker — a Nav2 GoalChecker plugin that fires only
// when the robot has traversed at least `progress_threshold` of the
// FTC path's poses, AND is near the final pose.
//
// Why: SimpleGoalChecker fires whenever the robot is within
// `xy_goal_tolerance` of the goal pose, REGARDLESS of how much of the
// path was actually traversed. F2C coverage paths can route the robot
// through the perimeter and end with a pose that happens to sit near
// the perimeter too — the goal-checker then fires during the headland
// phase, completing the action with <2% coverage.
//
// PathProgressGoalChecker gates completion on two conditions:
//   1. monotonically-tracked max-reached path index >= progress_threshold
//      * global_plan size (default 0.95 = 95%)
//   2. robot is within xy_goal_tolerance of the goal pose AND yaw is
//      within yaw_goal_tolerance (matches SimpleGoalChecker semantics
//      for the final-pose check)
//
// The plugin subscribes to the FTC controller's republished
// `<plugin_name>/global_plan` topic so it always has the latest path
// for index-tracking. Resets the max-reached index on every new path
// (detected by size change OR header timestamp change).

#ifndef MOWGLI_NAV2_PLUGINS__PATH_PROGRESS_GOAL_CHECKER_HPP_
#define MOWGLI_NAV2_PLUGINS__PATH_PROGRESS_GOAL_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mowgli_nav2_plugins
{

class PathProgressGoalChecker : public nav2_core::GoalChecker
{
public:
  PathProgressGoalChecker() = default;
  ~PathProgressGoalChecker() override = default;

  void initialize(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      const std::string& plugin_name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
      const geometry_msgs::msg::Pose& query_pose,
      const geometry_msgs::msg::Pose& goal_pose,
      const geometry_msgs::msg::Twist& velocity) override;

  bool getTolerances(
      geometry_msgs::msg::Pose& pose_tolerance,
      geometry_msgs::msg::Twist& vel_tolerance) override;

private:
  void onPath(nav_msgs::msg::Path::SharedPtr msg);

  rclcpp::Logger logger_{rclcpp::get_logger("path_progress_goal_checker")};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::shared_ptr<rclcpp::Clock> clock_;

  // Parameters
  double progress_threshold_{0.95};
  double xy_goal_tolerance_{0.20};
  double yaw_goal_tolerance_{0.30};
  std::string plan_topic_{};
  // Per-swath DISCONTINUOUS coverage feeds short paths (a mow swath, or a tiny
  // turn-connector between swaths). On a path this short the monotonic
  // 95%-progress gate is unreliable — a 3-pose connector never registers enough
  // index advance and the goal hangs, stalling the whole swath sequence. For any
  // path with <= short_path_poses_ poses, fall back to SimpleGoalChecker
  // semantics (fire on xy+yaw proximity to the goal pose), same as the n<=1
  // degenerate guard. Long coverage paths keep the progress gate (the reason
  // this plugin exists — see header). 0 disables (only n<=1 uses proximity).
  size_t short_path_poses_{10};
  // Bound on the forward search window in isGoalReached. Prevents
  // boustrophedon paths from letting max_reached_index_ kangaroo past
  // a loop-back point. Tuned at 10 poses (≥10× the per-call physical
  // bound at typical chassis speeds) — see the cpp for the calculation.
  size_t max_idx_advance_per_call_{10};

  // State guarded by mutex_ (controller_server may call isGoalReached
  // from one thread while the topic callback fires on another).
  std::mutex mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> path_poses_;
  size_t max_reached_index_{0};
  // Detect a fresh path so we can reset the max-reached index. Use the
  // pose count + first-pose XY as a cheap fingerprint (header.stamp is
  // unreliable when controller_server forwards a stale plan).
  size_t last_path_size_{0};
  double last_path_first_x_{0.0};
  double last_path_first_y_{0.0};

  // Watchdog: time-of-first isGoalReached call when path_poses_ is
  // still empty. If FTC's global_plan never publishes (DDS race,
  // plugin name mismatch, transient hiccup), the goal-checker would
  // otherwise never fire and the action would only abort on
  // controller goal_timeout (10 s). After fallback_timeout_s_ of
  // empty-path isGoalReached calls we drop to SimpleGoalChecker
  // semantics: assert as soon as the robot is within xy/yaw
  // tolerance of the goal pose. Reset on reset() and on any path
  // arrival.
  std::optional<rclcpp::Time> empty_path_first_call_;
  double fallback_timeout_s_{5.0};
};

}  // namespace mowgli_nav2_plugins

#endif  // MOWGLI_NAV2_PLUGINS__PATH_PROGRESS_GOAL_CHECKER_HPP_
