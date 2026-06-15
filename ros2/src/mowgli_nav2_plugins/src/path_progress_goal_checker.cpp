// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// PathProgressGoalChecker — implementation. See header for the why.

#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace mowgli_nav2_plugins
{

void PathProgressGoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    const std::string& plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::runtime_error(
        "PathProgressGoalChecker: failed to lock parent LifecycleNode");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string& key, auto default_value) {
    const std::string full = plugin_name + "." + key;
    if (!node->has_parameter(full))
    {
      node->declare_parameter(full, default_value);
    }
    return node->get_parameter(full);
  };

  progress_threshold_ = declare("progress_threshold", 0.95).as_double();
  xy_goal_tolerance_ = declare("xy_goal_tolerance", 0.20).as_double();
  yaw_goal_tolerance_ = declare("yaw_goal_tolerance", 0.30).as_double();
  fallback_timeout_s_ = declare("fallback_timeout_s", 5.0).as_double();
  // Max idx advance per isGoalReached call. Bounds the "find closest
  // pose forward of max_reached_index_" search so a boustrophedon-
  // style path that loops back over earlier ground doesn't let the
  // search jump from idx 114 directly to idx 3175 just because that
  // point happens to be closer to the robot's current pose. At a
  // ~5 cm path resolution and ~0.5 m/s max chassis speed, the robot
  // advances ≤ 10 poses per second; the goal checker is consulted
  // at ≤ 20 Hz so ≤ 1 pose per call is the physical bound, with
  // wide safety margin baked into the default.
  max_idx_advance_per_call_ =
      static_cast<size_t>(declare("max_idx_advance_per_call", 10).as_int());
  // Short paths (<= this many poses) complete on proximity, not progress —
  // per-swath DISCONTINUOUS coverage feeds short swaths + tiny turn-connectors
  // that the 95%-progress gate can't reliably register (stall). See header.
  short_path_poses_ = static_cast<size_t>(declare("short_path_poses", 10).as_int());

  // Which controller's republished plan to track. Default matches the
  // FollowCoveragePath FTC slot from nav2_params.yaml. If you have a
  // controller publishing under a different name, override via
  // `<this_checker_name>.plan_topic: /controller_server/<name>/global_plan`.
  plan_topic_ = declare("plan_topic",
                        std::string("/controller_server/FollowCoveragePath/global_plan"))
                    .as_string();

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      plan_topic_, qos,
      [this](nav_msgs::msg::Path::SharedPtr msg) { onPath(msg); });

  RCLCPP_INFO(logger_,
              "PathProgressGoalChecker[%s]: progress_threshold=%.2f, "
              "xy_tol=%.2fm, yaw_tol=%.2frad, plan_topic=%s",
              plugin_name.c_str(),
              progress_threshold_,
              xy_goal_tolerance_,
              yaw_goal_tolerance_,
              plan_topic_.c_str());
}

void PathProgressGoalChecker::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  max_reached_index_ = 0;
  empty_path_first_call_.reset();
}

void PathProgressGoalChecker::onPath(nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty())
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Path arrived — clear the watchdog so future empty-path windows
  // (e.g. between strips) get their own grace period.
  empty_path_first_call_.reset();

  const size_t n = msg->poses.size();
  const double fx = msg->poses.front().pose.position.x;
  const double fy = msg->poses.front().pose.position.y;

  // Detect a NEW path so we can reset progress. FTC re-publishes its
  // global_plan on EVERY tick during FOLLOWING — the start pose
  // wobbles by a few centimetres each tick because FTC uses the
  // robot's current TF position as the trim-front of its republished
  // plan. The earlier 5 cm "start moved" threshold reset
  // max_reached_index_=0 multiple times per second on coverage paths
  // longer than ~10 m, so the 95 % progress gate never fired and
  // FollowStrip systematically aborted on WAITING_FOR_GOAL_APPROACH.
  //
  // Use TWO insensitive-to-republish signals instead:
  //   1. Pose-count change (a fresh setPlan always changes n).
  //   2. Front pose moved by MORE than 2 m (a real new strip start;
  //      republish wobble is a few cm at most).
  const bool size_changed = (n != last_path_size_);
  const bool start_moved_far = std::hypot(fx - last_path_first_x_,
                                           fy - last_path_first_y_) > 2.0;
  if (size_changed || start_moved_far)
  {
    path_poses_ = msg->poses;
    last_path_size_ = n;
    last_path_first_x_ = fx;
    last_path_first_y_ = fy;
    max_reached_index_ = 0;
    RCLCPP_INFO(logger_,
                "PathProgressGoalChecker: new path with %zu poses, "
                "start=(%.2f,%.2f), end=(%.2f,%.2f) — reset progress",
                n, fx, fy,
                msg->poses.back().pose.position.x,
                msg->poses.back().pose.position.y);
  }
  else
  {
    // Same path, just FTC republishing — refresh the pose buffer
    // (FTC may have appended the carrot or trimmed the front) but
    // keep max_reached_index_ so the monotonic-progress invariant
    // holds across republishes.
    path_poses_ = msg->poses;
  }
}

bool PathProgressGoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose& query_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const geometry_msgs::msg::Twist& /*velocity*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Empty path watchdog. Normally we refuse to fire until FTC has
  // published its global_plan (prevents the legacy SimpleGoalChecker
  // behaviour where the action completes before any cmd_vel was
  // sent). But if the topic never arrives — DDS race during
  // controller_server activation, plugin_name mismatch, transient
  // hiccup — we'd block forever and only abort on goal_timeout
  // (10 s default in nav2_params). After fallback_timeout_s_ of
  // empty-path isGoalReached calls, fall back to SimpleGoalChecker
  // semantics: assert as soon as the robot is within tolerance of
  // the goal pose. WARN once when the fallback engages so an
  // operator can investigate.
  if (path_poses_.empty())
  {
    if (!empty_path_first_call_.has_value())
    {
      empty_path_first_call_ = clock_->now();
      return false;
    }
    const double age = (clock_->now() - *empty_path_first_call_).seconds();
    if (age < fallback_timeout_s_)
    {
      return false;
    }
    RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "PathProgressGoalChecker: no global_plan after %.1fs on %s — "
        "falling back to SimpleGoalChecker semantics. Check controller "
        "plugin_name vs plan_topic.",
        age, plan_topic_.c_str());
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    if (std::hypot(dx, dy) > xy_goal_tolerance_)
    {
      return false;
    }
    const double yaw_q = tf2::getYaw(query_pose.orientation);
    const double yaw_g = tf2::getYaw(goal_pose.orientation);
    const double yaw_err = std::atan2(std::sin(yaw_q - yaw_g),
                                      std::cos(yaw_q - yaw_g));
    return std::abs(yaw_err) <= yaw_goal_tolerance_;
  }

  // Path arrived — fall through to the normal progress check.

  // Short path: a single-pose path would divide by zero in the (n-1)
  // progress denominator, and a few-pose path (a per-swath mow segment or a
  // tiny turn-connector under DISCONTINUOUS coverage) is too short for the
  // monotonic-progress gate to register reliably — it hangs at <95% forever and
  // stalls the swath sequence. For n <= short_path_poses_ treat as "reached"
  // once the robot is within xy+yaw tolerance of the goal pose.
  const size_t n = path_poses_.size();
  if (n <= short_path_poses_)
  {
    const double dx = query_pose.position.x - goal_pose.position.x;
    const double dy = query_pose.position.y - goal_pose.position.y;
    if (std::hypot(dx, dy) > xy_goal_tolerance_)
    {
      return false;
    }
    const double yaw_q = tf2::getYaw(query_pose.orientation);
    const double yaw_g = tf2::getYaw(goal_pose.orientation);
    const double yaw_err = std::atan2(std::sin(yaw_q - yaw_g),
                                      std::cos(yaw_q - yaw_g));
    return std::abs(yaw_err) <= yaw_goal_tolerance_;
  }

  // Update max-reached index by finding the closest pose to the robot
  // FORWARD of the previous max, BOUNDED by max_idx_advance_per_call_.
  // The unbounded search (start..n-1) was correct for a one-way path
  // but broke on F2C boustrophedon coverage paths where the trajectory
  // loops back near earlier ground — once the robot moved a few cm
  // through a swath that geographically coincides with a later return
  // swath, the global-min search found a much-later idx as "closer"
  // and max_reached_index_ jumped forward by thousands of poses in
  // one call, instantly satisfying the 95% progress threshold and
  // declaring the action SUCCEEDED with 0% real coverage.
  //
  // The physical bound: the chassis advances ≤ vmax · dt / path_step
  // poses per call. With vmax = 0.5 m/s, dt = 50 ms (20 Hz controller
  // loop) and path_step ≈ 5 cm we get ≤ 0.5 poses/call. Cap at
  // max_idx_advance_per_call_ (default 10) for a safety margin
  // against tick-rate jitter and shorter-than-expected path steps.
  const size_t start = std::min(max_reached_index_, n - 1);
  const size_t end_exclusive =
      std::min(start + max_idx_advance_per_call_ + 1, n);
  double best_d2 = std::numeric_limits<double>::infinity();
  size_t best_idx = max_reached_index_;
  for (size_t i = start; i < end_exclusive; ++i)
  {
    const double dx = path_poses_[i].pose.position.x - query_pose.position.x;
    const double dy = path_poses_[i].pose.position.y - query_pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2)
    {
      best_d2 = d2;
      best_idx = i;
    }
  }
  if (best_idx > max_reached_index_)
  {
    max_reached_index_ = best_idx;
  }

  const double progress = static_cast<double>(max_reached_index_) /
                          static_cast<double>(n - 1);
  if (progress < progress_threshold_)
  {
    return false;
  }

  // Path progress condition met — also require XY proximity to the
  // goal pose and yaw within tolerance, matching SimpleGoalChecker's
  // final-pose check (so FTC can still do POST_ROTATE precision work).
  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;
  const double xy_err = std::hypot(dx, dy);
  if (xy_err > xy_goal_tolerance_)
  {
    return false;
  }

  const double yaw_q = tf2::getYaw(query_pose.orientation);
  const double yaw_g = tf2::getYaw(goal_pose.orientation);
  double yaw_err = std::atan2(std::sin(yaw_q - yaw_g),
                              std::cos(yaw_q - yaw_g));
  if (std::abs(yaw_err) > yaw_goal_tolerance_)
  {
    return false;
  }

  RCLCPP_INFO(logger_,
              "PathProgressGoalChecker: goal reached — progress=%.1f%% "
              "(idx %zu/%zu), xy_err=%.3fm, yaw_err=%.3frad",
              progress * 100.0, max_reached_index_, n - 1,
              xy_err, yaw_err);
  return true;
}

bool PathProgressGoalChecker::getTolerances(
    geometry_msgs::msg::Pose& pose_tolerance,
    geometry_msgs::msg::Twist& vel_tolerance)
{
  // Report XY + yaw tolerance for upstream (e.g., bt_navigator). Velocity
  // tolerance is unused — we don't gate on velocity at all.
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;

  // Pack the yaw tolerance into the quaternion's z field (a hack
  // matching SimpleGoalChecker — Nav2 plugins generally treat
  // pose_tolerance.orientation.z as a raw scalar yaw tolerance).
  pose_tolerance.orientation.z = yaw_goal_tolerance_;

  const double kLowest = std::numeric_limits<double>::lowest();
  vel_tolerance.linear.x = kLowest;
  vel_tolerance.linear.y = kLowest;
  vel_tolerance.angular.z = kLowest;
  return true;
}

}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::PathProgressGoalChecker,
                       nav2_core::GoalChecker)
