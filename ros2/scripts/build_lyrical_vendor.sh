#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the unreleased ROS dependencies without modifying vendored submodules.
# Prerequisite apt packages are installed by Docker/CI; this script never uses sudo.
set -eo pipefail
if [ "$(id -u)" -eq 0 ]; then
  echo "Run vendor builds as the project user, not root" >&2
  exit 1
fi
prefix="${1:-/opt/lyrical_vendor}"
# Docker supplies a dedicated cache directory; standalone/CI calls use a temp tree.
work="${VENDOR_WORKSPACE:-}"
if [ -z "$work" ]; then
  work="$(mktemp -d)"
  trap 'rm -rf "$work"' EXIT
fi
source /opt/ros/lyrical/setup.bash
mkdir -p "$prefix" "$work/src"

checkout() {
  git init -q "$work/src/$1"
  git -C "$work/src/$1" fetch --depth 1 "$2" "$3"
  git -C "$work/src/$1" checkout -q --force --detach FETCH_HEAD
}
checkout grid_map https://github.com/ANYbotics/grid_map.git 7ae24725d7effa0c22a23524b91b46bada5b2728
checkout beluga https://github.com/Ekumen-OS/beluga.git 22adc90e08cc7229cde042a756f044b908600fed
checkout sophus https://github.com/strasdat/Sophus.git de0f8d3d92bf776271e16de56d1803940ebccab9
checkout navigation2 https://github.com/ros-navigation/navigation2.git a6354f3f39d12c6e5c3e323f8021c831752edb23

# grid_map_ros still uses the ament macro removed in Lyrical. Patch only the
# pinned build copy, linking exported targets and the legacy grid_map_cv export.
python3 - "$work/src/grid_map/grid_map_ros/CMakeLists.txt" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
old = 'ament_target_dependencies(${PROJECT_NAME} SYSTEM\n  ${dependencies}\n)'
new = '''foreach(dep IN LISTS dependencies)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${${dep}_INCLUDE_DIRS})
  if(${dep}_TARGETS)
    target_link_libraries(${PROJECT_NAME} ${${dep}_TARGETS})
  else()
    target_link_libraries(${PROJECT_NAME} ${${dep}_LIBRARIES})
  endif()
endforeach()'''
assert old in s, 'grid_map patch no longer matches the pinned source'
p.write_text(s.replace(old, new))
PY

# Keep the shim's local-costmap collision simulation on the commanded arc.
# This patch is tied to the pinned Nav2 source above and participates in the
# Docker/CI vendor cache key through this script. RPP remains apt-provided.
python3 - "$work/src/navigation2/nav2_rotation_shim_controller" <<'PY_SHIM'
from pathlib import Path
import sys

root = Path(sys.argv[1])
source = root / 'src/nav2_rotation_shim_controller.cpp'
header = root / 'include/nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp'
s = source.read_text()
h = header.read_text()

def replace_once(text, old, new):
    assert text.count(old) == 1, f'RotationShim patch no longer matches: {old!r}'
    return text.replace(old, new)

# Goal alignment has no rough-heading release allowance. Initial alignment
# retains the existing entry/disengage hysteresis and releases at disengage.
# The two call sites are identical; assert their context before changing only goal alignment.
old = '''        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      }
    } catch'''
new = old.replace('pose, velocity);', 'pose, velocity, true);')
s = replace_once(s, old, new)
for text, path in [(s, source), (h, header)]:
    old = 'const geometry_msgs::msg::Twist & velocity)'
    new = ('const geometry_msgs::msg::Twist & velocity, const bool rotate_to_goal)' if path == source
           else 'const geometry_msgs::msg::Twist & velocity, const bool rotate_to_goal = false)')
    text = replace_once(text, old, new)
    # This suffix also belongs to other methods, so anchor the collision signature.
    signature = '''const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)''' if path == source else '''const double & angular_distance_to_heading,
    const geometry_msgs::msg::PoseStamped & pose)'''
    text = replace_once(text, signature, signature[:-1] + ', const bool rotate_to_goal)')
    if path == source:
        s = text
    else:
        h = text
s = replace_once(s, 'isCollisionFree(cmd_vel, angular_distance_to_heading, pose);',
                 'isCollisionFree(cmd_vel, angular_distance_to_heading, pose, rotate_to_goal);')
s = replace_once(s, '''  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - params_->angular_dist_threshold;''', '''  const double remaining_rotation_before_thresh = std::max(
    0.0, fabs(angular_distance_to_heading) -
    (rotate_to_goal ? 0.0 : params_->angular_disengage_threshold));
  const double heading_sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;''')
s = replace_once(s, '''    // Stop simulating past the point it would be passed onto the primary controller
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

''', '')
s = replace_once(s, '''    if (footprint_cost >= params_->max_cost_threshold) {
      throw nav2_core::NoValidControl("RotationShimController detected collision ahead!");
    }
''', '''    if (footprint_cost >= params_->max_cost_threshold) {
      throw nav2_core::NoValidControl("RotationShimController detected collision ahead!");
    }

    // Check the first command interval and the sample reaching the release
    // boundary before stopping. Opposite-sign slew is real motion, not progress
    // toward release, and must retain the complete simulation horizon.
    if (heading_sign * cmd_vel.twist.angular.z * simulated_time >=
      remaining_rotation_before_thresh)
    {
      break;
    }
''')
# A collision refusal must reach controller_server. Falling through to RPP
# can replace the refused rotation with an unchecked forward proposal.
old = '    } catch (const std::runtime_error & e) {'
assert s.count(old) == 2, 'RotationShim collision catch sites changed'
s = s.replace(old, '''    } catch (const nav2_core::NoValidControl &) {
      throw;
    } catch (const std::runtime_error & e) {''')

# Optional, startup-only complete-arc choice for initial alignment. Terminal
# alignment and the default shim contract retain the collision patch above.
s = replace_once(s, '''  const geometry_msgs::msg::PoseStamped & global_goal)
{
  current_path_ = transformed_global_plan;
  path_updated_ = params_->rotate_to_heading_once ? isGoalChanged(global_goal) : true;
  current_goal_ = global_goal;
  // Rotate to goal heading when in goal xy tolerance
  if (params_->rotate_to_goal_heading) {
''', '''  const geometry_msgs::msg::PoseStamped & global_goal)
{
  current_path_ = transformed_global_plan;
  const bool alternate_initial = params_->allow_alternate_initial_rotation &&
    params_->rotate_to_heading_once;
  if (alternate_initial) {
    // The raw path owns the goal for the whole dispatch, including primary
    // tracking after alignment. Delivery stamps and local pruning do not own it.
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
    (void)rawInitialTarget(pose, global_goal);
  }
  path_updated_ = alternate_initial ? raw_initial_pending_ :
    (params_->rotate_to_heading_once ? isGoalChanged(global_goal) : true);
  current_goal_ = global_goal;
  // Rotate to goal heading when in goal xy tolerance
  if (params_->rotate_to_goal_heading) {
''')
s = replace_once(s, '''      if (goal_checker->isGoalXYReached(pose.pose, global_goal.pose, velocity,
        transformed_global_plan))
      {
        double pose_yaw = tf2::getYaw(pose.pose.orientation);
        double goal_yaw = tf2::getYaw(global_goal.pose.orientation);

''', '''      if (goal_checker->isGoalXYReached(pose.pose, global_goal.pose, velocity,
        transformed_global_plan))
      {
        initial_rotation_selected_ = false;
        if (alternate_initial) {raw_initial_pending_ = false;}
        double pose_yaw = tf2::getYaw(pose.pose.orientation);
        double goal_yaw = tf2::getYaw(global_goal.pose.orientation);

''')
s = replace_once(s, '''
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
    try {
      auto sampled_pt = getSampledPathPt(global_goal);
      double angular_distance_to_heading;
      if (params_->use_path_orientations) {
        angular_distance_to_heading = angles::shortest_angular_distance(
          tf2::getYaw(pose.pose.orientation),
          tf2::getYaw(sampled_pt.pose.orientation));
''', '''
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
    try {
      const bool alternate = alternate_initial;
      auto sampled_pt = alternate ? geometry_msgs::msg::PoseStamped() : getSampledPathPt(global_goal);
      auto rotation_pose = pose;
      double angular_distance_to_heading;
      if (alternate) {
        rotation_pose = supportedInitialPose();
        sampled_pt = rawInitialTarget(rotation_pose, global_goal);
        try {
          if (sampled_pt.header.frame_id != rotation_pose.header.frame_id) {
            const auto transform = tf_->lookupTransform(rotation_pose.header.frame_id,
              sampled_pt.header.frame_id, rclcpp::Time(rotation_pose.header.stamp),
              tf2::durationFromSec(0.0));
            tf2::doTransform(sampled_pt, sampled_pt, transform);
          }
        } catch (const tf2::TransformException &) {
          throw nav2_core::NoValidControl("RotationShim initial target has no supported transform");
        }
        const auto & q = sampled_pt.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(sampled_pt.pose.position.x) ||
          !std::isfinite(sampled_pt.pose.position.y) ||
          (params_->use_path_orientations &&
          (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)))
        {
          throw nav2_core::NoValidControl("RotationShim initial rotation has invalid target");
        }
        const double target = params_->use_path_orientations ?
          tf2::getYaw(sampled_pt.pose.orientation) :
          std::atan2(sampled_pt.pose.position.y - rotation_pose.pose.position.y,
          sampled_pt.pose.position.x - rotation_pose.pose.position.x);
        angular_distance_to_heading = angles::shortest_angular_distance(
          tf2::getYaw(rotation_pose.pose.orientation), target);
      } else if (params_->use_path_orientations) {
        angular_distance_to_heading = angles::shortest_angular_distance(
          tf2::getYaw(pose.pose.orientation),
          tf2::getYaw(sampled_pt.pose.orientation));
''')
s = replace_once(s, '''          sampled_pt_base.position.x);
      }

      double angular_thresh =
        in_rotation_ ? params_->angular_disengage_threshold : params_->angular_dist_threshold;
      if (abs(angular_distance_to_heading) > angular_thresh) {
        RCLCPP_DEBUG(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        in_rotation_ = true;
        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      } else {
''', '''          sampled_pt_base.position.x);
      }

      if (alternate && !std::isfinite(angular_distance_to_heading)) {
        throw nav2_core::NoValidControl("RotationShim initial rotation has invalid target heading");
      }
      const bool selected_arc = alternate && initial_rotation_selected_;
      if (selected_arc) {
        // Update directed progress and recheck ownership before deciding to
        // release. A small fresh shortest angle can still leave almost a full
        // turn on the chosen arc after opposite or externally supplied motion.
        angular_distance_to_heading = initialRotationArc(rotation_pose, angular_distance_to_heading);
      }
      double angular_thresh =
        in_rotation_ ? params_->angular_disengage_threshold : params_->angular_dist_threshold;
      if (abs(angular_distance_to_heading) > angular_thresh) {
        if (alternate && (!std::isfinite(velocity.angular.z) || velocity.linear.x != 0.0 ||
          velocity.linear.y != 0.0 || velocity.linear.z != 0.0))
        {
          throw nav2_core::NoValidControl("RotationShim initial arc requires stopped translation");
        }
        RCLCPP_DEBUG(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        in_rotation_ = true;
        if (alternate && !selected_arc) {
          angular_distance_to_heading = initialRotationArc(rotation_pose, angular_distance_to_heading);
        }
        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, rotation_pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      } else {
''')
s = replace_once(s, '''          logger_,
          "Robot is at the new path's rough heading, passing to controller");
        path_updated_ = false;
      }
    } catch (const nav2_core::NoValidControl &) {
      throw;
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
''', '''          logger_,
          "Robot is at the new path's rough heading, passing to controller");
        path_updated_ = false;
        if (alternate) {raw_initial_pending_ = false;}
      }
    } catch (const nav2_core::NoValidControl &) {
      if (params_->allow_alternate_initial_rotation && params_->rotate_to_heading_once) {
        in_rotation_ = true;
      }
      throw;
    } catch (const std::runtime_error & e) {
      if (params_->allow_alternate_initial_rotation && params_->rotate_to_heading_once) {
        in_rotation_ = true;
        throw nav2_core::NoValidControl("RotationShim initial rotation lost its target");
      }
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
''')
s = replace_once(s, '''
  // If at this point, use the primary controller to path track
  in_rotation_ = false;
  auto cmd_vel = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker,
    transformed_global_plan, global_goal);
  last_angular_vel_ = cmd_vel.twist.angular.z;
''', '''
  // If at this point, use the primary controller to path track
  in_rotation_ = false;
  initial_rotation_selected_ = false;
  auto cmd_vel = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker,
    transformed_global_plan, global_goal);
  last_angular_vel_ = cmd_vel.twist.angular.z;
''')
s = replace_once(s, '''    cmd_vel.twist.angular.z = sign * max_vel_to_stop;
  }

  isCollisionFree(cmd_vel, angular_distance_to_heading, pose, rotate_to_goal);
  return cmd_vel;
}
''', '''    cmd_vel.twist.angular.z = sign * max_vel_to_stop;
  }

  if (!rotate_to_goal && params_->allow_alternate_initial_rotation &&
    params_->rotate_to_heading_once)
  {
    for (const double omega : {cmd_vel.twist.angular.z, velocity.angular.z, current}) {
      if (omega * angular_distance_to_heading < 0.0 &&
        !initialRotationArcClear(pose, omega * params_->simulate_ahead_time))
      {
        throw nav2_core::NoValidControl("RotationShim adverse initial angular motion is blocked");
      }
    }
  }
  isCollisionFree(cmd_vel, angular_distance_to_heading, pose, rotate_to_goal);
  return cmd_vel;
}
''')
s = replace_once(s, '''  }
}

bool RotationShimController::isGoalChanged(const geometry_msgs::msg::PoseStamped & goal)
{
  // Return true if rotating or if the goal pose is empty
''', '''  }
}

geometry_msgs::msg::PoseStamped RotationShimController::rawInitialTarget(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  if (raw_initial_path_.poses.size() < 2 || raw_initial_path_.header.frame_id.empty() ||
    raw_initial_path_.header.frame_id == costmap_ros_->getBaseFrameID())
  {
    throw nav2_core::NoValidControl("RotationShim initial rotation has no owned fixed-frame path");
  }
  auto source_goal = global_goal;
  try {
    if (source_goal.header.frame_id != raw_initial_path_.header.frame_id) {
      if (rclcpp::Time(source_goal.header.stamp).nanoseconds() == 0) {
        throw nav2_core::NoValidControl("RotationShim initial goal has no transform epoch");
      }
      const auto transform = tf_->lookupTransform(raw_initial_path_.header.frame_id,
        source_goal.header.frame_id, rclcpp::Time(source_goal.header.stamp),
        tf2::durationFromSec(0.0));
      tf2::doTransform(source_goal, source_goal, transform);
    }
  } catch (const tf2::TransformException &) {
    throw nav2_core::NoValidControl("RotationShim initial goal has no source-frame provenance");
  }
  const auto & owned_goal = raw_initial_path_.poses.back().pose;
  // Numerical roundoff allowance for recovering the supplied goal through TF;
  // this does not admit a materially different goal or reuse its old heading.
  const double goal_gap = std::hypot(source_goal.pose.position.x - owned_goal.position.x,
    source_goal.pose.position.y - owned_goal.position.y);
  const double goal_yaw_gap = angles::shortest_angular_distance(
    tf2::getYaw(source_goal.pose.orientation), tf2::getYaw(owned_goal.orientation));
  if (!std::isfinite(goal_gap) || !std::isfinite(goal_yaw_gap) ||
    goal_gap > 1e-6 || std::abs(goal_yaw_gap) > 1e-6)
  {
    initial_rotation_selected_ = false;
    raw_initial_path_.poses.clear();
    raw_initial_pending_ = false;
    throw nav2_core::NoValidControl("RotationShim initial goal requires a new owned path");
  }
  const auto & start = raw_initial_path_.poses.front().pose.position;
  auto sample = raw_initial_path_.poses.back();
  for (size_t i = 1; i < raw_initial_path_.poses.size(); ++i) {
    const auto & point = raw_initial_path_.poses[i].pose.position;
    if (std::hypot(point.x - start.x, point.y - start.y) >= params_->forward_sampling_distance) {
      sample = raw_initial_path_.poses[i];
      break;
    }
  }
  sample.header.frame_id = raw_initial_path_.header.frame_id;
  sample.header.stamp = pose.header.stamp;
  return sample;
}

geometry_msgs::msg::PoseStamped RotationShimController::supportedInitialPose()
{
  try {
    const auto now = clock_->now();
    const auto latest = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
      costmap_ros_->getBaseFrameID(), tf2::TimePointZero);
    const rclcpp::Time latest_epoch(latest.header.stamp, now.get_clock_type());
    const auto epoch = latest_epoch.nanoseconds() == 0 ? now : std::min(now, latest_epoch);
    const auto transform = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
      costmap_ros_->getBaseFrameID(), epoch, tf2::durationFromSec(0.0));
    const double age = (now - epoch).seconds();
    const auto & q = transform.transform.rotation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(age) || age < 0.0 || age > costmap_ros_->getTransformTolerance() ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    {
      throw nav2_core::NoValidControl("RotationShim initial rotation has invalid/stale pose");
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.header.stamp = epoch;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = q;
    return pose;
  } catch (const tf2::TransformException &) {
    throw nav2_core::NoValidControl("RotationShim initial rotation has no supported current pose");
  }
}

// Complete static local-map certificate for INITIAL rotation only. Each occupied
// cell is enclosed by a disc; sampling margin covers the intervening body sweep.
bool RotationShimController::initialRotationArcClear(
  const geometry_msgs::msg::PoseStamped & pose, double arc)
{
  auto * map = costmap_ros_->getCostmap();
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double x = pose.pose.position.x, y = pose.pose.position.y;
  const double resolution = map->getResolution();
  double reach = 0.0;
  if (footprint.size() < 3 || !std::isfinite(arc) || !std::isfinite(yaw) ||
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(resolution) || resolution <= 0.0)
  {
    return false;
  }
  for (const auto & pt : footprint) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {return false;}
    reach = std::max(reach, std::hypot(pt.x, pt.y));
  }
  if (pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {return false;}
  if (!std::isfinite(reach) || reach <= 0.0 || std::abs(arc) > 2.0 * M_PI) {return false;}
  const double step = 0.5 * resolution / reach;
  const double margin = resolution * (std::sqrt(0.5) + 0.25);
  unsigned int min_x, min_y, max_x, max_y;
  if (!map->worldToMap(x - reach - margin, y - reach - margin, min_x, min_y) ||
    !map->worldToMap(x + reach + margin, y + reach + margin, max_x, max_y))
  {
    return false;
  }
  const double intervals = std::ceil(std::abs(arc) / step);
  // Refuse an unsupported numerical/work bound instead of overflowing or
  // leaving an unbounded scan in the motion callback.
  if (!std::isfinite(intervals) || intervals > 100000.0) {return false;}
  const size_t count = std::max<size_t>(1, intervals);
  std::vector<double> samples;
  samples.reserve(count + 2);
  for (size_t i = 0; i <= count; ++i) {samples.push_back(arc * i / count);}
  samples.push_back(std::copysign(
    std::max(0.0, std::abs(arc) - params_->angular_disengage_threshold), arc));
  for (unsigned int mx = min_x; mx <= max_x; ++mx) {
    for (unsigned int my = min_y; my <= max_y; ++my) {
      const auto cost = map->getCost(mx, my);
      if (cost < params_->max_cost_threshold &&
        !(cost == nav2_costmap_2d::NO_INFORMATION &&
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown())) {continue;}
      double wx, wy;
      map->mapToWorld(mx, my, wx, wy);
      const double dx = wx - x, dy = wy - y;
      if (std::hypot(dx, dy) > reach + margin) {continue;}
      for (const double sample : samples) {
        const double c = std::cos(yaw + sample), sn = std::sin(yaw + sample);
        const double px = c * dx + sn * dy, py = -sn * dx + c * dy;
        bool inside = false;
        for (size_t i = 0, j = footprint.size() - 1; i < footprint.size(); j = i++) {
          const auto & a = footprint[j];
          const auto & b = footprint[i];
          if ((a.y > py) != (b.y > py) &&
            px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x) {inside = !inside;}
          const double ex = b.x - a.x, ey = b.y - a.y;
          const double length2 = ex * ex + ey * ey;
          if (length2 <= 0.0) {return false;}
          const double t = std::clamp(((px - a.x) * ex + (py - a.y) * ey) / length2, 0.0, 1.0);
          if (std::hypot(px - a.x - t * ex, py - a.y - t * ey) <= margin) {return false;}
        }
        if (inside) {return false;}
      }
    }
  }
  return true;
}

double RotationShimController::initialRotationArc(
  const geometry_msgs::msg::PoseStamped & pose, double heading)
{
  const double yaw = tf2::getYaw(pose.pose.orientation);
  if (!std::isfinite(yaw) || !std::isfinite(heading)) {
    throw nav2_core::NoValidControl("RotationShim initial rotation has invalid heading");
  }
  if (!initial_rotation_selected_) {
    const double opposite = heading - std::copysign(2.0 * M_PI, heading);
    const bool shortest_clear = initialRotationArcClear(pose, heading);
    const bool opposite_clear = initialRotationArcClear(pose, opposite);
    if (!shortest_clear && !opposite_clear) {
      throw nav2_core::NoValidControl("RotationShim initial rotation has no clear complete arc");
    }
    initial_rotation_remaining_ = shortest_clear ? heading : opposite;
    initial_rotation_target_ = yaw + heading;
    initial_rotation_previous_yaw_ = yaw;
    initial_rotation_selected_ = true;
  } else {
    // The target belongs to this initial alignment, rather than to successive
    // pruned path samples. A materially different target requires a new plan.
    if (std::abs(angles::shortest_angular_distance(initial_rotation_target_, yaw + heading)) >
      params_->angular_dist_threshold)
    {
      throw nav2_core::NoValidControl("RotationShim initial rotation target changed");
    }
    const double candidate_remaining = initial_rotation_remaining_ -
      angles::shortest_angular_distance(initial_rotation_previous_yaw_, yaw);
    // A refused update cannot change ownership/progress: an identical later
    // callback must not turn an overshoot into permission to reverse.
    if (initial_rotation_remaining_ * candidate_remaining < 0.0 &&
      std::abs(candidate_remaining) > params_->angular_disengage_threshold)
    {
      throw nav2_core::NoValidControl("RotationShim initial rotation overshot its owned arc");
    }
    if (!initialRotationArcClear(pose, candidate_remaining)) {
      throw nav2_core::NoValidControl("RotationShim selected initial arc is no longer clear");
    }
    initial_rotation_remaining_ = candidate_remaining;
    initial_rotation_previous_yaw_ = yaw;
  }
  return initial_rotation_remaining_;
}

bool RotationShimController::isGoalChanged(const geometry_msgs::msg::PoseStamped & goal)
{
  // Return true if rotating or if the goal pose is empty
''')
s = replace_once(s, '''
void RotationShimController::newPathReceived(const nav_msgs::msg::Path & raw_global_path)
{
  primary_controller_->newPathReceived(raw_global_path);
}

''', '''
void RotationShimController::newPathReceived(const nav_msgs::msg::Path & raw_global_path)
{
  initial_rotation_selected_ = false;
  if (params_->allow_alternate_initial_rotation && params_->rotate_to_heading_once) {
    in_rotation_ = false;
    raw_initial_path_ = raw_global_path;
    raw_initial_pending_ = true;
    current_goal_ = geometry_msgs::msg::PoseStamped();
  }
  primary_controller_->newPathReceived(raw_global_path);
}

''')
s = replace_once(s, '''
void RotationShimController::reset()
{
  last_angular_vel_ = std::numeric_limits<double>::max();
  primary_controller_->reset();
}
''', '''
void RotationShimController::reset()
{
  initial_rotation_selected_ = false;
  if (params_->allow_alternate_initial_rotation) {
    in_rotation_ = false;
    current_goal_ = geometry_msgs::msg::PoseStamped();
    raw_initial_pending_ = !raw_initial_path_.poses.empty();
  }
  last_angular_vel_ = std::numeric_limits<double>::max();
  primary_controller_->reset();
}
''')
h = replace_once(h, '''   * @param goal The goal to compare with the last goal.
   * @return True if the goal has changed, false otherwise.
   */
  bool isGoalChanged(const geometry_msgs::msg::PoseStamped & goal);

  nav2::LifecycleNode::WeakPtr node_;
''', '''   * @param goal The goal to compare with the last goal.
   * @return True if the goal has changed, false otherwise.
   */
  nav_msgs::msg::Path raw_initial_path_;
  bool raw_initial_pending_ = false;
  geometry_msgs::msg::PoseStamped rawInitialTarget(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & global_goal);
  geometry_msgs::msg::PoseStamped supportedInitialPose();
  bool initialRotationArcClear(const geometry_msgs::msg::PoseStamped & pose, double arc);
  double initialRotationArc(const geometry_msgs::msg::PoseStamped & pose, double heading);
  bool initial_rotation_selected_ = false;
  double initial_rotation_target_ = 0.0;
  double initial_rotation_previous_yaw_ = 0.0;
  double initial_rotation_remaining_ = 0.0;

  bool isGoalChanged(const geometry_msgs::msg::PoseStamped & goal);

  nav2::LifecycleNode::WeakPtr node_;
''')
ph_path = root / 'include/nav2_rotation_shim_controller/parameter_handler.hpp'
ph = ph_path.read_text()
ph = replace_once(ph, '''  double control_duration;
  bool rotate_to_goal_heading;
  bool rotate_to_heading_once;
  bool closed_loop;
  bool use_path_orientations;
  std::string primary_controller;
''', '''  double control_duration;
  bool rotate_to_goal_heading;
  bool rotate_to_heading_once;
  bool allow_alternate_initial_rotation;
  bool closed_loop;
  bool use_path_orientations;
  std::string primary_controller;
''')
ph_path.write_text(ph)
pc_path = root / 'src/parameter_handler.cpp'
pc = pc_path.read_text()
pc = replace_once(pc, '''    ".rotate_to_goal_heading", false);
  params_.rotate_to_heading_once = node->declare_or_get_parameter(plugin_name_ +
    ".rotate_to_heading_once", false);
  params_.closed_loop = node->declare_or_get_parameter(plugin_name_ + ".closed_loop", true);
  params_.use_path_orientations = node->declare_or_get_parameter(plugin_name_ +
    ".use_path_orientations", false);
''', '''    ".rotate_to_goal_heading", false);
  params_.rotate_to_heading_once = node->declare_or_get_parameter(plugin_name_ +
    ".rotate_to_heading_once", false);
  params_.allow_alternate_initial_rotation = node->declare_or_get_parameter(plugin_name_ +
    ".allow_alternate_initial_rotation", false);
  params_.closed_loop = node->declare_or_get_parameter(plugin_name_ + ".closed_loop", true);
  params_.use_path_orientations = node->declare_or_get_parameter(plugin_name_ +
    ".use_path_orientations", false);
''')
pc = replace_once(pc, '''    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".simulate_ahead_time" &&
''', '''    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_name == plugin_name_ + ".allow_alternate_initial_rotation") {
      result.successful = false;
      result.reason = "allow_alternate_initial_rotation is startup-only";
      return result;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".simulate_ahead_time" &&
''')
pc_path.write_text(pc)
source.write_text(s)
header.write_text(h)
PY_SHIM

cmake -S "$work/src/sophus" -B "$work/sophus-build" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_PREFIX="$prefix" -DBUILD_SOPHUS_TESTS=OFF \
  -DBUILD_SOPHUS_EXAMPLES=OFF -DSOPHUS_USE_BASIC_LOGGING=ON
cmake --build "$work/sophus-build" --parallel "${BUILD_JOBS:-3}"
cmake --install "$work/sophus-build"
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
cd "$work"
# GitHub's `ubuntu-26.04` runner is Ubuntu's amd64v3 variant: GCC 15 targets
# x86-64-v3 (AVX2) by default, which turns on Eigen code paths that GCC 15 +
# Eigen 3.4.0 do not compile warning-free, and grid_map (grid_map_cmake_helpers)
# and Nav2 (nav2_package) hardcode -Werror. Inert on baseline amd64 and arm64.
#  * -Wno-error=array-bounds: middle-end false positive on Eigen's AVX loads of
#    fixed-size vectors (grid_map::Position in GridMap.cpp); NOT suppressed by
#    system-header status. Same flag as the GTSAM recipe in ros2-ci.yml. The
#    `-Wno-error=` form is order-independent w.r.t. the later -Werror.
#  * -isystem /usr/include/eigen3: nav2_smac_planner reaches Eigen through OMPL
#    with a plain -I, so front-end warnings inside Eigen itself become errors
#    (F32ToBf16's unused `r` under EIGEN_VECTORIZE_AVX2, AVX/PacketMath.h:1275).
#    GCC de-duplicates -I against -isystem by directory identity, so this marks
#    Eigen a system header everywhere in the vendor build — exactly what
#    grid_map already gets through its SYSTEM include export.
MAKEFLAGS="-j${BUILD_JOBS:-3}" colcon build --merge-install --install-base "$prefix" \
  --base-paths src/grid_map src/beluga/beluga src/beluga/beluga_ros src/navigation2/nav2_smac_planner \
  src/navigation2/nav2_rotation_shim_controller \
  --packages-up-to grid_map_ros beluga_ros nav2_smac_planner nav2_rotation_shim_controller --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  "-DCMAKE_CXX_FLAGS=-Wno-error=array-bounds -isystem /usr/include/eigen3"
