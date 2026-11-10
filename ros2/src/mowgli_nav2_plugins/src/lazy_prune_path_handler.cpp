// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0 AND Apache-2.0
//
// Portions of this file are derived from Nav2
// (nav2_controller/plugins/feasible_path_handler.{hpp,cpp}, Nav2 1.5.2) and keep its notices:
//
//   Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//   Copyright (c) 2023 Dexory
//   Copyright (c) 2023 Open Navigation LLC
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
// Modified for MowgliNext: pruning keeps the passed poses behind an offset (see below).
//
// LazyPrunePathHandler — implementation. See the header for the why.

#include "mowgli_nav2_plugins/lazy_prune_path_handler.hpp"

#include <iterator>
#include <mutex>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mowgli_nav2_plugins
{

using nav2_util::geometry_utils::euclidean_distance;

void LazyPrunePathHandler::setPlan(const nav_msgs::msg::Path& path)
{
  FeasiblePathHandler::setPlan(path);
  std::lock_guard<std::mutex> lock(mutex_);
  pruned_ = 0;
}

void LazyPrunePathHandler::compact()
{
  auto& poses = global_plan_up_to_constraint_.poses;
  poses.erase(poses.begin(), poses.begin() + static_cast<std::ptrdiff_t>(pruned_));
  pruned_ = 0;
}

nav2_core::PathSegment LazyPrunePathHandler::findPlanSegment(
    const geometry_msgs::msg::PoseStamped& pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto& poses = global_plan_up_to_constraint_.poses;
  // Upstream's size checks (here and in transformToGlobalPlanFrame) read the
  // physical vector. With fewer than two live poses, make it the logical one.
  // (Upstream never prunes a plan below two poses, so this does not happen.)
  if (poses.size() - pruned_ < 2)
  {
    compact();
  }
  global_pose_ = transformToGlobalPlanFrame(pose);

  const auto begin = poses.begin() + static_cast<std::ptrdiff_t>(pruned_);
  // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
  auto closest_pose_upper_bound =
      nav2_util::geometry_utils::first_after_integrated_distance(begin,
                                                                 poses.end(),
                                                                 max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto closest_point =
      nav2_util::geometry_utils::min_by(begin,
                                        closest_pose_upper_bound,
                                        [this](const geometry_msgs::msg::PoseStamped& ps)
                                        {
                                          return euclidean_distance(global_pose_, ps);
                                        });

  // Do not prune the global plan below 2 points, so there are enough points to interpolate the
  // end-of-path direction.
  if (poses.size() - pruned_ > 1 && closest_point == std::prev(poses.end()))
  {
    closest_point = std::prev(closest_point);
  }

  auto pruned_plan_end =
      nav2_util::geometry_utils::first_after_integrated_distance(closest_point,
                                                                 poses.end(),
                                                                 prune_distance_);

  return {closest_point, pruned_plan_end};
}

nav_msgs::msg::Path LazyPrunePathHandler::transformLocalPlan(
    const nav2_core::PathIterator& closest_point, const nav2_core::PathIterator& pruned_plan_end)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  transformed_plan.header.stamp = global_pose_.header.stamp;
  unsigned int mx, my;
  // Find the furthest relevant pose on the path to consider within costmap
  // bounds
  // Transforming it to the costmap frame in the same loop
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *(costmap_ros_->getCostmap()->getMutex()));
  for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
       ++global_plan_pose)
  {
    // Transform from global plan frame to costmap frame
    geometry_msgs::msg::PoseStamped costmap_plan_pose;
    global_plan_pose->header.stamp = global_pose_.header.stamp;
    global_plan_pose->header.frame_id = global_plan_.header.frame_id;
    nav2_util::transformPoseInTargetFrame(*global_plan_pose,
                                          costmap_plan_pose,
                                          *tf_,
                                          costmap_ros_->getGlobalFrameID(),
                                          transform_tolerance_);

    // Check if pose is inside the costmap
    if (!costmap_ros_->getCostmap()->worldToMap(
            costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
    {
      break;
    }

    // Filling the transformed plan to return with the transformed pose
    transformed_plan.poses.push_back(costmap_plan_pose);
  }

  // Upstream erases [begin, closest_point) here. Mark it passed instead, and
  // erase only once the passed poses are the larger half: every pose is then
  // moved at most once per halving, O(1) amortized per cycle.
  auto& poses = global_plan_up_to_constraint_.poses;
  pruned_ = static_cast<std::size_t>(closest_point - poses.begin());
  if (2 * pruned_ > poses.size())
  {
    compact();
  }

  if ((enforce_path_inversion_ || enforce_path_rotation_) && constraint_locale_ != 0u)
  {
    // The constraint branch reads and replaces the pruned plan: give it the
    // vector upstream would have.
    compact();
    if (isWithinInversionTolerances(global_pose_))
    {
      prunePlan(global_plan_, global_plan_.poses.begin() + constraint_locale_);
      global_plan_up_to_constraint_ = global_plan_;
      constraint_locale_ = nav2_util::removePosesAfterFirstConstraint(global_plan_up_to_constraint_,
                                                                      enforce_path_inversion_,
                                                                      minimum_rotation_angle_);
    }
  }

  if (transformed_plan.poses.empty())
  {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

}  // namespace mowgli_nav2_plugins

PLUGINLIB_EXPORT_CLASS(mowgli_nav2_plugins::LazyPrunePathHandler, nav2_core::PathHandler)
