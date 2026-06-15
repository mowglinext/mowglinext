// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — dock-pose seed + periodic save. (The node implementation is split across
// several translation units to keep each file within the project's 600-line budget; all share
// fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

void FusionGraphNode::SeedFromDockPose()
{
  // Build a Pose2 from the dock_pose_* parameters and route it
  // through the same Initialize/ForceAnchor path that OnSetPose uses.
  // Using the persisted dock pose (vs. live GPS) makes re-docking
  // deterministic: the dock physically doesn't move, but RTK-Float /
  // multipath / lever-arm yaw error / wrong-fix can drift the live
  // GPS by 1-10 cm. Seeding from the stored dock pose treats the
  // charging signal as ground truth on the robot's location;
  // GnssLeverArmFactor observations then pull the trajectory back
  // toward GPS over the next few graph nodes.
  const gtsam::Pose2 pose(dock_pose_x_, dock_pose_y_, dock_pose_yaw_);
  const double sigma_xy = 0.10;  // 10 cm — robot is physically on the dock
  const double sigma_theta = std::max(dock_pose_yaw_sigma_rad_, 0.035);

  if (!graph_->IsInitialized())
  {
    graph_->Initialize(pose, this->now().seconds());
    t_map_odom_anchor_valid_ = false;
    RCLCPP_INFO(get_logger(),
                "fusion_graph: bootstrap init from dock pose "
                "(%.2f, %.2f, %.1f°)",
                pose.x(),
                pose.y(),
                pose.theta() * 180.0 / M_PI);
    return;
  }

  auto snap = graph_->LatestSnapshot();
  if (!snap)
    return;

  // Gauge reset: rigid-transform the entire trajectory so the latest
  // node lands exactly on dock_pose, instead of merely posting a
  // loose prior that gets absorbed by the accumulated chain of
  // between-factors.
  //
  // Rationale: a PriorFactor at σ≈10 cm on a single node is 400×
  // weaker than each of the ~7000 wheel between-factors (σ≈5 mm) in
  // the persisted chain, so the optimizer leaves the latest node
  // close to where the chain says it is — typically 10-30 cm off the
  // operator-calibrated dock_pose due to gyro-bias drift accumulated
  // without LiDAR loop closures. The rigid transform shifts every
  // pose by the same SE(2) correction, leaving every between-factor
  // residual unchanged (they're gauge-invariant) but realigning the
  // global frame so X_latest == dock_pose exactly.
  //
  // Below threshold (5 cm) we keep the cheap one-shot ForceAnchor —
  // the residual is small enough that the loose prior can absorb it,
  // and we avoid a full rebuild every is_charging tick.
  const double dx = pose.x() - snap->pose.x();
  const double dy = pose.y() - snap->pose.y();
  const double offset = std::hypot(dx, dy);
  constexpr double kRigidTransformThresholdM = 0.05;
  if (offset > kRigidTransformThresholdM)
  {
    const gtsam::Pose2 correction = pose.compose(snap->pose.inverse());
    graph_->RigidTransformAll(correction, sigma_xy, sigma_theta);
    RCLCPP_INFO(get_logger(),
                "fusion_graph: gauge reset to dock (%.2f, %.2f, %.1f°) — "
                "rigid-transformed %.2f m offset",
                pose.x(),
                pose.y(),
                pose.theta() * 180.0 / M_PI,
                offset);
  }
  else
  {
    graph_->ForceAnchor(snap->node_index, pose, sigma_xy, sigma_theta);
    RCLCPP_INFO(get_logger(),
                "fusion_graph: re-anchored at dock (%.2f, %.2f, %.1f°)",
                pose.x(),
                pose.y(),
                pose.theta() * 180.0 / M_PI);
  }
  // Re-base the dead-reckoning frame (fix C). The robot is parked on
  // the dock, so an odom→base discontinuity here is harmless (Nav2
  // isn't navigating). Collapsing dr_* to zero bounds the map→odom
  // lever arm every docking cycle: without it the odom frame keeps
  // whatever it drifted to during the session (metres), and the next
  // session starts with that same lever arm amplifying graph-vs-DR
  // yaw error into map-pose jumps. Anchor is invalidated just below
  // so the next OnTimer recomputes map→odom against the zeroed dr_*.
  {
    // tf_state_mu_: dr_* / anchor are read concurrently by TfBroadcastLoop.
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    dr_x_ = 0.0;
    dr_y_ = 0.0;
    dr_yaw_ = 0.0;
    t_map_odom_anchor_valid_ = false;
  }
  // Latch the RTK-Fixed override one-shot so it doesn't fire later if
  // the robot undocks mid-session — same rationale as OnSetPose: the
  // dock-pose seed we just applied is the operator's authoritative
  // anchor, GPS observations shouldn't fight it. (The boot-time gate
  // in OnGnss also defers the override while is_charging, but this
  // catches the case where SeedFromDockPose fires from
  // OnHardwareStatus before OnGnss runs.)
  rtk_autoload_override_done_ = true;
}

void FusionGraphNode::OnPeriodicSaveTimer()
{
  // Only checkpoint while autonomously mowing — saving on the dock
  // is already handled by OnHardwareStatus, and saving while idle
  // wastes I/O on a graph that's not changing.
  constexpr uint8_t kAutonomous =
      mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
  if (!last_hl_state_valid_ || last_hl_state_ != kAutonomous)
    return;
  if (!graph_->IsInitialized())
    return;
  DispatchAsyncSave("periodic");
}

}  // namespace fusion_graph
