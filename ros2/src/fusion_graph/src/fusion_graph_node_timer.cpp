// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — OnTimer (per-tick build/publish) + LatLonToMap. (The node implementation is
// split across several translation units to keep each file within the project's 600-line budget;
// all share fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

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

void FusionGraphNode::OnTimer()
{
  const double now_s = this->now().seconds();

  // Boot dock-seed fallback. The normal init paths are (a) is_charging →
  // SeedFromDockPose, and (b) GPS xy + COG/mag yaw → TrySeedInitialPose.
  // Both can silently fail on a fresh boot on the dock: is_charging may
  // never be delivered (degraded DDS discovery), and a parked chassis
  // produces no COG heading, so seed_yaw_ stays empty. When that happens
  // the graph never initializes, no map→odom TF is published, and Nav2's
  // planner_server aborts activation (global_costmap can't get map→base),
  // taking the whole lifecycle bringup down with it. If we're still
  // uninitialized a few seconds after boot, fall back to the calibrated
  // dock pose — the robot normally boots parked on the dock. Suppressed
  // when hardware status has been seen AND reports NOT charging, so a
  // robot that boots away from the dock is left to GPS-based init.
  constexpr double kDockSeedFallbackS = 6.0;
  if (!graph_->IsInitialized())
  {
    if (boot_stamp_s_ < 0.0)
    {
      boot_stamp_s_ = now_s;
    }
    else if (!dock_seed_fallback_done_ && (now_s - boot_stamp_s_) > kDockSeedFallbackS &&
             (!last_is_charging_valid_ || last_is_charging_))
    {
      RCLCPP_WARN(get_logger(),
                  "fusion_graph: still uninitialized %.1fs after boot "
                  "(is_charging/COG unavailable) — seeding from dock pose",
                  now_s - boot_stamp_s_);
      SeedFromDockPose();
      dock_seed_fallback_done_ = true;
    }
  }

  // Poll background tile I/O even with no incoming scans (save/clear completion).
  if (lidar_submaps_)
  {
    const auto snap = graph_->LatestSnapshot();
    PollLidarSubmaps(snap ? snap->pose.x() : dock_pose_x_, snap ? snap->pose.y() : dock_pose_y_);
  }

  // Consume each acquisition once. The IMU history supplies its odometry and
  // the offset to an existing graph node, independently of timer cadence.
  std::vector<Eigen::Vector2d> curr_scan;
  bool curr_valid = false;
  double scan_stamp = 0.0;
  const auto continuous = lidar_anchor_odom_.pose();
  const gtsam::Pose2 odom_now(continuous.translation().x(),
                              continuous.translation().y(),
                              continuous.so2().log());
  {
    std::lock_guard<std::mutex> lock(scan_mu_);
    if (latest_scan_valid_)
    {
      curr_scan = latest_scan_;
      curr_valid = true;
      scan_stamp = latest_scan_stamp_s_;
    }
  }

  const auto scan_time = lidar_scan_history_.At(scan_stamp, now_s, lidar_scan_max_age_s_);
  curr_valid = curr_valid && scan_stamp > consumed_scan_stamp_s_ && scan_time &&
               graph_->GetPose(scan_time->index).has_value();
  if (curr_valid)
  {
    consumed_scan_stamp_s_ = scan_stamp;
    lidar_scan_stamp_s_ = scan_stamp;
    lidar_scan_dr_ = Sophus::SE2d(scan_time->odom.theta(), scan_time->odom.translation());
  }

  // ── LiDAR map anchor (Beluga): absolute XY during a GNSS outage ───────
  // Builds the georeferenced occupancy grid under fresh RTK-Fixed. A usable
  // GNSS position, including Float, keeps the filter asleep; only a prolonged
  // absence lets it localise against the grid and queue an XY-only prior. See
  // fusion_graph_node_lidar_anchor.cpp.
  if (use_lidar_map_anchor_ && curr_valid)
  {
    LidarMapAnchorStep(curr_scan, *scan_time);
  }

  // A prior queued during a throttled tick must yield immediately to any
  // usable GNSS observation, including when no new scan arrives to run the
  // anchor step.
  if (UsableGnssReceiptIsFresh(lidar_anchor_engage_age_s_) || !last_is_charging_valid_ ||
      last_is_charging_)
    graph_->ClearLidarObservations();
  auto out = graph_->Tick(now_s);
  if (out)
  {
    if (last_imu_stamp_)
      lidar_scan_history_.AddNode(last_imu_stamp_->seconds(), out->node_index, odom_now);
  }

  // Local-frame DR (odom→base_footprint TF + /odometry/filtered) is
  // always published — it's independent of graph state. Nav2's local
  // costmap and FTCController need this TF before any GPS fix arrives,
  // and before the graph has been initialized.
  PublishLocalOdom();

  // Map-frame outputs (map→odom TF + /odometry/filtered_map). Two
  // jobs here:
  //   1. When a new node lands, recompute the constant T_map_odom
  //      anchor — see fusion_graph_node.hpp for the why. This is the
  //      only point where the anchor can be captured against a fresh
  //      dr_* (the same OnTimer invocation that just ran Tick); doing
  //      it later races subsequent OnImu integration.
  //   2. Re-broadcast TF + /odometry/filtered_map every OnTimer with
  //      that anchor extrapolated through the current dr_*. Keeping
  //      the publish rate at OnTimer cadence (vs. only on new-node)
  //      stops Nav2 from rejecting stale lookups during stationary
  //      windows.
  if (auto snap = graph_->LatestSnapshot())
  {
    if (!t_map_odom_anchor_valid_ || snap->node_index != last_anchored_node_index_)
    {
      // tf_state_mu_: the anchor VALUE and valid flag are read together by
      // TfBroadcastLoop; write the {value, valid=true} pair atomically so the
      // loop never composes a stale anchor with fresh dr_*. dr_* is read here
      // too (it is concurrently integrated by OnImu).
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      const gtsam::Pose2 dr_at_node(dr_x_, dr_y_, dr_yaw_);
      t_map_odom_anchor_ = snap->pose.compose(dr_at_node.inverse());
      last_anchored_node_index_ = snap->node_index;
      t_map_odom_anchor_valid_ = true;
    }
    // Odom re-base: once the robot has driven odom_rebase_dist_m from the odom
    // origin, reset the odom POSITION onto the robot (heading kept) and shift
    // the anchor so map→base is unchanged. This keeps the lever arm |dr| small
    // so graph-yaw jitter can't rotate it into large map→odom position steps.
    // OnImu (same executor thread) can't interrupt; the lock guards the TF
    // reader thread. force_pub_resync_ makes the slew snap the published anchor
    // to the new (coordinated) target so map→base stays continuous.
    if (odom_rebase_dist_m_ > 0.0 && t_map_odom_anchor_valid_)
    {
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      if (std::hypot(dr_x_, dr_y_) > odom_rebase_dist_m_)
      {
        const gtsam::Pose2 map_base =
            t_map_odom_anchor_.compose(gtsam::Pose2(dr_x_, dr_y_, dr_yaw_));
        dr_x_ = 0.0;
        dr_y_ = 0.0;
        lidar_anchor_odom_.RebaseRaw(Sophus::SE2d(dr_yaw_, Eigen::Vector2d::Zero()));
        t_map_odom_anchor_ = map_base.compose(gtsam::Pose2(0.0, 0.0, dr_yaw_).inverse());
        force_pub_resync_.store(true, std::memory_order_release);
      }
    }
    PublishOutputs(*snap);
  }
}

// ── Helpers ───────────────────────────────────────────────────────────

void FusionGraphNode::LatLonToMap(double lat, double lon, double& x, double& y) const
{
  const double dlat = (lat - datum_lat_) * M_PI / 180.0;
  const double dlon = (lon - datum_lon_) * M_PI / 180.0;
  x = kEarthRadius * datum_cos_lat_ * dlon;  // east
  y = kEarthRadius * dlat;  // north
}

}  // namespace fusion_graph
