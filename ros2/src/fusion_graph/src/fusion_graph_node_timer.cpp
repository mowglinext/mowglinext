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
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

void FusionGraphNode::OnTimer()
{
  ObserveMotionClockEpoch();
  const double now_s = this->now().seconds();
  // Mature accepted COG recovery checks before their queued yaw factors can
  // change the source estimate used by the recovery decision.
  ProcessPendingCogFlipChecks();
  TrySeedInitialPose();
  // Clear or Core failure can consume the original dock status edge. Retry
  // only while the live status still says charging; no dock pose is queued
  // into the unrelated explicit-pose path across an undock.
  if (!graph_->IsInitialized() && integrated_source_ns_ && last_is_charging_valid_ &&
      last_is_charging_ && gps_seen_once_)
  {
    SeedFromDockPose();
    dock_seeded_this_session_ = graph_->IsInitialized();
  }

  // Boot dock-seed fallback. The normal init paths are (a) is_charging →
  // SeedFromDockPose, and (b) GPS xy + COG/mag yaw → TrySeedInitialPose.
  // A parked chassis produces no COG heading, and charging may be confirmed
  // before GPS arrives. Retain the no-GNSS dock fallback for that known state.
  // Unknown hardware status is not evidence that the robot is on the dock:
  // wait for a real position/heading seed instead of inventing a dock prior.
  constexpr double kDockSeedFallbackS = 6.0;
  if (!graph_->IsInitialized())
  {
    if (boot_stamp_s_ < 0.0)
    {
      boot_stamp_s_ = now_s;
    }
    else if (!dock_seed_fallback_done_ && (now_s - boot_stamp_s_) > kDockSeedFallbackS &&
             last_is_charging_valid_ && last_is_charging_)
    {
      RCLCPP_WARN(get_logger(),
                  "fusion_graph: still uninitialized %.1fs after boot "
                  "(charging confirmed, GNSS/heading unavailable) — seeding from dock pose",
                  now_s - boot_stamp_s_);
      SeedFromDockPose();
      dock_seed_fallback_done_ = graph_->IsInitialized();
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
  const auto out = integrated_source_ns_ && !motion_source_fault_
                       ? graph_->TickSource(this->now().nanoseconds(),
                                            *integrated_source_ns_,
                                            integration_epoch_)
                       : std::nullopt;
  if (out)
  {
    // GNSS-only revisions also return an output, but cannot re-label current
    // scan odometry as the older graph snapshot's source state.
    if (out->source_epoch == integration_epoch_ && out->source_stamp_ns == integrated_source_ns_ &&
        out->node_index != last_lidar_source_node_index_)
    {
      lidar_scan_history_.AddNode(static_cast<double>(out->source_stamp_ns) * 1e-9,
                                  out->node_index,
                                  odom_now);
      last_lidar_source_node_index_ = out->node_index;
    }
  }

  // Local odometry also streams before graph initialization once an actual
  // source interval has been consumed; a timer supplies no new pose epoch.

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
    if (!t_map_odom_anchor_valid_ || snap->node_index != last_anchored_node_index_ ||
        snap->revision != last_anchored_revision_)
    {
      const auto dr_at_node = DrPoseAtSource(snap->source_epoch, snap->source_stamp_ns);
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      if (dr_at_node)
      {
        t_map_odom_anchor_ = snap->pose.compose(dr_at_node->inverse());
        last_anchored_node_index_ = snap->node_index;
        last_anchored_revision_ = snap->revision;
        anchored_dr_source_ns_ = snap->source_stamp_ns;
        t_map_odom_anchor_valid_ = true;
      }
      else
        t_map_odom_anchor_valid_ = false;
    }
    // Bound internal numeric coordinates while the published odom gauge and
    // source snapshots remain continuous.
    if (odom_rebase_dist_m_ > 0.0 && t_map_odom_anchor_valid_)
    {
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      if (std::hypot(dr_x_, dr_y_) > odom_rebase_dist_m_)
      {
        RecenterDrLocked(/*reset_heading=*/false);
      }
    }
    PublishLocalOdom();
    PublishOutputs(*snap);
  }
  else
  {
    // Core can fail closed internally without a new integration epoch. Clear
    // every map-pose cache when that removes its snapshot.
    pose_extrap_ = PoseExtrapolator();
    fast_pose_source_ns_.reset();
    {
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      t_map_odom_anchor_valid_ = false;
      t_map_odom_pub_shared_valid_ = false;
    }
    PublishLocalOdom();
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
