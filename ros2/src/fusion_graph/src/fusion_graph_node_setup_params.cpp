// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — DeclareParameters — declare/latch all ROS parameters. (The node implementation
// is split across several translation units to keep each file within the project's 600-line budget;
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

void FusionGraphNode::DeclareParameters()
{
  // ── Dock pose seed (always declared) ────────────────────────────
  // Read from mowgli_robot.yaml — calibrate_imu_yaw_node and the
  // map_server /set_docking_point service write back to that file,
  // so the values here are always the latest persisted dock anchor.
  // Declared outside the scan_matching gate because SeedFromDockPose()
  // is the only seed path that fires while the robot boots docked
  // and stationary (COG needs motion, mag is off by default) — the
  // no-LiDAR config must still be able to bootstrap.
  dock_pose_x_ = declare_parameter<double>("dock_pose_x", 0.0);
  dock_pose_y_ = declare_parameter<double>("dock_pose_y", 0.0);
  dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);
  dock_pose_yaw_sigma_rad_ = declare_parameter<double>("dock_pose_yaw_sigma_rad", 0.035);

  // ── Scan matching (optional) ─────────────────────────────────────
  use_scan_matching_ = declare_parameter<bool>("use_scan_matching", false);
  if (use_scan_matching_)
  {
    ScanMatcherParams sp;
    // 10 iters converges within 1 mm of the 15-iter solution on the
    // outdoor LiDAR shapes we see; the extra 5 iters were CPU sink.
    sp.max_iterations = declare_parameter<int>("icp_max_iter", 10);
    sp.max_correspondence_dist = declare_parameter<double>("icp_max_corresp_dist", 0.5);
    // 40 source points keeps ICP rmse within a few mm of the 60-pt
    // result while halving inner-loop NN cost. ARM hot-path saving.
    sp.source_subsample = static_cast<size_t>(declare_parameter<int>("icp_source_subsample", 40));
    sp.sigma_xy_base = declare_parameter<double>("icp_sigma_xy_base", 0.02);
    sp.sigma_theta_base = declare_parameter<double>("icp_sigma_theta_base", 0.005);
    scan_matcher_ = std::make_unique<ScanMatcher>(sp);

    // Per-tick ICP guard rails (see fusion_graph_node.hpp comments).
    icp_max_rmse_m_ = declare_parameter<double>("icp_max_rmse_m", 0.10);
    icp_max_delta_xy_m_ = declare_parameter<double>("icp_max_delta_xy_m", 0.30);
    icp_max_delta_theta_rad_ = declare_parameter<double>("icp_max_delta_theta_rad", 0.50);
    icp_max_divergence_xy_m_ = declare_parameter<double>("icp_max_divergence_xy_m", 0.15);
    icp_max_divergence_theta_rad_ = declare_parameter<double>("icp_max_divergence_theta_rad", 0.35);

    // Yield-to-RTK gating (see fusion_graph_node.hpp). When RTK-Fixed is
    // fresh, inflate the scan-between σ so GPS dominates and map→odom stays
    // pinned; scan-matching only carries the estimate once the fix is lost.
    scan_yield_to_rtk_ = declare_parameter<bool>("scan_yield_to_rtk", true);
    scan_yield_timeout_s_ = declare_parameter<double>("scan_yield_timeout_s", 2.0);
    scan_yield_sigma_xy_ = declare_parameter<double>("scan_yield_sigma_xy", 0.5);
    scan_yield_sigma_theta_ = declare_parameter<double>("scan_yield_sigma_theta", 0.3);

    // ── RTK-anchored keyframe map (absolute scan-to-keyframe localization) ──
    // Requires scan matching (this block). Default OFF. CAPTURE under stable
    // RTK-Fixed; APPLY a PoseTranslationPrior during RTK-Float to hold <2 cm.
    use_keyframe_map_ = declare_parameter<bool>("use_keyframe_map", false);
    kf_capture_sigma_max_m_ = declare_parameter<double>("kf_capture_sigma_max_m", 0.01);
    kf_capture_rtk_debounce_ = declare_parameter<int>("kf_capture_rtk_debounce", 3);
    kf_capture_max_omega_ = declare_parameter<double>("kf_capture_max_omega", 0.10);
    kf_match_max_dist_m_ = declare_parameter<double>("kf_match_max_dist_m", 3.0);
    kf_max_candidates_ = static_cast<size_t>(declare_parameter<int>("kf_max_candidates", 5));
    kf_apply_sigma_floor_m_ = declare_parameter<double>("kf_apply_sigma_floor_m", 0.02);
    kf_engage_age_s_ = declare_parameter<double>("kf_engage_age_s", 0.3);
  }

  // 180° yaw-flip recovery (see fusion_graph_node.hpp). Declared outside the
  // scan_matching gate — it keys off COG, not LiDAR.
  cog_flip_recovery_enabled_ = declare_parameter<bool>("cog_flip_recovery_enabled", true);
  cog_flip_threshold_rad_ = declare_parameter<double>("cog_flip_threshold_rad", 2.618);
  cog_flip_consecutive_n_ = declare_parameter<int>("cog_flip_consecutive_n", 3);
  cog_flip_require_rtk_ = declare_parameter<bool>("cog_flip_require_rtk", true);
  cog_flip_min_interval_s_ = declare_parameter<double>("cog_flip_min_interval_s", 10.0);
  cog_flip_consistency_rad_ = declare_parameter<double>("cog_flip_consistency_rad", 0.52);

  // ── Magnetometer (off by default) ───────────────────────────────
  // Motors near the chassis induce a heading-dependent bias on the
  // magnetometer that no static cal can remove (see CLAUDE.md
  // history). Default off so the graph never sees mag samples;
  // operators with a motor-isolated mag hardware setup can flip the
  // flag on at launch.
  use_magnetometer_ = declare_parameter<bool>("use_magnetometer", false);

  // Primary vs observer. Defaults to true for back-compat with the
  // standalone test harness; navigation.launch.py overrides to false
  // when no persisted graph exists yet (first session) so ekf_map
  // keeps driving Nav2 while fusion_graph builds the graph silently.
  primary_mode_ = declare_parameter<bool>("primary_mode", true);

  // ── Loop closure + persistence ───────────────────────────────────
  loop_closure_enabled_ = declare_parameter<bool>("use_loop_closure", false);
  lc_max_dist_m_ = declare_parameter<double>("lc_max_dist_m", 5.0);
  // Default 600s (10 min): stationary clutter at the dock or during
  // long IDLE windows produces O(N²) LC factors with the lower 30/120s
  // defaults. Real revisits across a mowing pattern are minutes apart,
  // so 600s is a comfortable floor. Override per-test if needed.
  lc_min_age_s_ = declare_parameter<double>("lc_min_age_s", 600.0);
  lc_max_candidates_ = static_cast<size_t>(declare_parameter<int>("lc_max_candidates", 3));
  lc_min_delta_m_ = declare_parameter<double>("lc_min_delta_m", 0.05);
  lc_min_delta_theta_ = declare_parameter<double>("lc_min_delta_theta", 0.05);
  // 0.10 m rmse rejected too aggressively in field tests: outdoor
  // LiDAR scans of the same place separated by minutes typically
  // see ~15-25 cm point-wise rmse from wind / shadow / dynamic
  // obstacles, even when the relative pose delta is sub-cm. The
  // BetweenFactor noise scales with rmse anyway (sigma_xy_base +
  // sigma_xy_scale * rmse), so a noisier match enters the graph
  // with proportionally lower weight rather than being dropped.
  lc_max_rmse_ = declare_parameter<double>("lc_max_rmse", 0.20);
  lc_sigma_xy_ = declare_parameter<double>("lc_sigma_xy", 0.05);
  lc_sigma_theta_ = declare_parameter<double>("lc_sigma_theta", 0.02);
  // Skip loop-closure generation while RTK-Fixed is fresh (see the member doc in
  // fusion_graph_node.hpp). This is the bound that stops the stationary-dwell
  // factor leak that OOM-killed the node; leave it on unless a site genuinely
  // mows under permanent RTK-Float and needs LC even near the dock.
  lc_skip_when_rtk_fixed_ = declare_parameter<bool>("lc_skip_when_rtk_fixed", true);

  graph_save_prefix_ =
      declare_parameter<std::string>("graph_save_prefix", "/ros2_ws/maps/fusion_graph");

  scan_retention_nodes_ =
      static_cast<uint64_t>(declare_parameter<int>("scan_retention_nodes", 18000));
  isam2_rebase_every_nodes_ =
      static_cast<uint64_t>(declare_parameter<int>("isam2_rebase_every_nodes", 2000));
  const bool autoload = declare_parameter<bool>("autoload_graph", true);

  // RTK-Fixed override of the autoloaded pose: if the autoloaded graph
  // disagrees with the first incoming RTK-Fixed sample by more than this
  // many metres, force a re-anchor at the GPS pose. Handles the case of
  // booting away from the dock — the persisted graph's last node is
  // typically the dock, so without this the published map→odom would
  // claim the robot is on the dock until the optimizer slowly walks the
  // trajectory over.
  rtk_autoload_override_threshold_m_ =
      declare_parameter<double>("rtk_autoload_override_threshold_m", 0.3);

  if (autoload)
  {
    if (graph_->Load(graph_save_prefix_))
    {
      autoload_succeeded_ = true;
      RCLCPP_INFO(get_logger(),
                  "fusion_graph: loaded persisted graph from '%s.*'",
                  graph_save_prefix_.c_str());
    }
  }

  // ── Auto-checkpoint configuration ───────────────────────────────
  // Persist the graph automatically on:
  //   - transition out of HIGH_LEVEL_STATE_RECORDING (the area
  //     polygon was just saved by the GUI; we want the matching pose
  //     graph + scans to land alongside it)
  //   - rising edge of is_charging (robot just docked; safe checkpoint
  //     before any potential power loss)
  //   - periodic timer during AUTONOMOUS state (default 5 min)
  // Set auto_save_enabled to false to keep checkpoints fully manual
  // via the ~/save_graph service.
  auto_save_enabled_ = declare_parameter<bool>("auto_save_enabled", true);
}

}  // namespace fusion_graph
