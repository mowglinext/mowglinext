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
    // Inlier floor for the scan-to-scan between-factor and the loop-closure /
    // cold-boot matches (near-total overlap, so 30/40 subsampled points is easy).
    sp.min_inliers = declare_parameter<int>("scan_min_inliers", 30);
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

    // LiDAR map anchor (Beluga particle filter against a grid built under
    // RTK-Fixed). Off by default until the field A/B validates it.
    use_lidar_map_anchor_ = declare_parameter<bool>("use_lidar_map_anchor", false);
    lidar_map_resolution_m_ = declare_parameter<double>("lidar_map_resolution_m", 0.10);
    lidar_map_half_extent_m_ = declare_parameter<double>("lidar_map_half_extent_m", 40.0);
    lidar_map_insert_period_s_ = declare_parameter<double>("lidar_map_insert_period_s", 0.5);
    lidar_map_rebuild_period_s_ = declare_parameter<double>("lidar_map_rebuild_period_s", 5.0);
    // 1.0 s, not 0.3: a 5 Hz receiver whose stamps arrive ~80 ms old sits at
    // ~0.28 s of age just before every next fix — 0.3 flapped on timer phase.
    lidar_anchor_engage_age_s_ = declare_parameter<double>("lidar_anchor_engage_age_s", 1.0);
    lidar_anchor_disengage_dwell_s_ =
        declare_parameter<double>("lidar_anchor_disengage_dwell_s", 1.0);
    lidar_anchor_max_beams_ = declare_parameter<int>("lidar_anchor_max_beams", 60);
    lidar_anchor_min_particles_ = declare_parameter<int>("lidar_anchor_min_particles", 300);
    lidar_anchor_max_particles_ = declare_parameter<int>("lidar_anchor_max_particles", 1500);
    lidar_anchor_update_min_d_ = declare_parameter<double>("lidar_anchor_update_min_d", 0.05);
    lidar_anchor_update_min_a_ = declare_parameter<double>("lidar_anchor_update_min_a", 0.05);
    lidar_anchor_seed_sigma_xy_m_ = declare_parameter<double>("lidar_anchor_seed_sigma_xy_m", 0.10);
    lidar_anchor_seed_sigma_theta_rad_ =
        declare_parameter<double>("lidar_anchor_seed_sigma_theta_rad", 0.10);
    lidar_anchor_z_hit_ = declare_parameter<double>("lidar_anchor_z_hit", 0.7);
    lidar_anchor_z_rand_ = declare_parameter<double>("lidar_anchor_z_rand", 0.3);
    lidar_anchor_sigma_hit_m_ = declare_parameter<double>("lidar_anchor_sigma_hit_m", 0.15);
    lidar_anchor_max_laser_distance_m_ =
        declare_parameter<double>("lidar_anchor_max_laser_distance_m", 12.0);
    lidar_anchor_odom_alpha_rot_ = declare_parameter<double>("lidar_anchor_odom_alpha_rot", 0.05);
    lidar_anchor_odom_alpha_trans_ =
        declare_parameter<double>("lidar_anchor_odom_alpha_trans", 0.05);
    // AMCL's augmented-MCL recovery (alpha_slow / alpha_fast > 0) injects
    // RANDOM particles across the whole map whenever the average weight drops
    // — on open lawn it drops every time, the injected particles that land
    // near saturated clutter (the terrace) win, and the estimate teleports:
    // replay 2026-09-07 showed a 2.7 m jump on the FIRST update after a
    // 0.10 m seed. Our seed is always trustworthy (fused pose or dead
    // reckoning), so global relocalisation is never wanted: both OFF.
    lidar_anchor_alpha_slow_ = declare_parameter<double>("lidar_anchor_alpha_slow", 0.0);
    lidar_anchor_alpha_fast_ = declare_parameter<double>("lidar_anchor_alpha_fast", 0.0);
    lidar_anchor_selective_resampling_ =
        declare_parameter<bool>("lidar_anchor_selective_resampling", true);
    // Per-estimate trust. Defaults sized on the 2026-09-06 replay: the lost
    // filter scored 0.13 while a healthy one scores ≥ 0.9; wheels + gyro
    // drifted < 0.4 m over 2.7 min and a U-turn.
    lidar_anchor_validator_.min_hit_ratio =
        declare_parameter<double>("lidar_anchor_min_hit_ratio", 0.5);
    lidar_anchor_validator_.min_hit_count =
        declare_parameter<int>("lidar_anchor_min_hit_count", 30);
    lidar_anchor_validator_.max_sigma_m =
        declare_parameter<double>("lidar_anchor_max_sigma_m", 0.5);
    lidar_anchor_validator_.dr_budget_m =
        declare_parameter<double>("lidar_anchor_dr_budget_m", 0.3);
    lidar_anchor_validator_.dr_drift_frac =
        declare_parameter<double>("lidar_anchor_dr_drift_frac", 0.02);
    lidar_anchor_reseed_after_s_ = declare_parameter<double>("lidar_anchor_reseed_after_s", 5.0);
    lidar_anchor_shadow_mode_ = declare_parameter<bool>("lidar_anchor_shadow_mode", false);
    lidar_map_import_topic_ = declare_parameter<std::string>("lidar_map_import_topic", "");
    lidar_anchor_shadow_ref_period_s_ =
        declare_parameter<double>("lidar_anchor_shadow_ref_period_s", 20.0);
    if (use_lidar_map_anchor_)
    {
      LidarOccupancyMapperParams mp;
      mp.resolution_m = lidar_map_resolution_m_;
      mp.half_extent_m = lidar_map_half_extent_m_;
      mp.max_range_m = lidar_anchor_max_laser_distance_m_;
      lidar_mapper_.emplace(mp);
      lidar_anchor_gate_.emplace(true,
                                 lidar_anchor_engage_age_s_,
                                 lidar_map_insert_period_s_,
                                 lidar_anchor_disengage_dwell_s_);
      RCLCPP_INFO(get_logger(),
                  "LiDAR map anchor ENABLED: grid %.2f m over ±%.0f m, engage when RTK-Fixed older "
                  "than %.2f s",
                  lidar_map_resolution_m_,
                  lidar_map_half_extent_m_,
                  lidar_anchor_engage_age_s_);
    }
  }

  // 180° yaw-flip recovery (see fusion_graph_node.hpp). Declared outside the
  // scan_matching gate — it keys off COG, not LiDAR.
  cog_flip_recovery_enabled_ = declare_parameter<bool>("cog_flip_recovery_enabled", true);
  cog_flip_threshold_rad_ = declare_parameter<double>("cog_flip_threshold_rad", 2.618);
  cog_flip_consecutive_n_ = declare_parameter<int>("cog_flip_consecutive_n", 3);
  cog_flip_require_rtk_ = declare_parameter<bool>("cog_flip_require_rtk", true);
  cog_flip_min_interval_s_ = declare_parameter<double>("cog_flip_min_interval_s", 10.0);
  cog_flip_consistency_rad_ = declare_parameter<double>("cog_flip_consistency_rad", 0.52);
  // Gate the COG yaw factor itself on RTK-Fixed (see fusion_graph_node.hpp):
  // a Float/NO_FIX-derived COG is garbage and corrupts the yaw.
  cog_require_rtk_ = declare_parameter<bool>("cog_require_rtk", true);
  cog_rtk_max_age_s_ = declare_parameter<double>("cog_rtk_max_age_s", 2.0);
  // OpenMower-style heading discipline (see fusion_graph_node.hpp): forward-speed
  // gate + σ floor so a slow/reverse/noisy COG can't yank the yaw.
  cog_min_speed_mps_ = declare_parameter<double>("cog_min_speed_mps", 0.08);
  cog_min_sigma_rad_ = declare_parameter<double>("cog_min_sigma_rad", 0.15);
  // Level 2: floor the LiDAR scan/loop-closure yaw σ so it can't bake a wrong
  // heading (keeps LiDAR position carry). 0 = disabled.
  scan_yaw_sigma_floor_rad_ = declare_parameter<double>("scan_yaw_sigma_floor_rad", 0.30);

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
  lc_min_age_s_ = declare_parameter<double>("lc_min_age_s", 30.0);
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
  // Rate/travel gate + GPS σ floor on loop closures (issue #513, see
  // loop_closure_gate.hpp + the member doc). <= 0 disables each one.
  lc_min_travel_m_ = declare_parameter<double>("lc_min_travel_m", 1.0);
  lc_min_interval_s_ = declare_parameter<double>("lc_min_interval_s", 2.0);
  lc_gps_sigma_ratio_ = declare_parameter<double>("lc_gps_sigma_ratio", 1.0);

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
