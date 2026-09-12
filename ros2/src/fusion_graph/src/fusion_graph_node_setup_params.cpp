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
  // SeedFromDockPose()
  // is the only seed path that fires while the robot boots docked
  // and stationary (COG needs motion, mag is off by default) — the
  // no-LiDAR config must still be able to bootstrap.
  dock_pose_x_ = declare_parameter<double>("dock_pose_x", 0.0);
  dock_pose_y_ = declare_parameter<double>("dock_pose_y", 0.0);
  dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);
  dock_pose_yaw_sigma_rad_ = declare_parameter<double>("dock_pose_yaw_sigma_rad", 0.035);

  // LiDAR map anchor (Beluga particle filter against a grid built under
  // RTK-Fixed). Complete-GNSS-outage fallback; navigation enables it only with
  // LiDAR present.
  use_lidar_map_anchor_ = declare_parameter<bool>("use_lidar_map_anchor", false);
  lidar_map_resolution_m_ = declare_parameter<double>("lidar_map_resolution_m", 0.10);
  lidar_map_tile_size_m_ = declare_parameter<double>("lidar_map_tile_size_m", 10.0);
  lidar_map_radius_tiles_ = declare_parameter<int>("lidar_map_radius_tiles", 2);
  lidar_map_insert_period_s_ = declare_parameter<double>("lidar_map_insert_period_s", 0.5);
  lidar_map_rebuild_period_s_ = declare_parameter<double>("lidar_map_rebuild_period_s", 5.0);
  // 1.0 s, not 0.3: a 5 Hz receiver whose stamps arrive ~80 ms old sits at
  // ~0.28 s of age just before every next fix — 0.3 flapped on timer phase.
  lidar_scan_max_age_s_ = declare_parameter<double>("lidar_scan_max_age_s", 0.5);
  lidar_anchor_apply_age_s_ = declare_parameter<double>("lidar_anchor_apply_age_s", 20.0);
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
  lidar_anchor_odom_alpha_trans_ = declare_parameter<double>("lidar_anchor_odom_alpha_trans", 0.05);
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
  lidar_anchor_validator_.min_hit_count = declare_parameter<int>("lidar_anchor_min_hit_count", 30);
  lidar_anchor_validator_.max_sigma_m = declare_parameter<double>("lidar_anchor_max_sigma_m", 0.5);
  lidar_anchor_validator_.dr_budget_m = declare_parameter<double>("lidar_anchor_dr_budget_m", 0.3);
  lidar_anchor_validator_.dr_drift_frac =
      declare_parameter<double>("lidar_anchor_dr_drift_frac", 0.02);
  lidar_anchor_reseed_after_s_ = declare_parameter<double>("lidar_anchor_reseed_after_s", 5.0);
  lidar_anchor_shadow_mode_ = declare_parameter<bool>("lidar_anchor_shadow_mode", false);
  lidar_map_import_topic_ = declare_parameter<std::string>("lidar_map_import_topic", "");
  lidar_anchor_shadow_ref_period_s_ =
      declare_parameter<double>("lidar_anchor_shadow_ref_period_s", 20.0);
  lidar_anchor_undock_dwell_s_ = declare_parameter<double>("lidar_anchor_undock_dwell_s", 10.0);
  // Self-calibrated σ floor: shadow-mode error vs RTK-Fixed, rolling window,
  // served as a quantile (field 2026-09-08: p90 0.15 m vs a fixed 0.05).
  lidar_anchor_adaptive_floor_ = declare_parameter<bool>("lidar_anchor_adaptive_floor", true);
  lidar_anchor_floor_quantile_ = declare_parameter<double>("lidar_anchor_floor_quantile", 0.9);
  lidar_anchor_shadow_stats_ = LidarAnchorShadowStats(
      static_cast<std::size_t>(
          std::max<int64_t>(1, declare_parameter<int64_t>("lidar_anchor_shadow_window", 300))),
      static_cast<std::size_t>(
          std::max<int64_t>(1, declare_parameter<int64_t>("lidar_anchor_shadow_min_samples", 50))));
  if (use_lidar_map_anchor_)
  {
    LidarOccupancyMapperParams mp;
    mp.resolution_m = lidar_map_resolution_m_;

    mp.max_range_m = lidar_anchor_max_laser_distance_m_;
    // Evidence weights. Defaults = one hit marks a cell occupied; a garden
    // (foliage, wind) wants a higher occupied threshold so a cell needs
    // several concordant hits before the filter may trust it — tuned on
    // the replay harness, not by hand.
    mp.log_odds_hit = declare_parameter<double>("lidar_map_log_odds_hit", mp.log_odds_hit);
    mp.log_odds_miss = declare_parameter<double>("lidar_map_log_odds_miss", mp.log_odds_miss);
    mp.occupied_threshold =
        declare_parameter<double>("lidar_map_occupied_threshold", 2.0 * mp.log_odds_hit);
    mp.free_threshold = declare_parameter<double>("lidar_map_free_threshold", mp.free_threshold);
    lidar_mapper_params_ = mp;

    lidar_anchor_gate_.emplace(true,
                               lidar_anchor_engage_age_s_,
                               lidar_map_insert_period_s_,
                               lidar_anchor_disengage_dwell_s_);
  }

  // 180° yaw-flip recovery from COG (see fusion_graph_node.hpp).
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

  // ── Graph persistence ───────────────────────────────────────────
  graph_save_prefix_ =
      declare_parameter<std::string>("graph_save_prefix", "/ros2_ws/maps/fusion_graph");

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

  if (use_lidar_map_anchor_)
  {
    if (lidar_map_tile_size_m_ * lidar_map_radius_tiles_ < lidar_anchor_max_laser_distance_m_ + 2.0)
      throw std::invalid_argument("LiDAR local window must cover laser range plus 2 m");
    lidar_submaps_ = std::make_unique<LidarSubmapStore>(lidar_mapper_params_,
                                                        graph_save_prefix_,
                                                        datum_lat_,
                                                        datum_lon_,
                                                        lidar_map_tile_size_m_,
                                                        lidar_map_radius_tiles_);
    LidarComputeGateParams cp;
    cp.engage_age_s = lidar_anchor_engage_age_s_;
    cp.apply_age_s = lidar_anchor_apply_age_s_;
    cp.max_rate_hz = declare_parameter<double>("lidar_anchor_max_rate_hz", 5.0);
    cp.calibration_period_s = declare_parameter<double>("lidar_anchor_calibration_period_s", 60.0);
    cp.calibration_burst_s = declare_parameter<double>("lidar_anchor_calibration_burst_s", 10.0);
    cp.warmup_s = declare_parameter<double>("lidar_anchor_warmup_s", 5.0);
    lidar_compute_gate_ = LidarComputeGate(cp);
    const auto nonnegative = [](double v)
    {
      return std::isfinite(v) && v >= 0.0;
    };
    const auto positive = [](double v)
    {
      return std::isfinite(v) && v > 0.0;
    };
    if (!lidar_compute_gate_.ValidConfiguration() ||
        !ValidLidarAnchorParams(lidar_anchor_validator_) || !positive(lidar_map_insert_period_s_) ||
        !nonnegative(lidar_map_rebuild_period_s_) ||
        !nonnegative(lidar_anchor_disengage_dwell_s_) ||
        !nonnegative(lidar_anchor_undock_dwell_s_) || !nonnegative(lidar_anchor_floor_quantile_) ||
        lidar_anchor_floor_quantile_ > 1.0 || !positive(lidar_anchor_sigma_floor_param_m_) ||
        !std::isfinite(lidar_anchor_z_hit_ + lidar_anchor_z_rand_) ||
        !positive(lidar_scan_max_age_s_) || !positive(lidar_anchor_seed_sigma_xy_m_) ||
        !positive(lidar_anchor_seed_sigma_theta_rad_) || !positive(lidar_anchor_sigma_hit_m_) ||
        !positive(lidar_anchor_shadow_ref_period_s_) ||
        !nonnegative(lidar_anchor_reseed_after_s_) || !nonnegative(lidar_anchor_update_min_d_) ||
        !nonnegative(lidar_anchor_update_min_a_) || !nonnegative(lidar_anchor_odom_alpha_rot_) ||
        !nonnegative(lidar_anchor_odom_alpha_trans_) || !nonnegative(lidar_anchor_alpha_slow_) ||
        !nonnegative(lidar_anchor_alpha_fast_) || !nonnegative(lidar_anchor_z_hit_) ||
        !nonnegative(lidar_anchor_z_rand_) || lidar_anchor_z_hit_ + lidar_anchor_z_rand_ <= 0.0 ||
        lidar_anchor_max_beams_ < 1 || lidar_anchor_min_particles_ < 1 ||
        lidar_anchor_max_particles_ < lidar_anchor_min_particles_)
      throw std::invalid_argument("invalid LiDAR scheduling, sensor or validation parameters");
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
