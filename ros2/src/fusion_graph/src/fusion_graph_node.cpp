// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — construction (ctor) + entry point. (The node implementation is split across
// several translation units to keep each file within the project's 600-line budget; all share
// fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include "fusion_graph/fusion_graph_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

FusionGraphNode::FusionGraphNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("fusion_graph_node", opts)
{
  // ── Parameters ────────────────────────────────────────────────────
  GraphParams gp;
  gp.node_period_s = declare_parameter<double>("node_period_s", 0.1);
  gp.wheel_sigma_x = declare_parameter<double>("wheel_sigma_x", 0.05);
  gp.wheel_sigma_y = declare_parameter<double>("wheel_sigma_y", 0.005);
  gp.wheel_sigma_theta = declare_parameter<double>("wheel_sigma_theta", 0.01);
  gp.gyro_sigma_theta = declare_parameter<double>("gyro_sigma_theta", 0.005);
  gps_sigma_floor_m_ = declare_parameter<double>("gps_sigma_floor", 0.003);
  gp.gps_sigma_floor = gps_sigma_floor_m_;
  gp.prior_sigma_xy = declare_parameter<double>("prior_sigma_xy", 0.05);
  gp.prior_sigma_theta = declare_parameter<double>("prior_sigma_theta", 0.05);
  gp.lever_arm_x = declare_parameter<double>("lever_arm_x", 0.0);
  gp.lever_arm_y = declare_parameter<double>("lever_arm_y", 0.0);
  gp.cov_update_every_n = declare_parameter<int>("cov_update_every_n", 10);
  gp.isam2_relinearize_skip = declare_parameter<int>("isam2_relinearize_skip", 5);
  gp.max_graph_nodes = static_cast<uint64_t>(declare_parameter<int>("max_graph_nodes", 3000));
  gp.stationary_motion_thresh_m = declare_parameter<double>("stationary_motion_thresh_m", 0.02);
  gp.stationary_motion_thresh_theta =
      declare_parameter<double>("stationary_motion_thresh_theta", 0.01);
  gp.stationary_node_period_s = declare_parameter<double>("stationary_node_period_s", 5.0);
  gp.stationary_thresh_xy_m = declare_parameter<double>("stationary_thresh_xy_m", 1.0e-3);
  gp.stationary_thresh_theta = declare_parameter<double>("stationary_thresh_theta", 2.0e-3);
  gp.stationary_sigma_theta = declare_parameter<double>("stationary_sigma_theta", 1.0e-3);
  gp.pivot_gate_dtheta_rad = declare_parameter<double>("pivot_gate_dtheta_rad", 0.012);
  gp.pivot_wheel_sigma_x = declare_parameter<double>("pivot_wheel_sigma_x", 0.5);
  gp.stationary_gyro_thresh_rad_per_s =
      declare_parameter<double>("stationary_gyro_thresh_rad_per_s", 0.10);
  // Slip veto: zero the BetweenFactor translation when wheel-vs-gyro
  // rotation disagreement signals the encoders are skating. See
  // graph_manager.cpp Tick() comments.
  gp.slip_residual_thresh_rad = declare_parameter<double>("slip_residual_thresh_rad", 0.01);
  gp.slip_gyro_max_rad = declare_parameter<double>("slip_gyro_max_rad", 0.005);
  gp.slip_wheel_min_rad = declare_parameter<double>("slip_wheel_min_rad", 0.005);
  gp.gyro_bias_estimation_enabled = declare_parameter<bool>("gyro_bias_estimation_enabled", true);
  gp.gyro_bias_ema_tau_s = declare_parameter<double>("gyro_bias_ema_tau_s", 30.0);
  gp.gyro_bias_max_sample_rad_per_s =
      declare_parameter<double>("gyro_bias_max_sample_rad_per_s", 0.10);
  // Full IMU preintegration with joint bias optimisation (opt-in).
  // When true, the EMA bias path is skipped and the graph carries a
  // per-node `bias` variable plus a GyroPreintFactor on each pair of
  // consecutive poses. See GraphParams docs in graph_manager.hpp.
  gp.use_imu_preint = declare_parameter<bool>("use_imu_preint", false);
  gp.gyro_noise_density_rad_per_s =
      declare_parameter<double>("gyro_noise_density_rad_per_s", 0.015);
  gp.gyro_bias_rw_rad_per_s = declare_parameter<double>("gyro_bias_rw_rad_per_s", 0.001);
  gp.gyro_bias_prior_sigma_rad_per_s =
      declare_parameter<double>("gyro_bias_prior_sigma_rad_per_s", 0.05);
  gp.adaptive_noise_enabled_gain = declare_parameter<double>("adaptive_noise_enabled_gain", 10.0);
  gp.adaptive_noise_ema_tau_s = declare_parameter<double>("adaptive_noise_ema_tau_s", 0.5);
  gp.adaptive_noise_residual_floor_rad =
      declare_parameter<double>("adaptive_noise_residual_floor_rad", 0.005);

  // Mobile RTK wrong-fix detection (handled in OnGnss, not in graph_manager).
  // The gate compares the new fix against the last ACCEPTED GNSS reference.
  // expected_motion = max(wheel_delta_since_accepted, cmd_delta_since_accepted)
  // allowed_delta   = expected_motion + gps_sigma_xy * multiplier + margin
  // innovation      = |delta_gps - expected_motion|
  // Mild outliers are downweighted; only strong ones are rejected.
  rtk_wrongfix_gps_sigma_multiplier_ =
      declare_parameter<double>("rtk_wrongfix_gps_sigma_multiplier", 2.0);
  rtk_wrongfix_min_margin_m_ = declare_parameter<double>("rtk_wrongfix_min_margin_m", 0.01);
  rtk_wrongfix_downweight_innovation_multiplier_ =
      declare_parameter<double>("rtk_wrongfix_downweight_innovation_multiplier", 2.0);
  rtk_wrongfix_downweight_sigma_multiplier_ =
      declare_parameter<double>("rtk_wrongfix_downweight_sigma_multiplier", 4.0);
  // Dock-pose hold while charging: re-assert a firm ForceAnchor at the full
  // dock_pose once per new node (replaces the weak live-GPS factor that walked
  // the docked pose 11.5 cm + 53° over a dwell — field 2026-06-10). σ small so
  // the dock prior dominates xy; yaw σ from dock_pose_yaw_sigma_rad.
  dock_reanchor_sigma_xy_m_ = declare_parameter<double>("dock_reanchor_sigma_xy_m", 0.03);
  // Dock-approach pose stabilisation (see header). During the final graceful
  // approach, drop COG yaw + reject RTK-float epochs so the dock controller
  // gets a stable target instead of a flickering pose.
  docking_active_timeout_s_ = declare_parameter<double>("docking_active_timeout_s", 1.0);
  gate_cog_during_docking_ = declare_parameter<bool>("gate_cog_during_docking", true);
  gate_float_gps_during_docking_ = declare_parameter<bool>("gate_float_gps_during_docking", true);

  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);

  // Keyframe-map GraphParams (used by the GraphManager store/persistence;
  // the node-side capture/apply params are declared in the scan-matching
  // block below). datum tags persisted maps to their garden (cross-site
  // guard on Load).
  gp.datum_lat = datum_lat_;
  gp.datum_lon = datum_lon_;
  gp.kf_spacing_m = declare_parameter<double>("kf_spacing_m", 0.5);
  gp.max_keyframes = static_cast<uint64_t>(declare_parameter<int>("max_keyframes", 2000));
  kf_spacing_m_ = gp.kf_spacing_m;  // node reuses for the capture-spacing gate

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  tf_publish_lead_s_ = declare_parameter<double>("tf_publish_lead_s", 0.0);
  // Dedicated TF-broadcast thread rate (see fusion_graph_node.hpp). 20 Hz
  // halves worst-case TF staleness vs the 25 Hz tick cadence while staying
  // cheap (constant anchor + integrated dr_*). <= 0 disables the thread and
  // falls back to inline OnTimer publishing.
  tf_broadcast_rate_hz_ = declare_parameter<double>("tf_broadcast_rate_hz", 20.0);

  graph_ = std::make_shared<GraphManager>(gp);
  DeclareParameters();
  SetupCommunications(gp.node_period_s);
}

FusionGraphNode::~FusionGraphNode()
{
  tf_thread_stop_.store(true, std::memory_order_release);
  if (tf_thread_.joinable())
  {
    tf_thread_.join();
  }
}

}  // namespace fusion_graph

// ── Entry point ──────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fusion_graph::FusionGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
