// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/fusion_graph_node.hpp"

#include <chrono>
#include <thread>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fusion_graph
{

namespace
{

constexpr double kEarthRadius = 6378137.0;  // WGS84 equatorial, metres.

// Extract yaw from a geometry_msgs Quaternion.
double YawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

geometry_msgs::msg::Quaternion QuatFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

}  // namespace

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
  gp.gps_sigma_floor = declare_parameter<double>("gps_sigma_floor", 0.003);
  gp.prior_sigma_xy = declare_parameter<double>("prior_sigma_xy", 0.05);
  gp.prior_sigma_theta = declare_parameter<double>("prior_sigma_theta", 0.05);
  gp.lever_arm_x = declare_parameter<double>("lever_arm_x", 0.0);
  gp.lever_arm_y = declare_parameter<double>("lever_arm_y", 0.0);
  // Cache the radial lever-arm magnitude for the RTK wrong-fix gate.
  // The graph itself consumes lever_arm_x/y via GnssLeverArmFactor;
  // we only mirror the magnitude here for the gate-side threshold
  // and never re-apply the offset to the GPS sample.
  lever_arm_radius_m_ = std::hypot(gp.lever_arm_x, gp.lever_arm_y);
  gp.cov_update_every_n = declare_parameter<int>("cov_update_every_n", 10);
  gp.isam2_relinearize_skip = declare_parameter<int>("isam2_relinearize_skip", 5);
  gp.max_graph_nodes =
      static_cast<uint64_t>(declare_parameter<int>("max_graph_nodes", 3000));
  gp.stationary_motion_thresh_m = declare_parameter<double>("stationary_motion_thresh_m", 0.02);
  gp.stationary_motion_thresh_theta =
      declare_parameter<double>("stationary_motion_thresh_theta", 0.01);
  gp.stationary_node_period_s = declare_parameter<double>("stationary_node_period_s", 5.0);
  gp.stationary_thresh_xy_m =
      declare_parameter<double>("stationary_thresh_xy_m", 1.0e-3);
  gp.stationary_thresh_theta =
      declare_parameter<double>("stationary_thresh_theta", 2.0e-3);
  gp.stationary_sigma_theta =
      declare_parameter<double>("stationary_sigma_theta", 1.0e-3);
  gp.pivot_gate_dtheta_rad =
      declare_parameter<double>("pivot_gate_dtheta_rad", 0.012);
  gp.pivot_wheel_sigma_x =
      declare_parameter<double>("pivot_wheel_sigma_x", 0.5);
  gp.stationary_gyro_thresh_rad_per_s =
      declare_parameter<double>("stationary_gyro_thresh_rad_per_s", 0.10);
  // Slip veto: zero the BetweenFactor translation when wheel-vs-gyro
  // rotation disagreement signals the encoders are skating. See
  // graph_manager.cpp Tick() comments.
  gp.slip_residual_thresh_rad =
      declare_parameter<double>("slip_residual_thresh_rad", 0.01);
  gp.slip_gyro_max_rad =
      declare_parameter<double>("slip_gyro_max_rad", 0.005);
  gp.slip_wheel_min_rad =
      declare_parameter<double>("slip_wheel_min_rad", 0.005);
  gp.gyro_bias_estimation_enabled =
      declare_parameter<bool>("gyro_bias_estimation_enabled", true);
  gp.gyro_bias_ema_tau_s =
      declare_parameter<double>("gyro_bias_ema_tau_s", 30.0);
  gp.gyro_bias_max_sample_rad_per_s =
      declare_parameter<double>("gyro_bias_max_sample_rad_per_s", 0.10);
  // Full IMU preintegration with joint bias optimisation (opt-in).
  // When true, the EMA bias path is skipped and the graph carries a
  // per-node `bias` variable plus a GyroPreintFactor on each pair of
  // consecutive poses. See GraphParams docs in graph_manager.hpp.
  gp.use_imu_preint =
      declare_parameter<bool>("use_imu_preint", false);
  gp.gyro_noise_density_rad_per_s =
      declare_parameter<double>("gyro_noise_density_rad_per_s", 0.015);
  gp.gyro_bias_rw_rad_per_s =
      declare_parameter<double>("gyro_bias_rw_rad_per_s", 0.001);
  gp.gyro_bias_prior_sigma_rad_per_s =
      declare_parameter<double>("gyro_bias_prior_sigma_rad_per_s", 0.05);
  gp.adaptive_noise_enabled_gain =
      declare_parameter<double>("adaptive_noise_enabled_gain", 10.0);
  gp.adaptive_noise_ema_tau_s =
      declare_parameter<double>("adaptive_noise_ema_tau_s", 0.5);
  gp.adaptive_noise_residual_floor_rad =
      declare_parameter<double>("adaptive_noise_residual_floor_rad", 0.005);

  // RTK wrong-fix detection (handled in OnGnss, not in graph_manager).
  rtk_wrongfix_max_jump_m_ =
      declare_parameter<double>("rtk_wrongfix_max_jump_m", 0.05);
  dock_gps_sigma_m_ = declare_parameter<double>("dock_gps_sigma_m", 0.50);
  rtk_wrongfix_max_wheel_m_ =
      declare_parameter<double>("rtk_wrongfix_max_wheel_m", 0.02);

  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  tf_publish_lead_s_ = declare_parameter<double>("tf_publish_lead_s", 0.0);

  graph_ = std::make_shared<GraphManager>(gp);

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
  dock_pose_yaw_sigma_rad_ =
      declare_parameter<double>("dock_pose_yaw_sigma_rad", 0.035);

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
  const double periodic_save_period_s = declare_parameter<double>("periodic_save_period_s", 300.0);

  // ── TF ────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Pubs/subs ─────────────────────────────────────────────────────
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered_map", 10);
  // /odometry/filtered + odom→base_footprint TF — replaces ekf_odom_node.
  // Continuous local-frame dead reckoning (wheel vx + gyro_z), never
  // sees GPS, never jumps. Nav2's odom_topic in nav2_params.yaml still
  // points here.
  pub_local_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

  // High-rate extrapolated pose (item #15). Off by default — set
  // fast_pose_publish_rate_hz > 0 in yaml to enable. 100 Hz is the
  // intended use; the publisher reuses the latest fusion pose and
  // projects yaw forward by latest IMU gyro_z. Marked as a
  // best-effort feed because consumers should fall back to the
  // canonical /odometry/filtered_map for control loops.
  fast_pose_publish_rate_hz_ =
      declare_parameter<double>("fast_pose_publish_rate_hz", 0.0);
  if (fast_pose_publish_rate_hz_ > 0.0)
  {
    pub_odom_fast_ = create_publisher<nav_msgs::msg::Odometry>(
        "/odometry/filtered_map_fast", rclcpp::SensorDataQoS());
    const auto period =
        std::chrono::duration<double>(1.0 / fast_pose_publish_rate_hz_);
    fast_pose_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]()
        {
          if (!pose_extrap_.HasFusionPose())
            return;
          const double now_s = this->now().seconds();
          auto extrap = pose_extrap_.Extrapolate(now_s);
          if (!extrap)
            return;
          nav_msgs::msg::Odometry msg;
          msg.header.stamp = this->now();
          msg.header.frame_id = map_frame_;
          msg.child_frame_id = base_frame_;
          msg.pose.pose.position.x = extrap->x();
          msg.pose.pose.position.y = extrap->y();
          msg.pose.pose.position.z = 0.0;
          msg.pose.pose.orientation = QuatFromYaw(extrap->theta());
          // Loose covariance — this is a display / latency-sensitive
          // feed, not a primary measurement. Consumers that need
          // tight σ should subscribe to /odometry/filtered_map.
          for (int i = 0; i < 36; ++i)
            msg.pose.covariance[i] = 0.0;
          msg.pose.covariance[0] = 0.10 * 0.10;
          msg.pose.covariance[7] = 0.10 * 0.10;
          msg.pose.covariance[35] = 0.10 * 0.10;
          pub_odom_fast_->publish(msg);
        });
    RCLCPP_INFO(get_logger(),
                "fusion_graph: high-rate pose extrapolator enabled at %.1f Hz",
                fast_pose_publish_rate_hz_);
  }

  // /imu/fg_yaw — yaw-only sensor_msgs/Imu published BEST_EFFORT to
  // match cog_to_imu / mag_yaw_publisher conventions. Lets ekf_map_node
  // (when running as primary in observer mode) subscribe as a yaw
  // source, replacing the mag_yaw_publisher slot. fusion_graph yaw is
  // typically σ ≈ 0.5° vs ekf's σ ≈ 13° in stationary, so this
  // dramatically tightens the EKF map-frame yaw without changing the
  // primary localizer.
  pub_fg_yaw_ = create_publisher<sensor_msgs::msg::Imu>("/imu/fg_yaw", rclcpp::SensorDataQoS());
  pub_diag_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fusion_graph/diagnostics", 10);
  pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/fusion_graph/markers",
                                                             rclcpp::QoS(1).transient_local());

  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_wheel_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 50, std::bind(&FusionGraphNode::OnWheelOdom, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos, std::bind(&FusionGraphNode::OnImu, this, std::placeholders::_1));

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", sensor_qos, std::bind(&FusionGraphNode::OnGnss, this, std::placeholders::_1));

  // /imu/cog_heading and /imu/mag_yaw are published BEST_EFFORT by
  // cog_to_imu.py and mag_yaw_publisher.py — use SensorDataQoS or
  // the subscription is silently dropped at the QoS handshake.
  sub_cog_ = create_subscription<sensor_msgs::msg::Imu>("/imu/cog_heading",
                                                        sensor_qos,
                                                        std::bind(&FusionGraphNode::OnCogHeading,
                                                                  this,
                                                                  std::placeholders::_1));

  if (use_magnetometer_)
  {
    sub_mag_ = create_subscription<sensor_msgs::msg::Imu>("/imu/mag_yaw",
                                                          sensor_qos,
                                                          std::bind(&FusionGraphNode::OnMagYaw,
                                                                    this,
                                                                    std::placeholders::_1));
  }

  if (use_scan_matching_ || loop_closure_enabled_)
  {
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos, std::bind(&FusionGraphNode::OnScan, this, std::placeholders::_1));
  }

  // /hardware_bridge/status is always subscribed — OnHardwareStatus
  // serves two purposes that are independent of auto-save:
  //   1. Dock-arrival pose seed (rising edge of is_charging anchors
  //      the graph at the operator-calibrated dock_pose_*).
  //   2. Auto-checkpoint to disk (gated on auto_save_enabled_).
  sub_hw_status_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      10,
      std::bind(&FusionGraphNode::OnHardwareStatus, this, std::placeholders::_1));

  if (auto_save_enabled_)
  {
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        10,
        std::bind(&FusionGraphNode::OnHighLevelStatus, this, std::placeholders::_1));
    if (periodic_save_period_s > 0.0)
    {
      periodic_save_timer_ =
          create_wall_timer(std::chrono::duration<double>(periodic_save_period_s),
                            std::bind(&FusionGraphNode::OnPeriodicSaveTimer, this));
    }
  }

  // ── External set-pose channel ───────────────────────────────────
  // Equivalent to robot_localization's /<node>/set_pose: takes a
  // PoseWithCovarianceStamped, anchors the latest graph node at the
  // given pose with covariance-derived sigmas. Used by the BT
  // calibration nodes after a yaw-cal manoeuvre. The dock-arrival
  // seed (formerly dock_yaw_to_set_pose_node) now bypasses this
  // topic entirely — see SeedFromDockPose().
  //
  // QoS: TRANSIENT_LOCAL with depth-1, matching dock_yaw_to_set_pose's
  // publisher. The boot seed is a one-shot rising-edge event; with
  // VOLATILE either side, a subscriber that hasn't finished discovery
  // when the message is published silently loses it and the graph
  // never bootstraps (observed 2026-05-03 after a force-recreate). With
  // TL on both sides, a late-joining subscriber gets the last seed
  // pose latched on connect — node bootstraps without manual republish.
  rclcpp::QoS set_pose_qos(rclcpp::KeepLast(1));
  set_pose_qos.reliable();
  set_pose_qos.transient_local();
  sub_set_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/set_pose",
      set_pose_qos,
      std::bind(&FusionGraphNode::OnSetPose, this, std::placeholders::_1));

  // ── Save-graph service ──────────────────────────────────────────
  // Trigger from the GUI / a BT node when transitioning out of
  // RECORDING, or manually via:
  //   ros2 service call /fusion_graph_node/save_graph std_srvs/Trigger
  srv_save_ = create_service<std_srvs::srv::Trigger>(
      "~/save_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        const bool ok = graph_->Save(graph_save_prefix_);
        resp->success = ok;
        resp->message = ok ? "saved to " + graph_save_prefix_ + ".*" : "save failed";
        if (ok)
          RCLCPP_INFO(get_logger(), "fusion_graph: %s", resp->message.c_str());
        else
          RCLCPP_WARN(get_logger(), "fusion_graph: %s", resp->message.c_str());
      });

  // ── Clear-graph service ─────────────────────────────────────────
  // Wipes iSAM2 + accumulated factors + per-node scans + loop-closure
  // edges. The node stays alive; the next valid pose seed (GPS, set_pose
  // or scan-match relocalization) re-initializes the graph.
  // Trigger from the GUI when the operator wants to start a clean
  // session (e.g. after relocating to a new garden) without restarting
  // the whole stack:
  //   ros2 service call /fusion_graph_node/clear_graph std_srvs/Trigger
  srv_clear_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        graph_->Reset();
        // Drop the latched seed too, otherwise a stale GPS / yaw seed
        // from before the clear would re-initialize the graph at the
        // old position the operator was trying to escape.
        seed_xy_.reset();
        seed_yaw_.reset();
        seed_xy_rtk_fixed_ = false;
        // Re-zero the dead-reckoning frame. Without this the odom→base
        // transform keeps whatever offset it had accumulated (observed
        // 74 m), so map→odom = graph_pose ∘ dr⁻¹ still has the huge
        // lever arm that amplifies any graph-vs-DR yaw difference into
        // metres of map-pose jump — i.e. clearing the graph alone does
        // NOT stop the robot from "sliding". Resetting dr_* collapses
        // the lever arm to zero so map→odom tracks the fresh graph
        // directly. The odom→base TF jumps here, but clear_graph is an
        // explicit operator escape hatch (robot parked), so the local
        // costmap discontinuity is acceptable and self-heals on the
        // next costmap clear.
        dr_x_ = 0.0;
        dr_y_ = 0.0;
        dr_yaw_ = 0.0;
        t_map_odom_anchor_valid_ = false;
        resp->success = true;
        resp->message = "graph cleared + odom re-based (waiting for re-initialization)";
        RCLCPP_WARN(get_logger(), "fusion_graph: %s", resp->message.c_str());
      });

  // ── Tick timer ────────────────────────────────────────────────────
  // Run at 1× node rate. Earlier 2× oversampling existed "to never
  // miss a node window" but doubled per-Tick CPU (ICP runs every
  // OnTimer call) for no functional gain — Tick() short-circuits when
  // dt < node_period_s, so a late wall_timer just creates the next
  // node a few ms late, with no graph-level effect.
  const double timer_period_s = gp.node_period_s;
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(timer_period_s),
                                  std::bind(&FusionGraphNode::OnTimer, this));

  // Maintenance timer at 30 s: prune old scans + check if iSAM2
  // needs to be rebased. PruneOldScans is cheap (just erasing old
  // entries under the lock) and stays inline. The rebase, however,
  // rebuilds the Bayes tree from ~50k PriorFactors — ~1 s of CPU
  // that used to block the executor and stall the map→odom TF
  // (observed 2026-05-14, caused DockRobot to abort with
  // `Transform data too old`). Dispatch it to a detached worker so
  // the callback returns immediately; GraphManager::RebaseISAM2
  // now does the heavy reconstruction outside the graph mutex and
  // only takes the lock briefly to replay pending factors + swap.
  maintenance_timer_ = create_wall_timer(
      std::chrono::seconds(30),
      [this]()
      {
        if (!graph_->IsInitialized())
          return;
        graph_->PruneOldScans(scan_retention_nodes_);
        const auto stats = graph_->Stats();
        if (stats.total_nodes - last_rebase_index_ < isam2_rebase_every_nodes_)
          return;
        bool expected = false;
        if (!rebase_in_flight_->compare_exchange_strong(expected, true))
        {
          // Previous async rebase still running — skip and try again
          // at the next maintenance tick.
          return;
        }
        // Commit the bookkeeping NOW so the next 30 s maintenance
        // tick doesn't re-trigger while the worker is still building.
        last_rebase_index_ = stats.total_nodes;
        std::thread(
            [graph = graph_, logger = get_logger(),
             total = stats.total_nodes, flag = rebase_in_flight_]()
            {
              graph->RebaseISAM2();
              RCLCPP_INFO(logger, "fusion_graph: iSAM2 rebased at node %lu",
                          static_cast<unsigned long>(total));
              flag->store(false);
            })
            .detach();
      });

  // Diagnostics timer at 1 Hz — coarse, just for the session monitor.
  diag_timer_ =
      create_wall_timer(std::chrono::seconds(1),
                        [this]()
                        {
                          auto stats = graph_->Stats();
                          auto snap = graph_->LatestSnapshot();

                          diagnostic_msgs::msg::DiagnosticArray msg;
                          msg.header.stamp = this->now();
                          diagnostic_msgs::msg::DiagnosticStatus s;
                          s.name = "fusion_graph";
                          s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                          s.message = graph_->IsInitialized() ? "running" : "waiting init";

                          auto add = [&s](const std::string& k, const std::string& v)
                          {
                            diagnostic_msgs::msg::KeyValue kv;
                            kv.key = k;
                            kv.value = v;
                            s.values.push_back(kv);
                          };
                          add("total_nodes", std::to_string(stats.total_nodes));
                          add("scans_attached", std::to_string(stats.scans_attached));
                          add("loop_closures", std::to_string(stats.loop_closures));
                          add("scans_received", std::to_string(scans_received_));
                          add("scan_matches_ok", std::to_string(scan_matches_ok_));
                          add("scan_matches_fail", std::to_string(scan_matches_fail_));
                          // Robustness-pass health counters. Each is a
                          // cumulative count since process start; the
                          // session monitor diffs consecutive samples
                          // to get a rate. A spike on any of these is
                          // worth surfacing — see PR notes.
                          add("gps_rejects_wrongfix",
                              std::to_string(stats.gps_rejects_wrongfix));
                          add("icp_rejects_rmse",
                              std::to_string(stats.icp_rejects_rmse));
                          add("icp_rejects_inliers",
                              std::to_string(stats.icp_rejects_inliers));
                          add("icp_rejects_sanity",
                              std::to_string(stats.icp_rejects_sanity));
                          add("icp_rejects_divergence",
                              std::to_string(stats.icp_rejects_divergence));
                          add("stationary_hand_push",
                              std::to_string(stats.stationary_hand_push));
                          add("slip_veto", std::to_string(stats.slip_veto));
                          add("live_nodes",
                              std::to_string(graph_->LiveNodeCount()));
                          // Gyro bias telemetry (item #3).
                          {
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%.5f", stats.gyro_bias_z);
                            add("gyro_bias_z_rad_per_s", buf);
                            add("gyro_bias_updates",
                                std::to_string(stats.gyro_bias_updates));
                          }
                          // Adaptive process-noise telemetry.
                          {
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%.5f", stats.residual_ema_rad);
                            add("residual_ema_rad", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", stats.wheel_sigma_x_eff);
                            add("wheel_sigma_x_eff", buf);
                          }
                          if (snap)
                          {
                            char buf[64];
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(0, 0));
                            add("cov_xx", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(1, 1));
                            add("cov_yy", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(2, 2));
                            add("cov_yawyaw", buf);
                          }
                          msg.status.push_back(s);
                          pub_diag_->publish(msg);

                          // ── Pose-graph viz ────────────────────────────────────────
                          // Emits a single MarkerArray with three markers, each owning
                          // its own id so subsequent publishes overwrite cleanly:
                          //   id=0  SPHERE_LIST  — every node's optimized xy
                          //   id=1  LINE_STRIP   — trajectory through nodes by index
                          //   id=2  LINE_LIST    — accepted loop-closure edges
                          // All in map_frame_; transient-local QoS so a Foxglove client
                          // joining mid-session sees the whole graph immediately.
                          const auto poses = graph_->GetAllPoses();
                          const auto loops = graph_->GetLoopClosureEdges();
                          const rclcpp::Time stamp = this->now();

                          visualization_msgs::msg::MarkerArray ma;

                          visualization_msgs::msg::Marker nodes;
                          nodes.header.stamp = stamp;
                          nodes.header.frame_id = map_frame_;
                          nodes.ns = "fusion_graph";
                          nodes.id = 0;
                          nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                          nodes.action = visualization_msgs::msg::Marker::ADD;
                          nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.10;
                          nodes.color.r = 0.1f;
                          nodes.color.g = 0.7f;
                          nodes.color.b = 1.0f;
                          nodes.color.a = 1.0f;
                          nodes.pose.orientation.w = 1.0;

                          visualization_msgs::msg::Marker traj;
                          traj.header = nodes.header;
                          traj.ns = "fusion_graph";
                          traj.id = 1;
                          traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
                          traj.action = visualization_msgs::msg::Marker::ADD;
                          traj.scale.x = 0.03;
                          traj.color.r = 0.5f;
                          traj.color.g = 0.5f;
                          traj.color.b = 0.5f;
                          traj.color.a = 0.8f;
                          traj.pose.orientation.w = 1.0;

                          // Marker bandwidth control. With 4 k+ nodes, dumping every
                          // node to the SPHERE_LIST every second produces ~50 KB of
                          // payload per tick — Foxglove + DDS choke. Cap at the most
                          // recent `viz_max_nodes` (default 1500) and stride-decimate
                          // older history if the cap is exceeded. Trajectory line still
                          // includes the same set so the topology stays connected.
                          constexpr size_t kVizMaxNodes = 1500;
                          const size_t total = poses.size();
                          const size_t stride =
                              total > kVizMaxNodes ? std::max<size_t>(1, total / kVizMaxNodes) : 1;
                          size_t i = 0;
                          for (const auto& [idx, p] : poses)
                          {
                            if (i++ % stride != 0)
                              continue;
                            geometry_msgs::msg::Point pt;
                            pt.x = p.x();
                            pt.y = p.y();
                            pt.z = 0.0;
                            nodes.points.push_back(pt);
                            traj.points.push_back(pt);
                          }
                          ma.markers.push_back(nodes);
                          ma.markers.push_back(traj);

                          visualization_msgs::msg::Marker lc;
                          lc.header = nodes.header;
                          lc.ns = "fusion_graph";
                          lc.id = 2;
                          lc.type = visualization_msgs::msg::Marker::LINE_LIST;
                          lc.action = visualization_msgs::msg::Marker::ADD;
                          lc.scale.x = 0.04;
                          lc.color.r = 1.0f;
                          lc.color.g = 0.2f;
                          lc.color.b = 0.2f;
                          lc.color.a = 0.9f;
                          lc.pose.orientation.w = 1.0;
                          for (const auto& [a, b] : loops)
                          {
                            auto ia = poses.find(a);
                            auto ib = poses.find(b);
                            if (ia == poses.end() || ib == poses.end())
                              continue;
                            geometry_msgs::msg::Point pa, pb;
                            pa.x = ia->second.x();
                            pa.y = ia->second.y();
                            pb.x = ib->second.x();
                            pb.y = ib->second.y();
                            lc.points.push_back(pa);
                            lc.points.push_back(pb);
                          }
                          ma.markers.push_back(lc);

                          pub_markers_->publish(ma);
                        });

  RCLCPP_INFO(get_logger(),
              "fusion_graph_node up: datum=(%.6f, %.6f), node_period=%.3fs",
              datum_lat_,
              datum_lon_,
              gp.node_period_s);
}

// ── Callbacks ─────────────────────────────────────────────────────────

void FusionGraphNode::OnWheelOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  // Latest forward velocity for the local-frame DR integration in OnImu.
  // twist.linear.y is non-holonomically locked to 0 by hardware_bridge
  // (tight vy covariance) — we mirror that by only integrating vx.
  wheel_vx_ = msg->twist.twist.linear.x;
  wheel_wz_ = msg->twist.twist.angular.z;
  if (last_wheel_stamp_)
  {
    double dt = (stamp - *last_wheel_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddWheelTwist(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z,
                            dt);
      // Track wheel-derived distance since the last GPS fix for the
      // RTK wrong-fix sanity gate in OnGnss. Speed-magnitude × dt is
      // the right scalar — direction doesn't matter for the threshold
      // test, only how far the chassis travelled.
      const double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      wheel_dist_since_last_gps_m_ += speed * dt;
    }
  }
  last_wheel_stamp_ = stamp;
}

void FusionGraphNode::OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (last_imu_stamp_)
  {
    double dt = (stamp - *last_imu_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddGyroDelta(msg->angular_velocity.z, dt);
      // Local-frame dead reckoning. Yaw integrates the bias-corrected
      // gyro_z (hardware_bridge subtracts the dock-time IMU bias);
      // position uses the latest wheel vx with the just-updated yaw.
      // Sub-cm/sub-° accuracy per IMU sample at typical 91 Hz / 0.5 m/s.
      const double gz = msg->angular_velocity.z;
      dr_yaw_ += gz * dt;
      // Slip veto (see header): if the wheels claim a yaw rate the
      // gyro doesn't see, the chassis is being skated, not driven —
      // its forward velocity is phantom. Drop the translation for
      // this sample; yaw still integrates from the gyro, which is the
      // honest source during a slipping pivot. Without this the odom
      // frame accumulates the fictitious forward motion unbounded.
      const bool dr_slip =
          std::abs(wheel_wz_ - gz) > dr_slip_wheel_min_rad_per_s_ &&
          std::abs(gz) < dr_slip_gyro_max_rad_per_s_ &&
          std::abs(wheel_wz_) > dr_slip_wheel_min_rad_per_s_;
      const double vx_eff = dr_slip ? 0.0 : wheel_vx_;
      dr_x_ += vx_eff * std::cos(dr_yaw_) * dt;
      dr_y_ += vx_eff * std::sin(dr_yaw_) * dt;
      // Accumulate |Δθ| since the last accepted GPS for the wrong-fix
      // gate. A stationary pivot sweeps the GPS antenna by lever_arm
      // × Δθ in the map frame; without this term the gate sees a
      // pure-sweep jump as if it were a phantom translation and
      // rejects every legitimate fix.
      abs_dtheta_since_last_gps_rad_ += std::abs(gz) * dt;
    }
  }
  last_imu_stamp_ = stamp;
  // Feed the high-rate extrapolator (item #15) too. Safe even when
  // fast_pose_timer_ is null — the extrapolator is just a value
  // cache.
  pose_extrap_.OnImuGyro(stamp.seconds(), msg->angular_velocity.z);
}

void FusionGraphNode::OnGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    return;
  // First valid fix gates the dock-arrival pose seed below. Without
  // this, a robot that boots already docked could anchor on the dock
  // before GPS is ready and walk the graph over once GPS arrives.
  gps_seen_once_ = true;
  if (datum_lat_ == 0.0 && datum_lon_ == 0.0)
  {
    // Self-seed datum from first valid fix. Not ideal — operator should
    // set datum in mowgli_robot.yaml — but keeps the node from refusing
    // to start during sim/dev.
    datum_lat_ = msg->latitude;
    datum_lon_ = msg->longitude;
    datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
    RCLCPP_WARN(get_logger(),
                "fusion_graph: datum self-seeded from first fix "
                "(%.6f, %.6f) — set datum_lat/lon explicitly",
                datum_lat_,
                datum_lon_);
  }

  double mx, my;
  LatLonToMap(msg->latitude, msg->longitude, mx, my);

  // RTK wrong-fix detection — fires before any QueueGnss so a bad
  // sample never reaches iSAM2. F9P can re-solve the carrier-phase
  // ambiguity on a different integer set after a brief signal drop
  // (vegetation, multipath spike) and the new solution jumps by
  // 3-10 cm while still reporting status=GBAS_FIX with sub-cm
  // covariance. If the wheel says we didn't move, the jump is not
  // real motion — drop the sample.
  if (last_gps_map_xy_)
  {
    const double jump = std::hypot(mx - (*last_gps_map_xy_).x(), my - (*last_gps_map_xy_).y());
    // Lever-arm sweep budget. A pure body rotation by |Δθ| shifts the
    // antenna in the map frame by up to lever_arm_radius·|Δθ| with no
    // chassis translation; without this slack the gate rejects every
    // GPS sample taken while the controller is pivoting in place
    // (PRE_ROTATE, headland turn), starving the graph of corrections
    // exactly when σ_x has nothing else pinning it. NOT applied to
    // mx/my themselves — the graph's GnssLeverArmFactor handles the
    // offset; we only loosen the gate threshold.
    const double expected_pivot_jump_m =
        lever_arm_radius_m_ * abs_dtheta_since_last_gps_rad_;
    const double jump_budget = rtk_wrongfix_max_jump_m_ + expected_pivot_jump_m;
    if (jump > jump_budget && wheel_dist_since_last_gps_m_ < rtk_wrongfix_max_wheel_m_)
    {
      graph_->RecordGpsRejectWrongFix();
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "fusion_graph: RTK wrong-fix? jump=%.3f m, wheel=%.3f m, sweep_budget=%.3f m — sample dropped",
                           jump,
                           wheel_dist_since_last_gps_m_,
                           expected_pivot_jump_m);
      // Reset accumulators + cache so a repeated wrong-fix doesn't
      // permanently lock us out — once two consecutive samples agree,
      // last_gps_map_xy_ updates and we resume normal flow.
      last_gps_map_xy_ = gtsam::Vector2(mx, my);
      wheel_dist_since_last_gps_m_ = 0.0;
      abs_dtheta_since_last_gps_rad_ = 0.0;
      return;
    }
  }
  last_gps_map_xy_ = gtsam::Vector2(mx, my);
  wheel_dist_since_last_gps_m_ = 0.0;
  abs_dtheta_since_last_gps_rad_ = 0.0;

  // covariance[0] is variance of east; take sqrt for sigma. Use the
  // diagonal mean for a single sigma_xy (factor model is isotropic).
  const double var_x = msg->position_covariance[0];
  const double var_y = msg->position_covariance[4];
  double sigma = std::sqrt(0.5 * (var_x + var_y));
  if (!std::isfinite(sigma) || sigma <= 0.0)
    sigma = -1.0;  // floor

  // Robust noise model on GPS — applied unconditionally now (was
  // RTK-Float only). Field measurement 2026-05-17 (gps_stability.py,
  // 10 min stationary on RTK-Fixed 100 %) showed even Fixed solutions
  // carry σ ≈ 8-12 mm of multipath/constellation noise and the
  // occasional ~3 cm wrong-fix outlier — well above the 3 mm σ_floor.
  // Huber at k=1.345 σ keeps Gaussian inliers fully efficient and
  // smoothly downweights the rare wrong-fix outlier even if the
  // pre-graph gate above doesn't catch it (e.g. first sample of a
  // session, or a slow drift that builds up to >5 cm without a
  // detectable wheel discrepancy).
  const bool rtk_fixed = msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  // Track the freshness of RTK-Fixed for the scan-match yield gate. Updated
  // even while docked (GPS factors are suppressed below, but the freshness is
  // still the honest signal of whether absolute position is available).
  if (rtk_fixed)
  {
    last_rtk_fixed_stamp_ = this->now();
  }
  // Suppress GPS factors while the robot is on the dock.
  //
  // When `is_charging=true`, the operator-calibrated dock_pose (anchored
  // by SeedFromDockPose with σ≈10 cm) is the authoritative ground truth
  // on the robot's position. Even RTK-Fixed GPS only matches the dock
  // pose to 1-3 cm at best — and routinely shifts 5-30 cm across F9P
  // re-acquisitions on different ambiguity sets between sessions. Every
  // GnssLeverArmFactor (σ≈5 mm, ~7 Hz) accumulated while docked pulls
  // the trajectory toward the live GPS measurement and away from
  // dock_pose, so after a minute or two the EKF has walked off the
  // anchor by 10-30 cm.
  //
  // Robot on dock = stationary chassis with known position; we don't
  // need GPS to know where it is. Skipping QueueGnss preserves the
  // dock_pose anchor exactly. When the robot undocks, the next OnGnss
  // (now with is_charging=false) resumes injecting GPS factors and the
  // trajectory transitions back to GPS-tracking. seed_xy_ is also
  // skipped because TrySeedInitialPose should use dock_pose, not GPS,
  // to bootstrap if the graph somehow becomes uninitialised.
  if (last_is_charging_valid_ && last_is_charging_)
  {
    // On the dock the dock_pose prior is authoritative, but FULLY suppressing
    // GPS leaves a stationary graph whose only absolute constraint is the
    // single bootstrap prior — after ~60 zero-motion nodes iSAM2 hits an
    // indeterminate (underconstrained) linear system and ABORTS the node
    // (field 2026-05-29, crash at x62). Instead inject a deliberately WEAK
    // GPS factor (σ = dock_gps_sigma_m_, far looser than the dock prior) so
    // the system stays well-posed without walking the trajectory off the dock
    // anchor. seed_xy_ is still NOT updated (TrySeedInitialPose must bootstrap
    // from dock_pose, not GPS).
    if (graph_->IsInitialized())
    {
      graph_->QueueGnss(mx, my, std::max(sigma, dock_gps_sigma_m_), /*robust=*/true);
    }
    TrySeedInitialPose();
    return;
  }
  graph_->QueueGnss(mx, my, sigma, /*robust=*/true);
  seed_xy_ = gtsam::Vector2(mx, my);
  // Latch whether the most recent seed came from RTK-Fixed so the next
  // graph initialization can use a tight prior matching that quality.
  // Stale once seeded but TrySeedInitialPose only fires once per
  // (re)initialization, so the freshness window is the same as the
  // seed itself.
  seed_xy_rtk_fixed_ = rtk_fixed;

  // RTK-Fixed override of an autoloaded init: the persisted graph's last
  // node is almost always the dock (auto-save fires on dock arrival), so
  // booting away from the dock leaves IsInitialized()=true at the wrong
  // pose and TrySeedInitialPose() short-circuits — GPS observations then
  // fight the dock prior for many seconds before the trajectory walks
  // over. RTK-Fixed is sub-cm and trustworthy: re-anchor the latest
  // loaded node at the GPS pose with a tight prior. One-shot per boot.
  //
  // BUT — if the robot is physically on the dock at boot (is_charging),
  // SeedFromDockPose owns the anchor — it's the operator-calibrated
  // ground truth on the robot's position, independent of how the F9P's
  // RTK integer ambiguities happened to land this session. The tight
  // RTK override (σ=5mm) would dominate the looser dock seed
  // (σ=10cm) and pull the trajectory to the live GPS, defeating the
  // whole point of having a persisted dock_pose. So:
  //   * If /hardware_bridge/status hasn't been seen yet
  //     (!last_is_charging_valid_) — defer; the next /gps/fix tick
  //     will re-check once we know whether we're docked.
  //   * If docked, suppress this override entirely (latch one-shot
  //     done) and let SeedFromDockPose anchor the graph.
  //   * Otherwise (off-dock, status valid) proceed as before.
  if (rtk_fixed && autoload_succeeded_ && !rtk_autoload_override_done_ &&
      graph_->IsInitialized() && last_is_charging_valid_ && !last_is_charging_)
  {
    auto snap = graph_->LatestSnapshot();
    if (snap)
    {
      const double dx = mx - snap->pose.x();
      const double dy = my - snap->pose.y();
      const double dist = std::hypot(dx, dy);
      if (dist > rtk_autoload_override_threshold_m_)
      {
        // Use the freshest yaw seed if we have one (COG/mag have already
        // populated seed_yaw_ if they're alive); otherwise keep the
        // autoloaded yaw — it's better than nothing and the next COG
        // sample will pull it.
        const double yaw = seed_yaw_.value_or(snap->pose.theta());
        const gtsam::Pose2 anchor(mx, my, yaw);
        // σ=5mm matches RTK-Fixed reported precision; σ_yaw 5° is loose
        // enough to let COG correct it without fighting if the
        // autoloaded yaw is wrong.
        graph_->ForceAnchor(snap->node_index, anchor, 0.005, 5.0 * M_PI / 180.0);
        // ForceAnchor shifts latest_.pose without bumping node_index;
        // OnTimer's "node_index changed" check would miss it, leaving
        // map→odom anchored at the pre-override correction. Force a
        // refresh so the next OnTimer recomputes against fresh dr_*.
        t_map_odom_anchor_valid_ = false;
        rtk_autoload_override_done_ = true;
        RCLCPP_WARN(get_logger(),
                    "fusion_graph: RTK-Fixed override of autoloaded pose — "
                    "re-anchored node %lu (%.2f, %.2f) → (%.2f, %.2f), Δ=%.2f m",
                    static_cast<unsigned long>(snap->node_index),
                    snap->pose.x(),
                    snap->pose.y(),
                    mx,
                    my,
                    dist);
      }
      else
      {
        // Within threshold — autoload is consistent with RTK, no
        // override needed. Latch so we don't keep checking.
        rtk_autoload_override_done_ = true;
      }
    }
  }

  TrySeedInitialPose();
}

void FusionGraphNode::OnCogHeading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const double yaw = YawFromQuat(msg->orientation);
  // covariance[8] is yaw variance.
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0)
    var = 0.05 * 0.05;
  const double sigma = std::sqrt(var);
  graph_->QueueYaw(yaw, sigma);
  seed_yaw_ = yaw;

  // 180° yaw-flip recovery. The COG yaw is the physical travel direction
  // (wheels + GPS displacement, only emitted on a solid straight baseline),
  // so a sustained ~180° disagreement with the fused estimate means the
  // estimate is flipped — and the non-robust COG unary above can fail to pull
  // it back across the half-turn. After N consecutive flipped samples, snap
  // the yaw onto the COG (keep the estimated xy) so the robot stops believing
  // it faces backwards.
  if (cog_flip_recovery_enabled_ && graph_->IsInitialized())
  {
    // Only trust the COG for a flip recovery when it is GPS-grounded
    // (RTK-Fixed fresh). With cog_to_imu's straight-baseline gate the COGs
    // that arrive are already clean; requiring RTK-Fixed avoids snapping the
    // yaw onto a Float-era COG.
    const bool rtk_fresh =
        !cog_flip_require_rtk_ ||
        (last_rtk_fixed_stamp_ &&
         (this->now() - *last_rtk_fixed_stamp_).seconds() < scan_yield_timeout_s_);
    auto snap = graph_->LatestSnapshot();
    if (!rtk_fresh || !snap)
    {
      cog_flip_count_ = 0;
      cog_flip_prev_yaw_.reset();
    }
    else
    {
      const double d = yaw - snap->pose.theta();
      const double err = std::fabs(std::atan2(std::sin(d), std::cos(d)));
      if (err > cog_flip_threshold_rad_)
      {
        // Consecutive flipped COGs must agree WITH EACH OTHER, else the COG
        // is jittering and snapping to it would amplify rather than fix.
        bool consistent = true;
        if (cog_flip_prev_yaw_)
        {
          const double dd = yaw - *cog_flip_prev_yaw_;
          consistent =
              std::fabs(std::atan2(std::sin(dd), std::cos(dd))) < cog_flip_consistency_rad_;
        }
        cog_flip_count_ = consistent ? (cog_flip_count_ + 1) : 1;
        cog_flip_prev_yaw_ = yaw;
        const bool rate_ok =
            !last_flip_recovery_stamp_ ||
            (this->now() - *last_flip_recovery_stamp_).seconds() > cog_flip_min_interval_s_;
        if (cog_flip_count_ >= cog_flip_consecutive_n_ && rate_ok)
        {
          last_flip_recovery_stamp_ = this->now();
          const gtsam::Pose2 anchor(snap->pose.x(), snap->pose.y(), yaw);
          // Tight yaw (1°) — we are deliberately overriding the flipped
          // estimate with the physics-grounded COG heading. Keep xy at its
          // current σ-equivalent (5 mm) since only yaw is wrong.
          graph_->ForceAnchor(snap->node_index, anchor, 0.005, 1.0 * M_PI / 180.0);
          // Re-datum dead reckoning to the re-anchored node. dr_* carries the
          // OLD (flipped) heading lineage; without this reset the map→odom
          // recompute cancels against the stale dr_yaw_ and odom→base keeps
          // publishing the flipped heading — the two TF legs disagree by 180°
          // the moment the robot moves. SeedFromDockPose does the same after
          // a large yaw change (the RTK-override path doesn't, because it only
          // shifts xy and dr_yaw_ stays valid there).
          dr_x_ = 0.0;
          dr_y_ = 0.0;
          dr_yaw_ = 0.0;
          t_map_odom_anchor_valid_ = false;
          ++cog_flip_recoveries_;
          cog_flip_count_ = 0;
          RCLCPP_WARN(get_logger(),
                      "fusion_graph: 180° yaw-flip recovery — estimate %.1f° vs "
                      "COG %.1f° (Δ=%.0f°); re-anchored yaw to COG on node %lu.",
                      snap->pose.theta() * 180.0 / M_PI,
                      yaw * 180.0 / M_PI,
                      err * 180.0 / M_PI,
                      static_cast<unsigned long>(snap->node_index));
        }
      }
      else
      {
        cog_flip_count_ = 0;
        cog_flip_prev_yaw_.reset();
      }
    }
  }

  TrySeedInitialPose();
}

void FusionGraphNode::OnMagYaw(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const double yaw = YawFromQuat(msg->orientation);
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0)
    var = 0.1 * 0.1;
  // Mag yaw carries heading-dependent calibration bias (~5-15° peaks)
  // even after tilt compensation. Always robustify so when COG is also
  // active the optimizer pulls toward COG and treats mag as a soft
  // anchor that prevents free drift, not as a precise observation.
  graph_->QueueYaw(yaw, std::sqrt(var), /*robust=*/true);
  if (!seed_yaw_)
    seed_yaw_ = yaw;
  TrySeedInitialPose();
}

void FusionGraphNode::OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Resolve scan_frame -> base_footprint at the scan timestamp; if TF
  // isn't ready yet, drop this scan rather than warp it with stale
  // extrinsics.
  geometry_msgs::msg::TransformStamped t_base_scan;
  try
  {
    t_base_scan = tf_buffer_->lookupTransform(base_frame_,
                                              msg->header.frame_id,
                                              msg->header.stamp,
                                              tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException&)
  {
    return;
  }

  tf2::Transform T_base_scan;
  tf2::fromMsg(t_base_scan.transform, T_base_scan);

  std::vector<Eigen::Vector2d> pts;
  pts.reserve(msg->ranges.size());
  const double a0 = msg->angle_min;
  const double da = msg->angle_increment;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
      continue;
    const double a = a0 + da * static_cast<double>(i);
    tf2::Vector3 p_scan(r * std::cos(a), r * std::sin(a), 0.0);
    tf2::Vector3 p_base = T_base_scan * p_scan;
    pts.emplace_back(p_base.x(), p_base.y());
  }

  std::lock_guard<std::mutex> lock(scan_mu_);
  latest_scan_ = std::move(pts);
  latest_scan_valid_ = !latest_scan_.empty();
  ++scans_received_;

  // Cold-boot relocalization: if we autoloaded a graph but never had
  // a fresh GPS+COG to validate the live pose, ICP-match this first
  // scan against scans of nodes near the dock and force-anchor the
  // last loaded node at the matched pose. This unsticks the case
  // "GPS dead since boot, robot was placed back on dock manually
  // between sessions".
  if (autoload_succeeded_ && !relocalize_done_ && scan_matcher_ && latest_scan_valid_ &&
      graph_->IsInitialized())
  {
    const auto candidates =
        graph_->FindNodesNearXY(0.0, 0.0, 5.0, 5);  // dock is map origin (datum)
    double best_rmse = std::numeric_limits<double>::infinity();
    gtsam::Pose2 best_pose;
    uint64_t best_idx = 0;
    for (uint64_t idx : candidates)
    {
      auto cand_scan = graph_->GetScan(idx);
      auto cand_pose = graph_->GetPose(idx);
      if (cand_scan.empty() || !cand_pose)
        continue;
      // Use the candidate's pose as init (we expect to be close).
      auto res = scan_matcher_->Match(cand_scan, latest_scan_, *cand_pose);
      if (res.ok && res.rmse < best_rmse)
      {
        best_rmse = res.rmse;
        best_pose = res.delta;
        best_idx = idx;
      }
    }
    if (std::isfinite(best_rmse) && best_rmse < 0.10)
    {
      // Anchor the latest loaded node at the matched pose so future
      // wheel/scan factors compose from a consistent reference.
      auto snap = graph_->LatestSnapshot();
      if (snap)
      {
        graph_->ForceAnchor(snap->node_index, best_pose, 0.05, 0.05);
        // ForceAnchor shifts latest_.pose without bumping node_index;
        // invalidate the cached map→odom anchor so OnTimer recomputes.
        t_map_odom_anchor_valid_ = false;
        relocalize_done_ = true;
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: relocalized via scan match "
                    "node=%lu rmse=%.3f → (%.2f, %.2f, %.2f rad)",
                    static_cast<unsigned long>(best_idx),
                    best_rmse,
                    best_pose.x(),
                    best_pose.y(),
                    best_pose.theta());
      }
    }
  }
}

void FusionGraphNode::OnHighLevelStatus(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  // Rising-edge detection on RECORDING → other transition.
  if (last_hl_state_valid_)
  {
    constexpr uint8_t kRecording =
        mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
    if (last_hl_state_ == kRecording && msg->state != kRecording &&
        graph_->IsInitialized())
    {
      DispatchAsyncSave("recording-exit");
    }
  }
  last_hl_state_ = msg->state;
  last_hl_state_valid_ = true;
}

void FusionGraphNode::OnSetPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // Extract Pose2 from the incoming PoseWithCovariance.
  const auto& q = msg->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const gtsam::Pose2 pose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

  // Pull σ_xy and σ_yaw from the 6×6 covariance (positions [0]/[7],
  // yaw at [35]). Floor at sane minimums so a zero-cov caller doesn't
  // create a singular constraint.
  const auto& cov = msg->pose.covariance;
  const double sigma_xy = std::sqrt(std::max(cov[0], 1e-4));
  const double sigma_theta = std::sqrt(std::max(cov[35], 1e-4));

  // If we're not yet initialized (no graph loaded, no GPS+COG seed),
  // use the message as the bootstrap seed: build X_0 here.
  if (!graph_->IsInitialized())
  {
    graph_->Initialize(pose, this->now().seconds());
    // Initialize updates latest_ but the map→odom anchor was either
    // never captured or based on a now-stale init pose; force OnTimer
    // to recompute it against the new snap.pose + current dr_*.
    t_map_odom_anchor_valid_ = false;
    RCLCPP_INFO(get_logger(),
                "fusion_graph: bootstrap init from /set_pose at "
                "(%.2f, %.2f, %.2f rad)",
                pose.x(),
                pose.y(),
                pose.theta());
    return;
  }

  // Otherwise, force-anchor the latest node at the provided pose.
  auto snap = graph_->LatestSnapshot();
  if (!snap)
    return;
  graph_->ForceAnchor(snap->node_index, pose, sigma_xy, sigma_theta);
  t_map_odom_anchor_valid_ = false;  // see comment above

  // Suppress the cold-boot scan-match relocalize heuristic. The
  // explicit seed (typically dock_yaw_to_set_pose firing on a
  // charging rising edge with the calibrated dock_pose from
  // mowgli_robot.yaml) is more authoritative than ICP against the
  // persisted graph's old scans — especially when the operator
  // has re-calibrated dock_pose since the persisted session, in
  // which case the scan-match relocalize would pull the trajectory
  // back to the OLD dock anchor. Mark relocalize_done_ so OnScan
  // skips the heuristic on the very first incoming scan.
  relocalize_done_ = true;
  // Same reasoning for the RTK-autoload override path: the seed
  // we just applied is the operator's intent, GPS shouldn't fight
  // it within the threshold window.
  rtk_autoload_override_done_ = true;

  RCLCPP_INFO(get_logger(),
              "fusion_graph: re-anchored node %lu via /set_pose to "
              "(%.2f, %.2f, %.2f rad) — relocalize suppressed",
              static_cast<unsigned long>(snap->node_index),
              pose.x(),
              pose.y(),
              pose.theta());
}

// Dispatch a Save to a detached worker. Returns true if the worker
// was launched, false if a previous save is still in flight (in which
// case the caller's reason for saving — dock arrival / periodic /
// state transition — gets skipped this round; another opportunity
// will come along). GraphManager::Save now does its file I/O outside
// the graph mutex, but the file writes themselves are still serial,
// so the in-flight guard prevents two writers fighting on the same
// .graph / .scans / .meta files.
void FusionGraphNode::DispatchAsyncSave(const char* reason)
{
  bool expected = false;
  if (!save_in_flight_->compare_exchange_strong(expected, true))
  {
    RCLCPP_INFO(get_logger(),
                "fusion_graph: %s save skipped — previous save still in flight",
                reason);
    return;
  }
  std::thread(
      [graph = graph_, logger = get_logger(), prefix = graph_save_prefix_,
       reason = std::string(reason), flag = save_in_flight_]()
      {
        const bool ok = graph->Save(prefix);
        RCLCPP_INFO(logger, "fusion_graph: %s auto-save → %s", reason.c_str(),
                    ok ? "ok" : "failed");
        flag->store(false);
      })
      .detach();
}

void FusionGraphNode::OnHardwareStatus(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  // Detect rising edge of is_charging = robot just docked, OR boot
  // while already docked (no prior state known). The dock-arrival
  // path serves two purposes that used to be split across two nodes:
  //   * Save the graph to disk (auto-save).
  //   * Anchor the graph at the operator-calibrated dock pose
  //     (formerly published by dock_yaw_to_set_pose_node).
  const bool docked = msg->is_charging;
  const bool rising_edge =
      last_is_charging_valid_ && !last_is_charging_ && docked;
  const bool boot_while_docked = !last_is_charging_valid_ && docked;
  const bool dock_event = rising_edge || boot_while_docked;
  if (dock_event && auto_save_enabled_ && graph_->IsInitialized())
  {
    DispatchAsyncSave("dock-arrival");
  }
  // Dock-pose seed: rising-edge / boot-while-docked one-shot is not
  // enough on its own. /hardware_bridge/status starts streaming as
  // soon as the bridge boots, well before /gps/fix is locked
  // (~4 s in sim, several seconds on real hardware). If the dock
  // event fires before gps_seen_once_ flips, the seed is lost
  // permanently because last_is_charging_valid_ goes true and we
  // never see a fall→rise of charging unless the robot physically
  // undocks. Pre-init, keep retrying every status callback while
  // docked + GPS-seen so the seed eventually lands once GPS arrives.
  //
  // Boot-while-docked race: the graph may already be Initialized by
  // OnGpsPose before this gate sees gps_seen_once_=true, so the
  // pre_init_seed_pending check silently expires without ever firing
  // SeedFromDockPose. Backstop with a one-shot session flag that
  // catches "docked + GPS now seen + we haven't seeded yet this
  // dock session". Resets when the robot undocks (true→false on
  // last_is_charging_) so subsequent dock arrivals re-seed.
  const bool pre_init_seed_pending =
      docked && gps_seen_once_ && !graph_->IsInitialized();
  const bool session_seed_pending =
      docked && gps_seen_once_ && !dock_seeded_this_session_;
  if (pre_init_seed_pending || session_seed_pending ||
      (dock_event && gps_seen_once_))
  {
    SeedFromDockPose();
    dock_seeded_this_session_ = true;
  }
  // Undock transition: clear the one-shot so the next dock arrival
  // can re-seed via the rising-edge path.
  if (last_is_charging_valid_ && last_is_charging_ && !docked)
  {
    dock_seeded_this_session_ = false;
  }
  last_is_charging_ = docked;
  last_is_charging_valid_ = true;
}

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
  dr_x_ = 0.0;
  dr_y_ = 0.0;
  dr_yaw_ = 0.0;
  t_map_odom_anchor_valid_ = false;
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

void FusionGraphNode::OnTimer()
{
  const double now_s = this->now().seconds();

  // Run scan-matching against the previous-node scan and queue the
  // resulting between-factor before Tick — Tick consumes the queue
  // when it creates a new node.
  std::vector<Eigen::Vector2d> curr_scan;
  bool curr_valid = false;
  {
    std::lock_guard<std::mutex> lock(scan_mu_);
    if (latest_scan_valid_)
    {
      curr_scan = latest_scan_;
      curr_valid = true;
    }
  }

  if (use_scan_matching_ && scan_matcher_ && curr_valid && prev_node_scan_valid_)
  {
    // ICP init guess: wheel translation + gyro rotation accumulated
    // since the previous node. PeekAccumulator returns the current
    // accum_ contents WITHOUT resetting (Tick() below will reset).
    // A non-identity init eliminates the 30°+ pivot mismatch that
    // sends ICP's brute-force NN looking at wrong correspondences;
    // measured ~3× drop in iteration count and matching success
    // rate jump on bench fixtures with rotation > 0.3 rad.
    double dx, dy, dth_gyro, dth_wheel;
    graph_->PeekAccumulator(dx, dy, dth_gyro, dth_wheel);
    const double dth_init = (std::abs(dth_gyro) > 1e-9) ? dth_gyro : dth_wheel;
    const gtsam::Pose2 init_guess(dx, dy, dth_init);

    auto res = scan_matcher_->Match(prev_node_scan_, curr_scan, init_guess);

    // Guard rails — drop the match if any signal screams degenerate.
    // The factor would otherwise corrupt iSAM2 for many ticks (ICP
    // scan-between σ is comparable to wheel σ_x; one bad sample
    // anchors the trajectory away from truth until a strong GPS
    // observation pulls it back).
    bool drop = false;
    if (!res.ok)
    {
      // ScanMatcher returned ok=false → too few correspondences for a
      // valid 2D rigid alignment. Count and skip.
      graph_->RecordIcpRejectInliers();
      drop = true;
    }
    else if (res.rmse > icp_max_rmse_m_)
    {
      graph_->RecordIcpRejectRmse();
      drop = true;
    }
    else if (std::abs(res.delta.x()) > icp_max_delta_xy_m_ ||
             std::abs(res.delta.y()) > icp_max_delta_xy_m_ ||
             std::abs(res.delta.theta()) > icp_max_delta_theta_rad_)
    {
      // Unphysical delta — at our node period (≤ 50 ms) the
      // chassis cannot travel > 30 cm or rotate > 0.5 rad.
      graph_->RecordIcpRejectSanity();
      drop = true;
    }
    else
    {
      // Divergence from initial guess: init.between(result) gives the
      // deviation expressed in Pose2 algebra. Large divergence signals
      // degenerate scenery (symmetric haie, pure grass) where ICP
      // converged on a non-truth optimum.
      const gtsam::Pose2 dev = init_guess.between(res.delta);
      if (std::hypot(dev.x(), dev.y()) > icp_max_divergence_xy_m_ ||
          std::abs(dev.theta()) > icp_max_divergence_theta_rad_)
      {
        graph_->RecordIcpRejectDivergence();
        drop = true;
      }
    }

    if (!drop)
    {
      // Yield to RTK: if a fix was seen within scan_yield_timeout_s, inflate
      // the scan-between σ so the (subtly-biased on open lawn) ICP factor
      // can't pull map→odom away from the GPS-pinned solution. Once the fix
      // has been gone longer than the timeout, keep the tight ICP σ so
      // scan-matching carries dead-reckoning through the no-fix window.
      double sm_sigma_xy = res.sigma_xy;
      double sm_sigma_theta = res.sigma_theta;
      if (scan_yield_to_rtk_ && last_rtk_fixed_stamp_ &&
          (this->now() - *last_rtk_fixed_stamp_).seconds() < scan_yield_timeout_s_)
      {
        sm_sigma_xy = std::max(sm_sigma_xy, scan_yield_sigma_xy_);
        sm_sigma_theta = std::max(sm_sigma_theta, scan_yield_sigma_theta_);
      }
      graph_->QueueScanBetween(res.delta, sm_sigma_xy, sm_sigma_theta);
      ++scan_matches_ok_;
    }
    else
    {
      ++scan_matches_fail_;
    }
  }

  auto out = graph_->Tick(now_s);
  if (out)
  {
    // Attach the current scan to the new node (used for loop closures
    // + persistence). Use the still-valid current_scan we captured
    // above; reusing it as prev_node_scan is OK since std::move only
    // happens after this block.
    if (curr_valid)
    {
      graph_->AttachScan(out->node_index, curr_scan);
    }

    // Loop closure search — gated on loop_closure_enabled_ and on
    // having a scan matcher (we reuse it). Find candidates within
    // lc_max_dist_m_ that are at least lc_min_age_s_ old, run ICP for
    // each, accept those with rmse < lc_max_rmse_.
    if (loop_closure_enabled_ && scan_matcher_ && curr_valid)
    {
      auto candidates = graph_->FindLoopClosureCandidates(out->node_index,
                                                          lc_max_dist_m_,
                                                          lc_min_age_s_,
                                                          lc_max_candidates_);
      for (uint64_t cand_idx : candidates)
      {
        auto cand_scan = graph_->GetScan(cand_idx);
        auto cand_pose = graph_->GetPose(cand_idx);
        if (cand_scan.empty() || !cand_pose)
          continue;

        // Init guess: transform from cand to current in map frame,
        // i.e. cand.between(curr).
        const gtsam::Pose2 init = cand_pose->between(out->pose);
        auto res = scan_matcher_->Match(cand_scan, curr_scan, init);
        if (!res.ok || res.rmse > lc_max_rmse_)
          continue;

        // Skip near-identity LC factors: they pin the same pose to
        // itself and carry no constraint info, but each one costs an
        // iSAM2 update. Common at dock IDLE / before-undock revisits.
        const double dt2 = res.delta.x() * res.delta.x() + res.delta.y() * res.delta.y();
        if (dt2 < lc_min_delta_m_ * lc_min_delta_m_ &&
            std::abs(res.delta.theta()) < lc_min_delta_theta_)
        {
          continue;
        }

        graph_->AddLoopClosure(cand_idx, out->node_index, res.delta, lc_sigma_xy_, lc_sigma_theta_);
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: loop closure %lu -> %lu accepted "
                    "(rmse=%.3f, dist=%.2f m)",
                    static_cast<unsigned long>(cand_idx),
                    static_cast<unsigned long>(out->node_index),
                    res.rmse,
                    std::hypot(out->pose.x() - cand_pose->x(), out->pose.y() - cand_pose->y()));
      }
    }

    if (curr_valid)
    {
      prev_node_scan_ = std::move(curr_scan);
      prev_node_scan_valid_ = true;
    }
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
      const gtsam::Pose2 dr_at_node(dr_x_, dr_y_, dr_yaw_);
      t_map_odom_anchor_ = snap->pose.compose(dr_at_node.inverse());
      last_anchored_node_index_ = snap->node_index;
      t_map_odom_anchor_valid_ = true;
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

bool FusionGraphNode::TrySeedInitialPose()
{
  if (graph_->IsInitialized())
    return true;
  if (!seed_xy_ || !seed_yaw_)
    return false;
  // When the seed came from an RTK-Fixed fix, override the prior to
  // match the measurement quality. 5 mm is conservative w.r.t the
  // F9P's typical RTK-Fixed σ ~3 mm; tight enough that the wheel
  // between-factors can't pull the first few nodes off the GPS
  // anchor, but loose enough to absorb a few mm of antenna lever-arm
  // residual.
  std::optional<double> prior_override;
  if (seed_xy_rtk_fixed_)
    prior_override = 0.005;
  graph_->Initialize(gtsam::Pose2(seed_xy_->x(), seed_xy_->y(), *seed_yaw_),
                     this->now().seconds(),
                     prior_override);
  t_map_odom_anchor_valid_ = false;
  RCLCPP_INFO(get_logger(),
              "fusion_graph: initialized at (%.3f, %.3f, %.3f rad)%s",
              seed_xy_->x(),
              seed_xy_->y(),
              *seed_yaw_,
              seed_xy_rtk_fixed_ ? " [RTK-Fixed seed, σ=5mm prior]" : " [non-Fixed seed]");
  return true;
}

void FusionGraphNode::PublishLocalOdom()
{
  // odom→base_footprint TF + /odometry/filtered from the dead-reckoning
  // state. Replaces ekf_odom_node. Streams every OnTimer tick so the
  // local frame is ready before the graph itself initializes (Nav2's
  // local costmap and FTCController both rely on this TF, and they
  // can come up before any GPS fix has landed).
  const rclcpp::Time now = this->now();
  // Same forward-stamp trick as map→odom: under sim_time, Nav2
  // lookups can race a few ms ahead of the latest publish. Adding
  // tf_publish_lead_s_ keeps tf2 in interpolation territory instead
  // of ExtrapolationException. On real hardware (lead=0) this is a
  // no-op.
  const rclcpp::Time stamp = now + rclcpp::Duration::from_seconds(tf_publish_lead_s_);

  const geometry_msgs::msg::Quaternion q_msg = QuatFromYaw(dr_yaw_);

  geometry_msgs::msg::TransformStamped t_odom_base;
  t_odom_base.header.stamp = stamp;
  t_odom_base.header.frame_id = odom_frame_;
  t_odom_base.child_frame_id = base_frame_;
  t_odom_base.transform.translation.x = dr_x_;
  t_odom_base.transform.translation.y = dr_y_;
  t_odom_base.transform.translation.z = 0.0;
  t_odom_base.transform.rotation = q_msg;
  // Only the PRIMARY localizer may own odom→base_footprint — same single-
  // publisher rule as map→odom below. In observer mode (A/B against another
  // odom source) we still publish /odometry/filtered for comparison but must
  // NOT broadcast the TF, or two nodes would fight over the same transform.
  if (primary_mode_)
  {
    tf_broadcaster_->sendTransform(t_odom_base);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = dr_x_;
  odom.pose.pose.position.y = dr_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;
  odom.twist.twist.linear.x = wheel_vx_;
  odom.twist.twist.linear.y = 0.0;
  // Dead reckoning has unbounded drift — leave pose covariance loose
  // and let Nav2 trust the graph's /odometry/filtered_map for absolute
  // positioning. Tight roll/pitch/z so 2D consumers don't see NaN.
  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = 0.05;    // x
  odom.pose.covariance[7] = 0.05;    // y
  odom.pose.covariance[14] = 1e-9;   // z
  odom.pose.covariance[21] = 1e-9;   // roll
  odom.pose.covariance[28] = 1e-9;   // pitch
  odom.pose.covariance[35] = 0.02;   // yaw — gyro short-term σ ≈ 0.14 rad
  pub_local_odom_->publish(odom);
}

void FusionGraphNode::PublishOutputs(const TickOutput& out)
{
  // Extrapolate the last-node pose through current odom integration
  // so the published map-frame outputs reflect motion that happened
  // since the snapshot was taken. Without this, /odometry/filtered_map
  // and the map→odom TF stay glued to out.pose for up to
  // stationary_node_period_s (5 s by default) → robot looks frozen
  // in viz, then teleports when the next Tick lands.
  const gtsam::Pose2 dr_now(dr_x_, dr_y_, dr_yaw_);
  const gtsam::Pose2 extrapolated_map_base = t_map_odom_anchor_valid_
      ? t_map_odom_anchor_.compose(dr_now)
      : out.pose;

  // 1. nav_msgs/Odometry on /odometry/filtered_map.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = extrapolated_map_base.x();
  odom.pose.pose.position.y = extrapolated_map_base.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = QuatFromYaw(extrapolated_map_base.theta());

  // Pose covariance is 6x6 row-major: x, y, z, roll, pitch, yaw.
  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = out.covariance(0, 0);
  odom.pose.covariance[1] = out.covariance(0, 1);
  odom.pose.covariance[5] = out.covariance(0, 2);
  odom.pose.covariance[6] = out.covariance(1, 0);
  odom.pose.covariance[7] = out.covariance(1, 1);
  odom.pose.covariance[11] = out.covariance(1, 2);
  odom.pose.covariance[30] = out.covariance(2, 0);
  odom.pose.covariance[31] = out.covariance(2, 1);
  odom.pose.covariance[35] = out.covariance(2, 2);
  // Z, roll, pitch — clamped, give them tiny variance so consumers
  // don't choke on zero.
  odom.pose.covariance[14] = 1e-9;
  odom.pose.covariance[21] = 1e-9;
  odom.pose.covariance[28] = 1e-9;
  pub_odom_->publish(odom);

  // Rebaseline the high-rate extrapolator (item #15). The fast
  // publisher will project yaw forward from this pose until the
  // next fusion tick.
  pose_extrap_.OnFusionPose(this->now().seconds(), out.pose);

  // 1b. /imu/fg_yaw — yaw-only sensor_msgs/Imu (cov_yaw, others 1e6
  //     to disable). Published in both primary and observer mode so
  //     ekf_map_node can fuse it as imu2 absolute yaw without
  //     creating a feedback loop (fusion_graph never reads
  //     /odometry/filtered_map).
  sensor_msgs::msg::Imu yaw_msg;
  yaw_msg.header.stamp = odom.header.stamp;
  yaw_msg.header.frame_id = base_frame_;
  yaw_msg.orientation = QuatFromYaw(extrapolated_map_base.theta());
  // Roll/pitch covariances marked huge so robot_localization ignores
  // them; only orientation_covariance[8] (yaw variance) is real.
  for (auto& v : yaw_msg.orientation_covariance)
    v = 0.0;
  yaw_msg.orientation_covariance[0] = 1e6;
  yaw_msg.orientation_covariance[4] = 1e6;
  yaw_msg.orientation_covariance[8] = std::max(out.covariance(2, 2), 1e-6);
  // We don't fuse angular velocity / linear acceleration here; mark
  // them all as -1 covariance to tell consumers the field is invalid.
  yaw_msg.angular_velocity_covariance[0] = -1.0;
  yaw_msg.linear_acceleration_covariance[0] = -1.0;
  pub_fg_yaw_->publish(yaw_msg);

  // 2. TF map -> odom. Skipped in observer mode so the active
  //    map-frame primary (typically ekf_map_node) keeps single
  //    ownership of the map→odom transform. The /odometry/filtered_map
  //    publish above still happens — downstream consumers that
  //    explicitly route to /fusion_graph/odometry (via launch remap)
  //    can use it for diagnostics or A/B comparison.
  if (!primary_mode_)
    return;

  // The map→odom TF is the CONSTANT anchor captured at the moment of
  // the last graph node creation (see fusion_graph_node.hpp). Using
  // out.pose × inv(dr_now) instead would zero out current odom
  // motion in the composition map→base = map→odom × odom→base, so
  // the robot would freeze at the snapshot pose between Ticks.
  tf2::Transform T_map_odom;
  if (t_map_odom_anchor_valid_)
  {
    T_map_odom.setOrigin(
        tf2::Vector3(t_map_odom_anchor_.x(), t_map_odom_anchor_.y(), 0.0));
    tf2::Quaternion q_map_odom;
    q_map_odom.setRPY(0.0, 0.0, t_map_odom_anchor_.theta());
    T_map_odom.setRotation(q_map_odom);
  }
  else
  {
    // Fallback for the brief OnTimer just after init but before the
    // anchor has been set (caller already gates on LatestSnapshot, so
    // we shouldn't reach here in practice). Identity is safer than
    // out.pose × inv(dr_now) — at least the robot doesn't get stuck.
    T_map_odom.setIdentity();
  }

  geometry_msgs::msg::TransformStamped t_map_odom;
  // Forward-stamp by tf_publish_lead_s_ so Nav2 controller_server /
  // RotationShim queries at clock_->now() find a TF in the buffer that
  // is >= the request time and tf2 interpolates back instead of raising
  // ExtrapolationException. Default 0 (real hardware); sim sets ~0.1s.
  t_map_odom.header.stamp =
      this->now() + rclcpp::Duration::from_seconds(tf_publish_lead_s_);
  t_map_odom.header.frame_id = map_frame_;
  t_map_odom.child_frame_id = odom_frame_;
  t_map_odom.transform = tf2::toMsg(T_map_odom);
  tf_broadcaster_->sendTransform(t_map_odom);
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
