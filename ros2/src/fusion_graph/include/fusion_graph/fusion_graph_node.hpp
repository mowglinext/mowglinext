// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — ROS2 entry point for the factor-graph localizer.
//
// Subscribes to wheel odom, IMU, GPS, COG heading, and mag yaw.
// Publishes:
//   - /odometry/filtered_map (nav_msgs/Odometry, frame=map)
//   - TF map -> odom
//
// Initialization: waits for the first NavSatFix at status >= STATUS_FIX
// AND a fresh COG heading. Without those, the graph would be unanchored
// and the very first iSAM2 update would produce garbage.

#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "fusion_graph/gnss_mobile_gate.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/pose_extrapolator.hpp"
#include "fusion_graph/scan_matcher.hpp"
#include <Eigen/Core>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mowgli_interfaces/msg/high_level_status.hpp>
#include <mowgli_interfaces/msg/status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fusion_graph
{

class FusionGraphNode : public rclcpp::Node
{
public:
  explicit FusionGraphNode(const rclcpp::NodeOptions& opts = {});
  ~FusionGraphNode() override;

private:
  // ── Construction helpers ───────────────────────────────────────────
  // The constructor is split into two methods (defined in
  // fusion_graph_node_setup_params.cpp / _setup_comms.cpp) so each
  // translation unit stays within the file-size budget. DeclareParameters
  // declares + latches every ROS parameter into members; SetupCommunications
  // creates the publishers/subscriptions/services/timers.
  void DeclareParameters();
  void SetupCommunications(double node_period_s);

  // ── Callbacks ──────────────────────────────────────────────────────
  void OnWheelOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void OnCmdVelNav(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void OnCogHeading(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnMagYaw(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void OnHighLevelStatus(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);
  void OnHardwareStatus(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  // The docking server publishes /cmd_vel_docking only while it is running the
  // final graceful approach. We use that as the "dock approach in progress"
  // signal to stabilise the pose (see DockingApproachActive()).
  void OnDockingCmd(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  // True while the dock approach is active (a non-zero /cmd_vel_docking seen
  // within docking_active_timeout_s_). During the slow reverse approach the COG
  // travel-direction yaw and RTK-float position epochs both jolt the estimate,
  // which the graceful controller then chases into divergence — so we suppress
  // them here (field 2026-06-10).
  bool DockingApproachActive() const;
  // Anchor the graph at the operator-calibrated dock pose. Called on
  // the rising edge of is_charging once GPS has arrived at least once.
  // Replaces the old dock_yaw_to_set_pose_node behavior.
  void SeedFromDockPose();
  void OnSetPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void OnTimer();
  void OnPeriodicSaveTimer();

  // ── Helpers ────────────────────────────────────────────────────────
  // Flat-earth ENU projection from (lat, lon) to map frame XY.
  void LatLonToMap(double lat, double lon, double& x, double& y) const;

  // Try to seed X_0. Returns true once initialization succeeded.
  bool TrySeedInitialPose();

  // Publish TF map->odom and /odometry/filtered_map.
  void PublishOutputs(const TickOutput& out);
  // Dedicated-thread TF broadcast loop (see tf_broadcast_rate_hz_).
  void TfBroadcastLoop();
  // Publishes /odometry/filtered + odom→base_footprint TF from the
  // dead-reckoning state. Called unconditionally from OnTimer so the
  // local frame keeps streaming even before the graph initializes.
  void PublishLocalOdom();

  // Launch GraphManager::Save on a detached worker. No-op if a
  // previous async save is still running. `reason` is logged.
  void DispatchAsyncSave(const char* reason);

  // ── Members ────────────────────────────────────────────────────────
  // shared_ptr (not unique_ptr) so background save / rebase threads
  // captured by-value keep the GraphManager alive past node teardown.
  std::shared_ptr<GraphManager> graph_;
  std::unique_ptr<ScanMatcher> scan_matcher_;
  bool use_scan_matching_ = false;
  bool use_magnetometer_ = false;
  // primary_mode = false → observer: doesn't broadcast map→odom TF
  // (so ekf_map_node can keep owning it). The graph still builds
  // and persists to disk so the next boot can detect a graph and
  // promote fusion_graph to primary.
  bool primary_mode_ = true;

  // Cold-boot relocalization state.
  bool autoload_succeeded_ = false;
  bool relocalize_done_ = false;
  // Set true once an RTK-Fixed GPS sample has overridden the autoloaded
  // pose with ForceAnchor. One-shot per boot — subsequent RTK fixes flow
  // through as normal GnssLeverArmFactor observations.
  bool rtk_autoload_override_done_ = false;
  // Distance threshold (m) above which an RTK-Fixed GPS sample is
  // considered to disagree with the autoloaded pose enough to force a
  // re-anchor. Below this, the autoloaded pose is trusted and GPS just
  // contributes factors normally.
  double rtk_autoload_override_threshold_m_ = 0.3;

  // Latched datum (read from parameters at startup).
  double datum_lat_ = 0.0;
  double datum_lon_ = 0.0;
  double datum_cos_lat_ = 1.0;

  // Most recent wheel timestamp (for accumulator dt).
  std::optional<rclcpp::Time> last_wheel_stamp_;
  std::optional<rclcpp::Time> last_imu_stamp_;

  // ── Local-frame dead reckoning ──────────────────────────────────
  // odom→base_footprint TF + /odometry/filtered. Wheel vx + gyro_z
  // integrated at IMU rate (~91 Hz) so the local frame is continuous,
  // GPS-independent, and never jumps — REP-105 odom invariants.
  // Replaces the standalone robot_localization ekf_odom_node (which
  // ran the same wheel+gyro fusion at 25 Hz via a generic EKF). The
  // non-holonomic constraint is enforced implicitly: only twist.linear.x
  // is integrated, twist.linear.y is ignored (matches hardware_bridge
  // which already publishes vy=0 with tight covariance).
  double dr_x_ = 0.0;
  double dr_y_ = 0.0;
  double dr_yaw_ = 0.0;
  // Latest DR velocities, cached under tf_state_mu_ alongside dr_*. The TF
  // broadcast uses them to forward-propagate the pose by tf_publish_lead_s_ so
  // the future-stamped TF is an honest constant-velocity prediction rather than
  // the current pose mislabelled into the future (which injects ~wz·lead of yaw
  // error during pivots). dr_last_vx_eff_ is the slip-vetoed forward velocity.
  double dr_last_gz_ = 0.0;  // bias-corrected gyro yaw rate (rad/s)
  double dr_last_vx_eff_ = 0.0;  // slip-adjusted forward velocity (m/s)
  double wheel_vx_ = 0.0;  // latest forward velocity cached from /wheel_odom
  double wheel_wz_ = 0.0;  // latest wheel-derived yaw rate (slip-veto cross-check)

  // Dead-reckoning slip veto (mirrors the graph-side slip veto in
  // graph_manager.cpp, but in rate form because OnImu integrates one
  // gyro sample at a time). When the wheel encoders claim a yaw rate
  // the chassis gyro doesn't corroborate — wheels skating on wet
  // grass during a pivot — the wheel's forward velocity is a fiction
  // and must NOT accumulate into dr_x_/dr_y_, otherwise the odom
  // frame drifts metres from the real chassis path (observed
  // 2026-05-27: odom→base reached 74 m while the robot sat on a ~10 m²
  // lawn, and the resulting map→odom lever arm amplified graph-vs-DR
  // yaw differences into 100 m map-pose jumps). Rate thresholds:
  //   dr_slip_gyro_max_rad_per_s : gyro must read near-still
  //   dr_slip_wheel_min_rad_per_s: wheel must claim a real yaw rate
  // The disagreement itself is |wheel_wz_ - gz|, gated by the two
  // above so a normal coordinated turn (both agree) is never vetoed.
  double dr_slip_gyro_max_rad_per_s_ = 0.15;
  double dr_slip_wheel_min_rad_per_s_ = 0.15;

  // ── map→odom static anchor ──────────────────────────────────────
  // The graph publishes one (map-frame) pose per Tick — every
  // node_period_s when moving, or every stationary_node_period_s
  // (5 s default) when the chassis appears still. Between Ticks the
  // snapshot pose is unchanged. If we recomputed T_map_odom on every
  // OnTimer as out.pose × inv(dr_*[NOW]), the composition
  //   map→base = T_map_odom × odom→base[NOW]
  // cancels to out.pose (constant) — the robot looks glued to the
  // last node pose in viz even while the chassis is genuinely
  // moving, and teleports the moment a new node lands. Real
  // hardware no-LiDAR sessions saw 5 s freezes followed by big
  // jumps as a result.
  //
  // The correct map→odom is a constant transform that captures the
  // map-vs-odom offset at the moment of the last graph node, namely
  //   T_map_odom = node_pose × inv(dr_at_node)
  // Re-broadcast at OnTimer rate (so the TF buffer stays fresh) but
  // recomputed only when a new node lands. Then
  //   map→base = T_map_odom × odom→base[NOW]
  // correctly extrapolates the last-node pose through current odom
  // integration, and /odometry/filtered_map publishes the same
  // extrapolated pose.
  gtsam::Pose2 t_map_odom_anchor_{0.0, 0.0, 0.0};
  uint64_t last_anchored_node_index_ = std::numeric_limits<uint64_t>::max();
  // Atomic: flipped to false from several executor-thread sites
  // (ForceAnchor / Initialize / clear paths) and read by the TF
  // broadcast thread. The anchor VALUE is only ever written together
  // with valid=true under tf_state_mu_ (OnTimer capture site), so a
  // reader holding the mutex always sees a coherent {value, valid}
  // pair; the lock-free false-stores can at worst suppress the
  // map→odom broadcast for one cycle, which is the intent.
  std::atomic<bool> t_map_odom_anchor_valid_{false};

  // Latched seeds for initialization.
  std::optional<gtsam::Vector2> seed_xy_;  // from latest GPS
  std::optional<double> seed_yaw_;  // from latest COG/mag

  // --- 180° yaw-flip recovery -----------------------------------------------
  // COG yaw is the PHYSICAL travel direction (wheels + GPS displacement) and
  // is only emitted on a solid straight-line baseline — it cannot lie about
  // which way the robot is facing. If the fused estimate disagrees with it by
  // ~180° for several consecutive COG samples, the estimate is flipped (a seed
  // that initialised backwards, or a gyro chain that jumped during a pivot
  // where COG was gated off). The normal non-robust COG unary can fail to pull
  // it back across the half-turn, so when this persistent disagreement is seen
  // we force-re-anchor the yaw onto the COG (trusting the physics). Gated on a
  // large threshold + N consecutive samples so it never fires in normal
  // operation. Field 2026-05-29: "robot thinks it faces backwards, drives in
  // reverse toward a goal that is in front."
  bool cog_flip_recovery_enabled_ = true;
  double cog_flip_threshold_rad_ = 2.618;  // ~150°
  int cog_flip_consecutive_n_ = 3;
  int cog_flip_count_ = 0;
  uint64_t cog_flip_recoveries_ = 0;  // diagnostic counter
  // Robustness gates so the recovery is a reliable safety net, not an
  // amplifier (it fired repeatedly on garbage COG during an FTC oscillation,
  // field 2026-05-29): require RTK-Fixed fresh (COG GPS-grounded), require the
  // consecutive flipped COGs to agree WITH EACH OTHER (so a jittering COG
  // can't drive it), and rate-limit re-fires.
  bool cog_flip_require_rtk_ = true;
  double cog_flip_min_interval_s_ = 10.0;
  double cog_flip_consistency_rad_ = 0.52;  // ~30°
  std::optional<double> cog_flip_prev_yaw_;
  std::optional<rclcpp::Time> last_flip_recovery_stamp_;
  // True when seed_xy_ was set from an RTK-Fixed fix (carr_soln=2).
  // Drives the prior sigma at Initialize: tight (sub-cm) when set,
  // configured default (cm-decimetre) otherwise. Without this the
  // 50 mm default prior dominates the first ~10 nodes after a clear
  // even when GPS sigma is 3 mm, and the wheel non-holo σ_y=5 mm
  // pins the trajectory away from the true GPS position.
  bool seed_xy_rtk_fixed_ = false;

  // Scan matching state.
  std::mutex scan_mu_;
  std::vector<Eigen::Vector2d> latest_scan_;  // latest scan in body frame
  bool latest_scan_valid_ = false;
  std::vector<Eigen::Vector2d> prev_node_scan_;  // scan stored at last node
  bool prev_node_scan_valid_ = false;

  // Frame names.
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_footprint";

  // Forward-stamps the published map→odom TF by this many seconds so that
  // controllers/costmaps querying lookupTransform at clock_->now() always
  // find a TF stamp in the buffer that is >= their request time, letting
  // tf2 interpolate back instead of throwing ExtrapolationException. Only
  // needed under sim_time, where Nav2 cycles can fall a few ms ahead of
  // the latest publish; safe on real hardware too because map→odom moves
  // very slowly relative to typical lead times (~100 ms = sub-cm error
  // even at full transit speed).
  double tf_publish_lead_s_ = 0.0;

  // ── Decoupled TF broadcast thread ───────────────────────────────
  // OnTimer runs graph_->Tick() (iSAM2 update) BEFORE publishing TF.
  // On a large graph under Pi CPU load a single Tick can take
  // 150-250 ms, and the 1 Hz diagnostics callback (GetAllPoses on
  // the full graph) can block the executor similarly — during those
  // stalls neither map→odom nor odom→base_footprint gets refreshed
  // and Nav2's controller_server throws ExtrapolationException even
  // after waiting its 100 ms transform timeout (field 2026-06-07:
  // RotationShim aborted ~19×/90 s, robot twitched in place instead
  // of following the path). Same failure class as the iSAM2 rebase
  // stall (see maintenance_timer_), which was fixed by moving the
  // rebase off-thread.
  //
  // The TF legs are cheap to compute (constant map→odom anchor +
  // integrated dr_* state), so broadcast them from a dedicated
  // thread at tf_broadcast_rate_hz, immune to executor stalls.
  // tf_state_mu_ guards the dr_* / anchor state shared between the
  // executor thread (writers) and the broadcast thread (reader).
  // Set tf_broadcast_rate_hz <= 0 to disable the thread and fall
  // back to inline OnTimer publishing.
  std::mutex tf_state_mu_;
  double tf_broadcast_rate_hz_ = 20.0;
  std::thread tf_thread_;
  std::atomic<bool> tf_thread_stop_{false};

  // Subscriptions.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_nav_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_cog_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_mag_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_hw_status_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_docking_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_set_pose_;

  // Save-graph service handle.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  // Clear-graph service handle (wipes iSAM2 + scans, keeps the node alive).
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;

  // Persistence + loop-closure config.
  std::string graph_save_prefix_;
  bool loop_closure_enabled_ = false;
  double lc_max_dist_m_ = 5.0;
  double lc_min_age_s_ = 30.0;
  size_t lc_max_candidates_ = 3;
  double lc_max_rmse_ = 0.10;  // ICP RMSE acceptance gate
  double lc_sigma_xy_ = 0.05;
  double lc_sigma_theta_ = 0.02;
  // Skip a loop-closure if its delta is so small it carries no
  // information (robot was effectively stationary at the candidate's
  // position) — saves iSAM2 bandwidth on dock-clutter revisits.
  double lc_min_delta_m_ = 0.05;  // m
  double lc_min_delta_theta_ = 0.05;  // rad (~3°)
  // Skip loop-closure GENERATION entirely while an RTK-Fixed sample is fresh
  // (within scan_yield_timeout_s). Under RTK-Fixed the GPS factor is already an
  // absolute mm-accurate constraint, so a loop closure carries ~no new
  // information — but every accepted LC still adds a factor to iSAM2. Over a long
  // stationary dwell (dock IDLE / charging) that is an UNBOUNDED factor leak: it
  // OOM-killed the node 2026-06-09 (graph → 10k nodes, 2307 LC factors, SIGKILL).
  // The scan-yield σ-inflation only stopped LC from biasing the pose; it still
  // ADDED the factor. Skipping generation bounds memory in the normal
  // (RTK-Fixed) operating state. When the fix goes stale past the timeout LC
  // re-enables, so it still carries global consistency through no-fix (tree)
  // windows. Default true.
  bool lc_skip_when_rtk_fixed_ = true;

  // Publishers.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  // /odometry/filtered — local-frame dead reckoning (REP-105 odom),
  // replaces what ekf_odom_node used to publish. Same topic name so
  // Nav2 / GUI consumers need no rewiring.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_fg_yaw_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  // High-rate extrapolated pose (item #15). Same Odometry shape as
  // /odometry/filtered_map but at 100 Hz, with yaw projected forward
  // by the latest IMU gyro sample. Position is the unmodified last
  // fusion-published value. See PoseExtrapolator for the math.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_fast_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF for odom->base_footprint (we publish map->odom; need to compose
  // with the local EKF's odom->base_footprint to back-compute).
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::TimerBase::SharedPtr periodic_save_timer_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
  // 100 Hz high-rate pose republisher (item #15). Only runs when
  // fast_pose_publish_rate_hz_ > 0 in yaml; default off so existing
  // installs aren't surprised by extra topic traffic.
  rclcpp::TimerBase::SharedPtr fast_pose_timer_;
  PoseExtrapolator pose_extrap_;
  double fast_pose_publish_rate_hz_ = 0.0;

  // Memory + compute bounding parameters.
  uint64_t scan_retention_nodes_ = 18000;  // 30 min @ 10 Hz
  uint64_t isam2_rebase_every_nodes_ = 2000;
  uint64_t last_rebase_index_ = 0;

  // Auto-checkpoint state.
  bool auto_save_enabled_ = true;
  uint8_t last_hl_state_ = 0;  // HighLevelStatus.state
  bool last_hl_state_valid_ = false;
  bool last_is_charging_ = false;
  bool last_is_charging_valid_ = false;
  // One-shot per dock session: ensures SeedFromDockPose fires exactly
  // once per docked interval, even when the boot-while-docked race
  // means neither the rising_edge nor boot_while_docked branches can
  // catch the moment gps_seen_once_ flips true. Reset on undock so
  // the next dock arrival re-seeds.
  bool dock_seeded_this_session_ = false;

  // Dock-arrival pose seed (formerly the dock_yaw_to_set_pose node).
  // On the rising edge of is_charging we anchor the graph at the
  // operator-calibrated dock pose. The two are deduplicated by
  // last_is_charging_/last_is_charging_valid_ above.
  double dock_pose_x_ = 0.0;
  double dock_pose_y_ = 0.0;
  double dock_pose_yaw_ = 0.0;
  double dock_pose_yaw_sigma_rad_ = 0.035;
  bool gps_seen_once_ = false;  // gate dock seed on at least one GPS arrival

  // Per-tick counters for diagnostics.
  uint64_t scans_received_ = 0;
  uint64_t scan_matches_ok_ = 0;
  uint64_t scan_matches_fail_ = 0;

  // RTK mobile wrong-fix detection state. Raw GNSS cadence and accepted GNSS
  // reference are tracked separately:
  //   - last_raw_gps_fix_stamp_: every received fix, for dt/frequency stats
  //   - last_accepted_gps_map_xy_/stamp_: only fixes that entered the graph
  //
  // The gate always compares the new fix against the LAST ACCEPTED reference.
  // That prevents a rejected wrong-fix cluster from becoming the new baseline.
  // Wheel motion is accumulated since the last accepted fix for the same
  // reason; it only resets when the sample is accepted or downweighted into
  // the graph. Rejections are counted in GraphStats.gps_rejects_wrongfix; mild
  // outliers are downweighted instead of hard-dropped.
  std::optional<gtsam::Vector2> last_accepted_gps_map_xy_;
  std::optional<rclcpp::Time> last_raw_gps_fix_stamp_;
  std::optional<rclcpp::Time> last_accepted_gps_fix_stamp_;
  double wheel_dist_since_last_accepted_gps_m_ = 0.0;
  std::mutex cmd_vel_mu_;
  double last_cmd_vx_mps_ = 0.0;
  std::optional<rclcpp::Time> last_cmd_stamp_;
  double gps_sigma_floor_m_ = 0.003;
  double rtk_wrongfix_gps_sigma_multiplier_ = 2.0;
  double rtk_wrongfix_min_margin_m_ = 0.01;
  double rtk_wrongfix_downweight_innovation_multiplier_ = 2.0;
  double rtk_wrongfix_downweight_sigma_multiplier_ = 4.0;
  uint64_t gps_downweights_wrongfix_mobile_ = 0;
  std::mutex gps_gate_diag_mu_;
  GnssMobileGateMetrics last_gps_gate_metrics_;
  bool last_gps_gate_metrics_valid_ = false;
  // Dock-pose hold while charging: re-assert a firm ForceAnchor at the FULL
  // dock_pose (x,y,yaw) ONCE PER NEW NODE, replacing the weak live-GPS factor
  // that walked the docked pose off the anchor (field 2026-06-10: 11.5 cm + 53°
  // walk over a charge dwell → re-docking aimed at the wrong target). One prior
  // per node keeps the count bounded by the sliding window (vs a fixed timer
  // posting several priors onto the same stationary node). It keeps iSAM2
  // well-posed (a fresh absolute constraint on each node — replaces the old weak
  // GPS that the 2026-05-29 indeterminate-system guard needed) AND pins position
  // and yaw with no live GPS to drag it off.
  double dock_reanchor_sigma_xy_m_ = 0.03;
  uint64_t last_dock_reanchor_node_ = std::numeric_limits<uint64_t>::max();
  // Dock-approach pose stabilisation. While /cmd_vel_docking is active the
  // graceful dock controller is steering the final approach; the slow reverse
  // motion makes the COG travel-direction yaw unreliable and the RTK fixed↔float
  // per-epoch flicker jolts position — both jolt the fused pose, which the
  // controller then chases into divergence (field 2026-06-10, dockmon trace).
  // During the approach we drop the COG yaw factor (gyro carries yaw) and reject
  // non-Fixed GPS epochs (hold position through the flicker) so the controller
  // sees a stable target.
  double docking_active_timeout_s_ = 1.0;  // /cmd_vel_docking freshness
  bool gate_cog_during_docking_ = true;
  bool gate_float_gps_during_docking_ = true;
  std::optional<rclcpp::Time> last_docking_cmd_stamp_;
  // ICP guard-rail thresholds — see GraphParams comments for the
  // physical intuition. Declared as ROS params so we can tighten or
  // loosen them in mowgli_robot.yaml without a rebuild.
  double icp_max_rmse_m_ = 0.10;
  double icp_max_delta_xy_m_ = 0.30;
  double icp_max_delta_theta_rad_ = 0.50;
  double icp_max_divergence_xy_m_ = 0.15;
  double icp_max_divergence_theta_rad_ = 0.35;

  // --- Scan-match yield-to-RTK gating ---------------------------------------
  // On a feature-poor open lawn, ICP scan-between factors (σ_xy ≈ 2 cm) are
  // subtly biased and, chained across many nodes, pull map→odom by 15-60 cm
  // even while RTK-Fixed GPS (σ ≈ 7 mm) is available — which jitters every
  // map-frame consumer (dock target, coverage strips) and broke docking
  // (field 2026-05-29: dock "drove to the side" as the target shifted under
  // it). Fix: when RTK-Fixed has been seen within scan_yield_timeout_s,
  // inflate the scan-between σ to scan_yield_sigma_* so GPS dominates and the
  // map frame stays pinned; once the fix is lost for longer than the timeout,
  // fall back to the tight ICP σ so scan-matching carries the estimate
  // through the no-fix window (its whole reason for existing). Set
  // scan_yield_to_rtk_=false to keep scan-matching always tight (feature-rich
  // sites). This does NOT affect the use_scan_matching_=false baseline.
  bool scan_yield_to_rtk_ = true;
  double scan_yield_timeout_s_ = 2.0;
  double scan_yield_sigma_xy_ = 0.5;
  double scan_yield_sigma_theta_ = 0.3;
  std::optional<rclcpp::Time> last_rtk_fixed_stamp_;

  // ── RTK-anchored keyframe map (scan-to-keyframe absolute localization) ──
  // Requires use_scan_matching_ (reuses scan_matcher_ + the scan subscription
  // + the ICP guard rails). CAPTURE: under stable RTK-Fixed, freeze the
  // GPS-fused node pose + scan as a keyframe (builds the absolute map). APPLY:
  // during RTK-Float, match the live scan to nearby keyframes and queue a
  // PoseTranslationPrior that pins absolute xy — the mechanism that holds <2 cm
  // through a Float window where dead-reckoning would otherwise drift. Default
  // OFF. See graph_manager_keyframe.cpp + the OnScan capture/apply blocks.
  bool use_keyframe_map_ = false;
  double kf_capture_sigma_max_m_ = 0.01;  // max GPS σ to allow a capture
  int kf_capture_rtk_debounce_ = 3;  // consecutive RTK-Fixed epochs first
  double kf_capture_max_omega_ = 0.10;  // rad/s — no capture while pivoting
  double kf_spacing_m_ = 0.5;  // min move between captures
  double kf_match_max_dist_m_ = 3.0;  // apply-side keyframe search radius
  size_t kf_max_candidates_ = 5;
  double kf_apply_sigma_floor_m_ = 0.02;  // floor on the PoseTranslationPrior σ
  double kf_engage_age_s_ = 0.3;  // engage apply when Fixed older than this
  // Latches updated in OnGnss for the capture gate.
  double last_gps_sigma_ = -1.0;  // most-recent valid GPS σ (m); <0 = none
  int rtk_fixed_streak_ = 0;  // consecutive RTK-Fixed epochs
  std::optional<gtsam::Vector2> last_kf_capture_xy_;
  uint64_t kf_matches_ok_ = 0;
  uint64_t kf_matches_fail_ = 0;

  // In-flight guards for the async maintenance jobs. Save and rebase
  // each run in a detached worker so the executor callback returns
  // immediately; the atomic flag prevents a second worker from
  // launching while the first is still running. (Save would race on
  // the output files; rebase is internally guarded too but skipping
  // here avoids paying the snapshot cost for nothing.)
  // shared_ptr because a detached worker may outlive the node at
  // shutdown — the worker captures this shared_ptr by value and
  // writes false on completion without touching `this`.
  std::shared_ptr<std::atomic<bool>> save_in_flight_ =
      std::make_shared<std::atomic<bool>>(false);
  std::shared_ptr<std::atomic<bool>> rebase_in_flight_ =
      std::make_shared<std::atomic<bool>>(false);
};

}  // namespace fusion_graph
