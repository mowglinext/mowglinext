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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "fusion_graph/graph_manager.hpp"
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

private:
  // ── Callbacks ──────────────────────────────────────────────────────
  void OnWheelOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void OnCogHeading(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnMagYaw(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void OnHighLevelStatus(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);
  void OnHardwareStatus(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
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
  double dock_pose_yaw_ = 0.0;

  // Latched datum (read from parameters at startup).
  double datum_lat_ = 0.0;
  double datum_lon_ = 0.0;
  double datum_cos_lat_ = 1.0;

  // Most recent wheel timestamp (for accumulator dt).
  std::optional<rclcpp::Time> last_wheel_stamp_;
  std::optional<rclcpp::Time> last_imu_stamp_;

  // Latched seeds for initialization.
  std::optional<gtsam::Vector2> seed_xy_;  // from latest GPS
  std::optional<double> seed_yaw_;  // from latest COG/mag
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

  // Subscriptions.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_cog_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_mag_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_hl_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_hw_status_;
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

  // Publishers.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_fg_yaw_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF for odom->base_footprint (we publish map->odom; need to compose
  // with the local EKF's odom->base_footprint to back-compute).
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::TimerBase::SharedPtr periodic_save_timer_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;

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

  // Per-tick counters for diagnostics.
  uint64_t scans_received_ = 0;
  uint64_t scan_matches_ok_ = 0;
  uint64_t scan_matches_fail_ = 0;

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
