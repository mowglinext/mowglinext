// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — input callbacks: docking/cog/mag/scan/status/set-pose. (The node implementation
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

#include "fusion_graph/cog_flip_recovery.hpp"
#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"
#include "fusion_graph/yaw_gates.hpp"

namespace fusion_graph
{

bool FusionGraphNode::RtkFixedReceiptIsFresh(const double maximum_age_s) const
{
  if (!last_rtk_fixed_stamp_ || !std::isfinite(maximum_age_s) || maximum_age_s < 0.0)
  {
    return false;
  }
  const auto maximum_age_ns = static_cast<std::int64_t>(maximum_age_s * 1.0e9);
  const rclcpp::Time ros_now = this->now();
  return mowgli_interfaces::gnss_observation_freshness::IsReceiptFresh(
      last_rtk_fixed_stamp_->nanoseconds(), ros_now.nanoseconds(), maximum_age_ns);
}

void FusionGraphNode::OnDockingCmd(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  // Stamp only NON-ZERO docking commands — the graceful controller emits ~0
  // between phases, and latching on a trailing zero would wrongly extend the
  // "approach active" window.
  if (std::abs(msg->twist.linear.x) > 1e-3 || std::abs(msg->twist.angular.z) > 1e-3)
  {
    last_docking_cmd_stamp_ = this->now();
  }
}

bool FusionGraphNode::DockingApproachActive() const
{
  return last_docking_cmd_stamp_.has_value() &&
         (this->now() - *last_docking_cmd_stamp_).seconds() < docking_active_timeout_s_;
}

void FusionGraphNode::OnCogHeading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // Suppress the COG yaw factor during the dock approach. The COG is the
  // physical travel direction; in the slow reverse approach it is noise-
  // dominated and jolts the fused yaw, which the graceful controller chases
  // into divergence (field 2026-06-10). Gyro carries yaw over the short approach.
  // NEVER gate a yet-uninitialized graph: TrySeedInitialPose needs the COG yaw
  // seed, so gating it during seeding stalls re-initialization (field 2026-06-10,
  // clear_graph just before docking left the graph stuck at total_nodes=0).
  if (gate_cog_during_docking_ && DockingApproachActive() && graph_->IsInitialized())
  {
    return;
  }
  // OpenMower-style single-antenna heading discipline (yaw_gates.hpp,
  // unit-tested). A COG derived from Float/NO_FIX GPS or from slow/reverse
  // motion is heading-garbage and corrupts the weakly-observable yaw (map→odom
  // balloons → lever-arm amplifies jitter into position jumps → robot drives
  // out of bounds). Apply it only when RTK-Fixed AND translating forward; else
  // the gyro carries yaw. Before init the seed always needs it.
  const bool rtk_fresh = RtkFixedReceiptIsFresh(cog_rtk_max_age_s_);
  if (!CogShouldApply(
          graph_->IsInitialized(), rtk_fresh, wheel_vx_, cog_require_rtk_, cog_min_speed_mps_))
  {
    ++cog_rtk_gated_;
    return;
  }
  const double yaw = YawFromQuat(msg->orientation);
  // Soft σ (floored): COG only TRENDS the gyro heading, never snaps to a noisy
  // per-fix course. covariance[8] is the message yaw variance.
  const double sigma = CogEffectiveSigma(msg->orientation_covariance[8], cog_min_sigma_rad_);
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
    const bool rtk_fresh = !cog_flip_require_rtk_ || RtkFixedReceiptIsFresh(cog_rtk_max_age_s_);
    auto snap = graph_->LatestSnapshot();
    if (!rtk_fresh || !snap)
    {
      cog_flip_count_ = 0;
      cog_flip_prev_yaw_.reset();
    }
    else
    {
      std::optional<double> seconds_since_last_recovery;
      if (last_flip_recovery_stamp_)
      {
        seconds_since_last_recovery = (this->now() - *last_flip_recovery_stamp_).seconds();
      }
      const CogFlipRecoveryCfg cfg{cog_flip_threshold_rad_,
                                   cog_flip_consistency_rad_,
                                   cog_flip_consecutive_n_,
                                   cog_flip_min_interval_s_};
      // See cog_flip_recovery.hpp for the pure decision function + unit
      // tests (test_cog_flip_recovery.cpp).
      const CogFlipRecoveryResult flip = CogFlipRecoveryFeed(yaw,
                                                             snap->pose.theta(),
                                                             seconds_since_last_recovery,
                                                             cfg,
                                                             cog_flip_count_,
                                                             cog_flip_prev_yaw_);
      if (flip.should_anchor)
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
        {
          // tf_state_mu_: dr_* / anchor are read concurrently by
          // TfBroadcastLoop.
          std::lock_guard<std::mutex> lock(tf_state_mu_);
          dr_x_ = 0.0;
          dr_y_ = 0.0;
          dr_yaw_ = 0.0;
          ResetLidarTiming();
          t_map_odom_anchor_valid_ = false;
        }
        ++cog_flip_recoveries_;
        RCLCPP_WARN(get_logger(),
                    "fusion_graph: 180° yaw-flip recovery — estimate %.1f° vs "
                    "COG %.1f° (Δ=%.0f°); re-anchored yaw to COG on node %lu.",
                    snap->pose.theta() * 180.0 / M_PI,
                    yaw * 180.0 / M_PI,
                    flip.err_rad * 180.0 / M_PI,
                    static_cast<unsigned long>(snap->node_index));
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
  latest_scan_stamp_s_ = rclcpp::Time(msg->header.stamp).seconds();
  ++scans_received_;
}

void FusionGraphNode::OnHighLevelStatus(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  // Rising-edge detection on RECORDING → other transition.
  if (last_hl_state_valid_)
  {
    constexpr uint8_t kRecording =
        mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
    if (auto_save_enabled_ && last_hl_state_ == kRecording && msg->state != kRecording &&
        graph_->IsInitialized())
    {
      DispatchAsyncSave("recording-exit");
    }
  }
  last_hl_state_ = msg->state;
  last_hl_state_name_ = msg->state_name;
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
  ResetLidarTiming();
  t_map_odom_anchor_valid_ = false;  // see comment above

  // The explicit seed is the operator's intent. Suppress the one-shot
  // RTK-autoload override so it cannot immediately undo this command.
  rtk_autoload_override_done_ = true;

  RCLCPP_INFO(get_logger(),
              "fusion_graph: re-anchored node %lu via /set_pose to "
              "(%.2f, %.2f, %.2f rad)",
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
  if (lidar_submaps_)
  {
    lidar_submaps_->RequestSave();
    lidar_mapper_ = lidar_submaps_->mapper();
  }
  std::thread(
      [graph = graph_,
       logger = get_logger(),
       prefix = graph_save_prefix_,
       reason = std::string(reason),
       flag = save_in_flight_]()
      {
        bool ok = graph->Save(prefix);
        RCLCPP_INFO(logger,
                    "fusion_graph: %s auto-save → %s",
                    reason.c_str(),
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
  const bool rising_edge = last_is_charging_valid_ && !last_is_charging_ && docked;
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
  const bool pre_init_seed_pending = docked && gps_seen_once_ && !graph_->IsInitialized();
  const bool session_seed_pending = docked && gps_seen_once_ && !dock_seeded_this_session_;
  if (pre_init_seed_pending || session_seed_pending || (dock_event && gps_seen_once_))
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

}  // namespace fusion_graph
