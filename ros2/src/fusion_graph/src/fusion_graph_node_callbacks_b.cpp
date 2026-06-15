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

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

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
          {
            // tf_state_mu_: dr_* / anchor are read concurrently by
            // TfBroadcastLoop.
            std::lock_guard<std::mutex> lock(tf_state_mu_);
            dr_x_ = 0.0;
            dr_y_ = 0.0;
            dr_yaw_ = 0.0;
            t_map_odom_anchor_valid_ = false;
          }
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
    if (last_hl_state_ == kRecording && msg->state != kRecording && graph_->IsInitialized())
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
      [graph = graph_,
       logger = get_logger(),
       prefix = graph_save_prefix_,
       reason = std::string(reason),
       flag = save_in_flight_]()
      {
        const bool ok = graph->Save(prefix);
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
