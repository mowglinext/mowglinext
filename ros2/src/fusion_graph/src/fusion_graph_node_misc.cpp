// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — dock-pose seed + periodic save. (The node implementation is split across
// several translation units to keep each file within the project's 600-line budget; all share
// fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include <algorithm>
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
#include "fusion_graph/rtk_wrongfix_gate.hpp"

namespace fusion_graph
{

gtsam::Pose2 FusionGraphNode::ContinuousDrPoseLocked() const
{
  return dr_continuous_from_raw_.compose(gtsam::Pose2(dr_x_, dr_y_, dr_yaw_));
}

void FusionGraphNode::RecenterDrLocked(bool reset_heading)
{
  // Only the internal numerical gauge changes. The published odom gauge and
  // every retained source pose remain continuous across this recenter.
  const auto continuous = ContinuousDrPoseLocked();
  dr_x_ = 0.0;
  dr_y_ = 0.0;
  if (reset_heading)
    dr_yaw_ = 0.0;
  const gtsam::Pose2 raw(dr_x_, dr_y_, dr_yaw_);
  dr_continuous_from_raw_ = continuous.compose(raw.inverse());
  lidar_anchor_odom_.RebaseRaw(Sophus::SE2d(raw.theta(), raw.translation()));
}

void FusionGraphNode::ResetMotionSourceEpoch()
{
  ++integration_epoch_;
  graph_->ResetConsumedMotionEpoch(integration_epoch_);
  last_wheel_stamp_.reset();
  last_imu_stamp_.reset();
  wheel_vx_ = wheel_vy_ = wheel_wz_ = 0.0;
  held_wheel_source_ = {};
  dr_source_history_.clear();
  dr_source_baseline_ns_.reset();
  anchored_dr_source_ns_.reset();
  last_lidar_source_node_index_ = std::numeric_limits<uint64_t>::max();
  seed_source_ns_.reset();
  seed_yaw_.reset();
  seed_yaw_source_ns_.reset();
  pending_explicit_seed_.reset();
  pending_cog_flip_checks_.clear();
  cog_flip_count_ = 0;
  cog_flip_prev_yaw_.reset();
  seed_xy_.reset();
  motion_source_fault_ = false;
  dock_seeded_this_session_ = false;
  dock_seed_fallback_done_ = false;
  pose_extrap_ = PoseExtrapolator();
  fast_pose_source_ns_.reset();
  force_pub_resync_.store(true, std::memory_order_release);
  last_map_pub_s_ = -1.0;
  {
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    integrated_source_ns_.reset();
    dr_last_gz_ = dr_last_vx_eff_ = 0.0;
    t_map_odom_anchor_valid_ = false;
    t_map_odom_pub_shared_valid_ = false;
  }
  ResetLidarTiming();
}

void FusionGraphNode::ResetForExplicitGraphClear()
{
  ObserveMotionClockEpoch();
  // An explicit stationary clear can retain a heading prior at an already
  // consumed source state. Real clock/gap resets never use this exception.
  const auto source_ns = integrated_source_ns_;
  const auto snapshot = graph_->LatestSnapshot();
  std::optional<double> heading;
  if (source_ns && !motion_source_fault_ && last_imu_stamp_ &&
      last_imu_stamp_->nanoseconds() == *source_ns && snapshot &&
      snapshot->source_epoch == integration_epoch_)
  {
    const auto dr_at_snapshot = DrPoseAtSource(integration_epoch_, snapshot->source_stamp_ns);
    const auto dr_at_source = DrPoseAtSource(integration_epoch_, *source_ns);
    if (dr_at_snapshot && dr_at_source)
      heading = snapshot->pose.compose(dr_at_snapshot->inverse()).compose(*dr_at_source).theta();
  }
  const auto imu_stamp = last_imu_stamp_;
  const auto wheel_stamp = last_wheel_stamp_;
  const auto held_wheel = held_wheel_source_;
  const double vx = wheel_vx_;
  const double vy = wheel_vy_;
  const double wz = wheel_wz_;
  const double dr_gz = dr_last_gz_;
  const double dr_vx = dr_last_vx_eff_;

  ResetMotionSourceEpoch();
  seed_xy_rtk_fixed_ = false;
  // Keep observation identity/high-water state: a cached pre-clear fix must
  // not become new evidence merely because the operator discarded the graph.
  last_rtk_fixed_stamp_.reset();
  last_usable_gnss_stamp_.reset();
  last_gps_map_xy_.reset();
  ResetRtkWrongFixAccumulators(wheel_dist_since_last_gps_m_, abs_dtheta_since_last_gps_rad_);
  {
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    dr_x_ = dr_y_ = dr_yaw_ = 0.0;
    dr_continuous_from_raw_ = gtsam::Pose2();
  }
  if (!heading)
    return;

  // Restore only the known accepted baseline and held inputs. Graph factors,
  // compatibility accumulators and learned bias remain reset.
  last_imu_stamp_ = imu_stamp;
  last_wheel_stamp_ = wheel_stamp;
  held_wheel_source_ = held_wheel;
  wheel_vx_ = vx;
  wheel_vy_ = vy;
  wheel_wz_ = wz;
  graph_->SetConsumedMotionBaseline(integration_epoch_, *source_ns);
  dr_source_baseline_ns_ = source_ns;
  dr_source_baseline_pose_ = gtsam::Pose2();
  seed_yaw_ = heading;
  seed_yaw_source_ns_ = source_ns;
  seed_yaw_source_epoch_ = integration_epoch_;
  {
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    integrated_source_ns_ = source_ns;
    dr_last_gz_ = dr_gz;
    dr_last_vx_eff_ = dr_vx;
  }
  lidar_anchor_odom_.Advance(Sophus::SE2d());
  lidar_scan_history_.Push(static_cast<double>(*source_ns) * 1e-9, gtsam::Pose2());
}

std::optional<gtsam::Pose2> FusionGraphNode::DrPoseAtSource(uint64_t epoch, int64_t stamp_ns) const
{
  if (epoch != integration_epoch_)
    return std::nullopt;
  if (dr_source_baseline_ns_ == stamp_ns)
    return dr_source_baseline_pose_;
  const auto it = std::lower_bound(dr_source_history_.begin(),
                                   dr_source_history_.end(),
                                   stamp_ns,
                                   [](const auto& step, int64_t stamp)
                                   {
                                     return step.end_ns < stamp;
                                   });
  if (it == dr_source_history_.end() || stamp_ns < it->start_ns)
    return std::nullopt;
  const double dt = static_cast<double>(stamp_ns - it->start_ns) * 1e-9;
  return it->start_pose.compose(
      gtsam::Pose2::Expmap(gtsam::Vector3(it->vx * dt, 0.0, it->gz * dt)));
}

bool FusionGraphNode::RecordConsumedImu(const rclcpp::Time& stamp, double raw_gz, double dt)
{
  // Historical GNSS is solved inside Core; its output is always the latest
  // source state. Once anchored at S, only S and later DR support can be needed
  // for a subsequent latest-state revision or a newly committed endpoint.
  if (anchored_dr_source_ns_)
  {
    while (!dr_source_history_.empty() &&
           dr_source_history_.front().end_ns < *anchored_dr_source_ns_)
      dr_source_history_.pop_front();
    if (dr_source_baseline_ns_ && *dr_source_baseline_ns_ < *anchored_dr_source_ns_)
      dr_source_baseline_ns_.reset();
  }
  if (dr_source_history_.size() >= kDrSourceHistoryCapacity)
  {
    // Exhausted support cannot be replaced by a guessed interpolation.
    motion_source_fault_ = true;
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    integrated_source_ns_.reset();
    t_map_odom_anchor_valid_ = false;
    return false;
  }
  const double bias = graph_->AddGyroDelta(raw_gz, dt);
  const ConsumedMotionStep step{
      integration_epoch_,
      last_imu_stamp_->nanoseconds(),
      stamp.nanoseconds(),
      held_wheel_source_,
      {{MotionSourceKind::Gyro, ++gyro_source_sequence_, stamp.nanoseconds()},
       {last_imu_stamp_->nanoseconds(), stamp.nanoseconds()},
       raw_gz},
      bias,
      true,
      true};
  if (!graph_->AddConsumedMotionStep(step))
  {
    ResetMotionSourceEpoch();
    motion_source_fault_ = true;
    return false;
  }
  const double gz = raw_gz - bias;
  const bool slip =
      DrSlipVetoed(wheel_wz_, raw_gz, dr_slip_wheel_min_rad_per_s_, dr_slip_gyro_max_rad_per_s_);
  const double vx = slip ? 0.0 : wheel_vx_;
  {
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    dr_source_history_.push_back(
        {integration_epoch_, step.start_ns, step.end_ns, ContinuousDrPoseLocked(), vx, gz});
    const auto next = gtsam::Pose2(dr_x_, dr_y_, dr_yaw_)
                          .compose(gtsam::Pose2::Expmap(gtsam::Vector3(vx * dt, 0.0, gz * dt)));
    dr_x_ = next.x();
    dr_y_ = next.y();
    dr_yaw_ = next.theta();
    dr_last_gz_ = gz;
    dr_last_vx_eff_ = vx;
    integrated_source_ns_ = step.end_ns;
    motion_source_fault_ = false;
  }
  pose_extrap_.OnImuGyro(stamp.seconds(), gz);
  const auto odom =
      lidar_anchor_odom_.Advance(Sophus::SE2d(dr_yaw_, Eigen::Vector2d(dr_x_, dr_y_)));
  lidar_scan_history_.Push(stamp.seconds(),
                           gtsam::Pose2(odom.translation().x(),
                                        odom.translation().y(),
                                        odom.so2().log()));
  return true;
}

bool FusionGraphNode::InitializeAtSource(const gtsam::Pose2& pose,
                                         int64_t source_ns,
                                         std::optional<double> prior_override)
{
  if (!DrPoseAtSource(integration_epoch_, source_ns))
    return false;
  const bool initialized = graph_->InitializeSource(
      pose, this->now().nanoseconds(), source_ns, integration_epoch_, prior_override);
  if (initialized)
    t_map_odom_anchor_valid_ = false;
  return initialized;
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
    if (!integrated_source_ns_ || !InitializeAtSource(pose, *integrated_source_ns_))
    {
      dock_seeded_this_session_ = false;
      return;
    }
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
  {
    // tf_state_mu_: dr_* / anchor are read concurrently by TfBroadcastLoop.
    std::lock_guard<std::mutex> lock(tf_state_mu_);
    RecenterDrLocked(/*reset_heading=*/true);
    t_map_odom_anchor_valid_ = false;
  }
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

}  // namespace fusion_graph
