// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — input callbacks: wheel/imu/gnss. (The node implementation is split across
// several translation units to keep each file within the project's 600-line budget; all share
// fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

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
      // Slip veto (see header): if the wheels claim a yaw rate the
      // gyro doesn't see, the chassis is being skated, not driven —
      // its forward velocity is phantom. Drop the translation for
      // this sample; yaw still integrates from the gyro, which is the
      // honest source during a slipping pivot. Without this the odom
      // frame accumulates the fictitious forward motion unbounded.
      const bool dr_slip = std::abs(wheel_wz_ - gz) > dr_slip_wheel_min_rad_per_s_ &&
                           std::abs(gz) < dr_slip_gyro_max_rad_per_s_ &&
                           std::abs(wheel_wz_) > dr_slip_wheel_min_rad_per_s_;
      const double vx_eff = dr_slip ? 0.0 : wheel_vx_;
      {
        // tf_state_mu_: dr_* is read concurrently by TfBroadcastLoop.
        std::lock_guard<std::mutex> lock(tf_state_mu_);
        dr_yaw_ += gz * dt;
        dr_x_ += vx_eff * std::cos(dr_yaw_) * dt;
        dr_y_ += vx_eff * std::sin(dr_yaw_) * dt;
        // Cache the velocities that produced this step so the TF broadcast can
        // honestly forward-propagate the pose by tf_publish_lead_s_.
        dr_last_gz_ = gz;
        dr_last_vx_eff_ = vx_eff;
      }
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
    const double expected_pivot_jump_m = lever_arm_radius_m_ * abs_dtheta_since_last_gps_rad_;
    const double jump_budget = rtk_wrongfix_max_jump_m_ + expected_pivot_jump_m;
    if (jump > jump_budget && wheel_dist_since_last_gps_m_ < rtk_wrongfix_max_wheel_m_)
    {
      graph_->RecordGpsRejectWrongFix();
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "fusion_graph: RTK wrong-fix? jump=%.3f m, wheel=%.3f m, "
                           "sweep_budget=%.3f m — sample dropped",
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
  // Latch the most-recent valid GPS σ for the keyframe-capture quality gate
  // (only capture when the fix is mm-accurate). <0 means no usable σ.
  last_gps_sigma_ = sigma;

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
  // Debounce counter for keyframe capture: only capture after RTK-Fixed has
  // held for several consecutive epochs (a single carrSoln Fixed flicker can
  // otherwise freeze a slightly-off anchor that poisons every later match).
  rtk_fixed_streak_ = rtk_fixed ? (rtk_fixed_streak_ + 1) : 0;
  // During the dock approach, hold position through the RTK fixed↔float
  // per-epoch flicker: drop non-Fixed epochs entirely so the dock controller's
  // target doesn't jump between cm-accurate Fixed and dm-noisy Float (the
  // flicker, not the controller, was the 2026-06-10 divergence trigger). Fixed
  // epochs update the graph normally below; the is_charging suppression takes
  // over once docked.
  // ...but never starve a yet-uninitialized graph of its bootstrap GPS seed.
  if (gate_float_gps_during_docking_ && !rtk_fixed && DockingApproachActive() &&
      graph_->IsInitialized())
  {
    return;
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
    // On the dock the dock_pose is authoritative ground truth — the dock does
    // not move. The previous approach injected a WEAK live-GPS factor here for
    // well-posedness, but a ~7 Hz stream of σ≈0.5 m factors at the live RTK
    // position (5-30 cm off dock_pose, and drifting yaw with nothing holding it)
    // out-votes the single one-shot dock prior and WALKS the docked pose off the
    // anchor over the charge dwell (field 2026-06-10: 11.5 cm + 53° walk, which
    // then made re-docking aim at the wrong target — the "never dock twice"
    // symptom). Instead, periodically re-assert a firm absolute anchor at the
    // FULL dock_pose (x, y, AND yaw). With no live GPS to drag it, the
    // accumulated dock priors keep iSAM2 well-posed AND pin the docked pose
    // exactly where the operator calibrated it, holding both position and yaw.
    // seed_xy_ is still NOT updated (TrySeedInitialPose bootstraps from dock_pose).
    if (graph_->IsInitialized())
    {
      if (auto snap = graph_->LatestSnapshot())
      {
        // Re-anchor each NEW node exactly once (nodes appear ~1/5 s while
        // stationary), so the prior count tracks the node count and stays
        // bounded by the sliding window rather than piling several priors onto
        // the same stationary node.
        if (snap->node_index != last_dock_reanchor_node_)
        {
          const gtsam::Pose2 dock(dock_pose_x_, dock_pose_y_, dock_pose_yaw_);
          graph_->ForceAnchor(snap->node_index,
                              dock,
                              dock_reanchor_sigma_xy_m_,
                              std::max(dock_pose_yaw_sigma_rad_, 0.035));
          last_dock_reanchor_node_ = snap->node_index;
        }
      }
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
  if (rtk_fixed && autoload_succeeded_ && !rtk_autoload_override_done_ && graph_->IsInitialized() &&
      last_is_charging_valid_ && !last_is_charging_)
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

}  // namespace fusion_graph
