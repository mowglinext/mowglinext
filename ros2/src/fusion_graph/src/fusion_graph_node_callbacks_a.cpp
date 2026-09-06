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

#include "fusion_graph/dock_gps_consistency.hpp"
#include "fusion_graph/dr_slip_veto.hpp"
#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"
#include "fusion_graph/rtk_wrongfix_gate.hpp"

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
      // Same scalar for the loop-closure travel gate (issue #513); reset only
      // when a loop closure is ACCEPTED in OnTimer.
      wheel_dist_since_last_lc_m_ += speed * dt;
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
      const bool dr_slip =
          DrSlipVetoed(wheel_wz_, gz, dr_slip_wheel_min_rad_per_s_, dr_slip_gyro_max_rad_per_s_);
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

  // The pinned Universal GNSS adapter stamps NavSatFix with the local receiver
  // acceptance/receipt time and preserves that same stamp for timer-driven
  // republications. It is therefore the strongest identity available on
  // /gps/fix itself: compare provenance, never coordinates or callback time.
  //
  // sequence=0 deliberately selects the tracker's receipt-only compatibility
  // mode. The stronger position_observation_sequence now crosses /gps/status
  // for consumers that subscribe to the typed contract, but associating two
  // independently delivered topics here would reintroduce the asynchronous
  // pairing defect deferred to MGNSS-002.
  const rclcpp::Time ros_now = this->now();
  const rclcpp::Time receipt_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
  const auto observation_update = gnss_observation_tracker_.Observe(0,
                                                                    receipt_stamp.nanoseconds(),
                                                                    ros_now.nanoseconds(),
                                                                    steady_now_ns);
  using mowgli_interfaces::gnss_observation_freshness::ObservationUpdate;
  if (observation_update == ObservationUpdate::kRosTimeDiscontinuity)
  {
    last_rtk_fixed_stamp_.reset();
    last_gps_sigma_ = -1.0;
    last_gps_map_xy_.reset();
    ResetRtkWrongFixAccumulators(wheel_dist_since_last_gps_m_, abs_dtheta_since_last_gps_rad_);
    RCLCPP_WARN(get_logger(), "fusion_graph: ROS time moved backward; GNSS evidence epoch reset");
    return;
  }
  if (observation_update != ObservationUpdate::kNewObservation &&
      observation_update != ObservationUpdate::kSourceRestart)
  {
    if (observation_update == ObservationUpdate::kInvalidProvenance)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "fusion_graph: GNSS receipt stamp is zero/future; sample dropped");
    }
    return;
  }

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
                "(%.9f, %.9f) — set datum_lat/lon explicitly",
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
    // Motion-consistent gate: the GPS step must be explainable by how far the
    // chassis ACTUALLY travelled since the last fix (wheel arc + lever-arm
    // sweep from in-place rotation) plus a fixed slack budget. The motion-
    // consistent gate compares the jump against actual wheel travel at any
    // speed and reduces to the old fixed budget when stationary (wheel_dist≈0).
    // rtk_wrongfix_max_jump_m is the slack on top of travel — size it to a
    // few × the raw GNSS jitter σ. See rtk_wrongfix_gate.hpp for the pure
    // decision function + unit tests (test_rtk_wrongfix_gate.cpp).
    if (GpsJumpImplausible(jump,
                           rtk_wrongfix_max_jump_m_,
                           lever_arm_radius_m_,
                           abs_dtheta_since_last_gps_rad_,
                           wheel_dist_since_last_gps_m_))
    {
      graph_->RecordGpsRejectWrongFix();
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           2000,
                           "fusion_graph: RTK wrong-fix? jump=%.3f m, wheel=%.3f m, "
                           "sweep_budget=%.3f m — sample dropped",
                           jump,
                           wheel_dist_since_last_gps_m_,
                           lever_arm_radius_m_ * abs_dtheta_since_last_gps_rad_);
      // Reset accumulators + cache so a repeated wrong-fix doesn't
      // permanently lock us out — once two consecutive samples agree,
      // last_gps_map_xy_ updates and we resume normal flow. See
      // rtk_wrongfix_gate.hpp's header comment: this reset MUST happen on
      // reject as well as accept, or the gate becomes the reverted
      // GnssMobileGate's reject-forever failure mode.
      last_gps_map_xy_ = gtsam::Vector2(mx, my);
      ResetRtkWrongFixAccumulators(wheel_dist_since_last_gps_m_, abs_dtheta_since_last_gps_rad_);
      return;
    }
  }
  last_gps_map_xy_ = gtsam::Vector2(mx, my);
  ResetRtkWrongFixAccumulators(wheel_dist_since_last_gps_m_, abs_dtheta_since_last_gps_rad_);

  // covariance[0] is variance of east; take sqrt for sigma. Use the
  // diagonal mean for a single sigma_xy (factor model is isotropic).
  const double var_x = msg->position_covariance[0];
  const double var_y = msg->position_covariance[4];
  double sigma = std::sqrt(0.5 * (var_x + var_y));
  // SAFETY: a fix with UNKNOWN covariance, or a zero/non-finite reported σ, has
  // NO trustworthy accuracy. The old code set σ=-1.0 here as a "floor" sentinel,
  // but graph_manager then clamps σ UP to gps_sigma_floor (3 mm) — so a fix with
  // no known accuracy (e.g. a standalone/SBAS position when the driver leaves
  // covariance UNKNOWN) was fused as a GnssLeverArmFactor at RTK-Fixed precision,
  // yanking map→odom by metres toward a garbage position (the next good fix then
  // looks like a jump and the wrong-fix gate drops it, locking GPS out). Reject
  // such fixes outright — wheel/gyro/COG keep localising — instead of trusting
  // them. (navsat_to_absolute_pose_node guards covariance_type the same way, but
  // it no longer feeds the localizer.)
  if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN ||
      !std::isfinite(sigma) || sigma <= 0.0)
  {
    graph_->RecordGpsRejectWrongFix();
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: GPS fix with unknown/zero covariance "
                         "(cov_type=%u, var_x=%.4g, var_y=%.4g) — rejected (would "
                         "otherwise be trusted as 3 mm RTK precision)",
                         msg->position_covariance_type,
                         var_x,
                         var_y);
    last_gps_sigma_ = -1.0;  // no usable σ this epoch (LC σ floor has nothing to use)
    return;
  }
  if (gps_sigma_speed_coeff_ > 0.0)
  {
    // Inflate the GPS σ with chassis speed. The receiver covariance reports
    // the instantaneous fix precision but ignores motion-induced position
    // error: GPS measurement latency × velocity, plus lever-arm sweep while
    // moving, both displace the reported antenna position from where the robot
    // actually is at fuse time. σ_eff = sqrt(σ_msg² + (coeff·v)²); coeff has
    // units of seconds (≈ effective GPS latency). 0 disables (raw receiver σ).
    const double v = std::abs(wheel_vx_);
    const double speed_term = gps_sigma_speed_coeff_ * v;
    sigma = std::sqrt(sigma * sigma + speed_term * speed_term);
  }
  // SAFETY: max-σ reject. A fix this imprecise is a garbage / standalone
  // position; fusing it (even at its honest large σ) is not worth the risk of a
  // wrong-fix step. Disabled at 0 (default) so genuine RTK-Float — which the
  // multi-minute ride-through depends on — is never rejected; operators size it
  // generously above their worst Float σ when they want the extra guard.
  if (gps_max_sigma_reject_m_ > 0.0 && sigma > gps_max_sigma_reject_m_)
  {
    graph_->RecordGpsRejectWrongFix();
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: GPS σ=%.3f m > reject threshold %.3f m — "
                         "sample dropped",
                         sigma,
                         gps_max_sigma_reject_m_);
    last_gps_sigma_ = -1.0;
    return;
  }
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

  // Resolve the receipt provenance against the live graph window BEFORE any
  // current-RTK latch is committed below. This is the same lookup the factor
  // path performs further down (measurement_stamp is this same stamp); it is
  // hoisted here because the docking and on-dock branches return early, and an
  // observation too old to still have a live graph node must not refresh their
  // freshness latches either. Stateless per sample — it cannot latch GPS out.
  if (graph_->IsInitialized() && !graph_->FindNodeAtOrBefore(receipt_stamp.seconds()))
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "fusion_graph: no live graph node at/before GNSS receipt time; sample dropped");
    return;
  }

  const auto commit_current_gnss_evidence = [this, rtk_fixed, receipt_stamp, sigma]()
  {
    // Receipt time, not callback time, is the freshness basis. Downstream gates
    // additionally reject negative age via IsReceiptFresh.
    if (rtk_fixed)
    {
      last_rtk_fixed_stamp_ = receipt_stamp;
    }
    last_gps_sigma_ = sigma;
  };
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
    commit_current_gnss_evidence();
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
    commit_current_gnss_evidence();
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
    //
    // Issue #512 — the prior is NOT unconditional. This sample has already
    // passed the wrong-fix / unknown-covariance / max-σ gates above, so it is
    // the most recent ACCEPTED fix; compare it, antenna-to-antenna, with where
    // dock_pose says the antenna is. A fresh RTK-Fixed sample with a small σ
    // that still lands metres away (2026-09-02: 1.88-2.45 m at "Fixed 14-20 mm"
    // for ~45 min after a receiver power cycle) is a confident contradiction
    // the 3 cm prior must not silently override: skip the prior for this node
    // and let the sample through to QueueGnss below instead. No fix / Float /
    // large σ — the terrace dock — keeps pinning exactly as before; see
    // dock_gps_consistency.hpp.
    const auto dock_antenna = DockAntennaMapXY(
        dock_pose_x_, dock_pose_y_, dock_pose_yaw_, lever_arm_x_m_, lever_arm_y_m_);
    dock_gps_disagreement_m_ = DockGpsDisagreementM(dock_antenna.x, dock_antenna.y, mx, my);
    const bool dock_prior_yields = DockPriorShouldYield(rtk_fixed,
                                                        sigma,
                                                        dock_gps_disagreement_m_,
                                                        dock_prior_max_gps_disagreement_m_,
                                                        dock_prior_max_gps_sigma_m_);
    if (graph_->IsInitialized())
    {
      if (auto snap = graph_->LatestSnapshot())
      {
        // Re-anchor each NEW node exactly once (nodes appear ~1/5 s while
        // stationary), so the prior count tracks the node count and stays
        // bounded by the sliding window rather than piling several priors onto
        // the same stationary node. A yielded node is marked the same way so
        // it is counted once and never gets the prior later in its lifetime.
        if (snap->node_index != last_dock_reanchor_node_)
        {
          if (dock_prior_yields)
          {
            ++dock_prior_yielded_;
            RCLCPP_ERROR_THROTTLE(get_logger(),
                                  *get_clock(),
                                  10000,
                                  "fusion_graph: dock prior vs RTK-Fixed GPS disagree by %.2f m "
                                  "(σ=%.3f m, threshold %.2f m) — skipping dock prior, GPS wins; "
                                  "check dock_pose / receiver",
                                  dock_gps_disagreement_m_,
                                  sigma,
                                  dock_prior_max_gps_disagreement_m_);
          }
          else
          {
            const gtsam::Pose2 dock(dock_pose_x_, dock_pose_y_, dock_pose_yaw_);
            graph_->ForceAnchor(snap->node_index,
                                dock,
                                dock_reanchor_sigma_xy_m_,
                                std::max(dock_pose_yaw_sigma_rad_, 0.035));
          }
          last_dock_reanchor_node_ = snap->node_index;
        }
      }
    }
    if (!dock_prior_yields || !graph_->IsInitialized())
    {
      TrySeedInitialPose();
      return;
    }
    // Yielding: fall through and fuse this sample like an off-dock fix.
  }
  else
  {
    dock_gps_disagreement_m_ = 0.0;
  }
  const bool docked_gps_override = last_is_charging_valid_ && last_is_charging_;
  // GNSS bridges preserve the receiver measurement epoch in header.stamp,
  // which may precede callback delivery by hundreds of milliseconds or more.
  // Resolve that epoch only after the quality/docking gates above so the
  // factor constrains the state that was actually observed instead of the
  // state at callback-delivery time.
  const rclcpp::Time measurement_stamp(msg->header.stamp, get_clock()->get_clock_type());
  std::optional<uint64_t> measurement_node;
  if (graph_->IsInitialized() && measurement_stamp.nanoseconds() != 0)
  {
    constexpr double kFutureStampToleranceS = 0.5;
    const double future_offset_s = (measurement_stamp - this->now()).seconds();
    if (future_offset_s > kFutureStampToleranceS)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "fusion_graph: GNSS stamp is %.3f s in the future; sample dropped",
                           future_offset_s);
      return;
    }
    measurement_node = graph_->FindNodeAtOrBefore(measurement_stamp.seconds());
    if (!measurement_node)
    {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "fusion_graph: no live graph node at/before GNSS epoch; sample dropped");
      return;
    }
  }

  if (!graph_->QueueGnss(mx, my, sigma, /*robust=*/true, measurement_node))
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: GNSS epoch node left the live graph; sample dropped");
    return;
  }
  commit_current_gnss_evidence();
  // Latch whether the most recent seed came from RTK-Fixed so the next
  // graph initialization can use a tight prior matching that quality.
  // Stale once seeded but TrySeedInitialPose only fires once per
  // (re)initialization, so the freshness window is the same as the
  // seed itself. NOT latched on the docked #512 override path: on the dock
  // a (re)initialization must still bootstrap from dock_pose, not GPS.
  if (!docked_gps_override)
  {
    seed_xy_ = gtsam::Vector2(mx, my);
    seed_xy_rtk_fixed_ = rtk_fixed;
  }

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
