// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — OnTimer (per-tick build/publish) + LatLonToMap. (The node implementation is
// split across several translation units to keep each file within the project's 600-line budget;
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
#include "fusion_graph/loop_closure_gate.hpp"
#include "fusion_graph/yaw_gates.hpp"

namespace fusion_graph
{

void FusionGraphNode::OnTimer()
{
  const double now_s = this->now().seconds();

  // Boot dock-seed fallback. The normal init paths are (a) is_charging →
  // SeedFromDockPose, and (b) GPS xy + COG/mag yaw → TrySeedInitialPose.
  // Both can silently fail on a fresh boot on the dock: is_charging may
  // never be delivered (degraded DDS discovery), and a parked chassis
  // produces no COG heading, so seed_yaw_ stays empty. When that happens
  // the graph never initializes, no map→odom TF is published, and Nav2's
  // planner_server aborts activation (global_costmap can't get map→base),
  // taking the whole lifecycle bringup down with it. If we're still
  // uninitialized a few seconds after boot, fall back to the calibrated
  // dock pose — the robot normally boots parked on the dock. Suppressed
  // when hardware status has been seen AND reports NOT charging, so a
  // robot that boots away from the dock is left to GPS-based init.
  constexpr double kDockSeedFallbackS = 6.0;
  if (!graph_->IsInitialized())
  {
    if (boot_stamp_s_ < 0.0)
    {
      boot_stamp_s_ = now_s;
    }
    else if (!dock_seed_fallback_done_ && (now_s - boot_stamp_s_) > kDockSeedFallbackS &&
             (!last_is_charging_valid_ || last_is_charging_))
    {
      RCLCPP_WARN(get_logger(),
                  "fusion_graph: still uninitialized %.1fs after boot "
                  "(is_charging/COG unavailable) — seeding from dock pose",
                  now_s - boot_stamp_s_);
      SeedFromDockPose();
      dock_seed_fallback_done_ = true;
    }
  }

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

    // Match(source=curr, target=prev) so the returned delta == prev.between(curr)
    // — the FORWARD prev→curr motion, the same convention the wheel between-factor
    // and BetweenFactor(k_prev, k_curr) expect (graph_manager_node.cpp:346). With
    // body-frame scans, Match returns target.between(source); calling it
    // (prev, curr) would yield curr.between(prev) = the INVERSE, pushing the nodes
    // apart the wrong way and mismatching the forward-motion init_guess.
    auto res = scan_matcher_->Match(curr_scan, prev_node_scan_, init_guess);

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
      if (scan_yield_to_rtk_ && RtkFixedReceiptIsFresh(scan_yield_timeout_s_))
      {
        sm_sigma_xy = std::max(sm_sigma_xy, scan_yield_sigma_xy_);
        sm_sigma_theta = std::max(sm_sigma_theta, scan_yield_sigma_theta_);
      }
      // Level 2: floor the scan yaw σ so LiDAR yields yaw to the gyro (keeps
      // σ_xy tight → still carries POSITION through Float). See header.
      sm_sigma_theta = ScanYawSigma(sm_sigma_theta, scan_yaw_sigma_floor_rad_);
      graph_->QueueScanBetween(res.delta, sm_sigma_xy, sm_sigma_theta);
      ++scan_matches_ok_;
      // ICP-only odometry: cache the latest accepted scan-between delta (motion
      // since the previous node). It is composed into icp_pose_ exactly ONCE,
      // when Tick() creates the next node (below). Do NOT compose here: OnTimer
      // runs ~N ticks per node and res.delta is cumulative-since-node, so
      // per-tick composition over-integrates ~N× (the cause of the large
      // static-robot heading drift). Latest-wins mirrors the QueueScanBetween
      // the graph itself consumes once per node.
      last_scan_between_delta_ = res.delta;
      last_scan_between_valid_ = true;
    }
    else
    {
      ++scan_matches_fail_;
    }
  }

  // ── LiDAR map anchor (Beluga): absolute XY during RTK-Float ──────────
  // Builds the georeferenced occupancy grid under fresh RTK-Fixed and, once the
  // fix goes stale, localises against it and queues an XY-only prior. See
  // fusion_graph_node_lidar_anchor.cpp.
  if (use_lidar_map_anchor_)
  {
    LidarMapAnchorStep(curr_scan, curr_valid);
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

    // Advance the LiDAR-only odometry exactly once per node, by the same
    // scan-between delta the graph just consumed. Seeds from the new node's
    // fused pose on the first node so it starts aligned, then drifts (relative
    // scan-matching, no absolute reference) — diagnostics only.
    if (last_scan_between_valid_)
    {
      if (!icp_pose_seeded_)
      {
        icp_pose_ = out->pose;
        icp_pose_seeded_ = true;
      }
      else
      {
        icp_pose_ = icp_pose_.compose(last_scan_between_delta_);
      }
      PublishIcpOdom();
      last_scan_between_valid_ = false;
    }

    // Loop closure search — gated on loop_closure_enabled_ and on
    // having a scan matcher (we reuse it). Find candidates within
    // lc_max_dist_m_ that are at least lc_min_age_s_ old, run ICP for
    // each, accept those with rmse < lc_max_rmse_.
    //
    // SKIP entirely while an RTK-Fixed sample is fresh: GPS is already an
    // absolute mm-accurate constraint there, so each LC adds an iSAM2 factor for
    // ~no information — an unbounded leak over a long stationary dwell that
    // OOM-killed the node 2026-06-09. Re-enables once the fix is stale past
    // scan_yield_timeout_s so LC still carries the no-fix (tree-cover) windows.
    const bool rtk_fixed_fresh = RtkFixedReceiptIsFresh(scan_yield_timeout_s_);

    // Rate/travel gate (issue #513): with LC enabled exactly when GPS is Float
    // or absent, the unbounded search accepted 13.7 LC/s on featureless grass
    // (3816 in 286 s of mowing), each a 5 cm factor snapping to the adjacent
    // swath. Skip the whole candidate search (no ICP paid) until BOTH
    // lc_min_interval_s AND lc_min_travel_m have accrued since the last
    // ACCEPTED LC; the accumulators are reset on accept only — see
    // loop_closure_gate.hpp for why that polarity is right here.
    const bool lc_search_enabled = loop_closure_enabled_ && scan_matcher_ && curr_valid &&
                                   !(lc_skip_when_rtk_fixed_ && rtk_fixed_fresh);

    const double time_since_lc_s = last_lc_accept_stamp_
                                       ? (this->now() - *last_lc_accept_stamp_).seconds()
                                       : std::numeric_limits<double>::infinity();

    const bool lc_gate_open =
        lc_search_enabled && LoopClosureRateAllows(wheel_dist_since_last_lc_m_,
                                                   time_since_lc_s,
                                                   lc_min_travel_m_,
                                                   lc_min_interval_s_);

    if (lc_search_enabled && !lc_gate_open)
      ++lc_rate_gated_;

    if (lc_gate_open)
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
        // i.e. cand.between(curr). Match(source=curr, target=cand) returns
        // cand.between(curr) — the measurement BetweenFactor(k_cand, k_curr)
        // expects (graph_manager_rebase.cpp:377). Calling it (cand, curr) would
        // return curr.between(cand) = the INVERSE, dragging the trajectory the
        // wrong way each time an LC fires.
        const gtsam::Pose2 init = cand_pose->between(out->pose);
        auto res = scan_matcher_->Match(curr_scan, cand_scan, init);
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

        // Yield to RTK — same policy as the scan-between factors above. When
        // an RTK-Fixed sample was seen within scan_yield_timeout_s, inflate the
        // loop-closure σ so this (subtly biased on open lawn) LiDAR constraint
        // can't hold map→odom off the GPS-pinned solution. Without it the
        // accumulated loop closures form a rigid LiDAR sub-graph that the
        // sparse but mm-accurate RTK-Fixed GPS factors can't fully re-base —
        // measured as a stable ~9 cm fused↔/gps/pose_cov offset at the dock
        // under RTK-Fixed (2026-06-08). The scan-yield already covered the
        // scan-between factors; loop closures were the remaining un-yielded
        // LiDAR constraint. Once the fix is stale past the timeout the tight LC
        // σ returns, so loop closures still carry global consistency through
        // no-fix (tree-cover) windows.
        double lc_sigma_xy = lc_sigma_xy_;
        double lc_sigma_theta = lc_sigma_theta_;
        if (scan_yield_to_rtk_ && RtkFixedReceiptIsFresh(scan_yield_timeout_s_))
        {
          lc_sigma_xy = std::max(lc_sigma_xy, scan_yield_sigma_xy_);
          lc_sigma_theta = std::max(lc_sigma_theta, scan_yield_sigma_theta_);
        }
        // Issue #513: the RTK-yield above only fires under a fresh Fixed sample,
        // i.e. never in the Float window where LC actually runs. Floor the LC
        // σ to the last accepted GNSS σ so an ICP match on grass can never
        // out-vote a decimetre-σ Float fix; -1 (no GPS ever) leaves it as is.
        lc_sigma_xy = LoopClosureSigmaFloor(lc_sigma_xy, last_gps_sigma_, lc_gps_sigma_ratio_);
        // Level 2: floor the loop-closure yaw σ too (yield yaw to gyro).
        lc_sigma_theta = ScanYawSigma(lc_sigma_theta, scan_yaw_sigma_floor_rad_);
        graph_->AddLoopClosure(cand_idx, out->node_index, res.delta, lc_sigma_xy, lc_sigma_theta);
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: loop closure %lu -> %lu accepted "
                    "(rmse=%.3f, dist=%.2f m, sigma_xy=%.3f)",
                    static_cast<unsigned long>(cand_idx),
                    static_cast<unsigned long>(out->node_index),
                    res.rmse,
                    std::hypot(out->pose.x() - cand_pose->x(), out->pose.y() - cand_pose->y()),
                    lc_sigma_xy);
        // One accepted LC per node, max — and re-arm the gate from THIS accept.
        wheel_dist_since_last_lc_m_ = 0.0;
        last_lc_accept_stamp_ = this->now();
        break;
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
      // tf_state_mu_: the anchor VALUE and valid flag are read together by
      // TfBroadcastLoop; write the {value, valid=true} pair atomically so the
      // loop never composes a stale anchor with fresh dr_*. dr_* is read here
      // too (it is concurrently integrated by OnImu).
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      const gtsam::Pose2 dr_at_node(dr_x_, dr_y_, dr_yaw_);
      t_map_odom_anchor_ = snap->pose.compose(dr_at_node.inverse());
      last_anchored_node_index_ = snap->node_index;
      t_map_odom_anchor_valid_ = true;
    }
    // Odom re-base: once the robot has driven odom_rebase_dist_m from the odom
    // origin, reset the odom POSITION onto the robot (heading kept) and shift
    // the anchor so map→base is unchanged. This keeps the lever arm |dr| small
    // so graph-yaw jitter can't rotate it into large map→odom position steps.
    // OnImu (same executor thread) can't interrupt; the lock guards the TF
    // reader thread. force_pub_resync_ makes the slew snap the published anchor
    // to the new (coordinated) target so map→base stays continuous.
    if (odom_rebase_dist_m_ > 0.0 && t_map_odom_anchor_valid_)
    {
      std::lock_guard<std::mutex> lock(tf_state_mu_);
      if (std::hypot(dr_x_, dr_y_) > odom_rebase_dist_m_)
      {
        const gtsam::Pose2 map_base =
            t_map_odom_anchor_.compose(gtsam::Pose2(dr_x_, dr_y_, dr_yaw_));
        dr_x_ = 0.0;
        dr_y_ = 0.0;
        t_map_odom_anchor_ = map_base.compose(gtsam::Pose2(0.0, 0.0, dr_yaw_).inverse());
        force_pub_resync_.store(true, std::memory_order_release);
      }
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

}  // namespace fusion_graph
