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

namespace fusion_graph
{

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

  // ── Scan-to-keyframe ABSOLUTE constraint (the RTK-Float carry) ───────
  // Match the live scan to nearby frozen RTK-anchored keyframes and queue a
  // PoseTranslationPrior that pins absolute xy — this is what holds <2 cm
  // through a Float window where dead-reckoning would otherwise drift. ENGAGE
  // only when RTK-Fixed is NOT recent (during Float / no-fix): under Fixed the
  // GnssLeverArmFactor owns absolute position and double-counting would
  // over-constrain. Reuses scan_matcher_ + the same ICP guard rails as
  // scan-between. abs_meas = kf.abs_pose.compose(delta.inverse()) — composition
  // direction locked by test_factors.cpp::ScanToKeyframeComposition.
  if (use_keyframe_map_ && scan_matcher_ && curr_valid)
  {
    const bool rtk_recent = last_rtk_fixed_stamp_ &&
                            (this->now() - *last_rtk_fixed_stamp_).seconds() < kf_engage_age_s_;
    if (!rtk_recent)
    {
      if (auto cur = graph_->LatestSnapshot())
      {
        double dx, dy, dtg, dtw;
        graph_->PeekAccumulator(dx, dy, dtg, dtw);
        const double dth = (std::abs(dtg) > 1e-9) ? dtg : dtw;
        const gtsam::Pose2 pred = cur->pose.compose(gtsam::Pose2(dx, dy, dth));
        const auto cand = graph_->FindKeyframesNearXY(pred.x(),
                                                      pred.y(),
                                                      kf_match_max_dist_m_,
                                                      kf_max_candidates_);
        double best_rmse = 1e9;
        gtsam::Vector2 best_xy;
        double best_sigma = 0.0;
        bool have_best = false;
        for (uint64_t kid : cand)
        {
          auto kf = graph_->GetKeyframe(kid);
          if (!kf || kf->scan_body.empty())
            continue;
          const gtsam::Pose2 init = kf->abs_pose.between(pred);
          const auto res = scan_matcher_->Match(kf->scan_body, curr_scan, init);
          if (!res.ok)
          {
            graph_->RecordIcpRejectInliers();
            continue;
          }
          if (res.rmse > icp_max_rmse_m_)
          {
            graph_->RecordIcpRejectRmse();
            continue;
          }
          if (std::abs(res.delta.x()) > icp_max_delta_xy_m_ ||
              std::abs(res.delta.y()) > icp_max_delta_xy_m_ ||
              std::abs(res.delta.theta()) > icp_max_delta_theta_rad_)
          {
            graph_->RecordIcpRejectSanity();
            continue;
          }
          const gtsam::Pose2 dev = init.between(res.delta);
          if (std::hypot(dev.x(), dev.y()) > icp_max_divergence_xy_m_ ||
              std::abs(dev.theta()) > icp_max_divergence_theta_rad_)
          {
            graph_->RecordIcpRejectDivergence();
            continue;
          }
          const gtsam::Pose2 abs_meas = kf->abs_pose.compose(res.delta.inverse());
          // Mirror-guard: a swapped/mirror match lands far from the
          // wheel-predicted pose (Huber can't reject a low-rmse mirror).
          if (std::hypot(abs_meas.x() - pred.x(), abs_meas.y() - pred.y()) >
              icp_max_divergence_xy_m_)
            continue;
          if (res.rmse < best_rmse)
          {
            best_rmse = res.rmse;
            best_xy = gtsam::Vector2(abs_meas.x(), abs_meas.y());
            best_sigma = std::max(res.sigma_xy, kf_apply_sigma_floor_m_);
            have_best = true;
          }
        }
        if (have_best)
        {
          graph_->QueueScanToKeyframe(best_xy, best_sigma, /*robust=*/true);
          ++kf_matches_ok_;
        }
        else if (!cand.empty())
        {
          ++kf_matches_fail_;
        }
      }
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

    // ── Keyframe capture (build the absolute map under stable RTK-Fixed) ──
    // Freeze the GPS-fused node pose (out->pose — NEVER the raw antenna ENU,
    // which is lever-offset) + scan as a keyframe when: the fix is mm-accurate
    // and stable (debounced ≥N epochs — a single carrSoln flicker would freeze a
    // wrong anchor), the robot has moved ≥ kf_spacing_m, and it isn't pivoting
    // (a smeared scan makes a bad map tile). The streak gate resets to 0 the
    // instant a non-Fixed epoch arrives, so capture (Fixed) and the apply block
    // above (Float) never both fire.
    if (use_keyframe_map_ && curr_valid && curr_scan.size() >= 10)
    {
      const bool rtk_fresh =
          last_rtk_fixed_stamp_ && (this->now() - *last_rtk_fixed_stamp_).seconds() < 1.0;
      const bool moved = !last_kf_capture_xy_ ||
                         std::hypot(out->pose.x() - last_kf_capture_xy_->x(),
                                    out->pose.y() - last_kf_capture_xy_->y()) >= kf_spacing_m_;
      if (rtk_fresh && rtk_fixed_streak_ >= kf_capture_rtk_debounce_ && last_gps_sigma_ >= 0.0 &&
          last_gps_sigma_ <= kf_capture_sigma_max_m_ &&
          std::abs(wheel_wz_) < kf_capture_max_omega_ && moved)
      {
        if (graph_->AddKeyframe(out->pose, curr_scan))
          last_kf_capture_xy_ = gtsam::Vector2(out->pose.x(), out->pose.y());
      }
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
    const bool rtk_fixed_fresh =
        last_rtk_fixed_stamp_ &&
        (this->now() - *last_rtk_fixed_stamp_).seconds() < scan_yield_timeout_s_;
    if (loop_closure_enabled_ && scan_matcher_ && curr_valid &&
        !(lc_skip_when_rtk_fixed_ && rtk_fixed_fresh))
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
        if (scan_yield_to_rtk_ && last_rtk_fixed_stamp_ &&
            (this->now() - *last_rtk_fixed_stamp_).seconds() < scan_yield_timeout_s_)
        {
          lc_sigma_xy = std::max(lc_sigma_xy, scan_yield_sigma_xy_);
          lc_sigma_theta = std::max(lc_sigma_theta, scan_yield_sigma_theta_);
        }
        graph_->AddLoopClosure(cand_idx, out->node_index, res.delta, lc_sigma_xy, lc_sigma_theta);
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
