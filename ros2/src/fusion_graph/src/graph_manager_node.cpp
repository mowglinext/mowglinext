// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — node creation: Tick + CreateNodeLocked. (The class implementation
// is split across several translation units to keep each file within the project's 600-line budget;
// all share graph_manager.hpp + the inline PoseKey().)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
std::optional<TickOutput> GraphManager::Tick(double now_s)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_)
    return std::nullopt;
  // Self-heal a last_node_time_s_ parked in the FUTURE relative to the tick
  // clock. Load() clamps the restored timestamp to system_clock (wall time),
  // but Tick() is driven by the node clock — under use_sim_time those differ
  // by ~1.79e9 s, so the gate below would never fire and node creation would
  // freeze for the whole sim. Snapping down to now_s resumes the cadence and
  // is clock-source-agnostic (no-op on real hardware where they agree). The
  // timestamp index belongs to the old clock epoch, so rebuild it from the
  // latest live node rather than allowing new sensor stamps to match stale
  // entries after a bag loop / simulation reset.
  if (last_node_time_s_ > now_s)
  {
    last_node_time_s_ = now_s;
    node_time_index_.clear();
    if (latest_ && HasPoseAt(latest_->node_index))
      node_time_index_.emplace_back(now_s, latest_->node_index);
  }
  if (now_s - last_node_time_s_ < params_.node_period_s)
    return std::nullopt;

  // Stationary throttle: when the wheel + gyro accumulators show no
  // meaningful motion since the last node, drop the node period to
  // 1 / stationary_node_period_s. Stops the graph from inflating by
  // ~10 nodes/s while parked at the dock — both for memory bound
  // and to keep iSAM2 / LC search bounded.
  const double motion_xy_sq = accum_.dx * accum_.dx + accum_.dy * accum_.dy;
  const double abs_dtheta =
      std::abs(std::abs(accum_.dtheta_gyro) > 1e-9 ? accum_.dtheta_gyro : accum_.dtheta_wheel);
  const bool stationary =
      motion_xy_sq < params_.stationary_motion_thresh_m * params_.stationary_motion_thresh_m &&
      abs_dtheta < params_.stationary_motion_thresh_theta;
  if (stationary && now_s - last_node_time_s_ < params_.stationary_node_period_s)
  {
    return std::nullopt;
  }

  return CreateNodeLocked(now_s);
}

std::optional<TickOutput> GraphManager::CreateNodeLocked(double now_s)
{
  // Guard against next_index_ == 0: would underflow PoseKey(next_index_ - 1)
  // and crash GTSAM with "Symbol index is too large" when j wraps to 2^64-1.
  // Reachable historically when Load() restored an empty persisted graph
  // (next_index=0) and marked the manager initialized; both Save() and Load()
  // now refuse the empty case, but keep this defensive — the cost of the
  // check is one compare and the upside is no abort if a future code path
  // reintroduces the same hole.
  if (next_index_ == 0)
  {
    return latest_.value_or(TickOutput{});
  }

  // Cadence scaling for the per-tick dead-reckoning gates below. Each gate
  // compares the PER-TICK accumulated wheel/gyro delta against a fixed radian/
  // metre threshold, so its effective rad/s (or m/s) trip point is
  // threshold / node_period_s and would drift with cadence. Multiplying the
  // thresholds by this factor keeps the tuned trip points invariant across the
  // 25 Hz (launch default), 50 Hz (yaml default) and 10 Hz configurations.
  // At the 25 Hz reference the factor is exactly 1.0 (no behaviour change on
  // the deployed robot). See kTunedNodePeriodS in graph_params.hpp.
  const double tick_scale = params_.node_period_s / kTunedNodePeriodS;

  // 1. Build the wheel between-factor: relative pose from X_{k-1} to X_k.
  //    Yaw selection rules:
  //    a. Wheel encoder is ground truth when it reads zero. Encoders
  //       cannot slip "into stationary" — when the per-tick wheel
  //       accumulator shows no motion (|dx|, |dy|, |dtheta_wheel| all
  //       under their thresholds), the robot really isn't moving under
  //       power. Trust the wheel regardless of residual gyro
  //       bias / noise and snap dtheta to 0 with a tight sigma. The
  //       previous version of this block also gated on |dtheta_gyro| <
  //       stationary_thresh_theta, which on this robot's live IMU
  //       (residual wz ≈ -0.023 rad/s ≈ -1.32°/s after hardware_bridge
  //       calibration, dominated by thermal drift) was always false —
  //       the AND fell through to the gyro path and yaw drifted
  //       -4.28°/min vs the +0.43°/min pre-suppressor baseline.
  //
  //       Edge case: a hand-pushed robot has wheels off the ground but
  //       is physically rotating. We accept the trade-off — a manually
  //       repositioned robot will lose its yaw estimate, but it is far
  //       more common to be parked with a noisy gyro than to be hand
  //       spun, and the next session's GPS-COG fusion + dock_yaw seed
  //       re-anchor yaw when the robot starts moving again.
  //    b. Otherwise, prefer gyro: at speed the differential-drive yaw
  //       estimate is dominated by encoder slip and the gyro is strictly
  //       better. The wheel sigma_theta path only fires when no gyro
  //       sample arrived this tick (pre-cog seed window, IMU restart).
  const double stationary_thresh_xy_m = params_.stationary_thresh_xy_m * tick_scale;
  const double stationary_thresh_theta = params_.stationary_thresh_theta * tick_scale;
  const bool wheel_stationary = std::abs(accum_.dx) < stationary_thresh_xy_m &&
                                std::abs(accum_.dy) < stationary_thresh_xy_m &&
                                std::abs(accum_.dtheta_wheel) < stationary_thresh_theta;
  // Publish to AddGyroDelta so it can decide whether to EMA-update
  // the bias estimate from incoming samples. wheel_stationary_now_
  // stays at the latest tick's value until the next tick, so the
  // bias EMA sees the right gate state across many IMU samples.
  wheel_stationary_now_ = wheel_stationary;
  // Multi-source confirmation: if the wheel claims stationary but the
  // gyro reports a rotation rate above the residual-bias floor, the
  // robot is being externally rotated (hand-pushed off the dock,
  // lifted while spinning) and the encoders cannot see it because
  // they're free in mid-air. Don't snap dθ to 0 in that case — fall
  // through to the gyro path so yaw still tracks reality.
  const bool gyro_disagrees =
      accum_.max_abs_gyro_rad_per_s > params_.stationary_gyro_thresh_rad_per_s;
  const bool truly_stationary = wheel_stationary && !gyro_disagrees;

  double dtheta;
  double sigma_theta;
  if (truly_stationary)
  {
    dtheta = 0.0;
    sigma_theta = params_.stationary_sigma_theta;
  }
  else if (std::abs(accum_.dtheta_gyro) > 1e-9)
  {
    dtheta = accum_.dtheta_gyro;
    sigma_theta = params_.gyro_sigma_theta;
    // Stat: count cases where wheel said stationary but the gyro
    // overrode. Useful operational signal — if it spikes when the
    // robot is parked, the gyro threshold may be too tight.
    if (wheel_stationary && gyro_disagrees)
      ++stats_hand_push_;
  }
  else
  {
    dtheta = accum_.dtheta_wheel;
    sigma_theta = params_.wheel_sigma_theta;
  }

  // Slip veto on (dx, dy).
  //
  // The yaw selection above already chooses gyro over wheel encoders
  // when they disagree, so the BetweenFactor's *rotation* component is
  // honest. The translation is harder: wheel integration assumes
  // encoders measure ground-contact distance, which holds on dry
  // surfaces but breaks down on wet grass and during low-speed pivot
  // attempts where both drive wheels slip in the same direction. The
  // chassis IMU sees the whole truth — angular velocity directly, no
  // wheel-traction assumption — so a wheel-vs-gyro disagreement is
  // ground truth that the wheel readings are not trustworthy this
  // tick. Field-observed 2026-05-27: during a stuck dock-rotate
  // attempt the wheels reported a steady ~0.1 m/s forward velocity
  // and ~0.3 rad/s rotation, while the gyro saw <0.02 rad/s — the
  // wheel translation slid the map-frame estimate by 0.6 m in 6 s
  // even though the chassis hadn't moved, and the controller chased
  // the drift with more commanded motion, fueling the slip.
  //
  // Rule: when |dtheta_wheel - dtheta_gyro| is large enough that the
  // wheel-reported rotation can't be explained by gyro noise, zero
  // out the BetweenFactor's translation. The pose still advances in
  // yaw (from the gyro), and any GPS / scan-matching unary will pull
  // (x,y) in the right direction; without the veto the wheel
  // integration carries the pose along the phantom slip path
  // unopposed. The slip_sigma_xy floor keeps sigma_x/sigma_y tight
  // enough that GPS still anchors the estimate when available, but
  // not so loose that one tick of slip can shove the pose by tens of
  // centimetres.
  //
  // Threshold is gated by both the disagreement magnitude AND a
  // minimum gyro stillness — otherwise the slip detector would fire
  // every time the gyro updates faster than the wheel encoders, which
  // happens on every normal turn. The combination "wheels rotating
  // hard, gyro near zero" is the genuine slip signature.
  //
  // Evaluated over a sliding WINDOW of nodes, not on this node alone
  // (issue #516, slip_window.hpp). Per node the gate cannot separate the
  // encoder quantum — one tick of L/R asymmetry is ≈ 0.011 rad of wheel
  // dθ in a 40 ms frame — from genuine slip (≈ 0.012 rad/frame): field
  // 2026-09-02 it fired 1.33/s on STRAIGHT swaths (wheel 0.00 vs gyro
  // 0.05 rad/s at the increments) and zeroed ~5 % of nodes' translation
  // on jitter. The three thresholds keep their per-node meaning; the
  // window sums the deltas and scales the thresholds by its node count,
  // so sustained slip (~0.15 rad over 0.5 s) trips it while jitter
  // averages to ~0. The veto is still APPLIED per node — this node's
  // translation is zeroed while the window is flagged. A node that spans
  // longer than the window (stationary-throttled, up to 5 s) restarts it:
  // that node covers the whole window by itself, and carrying its
  // gyro-bias drift into the next 0.5 s of motion would mask slip onset.
  // slip_window_s: 0 therefore restarts on every node = the old per-node
  // gate, exactly.
  if (now_s - last_node_time_s_ > params_.slip_window_s)
    slip_window_.Clear();
  slip_window_.Push(accum_.dtheta_wheel, accum_.dtheta_gyro);
  const bool slip_detected = slip_window_.Detected(params_.slip_residual_thresh_rad * tick_scale,
                                                   params_.slip_gyro_max_rad * tick_scale,
                                                   params_.slip_wheel_min_rad * tick_scale);
  double dx_eff = accum_.dx;
  double dy_eff = accum_.dy;
  if (slip_detected)
  {
    dx_eff = 0.0;
    dy_eff = 0.0;
    ++stats_slip_veto_;
  }

  const gtsam::Pose2 between(dx_eff, dy_eff, dtheta);

  const auto k_prev = PoseKey(next_index_ - 1);
  const auto k_curr = PoseKey(next_index_);

  // Predict X_k from current estimate of X_{k-1}, fall back to last
  // known pose if iSAM2 hasn't seen X_{k-1} yet (shouldn't happen).
  gtsam::Pose2 X_prev;
  if (HasPoseAt(next_index_ - 1))
  {
    X_prev = PoseAt(next_index_ - 1);
  }
  else
  {
    X_prev = latest_ ? latest_->pose : gtsam::Pose2();
  }
  const gtsam::Pose2 X_pred = X_prev.compose(between);
  new_values_.insert(k_curr, X_pred);

  // 2. Wheel between-factor.
  // sigma_theta was already selected above with the same wheel/gyro/
  // stationary logic that drove dtheta — reuse it here so both halves
  // of the BetweenFactor stay consistent.
  //
  // ── Translational sigmas: distance-scaled random walk (issue #491) ──
  //
  // σ_x / σ_y are NOT fixed per-node constants. A per-node sigma makes the
  // accumulated uncertainty a function of how many nodes we happened to
  // create, not of how far the robot actually drove. Measured 2026-08-24 on
  // RTK-Fixed: the published major-axis σ read p50 0.574 m / max 1.391 m
  // while the GNSS receiver reported 0.014 m and FTC's cross-track error was
  // 0.008-0.016 m. At node_period_s = 0.04 and 0.2 m/s each hop covers 8 mm
  // yet the old fixed σ_x claimed 50-92 mm for it (~10x the step), so the
  // marginal on the newest node grew as σ·√N — 0.092·√40 = 0.58 m for the
  // ~40 hops between the last GNSS-anchored node and the head, matching the
  // observation. It was also cadence-dependent: the same physical motion
  // accumulated √2.5x more uncertainty at 25 Hz than at 10 Hz.
  //
  // Fix: make VARIANCE (not sigma) proportional to the distance the step
  // covered — σ = k·√d, Var = k²·d — so N hops of d/N metres sum to exactly
  // the variance of one hop of d metres. That is cadence-invariant by
  // construction, and is the noise-model counterpart of the tick_scale
  // applied to the per-tick GATES above (same reasoning: a quantity tuned
  // per-tick must be rescaled when the tick length changes). Sigma-
  // proportional scaling (σ = k·d, as the issue sketched) would have been
  // wrong in the opposite direction: it accumulates as k·d/√N, i.e. tighter
  // the faster you tick. Precedent for the √ form is a few lines below —
  // the gyro-bias random walk already uses σ = σ_rw·√dt.
  //
  // The noise distance uses the RAW accumulator, not the slip-vetoed
  // (dx_eff, dy_eff). What the encoders CLAIMED bounds how wrong they can
  // be; when the veto zeroes the delta the factor's meaning becomes "we
  // believe we did not move, ± the distance the wheels claimed", which is
  // exactly the uncertainty we want. Scoring the vetoed zero would instead
  // pin the pose to "stationary" at the moment we know least.
  const double claimed_step_m = std::hypot(accum_.dx, accum_.dy);
  const double accum_dt = (accum_.dt_total > 0.0) ? accum_.dt_total : params_.node_period_s;
  const double noise_dist_m = claimed_step_m + params_.wheel_creep_speed_mps * accum_dt;
  const double sigma_x_travel =
      std::max(kMinWheelSigmaM, params_.wheel_sigma_x_per_sqrt_m * std::sqrt(noise_dist_m));
  const double sigma_y_travel =
      std::max(kMinWheelSigmaM, params_.wheel_sigma_y_per_sqrt_m * std::sqrt(noise_dist_m));

  // ── Fault releases: pivot gate + adaptive inflation ────────────────
  //
  // These two stay PER-NODE ABSOLUTE sigmas and are applied as a lower BOUND
  // on the travel-scaled value, rather than being scaled by distance
  // themselves. That is deliberate: they do not model a distance-driven
  // random walk, they model a wheel FAULT — a measurement that is wrong in a
  // way unrelated to how far the encoders say we went — and their job is to
  // RELEASE the X constraint so GPS / scan-matching set XY.
  //
  // Scaling them per-distance would defeat them precisely in the case they
  // exist for. During a pivot the encoders report ~0.8 mm of phantom
  // translation per tick at 25 Hz, so a per-distance pivot sigma would
  // collapse to ~1.4 mm and pin the pose to the phantom motion — the exact
  // 0.2-0.4 m per-spin drift pivot_wheel_sigma_x was added to stop. Likewise
  // the adaptive term is driven by a yaw residual measuring rotation the
  // wheels invented; the linear error that implies is not bounded by the
  // (often vetoed, hence zero) reported step.
  //
  // Because these fire only on a detected fault, keeping them per-node does
  // not reintroduce the √N growth in normal driving: net_residual is 0 and
  // the pivot gate is shut, so the release is 0 and the travel term wins.
  //
  // sigma_x gates on the per-tick gyro yaw delta: during fast pivots
  // the wheels report phantom forward velocity (see GraphParams
  // comment) so swap to a loose sigma and let GPS / scan-matching
  // constrain XY. Gating on the gyro (not wheel-derived) dtheta
  // avoids feedback from the same encoder that's misreporting.
  double wheel_sigma_x_release =
      std::abs(accum_.dtheta_gyro) > params_.pivot_gate_dtheta_rad * tick_scale
          ? params_.pivot_wheel_sigma_x
          : 0.0;

  // Adaptive σ_x inflation from wheel↔gyro residual EMA. Skipped
  // entirely when adaptive_noise_enabled_gain == 0 (the parameter
  // defaults to 10 but yaml can disable). Pivot mode already
  // inflates σ_x to params_.pivot_wheel_sigma_x; in that case the
  // adaptive term layers on top, but the floor (pivot sigma) is
  // typically much larger than any residual-driven contribution
  // so the practical effect is negligible during pivots.
  if (params_.adaptive_noise_enabled_gain > 0.0)
  {
    // |wheel↔gyro residual| this tick. We compare the per-tick yaw
    // deltas (not the rate) so the noise scales naturally with
    // node_period_s: longer ticks = more accumulated slip = larger
    // residual = larger inflation.
    const double residual = std::abs(accum_.dtheta_wheel - accum_.dtheta_gyro);

    // EMA in continuous time: α = dt / (τ + dt). accum_dt (computed with
    // the travel sigmas above) is the wall time we've been accumulating
    // wheel/gyro samples; safe approximation of node_period_s when the
    // inputs are arriving on schedule, and it already falls back to one
    // full step when dt_total is 0 (rare — happens on the very first tick
    // before any wheel sample).
    const double tau = std::max(params_.adaptive_noise_ema_tau_s, 1.0e-3);
    const double alpha = accum_dt / (tau + accum_dt);
    residual_ema_ = (1.0 - alpha) * residual_ema_ + alpha * residual;

    // Floor: anything below this is sensor jitter, not slip.
    const double net_residual =
        std::max(0.0, residual_ema_ - params_.adaptive_noise_residual_floor_rad);
    wheel_sigma_x_release += params_.adaptive_noise_enabled_gain * net_residual;
  }

  // Travel-scaled random walk, floored by whichever fault release is active.
  const double wheel_sigma_x_eff = std::max(sigma_x_travel, wheel_sigma_x_release);
  last_wheel_sigma_x_eff_ = wheel_sigma_x_eff;

  // When IMU preintegration is active, the GyroPreintFactor below
  // owns the yaw constraint with a tight statistically-grounded
  // sigma. The wheel between-factor still carries xy translation but
  // we inflate its sigma_theta so it doesn't fight the preint factor.
  if (params_.use_imu_preint)
  {
    sigma_theta = 0.5;  // very loose — preint dominates yaw
  }

  auto between_noise = MakeDiagonal({
      wheel_sigma_x_eff,
      sigma_y_travel,
      sigma_theta,
  });
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, between, between_noise));

  // ── IMU preintegration: ternary GyroPreintFactor + bias RW ──────
  // The preint factor adds a yaw constraint that depends on the
  // jointly-optimised bias variable, and the random-walk between-
  // factor links consecutive bias variables so iSAM2 can propagate
  // bias estimates through the trajectory.
  if (params_.use_imu_preint && accum_.gyro_preint_dt > 0.0)
  {
    using gtsam::Symbol;
    const auto k_bias_prev = Symbol('b', next_index_ - 1);
    const auto k_bias_curr = Symbol('b', next_index_);

    // Insert the new bias variable initialised at the current best
    // estimate. iSAM2 will refine it on the next update.
    new_values_.insert(k_bias_curr, current_bias_estimate_);

    // Preint factor noise: sigma = √variance. Floor at a small value
    // to avoid a singular constraint when dt is tiny.
    const double sigma_preint = std::max(std::sqrt(accum_.gyro_preint_variance), 1e-4);
    auto preint_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(sigma_preint));
    new_factors_.add(GyroPreintFactor(k_prev,
                                      k_curr,
                                      k_bias_curr,
                                      accum_.gyro_preint_dtheta,
                                      accum_.gyro_preint_dt,
                                      preint_noise));

    // Bias random-walk between: bias_{k} = bias_{k-1} + N(0, σ_rw·√dt)
    const double sigma_bias_rw = params_.gyro_bias_rw_rad_per_s * std::sqrt(accum_.gyro_preint_dt);
    auto bias_rw_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(std::max(sigma_bias_rw, 1e-6)));
    new_factors_.add(gtsam::BetweenFactor<double>(k_bias_prev, k_bias_curr, 0.0, bias_rw_noise));
  }

  // 3. Queued unary factors. Wrap in Huber when caller flagged the
  // measurement as outlier-prone (RTK-Float / single fix on GPS;
  // magnetometer on yaw).
  if (queue_.gnss)
  {
    gtsam::SharedNoiseModel noise = MakeDiagonal({
        queue_.gnss->sigma,
        queue_.gnss->sigma,
    });
    if (queue_.gnss->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_gps),
                                                noise);
    }
    const auto factor_key = queue_.gnss->target_node ? PoseKey(*queue_.gnss->target_node) : k_curr;
    // A windowed asynchronous rebase may remove a historical target after
    // QueueGnss validated it. Drop that stale observation rather than silently
    // applying it to the current state at the wrong epoch.
    if (!queue_.gnss->target_node || HasPoseAt(*queue_.gnss->target_node))
    {
      new_factors_.add(GnssLeverArmFactor(factor_key,
                                          queue_.gnss->xy,
                                          gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y),
                                          noise));
    }
  }
  if (queue_.yaw)
  {
    gtsam::SharedNoiseModel noise = MakeDiagonal({queue_.yaw->sigma});
    if (queue_.yaw->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_yaw),
                                                noise);
    }
    new_factors_.add(YawUnaryFactor(k_curr, queue_.yaw->yaw, noise));
  }
  if (queue_.scan_between)
  {
    auto noise = MakeDiagonal({
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_theta,
    });
    new_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, queue_.scan_between->delta, noise));
  }
  if (queue_.lidar_map_xy)
  {
    // LiDAR map anchor: ABSOLUTE XY-ONLY constraint from the particle filter
    // localising against the occupancy grid built under RTK-Fixed. The filter
    // returns a full covariance: along a lone wall it is wide, across it
    // narrow — exactly the partial constraint a factor graph wants, so the
    // matrix is used as-is (floored) rather than collapsed to one sigma.
    // Heading is deliberately NOT constrained: a LiDAR-derived yaw prior once
    // injected a mirrored / 180°-flipped ICP heading that corrupted map→odom
    // on the robot (2026-07-22), so no LiDAR heading is ever fed into the
    // graph — yaw stays owned by the gyro between-factors and the loose
    // (σ≥0.30 rad) scan-between yaw. Huber-wrapped so a single biased
    // localisation is down-weighted.
    Eigen::Matrix2d cov = queue_.lidar_map_xy->cov;
    const double floor_var =
        params_.lidar_anchor_sigma_floor_m * params_.lidar_anchor_sigma_floor_m;
    cov(0, 0) = std::max(cov(0, 0), floor_var);
    cov(1, 1) = std::max(cov(1, 1), floor_var);
    // Symmetrise and keep it positive definite even if the filter's estimate
    // was numerically off.
    cov = 0.5 * (cov + cov.transpose());
    if (cov.determinant() <= 0.0)
      cov = Eigen::Matrix2d::Identity() * floor_var;
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Covariance(cov);
    if (queue_.lidar_map_xy->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                noise);
    }
    new_factors_.add(gtsam::PoseTranslationPrior<gtsam::Pose2>(
        k_curr, gtsam::Point2(queue_.lidar_map_xy->xy), noise));
    ++lidar_anchor_factors_;
  }

  // 4. iSAM2 update. Mark the cached full estimate dirty — callers
  //    that need ALL poses (viz / Save / LC search fallback) will
  //    refresh on demand. Per-Tick / per-LC lookups go through
  //    PoseAt() / HasPoseAt() which are O(depth) on the Bayes tree.
  if (!ApplyIsamUpdateLocked(new_factors_, new_values_))
  {
    // Graph was reset (ill-posed system). Don't publish a node this tick —
    // the manager is uninitialised; PoseAt(0) would return the datum origin
    // and teleport the robot. The next GPS/dock seed re-bootstraps.
    return std::nullopt;
  }
  estimate_dirty_ = true;
  new_factors_.resize(0);
  new_values_.clear();

  // 4b. Refresh the bias linearisation point from the new estimate
  //     when IMU preint is active. AddGyroDelta will subtract this
  //     from incoming samples until the next node creation.
  if (params_.use_imu_preint)
  {
    try
    {
      const auto k_bias = gtsam::Symbol('b', next_index_);
      current_bias_estimate_ = isam_.calculateEstimate<double>(k_bias);
    }
    catch (const std::exception&)
    {
      // Keep the previous estimate if iSAM2 hasn't materialised this
      // variable yet (shouldn't happen — we just inserted it).
    }
  }

  // 5. Marginal covariance — throttled. marginalCovariance is O(node
  //    count) on the Bayes tree path and dominates CPU once the graph
  //    passes a few thousand nodes. The value is only consumed by the
  //    diagnostics topic + published Odometry, neither of which needs
  //    10 Hz freshness — recomputing every Nth tick (default 10 → 1 Hz)
  //    keeps the displayed σ accurate without burning CPU on every
  //    Tick. Re-uses the previous tick's covariance when not due.
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0;
  ++ticks_since_cov_;
  const bool refresh_cov = ticks_since_cov_ >= std::max(1, params_.cov_update_every_n);
  if (refresh_cov)
  {
    try
    {
      cov = isam_.marginalCovariance(k_curr);
    }
    catch (const std::exception&)
    {
      // leave conservative default
    }
    ticks_since_cov_ = 0;
  }
  else if (latest_)
  {
    cov = latest_->covariance;
  }

  TickOutput out;
  out.pose = PoseAt(next_index_);
  out.covariance = cov;
  out.node_index = next_index_;
  out.timestamp = now_s;
  latest_ = out;

  // 6. Reset for next tick.
  ++next_index_;
  last_node_time_s_ = now_s;
  node_time_index_.emplace_back(now_s, out.node_index);
  const size_t time_index_cap =
      params_.max_graph_nodes > 0 ? static_cast<size_t>(params_.max_graph_nodes) : 10000U;
  while (node_time_index_.size() > std::max<size_t>(1U, time_index_cap))
    node_time_index_.pop_front();
  accum_.Reset();
  queue_.gnss.reset();
  queue_.yaw.reset();
  queue_.scan_between.reset();
  queue_.lidar_map_xy.reset();

  return out;
}

}  // namespace fusion_graph
