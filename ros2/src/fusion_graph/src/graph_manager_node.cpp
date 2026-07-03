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
  // is clock-source-agnostic (no-op on real hardware where they agree).
  if (last_node_time_s_ > now_s)
    last_node_time_s_ = now_s;
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
  const bool wheel_stationary = std::abs(accum_.dx) < params_.stationary_thresh_xy_m &&
                                std::abs(accum_.dy) < params_.stationary_thresh_xy_m &&
                                std::abs(accum_.dtheta_wheel) < params_.stationary_thresh_theta;
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
  const double wheel_gyro_residual = std::abs(accum_.dtheta_wheel - accum_.dtheta_gyro);
  const bool slip_detected = wheel_gyro_residual > params_.slip_residual_thresh_rad &&
                             std::abs(accum_.dtheta_gyro) < params_.slip_gyro_max_rad &&
                             std::abs(accum_.dtheta_wheel) > params_.slip_wheel_min_rad;
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
  // sigma_x gates on the per-tick gyro yaw delta: during fast pivots
  // the wheels report phantom forward velocity (see GraphParams
  // comment) so swap to a loose sigma and let GPS / scan-matching
  // constrain XY. Gating on the gyro (not wheel-derived) dtheta
  // avoids feedback from the same encoder that's misreporting.
  double wheel_sigma_x_eff = std::abs(accum_.dtheta_gyro) > params_.pivot_gate_dtheta_rad
                                 ? params_.pivot_wheel_sigma_x
                                 : params_.wheel_sigma_x;

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

    // EMA in continuous time: α = dt / (τ + dt). dt_total here is the
    // wall time we've been accumulating wheel/gyro samples; safe
    // approximation of node_period_s when the inputs are arriving on
    // schedule. Falls back to one full step when dt_total is 0 (rare —
    // happens on the very first tick before any wheel sample).
    const double dt = (accum_.dt_total > 0.0) ? accum_.dt_total : params_.node_period_s;
    const double tau = std::max(params_.adaptive_noise_ema_tau_s, 1.0e-3);
    const double alpha = dt / (tau + dt);
    residual_ema_ = (1.0 - alpha) * residual_ema_ + alpha * residual;

    // Floor: anything below this is sensor jitter, not slip.
    const double net_residual =
        std::max(0.0, residual_ema_ - params_.adaptive_noise_residual_floor_rad);
    wheel_sigma_x_eff += params_.adaptive_noise_enabled_gain * net_residual;
  }
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
      params_.wheel_sigma_y,
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
    new_factors_.add(GnssLeverArmFactor(
        k_curr, queue_.gnss->xy, gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y), noise));
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
  if (queue_.scan_to_keyframe)
  {
    // ABSOLUTE xy constraint on the current node from a frozen RTK-anchored
    // keyframe match. PoseTranslationPrior pins X_curr.translation() to abs_xy
    // (yaw untouched). Huber-wrapped like the GPS factor so a single biased
    // keyframe match is down-weighted, not trusted. This is the factor that
    // bounds absolute error during RTK-Float; rides the same new_factors_ batch
    // through ApplyIsamUpdateLocked so it's rebase-safe.
    const double s = std::max(queue_.scan_to_keyframe->sigma_xy, 1.0e-4);
    gtsam::SharedNoiseModel noise = MakeDiagonal({s, s});
    if (queue_.scan_to_keyframe->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_gps),
                                                noise);
    }
    new_factors_.add(gtsam::PoseTranslationPrior<gtsam::Pose2>(
        k_curr, gtsam::Point2(queue_.scan_to_keyframe->abs_xy), noise));
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
  accum_.Reset();
  queue_.gnss.reset();
  queue_.yaw.reset();
  queue_.scan_between.reset();
  queue_.scan_to_keyframe.reset();

  return out;
}

}  // namespace fusion_graph
