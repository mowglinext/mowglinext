// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "process_window_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fusion_graph
{
ProcessWindowPolicyResult SelectProcessWindowPolicy(const GraphParams& params,
                                                    const ProcessWindowPolicyInput& input,
                                                    ProcessWindowPolicyState state)
{
  // Cadence scaling for the per-tick dead-reckoning gates below. Each gate
  // compares the PER-TICK accumulated wheel/gyro delta against a fixed radian/
  // metre threshold, so its effective rad/s (or m/s) trip point is
  // threshold / node_period_s and would drift with cadence. Multiplying the
  // thresholds by this factor keeps the tuned trip points invariant across the
  // 25 Hz (launch default), 50 Hz (yaml default) and 10 Hz configurations.
  // At the 25 Hz reference the factor is exactly 1.0 (no behaviour change on
  // the deployed robot). See kTunedNodePeriodS in graph_params.hpp.
  const double tick_scale = params.node_period_s / kTunedNodePeriodS;

  // 1. Select the wheel between-factor's relative pose from X_{k-1} to X_k.
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
  const double stationary_thresh_xy_m = params.stationary_thresh_xy_m * tick_scale;
  const double stationary_thresh_theta = params.stationary_thresh_theta * tick_scale;
  const bool wheel_stationary = std::abs(input.dx) < stationary_thresh_xy_m &&
                                std::abs(input.dy) < stationary_thresh_xy_m &&
                                std::abs(input.dtheta_wheel) < stationary_thresh_theta;
  // Propose the AddGyroDelta gate. Once adopted it stays at the latest
  // tick's value until the next tick, so the bias EMA sees the right
  // gate state across many IMU samples.
  state.wheel_stationary_now = wheel_stationary;
  // Multi-source confirmation: if the wheel claims stationary but the
  // gyro reports a rotation rate above the residual-bias floor, the
  // robot is being externally rotated (hand-pushed off the dock,
  // lifted while spinning) and the encoders cannot see it because
  // they're free in mid-air. Don't snap dθ to 0 in that case — fall
  // through to the gyro path so yaw still tracks reality.
  const bool gyro_disagrees =
      input.max_abs_gyro_rad_per_s > params.stationary_gyro_thresh_rad_per_s;
  const bool truly_stationary = wheel_stationary && !gyro_disagrees;

  double dtheta;
  double sigma_theta;
  bool hand_push_detected = false;
  if (truly_stationary)
  {
    dtheta = 0.0;
    sigma_theta = params.stationary_sigma_theta;
  }
  else if (std::abs(input.dtheta_gyro) > 1e-9)
  {
    dtheta = input.dtheta_gyro;
    sigma_theta = params.gyro_sigma_theta;
    // Stat: count cases where wheel said stationary but the gyro
    // overrode. Useful operational signal — if it spikes when the
    // robot is parked, the gyro threshold may be too tight.
    if (wheel_stationary && gyro_disagrees)
    {
      ++state.hand_push_count;
      hand_push_detected = true;
    }
  }
  else
  {
    dtheta = input.dtheta_wheel;
    sigma_theta = params.wheel_sigma_theta;
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
  // yaw (from the gyro), and any GPS / LiDAR map unary will pull
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
  if (input.now_s - input.last_node_time_s > params.slip_window_s)
    state.slip_window.Clear();
  state.slip_window.Push(input.dtheta_wheel, input.dtheta_gyro);
  const bool slip_detected =
      state.slip_window.Detected(params.slip_residual_thresh_rad * tick_scale,
                                 params.slip_gyro_max_rad * tick_scale,
                                 params.slip_wheel_min_rad * tick_scale);
  double dx_eff = input.dx;
  double dy_eff = input.dy;
  if (slip_detected)
  {
    dx_eff = 0.0;
    dy_eff = 0.0;
    ++state.slip_veto_count;
  }

  // 2. Wheel between-factor noise.
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
  // Signed displacement can cancel within a node when direction changes.
  // Random-walk variance follows all reported travel, including that return.
  const double claimed_step_m = input.wheel_travel_m;
  const double accum_dt = (input.dt_total > 0.0) ? input.dt_total : params.node_period_s;
  const double noise_dist_m = claimed_step_m + params.wheel_creep_speed_mps * accum_dt;
  const double sigma_x_travel =
      std::max(kMinWheelSigmaM, params.wheel_sigma_x_per_sqrt_m * std::sqrt(noise_dist_m));
  const double sigma_y_travel =
      std::max(kMinWheelSigmaM, params.wheel_sigma_y_per_sqrt_m * std::sqrt(noise_dist_m));

  // ── Fault releases: pivot gate + adaptive inflation ────────────────
  //
  // These two stay PER-NODE ABSOLUTE sigmas and are applied as a lower BOUND
  // on the travel-scaled value, rather than being scaled by distance
  // themselves. That is deliberate: they do not model a distance-driven
  // random walk, they model a wheel FAULT — a measurement that is wrong in a
  // way unrelated to how far the encoders say we went — and their job is to
  // RELEASE the X constraint so GPS / LiDAR map set XY.
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
  // comment) so swap to a loose sigma and let GPS / LiDAR map
  // constrain XY. Gating on the gyro (not wheel-derived) dtheta
  // avoids feedback from the same encoder that's misreporting.
  double wheel_sigma_x_release =
      std::abs(input.dtheta_gyro) > params.pivot_gate_dtheta_rad * tick_scale
          ? params.pivot_wheel_sigma_x
          : 0.0;

  // Adaptive σ_x inflation from wheel↔gyro residual EMA. Skipped
  // entirely when adaptive_noise_enabled_gain == 0 (the parameter
  // defaults to 10 but yaml can disable). Pivot mode already
  // inflates σ_x to params.pivot_wheel_sigma_x; in that case the
  // adaptive term layers on top, but the floor (pivot sigma) is
  // typically much larger than any residual-driven contribution
  // so the practical effect is negligible during pivots.
  if (params.adaptive_noise_enabled_gain > 0.0)
  {
    // |wheel↔gyro residual| this tick. We compare the per-tick yaw
    // deltas (not the rate) so the noise scales naturally with
    // node_period_s: longer ticks = more accumulated slip = larger
    // residual = larger inflation.
    const double residual = std::abs(input.dtheta_wheel - input.dtheta_gyro);

    // EMA in continuous time: α = dt / (τ + dt). accum_dt (computed with
    // the travel sigmas above) is the wall time we've been accumulating
    // wheel/gyro samples; safe approximation of node_period_s when the
    // inputs are arriving on schedule, and it already falls back to one
    // full step when dt_total is 0 (rare — happens on the very first tick
    // before any wheel sample).
    const double tau = std::max(params.adaptive_noise_ema_tau_s, 1.0e-3);
    const double alpha = accum_dt / (tau + accum_dt);
    state.residual_ema = (1.0 - alpha) * state.residual_ema + alpha * residual;

    // Floor: anything below this is sensor jitter, not slip.
    const double net_residual =
        std::max(0.0, state.residual_ema - params.adaptive_noise_residual_floor_rad);
    wheel_sigma_x_release += params.adaptive_noise_enabled_gain * net_residual;
  }

  // Travel-scaled random walk, floored by whichever fault release is active.
  const double wheel_sigma_x_eff = std::max(sigma_x_travel, wheel_sigma_x_release);
  state.last_wheel_sigma_x_eff = wheel_sigma_x_eff;

  // When IMU preintegration is active, the caller's GyroPreintFactor
  // owns the yaw constraint with a tight statistically-grounded
  // sigma. The wheel between-factor still carries xy translation but
  // we inflate its sigma_theta so it doesn't fight the preint factor.
  if (params.use_imu_preint)
  {
    sigma_theta = 0.5;  // very loose — preint dominates yaw
  }

  return {gtsam::Pose2(dx_eff, dy_eff, dtheta),
          !slip_detected,
          !truly_stationary || params.use_imu_preint,
          hand_push_detected,
          slip_detected,
          wheel_sigma_x_eff,
          sigma_y_travel,
          sigma_theta,
          std::move(state)};
}
}  // namespace fusion_graph
