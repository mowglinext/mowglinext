// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for the wheel between-factor's TRANSLATIONAL noise model
// (issue #491).
//
// Bug: σ_x was a fixed PER-NODE constant with no reference to the distance the
// step covered. At node_period_s = 0.04 (25 Hz) and 0.2 m/s each hop covers
// 8 mm while the factor claimed 50-92 mm of along-track uncertainty for it, so
// the marginal on the newest node grew as σ·√N per HOP rather than with
// distance travelled. Measured 2026-08-24 on RTK-Fixed: published major-axis σ
// p50 0.574 m / p90 0.957 / max 1.391 against a GNSS horizontal accuracy of
// 0.014 m and an FTC cross-track error of 0.008-0.016 m. It was also
// cadence-dependent — the same physical motion accumulated √2.5x more
// uncertainty at 25 Hz than at 10 Hz.
//
// Fix: VARIANCE proportional to the distance the step covered,
//
//   σ = wheel_sigma_x_per_sqrt_m · √(step_m + wheel_creep_speed_mps · dt)
//
// so N hops of d/N metres sum to exactly the variance of one hop of d metres.
//
// The tests below observe the per-node σ_x through GraphManager::Stats()
// (wheel_sigma_x_eff, the sigma actually handed to the BetweenFactor) and sum
// its square across the nodes a run produced — that sum is the dead-reckoned
// variance the marginal accumulates between two GNSS-anchored nodes, which is
// the quantity the bug was about.

#include <cmath>
#include <cstdio>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

// Wheel/gyro samples arrive at 100 Hz; node cadence is what the tests vary.
constexpr double kSampleDt = 0.01;  // s — one wheel/gyro sample
constexpr int kSamples = 160;  // 1.6 s of driving
constexpr double kSpeed = 0.20;  // m/s — mowing speed
constexpr double kTotalDist = kSpeed * kSampleDt * kSamples;  // 0.32 m
constexpr double kTotalTime = kSampleDt * kSamples;  // 1.6 s

// Node period that fires exactly every `samples` wheel samples. The 1 ns
// shave absorbs binary-float error in kSampleDt * n (0.01 * 10 evaluates just
// BELOW 0.1), which would otherwise slip a node by one sample and leave the
// cadences covering slightly different distances.
constexpr double NodePeriodForSamples(int samples)
{
  return kSampleDt * samples - 1.0e-9;
}

fg::GraphParams MakeParams(double node_period_s)
{
  fg::GraphParams gp;
  gp.node_period_s = node_period_s;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_creep_speed_mps = 0.04;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_thresh_xy_m = 1.0e-3;
  gp.stationary_thresh_theta = 2.0e-3;
  gp.stationary_sigma_theta = 1.0e-3;
  // Disable the stationary node throttle so cadence is purely node_period_s.
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  // No slip in these runs, but keep the gain live so the tests also prove the
  // adaptive term contributes nothing when the wheels and gyro agree.
  gp.adaptive_noise_enabled_gain = 10.0;
  gp.adaptive_noise_ema_tau_s = 0.5;
  gp.adaptive_noise_residual_floor_rad = 0.005;
  return gp;
}

struct RunResult
{
  int nodes = 0;
  double sum_var = 0.0;  // Σ σ_x², the accumulated dead-reckoning variance
  double last_sigma_x = 0.0;

  double sigma() const
  {
    return std::sqrt(sum_var);
  }
};

// Drive straight at `speed` for `samples` × kSampleDt seconds, feeding matched
// wheel + gyro samples, and accumulate σ_x² over every node the manager
// created. Node cadence comes from the params, so the SAME physical motion can
// be replayed at different cadences.
RunResult DriveStraight(const fg::GraphParams& gp, int samples, double speed)
{
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  RunResult r;
  for (int i = 0; i < samples; ++i)
  {
    gm.AddWheelTwist(speed, 0.0, 0.0, kSampleDt);
    gm.AddGyroDelta(0.0, kSampleDt);
    if (gm.Tick(kSampleDt * (i + 1)).has_value())
    {
      const double sigma_x = gm.Stats().wheel_sigma_x_eff;
      r.sum_var += sigma_x * sigma_x;
      r.last_sigma_x = sigma_x;
      ++r.nodes;
    }
  }
  return r;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// 1. THE key property: N hops of d/N metres must accumulate exactly the
//    same uncertainty as one hop of d metres. This is what "per-distance"
//    means, and it is precisely what the per-node model got wrong.
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, ManySmallHopsEqualOneBigHop)
{
  // Identical wheel/gyro input, identical elapsed time, identical distance —
  // only the node cadence differs: 160 hops of 2 mm vs 1 hop of 0.32 m.
  const auto fine = DriveStraight(MakeParams(NodePeriodForSamples(1)), kSamples, kSpeed);
  const auto coarse = DriveStraight(MakeParams(NodePeriodForSamples(kSamples)), kSamples, kSpeed);

  std::printf("[WheelSigmaScaling] %d hops → σ=%.6f m | %d hop → σ=%.6f m\n",
              fine.nodes,
              fine.sigma(),
              coarse.nodes,
              coarse.sigma());

  ASSERT_EQ(fine.nodes, kSamples);
  ASSERT_EQ(coarse.nodes, 1);

  // Accumulated σ must match to numerical precision, not merely "closely".
  EXPECT_NEAR(fine.sigma(), coarse.sigma(), 1.0e-9);

  // And it must equal the closed form σ = k·√(d + creep·t).
  const double expected = 0.05 * std::sqrt(kTotalDist + 0.04 * kTotalTime);
  EXPECT_NEAR(fine.sigma(), expected, 1.0e-9);

  // Guard against the trivially-passing implementation where BOTH collapse to
  // the per-node floor: the per-hop σ must be much smaller than the total.
  EXPECT_LT(fine.last_sigma_x, 0.25 * fine.sigma());
}

// ─────────────────────────────────────────────────────────────────────
// 2. Cadence invariance. The 10 Hz / 25 Hz / 50 Hz configurations must
//    report the same uncertainty for the same physical drive — the same
//    property tick_scale gives the per-tick GATES, applied to the noise
//    model. Under the old per-node σ, 25 Hz accumulated √2.5x more than
//    10 Hz for identical motion.
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, AccumulatedSigmaIsCadenceInvariant)
{
  const auto at_50hz = DriveStraight(MakeParams(NodePeriodForSamples(2)), kSamples, kSpeed);
  const auto at_25hz = DriveStraight(MakeParams(NodePeriodForSamples(4)), kSamples, kSpeed);
  const auto at_10hz = DriveStraight(MakeParams(NodePeriodForSamples(10)), kSamples, kSpeed);

  std::printf(
      "[WheelSigmaScaling] 50Hz %d nodes σ=%.6f | 25Hz %d nodes σ=%.6f "
      "| 10Hz %d nodes σ=%.6f\n",
      at_50hz.nodes,
      at_50hz.sigma(),
      at_25hz.nodes,
      at_25hz.sigma(),
      at_10hz.nodes,
      at_10hz.sigma());

  // Different node counts...
  EXPECT_GT(at_25hz.nodes, at_10hz.nodes);
  // ...same accumulated uncertainty.
  EXPECT_NEAR(at_25hz.sigma(), at_10hz.sigma(), 1.0e-9);
  EXPECT_NEAR(at_50hz.sigma(), at_25hz.sigma(), 1.0e-9);
}

// ─────────────────────────────────────────────────────────────────────
// 3. Uncertainty must grow with DISTANCE: twice the drive, √2 the sigma
//    (a random walk, not a straight line and not a constant).
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, SigmaGrowsAsSqrtOfDistance)
{
  const auto one_leg = DriveStraight(MakeParams(NodePeriodForSamples(1)), kSamples, kSpeed);
  const auto two_legs = DriveStraight(MakeParams(NodePeriodForSamples(1)), 2 * kSamples, kSpeed);

  std::printf("[WheelSigmaScaling] %.2f m → σ=%.6f | %.2f m → σ=%.6f (ratio %.4f)\n",
              kTotalDist,
              one_leg.sigma(),
              2.0 * kTotalDist,
              two_legs.sigma(),
              two_legs.sigma() / one_leg.sigma());

  EXPECT_NEAR(two_legs.sigma() / one_leg.sigma(), std::sqrt(2.0), 1.0e-9);
}

// ─────────────────────────────────────────────────────────────────────
// 4. The reported number is now physically plausible. This is the
//    user-visible symptom in #491: on RTK-Fixed the published σ read
//    0.574 m (p50) where the receiver reported 0.014 m and FTC held
//    8-16 mm of cross-track error. Between two GNSS-anchored nodes the
//    wheel chain must contribute centimetres, not half a metre.
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, DeployedMowingRegimeIsCentimetreScale)
{
  const auto run = DriveStraight(MakeParams(NodePeriodForSamples(4)), kSamples, kSpeed);

  std::printf("[WheelSigmaScaling] deployed regime (25 Hz): %d hops over %.2f m → σ=%.4f m\n",
              run.nodes,
              kTotalDist,
              run.sigma());

  // The old per-node model produced 0.05·√40 = 0.32 m over these same 40
  // hops (0.58 m once the adaptive term had inflated σ_x to the
  // field-observed 0.092).
  EXPECT_LT(run.sigma(), 0.10);
  // ...but not so tight that the wheel chain out-votes an RTK-Fixed fix
  // (σ ≈ 0.014 m) over the same window.
  EXPECT_GT(run.sigma(), 0.014);
}

// ─────────────────────────────────────────────────────────────────────
// 5. A zero-length step must still yield a usable, non-singular sigma.
//    The floor is a creep DISTANCE (wheel_creep_speed_mps · dt), not a
//    constant sigma, so it too stays cadence-invariant.
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, ZeroStepUsesCreepFloorAndStaysNonSingular)
{
  const auto parked = DriveStraight(MakeParams(NodePeriodForSamples(1)), kSamples, 0.0);

  std::printf("[WheelSigmaScaling] parked: %d nodes, per-node σ=%.6f m, total σ=%.6f m\n",
              parked.nodes,
              parked.last_sigma_x,
              parked.sigma());

  ASSERT_GT(parked.nodes, 0);
  // Strictly positive — a zero sigma would make the GTSAM noise model
  // singular.
  EXPECT_GT(parked.last_sigma_x, 0.0);
  EXPECT_GE(parked.last_sigma_x, fg::kMinWheelSigmaM);
  // Exactly the creep floor for one node.
  EXPECT_NEAR(parked.last_sigma_x, 0.05 * std::sqrt(0.04 * kSampleDt), 1.0e-9);
  // And the accumulated parked uncertainty is still time-invariant across
  // cadences: k·√(creep·t).
  EXPECT_NEAR(parked.sigma(), 0.05 * std::sqrt(0.04 * kTotalTime), 1.0e-9);
}

// ─────────────────────────────────────────────────────────────────────
// 6. The pivot release must survive the change. It is deliberately a
//    PER-NODE absolute sigma applied as a floor, not a distance-scaled
//    term: during a pivot the encoders report ~0.8 mm of phantom
//    translation per tick, so a distance-scaled pivot sigma would
//    collapse to ~1.4 mm and pin the pose to the phantom motion — the
//    exact 0.2-0.4 m per-spin drift pivot_wheel_sigma_x exists to stop.
// ─────────────────────────────────────────────────────────────────────
TEST(WheelSigmaScaling, PivotReleaseStillOverridesTheTravelTerm)
{
  auto gp = MakeParams(NodePeriodForSamples(1));
  gp.pivot_gate_dtheta_rad = 0.012;
  gp.pivot_wheel_sigma_x = 0.5;

  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  // In-place spin at 1 rad/s with the diff-drive's phantom forward vx.
  // Per-tick gyro dθ = 0.01 rad, well over the cadence-scaled 0.003 rad gate.
  constexpr double kPhantomVx = 0.021;  // m/s, measured 2026-05-14
  constexpr double kSpinWz = 1.0;  // rad/s
  for (int i = 0; i < kSamples; ++i)
  {
    gm.AddWheelTwist(kPhantomVx, 0.0, kSpinWz, kSampleDt);
    gm.AddGyroDelta(kSpinWz, kSampleDt);
    gm.Tick(kSampleDt * (i + 1));
  }

  const double sigma_x = gm.Stats().wheel_sigma_x_eff;
  std::printf("[WheelSigmaScaling] pivot: σ_x_eff=%.4f m (travel term would be %.6f)\n",
              sigma_x,
              0.05 * std::sqrt(kPhantomVx * kSampleDt + 0.04 * kSampleDt));

  // The travel term for an 0.84 mm phantom step is ~1.3 mm; the release must
  // win.
  EXPECT_GE(sigma_x, gp.pivot_wheel_sigma_x);
  EXPECT_LT(0.05 * std::sqrt(kPhantomVx * kSampleDt + 0.04 * kSampleDt), 0.01);
}
