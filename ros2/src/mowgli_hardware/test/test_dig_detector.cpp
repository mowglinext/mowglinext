// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the wheel-slip "digging" detector and its bounded
// reverse-escape budget. Pure logic, no ROS — mirrors test_ftc_stall.cpp.

#include <limits>

#include "mowgli_hardware/dig_detector.hpp"
#include "mowgli_hardware/gnss_hardware_status.hpp"
#include "mowgli_interfaces/gnss_observation_freshness.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;
namespace freshness = mowgli_interfaces::gnss_observation_freshness;

namespace
{
constexpr std::int64_t kSecond = 1000000000LL;

// The map side is fed as a POSITION (DigDecide measures net displacement from
// its own per-window anchor), so the helpers integrate a straight-line track.

// Drive `seconds` of perfectly-tracking motion (map keeps up with wheels).
mh::DigAction RunGoodTraction(const mh::DigDetectorCfg& cfg,
                              mh::DigDetectorState& st,
                              double seconds,
                              double dt = 0.1,
                              double yaw_rate = 0.0,
                              double* map_x = nullptr)
{
  double local_x = 0.0;
  double& x = map_x ? *map_x : local_x;
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    // 0.3 m/s commanded, 0.03 m per 0.1 s tick on BOTH wheel and map.
    x += 0.03;
    last = mh::DigDecide(cfg, st, 0.3, 0.03, x, 0.0, 0.003, yaw_rate, dt).action;
  }
  return last;
}

// Drive `seconds` of digging: wheels turn, the fused map pose barely moves.
mh::DigAction RunDigging(const mh::DigDetectorCfg& cfg,
                         mh::DigDetectorState& st,
                         double seconds,
                         double pos_sigma = 0.003,
                         double dt = 0.1,
                         double yaw_rate = 0.0,
                         double* map_x = nullptr)
{
  double local_x = 0.0;
  double& x = map_x ? *map_x : local_x;
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < seconds; t += dt)
  {
    x += 0.001;
    last = mh::DigDecide(cfg, st, 0.3, 0.03, x, 0.0, pos_sigma, yaw_rate, dt).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  return last;
}
}  // namespace

// ── Detection ───────────────────────────────────────────────────────────────

TEST(DigDetector, GoodTractionNeverTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  EXPECT_EQ(RunGoodTraction(cfg, st, 10.0), mh::DigAction::kNone);
}

TEST(DigDetector, SpinningWheelsWithNoMapProgressTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 10.0), mh::DigAction::kDig);
}

TEST(DigDetector, DoesNotTripBeforeTheWindowElapses)
{
  mh::DigDetectorCfg cfg;  // window_s = 1.2
  mh::DigDetectorState st;

  // 0.5 s of digging is not yet enough evidence.
  EXPECT_EQ(RunDigging(cfg, st, 0.5), mh::DigAction::kNone);
}

TEST(DigDetector, UntrustworthyFusedPoseSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Same digging signature, but RTK-Float: sigma way above max_pos_sigma.
  // Must NOT trip — the map pose can't distinguish slip from GPS noise.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.9), mh::DigAction::kNone);
}

TEST(DigDetector, StationaryRobotNeverTrips)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Not commanded to move: no wheel travel, no map travel.
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_EQ(mh::DigDecide(cfg, st, 0.0, 0.0, 0.0, 0.0, 0.003, 0.0, 0.1).action,
              mh::DigAction::kNone);
  }
}

TEST(DigDetector, SlowLegitimateCreepIsNotADig)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Docking creep: commanded slowly, wheels AND map agree at 0.005 m/tick.
  mh::DigAction last = mh::DigAction::kNone;
  double map_x = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    map_x += 0.005;
    last = mh::DigDecide(cfg, st, 0.06, 0.005, map_x, 0.0, 0.003, 0.0, 0.1).action;
  }
  EXPECT_EQ(last, mh::DigAction::kNone);
}

TEST(DigDetector, BlockedChassisWithNoWheelTravelIsNotOurCase)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Hard stall: wheels don't turn either. Firmware anti-dig owns this;
  // we require min_wheel_dist of claimed travel before calling it a dig.
  mh::DigAction last = mh::DigAction::kNone;
  for (int i = 0; i < 100; ++i)
  {
    last = mh::DigDecide(cfg, st, 0.3, 0.0, 0.0, 0.0, 0.003, 0.0, 0.1).action;
  }
  EXPECT_EQ(last, mh::DigAction::kNone);
}

TEST(DigDetector, TractionReturningResetsTheWindow)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  double map_x = 0.0;  // one continuous track across all three phases
  RunDigging(cfg, st, 0.5, 0.003, 0.1, 0.0, &map_x);  // partial evidence
  RunGoodTraction(cfg, st, 2.0, 0.1, 0.0, &map_x);  // traction returns -> must clear
  EXPECT_EQ(RunDigging(cfg, st, 0.5, 0.003, 0.1, 0.0, &map_x), mh::DigAction::kNone);
}

TEST(DigDetector, DisabledNeverTrips)
{
  mh::DigDetectorCfg cfg;
  cfg.enabled = false;
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 10.0), mh::DigAction::kNone);
}

// ── Bounded reverse escape ──────────────────────────────────────────────────

TEST(DigEscape, ReversesWhileBudgetRemains)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  const double v = mh::DigEscapeStep(cfg, st, 0.1);
  EXPECT_LT(v, 0.0) << "escape must command reverse";
  EXPECT_NEAR(v, -cfg.reverse_speed, 1e-9);
}

TEST(DigEscape, StopsAfterDistanceBudgetSpent)
{
  mh::DigEscapeCfg cfg;  // reverse_dist default 0.30 m
  mh::DigEscapeState st;

  double v = 0.0;
  for (int i = 0; i < 200; ++i)
  {
    v = mh::DigEscapeStep(cfg, st, 0.1);
  }
  EXPECT_EQ(v, 0.0);
  EXPECT_TRUE(mh::DigEscapeDone(cfg, st));
  EXPECT_LE(st.travelled, cfg.reverse_dist + 1e-9) << "must not overshoot the budget";
}

TEST(DigEscape, StopsAfterTimeoutEvenWithoutDistance)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  // dt large enough to blow the timeout before the distance budget.
  mh::DigEscapeStep(cfg, st, cfg.timeout_s + 0.1);
  EXPECT_TRUE(mh::DigEscapeDone(cfg, st));
  EXPECT_EQ(mh::DigEscapeStep(cfg, st, 0.1), 0.0);
}

TEST(DigEscape, IgnoresNonPositiveDt)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  mh::DigEscapeStep(cfg, st, -0.5);
  EXPECT_EQ(st.travelled, 0.0);
  EXPECT_EQ(st.elapsed, 0.0);
}

TEST(DigEscape, ResetRestoresFullBudget)
{
  mh::DigEscapeCfg cfg;
  mh::DigEscapeState st;

  for (int i = 0; i < 200; ++i)
  {
    mh::DigEscapeStep(cfg, st, 0.1);
  }
  ASSERT_TRUE(mh::DigEscapeDone(cfg, st));

  st = mh::DigEscapeState{};
  EXPECT_FALSE(mh::DigEscapeDone(cfg, st));
  EXPECT_LT(mh::DigEscapeStep(cfg, st, 0.1), 0.0);
}

// Regression: the verdict must CARRY the evidence, because DigDecide resets
// its window on every verdict. Reading the state after a kDig reports zeros,
// which is what the dig event and the operator-facing log would have shown.
TEST(DigDetector, VerdictCarriesEvidenceAfterWindowReset)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  mh::DigVerdict v;
  double map_x = 0.0;
  for (int i = 0; i < 100; ++i)
  {
    map_x += 0.001;
    v = mh::DigDecide(cfg, st, 0.3, 0.03, map_x, 0.0, 0.003, 0.0, 0.1);
    if (v.action == mh::DigAction::kDig)
    {
      break;
    }
  }

  ASSERT_EQ(v.action, mh::DigAction::kDig);
  EXPECT_GT(v.wheel_dist, cfg.min_wheel_dist) << "evidence must survive the reset";
  EXPECT_LT(v.map_dist, v.wheel_dist * cfg.progress_fraction);
  EXPECT_EQ(st.wheel_dist, 0.0) << "window itself is reset for the next pass";
}

// ─────────────────────────────────────────────────────────────────────────────
// Turn exclusion
// ─────────────────────────────────────────────────────────────────────────────

TEST(DigDetector, TurningSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // A swath U-turn at connector_turn_radius 0.18 m and 0.2 m/s is ~1.1 rad/s.
  // The wheel-vs-map ratio collapses there on a perfectly healthy robot
  // (arc vs chord, plus the graph's own estimate degrading), so it must not
  // be read as a dig.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, 1.1), mh::DigAction::kNone);
}

TEST(DigDetector, StaleGyroSuppressesDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // The bridge passes infinity when the gyro is stale: the turn exclusion
  // cannot be evaluated, so stand down rather than guess.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, std::numeric_limits<double>::infinity()),
            mh::DigAction::kNone);

  mh::DigDetectorState st_nan;
  EXPECT_EQ(RunDigging(cfg, st_nan, 10.0, 0.003, 0.1, std::numeric_limits<double>::quiet_NaN()),
            mh::DigAction::kNone);
}

TEST(DigDetector, GentleCurveStillDetects)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // Coverage straights carry a little yaw correction. Below max_yaw_rate the
  // detector must stay armed, or a dig on a slightly curving swath is missed.
  EXPECT_EQ(RunDigging(cfg, st, 10.0, 0.003, 0.1, 0.15), mh::DigAction::kDig);
}

// ─────────────────────────────────────────────────────────────────────────────
// Regression: the dig recorded on the robot, 2026-08-24 09:48:23 UTC
// ─────────────────────────────────────────────────────────────────────────────

// Left encoder ran 889 ticks (3.17 m) while the right ran 50 (0.18 m) over
// 9 s; the chassis neither translated nor rotated (FTC's error vector was
// frozen at lat=0.551 lon=0.835 ang=-41.2 deg for the whole window, and the
// gyro read ~0). Mean wheel travel is therefore ~1.68 m of phantom advance.
// Nothing fired at the time. This pins that it now does.
TEST(DigDetector, FieldRecordedOneWheelSlipIsDetected)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // 1.68 m of claimed travel over 9 s => 0.0187 m per 0.1 s tick, chassis
  // static, gyro ~0 because only ONE wheel was spinning.
  constexpr double kWheelStep = 0.0187;
  constexpr double kGyro = 0.01;  // chassis not rotating
  constexpr double kSigma = 0.09;  // measured baseline major axis, RTK-Fixed

  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < 9.0; t += 0.1)
  {
    // Chassis frozen: the map position never changes.
    last = mh::DigDecide(cfg, st, 0.2, kWheelStep, 0.0, 0.0, kSigma, kGyro, 0.1).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  EXPECT_EQ(last, mh::DigAction::kDig) << "the 2026-08-24 one-wheel dig must trip the detector";
}

// The same event, gated by the OLD 0.10 m sigma threshold, did not fire —
// the measured baseline major axis (0.05-0.14 m) straddles it. Pins why the
// threshold moved.
TEST(DigDetector, OldSigmaThresholdWouldHaveMissedTheFieldDig)
{
  mh::DigDetectorCfg cfg;
  cfg.max_pos_sigma = 0.10;  // the pre-fix value
  mh::DigDetectorState st;

  EXPECT_EQ(RunDigging(cfg, st, 9.0, 0.14, 0.1, 0.01), mh::DigAction::kNone)
      << "documents the miss the new threshold fixes";
}

// ─────────────────────────────────────────────────────────────────────────────
// Regression: detection LATENCY, from the 2026-08-24 mow3 log (issue #500)
//
// 33 episodes of sustained one-wheel spin ground 69.7 m of tyre across the
// lawn; only 4 latched, at a median 9.6 s after the anomaly began. The two
// causes were how each side of the comparison was MEASURED — both fixed here.
// ─────────────────────────────────────────────────────────────────────────────

// Cause 1: the wheel side used the chassis-CENTRE distance. These digs are
// asymmetric (issue #499: a commanded turn radius below the half-track drives
// the inner wheel backwards), so the centre measure cancels most of the spin.
// Numbers from the episode at t=1787593684.02: the worst wheel ground
// 0.192 m/s of tyre while the centre distance accrued only 0.049 m/s — too
// slow to reach min_wheel_dist inside one 1.2 s window, so that episode (and
// 12 more like it) could never latch no matter what the other gates did.
TEST(DigDetector, WorstWheelClearsTheFloorWhereCentreDistanceCannot)
{
  mh::DigDetectorCfg cfg;

  constexpr double kDt = 0.1;
  constexpr double kCentreStep = 0.0049;  // 0.049 m/s — what travelled() saw
  constexpr double kWorstWheelStep = 0.0192;  // 0.192 m/s — what actually grinds

  // The centre measure never accumulates min_wheel_dist inside a window.
  EXPECT_LT(kCentreStep * (cfg.window_s / kDt), cfg.min_wheel_dist)
      << "documents why 13 of the 33 field episodes were arithmetically undetectable";

  mh::DigDetectorState st_centre;
  mh::DigAction last = mh::DigAction::kNone;
  for (double t = 0.0; t < 10.0; t += kDt)
  {
    last = mh::DigDecide(cfg, st_centre, 0.2, kCentreStep, 0.0, 0.0, 0.014, 0.01, kDt).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  EXPECT_EQ(last, mh::DigAction::kNone) << "the old centre-distance measure could not see it";

  // The worst wheel does, so the same episode now latches.
  mh::DigDetectorState st_worst;
  last = mh::DigAction::kNone;
  for (double t = 0.0; t < 10.0; t += kDt)
  {
    last = mh::DigDecide(cfg, st_worst, 0.2, kWorstWheelStep, 0.0, 0.0, 0.014, 0.01, kDt).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  EXPECT_EQ(last, mh::DigAction::kDig);
}

// Cause 2: the map side summed per-tick |steps|, which is a PATH LENGTH — it
// grows with estimator wander and with the monitor rate, so a parked chassis
// whose fused pose merely jitters looked like it was making progress. Field
// numbers: at the five latches the summed map travel over 1.2 s ranged
// 0.00-0.07 m against a ~0.09 m threshold, while the robot's true motion was
// 0.13 m in 18 s. The statistic swung across its whole budget on a robot that
// was parked, so detection landed only in the quiet windows.
TEST(DigDetector, PoseWanderNoLongerDelaysDetection)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  constexpr double kDt = 0.1;
  constexpr double kWheelStep = 0.0225;  // 0.27 m per window, as logged at latch
  constexpr double kWander = 0.008;  // per-tick jitter of the fused pose [m]

  // Summed, that jitter alone exceeds the ratio budget for the whole window —
  // which is exactly what kept the old code from latching.
  EXPECT_GT(kWander * (cfg.window_s / kDt),
            cfg.progress_fraction * kWheelStep * (cfg.window_s / kDt))
      << "documents the statistic that blocked detection";

  // The chassis is parked; the estimate wanders about a fixed mean.
  double elapsed = 0.0;
  mh::DigAction last = mh::DigAction::kNone;
  for (int i = 0; i < 200; ++i)
  {
    const double map_x = (i % 2 == 0) ? kWander : 0.0;
    elapsed += kDt;
    last = mh::DigDecide(cfg, st, 0.2, kWheelStep, map_x, 0.0, 0.014, 0.01, kDt).action;
    if (last == mh::DigAction::kDig)
    {
      break;
    }
  }
  ASSERT_EQ(last, mh::DigAction::kDig);
  EXPECT_LE(elapsed, cfg.window_s + kDt + 1e-9)
      << "must latch on the FIRST full window, not after wander happens to go quiet";
}

// The two fixes must not buy latency at the cost of a false fire: a healthy
// robot really translating is unaffected, because net displacement and summed
// steps agree on a near-straight track.
TEST(DigDetector, HealthyTrackingIsUnaffectedByTheNetDisplacementMeasure)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;

  // 0.2 m/s along a gently curving swath, worst wheel slightly faster than
  // the centre (outer wheel), map keeping up. Must stay silent indefinitely.
  double map_x = 0.0;
  mh::DigAction last = mh::DigAction::kNone;
  for (int i = 0; i < 600; ++i)
  {
    map_x += 0.020;
    last = mh::DigDecide(cfg, st, 0.2, 0.0232, map_x, 0.0, 0.014, 0.15, 0.1).action;
    ASSERT_EQ(last, mh::DigAction::kNone) << "false fire at tick " << i;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Trust-signal selection (DigTrustSigma)
// ─────────────────────────────────────────────────────────────────────────────

TEST(DigTrustSigma, RtkFixedWithValidAccuracyIsTrusted)
{
  EXPECT_NEAR(mh::DigTrustSigma(true, true, true, 0.014), 0.014, 1e-12);
}

TEST(DigTrustSigma, AnythingMissingSuppresses)
{
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(mh::DigTrustSigma(false, true, true, 0.014), inf) << "stale /gps/status";
  EXPECT_EQ(mh::DigTrustSigma(true, false, true, 0.014), inf) << "not RTK-Fixed";
  EXPECT_EQ(mh::DigTrustSigma(true, true, false, 0.014), inf) << "no accuracy in message";
  EXPECT_EQ(mh::DigTrustSigma(true, true, true, -1.0), inf) << "negative accuracy";
}

// RTK-Float reports decimetres to metres; the detector must stand down there.
TEST(DigTrustSigma, FloatAccuracyExceedsTheGate)
{
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState st;
  const double float_acc = mh::DigTrustSigma(true, true, true, 0.45);
  EXPECT_GT(float_acc, cfg.max_pos_sigma);
  EXPECT_EQ(RunDigging(cfg, st, 9.0, float_acc, 0.1, 0.0), mh::DigAction::kNone);
}

// The measurement that moved the gate off the graph marginal. Sampled over 90 s
// of live RTK-Fixed mowing on 2026-08-24, fusion_graph's published position
// sigma had p50 0.574 m while the receiver reported 0.014 m and FTC tracked to
// 8-16 mm. Feeding the graph value blocks; feeding the receiver value detects.
TEST(DigTrustSigma, GraphMarginalWouldBlockWhereReceiverAccuracyDoesNot)
{
  mh::DigDetectorCfg cfg;

  mh::DigDetectorState st_graph;
  EXPECT_EQ(RunDigging(cfg, st_graph, 9.0, 0.574, 0.1, 0.01), mh::DigAction::kNone)
      << "graph marginal p50 — this is why the gate moved off it";

  mh::DigDetectorState st_gnss;
  EXPECT_EQ(RunDigging(cfg, st_gnss, 9.0, 0.014, 0.1, 0.01), mh::DigAction::kDig)
      << "receiver accuracy at the same instant";
}

TEST(DigGnssFreshness, GenuineRtkObservationPermitsExistingTrustPolicy)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(1, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  const bool fresh = tracker.ObservationIsFresh(10 * kSecond, 1 * kSecond, 2 * kSecond);
  EXPECT_NEAR(mh::DigTrustSigma(fresh, true, true, 0.014), 0.014, 1e-12);
}

TEST(DigGnssFreshness, CachedRtkObservationCannotExtendTrust)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(2, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(2, 10 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  EXPECT_EQ(mh::DigTrustSigma(fresh, true, true, 0.014), std::numeric_limits<double>::infinity());
}

TEST(DigGnssFreshness, StaleCachedFixedCannotCreateDigAnchor)
{
  freshness::PhysicalObservationTracker tracker;
  mh::DigDetectorCfg cfg;
  mh::DigDetectorState state;
  ASSERT_EQ(tracker.Observe(3, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(3, 10 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  const double sigma = mh::DigTrustSigma(fresh, true, true, 0.014);

  EXPECT_EQ(mh::DigDecide(cfg, state, 0.3, 0.03, 1.0, 2.0, sigma, 0.0, 0.1).action,
            mh::DigAction::kNone);
  EXPECT_FALSE(state.have_anchor);
}

TEST(DigGnssFreshness, NewGenuineObservationRestoresTrust)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(4, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_FALSE(tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond));
  ASSERT_EQ(tracker.Observe(5, 13 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  EXPECT_NEAR(mh::DigTrustSigma(fresh, true, true, 0.014), 0.014, 1e-12);
}

TEST(GnssLockFreshness, FreshFixedKeepsExistingQualityIndication)
{
  EXPECT_EQ(mh::GnssQualityForFirmware(true, 100U), 100U);
  EXPECT_EQ(mh::GnssQualityForFirmware(true, 70U), 70U);
}

TEST(GnssLockFreshness, CachedFixedCannotKeepIndicationFresh)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(6, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_EQ(tracker.Observe(6, 10 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kCachedPublication);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  EXPECT_EQ(mh::GnssQualityForFirmware(fresh, 100U), 0U);
}

TEST(GnssLockFreshness, TimeoutUsesExistingNoCurrentDataValue)
{
  EXPECT_EQ(mh::GnssQualityForFirmware(false, 100U), 0U);
}

TEST(GnssLockFreshness, NewGenuineObservationRestoresQuality)
{
  freshness::PhysicalObservationTracker tracker;
  ASSERT_EQ(tracker.Observe(7, 10 * kSecond, 10 * kSecond, 1 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  ASSERT_FALSE(tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond));
  ASSERT_EQ(tracker.Observe(8, 13 * kSecond, 13 * kSecond, 4 * kSecond),
            freshness::ObservationUpdate::kNewObservation);
  const bool fresh = tracker.ObservationIsFresh(13 * kSecond, 4 * kSecond, 2 * kSecond);
  EXPECT_EQ(mh::GnssQualityForFirmware(fresh, 100U), 100U);
}
