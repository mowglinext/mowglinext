// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the loop-closure rate/travel gate + GPS σ floor. Pure
// logic, no ROS/GTSAM. See loop_closure_gate.hpp for the field evidence
// (issue #513: 13.7 accepted LC/s under RTK-Float, 3816 in 286 s of mowing) and for the
// reset-on-accept-only polarity argument these tests pin down.

#include <algorithm>
#include <cmath>
#include <limits>

#include "fusion_graph/loop_closure_gate.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

// Production defaults (fusion_graph.yaml / fusion_graph_node_setup_params.cpp).
constexpr double kMinTravelM = 1.0;
constexpr double kMinIntervalS = 2.0;
constexpr double kLcSigmaXy = 0.05;

// ── LoopClosureRateAllows ───────────────────────────────────────────────
TEST(LoopClosureRateAllows, BothThresholdsMetAllows)
{
  EXPECT_TRUE(fg::LoopClosureRateAllows(1.5, 3.0, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, FreshAfterAcceptBlocks)
{
  // Right after an accept both accumulators are 0: nothing may fire.
  EXPECT_FALSE(fg::LoopClosureRateAllows(0.0, 0.0, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, EnoughTimeButNotEnoughTravelBlocks)
{
  // Stationary robot: the clock alone must not open the gate (a parked robot
  // among 30 s-old nodes must not loop-close every 2 s).
  EXPECT_FALSE(fg::LoopClosureRateAllows(0.2, 10.0, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, EnoughTravelButNotEnoughTimeBlocks)
{
  EXPECT_FALSE(fg::LoopClosureRateAllows(2.0, 0.5, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, ExactlyAtThresholdsAllows)
{
  // >= on both halves: reaching the threshold opens the gate.
  EXPECT_TRUE(fg::LoopClosureRateAllows(kMinTravelM, kMinIntervalS, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, JustBelowEitherThresholdBlocks)
{
  EXPECT_FALSE(
      fg::LoopClosureRateAllows(kMinTravelM - 1e-9, kMinIntervalS, kMinTravelM, kMinIntervalS));
  EXPECT_FALSE(
      fg::LoopClosureRateAllows(kMinTravelM, kMinIntervalS - 1e-9, kMinTravelM, kMinIntervalS));
}

TEST(LoopClosureRateAllows, ZeroTravelThresholdDisablesTravelHalf)
{
  EXPECT_TRUE(fg::LoopClosureRateAllows(0.0, 3.0, 0.0, kMinIntervalS));
  EXPECT_FALSE(fg::LoopClosureRateAllows(0.0, 1.0, 0.0, kMinIntervalS));
}

TEST(LoopClosureRateAllows, ZeroIntervalThresholdDisablesTimeHalf)
{
  EXPECT_TRUE(fg::LoopClosureRateAllows(1.5, 0.0, kMinTravelM, 0.0));
  EXPECT_FALSE(fg::LoopClosureRateAllows(0.5, 0.0, kMinTravelM, 0.0));
}

TEST(LoopClosureRateAllows, NegativeThresholdsDisableBothHalves)
{
  // Both halves off: every node may loop-close (pre-#513 behaviour).
  EXPECT_TRUE(fg::LoopClosureRateAllows(0.0, 0.0, -1.0, -1.0));
}

// ── Reset-on-ACCEPT-only polarity (the inverse of the GnssMobileGate) ───
//
// Replays the issue #513 scenario: 25 Hz nodes, robot mowing at 0.3 m/s, an
// ICP candidate that would pass lc_max_rmse on EVERY node. With the
// accumulators reset only on accept, the gate must open periodically (one
// LC per 2 s / 1 m) instead of never — and must never let two accepts
// through inside one interval.
TEST(LoopClosureGateState, ResetOnAcceptOnlyYieldsOneAcceptPerInterval)
{
  constexpr double kNodePeriodS = 0.04;
  constexpr double kSpeedMps = 0.3;
  constexpr double kDurationS = 286.0;  // mow #1's MOWING phase (Float).
  const int num_nodes = static_cast<int>(kDurationS / kNodePeriodS);

  fg::LoopClosureGateState st;
  int accepts = 0;
  double last_accept_t = -std::numeric_limits<double>::infinity();
  for (int n = 0; n < num_nodes; ++n)
  {
    const double t = n * kNodePeriodS;
    st.travel_since_accept_m += kSpeedMps * kNodePeriodS;
    st.time_since_accept_s += kNodePeriodS;
    if (!fg::LoopClosureRateAllows(
            st.travel_since_accept_m, st.time_since_accept_s, kMinTravelM, kMinIntervalS))
    {
      continue;  // rejected by the gate: accumulators deliberately untouched.
    }
    // ICP "accepts" every time it is allowed to run.
    ++accepts;
    EXPECT_GE(t - last_accept_t, kMinIntervalS - 1e-9) << "two accepts inside one interval";
    last_accept_t = t;
    st = fg::LoopClosureGateState{};  // reset on accept only.
  }
  // At 0.3 m/s the 1 m travel half dominates (3.33 s per accept), so the
  // 286 s window yields ~86 accepts — vs 3816 measured without the gate.
  EXPECT_GT(accepts, 0) << "gate never opened: reset polarity is wrong";
  EXPECT_LE(accepts, static_cast<int>(kDurationS / (kMinTravelM / kSpeedMps)) + 1);
  EXPECT_LT(accepts, 3816 / 10);
}

TEST(LoopClosureGateState, ResettingOnRejectWouldLockTheGateForever)
{
  // Contrast case: the WRONG polarity. Zeroing the accumulators on every
  // gate rejection means they can never exceed one node's worth, so the gate
  // never opens — the mirror image of the GnssMobileGate lockout.
  constexpr double kNodePeriodS = 0.04;
  constexpr double kSpeedMps = 0.3;
  fg::LoopClosureGateState st;
  int accepts = 0;
  for (int n = 0; n < 10000; ++n)
  {
    st.travel_since_accept_m += kSpeedMps * kNodePeriodS;
    st.time_since_accept_s += kNodePeriodS;
    if (fg::LoopClosureRateAllows(
            st.travel_since_accept_m, st.time_since_accept_s, kMinTravelM, kMinIntervalS))
    {
      ++accepts;
    }
    st = fg::LoopClosureGateState{};  // reset unconditionally (wrong here).
  }
  EXPECT_EQ(accepts, 0);
}

TEST(LoopClosureGateState, DefaultConstructedIsZero)
{
  const fg::LoopClosureGateState st;
  EXPECT_DOUBLE_EQ(st.travel_since_accept_m, 0.0);
  EXPECT_DOUBLE_EQ(st.time_since_accept_s, 0.0);
}

// ── LoopClosureSigmaFloor ───────────────────────────────────────────────
TEST(LoopClosureSigmaFloor, FloatGpsSigmaRaisesLcSigma)
{
  // RTK-Float σ≈0.3 m: the 5 cm LC must be inflated to 0.3 m (ratio 1).
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.30, 1.0), 0.30);
}

TEST(LoopClosureSigmaFloor, FixedGpsSigmaBelowLcSigmaLeavesItUnchanged)
{
  // RTK-Fixed σ≈3 mm: the floor is below lc_sigma → max keeps lc_sigma.
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.003, 1.0), kLcSigmaXy);
}

TEST(LoopClosureSigmaFloor, RatioScalesTheFloor)
{
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.30, 0.5), 0.15);
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.30, 2.0), 0.60);
}

TEST(LoopClosureSigmaFloor, NoGpsEverReturnsLcSigmaUnchanged)
{
  // The node latches -1 for "no usable σ".
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, -1.0, 1.0), kLcSigmaXy);
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.0, 1.0), kLcSigmaXy);
}

TEST(LoopClosureSigmaFloor, NonFiniteGpsSigmaReturnsLcSigmaUnchanged)
{
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy,
                                             std::numeric_limits<double>::quiet_NaN(),
                                             1.0),
                   kLcSigmaXy);
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy,
                                             std::numeric_limits<double>::infinity(),
                                             1.0),
                   kLcSigmaXy);
}

TEST(LoopClosureSigmaFloor, NonPositiveRatioDisablesTheFloor)
{
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.30, 0.0), kLcSigmaXy);
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(kLcSigmaXy, 0.30, -1.0), kLcSigmaXy);
}

TEST(LoopClosureSigmaFloor, ComposesAfterRtkYieldMax)
{
  // The node applies the floor AFTER the RTK-yield max: whichever is larger
  // wins, so a yielded 0.5 m σ is never pulled back down by a 0.3 m GPS σ.
  const double yielded = std::max(kLcSigmaXy, 0.5);
  EXPECT_DOUBLE_EQ(fg::LoopClosureSigmaFloor(yielded, 0.30, 1.0), 0.5);
}
