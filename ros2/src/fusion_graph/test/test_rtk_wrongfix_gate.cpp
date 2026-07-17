// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the RTK wrong-fix motion-consistency gate. Pure logic, no
// ROS/GTSAM. See rtk_wrongfix_gate.hpp for the mechanism this guards against
// (the reverted GnssMobileGate reject-forever regression, CLAUDE.md "What
// NOT to Do").

#include "fusion_graph/rtk_wrongfix_gate.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

// ── GpsJumpImplausible ──────────────────────────────────────────────────
TEST(GpsJumpImplausible, WithinSlackBudgetWhileStationaryAccepts)
{
  // Stationary (no wheel travel, no rotation): jump must fit under the
  // fixed slack alone.
  EXPECT_FALSE(fg::GpsJumpImplausible(/*jump=*/0.02,
                                      /*max_jump=*/0.05,
                                      /*lever_arm_radius=*/0.20,
                                      /*abs_dtheta=*/0.0,
                                      /*wheel_dist=*/0.0));
}

TEST(GpsJumpImplausible, ExceedsSlackBudgetWhileStationaryRejects)
{
  EXPECT_TRUE(fg::GpsJumpImplausible(0.20, 0.05, 0.20, 0.0, 0.0));
}

TEST(GpsJumpImplausible, ExactlyAtBudgetIsNotImplausible)
{
  // budget = max_jump + lever*dtheta + wheel_dist = 0.05 + 0 + 0.10 = 0.15.
  // jump == budget must NOT reject (strict > only).
  EXPECT_FALSE(fg::GpsJumpImplausible(0.15, 0.05, 0.20, 0.0, 0.10));
}

TEST(GpsJumpImplausible, JustOverBudgetRejects)
{
  EXPECT_TRUE(fg::GpsJumpImplausible(0.150001, 0.05, 0.20, 0.0, 0.10));
}

TEST(GpsJumpImplausible, WheelTravelExpandsBudget)
{
  // A jump too big to explain while stationary becomes explainable once
  // wheel_dist accounts for it.
  EXPECT_TRUE(fg::GpsJumpImplausible(0.30, 0.05, 0.20, 0.0, 0.0));
  EXPECT_FALSE(fg::GpsJumpImplausible(0.30, 0.05, 0.20, 0.0, 0.30));
}

TEST(GpsJumpImplausible, InPlaceRotationSweepExpandsBudgetViaLeverArm)
{
  // A stationary pivot sweeps the GPS antenna by lever_arm * |dtheta| even
  // though wheel_dist stays ~0. Without this term the gate would reject a
  // legitimate fix during an in-place turn.
  const double lever_arm = 0.20;
  const double dtheta = M_PI / 2.0;  // 90 deg pivot.
  const double sweep = lever_arm * dtheta;
  EXPECT_TRUE(fg::GpsJumpImplausible(0.05 + sweep + 0.01, 0.05, lever_arm, 0.0, 0.0));
  EXPECT_FALSE(fg::GpsJumpImplausible(0.05 + sweep - 0.01, 0.05, lever_arm, dtheta, 0.0));
}

TEST(GpsJumpImplausible, ZeroMaxJumpStillAllowsExplainedMotion)
{
  // max_jump_m=0: gate degenerates to "jump must be <= actual travel".
  EXPECT_FALSE(fg::GpsJumpImplausible(0.10, 0.0, 0.20, 0.0, 0.10));
  EXPECT_TRUE(fg::GpsJumpImplausible(0.10 + 1e-6, 0.0, 0.20, 0.0, 0.10));
}

// ── ResetRtkWrongFixAccumulators ────────────────────────────────────────
TEST(ResetRtkWrongFixAccumulators, ZeroesBothAccumulators)
{
  double wheel_dist = 1.23;
  double abs_dtheta = 4.56;
  fg::ResetRtkWrongFixAccumulators(wheel_dist, abs_dtheta);
  EXPECT_DOUBLE_EQ(wheel_dist, 0.0);
  EXPECT_DOUBLE_EQ(abs_dtheta, 0.0);
}

// ── Bounded-vs-runaway regression (the GnssMobileGate incident) ─────────
//
// The reverted GnssMobileGate compared each fix against "motion since the
// last ACCEPTED fix" and only reset its accumulator on accept. Once it
// started rejecting, the accumulator never cleared: motion kept piling up
// against a stale reference, the expected-motion budget ran away (20.9 m
// observed for <1 m of real travel), and every subsequent fix rejected
// forever. These tests drive the pure gate through the same scenario —
// a run of consecutive wrong-fixes with only small real travel between
// each — and assert the budget stays bounded to one inter-fix interval
// instead of accumulating, because the reset is unconditional (see
// rtk_wrongfix_gate.hpp's ResetRtkWrongFixAccumulators contract: call it on
// EVERY fix, accept or reject).

// Mirrors the node's OnGnss loop (fusion_graph_node_callbacks_a.cpp): reset
// unconditionally on every fix, exactly like the production call site does.
TEST(RtkWrongFixGate, RepeatedRejectsStayBoundedWhenAccumulatorResetEveryFix)
{
  const double max_jump_m = 0.05;
  const double lever_arm_radius_m = 0.20;
  const double wrong_fix_jump_m = 0.20;  // a implausible jump, every fix.
  const double per_interval_wheel_travel_m = 0.01;  // <1 m real motion, matching the incident.

  double wheel_dist_since_last_gps_m = 0.0;
  double abs_dtheta_since_last_gps_rad = 0.0;

  for (int fix = 0; fix < 50; ++fix)
  {
    // Chassis inches forward a little between fixes, same as the incident.
    wheel_dist_since_last_gps_m += per_interval_wheel_travel_m;

    const bool rejected = fg::GpsJumpImplausible(wrong_fix_jump_m,
                                                 max_jump_m,
                                                 lever_arm_radius_m,
                                                 abs_dtheta_since_last_gps_rad,
                                                 wheel_dist_since_last_gps_m);
    // The wrong-fix jump (0.20 m) always exceeds the per-interval budget
    // (0.05 + 0.01 = 0.06 m) — every fix in this run is correctly rejected.
    EXPECT_TRUE(rejected) << "fix #" << fix;

    // Production call site resets on BOTH accept and reject paths.
    fg::ResetRtkWrongFixAccumulators(wheel_dist_since_last_gps_m, abs_dtheta_since_last_gps_rad);

    // The regression signature: the accumulator must stay pinned to one
    // interval's worth of travel, never grow across the reject streak.
    EXPECT_DOUBLE_EQ(wheel_dist_since_last_gps_m, 0.0);
    EXPECT_DOUBLE_EQ(abs_dtheta_since_last_gps_rad, 0.0);
  }
}

// Reproduces the reverted GnssMobileGate's reset discipline for contrast:
// reset ONLY on accept, decoupled from GpsJumpImplausible's verdict so the
// comparison isolates the accumulator arithmetic itself (which is the part
// CLAUDE.md's incident writeup blames — "expected_motion... only reset on
// acceptance"). Runs both disciplines over the same reject-every-fix
// scenario and shows the accept-only variant's accumulator grows linearly
// with the number of intervals instead of staying pinned to one interval.
TEST(RtkWrongFixGate, AccumulatorRunsAwayIfResetOnlyOnAccept)
{
  const double per_interval_wheel_travel_m = 0.01;
  const int kNumFixes = 50;
  const bool kEveryFixRejected = true;  // the incident scenario: gate never accepts again.

  double bounded_wheel_dist_m = 0.0;
  double runaway_wheel_dist_m = 0.0;

  for (int fix = 0; fix < kNumFixes; ++fix)
  {
    bounded_wheel_dist_m += per_interval_wheel_travel_m;
    runaway_wheel_dist_m += per_interval_wheel_travel_m;

    double unused_dtheta = 0.0;
    fg::ResetRtkWrongFixAccumulators(bounded_wheel_dist_m, unused_dtheta);  // unconditional (fixed gate).
    if (!kEveryFixRejected)
    {
      fg::ResetRtkWrongFixAccumulators(runaway_wheel_dist_m, unused_dtheta);  // accept-only (reverted gate).
    }
  }

  EXPECT_DOUBLE_EQ(bounded_wheel_dist_m, 0.0);
  EXPECT_DOUBLE_EQ(runaway_wheel_dist_m, per_interval_wheel_travel_m * kNumFixes);
  EXPECT_GT(runaway_wheel_dist_m, bounded_wheel_dist_m);
}
