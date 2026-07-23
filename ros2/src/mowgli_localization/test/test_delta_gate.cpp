// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// test_delta_gate.cpp
//
// Unit tests for the Δ (map→odom yaw) stabilisation gate (delta_gate.hpp) used
// by gps_dock_detection. Guards two behaviours:
//   1. jump-reject + EMA around a healthy baseline;
//   2. STALE-LATCH recovery (issue #390) — the gate must NOT hold a stale Δ
//      forever once it starts rejecting; a sustained MUTUALLY-consistent
//      rejection re-seeds Δ, while a flapping transient never does.

#include <cmath>

#include "mowgli_localization/delta_gate.hpp"
#include <gtest/gtest.h>

using mowgli_localization::DeltaGate;
using mowgli_localization::DeltaGateAction;
using mowgli_localization::DeltaGateConfig;
using mowgli_localization::wrap_to_pi;

namespace
{
constexpr double kRad = M_PI / 180.0;

DeltaGateConfig default_cfg()
{
  return DeltaGateConfig{25.0 * kRad, 0.15, 5.0};
}

double ang_diff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}
}  // namespace

// ── wrap_to_pi ──────────────────────────────────────────────────────────────

TEST(WrapToPi, WrapsIntoRange)
{
  EXPECT_NEAR(wrap_to_pi(M_PI + 0.1), -M_PI + 0.1, 1e-9);
  EXPECT_NEAR(wrap_to_pi(-M_PI - 0.1), M_PI - 0.1, 1e-9);
  EXPECT_NEAR(wrap_to_pi(0.3), 0.3, 1e-9);
}

// ── seed / accept / reject basics ───────────────────────────────────────────

TEST(DeltaGate, FirstSampleSeeds)
{
  DeltaGate g(default_cfg());
  EXPECT_FALSE(g.has_offset());
  EXPECT_EQ(g.update(-40.0 * kRad, 0.0), DeltaGateAction::SEEDED);
  EXPECT_TRUE(g.has_offset());
  EXPECT_NEAR(g.value(), -40.0 * kRad, 1e-9);
}

TEST(DeltaGate, SmallStepAcceptedAndEmaSmoothed)
{
  DeltaGate g(default_cfg());
  g.update(-40.0 * kRad, 0.0);  // seed
  // A 10° step is within the 25° band → accepted, moved by alpha*10° = 1.5°.
  EXPECT_EQ(g.update(-30.0 * kRad, 0.1), DeltaGateAction::ACCEPTED);
  EXPECT_NEAR(g.value(), -40.0 * kRad + 0.15 * 10.0 * kRad, 1e-9);
}

TEST(DeltaGate, LoneLargeJumpRejectedHoldingBaseline)
{
  DeltaGate g(default_cfg());
  g.update(-40.0 * kRad, 0.0);  // seed
  // A 115° corrupt-reload step is rejected; baseline held.
  EXPECT_EQ(g.update(-155.0 * kRad, 0.1), DeltaGateAction::REJECTED);
  EXPECT_NEAR(g.value(), -40.0 * kRad, 1e-9);
}

// ── issue #390: stale-latch recovery ────────────────────────────────────────

// A baseline slewed off truth (as the docked IMU/mag recal did) must NOT lock
// out recovery forever: a SUSTAINED, mutually-consistent rejection re-seeds Δ.
TEST(DeltaGate, SustainedConsistentRejectionReseeds)
{
  DeltaGate g(default_cfg());
  g.update(-156.0 * kRad, 0.0);  // stale baseline latched during recal

  const double truth = -41.0 * kRad;  // the correct offset, ~115° away
  DeltaGateAction last = DeltaGateAction::REJECTED;
  bool reseeded = false;
  // Feed the true offset at 10 Hz; the first several are rejected (>25° from the
  // stale baseline), then after reseed_after_s (5 s) the gate re-seeds to truth.
  for (int i = 1; i <= 60; ++i)
  {
    last = g.update(truth, i * 0.1);
    if (last == DeltaGateAction::RESEEDED)
    {
      reseeded = true;
      break;
    }
    EXPECT_EQ(last, DeltaGateAction::REJECTED) << "i=" << i;
  }
  ASSERT_TRUE(reseeded);
  EXPECT_NEAR(ang_diff(g.value(), truth), 0.0, 1e-9);
  // The re-seed must take at least reseed_after_s worth of samples, not fire early.
}

// The re-seed must not fire before the sustained window elapses.
TEST(DeltaGate, DoesNotReseedBeforeWindow)
{
  DeltaGate g(default_cfg());
  g.update(-156.0 * kRad, 0.0);
  const double truth = -41.0 * kRad;
  // Only 4 s of consistent rejection (< 5 s window) → still rejecting, held.
  for (int i = 1; i <= 40; ++i)
  {
    EXPECT_EQ(g.update(truth, i * 0.1), DeltaGateAction::REJECTED) << "i=" << i;
  }
  EXPECT_NEAR(g.value(), -156.0 * kRad, 1e-9);
}

// A FLAPPING transient (samples that keep jumping around) must never re-seed —
// each inconsistent sample restarts the candidate run, so a glitch can't latch.
TEST(DeltaGate, FlappingTransientNeverReseeds)
{
  DeltaGate g(default_cfg());
  g.update(-40.0 * kRad, 0.0);  // healthy baseline
  // Alternate two far-apart rejected values for well over the window.
  for (int i = 1; i <= 100; ++i)
  {
    const double v = (i % 2 == 0) ? (120.0 * kRad) : (-120.0 * kRad);
    EXPECT_EQ(g.update(v, i * 0.1), DeltaGateAction::REJECTED) << "i=" << i;
  }
  EXPECT_NEAR(g.value(), -40.0 * kRad, 1e-9);  // never abandoned the good baseline
}

// After a re-seed the gate resumes normal accept/EMA behaviour around the new Δ.
TEST(DeltaGate, ResumesTrackingAfterReseed)
{
  DeltaGate g(default_cfg());
  g.update(-156.0 * kRad, 0.0);
  const double truth = -41.0 * kRad;
  double t = 0.1;
  while (g.update(truth, t) != DeltaGateAction::RESEEDED)
  {
    t += 0.1;
    ASSERT_LT(t, 20.0);
  }
  // A small step near the new baseline is now accepted (not rejected).
  EXPECT_EQ(g.update(truth + 5.0 * kRad, t + 0.1), DeltaGateAction::ACCEPTED);
}

// An accepted sample cancels a partially-accumulated rejection run so a brief
// excursion that returns within band does not later trigger a spurious re-seed.
TEST(DeltaGate, AcceptedSampleClearsCandidateRun)
{
  DeltaGate g(default_cfg());
  g.update(-40.0 * kRad, 0.0);
  const double truth = -41.0 * kRad;
  g.update(-160.0 * kRad, 0.1);  // one rejected sample → starts a candidate run
  g.update(-160.0 * kRad, 0.2);  // consistent, but window not elapsed
  g.update(truth, 0.3);          // back within band → ACCEPTED, run cleared
  // Now a fresh far sample must be treated as a first rejection, not near-window.
  EXPECT_EQ(g.update(-160.0 * kRad, 0.4), DeltaGateAction::REJECTED);
  // ...and it should NOT immediately re-seed on the very next tick.
  EXPECT_EQ(g.update(-160.0 * kRad, 0.5), DeltaGateAction::REJECTED);
}

// reseed_after_s <= 0 disables recovery (legacy hold-forever behaviour).
TEST(DeltaGate, ReseedDisabledHoldsForever)
{
  DeltaGateConfig cfg = default_cfg();
  cfg.reseed_after_s = 0.0;
  DeltaGate g(cfg);
  g.update(-156.0 * kRad, 0.0);
  const double truth = -41.0 * kRad;
  for (int i = 1; i <= 200; ++i)
  {
    EXPECT_EQ(g.update(truth, i * 0.1), DeltaGateAction::REJECTED) << "i=" << i;
  }
  EXPECT_NEAR(g.value(), -156.0 * kRad, 1e-9);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
