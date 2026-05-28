// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for pulse_modulate_wz. Pure math, no ROS plumbing.

#include <cmath>

#include "mowgli_hardware/wz_pulse_modulator.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// Default minimum burst width used by most tests (matches the node default).
constexpr int kMinBurst = 4;

// Run a sustained command for `n` ticks and return the time-average output.
double average_over(double wz_cmd, double deadband, double& accum, int& burst, int n,
                    int min_burst = kMinBurst)
{
  double sum = 0.0;
  for (int i = 0; i < n; ++i)
  {
    sum += mh::pulse_modulate_wz(wz_cmd, deadband, accum, burst, min_burst);
  }
  return sum / static_cast<double>(n);
}

}  // namespace

// (a) Commands at or above the deadband pass through unchanged, and never
//     mutate the accumulator phase.
TEST(WzPulseModulator, AtOrAboveDeadbandPassesThrough)
{
  double accum = 0.0;
  int burst = 0;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.5, 0.5, accum, burst, kMinBurst), 0.5);
  EXPECT_DOUBLE_EQ(accum, 0.0);
  EXPECT_EQ(burst, 0);
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.8, 0.5, accum, burst, kMinBurst), 0.8);
  EXPECT_DOUBLE_EQ(accum, 0.0);
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(-1.2, 0.5, accum, burst, kMinBurst), -1.2);
  EXPECT_DOUBLE_EQ(accum, 0.0);
}

// (b) A sustained sub-deadband command averages to the commanded rate over
//     many ticks regardless of the minimum burst width (the "debt" model
//     preserves the average — it only batches the ON-ticks into runs).
TEST(WzPulseModulator, SubDeadbandAveragesToCommand)
{
  const double deadband = 0.5;
  const int n = 10000;
  double accum;
  int burst;

  accum = 0.0; burst = 0;  // 0.25/0.5 → duty 50%
  EXPECT_NEAR(average_over(0.25, deadband, accum, burst, n), 0.25, 2.0e-3);

  accum = 0.0; burst = 0;  // 0.30 → must NOT be the 0.38-0.49 over-rotation of the old clamp
  EXPECT_NEAR(average_over(0.30, deadband, accum, burst, n), 0.30, 2.0e-3);

  accum = 0.0; burst = 0;  // 0.10 → duty 20%, negative sign preserved
  EXPECT_NEAR(average_over(-0.10, deadband, accum, burst, n), -0.10, 2.0e-3);
}

// (c) THE FIX: every emitted pulse is a run of >= min_burst_ticks consecutive
//     ON-ticks. A single-tick pulse (the old behaviour) was too short to break
//     chassis stiction, leaving the robot frozen on sub-deadband commands.
TEST(WzPulseModulator, PulsesComeInMinWidthBursts)
{
  const double deadband = 0.5;
  const int min_burst = 4;
  double accum = 0.0;
  int burst = 0;

  int run = 0;          // current consecutive-ON run length
  int observed_runs = 0;
  for (int i = 0; i < 5000; ++i)
  {
    const double out = mh::pulse_modulate_wz(0.2, deadband, accum, burst, min_burst);
    EXPECT_TRUE(out == 0.0 || out == deadband) << "unexpected amplitude " << out;
    if (out != 0.0)
    {
      ++run;
    }
    else if (run > 0)
    {
      // A completed ON-run must be at least min_burst ticks long.
      EXPECT_GE(run, min_burst) << "burst too short: " << run << " ticks";
      ++observed_runs;
      run = 0;
    }
  }
  EXPECT_GT(observed_runs, 0) << "no pulses fired at all (robot would not rotate)";
}

// Every nonzero output of a sub-deadband command is the full deadband
// amplitude with the command's sign.
TEST(WzPulseModulator, PulsesAtDeadbandAmplitude)
{
  const double deadband = 0.5;
  double accum = 0.0;
  int burst = 0;
  for (int i = 0; i < 1000; ++i)
  {
    const double out = mh::pulse_modulate_wz(0.3, deadband, accum, burst, kMinBurst);
    EXPECT_TRUE(out == 0.0 || out == deadband) << "unexpected pulse amplitude " << out;
  }
}

// (d) Near-zero commands stay exactly zero and clear phase + burst.
TEST(WzPulseModulator, NearZeroStaysZero)
{
  double accum = 0.4;
  int burst = 2;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.0, 0.5, accum, burst, kMinBurst), 0.0);
  EXPECT_DOUBLE_EQ(accum, 0.0);
  EXPECT_EQ(burst, 0);

  accum = 0.4;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(5.0e-4, 0.5, accum, burst, kMinBurst), 0.0);
  EXPECT_DOUBLE_EQ(accum, 0.0);
}

// (e) A sign flip resets phase so a direction change does not carry stale
//     phase (or an in-progress burst) into the new direction.
TEST(WzPulseModulator, SignFlipResetsPhase)
{
  const double deadband = 0.5;
  double accum = 0.0;
  int burst = 0;

  // One sub-deadband tick: accrues duty (0.2/0.5=0.4) but < min_burst so no fire.
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.2, deadband, accum, burst, kMinBurst), 0.0);
  EXPECT_NEAR(accum, 0.4, 1.0e-9);

  // Flip sign: the stored +0.4 phase must be dropped before integrating the
  // new negative duty.
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(-0.2, deadband, accum, burst, kMinBurst), 0.0);
  EXPECT_NEAR(accum, -0.4, 1.0e-9);

  // Long-run average after the flip still tracks the new command.
  accum = 0.0; burst = 0;
  EXPECT_NEAR(average_over(-0.25, deadband, accum, burst, 10000), -0.25, 2.0e-3);
}

// Degenerate deadband (<= 0) must not divide-by-zero or pulse — pass through.
TEST(WzPulseModulator, NonPositiveDeadbandPassesThrough)
{
  double accum = 0.3;
  int burst = 0;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.1, 0.0, accum, burst, kMinBurst), 0.1);
  EXPECT_DOUBLE_EQ(accum, 0.0);
}
