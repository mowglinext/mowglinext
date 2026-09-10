// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the pure status -> pixel-buffer logic. No ROS, no hardware,
// no clock: every animation is a function of `now_s`, so exact phases are
// rendered directly.

#include <cstddef>
#include <limits>
#include <vector>

#include "mowgli_leds/led_pattern.hpp"
#include <gtest/gtest.h>

namespace mowgli_leds
{
namespace
{

LedPatternCfg MakeCfg(std::size_t led_count = 16u)
{
  LedPatternCfg cfg;
  cfg.led_count = led_count;
  return cfg;
}

/// A healthy, fresh, mid-mow robot. Individual tests perturb one field.
LedInputs MakeMowingInputs()
{
  LedInputs in;
  in.status_fresh = true;
  in.state = HighLevelState::kAutonomous;
  in.coverage_percent = 50.0f;
  in.battery_valid = true;
  in.battery_percent = 80.0f;
  in.rtk_fixed = true;
  in.now_s = 0.0;
  return in;
}

std::size_t CountLit(const std::vector<Rgb>& pixels)
{
  std::size_t lit = 0;
  for (const Rgb& pixel : pixels)
  {
    if (pixel != colors::kOff)
    {
      ++lit;
    }
  }
  return lit;
}

// ---------------------------------------------------------------------------
// Mode selection / priority
// ---------------------------------------------------------------------------

TEST(LedPatternMode, EmergencyOutranksChargingAndEveryActivity)
{
  LedInputs in = MakeMowingInputs();
  in.emergency = true;
  in.is_charging = true;
  in.battery_percent = 5.0f;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kEmergency);
}

TEST(LedPatternMode, EmergencyFlagIsIgnoredOnceTheStatusIsStale)
{
  // A latched flag from a dead behavior tree must not hold the ring red
  // forever -- the honest answer at that point is "I am not being told".
  LedInputs in = MakeMowingInputs();
  in.emergency = true;
  in.status_fresh = false;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kStale);
}

TEST(LedPatternMode, ChargingOutranksStaleSoADockedRobotStillReadsAsCharging)
{
  // is_charging can come from the hardware bridge's Power message, which is
  // alive even when the behavior tree is not.
  LedInputs in;
  in.status_fresh = false;
  in.is_charging = true;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kCharging);
}

TEST(LedPatternMode, StaleOutranksLowBatteryAndActivity)
{
  LedInputs in = MakeMowingInputs();
  in.status_fresh = false;
  in.battery_percent = 5.0f;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kStale);
}

TEST(LedPatternMode, LowBatteryOnlyFiresWithAValidBatteryReading)
{
  LedInputs in = MakeMowingInputs();
  in.battery_percent = 5.0f;

  in.battery_valid = true;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kLowBattery);

  // Same number, but flagged as "no reading" -- it must not blink red on a
  // default-constructed zero.
  in.battery_valid = false;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kMowing);
}

TEST(LedPatternMode, AutonomousWithRtkFixedSelectsMowing)
{
  LedInputs in = MakeMowingInputs();
  in.rtk_fixed = true;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kMowing);
}

TEST(LedPatternMode, AutonomousWithoutRtkFixedSelectsDegradedMowing)
{
  LedInputs in = MakeMowingInputs();
  in.rtk_fixed = false;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kMowingDegraded);
}

TEST(LedPatternMode, RecordingAndManualMowingMapToTheirOwnModes)
{
  LedInputs in = MakeMowingInputs();

  in.state = HighLevelState::kRecording;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kRecording);

  in.state = HighLevelState::kManualMowing;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kManual);
}

TEST(LedPatternMode, IdleIsTheFallbackForIdleAndNullStates)
{
  LedInputs in = MakeMowingInputs();

  in.state = HighLevelState::kIdle;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kIdle);

  in.state = HighLevelState::kNull;
  EXPECT_EQ(SelectMode(in, MakeCfg()), LedMode::kIdle);
}

// ---------------------------------------------------------------------------
// Progress arc arithmetic
// ---------------------------------------------------------------------------

TEST(LedPatternArc, AnyNonZeroProgressLightsAtLeastOnePixel)
{
  // 1 % of 16 floors to 0, which would render "started" as an unlit ring.
  EXPECT_EQ(FilledCount(1.0f, 16u), 1u);
  EXPECT_EQ(FilledCount(0.01f, 16u), 1u);
}

TEST(LedPatternArc, ProgressShortOfOneHundredLeavesOnePixelDark)
{
  // 99.9 % of 16 floors to 15 already, but 100 % must be the ONLY input that
  // fills the ring, so a full ring is unambiguous at a glance.
  EXPECT_EQ(FilledCount(99.9f, 16u), 15u);
  EXPECT_EQ(FilledCount(99.9999f, 16u), 15u);
}

TEST(LedPatternArc, OnlyFullProgressFillsTheWholeRing)
{
  EXPECT_EQ(FilledCount(100.0f, 16u), 16u);
  EXPECT_EQ(FilledCount(150.0f, 16u), 16u);
}

TEST(LedPatternArc, ZeroProgressAndEmptyRingsFillNothing)
{
  EXPECT_EQ(FilledCount(0.0f, 16u), 0u);
  EXPECT_EQ(FilledCount(-20.0f, 16u), 0u);
  EXPECT_EQ(FilledCount(50.0f, 0u), 0u);
  EXPECT_EQ(FilledCount(std::numeric_limits<float>::quiet_NaN(), 16u), 0u);
}

TEST(LedPatternArc, FillIsProportionalInTheMiddleOfTheRange)
{
  EXPECT_EQ(FilledCount(50.0f, 16u), 8u);
  EXPECT_EQ(FilledCount(25.0f, 16u), 4u);
  EXPECT_EQ(FilledCount(75.0f, 16u), 12u);
}

// ---------------------------------------------------------------------------
// Animation helpers
// ---------------------------------------------------------------------------

TEST(LedPatternAnimation, BreatheStaysInBoundsAndPeaksAtHalfPeriod)
{
  EXPECT_NEAR(Breathe(0.0, 2.0, 0.25f, 1.0f), 0.25f, 1e-5f);
  EXPECT_NEAR(Breathe(1.0, 2.0, 0.25f, 1.0f), 1.0f, 1e-5f);
  EXPECT_NEAR(Breathe(2.0, 2.0, 0.25f, 1.0f), 0.25f, 1e-5f);

  for (int step = 0; step <= 40; ++step)
  {
    const float value = Breathe(step * 0.05, 2.0, 0.25f, 1.0f);
    EXPECT_GE(value, 0.25f - 1e-5f);
    EXPECT_LE(value, 1.0f + 1e-5f);
  }
}

TEST(LedPatternAnimation, BlinkIsOnForTheFirstHalfOfEachPeriod)
{
  EXPECT_TRUE(BlinkOn(0.0, 1.0));
  EXPECT_TRUE(BlinkOn(0.49, 1.0));
  EXPECT_FALSE(BlinkOn(0.51, 1.0));
  EXPECT_TRUE(BlinkOn(1.01, 1.0));
}

TEST(LedPatternAnimation, RotationAdvancesOnceAroundPerPeriod)
{
  EXPECT_EQ(RotationIndex(0.0, 2.0, 16u), 0u);
  EXPECT_EQ(RotationIndex(1.0, 2.0, 16u), 8u);
  EXPECT_EQ(RotationIndex(1.9999, 2.0, 16u), 15u);
  EXPECT_EQ(RotationIndex(2.0, 2.0, 16u), 0u);
}

TEST(LedPatternAnimation, HelpersSurviveZeroPeriodsAndEmptyRings)
{
  EXPECT_EQ(RotationIndex(1.0, 0.0, 16u), 0u);
  EXPECT_EQ(RotationIndex(1.0, 2.0, 0u), 0u);
  EXPECT_TRUE(BlinkOn(1.0, 0.0));
  EXPECT_NEAR(Breathe(1.0, 0.0, 0.25f, 1.0f), 1.0f, 1e-5f);
}

TEST(LedPatternAnimation, NegativeTimestampsDoNotInvertTheAnimations)
{
  // std::fmod keeps the numerator's sign; PositivePhase must undo that or a
  // pre-epoch timestamp would drive the ring backwards.
  EXPECT_NEAR(PositivePhase(-0.25, 1.0), 0.75, 1e-9);
  EXPECT_LT(RotationIndex(-0.25, 1.0, 16u), 16u);
  EXPECT_GE(Breathe(-0.5, 2.0, 0.25f, 1.0f), 0.25f);
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

TEST(LedPatternRender, AlwaysReturnsExactlyLedCountPixels)
{
  for (const std::size_t count : {1u, 8u, 16u, 24u, 60u})
  {
    const auto pixels = RenderFrame(MakeMowingInputs(), MakeCfg(count));
    EXPECT_EQ(pixels.size(), count);
  }
}

TEST(LedPatternRender, HandlesAZeroLengthRingWithoutTouchingAnything)
{
  const auto pixels = RenderFrame(MakeMowingInputs(), MakeCfg(0u));
  EXPECT_TRUE(pixels.empty());
}

TEST(LedPatternRender, EmergencyIsEveryPixelSolidRed)
{
  LedInputs in = MakeMowingInputs();
  in.emergency = true;

  const auto pixels = RenderFrame(in, MakeCfg());
  ASSERT_EQ(pixels.size(), 16u);
  for (const Rgb& pixel : pixels)
  {
    EXPECT_EQ(pixel, colors::kRed);
  }

  // ...and it must be STATIC: the low-battery blink is the only other red, and
  // the two are told apart by motion.
  in.now_s = 0.7;
  EXPECT_EQ(RenderFrame(in, MakeCfg()), pixels);
}

TEST(LedPatternRender, MowingArcIsGreenWithAWhiteHeadPixelAtTheTip)
{
  LedInputs in = MakeMowingInputs();
  in.coverage_percent = 50.0f;

  const auto pixels = RenderFrame(in, MakeCfg());
  ASSERT_EQ(pixels.size(), 16u);
  for (std::size_t i = 0; i < 7u; ++i)
  {
    EXPECT_EQ(pixels[i], colors::kGreen) << "arc body pixel " << i;
  }
  EXPECT_EQ(pixels[7], colors::kWhite) << "head pixel";
  for (std::size_t i = 8; i < 16u; ++i)
  {
    EXPECT_EQ(pixels[i], colors::kOff) << "unfilled pixel " << i;
  }
}

TEST(LedPatternRender, DegradedMowingArcIsAmberAndItsHeadBlinks)
{
  LedInputs in = MakeMowingInputs();
  in.rtk_fixed = false;
  in.coverage_percent = 50.0f;

  in.now_s = 0.0;  // blink on
  const auto lit = RenderFrame(in, MakeCfg());
  EXPECT_EQ(lit[0], colors::kAmber);
  EXPECT_EQ(lit[7], colors::kWhite);

  in.now_s = 0.3;  // blink off (2 Hz -> 0.5 s period)
  const auto dark = RenderFrame(in, MakeCfg());
  EXPECT_EQ(dark[0], colors::kAmber);
  EXPECT_EQ(dark[7], colors::kAmber);

  // The whole point is that the two frames differ, so the node's change
  // detector actually pushes the animation to the wire.
  EXPECT_NE(lit, dark);
}

TEST(LedPatternRender, ChargingFillsProportionallyToBatteryAndBreathes)
{
  LedInputs in;
  in.status_fresh = true;
  in.state = HighLevelState::kIdle;
  in.is_charging = true;
  in.battery_valid = true;
  in.battery_percent = 50.0f;

  in.now_s = 1.5;  // breathe peak of the 3 s period
  const auto peak = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(peak), 8u);
  EXPECT_EQ(peak[0], colors::kGreen);
  EXPECT_EQ(peak[8], colors::kOff);

  in.now_s = 0.0;  // breathe trough
  const auto trough = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(trough), 8u);
  EXPECT_LT(trough[0].g, peak[0].g);
  EXPECT_GT(trough[0].g, 0u);
}

TEST(LedPatternRender, ChargingWithoutABatteryReadingShowsACometNotAnEmptyRing)
{
  // The Power-message fallback knows the charger is on but not the level.
  // Rendering FilledCount(0) there would look identical to "off".
  LedInputs in;
  in.status_fresh = false;
  in.is_charging = true;
  in.battery_valid = false;

  const auto pixels = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(pixels), 3u) << "comet head plus two tail pixels";
}

TEST(LedPatternRender, AFullBatteryStopsBreathingAndFillsTheRing)
{
  LedInputs in;
  in.status_fresh = true;
  in.state = HighLevelState::kIdle;
  in.is_charging = true;
  in.battery_valid = true;
  in.battery_percent = 100.0f;

  in.now_s = 0.0;
  const auto trough = RenderFrame(in, MakeCfg());
  in.now_s = 1.5;
  const auto peak = RenderFrame(in, MakeCfg());

  EXPECT_EQ(trough, peak) << "a finished charge must be steady, not breathing";
  for (const Rgb& pixel : peak)
  {
    EXPECT_EQ(pixel, colors::kGreen);
  }
}

TEST(LedPatternRender, LowBatteryBlinksTheWholeRingBetweenRedAndOff)
{
  LedInputs in = MakeMowingInputs();
  in.battery_percent = 5.0f;

  in.now_s = 0.0;
  const auto on = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(on), 16u);
  EXPECT_EQ(on[0], colors::kRed);

  in.now_s = 0.6;
  const auto off = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(off), 0u);
}

TEST(LedPatternRender, StaleShowsAnAmberCometOnAnOtherwiseDarkRing)
{
  LedInputs in;
  in.status_fresh = false;
  in.now_s = 0.0;

  const auto pixels = RenderFrame(in, MakeCfg());
  EXPECT_EQ(CountLit(pixels), 3u);
  EXPECT_EQ(pixels[0], colors::kAmber) << "head at full brightness";
  EXPECT_LT(pixels[15].r, pixels[0].r) << "tail trails behind the head";
}

TEST(LedPatternRender, IdleRingIsDimButNotOff)
{
  LedInputs in;
  in.status_fresh = true;
  in.state = HighLevelState::kIdle;

  const auto pixels = RenderFrame(in, MakeCfg());
  ASSERT_EQ(pixels.size(), 16u);
  for (const Rgb& pixel : pixels)
  {
    EXPECT_GT(pixel.r, 0u);
    EXPECT_LT(pixel.r, 128u) << "idle must be clearly dimmer than any active mode";
  }

  // Static: it costs one SPI write per keepalive interval and nothing more.
  in.now_s = 3.7;
  EXPECT_EQ(RenderFrame(in, MakeCfg()), pixels);
}

TEST(LedPatternRender, ManualMowingBreathesPurpleAcrossTheWholeRing)
{
  LedInputs in = MakeMowingInputs();
  in.state = HighLevelState::kManualMowing;

  in.now_s = 1.0;  // breathe peak of the 2 s period
  const auto peak = RenderFrame(in, MakeCfg());
  in.now_s = 0.0;  // trough
  const auto trough = RenderFrame(in, MakeCfg());

  EXPECT_EQ(CountLit(peak), 16u);
  EXPECT_EQ(peak[0], colors::kPurple);
  EXPECT_LT(trough[0].b, peak[0].b);
  EXPECT_GT(trough[0].b, 0u);
}

TEST(LedPatternRender, RecordingCometMovesAroundTheRingOverItsPeriod)
{
  LedInputs in = MakeMowingInputs();
  in.state = HighLevelState::kRecording;

  in.now_s = 0.0;
  const auto start = RenderFrame(in, MakeCfg());
  in.now_s = 1.0;  // half of the 2 s revolution
  const auto half = RenderFrame(in, MakeCfg());

  EXPECT_EQ(start[0], colors::kCyan);
  EXPECT_EQ(half[8], colors::kCyan);
  EXPECT_NE(start, half);
}

}  // namespace
}  // namespace mowgli_leds
