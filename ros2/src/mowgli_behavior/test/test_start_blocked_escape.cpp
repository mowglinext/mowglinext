// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_start_blocked_escape.cpp
 * @brief Unit tests for the bounded start-pose escape motion (issue #487).
 *
 * SAFETY: this is the only new PHYSICAL MOTION in the #487 workstream, on a
 * machine with spinning blades. Everything these tests pin is a safety
 * property, not a nicety:
 *
 *   * the two field cases resolve to OPPOSITE directions from one signal
 *     (post-undock the robot reversed in, so it must drive forward; mid-mow it
 *     arrived forward, so it must reverse);
 *   * an unknown / stale / ~zero direction signal commands NOTHING rather than
 *     guessing;
 *   * the escape cannot fire without #495's confirmed start-blocked signal;
 *   * the escape cannot fire unless the blade is VERIFIED off;
 *   * the distance bound and the time bound each independently end it;
 *   * a bad config cannot exceed the compiled ceilings.
 */

#include <cmath>

#include "mowgli_behavior/start_blocked_escape.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::EscapeDecide;
using mowgli_behavior::EscapeDirection;
using mowgli_behavior::EscapeDirectionFromLastMotion;
using mowgli_behavior::EscapeDone;
using mowgli_behavior::EscapeMoves;
using mowgli_behavior::EscapePreconditions;
using mowgli_behavior::EscapeStep;
using mowgli_behavior::EscapeVerdict;
using mowgli_behavior::EscapeVerdictDirection;
using mowgli_behavior::kEscapeMaxDistance;
using mowgli_behavior::kEscapeMaxSpeed;
using mowgli_behavior::kEscapeMaxTimeout;
using mowgli_behavior::LastMotionSignal;
using mowgli_behavior::SanitizeEscapeCfg;
using mowgli_behavior::StartBlockedEscapeCfg;
using mowgli_behavior::StartBlockedEscapeState;

namespace
{

/// Everything green: confirmed start-blocked pass, blade verified off.
EscapePreconditions ReadyToEscape()
{
  return EscapePreconditions{/*start_blocked_armed=*/true,
                             /*blade_state_fresh=*/true,
                             /*blade_off=*/true};
}

LastMotionSignal Motion(double vx, double age_s)
{
  return LastMotionSignal{/*valid=*/true, vx, age_s};
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. The two field cases, from one signal.
// ---------------------------------------------------------------------------

// The reported #487 event. The dock is at ~(6.2, 2.8); undock backs out on a
// ~126° bearing straight at a 0.25 m obstacle circle, and the robot came to
// rest at (4.47, 4.70) inside its inflated keepout. The LAST MOTION WAS REVERSE
// — so the escape must drive FORWARD, back the way it came, away from the thing
// it reversed onto. Reversing here is the one direction that makes it worse.
TEST(StartBlockedEscapeDirection, PostUndockLastMotionReverseDrivesForward)
{
  const StartBlockedEscapeCfg cfg;
  const auto verdict = EscapeDecide(cfg, ReadyToEscape(), Motion(-0.15, 2.0));

  EXPECT_EQ(verdict, EscapeVerdict::kEscapeForward);
  EXPECT_TRUE(EscapeMoves(verdict));

  StartBlockedEscapeState st;
  EXPECT_GT(EscapeStep(cfg, st, EscapeVerdictDirection(verdict), 0.1), 0.0)
      << "post-undock the robot REVERSED into the blocked cell; reversing again drives it deeper";
}

// Mid-mow (a promoted dig keepout can appear under a robot that is already
// standing on it): the robot ARRIVED DRIVING FORWARD, so reversing retraces
// ground it physically occupied seconds ago.
TEST(StartBlockedEscapeDirection, MidMowLastMotionForwardDrivesReverse)
{
  const StartBlockedEscapeCfg cfg;
  const auto verdict = EscapeDecide(cfg, ReadyToEscape(), Motion(0.20, 1.0));

  EXPECT_EQ(verdict, EscapeVerdict::kEscapeReverse);
  EXPECT_TRUE(EscapeMoves(verdict));

  StartBlockedEscapeState st;
  EXPECT_LT(EscapeStep(cfg, st, EscapeVerdictDirection(verdict), 0.1), 0.0)
      << "mid-mow the robot arrived driving FORWARD; reversing retraces known-good ground";
}

// ---------------------------------------------------------------------------
// 2. Stand-downs — never guess a direction.
// ---------------------------------------------------------------------------

TEST(StartBlockedEscapeStandDown, UnknownDirectionCommandsNothing)
{
  const StartBlockedEscapeCfg cfg;
  LastMotionSignal never_seen;  // valid == false
  const auto verdict = EscapeDecide(cfg, ReadyToEscape(), never_seen);

  EXPECT_EQ(verdict, EscapeVerdict::kNoDirectionSignal);
  EXPECT_FALSE(EscapeMoves(verdict));

  StartBlockedEscapeState st;
  EXPECT_DOUBLE_EQ(EscapeStep(cfg, st, EscapeVerdictDirection(verdict), 0.1), 0.0);
  EXPECT_DOUBLE_EQ(st.travelled, 0.0);
  EXPECT_DOUBLE_EQ(st.elapsed, 0.0) << "a stand-down must not even start the clock";
}

TEST(StartBlockedEscapeStandDown, StaleDirectionCommandsNothing)
{
  StartBlockedEscapeCfg cfg;
  cfg.signal_max_age_s = 90.0;

  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(0.20, 89.0)), EscapeVerdict::kEscapeReverse);
  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(0.20, 91.0)),
            EscapeVerdict::kNoDirectionSignal)
      << "a direction older than signal_max_age_s no longer describes how we got here";
}

TEST(StartBlockedEscapeStandDown, NearZeroDirectionCommandsNothing)
{
  StartBlockedEscapeCfg cfg;
  cfg.min_signal_speed = 0.03;

  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(0.005, 1.0)),
            EscapeVerdict::kNoDirectionSignal);
  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(-0.005, 1.0)),
            EscapeVerdict::kNoDirectionSignal);
  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(0.0, 1.0)),
            EscapeVerdict::kNoDirectionSignal);
}

// NaN in either field must stand down, not fall through a sign comparison.
TEST(StartBlockedEscapeStandDown, NaNSignalCommandsNothing)
{
  const StartBlockedEscapeCfg cfg;
  const double nan = std::nan("");

  EXPECT_EQ(EscapeDirectionFromLastMotion(cfg, Motion(nan, 1.0)), EscapeDirection::kUnknown);
  EXPECT_EQ(EscapeDirectionFromLastMotion(cfg, Motion(0.20, nan)), EscapeDirection::kUnknown);
}

// ---------------------------------------------------------------------------
// 3. It cannot fire without #495's confirmed start-blocked signal.
// ---------------------------------------------------------------------------

// The gate that keeps this motion off every other failure in the tree.
TEST(StartBlockedEscapeGate, CannotFireWithoutTheStartBlockedSignal)
{
  const StartBlockedEscapeCfg cfg;
  EscapePreconditions pre = ReadyToEscape();
  pre.start_blocked_armed = false;

  const auto verdict = EscapeDecide(cfg, pre, Motion(0.20, 1.0));
  EXPECT_EQ(verdict, EscapeVerdict::kNotStartBlocked);
  EXPECT_FALSE(EscapeMoves(verdict))
      << "the escape may only fire on a CONFIRMED START_OCCUPIED-with-zero-progress pass (#495)";
}

// Checked before the direction, so a perfectly good direction signal can never
// smuggle the motion past a missing gate.
TEST(StartBlockedEscapeGate, MissingSignalOutranksAGoodDirection)
{
  const StartBlockedEscapeCfg cfg;
  EscapePreconditions pre;  // all false
  EXPECT_EQ(EscapeDecide(cfg, pre, Motion(-0.15, 0.5)), EscapeVerdict::kNotStartBlocked);
}

TEST(StartBlockedEscapeGate, DisabledByConfigCommandsNothing)
{
  StartBlockedEscapeCfg cfg;
  cfg.enabled = false;
  EXPECT_EQ(EscapeDecide(cfg, ReadyToEscape(), Motion(0.20, 1.0)), EscapeVerdict::kDisabled);
}

// A zero speed / distance / timeout is a configured "do nothing".
TEST(StartBlockedEscapeGate, ZeroBudgetCommandsNothing)
{
  StartBlockedEscapeCfg no_speed;
  no_speed.speed = 0.0;
  EXPECT_EQ(EscapeDecide(no_speed, ReadyToEscape(), Motion(0.20, 1.0)), EscapeVerdict::kDisabled);

  StartBlockedEscapeCfg no_distance;
  no_distance.distance = 0.0;
  EXPECT_EQ(EscapeDecide(no_distance, ReadyToEscape(), Motion(0.20, 1.0)),
            EscapeVerdict::kDisabled);

  StartBlockedEscapeCfg no_time;
  no_time.timeout_s = 0.0;
  EXPECT_EQ(EscapeDecide(no_time, ReadyToEscape(), Motion(0.20, 1.0)), EscapeVerdict::kDisabled);
}

// ---------------------------------------------------------------------------
// 4. Blade must be VERIFIED off, not merely requested off.
// ---------------------------------------------------------------------------

TEST(StartBlockedEscapeBlade, BladeOnCommandsNothing)
{
  const StartBlockedEscapeCfg cfg;
  EscapePreconditions pre = ReadyToEscape();
  pre.blade_off = false;
  EXPECT_EQ(EscapeDecide(cfg, pre, Motion(0.20, 1.0)), EscapeVerdict::kBladeNotVerifiedOff);
}

// "We asked for blade off and heard nothing back" is NOT verification.
TEST(StartBlockedEscapeBlade, StaleBladeStateCommandsNothing)
{
  const StartBlockedEscapeCfg cfg;
  EscapePreconditions pre = ReadyToEscape();
  pre.blade_state_fresh = false;
  EXPECT_EQ(EscapeDecide(cfg, pre, Motion(0.20, 1.0)), EscapeVerdict::kBladeNotVerifiedOff);
}

// ---------------------------------------------------------------------------
// 5. Both bounds terminate the escape.
// ---------------------------------------------------------------------------

TEST(StartBlockedEscapeBounds, DistanceBoundTerminatesTheEscape)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = 0.10;
  cfg.distance = 0.40;
  cfg.timeout_s = 60.0;  // deliberately far away, so distance is what ends it

  StartBlockedEscapeState st;
  int ticks = 0;
  while (!EscapeDone(cfg, st) && ticks < 10000)
  {
    EscapeStep(cfg, st, EscapeDirection::kReverse, 0.1);
    ++ticks;
  }

  EXPECT_TRUE(EscapeDone(cfg, st));
  EXPECT_NEAR(st.travelled, cfg.distance, 1e-9);
  EXPECT_LE(st.travelled, cfg.distance) << "the distance budget must never be overshot";
  EXPECT_LT(st.elapsed, cfg.timeout_s);
  EXPECT_DOUBLE_EQ(EscapeStep(cfg, st, EscapeDirection::kReverse, 0.1), 0.0)
      << "once the budget is spent the command must be exactly zero";
}

TEST(StartBlockedEscapeBounds, TimeBoundTerminatesTheEscape)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = 0.10;
  cfg.distance = 5.0;  // deliberately unreachable, so the timeout is what ends it
  cfg.timeout_s = 6.0;

  StartBlockedEscapeState st;
  int ticks = 0;
  while (!EscapeDone(cfg, st) && ticks < 10000)
  {
    EscapeStep(cfg, st, EscapeDirection::kForward, 0.1);
    ++ticks;
  }

  EXPECT_TRUE(EscapeDone(cfg, st));
  EXPECT_GE(st.elapsed, cfg.timeout_s);
  EXPECT_LT(st.travelled, cfg.distance);
  EXPECT_DOUBLE_EQ(EscapeStep(cfg, st, EscapeDirection::kForward, 0.1), 0.0);
}

// The manoeuvre must terminate even if the caller keeps ticking with a dt of
// zero or a nonsensical negative dt.
TEST(StartBlockedEscapeBounds, NonPositiveDtCommandsNothingAndDoesNotAdvance)
{
  const StartBlockedEscapeCfg cfg;
  StartBlockedEscapeState st;

  EXPECT_DOUBLE_EQ(EscapeStep(cfg, st, EscapeDirection::kForward, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(EscapeStep(cfg, st, EscapeDirection::kForward, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(st.elapsed, 0.0);
  EXPECT_DOUBLE_EQ(st.travelled, 0.0);
}

// The commanded magnitude never exceeds the configured speed, in either
// direction, at any point during the manoeuvre.
TEST(StartBlockedEscapeBounds, CommandMagnitudeNeverExceedsTheConfiguredSpeed)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = 0.10;

  for (const auto dir : {EscapeDirection::kForward, EscapeDirection::kReverse})
  {
    StartBlockedEscapeState st;
    for (int i = 0; i < 200; ++i)
    {
      const double v = EscapeStep(cfg, st, dir, 0.05);
      EXPECT_LE(std::abs(v), cfg.speed + 1e-12);
      if (dir == EscapeDirection::kForward)
      {
        EXPECT_GE(v, 0.0);
      }
      else
      {
        EXPECT_LE(v, 0.0);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 6. A bad config cannot exceed the compiled ceilings.
// ---------------------------------------------------------------------------

TEST(StartBlockedEscapeBounds, SanitizeClampsAnAbsurdConfig)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = 5.0;
  cfg.distance = 100.0;
  cfg.timeout_s = 3600.0;
  cfg.signal_max_age_s = 1.0e9;

  const auto safe = SanitizeEscapeCfg(cfg);
  EXPECT_DOUBLE_EQ(safe.speed, kEscapeMaxSpeed);
  EXPECT_DOUBLE_EQ(safe.distance, kEscapeMaxDistance);
  EXPECT_DOUBLE_EQ(safe.timeout_s, kEscapeMaxTimeout);
  EXPECT_LE(safe.signal_max_age_s, 300.0);
}

// Negative values in the config must not flip the escape direction — the sign
// is owned by the direction logic, never by the speed parameter.
TEST(StartBlockedEscapeBounds, SanitizeMakesNegativeMagnitudesPositive)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = -0.10;
  cfg.distance = -0.40;

  const auto safe = SanitizeEscapeCfg(cfg);
  EXPECT_DOUBLE_EQ(safe.speed, 0.10);
  EXPECT_DOUBLE_EQ(safe.distance, 0.40);

  StartBlockedEscapeState st;
  EXPECT_GT(EscapeStep(safe, st, EscapeDirection::kForward, 0.1), 0.0);
}

// Even if SanitizeEscapeCfg is skipped, EscapeStep clamps the magnitude itself.
TEST(StartBlockedEscapeBounds, EscapeStepClampsAnUnsanitizedSpeed)
{
  StartBlockedEscapeCfg cfg;
  cfg.speed = 5.0;
  cfg.distance = 10.0;
  cfg.timeout_s = 10.0;

  StartBlockedEscapeState st;
  EXPECT_LE(std::abs(EscapeStep(cfg, st, EscapeDirection::kReverse, 0.1)), kEscapeMaxSpeed);
}

// The default configuration must sit inside the ceilings — a default that
// needed clamping would mean the defaults and the safety envelope disagree.
TEST(StartBlockedEscapeBounds, DefaultsAreInsideTheCompiledCeilings)
{
  const StartBlockedEscapeCfg cfg;
  EXPECT_LE(cfg.speed, kEscapeMaxSpeed);
  EXPECT_LE(cfg.distance, kEscapeMaxDistance);
  EXPECT_LE(cfg.timeout_s, kEscapeMaxTimeout);
  EXPECT_GT(cfg.timeout_s, cfg.distance / cfg.speed)
      << "the timeout must leave room for the nominal manoeuvre, or it always wins";
}
