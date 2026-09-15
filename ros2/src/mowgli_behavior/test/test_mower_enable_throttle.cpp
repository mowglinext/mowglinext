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
//
// Docked Orange Pi 5B, 2026-09-16: IdleSequence ticked SetMowerEnabled(false)
// 7.9x/s, re-sending an identical blade-disable service request forever. The
// throttle must cut that to the refresh rate WITHOUT ever delaying a changed
// value — a swallowed blade-off would be a safety regression, not a saving.
//
// The clock is a plain parameter, so every case below drives a fake one and no
// test sleeps or depends on wall-clock timing.
#include "mowgli_behavior/mower_enable_throttle.hpp"
#include <gtest/gtest.h>

namespace mb = mowgli_behavior;

namespace
{
using Clock = std::chrono::steady_clock;

/// Fixed fake epoch; offsets below are relative to it.
Clock::time_point t0()
{
  return Clock::time_point{};
}

Clock::time_point at(double seconds)
{
  return t0() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}
}  // namespace

// ── never swallow the first call ────────────────────────────────────────────

TEST(MowerEnableThrottle, FirstCallAlwaysSends)
{
  const mb::MowerEnableThrottleState fresh;
  EXPECT_TRUE(mb::ShouldSendMowerEnable(fresh, /*requested=*/false, at(0.0)));
  EXPECT_TRUE(mb::ShouldSendMowerEnable(fresh, /*requested=*/true, at(0.0)));
}

// ── dedup inside the refresh window ─────────────────────────────────────────

TEST(MowerEnableThrottle, RepeatWithinWindowIsSkipped)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/false, at(0.0));

  // The BT tick rate is 10 Hz; every one of these repeats must be skipped.
  for (int tick = 1; tick <= 19; ++tick)
  {
    const double now_s = 0.1 * tick;  // 0.1 .. 1.9 s, all < kMowerRefreshPeriodSec
    EXPECT_FALSE(mb::ShouldSendMowerEnable(st, /*requested=*/false, at(now_s)))
        << "unchanged request at t=" << now_s << " s should have been deduped";
  }
}

TEST(MowerEnableThrottle, SameTickAsTheSendIsSkipped)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/true, at(5.0));
  EXPECT_FALSE(mb::ShouldSendMowerEnable(st, /*requested=*/true, at(5.0)));
}

// ── never swallow a state change ────────────────────────────────────────────

TEST(MowerEnableThrottle, ChangedValueSendsImmediately)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/false, at(0.0));

  // One BT tick later — deep inside the refresh window — a blade-ON request
  // must still go out on this very tick.
  EXPECT_TRUE(mb::ShouldSendMowerEnable(st, /*requested=*/true, at(0.1)));
}

TEST(MowerEnableThrottle, BladeOffIsNeverDelayedByTheWindow)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/true, at(0.0));

  // The safety-relevant direction: a stop request must not wait for the
  // refresh period, no matter how recently the enable was sent.
  EXPECT_TRUE(mb::ShouldSendMowerEnable(st, /*requested=*/false, at(0.001)));
}

// ── periodic refresh ────────────────────────────────────────────────────────

TEST(MowerEnableThrottle, SendsAgainAfterTheRefreshPeriod)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/false, at(0.0));

  EXPECT_FALSE(mb::ShouldSendMowerEnable(st, false, at(mb::kMowerRefreshPeriodSec - 0.05)));
  EXPECT_TRUE(mb::ShouldSendMowerEnable(st, false, at(mb::kMowerRefreshPeriodSec)));
  EXPECT_TRUE(mb::ShouldSendMowerEnable(st, false, at(mb::kMowerRefreshPeriodSec + 10.0)));
}

TEST(MowerEnableThrottle, RefreshWindowRestartsFromEachSend)
{
  mb::MowerEnableThrottleState st;
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/false, at(0.0));

  // Refresh fires at 2.0 s...
  ASSERT_TRUE(mb::ShouldSendMowerEnable(st, false, at(2.0)));
  mb::RecordMowerEnableSent(st, /*sent_enabled=*/false, at(2.0));
  // ...and the next window is measured from there, not from t0.
  EXPECT_FALSE(mb::ShouldSendMowerEnable(st, false, at(3.9)));
  EXPECT_TRUE(mb::ShouldSendMowerEnable(st, false, at(4.0)));
}

// ── the measured regression this exists to prevent ──────────────────────────

TEST(MowerEnableThrottle, DockedIdleSendRateCollapsesToTheRefreshRate)
{
  mb::MowerEnableThrottleState st;
  int sends = 0;

  // 60 s of IdleSequence at the 10 Hz BT tick rate, always requesting OFF.
  for (int tick = 0; tick < 600; ++tick)
  {
    const auto now = at(0.1 * tick);
    if (mb::ShouldSendMowerEnable(st, /*requested=*/false, now))
    {
      mb::RecordMowerEnableSent(st, false, now);
      ++sends;
    }
  }

  // The t=0 send plus one per 2 s window across 0.0-59.9 s: 30, versus the
  // 600 the untrottled node issued over the same minute.
  EXPECT_EQ(sends, 30);
}

TEST(MowerEnableThrottle, AChangeMidIdleStillGetsThrough)
{
  mb::MowerEnableThrottleState st;
  int on_sends = 0;

  for (int tick = 0; tick < 100; ++tick)
  {
    const auto now = at(0.1 * tick);
    // The operator presses Play at tick 45 (t = 4.5 s), 0.5 s into a window.
    const bool requested = tick >= 45;
    if (mb::ShouldSendMowerEnable(st, requested, now))
    {
      mb::RecordMowerEnableSent(st, requested, now);
      if (requested && on_sends == 0)
      {
        // The blade-on request went out on the very tick it was requested.
        EXPECT_EQ(tick, 45);
      }
      if (requested)
        ++on_sends;
    }
  }

  EXPECT_GT(on_sends, 0);
}
