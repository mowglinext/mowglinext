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
// Field 2026-09-12: an LD19 UART link dropping out for 1–2 s every 15–20 s
// halted the Root through IsScanStale each time and the robot mowed nothing.
// The blade pause must cut the blade on the dropout, keep it off while the
// stream is down, and restore it only after a continuous fresh window.
#include "mowgli_behavior/scan_pause.hpp"
#include <gtest/gtest.h>

namespace mb = mowgli_behavior;

TEST(ScanPause, NoLidarInstallNeverPauses)
{
  mb::ScanPauseState st;
  EXPECT_EQ(mb::ScanPauseStep(st, /*have_scan=*/false, 99.0, 0.1), mb::ScanPauseAction::kNone);
  EXPECT_FALSE(st.paused);
}

TEST(ScanPause, FreshStreamDoesNothing)
{
  mb::ScanPauseState st;
  for (int i = 0; i < 50; ++i)
  {
    EXPECT_EQ(mb::ScanPauseStep(st, true, 0.12, 0.1), mb::ScanPauseAction::kNone);
  }
  EXPECT_FALSE(st.paused);
}

TEST(ScanPause, DropoutPausesOnceAndResumesAfterContinuousFreshWindow)
{
  mb::ScanPauseState st;
  // 1.7 s gap as measured on 2026-09-12: pause fires once, then stays quiet.
  EXPECT_EQ(mb::ScanPauseStep(st, true, 1.1, 0.1), mb::ScanPauseAction::kPause);
  EXPECT_TRUE(st.paused);
  EXPECT_EQ(mb::ScanPauseStep(st, true, 1.4, 0.1), mb::ScanPauseAction::kNone);
  EXPECT_EQ(mb::ScanPauseStep(st, true, 1.7, 0.1), mb::ScanPauseAction::kNone);
  // Stream back: no resume before kScanResumeFreshSec of fresh scans.
  double fresh = 0.0;
  mb::ScanPauseAction last = mb::ScanPauseAction::kNone;
  int ticks = 0;
  while (last != mb::ScanPauseAction::kResume && ticks < 100)
  {
    last = mb::ScanPauseStep(st, true, 0.1, 0.1);
    fresh += 0.1;
    ++ticks;
  }
  EXPECT_EQ(last, mb::ScanPauseAction::kResume);
  EXPECT_GE(fresh, mb::kScanResumeFreshSec);
  EXPECT_LT(fresh, mb::kScanResumeFreshSec + 0.25);
  EXPECT_FALSE(st.paused);
}

TEST(ScanPause, FlappingStreamDoesNotToggleTheBlade)
{
  mb::ScanPauseState st;
  ASSERT_EQ(mb::ScanPauseStep(st, true, 1.2, 0.1), mb::ScanPauseAction::kPause);
  // Fresh for 0.3 s, stale again, fresh 0.3 s, stale again: never resumes.
  for (int cycle = 0; cycle < 5; ++cycle)
  {
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_EQ(mb::ScanPauseStep(st, true, 0.1, 0.1), mb::ScanPauseAction::kNone);
    }
    EXPECT_EQ(mb::ScanPauseStep(st, true, 1.3, 0.1), mb::ScanPauseAction::kNone);
  }
  EXPECT_TRUE(st.paused);
}

TEST(ScanPause, ThresholdsMatchTheMotionLayer)
{
  // Blade cut at the historical IsScanStale threshold; collision_monitor's
  // source_timeout (1.5 s) then holds the wheels — the blade must never be the
  // last thing still running on a blind robot.
  EXPECT_DOUBLE_EQ(mb::kScanPauseMaxAgeSec, 1.0);
  EXPECT_LT(mb::kScanPauseMaxAgeSec, 1.5);
  EXPECT_GT(mb::kScanResumeFreshSec, 0.0);
}
