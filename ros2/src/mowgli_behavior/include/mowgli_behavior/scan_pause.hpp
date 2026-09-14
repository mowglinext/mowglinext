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
// Blade pause/resume across a SHORT LiDAR dropout, decided per FollowStrip tick.
// Pure logic, no ROS — unit-tested standalone (test_scan_pause.cpp), same
// shape as start_blocked_escape.hpp / ftc_stall.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// Field 2026-09-12: an LD19 whose UART link dropped out for 1–2 s every 15–20 s
// tripped the Root-level IsScanStale guard (1.0 s) each time. Halting the Root
// is the right answer to a DEAD scanner, but for a blip it costs far more than
// the blip: FollowPath goal cancelled, full F2C replan, transit back to the
// resume cursor, 1.5 s blade spin-up — and the sequence restarted before the
// next dropout. The robot cycled CALIBRATING → UNDOCKING → PLANNING → TRANSIT
// for a whole session and mowed nothing (up to 200 re-dispatches, ~28 min).
//
// ── What a short dropout actually needs ─────────────────────────────────────
// Motion is already handled below the tree: collision_monitor's
// source_timeout (1.5 s, nav2_params_base.yaml) turns a stale scan source into
// a STOP action and zeroes cmd_vel until the source is valid again. The only
// thing the tree must add is the BLADE — cut it while the robot is blind, put
// it back when the scan stream is back. The FollowPath goal, FTC's path index
// and the resume cursor are all untouched, so mowing continues from the exact
// pose where it paused, with no replan and no transit.
//
// ── Two stages ──────────────────────────────────────────────────────────────
//   * kScanPauseMaxAgeSec (1.0 s): blade OFF, goal kept  (this header, FollowStrip)
//   * IsScanStale max_age_sec in main_tree.xml (20 s): the existing Root halt,
//     blade OFF + MarkGuardHalt, for a scanner that is really gone. Kept under
//     the controller's movement_time_allowance (30 s) so a long outage is ended
//     by the halt (clean cursor resume) and not by FTC's "Failed to make
//     progress" abort (which steps the cursor forward).
//
// ── Stand-downs ─────────────────────────────────────────────────────────────
//   * no scan EVER received (no-LiDAR install): never pauses;
//   * resume only after kScanResumeFreshSec of CONTINUOUS fresh scans, so a
//     stream that flaps at the threshold does not toggle the blade every tick.
#pragma once

namespace mowgli_behavior
{

/// Scan older than this while a coverage goal is active → blade OFF, goal kept.
/// Matches the historical IsScanStale threshold; collision_monitor stops the
/// wheels 0.5 s later on its own.
constexpr double kScanPauseMaxAgeSec = 1.0;

/// Continuous fresh-scan time required before the blade is re-enabled.
constexpr double kScanResumeFreshSec = 0.5;

enum class ScanPauseAction
{
  kNone,
  kPause,  ///< cut the blade now (scan just went stale)
  kResume,  ///< re-enable the blade now (scan has been fresh long enough)
};

struct ScanPauseState
{
  bool paused = false;
  double fresh_for_s = 0.0;  ///< continuous fresh time accumulated while paused
};

/// One tick of the pause/resume decision.
/// @param st          caller-owned state
/// @param have_scan   false when no scan was ever received this session
/// @param scan_age_s  age of the newest /scan_collision message [s]
/// @param dt_s        time since the previous tick [s]
inline ScanPauseAction ScanPauseStep(ScanPauseState& st,
                                     bool have_scan,
                                     double scan_age_s,
                                     double dt_s)
{
  if (!have_scan)
  {
    return ScanPauseAction::kNone;  // no-LiDAR install: inert, like IsScanStale
  }
  const bool stale = scan_age_s > kScanPauseMaxAgeSec;
  if (stale)
  {
    st.fresh_for_s = 0.0;
    if (st.paused)
    {
      return ScanPauseAction::kNone;
    }
    st.paused = true;
    return ScanPauseAction::kPause;
  }
  if (!st.paused)
  {
    return ScanPauseAction::kNone;
  }
  st.fresh_for_s += (dt_s > 0.0 ? dt_s : 0.0);
  if (st.fresh_for_s < kScanResumeFreshSec)
  {
    return ScanPauseAction::kNone;
  }
  st.paused = false;
  st.fresh_for_s = 0.0;
  return ScanPauseAction::kResume;
}

}  // namespace mowgli_behavior
