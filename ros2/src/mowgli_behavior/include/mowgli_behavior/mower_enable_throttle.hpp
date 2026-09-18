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
// Per-tick send decision for SetMowerEnabled. Pure logic, no ROS — unit-tested
// standalone (test_mower_enable_throttle.cpp), same shape as scan_pause.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// IdleSequence ticks <SetMowerEnabled enabled="false"/> unconditionally, so a
// robot parked on the dock re-sent the SAME blade-disable service request at
// the BT tick rate. Measured on a docked Orange Pi 5B (2026-09-16): 7.9
// calls/s, i.e. 2375 service round-trips and ~4800 log lines per 5 minutes,
// costing ~5 % of one core split across behavior_tree_node and
// hardware_bridge_node while the robot was doing nothing at all.
//
// ── Why it is a throttle and not a pure edge trigger ────────────────────────
// The blade command is fire-and-forget by design (the firmware is the sole
// safety authority, see CLAUDE.md § Safety) and the ROS2 side never learns
// whether the request landed. A pure send-on-change would therefore lose the
// blade-off state for good if one request were dropped, so a periodic refresh
// is kept: the same value is re-sent every kMowerRefreshPeriodSec.
//
// ── Invariants this must keep ───────────────────────────────────────────────
//   * the FIRST request of a session is always sent (no state to dedup against)
//   * a CHANGED value is always sent immediately, never swallowed or delayed —
//     that covers both blade-on and, more importantly, blade-off
//   * only an unchanged value inside the refresh window is skipped
#pragma once

#include <chrono>

namespace mowgli_behavior
{

/// An unchanged blade request is re-sent at least this often, so a dropped
/// fire-and-forget service call cannot leave the firmware holding a stale
/// command indefinitely. 2.0 s drops the docked-idle rate from ~7.9 calls/s to
/// 0.5 calls/s (a ~16x cut) while still refreshing well inside the firmware
/// heartbeat window.
constexpr double kMowerRefreshPeriodSec = 2.0;

/// Last blade request actually put on the wire by SetMowerEnabled. Lives on
/// the shared BTContext (guarded by its context_mutex) rather than on the node
/// instance, so a tree halt/rebuild — which destroys and recreates the BT node
/// — cannot silently reset the throttle and restart the spam.
struct MowerEnableThrottleState
{
  /// False until the first request has been sent this session.
  bool has_sent{false};
  /// `enabled` value of the last request sent.
  bool last_sent_enabled{false};
  /// When that last request was sent.
  std::chrono::steady_clock::time_point last_sent_time{};
};

/// Decide whether this tick must actually call the mower_control service.
///
/// @param state      last-send bookkeeping (not modified; see RecordMowerEnableSent)
/// @param requested  the `enabled` value this tick wants
/// @param now        current steady-clock reading (injectable for tests)
/// @return true when the request must go out on the wire
inline bool ShouldSendMowerEnable(const MowerEnableThrottleState& state,
                                  bool requested,
                                  std::chrono::steady_clock::time_point now)
{
  if (!state.has_sent)
  {
    return true;  // never swallow the first call
  }
  if (requested != state.last_sent_enabled)
  {
    return true;  // never swallow a state change
  }
  const double since_s = std::chrono::duration<double>(now - state.last_sent_time).count();
  return since_s >= kMowerRefreshPeriodSec;
}

/// Record a request that was actually sent. Call ONLY after the service
/// request has been dispatched — recording a send that never happened would
/// let the throttle suppress the retry.
inline void RecordMowerEnableSent(MowerEnableThrottleState& state,
                                  bool sent_enabled,
                                  std::chrono::steady_clock::time_point now)
{
  state.has_sent = true;
  state.last_sent_enabled = sent_enabled;
  state.last_sent_time = now;
}

}  // namespace mowgli_behavior
