// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wheel-slip "digging" detector + bounded reverse escape. Pure logic, no ROS,
// so it is unit-testable standalone (see test_dig_detector.cpp) — same shape
// as mowgli_nav2_plugins/ftc_stall.hpp.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// Every pre-existing "am I stuck?" check on this robot measures THE WHEELS,
// so none of them can see a dig — the failure where the wheels turn freely
// while the chassis goes nowhere and the tyres carve a hole in the lawn:
//
//   * Firmware anti-dig (cpp_main.cpp ANTIDIG_*) compares encoder ticks to
//     the travel the command implies. A SPINNING wheel produces the full
//     tick count, so the fraction test never trips. It catches a BLOCKED
//     wheel, not a slipping one — and that remains its job (it is the
//     un-bypassable backstop and stays exactly as it is).
//   * FTC's stall cap (ftc_stall.hpp) reads measured forward speed from
//     /wheel_odom (nav2_params_base.yaml controller.odom_topic), i.e. the
//     same encoders, which cheerfully confirm the phantom motion. So the
//     controller ramps to speed_fast and floors it — that IS what digs.
//   * fusion_graph's slip veto (graph_params.hpp slip_*) is ROTATIONAL only
//     (gyro yaw vs wheel yaw). A straight-line dig has gyro ~= 0 AND wheel
//     yaw ~= 0, so nothing is vetoed and the graph dead-reckons the robot
//     forward THROUGH the hole it is digging.
//
// The one signal that is independent of the wheels is the GNSS-anchored
// fused pose (/odometry/filtered_map). Over a short window, "encoders claim
// we travelled X, the map says we moved much less than X" is an unambiguous
// dig signature — no other failure mode produces it. That comparison is what
// this header makes.
//
// ── Trust gating ────────────────────────────────────────────────────────────
// The comparison is only as good as the fused pose, so a sigma above
// max_pos_sigma SUPPRESSES detection (and resets the window) rather than
// guessing. This deliberately makes the detector self-disabling exactly when
// it would otherwise false-fire — the firmware backstop still covers the
// blocked-wheel case throughout.
//
// The sigma to gate on is the RECEIVER's reported horizontal accuracy under a
// confirmed RTK-Fixed solution, NOT the factor graph's own marginal.
//
// This was measured the hard way. The graph's marginal was tried first (major
// axis of the Pose2 position ellipse) and it does not work at ANY threshold,
// because it does not describe the robot's actual position quality. Sampled
// over 90 s of live mowing on 2026-08-24, RTK-Fixed throughout:
//
//     fusion_graph published sigma   p50 0.574 m   p90 0.957 m   max 1.391 m
//     GNSS horizontal_accuracy_m     0.014 m
//     FTC cross-track error          0.008 - 0.016 m
//
// The graph reports metre-level uncertainty while the receiver reports 14 mm
// and the controller demonstrably tracks to the centimetre. Threshold sweeps
// over the same sample confirm there is no usable setting: 0.25 m blocks
// 57.6 % of ticks, 0.40 m still blocks 56.4 %, and only ~1.0 m gets blocking
// into single digits — by which point the gate is meaningless, since a dig is
// itself only a ~0.2 m discrepancy per window. (The marginal is also erratic
// rather than merely large: consecutive ~1 Hz refreshes were observed swinging
// var_xx 0.0077 -> 5.55 while the robot tracked a path perfectly.)
//
// So the trust gate reads /gps/status. The REFERENCE for the comparison is
// unchanged and remains the GNSS-anchored fused pose — only the question "is
// that pose currently trustworthy?" moved onto a signal that answers it
// honestly. Under RTK-Fixed the receiver reports ~0.014 m, so a 0.10 m
// threshold passes comfortably; under RTK-Float it reports decimetres to
// metres and the detector stands down, which is the property Invariant 16
// requires.
//
// ── Turn exclusion (and why it MUST read the gyro) ──────────────────────────
// While the robot is turning, "encoders claim X, the map says less than X" is
// not evidence of a dig. Two effects produce it on a healthy robot: the
// encoders measure arc length while the map measures the chord, and — far
// larger in practice — fusion_graph's own estimate degrades through a
// turn-around (measured 2026-08-24: the published position sigma spikes from
// a ~0.05-0.09 m baseline to 1-3 m at every swath U-turn, and the wheel-vs-map
// ratio drops to 0.2-0.3 with the robot tracking perfectly well). Before this
// exclusion existed the sigma gate was the only thing suppressing those, which
// is why it could not simply be loosened to let genuine digs through.
//
// The yaw rate MUST come from the GYRO, never from a wheel-derived yaw. The
// dig recorded on 2026-08-24 was a ONE-WHEEL slip: the left encoder ran 889
// ticks (3.17 m) while the right ran 50, so the WHEEL yaw rate was enormous
// while the gyro read ~0 and the chassis did not rotate at all. A wheel-based
// turn exclusion would have stood down for exactly the event it exists to
// catch. The gyro is ~0 for both a straight-line dig and a one-wheel dig, and
// large only when the chassis genuinely rotates.
//
// ── What the two sides of the comparison must MEASURE ───────────────────────
// Both sides were originally measured in a way that hid the very event this
// detector exists to catch. Re-derived from the 2026-08-24 mow3 log (274 min,
// 24 948 encoder intervals), in which 33 episodes of sustained one-wheel spin
// ground 69.7 m of tyre across the lawn and only 4 of them ever latched:
//
//  * WHEEL side — use the WORST WHEEL's ground travel, not the chassis-centre
//    distance. The digs on this robot are asymmetric by construction: the
//    planner asks for a turn radius below the half-track, so the inner wheel
//    is commanded BACKWARDS while the outer wheel spins forward (issue #499).
//    The centre distance (dL+dR)/2 then cancels most of the motion — measured
//    over those 33 episodes it retained only 42 % of the grinding tyre's
//    travel, and in a symmetric pivot (dL = -dR) it cancels to exactly zero.
//    That directly starves min_wheel_dist: in 13 of the 33 episodes the centre
//    rate was so low that 0.15 m could not accrue inside one window AT ALL, so
//    those digs were arithmetically undetectable regardless of every other
//    gate. Against the worst wheel all 33 clear the floor (slowest 0.152 m/s
//    -> 0.18 m per 1.2 s window). Driving straight, the two measures are
//    identical, so nothing about normal operation changes.
//
//  * MAP side — use NET DISPLACEMENT across the window, not the sum of
//    per-tick |steps|. A sum of magnitudes is a path length: it grows with
//    estimator wander and with the tick rate, so raising dig_monitor_rate made
//    detection WORSE. During a dig the graph is a tug-of-war between a lying
//    wheel factor and GNSS, and /odometry/filtered_map wanders about a fixed
//    mean instead of translating. Measured at the five latches, the summed
//    map travel over 1.2 s was 0.00-0.07 m against a ~0.09 m threshold, while
//    the robot's TRUE motion was 0.13 m in 18 s (~0.009 m per window) — the
//    statistic swung across its whole budget on a robot that was parked, and
//    latched only in the windows where the wander happened to be quiet. Net
//    displacement does not accumulate wander, so a parked chassis reads ~0
//    every window. For a healthy robot the two agree: over 1.2 s at 0.2 m/s
//    the path is near-straight, and even at the turn-exclusion limit
//    (0.20 rad/s) chord/arc = 0.998.
//
// Together these cost the field-measured latency: onset of the wheel anomaly
// to DIG DETECTED ran 5.9-11.6 s (median 9.6 s), during which the digging
// tyre turned 2.7-5.6 m of surface. The 1.2 s window plus one 10 Hz monitor
// tick is the floor, so the best case is ~1.3 s (~0.6 m of tyre at the
// observed 0.47 m/s spin) — window_s is the false-fire guard and is NOT
// reduced here, because nothing in the log justifies a shorter one.
//
// Only ever REDUCES commanded motion, and the escape only ever commands
// REVERSE, bounded in both distance and time.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace mowgli_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// Detection
// ─────────────────────────────────────────────────────────────────────────────

struct DigDetectorCfg
{
  bool enabled = true;
  /// Sustained evidence required before latching a dig [s].
  double window_s = 1.2;
  /// Below this commanded speed the robot isn't being told to travel [m/s].
  double min_cmd_speed = 0.05;
  /// The WORST WHEEL must claim at least this much ground travel in the
  /// window before we call it a dig [m]. Without it, a hard stall (wheels not
  /// turning) — which is the FIRMWARE's case, not ours — would look
  /// identical, and a blocked wheel still produces ~0 travel here.
  double min_wheel_dist = 0.15;
  /// Latch when map travel is below this fraction of encoder travel.
  double progress_fraction = 0.35;
  /// Max position uncertainty to trust the comparison [m]. Fed from the GNSS
  /// receiver's reported horizontal accuracy under a confirmed RTK-Fixed
  /// solution — NOT the factor graph's marginal, which is unusable for this
  /// (see the trust-gating note above).
  double max_pos_sigma = 0.10;
  /// Above this measured yaw rate the robot is turning and the wheel-vs-map
  /// comparison is not evidence of anything — see the turn-exclusion note
  /// above [rad/s].
  double max_yaw_rate = 0.20;
};

/// Position uncertainty the trust gate should use, or +infinity when it cannot
/// be established — which makes DigDecide stand down rather than guess.
///
/// Deliberately demands ALL of: a fresh /gps/status, a confirmed RTK-Fixed
/// solution, and a receiver accuracy value the message actually carries
/// (value_flags). Anything less and we do not know how good the pose is.
inline double DigTrustSigma(bool gnss_fresh, bool rtk_fixed, bool accuracy_valid, double accuracy_m)
{
  if (!gnss_fresh || !rtk_fixed || !accuracy_valid || !(accuracy_m >= 0.0))
  {
    return std::numeric_limits<double>::infinity();
  }
  return accuracy_m;
}

enum class DigAction
{
  kNone,
  kDig
};

/// Verdict plus the evidence that produced it. The evidence is returned
/// (rather than left in the state) because DigDecide resets its window on
/// every verdict — without this, a caller reading the state after a kDig
/// would report zeros in its logs and in the DigEvent it publishes.
struct DigVerdict
{
  DigAction action = DigAction::kNone;
  double wheel_dist = 0.0;  ///< worst wheel's claimed ground travel [m]
  double map_dist = 0.0;  ///< fused-pose NET displacement over the window [m]
};

/// Caller-owned window accumulators (mirrors ftc_stall's in-place state).
struct DigDetectorState
{
  double window_time = 0.0;
  double wheel_dist = 0.0;
  /// Fused-pose position at the first tick of the current window. The map
  /// side is measured as NET DISPLACEMENT from here — see the note above on
  /// why summing per-tick |steps| cannot work.
  bool have_anchor = false;
  double anchor_x = 0.0;
  double anchor_y = 0.0;
};

inline void DigResetWindow(DigDetectorState& st)
{
  st = DigDetectorState{};
}

/// Feed one control tick.
///
/// @param cmd_speed  commanded forward speed this tick [m/s]
/// @param wheel_step WORST WHEEL's ground travel since last tick [m] — not
///                   the chassis-centre distance, which cancels the
///                   asymmetric spin these digs are made of
/// @param map_x      fused-pose (GNSS-anchored) map x [m]. The window's map
/// @param map_y      travel is the NET DISPLACEMENT from the first tick of
///                   the window to this one, never a sum of |steps|
/// @param pos_sigma  trusted 1-sigma position uncertainty [m], from
///                   DigTrustSigma — infinity when it cannot be established
/// @param yaw_rate   MEASURED (gyro) yaw rate [rad/s] — never wheel-derived
/// @param dt         tick duration [s]
inline DigVerdict DigDecide(const DigDetectorCfg& cfg,
                            DigDetectorState& st,
                            double cmd_speed,
                            double wheel_step,
                            double map_x,
                            double map_y,
                            double pos_sigma,
                            double yaw_rate,
                            double dt)
{
  if (!cfg.enabled || dt <= 0.0)
  {
    return {};
  }

  // Can't trust the only wheel-independent signal we have -> stand down.
  if (!(pos_sigma <= cfg.max_pos_sigma))  // NaN-safe: NaN sigma suppresses too
  {
    DigResetWindow(st);
    return {};
  }

  // Turning -> arc-vs-chord and a degraded fused estimate both shrink the
  // ratio on a perfectly healthy robot. Stand down. NaN suppresses too.
  if (!(std::abs(yaw_rate) <= cfg.max_yaw_rate))
  {
    DigResetWindow(st);
    return {};
  }

  // Not commanded to travel -> nothing to compare against.
  if (std::abs(cmd_speed) < cfg.min_cmd_speed)
  {
    DigResetWindow(st);
    return {};
  }

  if (!st.have_anchor)
  {
    st.anchor_x = map_x;
    st.anchor_y = map_y;
    st.have_anchor = true;
  }

  st.window_time += dt;
  st.wheel_dist += std::abs(wheel_step);

  if (st.window_time < cfg.window_s)
  {
    return {};  // not enough evidence yet
  }

  const double map_dist = std::hypot(map_x - st.anchor_x, map_y - st.anchor_y);

  const bool wheels_claim_travel = st.wheel_dist >= cfg.min_wheel_dist;
  const bool map_disagrees = map_dist < cfg.progress_fraction * st.wheel_dist;

  const DigVerdict verdict{(wheels_claim_travel && map_disagrees) ? DigAction::kDig
                                                                  : DigAction::kNone,
                           st.wheel_dist,
                           map_dist};

  DigResetWindow(st);  // start a fresh window either way
  return verdict;
}

// ─────────────────────────────────────────────────────────────────────────────
// Bounded reverse escape
// ─────────────────────────────────────────────────────────────────────────────
//
// After a dig latches, the robot backs straight out of the hole it just made.
// The budget is bounded BOTH ways on purpose: distance is the intent, but the
// distance is integrated from the COMMANDED speed rather than the encoders,
// because the encoders are precisely the signal we just decided we cannot
// trust. If the tyres are still slipping, the timeout is what ends the
// manoeuvre. Reverse is chosen (not a turn) because it retraces ground the
// robot already occupied a moment ago and so is the least likely direction to
// find a new obstacle — the same reasoning as FTC's reverse-escape.

struct DigEscapeCfg
{
  double reverse_speed = 0.12;  ///< magnitude of the reverse command [m/s]
  double reverse_dist = 0.30;  ///< distance budget [m]
  double timeout_s = 4.0;  ///< hard time bound [s]
};

struct DigEscapeState
{
  double travelled = 0.0;  ///< commanded-integral distance so far [m]
  double elapsed = 0.0;  ///< time spent escaping [s]
};

inline bool DigEscapeDone(const DigEscapeCfg& cfg, const DigEscapeState& st)
{
  return st.travelled >= cfg.reverse_dist || st.elapsed >= cfg.timeout_s;
}

/// Advance the escape by one tick. Returns the forward velocity to command:
/// negative while escaping, exactly 0.0 once the budget is spent.
inline double DigEscapeStep(const DigEscapeCfg& cfg, DigEscapeState& st, double dt)
{
  if (dt <= 0.0 || DigEscapeDone(cfg, st))
  {
    return 0.0;
  }

  st.elapsed += dt;
  st.travelled = std::min(cfg.reverse_dist, st.travelled + std::abs(cfg.reverse_speed) * dt);

  return -std::abs(cfg.reverse_speed);
}

}  // namespace mowgli_hardware
