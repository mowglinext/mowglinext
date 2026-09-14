// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Repeat-dig escalation. Pure logic, no ROS, so it is unit-testable
// standalone (see test_dig_escalation.cpp) — same shape as dig_detector.hpp,
// whose verdicts feed it.
//
// ── Why this exists ─────────────────────────────────────────────────────────
// dig_detector.hpp answers "am I digging RIGHT NOW?" and its per-event
// response — hard stop on the wire, bounded reverse, a pending keepout
// stamped by map_server — is correct and is NOT changed by this header. What
// it cannot answer is "have I been digging in the same place all afternoon?",
// and that is a different failure with a different remedy: N latches inside a
// small radius and a short window is not a dig to route around, it is an
// obstruction the robot cannot resolve on its own.
//
// ── The field evidence (issue #500, instrumented capture 2026-09-04) ────────
// The robot wedged against a physical object and latched SEVENTEEN times.
// The first three were isolated and genuine — 4.7 m, 5.9 m and 1.2 m from
// each other, minutes apart. The remaining fourteen are one obstruction:
//
//     t (s)     map (x, y)        tyre travel   map travel
//     11517.1   (-5.39, 11.86)      0.42 m        0.01 m
//     11522.8   (-5.38, 11.84)      0.52 m        0.08 m
//     11528.6   (-5.40, 11.87)      0.55 m        0.03 m
//     11534.4   (-5.39, 11.84)      0.55 m        0.00 m
//     11540.0   (-5.37, 11.80)      0.29 m        0.05 m
//     ... 9 more over the next 5.5 min, still inside 20 cm
//
// Five latches in 23 seconds inside a 6 cm square, then 5.5 more minutes of
// latching while the chassis crawled 20 cm. Throughout, the LiDAR reported
// 31-108 beams inside 1 m with a minimum range of 0.29-0.52 m, and the blade
// RPM collapsed and recovered repeatedly (3482 -> 622 -> 0 -> 503 -> 1394 ->
// 3259 -> 0): the blade was striking something solid.
//
// The existing responses cannot break that cycle, and it is worth being
// precise about why, because each of them is doing its job correctly:
//
//   * The bounded reverse WORKS — wheel_vx goes to -0.11 m/s after every
//     latch and the robot backs off a few centimetres. But nothing tells the
//     controller that the NEXT manoeuvre must differ, so it drives straight
//     back in. Widening the reverse does not help; the robot is not failing
//     to escape, it is being re-aimed at the same object.
//   * The 0.60 m pending keepout WORKS — but the robot is not re-entering a
//     mapped cell. It is being pushed back into the same PHYSICAL object by
//     the next planned motion, and the keepout only constrains where coverage
//     plans, not where a wedged chassis ends up.
//
// So the missing piece is not a bigger reaction to one dig. It is noticing
// that the same dig keeps happening, and handing the problem to the operator
// instead of grinding. Seventeen latches with the blade stalling is a far
// worse outcome than an early give-up: this only ever makes the robot stop
// SOONER, and it commands no motion of its own.
//
// ── What this header does NOT decide ────────────────────────────────────────
// Escalating is a verdict, not an action. The bridge keeps doing exactly what
// it did per event (it is a layer that only ever REDUCES motion) and merely
// raises a flag; stopping the MISSION is the behaviour tree's job, via the
// IsDigEscalated condition node.

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace mowgli_hardware
{

/// One recorded dig latch: where the robot was, and when.
struct DigLatch
{
  double x = 0.0;  ///< map-frame x [m]
  double y = 0.0;  ///< map-frame y [m]
  double t = 0.0;  ///< monotonic timestamp [s]
};

using DigLatchHistory = std::vector<DigLatch>;

/// Hard cap on retained latches. Pruning by age already bounds the history
/// under any realistic latch rate (the field capture managed 5 in 23 s), but
/// the cap makes the bound unconditional and independent of the clock.
inline constexpr std::size_t kDigLatchHistoryMax = 32;

struct DigEscalationCfg
{
  /// Latches within this distance of each other count as "the same spot" [m].
  /// 0.50 m is comfortably wider than the 6 cm square the field cluster fell
  /// in, and comfortably tighter than the 1.2 m separating the closest pair
  /// of genuinely distinct digs in the same run. <= 0 disables escalation.
  double radius_m = 0.50;
  /// How far back to look [s]. 60 s spans the 23 s cluster with room to
  /// spare while letting a spot the robot successfully left go stale.
  /// <= 0 disables escalation.
  double window_s = 60.0;
  /// Latches (including the current one) required inside radius_m and
  /// window_s before escalating. 3 fires on the third of the field cluster,
  /// ~11 s into an episode that ran 5.5 minutes. <= 1 disables escalation —
  /// escalating on a single latch would defeat the per-event response, which
  /// is the correct handling of an isolated dig.
  int min_count = 3;
};

/// True when the configuration can ever escalate. All three bounds are
/// disable sentinels so an operator can turn the feature off from any one of
/// them without the others having to agree.
inline bool DigEscalationEnabled(const DigEscalationCfg& cfg)
{
  return cfg.radius_m > 0.0 && cfg.window_s > 0.0 && cfg.min_count > 1;
}

/// Returns a NEW history with everything older than window_s dropped.
/// Entries stamped in the future (a clock jump) are kept — discarding them
/// would silently forget real latches.
inline DigLatchHistory DigPruneLatches(const DigLatchHistory& history,
                                       double now_t,
                                       double window_s)
{
  DigLatchHistory kept;
  kept.reserve(history.size());
  for (const DigLatch& latch : history)
  {
    if (now_t - latch.t <= window_s)
    {
      kept.push_back(latch);
    }
  }
  return kept;
}

/// Returns a NEW history: the pruned old one plus this latch. Immutable by
/// construction — the caller assigns the result, so a history is never half
/// updated. The oldest entry is dropped if the cap would be exceeded.
inline DigLatchHistory DigRecordLatch(
    const DigLatchHistory& history, double x, double y, double t, double window_s)
{
  DigLatchHistory next = DigPruneLatches(history, t, window_s);
  if (next.size() >= kDigLatchHistoryMax)
  {
    const auto drop = static_cast<std::ptrdiff_t>(next.size() - kDigLatchHistoryMax + 1);
    next.erase(next.begin(), next.begin() + drop);
  }
  next.push_back(DigLatch{x, y, t});
  return next;
}

/// How many latches — this one included — sit within radius_m of (x, y) and
/// within window_s of t. `history` holds the PREVIOUS latches; the current
/// one is counted here rather than being required to be recorded first, so
/// the caller can decide and record in either order.
inline int DigNearbyLatchCount(
    const DigLatchHistory& history, double x, double y, double t, double radius_m, double window_s)
{
  int count = 1;  // the latch being judged
  for (const DigLatch& latch : history)
  {
    const double age = t - latch.t;
    if (age < 0.0 || age > window_s)
    {
      continue;
    }
    if (std::hypot(x - latch.x, y - latch.y) <= radius_m)
    {
      ++count;
    }
  }
  return count;
}

/// True when this latch is the min_count'th inside radius_m and window_s —
/// i.e. the robot is stuck against something it cannot get past, rather than
/// having found one soft patch.
///
/// @param history   previous latches, oldest first (this one NOT included)
/// @param x,y       map-frame position of the latch being judged [m]
/// @param t         monotonic timestamp of that latch [s]
/// @param radius_m  "same spot" radius; <= 0 disables
/// @param window_s  look-back window; <= 0 disables
/// @param min_count latches required to escalate; <= 1 disables
inline bool ShouldEscalate(const DigLatchHistory& history,
                           double x,
                           double y,
                           double t,
                           double radius_m,
                           double window_s,
                           int min_count)
{
  if (radius_m <= 0.0 || window_s <= 0.0 || min_count <= 1)
  {
    return false;
  }
  return DigNearbyLatchCount(history, x, y, t, radius_m, window_s) >= min_count;
}

/// Config-taking overload — the form the bridge uses.
inline bool ShouldEscalate(
    const DigEscalationCfg& cfg, const DigLatchHistory& history, double x, double y, double t)
{
  return ShouldEscalate(history, x, y, t, cfg.radius_m, cfg.window_s, cfg.min_count);
}

}  // namespace mowgli_hardware
