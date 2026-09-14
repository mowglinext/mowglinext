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
// LocalizationGuard input: is our ABSOLUTE POSITION trustworthy right now?
// Pure logic, no ROS, so it is unit-testable standalone (see
// test_localization_health.cpp) — same shape as mowgli_hardware/dig_detector.hpp.
//
// ── Why this exists (and why it is NOT the fused covariance) ────────────────
// The guard was added 2026-08-04 (#422) after the 2026-08-02 incident: RTK
// fell back to plain GPS (σ ≈ 1.5 m) for >60 s and FTC kept steering on the
// drifting estimate until the robot physically left the area. It keyed on
// σ_xy from /odometry/filtered_map pose covariance.
//
// That was the wrong signal. fusion_graph DELIBERATELY releases the X
// constraint as a design mechanism, so its marginal covariance balloons while
// localization is perfectly healthy:
//
//   * pivot_wheel_sigma_x = 0.5 m/node whenever per-tick gyro dθ exceeds
//     pivot_gate_dtheta_rad (~0.3 rad/s) — "effectively releasing the X
//     constraint so GPS + scan-matching set XY" (graph_params.hpp).
//   * adaptive_noise_enabled_gain = 10 inflates σ_x on the wheel↔gyro
//     residual, up to ~1.05 m/node on a slip event.
//
// With node_period_s = 0.04 (25 Hz) and GNSS at 5 Hz there are ~5
// GPS-unconstrained nodes per fix, so the head marginal reaches
// sqrt(5)·0.5 ≈ 1.1 m on any pivot and sqrt(5)·0.092 ≈ 0.21 m in plain
// driving — both above the old 0.15 m pause threshold. Parked, node cadence
// drops to stationary_node_period_s = 5 s and GPS pins every node, so σ
// settles at ~0.05 m, below the old 0.08 m resume threshold.
//
// UPDATE 2026-08-24 (issue #491): the "plain driving" half of that arithmetic
// has since been fixed — the wheel between-factor's translational sigma is now
// a distance-scaled random walk instead of a per-node constant, so the same
// window contributes centimetres rather than decimetres. The PIVOT half is
// unchanged and deliberate: pivot_wheel_sigma_x stays a per-node absolute
// release, so the marginal still balloons on every PRE_ROTATE. This guard must
// therefore keep off the fused covariance regardless.
//
// The guard was therefore ONLY satisfiable while stopped, and FTC opens every
// strip with a PRE_ROTATE pivot. Field 2026-08-20 (robot on RTK-Fixed at
// hacc 0.014 m throughout): degrade at σ 0.57/0.74/0.81/0.92/0.99/1.07/1.38/
// 1.86 m, recover at exactly 0.05 m every time, ten cycles in eight minutes,
// FollowStrip pinned at pose 32/14104 (0%). A livelock — mowing never
// started. 45a1cf8e made the guard actually halt the tree, which turned the
// pre-existing false positive into a hard block (its own message records
// "σ_xy = 0.83 m while the receiver held RTK-Fixed at 100 %").
//
// So: key the guard on what it actually cares about — GNSS solution quality
// from the typed /gps/status contract, which reports the plain-GPS fallback
// directly and does not move when the robot pivots.
//
// ── The σ backstop ─────────────────────────────────────────────────────────
// One real hazard /gps/status cannot see: the localizer rejecting every fix
// while the receiver stays healthy (the reverted GnssMobileGate runaway
// documented in CLAUDE.md — "every fix rejected forever, GPS locked out, and
// cov_xx ballooned to ~2.5 m σ"). So a SECOND, deliberately generous latch
// keeps watching σ_xy. Its threshold must sit above the localizer's own
// inflation ceiling (~2.35 m worst case above) — hence a 5 m default with a
// long persistence, which no pivot can reach or sustain.
//
// This only ever PAUSES blade-on mowing. Firmware remains the sole blade
// safety authority, and BoundaryGuard still independently covers "the robot
// left the area".

#pragma once

#include <cmath>
#include <cstdint>

namespace mowgli_behavior
{

// RTK solution mode, mirroring mowgli_interfaces/GnssStatus RTK_MODE_*.
// Duplicated as a plain enum so this header stays ROS-free and testable.
enum class RtkMode : uint8_t
{
  kUnknown = 0,
  kNone = 1,
  kFloat = 2,
  kFixed = 3,
};

/// Why the guard latched. Carried into the log line and asserted in tests.
enum class LocalizationFault : uint8_t
{
  kNone = 0,
  kGnssAccuracy,  ///< reported horizontal accuracy exceeded the pause threshold
  kGnssFixLost,  ///< no accuracy available AND the receiver reports no RTK solution
  kGnssStale,  ///< /gps/status stopped arriving
  kSigmaBackstop,  ///< fused σ_xy implausible for long enough to mean divergence
};

struct LocalizationHealthCfg
{
  /// GNSS horizontal accuracy that pauses mowing [m], and the tighter value
  /// that resumes it. Default 0.30/0.15 sits far above RTK-Fixed (~0.015 m)
  /// and ordinary RTK-Float, but well under the ~1.5 m plain-GPS fallback the
  /// guard exists to catch.
  double gnss_acc_pause_m = 0.30;
  double gnss_acc_resume_m = 0.15;

  /// /gps/status older than this counts as a dead GNSS feed [s].
  double gnss_stale_s = 5.0;

  /// Debounce on the GNSS path: the condition must hold this long before the
  /// latch flips, so per-epoch fix flicker does not chatter the guard.
  double gnss_pause_persist_s = 3.0;
  double gnss_resume_persist_s = 2.0;

  /// Divergence backstop on the fused marginal [m]. MUST stay above the
  /// localizer's deliberate pivot/slip inflation ceiling (~2.35 m) — see the
  /// file header. Set 0 to disable the backstop entirely.
  double sigma_backstop_pause_m = 5.0;
  double sigma_backstop_resume_m = 2.0;
  double sigma_backstop_persist_s = 10.0;
};

/// Hysteresis latch with independent enter/exit persistence.
///
/// `bad` and `good` are evaluated against DIFFERENT thresholds, so both are
/// false inside the dead-band — which holds the latch wherever it already is.
class PersistentLatch
{
public:
  bool Update(double now_s, bool bad, bool good, double bad_persist_s, double good_persist_s)
  {
    if (!latched_)
    {
      good_since_s_ = -1.0;
      if (!bad)
      {
        bad_since_s_ = -1.0;
        return latched_;
      }
      if (bad_since_s_ < 0.0)
        bad_since_s_ = now_s;
      if (now_s - bad_since_s_ >= bad_persist_s)
      {
        latched_ = true;
        bad_since_s_ = -1.0;
      }
      return latched_;
    }

    bad_since_s_ = -1.0;
    if (!good)
    {
      good_since_s_ = -1.0;
      return latched_;
    }
    if (good_since_s_ < 0.0)
      good_since_s_ = now_s;
    if (now_s - good_since_s_ >= good_persist_s)
    {
      latched_ = false;
      good_since_s_ = -1.0;
    }
    return latched_;
  }

  bool latched() const
  {
    return latched_;
  }

  void ForceLatched()
  {
    latched_ = true;
    bad_since_s_ = -1.0;
    good_since_s_ = -1.0;
  }

  /// Seconds the pending (not yet flipped) condition has been held, or 0.
  double pending_for_s(double now_s) const
  {
    const double since = latched_ ? good_since_s_ : bad_since_s_;
    return since < 0.0 ? 0.0 : now_s - since;
  }

private:
  bool latched_ = false;
  double bad_since_s_ = -1.0;
  double good_since_s_ = -1.0;
};

/// Everything the monitor knows about the current instant. Assembled by the
/// caller from the two subscriptions; unavailable values are negative.
struct LocalizationObservation
{
  /// True once ANY /gps/status has been received. While false the monitor
  /// stays inert — see LocalizationHealthMonitor::Update.
  bool gnss_seen = false;
  /// True only while the last genuinely new receiver observation satisfies
  /// the shared receipt-provenance and monotonic-age policy. Cached ROS
  /// republication never changes this value.
  bool gnss_fresh = false;
  RtkMode rtk_mode = RtkMode::kUnknown;
  /// Receiver-reported horizontal accuracy [m]; negative when unavailable
  /// (capability bit clear, or NaN in the message).
  double gnss_accuracy_m = -1.0;
  /// σ_xy from the fused pose covariance [m]; negative when unavailable.
  double fused_sigma_xy_m = -1.0;
};

/// Latches "absolute position is not trustworthy" from GNSS solution quality,
/// with a generous fused-σ divergence backstop.
class LocalizationHealthMonitor
{
public:
  LocalizationHealthMonitor() = default;
  explicit LocalizationHealthMonitor(const LocalizationHealthCfg& cfg) : cfg_(cfg)
  {
  }

  /// Fold one observation in and return the latched degraded state.
  bool Update(double now_s, const LocalizationObservation& obs)
  {
    // A guard that cannot observe its input must not block a bladed robot
    // from operating. Legacy bring-ups without a /gps/status publisher keep
    // mowing; BoundaryGuard still covers "the robot left the area".
    if (!obs.gnss_seen)
    {
      fault_ = LocalizationFault::kNone;
      return false;
    }

    const bool stale = !obs.gnss_fresh;
    const bool has_acc = obs.gnss_accuracy_m >= 0.0 && std::isfinite(obs.gnss_accuracy_m);
    // Prefer the metric signal. rtk_mode only decides when the receiver does
    // not report an accuracy at all — otherwise a receiver that leaves
    // rtk_mode UNKNOWN while reporting 1.4 cm would be called degraded.
    const bool no_rtk_solution =
        obs.rtk_mode == RtkMode::kNone || obs.rtk_mode == RtkMode::kUnknown;

    bool gnss_bad = stale;
    bool gnss_good = !stale;
    LocalizationFault pending = stale ? LocalizationFault::kGnssStale : LocalizationFault::kNone;
    if (!stale)
    {
      if (has_acc)
      {
        gnss_bad = obs.gnss_accuracy_m > cfg_.gnss_acc_pause_m;
        gnss_good = obs.gnss_accuracy_m < cfg_.gnss_acc_resume_m;
        if (gnss_bad)
          pending = LocalizationFault::kGnssAccuracy;
      }
      else
      {
        gnss_bad = no_rtk_solution;
        gnss_good = !no_rtk_solution;
        if (gnss_bad)
          pending = LocalizationFault::kGnssFixLost;
      }
    }

    const bool was_gnss = gnss_latch_.latched();
    bool gnss_degraded = false;
    if (stale)
    {
      // Observation freshness is an authorization boundary, not receiver
      // quality flicker. Once the existing provenance timeout has elapsed,
      // fail closed immediately; retain the configured resume persistence for
      // a later genuine observation.
      gnss_latch_.ForceLatched();
      gnss_degraded = true;
    }
    else
    {
      gnss_degraded = gnss_latch_.Update(
          now_s, gnss_bad, gnss_good, cfg_.gnss_pause_persist_s, cfg_.gnss_resume_persist_s);
    }

    // Divergence backstop. Disabled at 0, and skipped when σ is unavailable
    // so a missing covariance never latches on its own.
    bool sigma_degraded = false;
    const bool was_sigma = sigma_latch_.latched();
    if (cfg_.sigma_backstop_pause_m > 0.0 && obs.fused_sigma_xy_m >= 0.0 &&
        std::isfinite(obs.fused_sigma_xy_m))
    {
      sigma_degraded = sigma_latch_.Update(now_s,
                                           obs.fused_sigma_xy_m > cfg_.sigma_backstop_pause_m,
                                           obs.fused_sigma_xy_m < cfg_.sigma_backstop_resume_m,
                                           cfg_.sigma_backstop_persist_s,
                                           cfg_.gnss_resume_persist_s);
    }
    else
    {
      sigma_degraded = was_sigma;
    }

    // Attribute the fault to whichever latch just closed; keep the existing
    // attribution while both stay closed.
    if (gnss_degraded && !was_gnss)
      fault_ = pending;
    else if (sigma_degraded && !was_sigma)
      fault_ = LocalizationFault::kSigmaBackstop;
    else if (!gnss_degraded && !sigma_degraded)
      fault_ = LocalizationFault::kNone;

    return gnss_degraded || sigma_degraded;
  }

  bool degraded() const
  {
    return gnss_latch_.latched() || sigma_latch_.latched();
  }
  LocalizationFault fault() const
  {
    return fault_;
  }
  double gnss_stale_s() const
  {
    return cfg_.gnss_stale_s;
  }

private:
  LocalizationHealthCfg cfg_{};
  PersistentLatch gnss_latch_{};
  PersistentLatch sigma_latch_{};
  LocalizationFault fault_ = LocalizationFault::kNone;
};

/// Short human-readable reason for the log line.
inline const char* LocalizationFaultName(LocalizationFault fault)
{
  switch (fault)
  {
    case LocalizationFault::kGnssAccuracy:
      return "GNSS accuracy";
    case LocalizationFault::kGnssFixLost:
      return "no RTK solution";
    case LocalizationFault::kGnssStale:
      return "GNSS feed stale";
    case LocalizationFault::kSigmaBackstop:
      return "fused-sigma divergence backstop";
    case LocalizationFault::kNone:
    default:
      return "none";
  }
}

}  // namespace mowgli_behavior
