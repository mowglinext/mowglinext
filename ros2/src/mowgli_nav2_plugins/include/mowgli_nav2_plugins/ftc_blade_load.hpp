// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure blade-load slowdown decision, factored out of
// FTCController::update_control_point so it is unit-testable without ROS
// (see ftc_controller.cpp, test_ftc_blade_load.cpp). When the blade motor
// bogs down in thick or wet grass its RPM sags under load. Driving on at the
// nominal mowing speed then feeds the blade faster than it can cut — it
// stalls, leaves an uncut strip, or the firmware trips it. Instead, scale the
// carrot's target speed DOWN in proportion to the RPM sag so the blade gets
// time to chew through, and back up as soon as the RPM recovers.
//
// The map from RPM to speed scale is a linear ramp between two operator
// thresholds (rpm_min → min_speed_ratio, rpm_full → 1.0), clamped at both
// ends. A ramp rather than a step makes the closed loop (slower feed → less
// load → RPM recovers → faster feed → more load) settle at an equilibrium
// instead of bang-banging between the two speeds.
//
// FAIL-OPEN by design: the slowdown only ever engages on POSITIVE evidence
// of a loaded, spinning blade. A blade reported inactive (dry run,
// mowing_enabled=false, spin-up not yet confirmed), stale telemetry, or
// nonsensical thresholds all yield scale 1.0 — the robot must never crawl
// because a sensor went quiet. This is a cut-quality feature, not a safety
// interlock; the STM32 firmware stays the sole blade safety authority.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins
{

struct FtcBladeLoadCfg
{
  bool enabled = false;  ///< Master switch (GUI toggle).
  double rpm_full = 2500.0;  ///< RPM at/above which the speed scale is 1.0.
  double rpm_min = 1800.0;  ///< RPM at/below which the scale is min_speed_ratio.
  double min_speed_ratio = 0.4;  ///< Floor of the scale (fraction of the target speed).
  double telemetry_max_age_s = 1.0;  ///< Older blade telemetry is treated as absent.
};

struct FtcBladeLoadResult
{
  double target_speed;  ///< Path speed after blade-load scaling.
  double scale;  ///< Applied scale in [min_speed_ratio, 1.0]; 1.0 when not limiting.
  bool is_limited;  ///< True while the blade load is holding the speed down.
};

/// Fraction of the target speed the blade load permits. Pure map from RPM
/// to scale; the caller-facing gates live in BladeLoadDecision().
inline double BladeLoadScale(double rpm, const FtcBladeLoadCfg& cfg)
{
  const double min_ratio = std::clamp(cfg.min_speed_ratio, 0.0, 1.0);
  const double span = cfg.rpm_full - cfg.rpm_min;
  if (!std::isfinite(rpm) || !std::isfinite(span) || span <= 0.0)
  {
    return 1.0;  // Degenerate thresholds: never limit on a misconfiguration.
  }
  const double load_ratio = std::clamp((rpm - cfg.rpm_min) / span, 0.0, 1.0);
  return min_ratio + (1.0 - min_ratio) * load_ratio;
}

// Apply blade-load scaling to `target_speed` (the speed already computed
// from the path's straight-ahead lookahead and any stall easing).
// `blade_active` is the blade controller's own is_active report
// (Status.mower_esc_status != 0), `rpm` its last reported RPM and
// `telemetry_age_s` how old that report is (negative = never received).
// `speed_floor` bounds the slowed speed from below — the caller passes its
// crawl speed so a heavily loaded blade never drops the chassis under the
// firmware wheel deadband, where the robot would sit still with the blade
// grinding one spot. A floor above the unscaled target is ignored (the
// slowdown must never SPEED UP the robot).
inline FtcBladeLoadResult BladeLoadDecision(double target_speed,
                                            bool blade_active,
                                            double rpm,
                                            double telemetry_age_s,
                                            double speed_floor,
                                            const FtcBladeLoadCfg& cfg)
{
  const bool telemetry_fresh = telemetry_age_s >= 0.0 && telemetry_age_s <= cfg.telemetry_max_age_s;
  if (!cfg.enabled || !blade_active || !telemetry_fresh)
  {
    return {target_speed, 1.0, false};
  }

  const double scale = BladeLoadScale(rpm, cfg);
  if (scale >= 1.0)
  {
    return {target_speed, 1.0, false};
  }

  const double floor = std::min(std::max(speed_floor, 0.0), target_speed);
  const double scaled = std::max(target_speed * scale, floor);
  return {scaled, scale, scaled < target_speed};
}

}  // namespace mowgli_nav2_plugins
