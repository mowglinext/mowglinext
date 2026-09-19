// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Robot status -> pixel buffer. Pure logic, no ROS and no hardware, so it is
// unit-testable standalone (see test_led_pattern.cpp) -- same shape as
// mowgli_hardware/dig_detector.hpp and mowgli_nav2_plugins/ftc_start_index.hpp.
//
// Every animation is a pure function of `now_s`, so a test can render an exact
// phase without sleeping and without a clock.
//
// -- Display semantics -------------------------------------------------------
// This is an OUTDOOR machine: the ring has to be readable in daylight from
// several metres, by someone who is not holding a phone. That rules out
// low-saturation colours, single-pixel state encodings and anything that needs
// counting. Every mode below is distinguishable by COLOUR plus MOTION alone.
//
// Priority (first match wins -- an alarm always beats an activity):
//
//   1. EMERGENCY      solid red, whole ring, no animation.
//                     The one pattern nothing else can be confused with:
//                     it is the only SOLID red and the only static full ring.
//   2. CHARGING       green arc proportional to battery, breathing ~3 s.
//                     Steady (non-breathing) full green once battery is at
//                     charge_full_percent -- "done, you can take it". If that
//                     steady state persists for charge_complete_timeout_s (a
//                     robot left on the dock overnight), the ring dims to
//                     charge_complete_dim_scale instead -- 0 by default, i.e.
//                     off -- so a fully charged robot does not shine a bright
//                     green light indefinitely. Optionally, a handful of
//                     pixels stay lit at charge_complete_indicator_scale
//                     instead of dimming with the rest, so an operator can
//                     still tell at a glance the robot is powered and
//                     charged: either charge_complete_indicator_count of
//                     them, evenly spaced around the ring, or the exact
//                     pixels named in charge_complete_indicator_ids (which
//                     takes priority when non-empty). Both empty/0 (default)
//                     disables this and the whole ring dims uniformly. Any
//                     change (unplugged, level drops below
//                     charge_full_percent) resets the timer and the ring
//                     goes back to full brightness immediately.
//                     Ranked above STALE so a robot charging on the dock with
//                     the behavior tree down still reads as charging: the
//                     charge state comes from the hardware bridge's Power
//                     message, which does not depend on the BT being up. In
//                     exactly that case the battery PERCENT is unknown (only
//                     the BT derives it from pack voltage), so the ring shows
//                     a green comet instead of an arc rather than inventing a
//                     level.
//   3. STALE          amber comet, dark ring, ~1.5 s per revolution.
//                     "Powered and running, but the behavior tree is not
//                     talking to me." Distinct from every normal mode because
//                     it is the only DARK ring with a moving amber head.
//   4. LOW BATTERY    whole ring blinking red at 1 Hz.
//                     BLINKING, where emergency is SOLID -- deliberately the
//                     same hue because both mean "attend to me", with motion
//                     as the discriminator.
//   5. MOWING         green arc proportional to coverage_percent, with a white
//                     head pixel at the arc tip so the boundary is crisp at
//                     distance. Steady.
//      MOWING_DEGRADED (autonomous, RTK not fixed) same arc in AMBER with the
//                     head pixel blinking at 2 Hz. Cutting accuracy is
//                     degraded without an RTK-Fixed solution, and that is
//                     worth seeing from across the lawn -- warm colour plus
//                     motion, both changed, so it cannot be mistaken for the
//                     healthy arc in bright sun where hue alone is unreliable.
//      TRANSIT        (autonomous, blade off, driving a blade-off transit
//                     between coverage sub-paths) the two ring HALVES
//                     alternate lit/dark in light orange, ~1 s each -- not a
//                     comet (nothing rotates around the ring) and not a
//                     whole-ring blink (LOW BATTERY), so it cannot be
//                     confused with either: exactly half the ring is ever
//                     lit, and which half flips. Ranked ahead of MOWING/
//                     MOWING_DEGRADED because it overrides them for the
//                     duration of the transit -- coverage progress is not
//                     advancing right now regardless of RTK state.
//   6. RECORDING      cyan comet, ~2 s per revolution. The operator is walking
//                     the boundary; motion says "I am tracking you".
//   7. MANUAL MOWING  whole ring breathing purple, ~2 s. No progress exists to
//                     show, so the ring only says "blades may be running".
//   8. IDLE           whole ring dim white, static. Cheapest possible frame
//                     (it never changes, so it only costs the keepalive
//                     write) and answers the only question idle raises:
//                     is the robot powered and awake?
//
// Colour choices are the saturated primaries/secondaries that survive daylight;
// blue is deliberately unused as a state colour because it is the weakest of
// them outdoors.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mowgli_leds/ws2812_encoder.hpp"

namespace mowgli_leds
{

/// Mirror of mowgli_interfaces/HighLevelStatus HIGH_LEVEL_STATE_*.
/// Kept as a plain enum so this header stays ROS-free; led_ring_node.cpp
/// static_asserts the values against the generated message constants, so a
/// change on either side is a COMPILE error rather than a silently wrong ring.
enum class HighLevelState : std::uint8_t
{
  kNull = 0,
  kIdle = 1,
  kAutonomous = 2,
  kRecording = 3,
  kManualMowing = 4,
};

enum class LedMode
{
  kEmergency,
  kCharging,
  kStale,
  kLowBattery,
  kMowing,
  kMowingDegraded,
  kTransit,
  kRecording,
  kManual,
  kIdle,
};

/// Everything the ring is allowed to know about the robot. The node resolves
/// staleness and source precedence before filling this in, so the pattern
/// logic never has to reason about topics or clocks.
struct LedInputs
{
  /// A HighLevelStatus arrived within the staleness timeout.
  bool status_fresh = false;
  HighLevelState state = HighLevelState::kNull;
  /// Smooth mowing progress of the current area, 0..100.
  float coverage_percent = 0.0f;
  /// battery_percent carries a real reading (from HighLevelStatus, or from
  /// the hardware bridge's Power message when the behavior tree is silent).
  bool battery_valid = false;
  float battery_percent = 0.0f;
  bool is_charging = false;
  bool emergency = false;
  /// GNSS currently reports an RTK-Fixed solution.
  bool rtk_fixed = false;
  /// FollowStrip is currently driving a blade-off transit between coverage
  /// sub-paths (HighLevelStatus.sub_state_name == "TRANSIT" while state is
  /// AUTONOMOUS). Only meaningful in that state; the node does not need to
  /// clear it elsewhere since SelectMode only reads it inside the
  /// kAutonomous branch.
  bool transiting = false;
  /// Seconds the ring has continuously shown the steady "charge complete"
  /// frame (is_charging && battery_valid && battery_percent >=
  /// charge_full_percent). Zero whenever that condition does not hold. The
  /// node resets this the instant the condition breaks, so it never survives
  /// an unplug or a level dropping back below charge_full_percent.
  double charge_complete_elapsed_s = 0.0;
  /// Monotonic seconds, the only time base the animations use.
  double now_s = 0.0;
};

struct LedPatternCfg
{
  std::size_t led_count = 16u;
  /// Below this, and not charging, the ring blinks red.
  float low_battery_percent = 20.0f;
  /// At or above this the charging ring stops breathing and goes steady.
  float charge_full_percent = 99.0f;
  /// Scale applied to the idle ring, on top of the global led_brightness.
  float idle_scale = 0.10f;
  /// After the ring has shown steady full-green "charge complete" for this
  /// many seconds, dim it to charge_complete_dim_scale. 0 disables the
  /// feature (the ring stays full green indefinitely, the original
  /// behaviour).
  double charge_complete_timeout_s = 600.0;
  /// Brightness scale applied once charge_complete_timeout_s has elapsed.
  /// 0 (the default) turns the ring off; a small non-zero value keeps a faint
  /// green glow as an at-a-glance "still on and charged" indicator.
  float charge_complete_dim_scale = 0.0f;
  /// Number of pixels, evenly spaced around the ring, kept at
  /// charge_complete_indicator_scale instead of charge_complete_dim_scale
  /// once charge_complete_timeout_s has elapsed -- e.g. 1 keeps a single
  /// pixel lit, 4 keeps four pixels spaced 90 degrees apart. Clamped to
  /// [0, led_count] at render time. 0 (the default) disables this: the
  /// whole ring dims uniformly to charge_complete_dim_scale, exactly as if
  /// this parameter did not exist. Ignored when charge_complete_indicator_
  /// ids below is non-empty.
  std::size_t charge_complete_indicator_count = 0u;
  /// Explicit pixel indices kept lit instead of the even spacing above --
  /// lets an operator pick exactly which pixels stay on (e.g. the one
  /// facing the house) rather than an auto-computed spacing. Empty (the
  /// default) falls back to charge_complete_indicator_count. Indices
  /// outside [0, led_count) are silently ignored at render time (a stale
  /// entry left over after led_count was lowered must not crash or wrap).
  std::vector<std::size_t> charge_complete_indicator_ids;
  /// Brightness of the indicator pixels above, relative to led_brightness.
  /// Independent of charge_complete_dim_scale so the indicator pixels can
  /// stay visibly brighter than the dimmed (or fully off) background.
  float charge_complete_indicator_scale = 0.15f;
};

namespace colors
{
inline constexpr Rgb kOff{0, 0, 0};
inline constexpr Rgb kRed{255, 0, 0};
inline constexpr Rgb kGreen{0, 255, 0};
inline constexpr Rgb kAmber{255, 110, 0};
/// Deliberately lighter/warmer than kAmber (Stale/Degraded) -- both are in
/// the orange family, but TRANSIT is never shown alongside either (Stale
/// requires a non-fresh status; Degraded is superseded by Transit inside the
/// same kAutonomous branch), so the pairing only needs to read as distinct
/// when directly compared, not be confusable at a glance in isolation --
/// MOTION (alternating halves vs. a rotating comet) is the real
/// discriminator, per the design rule at the top of this file.
inline constexpr Rgb kLightOrange{255, 170, 40};
inline constexpr Rgb kCyan{0, 200, 255};
inline constexpr Rgb kPurple{170, 0, 255};
inline constexpr Rgb kWhite{255, 255, 255};
}  // namespace colors

/// Scale a colour by `scale` (clamped to [0, 1]); non-finite scales go dark.
inline Rgb Dim(const Rgb& color, float scale)
{
  return Rgb{
      ws2812::ScaleChannel(color.r, scale),
      ws2812::ScaleChannel(color.g, scale),
      ws2812::ScaleChannel(color.b, scale),
  };
}

/// Positive remainder of `value / period`, so animations behave for negative
/// timestamps too (std::fmod keeps the sign of the numerator).
inline double PositivePhase(double value, double period)
{
  if (!(period > 0.0) || !std::isfinite(value))
  {
    return 0.0;
  }
  const double remainder = std::fmod(value, period);
  return remainder < 0.0 ? remainder + period : remainder;
}

/// Sinusoidal breathing envelope in [min_scale, max_scale]. Starts at
/// min_scale at phase 0 so a mode change fades IN rather than snapping on.
inline float Breathe(double now_s, double period_s, float min_scale, float max_scale)
{
  if (!(period_s > 0.0))
  {
    return max_scale;
  }
  // Spelled out rather than M_PI: that macro is a POSIX extension and glibc
  // hides it under -std=c++17 (strict ANSI), which is exactly what this
  // package builds with (CMAKE_CXX_EXTENSIONS OFF).
  constexpr double kTwoPi = 6.283185307179586;
  const double phase = PositivePhase(now_s, period_s) / period_s;
  const double envelope = 0.5 * (1.0 - std::cos(kTwoPi * phase));
  return min_scale + (max_scale - min_scale) * static_cast<float>(envelope);
}

/// True during the "on" part of a square blink. `duty` is clamped to [0, 1].
inline bool BlinkOn(double now_s, double period_s, float duty = 0.5f)
{
  if (!(period_s > 0.0))
  {
    return true;
  }
  const double phase = PositivePhase(now_s, period_s) / period_s;
  return phase < static_cast<double>(std::clamp(duty, 0.0f, 1.0f));
}

/// Index of the comet head for a ring that revolves once per `period_s`.
inline std::size_t RotationIndex(double now_s, double period_s, std::size_t led_count)
{
  if (led_count == 0u || !(period_s > 0.0))
  {
    return 0u;
  }
  const double phase = PositivePhase(now_s, period_s) / period_s;
  const auto index = static_cast<std::size_t>(phase * static_cast<double>(led_count));
  return index % led_count;
}

/// Pixels to light for a progress arc of `percent` on `led_count` pixels.
///
/// Two rules make the arc honest at the ends, which is where a plain floor()
/// lies: any non-zero progress lights at least one pixel (so "started" is
/// visible immediately), and anything short of 100 % leaves at least one pixel
/// dark (so a full ring means DONE and nothing else).
inline std::size_t FilledCount(float percent, std::size_t led_count)
{
  if (led_count == 0u)
  {
    return 0u;
  }
  if (!std::isfinite(percent))
  {
    return 0u;
  }
  const float clamped = std::clamp(percent, 0.0f, 100.0f);
  auto filled = static_cast<std::size_t>(
      std::floor(static_cast<double>(clamped) / 100.0 * static_cast<double>(led_count)));
  filled = std::min(filled, led_count);

  if (clamped > 0.0f && filled == 0u)
  {
    filled = 1u;
  }
  if (clamped < 100.0f && filled == led_count)
  {
    filled = led_count - 1u;
  }
  return filled;
}

/// Which display mode the inputs select. See the priority list at the top.
inline LedMode SelectMode(const LedInputs& in, const LedPatternCfg& cfg)
{
  // emergency and state are only meaningful while the status is fresh --
  // a latched stale flag must not keep the ring red forever.
  if (in.status_fresh && in.emergency)
  {
    return LedMode::kEmergency;
  }
  if (in.is_charging)
  {
    return LedMode::kCharging;
  }
  if (!in.status_fresh)
  {
    return LedMode::kStale;
  }
  if (in.battery_valid && in.battery_percent < cfg.low_battery_percent)
  {
    return LedMode::kLowBattery;
  }

  switch (in.state)
  {
    case HighLevelState::kAutonomous:
      if (in.transiting)
      {
        return LedMode::kTransit;
      }
      return in.rtk_fixed ? LedMode::kMowing : LedMode::kMowingDegraded;
    case HighLevelState::kRecording:
      return LedMode::kRecording;
    case HighLevelState::kManualMowing:
      return LedMode::kManual;
    case HighLevelState::kIdle:
    case HighLevelState::kNull:
    default:
      return LedMode::kIdle;
  }
}

namespace detail
{

/// Progress arc with a head pixel, used by both the mowing and charging modes.
inline void PaintArc(std::vector<Rgb>& pixels, std::size_t filled, const Rgb& body, const Rgb& head)
{
  for (std::size_t i = 0; i < filled && i < pixels.size(); ++i)
  {
    pixels[i] = body;
  }
  if (filled > 0u && filled <= pixels.size())
  {
    pixels[filled - 1u] = head;
  }
}

/// Rotating head with a two-pixel fading tail, trailing BEHIND the head so the
/// direction of travel is unambiguous.
inline void PaintComet(std::vector<Rgb>& pixels, std::size_t head_index, const Rgb& color)
{
  const std::size_t count = pixels.size();
  if (count == 0u)
  {
    return;
  }
  constexpr float kTail[] = {1.0f, 0.35f, 0.12f};
  for (std::size_t offset = 0; offset < 3u && offset < count; ++offset)
  {
    const std::size_t index = (head_index + count - offset) % count;
    pixels[index] = Dim(color, kTail[offset]);
  }
}

/// Overwrite `indicator_count` pixels, spaced as evenly as integer indices
/// allow around the ring (index i*count/indicator_count), with `color`.
/// Used to keep a handful of "still on" indicator pixels lit against an
/// otherwise dimmed background -- readable from any angle, not just one
/// side of the ring. `indicator_count` is clamped to `pixels.size()`;
/// count 0 or an empty ring is a no-op.
inline void PaintEvenIndicators(std::vector<Rgb>& pixels,
                                std::size_t indicator_count,
                                const Rgb& color)
{
  const std::size_t count = pixels.size();
  if (count == 0u)
  {
    return;
  }
  indicator_count = std::min(indicator_count, count);
  for (std::size_t i = 0; i < indicator_count; ++i)
  {
    pixels[(i * count) / indicator_count] = color;
  }
}

/// Overwrite the pixels named in `ids` with `color`. Any index outside
/// [0, pixels.size()) is silently skipped -- e.g. an operator-entered ID
/// left stale after led_count was lowered.
inline void PaintIndicatorIds(std::vector<Rgb>& pixels,
                              const std::vector<std::size_t>& ids,
                              const Rgb& color)
{
  for (const std::size_t idx : ids)
  {
    if (idx < pixels.size())
    {
      pixels[idx] = color;
    }
  }
}

}  // namespace detail

/// Render the frame the ring should currently show.
inline std::vector<Rgb> RenderFrame(const LedInputs& in, const LedPatternCfg& cfg)
{
  std::vector<Rgb> pixels(cfg.led_count, colors::kOff);
  if (cfg.led_count == 0u)
  {
    return pixels;
  }

  switch (SelectMode(in, cfg))
  {
    case LedMode::kEmergency:
      std::fill(pixels.begin(), pixels.end(), colors::kRed);
      break;

    case LedMode::kCharging:
    {
      if (!in.battery_valid)
      {
        // Charging, level unknown -- the behavior tree is the only thing that
        // derives a percent from pack voltage (batteryPercentFromVoltage plus
        // its transient filter), and duplicating that curve here to fill an
        // arc would be a second source of truth for the SoC. A green comet
        // says "charging" without claiming a level it does not have.
        detail::PaintComet(pixels, RotationIndex(in.now_s, 1.5, cfg.led_count), colors::kGreen);
        break;
      }
      if (in.battery_percent >= cfg.charge_full_percent)
      {
        const bool gone_dim = cfg.charge_complete_timeout_s > 0.0 &&
                              in.charge_complete_elapsed_s >= cfg.charge_complete_timeout_s;
        if (!gone_dim)
        {
          std::fill(pixels.begin(), pixels.end(), colors::kGreen);
          break;
        }
        std::fill(pixels.begin(), pixels.end(), Dim(colors::kGreen, cfg.charge_complete_dim_scale));
        const Rgb indicator = Dim(colors::kGreen, cfg.charge_complete_indicator_scale);
        if (!cfg.charge_complete_indicator_ids.empty())
        {
          detail::PaintIndicatorIds(pixels, cfg.charge_complete_indicator_ids, indicator);
        }
        else
        {
          detail::PaintEvenIndicators(pixels, cfg.charge_complete_indicator_count, indicator);
        }
        break;
      }
      const Rgb body = Dim(colors::kGreen, Breathe(in.now_s, 3.0, 0.25f, 1.0f));
      detail::PaintArc(pixels, FilledCount(in.battery_percent, cfg.led_count), body, body);
      break;
    }

    case LedMode::kStale:
      detail::PaintComet(pixels, RotationIndex(in.now_s, 1.5, cfg.led_count), colors::kAmber);
      break;

    case LedMode::kLowBattery:
      if (BlinkOn(in.now_s, 1.0))
      {
        std::fill(pixels.begin(), pixels.end(), colors::kRed);
      }
      break;

    case LedMode::kMowing:
      detail::PaintArc(pixels,
                       FilledCount(in.coverage_percent, cfg.led_count),
                       colors::kGreen,
                       colors::kWhite);
      break;

    case LedMode::kMowingDegraded:
    {
      const Rgb head = BlinkOn(in.now_s, 0.5) ? colors::kWhite : colors::kAmber;
      detail::PaintArc(pixels,
                       FilledCount(in.coverage_percent, cfg.led_count),
                       colors::kAmber,
                       head);
      break;
    }

    case LedMode::kTransit:
    {
      // Alternating halves, ~1 s each -- deliberately NOT a comet (nothing
      // rotates around the ring) and NOT a whole-ring blink (that is LOW
      // BATTERY): exactly half the ring is lit at any instant, and which
      // half flips. Integer division on an odd led_count hands the extra
      // pixel to the second half; both halves still light at the
      // configured led_count, nothing hardcoded.
      const std::size_t half = pixels.size() / 2u;
      if (BlinkOn(in.now_s, 1.0))
      {
        std::fill(pixels.begin(),
                  pixels.begin() + static_cast<std::ptrdiff_t>(half),
                  colors::kLightOrange);
      }
      else
      {
        std::fill(pixels.begin() + static_cast<std::ptrdiff_t>(half),
                  pixels.end(),
                  colors::kLightOrange);
      }
      break;
    }

    case LedMode::kRecording:
      detail::PaintComet(pixels, RotationIndex(in.now_s, 2.0, cfg.led_count), colors::kCyan);
      break;

    case LedMode::kManual:
      std::fill(pixels.begin(),
                pixels.end(),
                Dim(colors::kPurple, Breathe(in.now_s, 2.0, 0.35f, 1.0f)));
      break;

    case LedMode::kIdle:
    default:
      std::fill(pixels.begin(), pixels.end(), Dim(colors::kWhite, cfg.idle_scale));
      break;
  }

  return pixels;
}

}  // namespace mowgli_leds
