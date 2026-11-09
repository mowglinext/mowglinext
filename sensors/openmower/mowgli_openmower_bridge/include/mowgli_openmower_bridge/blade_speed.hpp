// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

namespace mowgli_openmower_bridge
{

// Status.mower_motor_rpm is a speed, never a direction: the Mowgli STM32
// reports it as a uint16, and map_server only stamps mow progress while it is
// >= mow_progress_min_blade_rpm. The direction travels separately
// (blade_requested_direction). Both xESC flavours must therefore report a
// non-negative shaft speed.

/// xESC mini (VESC): signed electrical rpm -> shaft speed.
[[nodiscard]] inline double ShaftRpmFromErpm(double erpm, int pole_pairs) noexcept
{
  if (!std::isfinite(erpm) || pole_pairs < 1)
  {
    return 0.0;
  }
  return std::abs(erpm) / static_cast<double>(pole_pairs);
}

/**
 * @brief xESC 2040: shaft speed from its hall tick counter.
 *
 * The 2040 reports no speed. Its firmware adds one to tacho_absolute per hall
 * state change (xESC2040 main.cpp), i.e. 6 per electrical revolution. Speed
 * is measured over at least kWindow so a 50 Hz status stream does not
 * quantise it. A counter that goes backwards or jumps beyond kMaxRpm (the
 * controller rebooted) re-anchors instead of reporting a spike.
 */
class HallSpeedMeter
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::chrono::milliseconds kWindow{200};
  static constexpr double kMaxRpm = 20000.0;
  static constexpr int kHallStepsPerElectricalRev = 6;

  explicit HallSpeedMeter(int pole_pairs) noexcept : pole_pairs_(pole_pairs < 1 ? 1 : pole_pairs)
  {
  }

  /// Feed one status sample; returns the current speed estimate (rpm, >= 0).
  double Update(uint32_t tacho_absolute, Clock::time_point now) noexcept
  {
    if (!have_anchor_)
    {
      Anchor(tacho_absolute, now);
      return rpm_;
    }
    const auto elapsed = now - anchor_time_;
    if (elapsed < kWindow)
    {
      return rpm_;
    }
    const double seconds = std::chrono::duration<double>(elapsed).count();
    const double revs = static_cast<double>(tacho_absolute - anchor_ticks_) /
                        static_cast<double>(kHallStepsPerElectricalRev * pole_pairs_);
    const double rpm = revs / seconds * 60.0;
    rpm_ = (tacho_absolute < anchor_ticks_ || rpm > kMaxRpm) ? 0.0 : rpm;
    Anchor(tacho_absolute, now);
    return rpm_;
  }

  /// Forget the anchor (port reopened): the next sample starts a new window.
  void Reset() noexcept
  {
    have_anchor_ = false;
    rpm_ = 0.0;
  }

  [[nodiscard]] double rpm() const noexcept
  {
    return rpm_;
  }

private:
  void Anchor(uint32_t ticks, Clock::time_point now) noexcept
  {
    anchor_ticks_ = ticks;
    anchor_time_ = now;
    have_anchor_ = true;
  }

  int pole_pairs_;
  bool have_anchor_{false};
  uint32_t anchor_ticks_{0u};
  Clock::time_point anchor_time_{};
  double rpm_{0.0};
};

}  // namespace mowgli_openmower_bridge
