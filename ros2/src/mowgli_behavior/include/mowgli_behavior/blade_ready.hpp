// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace mowgli_behavior
{

struct BladeReadyConfig
{
  double min_rpm{1000.0};
  double fallback_sec{1.5};
  double timeout_sec{6.0};

  void validate() const
  {
    if (!std::isfinite(min_rpm) || min_rpm <= 0.0 || !std::isfinite(fallback_sec) ||
        fallback_sec <= 0.0 || !std::isfinite(timeout_sec) || timeout_sec <= fallback_sec ||
        timeout_sec <= kStableSec || timeout_sec > 30.0)
      throw std::invalid_argument(
          "blade-ready parameters require positive finite RPM/delay "
          "and a timeout <= 30s, longer than the fallback and stability windows");
  }

  static constexpr double kStableSec = 0.3;
  static constexpr double kMaxAgeSec = 1.0;
};

enum class BladeReadyResult
{
  kWaiting,
  kTelemetryReady,
  kTimerFallback,
  kFailed,
};

/// A dispatch barrier, not a replacement for firmware blade interlocks.
/// Source stamps identify direct blade observations; steady time bounds waiting.
/// Once any blade telemetry is known, a missing/stale/stalled stream must never
/// degrade to the legacy timer. A successful ON service response is necessary
/// for both paths, but does not itself prove that the blade is turning.
class BladeReadyGate
{
public:
  using Clock = std::chrono::steady_clock;

  void noteTelemetrySeen(bool seen)
  {
    telemetry_seen_ = telemetry_seen_ || seen;
  }

  void start(Clock::time_point now,
             int64_t request_stamp_ns,
             int64_t previous_stamp_ns,
             bool telemetry_seen)
  {
    start_ = now;
    request_stamp_ns_ = request_stamp_ns;
    last_stamp_ns_ = previous_stamp_ns;
    telemetry_seen_ = telemetry_seen || previous_stamp_ns > 0;
    good_since_.reset();
    accepted_since_.reset();
    last_observation_time_.reset();
  }

  BladeReadyResult step(const BladeReadyConfig& config,
                        Clock::time_point now,
                        int64_t ros_now_ns,
                        int64_t stamp_ns,
                        bool delivery_fresh,
                        bool requested,
                        bool active,
                        double rpm,
                        bool command_accepted)
  {
    telemetry_seen_ = telemetry_seen_ || stamp_ns > 0;
    const double elapsed = std::chrono::duration<double>(now - start_).count();
    if (elapsed >= config.timeout_sec)
      return BladeReadyResult::kFailed;
    if (!command_accepted)
    {
      good_since_.reset();
      accepted_since_.reset();
      return BladeReadyResult::kWaiting;
    }
    if (!accepted_since_)
      accepted_since_ = now;
    if (!telemetry_seen_)
      return std::chrono::duration<double>(now - *accepted_since_).count() >= config.fallback_sec
                 ? BladeReadyResult::kTimerFallback
                 : BladeReadyResult::kWaiting;

    const double source_age = static_cast<double>(ros_now_ns - stamp_ns) / 1e9;
    const bool good = stamp_ns > 0 && stamp_ns >= request_stamp_ns_ && source_age >= 0.0 &&
                      source_age <= BladeReadyConfig::kMaxAgeSec && delivery_fresh && requested &&
                      active && std::isfinite(rpm) && rpm >= config.min_rpm;
    const bool new_sample = stamp_ns > last_stamp_ns_;
    // General Status receipt can stay live while the identified blade
    // observation is frozen (including a paused ROS clock). A resumed stream
    // must establish a new stable window, rather than inherit that silence.
    if (last_observation_time_ &&
        std::chrono::duration<double>(now - *last_observation_time_).count() >
            BladeReadyConfig::kMaxAgeSec)
      good_since_.reset();
    if (new_sample)
    {
      last_stamp_ns_ = stamp_ns;
      last_observation_time_ = now;
    }
    if (!good)
    {
      good_since_.reset();
      return BladeReadyResult::kWaiting;
    }
    // Re-publications can keep /status alive, but cannot extend a physical
    // readiness observation. Require another distinct sample to finish stability.
    if (!new_sample)
      return BladeReadyResult::kWaiting;
    if (!good_since_)
      good_since_ = now;
    if (std::chrono::duration<double>(now - *good_since_).count() >= BladeReadyConfig::kStableSec)
      return BladeReadyResult::kTelemetryReady;
    return BladeReadyResult::kWaiting;
  }

private:
  Clock::time_point start_{};
  int64_t request_stamp_ns_{0};
  int64_t last_stamp_ns_{0};
  bool telemetry_seen_{false};
  std::optional<Clock::time_point> good_since_;
  std::optional<Clock::time_point> accepted_since_;
  std::optional<Clock::time_point> last_observation_time_;
};

}  // namespace mowgli_behavior
