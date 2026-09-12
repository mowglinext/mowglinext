// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace fusion_graph
{
struct LidarComputeGateParams
{
  double max_rate_hz = 5.0;
  double calibration_period_s = 60.0;
  double calibration_burst_s = 10.0;
  double engage_age_s = 1.0;
  double apply_age_s = 20.0;
  double warmup_s = 5.0;
};

struct LidarComputeDecision
{
  bool run = false;
  bool reseed = false;
};

// Compute scheduling only: observation validation and application gates remain
// authoritative. Call with ROS time; invalid scans never spend the compute budget.
class LidarComputeGate
{
public:
  explicit LidarComputeGate(LidarComputeGateParams params = {}) : p_(params)
  {
  }

  bool ValidConfiguration() const
  {
    return ValidParams();
  }

  void Reset()
  {
    epoch_.reset();
    last_now_.reset();
    last_run_.reset();
    last_phase_ = Phase::kIdle;
    last_burst_ = -1.0;
  }

  LidarComputeDecision Step(double now_s,
                            double usable_gnss_age_s,
                            bool map_has_structure,
                            bool valid_scan,
                            bool shadow_mode = false,
                            bool calibrate = true)
  {
    if (!std::isfinite(now_s) || now_s < 0.0 || !std::isfinite(usable_gnss_age_s) ||
        usable_gnss_age_s < 0.0 || !ValidParams())
      return {};
    if (last_now_ && now_s < *last_now_)
      Reset();
    last_now_ = now_s;
    if (!map_has_structure)
    {
      last_phase_ = Phase::kIdle;
      return {};
    }
    if (!epoch_)
    {
      if (!valid_scan)
        return {};
      epoch_ = now_s;
    }

    const double elapsed = now_s - *epoch_;
    const double burst = std::floor(elapsed / p_.calibration_period_s);
    Phase phase = Phase::kIdle;
    if (usable_gnss_age_s <= p_.engage_age_s)
    {
      if (shadow_mode)
        phase = Phase::kShadow;
      else if (calibrate && std::fmod(elapsed, p_.calibration_period_s) < p_.calibration_burst_s)
        phase = Phase::kCalibration;
    }
    else if (usable_gnss_age_s >= std::max(p_.engage_age_s, p_.apply_age_s - p_.warmup_s))
      phase = Phase::kAnchoring;

    if (phase == Phase::kIdle)
    {
      last_phase_ = phase;
      return {};
    }
    const double period = 1.0 / p_.max_rate_hz;
    if (!valid_scan || (last_run_ && now_s - *last_run_ < period))
      return {};
    const bool reseed = !last_run_ || phase != last_phase_ ||
                        (phase == Phase::kCalibration && burst != last_burst_) ||
                        now_s - *last_run_ > std::max(0.5, 2.0 * period);
    last_run_ = now_s;
    last_phase_ = phase;
    last_burst_ = burst;
    return {true, reseed};
  }

private:
  enum class Phase
  {
    kIdle,
    kCalibration,
    kShadow,
    kAnchoring
  };
  bool ValidParams() const
  {
    return std::isfinite(p_.max_rate_hz) && p_.max_rate_hz > 0.0 &&
           std::isfinite(p_.calibration_period_s) && p_.calibration_period_s > 0.0 &&
           std::isfinite(p_.calibration_burst_s) && p_.calibration_burst_s >= 0.0 &&
           p_.calibration_burst_s <= p_.calibration_period_s && std::isfinite(p_.engage_age_s) &&
           p_.engage_age_s >= 0.0 && std::isfinite(p_.apply_age_s) && p_.apply_age_s >= 0.0 &&
           std::isfinite(p_.warmup_s) && p_.warmup_s >= 0.0;
  }
  LidarComputeGateParams p_;
  std::optional<double> epoch_;
  std::optional<double> last_now_;
  std::optional<double> last_run_;
  Phase last_phase_ = Phase::kIdle;
  double last_burst_ = -1.0;
};
}  // namespace fusion_graph
