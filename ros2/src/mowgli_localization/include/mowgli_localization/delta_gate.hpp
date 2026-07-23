// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
/**
 * @file delta_gate.hpp
 * @brief Δ (map→odom yaw) stabilisation gate for gps_dock_detection.
 *
 * Folds successive Δ measurements (from the live map→odom TF, or the legacy COG
 * estimator) into one smoothed offset while guarding TWO failure modes:
 *
 *   1. CORRUPT-RELOAD STEP / COG REVERSE-FLIP — a lone large jump is rejected,
 *      holding the last good Δ; survivors within the jump band are EMA-smoothed.
 *
 *   2. STALE LATCH (issue #390) — a jump gate that only ever rejects will hold a
 *      stale baseline FOREVER once it starts rejecting: after the docked IMU/mag
 *      recal slewed the held Δ ~115° off truth, every correct sample thereafter
 *      looked like a >max_jump "jump" and was rejected indefinitely (the same
 *      "hold forever once it starts rejecting" trap as the reverted
 *      GnssMobileGate). The fix: rejected samples are accumulated into a
 *      candidate; if they stay MUTUALLY consistent (each within the jump band of
 *      the running candidate) for `reseed_after_s`, the held baseline — not the
 *      samples — is the outlier, so Δ is re-seeded to the settled candidate. A
 *      flapping transient keeps breaking the candidate and restarting the run, so
 *      it never accumulates the sustained window and a glitch cannot re-latch.
 *
 * Pure / header-only and clock-agnostic (the caller passes a monotonic seconds
 * timestamp), so it unit-tests without ROS. See gps_dock_detection_node.cpp.
 */
#ifndef MOWGLI_LOCALIZATION__DELTA_GATE_HPP_
#define MOWGLI_LOCALIZATION__DELTA_GATE_HPP_

#include <cmath>

namespace mowgli_localization
{

/// Wrap an angle (radians) to (-pi, pi].
inline double wrap_to_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a <= -M_PI)
    a += 2.0 * M_PI;
  return a;
}

struct DeltaGateConfig
{
  /// Reject a step larger than this (rad) — guards a corrupt-reload TF step /
  /// COG reverse-flip.
  double max_jump_rad{25.0 * M_PI / 180.0};
  /// EMA weight applied to surviving jumps AND to the reject candidate.
  double ema_alpha{0.15};
  /// Sustained window (s) of MUTUALLY consistent rejected samples before Δ is
  /// re-seeded to the candidate. <= 0 disables the re-seed (legacy: hold forever).
  double reseed_after_s{5.0};
};

/// Outcome of folding one measurement.
enum class DeltaGateAction
{
  SEEDED,    ///< First sample: Δ set directly.
  ACCEPTED,  ///< Within the jump band: EMA-smoothed into Δ.
  REJECTED,  ///< Jump too large: held the baseline (candidate run accumulating).
  RESEEDED   ///< Sustained consistent rejection: Δ jumped to the new candidate.
};

class DeltaGate
{
public:
  DeltaGate() = default;
  explicit DeltaGate(const DeltaGateConfig& cfg) : cfg_(cfg) {}

  /// Fold one Δ measurement sampled at monotonic time `t_s` (seconds).
  DeltaGateAction update(double delta, double t_s)
  {
    delta = wrap_to_pi(delta);

    if (!have_offset_)
    {
      offset_ = delta;
      have_offset_ = true;
      have_candidate_ = false;
      last_jump_ = 0.0;
      return DeltaGateAction::SEEDED;
    }

    const double jump = wrap_to_pi(delta - offset_);
    last_jump_ = jump;

    if (std::abs(jump) <= cfg_.max_jump_rad)
    {
      // Accepted → the baseline is tracking truth; abandon any pending run.
      have_candidate_ = false;
      offset_ = wrap_to_pi(offset_ + cfg_.ema_alpha * jump);
      return DeltaGateAction::ACCEPTED;
    }

    // Rejected. Accumulate a consistent-candidate run so a stale baseline cannot
    // latch forever (issue #390).
    if (cfg_.reseed_after_s > 0.0)
    {
      if (have_candidate_)
      {
        const double cjump = wrap_to_pi(delta - candidate_);
        if (std::abs(cjump) <= cfg_.max_jump_rad)
        {
          candidate_ = wrap_to_pi(candidate_ + cfg_.ema_alpha * cjump);
          if (t_s - candidate_start_s_ >= cfg_.reseed_after_s)
          {
            offset_ = candidate_;
            have_candidate_ = false;
            return DeltaGateAction::RESEEDED;
          }
          return DeltaGateAction::REJECTED;
        }
      }
      // First rejection, or the sample broke from the candidate → (re)start.
      have_candidate_ = true;
      candidate_ = delta;
      candidate_start_s_ = t_s;
    }
    return DeltaGateAction::REJECTED;
  }

  bool has_offset() const { return have_offset_; }
  double value() const { return offset_; }
  double last_jump_rad() const { return last_jump_; }
  bool has_candidate() const { return have_candidate_; }
  double candidate_rad() const { return candidate_; }

private:
  DeltaGateConfig cfg_{};
  bool have_offset_{false};
  double offset_{0.0};
  double last_jump_{0.0};

  bool have_candidate_{false};
  double candidate_{0.0};
  double candidate_start_s_{0.0};
};

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__DELTA_GATE_HPP_
