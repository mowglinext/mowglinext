// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Dead-IMU detection. Pure logic, no ROS, so it is unit-testable standalone
// (see test_imu_liveness.cpp) — same shape as dig_detector.hpp / blade_gate.hpp.
//
// ── Why this exists (field-observed 2026-09-02) ─────────────────────────────
// After a warm reboot of the Orange Pi the STM32 kept streaming IMU packets
// whose accelerometer AND gyro were all exactly 0.0 — the WT901's bit-banged
// I²C bus had hung (a power cycle revived it). Nothing downstream noticed:
//
//   * hardware_bridge ran its on-dock calibration over 200 all-zero samples,
//     ACCEPTED it and PERSISTED it — offsets 0, covariances 0 — throwing away
//     the good gyro bias (gz ~ -0.056 rad/s). The persisted-cal load gate only
//     rejects |gyro offset| > 0.2 rad/s, so the zeros loaded fine at the next
//     restart too.
//   * /imu/data kept publishing zero accel + zero gyro at 90 Hz for the whole
//     mow. fusion_graph's slip veto fired 3021 times (gyro says 0, wheels say
//     turning), loop closures ran away, and the dig detector's turn exclusion
//     (dig_detector.hpp — which MUST read the gyro) was blind.
//
// Gravity is never zero. An accelerometer reporting |a| ~ 0 is a dead sensor,
// not a measurement — that single fact is what every check here rests on.
//
// ── What this header decides ────────────────────────────────────────────────
//   IsImuSampleDead        one sample: |accel| below any plausible gravity.
//                          Sufficient on its own — a mounting tilt cannot
//                          shrink |g| = 9.8 anywhere near the threshold, and
//                          requiring gyro == 0 as well would only let a
//                          half-dead bus through.
//   UpdateImuLiveness      debounced alive/dead state over consecutive dead
//                          samples (0.5 s at 90 Hz), back to alive on the
//                          FIRST live sample. Returns a new state.
//   IsCalibrationPlausible the completed at-rest calibration window: mean
//                          |accel| must look like gravity, and a sensor with
//                          exactly zero gyro noise across the whole window
//                          is not a sensor.
//   IsDeadSensorCovariance a persisted calibration file whose five variances
//                          are all exactly 0 was recorded from a dead sensor.
//
// This header only DETECTS. Gating mowing / the BT on it is a follow-up.

#ifndef MOWGLI_HARDWARE__IMU_LIVENESS_HPP_
#define MOWGLI_HARDWARE__IMU_LIVENESS_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_hardware
{

// Any |accel| below this is not gravity. g ~ 9.81 m/s²; no mounting tilt
// brings the measured magnitude anywhere near 3 (magnitude is tilt-invariant).
constexpr double kMinPlausibleAccelMps2 = 3.0;

// Consecutive dead samples before the tracker flips to `dead`: 0.5 s at the
// 90 Hz IMU packet rate. Short enough to catch a hang before a calibration
// window (200 samples) completes, long enough to ignore a single bad frame.
constexpr int kImuDeadSampleThreshold = 45;

// A single IMU sample is dead when its acceleration magnitude cannot be
// gravity. The gyro values are accepted for symmetry with the packet layout
// but deliberately NOT required to be zero (see header comment).
inline bool IsImuSampleDead(double ax,
                            double ay,
                            double az,
                            [[maybe_unused]] double gx,
                            [[maybe_unused]] double gy,
                            [[maybe_unused]] double gz)
{
  const double accel_mag = std::sqrt(ax * ax + ay * ay + az * az);
  return !(accel_mag >= kMinPlausibleAccelMps2);  // NaN counts as dead
}

struct ImuLivenessState
{
  int consecutive_dead{0};  // capped at kImuDeadSampleThreshold
  bool dead{false};
};

struct ImuLivenessUpdate
{
  ImuLivenessState state;
  bool became_dead{false};  // alive -> dead on this sample
  bool became_alive{false};  // dead -> alive on this sample
};

// Fold one sample into the liveness state. Never mutates `prev`.
inline ImuLivenessUpdate UpdateImuLiveness(const ImuLivenessState& prev, bool sample_dead)
{
  if (!sample_dead)
  {
    return ImuLivenessUpdate{ImuLivenessState{0, false}, false, prev.dead};
  }
  const int run = std::min(prev.consecutive_dead + 1, kImuDeadSampleThreshold);
  const bool dead = run >= kImuDeadSampleThreshold;
  return ImuLivenessUpdate{ImuLivenessState{run, dead}, dead && !prev.dead, false};
}

// A completed calibration window is plausible only if the mean acceleration
// looked like gravity AND at least one gyro axis showed some noise. Exactly
// zero variance on every gyro axis over the whole window means every sample
// was identical — a hung bus, not a quiet chip.
inline bool IsCalibrationPlausible(double mean_accel_magnitude,
                                   double cov_gx,
                                   double cov_gy,
                                   double cov_gz)
{
  if (!(mean_accel_magnitude >= kMinPlausibleAccelMps2))
  {
    return false;
  }
  const bool gyro_silent = cov_gx == 0.0 && cov_gy == 0.0 && cov_gz == 0.0;
  return !gyro_silent;
}

// Persisted-calibration gate: five variances that are all exactly 0 can only
// have been written from a dead sensor (see header comment).
inline bool IsDeadSensorCovariance(
    double cov_ax, double cov_ay, double cov_gx, double cov_gy, double cov_gz)
{
  return cov_ax == 0.0 && cov_ay == 0.0 && cov_gx == 0.0 && cov_gy == 0.0 && cov_gz == 0.0;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__IMU_LIVENESS_HPP_
