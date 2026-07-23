// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// dock_cog_gate.hpp
//
// Pure (ROS-free) COG-coherence gate for the one-click dock calibration
// (dock_calibration_node). During the straight, slow reverse leg off the
// dock the node collects the body-heading samples published on
// /imu/cog_heading and the net GPS displacement, then calls
// evaluate_dock_cog_gate() to decide whether the reverse produced a
// trustworthy dock heading.
//
// RESOLUTION A (maintainer, 2026-07-20): /imu/cog_heading already carries the
// lever-arm-corrected, reverse-aware CHASSIS/BODY heading (see
// cog_to_imu_node.cpp:compute_cog_body_yaw + cog_yaw_math.hpp). During the
// undock reverse that body heading already equals the dock-facing heading, so
//   dock_yaw = circular_mean(/imu/cog_heading samples)   —  NO +pi.
// (The naive "+pi" only applies to RAW course-over-ground, which this topic is
// NOT. The topic name "cog_heading" is the trap — its content is body heading.)
//
// The coherence gate cross-checks that heading against the INDEPENDENT GPS
// travel bearing: in reverse, travel_bearing = body_heading - pi, so the
// expected body heading is wrap(atan2(dy, dx) + pi). A large mismatch means
// the RTK was not truly fixed / the GPS was noisy → reject and ask the
// operator to retry. Kept ROS-free so the algebra is unit-tested in isolation
// (test/test_dock_cog_gate.cpp), mirroring cog_yaw_math.hpp.

#ifndef MOWGLI_LOCALIZATION__DOCK_COG_GATE_HPP_
#define MOWGLI_LOCALIZATION__DOCK_COG_GATE_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mowgli_localization/cog_yaw_math.hpp"  // wrap_angle

namespace mowgli_localization
{

// Why the gate rejected the reverse leg (0 == accepted). Mapped by the node to
// the CalibrateDock action's RETRY_* result codes; kept as a local enum so
// this pure header does not depend on the generated action interface.
enum class DockCogReason : std::uint8_t
{
  OK = 0,
  INSUFFICIENT_DISPLACEMENT = 1,  // reverse leg too short to fit a heading
  INSUFFICIENT_SAMPLES = 2,  // too few valid /imu/cog_heading samples
  HIGH_STD = 3,  // COG scattered — RTK not truly fixed / noisy
  BEARING_MISMATCH = 4,  // COG mean disagrees with the GPS travel bearing
};

struct DockCogGateResult
{
  bool coherent = false;
  DockCogReason reason = DockCogReason::INSUFFICIENT_SAMPLES;
  double dock_yaw_rad = 0.0;  // circular_mean(headings) — Resolution A, NO +pi
  double cog_std_rad = 0.0;  // circular std of the heading samples
  double bearing_err_rad = 0.0;  // |mean_heading − expected_heading_from_gps|
  double displacement_m = 0.0;  // net GPS displacement of the reverse leg
};

// Circular mean of angles (rad), wrapped to (-pi, pi]. Returns 0 for an empty
// input (caller gates on sample count separately).
inline double circular_mean(const std::vector<double>& angles)
{
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const double a : angles)
  {
    sum_cos += std::cos(a);
    sum_sin += std::sin(a);
  }
  if (sum_cos == 0.0 && sum_sin == 0.0)
  {
    return 0.0;
  }
  return std::atan2(sum_sin, sum_cos);
}

// Circular standard deviation = sqrt(-2 ln R), where R is the mean resultant
// length. 0 for identical angles; grows without the ±pi wrap pathology of a
// linear std. Returns 0 for fewer than two samples.
inline double circular_std(const std::vector<double>& angles)
{
  const std::size_t n = angles.size();
  if (n < 2u)
  {
    return 0.0;
  }
  double sum_cos = 0.0;
  double sum_sin = 0.0;
  for (const double a : angles)
  {
    sum_cos += std::cos(a);
    sum_sin += std::sin(a);
  }
  const double resultant = std::hypot(sum_cos, sum_sin) / static_cast<double>(n);
  // Clamp to (0, 1]; R can exceed 1 only by FP rounding, and R→0 → std→∞.
  const double r = std::min(1.0, std::max(resultant, 1e-12));
  return std::sqrt(-2.0 * std::log(r));
}

// Evaluate the reverse-leg COG coherence gate (Resolution A).
//
//   heading_samples : /imu/cog_heading yaw values over the reverse leg
//                     (already body/chassis heading, reverse-aware — do NOT
//                     add pi).
//   gps_dx, gps_dy  : net GPS displacement of the reverse leg (end − start),
//                     map frame (ENU) metres.
//   min_samples            : minimum valid COG samples to trust the mean.
//   max_cog_std_rad        : reject if circular std exceeds this.
//   max_bearing_err_rad    : reject if |mean − expected_from_gps| exceeds this.
//   min_displacement_m     : reject if the reverse travelled less than this.
//
// On acceptance, dock_yaw_rad = circular_mean(heading_samples).
inline DockCogGateResult evaluate_dock_cog_gate(const std::vector<double>& heading_samples,
                                                double gps_dx,
                                                double gps_dy,
                                                std::size_t min_samples,
                                                double max_cog_std_rad,
                                                double max_bearing_err_rad,
                                                double min_displacement_m)
{
  DockCogGateResult res;
  res.displacement_m = std::hypot(gps_dx, gps_dy);
  res.cog_std_rad = circular_std(heading_samples);
  res.dock_yaw_rad = circular_mean(heading_samples);

  // (d) The reverse must have actually moved, else the travel bearing (and any
  // heading fit) is meaningless.
  if (res.displacement_m < min_displacement_m)
  {
    res.reason = DockCogReason::INSUFFICIENT_DISPLACEMENT;
    return res;
  }

  // (a) Enough valid COG samples. /imu/cog_heading only publishes when straight
  // + RTK-Fixed, so a healthy count is already strong evidence.
  if (heading_samples.size() < min_samples)
  {
    res.reason = DockCogReason::INSUFFICIENT_SAMPLES;
    return res;
  }

  // (b) The heading must be steady across the leg.
  if (res.cog_std_rad > max_cog_std_rad)
  {
    res.reason = DockCogReason::HIGH_STD;
    return res;
  }

  // (c) The COG mean must agree with the INDEPENDENT GPS travel bearing. In
  // reverse, travel_bearing = body_heading − pi, so the expected body heading
  // is wrap(atan2(dy, dx) + pi).
  const double travel_bearing = std::atan2(gps_dy, gps_dx);
  const double expected_heading = wrap_angle(travel_bearing + M_PI);
  res.bearing_err_rad = std::abs(wrap_angle(res.dock_yaw_rad - expected_heading));
  if (res.bearing_err_rad > max_bearing_err_rad)
  {
    res.reason = DockCogReason::BEARING_MISMATCH;
    return res;
  }

  res.reason = DockCogReason::OK;
  res.coherent = true;
  return res;
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__DOCK_COG_GATE_HPP_
