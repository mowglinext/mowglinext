// SPDX-License-Identifier: GPL-3.0
/**
 * @file dock_antenna_capture.hpp
 * @brief Raw-antenna dock position capture — pure math + validity, no ROS deps.
 *
 * A dock POSITION may only ever come from the RAW GPS antenna position
 * (/gps/fix projected with the datum). While the robot charges, fusion_graph
 * gauge-resets the fused pose onto the STORED dock pose, and /gps/absolute_pose
 * / /gps/pose_cov are lever-arm-corrected with that pinned yaw — anything
 * derived from them on the dock is circular and just re-saves the old value.
 *
 * The antenna is NOT the robot: base_footprint sits one lever arm behind it
 * along the chassis heading. The heading is only measurable by driving OFF the
 * dock, the antenna-on-the-dock only while seated ON it. So the two halves are
 * taken at different moments and joined here:
 *
 *   1. on the dock, charging, RTK-Fixed  -> PendingAntennaCapture (mean ENU,
 *      yaw-free, held in map_server memory, never persisted, expires);
 *   2. off the dock, fresh motion yaw    -> DockBaseFromAntenna(capture, yaw)
 *      gives the base_footprint dock position, written together with the yaw.
 */

#pragma once

#include <cmath>
#include <cstddef>

namespace mowgli_map
{

struct Enu
{
  double east{0.0};
  double north{0.0};
};

/**
 * @brief base_footprint position from the antenna position and chassis yaw.
 *
 *   antenna_enu = base_enu + R(yaw) * lever_arm_body
 *   => base_enu = antenna_enu - R(yaw) * lever_arm_body
 *
 * `lever_arm_*` is the antenna position IN the base_footprint frame
 * (TF base_footprint -> gps_link translation; +x = antenna AHEAD of the base).
 * Applied exactly ONCE: the input must be the RAW antenna position, never an
 * already lever-arm-corrected one (a second application puts the dock one
 * lever arm SHORT of the contacts).
 */
inline Enu DockBaseFromAntenna(const Enu& antenna,
                               double yaw_rad,
                               double lever_arm_x,
                               double lever_arm_y)
{
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);
  Enu base;
  base.east = antenna.east - (cos_yaw * lever_arm_x - sin_yaw * lever_arm_y);
  base.north = antenna.north - (sin_yaw * lever_arm_x + cos_yaw * lever_arm_y);
  return base;
}

/// Averaged raw antenna position taken on the dock, waiting for a yaw.
struct PendingAntennaCapture
{
  bool valid{false};
  Enu antenna{};
  std::size_t sample_count{0};
  double stamp_s{0.0};  ///< capture time, seconds (node clock)
};

enum class PendingAntennaState
{
  USABLE,
  ABSENT,  ///< never captured, or already consumed by a successful write
  EXPIRED,  ///< older than the TTL — the robot may have been moved since
};

inline PendingAntennaState ClassifyPendingAntenna(const PendingAntennaCapture& capture,
                                                  double now_s,
                                                  double ttl_s)
{
  if (!capture.valid)
  {
    return PendingAntennaState::ABSENT;
  }
  const double age_s = now_s - capture.stamp_s;
  if (age_s < 0.0 || age_s > ttl_s)
  {
    return PendingAntennaState::EXPIRED;
  }
  return PendingAntennaState::USABLE;
}

}  // namespace mowgli_map
