// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// dock_persist_plan.hpp
//
// Pure (ROS-free) description of HOW the one-click dock calibration persists
// its result through map_server's ~/set_docking_point — the two requests it
// sends, and the operator-facing text for each way the sequence can end.
//
// The persistence is TWO steps because the two halves of a dock pose are only
// measurable in two different places:
//
//   YAW       is measured by REVERSING OFF the dock, so it exists when the
//             robot is ~1.5 m away from the charger. It is written right
//             there (YawStep), and it STAYS written whatever happens next —
//             docking_server only reads dock_pose at container startup, so a
//             same-session re-dock proves nothing about the new heading and
//             gating the write on it only ever discarded good measurements
//             (94f01b38).
//   POSITION  is averaged from raw GPS antenna samples and only means "the
//             dock" while the robot is SEATED on it, so it is captured after
//             the re-dock has been verified by the charger (PositionStep).
//
// 94f01b38 moved the write before the re-dock but kept ONE request doing both
// (use_gps_position=true, MOTION). map_server rejects every position capture
// while is_charging is false, so the calibration failed on every run. Keeping
// the field combinations here, under test, is what stops the two steps from
// being merged back into one.

#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace mowgli_localization
{

// Mirrors mowgli_interfaces/srv/SetDockingPoint yaw_source constants; pinned
// by a static_assert next to the service call in calibrate_imu_yaw_node.cpp.
inline constexpr uint8_t kSetDockYawPreserve = 0;
inline constexpr uint8_t kSetDockYawMotion = 2;

struct DockPersistRequest
{
  bool use_gps_position{false};
  bool preserve_position{false};
  uint8_t yaw_source{kSetDockYawPreserve};
  double yaw_rad{0.0};
};

/// Step 1 — robot OFF the dock: write the motion-derived yaw, keep stored X/Y.
inline DockPersistRequest DockYawStep(double dock_yaw_rad)
{
  DockPersistRequest req;
  req.use_gps_position = false;
  req.preserve_position = true;
  req.yaw_source = kSetDockYawMotion;
  req.yaw_rad = dock_yaw_rad;
  return req;
}

/// Step 2 — robot SEATED and charging: capture X/Y, keep the yaw from step 1.
inline DockPersistRequest DockPositionStep()
{
  DockPersistRequest req;
  req.use_gps_position = true;
  req.preserve_position = false;
  req.yaw_source = kSetDockYawPreserve;
  return req;
}

inline int DockYawDegrees(double dock_yaw_rad)
{
  return static_cast<int>(std::lround(dock_yaw_rad * 180.0 / M_PI));
}

/// Suffix for every failure that happens AFTER step 1 succeeded: the operator
/// must be told the yaw is already on disk and the position is not.
inline std::string DockYawSavedNote(double dock_yaw_rad)
{
  return " Dock yaw (" + std::to_string(DockYawDegrees(dock_yaw_rad)) +
         "°) was already saved; dock position NOT updated.";
}

/// Re-dock never reached the charger: yaw saved, position not, robot off-dock.
inline std::string DockRedockFailedMessage(double dock_yaw_rad)
{
  return "Yaw measured and saved (" + std::to_string(DockYawDegrees(dock_yaw_rad)) +
         "°), but the robot did NOT re-dock: dock position NOT updated, robot is "
         "NOT on the dock. docking_server only reads dock_pose at container "
         "startup, so it was still steering on the OLD heading. Restart "
         "mowgli-ros2 (Logs page → select it → Restart, or `docker restart "
         "mowgli-ros2`), dock the robot, then run this calibration again to "
         "capture the position.";
}

/// Re-docked and charging, but map_server refused the position capture.
inline std::string DockPositionFailedMessage(double dock_yaw_rad, const std::string& reason)
{
  return "Yaw measured and saved (" + std::to_string(DockYawDegrees(dock_yaw_rad)) +
         "°) and the robot re-docked, but the dock position was NOT updated: " + reason;
}

}  // namespace mowgli_localization
