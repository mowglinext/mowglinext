// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// dock_persist_plan.hpp
//
// Pure (ROS-free) description of HOW the dock calibration persists its result
// through map_server — the requests it sends, and the verdict + operator text
// for each way the sequence can end.
//
// The two halves of a dock pose are only measurable in two different places:
//
//   POSITION  the RAW GPS antenna, averaged while the robot is SEATED on the
//             dock, charging and RTK-Fixed. The routine starts in exactly that
//             state (it waits for RTK-Fixed on the dock before it moves), so
//             the antenna is captured THERE — map_server's
//             ~/capture_dock_antenna — and held in map_server's memory.
//   YAW       measured by REVERSING OFF the dock, so it exists when the robot
//             is ~1.5 m away from the charger.
//
// They are joined in ONE write after the reverse leg (DockPoseStep): map_server
// lever-arm-corrects the pending antenna with the fresh yaw and stores x, y and
// yaw together. If no capture is pending, the yaw alone is written and the
// stored position kept (DockYawStep).
//
// History, so the two earlier orderings are not re-tried:
//   - persist everything AFTER the verified re-dock: docking_server only reads
//     dock_pose at container startup, so the same-session re-dock steers on
//     the OLD pose and proves nothing about the new one; gating on it only
//     discarded good measurements (94f01b38).
//   - yaw off the dock, position after the re-dock (first version of this
//     header): circular. A wrong stored POSITION makes the re-dock stop short
//     of the contacts, the robot never charges, and the position capture that
//     would have fixed it is never reached (field test 2026-09-17: 0.35 m
//     short, three "timed out waiting for charge"; and when pushed onto the
//     contacts by hand the receiver sat at RTK-Float under the dock, 0 usable
//     samples). The re-dock is therefore a pure CONFIRMATION pass now: nothing
//     is measured or saved after it.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace mowgli_localization
{

// Mirrors mowgli_interfaces/srv/SetDockingPoint yaw_source constants; pinned
// by a static_assert next to the service call in calibrate_imu_yaw_node.cpp.
inline constexpr uint8_t kSetDockYawPreserve = 0;
inline constexpr uint8_t kSetDockYawMotion = 2;

// Mirrors CalibrateDock.action RETRY_* (same static_assert pinning).
inline constexpr uint8_t kDockRetryNone = 0;
inline constexpr uint8_t kDockRetryNoChargeOnRedock = 4;
inline constexpr uint8_t kDockRetryPersistFailed = 7;

struct DockPersistRequest
{
  bool use_gps_position{false};
  bool preserve_position{false};
  bool use_pending_antenna{false};
  uint8_t yaw_source{kSetDockYawPreserve};
  double yaw_rad{0.0};
};

/// Normal write — robot OFF the dock: motion yaw + the antenna captured ON it.
inline DockPersistRequest DockPoseStep(double dock_yaw_rad)
{
  DockPersistRequest req;
  req.use_pending_antenna = true;
  req.yaw_source = kSetDockYawMotion;
  req.yaw_rad = dock_yaw_rad;
  return req;
}

/// Fallback write — no usable capture: motion yaw only, stored X/Y kept.
inline DockPersistRequest DockYawStep(double dock_yaw_rad)
{
  DockPersistRequest req;
  req.preserve_position = true;
  req.yaw_source = kSetDockYawMotion;
  req.yaw_rad = dock_yaw_rad;
  return req;
}

/// What actually reached mowgli_robot.yaml.
enum class DockSaved : uint8_t
{
  NOTHING,
  YAW_ONLY,
  YAW_AND_POSITION,
};

struct DockSavedPose
{
  DockSaved saved{DockSaved::NOTHING};
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
  /// Why the position was not saved (only meaningful for YAW_ONLY).
  std::string position_skip_reason;
};

struct DockVerdict
{
  bool success{false};
  uint8_t retry_reason{kDockRetryNone};
  std::string message;
};

inline int DockYawDegrees(double dock_yaw_rad)
{
  return static_cast<int>(std::lround(dock_yaw_rad * 180.0 / M_PI));
}

/// "yaw -54°, position (6.263, 2.811)" / "yaw -54° ONLY" — exactly what was saved.
inline std::string DescribeDockSaved(const DockSavedPose& pose)
{
  char buf[160];
  switch (pose.saved)
  {
    case DockSaved::YAW_AND_POSITION:
      std::snprintf(buf,
                    sizeof(buf),
                    "yaw %d° and position (%.3f, %.3f)",
                    DockYawDegrees(pose.yaw_rad),
                    pose.x,
                    pose.y);
      return buf;
    case DockSaved::YAW_ONLY:
      std::snprintf(buf, sizeof(buf), "yaw %d° ONLY", DockYawDegrees(pose.yaw_rad));
      return buf;
    case DockSaved::NOTHING:
    default:
      return "nothing";
  }
}

/// Appended to every outcome that saved something: the saved pose is not what
/// the running stack steers by.
inline const char* DockRestartNote()
{
  return " docking_server and gps_dock_detection only load the dock pose at stack startup: "
         "restart mowgli-ros2 (Logs page → select it → Restart, or `docker restart "
         "mowgli-ros2`) for docking to use the new pose.";
}

/// Suffix for an ABORT (cancel / emergency / BT refusal) after the write.
inline std::string DockSavedNote(const DockSavedPose& pose)
{
  return " Already saved: " + DescribeDockSaved(pose) + ". The robot is NOT on the dock." +
         DockRestartNote();
}

/**
 * @brief Verdict once the persistence and the confirmation re-dock are over.
 *
 * SUCCESS means "the dock pose — yaw AND position — is saved". That is the
 * calibration's product. Whether the confirmation re-dock reached the charger
 * is reported, loudly, but does not decide success: inside this run the
 * docking stack still steers on the OLD pose (loaded at startup), so with a
 * wrong old position it stops short of the contacts BY CONSTRUCTION — calling
 * that a calibration failure told the operator to retry a calibration that had
 * worked, and could never converge. retry_reason stays NO_CHARGE_ON_REDOCK on
 * such a success so the GUI can render it as a warning, and the message says
 * first and unambiguously that the robot is NOT on the dock.
 *
 * YAW_ONLY is reported as a failure (position was not updated — the operator
 * should run it again), with the yaw explicitly listed as saved.
 */
inline DockVerdict DecideDockVerdict(const DockSavedPose& pose, bool redock_verified)
{
  DockVerdict v;
  const std::string saved = DescribeDockSaved(pose);
  if (pose.saved == DockSaved::YAW_AND_POSITION)
  {
    v.success = true;
    if (redock_verified)
    {
      v.retry_reason = kDockRetryNone;
      v.message = "Dock calibrated: " + saved + " saved; re-dock verified, robot is charging." +
                  DockRestartNote();
    }
    else
    {
      v.retry_reason = kDockRetryNoChargeOnRedock;
      v.message = "Dock calibrated: " + saved +
                  " saved. BUT the robot is NOT on the dock: the confirmation re-dock did "
                  "not reach the charger. That is expected when the OLD dock pose was wrong "
                  "— this run's re-dock still steers on it — and is not a calibration "
                  "failure." +
                  DockRestartNote() + " Then send HOME (or place the robot on the dock).";
    }
    return v;
  }
  if (pose.saved == DockSaved::YAW_ONLY)
  {
    v.success = false;
    v.retry_reason = redock_verified ? kDockRetryPersistFailed : kDockRetryNoChargeOnRedock;
    v.message = "Partial: " + saved + " saved, dock position NOT updated (" +
                pose.position_skip_reason + "). " +
                (redock_verified ? "The robot re-docked and is charging."
                                 : "The robot is NOT on the dock (re-dock did not reach the "
                                   "charger).") +
                DockRestartNote() + " Then run the calibration again to capture the position.";
    return v;
  }
  v.success = false;
  v.retry_reason = kDockRetryPersistFailed;
  v.message = "Nothing was saved.";
  return v;
}

}  // namespace mowgli_localization
