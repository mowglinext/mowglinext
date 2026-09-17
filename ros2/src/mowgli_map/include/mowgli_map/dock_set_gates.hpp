// SPDX-License-Identifier: GPL-3.0
/**
 * @file dock_set_gates.hpp
 * @brief Which gates a SetDockingPoint request must pass — pure, no ROS deps.
 *
 * `map_server_node`'s `~/set_docking_point` serves three different kinds of
 * write, and they do NOT need the same protection:
 *
 *   POSITION_CAPTURE  use_gps_position=true. X/Y is averaged from the raw GPS
 *                     antenna samples, so the robot MUST be seated on the dock
 *                     (is_charging) or the average is a position somewhere on
 *                     the lawn. All gates apply.
 *   MANUAL            use_gps_position=false, preserve_position=false. The
 *                     operator typed / dragged the pose. All gates apply,
 *                     exactly as before this header existed.
 *   YAW_ONLY_MOTION   preserve_position=true, yaw_source=MOTION. The one-click
 *                     dock calibration writes the heading it measured while
 *                     REVERSING OFF the dock. By construction the robot is
 *                     ~1.5 m away from the charger when the measurement
 *                     exists, so the charging gate can never be satisfied —
 *                     requiring it made the calibration fail deterministically
 *                     (field-diagnosed 2026-09-17). The stored X/Y is kept, so
 *                     nothing position-related is at stake.
 *
 * Extracted so the decision is unit-testable without spinning a node, and so
 * the charging exemption cannot silently widen: it is reachable through ONE
 * exact field combination and nothing else.
 */

#pragma once

#include <cstdint>

namespace mowgli_map
{

// Mirrors mowgli_interfaces/srv/SetDockingPoint yaw_source constants. Kept as
// plain integers so this header stays free of generated-message includes; a
// static_assert in area_manager.cpp pins them to the .srv values.
inline constexpr uint8_t kDockYawSourcePreserve = 0;
inline constexpr uint8_t kDockYawSourceRequest = 1;
inline constexpr uint8_t kDockYawSourceMotion = 2;

enum class DockSetKind : uint8_t
{
  POSITION_CAPTURE,
  MANUAL,
  YAW_ONLY_MOTION,
  INVALID,
};

struct DockSetGates
{
  DockSetKind kind{DockSetKind::INVALID};
  /// Non-null only when kind == INVALID: why the field combination is refused.
  const char* invalid_reason{nullptr};
  /// Gate (1): firmware reports is_charging (robot physically on the dock).
  bool require_charging{true};
  /// Gate (2): fresh /gps/pose_cov with sigma under dock_set_gps_accuracy_max_m.
  bool require_gps_accuracy{true};
  /// Gate (3): the FUSED yaw is quiet over the rolling window.
  bool require_yaw_convergence{true};
  /// A dock position must already be stored (there is one to preserve).
  bool require_existing_pose{false};
};

/**
 * @brief Classify a request and return the gates it must pass.
 *
 * YAW_ONLY_MOTION drops exactly two gates, each for a stated reason:
 *   - charging: the yaw is motion-derived, the robot is necessarily off the
 *     dock (see file comment).
 *   - fused-yaw convergence: that gate protects writes that READ the fused
 *     yaw (or a position lever-arm-corrected with it). A MOTION yaw is the
 *     COG circular mean of the reverse leg, validated by the calibration
 *     node's own gate (min samples, sigma ceiling, bearing match, baseline
 *     displacement); the fused yaw is neither written nor consulted, and right
 *     after the drive it is still settling (~6 deg window-std observed), so
 *     the gate could only ever produce false rejections here.
 * It KEEPS the GPS-accuracy gate: the MOTION contract is "RTK-gated", and
 * live RTK quality at write time is the only part of that claim map_server
 * can check for itself.
 *
 * Every other combination keeps every gate — in particular nothing that
 * captures or sets a POSITION is ever exempt from the charging gate.
 */
inline DockSetGates ResolveDockSetGates(bool use_gps_position,
                                        bool preserve_position,
                                        uint8_t yaw_source)
{
  DockSetGates gates;
  if (!preserve_position)
  {
    gates.kind = use_gps_position ? DockSetKind::POSITION_CAPTURE : DockSetKind::MANUAL;
    return gates;
  }
  if (use_gps_position)
  {
    gates.invalid_reason = "preserve_position and use_gps_position are mutually exclusive";
    return gates;
  }
  if (yaw_source != kDockYawSourceMotion)
  {
    // PRESERVE would be a no-op write; REQUEST is a manual heading edit, which
    // already has a path (send the position back) and has no reason to bypass
    // the on-dock gates. Refuse rather than grow a second exemption.
    gates.invalid_reason = "preserve_position requires yaw_source=MOTION";
    return gates;
  }
  gates.kind = DockSetKind::YAW_ONLY_MOTION;
  gates.require_charging = false;
  gates.require_gps_accuracy = true;
  gates.require_yaw_convergence = false;
  gates.require_existing_pose = true;
  return gates;
}

}  // namespace mowgli_map
