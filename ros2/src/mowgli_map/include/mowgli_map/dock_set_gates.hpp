// SPDX-License-Identifier: GPL-3.0
/**
 * @file dock_set_gates.hpp
 * @brief Which gates a SetDockingPoint request must pass — pure, no ROS deps.
 *
 * `map_server_node`'s `~/set_docking_point` serves four different kinds of
 * write, and they do NOT need the same protection:
 *
 *   POSITION_CAPTURE  use_gps_position=true. X/Y is averaged from the raw GPS
 *                     antenna samples NOW, so the robot MUST be seated on the
 *                     dock (is_charging) or the average is a position
 *                     somewhere on the lawn. All gates apply.
 *   MANUAL            no position flag set. The operator typed / dragged the
 *                     pose. All gates apply, exactly as before this header.
 *   PENDING_ANTENNA_MOTION
 *                     use_pending_antenna=true, yaw_source=MOTION. The normal
 *                     dock-calibration write: X/Y from the raw antenna mean
 *                     that `~/capture_dock_antenna` took ON the dock (THAT
 *                     call ran the charging + RTK + sample-count gates, at the
 *                     one moment "on the dock" and "RTK-Fixed" are both
 *                     guaranteed), lever-arm-corrected with the fresh MOTION
 *                     yaw and written together with it. The yaw only exists
 *                     once the robot has reversed ~1.5 m OFF the dock, so the
 *                     charging gate cannot apply to THIS call; the position's
 *                     on-dock guarantee is the pending capture itself, which
 *                     must exist, be unexpired and unconsumed.
 *   YAW_ONLY_MOTION   preserve_position=true, yaw_source=MOTION. The fallback
 *                     when no capture is pending: write the motion yaw, keep
 *                     the stored X/Y. Same off-dock situation — requiring
 *                     is_charging here made the calibration fail
 *                     deterministically (field-diagnosed 2026-09-17).
 *
 * Extracted so the decision is unit-testable without spinning a node, and so
 * the charging exemption cannot silently widen: it is reachable through TWO
 * exact field combinations and nothing else, and neither lets a caller supply
 * a position that was measured off the dock.
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
  PENDING_ANTENNA_MOTION,
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
  /// A usable ~/capture_dock_antenna result must be pending (it is consumed).
  bool require_pending_antenna{false};
};

/**
 * @brief Classify a request and return the gates it must pass.
 *
 * The two off-dock MOTION kinds drop exactly two gates, each for a stated
 * reason:
 *   - charging: the yaw is motion-derived, the robot is necessarily off the
 *     dock (see file comment).
 *   - fused-yaw convergence: that gate protects writes that READ the fused
 *     yaw (or a position lever-arm-corrected with it). A MOTION yaw is the
 *     COG circular mean of the reverse leg, validated by the calibration
 *     node's own gate (min samples, sigma ceiling, bearing match, baseline
 *     displacement); the fused yaw is neither written nor consulted, and right
 *     after the drive it is still settling (~6 deg window-std observed), so
 *     the gate could only ever produce false rejections here.
 * They KEEP the GPS-accuracy gate: the MOTION contract is "RTK-gated", and
 * live RTK quality at write time is the only part of that claim map_server
 * can check for itself.
 *
 * Every other combination keeps every gate — in particular nothing that
 * captures a position NOW, or takes one from the caller, is ever exempt from
 * the charging gate.
 */
inline DockSetGates ResolveDockSetGates(bool use_gps_position,
                                        bool preserve_position,
                                        bool use_pending_antenna,
                                        uint8_t yaw_source)
{
  DockSetGates gates;
  const int position_flags =
      (use_gps_position ? 1 : 0) + (preserve_position ? 1 : 0) + (use_pending_antenna ? 1 : 0);
  if (position_flags > 1)
  {
    gates.invalid_reason =
        "use_gps_position, preserve_position and use_pending_antenna are mutually exclusive";
    return gates;
  }
  if (position_flags == 0)
  {
    gates.kind = DockSetKind::MANUAL;
    return gates;
  }
  if (use_gps_position)
  {
    gates.kind = DockSetKind::POSITION_CAPTURE;
    return gates;
  }
  if (yaw_source != kDockYawSourceMotion)
  {
    // Both off-dock kinds exist ONLY to carry a fresh motion yaw. With
    // PRESERVE, preserve_position is a no-op and use_pending_antenna would pair
    // a fresh position with a possibly stale yaw (the on-dock
    // use_gps_position capture already does that, behind every gate); REQUEST
    // is a manual heading edit with no reason to bypass the on-dock gates.
    // Refuse rather than grow a third exemption.
    gates.invalid_reason = "preserve_position / use_pending_antenna require yaw_source=MOTION";
    return gates;
  }
  gates.require_charging = false;
  gates.require_gps_accuracy = true;
  gates.require_yaw_convergence = false;
  if (use_pending_antenna)
  {
    gates.kind = DockSetKind::PENDING_ANTENNA_MOTION;
    gates.require_pending_antenna = true;
  }
  else
  {
    gates.kind = DockSetKind::YAW_ONLY_MOTION;
    gates.require_existing_pose = true;
  }
  return gates;
}

}  // namespace mowgli_map
