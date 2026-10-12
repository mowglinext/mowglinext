/**
 * Mirrors the obstacle_margin floor derivation in
 * ros2/src/mowgli_bringup/launch/robot_config_util.py
 * (`planning_obstacle_margin_floor` / `chassis_half_width`). The GUI backend
 * cannot read the ROS2 template at runtime (gui/CLAUDE.md), and there is no
 * live-parameter round-trip wired for this value, so it is mirrored here —
 * same pattern as MowingSection's headland/mow-angle sentinels. Keep the
 * constants in lockstep with robot_config_util.py if either side changes.
 *
 * Why this exists: navigation.launch.py silently raises a requested
 * obstacle_margin below this floor (only a console line on the robot, never
 * seen in the GUI) — an operator can set 0.10 m and have 0.389 m actually
 * used with no visible feedback. This lets the Settings page show the real,
 * effective value instead.
 */

const CHASSIS_FOOTPRINT_MARGIN_M = 0.05;
const DEFAULT_CHASSIS_WIDTH_M = 0.45;

const FTC_TRACKING_SLACK_M = 0.05;
const FTC_CLEARANCE_MARGIN_MIN_M = 0.0;
const FTC_CLEARANCE_MARGIN_MAX_M = 0.5;
const DEFAULT_FTC_CLEARANCE_MARGIN_M = 0.05;

const GLOBAL_COSTMAP_RESOLUTION_M = 0.08;

export const chassisHalfWidthM = (chassisWidthM?: number): number =>
    (chassisWidthM ?? DEFAULT_CHASSIS_WIDTH_M) / 2 + CHASSIS_FOOTPRINT_MARGIN_M;

const clampFtcClearanceMargin = (requested?: number): number => {
    const value = requested ?? DEFAULT_FTC_CLEARANCE_MARGIN_M;
    return Math.min(FTC_CLEARANCE_MARGIN_MAX_M, Math.max(FTC_CLEARANCE_MARGIN_MIN_M, value));
};

const keepoutRasterSlackM = GLOBAL_COSTMAP_RESOLUTION_M * Math.sqrt(2);

/** Least distance coverage may plan its centreline from a drawn obstacle. */
export const planningObstacleMarginFloorM = (
    chassisWidthM?: number,
    obstacleClearanceMarginM?: number,
): number => {
    const halfWidth = chassisHalfWidthM(chassisWidthM);
    const controllerDemand =
        halfWidth + clampFtcClearanceMargin(obstacleClearanceMarginM) + FTC_TRACKING_SLACK_M;
    const transitDemand = halfWidth + keepoutRasterSlackM;
    const floor = Math.max(controllerDemand, transitDemand);
    // Rounded UP to the millimetre, mirroring the Python side's math.ceil.
    return Math.ceil(floor * 1000 - 1e-9) / 1000;
};

/** How the effective obstacle_margin (a centreline distance) reads beside the
 * chassis body — the unit an operator actually judges the gap by. */
export const besideBodyClearanceM = (marginM: number, chassisWidthM?: number): number =>
    marginM - chassisHalfWidthM(chassisWidthM);
