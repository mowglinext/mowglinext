import type { HighLevelStatus, Status } from "../types/ros.generated.ts";
import type { WheelOdom } from "../hooks/useWheelOdom.ts";

export interface Timestamped<T> {
    data: T;
    lastMessageAt: number | null;
}

export interface FirmwareParamResetSafetyInput {
    status: Timestamped<Status>;
    highLevelStatus: Timestamped<HighLevelStatus>;
    wheelOdom: Timestamped<WheelOdom>;
    nowMs: number;
}

export type FirmwareParamResetBlockReason =
    | "status_unknown"
    | "status_stale"
    | "firmware_incompatible"
    | "board_uninitialized"
    | "high_level_unknown"
    | "high_level_stale"
    | "not_idle"
    | "odometry_unknown"
    | "odometry_stale"
    | "wheels_moving"
    | "blade_state_unknown"
    | "blade_state_stale"
    | "blade_active";

const MAX_STATUS_AGE_MS = 1500;
const MAX_HIGH_LEVEL_AGE_MS = 2000;
const MAX_ODOMETRY_AGE_MS = 500;
const MAX_BLADE_STATUS_AGE_MS = 1500;
const MAX_LINEAR_SPEED_M_S = 0.005;
const MAX_ANGULAR_SPEED_RAD_S = 0.01;
const FIRMWARE_PROTOCOL_VERSION = 8;

const isFresh = (at: number | null, now: number, maxAge: number): boolean =>
    at !== null && Number.isFinite(at) && now >= at && now - at <= maxAge;

const rosTimeMs = (time: { sec?: number; nanosec?: number } | undefined): number | null => {
    if (time?.sec === undefined || time.nanosec === undefined) return null;
    if (!Number.isFinite(time.sec) || !Number.isFinite(time.nanosec)) return null;
    return time.sec * 1000 + time.nanosec / 1_000_000;
};

/**
 * Fail-closed UI gate for the explicit parameter-store reset action. Firmware
 * and the host service independently recheck their own live safety inputs.
 */
export const firmwareParamResetBlockReason = (
    input: FirmwareParamResetSafetyInput,
): FirmwareParamResetBlockReason | null => {
    const { status, highLevelStatus, wheelOdom, nowMs } = input;

    if (status.lastMessageAt === null) return "status_unknown";
    if (!isFresh(status.lastMessageAt, nowMs, MAX_STATUS_AGE_MS)) return "status_stale";
    if (status.data.firmware_compatible !== true || status.data.firmware_protocol_version !== FIRMWARE_PROTOCOL_VERSION) {
        return "firmware_incompatible";
    }
    if (status.data.mower_status !== 255) return "board_uninitialized";

    if (highLevelStatus.lastMessageAt === null) return "high_level_unknown";
    if (!isFresh(highLevelStatus.lastMessageAt, nowMs, MAX_HIGH_LEVEL_AGE_MS)) return "high_level_stale";
    if (highLevelStatus.data.state !== 1) return "not_idle";

    if (wheelOdom.lastMessageAt === null) return "odometry_unknown";
    if (!isFresh(wheelOdom.lastMessageAt, nowMs, MAX_ODOMETRY_AGE_MS)) return "odometry_stale";
    const linear = wheelOdom.data.twist?.twist?.linear?.x;
    const angular = wheelOdom.data.twist?.twist?.angular?.z;
    if (
        linear === undefined || angular === undefined ||
        !Number.isFinite(linear) || !Number.isFinite(angular) ||
        Math.abs(linear) > MAX_LINEAR_SPEED_M_S || Math.abs(angular) > MAX_ANGULAR_SPEED_RAD_S
    ) {
        return "wheels_moving";
    }

    const statusTime = rosTimeMs(status.data.stamp);
    const bladeTime = rosTimeMs(status.data.blade_status_stamp);
    if (statusTime === null || bladeTime === null) return "blade_state_unknown";
    const bladeAge = statusTime - bladeTime;
    if (bladeAge < -100 || bladeAge > MAX_BLADE_STATUS_AGE_MS) return "blade_state_stale";
    if (
        status.data.mow_enabled !== false || status.data.mower_esc_status !== 0 ||
        status.data.mower_motor_rpm !== 0
    ) {
        return "blade_active";
    }
    return null;
};
