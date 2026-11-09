import { describe, expect, it } from "vitest";
import {
    firmwareParamResetBlockReason,
    type FirmwareParamResetBlockReason,
    type FirmwareParamResetSafetyInput,
} from "./firmwareParamReset.ts";

const nowMs = 20_000;

const safeInput = (): FirmwareParamResetSafetyInput => ({
    nowMs,
    status: {
        lastMessageAt: nowMs,
        data: {
            stamp: { sec: 20, nanosec: 0 },
            blade_status_stamp: { sec: 20, nanosec: 0 },
            mower_status: 255,
            firmware_protocol_version: 8,
            firmware_compatible: true,
            mow_enabled: false,
            mower_esc_status: 0,
            mower_motor_rpm: 0,
        },
    },
    highLevelStatus: { lastMessageAt: nowMs, data: { state: 1 } },
    wheelOdom: {
        lastMessageAt: nowMs,
        data: {
            twist: {
                twist: {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 },
                },
            },
        },
    },
});

describe("firmware parameter reset safety", () => {
    it("allows only a fresh, compatible, initialized, idle, stationary, blade-off state", () => {
        expect(firmwareParamResetBlockReason(safeInput())).toBeNull();
    });

    const blockedCases: Array<[
        string,
        (input: FirmwareParamResetSafetyInput) => void,
        FirmwareParamResetBlockReason,
    ]> = [
        ["unknown board status", (input: FirmwareParamResetSafetyInput) => { input.status.lastMessageAt = null; }, "status_unknown"],
        ["stale board status", (input: FirmwareParamResetSafetyInput) => { input.status.lastMessageAt = nowMs - 1501; }, "status_stale"],
        ["incompatible firmware", (input: FirmwareParamResetSafetyInput) => { input.status.data.firmware_compatible = false; }, "firmware_incompatible"],
        ["unknown protocol", (input: FirmwareParamResetSafetyInput) => { input.status.data.firmware_protocol_version = 7; }, "firmware_incompatible"],
        ["uninitialized board", (input: FirmwareParamResetSafetyInput) => { input.status.data.mower_status = 0; }, "board_uninitialized"],
        ["stale high-level status", (input: FirmwareParamResetSafetyInput) => { input.highLevelStatus.lastMessageAt = nowMs - 2001; }, "high_level_stale"],
        ["non-idle mode", (input: FirmwareParamResetSafetyInput) => { input.highLevelStatus.data.state = 2; }, "not_idle"],
        ["stale odometry", (input: FirmwareParamResetSafetyInput) => { input.wheelOdom.lastMessageAt = nowMs - 501; }, "odometry_stale"],
        ["moving wheels", (input: FirmwareParamResetSafetyInput) => { input.wheelOdom.data.twist!.twist!.linear!.x = 0.02; }, "wheels_moving"],
        ["missing blade sample", (input: FirmwareParamResetSafetyInput) => { input.status.data.blade_status_stamp = undefined; }, "blade_state_unknown"],
        ["stale blade sample", (input: FirmwareParamResetSafetyInput) => { input.status.data.blade_status_stamp = { sec: 18, nanosec: 0 }; }, "blade_state_stale"],
        ["blade enabled", (input: FirmwareParamResetSafetyInput) => { input.status.data.mow_enabled = true; }, "blade_active"],
        ["blade ESC active", (input: FirmwareParamResetSafetyInput) => { input.status.data.mower_esc_status = 1; }, "blade_active"],
        ["blade RPM nonzero", (input: FirmwareParamResetSafetyInput) => { input.status.data.mower_motor_rpm = 1; }, "blade_active"],
    ];

    it.each(blockedCases)("fails closed for %s", (_name, change, expected) => {
        const input = safeInput();
        change(input);
        expect(firmwareParamResetBlockReason(input)).toBe(expected);
    });
});
