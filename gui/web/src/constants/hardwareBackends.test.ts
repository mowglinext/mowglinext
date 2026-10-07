import { describe, expect, it } from "vitest";
import { liveHardwareParameters, settingsSectionsForBackend } from "./hardwareBackends.ts";

describe("hardware backend settings", () => {
    it("keeps the Drive section for both backends", () => {
        const sections = [{ id: "hardware" }, { id: "drive_motor" }, { id: "navigation" }];
        expect(settingsSectionsForBackend(sections, "mowgli")).toEqual(sections);
        expect(settingsSectionsForBackend(sections, "mavros")).toEqual(sections);
    });

    it("routes the canonical calibration without leaking Mowgli PID", () => {
        const routes = {
            ticks_per_meter: {
                parameter: "mavros/esc_wheel_odometry.ticks_per_meter",
                runtime: "available" as const,
            },
            wheel_track: {
                parameter: "mavros/esc_wheel_odometry.track_width_m",
                runtime: "available" as const,
            },
        };
        expect(routes.ticks_per_meter.parameter).toBe("mavros/esc_wheel_odometry.ticks_per_meter");
        expect(routes.wheel_track.parameter).toBe("mavros/esc_wheel_odometry.track_width_m");
        const parameters = liveHardwareParameters(
            new Set(["ticks_per_meter", "wheel_track", "wheel_pid_kp"]),
            { ticks_per_meter: 401.5, wheel_track: 0.325, wheel_pid_kp: 10 },
            routes,
        );
        expect(parameters).toEqual([
            { name: "mavros/esc_wheel_odometry.ticks_per_meter", value: 401.5 },
            { name: "mavros/esc_wheel_odometry.track_width_m", value: 0.325 },
        ]);
    });

    it("preserves the existing Mowgli live parameter requests", () => {
        const routes = {
            ticks_per_meter: { parameter: "hardware_bridge.ticks_per_meter", runtime: "available" as const },
            wheel_pid_kp: { parameter: "hardware_bridge.wheel_pid_kp", runtime: "available" as const },
            wheel_pid_pwm_per_mps: {
                parameter: "hardware_bridge.wheel_pid_pwm_per_mps",
                runtime: "available" as const,
            },
        };
        expect(liveHardwareParameters(
            new Set(Object.keys(routes)),
            { ticks_per_meter: 399, wheel_pid_kp: 10, wheel_pid_pwm_per_mps: 282.135 },
            routes,
        )).toEqual([
            { name: "hardware_bridge.ticks_per_meter", value: 399 },
            { name: "hardware_bridge.wheel_pid_kp", value: 10 },
            { name: "hardware_bridge.wheel_pid_pwm_per_mps", value: 282.135 },
        ]);
    });
    it("routes MAVROS feed-forward through the shared canonical value", () => {
        const routes = {
            wheel_pid_pwm_per_mps: { parameter: "hardware_bridge.manual_control_linear_scale", runtime: "available" as const },
        };
        expect(liveHardwareParameters(new Set(Object.keys(routes)), {
            wheel_pid_pwm_per_mps: 282.135,
        }, routes)).toEqual([
            { name: "hardware_bridge.manual_control_linear_scale", value: 282.135 },
        ]);
    });
});
