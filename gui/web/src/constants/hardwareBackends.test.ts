import { describe, expect, it } from "vitest";
import {
    appliesToBackend,
    liveHardwareParameters,
    normalizeHardwareBackend,
    presetValuesForBackend,
    settingsSectionsForBackend,
} from "./hardwareBackends.ts";
import { MOWER_MODELS } from "./mowerModels.ts";
import {
    FIRMWARE_SAFETY_GROUP,
    OPENMOWER_WIRING_GROUP,
    YAW_LOOP_GROUP,
    groupForBackend,
    groupKeys,
} from "../components/settings/settingsFieldGroups.ts";

describe("normalizeHardwareBackend", () => {
    it("keeps known backends and falls back to mowgli", () => {
        expect(normalizeHardwareBackend("openmower")).toBe("openmower");
        expect(normalizeHardwareBackend("mavros")).toBe("mavros");
        expect(normalizeHardwareBackend("bogus")).toBe("mowgli");
        expect(normalizeHardwareBackend(undefined)).toBe("mowgli");
    });
});

describe("appliesToBackend", () => {
    it("treats an absent limit as every backend", () => {
        expect(appliesToBackend(undefined, "openmower")).toBe(true);
        expect(appliesToBackend(["mowgli"], "openmower")).toBe(false);
        expect(appliesToBackend(["mowgli", "openmower"], "openmower")).toBe(true);
    });
});

describe("presetValuesForBackend", () => {
    it("never writes a model's STM32 ticks_per_meter on an OpenMower robot", () => {
        const yardforce = MOWER_MODELS.find((m) => m.value === "YardForce500")!;
        const overrides = { ticks_per_meter: 1600, both_wheels_lift_emergency_ms: 100 };

        const values = presetValuesForBackend(yardforce.defaults, overrides);

        expect(values.ticks_per_meter).toBe(1600);
        expect(values.wheel_track).toBe(yardforce.defaults.wheel_track);
        expect(values).not.toHaveProperty("both_wheels_lift_emergency_ms");
    });

    it("leaves the preset untouched without backend overrides", () => {
        const preset = { ticks_per_meter: 300, wheel_track: 0.325 };
        expect(presetValuesForBackend(preset, {})).toEqual(preset);
    });
});

describe("groupForBackend", () => {
    it("hides STM32-only fields behind the OpenMower board", () => {
        const om = groupForBackend(FIRMWARE_SAFETY_GROUP, "openmower");
        expect(om && groupKeys(om)).toEqual([
            "max_mps",
            "one_wheel_lift_emergency_ms",
            "both_wheels_lift_emergency_ms",
        ]);
        expect(groupForBackend(FIRMWARE_SAFETY_GROUP, "mowgli")?.fields).toHaveLength(
            FIRMWARE_SAFETY_GROUP.fields.length,
        );
    });

    it("drops a group that does not exist on the backend", () => {
        expect(groupForBackend(YAW_LOOP_GROUP, "openmower")).toBeNull();
        expect(groupForBackend(OPENMOWER_WIRING_GROUP, "mowgli")).toBeNull();
        expect(groupForBackend(OPENMOWER_WIRING_GROUP, "openmower")).not.toBeNull();
    });
});

describe("hardware backend settings", () => {
    it("keeps the Drive section for all three backends", () => {
        const sections = [
            { id: "hardware" },
            { id: "drive_motor" },
            { id: "navigation" },
        ];

        expect(settingsSectionsForBackend(sections, "mowgli")).toEqual(sections);
        expect(settingsSectionsForBackend(sections, "mavros")).toEqual(sections);
        expect(settingsSectionsForBackend(sections, "openmower")).toEqual(sections);
    });

    it("does not send runtime-pending MAVROS routes or Mowgli PID parameters", () => {
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
            ticks_per_meter: {
                parameter: "hardware_bridge.ticks_per_meter",
                runtime: "available" as const,
            },
            wheel_pid_kp: {
                parameter: "hardware_bridge.wheel_pid_kp",
                runtime: "available" as const,
            },
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
