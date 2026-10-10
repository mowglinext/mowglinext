import { describe, expect, it } from "vitest";
import {
    appliesToBackend,
    normalizeHardwareBackend,
    presetValuesForBackend,
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
        // Arrange
        const yardforce = MOWER_MODELS.find((m) => m.value === "YardForce500")!;
        const overrides = { ticks_per_meter: 1600, both_wheels_lift_emergency_ms: 100 };

        // Act
        const values = presetValuesForBackend(yardforce.defaults, overrides);

        // Assert
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
