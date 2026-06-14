import { describe, expect, it } from "vitest";
import {
    GNSS_ADVANCED_SETTINGS_BY_FAMILY,
    GNSS_CUSTOM_OPTION_VALUE,
    gnssProfileLabel,
    gnssSignalProfileDescription,
    gnssSignalProfileLabel,
    inferPresetTextSelection,
    normalizeGnssProfile,
    normalizeGnssSignalGroup,
} from "./gnssConfig.ts";

describe("gnssConfig", () => {
    it("normalizes Unicore signal-group whitespace", () => {
        expect(normalizeGnssSignalGroup("  3   6  ")).toBe("3 6");
    });

    it("matches known signal-group presets before falling back to custom", () => {
        const field = GNSS_ADVANCED_SETTINGS_BY_FAMILY.unicore?.fields[0];
        expect(field).toBeDefined();
        expect(field?.kind).toBe("presetText");
        const presetField = field!;
        if (presetField.kind !== "presetText") {
            throw new Error("expected presetText field");
        }
        expect(inferPresetTextSelection(presetField, "3 6")).toBe("3 6");
        expect(inferPresetTextSelection(presetField, "3")).toBe("3");
        expect(inferPresetTextSelection(presetField, "4 9")).toBe(GNSS_CUSTOM_OPTION_VALUE);
    });

    it("formats profile labels for the UI", () => {
        expect(gnssProfileLabel("runtime_only")).toBe("Runtime only");
        expect(gnssProfileLabel("Rover-High-Precision-Debug")).toBe("Rover high precision + debug");
        expect(gnssSignalProfileLabel("all_signals")).toBe("Maximum Compatibility");
        expect(gnssSignalProfileDescription("minimal")).toContain("bandwidth");
    });

    it("normalizes legacy or shorthand profile values into Universal GNSS profile ids", () => {
        expect(normalizeGnssProfile("balanced")).toBe("runtime_only");
        expect(normalizeGnssProfile("high_precision")).toBe("rover_high_precision");
        expect(normalizeGnssProfile("debug")).toBe("rover_high_precision_debug");
    });
});
