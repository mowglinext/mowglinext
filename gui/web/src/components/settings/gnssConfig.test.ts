import { describe, expect, it } from "vitest";
import i18n from "../../i18n";
import en from "../../i18n/locales/en.json";
import {
    GNSS_ADVANCED_SETTINGS_BY_FAMILY,
    GNSS_ACTION_SETTINGS_KEYS,
    GNSS_CUSTOM_OPTION_VALUE,
    gnssProfileLabel,
    gnssSignalProfileDescription,
    gnssSignalProfileLabel,
    inferPresetTextSelection,
    normalizeGnssProfile,
    normalizeGnssReceiverModel,
    normalizeGnssSignalGroup,
} from "./gnssConfig.ts";

describe("gnssConfig", () => {
    it("normalizes Unicore signal-group whitespace", () => {
        expect(normalizeGnssSignalGroup("  3   6  ")).toBe("3 6");
    });

    it("normalizes receiver-model placeholders into the config-auto path", () => {
        expect(normalizeGnssReceiverModel("unknown")).toBe("");
        expect(normalizeGnssReceiverModel(" um982 ")).toBe("UM982");
    });

    it("matches known signal-group presets before falling back to custom", () => {
        const field = GNSS_ADVANCED_SETTINGS_BY_FAMILY.unicore?.fields[1];
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
        // The label helpers now return i18n keys; resolve them through the
        // active locale (pinned to English in test/setup.ts) and compare to the
        // locale JSON so reworded copy doesn't break the test.
        expect(i18n.t(gnssProfileLabel("runtime_only"))).toBe(en.gnssConfig.profile.runtime_only.label);
        expect(i18n.t(gnssProfileLabel("Rover-High-Precision-Debug"))).toBe(en.gnssConfig.profile.rover_high_precision_debug.label);
        expect(i18n.t(gnssSignalProfileLabel("all_signals"))).toBe(en.gnssConfig.signalProfile.all_signals.label);
        expect(i18n.t(gnssSignalProfileDescription("minimal"))).toBe(en.gnssConfig.signalProfile.minimal.description);
    });

    it("normalizes legacy or shorthand profile values into Universal GNSS profile ids", () => {
        expect(normalizeGnssProfile("balanced")).toBe("runtime_only");
        expect(normalizeGnssProfile("high_precision")).toBe("rover_high_precision");
        expect(normalizeGnssProfile("debug")).toBe("rover_high_precision_debug");
    });

    it("includes a dedicated persisted receiver-model field in the unicore expert seam", () => {
        const receiverModelField = GNSS_ADVANCED_SETTINGS_BY_FAMILY.unicore?.fields[0];
        expect(receiverModelField?.kind).toBe("select");
        expect(receiverModelField?.key).toBe("gnss_receiver_model");
        expect(GNSS_ACTION_SETTINGS_KEYS).toContain("gnss_receiver_model");
    });
});
