import { describe, expect, it } from "vitest";
import i18n from "../../i18n";
import en from "../../i18n/locales/en.json";
import {
    GNSS_ADVANCED_SETTINGS_BY_FAMILY,
    GNSS_ACTION_SETTINGS_KEYS,
    GNSS_EXECUTION_BAUD_OPTIONS,
    gnssProfileLabel,
    gnssSignalProfileDescription,
    gnssSignalProfileLabel,
    normalizeGnssProfile,
    normalizeGnssReceiverModel,
    normalizeGnssSignalGroup,
    rawGnssInputString,
} from "./gnssConfig.ts";

describe("gnssConfig", () => {
    it("normalizes Unicore signal-group whitespace", () => {
        expect(normalizeGnssSignalGroup("  3   6  ")).toBe("3 6");
        expect(normalizeGnssSignalGroup("3,6")).toBe("3 6");
        expect(normalizeGnssSignalGroup("3/6")).toBe("3 6");
    });

    it("preserves raw signal-group typing until blur/save normalization runs", () => {
        expect(rawGnssInputString("3 6")).toBe("3 6");
        expect(rawGnssInputString("3,6")).toBe("3,6");
    });

    it("normalizes receiver-model placeholders into the config-auto path", () => {
        expect(normalizeGnssReceiverModel("unknown")).toBe("");
        expect(normalizeGnssReceiverModel(" um982 ")).toBe("UM982");
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
        if (receiverModelField?.kind !== "select") {
            throw new Error("expected select field");
        }
        expect(receiverModelField.options.map((option) => option.value)).toEqual([
            "",
            "UM960",
            "UM980",
            "UM981",
            "UM982",
        ]);
        expect(i18n.t(receiverModelField.options[1].label)).toBe(en.gnssConfig.unicore.receiverModel.option.um960.label);
        expect(i18n.t(receiverModelField.options[3].label)).toBe(en.gnssConfig.unicore.receiverModel.option.um981.label);
    });

    it("keeps signal-group editing as a raw expert override without Mowgli-side presets", () => {
        const signalGroupField = GNSS_ADVANCED_SETTINGS_BY_FAMILY.unicore?.fields[1];
        expect(signalGroupField?.kind).toBe("text");
        expect(signalGroupField?.key).toBe("gnss_signal_group");
    });

    it("persists an optional execution-baud override separately from runtime/config baud", () => {
        expect(GNSS_ACTION_SETTINGS_KEYS).toContain("gnss_execution_baud");
        expect(GNSS_EXECUTION_BAUD_OPTIONS.map((option) => option.value)).toEqual([
            "auto",
            "115200",
            "230400",
            "460800",
            "921600",
        ]);
        expect(i18n.t(GNSS_EXECUTION_BAUD_OPTIONS[0].label)).toBe(en.gnssConfig.executionBaud.auto.label);
    });
});
