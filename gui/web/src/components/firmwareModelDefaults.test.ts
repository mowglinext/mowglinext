import {describe, expect, it} from "vitest";
import {
    applyFirmwareModelDefaults,
    firmwareDefaultsForModel,
    manualOverridesFromProvenance,
} from "./firmwareModelDefaults.ts";

describe("firmware model defaults", () => {
    it("maps the canonical YardForce 500 permutation", () => {
        expect(firmwareDefaultsForModel("YardForce500")).toEqual({
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        });
    });

    it("maps the canonical YardForce 500B permutation", () => {
        expect(firmwareDefaultsForModel("YardForce500B")).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        });
    });

    it.each(["CUSTOM", "LUV1000RI", "unknown", undefined])(
        "does not guess for unsupported model %s",
        (model) => {
            expect(firmwareDefaultsForModel(model)).toBeUndefined();
            expect(applyFirmwareModelDefaults(model, {
                boardType: "BOARD_YARDFORCE500",
                panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            })).toEqual({boardType: "", panelType: ""});
        },
    );

    it("keeps manual board and panel overrides across model changes", () => {
        const manual = {
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
        };
        expect(applyFirmwareModelDefaults("YardForce500B", manual, {
            boardType: true,
            panelType: true,
        })).toEqual(manual);
    });

    it("updates the model-following field when only the other field is overridden", () => {
        expect(applyFirmwareModelDefaults("YardForce500B", {
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        }, {boardType: true})).toEqual({
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        });
    });

    it("updates only automatic fields and preserves unrelated firmware settings", () => {
        const current = {
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            repository: "https://example.test/mowgli",
            maxMps: 0.7,
        };
        expect(applyFirmwareModelDefaults("YardForce500B", current)).toEqual({
            ...current,
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        });
    });

    it("treats missing provenance as a legacy manual selection", () => {
        expect(manualOverridesFromProvenance({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        })).toEqual({boardType: true, panelType: true});
    });

    it("keeps only explicitly automatic fields following model changes", () => {
        expect(manualOverridesFromProvenance({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            boardTypeOrigin: "manual",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500",
        })).toEqual({boardType: true, panelType: false});
    });
});
