import {describe, expect, it} from "vitest";
import {exportFcuYaml, parseFcuYaml, requiredFcuFeatures} from "./fcuFeatureYaml.ts";

describe("FCU feature YAML", () => {
    it("parses a complete document and treats identity as informational", () => {
        const parsed = parseFcuYaml(`vehicle: {id: copter, name: Copter}\nboard: {id: OtherBoard}\nversion: {name: 9.9}\nunknown: keep-me\nselected_features:\n  - DroneCAN\n  - EXTRA\n`);
        expect(parsed.features).toEqual(["DroneCAN", "EXTRA"]);
        expect(parsed.info).toEqual({vehicle: "Copter (copter)", board: "OtherBoard", version: "9.9"});
    });

    it("updates only selected_features on export", () => {
        const source = "# retained comment\nvehicle:\n  id: plane\nunknown: keep-me\nselected_features:\n- EXTRA\n";
        const result = exportFcuYaml(parseFcuYaml(source), ["EXTRA", "DroneCAN"]);
        expect(result).toContain("# retained comment");
        expect(result).toContain("unknown: keep-me");
        expect(parseFcuYaml(result).features).toEqual(["EXTRA", "DroneCAN"]);
    });

    it("rejects invalid or missing selected_features", () => {
        expect(() => parseFcuYaml("selected_features: [broken\n")).toThrow();
        expect(() => parseFcuYaml("vehicle: {id: rover}\n")).toThrow(/selected_features/);
    });

    it("loads the versioned minimum profile", () => {
        expect(requiredFcuFeatures).toContain("RPM_ESC_TELEM");
        expect(requiredFcuFeatures).toContain("DroneCAN");
    });
});
