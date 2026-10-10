import { describe, expect, it } from "vitest";
import { mavrosDiagnosticFields } from "./useMavrosDiagnostics.ts";

describe("MAVROS diagnostic freshness", () => {
    it("marks expired switch/lift data unknown even if cached in the diagnostics hook", () => {
        const diagnostic = { name: "mowgli_mavros_bridge/wheel_lift", receivedAt: 1000, level: 0, message: "on_ground", hardware_id: "", values: [{ key: "state_valid", value: "true" }] };
        expect(mavrosDiagnosticFields({ status: [diagnostic] }, diagnostic.name, 1001)?.fields.state_valid).toBe("true");
        expect(mavrosDiagnosticFields({ status: [diagnostic] }, diagnostic.name, 32000)).toBeNull();
        expect(mavrosDiagnosticFields({}, diagnostic.name, 1001)).toBeNull();
    });
});
