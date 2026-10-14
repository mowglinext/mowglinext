import {describe, expect, it} from "vitest";
import {formatMavrosBoardIdentity, formatMavrosFirmwareVersion, mavrosAutopilotName, mavrosVehicleTypeName} from "./useMavrosInfo.ts";

describe("MAVROS vehicle info formatting", () => {
    it("decodes MAVLink's packed flight software version", () => {
        expect(formatMavrosFirmwareVersion({available_info: 2, flight_sw_version: 0x040506ff,
            flight_custom_version: "abc123\0\0"})).toBe("4.5.6 · abc123");
    });
    it("does not present absent autopilot-version fields as unknown STM32 data", () => {
        expect(formatMavrosFirmwareVersion({available_info: 1, flight_sw_version: 0})).toBeNull();
        expect(formatMavrosBoardIdentity({available_info: 1, board_version: 42})).toBeNull();
    });
    it("formats native MAVROS provider, vehicle and board fields", () => {
        expect(mavrosAutopilotName(3)).toBe("ArduPilot");
        expect(mavrosVehicleTypeName(10)).toBe("Ground rover");
        expect(formatMavrosBoardIdentity({available_info: 2, board_version: 0x01020304,
            vendor_id: 0x26ac, product_id: 0x0011}))
            .toBe("HW 0x01020304 · VID 0x26ac / PID 0x0011");
    });
});
