import {render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {MavrosFirmwareCard} from "./MavrosFirmwareCard.tsx";

vi.mock("../../hooks/useMavrosInfo.ts", async importOriginal => {
    const actual = await importOriginal<typeof import("../../hooks/useMavrosInfo.ts")>();
    return {...actual, useMavrosInfo: () => ({state: {connected: true}, vehicle: {available_info: 3,
        autopilot: 3, type: 10, flight_sw_version: 0x040506ff, flight_custom_version: "mowgli42",
        board_version: 0x01020304, vendor_id: 0x26ac, product_id: 0x0011}})};
});

describe("MavrosFirmwareCard", () => {
    it("shows native MAVROS inventory and remains read-only", () => {
        render(<MavrosFirmwareCard/>);
        expect(screen.getByText("4.5.6 · mowgli42")).toBeInTheDocument();
        expect(screen.getByText("ArduPilot")).toBeInTheDocument();
        expect(screen.getByText(/HW 0x01020304/)).toBeInTheDocument();
        expect(screen.getByText("Firmware updates via MAVROS are not supported yet.")).toBeInTheDocument();
        expect(screen.queryByText(/Firmware incompatible/i)).not.toBeInTheDocument();
        expect(screen.queryByRole("button", {name: /flash/i})).not.toBeInTheDocument();
    });
});
