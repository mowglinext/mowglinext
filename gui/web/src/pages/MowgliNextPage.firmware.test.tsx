import {render, screen} from "@testing-library/react";
import {MemoryRouter} from "react-router-dom";
import {describe, expect, it, vi} from "vitest";
import {HealthCard} from "./MowgliNextPage.tsx";

vi.mock("../hooks/useWeather.ts", () => ({useWeather: () => null}));

const data = {gps: 100, gpsLabel: "RTK Fixed", rain: false, emergency: false, motorTemp: 25,
    firmwareCompatible: false, firmwareVersion: ""} as Parameters<typeof HealthCard>[0]["data"];

describe("dashboard firmware inventory by hardware backend", () => {
    it("keeps the existing Mowgli incompatible state and flash action", () => {
        render(<MemoryRouter><HealthCard data={data}
            hardware={{backend: "mowgli", parameterRoutes: {}, runtimeRouting: "available", loading: false}}
            mavros={{state: {}, vehicle: null}}/></MemoryRouter>);
        expect(screen.getByText("Firmware incompatible")).toBeInTheDocument();
        expect(screen.getByRole("button", {name: "Flash firmware"})).toBeInTheDocument();
    });
    it("never treats unset STM32 fields as a MAVROS incompatibility", () => {
        render(<MemoryRouter><HealthCard data={data}
            hardware={{backend: "mavros", parameterRoutes: {}, runtimeRouting: "pending_image", loading: false}}
            mavros={{state: {connected: true}, vehicle: {available_info: 3, autopilot: 12,
                flight_sw_version: 0x010203ff, board_version: 0x01000001}}}/></MemoryRouter>);
        expect(screen.queryByText("Firmware incompatible")).not.toBeInTheDocument();
        expect(screen.queryByRole("button", {name: "Flash firmware"})).not.toBeInTheDocument();
        expect(screen.getByText("FCU connected")).toBeInTheDocument();
        expect(screen.getByText(/PX4 · 1.2.3/)).toBeInTheDocument();
        expect(screen.getByText(/HW 0x01000001/)).toBeInTheDocument();
    });
});
