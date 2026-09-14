import {describe, expect, it, vi} from "vitest";
import {render, screen} from "@testing-library/react";
import {SoilWetBanner} from "./SoilWetBanner.tsx";
import type {SoilStatus} from "../../types/irrisense.ts";

const requestMock = vi.fn();
vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({request: requestMock}),
}));

function status(overrides: Partial<SoilStatus>): SoilStatus {
    return {
        enabled: true, configured: true, gateScheduler: true,
        fresh: true, wet: false, unknown: false, reason: "", zones: [],
        ...overrides,
    };
}

describe("SoilWetBanner", () => {
    it("warns with the reason and the scheduler suspension when the garden is wet", () => {
        render(<SoilWetBanner status={status({wet: true, reason: "zone Lawn: deficit 0.8 mm"})}/>);
        const banner = screen.getByTestId("soil-wet-banner");
        expect(banner).toHaveTextContent(/garden is wet/i);
        expect(banner).toHaveTextContent("zone Lawn: deficit 0.8 mm");
        expect(banner).toHaveTextContent(/scheduled mows are suspended/i);
    });

    it("does not mention the scheduler when the gate is off", () => {
        render(<SoilWetBanner status={status({wet: true, gateScheduler: false})}/>);
        expect(screen.getByTestId("soil-wet-banner")).not.toHaveTextContent(/suspended/i);
    });

    it("renders nothing when dry, unknown, or disabled", () => {
        const {rerender} = render(<SoilWetBanner status={status({wet: false})}/>);
        expect(screen.queryByTestId("soil-wet-banner")).not.toBeInTheDocument();
        rerender(<SoilWetBanner status={status({wet: true, unknown: true, fresh: false})}/>);
        expect(screen.queryByTestId("soil-wet-banner")).not.toBeInTheDocument();
        rerender(<SoilWetBanner status={status({wet: true, enabled: false})}/>);
        expect(screen.queryByTestId("soil-wet-banner")).not.toBeInTheDocument();
    });
});
