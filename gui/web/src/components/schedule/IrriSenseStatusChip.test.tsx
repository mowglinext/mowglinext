import {describe, expect, it, vi} from "vitest";
import {render, screen} from "@testing-library/react";
import {ThemeProvider} from "../../theme/ThemeContext.tsx";
import {IrriSenseStatusChip} from "./IrriSenseStatusChip.tsx";
import type {SoilStatus} from "../../types/irrisense.ts";

// The chip is given its status directly here; the hook path is exercised by
// the section test. Guard against a stray fetch all the same.
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

function renderChip(s: SoilStatus | null) {
    return render(
        <ThemeProvider>
            <IrriSenseStatusChip status={s}/>
        </ThemeProvider>,
    );
}

describe("IrriSenseStatusChip", () => {
    it("renders nothing while the integration is disabled", () => {
        renderChip(status({enabled: false}));
        expect(screen.queryByTestId("irrisense-status-chip")).not.toBeInTheDocument();
        expect(requestMock).not.toHaveBeenCalled();
    });

    it("says scheduled mows are suspended when wet and gating", () => {
        renderChip(status({wet: true, reason: 'zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm'}));
        const chip = screen.getByTestId("irrisense-status-chip");
        expect(chip).toHaveTextContent("Soil wet — scheduled mows suspended");
        expect(chip).toHaveAttribute("data-verdict", "wet");
    });

    it("only reports wet when the gate is switched off", () => {
        renderChip(status({wet: true, gateScheduler: false}));
        expect(screen.getByTestId("irrisense-status-chip")).toHaveTextContent(/^Soil wet$/i);
    });

    it("reports dry on a fresh dry verdict", () => {
        renderChip(status({wet: false}));
        expect(screen.getByTestId("irrisense-status-chip")).toHaveTextContent("Soil dry");
    });

    // A wet-but-stale verdict must read as unknown: that is the fail-open
    // contract the scheduler applies, and the chip must not claim otherwise.
    it("reports unknown when the data is stale even if it says wet", () => {
        renderChip(status({wet: true, fresh: false, unknown: true}));
        const chip = screen.getByTestId("irrisense-status-chip");
        expect(chip).toHaveTextContent("IrriSense: state unknown");
        expect(chip).toHaveAttribute("data-verdict", "unknown");
    });
});
