import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { App } from "antd";
import { MavrosDriveSection } from "./MavrosDriveSection.tsx";

vi.mock("../../hooks/useMavrosDiagnostics.ts", () => ({ useMavrosDiagnostics: () => ({ odometry: null }) }));
vi.mock("./MavrosCalibrationCard.tsx", () => ({ MavrosCalibrationCard: () => <div>RTK calibration</div> }));

describe("MAVROS Drive", () => {
    it("offers only odometry calibration and explicit traction opt-in", async () => {
        const onChange = vi.fn();
        render(<App><MavrosDriveSection values={{ ticks_per_meter: 399, wheel_track: 0.325 }} onChange={onChange} /></App>);
        expect(screen.getByText("RTK calibration")).toBeInTheDocument();
        expect(screen.getAllByRole("spinbutton")).toHaveLength(2);
        expect(screen.queryByRole("button", { name: /PID|feed.forward|flash/i })).not.toBeInTheDocument();
        const toggle = screen.getByRole("switch"); expect(toggle).not.toBeChecked();
        fireEvent.click(toggle); expect(onChange).not.toHaveBeenCalled();
        expect(await screen.findByRole("dialog", { name: "Enable traction commands?" })).toBeInTheDocument();
        expect(screen.getByText(/never arms the FCU/)).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", { name: "OK" }));
        expect(onChange).toHaveBeenCalledWith("mavros_manual_control_enabled", true);
    });
});
