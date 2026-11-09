import {fireEvent, render, screen} from "@testing-library/react";
import {describe, expect, it, vi} from "vitest";
import {HardwareViewSwitcher} from "./HardwareViewSwitcher.tsx";

const info = vi.hoisted(() => ({state: {connected: true}, vehicle: null as import("../../hooks/useMavrosInfo").MavrosVehicleInfo | null}));
vi.mock("../../hooks/useMavrosInfo.ts", () => ({useMavrosInfo: () => info, formatMavrosBoardIdentity: () => null}));
vi.mock("./FcuParamTool.tsx", () => ({FcuParamTool: () => <div>parameter tool</div>}));

describe("HardwareViewSwitcher", () => {
    it("hides the FCU tab for the Mowgli backend", () => {
        render(<HardwareViewSwitcher backend="mowgli" chassis={<div>chassis</div>} fcu={<div>fcu tool</div>}/>);
        expect(screen.queryByText("FCU")).not.toBeInTheDocument();
        expect(screen.getByText("chassis")).toBeInTheDocument();
    });

    it("shows and switches to the FCU tool for MAVROS", () => {
        render(<HardwareViewSwitcher backend="mavros" chassis={<div>chassis</div>} fcu={<div>fcu tool</div>}/>);
        fireEvent.click(screen.getByText("FCU"));
        expect(screen.getByText("fcu tool")).toBeInTheDocument();
        expect(screen.queryByText("chassis")).not.toBeInTheDocument();
    });
});

    it.each([[3, 'ArduPilot'], [12, 'PX4'], [0, 'FCU parameters']] as const)('labels the third tab from actual autopilot %s', (autopilot, label) => {
        info.vehicle = {autopilot, type: 10, available_info: 1};
        render(<HardwareViewSwitcher backend="mavros" chassis={<div>chassis</div>} fcu={<div>yaml builder</div>}/>);
        fireEvent.click(screen.getByText(label)); expect(screen.getByText('parameter tool')).toBeInTheDocument();
        fireEvent.click(screen.getByText('FCU')); expect(screen.getByText('yaml builder')).toBeInTheDocument();
    });
