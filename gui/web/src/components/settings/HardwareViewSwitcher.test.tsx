import {fireEvent, render, screen} from "@testing-library/react";
import {describe, expect, it} from "vitest";
import {HardwareViewSwitcher} from "./HardwareViewSwitcher.tsx";

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
