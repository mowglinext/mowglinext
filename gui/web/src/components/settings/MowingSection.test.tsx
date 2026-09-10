import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { MowingSection } from "./MowingSection.tsx";

// Regression guard for issue #429: num_headland_passes is a THREE-WAY sentinel
// (<0 = none, 0 = auto, >0 = forced count), so the control must be able to emit
// a NEGATIVE value — the old InputNumber was floored at min={0}.
describe("MowingSection — headland passes", () => {
    const baseValues = {
        tool_width: 0.18,
        headland_width: 0.18,
        num_headland_passes: 2,
    };

    function renderSection(onChange: (key: string, value: unknown) => void) {
        render(
            <ThemeProvider>
                <MowingSection values={baseValues} onChange={onChange} />
            </ThemeProvider>,
        );
    }

    it("offers a 'None' option that disables the perimeter rings", async () => {
        const onChange = vi.fn();
        renderSection(onChange);

        // The headland-passes Select renders its current value (2) as the
        // combobox text; open it and pick "None".
        const combobox = screen.getByTitle("2");
        fireEvent.mouseDown(combobox);

        const none = await screen.findByTitle("None");
        fireEvent.click(none);

        expect(onChange).toHaveBeenCalledWith("num_headland_passes", -1);
    });

    it("offers an 'Auto' option mapped to the 0 sentinel", async () => {
        const onChange = vi.fn();
        renderSection(onChange);

        fireEvent.mouseDown(screen.getByTitle("2"));
        const auto = await screen.findByTitle("Auto");
        fireEvent.click(auto);

        expect(onChange).toHaveBeenCalledWith("num_headland_passes", 0);
    });
});
