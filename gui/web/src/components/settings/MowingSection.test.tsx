import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { MowingSection } from "./MowingSection.tsx";

describe("MowingSection — automatic blade direction", () => {
    it("defaults off and emits a boolean when enabled", () => {
        const onChange = vi.fn();
        render(<ThemeProvider><MowingSection values={{}} onChange={onChange} /></ThemeProvider>);
        const toggle = screen.getByRole("switch", { name: "Automatic blade direction" });
        expect(toggle).not.toBeChecked();
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledWith("blade_auto_reverse", true);
        expect(screen.getByText(/firmware with safe reversal support/)).toBeInTheDocument();
    });

    it("shows a saved enabled value and allows disabling it", () => {
        const onChange = vi.fn();
        render(<ThemeProvider><MowingSection values={{ blade_auto_reverse: true }} onChange={onChange} /></ThemeProvider>);
        const toggle = screen.getByRole("switch", { name: "Automatic blade direction" });
        expect(toggle).toBeChecked();
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledWith("blade_auto_reverse", false);
    });

    it("offers reset to the schema default", () => {
        const onReset = vi.fn();
        render(<ThemeProvider><MowingSection
            values={{ blade_auto_reverse: true }} onChange={vi.fn()}
            isOverridden={(key) => key === "blade_auto_reverse"}
            hasDefault={(key) => key === "blade_auto_reverse"} onReset={onReset}
        /></ThemeProvider>);
        fireEvent.click(screen.getByRole("button", { name: "Reset to default" }));
        expect(onReset).toHaveBeenCalledWith("blade_auto_reverse");
    });
});

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

// Blade-load slowdown (FollowCoveragePath.blade_load_*): the toggle must emit
// the yaml key the launch file injects, the thresholds must be inert while it
// is off, and an empty ramp (full <= min) must be flagged — navigation.launch.py
// disables the feature on such a ramp, so saving it would leave a toggle that
// silently never engages.
describe("MowingSection — blade load slowdown", () => {
    const baseValues = {
        tool_width: 0.18,
        headland_width: 0.18,
        num_headland_passes: 2,
    };

    function renderSection(
        onChange: (key: string, value: unknown) => void,
        values: Record<string, unknown> = baseValues,
    ) {
        render(
            <ThemeProvider>
                <MowingSection values={values} onChange={onChange} />
            </ThemeProvider>,
        );
    }

    it("emits blade_load_slowdown_enabled when the toggle is flipped", () => {
        const onChange = vi.fn();
        renderSection(onChange);

        fireEvent.click(screen.getByRole("switch", { name: "Blade Load Slowdown" }));

        expect(onChange).toHaveBeenCalledWith("blade_load_slowdown_enabled", true);
    });

    it("keeps the RPM thresholds disabled while the feature is off", () => {
        renderSection(vi.fn());

        // Sparse config: no blade_load_* keys at all → template defaults, off.
        expect(screen.getByRole("switch", { name: "Blade Load Slowdown" })).not.toBeChecked();
        expect(screen.getByDisplayValue("2500")).toBeDisabled();
        expect(screen.getByDisplayValue("1800")).toBeDisabled();
    });

    it("enables the thresholds and emits them once the feature is on", () => {
        const onChange = vi.fn();
        renderSection(onChange, { ...baseValues, blade_load_slowdown_enabled: true });

        const full = screen.getByDisplayValue("2500");
        expect(full).not.toBeDisabled();
        fireEvent.change(full, { target: { value: "2800" } });
        fireEvent.blur(full);

        expect(onChange).toHaveBeenCalledWith("blade_load_rpm_full", 2800);
    });

    it("warns about an empty ramp only while the feature is on", () => {
        const invalid = { ...baseValues, blade_load_rpm_full: 1500, blade_load_rpm_min: 2000 };

        const { unmount } = render(
            <ThemeProvider>
                <MowingSection values={{ ...invalid, blade_load_slowdown_enabled: true }} onChange={vi.fn()} />
            </ThemeProvider>,
        );
        expect(screen.getByText(/full-speed RPM must be above/)).toBeInTheDocument();
        unmount();

        renderSection(vi.fn(), { ...invalid, blade_load_slowdown_enabled: false });
        expect(screen.queryByText(/full-speed RPM must be above/)).toBeNull();
    });
});
