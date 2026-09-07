import {describe, expect, it, vi} from "vitest";
import {fireEvent, render, screen} from "@testing-library/react";
import {ThemeProvider} from "../../theme/ThemeContext.tsx";
import {LocalizationSection} from "./LocalizationSection.tsx";

// The two LiDAR map-anchor keys (use_lidar_map_anchor,
// lidar_anchor_shadow_mode) are template keys in mowgli_robot.yaml with no
// schema entry, written straight through like use_scan_matching. Shadow mode
// is meaningless with the anchor off, so its switch is locked until the anchor
// is on.
vi.mock("../../hooks/useDiagnostics.ts", () => ({
    useDiagnostics: () => ({diagnostics: {status: []}}),
}));

describe("LocalizationSection LiDAR map anchor toggles", () => {
    function renderSection(values: Record<string, unknown>) {
        const onChange = vi.fn();
        render(
            <ThemeProvider>
                <LocalizationSection values={values} onChange={onChange}/>
            </ThemeProvider>,
        );
        return onChange;
    }

    function switchFor(label: RegExp): HTMLElement {
        const title = screen.getByText(label);
        const card = title.closest(".ant-card");
        if (!card) throw new Error(`no card around ${label}`);
        const sw = card.querySelector('[role="switch"]');
        if (!sw) throw new Error(`no switch in card ${label}`);
        return sw as HTMLElement;
    }

    it("renders both anchor toggles next to scan matching and loop closure", () => {
        renderSection({lidar_enabled: true, use_scan_matching: true, use_loop_closure: true});
        expect(screen.getByText(/LiDAR scan matching/)).toBeInTheDocument();
        expect(screen.getByText(/^Loop closure$/)).toBeInTheDocument();
        expect(screen.getByText(/^LiDAR map anchor$/)).toBeInTheDocument();
        expect(screen.getByText(/^Anchor shadow mode$/)).toBeInTheDocument();
    });

    it("locks the shadow-mode switch while the anchor is off", () => {
        renderSection({lidar_enabled: true, use_lidar_map_anchor: false, lidar_anchor_shadow_mode: false});
        expect(switchFor(/^Anchor shadow mode$/)).toBeDisabled();
        expect(switchFor(/^LiDAR map anchor$/)).not.toBeDisabled();
    });

    it("unlocks the shadow-mode switch once the anchor is on", () => {
        renderSection({lidar_enabled: true, use_lidar_map_anchor: true});
        expect(switchFor(/^Anchor shadow mode$/)).not.toBeDisabled();
    });

    it("writes the yaml keys straight through onChange", () => {
        const onChange = renderSection({lidar_enabled: true, use_lidar_map_anchor: true});
        fireEvent.click(switchFor(/^LiDAR map anchor$/));
        expect(onChange).toHaveBeenCalledWith("use_lidar_map_anchor", false);
        fireEvent.click(switchFor(/^Anchor shadow mode$/));
        expect(onChange).toHaveBeenCalledWith("lidar_anchor_shadow_mode", true);
    });
});
