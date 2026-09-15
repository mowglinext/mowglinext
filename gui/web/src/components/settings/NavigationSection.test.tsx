import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { NavigationSection } from "./NavigationSection.tsx";

// idle_nav2_suspend ships ON, and that changes behaviour on every robot: a
// Play press now waits 10-26 s for the Nav2 stack to come back. It therefore
// has to be switchable from the UI — before this it lived only in the ROS2
// package template, which the sparse installed yaml never surfaces, so an
// operator had no way to turn it off short of hand-editing a file on the robot.
describe("idle Nav2 suspend setting", () => {
    it.each([true, false, "false", "true"])("renders %s and toggles only that key", (value) => {
        const onChange = vi.fn();
        render(<NavigationSection values={{ idle_nav2_suspend: value }} onChange={onChange} />);

        const toggle = screen.getByRole("switch", { name: "Suspend navigation on the dock" });
        const enabled = value === true || value === "true";
        expect(toggle).toHaveAttribute("aria-checked", String(enabled));

        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledExactlyOnceWith("idle_nav2_suspend", !enabled);
    });

    it("tells the operator what the trade-off is", () => {
        render(<NavigationSection values={{ idle_nav2_suspend: true }} onChange={vi.fn()} />);
        expect(screen.getByText(/parked on the dock with nothing to do/)).toBeInTheDocument();
    });

    // The schema entry is what makes "reset to default" work at all:
    // SettingFieldLabel only renders the undo button when the key has a known
    // schema default, and the backend's sparse-prune uses the same default.
    it("offers a reset back to the default once the operator has overridden it", () => {
        const onReset = vi.fn();
        render(
            <NavigationSection
                values={{ idle_nav2_suspend: false }}
                onChange={vi.fn()}
                isOverridden={(k) => k === "idle_nav2_suspend"}
                hasDefault={() => true}
                onReset={onReset}
            />,
        );

        fireEvent.click(screen.getAllByRole("button", { name: "Reset to default" })[0]);
        expect(onReset).toHaveBeenCalledExactlyOnceWith("idle_nav2_suspend");
    });
});
