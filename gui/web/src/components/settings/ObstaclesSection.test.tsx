import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { normalizeDigSensitivity, ObstaclesSection } from "./ObstaclesSection.tsx";

describe("dig obstacle setting", () => {
    it.each([true, false, "false"])("renders %s and changes the map setting only", (value) => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ dig_obstacle_enabled: value }} onChange={onChange} />);
        const toggle = screen.getByRole("switch", { name: "Propose an obstacle where the robot dug in" });
        const enabled = value === true;
        expect(toggle).toHaveAttribute("aria-checked", String(enabled));
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledExactlyOnceWith("dig_obstacle_enabled", !enabled);
        expect(screen.getByText(/dig detection, stopping and recovery remain active/)).toBeInTheDocument();
    });
});

describe("dig sensitivity setting", () => {
    it("shows the four levels and writes the chosen one", () => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ dig_sensitivity: "medium" }} onChange={onChange} />);
        for (const label of ["Off", "Low", "Medium", "High"]) {
            expect(screen.getByText(label)).toBeInTheDocument();
        }
        fireEvent.click(screen.getByText("Low"));
        expect(onChange).toHaveBeenCalledExactlyOnceWith("dig_sensitivity", "low");
    });

    it.each([
        [false, "off"],
        ["OFF", "off"],
        [" High ", "high"],
        [undefined, "medium"],
        ["nonsense", "medium"],
    ])("normalises %s to %s", (raw, expected) => {
        expect(normalizeDigSensitivity(raw)).toBe(expected);
    });
});

describe("drawn obstacle margin floor transparency", () => {
    // Default chassis (0.45 m) + default clearance margin (0.05 m) floors the
    // centreline distance at 0.389 m, i.e. ~11 cm beside the body — see
    // utils/obstacleMargin.ts. navigation.launch.py silently raises any
    // requested value below this floor with no feedback visible in the GUI;
    // these pin that the Settings page now surfaces it instead.
    it("warns with the real effective value when the setting is below the floor", () => {
        render(<ObstaclesSection values={{ obstacle_margin: 0.1 }} onChange={vi.fn()} />);
        expect(
            screen.getByText(/0\.389 m.*11 cm beside the chassis.*will actually be used/),
        ).toBeInTheDocument();
    });

    it("shows the beside-body reading instead of a warning once at or above the floor", () => {
        // 0.475 - 0.275 = 0.2 m = 20 cm, clear of the 22.5 cm floating-point
        // rounding boundary an exact-.5-cm result (e.g. 0.5 m -> 22.5 cm)
        // would sit on.
        render(<ObstaclesSection values={{ obstacle_margin: 0.475 }} onChange={vi.fn()} />);
        expect(screen.getByText(/20 cm beside the chassis/)).toBeInTheDocument();
        expect(screen.queryByText(/will actually be used/)).not.toBeInTheDocument();
    });

    it("floors against an overridden chassis_width, not the shipped default", () => {
        // A 0.60 m chassis raises half_width to 0.35 m, so the transit floor
        // becomes 0.35 + 0.1131 ≈ 0.463 m -> ceil to 0.464 m.
        render(
            <ObstaclesSection
                values={{ obstacle_margin: 0.1, chassis_width: 0.6 }}
                onChange={vi.fn()}
            />,
        );
        expect(screen.getByText(/0\.464 m.*11 cm beside the chassis.*will actually be used/)).toBeInTheDocument();
    });
});
