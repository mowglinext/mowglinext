import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { ObstaclesSection } from "./ObstaclesSection.tsx";

describe("dig obstacle setting", () => {
    it.each([true, false, "false"])("renders %s and changes the map setting only", (value) => {
        const onChange = vi.fn();
        render(<ObstaclesSection values={{ dig_obstacle_enabled: value }} onChange={onChange} />);
        const toggle = screen.getByRole("switch", { name: "Automatically create dig keepouts" });
        const enabled = value === true;
        expect(toggle).toHaveAttribute("aria-checked", String(enabled));
        fireEvent.click(toggle);
        expect(onChange).toHaveBeenCalledExactlyOnceWith("dig_obstacle_enabled", !enabled);
        expect(screen.getByText(/dig detection, stopping and recovery remain active/)).toBeInTheDocument();
    });
});
