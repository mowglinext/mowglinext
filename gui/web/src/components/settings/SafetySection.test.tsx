import { describe, expect, it, vi } from "vitest";
import { render, screen } from "@testing-library/react";
import { ThemeProvider } from "../../theme/ThemeContext.tsx";
import { SafetySection } from "./SafetySection.tsx";

// Issue #195: the Motor Temperature Limits card promised a thermal blade cutoff
// that exists in NO layer of the stack (the firmware only measures and reports
// blade temperature; no ROS2 node read motor_temp_high_c / motor_temp_low_c).
// It was removed and NOT replaced: the section's remaining keys are
// lift_recovery_mode (safety-adjacent — it suppresses the ROS2 lift emergency
// and auto-releases the firmware latch) and lift_blade_resume_delay_sec (inert
// unless that mode is on), so the section is now informational only.
describe("SafetySection", () => {
    const baseValues = {
        lift_recovery_mode: false,
        lift_blade_resume_delay_sec: 1.0,
        // A stale installed yaml may still carry these; they must not render.
        motor_temp_high_c: 80,
        motor_temp_low_c: 40,
    };

    function renderSection() {
        render(
            <ThemeProvider>
                <SafetySection values={baseValues} onChange={vi.fn()} />
            </ThemeProvider>,
        );
    }

    it("no longer renders the motor-temperature controls", () => {
        renderSection();
        expect(screen.queryByText(/Motor Temperature Limits/i)).not.toBeInTheDocument();
        expect(screen.queryByText(/Stop Above/i)).not.toBeInTheDocument();
        expect(screen.queryByText(/Resume Below/i)).not.toBeInTheDocument();
    });

    it("exposes no editable control at all", () => {
        renderSection();
        expect(screen.queryByRole("switch")).not.toBeInTheDocument();
        expect(screen.queryByRole("spinbutton")).not.toBeInTheDocument();
        expect(screen.queryByRole("textbox")).not.toBeInTheDocument();
        expect(screen.queryByRole("combobox")).not.toBeInTheDocument();
    });

    it("does not expose the safety-adjacent lift recovery switch", () => {
        renderSection();
        expect(screen.queryByText(/Auto-recover after lift/i)).not.toBeInTheDocument();
        expect(screen.queryByText(/Blade resume delay/i)).not.toBeInTheDocument();
    });

    it("explains that lift, tilt and the blade cut-out are firmware-owned", () => {
        renderSection();
        expect(screen.getByText(/Handled by the firmware/i)).toBeInTheDocument();
        expect(screen.getByText(/emergency-stop latch/i)).toBeInTheDocument();
    });

    it("points at the Parameters page for the diagnostics temperature thresholds", () => {
        renderSection();
        expect(screen.getByText(/motor_temp_warn_c/)).toBeInTheDocument();
        expect(screen.getByText(/Parameters page/i)).toBeInTheDocument();
    });
});
