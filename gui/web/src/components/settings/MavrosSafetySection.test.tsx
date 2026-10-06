import { beforeEach, describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen } from "@testing-library/react";
import { App } from "antd";
import { MavrosSafetySection } from "./MavrosSafetySection.tsx";

const mock = vi.hoisted(() => ({ diagnostics: { safety: null as unknown, lift: null as unknown } }));
vi.mock("../../hooks/useMavrosDiagnostics.ts", () => ({ useMavrosDiagnostics: () => mock.diagnostics }));

describe("MAVROS Safety", () => {
    beforeEach(() => { mock.diagnostics = { safety: null, lift: null }; });
    const mount = (enabled = true) => {
        const onChange = vi.fn();
        render(<App><MavrosSafetySection values={{ mavros_wheel_lift_safety_enabled: enabled }} onChange={onChange} /></App>);
        return onChange;
    };
    it("displays unknown switch and both wheels until verified diagnostics arrive", () => {
        mount(); expect(screen.getAllByText("Unknown")).toHaveLength(4);
        expect(screen.getByText(/cannot be bypassed/)).toBeInTheDocument();
    });
    it.each(["engaged", "released"])("shows the physical switch as %s", (message) => {
        mock.diagnostics.safety = { entry: { message } };
        mount(); expect(screen.getByText(message === "engaged" ? "Engaged" : "Released")).toBeInTheDocument();
    });
    it("keeps a lifted wheel orange while protection is off", () => {
        mock.diagnostics.lift = { fields: { state_valid: "true", left_lifted: "true", right_lifted: "false", safety_enabled: "false" } };
        mount(false); expect(screen.getByText("Lifted")).toHaveClass("ant-tag-orange");
        expect(screen.getByText("On ground")).toBeInTheDocument(); expect(screen.getByText("Off")).toBeInTheDocument();
    });
    it("shows both lifted wheels red", () => {
        mock.diagnostics.lift = { fields: { state_valid: "true", left_lifted: "true", right_lifted: "true", safety_enabled: "true" } };
        mount(); screen.getAllByText("Lifted").forEach((tag) => expect(tag).toHaveClass("ant-tag-error"));
    });
    it("requires confirmation before disabling protection", async () => {
        const onChange = mount(); fireEvent.click(screen.getByRole("switch"));
        expect(onChange).not.toHaveBeenCalled(); expect(await screen.findByRole("dialog", { name: "Disable wheel lift protection?" })).toBeInTheDocument();
        fireEvent.click(screen.getByRole("button", { name: "OK" }));
        expect(onChange).toHaveBeenCalledWith("mavros_wheel_lift_safety_enabled", false);
    });
});
