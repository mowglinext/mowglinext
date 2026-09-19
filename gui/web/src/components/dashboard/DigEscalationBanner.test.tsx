import {describe, expect, it, vi} from "vitest";
import {render, screen, fireEvent, waitFor} from "@testing-library/react";
import {App} from "antd";
import {DigEscalationBanner} from "./DigEscalationBanner.tsx";

// mowgli_hardware's Status.dig_escalated{,_distance_m,_required_distance_m} —
// see hardware_bridge_node.cpp's on_clear_dig_escalation and Status.msg.
const mockStatus = vi.fn();
vi.mock("../../hooks/useStatus.ts", () => ({useStatus: () => mockStatus()}));

const callCreateMock = vi.fn();
vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({mowglinext: {callCreate: callCreateMock}}),
}));

function renderWithApp(ui: React.ReactElement) {
    return render(<App>{ui}</App>);
}

describe("DigEscalationBanner", () => {
    it("renders nothing while no escalation is latched", () => {
        mockStatus.mockReturnValue({dig_escalated: false});
        renderWithApp(<DigEscalationBanner/>);
        expect(screen.queryByTestId("dig-escalation-banner")).not.toBeInTheDocument();
    });

    it("warns and disables Clear while still within the required distance", () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.2,
            dig_escalated_required_distance_m: 0.5,
        });
        renderWithApp(<DigEscalationBanner/>);
        const banner = screen.getByTestId("dig-escalation-banner");
        expect(banner).toHaveTextContent(/repeat dig/i);
        expect(banner).toHaveTextContent("0.20");
        expect(banner).toHaveTextContent("0.50");
        expect(screen.getByRole("button", {name: /clear/i})).toBeDisabled();
    });

    it("enables Clear once the chassis has moved past the required distance", async () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.51,
            dig_escalated_required_distance_m: 0.5,
        });
        callCreateMock.mockResolvedValue({error: undefined});
        renderWithApp(<DigEscalationBanner/>);
        const button = screen.getByRole("button", {name: /clear/i});
        expect(button).not.toBeDisabled();

        fireEvent.click(button);

        await waitFor(() => {
            expect(callCreateMock).toHaveBeenCalledWith("clear_dig_escalation", {});
        });
    });

    it("treats exactly-at-the-threshold distance as not yet clear (matches the server's strict >)", () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.5,
            dig_escalated_required_distance_m: 0.5,
        });
        renderWithApp(<DigEscalationBanner/>);
        expect(screen.getByRole("button", {name: /clear/i})).toBeDisabled();
    });
});
