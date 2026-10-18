import {beforeEach, describe, expect, it, vi} from "vitest";
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
    beforeEach(() => {
        callCreateMock.mockReset();
    });

    it("renders nothing while no escalation is latched", () => {
        mockStatus.mockReturnValue({dig_escalated: false});
        renderWithApp(<DigEscalationBanner/>);
        expect(screen.queryByTestId("dig-escalation-banner")).not.toBeInTheDocument();
    });

    it("asks for an acknowledgement before releasing the hold at the spot", async () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.2,
            dig_escalated_required_distance_m: 0.5,
        });
        callCreateMock.mockResolvedValue({error: undefined});
        renderWithApp(<DigEscalationBanner/>);
        const banner = screen.getByTestId("dig-escalation-banner");
        expect(banner).toHaveTextContent(/repeat dig/i);
        expect(banner).toHaveTextContent("0.20");
        expect(banner).toHaveTextContent("0.50");
        const button = screen.getByText("Clear & resume").closest("button")!;
        expect(button).not.toBeDisabled();

        fireEvent.click(button);
        expect(await screen.findAllByText(/resume from the blocked spot/i)).not.toHaveLength(0);
        expect(callCreateMock).not.toHaveBeenCalled();

        fireEvent.click(screen.getByText(/resume mowing/i).closest("button")!);
        await waitFor(() => {
            expect(callCreateMock).toHaveBeenNthCalledWith(1, "clear_dig_escalation", {});
            expect(callCreateMock).toHaveBeenNthCalledWith(2, "high_level_control", {Command: 1});
        });
    });

    it("clears and resumes without a prompt once moved past the required distance", async () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.51,
            dig_escalated_required_distance_m: 0.5,
        });
        callCreateMock.mockResolvedValue({error: undefined});
        renderWithApp(<DigEscalationBanner/>);

        fireEvent.click(screen.getByText("Clear & resume").closest("button")!);

        await waitFor(() => {
            expect(callCreateMock).toHaveBeenNthCalledWith(1, "clear_dig_escalation", {});
            expect(callCreateMock).toHaveBeenNthCalledWith(2, "high_level_control", {Command: 1});
        });
        expect(screen.queryAllByText(/resume from the blocked spot/i)).toHaveLength(0);
    });

    it("does not release the hold when the acknowledgement is cancelled", async () => {
        mockStatus.mockReturnValue({
            dig_escalated: true,
            dig_escalated_distance_m: 0.5,
            dig_escalated_required_distance_m: 0.5,
        });
        renderWithApp(<DigEscalationBanner/>);

        fireEvent.click(screen.getByText("Clear & resume").closest("button")!);
        fireEvent.click((await screen.findByText(/^cancel$/i)).closest("button")!);

        // The AsyncButton settles (spinner gone) once the wrapped call resolves.
        await waitFor(() => {
            expect(screen.getByText("Clear & resume").closest("button")).not.toHaveClass("ant-btn-loading");
        });
        expect(callCreateMock).not.toHaveBeenCalled();
    });
});
