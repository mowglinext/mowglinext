import {App} from "antd";
import {cleanup, fireEvent, render, screen, waitFor} from "@testing-library/react";
import type {ReactNode} from "react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {ThemeProvider} from "../theme/ThemeContext.tsx";
import {FlashBoardComponent} from "./FlashBoardComponent.tsx";
import {fetchEventSource} from "@microsoft/fetch-event-source";

const state = vi.hoisted((): {
    savedConfig: string;
    settingsModel: string | undefined;
    settingsError: boolean;
    firmwareCompatible: boolean | null;
    firmwareCapabilities: number;
    firmwareVersion: string;
} => ({
    savedConfig: "",
    settingsModel: "YardForce500B",
    settingsError: false,
    firmwareCompatible: true,
    firmwareCapabilities: 1,
    firmwareVersion: "1.0.193",
}));

vi.mock("../hooks/useApi.ts", () => ({
    useApi: () => ({
        config: {
            keysGetCreate: vi.fn(() => Promise.resolve({
                data: {"gui.firmware.config": state.savedConfig},
            })),
        },
        settings: {
            yamlList: vi.fn(() => {
                if (state.settingsError) return Promise.reject(new Error("settings unavailable"));
                return Promise.resolve({data: {mower_model: state.settingsModel}});
            }),
        },
    }),
}));

vi.mock("@microsoft/fetch-event-source", () => ({
    fetchEventSource: vi.fn(),
}));

vi.mock("../hooks/useFirmwareStatus.ts", () => ({
    useFirmwareStatus: () => ({
        firmwareCompatible: state.firmwareCompatible,
        firmwareCapabilities: state.firmwareCapabilities,
        firmwareVersion: state.firmwareVersion,
        firmwareProtocolVersion: 7,
        firmwareConnectionGeneration: 1,
    }),
}));

vi.mock("react-terminal-ui", () => ({
    ColorMode: {Dark: "dark"},
    default: ({children}: {children: ReactNode}) => <div>{children}</div>,
    TerminalOutput: ({children}: {children: ReactNode}) => <div>{children}</div>,
}));

const renderComponent = (mowerModel?: string) => render(
    <ThemeProvider>
        <App>
            <FlashBoardComponent mowerModel={mowerModel} onNext={vi.fn()} />
        </App>
    </ThemeProvider>,
);

const selectByIndex = (index: number) => {
    const selects = document.querySelectorAll(".ant-select");
    if (!selects[index]) throw new Error(`missing select ${index}`);
    return selects[index] as HTMLElement;
};

const selectedLabel = (index: number) =>
    selectByIndex(index).querySelector(".ant-select-selection-item")?.textContent ?? "";

const chooseOption = async (index: number, option: string) => {
    fireEvent.mouseDown(selectByIndex(index).querySelector(".ant-select-selector")!);
    const optionNode = await screen.findByText(option, {exact: true});
    fireEvent.click(optionNode);
};

const jsonResponse = (body: unknown, status = 200): Response => ({
    ok: status >= 200 && status < 300,
    status,
    json: vi.fn().mockResolvedValue(body),
}) as unknown as Response;

describe("FlashBoardComponent model/default integration", () => {
    beforeEach(() => {
        state.savedConfig = "";
        state.settingsModel = "YardForce500B";
        state.settingsError = false;
        state.firmwareCompatible = true;
        state.firmwareCapabilities = 1;
        state.firmwareVersion = "1.0.193";
        sessionStorage.clear();
        vi.stubGlobal("fetch", vi.fn());
        vi.mocked(fetchEventSource).mockReset().mockResolvedValue(undefined);
    });

    afterEach(() => {
        cleanup();
        vi.unstubAllGlobals();
    });

    it("fresh YardForce500B selects its canonical board and panel", async () => {
        renderComponent("YardForce500B");

        await waitFor(() => {
            expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant");
            expect(selectedLabel(1)).toBe("YardForce 500B Classic");
        });
        expect(screen.getByRole("button", {name: /update via usb/i})).toBeEnabled();
    });

    it("follows a model change while preserving an individual board override", async () => {
        const view = renderComponent("YardForce500");
        await waitFor(() => expect(selectedLabel(1)).toBe("YardForce 500 Classic"));

        await chooseOption(0, "Vermut - YardForce 500 Classic");
        expect(selectedLabel(0)).toBe("Vermut - YardForce 500 Classic");

        view.rerender(
            <ThemeProvider>
                <App>
                    <FlashBoardComponent mowerModel="YardForce500B" onNext={vi.fn()} />
                </App>
            </ThemeProvider>,
        );
        await waitFor(() => expect(selectedLabel(1)).toBe("YardForce 500B Classic"));
        expect(selectedLabel(0)).toBe("Vermut - YardForce 500 Classic");
    });

    it("updates persisted automatic fields for the current model", async () => {
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            boardTypeOrigin: "auto",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500",
        });
        renderComponent("YardForce500B");

        await waitFor(() => {
            expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant");
            expect(selectedLabel(1)).toBe("YardForce 500B Classic");
        });
    });

    it("preserves legacy and manual persisted selections conservatively", async () => {
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        });
        renderComponent("YardForce500B");
        await waitFor(() => expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 Classic"));
        expect(selectedLabel(1)).toBe("YardForce 500 Classic");

        // The component is remounted to exercise a separately persisted
        // manual-board/automatic-panel configuration.
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_VERMUT_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            boardTypeOrigin: "manual",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500",
        });
        cleanup();
        renderComponent("YardForce500B");
        await waitFor(() => expect(selectedLabel(0)).toBe("Vermut - YardForce 500 Classic"));
        expect(selectedLabel(1)).toBe("YardForce 500B Classic");
    });

    it("restores a saved config when settings lookup fails", async () => {
        state.settingsError = true;
        state.settingsModel = undefined;
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            boardTypeOrigin: "auto",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500B",
        });
        renderComponent();

        await waitFor(() => {
            expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant");
            expect(selectedLabel(1)).toBe("YardForce 500B Classic");
        });
    });

    it("blocks flashing until both board and panel are explicitly selected", async () => {
        renderComponent("YardForce500");
        await waitFor(() => expect(selectedLabel(1)).toBe("YardForce 500 Classic"));

        const flashButton = screen.getByRole("button", {name: /flash using st-link/i});
        expect(flashButton).toBeDisabled();
        expect(screen.getByText("Select a board and panel before flashing")).toBeInTheDocument();
        expect(screen.queryByText("No prebuilt firmware for this model yet")).not.toBeInTheDocument();

        await chooseOption(0, "Mowgli - YardForce 500 Classic");
        await waitFor(() => expect(flashButton).toBeEnabled());
    });

    it("submits explicit provenance after a manual field change and confirmation", async () => {
        renderComponent("YardForce500B");
        await waitFor(() => {
            expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant");
            expect(selectedLabel(1)).toBe("YardForce 500B Classic");
        });

        // The board is intentionally changed while the panel keeps following
        // the YardForce500B automatic default.
        await chooseOption(0, "Vermut - YardForce 500 Classic");
        const flashButton = screen.getByRole("button", {name: /flash using st-link/i});
        await waitFor(() => expect(flashButton).toBeEnabled());
        fireEvent.click(flashButton);

        const confirmButton = await screen.findByRole("button", {name: /^Flash$/});
        fireEvent.click(confirmButton);
        await waitFor(() => expect(fetchEventSource).toHaveBeenCalledTimes(1));

        const request = vi.mocked(fetchEventSource).mock.calls[0]?.[1] as {body?: string};
        const payload = JSON.parse(request.body ?? "{}") as {
            boardType?: string;
            panelType?: string;
            boardTypeOrigin?: string;
            panelTypeOrigin?: string;
            firmwareSelectionModel?: string;
        };
        expect(payload.boardType).toBe("BOARD_VERMUT_YARDFORCE500");
        expect(payload.panelType).toBe("PANEL_TYPE_YARDFORCE_500B_CLASSIC");
        expect(payload.boardTypeOrigin).toBe("manual");
        expect(payload.panelTypeOrigin).toBe("auto");
        expect(payload.firmwareSelectionModel).toBe("YardForce500B");
    });

    it("defaults a capable F401 board to USB and keeps ST-Link selectable", async () => {
        renderComponent("YardForce500B");
        await waitFor(() => expect(screen.getByRole("radio", {name: /update via usb/i})).toBeChecked());
        expect(screen.getByText("Recommended")).toBeInTheDocument();

        fireEvent.click(screen.getByRole("radio", {name: /flash using st-link/i}));
        expect(screen.getByRole("button", {name: /flash using st-link/i})).toBeEnabled();
    });

    it("does not offer a functional USB option for F1", async () => {
        renderComponent("YardForce500");
        await waitFor(() => expect(selectedLabel(1)).toBe("YardForce 500 Classic"));
        await chooseOption(0, "Mowgli - YardForce 500 Classic");
        expect(screen.getByRole("radio", {name: /update via usb/i})).toBeDisabled();
        expect(screen.getByText("USB firmware update is not supported on this board")).toBeInTheDocument();
    });

    it("explains the one-time ST-Link migration for legacy F401 firmware", async () => {
        state.firmwareCapabilities = 0;
        state.firmwareVersion = "1.0.192";
        renderComponent("YardForce500B");

        await waitFor(() => expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant"));
        expect(screen.getByRole("radio", {name: /update via usb/i})).toBeDisabled();
        expect(screen.getByText("One ST-Link update is required before USB updates")).toBeInTheDocument();
        expect(screen.getByText(/1\.0\.192/)).toBeInTheDocument();
    });

    it("starts one idempotent USB operation and reconnects with GET only", async () => {
        const operation = {
            id: "usb-dfu-1",
            idempotencyKey: "fixed-key",
            state: "validating",
            cancellable: true,
            updatedAt: "2026-09-16T00:00:00Z",
        };
        const succeeded = {...operation, state: "succeeded", cancellable: false};
        const fetchMock = vi.mocked(fetch)
            .mockResolvedValueOnce(jsonResponse({operation, attached: false}, 202))
            .mockResolvedValueOnce(jsonResponse(succeeded));

        renderComponent("YardForce500B");
        const button = await screen.findByRole("button", {name: /update via usb/i});
        fireEvent.click(button);
        fireEvent.click(await screen.findByRole("button", {name: /^Flash$/}));

        await waitFor(() => expect(fetchMock).toHaveBeenCalledTimes(2));
        expect(fetchMock.mock.calls[0]?.[0]).toBe("/api/setup/firmware-update");
        expect((fetchMock.mock.calls[0]?.[1] as RequestInit).method).toBe("POST");
        expect(fetchMock.mock.calls[1]?.[0]).toBe("/api/setup/firmware-update/usb-dfu-1");
        expect((fetchMock.mock.calls[1]?.[1] as RequestInit).method).toBeUndefined();
        expect(screen.getByText(/firmware flashed successfully/i)).toBeInTheDocument();
    }, 60_000);

    it("requires a fresh physical confirmation for USB Recovery", async () => {
        renderComponent("YardForce500B");
        await waitFor(() => expect(screen.getByRole("button", {name: /update via usb/i})).toBeEnabled());

        fireEvent.click(screen.getByText("Advanced: USB Recovery"));
        fireEvent.click(await screen.findByRole("checkbox", {name: /use usb recovery/i}));
        expect(screen.getByRole("button", {name: /update via usb/i})).toBeDisabled();
        fireEvent.click(screen.getByRole("checkbox", {name: /physically confirmed/i}));
        expect(screen.getByRole("button", {name: /update via usb/i})).toBeEnabled();

        cleanup();
        renderComponent("YardForce500B");
        await waitFor(() => expect(screen.getByRole("button", {name: /update via usb/i})).toBeEnabled());
        fireEvent.click(screen.getByText("Advanced: USB Recovery"));
        expect(screen.getByRole("checkbox", {name: /use usb recovery/i})).not.toBeChecked();
    }, 60_000);
});
