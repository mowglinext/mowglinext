import {App} from "antd";
import {cleanup, fireEvent, render, screen, waitFor} from "@testing-library/react";
import type {ReactNode} from "react";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {ThemeProvider} from "../theme/ThemeContext.tsx";
import {FlashBoardComponent} from "./FlashBoardComponent.tsx";
import {fetchEventSource} from "@microsoft/fetch-event-source";

const state = vi.hoisted(() => ({
    savedConfig: "",
    settingsModel: "YardForce500B" as string | undefined,
    settingsError: false,
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

describe("FlashBoardComponent model/default integration", () => {
    beforeEach(() => {
        state.savedConfig = "";
        state.settingsModel = "YardForce500B";
        state.settingsError = false;
        vi.mocked(fetchEventSource).mockReset().mockResolvedValue(undefined);
    });

    it("fresh YardForce500B selects its canonical board and panel", async () => {
        renderComponent("YardForce500B");

        await waitFor(() => {
            expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant");
            expect(selectedLabel(1)).toBe("YardForce 500B Classic");
        });
        expect(screen.getByRole("button", {name: /flash firmware/i})).toBeEnabled();
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

        const flashButton = screen.getByRole("button", {name: /flash firmware/i});
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
        const flashButton = screen.getByRole("button", {name: /flash firmware/i});
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
});
