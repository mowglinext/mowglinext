import {App} from "antd";
import {act, cleanup, fireEvent, render, screen, waitFor, within} from "@testing-library/react";
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

const targetSummary = () => screen.getByTestId("flash-target-summary");

const openTargetPicker = () => {
    fireEvent.click(within(targetSummary()).getByText("Change"));
};

// Index 0/1 are the board/panel pickers once they are visible (the firmware
// source dropdown comes after them in the form).
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

// Text lookups, not role lookups: `getByRole` computes accessible names over
// the whole DOM on every retry, and this form keeps ~30 Formily fields mounted
// (the expert fold is forceRender), which made a `waitFor` on a role query
// cost 10+ s and time out on the CI runner.
const buttonByText = (text: string): HTMLButtonElement => {
    const button = screen.getByText(text, {selector: "span"}).closest("button");
    if (!button) throw new Error(`no button labelled ${text}`);
    return button;
};
const flashButton = () => buttonByText("Flash Firmware");
const confirmFlashButton = async (): Promise<HTMLButtonElement> => {
    const label = await screen.findByText("Flash", {selector: "span"});
    const button = label.closest("button");
    if (!button) throw new Error("no confirm button");
    return button;
};

type SseHandlers = {
    onmessage?: (event: {event: string; data: string}) => void;
};

describe("FlashBoardComponent model-derived target", () => {
    beforeEach(() => {
        state.savedConfig = "";
        state.settingsModel = "YardForce500B";
        state.settingsError = false;
        vi.mocked(fetchEventSource).mockReset().mockResolvedValue(undefined);
    });

    it("shows the target derived from the model instead of asking for it", async () => {
        renderComponent("YardForce500");

        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - YardForce 500 Classic")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 500 Classic")).toBeInTheDocument();
        });
        expect(within(targetSummary()).getByText(/from your mower model/i)).toBeInTheDocument();
        // No board/panel pickers: only the firmware-source dropdown and the
        // expert-fold firmware-target dropdown are rendered.
        expect(document.querySelectorAll(".ant-select")).toHaveLength(2);
        expect(screen.queryByText("Board Selection")).not.toBeInTheDocument();
        expect(screen.queryByText("Panel Selection")).not.toBeInTheDocument();
        expect(screen.queryByText("Select a board and panel before flashing")).not.toBeInTheDocument();
        expect(flashButton()).toBeEnabled();
    });

    it("maps the YardForce 500B to its own board and panel", async () => {
        renderComponent("YardForce500B");

        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - YardForce 500 B Variant")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 500B Classic")).toBeInTheDocument();
        });
        expect(flashButton()).toBeEnabled();
    });

    it("selects the native Biltema RM1000 target for the RM1000 mower model", async () => {
        renderComponent("BiltemaRM1000");
        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - Biltema RM1000")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 900 ECO")).toBeInTheDocument();
        });
        expect(flashButton()).toBeEnabled();

        fireEvent.click(flashButton());
        fireEvent.click(await confirmFlashButton());
        await waitFor(() => expect(fetchEventSource).toHaveBeenCalledTimes(1));
        const request = vi.mocked(fetchEventSource).mock.calls[0]?.[1] as {body?: string};
        const payload = JSON.parse(request.body ?? "{}") as Record<string, unknown>;
        expect(payload.boardType).toBe("BOARD_YARDFORCE500B");
        expect(payload.panelType).toBe("PANEL_TYPE_YARDFORCE_900_ECO");
        expect(payload.firmwareTarget).toBe("BiltemaRM1000");
        expect(payload.firmwareTargetOrigin).toBe("auto");
    });

    it("steers a model without a prebuilt to the custom build path", async () => {
        renderComponent("LUV1000RI");

        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - LUV1000RI")).toBeInTheDocument();
        });
        expect(screen.getByText("No prebuilt firmware for this model yet")).toBeInTheDocument();
        expect(flashButton()).toBeDisabled();
    });

    it("falls back to the pickers when the model cannot identify the board", async () => {
        renderComponent("YardForce900ECO");

        await waitFor(() => expect(selectedLabel(1)).toBe("YardForce 900 ECO"));
        expect(screen.queryByTestId("flash-target-summary")).not.toBeInTheDocument();
        expect(screen.getByText("Select a board and panel before flashing")).toBeInTheDocument();
        expect(flashButton()).toBeDisabled();

        await chooseOption(0, "Mowgli - YardForce 500 Classic");
        await waitFor(() => expect(flashButton()).toBeEnabled());
    });

    it("opens the pickers on Change and keeps the override across a model change", async () => {
        const view = renderComponent("YardForce500");
        await waitFor(() => expect(targetSummary()).toBeInTheDocument());

        openTargetPicker();
        await waitFor(() => expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 Classic"));
        expect(screen.getByText("Overriding the hardware target")).toBeInTheDocument();

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
            expect(within(targetSummary()).getByText("Mowgli - YardForce 500 B Variant")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 500B Classic")).toBeInTheDocument();
        });
    });

    it("preserves legacy and manual persisted selections and says so", async () => {
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        });
        renderComponent("YardForce500B");
        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - YardForce 500 Classic")).toBeInTheDocument();
        });
        expect(within(targetSummary()).getByText("YardForce 500 Classic")).toBeInTheDocument();
        expect(within(targetSummary()).getByText(/set manually/i)).toBeInTheDocument();

        // Remounted to exercise a separately persisted manual-board /
        // automatic-panel configuration.
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_VERMUT_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            boardTypeOrigin: "manual",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500",
        });
        cleanup();
        renderComponent("YardForce500B");
        await waitFor(() => {
            expect(within(targetSummary()).getByText("Vermut - YardForce 500 Classic")).toBeInTheDocument();
        });
        expect(within(targetSummary()).getByText("YardForce 500B Classic")).toBeInTheDocument();
    });

    it("migrates a targetless legacy config to the RM1000 board, SA900ECO panel, and target", async () => {
        state.savedConfig = JSON.stringify({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        });
        renderComponent("BiltemaRM1000");
        await waitFor(() => {
            expect(within(targetSummary()).getByText("Mowgli - Biltema RM1000")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 900 ECO")).toBeInTheDocument();
        });
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
            expect(within(targetSummary()).getByText("Mowgli - YardForce 500 B Variant")).toBeInTheDocument();
            expect(within(targetSummary()).getByText("YardForce 500B Classic")).toBeInTheDocument();
        });
        expect(within(targetSummary()).getByText(/restored from your last flash/i)).toBeInTheDocument();
    });

    it("submits the derived target with automatic provenance", async () => {
        renderComponent("YardForce500");
        await waitFor(() => expect(targetSummary()).toBeInTheDocument());
        expect(flashButton()).toBeEnabled();
        fireEvent.click(flashButton());

        const confirmButton = await confirmFlashButton();
        fireEvent.click(confirmButton);
        await waitFor(() => expect(fetchEventSource).toHaveBeenCalledTimes(1));

        const request = vi.mocked(fetchEventSource).mock.calls[0]?.[1] as {body?: string};
        const payload = JSON.parse(request.body ?? "{}") as Record<string, unknown>;
        expect(payload.boardType).toBe("BOARD_YARDFORCE500");
        expect(payload.panelType).toBe("PANEL_TYPE_YARDFORCE_500_CLASSIC");
        expect(payload.boardTypeOrigin).toBe("auto");
        expect(payload.panelTypeOrigin).toBe("auto");
        expect(payload.firmwareSelectionModel).toBe("YardForce500");
        expect(payload.firmwareSource).toBe("prebuilt");
    });

    it("submits explicit provenance after a manual field change and confirmation", async () => {
        renderComponent("YardForce500B");
        await waitFor(() => expect(targetSummary()).toBeInTheDocument());

        // The board is intentionally changed while the panel keeps following
        // the YardForce500B automatic default.
        openTargetPicker();
        await waitFor(() => expect(selectedLabel(0)).toBe("Mowgli - YardForce 500 B Variant"));
        await chooseOption(0, "Vermut - YardForce 500 Classic");
        await waitFor(() => expect(flashButton()).toBeEnabled());
        fireEvent.click(flashButton());

        const confirmButton = await confirmFlashButton();
        fireEvent.click(confirmButton);
        await waitFor(() => expect(fetchEventSource).toHaveBeenCalledTimes(1));

        const request = vi.mocked(fetchEventSource).mock.calls[0]?.[1] as {body?: string};
        const payload = JSON.parse(request.body ?? "{}") as Record<string, unknown>;
        expect(payload.boardType).toBe("BOARD_VERMUT_YARDFORCE500");
        expect(payload.panelType).toBe("PANEL_TYPE_YARDFORCE_500B_CLASSIC");
        expect(payload.boardTypeOrigin).toBe("manual");
        expect(payload.panelTypeOrigin).toBe("auto");
        expect(payload.firmwareTarget).toBe("");
        expect(payload.firmwareTargetOrigin).toBe("auto");
        expect(payload.firmwareSelectionModel).toBe("YardForce500B");
    });
});

describe("FlashBoardComponent flash progress", () => {
    beforeEach(() => {
        state.savedConfig = "";
        state.settingsModel = "YardForce500";
        state.settingsError = false;
        vi.mocked(fetchEventSource).mockReset();
    });

    const startFlash = async () => {
        let handlers: SseHandlers = {};
        vi.mocked(fetchEventSource).mockImplementation((_url, init) => {
            handlers = init as SseHandlers;
            return new Promise(() => undefined);
        });
        renderComponent("YardForce500");
        await waitFor(() => expect(targetSummary()).toBeInTheDocument());
        fireEvent.click(flashButton());
        fireEvent.click(await confirmFlashButton());
        await waitFor(() => expect(fetchEventSource).toHaveBeenCalledTimes(1));
        return {
            send: (event: string, data: string) => act(() => handlers.onmessage?.({event, data})),
        };
    };

    it("renders the announced plan as steps and advances the bar per stage", async () => {
        const {send} = await startFlash();
        expect(screen.getByText("Flashing firmware...")).toBeInTheDocument();
        expect(screen.getByTestId("flash-status-line")).toHaveTextContent("Starting…");

        send("stage", JSON.stringify({stages: ["manifest", "download", "flash", "verify"], current: 0}));
        send("message", "------> Fetching firmware manifest...");

        const steps = document.querySelectorAll(".ant-steps-item");
        expect(steps).toHaveLength(4);
        expect(steps[0]).toHaveTextContent("Locate firmware");
        expect(steps[3]).toHaveTextContent("Verify");
        expect(document.querySelector(".ant-progress-text")).toHaveTextContent("0%");

        send("stage", JSON.stringify({stages: ["manifest", "download", "flash", "verify"], current: 2}));
        expect(document.querySelector(".ant-progress-text")).toHaveTextContent("50%");
        expect(screen.getByTestId("flash-status-line")).toHaveTextContent(/do not power off/i);
        expect(document.querySelectorAll(".ant-steps-item")[2]).toHaveClass("ant-steps-item-process");

        // The raw log is folded away, not gone.
        expect(screen.getByText("Show detailed log")).toBeInTheDocument();
        expect(screen.queryByText("------> Fetching firmware manifest...")).not.toBeInTheDocument();

        send("end", "end");
        expect(screen.getByText("Flash complete")).toBeInTheDocument();
        expect(document.querySelector(".ant-progress-text")).not.toHaveTextContent("50%");
        expect(screen.getByTestId("flash-status-line")).toHaveTextContent("Firmware flashed successfully!");
        expect(buttonByText("Next")).toBeEnabled();
    });

    it("marks the failing stage and surfaces the log on error", async () => {
        const {send} = await startFlash();
        send("stage", JSON.stringify({stages: ["manifest", "download", "flash", "verify"], current: 1}));
        send("message", "------> Error fetching manifest: 404");
        send("error", "downloading prebuilt firmware: 404");

        expect(screen.getByText("Flash failed")).toBeInTheDocument();
        expect(screen.getByTestId("flash-status-line")).toHaveTextContent("Error: downloading prebuilt firmware: 404");
        expect(document.querySelectorAll(".ant-steps-item")[1]).toHaveClass("ant-steps-item-error");
        expect(screen.getByText("------> Error fetching manifest: 404")).toBeInTheDocument();
        expect(buttonByText("Back to config")).toBeInTheDocument();
    });

    it("ignores a malformed stage event", async () => {
        const {send} = await startFlash();
        send("stage", JSON.stringify({stages: ["flash"], current: 5}));
        send("stage", "not json");

        expect(document.querySelectorAll(".ant-steps-item")).toHaveLength(0);
        expect(screen.getByTestId("flash-status-line")).toHaveTextContent("Starting…");
    });
});
