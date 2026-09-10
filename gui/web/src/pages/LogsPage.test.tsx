import {beforeEach, describe, expect, it, vi} from "vitest";
import {act, fireEvent, render, screen} from "@testing-library/react";
import {App} from "antd";
import {ThemeProvider} from "../theme/ThemeContext.tsx";
import {TimeFormatProvider} from "../hooks/useTimeFormat.tsx";

// The page's only inputs are the log WebSocket and the container list, so both
// are stubbed and the stream is driven by hand.
type StreamHandler = (data: string, first?: boolean) => void;
let pushLine: StreamHandler = () => { /* replaced on render */ };

vi.mock("../hooks/useWS.ts", () => ({
    useWS: (_onError: unknown, _onInfo: unknown, onData: StreamHandler) => {
        pushLine = onData;
        return {start: vi.fn(), stop: vi.fn()};
    },
}));

vi.mock("../hooks/useApi.ts", () => ({
    useApi: () => ({
        containers: {
            containersList: () => Promise.resolve({
                data: {
                    containers: [{
                        id: "mock-container",
                        names: ["/mock-container"],
                        state: "running",
                        labels: {app: "mock"},
                    }],
                },
            }),
            containersCreate: vi.fn(),
        },
    }),
}));

import LogsPage from "./LogsPage.tsx";

// 1747087353.123 s == 2025-05-12T22:02:33.123Z
const ROS_LINE = "[INFO] [1747087353.123456789] [map_server_node]: planning";
const TIMESTAMPLESS_LINE = "INFO synthetic log line 999";

async function renderLogsPage() {
    render(
        <ThemeProvider>
            <TimeFormatProvider>
                <App>
                    <LogsPage/>
                </App>
            </TimeFormatProvider>
        </ThemeProvider>,
    );
    // Let the container list resolve first: selecting a container re-opens the
    // stream and clears the buffer, which would wipe anything pushed earlier.
    await act(async () => {
        await new Promise((resolve) => setTimeout(resolve, 0));
    });
}

describe("LogsPage timestamps", () => {
    beforeEach(() => {
        const storage = new Map<string, string>();
        Object.defineProperty(window, "localStorage", {
            configurable: true,
            value: {
                getItem: (key: string) => storage.get(key) ?? null,
                setItem: (key: string, value: string) => storage.set(key, value),
                removeItem: (key: string) => storage.delete(key),
                clear: () => storage.clear(),
            },
        });
    });

    async function pushAndFlush(lines: string[]) {
        // The page batches stream pushes on a 100 ms timer.
        await act(async () => {
            lines.forEach((line, index) => pushLine(line, index === 0));
            await new Promise((resolve) => setTimeout(resolve, 200));
        });
    }

    it("renders a readable timestamp column for a ROS epoch line and drops the raw epoch", async () => {
        // Arrange
        await renderLogsPage();

        // Act
        await pushAndFlush([ROS_LINE]);

        // Assert — the epoch token is gone from the body...
        expect(screen.getByText("[INFO] [map_server_node]: planning")).toBeInTheDocument();
        expect(screen.queryByText(/1747087353\.123456789/)).not.toBeInTheDocument();
        // ...and a readable stamp took its place.
        expect(screen.getByText(/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$/)).toBeInTheDocument();
    });

    it("falls back to the receive time for a line that carries no timestamp", async () => {
        // Arrange
        await renderLogsPage();

        // Act
        await pushAndFlush([TIMESTAMPLESS_LINE]);

        // Assert — the body text node stays exactly intact (the e2e spec
        // asserts on it with `exact: true`), and a stamp is still rendered.
        expect(screen.getByText(TIMESTAMPLESS_LINE)).toBeInTheDocument();
        expect(screen.getByText(/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$/)).toBeInTheDocument();
    });

    async function toggleUtc() {
        await act(async () => {
            screen.getByRole("button", {name: /Switch between local time and UTC/i}).click();
            await Promise.resolve();
        });
    }

    async function typeSearch(query: string) {
        const input = screen.getByPlaceholderText(/Search logs/i);
        await act(async () => {
            fireEvent.change(input, {target: {value: query}});
            await Promise.resolve();
        });
    }

    // Issue #207 moved the timestamp out of the line body into its own column,
    // which silently broke searching for a time. These three fail without the
    // widened `filtered` predicate in LogsPage.tsx.
    it("keeps a line whose rendered timestamp matches the query", async () => {
        // Arrange — UTC so the rendered column is deterministic on any runner.
        await renderLogsPage();
        await pushAndFlush([ROS_LINE]);
        await toggleUtc();

        // Act — a fragment of the rendered column, absent from the body.
        await typeSearch("22:02");

        // Assert
        expect(screen.getByText("[INFO] [map_server_node]: planning")).toBeInTheDocument();
        expect(screen.queryByText(/No lines match the active filters/i)).not.toBeInTheDocument();
    });

    it("keeps a line whose raw epoch seconds match a pasted query", async () => {
        // Arrange
        await renderLogsPage();
        await pushAndFlush([ROS_LINE]);

        // Act — the epoch the operator pasted out of an older console dump.
        await typeSearch("1747087353");

        // Assert
        expect(screen.getByText("[INFO] [map_server_node]: planning")).toBeInTheDocument();
    });

    it("still filters out a line that matches neither body nor timestamp", async () => {
        // Arrange
        await renderLogsPage();
        await pushAndFlush([ROS_LINE]);

        // Act
        await typeSearch("zzz");

        // Assert
        expect(screen.queryByText("[INFO] [map_server_node]: planning")).not.toBeInTheDocument();
        expect(screen.getByText(/No lines match the active filters/i)).toBeInTheDocument();
    });

    it("re-renders the column in UTC when the time zone chip is toggled", async () => {
        // Arrange
        await renderLogsPage();
        await pushAndFlush([ROS_LINE]);
        const before = screen.getByText(/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$/).textContent;

        // Act
        await act(async () => {
            screen.getByRole("button", {name: /Switch between local time and UTC/i}).click();
            await Promise.resolve();
        });

        // Assert — the same instant, rendered as UTC.
        const after = screen.getByText(/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$/).textContent;
        expect(after).toBe("2025-05-12T22:02:33");
        expect(before).not.toBeNull();
    });
});
