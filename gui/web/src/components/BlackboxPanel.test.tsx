import {cleanup, fireEvent, render, screen, waitFor} from "@testing-library/react";
import {App} from "antd";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {BlackboxPanel} from "./BlackboxPanel";
import type {BlackboxStatus} from "../hooks/useBlackbox";

const fixture = vi.hoisted(() => ({status: null as BlackboxStatus | null, error: null as string | null,
    busy: false, save: vi.fn(), configure: vi.fn(), remove: vi.fn(), downloadUrl: (name: string) => `/api/tools/blackbox/download/${name}`}));
vi.mock("../hooks/useBlackbox", () => ({useBlackbox: () => fixture}));

describe("Diagnostics blackbox", () => {
    afterEach(cleanup);
    beforeEach(() => {
        vi.clearAllMocks(); fixture.error = null; fixture.busy = false;
        fixture.save.mockResolvedValue(undefined); fixture.configure.mockResolvedValue(undefined);
        fixture.status = {
            config: {enabled: true, pre_seconds: 60, post_seconds: 15, memory_bytes: 16 << 20, max_message_bytes: 8192,
                max_snapshots: 20, max_disk_bytes: 256 << 20, cooldown_seconds: 60, min_free_disk_bytes: 32 << 20},
            phase: "ready", buffered_seconds: 12.5, buffered_bytes: 1024, effective_memory_bytes: 8 << 20, dropped_messages: 3,
            memory_pressure: false, topics: [], recordings: [{name: "sample.jsonl", size: 1024, triggered_at: "2026-10-10T00:00:00Z",
                reasons: ["emergency"], actual_pre_seconds: 12.5, actual_post_seconds: 15, capture_dropped_messages: 0, interrupted: false}],
        };
    });
    it("shows actual coverage, settings and a direct download, and saves manually", async () => {
        render(<App><BlackboxPanel/></App>);
        expect(screen.getByText("12.5 s before · 15.0 s after")).toBeInTheDocument();
        expect(screen.getByRole("link", {name: /Download/})).toHaveAttribute("href", "/api/tools/blackbox/download/sample.jsonl");
        expect(screen.getByLabelText("Maximum memory (MiB)")).toHaveValue("16");
        fireEvent.click(screen.getByRole("button", {name: /Save blackbox now/}));
        await waitFor(() => expect(fixture.save).toHaveBeenCalledOnce());
    });
    it("prevents duplicate captures and config changes during a capture", () => {
        fixture.status!.phase = "capturing";
        render(<App><BlackboxPanel/></App>);
        expect(screen.getByRole("button", {name: /Save blackbox now/})).toBeDisabled();
        expect(screen.getByRole("button", {name: "Apply settings"})).toBeDisabled();
        expect(screen.getByText("Collecting post-event history")).toBeInTheDocument();
    });
    it("exposes backend failures and memory pressure without hiding saved files", () => {
        fixture.error = "Disconnected"; fixture.status!.memory_pressure = true;
        render(<App><BlackboxPanel/></App>);
        expect(screen.getByText("Disconnected")).toBeInTheDocument();
        expect(screen.getByText(/Recording paused under memory pressure/)).toBeInTheDocument();
        expect(screen.getByRole("link", {name: /Download/})).toBeInTheDocument();
    });
    it("applies labelled resource limits preserving advanced settings", async () => {
        render(<App><BlackboxPanel/></App>);
        fireEvent.change(screen.getByLabelText("History before event (seconds)"), {target: {value: "30"}});
        fireEvent.click(screen.getByRole("button", {name: "Apply settings"}));
        await waitFor(() => expect(fixture.configure).toHaveBeenCalledWith(expect.objectContaining({pre_seconds: 30, max_message_bytes: 8192})));
    });
});
