import {App} from "antd";
import {act, render, screen, waitFor} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {IrriSenseSection} from "./IrriSenseSection.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";
import type {IrriSenseSettings, SoilStatus} from "../../types/irrisense.ts";

const requestMock = vi.fn();

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({request: requestMock}),
}));

const defaultSettings: IrriSenseSettings = {
    enabled: false,
    baseUrl: "https://irrisense-cloud.fly.dev",
    tokenSet: false,
    tokenMasked: "",
    gardenId: "",
    zoneIds: [],
    wetDeficitMm: 2,
    dryAfterWateringHours: 3,
    maxStaleMinutes: 90,
    gateScheduler: true,
};

const unknownStatus: SoilStatus = {
    enabled: false, configured: false, gateScheduler: true,
    fresh: false, wet: false, unknown: true, reason: "IrriSense integration disabled", zones: [],
};

type Req = {path: string; method: string; body?: unknown};

/** A tiny fake of the backend: settings round-trip through `stored`. */
function installFakeBackend(initial: IrriSenseSettings = defaultSettings) {
    let stored = {...initial};
    requestMock.mockImplementation((req: Req) => {
        if (req.path === "/irrisense/settings" && req.method === "GET") return Promise.resolve({data: stored, error: null});
        if (req.path === "/irrisense/settings" && req.method === "PUT") {
            const body = req.body as Record<string, unknown>;
            stored = {
                ...stored,
                ...Object.fromEntries(Object.entries(body).filter(([k]) => k !== "token" && k !== "clearToken")),
                tokenSet: body.clearToken ? false : body.token ? true : stored.tokenSet,
                tokenMasked: body.clearToken ? "" : body.token ? "irs_••••••••" : stored.tokenMasked,
            } as IrriSenseSettings;
            return Promise.resolve({data: stored, error: null});
        }
        if (req.path === "/irrisense/status") return Promise.resolve({data: unknownStatus, error: null});
        if (req.path === "/irrisense/gardens") {
            return Promise.resolve({data: {gardens: [{id: "g1", name: "Jardin", zones: [{id: "z1", label: "Pelouse", enabled: true}]}]}, error: null});
        }
        return Promise.reject(new Error(`unexpected request ${req.method} ${req.path}`));
    });
    return () => stored;
}

const putCalls = () => requestMock.mock.calls
    .map(([req]) => req as Req)
    .filter((req) => req.path === "/irrisense/settings" && req.method === "PUT");

// The section has no Save button of its own: it registers with the settings
// page (useSettingsManager) and the page's single Save button calls it.
let savers: Record<string, ExternalSaver> = {};
const registerSaver = (id: string, saver: ExternalSaver) => { savers[id] = saver; };
const unregisterSaver = (id: string) => { delete savers[id]; };
async function saveViaPage() {
    await act(async () => { await savers["irrisense"].save(); });
}

function renderSection() {
    savers = {};
    return render(
        <App>
            <IrriSenseSection registerSaver={registerSaver} unregisterSaver={unregisterSaver}/>
        </App>,
    );
}

describe("IrriSenseSection", () => {
    beforeEach(() => {
        requestMock.mockReset();
    });

    it("loads the settings and hides the controls until enabled", async () => {
        installFakeBackend();
        renderSection();

        const toggle = await screen.findByRole("switch", {name: /IrriSense Cloud soil moisture/i});
        expect(toggle).not.toBeChecked();
        expect(screen.queryByLabelText(/Read-only API token/i)).not.toBeInTheDocument();
        expect(screen.queryByRole("button", {name: /Save IrriSense settings/i})).not.toBeInTheDocument();
        expect(savers["irrisense"]?.dirtyCount ?? 0).toBe(0);
    });

    it("stores a pasted token on save without ever echoing it", async () => {
        const stored = installFakeBackend();
        const user = userEvent.setup();
        renderSection();

        await user.click(await screen.findByRole("switch", {name: /IrriSense Cloud soil moisture/i}));
        await user.type(screen.getByLabelText(/Read-only API token/i), "irs_secret_token_123");
        await user.click(screen.getByRole("button", {name: /^Set$/i}));
        expect(screen.getByTestId("irrisense-token-state")).toHaveTextContent(/stored on save/i);

        await saveViaPage();

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        expect(putCalls()[0].body).toMatchObject({enabled: true, token: "irs_secret_token_123"});
        await waitFor(() => expect(screen.getByTestId("irrisense-token-state")).toHaveTextContent("irs_••••••••"));
        expect(stored().tokenSet).toBe(true);
        expect(screen.queryByDisplayValue("irs_secret_token_123")).not.toBeInTheDocument();
    });

    it("sends clearToken when the operator clears a stored token", async () => {
        installFakeBackend({...defaultSettings, enabled: true, tokenSet: true, tokenMasked: "irs_••••••••"});
        const user = userEvent.setup();
        renderSection();

        expect(await screen.findByTestId("irrisense-token-state")).toHaveTextContent("irs_••••••••");
        await user.click(screen.getByRole("button", {name: /^Clear$/i}));
        await saveViaPage();

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        expect(putCalls()[0].body).toMatchObject({clearToken: true});
        expect((putCalls()[0].body as Record<string, unknown>).token).toBeUndefined();
    });

    it("test connection saves pending edits first, then lists the gardens", async () => {
        installFakeBackend({...defaultSettings, enabled: true, tokenSet: true, tokenMasked: "irs_••••••••"});
        const user = userEvent.setup();
        renderSection();

        const url = await screen.findByLabelText(/Service URL/i);
        await user.clear(url);
        await user.type(url, "https://example.test");
        await user.click(screen.getByRole("button", {name: /Test connection/i}));

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        expect(putCalls()[0].body).toMatchObject({baseUrl: "https://example.test"});
        await waitFor(() => expect(requestMock.mock.calls.some(([req]) => (req as Req).path === "/irrisense/gardens")).toBe(true));
        expect(await screen.findByText(/IrriSense reachable: 1 garden/i)).toBeInTheDocument();
    });

    it("shows the live verdict line when enabled", async () => {
        installFakeBackend({...defaultSettings, enabled: true});
        renderSection();

        expect(await screen.findByTestId("irrisense-verdict")).toHaveTextContent("Unknown");
        expect(screen.getByTestId("irrisense-reason")).toHaveTextContent("IrriSense integration disabled");
    });
});
