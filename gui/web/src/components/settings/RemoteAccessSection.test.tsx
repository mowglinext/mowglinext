import {App} from "antd";
import {act, render, screen, waitFor} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {RemoteAccessSection} from "./RemoteAccessSection.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";
import type {RemoteAccessSettings, RemoteAccessStatus} from "../../types/remoteAccess.ts";

const requestMock = vi.fn();

vi.mock("../../hooks/useApi.ts", () => ({
    useApi: () => ({request: requestMock}),
}));

const defaultSettings: RemoteAccessSettings = {
    enabled: false,
    hostname: "mowgli",
    authKeySet: false,
    authKeyMasked: "",
    serveHttps: true,
    image: "tailscale/tailscale:v1.102.3",
    defaultImage: "tailscale/tailscale:v1.102.3",
    containerName: "mowgli-remote",
};

const disabledStatus: RemoteAccessStatus = {
    enabled: false, phase: "disabled", hostname: "mowgli", tailscaleIps: [], httpUrls: [],
    magicDnsEnabled: false, health: [], checkedAt: "2026-09-15T12:00:00Z",
};

type Req = {path: string; method: string; body?: unknown};

/** A tiny fake of the backend: settings round-trip through `stored`. */
function installFakeBackend(initial: RemoteAccessSettings = defaultSettings, status: RemoteAccessStatus = disabledStatus) {
    let stored = {...initial};
    requestMock.mockImplementation((req: Req) => {
        if (req.path === "/remote-access/settings" && req.method === "GET") return Promise.resolve({data: stored, error: null});
        if (req.path === "/remote-access/settings" && req.method === "PUT") {
            const body = req.body as Record<string, unknown>;
            stored = {
                ...stored,
                ...Object.fromEntries(Object.entries(body).filter(([k]) => k !== "authKey" && k !== "clearAuthKey")),
                authKeySet: body.clearAuthKey ? false : body.authKey ? true : stored.authKeySet,
                authKeyMasked: body.clearAuthKey ? "" : body.authKey ? "tskey-auth-••••••••" : stored.authKeyMasked,
            } as RemoteAccessSettings;
            return Promise.resolve({data: stored, error: null});
        }
        if (req.path === "/remote-access/status") return Promise.resolve({data: status, error: null});
        if (req.path === "/remote-access/apply" || req.path === "/remote-access/logout") return Promise.resolve({data: {ok: "ok"}, error: null});
        return Promise.reject(new Error(`unexpected request ${req.method} ${req.path}`));
    });
    return () => stored;
}

const putCalls = () => requestMock.mock.calls
    .map(([req]) => req as Req)
    .filter((req) => req.path === "/remote-access/settings" && req.method === "PUT");

let savers: Record<string, ExternalSaver> = {};
const registerSaver = (id: string, saver: ExternalSaver) => { savers[id] = saver; };
const unregisterSaver = (id: string) => { delete savers[id]; };
async function saveViaPage() {
    await act(async () => { await savers["remote_access"].save(); });
}

function renderSection() {
    savers = {};
    return render(
        <App>
            <RemoteAccessSection registerSaver={registerSaver} unregisterSaver={unregisterSaver}/>
        </App>,
    );
}

describe("RemoteAccessSection", () => {
    beforeEach(() => {
        requestMock.mockReset();
    });

    it("loads settings and hides the form while disabled", async () => {
        installFakeBackend();
        renderSection();

        const toggle = await screen.findByRole("switch", {name: /settingsRemoteAccess.title|Remote access/i});
        expect(toggle).toHaveAttribute("aria-checked", "false");
        expect(screen.queryByTestId("remote-access-key-state")).toBeNull();
        // Disabled → no status polling, no docker exec on the robot.
        expect(requestMock.mock.calls.some(([req]) => (req as Req).path === "/remote-access/status")).toBe(false);
    });

    it("enabling, naming and setting a key is sent as one partial PUT on page save", async () => {
        const stored = installFakeBackend();
        renderSection();
        const user = userEvent.setup();

        await user.click(await screen.findByRole("switch", {name: /settingsRemoteAccess.title|Remote access/i}));
        const hostname = await screen.findByRole("textbox", {name: /settingsRemoteAccess.hostname|Node name/i});
        await user.clear(hostname);
        await user.type(hostname, "Lawn-Bot");
        const keyBox = screen.getByLabelText(/settingsRemoteAccess.key$|Auth key/i);
        await user.type(keyBox, "tskey-auth-abc123-secret");
        await user.click(screen.getByRole("button", {name: /settingsRemoteAccess.keyApply|^Set$/i}));
        expect(screen.getByTestId("remote-access-key-state").textContent).toMatch(/keyPending|will be stored/i);

        expect(savers["remote_access"].dirtyCount).toBe(3);
        await saveViaPage();

        expect(putCalls()).toHaveLength(1);
        expect(putCalls()[0].body).toEqual({enabled: true, hostname: "lawn-bot", authKey: "tskey-auth-abc123-secret"});
        expect(stored().enabled).toBe(true);
        expect(stored().authKeySet).toBe(true);
        await waitFor(() => expect(savers["remote_access"].dirtyCount).toBe(0));
    });

    it("shows the login link while the node waits for approval", async () => {
        installFakeBackend({...defaultSettings, enabled: true}, {
            ...disabledStatus, enabled: true, phase: "running", containerState: "running",
            backendState: "NeedsLogin", loginUrl: "https://login.tailscale.com/a/abc123",
        });
        renderSection();

        const link = await screen.findByTestId("remote-access-login-link");
        expect(link).toHaveAttribute("href", "https://login.tailscale.com/a/abc123");
        expect(screen.getByTestId("remote-access-verdict").textContent).toMatch(/needsLogin|Login required/i);
    });

    it("lists the reachable URLs once connected, HTTPS first", async () => {
        installFakeBackend({...defaultSettings, enabled: true}, {
            ...disabledStatus, enabled: true, phase: "running", containerState: "running", backendState: "Running",
            dnsName: "mowgli.tail1234.ts.net", magicDnsEnabled: true,
            tailscaleIps: ["100.64.0.7"], httpsUrl: "https://mowgli.tail1234.ts.net",
            httpUrls: ["http://mowgli.tail1234.ts.net:4006", "http://100.64.0.7:4006"],
        });
        renderSection();

        const urls = await screen.findByTestId("remote-access-urls");
        const links = Array.from(urls.querySelectorAll("a")).map((a) => a.getAttribute("href"));
        expect(links).toEqual([
            "https://mowgli.tail1234.ts.net",
            "http://mowgli.tail1234.ts.net:4006",
            "http://100.64.0.7:4006",
        ]);
        expect(screen.getByTestId("remote-access-verdict").textContent).toMatch(/connected/i);
    });

    it("offers a retry that posts to /remote-access/apply after an error", async () => {
        installFakeBackend({...defaultSettings, enabled: true}, {
            ...disabledStatus, enabled: true, phase: "error", error: "pull tailscale/tailscale: registry unreachable",
        });
        renderSection();
        const user = userEvent.setup();

        await screen.findByText(/registry unreachable/);
        await user.click(screen.getByRole("button", {name: /settingsRemoteAccess.retry|Retry/i}));

        await waitFor(() => expect(requestMock.mock.calls.some(([req]) => (req as Req).path === "/remote-access/apply")).toBe(true));
    });

    it("revert drops pending edits", async () => {
        installFakeBackend();
        renderSection();
        const user = userEvent.setup();

        await user.click(await screen.findByRole("switch", {name: /settingsRemoteAccess.title|Remote access/i}));
        expect(savers["remote_access"].dirtyCount).toBe(1);

        act(() => savers["remote_access"].revert?.());

        await waitFor(() => expect(savers["remote_access"].dirtyCount).toBe(0));
        expect(putCalls()).toHaveLength(0);
    });
});
