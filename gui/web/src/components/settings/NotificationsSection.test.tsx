import {App} from "antd";
import {act, render, screen, waitFor} from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import {beforeEach, describe, expect, it, vi} from "vitest";
import {NotificationsSection} from "./NotificationsSection.tsx";
import type {ExternalSaver} from "../../hooks/useSettingsManager.ts";
import {
    NOTIFICATION_EVENT_KINDS,
    type NotificationDeliveryStatus,
    type NotificationSettings,
} from "../../types/notifications.ts";

const requestMock = vi.fn();

// One stable instance, like the real useApi (a module singleton): a fresh
// object per render would re-run every guiApi-keyed effect on each render.
vi.mock("../../hooks/useApi.ts", () => {
    const fakeApi = {request: (...args: unknown[]): unknown => requestMock(...args)};
    return {useApi: () => fakeApi};
});

const defaultSettings: NotificationSettings = {
    enabled: false,
    channel: "telegram",
    language: "en",
    title: "MowgliNext",
    ntfyServer: "https://ntfy.sh",
    ntfyTopic: "",
    ntfyTokenSet: false,
    ntfyTokenMasked: "",
    telegramBotTokenSet: false,
    telegramBotTokenMasked: "",
    telegramChatId: "",
    pushoverAppTokenSet: false,
    pushoverAppTokenMasked: "",
    pushoverUserKey: "",
    webhookUrl: "",
    events: Object.fromEntries(NOTIFICATION_EVENT_KINDS.map((k) => [k, k !== "mowStopped"])),
    eventKinds: [...NOTIFICATION_EVENT_KINDS],
    channels: ["telegram", "pushover", "ntfy", "webhook"],
};

const idleStatus: NotificationDeliveryStatus = {
    enabled: false, configured: false, channel: "telegram", sentCount: 0, failedCount: 0,
};

type Req = {path: string; method: string; body?: unknown};

/** A tiny fake of the backend: settings round-trip through `stored`. */
function installFakeBackend(initial: NotificationSettings = defaultSettings, testStatus = 200) {
    let stored = {...initial};
    let testsSent = 0;
    requestMock.mockImplementation((req: Req) => {
        if (req.path === "/notifications/settings" && req.method === "GET") return Promise.resolve({data: stored, error: null});
        if (req.path === "/notifications/settings" && req.method === "PUT") {
            const body = req.body as Record<string, unknown>;
            const secretKeys = ["ntfyToken", "clearNtfyToken", "telegramBotToken", "clearTelegramBotToken", "pushoverAppToken", "clearPushoverAppToken"];
            stored = {
                ...stored,
                ...Object.fromEntries(Object.entries(body).filter(([k]) => !secretKeys.includes(k) && k !== "events")),
                events: {...stored.events, ...((body.events as Record<string, boolean>) ?? {})},
                ntfyTokenSet: body.clearNtfyToken ? false : body.ntfyToken ? true : stored.ntfyTokenSet,
                ntfyTokenMasked: body.clearNtfyToken ? "" : body.ntfyToken ? "tk_a••••••••" : stored.ntfyTokenMasked,
                telegramBotTokenSet: body.clearTelegramBotToken ? false : body.telegramBotToken ? true : stored.telegramBotTokenSet,
                telegramBotTokenMasked: body.clearTelegramBotToken ? "" : body.telegramBotToken ? "1234••••••••" : stored.telegramBotTokenMasked,
            } as NotificationSettings;
            return Promise.resolve({data: stored, error: null});
        }
        if (req.path === "/notifications/status") {
            const configured = stored.channel === "ntfy" ? stored.ntfyTopic !== "" : stored.telegramBotTokenSet && stored.telegramChatId !== "";
            return Promise.resolve({data: {...idleStatus, channel: stored.channel, configured, enabled: stored.enabled, sentCount: testsSent}, error: null});
        }
        if (req.path === "/notifications/test") {
            if (testStatus >= 400) return Promise.reject(new Error("ntfy: HTTP 403: forbidden"));
            testsSent += 1;
            return Promise.resolve({data: {ok: "true"}, error: null});
        }
        return Promise.reject(new Error(`unexpected request ${req.method} ${req.path}`));
    });
    return () => stored;
}

const putCalls = () => requestMock.mock.calls
    .map(([req]) => req as Req)
    .filter((req) => req.path === "/notifications/settings" && req.method === "PUT");
const testCalls = () => requestMock.mock.calls
    .map(([req]) => req as Req)
    .filter((req) => req.path === "/notifications/test");

let savers: Record<string, ExternalSaver> = {};
const registerSaver = (id: string, saver: ExternalSaver) => { savers[id] = saver; };
const unregisterSaver = (id: string) => { delete savers[id]; };
async function saveViaPage() {
    await act(async () => { await savers["notifications"].save(); });
}

function renderSection() {
    savers = {};
    return render(
        <App>
            <NotificationsSection registerSaver={registerSaver} unregisterSaver={unregisterSaver}/>
        </App>,
    );
}

describe("NotificationsSection", () => {
    beforeEach(() => {
        requestMock.mockReset();
    });

    it("loads the settings, lists every event toggle and registers a clean saver", async () => {
        installFakeBackend();
        renderSection();

        const toggle = await screen.findByRole("switch", {name: /^Push notifications$/i});
        expect(toggle).not.toBeChecked();
        expect(screen.getByRole("switch", {name: /Stuck \/ blocked/i})).toBeChecked();
        expect(screen.getByRole("switch", {name: /Mowing stopped/i})).not.toBeChecked();
        expect(screen.getByLabelText(/Bot token/i)).toBeInTheDocument();
        expect(screen.queryByLabelText(/^Topic$/i)).not.toBeInTheDocument();
        expect(savers["notifications"]?.dirtyCount ?? 0).toBe(0);
        expect(screen.getByText(/Channel not configured/i)).toBeInTheDocument();
    });

    it("saves bot token, chat id, enable flag and event toggles through the page saver", async () => {
        const stored = installFakeBackend();
        const user = userEvent.setup();
        renderSection();

        await user.click(await screen.findByRole("switch", {name: /^Push notifications$/i}));
        await user.type(screen.getByLabelText(/Bot token/i), "123456:abcdef");
        await user.click(screen.getByRole("button", {name: /^Set$/i}));
        await user.type(screen.getByLabelText(/Chat id/i), "-100424242");
        await user.click(screen.getByRole("switch", {name: /Zone started/i}));
        expect(savers["notifications"].dirtyCount).toBeGreaterThan(0);

        await saveViaPage();

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        expect(putCalls()[0].body).toMatchObject({enabled: true, telegramBotToken: "123456:abcdef", telegramChatId: "-100424242", events: {zoneStarted: false}});
        expect(screen.queryByDisplayValue("123456:abcdef")).not.toBeInTheDocument();
        expect(stored().enabled).toBe(true);
        expect(stored().events.zoneStarted).toBe(false);
        expect(savers["notifications"].dirtyCount).toBe(0);
    });

    it("stores a pasted ntfy token without ever echoing it", async () => {
        installFakeBackend({...defaultSettings, channel: "ntfy"});
        const user = userEvent.setup();
        renderSection();

        await user.type(await screen.findByLabelText(/Access token/i), "tk_abcdefghijklmnop");
        await user.click(screen.getByRole("button", {name: /^Set$/i}));
        expect(screen.getByText(/Will be stored on save/i)).toBeInTheDocument();

        await saveViaPage();

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        expect(putCalls()[0].body).toMatchObject({ntfyToken: "tk_abcdefghijklmnop"});
        await waitFor(() => expect(screen.getByText(/Stored \(tk_a••••••••\)/)).toBeInTheDocument());
        expect(screen.queryByDisplayValue("tk_abcdefghijklmnop")).not.toBeInTheDocument();
    });

    it("switching to Pushover shows its fields and hides the Telegram ones", async () => {
        installFakeBackend();
        const user = userEvent.setup();
        renderSection();

        await user.click(await screen.findByRole("combobox", {name: /^Channel$/i}));
        // antd renders each option twice (a hidden a11y listbox entry and the
        // visible dropdown item); only the visible item reacts to a click.
        const items = await screen.findAllByText(/^Pushover$/i);
        await user.click(items.find((el) => el.closest(".ant-select-item")) ?? items[items.length - 1]);

        expect(await screen.findByLabelText(/Application API token/i)).toBeInTheDocument();
        expect(screen.getByLabelText(/User key/i)).toBeInTheDocument();
        expect(screen.queryByLabelText(/Bot token/i)).not.toBeInTheDocument();
    });

    it("send test saves pending edits first, then posts the test", async () => {
        installFakeBackend({...defaultSettings, channel: "ntfy"});
        const user = userEvent.setup();
        renderSection();

        await user.type(await screen.findByLabelText(/^Topic$/i), "mowgli-garden");
        await user.click(screen.getByRole("button", {name: /Send test notification/i}));

        await waitFor(() => expect(putCalls()).toHaveLength(1));
        await waitFor(() => expect(testCalls()).toHaveLength(1));
        expect(await screen.findByText(/Test notification sent/i)).toBeInTheDocument();
    });

    it("surfaces a failed test with the backend's reason", async () => {
        installFakeBackend({...defaultSettings, channel: "ntfy", ntfyTopic: "mowgli-garden"}, 403);
        const user = userEvent.setup();
        renderSection();

        await user.click(await screen.findByRole("button", {name: /Send test notification/i}));

        expect(await screen.findByText(/Test notification failed/i)).toBeInTheDocument();
        expect(screen.getByText(/HTTP 403/)).toBeInTheDocument();
        expect(putCalls()).toHaveLength(0);
    });
});
