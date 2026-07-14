import {pack} from "msgpackr";
import type {Page, Route} from "@playwright/test";
import type {Scenario} from "./scenarios.ts";

/**
 * Neutral REST responses so every page renders without the real Go backend.
 * A scenario's `rest` map overrides individual pathnames. Keys are matched by
 * pathname (query string ignored).
 */
const DEFAULT_REST: Record<string, unknown> = {
    // Critical: keeps AppShell from redirecting to /onboarding.
    "/api/settings/status": {onboarding_completed: true},
    "/api/settings": {},
    "/api/settings/yaml": {},
    "/api/settings/yaml/defaults": {},
    "/api/settings/schema": {properties: {}},
    "/api/weather": {current: {temp_c: 19, condition: "clear", is_raining: false}, daily: []},
    "/api/diagnostics/snapshot": {coverage: [], crossChecks: {status: "ok", warnings: []}, containers: []},
    "/api/diagnostics/sessions": {sessions: []},
    "/api/diagnostics/sessions/stats": {total_sessions: 0, total_area_m2: 0, total_seconds: 0},
    "/api/calibration/status": {imu: {present: false}, magnetometer: {present: false}},
    "/api/schedules": {schedules: []},
    "/api/containers": {containers: []},
    "/api/params": {parameters: []},
    "/api/settings/gnss/runtime-config": {device: "", baud: 0},
};

const okJson = (route: Route, body: unknown) =>
    route.fulfill({status: 200, contentType: "application/json", body: JSON.stringify(body)});

/**
 * Install all mocks for a scenario on a page. Call BEFORE page.goto so the
 * first render already sees mocked data.
 */
export async function installMockBackend(page: Page, scenario: Scenario): Promise<void> {
    const rest = {...DEFAULT_REST, ...(scenario.rest ?? {})};

    // ---- REST ---------------------------------------------------------------
    // Anchor to a "/api/" path ROOT — a bare "**/api/**" glob also matches the
    // app's own vite modules (e.g. /src/api/Api.ts) and would break the boot.
    await page.route(/^https?:\/\/[^/]+\/api\//, async (route) => {
        const url = new URL(route.request().url());
        if (url.pathname.includes("/multiplex") || url.pathname.includes("/subscribe/")) {
            return route.continue(); // WebSocket upgrade — handled by routeWebSocket
        }
        if (url.pathname in rest) return okJson(route, rest[url.pathname]);
        // POST /api/config/keys/get echoes an empty value map; everything else
        // gets a benign empty object so no fetch throws.
        if (url.pathname === "/api/config/keys/get") return okJson(route, {});
        return okJson(route, {});
    });

    // ---- Multiplex WebSocket (msgpack binary frames) ------------------------
    const topics = scenario.topics ?? {};
    await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, (ws) => {
        // Mock mode: no upstream server. We answer subscribe ops directly.
        ws.onMessage((message) => {
            if (scenario.silentSocket) return;
            let op: {op?: string; topic?: string};
            try {
                op = JSON.parse(typeof message === "string" ? message : message.toString());
            } catch {
                return;
            }
            if (op.op !== "subscribe" || !op.topic) return;
            const data = topics[op.topic];
            if (data === undefined) return;
            // Wire format matches MultiplexRoute: msgpack({topic, data}).
            ws.send(pack({topic: op.topic, data}));
        });
    });

    // Dedicated (non-multiplex) subscribe sockets used by a few pages: accept
    // and stay silent so nothing errors.
    await page.routeWebSocket(/\/api\/mowglinext\/subscribe\//, () => { /* silent */ });
}
