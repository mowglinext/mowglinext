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
    "/api/settings/hardware-backend": {
        backend: "mowgli",
        supported: ["mowgli", "mavros"],
        runtime_routing: "available",
        parameter_routes: {
            ticks_per_meter: {parameter: "hardware_bridge.ticks_per_meter", runtime: "available"},
            wheel_pid_kp: {parameter: "hardware_bridge.wheel_pid_kp", runtime: "available"},
            wheel_pid_ki: {parameter: "hardware_bridge.wheel_pid_ki", runtime: "available"},
            wheel_pid_kd: {parameter: "hardware_bridge.wheel_pid_kd", runtime: "available"},
            wheel_pid_integral_limit: {
                parameter: "hardware_bridge.wheel_pid_integral_limit",
                runtime: "available",
            },
            wheel_pid_pwm_per_mps: {
                parameter: "hardware_bridge.wheel_pid_pwm_per_mps",
                runtime: "available",
            },
        },
    },
    "/api/weather": {current: {temp_c: 19, condition: "clear", is_raining: false}, daily: []},
    "/api/diagnostics/snapshot": {coverage: [], crossChecks: {status: "ok", warnings: []}, containers: []},
    "/api/diagnostics/sessions": {sessions: []},
    "/api/diagnostics/sessions/stats": {total_sessions: 0, total_area_m2: 0, total_seconds: 0},
    "/api/calibration/status": {imu: {present: false}, magnetometer: {present: false}},
    "/api/schedules": {schedules: []},
    "/api/containers": {containers: []},
    "/api/system/updates": {channel: 'dev', state: 'not_checked', components: []},
    "/api/system/updates/changelog": {features: [], fixes: [], other: 0, total: 0, truncated: false, url: "https://github.com/mowglinext/mowglinext/compare"},
    "/api/notifications/settings": {enabled: false, channel: "ntfy", language: "en", events: {}, eventKinds: [], channels: ["ntfy"]},
    "/api/notifications/status": {enabled: false, configured: false, channel: "ntfy", sentCount: 0, failedCount: 0},
    "/api/params": {parameters: []},
    "/api/settings/gnss/runtime-config": {device: "", baud: 0},
};

const okJson = (route: Route, body: unknown) =>
    route.fulfill({status: 200, contentType: "application/json", body: JSON.stringify(body)});

/**
 * Install all mocks for a scenario on a page. Call BEFORE page.goto so the
 * first render already sees mocked data.
 */
export async function installMockBackend(page: Page, scenario: Scenario, options?: {liveStatusIntervalMs: number}): Promise<void> {
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
    const topicSequences = scenario.topicSequences ?? {};
    await page.routeWebSocket(/\/api\/mowglinext\/multiplex/, (ws) => {
        let statusTimer: ReturnType<typeof setInterval> | undefined;
        const topicTimers = new Map<string, ReturnType<typeof setInterval>>();
        let sequence = 1;
        ws.onClose(() => {
            clearInterval(statusTimer);
            topicTimers.forEach(clearInterval);
            ws.close();
        });
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
            const values = topicSequences[op.topic];
            const data = values?.[0] ?? topics[op.topic];
            if (data === undefined) return;
            // Wire format matches MultiplexRoute: msgpack({topic, data}).
            ws.send(pack({topic: op.topic, data}));
            if (values && values.length > 1 && options && !topicTimers.has(op.topic)) {
                let index = 1;
                topicTimers.set(op.topic, setInterval(() => {
                    ws.send(pack({topic: op.topic, data: values[index % values.length]}));
                    index++;
                }, options.liveStatusIntervalMs));
            }
            if (op.topic === 'status' && options && !statusTimer) {
                statusTimer = setInterval(() => ws.send(pack({topic: 'status', data: {...data as object, stamp: {sec: ++sequence, nanosec: 0}}})), options.liveStatusIntervalMs);
            }
        });
    });

    await page.routeWebSocket(/\/api\/containers\/[^/]+\/logs/, (ws) => {
        if (!scenario.containerLogs) return;
        setTimeout(() => {
            for (const line of scenario.containerLogs ?? []) ws.send(Buffer.from(line).toString("base64"));
        }, 0);
    });

    // Dedicated (non-multiplex) subscribe sockets used by a few pages: accept
    // and stay silent so nothing errors.
    await page.routeWebSocket(/\/api\/mowglinext\/subscribe\//, () => { /* silent */ });
}
