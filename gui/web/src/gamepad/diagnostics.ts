import {httpBase} from '../utils/apiHost';

type Value = string | number | boolean | null | undefined;
export type ControllerHealth = Record<string, Value>;
interface DiagnosticEvent {at: string; event: string; detail: ControllerHealth}
export interface GamepadDiagnosticReport {
    version: number;
    clientId: string;
    events: DiagnosticEvent[];
    snapshot: ControllerHealth;
}

// This recorder is observational: it never calls the driver, renews a lease or
// changes ownership. Keep work and traffic bounded on the same thread as input.
export class GamepadDiagnostics {
    readonly clientId = `browser-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
    private events: DiagnosticEvent[] = [];
    private snapshot: ControllerHealth = {};
    private inFlight = false;
    private dirty = false;
    private urgent = false;

    update(health: ControllerHealth) { this.snapshot = {...this.snapshot, ...health}; this.dirty = true; }

    record(event: string, detail: ControllerHealth = {}) {
        this.events.push({at: new Date().toISOString(), event, detail});
        if (this.events.length > 32) this.events.shift();
        this.dirty = true;
    }

    // Flush after the caller has sent zero/revoked ownership. A fault during a
    // slow upload gets one more attempt after it settles, without a send queue.
    flushSoon() {
        this.urgent = true;
        queueMicrotask(() => { void this.flush(); });
    }

    async flush() {
        if (this.inFlight || !this.dirty) return;
        this.inFlight = true;
        this.urgent = false;
        this.dirty = false;
        const events = [...this.events];
        const report = {version: 1, clientId: this.clientId, events, snapshot: this.snapshot};
        let body = JSON.stringify(report);
        // Avoid exceeding the server/keepalive limit even with long native IDs.
        while (new TextEncoder().encode(body).length > 14_000 && events.length) {
            events.shift();
            body = JSON.stringify(report);
        }
        const abort = new AbortController();
        const timeout = window.setTimeout(() => abort.abort(), 2000);
        try {
            const response = await fetch(`${httpBase()}/api/mowglinext/gamepad/diagnostics`, {
                method: 'POST', headers: {'Content-Type': 'application/json'},
                body, signal: abort.signal, keepalive: true,
            });
            if (!response.ok) this.dirty = true;
        } catch { this.dirty = true; /* Diagnostics must never interrupt control. */ }
        finally {
            window.clearTimeout(timeout);
            this.inFlight = false;
            if (this.urgent) this.flushSoon();
        }
    }
}
