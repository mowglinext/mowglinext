// Single-connection WebSocket multiplexer for /api/mowglinext/multiplex.
//
// Replaces the one-WebSocket-per-topic pattern that opened ~25 connections
// per browser tab. All callers share one socket; each subscribe() registers
// a listener and tracks ref-count per topic so the server keeps exactly one
// upstream subscription per topic per tab.
//
// Wire format (matches MultiplexRoute in gui/pkg/api/mowglinext.go):
//   client → server: {"op": "subscribe"|"unsubscribe", "topic": "<key>"}
//   server → client: {"topic": "<key>", "data": "<base64>"}

type Listener = (data: string, first: boolean) => void;

interface ServerFrame {
    topic: string;
    data: string;
}

interface ClientOp {
    op: "subscribe" | "unsubscribe";
    topic: string;
}

class MultiplexedSocket {
    private url: string;
    private ws: WebSocket | null = null;
    private state: "idle" | "connecting" | "open" = "idle";
    private listeners = new Map<string, Set<Listener>>();
    // Functions that have not yet received their first payload — they get
    // first=true on the next delivery, then are removed.
    private pendingFirst = new WeakSet<Listener>();
    private reconnectAttempt = 0;
    private reconnectTimer: number | null = null;

    constructor(url: string) {
        this.url = url;
    }

    subscribe(topic: string, listener: Listener): () => void {
        let set = this.listeners.get(topic);
        const isFirstSubscriberForTopic = !set || set.size === 0;
        if (!set) {
            set = new Set();
            this.listeners.set(topic, set);
        }
        set.add(listener);
        this.pendingFirst.add(listener);

        if (this.state === "idle") {
            this.connect();
        } else if (this.state === "open" && isFirstSubscriberForTopic) {
            this.send({op: "subscribe", topic});
        }

        return () => this.unsubscribe(topic, listener);
    }

    private unsubscribe(topic: string, listener: Listener): void {
        const set = this.listeners.get(topic);
        if (!set) return;
        set.delete(listener);
        this.pendingFirst.delete(listener);
        if (set.size === 0) {
            this.listeners.delete(topic);
            if (this.state === "open") {
                this.send({op: "unsubscribe", topic});
            }
        }
        if (this.listeners.size === 0) {
            // Cancel any pending reconnect — nothing to subscribe for.
            if (this.reconnectTimer != null) {
                clearTimeout(this.reconnectTimer);
                this.reconnectTimer = null;
            }
            // Close the live socket if open; onclose will not reconnect
            // because the listeners map is now empty.
            if (this.state === "open" && this.ws) {
                try { this.ws.close(); } catch { /* ignore */ }
            }
        }
    }

    private connect(): void {
        if (this.state !== "idle") return;
        if (this.listeners.size === 0) return;
        this.state = "connecting";

        const ws = new WebSocket(this.url);
        this.ws = ws;

        ws.onopen = () => {
            this.state = "open";
            this.reconnectAttempt = 0;
            // Re-subscribe to every topic that still has listeners.
            for (const topic of this.listeners.keys()) {
                this.send({op: "subscribe", topic});
            }
        };

        ws.onmessage = (e: MessageEvent) => {
            let frame: ServerFrame;
            try {
                frame = JSON.parse(typeof e.data === "string" ? e.data : "") as ServerFrame;
            } catch {
                return;
            }
            const set = this.listeners.get(frame.topic);
            if (!set || set.size === 0) return;
            let decoded: string;
            try {
                decoded = atob(frame.data);
            } catch {
                return;
            }
            // Snapshot listeners so a callback that unsubscribes mid-iteration
            // does not affect the current dispatch.
            const snapshot = Array.from(set);
            for (const cb of snapshot) {
                const isFirst = this.pendingFirst.has(cb);
                if (isFirst) this.pendingFirst.delete(cb);
                try {
                    cb(decoded, isFirst);
                } catch (err) {
                    console.error("MultiplexedSocket: listener threw", err);
                }
            }
        };

        ws.onerror = () => {
            try { ws.close(); } catch { /* ignore */ }
        };

        ws.onclose = () => {
            this.ws = null;
            this.state = "idle";
            // Reconnect only if there's still something to listen for.
            if (this.listeners.size > 0) {
                this.scheduleReconnect();
            }
        };
    }

    private scheduleReconnect(): void {
        if (this.reconnectTimer != null) return;
        const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempt), 30000);
        this.reconnectAttempt += 1;
        this.reconnectTimer = window.setTimeout(() => {
            this.reconnectTimer = null;
            this.connect();
        }, delay);
    }

    private send(op: ClientOp): void {
        if (!this.ws || this.state !== "open") return;
        try {
            this.ws.send(JSON.stringify(op));
        } catch (err) {
            console.warn("MultiplexedSocket: send failed", err);
        }
    }
}

let singleton: MultiplexedSocket | null = null;

function multiplexUrl(): string {
    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    if (import.meta.env.DEV) {
        return `${protocol}://localhost:4006/api/mowglinext/multiplex`;
    }
    return `${protocol}://${window.location.host}/api/mowglinext/multiplex`;
}

export function getMultiplexedSocket(): MultiplexedSocket {
    if (singleton == null) {
        singleton = new MultiplexedSocket(multiplexUrl());
    }
    return singleton;
}

// Match /api/mowglinext/subscribe/<topic> exactly; everything else (e.g.
// /publish/joy) keeps its dedicated socket.
const SUBSCRIBE_PREFIX = "/api/mowglinext/subscribe/";

export function isMultiplexableSubscribeUri(uri: string): boolean {
    return uri.startsWith(SUBSCRIBE_PREFIX) && uri.length > SUBSCRIBE_PREFIX.length;
}

export function topicFromSubscribeUri(uri: string): string {
    return uri.slice(SUBSCRIBE_PREFIX.length);
}
