// Single-connection WebSocket multiplexer for /api/mowglinext/multiplex.
//
// Replaces the one-WebSocket-per-topic pattern that opened ~25 connections
// per browser tab. All callers share one socket; each subscribe() registers
// a listener and tracks ref-count per topic so the server keeps exactly one
// upstream subscription per topic per tab.
//
// Wire format (matches MultiplexRoute in gui/pkg/api/mowglinext.go):
//   client → server: {"op": "subscribe"|"unsubscribe", "topic": "<key>"}  (JSON text)
//   server → client: MessagePack BINARY frame {"topic": "<key>", "data": <decoded msg object>}
//
// The server msgpack-encodes the already-decoded ROS message (snake_case keys
// preserved), so listeners receive the message OBJECT directly — no per-hook
// JSON.parse, no base64. Keeps the heavy number-array parse off the browser
// main thread (one msgpack decode vs JSON.parse(envelope)+atob+JSON.parse).

import {unpack} from "msgpackr";

type Listener = (data: unknown, first: boolean) => void;

/**
 * Public connection status. "closed" covers both never-connected/idle and a
 * dropped connection awaiting reconnect — consumers only need to know whether
 * live data can currently arrive.
 */
export type MultiplexStatus = "connecting" | "open" | "closed";

type StatusListener = (status: MultiplexStatus) => void;

/** Minimum interval between "malformed frame" console warnings. */
const DECODE_WARN_INTERVAL_MS = 10_000;

const STREAM_SILENCE_MS = 30_000;
// These topics normally publish continuously. Latched/on-change topics (map,
// path, plan, etc.) may legitimately stay quiet indefinitely.
const CONTINUOUS_TOPICS = new Set([
    "status", "highLevelStatus", "gps", "gnssStatus", "pose", "imu", "ticks",
    "wheelOdom", "lidar", "power", "diagnostics", "fusionDiag", "fusionRaw",
]);

interface ServerFrame {
    topic: string;
    data: unknown;
}

interface ClientOp {
    op: "subscribe" | "unsubscribe";
    topic: string;
}

export class MultiplexedSocket {
    private url: string;
    private ws: WebSocket | null = null;
    private state: "idle" | "connecting" | "open" = "idle";
    private listeners = new Map<string, Set<Listener>>();
    // Functions that have not yet received their first payload — they get
    // first=true on the next delivery, then are removed.
    private pendingFirst = new WeakSet<Listener>();
    private reconnectAttempt = 0;
    private reconnectTimer: number | null = null;
    private silenceTimer: number | null = null;
    private lastFrameAt = 0;
    private statusListeners = new Set<StatusListener>();
    private lastDecodeWarnAt = 0;

    constructor(url: string) {
        this.url = url;
    }

    /** Current status for the shared connection ("closed" when idle). */
    getStatus(): MultiplexStatus {
        switch (this.state) {
            case "open":
                return "open";
            case "connecting":
                return "connecting";
            default:
                return "closed";
        }
    }

    /**
     * Register for status transitions (connecting → open → closed → ...).
     * Returns an unregister function. The callback is NOT invoked with the
     * current status on registration — read {@link getStatus} for that.
     */
    onStatusChange(cb: StatusListener): () => void {
        this.statusListeners.add(cb);
        return () => {
            this.statusListeners.delete(cb);
        };
    }

    private notifyStatus(): void {
        const status = this.getStatus();
        for (const cb of Array.from(this.statusListeners)) {
            try {
                cb(status);
            } catch (err) {
                console.error("MultiplexedSocket: status listener threw", err);
            }
        }
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
        this.updateSilenceWatchdog();

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
            // Retire immediately: close events can be delayed or never arrive.
            // A new subscriber must not inherit this closing socket.
            this.disconnect();
        }
        this.updateSilenceWatchdog();
    }

    private connect(): void {
        if (this.state !== "idle") return;
        if (this.listeners.size === 0) return;
        this.state = "connecting";

        const ws = new WebSocket(this.url);
        // Server frames are MessagePack binary; receive them as ArrayBuffer.
        ws.binaryType = "arraybuffer";
        this.ws = ws;

        ws.onopen = () => {
            if (this.ws !== ws) return;
            // Every subscriber may have gone away during the handshake —
            // don't keep an orphan connection alive.
            if (this.listeners.size === 0) {
                this.disconnect();
                return;
            }
            this.state = "open";
            this.reconnectAttempt = 0;
            // Re-subscribe to every topic that still has listeners.
            for (const topic of this.listeners.keys()) {
                this.send({op: "subscribe", topic});
            }
            this.updateSilenceWatchdog();
            this.notifyStatus();
        };

        ws.onmessage = (e: MessageEvent) => {
            if (this.ws !== ws) return;
            // MessagePack binary frame → {topic, data: <decoded object>}.
            const data: unknown = e.data;
            if (!(data instanceof ArrayBuffer)) return;
            let frame: ServerFrame;
            try {
                frame = unpack(new Uint8Array(data)) as ServerFrame;
                if (!frame || typeof frame.topic !== "string" || !("data" in frame)) return;
            } catch (err) {
                this.warnDecodeFailure(data, err);
                return;
            }
            const set = this.listeners.get(frame.topic);
            if (!set || set.size === 0) return;
            // Transport liveness only: a cached ROS value is not evidence of a
            // fresh physical observation. Use a monotonic clock for delivery.
            this.lastFrameAt = performance.now();
            // Snapshot listeners so a callback that unsubscribes mid-iteration
            // does not affect the current dispatch.
            const snapshot = Array.from(set);
            for (const cb of snapshot) {
                const isFirst = this.pendingFirst.has(cb);
                if (isFirst) this.pendingFirst.delete(cb);
                try {
                    cb(frame.data, isFirst);
                } catch (err) {
                    console.error("MultiplexedSocket: listener threw", err);
                }
            }
        };

        ws.onerror = () => {
            if (this.ws !== ws) return;
            this.disconnect();
            this.scheduleReconnect();
        };

        ws.onclose = () => {
            if (this.ws !== ws) return;
            this.disconnect();
            this.scheduleReconnect();
        };
        this.notifyStatus();
    }

    private disconnect(): void {
        this.stopSilenceWatchdog();
        const ws = this.ws;
        this.ws = null;
        this.state = "idle";
        if (ws) {
            // Neither delayed close events nor queued messages from a retired
            // socket may affect its replacement.
            ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
            try { ws.close(); } catch { /* best effort */ }
        }
        this.notifyStatus();
    }

    private stopSilenceWatchdog(): void {
        if (this.silenceTimer != null) {
            clearTimeout(this.silenceTimer);
            this.silenceTimer = null;
        }
    }

    private updateSilenceWatchdog(): void {
        const expectsTraffic = Array.from(this.listeners.keys()).some(topic => CONTINUOUS_TOPICS.has(topic));
        if (this.state !== "open" || !expectsTraffic) {
            this.stopSilenceWatchdog();
        } else if (this.silenceTimer == null) {
            // Give a newly opened connection or newly enabled continuous
            // subscription the full grace period, even after a quiet map view.
            this.lastFrameAt = performance.now();
            this.silenceTimer = window.setTimeout(() => this.checkSilence(), STREAM_SILENCE_MS);
        }
    }

    private checkSilence(): void {
        this.silenceTimer = null;
        const remaining = STREAM_SILENCE_MS - (performance.now() - this.lastFrameAt);
        if (remaining > 0) {
            // Incoming frames update the timestamp without allocating a timer
            // for every high-rate ROS message.
            this.silenceTimer = window.setTimeout(() => this.checkSilence(), remaining);
            return;
        }
        this.disconnect();
        this.scheduleReconnect();
    }

    /**
     * Rate-limited (max one per {@link DECODE_WARN_INTERVAL_MS}) warning for
     * frames that fail msgpack decoding. The topic is part of the frame that
     * failed to decode, so only the raw size + subscribed topics are known.
     */
    private warnDecodeFailure(data: ArrayBuffer, err: unknown): void {
        const now = Date.now();
        if (now - this.lastDecodeWarnAt < DECODE_WARN_INTERVAL_MS) return;
        this.lastDecodeWarnAt = now;
        console.warn(
            "MultiplexedSocket: dropping undecodable frame",
            {
                byteLength: data.byteLength,
                subscribedTopics: Array.from(this.listeners.keys()),
            },
            err,
        );
    }

    private scheduleReconnect(): void {
        if (this.state !== "idle" || this.listeners.size === 0) return;
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

import {wsBase} from "../utils/apiHost";

let singleton: MultiplexedSocket | null = null;

function multiplexUrl(): string {
    return `${wsBase()}/api/mowglinext/multiplex`;
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
