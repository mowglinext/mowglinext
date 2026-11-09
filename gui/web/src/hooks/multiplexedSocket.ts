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

/**
 * Topics that only feed pictures and that publish CONTINUOUSLY or periodically: the
 * mow-progress grid (republished every couple of seconds), the scan, the pose and the raw
 * sensor streams. Nobody looks at them while the tab is hidden, and they are the bulk of the
 * bytes over the mower's wifi — weakest exactly where it mows. They are unsubscribed while
 * the tab is hidden and resubscribed when it is shown again; the next frame follows within
 * moments, so the page is complete again at once.
 *
 * Deliberately NOT here:
 *  - anything background code acts on — highLevelStatus, status, emergency, power,
 *    diagnostics, gps/gnssStatus — which drive notifications, the battery gauge and the
 *    app shell even when the tab is not in front;
 *  - topics that publish only ON CHANGE (map, plan, path, obstacles, recordingTrajectory,
 *    lidarMap, cogHeading, magYaw): the server forgets its copy once the last listener
 *    leaves, so after a resubscribe they could stay empty until the next change.
 *    Pausing them needs the server to keep its latest message first.
 */
export const PAUSE_WHEN_HIDDEN = new Set([
    "mowProgress", "lidar", "pose", "fusionRaw", "imu", "ticks", "wheelOdom",
]);

/**
 * Occupancy-grid topics the server can send as PATCHES (gui/pkg/api/grid_delta.go): after one
 * full grid only the cells that changed, which while mowing is a few dozen of over a million.
 * The browser keeps the grid and applies each patch, so listeners still receive a complete
 * grid object. A patch that does not fit what is held (a frame was lost) is dropped and a full
 * grid is requested, so a wrong picture can never persist.
 */
export const DELTA_TOPICS = new Set(["mowProgress", "lidarMap"]);

interface GridPatch {
    /** The seq the held grid must have for this patch to apply. */
    base: number;
    seq: number;
    header?: unknown;
    /** gaps[i] = distance from the previous changed index (from 0 for the first). */
    gaps: number[];
    vals: number[];
}

type HeldGrid = Record<string, unknown> & {data: number[]};

interface ServerFrame {
    topic: string;
    data?: unknown;
    /** Present on the full grid frames of a delta subscription. */
    seq?: number;
    patch?: GridPatch;
}

interface ClientOp {
    op: "subscribe" | "unsubscribe" | "resync";
    topic: string;
    delta?: boolean;
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
    // True while the tab is hidden: the PAUSE_WHEN_HIDDEN topics are then not
    // subscribed on the server, although their listeners stay registered.
    private hidden = false;
    // The grid each delta topic currently holds, and the topics we asked a full grid for.
    private grids = new Map<string, {seq: number; grid: HeldGrid}>();
    private resyncing = new Set<string>();

    constructor(url: string) {
        this.url = url;
    }

    /** Whether the server should currently be streaming this topic to us. */
    private isServed(topic: string): boolean {
        return !(this.hidden && PAUSE_WHEN_HIDDEN.has(topic));
    }

    /**
     * Tell the socket whether the tab is in front. Hidden: stop the visual-only
     * topics on the server. Shown: start them again (the server replays its latest
     * message per topic, so the first frame is a complete picture).
     */
    setHidden(hidden: boolean): void {
        if (hidden === this.hidden) return;
        this.hidden = hidden;
        if (this.state === "open") {
            for (const topic of this.listeners.keys()) {
                if (PAUSE_WHEN_HIDDEN.has(topic)) {
                    this.send({op: hidden ? "unsubscribe" : "subscribe", topic});
                }
            }
        }
        if (!hidden) this.lastFrameAt = performance.now();
        this.updateSilenceWatchdog();
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
        } else if (this.state === "open" && isFirstSubscriberForTopic && this.isServed(topic)) {
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
            this.forgetGrid(topic);
            if (this.state === "open" && this.isServed(topic)) {
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
                if (this.isServed(topic)) this.send({op: "subscribe", topic});
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
                if (!frame || typeof frame.topic !== "string" || !("data" in frame || "patch" in frame)) return;
            } catch (err) {
                this.warnDecodeFailure(data, err);
                return;
            }
            const set = this.listeners.get(frame.topic);
            if (!set || set.size === 0) return;
            let payload: unknown = frame.data;
            if (frame.patch !== undefined) {
                payload = this.applyGridPatch(frame.topic, frame.patch);
                if (payload === undefined) return; // not applicable: a full grid was requested
            } else if (typeof frame.seq === "number") {
                this.rememberGrid(frame.topic, frame.seq, frame.data);
            }
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
                    cb(payload, isFirst);
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
        this.grids.clear();
        this.resyncing.clear();
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
        // A topic paused with the tab hidden is silent by design, not by failure.
        const expectsTraffic = Array.from(this.listeners.keys()).some(topic => CONTINUOUS_TOPICS.has(topic) && this.isServed(topic));
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

    private forgetGrid(topic: string): void {
        this.grids.delete(topic);
        this.resyncing.delete(topic);
    }

    private rememberGrid(topic: string, seq: number, grid: unknown): void {
        this.resyncing.delete(topic);
        if (grid && typeof grid === "object" && Array.isArray((grid as {data?: unknown}).data)) {
            this.grids.set(topic, {seq, grid: grid as HeldGrid});
        } else {
            this.grids.delete(topic);
        }
    }

    /** The grid with the patch applied, or undefined (after asking for a full grid) when it does not fit. */
    private applyGridPatch(topic: string, patch: GridPatch): HeldGrid | undefined {
        const held = this.grids.get(topic);
        if (!held || held.seq !== patch.base || !Array.isArray(patch.gaps) || !Array.isArray(patch.vals)
            || patch.gaps.length !== patch.vals.length) {
            this.requestFullGrid(topic);
            return undefined;
        }
        const data = held.grid.data;
        let at = 0;
        for (let i = 0; i < patch.gaps.length; i++) {
            at += patch.gaps[i];
            if (at < 0 || at >= data.length) {
                this.requestFullGrid(topic);
                return undefined;
            }
            data[at] = patch.vals[i];
        }
        held.seq = patch.seq;
        // A new top-level object (consumers may compare references), the same cell array.
        held.grid = {...held.grid, header: patch.header ?? held.grid.header};
        return held.grid;
    }

    private requestFullGrid(topic: string): void {
        this.grids.delete(topic);
        if (this.resyncing.has(topic)) return;
        this.resyncing.add(topic);
        this.send({op: "resync", topic});
    }

    private send(op: ClientOp): void {
        if (!this.ws || this.state !== "open") return;
        if (op.op === "subscribe" && DELTA_TOPICS.has(op.topic)) op = {...op, delta: true};
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
        const socket = new MultiplexedSocket(multiplexUrl());
        if (typeof document !== "undefined") {
            socket.setHidden(document.hidden);
            document.addEventListener("visibilitychange", () => socket.setHidden(document.hidden));
        }
        singleton = socket;
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
