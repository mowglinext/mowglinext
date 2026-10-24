import {useEffect, useRef, useState} from "react";
import {isJsonObject} from "../utils/json";
import {useWS} from "./useWS.ts";

export interface BTStatusChange {
    node_name: string;
    uid: number;
    previous_status: string;
    current_status: string;
}

export interface BTLog {
    event_log?: BTStatusChange[];
}

/** Only fields used by the state projection are required; no UID or timestamp is invented. */
function decodeStateChanges(raw: unknown): Array<Pick<BTStatusChange, 'node_name' | 'current_status'>> | null {
    if (!isJsonObject(raw)) return null;
    if (!('event_log' in raw)) return [];
    if (!Array.isArray(raw.event_log)) return null;
    const events: Array<Pick<BTStatusChange, 'node_name' | 'current_status'>> = [];
    const rawEvents: unknown[] = raw.event_log;
    for (const event of rawEvents) {
        if (!isJsonObject(event) || typeof event.node_name !== 'string' || typeof event.current_status !== 'string') return null;
        events.push({node_name: event.node_name, current_status: event.current_status});
    }
    return events;
}

/** Latest status of a BT node plus when that status was last reported. */
export interface BTNodeState {
    status: string;
    /** Browser receipt wall time, not the ROS event timestamp or acquisition time. */
    updatedAt: number;
}

/** Nodes not updated within this window should be rendered as stale. */
export const BT_NODE_STALE_MS = 30_000;

/** True when a node state has not been refreshed for BT_NODE_STALE_MS. */
export const isBTNodeStale = (state: BTNodeState, nowMs: number): boolean =>
    nowMs - state.updatedAt > BT_NODE_STALE_MS;

/**
 * Subscribes to /behavior_tree_log and accumulates node states.
 * Returns a map of node_name → {status, updatedAt} for all nodes that
 * have reported at least one status change; `updatedAt` lets consumers
 * grey out nodes that stopped reporting.
 */
export const useBTLog = () => {
    const [nodeStates, setNodeStates] = useState<Map<string, BTNodeState>>(new Map());
    const accRef = useRef<Map<string, BTNodeState>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    const stream = useWS<unknown>(() => {}, () => {},
        (e) => {
            const events = decodeStateChanges(e);
            if (!events) return;
            if (events.length > 0) {
                const updatedAt = Date.now();
                for (const ev of events) {
                    accRef.current.set(ev.node_name, {status: ev.current_status, updatedAt});
                }
            }
            // Throttle updates to 2/s
            if (!throttleRef.current) {
                throttleRef.current = setTimeout(() => {
                    throttleRef.current = null;
                    setNodeStates(new Map(accRef.current));
                }, 500);
            }
        });

    const {start, stop} = stream;
    useEffect(() => {
        start("/api/mowglinext/subscribe/btLog");
        return () => {
            stop();
            if (throttleRef.current) clearTimeout(throttleRef.current);
            throttleRef.current = null;
        };
    }, [start, stop]);

    return nodeStates;
};
