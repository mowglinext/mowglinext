import {useEffect, useRef, useState} from "react";
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

/** Latest status of a BT node plus when that status was last reported. */
export interface BTNodeState {
    status: string;
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

    const stream = useWS<string>(() => {}, () => {},
        (e) => {
            const msg: BTLog = (e as any);
            if (msg.event_log) {
                const updatedAt = Date.now();
                for (const ev of msg.event_log) {
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

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/btLog");
        return () => {
            stream.stop();
            if (throttleRef.current) clearTimeout(throttleRef.current);
        };
    }, []);

    return nodeStates;
};
