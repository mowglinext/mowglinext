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

/**
 * Subscribes to /behavior_tree_log and accumulates node states.
 * Returns a map of node_name → current_status for all nodes that
 * have reported at least one status change.
 */
export const useBTLog = () => {
    const [nodeStates, setNodeStates] = useState<Map<string, string>>(new Map());
    const accRef = useRef<Map<string, string>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    const stream = useWS<string>(() => {}, () => {},
        (e) => {
            const msg: BTLog = (e as any);
            if (msg.event_log) {
                for (const ev of msg.event_log) {
                    accRef.current.set(ev.node_name, ev.current_status);
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
