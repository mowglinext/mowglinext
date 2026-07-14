import {useEffect, useRef, useState} from "react";
import {useWS} from "./useWS.ts";

export interface DiagnosticStatus {
    level: number;
    name: string;
    message: string;
    hardware_id: string;
    values: { key: string; value: string }[];
}

/** A diagnostic entry stamped with the wall-clock time it was last received. */
export interface TimestampedDiagnosticStatus extends DiagnosticStatus {
    receivedAt: number;
}

export interface DiagnosticArray {
    status?: TimestampedDiagnosticStatus[];
}

/** Entries not refreshed within this window should be rendered as stale. */
export const DIAGNOSTIC_STALE_MS = 30_000;

/** True when an accumulated entry has not been refreshed for DIAGNOSTIC_STALE_MS. */
export const isDiagnosticStale = (
    entry: Pick<TimestampedDiagnosticStatus, "receivedAt">,
    nowMs: number,
): boolean => nowMs - entry.receivedAt > DIAGNOSTIC_STALE_MS;

/**
 * Subscribes to the /diagnostics topic and accumulates entries by name.
 * Entries are merged (latest wins per name), stamped with `receivedAt`
 * so consumers can grey out entries that stopped updating, and throttled
 * to avoid flicker from high-frequency diagnostic updates.
 */
export const useDiagnostics = () => {
    const [diagnostics, setDiagnostics] = useState<DiagnosticArray>({})
    const accumulatorRef = useRef<Map<string, TimestampedDiagnosticStatus>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    const diagnosticsStream = useWS<string>(() => {}, () => {},
        (e) => {
            const msg: { status?: DiagnosticStatus[] } = (e as any);
            // Merge incoming entries by name (latest value wins)
            if (msg.status) {
                const receivedAt = Date.now();
                for (const entry of msg.status) {
                    accumulatorRef.current.set(entry.name, {...entry, receivedAt});
                }
            }
            // Throttle state updates to max 1 per second
            if (!throttleRef.current) {
                throttleRef.current = setTimeout(() => {
                    throttleRef.current = null;
                    setDiagnostics({
                        status: Array.from(accumulatorRef.current.values()),
                    });
                }, 1000);
            }
        })

    useEffect(() => {
        diagnosticsStream.start("/api/mowglinext/subscribe/diagnostics")
        return () => {
            diagnosticsStream.stop()
            if (throttleRef.current) {
                clearTimeout(throttleRef.current);
            }
        }
    }, []);

    return {diagnostics, stop: diagnosticsStream.stop, start: diagnosticsStream.start};
};
