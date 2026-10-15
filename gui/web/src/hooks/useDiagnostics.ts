import {useEffect, useRef, useState} from "react";
import {isDiagnosticStatus, diagnosticPublicationIdentity, advanceDiagnosticProgress, type DiagnosticProgress} from '../utils/diagnosticTelemetry';
import {isJsonObject} from '../utils/json';
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
    /** Opaque publication identity; cached replay does not renew receivedAt. */
    publicationStamp?: string;
    /** Display-only ordered publication evidence; does not alter legacy receipt semantics. */
    healthProgressAt?: number;
    healthBlocked?: boolean;
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
    const [disconnectedAt, setDisconnectedAt] = useState<number | null>(null);
    const healthProgress = useRef(new Map<string, DiagnosticProgress>());
    const sourceForName = useRef(new Map<string, string>());
    const malformedNames = useRef(new Set<string>());
    const receiptsRef = useRef(new Map<string, {identity: string; at: number}>());
    const accumulatorRef = useRef<Map<string, TimestampedDiagnosticStatus>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    const diagnosticsStream = useWS<unknown>(() => {
        setDisconnectedAt(Date.now());
        for (const [key, value] of healthProgress.current) healthProgress.current.set(key, {...value, blocked: true});
        for (const [key, entry] of accumulatorRef.current) accumulatorRef.current.set(key, {...entry, healthBlocked: true});
        setDiagnostics({status: Array.from(accumulatorRef.current.values())});
    }, () => {},
        (e) => {
            if (!isJsonObject(e) || !Array.isArray(e.status)) return;
            const entries: unknown[] = e.status;
            // Merge incoming entries by name (latest value wins)
            if (entries.length > 0) {
                const receivedAt = Date.now();
                const identity = diagnosticPublicationIdentity(e);
                for (const entry of entries) {
                    if (isDiagnosticStatus(entry)) {
                        const key = JSON.stringify([entry.name, entry.hardware_id]);
                        const known = healthProgress.current.get(key);
                        const before = malformedNames.current.has(entry.name) && known ? {...known, blocked: true} : known;
                        let progress = advanceDiagnosticProgress(before, e, receivedAt);
                        const oldSource = sourceForName.current.get(entry.name);
                        if (oldSource !== undefined && oldSource !== key && (!before || before.stamp === progress.stamp)) {
                            progress = {...progress, blocked: true};
                        }
                        healthProgress.current.set(key, progress);
                        sourceForName.current.set(entry.name, key);
                        if (!progress.blocked) malformedNames.current.delete(entry.name);
                        const previous = receiptsRef.current.get(entry.name);
                        const at = identity !== null && previous?.identity === identity ? previous.at : receivedAt;
                        if (identity !== null) receiptsRef.current.set(entry.name, {identity, at});
                        accumulatorRef.current.set(entry.name, {
                            ...entry, receivedAt: at, healthProgressAt: progress.at, healthBlocked: progress.blocked, ...(identity !== null ? {publicationStamp: identity} : {}),
                        });
                    } else if (isJsonObject(entry) && typeof entry.name === 'string') {
                        // A named malformed update cannot renew an older valid entry.
                        accumulatorRef.current.delete(entry.name);
                        malformedNames.current.add(entry.name);
                    }
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

    const {start, stop} = diagnosticsStream;
    useEffect(() => {
        start("/api/mowglinext/subscribe/diagnostics")
        return () => {
            stop()
            if (throttleRef.current) {
                clearTimeout(throttleRef.current);
                throttleRef.current = null;
            }
        }
    }, [start, stop]);

    return {diagnostics, disconnectedAt, stop: diagnosticsStream.stop, start: diagnosticsStream.start};
};
