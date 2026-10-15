import {useEffect, useState} from "react";
import {isJsonObject} from "../utils/json";
import {useWS} from "./useWS.ts";

export interface FusionGraphStats {
    /** Browser receipt wall time, retained for existing readiness consumers. Not acquisition time. */
    receivedAt: number;
    /** Monotonic browser delivery time; never compare with a ROS stamp. */
    receivedMonotonic?: number;
    /** First receipt of this producer + publication stamp (cached repeats do not renew it). */
    distinctMonotonic?: number;
    sourceIdentity?: string;
    sourceStamp?: string;
    level: number;
    message: string;
    /**
     * Raw key/value map from the DiagnosticStatus.values array. Stringly-typed
     * because that's what diagnostic_msgs/DiagnosticArray emits — callers parse
     * known keys to numbers as needed.
     */
    values: Record<string, string>;
}

type DiagnosticEntry = {
    name: string;
    hardware_id?: string;
    level: number;
    message: string;
    values: Array<{key: string; value: string}>;
};
type DiagnosticFrame = {
    entry: DiagnosticEntry;
    stamp?: {sec: number; nanosec: number};
};

/** Decode the first status only, retaining support for absent legacy metadata. */
function decodeDiagnosticFrame(raw: unknown): DiagnosticFrame | null {
    if (!isJsonObject(raw) || !Array.isArray(raw.status)) return null;
    const entry: unknown = raw.status[0];
    if (!isJsonObject(entry) || typeof entry.name !== 'string' || typeof entry.message !== 'string'
        || typeof entry.level !== 'number' || !Number.isInteger(entry.level) || entry.level < 0 || entry.level > 3
        || ('hardware_id' in entry && typeof entry.hardware_id !== 'string')) return null;
    const values: DiagnosticEntry['values'] = [];
    if ('values' in entry) {
        if (!Array.isArray(entry.values)) return null;
        const items: unknown[] = entry.values;
        for (const item of items) {
            if (!isJsonObject(item) || typeof item.key !== 'string' || typeof item.value !== 'string') return null;
            values.push({key: item.key, value: item.value});
        }
    }
    let stamp: DiagnosticFrame['stamp'];
    if ('header' in raw) {
        if (!isJsonObject(raw.header)) return null;
        if ('stamp' in raw.header) {
            const candidate: unknown = raw.header.stamp;
            if (!isJsonObject(candidate) || typeof candidate.sec !== 'number' || !Number.isInteger(candidate.sec)
                || candidate.sec < -2147483648 || candidate.sec > 2147483647
                || typeof candidate.nanosec !== 'number' || !Number.isInteger(candidate.nanosec)
                || candidate.nanosec < 0 || candidate.nanosec >= 1_000_000_000) return null;
            stamp = {sec: candidate.sec, nanosec: candidate.nanosec};
        }
    }
    return {entry: {name: entry.name, message: entry.message, level: entry.level, values,
        ...(typeof entry.hardware_id === 'string' ? {hardware_id: entry.hardware_id} : {})}, stamp};
}

/**
 * Subscribes to /fusion_graph/diagnostics (1 Hz) and exposes the latest
 * GraphStats snapshot. The fusion_graph_node publishes a single
 * DiagnosticStatus per array; we just take status[0] and flatten the
 * KeyValue list.
 */
export const useFusionGraphDiagnostics = () => {
    const [stats, setStats] = useState<FusionGraphStats | null>(null);


    const stream = useWS<unknown>(
        () => { /* closed */ },
        () => { /* connected */ },
        (e) => {
            try {
                const decoded = decodeDiagnosticFrame(e);
                if (!decoded) return;
                const {entry, stamp} = decoded;
                const values: Record<string, string> = Object.fromEntries(entry.values.map(({key, value}) => [key, value]));
                const receivedMonotonic = performance.now();
                const sourceStamp = stamp && Number.isFinite(stamp.sec) && Number.isFinite(stamp.nanosec)
                    && (stamp.sec > 0 || stamp.nanosec > 0)
                    ? `${stamp.sec}.${String(stamp.nanosec).padStart(9, '0')}` : undefined;
                const sourceIdentity = sourceStamp ? `${entry.name}|${entry.hardware_id ?? ''}|${sourceStamp}` : undefined;
                setStats(previous => ({
                    receivedAt: Date.now(),
                    receivedMonotonic,
                    distinctMonotonic: sourceIdentity && sourceIdentity === previous?.sourceIdentity
                        ? previous.distinctMonotonic : receivedMonotonic,
                    sourceIdentity,
                    sourceStamp,
                    level: entry.level,
                    message: entry.message,
                    values,
                }));
            } catch {
                /* ignore malformed messages */
            }
        },
    );

    const {start, stop} = stream;
    useEffect(() => {
        start("/api/mowglinext/subscribe/fusionDiag");
        return () => stop();
    }, [start, stop]);

    return {stats};
};
