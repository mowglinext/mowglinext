import {useEffect, useState} from "react";
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

interface DiagnosticArray {
    header?: {stamp?: {sec: number; nanosec: number}};
    status?: Array<{
        hardware_id?: string;
        level: number;
        name: string;
        message: string;
        values: { key: string; value: string }[];
    }>;
}

/**
 * Subscribes to /fusion_graph/diagnostics (1 Hz) and exposes the latest
 * GraphStats snapshot. The fusion_graph_node publishes a single
 * DiagnosticStatus per array; we just take status[0] and flatten the
 * KeyValue list.
 */
export const useFusionGraphDiagnostics = () => {
    const [stats, setStats] = useState<FusionGraphStats | null>(null);


    const stream = useWS<string>(
        () => { /* closed */ },
        () => { /* connected */ },
        (e) => {
            try {
                const msg: DiagnosticArray = (e as any);
                const entry = msg.status?.[0];
                if (!entry) return;
                const values: Record<string, string> = {};
                for (const v of entry.values ?? []) {
                    values[v.key] = v.value;
                }
                const receivedMonotonic = performance.now();
                const stamp = msg.header?.stamp;
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

    useEffect(() => {
        stream.start("/api/mowglinext/subscribe/fusionDiag");
        return () => stream.stop();
    }, []);

    return {stats};
};
