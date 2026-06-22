import {useEffect, useRef, useState} from "react";
import {useWS} from "./useWS.ts";

export interface FusionGraphStats {
    /** Wall-clock time when this snapshot was published (rclcpp::now()). */
    receivedAt: number;
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
    status?: Array<{
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
    const lastReceiveRef = useRef<number>(0);

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
                lastReceiveRef.current = Date.now();
                setStats({
                    receivedAt: lastReceiveRef.current,
                    level: entry.level,
                    message: entry.message,
                    values,
                });
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
