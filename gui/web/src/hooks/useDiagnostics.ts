import {useEffect, useRef, useState} from "react";
import {useWS} from "./useWS.ts";

export interface DiagnosticStatus {
    level: number;
    name: string;
    message: string;
    hardware_id: string;
    values: { key: string; value: string }[];
}

export interface DiagnosticArray {
    status?: DiagnosticStatus[];
}

/**
 * Subscribes to the /diagnostics topic and accumulates entries by name.
 * Entries are merged (latest wins per name) and throttled to avoid flicker
 * from high-frequency diagnostic updates.
 */
export const useDiagnostics = () => {
    const [diagnostics, setDiagnostics] = useState<DiagnosticArray>({})
    const accumulatorRef = useRef<Map<string, DiagnosticStatus>>(new Map());
    const throttleRef = useRef<ReturnType<typeof setTimeout> | null>(null);

    const diagnosticsStream = useWS<string>(() => {
            console.log({ message: "Diagnostics Stream closed" })
        }, () => {
            console.log({ message: "Diagnostics Stream connected" })
        },
        (e) => {
            const msg: DiagnosticArray = (e as any);
            // Merge incoming entries by name (latest value wins)
            if (msg.status) {
                for (const entry of msg.status) {
                    accumulatorRef.current.set(entry.name, entry);
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
