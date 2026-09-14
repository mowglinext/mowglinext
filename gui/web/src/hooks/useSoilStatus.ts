import {useCallback, useEffect, useRef, useState} from "react";
import {useApi} from "./useApi.ts";
import type {SoilStatus} from "../types/irrisense.ts";

const DEFAULT_POLL_MS = 30_000;

interface UseSoilStatusOptions {
    /** Poll period; 0 fetches once on mount only. */
    pollMs?: number;
    /** false skips fetching entirely (a parent already holds the status). */
    enabled?: boolean;
}

/**
 * Polls the cached IrriSense soil verdict. The backend refreshes it every 10
 * minutes; polling the cache more often only costs a local round-trip, so the
 * chip and the settings status line stay in step without a socket.
 */
export function useSoilStatus({pollMs = DEFAULT_POLL_MS, enabled = true}: UseSoilStatusOptions = {}) {
    const guiApi = useApi();
    const [status, setStatus] = useState<SoilStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const mounted = useRef(true);

    const refresh = useCallback(async () => {
        try {
            const res = await guiApi.request<SoilStatus>({path: "/irrisense/status", method: "GET", format: "json"});
            if (!mounted.current) return;
            setStatus(res.data);
            setError(null);
        } catch (e: unknown) {
            if (!mounted.current) return;
            setError(e instanceof Error ? e.message : "status unavailable");
        }
    }, [guiApi]);

    useEffect(() => {
        mounted.current = true;
        if (!enabled) return () => { mounted.current = false; };
        // Kick the first fetch off the synchronous effect path (a setState
        // inside the effect itself would cascade a render) and poll from there.
        const first = window.setTimeout(() => void refresh(), 0);
        const timer = pollMs > 0 ? window.setInterval(() => void refresh(), pollMs) : undefined;
        return () => {
            mounted.current = false;
            window.clearTimeout(first);
            if (timer !== undefined) window.clearInterval(timer);
        };
    }, [refresh, pollMs, enabled]);

    return {status, error, refresh};
}
