import {useCallback, useEffect, useRef, useState} from "react";
import {useApi} from "./useApi.ts";
import type {RemoteAccessStatus} from "../types/remoteAccess.ts";

const DEFAULT_POLL_MS = 5_000;

interface UseRemoteAccessStatusOptions {
    /** Poll period; 0 fetches once on mount only. */
    pollMs?: number;
    /** false skips fetching entirely. */
    enabled?: boolean;
}

/**
 * Polls the remote-access sidecar status. Each call costs one `docker exec`
 * on the robot, so this is only polled while the settings section is open;
 * 5 s keeps the image pull → login link → connected progression visible.
 */
export function useRemoteAccessStatus({pollMs = DEFAULT_POLL_MS, enabled = true}: UseRemoteAccessStatusOptions = {}) {
    const guiApi = useApi();
    const [status, setStatus] = useState<RemoteAccessStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const mounted = useRef(true);

    const refresh = useCallback(async () => {
        try {
            const res = await guiApi.request<RemoteAccessStatus>({path: "/remote-access/status", method: "GET", format: "json"});
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
