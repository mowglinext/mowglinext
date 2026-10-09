import {useCallback, useEffect, useState} from "react";
import {ContentType} from "../api/Api";
import {useApi} from "./useApi";

export interface BlackboxConfig {
    enabled: boolean;
    pre_seconds: number;
    post_seconds: number;
    memory_bytes: number;
    max_message_bytes: number;
    max_snapshots: number;
    max_disk_bytes: number;
    cooldown_seconds: number;
    min_free_disk_bytes: number;
}
export interface BlackboxRecording {
    name: string;
    size: number;
    triggered_at: string;
    reasons: string[];
    actual_pre_seconds: number;
    actual_post_seconds: number;
    capture_dropped_messages: number;
    interrupted: boolean;
}
export interface BlackboxStatus {
    config: BlackboxConfig;
    phase: string;
    buffered_seconds: number;
    buffered_bytes: number;
    effective_memory_bytes: number;
    dropped_messages: number;
    memory_pressure: boolean;
    last_error?: string;
    warning?: string;
    recordings: BlackboxRecording[];
    topics: {topic: string; last_received_at?: string}[];
}

function errorMessage(error: unknown): string {
    if (error instanceof Error) return error.message;
    const response = error as {error?: string | {error?: string}};
    if (typeof response?.error === "string") return response.error;
    return response?.error?.error ?? "Blackbox request failed";
}

export function useBlackbox() {
    const api = useApi();
    const [status, setStatus] = useState<BlackboxStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [busy, setBusy] = useState(false);
    const refresh = useCallback(async () => {
        const response = await api.request<BlackboxStatus>({path: "/tools/blackbox/status", method: "GET", format: "json"});
        setStatus(response.data);
        setError(null);
    }, [api]);
    useEffect(() => {
        let cancelled = false;
        let timeout: ReturnType<typeof setTimeout>;
        const poll = async () => {
            try { await refresh(); } catch (e) { if (!cancelled) setError(errorMessage(e)); }
            if (!cancelled) timeout = setTimeout(poll, 4000);
        };
        void poll();
        return () => { cancelled = true; clearTimeout(timeout); };
    }, [refresh]);
    const mutate = useCallback(async (path: string, method: "POST" | "PUT" | "DELETE", body?: BlackboxConfig) => {
        setBusy(true);
        try {
            await api.request({path: `/tools/blackbox${path}`, method, body, type: ContentType.Json, format: "json"});
            await refresh();
        } catch (e) { throw new Error(errorMessage(e)); }
        finally { setBusy(false); }
    }, [api, refresh]);
    return {
        status, error, busy,
        save: () => mutate("/save", "POST"),
        configure: (config: BlackboxConfig) => mutate("/config", "PUT", config),
        remove: (name: string) => mutate(`/${encodeURIComponent(name)}`, "DELETE"),
        downloadUrl: (name: string) => `${api.baseUrl}/tools/blackbox/download/${encodeURIComponent(name)}`,
    };
}
