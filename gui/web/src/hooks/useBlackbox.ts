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

const isObject = (value: unknown): value is Record<string, unknown> =>
    typeof value === "object" && value !== null;
const isNumber = (value: unknown): value is number => typeof value === "number" && Number.isFinite(value);
const optionalString = (value: unknown) => value === undefined || typeof value === "string";

// The panel shares Diagnostics with unrelated tools. A missing, old or malformed
// endpoint must become a local error rather than crash the whole page.
function isBlackboxStatus(value: unknown): value is BlackboxStatus {
    if (!isObject(value) || !isObject(value.config)) return false;
    const config = value.config;
    return typeof config.enabled === "boolean"
        && ["pre_seconds", "post_seconds", "memory_bytes", "max_message_bytes", "max_snapshots", "max_disk_bytes",
            "cooldown_seconds", "min_free_disk_bytes"].every(key => isNumber(config[key]))
        && typeof value.phase === "string" && typeof value.memory_pressure === "boolean"
        && [value.buffered_seconds, value.buffered_bytes, value.effective_memory_bytes, value.dropped_messages].every(isNumber)
        && optionalString(value.warning) && optionalString(value.last_error)
        && Array.isArray(value.topics) && value.topics.every(topic => isObject(topic)
            && typeof topic.topic === "string" && optionalString(topic.last_received_at))
        && Array.isArray(value.recordings) && value.recordings.every(recording => isObject(recording)
            && typeof recording.name === "string" && /^[a-zA-Z0-9._-]+$/.test(recording.name)
            && typeof recording.triggered_at === "string" && Number.isFinite(Date.parse(recording.triggered_at))
            && Array.isArray(recording.reasons) && recording.reasons.every(reason => typeof reason === "string")
            && [recording.size, recording.actual_pre_seconds, recording.actual_post_seconds, recording.capture_dropped_messages].every(isNumber)
            && typeof recording.interrupted === "boolean");
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
        const response = await api.request<unknown>({path: "/tools/blackbox/status", method: "GET", format: "json"});
        if (!isBlackboxStatus(response.data)) throw new Error("Invalid blackbox status response");
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
