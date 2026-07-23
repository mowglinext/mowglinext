import {useCallback, useEffect, useRef, useState} from "react";
import {ContentType} from "../api/Api.ts";
import {useApi} from "./useApi.ts";

export interface RosbagRecording {
    name: string;
    size_bytes: number;
    modified_at: string;
    active: boolean;
}

export interface RosbagStatus {
    active: boolean;
    active_name?: string;
    recordings: RosbagRecording[];
    warning?: string;
}

export interface RosbagStartResponse {
    name: string;
    started_at: string;
}

export interface RosbagStopResponse {
    stopped: boolean;
    stopped_name?: string;
}

const POLL_INTERVAL_MS = 4000;

// The generated HttpClient rejects with the parsed error body (e.g.
// {error: "..."}) rather than an Error, so normalize it to a real Error whose
// .message the UI can display.
function toError(e: unknown, fallback: string): Error {
    if (e instanceof Error) return e;
    const body = e as any;
    const msg = body?.error?.error ?? body?.error ?? body?.message;
    return new Error(typeof msg === "string" ? msg : fallback);
}

// useRosbag drives the "record all topics" panel: it polls recording state,
// starts/stops a `ros2 bag record -a` capture on the robot, deletes captures,
// and exposes the direct download URL for a finished bag.
export const useRosbag = (enabled: boolean = true) => {
    const guiApi = useApi();
    const [status, setStatus] = useState<RosbagStatus | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [busy, setBusy] = useState<"start" | "stop" | null>(null);
    const enabledRef = useRef(enabled);
    enabledRef.current = enabled;

    const refresh = useCallback(async () => {
        try {
            const response = await guiApi.request<RosbagStatus>({
                path: "/tools/rosbag/status",
                method: "GET",
                format: "json",
            });
            setStatus(response.data);
            setError(null);
        } catch (e) {
            setError(e instanceof Error ? e.message : "Failed to fetch rosbag status");
        }
    }, [guiApi]);

    useEffect(() => {
        if (!enabled) return;
        refresh();
        const id = setInterval(() => {
            if (enabledRef.current) refresh();
        }, POLL_INTERVAL_MS);
        return () => clearInterval(id);
    }, [enabled, refresh]);

    const start = useCallback(async () => {
        setBusy("start");
        try {
            const response = await guiApi.request<RosbagStartResponse>({
                path: "/tools/rosbag/start",
                method: "POST",
                type: ContentType.Json,
                format: "json",
            });
            await refresh();
            return response.data;
        } catch (e) {
            throw toError(e, "start failed");
        } finally {
            setBusy(null);
        }
    }, [guiApi, refresh]);

    const stop = useCallback(async () => {
        setBusy("stop");
        try {
            const response = await guiApi.request<RosbagStopResponse>({
                path: "/tools/rosbag/stop",
                method: "POST",
                type: ContentType.Json,
                format: "json",
            });
            await refresh();
            return response.data;
        } catch (e) {
            throw toError(e, "stop failed");
        } finally {
            setBusy(null);
        }
    }, [guiApi, refresh]);

    const remove = useCallback(async (name: string) => {
        try {
            await guiApi.request({
                path: `/tools/rosbag/${encodeURIComponent(name)}`,
                method: "DELETE",
                format: "json",
            });
        } catch (e) {
            throw toError(e, "delete failed");
        }
        await refresh();
    }, [guiApi, refresh]);

    // Direct download link — the backend streams a .tar.gz attachment, so a
    // plain anchor triggers the browser's download without a blob round-trip.
    const downloadUrl = useCallback((name: string) => {
        return `${guiApi.baseUrl}/tools/rosbag/download/${encodeURIComponent(name)}`;
    }, [guiApi]);

    return {status, error, busy, refresh, start, stop, remove, downloadUrl};
};
