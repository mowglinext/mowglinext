import { useCallback, useEffect, useState } from "react";
import { ContentType } from "../api/Api.ts";
import { useApi } from "./useApi.ts";

export type DriveTuningValidationStatus = "not_validated" | "validated" | "warning";
export type DriveTuningRunState = "idle" | "running" | "succeeded" | "warning" | "failed";

export interface DriveTuningValidationSummary {
    status: DriveTuningValidationStatus;
    message?: string;
    generated_at?: string;
    report_path?: string;
}

export interface DriveTuningLatestReportMeta {
    mode: string;
    generated_at?: string;
    report_path: string;
}

export interface DriveTuningJobStatus {
    id: string;
    mode: string;
    state: DriveTuningRunState;
    started_at: string;
    finished_at?: string;
    apply: boolean;
    report_path: string;
    exec_id?: string;
    exit_code?: number;
    error?: string;
    logs?: string;
}

export interface DriveTuningStatusResponse {
    job?: DriveTuningJobStatus;
    feed_forward: DriveTuningValidationSummary;
    pid: DriveTuningValidationSummary;
    latest_report?: DriveTuningLatestReportMeta;
}

export interface DriveTuningTrial {
    name: string;
    phase: string;
    target_speed: number;
    measured_speed_mean: number;
    overshoot: number;
    settling_time?: number | null;
    stall_detected: boolean;
    oscillation_detected: boolean;
    integral_saturation_suspected: boolean;
    ground_speed_mean?: number | null;
    odom_distance_m?: number | null;
    rtk_distance_m?: number | null;
    rtk_accepted: boolean;
    left_right_tick_imbalance?: number | null;
    notes?: string[];
}

export interface DriveTuningReport {
    generated_at: string;
    mode: string;
    profile: string;
    backup_file: string;
    cmd_topic: string;
    applied_live: boolean;
    requested_apply: boolean;
    distance_m: number;
    max_speed_mps: number;
    test_speed_mps?: number | null;
    segment_duration_s: number;
    passes: number;
    auto_turn: boolean;
    turn_direction: string;
    current_params: Record<string, number>;
    starting_params: Record<string, number>;
    proposed_params: Record<string, number>;
    reasons: string[];
    trials: DriveTuningTrial[];
}

export interface DriveTuningLatestReportResponse {
    latest_report?: DriveTuningLatestReportMeta;
    feed_forward: DriveTuningValidationSummary;
    pid: DriveTuningValidationSummary;
    parsed?: DriveTuningReport;
    raw_yaml?: string;
}

export interface DriveTuningRollbackResponse {
    success: boolean;
    message?: string;
    restored?: Record<string, number>;
    backup_path?: string;
    report_path?: string;
    execution_log?: string;
}

const POLL_INTERVAL_MS = 3000;

export const useDriveTuning = () => {
    const guiApi = useApi();
    const [status, setStatus] = useState<DriveTuningStatusResponse | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [latestReport, setLatestReport] = useState<DriveTuningLatestReportResponse | null>(null);
    const [loadingLatestReport, setLoadingLatestReport] = useState(false);

    const refresh = useCallback(async () => {
        try {
            const response = await guiApi.request<DriveTuningStatusResponse>({
                path: "/tools/drive/tuning/status",
                method: "GET",
                format: "json",
            });
            setStatus(response.data);
            setError(null);
        } catch (e) {
            setError(e instanceof Error ? e.message : "Failed to fetch drive tuning status");
        }
    }, [guiApi]);

    useEffect(() => {
        refresh();
        const id = setInterval(refresh, POLL_INTERVAL_MS);
        return () => clearInterval(id);
    }, [refresh]);

    const startFeedForward = useCallback(async (payload: Record<string, any>) => {
        const response = await guiApi.request({
            path: "/tools/drive/ff-calibration/start",
            method: "POST",
            type: ContentType.Json,
            body: payload,
            format: "json",
        });
        await refresh();
        return response.data;
    }, [guiApi, refresh]);

    const startPID = useCallback(async (payload: Record<string, any>) => {
        const response = await guiApi.request({
            path: "/tools/drive/pid-tuning/start",
            method: "POST",
            type: ContentType.Json,
            body: payload,
            format: "json",
        });
        await refresh();
        return response.data;
    }, [guiApi, refresh]);

    const rollback = useCallback(async () => {
        const response = await guiApi.request<DriveTuningRollbackResponse>({
            path: "/tools/drive/tuning/rollback",
            method: "POST",
            type: ContentType.Json,
            body: { confirm: true },
            format: "json",
        });
        await refresh();
        return response.data;
    }, [guiApi, refresh]);

    const loadLatestReport = useCallback(async () => {
        setLoadingLatestReport(true);
        try {
            const response = await guiApi.request<DriveTuningLatestReportResponse>({
                path: "/tools/drive/tuning/report/latest",
                method: "GET",
                format: "json",
            });
            setLatestReport(response.data);
            return response.data;
        } finally {
            setLoadingLatestReport(false);
        }
    }, [guiApi]);

    return {
        status,
        error,
        latestReport,
        loadingLatestReport,
        refresh,
        startFeedForward,
        startPID,
        rollback,
        loadLatestReport,
    };
};
