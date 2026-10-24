import {observeAsync} from "../utils/async";
import { useCallback, useEffect, useRef, useState } from "react";
import {decodeDriveTuningStatus, decodeDriveTuningReport, decodeDriveTuningRollback, decodeDriveTuningStart,
    validateDriveTuningRequest, type DriveFeedForwardRequest, type DrivePIDRequest} from '../utils/driveTuningContract';
import {requestErrorMessage} from '../utils/errors';
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
    ticks_per_meter?: number;
    left_ticks_seen?: number;
    right_ticks_seen?: number;
    name: string;
    phase: string;
    target_speed: number;
    measured_speed_mean: number;
    overshoot: number;
    settling_time?: number | null;
    stall_detected: boolean;
    oscillation_detected: boolean;
    live_oscillation_detected?: boolean;
    trial_quality?: string;
    integral_saturation_suspected: boolean;
    ground_speed_mean?: number | null;
    odom_distance_m?: number | null;
    rtk_distance_m?: number | null;
    rtk_accepted: boolean;
    left_right_tick_imbalance?: number | null;
    warnings?: string[];
    notes?: string[];
}

export interface DriveTuningStatusSnapshot {
    active_emergency?: boolean | null;
    latched_emergency?: boolean | null;
    is_charging?: boolean | null;
    mower_status?: number | null;
    esc_power?: boolean | null;
    wheel_tick_factor?: number | null;
    last_wheel_tick_timestamp?: string | null;
}

export interface DriveTuningDrivetrainDiagnostics {
    wheel_radius_m?: number | null;
    wheel_circumference_m?: number | null;
    estimated_wheel_revolutions_per_meter?: number | null;
    estimated_encoder_counts_per_wheel_revolution?: number | null;
    configured_ticks_per_revolution?: number | null;
    notes?: string[];
}

export interface DriveTuningReport {
    recommended?: Record<string, number>;
    generated_at: string;
    mode: string;
    profile: string;
    backup_file: string;
    cmd_topic: string;
    cmd_vel_topic?: string;
    applied_live: boolean;
    requested_apply: boolean;
    distance_m: number;
    max_speed_mps: number;
    test_speed_mps?: number | null;
    segment_duration_s: number;
    odom_timeout_s?: number;
    passes: number;
    auto_turn: boolean;
    turn_direction: string;
    robot_mass_kg?: number | null;
    internal_tuning_tier?: string;
    hardware_config_path?: string;
    drivetrain_diagnostics?: DriveTuningDrivetrainDiagnostics;
    current_params: Record<string, number>;
    starting_params: Record<string, number>;
    proposed_params: Record<string, number>;
    failure_message?: string;
    status_snapshot?: DriveTuningStatusSnapshot;
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

    const active = useRef(true);
    const statusRequest = useRef(0);
    const reportRequest = useRef(0);
    const refresh = useCallback(async () => {
        if (!active.current) return;
        const request = ++statusRequest.current;
        try {
            const response = await guiApi.request<unknown>({
                path: "/tools/drive/tuning/status",
                method: "GET",
                format: "json",
            });
            if (response.error) throw new Error(requestErrorMessage(response) ?? "Drive tuning request failed");
            const decoded = decodeDriveTuningStatus(response.data);
            if (active.current && request === statusRequest.current) {
                setStatus(decoded);
                setError(null);
            }
        } catch (e) {
            if (active.current && request === statusRequest.current) {
                setStatus(null);
                setError(requestErrorMessage(e) ?? "Failed to fetch drive tuning status");
            }
        }
    }, [guiApi]);

    const invalidateRequests = useCallback(() => {
        ++statusRequest.current;
        ++reportRequest.current;
    }, []);
    useEffect(() => {
        active.current = true;
        observeAsync(refresh());
        const id = setInterval(() => observeAsync(refresh()), POLL_INTERVAL_MS);
        return () => {
            active.current = false;
            invalidateRequests();
            clearInterval(id);
        };
    }, [refresh, invalidateRequests]);

    const startFeedForward = useCallback(async (payload: DriveFeedForwardRequest) => {
        validateDriveTuningRequest(payload);
        const response = await guiApi.request<unknown>({
            path: "/tools/drive/ff-calibration/start",
            method: "POST",
            type: ContentType.Json,
            body: payload,
            format: "json",
        });
        if (response.error) throw new Error(requestErrorMessage(response) ?? "Drive tuning request failed");
        const decoded = decodeDriveTuningStart(response.data, 'ff');
        await refresh();
        return decoded;
    }, [guiApi, refresh]);

    const startPID = useCallback(async (payload: DrivePIDRequest) => {
        validateDriveTuningRequest(payload);
        const response = await guiApi.request<unknown>({
            path: "/tools/drive/pid-tuning/start",
            method: "POST",
            type: ContentType.Json,
            body: payload,
            format: "json",
        });
        if (response.error) throw new Error(requestErrorMessage(response) ?? "Drive tuning request failed");
        const decoded = decodeDriveTuningStart(response.data, 'pid');
        await refresh();
        return decoded;
    }, [guiApi, refresh]);

    const rollback = useCallback(async () => {
        const response = await guiApi.request<unknown>({
            path: "/tools/drive/tuning/rollback",
            method: "POST",
            type: ContentType.Json,
            body: { confirm: true },
            format: "json",
        });
        if (response.error) throw new Error(requestErrorMessage(response) ?? "Drive tuning request failed");
        const decoded = decodeDriveTuningRollback(response.data);
        await refresh();
        return decoded;
    }, [guiApi, refresh]);

    const loadLatestReport = useCallback(async () => {
        const request = ++reportRequest.current;
        if (active.current) setLoadingLatestReport(true);
        try {
            const response = await guiApi.request<unknown>({
                path: "/tools/drive/tuning/report/latest",
                method: "GET",
                format: "json",
            });
            if (response.error) throw new Error(requestErrorMessage(response) ?? "Drive tuning request failed");
            const decoded = decodeDriveTuningReport(response.data);
            if (active.current && request === reportRequest.current) setLatestReport(decoded);
            return decoded;
        } catch (e) {
            if (active.current && request === reportRequest.current) setLatestReport(null);
            throw e;
        } finally {
            if (active.current && request === reportRequest.current) setLoadingLatestReport(false);
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
