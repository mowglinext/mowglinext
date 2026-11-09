import {isJsonObject} from './json';
import type {
    DriveTuningStatusResponse, DriveTuningLatestReportResponse, DriveTuningReport,
    DriveTuningTrial, DriveTuningRollbackResponse, DriveTuningJobStatus,
    DriveTuningValidationSummary, DriveTuningLatestReportMeta,
    DriveTuningStatusSnapshot, DriveTuningDrivetrainDiagnostics,
} from '../hooks/useDriveTuning';

export interface DriveTuningStartOptions {
    apply: boolean;
    allow_undock: boolean;
    undock_distance_m: number;
}
export interface DriveFeedForwardRequest extends DriveTuningStartOptions {
    distance_m: number;
    test_speed_mps: number;
    odom_timeout_s: number;
    passes: number;
    auto_turn: boolean;
    turn_direction: 'left' | 'right';
}
export interface DrivePIDRequest extends DriveTuningStartOptions {
    max_speed_mps: number;
    segment_duration_s: number;
    passes: number;
}
export interface DriveTuningStartResponse {
    job_id: string;
    mode: string;
    state: 'running';
    started_at: string;
    apply: boolean;
    report_path: string;
}
const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const text = (value: unknown): value is string => typeof value === 'string';
const nonemptyText = (value: unknown): boolean => text(value) && value.trim().length > 0;
const boolean = (value: unknown): value is boolean => typeof value === 'boolean';
const integer = (value: unknown): boolean => finite(value) && Number.isSafeInteger(value);
const texts = (value: unknown): value is string[] => Array.isArray(value) && value.every(text);
const numbers = (value: unknown): value is Record<string, number> => isJsonObject(value) && Object.values(value).every(finite);
const nullableNumber = (value: unknown): boolean => value === null || finite(value);
const nullableBoolean = (value: unknown): boolean => value === null || boolean(value);
const nullableText = (value: unknown): boolean => value === null || text(value);
const required = (raw: Record<string, unknown>, keys: string[], check: (value: unknown) => boolean) => keys.every(key => check(raw[key]));
const optional = (raw: Record<string, unknown>, keys: string[], check: (value: unknown) => boolean) => keys.every(key => !(key in raw) || check(raw[key]));
const runState = (value: unknown): boolean => ['idle', 'running', 'succeeded', 'warning', 'failed'].some(state => state === value);

function summary(raw: unknown): raw is DriveTuningValidationSummary {
    return isJsonObject(raw) && ['not_validated', 'validated', 'warning'].some(state => raw.status === state)
        && optional(raw, ['message', 'generated_at', 'report_path'], text);
}
function meta(raw: unknown): raw is DriveTuningLatestReportMeta {
    return isJsonObject(raw) && required(raw, ['mode'], text) && nonemptyText(raw.report_path) && optional(raw, ['generated_at'], text);
}
function job(raw: unknown): raw is DriveTuningJobStatus {
    return isJsonObject(raw) && required(raw, ['mode', 'started_at'], text) && required(raw, ['id', 'report_path'], nonemptyText)
        && runState(raw.state) && boolean(raw.apply)
        && optional(raw, ['finished_at', 'exec_id', 'error', 'logs'], text) && optional(raw, ['exit_code'], integer);
}
function snapshot(raw: unknown): raw is DriveTuningStatusSnapshot {
    return isJsonObject(raw) && optional(raw, ['active_emergency', 'latched_emergency', 'is_charging', 'esc_power'], nullableBoolean)
        && optional(raw, ['mower_status'], value => value === null || integer(value))
        && optional(raw, ['wheel_tick_factor'], nullableNumber) && optional(raw, ['last_wheel_tick_timestamp'], nullableText);
}
function drivetrain(raw: unknown): raw is DriveTuningDrivetrainDiagnostics {
    return isJsonObject(raw) && optional(raw, ['wheel_radius_m', 'wheel_circumference_m', 'estimated_wheel_revolutions_per_meter',
        'estimated_encoder_counts_per_wheel_revolution', 'configured_ticks_per_revolution'], nullableNumber)
        && optional(raw, ['notes'], texts);
}
function trial(raw: unknown): raw is DriveTuningTrial {
    return isJsonObject(raw) && required(raw, ['name', 'phase'], text)
        && required(raw, ['target_speed', 'measured_speed_mean', 'overshoot'], finite)
        && required(raw, ['stall_detected', 'oscillation_detected', 'integral_saturation_suspected', 'rtk_accepted'], boolean)
        && optional(raw, ['ticks_per_meter'], finite) && optional(raw, ['left_ticks_seen', 'right_ticks_seen'], integer)
        && optional(raw, ['settling_time', 'ground_speed_mean', 'odom_distance_m', 'rtk_distance_m', 'left_right_tick_imbalance'], nullableNumber)
        && optional(raw, ['live_oscillation_detected'], boolean) && optional(raw, ['trial_quality'], text)
        && optional(raw, ['warnings', 'notes'], texts);
}
function report(raw: unknown): raw is DriveTuningReport {
    return isJsonObject(raw) && required(raw, ['generated_at', 'mode', 'profile', 'backup_file', 'cmd_topic', 'turn_direction'], text)
        && required(raw, ['distance_m', 'max_speed_mps', 'segment_duration_s'], finite) && integer(raw.passes)
        && required(raw, ['applied_live', 'requested_apply', 'auto_turn'], boolean)
        && required(raw, ['current_params', 'starting_params', 'proposed_params'], numbers)
        && optional(raw, ['recommended'], numbers)
        && optional(raw, ['test_speed_mps', 'robot_mass_kg'], nullableNumber) && optional(raw, ['odom_timeout_s'], finite)
        && optional(raw, ['cmd_vel_topic', 'internal_tuning_tier', 'hardware_config_path', 'failure_message'], text)
        && optional(raw, ['status_snapshot'], snapshot) && optional(raw, ['drivetrain_diagnostics'], drivetrain)
        && texts(raw.reasons) && Array.isArray(raw.trials) && raw.trials.every(trial);
}
export function decodeDriveTuningStatus(raw: unknown): DriveTuningStatusResponse {
    if (!isJsonObject(raw) || !summary(raw.feed_forward) || !summary(raw.pid)
        || !optional(raw, ['job'], job) || !optional(raw, ['latest_report'], meta)) throw new Error('Invalid drive tuning status');
    return {feed_forward: raw.feed_forward, pid: raw.pid,
        ...(job(raw.job) ? {job: raw.job} : {}), ...(meta(raw.latest_report) ? {latest_report: raw.latest_report} : {})};
}
export function decodeDriveTuningReport(raw: unknown): DriveTuningLatestReportResponse {
    const status = decodeDriveTuningStatus(raw);
    if (!isJsonObject(raw) || !optional(raw, ['raw_yaml'], text)) throw new Error('Invalid drive tuning report');
    let parsed: DriveTuningReport | undefined;
    if ('parsed' in raw) {
        if (!isJsonObject(raw.parsed)) throw new Error('Invalid parsed drive tuning report');
        // Go encodes nil non-omitempty slices/maps as null. Empty collections
        // carry no validation result or suggested values; normalize only these.
        const candidate = {...raw.parsed,
            reasons: raw.parsed.reasons === null ? [] : raw.parsed.reasons,
            trials: raw.parsed.trials === null ? [] : raw.parsed.trials,
            current_params: raw.parsed.current_params === null ? {} : raw.parsed.current_params,
            starting_params: raw.parsed.starting_params === null ? {} : raw.parsed.starting_params,
            proposed_params: raw.parsed.proposed_params === null ? {} : raw.parsed.proposed_params};
        if (!report(candidate)) throw new Error('Invalid parsed drive tuning report');
        parsed = candidate;
    }
    return {...status, ...(parsed ? {parsed} : {}), ...(text(raw.raw_yaml) ? {raw_yaml: raw.raw_yaml} : {})};
}
export function decodeDriveTuningRollback(raw: unknown): DriveTuningRollbackResponse {
    if (!isJsonObject(raw) || !boolean(raw.success) || !optional(raw, ['message', 'backup_path', 'report_path', 'execution_log'], text)
        || !optional(raw, ['restored'], numbers)) throw new Error('Invalid drive tuning rollback response');
    if (!raw.success) throw new Error(text(raw.message) ? raw.message : 'Drive tuning rollback failed');
    return {success: raw.success, ...(text(raw.message) ? {message: raw.message} : {}), ...(numbers(raw.restored) ? {restored: raw.restored} : {}),
        ...(text(raw.backup_path) ? {backup_path: raw.backup_path} : {}), ...(text(raw.report_path) ? {report_path: raw.report_path} : {}),
        ...(text(raw.execution_log) ? {execution_log: raw.execution_log} : {})};
}
export function decodeDriveTuningStart(raw: unknown, expectedMode?: 'ff' | 'pid'): DriveTuningStartResponse {
    if (!isJsonObject(raw) || !required(raw, ['job_id', 'mode', 'started_at', 'report_path'], text) || !runState(raw.state) || !boolean(raw.apply)) throw new Error('Invalid drive tuning start response');
    // Validated through the same complete field predicate as job status.
    const candidate = {...raw, id: raw.job_id};
    if (!job(candidate) || !text(raw.job_id) || candidate.state !== 'running' || (expectedMode !== undefined && candidate.mode !== expectedMode)) throw new Error('Invalid drive tuning start response');
    return {job_id: raw.job_id, mode: candidate.mode, state: candidate.state, started_at: candidate.started_at, apply: candidate.apply, report_path: candidate.report_path};
}
export function validateDriveTuningRequest(raw: DriveFeedForwardRequest | DrivePIDRequest): void {
    if (!required({...raw}, ['apply', 'allow_undock'], boolean) || !finite(raw.undock_distance_m) || !integer(raw.passes)
        || ('distance_m' in raw ? !required({...raw}, ['distance_m', 'test_speed_mps', 'odom_timeout_s'], finite)
            || !boolean(raw.auto_turn) || (raw.turn_direction !== 'left' && raw.turn_direction !== 'right') : !required({...raw}, ['max_speed_mps', 'segment_duration_s'], finite))) throw new Error('Invalid drive tuning request');
}
