import {observeAsync} from "../utils/async";
import {useCallback, useEffect, useRef, useState} from "react";
import {isJsonObject} from "../utils/json";
import {isExplicitReadError, requestErrorMessage} from "../utils/errors";
import {useApi} from "./useApi.ts";

export interface ContainerHealth {
    component?: string;
    name: string;
    state: string;
    status: string;
    started_at: string;
}

export interface AreaCoverageInfo {
    area_index: number;
    coverage_percent: number;
    total_cells: number;
    mowed_cells: number;
    obstacle_cells: number;
    strips_remaining: number;
}

export interface DockPoseCrossCheck {
    configured_x: number;
    configured_y: number;
    configured_yaw: number;
    datum_lat: number;
    datum_lon: number;
    has_config: boolean;
    // "mowgli_robot.yaml" | ""
    source?: string;
}

export interface CrossChecks {
    dock_pose: DockPoseCrossCheck;
    warnings: string[];
    overall_status: string; // "ok", "warn", "error"
}

export interface DiagnosticsSnapshot {
    timestamp: string;
    containers_available?: boolean;
    containers: ContainerHealth[];
    system: { cpu_temperature_available?: boolean; cpu_temperature?: number; cpu_usage?: number | null };
    coverage: AreaCoverageInfo[];
    cross_checks: CrossChecks;
}

export const SNAPSHOT_STALE_MS = 30_000;

/** Validate fields used by snapshot consumers; availability extensions are optional for old servers. */
export function decodeDiagnosticsSnapshot(raw: unknown): DiagnosticsSnapshot {
    if (!isJsonObject(raw) || typeof raw.timestamp !== 'string' || !Number.isFinite(Date.parse(raw.timestamp))
        || !Array.isArray(raw.containers) || !Array.isArray(raw.coverage) || !isJsonObject(raw.system)
        || !isJsonObject(raw.cross_checks)) throw new Error('Invalid diagnostics snapshot');
    const containers: ContainerHealth[] = [];
    for (const item of raw.containers) {
        if (!isJsonObject(item) || typeof item.name !== 'string' || typeof item.state !== 'string'
            || typeof item.status !== 'string' || typeof item.started_at !== 'string'
            || ('component' in item && typeof item.component !== 'string')) throw new Error('Invalid diagnostics container');
        containers.push({name: item.name, state: item.state, status: item.status, started_at: item.started_at,
            ...(typeof item.component === 'string' ? {component: item.component} : {})});
    }
    const system = raw.system;
    if (['cpu_temperature_available'].some(key => key in system && typeof system[key] !== 'boolean')
        || 'containers_available' in raw && typeof raw.containers_available !== 'boolean'
        || 'cpu_temperature' in system && (typeof system.cpu_temperature !== 'number' || !Number.isFinite(system.cpu_temperature))
        || system.cpu_usage != null && (typeof system.cpu_usage !== 'number' || !Number.isFinite(system.cpu_usage))) throw new Error('Invalid diagnostics system');
    const coverage: AreaCoverageInfo[] = [];
    for (const item of raw.coverage) {
        if (!isJsonObject(item)) throw new Error('Invalid diagnostics coverage');
        const {area_index, coverage_percent, total_cells, mowed_cells, obstacle_cells, strips_remaining} = item;
        if (typeof area_index !== 'number' || typeof coverage_percent !== 'number' || typeof total_cells !== 'number'
            || typeof mowed_cells !== 'number' || typeof obstacle_cells !== 'number' || typeof strips_remaining !== 'number'
            || ![area_index,coverage_percent,total_cells,mowed_cells,obstacle_cells,strips_remaining].every(Number.isFinite)) throw new Error('Invalid diagnostics coverage');
        coverage.push({area_index, coverage_percent, total_cells, mowed_cells, obstacle_cells, strips_remaining});
    }
    const cross = raw.cross_checks, dock = cross.dock_pose;
    if (!Array.isArray(cross.warnings) || !cross.warnings.every((v: unknown) => typeof v === 'string')
        || typeof cross.overall_status !== 'string' || !isJsonObject(dock) || typeof dock.has_config !== 'boolean'
        || typeof dock.configured_x !== 'number' || typeof dock.configured_y !== 'number' || typeof dock.configured_yaw !== 'number'
        || typeof dock.datum_lat !== 'number' || typeof dock.datum_lon !== 'number'
        || ![dock.configured_x,dock.configured_y,dock.configured_yaw,dock.datum_lat,dock.datum_lon].every(Number.isFinite)
        || 'source' in dock && typeof dock.source !== 'string') throw new Error('Invalid diagnostics cross-checks');
    return {timestamp: raw.timestamp, containers, coverage, system: {
        ...(typeof system.cpu_temperature === 'number' ? {cpu_temperature: system.cpu_temperature} : {}),
        ...(typeof system.cpu_temperature_available === 'boolean' ? {cpu_temperature_available: system.cpu_temperature_available} : {}),
        ...(system.cpu_usage === null || typeof system.cpu_usage === 'number' ? {cpu_usage: system.cpu_usage} : {})},
    ...(typeof raw.containers_available === 'boolean' ? {containers_available: raw.containers_available} : {}),
    cross_checks: {overall_status: cross.overall_status, warnings: cross.warnings, dock_pose: {
        configured_x: dock.configured_x, configured_y: dock.configured_y, configured_yaw: dock.configured_yaw,
        datum_lat: dock.datum_lat, datum_lon: dock.datum_lon, has_config: dock.has_config,
        ...(typeof dock.source === 'string' ? {source: dock.source} : {})}}};
}

export const useDiagnosticsSnapshot = () => {
    const guiApi = useApi();
    const [snapshot, setSnapshot] = useState<DiagnosticsSnapshot | null>(null);
    const active = useRef(false);
    const requestId = useRef(0);
    const inFlight = useRef(false);
    const last = useRef<{timestamp: string; at: number} | null>(null);
    const [receivedAt, setReceivedAt] = useState<number | null>(null);
    const [loading, setLoading] = useState(false);
    const [readError, setReadError] = useState<string | null>(null);
    const [error, setError] = useState<string | null>(null);

    const refresh = useCallback(async () => {
        if (!active.current) return;
        const request = ++requestId.current;
        inFlight.current = true;
        setLoading(true);
        let processing = false;
        try {
            const response = await guiApi.request<unknown>({
                path: "/diagnostics/snapshot",
                method: "GET",
                format: "json",
                cache: "no-store",
            });
            processing = response.error instanceof SyntaxError;
            if (response.error || response.ok === false) throw Object.assign(
                new Error(requestErrorMessage(response) || requestErrorMessage(response.error) || 'Read request failed'), {status: response.status});
            processing = true;
            const next = decodeDiagnosticsSnapshot(response.data);
            if (!active.current || request !== requestId.current) return;
            if (last.current && Date.parse(next.timestamp) < Date.parse(last.current.timestamp)) throw new Error('Regressed diagnostics snapshot');
            const at = last.current?.timestamp === next.timestamp ? last.current.at : performance.now();
            last.current = {timestamp: next.timestamp, at};
            setReceivedAt(at);
            setSnapshot(next);
            setError(null);
            setReadError(null);
        } catch (e) {
            if (!active.current || request !== requestId.current) return;
            const message = requestErrorMessage(e)?.trim() || "Failed to fetch diagnostics snapshot";
            setError(message);
            setReadError(processing || isExplicitReadError(e) ? message : null);
        } finally {
            if (request === requestId.current) {
                inFlight.current = false;
                if (active.current) setLoading(false);
            }
        }
    }, [guiApi]);

    useEffect(() => {
        active.current = true;
        observeAsync(refresh());
        const interval = setInterval(() => {if (!inFlight.current) observeAsync(refresh());}, 10000);
        return () => {active.current = false; clearInterval(interval);};
    }, [refresh]);

    return {snapshot, receivedAt, loading, error, readError, refresh};
};
