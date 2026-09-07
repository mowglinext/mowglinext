import {useCallback, useEffect, useState} from "react";
import {useApi} from "./useApi.ts";

export interface ContainerHealth {
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
    containers: ContainerHealth[];
    system: { cpu_temperature: number; cpu_usage?: number | null };
    coverage: AreaCoverageInfo[];
    cross_checks: CrossChecks;
}

export const useDiagnosticsSnapshot = () => {
    const guiApi = useApi();
    const [snapshot, setSnapshot] = useState<DiagnosticsSnapshot | null>(null);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const refresh = useCallback(async () => {
        setLoading(true);
        setError(null);
        try {
            const response = await guiApi.request<DiagnosticsSnapshot>({
                path: "/diagnostics/snapshot",
                method: "GET",
                format: "json",
            });
            setSnapshot(response.data);
        } catch (e) {
            setError(e instanceof Error ? e.message : "Failed to fetch diagnostics snapshot");
        } finally {
            setLoading(false);
        }
    }, []);

    useEffect(() => {
        refresh();
        const interval = setInterval(refresh, 10000);
        return () => clearInterval(interval);
    }, [refresh]);

    return {snapshot, loading, error, refresh};
};
