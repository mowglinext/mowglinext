import {useCallback, useEffect, useState} from "react";
import {useApi} from "../../../hooks/useApi.ts";
import type {LidarIgnoreCorridor} from "../../../types/ros.ts";

/// Default ignore width (metres, total — half on each side of the line) given
/// to a freshly drawn corridor. map_server clamps to [0.05, 1.0] m regardless.
export const DEFAULT_CORRIDOR_WIDTH_M = 0.2;

const errorMessage = (error: unknown): string =>
    (error as {error?: {error?: string}} | null)?.error?.error
    ?? (error instanceof Error ? error.message : "request failed");

/// Owns the operator-drawn LiDAR-ignore corridor list (map_server's
/// LidarIgnoreCorridorEntry). Kept apart from useMapEditing on purpose: a
/// corridor is not an area/obstacle polygon and must not go through the
/// polygon edit/undo/save pipeline. Every change replaces the whole list
/// (set_lidar_ignore_corridors = clear + add each), then reloads it.
export const useLidarCorridors = () => {
    const api = useApi();
    const [corridors, setCorridors] = useState<LidarIgnoreCorridor[]>([]);
    const [busy, setBusy] = useState(false);

    const reload = useCallback(async () => {
        try {
            const res = await api.mowglinext.callCreate("get_lidar_ignore_corridors", {});
            if (res.error) return;
            const list = (res.data as unknown as {corridors?: LidarIgnoreCorridor[]} | undefined)?.corridors;
            const next = Array.isArray(list) ? list : [];
            // Keep the same array when nothing changed, so the periodic refresh
            // does not re-render the map every 10 s.
            setCorridors((prev) => JSON.stringify(prev) === JSON.stringify(next) ? prev : next);
        } catch {
            // map_server may not be up yet; the panel just shows an empty list.
        }
    }, [api]);

    // Load on mount and keep refreshing: right after a ROS2 restart map_server's
    // service is not advertised yet, so a one-shot load left the panel empty
    // (field 2026-09-25: line "gone" after a reboot although map_server had it).
    useEffect(() => {
        void reload();
        const timer = window.setInterval(() => void reload(), 10000);
        return () => window.clearInterval(timer);
    }, [reload]);

    /// Replace the whole list. Throws the backend's message on failure.
    const save = useCallback(async (next: LidarIgnoreCorridor[]) => {
        setBusy(true);
        try {
            const res = await api.mowglinext.callCreate("set_lidar_ignore_corridors", {
                corridors: next.map((c) => ({
                    name: c.name ?? "",
                    polyline: {points: c.polyline?.points ?? []},
                    width_m: c.width_m ?? DEFAULT_CORRIDOR_WIDTH_M,
                    id: c.id ?? 0,
                })),
            });
            if (res.error) throw new Error(res.error.error);
        } catch (error: unknown) {
            await reload();
            throw new Error(errorMessage(error));
        } finally {
            setBusy(false);
        }
        await reload();
    }, [api, reload]);

    return {corridors, busy, reload, save};
};
