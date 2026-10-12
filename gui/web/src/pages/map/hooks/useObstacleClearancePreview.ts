import {useCallback, useEffect, useMemo, useState} from "react";
import type {Feature, FeatureCollection} from "geojson";
import {useApi} from "../../../hooks/useApi.ts";
import {itranspose, transpose} from "../../../utils/map.tsx";
import type {ObstacleFeature} from "../../../types/map.ts";

/// Toggleable preview of what coverage_server's LIVE obstacle_margin actually
/// grows every drawn obstacle to at plan time (bufferRingOutward, reused
/// server-side via the new preview_obstacle_clearance service — never
/// reimplemented client-side, so this can never drift from the real plan).
/// Off by default: a dashed outline per obstacle is exactly the kind of extra
/// map clutter operators asked to be able to turn off.
export const useObstacleClearancePreview = (
    obstacles: ObstacleFeature[],
    datum: [number, number, number],
    offsetX: number,
    offsetY: number,
) => {
    const api = useApi();
    const [enabled, setEnabled] = useState(false);
    const [loading, setLoading] = useState(false);
    const [features, setFeatures] = useState<FeatureCollection>({type: "FeatureCollection", features: []});

    // A content signature, not the array reference: `obstacles` is rebuilt on
    // every render by the caller (Object.values(...).filter(...)), so keying
    // the fetch on the raw array would refetch every render.
    const signature = useMemo(
        () => obstacles.map((f) => `${f.id}:${JSON.stringify(f.geometry.coordinates[0] ?? [])}`).join("|"),
        [obstacles],
    );

    const refresh = useCallback(async () => {
        if (!enabled || datum[0] === 0 || obstacles.length === 0) {
            setFeatures({type: "FeatureCollection", features: []});
            return;
        }
        setLoading(true);
        try {
            const rosObstacles = obstacles.map((f) => ({
                points: (f.geometry.coordinates[0] ?? []).map((coord) => {
                    const [lon, lat] = coord as [number, number];
                    const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
                    return {x, y};
                }),
            }));
            const res = await api.mowglinext.callCreate("preview_obstacle_clearance", {obstacles: rosObstacles});
            if (res.error) return;
            const data = res.data as unknown as {buffered?: {points?: {x?: number; y?: number}[]}[]};
            const out: Feature[] = (data.buffered ?? [])
                .filter((poly) => (poly.points?.length ?? 0) >= 3)
                .map((poly) => ({
                    type: "Feature",
                    properties: {kind: "obstacle-clearance-preview"},
                    geometry: {
                        type: "Polygon",
                        coordinates: [(poly.points ?? []).map((p) =>
                            transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0))],
                    },
                }));
            setFeatures({type: "FeatureCollection", features: out});
        } catch {
            // coverage_server may not be advertised yet; just show nothing.
            setFeatures({type: "FeatureCollection", features: []});
        } finally {
            setLoading(false);
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [enabled, signature, datum, offsetX, offsetY, api]);

    useEffect(() => { void refresh(); }, [refresh]);

    return {enabled, setEnabled, loading, features, refresh};
};
