import React, {ChangeEvent} from "react";
import {useTranslation} from "react-i18next";
import type {NotificationInstance} from "antd/es/notification/interface";
import type {FeatureCollection} from "geojson";
import type {Map as MapType} from "../../../types/ros.ts";
import {
    MowingFeature,
    MowingAreaFeature,
    NavigationFeature,
    ObstacleFeature,
    DockFeatureBase,
    MowingFeatureBase,
    featuresFromJSON,
    type SerializedMapFeature,
} from "../../../types/map.ts";
import type {Api, MowgliMapArea, MowgliReplaceMapReq} from "../../../api/Api.ts";
import {dedupePoints, getQuaternionFromHeading, isRingInsidePolygon, itranspose} from "../../../utils/map.tsx";

interface UseMapFilesOptions {
    features: Record<string, MowingFeature>;
    setFeatures: React.Dispatch<React.SetStateAction<Record<string, MowingFeature>>>;
    map: MapType | undefined;
    setMap: React.Dispatch<React.SetStateAction<MapType | undefined>>;
    editMap: boolean;
    setEditMap: React.Dispatch<React.SetStateAction<boolean>>;
    setHasUnsavedChanges: (v: boolean) => void;
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    notification: NotificationInstance;
    guiApi: Api<unknown>;
    dockDirty: boolean;
    setDockDirty: (v: boolean) => void;
    // Builds the editable feature set (areas, obstacles, dock) from a Map
    // message. Needed by handleRestoreMap because the MapPage effect that
    // normally does this is intentionally skipped while editMap is true.
    buildFeaturesFromMap: (m: MapType) => Record<string, MowingFeature>;
}

export function useMapFiles({
    features,
    setFeatures,
    map,
    setMap,
    setEditMap,
    setHasUnsavedChanges,
    offsetX,
    offsetY,
    datum,
    notification,
    guiApi,
    dockDirty,
    setDockDirty,
    buildFeaturesFromMap,
}: UseMapFilesOptions) {
    const {t} = useTranslation();

    async function handleSaveMap() {
        const areas: Record<string, MowgliMapArea[]> = {
            "area": [],
            "navigation": [],
        };

        // Separate features by role: workareas/nav first, obstacles second
        const areaFeatures: MowingFeatureBase[] = [];
        const obstacleFeatures: ObstacleFeature[] = [];

        for (const f of Object.values(features)) {
            if (f instanceof ObstacleFeature) {
                obstacleFeatures.push(f);
            } else if (f instanceof MowingAreaFeature || f instanceof NavigationFeature) {
                areaFeatures.push(f);
            }
        }

        // Sort workareas by mowing_order, navigation areas come after
        areaFeatures.sort((a, b) => {
            if (a instanceof MowingAreaFeature && !(b instanceof MowingAreaFeature)) return -1;
            if (!(a instanceof MowingAreaFeature) && b instanceof MowingAreaFeature) return 1;
            return (a.properties.mowing_order ?? 9999) - (b.properties.mowing_order ?? 9999);
        });

        // Track per-type index counters and map feature ID → index in areas array
        const typeCounters: Record<string, number> = {"area": 0, "navigation": 0};
        const featureIndexMap: Record<string, {type: string; index: number}> = {};

        for (const f of areaFeatures) {
            // Bucket by CLASS, not by id prefix: a workarea↔navigation type
            // change keeps the old id (e.g. "area-1-area-1" is now a
            // NavigationFeature), so id-prefix bucketing silently dropped the
            // converted area from the save payload.
            const type = f instanceof NavigationFeature ? "navigation" : "area";

            const index = typeCounters[type]++;
            featureIndexMap[f.id] = {type, index};

            const rawPoints = f.geometry.coordinates[0].map((point) => {
                const p = itranspose(offsetX, offsetY, datum, point[1], point[0]);
                return {x: p[0], y: p[1], z: 0};
            });
            const points = dedupePoints(rawPoints);

            areas[type][index] = {
                name: f.properties?.name ?? '',
                area: {points},
            };
        }

        // Process obstacles and attach them to their parent area. When the
        // recorded parent link is broken (stale reference after edits), try
        // to re-parent by geometry containment before giving up; anything
        // still unmatched is reported to the user instead of vanishing
        // silently from the saved map.
        const droppedObstacles: string[] = [];
        for (const f of obstacleFeatures) {
            const parentArea = f.getMowingArea();
            let parentMapping = parentArea ? featureIndexMap[parentArea.id] : undefined;
            if (!parentMapping) {
                const obstacleRing = f.geometry.coordinates[0] ?? [];
                const containing = areaFeatures.find(
                    (a) =>
                        a instanceof MowingAreaFeature &&
                        isRingInsidePolygon(obstacleRing, a.geometry.coordinates[0] ?? [])
                );
                if (containing) parentMapping = featureIndexMap[containing.id];
            }
            if (!parentMapping) {
                droppedObstacles.push(String(f.id));
                continue;
            }

            const rawPoints = f.geometry.coordinates[0].map((point) => {
                const p = itranspose(offsetX, offsetY, datum, point[1], point[0]);
                return {x: p[0], y: p[1], z: 0};
            });
            const points = dedupePoints(rawPoints);

            const target = areas[parentMapping.type][parentMapping.index];
            target.obstacles = [...(target.obstacles ?? []), {points}];
        }

        const updateMsg: MowgliReplaceMapReq = {
            areas: [],
        };
        for (const [type, areasOfType] of Object.entries(areas)) {
            for (const area of areasOfType) {
                updateMsg.areas!.push({
                    area,
                    is_navigation_area: type === "navigation",
                });
            }
        }

        if (droppedObstacles.length > 0) {
            notification.warning({
                message: t('mapFiles.obstaclesDropped'),
                description: t('mapFiles.obstaclesDroppedDescription', {
                    ids: droppedObstacles.join(", "),
                }),
            });
        }

        // Sequence BOTH saves in one try/catch: success is only reported (and
        // edit mode only exited) once the areas AND the dock pose are actually
        // persisted. A dock POST failure previously escaped the handler after
        // the success toast had already fired.
        try {
            await guiApi.mowglinext.putMowglinext(updateMsg);

            // Save dock position only when the user actually edited it.
            // Otherwise the dock feature reflects whatever the /map topic
            // last published, which can be stale relative to the dock pose
            // persisted in mowgli_robot.yaml (e.g. just-written by the
            // calibration service). Saving unconditionally would clobber it.
            const dockFeature = features["dock"];
            if (dockDirty && dockFeature instanceof DockFeatureBase) {
                const coords = dockFeature.getCoordinates();
                const rosCoords = itranspose(offsetX, offsetY, datum, coords[1], coords[0]);
                const heading = dockFeature.getHeading();
                const quaternionFromHeading = getQuaternionFromHeading(heading);
                await guiApi.mowglinext.mapDockingCreate({
                    docking_pose: {
                        orientation: {
                            x: quaternionFromHeading.x!!,
                            y: quaternionFromHeading.y!!,
                            z: quaternionFromHeading.z!!,
                            w: quaternionFromHeading.w!!,
                        },
                        position: {
                            x: rosCoords[0],
                            y: rosCoords[1],
                            z: 0,
                        },
                    },
                    // Manual map-drag: use the dragged coordinates as-is (operator
                    // placed the dock marker explicitly — do NOT override with GPS).
                    use_gps_position: false,
                    // Honour the dragged heading (SetDockingPoint yaw_source REQUEST=1);
                    // the map-drag yaw is operator-set, never circular.
                    yaw_source: 1,
                });
                setDockDirty(false);
            }

            notification.success({
                message: t('mapFiles.areaSaved'),
            });
            setHasUnsavedChanges(false);
            setEditMap(false);
        } catch (e: any) {
            // Stay in edit mode so the user's changes are not lost.
            notification.error({
                message: t('mapFiles.failedToSaveArea'),
                description: e?.message ?? String(e),
            });
        }
    }

    const handleBackupMap = () => {
        const a = document.createElement("a");
        document.body.appendChild(a);
        a.style.display = "none";
        const json = JSON.stringify(map),
            blob = new Blob([json], {type: "octet/stream"}),
            url = window.URL.createObjectURL(blob);
        a.href = url;
        a.download = "map.json";
        a.click();
        window.URL.revokeObjectURL(url);
    };

    const handleRestoreMap = () => {
        const input = document.createElement("input");
        input.type = "file";
        input.style.display = "none";
        document.body.appendChild(input);
        input.addEventListener('change', (event) => {
            setEditMap(true);
            const file = (event as unknown as ChangeEvent<HTMLInputElement>).target?.files?.[0];
            if (!file) {
                return;
            }
            const reader = new FileReader();
            reader.addEventListener('load', (event) => {
                const content = event.target?.result as string;
                const parts = content.split(",");
                const newMap = JSON.parse(atob(parts[1])) as MapType;
                setMap(newMap);
                // The MapPage effect that turns a Map into editable features
                // skips while editMap is true (set just above), so build them
                // here — otherwise handleSaveMap would persist the stale
                // features and the restored map would be silently discarded.
                setFeatures(buildFeaturesFromMap(newMap));
                setHasUnsavedChanges(true);
                setDockDirty(true);
            });
            reader.readAsDataURL(file);
        });
        input.click();
    };

    const handleDownloadGeoJSON = () => {
        // Export only the user's map features (areas, obstacles, dock) —
        // never the transient display features (mower, footprint, heading,
        // plan, dyn-obs…), which would otherwise pollute the file and break
        // re-import. Serialize plain GeoJSON, not class instances.
        const geojson = {
            type: "FeatureCollection",
            features: Object.values(features)
                .filter((f) => f instanceof MowingFeatureBase || f instanceof DockFeatureBase)
                .map((f) => ({
                    type: "Feature" as const,
                    id: f.id,
                    geometry: f.geometry,
                    properties: f.properties,
                })),
        };
        const a = document.createElement("a");
        document.body.appendChild(a);
        a.style.display = "none";
        const json = JSON.stringify(geojson),
            blob = new Blob([json], {type: "application/geo+json"}),
            url = window.URL.createObjectURL(blob);
        a.href = url;
        a.download = "map.geojson";
        a.click();
        window.URL.revokeObjectURL(url);
    };

    /**
     * Pick + parse an OpenMower map.json (or a `.bag` — currently
     * unsupported, see docs/IMPORT_OPENMOWER_MAP.md §6) and POST it to
     * /api/import/openmower in **preview mode**. The Go backend returns
     * an `ImportOpenMowerSummary` which the caller passes to
     * `setImportPreview` so MapPage can render a confirmation modal.
     *
     * The picked file body is also stashed via `setImportFileText` so
     * `handleApplyOpenMowerImport` can re-POST it with `apply: true`
     * when the user confirms in the modal. Stashing the verbatim text
     * (instead of re-walking the parsed JSON) keeps the apply payload
     * byte-identical to what was previewed.
     */
    const handleImportOpenMower = (
        setImportPreview: (preview: ImportOpenMowerSummary | null) => void,
        setImportFileText: (text: string | null) => void,
    ) => {
        const input = document.createElement("input");
        input.type = "file";
        // Accept .json directly. .bag files are caught client-side and
        // the user is told the path isn't ready yet (matches the
        // "coming soon" behaviour described in the design doc).
        input.accept = ".json,.bag,application/json";
        input.style.display = "none";
        document.body.appendChild(input);
        input.addEventListener('change', async (event) => {
            const file = (event as unknown as ChangeEvent<HTMLInputElement>).target?.files?.[0];
            if (!file) {
                return;
            }

            if (file.name.toLowerCase().endsWith(".bag")) {
                notification.info({
                    message: t('mapFiles.bagImportComingSoonMessage'),
                    description: t('mapFiles.bagImportComingSoonDescription'),
                });
                return;
            }

            try {
                const text = await file.text();
                // Parse client-side first so a totally bogus file fails
                // before we hit the network.
                JSON.parse(text);

                const res = await fetch("/api/import/openmower", {
                    method: "POST",
                    headers: {"Content-Type": "application/json"},
                    body: text,
                });
                if (!res.ok) {
                    const errBody = await res.text();
                    throw new Error(`HTTP ${res.status}: ${errBody}`);
                }
                const summary = (await res.json()) as ImportOpenMowerSummary;
                setImportFileText(text);
                setImportPreview(summary);
            } catch (e: any) {
                notification.error({
                    message: t('mapFiles.openMowerImportFailed'),
                    description: e?.message ?? String(e),
                });
            }
        });
        input.click();
    };

    /**
     * Re-run the preview for an already-stashed map.json, this time
     * supplying the OpenMower datum (OM_DATUM_LAT/LONG). The server
     * reprojects every vertex from the OM datum into the MowgliNext map
     * frame; without it the import lands at a constant offset (and a slight
     * skew). Called by the modal when the user fills in / clears the datum
     * fields. Returns the fresh summary so the modal can re-render the
     * preview (datum-shift alert, dock pose, warnings).
     */
    const handleReprojectOpenMowerPreview = async (
        importFileText: string,
        omDatumLat?: number,
        omDatumLon?: number,
    ): Promise<ImportOpenMowerSummary> => {
        if (!importFileText) {
            throw new Error("no stashed map text — pick a file before re-projecting");
        }
        const body: Record<string, unknown> = {map: JSON.parse(importFileText)};
        if (omDatumLat !== undefined && omDatumLon !== undefined) {
            body.om_datum_lat = omDatumLat;
            body.om_datum_lon = omDatumLon;
        }
        const res = await fetch("/api/import/openmower", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(body),
        });
        if (!res.ok) {
            const errBody = await res.text();
            throw new Error(`HTTP ${res.status}: ${errBody}`);
        }
        return (await res.json()) as ImportOpenMowerSummary;
    };

    /**
     * Confirm step of the OpenMower import. Re-POSTs the stashed file
     * body with `apply: true` so the server runs the same parse +
     * validate pipeline the user already saw in the preview modal, then
     * fires clear_map → add_area×N → save_areas → set_docking_point.
     *
     * The OpenMower datum (when the user supplied it) is sent again so the
     * applied geometry matches the previewed, reprojected geometry exactly.
     *
     * On success the modal closes; `/map` will refresh on its own via
     * the existing websocket stream — no manual refetch needed. We do
     * drop the dirty / editMap flags so a previous in-progress local
     * edit doesn't reappear over the freshly imported areas.
     */
    const handleApplyOpenMowerImport = async (
        importFileText: string,
        omDatumLat?: number,
        omDatumLon?: number,
    ): Promise<void> => {
        if (!importFileText) {
            throw new Error("no stashed map text — preview must run before apply");
        }
        const body: Record<string, unknown> = {map: JSON.parse(importFileText), apply: true};
        if (omDatumLat !== undefined && omDatumLon !== undefined) {
            body.om_datum_lat = omDatumLat;
            body.om_datum_lon = omDatumLon;
        }
        const res = await fetch("/api/import/openmower", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(body),
        });
        if (!res.ok) {
            const errBody = await res.text();
            throw new Error(`HTTP ${res.status}: ${errBody}`);
        }
        // Drop any in-progress edit so the freshly-imported map shows clean.
        setEditMap(false);
        setHasUnsavedChanges(false);
        setDockDirty(false);
        notification.success({
            message: t('mapFiles.openMowerMapImported'),
            description: t('mapFiles.openMowerMapImportedDescription'),
        });
    };

    const handleUploadGeoJSON = () => {
        const input = document.createElement("input");
        input.type = "file";
        input.style.display = "none";
        document.body.appendChild(input);
        input.addEventListener('change', (event) => {
            const file = (event as unknown as ChangeEvent<HTMLInputElement>).target?.files?.[0];
            if (!file) {
                return;
            }
            const reader = new FileReader();
            reader.onload = (event) => {
                // Parse + VALIDATE the whole file into plain snapshots BEFORE
                // touching state. The old code type-cast raw GeoJSON objects to
                // feature classes (never constructing them), so every
                // `instanceof` check downstream failed: the polygons never
                // rendered and Save persisted an empty map. We now build a
                // SerializedMapFeature snapshot and rehydrate real class
                // instances through the same factory the edit-history uses.
                let geojson: FeatureCollection;
                try {
                    geojson = JSON.parse(event.target?.result as string) as FeatureCollection;
                } catch {
                    notification.error({message: t('mapFiles.uploadParseError')});
                    return;
                }
                if (!geojson || !Array.isArray(geojson.features)) {
                    notification.error({message: t('mapFiles.uploadParseError')});
                    return;
                }

                const KNOWN_TYPES = new Set(['workarea', 'navigation', 'obstacle', 'dock']);
                const snapshot: Record<string, SerializedMapFeature> = {};
                for (const element of geojson.features) {
                    if (element.id == null || !element.geometry) continue;
                    const id = String(element.id);
                    const featureType = element.properties?.feature_type as string | undefined;
                    if (!featureType || !KNOWN_TYPES.has(featureType)) {
                        // Abort WITHOUT mutating state — a single bad feature
                        // must not partially overwrite the current map.
                        notification.error({
                            message: t('mapFiles.unknownType', {type: featureType ?? '?'}),
                        });
                        return;
                    }
                    snapshot[id] = {
                        id,
                        type: 'Feature',
                        geometry: element.geometry as SerializedMapFeature['geometry'],
                        properties: (element.properties ?? {}) as Record<string, unknown>,
                        parent_id: element.properties?.mowing_area as string | undefined,
                    };
                }

                if (Object.keys(snapshot).length === 0) {
                    notification.error({message: t('mapFiles.uploadEmpty')});
                    return;
                }

                const newFeatures = featuresFromJSON(snapshot);
                setFeatures(newFeatures);
                setHasUnsavedChanges(true);
                notification.success({message: t('mapFiles.uploadSuccess')});
            };
            reader.readAsText(file);
        });
        input.click();
    };

    return {
        handleSaveMap,
        handleBackupMap,
        handleRestoreMap,
        handleDownloadGeoJSON,
        handleUploadGeoJSON,
        handleImportOpenMower,
        handleReprojectOpenMowerPreview,
        handleApplyOpenMowerImport,
    };
}

/**
 * Mirror of api.ImportOpenMowerSummary (gui/pkg/api/openmower_import.go).
 * Kept inline rather than re-generating Api.ts because the importer
 * route is hand-rolled (not driven by swagger). Once the apply path
 * goes live this should be promoted to the swagger surface and
 * re-generated.
 */
export interface ImportOpenMowerSummary {
    mowing_areas: number;
    navigation_areas: number;
    obstacles: number;
    orphan_obstacles: number;
    dock_pose?: {x: number; y: number; yaw_rad: number} | null;
    datum_shift_east_m: number;
    datum_shift_north_m: number;
    warnings: string[];
    areas: Array<{
        name: string;
        type: string;
        vertices: number;
        obstacles: number;
        is_navigation_area: boolean;
        approx_area_sqm: number;
    }>;
    applied: boolean;
}
