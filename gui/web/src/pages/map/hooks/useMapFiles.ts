import {asyncEvent, reportAsyncError} from "../../../utils/async";
import {errorMessage} from "../../../utils/errors";
import React from "react";
import {useTranslation} from "react-i18next";
import type {NotificationInstance} from "antd/es/notification/interface";
import type {Point, Polygon, Position} from "geojson";
import {isJsonObject} from "../../../utils/json";
import type {LidarIgnoreCorridor, Map as MapType} from "../../../types/ros.ts";
import {
    MowingFeature,
    MowingAreaFeature,
    MapAreaFeature,
    NavigationFeature,
    ObstacleFeature,
    DockFeatureBase,
    MowingFeatureBase,
    featuresFromJSON,
    type SerializedMapFeature,
} from "../../../types/map.ts";
import type {Api, MowgliMapArea, MowgliReplaceMapReq} from "../../../api/Api.ts";
import {BACKUP_CORRIDORS_KEY, BACKUP_ORIGINALS_KEY, parseMapBackup} from "../utils/mapBackup.ts";
import {originalsForObstacles, type ObstacleOriginal} from "../utils/obstacleOriginals.ts";
import {getQuaternionFromHeading, isRingInsidePolygon, itranspose} from "../../../utils/map.tsx";
import {MapGeometryError, assertSupportedMapPolygon, projectMapPosition} from '../utils/mapGeometryContract';
import {baselineRings, prepareSavePolygon, type RosPoint} from '../utils/mapSaveGeometry';

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
    // Outlines obstacles had before the recorded-obstacle shrink (kept in the
    // GUI config store): without them a restored map could not undo a shrink.
    obstacleOriginals: ObstacleOriginal[];
    restoreObstacleOriginals: (next: ObstacleOriginal[]) => Promise<void>;
    // The operator-drawn LiDAR-ignore lines. They live in map_server, not in the
    // Map message, so the backup file carries them separately and a restore
    // writes them back through restoreCorridors (applied immediately, like every
    // other ignore-line change; Cancel reverts them with the edit session).
    corridors: LidarIgnoreCorridor[];
    restoreCorridors: (next: LidarIgnoreCorridor[]) => Promise<void>;
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
    obstacleOriginals,
    restoreObstacleOriginals,
    corridors,
    restoreCorridors,
}: UseMapFilesOptions) {
    const {t} = useTranslation();

    const featureName = (feature: MowingFeatureBase | undefined) =>
        String(feature?.properties?.name || feature?.id || t('mapFiles.unnamedPolygon'));

    // Human name of a polygon for save errors/warnings: the area's name, or an
    // obstacle's 1-based position within its parent area.
    const polygonLabel = (feature: MowingFeatureBase, obstacles: readonly ObstacleFeature[]) => {
        if (feature instanceof ObstacleFeature) {
            const parent: MowingFeatureBase | undefined = feature.getMowingArea();
            const siblings = obstacles.filter(o => o.getMowingArea()?.id === parent?.id);
            return t('mapFiles.polygonObstacle', {index: siblings.indexOf(feature) + 1, area: featureName(parent)});
        }
        const key = feature instanceof NavigationFeature ? 'mapFiles.polygonNavigation' : 'mapFiles.polygonArea';
        return t(key, {name: featureName(feature)});
    };

    const geometryErrorMessage = (error: unknown) => error instanceof MapGeometryError
        ? t(error.reason === 'unsupported' ? 'mapFiles.unsupportedGeometry' : 'mapFiles.invalidGeometry')
        : errorMessage(error);

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

        // Validate every polygon and the edited dock before any ROS call. A
        // draft/history entry may bypass the file decoder; never drop its rings.
        // An invalid ring the operator did not touch (identical to the loaded
        // map) is sent back unchanged with a warning instead of blocking the save.
        const baseline = baselineRings(map);
        const failures: Array<{label: string; error: unknown}> = [];
        const preExistingInvalid: string[] = [];
        const prepare = <F extends MowingFeatureBase>(feature: F): Array<{feature: F; points: RosPoint[]}> => {
            try {
                const {points, isPreExistingInvalid} = prepareSavePolygon(feature.geometry, baseline, offsetX, offsetY, datum);
                if (isPreExistingInvalid) preExistingInvalid.push(polygonLabel(feature, obstacleFeatures));
                return [{feature, points}];
            } catch (error) {
                failures.push({label: polygonLabel(feature, obstacleFeatures), error});
                return [];
            }
        };
        const preparedAreas = areaFeatures.flatMap(prepare);
        const preparedObstacles = obstacleFeatures.flatMap(prepare);
        if (failures.length > 0) {
            notification.error({
                message: t('mapFiles.failedToSaveArea'),
                description: `${geometryErrorMessage(failures[0].error)} ${t('mapFiles.affectedPolygons', {
                    polygons: failures.map(f => f.label).join(', '),
                })}`,
            });
            return;
        }
        try {
            const dock = features.dock;
            if (dockDirty && dock instanceof DockFeatureBase) {
                projectMapPosition(dock.getCoordinates(), offsetX, offsetY, datum);
                if (!Number.isFinite(dock.getHeading())) throw new MapGeometryError('invalid');
            }
        } catch (error) {
            notification.error({message: t('mapFiles.failedToSaveArea'), description: geometryErrorMessage(error)});
            return;
        }

        // Track per-type index counters and map feature ID → index in areas array
        const typeCounters: Record<string, number> = {"area": 0, "navigation": 0};
        const featureIndexMap = new Map<string, {type: string; index: number}>();

        for (const {feature: f, points} of preparedAreas) {
            // Bucket by CLASS, not by id prefix: a workarea↔navigation type
            // change keeps the old id (e.g. "area-1-area-1" is now a
            // NavigationFeature), so id-prefix bucketing silently dropped the
            // converted area from the save payload.
            const type = f instanceof NavigationFeature ? "navigation" : "area";

            const index = typeCounters[type]++;
            featureIndexMap.set(f.id, {type, index});

            areas[type][index] = {
                name: f.properties?.name ?? '',
                area: {points},
                // Round-trip the stable area id (mowglinext#637) the feature
                // was loaded with, so map_server's on_add_area preserves it
                // instead of minting a fresh one. PUT /mowglinext/map clears
                // and re-adds every area on ANY edit, so omitting this would
                // re-index the WHOLE map on every save, discarding all
                // coverage-resume progress even for areas the operator never
                // touched. Undefined for a genuinely new (never-saved) area,
                // which is exactly when a fresh id should be minted.
                id: f instanceof MapAreaFeature ? f.area?.id : undefined,
                // The same goes for the area's own mow angle / perimeter
                // winding (Map page "mowing lines"): map_server rebuilds the
                // list from this payload, so leaving them out would silently
                // reset every area to the robot-wide settings on any map save.
                // Undefined for a never-saved area, which has no overrides yet.
                ...(f instanceof MapAreaFeature && f.area
                    ? {
                        has_mow_angle: f.area.has_mow_angle,
                        mow_angle_deg: f.area.mow_angle_deg,
                        has_ring_direction: f.area.has_ring_direction,
                        ring_direction: f.area.ring_direction,
                        // And where the route starts.
                        has_start_point: f.area.has_start_point,
                        start_x: f.area.start_x,
                        start_y: f.area.start_y,
                    }
                    : {}),
            };
        }

        // Process obstacles and attach them to their parent area. When the
        // recorded parent link is broken (stale reference after edits), try
        // to re-parent by geometry containment before giving up; anything
        // still unmatched is reported to the user instead of vanishing
        // silently from the saved map.
        const droppedObstacles: string[] = [];
        for (const {feature: f, points} of preparedObstacles) {
            const parentArea = f.getMowingArea();
            let parentMapping = parentArea ? featureIndexMap.get(parentArea.id) : undefined;
            if (!parentMapping) {
                const obstacleRing = f.geometry.coordinates[0] ?? [];
                const containing = areaFeatures.find(
                    (a) =>
                        a instanceof MowingAreaFeature &&
                        isRingInsidePolygon(obstacleRing, a.geometry.coordinates[0] ?? [])
                );
                if (containing) parentMapping = featureIndexMap.get(containing.id);
            }
            if (!parentMapping) {
                droppedObstacles.push(String(f.id));
                continue;
            }

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
            notification.error({
                message: t('mapFiles.failedToSaveArea'),
                description: t('mapFiles.obstacleParentMissing', {
                    ids: droppedObstacles.join(", "),
                }),
            });
            return;
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
                            x: quaternionFromHeading.x!,
                            y: quaternionFromHeading.y!,
                            z: quaternionFromHeading.z!,
                            w: quaternionFromHeading.w!,
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
            if (preExistingInvalid.length > 0) {
                notification.warning({
                    message: t('mapFiles.preExistingInvalidGeometryTitle'),
                    description: t('mapFiles.preExistingInvalidGeometry', {polygons: preExistingInvalid.join(', ')}),
                });
            }
            setHasUnsavedChanges(false);
            setEditMap(false);
        } catch (e) {
            // Stay in edit mode so the user's changes are not lost.
            notification.error({
                message: t('mapFiles.failedToSaveArea'),
                description: errorMessage(e),
            });
        }
    }

    const handleBackupMap = () => {
        const a = document.createElement("a");
        document.body.appendChild(a);
        a.style.display = "none";
        const json = JSON.stringify({
                ...map,
                // Only the records that still describe an obstacle on this map.
                [BACKUP_ORIGINALS_KEY]: originalsForObstacles(
                    obstacleOriginals,
                    (map?.working_area ?? []).flatMap((a) => (a.obstacles ?? []).map(
                        (o) => (o.points ?? []).map((p) => ({x: p.x ?? 0, y: p.y ?? 0})))),
                ),
                [BACKUP_CORRIDORS_KEY]: corridors.map(({name, polyline, width_m}) => ({name, polyline, width_m})),
            }),
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
        input.addEventListener('cancel', () => input.remove(), {once: true});
        input.addEventListener('change', asyncEvent(async () => {
            input.remove();
            const file = input.files?.[0];
            if (!file) {
                return;
            }
            // Validate the whole candidate BEFORE touching any editor state
            // (#704). file.text() decodes UTF-8; the old data-URL + atob()
            // path yielded a byte string and corrupted names like "Ängen".
            const fail = (reason: string) => notification.error({
                message: t('mapFiles.restoreFailed'),
                description: reason,
            });
            const parsed = parseMapBackup(await file.text());
            if (!parsed.ok) {
                fail(parsed.reason);
                return;
            }
            let restoredFeatures: Record<string, MowingFeature>;
            try {
                restoredFeatures = buildFeaturesFromMap(parsed.map);
            } catch (e) {
                fail(errorMessage(e));
                return;
            }
            // A backup without dock fields leaves the current dock alone: it
            // is neither rebuilt at (0, 0, 0) nor marked dirty for Save.
            const currentDock = features["dock"];
            const nextFeatures = parsed.hasDock || !currentDock
                ? restoredFeatures
                : {...restoredFeatures, dock: currentDock};
            setEditMap(true);
            setMap(parsed.map);
            // The MapPage effect that turns a Map into editable features
            // skips while editMap is true (set just above), so build them
            // here — otherwise handleSaveMap would persist the stale
            // features and the restored map would be silently discarded.
            setFeatures(nextFeatures);
            setHasUnsavedChanges(true);
            if (parsed.hasDock) {
                setDockDirty(true);
            }
            if (parsed.obstacleOriginals !== null) {
                try {
                    await restoreObstacleOriginals(parsed.obstacleOriginals);
                } catch (e: unknown) {
                    notification.error({
                        message: t('mapFiles.restoreOriginalsFailed'),
                        description: e instanceof Error ? e.message : String(e),
                    });
                }
            }
            // A backup from before the ignore lines were included has no
            // field: leave the current lines alone rather than wiping them.
            if (parsed.corridors !== null) {
                try {
                    await restoreCorridors(parsed.corridors);
                } catch (e: unknown) {
                    notification.error({
                        message: t('mapFiles.restoreCorridorsFailed'),
                        description: e instanceof Error ? e.message : String(e),
                    });
                }
            }
        }, (error) => {
            reportAsyncError(error);
            notification.error({message: t('mapFiles.restoreFailed'), description: errorMessage(error)});
        }), {once: true});
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
        input.addEventListener('cancel', () => input.remove(), {once: true});
        input.addEventListener('change', asyncEvent(async () => {
            input.remove();
            const file = input.files?.[0];
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
                const candidate: unknown = JSON.parse(text);
                // Files are data, not permission to run this route's write path.
                if (isJsonObject(candidate) && Object.keys(candidate).some(key => key.toLowerCase() === 'map')
                    && Object.entries(candidate).some(([key, value]) => key.toLowerCase() === 'apply' && value === true)) {
                    throw new Error('An OpenMower file cannot request apply before confirmation');
                }

                const res = await fetch("/api/import/openmower", {
                    method: "POST",
                    headers: {"Content-Type": "application/json"},
                    body: text,
                });
                if (!res.ok) {
                    const errBody = await res.text();
                    throw new Error(`HTTP ${res.status}: ${errBody}`);
                }
                const summary = decodeImportOpenMowerSummary(await res.json(), false);
                setImportFileText(text);
                setImportPreview(summary);
            } catch (e) {
                notification.error({
                    message: t('mapFiles.openMowerImportFailed'),
                    description: errorMessage(e),
                });
            }
        }, (error) => {
            reportAsyncError(error);
            notification.error({message: t('mapFiles.openMowerImportFailed'), description: errorMessage(error)});
        }), {once: true});
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
        const source: unknown = JSON.parse(importFileText);
        const body: Record<string, unknown> = {map: source};
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
        return decodeImportOpenMowerSummary(await res.json(), false);
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
     *
     * When `importDatum` is true (a fresh install, where the operator
     * confirmed adopting the OpenMower datum), the OM datum is also
     * persisted to mowgli_robot.yaml through the standard settings write
     * path (`POST /api/settings/yaml`) AFTER the map/dock apply succeeds.
     * The importer itself stays side-effect-free on the datum; the datum
     * write is a separate, independently-confirmed step so the two never
     * get entangled (and a datum failure can't roll back the imported map).
     */
    const handleApplyOpenMowerImport = async (
        importFileText: string,
        omDatumLat?: number,
        omDatumLon?: number,
        importDatum?: boolean,
    ): Promise<void> => {
        if (!importFileText) {
            throw new Error("no stashed map text — preview must run before apply");
        }
        const source: unknown = JSON.parse(importFileText);
        const body: Record<string, unknown> = {map: source, apply: true};
        if (omDatumLat !== undefined && omDatumLon !== undefined) {
            body.om_datum_lat = omDatumLat;
            body.om_datum_lon = omDatumLon;
        }
        let res: Response;
        try {
            res = await fetch("/api/import/openmower", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify(body),
            });
        } catch (error) {
            throw new Error(`${t('mapImportOpenMower.outcomeUnknown')} — ${errorMessage(error)}`);
        }
        if (!res.ok) {
            let raw: unknown;
            try {raw = await res.json();} catch { /* An unreadable response cannot establish an outcome. */ }
            let outcome: ImportOutcome | undefined;
            try {outcome = decodeImportOutcome(raw);} catch { /* Invalid structured metadata is unconfirmed. */ }
            let key = 'outcomeUnknown';
            if (outcome) {
                key = outcome.map_status === 'persisted'
                    ? outcome.dock_status === 'rejected' ? 'dockRejected'
                        : outcome.dock_status === 'not_attempted' ? 'dockNotAttempted'
                        : outcome.dock_status === 'active_persistence_unconfirmed' ? 'dockPersistenceUnconfirmed' : 'dockUnconfirmed'
                    : outcome.map_status === 'unchanged' ? 'notStarted'
                        : outcome.map_status === 'restored' ? 'mapRestored'
                            : outcome.map_status === 'active_persistence_unconfirmed' ? 'mapPersistenceUnconfirmed' : 'outcomeUnknown';
            } else if (isJsonObject(raw) && !('map_status' in raw) && !('dock_status' in raw)
                && typeof raw.error === 'string' && raw.error.startsWith('set docking point:')) {
                // Legacy servers prove only that map replacement finished. The
                // old text does not distinguish a dock rejection from lost transport.
                key = 'dockUnconfirmed';
            }
            const detail = isJsonObject(raw) && typeof raw.error === 'string' ? ` — ${raw.error}` : '';
            throw new Error(`${t(`mapImportOpenMower.${key}`)} (HTTP ${res.status})${detail}`);
        }

        // The server may already have written the map. An invalid acknowledgment
        // is unconfirmed success, never evidence of rollback; do not auto-retry.
        let appliedSummary: ImportOpenMowerSummary;
        try {
            appliedSummary = decodeImportOpenMowerSummary(await res.json(), true);
        } catch (error) {
            throw new Error(`${t('mapImportOpenMower.outcomeUnknown')} — ${errorMessage(error)}`);
        }

        // Older servers acknowledge apply but do not expose dock-persistence
        // warnings. Keep that acknowledgement distinct from full confirmation.
        if (appliedSummary.map_status === undefined) {
            throw new Error(t('mapImportOpenMower.legacyApplied'));
        }

        // Persist the datum only after the map/dock apply succeeded, and only
        // when the operator opted in. Uses the settings write path so the
        // sparse-config + fixed-precision handling stays in one place.
        if (importDatum && omDatumLat !== undefined && omDatumLon !== undefined) {
            const datumRes = await fetch("/api/settings/yaml", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({datum_lat: omDatumLat, datum_lon: omDatumLon}),
            });
            if (!datumRes.ok) {
                const errBody = await datumRes.text();
                // The map imported fine; surface the datum failure without
                // pretending the whole import failed.
                throw new Error(`map imported, but writing the datum failed — HTTP ${datumRes.status}: ${errBody}`);
            }
        }

        // Drop any in-progress edit so the freshly-imported map shows clean.
        setEditMap(false);
        setHasUnsavedChanges(false);
        setDockDirty(false);
        notification.success({
            message: t('mapFiles.openMowerMapImported'),
            description: t(appliedSummary.dock_status === 'not_requested' || !appliedSummary.dock_pose
                ? 'mapFiles.openMowerMapImportedWithoutDock' : 'mapFiles.openMowerMapImportedDescription'),
        });
    };

    const handleUploadGeoJSON = () => {
        const input = document.createElement("input");
        input.type = "file";
        input.style.display = "none";
        document.body.appendChild(input);
        input.addEventListener('cancel', () => input.remove(), {once: true});
        input.addEventListener('change', asyncEvent(async () => {
            input.remove();
            const file = input.files?.[0];
            if (!file) return;
            try {
                let raw: unknown;
                try {raw = JSON.parse(await file.text());}
                catch (error) {
                    notification.error({message: t('mapFiles.uploadParseError'), description: errorMessage(error)});
                    return;
                }
                const snapshot = decodeEditableGeoJSON(raw, {
                    invalid: t('mapFiles.uploadParseError'), empty: t('mapFiles.uploadEmpty'),
                    unknownType: type => t('mapFiles.unknownType', {type}),
                });
                // Rehydrate everything before committing any editor state.
                const next = featuresFromJSON(snapshot);
                if (Object.keys(next).length !== Object.keys(snapshot).length
                    || Object.entries(next).some(([id, feature]) => feature.id !== id
                        || feature instanceof ObstacleFeature && !(feature.getMowingArea() instanceof MowingAreaFeature))) {
                    throw new Error(t('mapFiles.uploadParseError'));
                }
                setFeatures(next);
                setHasUnsavedChanges(true);
                notification.success({message: t('mapFiles.uploadSuccess')});
            } catch (error) {
                notification.error({message: geometryErrorMessage(error)});
            }
        }, error => {
            reportAsyncError(error);
            notification.error({message: t('mapFiles.uploadParseError'), description: errorMessage(error)});
        }), {once: true});
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
    map_status?: ImportOutcome['map_status'];
    dock_status?: ImportOutcome['dock_status'];
    mowing_areas: number;
    navigation_areas: number;
    obstacles: number;
    orphan_obstacles: number;
    dock_pose?: {x: number; y: number; yaw_rad: number} | null;
    datum_shift_east_m: number;
    datum_shift_north_m: number;
    /**
     * True when mowgli_robot.yaml already has a datum_lat/datum_lon.
     * Drives whether the modal offers "import the datum too": only a
     * fresh install (false) can safely adopt the OpenMower datum.
     */
    mn_datum_configured?: boolean;
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

export interface ImportOutcome {
    map_status: 'unchanged' | 'restored' | 'unconfirmed' | 'active_persistence_unconfirmed' | 'persisted';
    dock_status: 'not_requested' | 'not_attempted' | 'rejected' | 'unconfirmed' | 'active_persistence_unconfirmed' | 'confirmed';
}

/** Missing metadata is legacy; partial, unknown or contradictory metadata is invalid. */
export function decodeImportOutcome(raw: unknown): ImportOutcome | undefined {
    if (!isJsonObject(raw) || !('map_status' in raw) && !('dock_status' in raw)) return undefined;
    const map = raw.map_status, dock = raw.dock_status;
    if (!(map === 'unchanged' || map === 'restored' || map === 'unconfirmed'
        || map === 'active_persistence_unconfirmed' || map === 'persisted')
        || !(dock === 'not_requested' || dock === 'not_attempted' || dock === 'rejected'
            || dock === 'unconfirmed' || dock === 'active_persistence_unconfirmed' || dock === 'confirmed')
        || map !== 'persisted' && dock !== 'not_requested' && dock !== 'not_attempted') {
        throw new Error('Invalid OpenMower import outcome');
    }
    return {map_status: map, dock_status: dock};
}


const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value);
const count = (value: unknown): boolean => finite(value) && Number.isInteger(value) && value >= 0;
function importArea(raw: unknown): raw is ImportOpenMowerSummary['areas'][number] {
    return isJsonObject(raw) && typeof raw.name === 'string' && typeof raw.type === 'string'
        && count(raw.vertices) && count(raw.obstacles) && typeof raw.is_navigation_area === 'boolean' && finite(raw.approx_area_sqm);
}
function importSummary(raw: unknown): raw is ImportOpenMowerSummary {
    return isJsonObject(raw) && count(raw.mowing_areas) && count(raw.navigation_areas)
        && count(raw.obstacles) && count(raw.orphan_obstacles) && finite(raw.datum_shift_east_m) && finite(raw.datum_shift_north_m)
        && typeof raw.applied === 'boolean'
        && (!('mn_datum_configured' in raw) || typeof raw.mn_datum_configured === 'boolean')
        && (!('dock_pose' in raw) || raw.dock_pose === null || isJsonObject(raw.dock_pose)
            && finite(raw.dock_pose.x) && finite(raw.dock_pose.y) && finite(raw.dock_pose.yaw_rad))
        && Array.isArray(raw.warnings) && raw.warnings.every((warning: unknown) => typeof warning === 'string')
        && Array.isArray(raw.areas) && raw.areas.every(importArea);
}
export function decodeImportOpenMowerSummary(raw: unknown, applied: boolean): ImportOpenMowerSummary {
    if (!importSummary(raw) || raw.applied !== applied) throw new Error('Invalid OpenMower import summary');
    const outcome = decodeImportOutcome(raw);
    if (outcome && (applied
        ? outcome.map_status !== 'persisted' || outcome.dock_status !== 'confirmed' && outcome.dock_status !== 'not_requested'
        : outcome.map_status !== 'unchanged' || outcome.dock_status !== 'not_attempted' && outcome.dock_status !== 'not_requested')) {
        throw new Error('Contradictory OpenMower import summary');
    }
    if (outcome && (outcome.dock_status === 'not_requested' ? raw.dock_pose != null : raw.dock_pose == null)) {
        throw new Error('Contradictory OpenMower dock summary');
    }
    return raw;
}

const position = (raw: unknown): raw is Position => Array.isArray(raw) && raw.length >= 2 && raw.every(finite);
const bbox = (raw: unknown): boolean => raw === undefined || Array.isArray(raw) && (raw.length === 4 || raw.length === 6) && raw.every(finite);
function editableGeometry(raw: unknown): raw is Point | Polygon {
    if (!isJsonObject(raw) || !bbox(raw.bbox)) return false;
    if (raw.type === 'Point') return position(raw.coordinates);
    if (raw.type !== 'Polygon' && raw.type !== 'MultiPolygon') return false;
    assertSupportedMapPolygon(raw);
    return true;
}

export function decodeEditableGeoJSON(raw: unknown, messages: {invalid: string; empty: string; unknownType: (type: string) => string}): Record<string, SerializedMapFeature> {
    if (!isJsonObject(raw) || raw.type !== 'FeatureCollection' || !Array.isArray(raw.features)) throw new Error(messages.invalid);
    const entries = new Map<string, SerializedMapFeature>();
    for (const value of raw.features) {
        const element: unknown = value;
        if (!isJsonObject(element) || element.type !== 'Feature'
            || !(typeof element.id === 'string' || finite(element.id)) || !editableGeometry(element.geometry)) throw new Error(messages.invalid);
        const props = element.properties;
        const featureType = isJsonObject(props) ? props.feature_type : undefined;
        if (typeof featureType !== 'string' || !['workarea', 'navigation', 'obstacle', 'dock'].includes(featureType)) {
            throw new Error(messages.unknownType(typeof featureType === 'string' ? featureType : '?'));
        }
        if (!isJsonObject(props)) throw new Error(messages.invalid);
        const id = String(element.id);
        // The factory uses object dictionaries: duplicate IDs or a prototype
        // setter must never cause a feature to disappear during rehydration.
        if (id === '__proto__' || entries.has(id) || (featureType === 'dock' ? element.geometry.type !== 'Point' : element.geometry.type !== 'Polygon')) throw new Error(messages.invalid);
        if (featureType === 'dock') {
            // Null heading already means unavailable in the feature factory.
            if (props.heading != null && !finite(props.heading)) throw new Error(messages.invalid);
        } else {
            // Present overrides must match the class contract: spreading null
            // over its defaults would later break labels/order handling.
            for (const key of ['index', 'mowing_order', 'source_working_area_index', 'source_working_area_id']) {
                if (key in props && !finite(props[key])) throw new Error(messages.invalid);
            }
            for (const key of ['name', 'color']) {
                if (key in props && typeof props[key] !== 'string') throw new Error(messages.invalid);
            }
        }
        const parent = featureType === 'obstacle' ? props.mowing_area : undefined;
        if (parent != null && typeof parent !== 'string' && !finite(parent)) throw new Error(messages.invalid);
        entries.set(id, {id, type: 'Feature', geometry: element.geometry, properties: props,
            ...(parent == null ? {} : {parent_id: String(parent)})});
    }
    if (entries.size === 0) throw new Error(messages.empty);
    // Only actual workarea keys can be parents. Clear unresolved references so
    // the existing factory falls back to an available area, never Object.prototype.
    const areaIds = new Set([...entries].filter(([, feature]) => feature.properties.feature_type === 'workarea').map(([id]) => id));
    for (const feature of entries.values()) {
        if (feature.properties.feature_type === 'obstacle' && feature.parent_id !== undefined && !areaIds.has(feature.parent_id)) {
            delete feature.parent_id;
        }
    }
    return Object.fromEntries(entries);
}
