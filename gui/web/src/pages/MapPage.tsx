import {StyledDockMarker} from "./map/components/StyledDockMarker";
import {AssembledMowerMarker} from "./map/components/AssembledMowerMarker";
import {formatArea} from "../utils/areaLabel.ts";
import {mowingAreaIndexById} from "../utils/mapAreaIndex.ts";
import "@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css";
import {useApi} from "../hooks/useApi.ts";
import {App, Button} from "antd";
import turfArea from "@turf/area";
import React, {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {useTranslation} from "react-i18next";
import {MapArea, Map as MapType} from "../types/ros.ts";
import DrawControl from "../components/DrawControl.tsx";
import Map, {Layer, Source} from 'react-map-gl/mapbox';
import type {Map as MapboxMap} from 'mapbox-gl';
import type {Feature, LineString, Polygon} from 'geojson';
import {FeatureCollection, Position} from "geojson";
import {useMowerAction} from "../components/MowerActions.tsx";
import {MapStyle} from "./MapStyle.tsx";
import {drawLine, isRingInsidePolygon, itranspose, transpose} from "../utils/map.tsx";
import {getDockAppearanceResetForMowerChange, resolveDockAppearance, resolveMowerAppearance, shouldDisplayMapImage, shouldDisplayMowerImage, type DockAppearanceId, type MowerAppearanceId} from "../constants/mowerAppearances.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {useConfig} from "../hooks/useConfig.tsx";
import {useEnv} from "../hooks/useEnv.tsx";
import {Spinner} from "../components/Spinner.tsx";
import {MowingFeature, MowingAreaFeature, DockFeatureBase, LineFeatureBase, MowingFeatureBase, MowerFeatureBase, NavigationFeature, ObstacleFeature, ActivePathFeature, PathFeature, closeRing} from "../types/map.ts";
import {useMapEditHistory} from "./map/hooks/useMapEditHistory.ts";
import {useMapOffset} from "./map/hooks/useMapOffset.ts";
import {useMapBearing} from "./map/hooks/useMapBearing.ts";
import {useMapBearingCamera} from "./map/hooks/useMapBearingCamera.ts";
import {useManualMode} from "./map/hooks/useManualMode.ts";
import {useMapEditing, type ShrinkMemory} from "./map/hooks/useMapEditing.ts";
import {useObstacleOriginals} from "./map/hooks/useObstacleOriginals.ts";
import {findOriginal, type XY as OutlineXY} from "./map/utils/obstacleOriginals.ts";
import {useMapStreams} from "./map/hooks/useMapStreams.ts";
import {useMapFiles, type ImportOpenMowerSummary} from "./map/hooks/useMapFiles.ts";
import {useResetMowingProgress} from "./map/hooks/useResetMowingProgress.tsx";
import {ImportOpenMowerModal} from "./map/components/ImportOpenMowerModal.tsx";
import {NewAreaModal} from "./map/components/NewAreaModal.tsx";
import {EditAreaModal} from "./map/components/EditAreaModal.tsx";
import {AreasListPanel} from "./map/components/AreasListPanel.tsx";
import {TrackedObstaclesPanel} from "./map/components/TrackedObstaclesPanel.tsx";
import {ObstacleProposalsPanel} from "./map/components/ObstacleProposalsPanel.tsx";
import {CORRIDOR_COLOR, LidarCorridorsInfo, LidarCorridorsPanel} from "./map/components/LidarCorridorsPanel.tsx";
import {MapSidebarAccordion} from "./map/components/MapSidebarAccordion.tsx";
import {EditLidarCorridorModal} from "./map/components/EditLidarCorridorModal.tsx";
import {DEFAULT_CORRIDOR_WIDTH_M, useLidarCorridors} from "./map/hooks/useLidarCorridors.ts";
import {buildCorridorSideRuns, dropLiveVertex, simplifyPolyline, smoothPolyline, type XY} from "./map/utils/corridorGeometry.ts";
import {useObstacleClearancePreview} from "./map/hooks/useObstacleClearancePreview.ts";
import {useCoveragePreview} from "./map/hooks/useCoveragePreview.ts";
import {useCoverageResumeAvailable} from "../hooks/useCoverageResumeAvailable.ts";
import {CoveragePreviewPanel} from "./map/components/CoveragePreviewPanel.tsx";
import {CoverageStartMarker} from "./map/components/CoverageStartMarker.tsx";
import type {AreaOverrideFields} from "./map/coveragePreview.ts";
import {calculateMapViewportBounds} from "./map/utils/mapViewport.ts";

// Distinct from the red drawn-obstacle fill, so the toggleable
// clearance-preview outline is never mistaken for it.
const OBSTACLE_CLEARANCE_PREVIEW_COLOR = '#faad14';
// Mowing-lines overlay: perimeter rounds and swaths need to stay distinct from
// each other and from the amber clearance outline, on satellite imagery.
const COVERAGE_RING_COLOR = '#00d8ff';
const COVERAGE_SWATH_COLOR = '#ffe14a';
import {extractObstacleProposals, isDigProposal} from "./map/utils/obstacleProposals.ts";
import {MapOffsetPanel, MapRotationPanel} from "./map/components/MapOffsetPanel.tsx";
import {MapImageMarker} from "./map/components/MapImageMarker.tsx";
import {getMowerHeadingRad, hasValidMapPosition} from "./map/components/mapImageMarkerMath.ts";
import {buildMapDisplayFeatures} from "./map/mapDisplayFeatures.ts";
import {MapToolbar} from "./map/components/MapToolbar.tsx";
import {MapToolbarMobile} from "./map/components/MapToolbarMobile.tsx";
import {MapEditorToolbar} from "./map/components/MapEditorToolbar.tsx";
import {RestoreMapBackupModal} from "./map/components/RestoreMapBackupModal.tsx";
import {useMapBackups} from "./map/hooks/useMapBackups.ts";
import {JoystickOverlay} from "./map/components/JoystickOverlay.tsx";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useFleetPeers} from "../hooks/useFleetPeers.ts";
import {useRobotDescription} from "../hooks/useRobotDescription.ts";
import {FLEET_PEER_COLOR, peerDisplayFeatures, peerMapPoses} from "../utils/fleetPeers.ts";


// Mapbox access token comes from the build env only — no hardcoded fallback.
// When it is missing the page renders a clear error panel instead of a broken
// (blank) map, so the misconfiguration is obvious rather than silent.
const MAPBOX_TOKEN = import.meta.env.VITE_MAPBOX_TOKEN as string | undefined || "pk.eyJ1IjoiY2VkYm9zc25lbyIsImEiOiJjbGxldjB4aDEwOW5vM3BxamkxeWRwb2VoIn0.WOccbQZZyO1qfAgNxnHAnA";

// Mapbox GL stops zooming at 22 unless told otherwise; 24 is its hard ceiling.
// The extra two levels (4x closer) let the operator place and edit very small
// hand-drawn obstacles. The vector overlays stay sharp; the satellite imagery
// is simply overscaled past its native resolution.
const MAP_MAX_ZOOM = 24;

// Stable ordering is required because Mapbox mounts image markers as their
// selected assets change: dock base < mower < dock tongue foreground.
const MAP_IMAGE_LAYER_Z_INDEX = {dockBase: 998, mower: 999, dockForeground: 1000} as const;

// Layers the full map queries on hover to drive the two-way obstacle
// highlight (map polygon → panel row). Module-level so the array identity is
// stable across renders and react-map-gl does not re-bind the query on every
// render.
const DYN_OBSTACLE_INTERACTIVE_LAYERS = ['dyn-obstacle-fill'];

export const MapPage: React.FC<{compact?: boolean}> = ({compact = false}) => {
    const {notification} = App.useApp();
    const {t, i18n} = useTranslation();
    const {colors, displayMode} = useThemeMode();
    const isMobile = useIsMobile();
    const mowerAction = useMowerAction()
    const resetMowingProgress = useResetMowingProgress()

    // Brand tokens for the Mapbox display-only layers (dock, mower, lidar).
    // Shared between the compact and full render branches so the two stay in
    // lockstep — never re-hardcode these hex values inline below.
    const LAYER_COLORS = useMemo(() => ({
        dock: colors.auroraViolet,           // dock marker + label
        dockHeading: colors.primary,         // dock heading line (lime)
        mower: colors.info,                  // mower center point (aurora-cyan)
        mowerOutline: colors.emeraldDeep,    // mower footprint outline (deep accent)
        lidarHit: colors.danger,             // lidar hit points (rose)
        lidarMiss: colors.amber,             // lidar miss points (amber)
        coveragePath: colors.mint,           // full F2C coverage plan line (mint)
        halo: colors.text,                   // white halo → ink
        labelText: colors.text,              // symbol label text → ink
        labelHalo: colors.bgBase,            // symbol label halo → deep bg
    }), [colors]);

    const {settings} = useSettings()
    const [labelsCollection, setLabelsCollection] = useState<FeatureCollection>({
        type: "FeatureCollection",
        features: []
    })
    const {config, setConfig} = useConfig(["gui.map.offset.x", "gui.map.offset.y", "gui.map.display.bearing", "gui.map.mower.appearance", "gui.map.dock.appearance"])
    const envs = useEnv()
    const guiApi = useApi()
    const obstacleOriginals = useObstacleOriginals();
    const [tileUri, setTileUri] = useState<string | undefined>()
    const [editMap, setEditMap] = useState<boolean>(false)
    // Shared hover/selection link between the tracked-obstacles panel and the
    // map. Hovering a panel row (or a map polygon) sets this id; both the
    // Mapbox highlight layer (via its filter) and the panel read it, so the
    // operator sees exactly which obstacle they're about to promote. null =
    // nothing highlighted.
    const [selectedObstacleId, setSelectedObstacleId] = useState<number | null>(null);
    // Same link for PENDING obstacle proposals (wheel-slip dig reports).
    const [selectedProposalId, setSelectedProposalId] = useState<number | null>(null);
    const [features, setFeatures] = useState<Record<string, MowingFeature>>({});
    const mowerAppearance = resolveMowerAppearance(config["gui.map.mower.appearance"]);
    const dockAppearance = resolveDockAppearance(config["gui.map.dock.appearance"], mowerAppearance.id);
    const [loadedMowerImageSrc, setLoadedMowerImageSrc] = useState<string>();
    const [loadedDockImageSrc, setLoadedDockImageSrc] = useState<string>();
    const mowerImage = mowerAppearance.mowerImage;
    const dockImage = dockAppearance.image;
    const mowerFeature = features.mower;
    const dockFeature = features.dock;
    const mowerHasValidPose = mowerFeature instanceof MowerFeatureBase && hasValidMapPosition(mowerFeature.geometry.coordinates);
    const dockHasValidPose = dockFeature instanceof DockFeatureBase && hasValidMapPosition(dockFeature.getCoordinates());
    const dockHeadingRad = dockFeature instanceof DockFeatureBase && dockFeature.hasValidHeading() && Number.isFinite(dockFeature.getHeading())
        ? dockFeature.getHeading()
        : undefined;
    const mowerHeadingFeature = features["mower-heading"];
    const handleMowerAppearanceChange = (id: MowerAppearanceId) => {
        setLoadedMowerImageSrc(undefined);
        const dockAppearanceReset = getDockAppearanceResetForMowerChange(dockAppearance, id);
        if (dockAppearanceReset) setLoadedDockImageSrc(undefined);
        void setConfig({
            "gui.map.mower.appearance": id,
            ...(dockAppearanceReset ? {"gui.map.dock.appearance": dockAppearanceReset} : {}),
        });
    };
    const handleDockAppearanceChange = (id: DockAppearanceId) => {
        setLoadedDockImageSrc(undefined);
        void setConfig({"gui.map.dock.appearance": id});
    };
    const [dockPlacementMode, setDockPlacementMode] = useState<boolean>(false);
    // LiDAR-ignore line drawing. Field-reported 2026-09-29: a hand-rolled
    // point collector (a growing [lng,lat][] fed by our own <Map onClick>)
    // never received taps on mobile at all — DrawControl's mapbox-gl-draw
    // instance sits on top of the same map, permanently in some draw mode
    // (simple_select when idle), and its own touch handling appears to
    // consume single taps before they'd reach a sibling click listener; the
    // symptom was identical on desktop-vs-mobile clicks vs taps despite
    // running the exact same handler. Polygon/shape drawing never had this
    // problem because it already goes through gl-draw's OWN draw_polygon
    // mode (handleDrawPolygon, useMapEditing.ts) instead of a parallel
    // custom collector — so corridor lines now do the same: drawRef.current
    // .changeMode('draw_line_string') for a corridor, reusing gl-draw's
    // proven-on-touch point placement instead of fighting it. This flag is
    // just "is that mode currently active" — the points themselves live in
    // gl-draw's OWN store (read via drawRef.current.getAll() below), not
    // here.
    const [corridorDrawing, setCorridorDrawing] = useState(false);
    // Mobile: index into lidarCorridors.corridors of the line whose width the
    // operator is editing (there is no side panel to hold the width field there).
    const [corridorWidthModalIndex, setCorridorWidthModalIndex] = useState<number | null>(null);
    const lidarCorridors = useLidarCorridors();
    const mapBackups = useMapBackups();
    const [restoreBackupOpen, setRestoreBackupOpen] = useState(false);
    // Snapshot of the corridor list taken the moment map-edit mode is
    // entered. Unlike areas/obstacles (which live only in local `features`
    // state until "Save Map"), corridor edits are written to map_server
    // immediately (useLidarCorridors's own design) — so "Cancel" cannot rely
    // on simply never having persisted anything and must explicitly put this
    // back. Read only from the onDiscard callback below; re-captured every
    // time editMap flips false -> true (a later Save Map updates what the
    // "current" corridor state is, so the next edit session's snapshot is
    // taken fresh).
    const corridorEditSnapshotRef = useRef<typeof lidarCorridors.corridors | null>(null);
    // True while the corridor list has actually diverged from the snapshot
    // taken at edit-session start — a corridor-only edit (no area/obstacle
    // touched) would otherwise leave useMapEditHistory's own
    // hasUnsavedChanges false and let Cancel silently skip both the confirm
    // dialog and the revert.
    const corridorsDirtySinceEdit = editMap && corridorEditSnapshotRef.current !== null &&
        JSON.stringify(corridorEditSnapshotRef.current) !== JSON.stringify(lidarCorridors.corridors);

    // Put the corridor list back the way it was when this edit session
    // started. Called from useMapEditHistory's onDiscard — i.e. on Cancel,
    // never on a successful Save Map.
    const revertCorridorsOnDiscard = useCallback(async () => {
        const snapshot = corridorEditSnapshotRef.current;
        if (!snapshot) return;
        try {
            await lidarCorridors.save(snapshot);
        } catch (error: unknown) {
            notification.error({
                message: t('mapLidarCorridors.saveFailed'),
                description: error instanceof Error ? error.message : undefined,
            });
        }
    }, [lidarCorridors, notification, t]);
    // OpenMower import preview — populated by handleImportOpenMower after
    // the file is uploaded + parsed server-side. Modal renders when set.
    const [importPreview, setImportPreview] = useState<ImportOpenMowerSummary | null>(null);
    // Verbatim text of the imported map.json. Stashed at preview time so
    // the apply step can re-POST byte-identical content (the server then
    // runs the same parse + validate flow before writing).
    const [importFileText, setImportFileText] = useState<string | null>(null);
    // Track whether the user actually moved the dock during this edit
    // session. The dock feature is rebuilt from the live /map topic on
    // every render, and saving without this flag would clobber the
    // persisted dock pose with a stale value (e.g. when dock_pose was
    // updated by the calibration service but the /map topic hasn't
    // re-emitted yet).
    const [dockDirty, setDockDirty] = useState<boolean>(false);
    const [mapKey, setMapKey] = useState<string>("origin")
    const [useSatellite, setUseSatellite] = useState(true)
    const [pitched, setPitched] = useState(false)
    const togglePitch = useCallback(() => {
        setPitched(prev => {
            const next = !prev;
            const m = mapInstanceRef.current;
            if (m) m.easeTo({pitch: next ? 50 : 0, duration: 600});
            return next;
        });
    }, [])
    const robotPoseRef = useRef<{ x: number; y: number; heading: number } | null>(null)
    const mapInstanceRef = useRef<MapboxMap | null>(null)
    const drawRef = useRef<import('@mapbox/mapbox-gl-draw').default | null>(null);

    // Only include editable polygon features for DrawControl — exclude mower,
    // paths, and other display-only features so that frequent pose updates don't
    // trigger DrawControl to deleteAll() + re-add, which wipes out selection state.
    // Stable ref ensures the array identity only changes when mowing areas actually
    // change, not on every pose update, preventing DrawControl sync timer thrashing.
    const prevMowingRef = useRef<GeoJSON.Feature[]>([]);
    const drawableFeatures = useMemo(() => {
        const next = Object.values(features).filter(f => f instanceof MowingFeatureBase) as GeoJSON.Feature[];
        const prev = prevMowingRef.current;
        const unchanged =
            next.length === prev.length &&
            next.every((f, i) => f.id === prev[i]?.id && JSON.stringify(f.geometry) === JSON.stringify(prev[i]?.geometry));
        if (unchanged) return prev;
        prevMowingRef.current = next;
        return next;
    }, [features]);

    // Extracted hooks
    const {offsetX, offsetY, handleOffsetX, handleOffsetY} = useMapOffset({config, setConfig, notification});
    const {bearing, handleBearing} = useMapBearing({config, setConfig, notification});

    const onMapLoad = useMapBearingCamera({mapInstanceRef, bearing, onBearingChange: handleBearing, interactive: !compact});

    const _datumLon = parseFloat(settings["datum_lon"] ?? 0)
    const _datumLat = parseFloat(settings["datum_lat"] ?? 0)

    // Datum as [lat, lon, 0] for equirectangular projection
    // (matches the navsat_to_absolute_pose ROS node projection)
    const datum = useMemo<[number, number, number]>(() => {
        if (_datumLon == 0 || _datumLat == 0) {
            return [0, 0, 0]
        }
        return [_datumLat, _datumLon, 0]
    }, [_datumLat, _datumLon])

    const mowerHeadingRad = mowerHeadingFeature instanceof LineFeatureBase
        ? getMowerHeadingRad(mowerHeadingFeature.geometry.coordinates, (lng, lat) =>
            itranspose(offsetX, offsetY, datum, lat, lng))
        : undefined;
    const robotGeometry = useRobotDescription();
    const assembledMowerReady = mowerAppearance.id === "urdf" && !!robotGeometry.fromUrdf && mowerHasValidPose && mowerHeadingRad !== undefined;
    const mowerImageReady = shouldDisplayMowerImage(
        mowerAppearance,
        loadedMowerImageSrc,
        mowerHasValidPose,
        mowerHeadingRad !== undefined,
    );
    const styledDockReady = dockAppearance.id === "styled" && dockHasValidPose && dockHeadingRad !== undefined;
    const dockImageReady = styledDockReady || shouldDisplayMapImage(
        dockImage,
        loadedDockImageSrc,
        dockHasValidPose,
        dockHeadingRad !== undefined,
    );

    // Display-only features (mower, dock, heading, paths) rendered as separate layers
    const displayFeatures = useMemo(() => buildMapDisplayFeatures(
        features,
        mowerImageReady || assembledMowerReady,
        dockImageReady,
        (dock) => {
            const coords = dock.getCoordinates();
            const rosCoords = datum[0] !== 0 ? itranspose(offsetX, offsetY, datum, coords[1], coords[0]) : [0, 0];
            const endPoint = drawLine(offsetX, offsetY, datum, rosCoords[1], rosCoords[0], dock.getHeading());
            return {type: "LineString", coordinates: [coords, endPoint]};
        },
        LAYER_COLORS.dockHeading,
    ), [features, offsetX, offsetY, datum, LAYER_COLORS, mowerImageReady, assembledMowerReady, dockImageReady]);

    // The other robots of the fleet (docs/MULTI_ROBOT.md), drawn with this
    // robot's silhouette in their own colour and named.
    const fleetRobots = useFleetPeers();
    const mapDisplayData = useMemo(() => {
        const peers = peerDisplayFeatures(peerMapPoses(fleetRobots, datum), offsetX, offsetY, datum, robotGeometry);
        return peers.length ? {...displayFeatures, features: [...displayFeatures.features, ...peers]} : displayFeatures;
    }, [displayFeatures, fleetRobots, datum, offsetX, offsetY, robotGeometry]);

    // Layers for the persistent tracked-obstacle polygons (feature_type
    // 'dyn-obstacle', carried in the same display-features source). Rendered as
    // a translucent fill + rose outline + an id label, so the map and the
    // TrackedObstaclesPanel share one visible identifier. `withHighlight` adds
    // an amber outline on the currently selected/hovered obstacle — its filter
    // reads selectedObstacleId, so a hover only re-binds this one layer's
    // filter (no feature rebuild). Only the full map passes withHighlight; the
    // compact overview just shows the fill/outline/label.
    const renderDynObstacleLayers = (withHighlight: boolean) => (
        [
            <Layer key={"dyn-obstacle-fill"} type={"fill"} id={"dyn-obstacle-fill"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                paint={{'fill-color': ['get', 'color']}}/>,
            <Layer key={"dyn-obstacle-outline"} type={"line"} id={"dyn-obstacle-outline"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                paint={{'line-color': LAYER_COLORS.lidarHit, 'line-width': 2}}/>,
            ...(withHighlight ? [
                <Layer key={"dyn-obstacle-highlight"} type={"line"} id={"dyn-obstacle-highlight"}
                    filter={['all',
                        ['==', ['get', 'feature_type'], 'dyn-obstacle'],
                        ['==', ['get', 'obs_id'], selectedObstacleId ?? -1]]}
                    paint={{'line-color': LAYER_COLORS.lidarMiss, 'line-width': 4}}/>,
            ] : []),
            <Layer key={"dyn-obstacle-label"} type={"symbol"} id={"dyn-obstacle-label"}
                filter={['==', ['get', 'feature_type'], 'dyn-obstacle']}
                layout={{
                    'text-field': ['concat', '#', ['get', 'obs_label']],
                    'text-size': 13,
                    'text-font': ['Open Sans Bold'],
                    'text-allow-overlap': true,
                }}
                paint={{
                    'text-color': LAYER_COLORS.labelText,
                    'text-halo-color': LAYER_COLORS.labelHalo,
                    'text-halo-width': 1.5,
                }}/>,
        ]
    );

    const [mowingAreas, setMowingAreas] = useState<{ key: string, label: string, feat: Feature }[]>([])

    const {map, setMap, path, plan, lidarCollection, mowProgressImage, lidarMapImage, highLevelStatus, joyStream, dynamicObstacles} = useMapStreams({
        editMap,
        settings,
        offsetX,
        offsetY,
        datum,
        setFeatures,
        setEditMap,
        setMapKey,
        mapInstanceRef,
        robotPoseRef,
    });

    const mowerImageMarker = mowerImage && mowerFeature instanceof MowerFeatureBase && mowerHasValidPose && mowerHeadingRad !== undefined
        ? <MapImageMarker
            image={mowerImage}
            alt={t(mowerImage.altKey)}
            longitude={mowerFeature.geometry.coordinates[0]}
            latitude={mowerFeature.geometry.coordinates[1]}
            headingRad={mowerHeadingRad}
            zIndex={MAP_IMAGE_LAYER_Z_INDEX.mower}
            onLoad={() => setLoadedMowerImageSrc(mowerImage.src)}
            onError={() => setLoadedMowerImageSrc(undefined)}
        />
        : assembledMowerReady && mowerFeature instanceof MowerFeatureBase
            ? <AssembledMowerMarker robot={robotGeometry} longitude={mowerFeature.geometry.coordinates[0]}
                latitude={mowerFeature.geometry.coordinates[1]} headingRad={mowerHeadingRad}/>
            : null;
    const dockImageMarker = styledDockReady && dockFeature instanceof DockFeatureBase
        ? <StyledDockMarker longitude={dockFeature.geometry.coordinates[0]} latitude={dockFeature.geometry.coordinates[1]} headingRad={dockHeadingRad}/>
        : dockImage && dockFeature instanceof DockFeatureBase && dockHasValidPose && dockHeadingRad !== undefined
        ? <MapImageMarker
            image={dockImage}
            alt={t(dockImage.altKey)}
            longitude={dockFeature.geometry.coordinates[0]}
            latitude={dockFeature.geometry.coordinates[1]}
            headingRad={dockHeadingRad}
            zIndex={MAP_IMAGE_LAYER_Z_INDEX.dockBase}
            onLoad={() => setLoadedDockImageSrc(dockImage.src)}
            onError={() => setLoadedDockImageSrc(undefined)}
        />
        : null;
    const dockForegroundMarker = dockImage && dockAppearance.foregroundClipPath &&
        dockFeature instanceof DockFeatureBase && dockHasValidPose && dockHeadingRad !== undefined
        ? <MapImageMarker
            image={dockImage}
            alt=""
            longitude={dockFeature.geometry.coordinates[0]}
            latitude={dockFeature.geometry.coordinates[1]}
            headingRad={dockHeadingRad}
            clipPath={dockAppearance.foregroundClipPath}
            zIndex={MAP_IMAGE_LAYER_Z_INDEX.dockForeground}
            onLoad={() => {}}
            onError={() => {}}
        />
        : null;

    // Fit every configured area. The helper also expands the metric extents for
    // the saved bearing because Mapbox fits north-up before onMapLoad restores
    // rotation; without that compensation, rotated gardens can be clipped.
    const [map_ne, map_sw] = useMemo<[[number, number], [number, number]]>(() => {
        if (_datumLon == 0 || _datumLat == 0) {
            return [[0, 0], [0, 0]]
        }
        const bounds = calculateMapViewportBounds(map, bearing);
        const map_sw = transpose(offsetX, offsetY, datum, bounds.minY, bounds.minX)
        const map_ne = transpose(offsetX, offsetY, datum, bounds.maxY, bounds.maxX)
        return [map_ne, map_sw]
    }, [_datumLat, _datumLon, map, offsetX, offsetY, datum, bearing])

    const {
        hasUnsavedChanges, setHasUnsavedChanges, handleEditMap, exitEditMode,
        handleUndo, handleRedo, historyIndex, editHistory,
    } = useMapEditHistory({
        features, setFeatures, editMap, setEditMap,
        extraUnsavedChanges: corridorsDirtySinceEdit,
        onDiscard: () => void revertCorridorsOnDiscard(),
        // A copy of the map is made BEFORE the editor opens; no copy, no edit.
        beforeEdit: mapBackups.backupBeforeEdit,
    });

    // A map backup was restored: the editor's buffered features and the corridor snapshot
    // describe the map that was just replaced, so leave the editor without reverting anything.
    const handleBackupRestored = useCallback(() => {
        corridorEditSnapshotRef.current = null;
        setRestoreBackupOpen(false);
        exitEditMode();
        void lidarCorridors.reload();
    }, [exitEditMode, lidarCorridors]);

    useEffect(() => {
        if (envs) {
            setTileUri(envs.tileUri)
        }
    }, [envs]);

    // Outline an obstacle had before the recorded-obstacle shrink, kept in map-frame
    // metres in the robot's config store (survives reloads and backups); the
    // editor works in lng/lat, so convert at this boundary.
    const {getRecords: getOutlineRecords, remember: rememberOutline, forget: forgetOutline} = obstacleOriginals;
    const shrinkMemory = useMemo<ShrinkMemory>(() => {
        const toXY = (ring: Position[]): OutlineXY[] => ring.map(([lng, lat]) => {
            const [x, y] = itranspose(offsetX, offsetY, datum, lat, lng);
            return {x, y};
        });
        const toRing = (xy: OutlineXY[]): Position[] =>
            closeRing(xy.map((p) => transpose(offsetX, offsetY, datum, p.y, p.x)));
        return {
            find: (ring) => {
                if (datum[0] === 0) return undefined;
                const original = findOriginal(getOutlineRecords(), toXY(ring));
                return original ? toRing(original) : undefined;
            },
            remember: (originalRing, shrunkRing) => {
                if (datum[0] === 0) return;
                void rememberOutline({original: toXY(originalRing), shrunk: toXY(shrunkRing)});
            },
            forget: (shrunkRing) => {
                if (datum[0] === 0) return;
                void forgetOutline(toXY(shrunkRing));
            },
        };
    }, [offsetX, offsetY, datum, getOutlineRecords, rememberOutline, forgetOutline]);

    const {
        modalOpen,
        areaModelOpen,
        newAreaName, setNewAreaName,
        newAreaType, setNewAreaType,
        curMowingAreaFeature, setCurMowingAreaFeature,
        selectedFeatureIds,
        buildLabels,
        onCreate, onUpdate, onCombine, onDelete, onSelectionChange, onOpenDetails,
        handleEditSelectedFeature, handleDrawPolygon, handleDrawShape, handleDrawEmoji,
        handleTrash, handleCombine,
        handleAreaSelect, handleSubtract, handleSplit,
        handleSaveNewArea, updateMowingArea, cancelAreaModal, deleteFeature,
    } = useMapEditing({
        features,
        setFeatures,
        editMap,
        mowingAreas,
        drawRef,
        notification,
        mapInstanceRef,
        shrinkMemory,
    });

    // A just-recorded area turned into an obstacle: the recording followed the
    // robot's own centre, so driving the chassis edge along the object left the
    // outline half a chassis too large. Shrink it ONCE here — the BT only ever
    // records mowing areas, so this conversion is the only moment it can happen
    // (coverage_server owns the geometry; nothing is reimplemented client-side).
    // A failed correction aborts the conversion rather than saving the larger
    // outline silently.
    const handleSaveAreaModal = useCallback(async () => {
        const converting = curMowingAreaFeature.feature_type === 'obstacle'
            && curMowingAreaFeature.orig_feature_type !== 'obstacle';
        if (!converting || !curMowingAreaFeature.shrink_recorded || !curMowingAreaFeature.id) {
            updateMowingArea();
            return;
        }
        const feature = features[curMowingAreaFeature.id];
        const ring = feature && 'geometry' in feature ? (feature.geometry as Polygon).coordinates?.[0] : undefined;
        if (!ring || ring.length < 3 || datum[0] === 0) {
            updateMowingArea();
            return;
        }
        try {
            const res = await guiApi.mowglinext.callCreate("correct_recorded_obstacle", {
                polygon: {
                    points: ring.map((coord) => {
                        const [lon, lat] = coord as [number, number];
                        const [x, y] = itranspose(offsetX, offsetY, datum, lat, lon);
                        return {x, y};
                    }),
                },
            });
            const data = res.data as unknown as {success?: boolean; message?: string; corrected?: {points?: {x?: number; y?: number}[]}};
            const points = data?.corrected?.points ?? [];
            if (res.error || !data?.success || points.length < 3) {
                notification.error({
                    message: t('mapEditArea.shrinkFailed'),
                    description: data?.message ?? res.error?.error,
                });
                return;
            }
            const corrected = points.map((p) => transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0));
            updateMowingArea({type: "Polygon", coordinates: [closeRing(corrected)]});
        } catch (e) {
            notification.error({message: t('mapEditArea.shrinkFailed'), description: String(e)});
        }
    }, [curMowingAreaFeature, features, datum, offsetX, offsetY, guiApi, notification, t, updateMowingArea]);

    useEffect(() => {
        // Don't rebuild features from stream data while in edit mode —
        // path/plan becoming undefined when streams stop would wipe user edits.
        if (editMap) return;

        let newFeatures: Record<string, MowingFeature> = {}
        if (map) {
            const workingAreas = buildFeatures(map.working_area??[], "area", true)
            const navigationAreas = buildFeatures(map.navigation_areas??[], "navigation")
            newFeatures = {...workingAreas, ...navigationAreas}

            // dock_x/dock_y are optional on the wire. `map?.dock_y!` claimed
            // otherwise and pushed `undefined` into transpose(), painting the
            // dock at NaN; skip the marker instead when the pose is absent.
            if (map.dock_x !== undefined && map.dock_y !== undefined) {
                const dock_lonlat = transpose(offsetX, offsetY, datum, map.dock_y, map.dock_x)
                newFeatures["dock"] = new DockFeatureBase(dock_lonlat, map.dock_heading);
            }
        }
        if (path?.xy && path.subpath_offsets?.length) {
            // The backend sends interleaved float32 XY and exact point-index
            // boundaries, so no giant Path/pose objects or gap heuristics.
            const offsets = path.subpath_offsets;
            for (let segmentIdx = 0; segmentIdx + 1 < offsets.length; segmentIdx++) {
                const start = offsets[segmentIdx], end = offsets[segmentIdx + 1];
                if (!Number.isInteger(start) || !Number.isInteger(end) || start < 0 || end > path.xy.length / 2 || end - start < 2) continue;
                const segment: Position[] = [];
                for (let i = start; i < end; i++) {
                    const x = path.xy[i * 2], y = path.xy[i * 2 + 1];
                    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
                    segment.push(transpose(offsetX, offsetY, datum, y, x));
                }
                if (segment.length > 1) {
                    const feature = new PathFeature(`coverage-path-${segmentIdx}`, segment, LAYER_COLORS.coveragePath, 2);
                    newFeatures[feature.id] = feature;
                }
            }
        }
        if (plan?.poses) {
            const coordinates = plan.poses.flatMap((pose) => {
                const x = pose.pose?.position?.x;
                const y = pose.pose?.position?.y;
                if (x === undefined || y === undefined) return [];
                return [transpose(offsetX, offsetY, datum, y, x)];
            });
            const feature = new ActivePathFeature("plan", coordinates);
            newFeatures[feature.id] = feature
        }
        // Preserve the live robot features — they are owned by the pose stream
        // (useMapStreams merges mower/mower-* in) and must survive this
        // map/path/plan-driven rebuild. Replacing the record wholesale wiped
        // the mower on every path update, so the robot only stayed visible
        // while the path overlays were NOT being streamed.
        setFeatures((old) => ({
            ...newFeatures,
            ...Object.fromEntries(
                Object.entries(old).filter(([k]) => k === "mower" || k.startsWith("mower-"))
            ),
        }))
    }, [map, path, plan, offsetX, offsetY, datum, editMap, LAYER_COLORS]);

    useEffect(() => {
        const labels = buildLabels(Object.values(features))
        setLabelsCollection({
            type: "FeatureCollection",
            features: labels
        });
        setMowingAreas(labels.flatMap(feat => {
            if (feat.properties?.title == undefined) {
                return []
            }
            return [{
                key: feat.id as string,
                label: feat.properties.title,
                feat: feat
            }]
        }))
    }, [features]);

    // For each tracked obstacle (transient /obstacle_tracker/obstacles
    // observation), figure out which mowing area's polygon contains its
    // centroid. The result is the area_index map_server expects when we
    // promote the obstacle — the position of the matching area in
    // map_server's areas_ vector. Mowing order and the filtered working-area
    // position are not ROS IDs; resolve the CURRENT index from the area's
    // stable id (mowingAreaIndexById, mowglinext#637) rather than a position
    // captured from `features`, which is frozen while editMap is true and can
    // go stale relative to the still-live-polling `map`.
    // Returns null when no workarea contains the centroid → promote button
    // is disabled because we'd have nowhere to attach it.
    const obstacleAreaIndex = useMemo(() => {
        const result: Record<number, number | null> = {};
        const workareas = Object.values(features)
            .filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature)
            .sort((a, b) => (a.getMowingOrder() ?? 9999) - (b.getMowingOrder() ?? 9999));
        for (const obs of dynamicObstacles) {
            const id = obs.id ?? 0;
            // Compute centroid of the obstacle polygon in ROS coords. The
            // tracker publishes points in ROS map frame (x/y metres).
            const pts = obs.polygon?.points ?? [];
            if (pts.length < 3) {
                result[id] = null;
                continue;
            }
            let cxRos = 0;
            let cyRos = 0;
            for (const p of pts) {
                cxRos += p.x ?? 0;
                cyRos += p.y ?? 0;
            }
            cxRos /= pts.length;
            cyRos /= pts.length;
            // Transpose to lng/lat so we can use the GeoJSON polygons.
            const [cLon, cLat] = transpose(offsetX, offsetY, datum, cyRos, cxRos);
            // Find the first workarea whose polygon contains the centroid.
            // Manual ray-casting (point-in-polygon) — avoids pulling in
            // turf-boolean-point-in-polygon for one call.
            let matchedIdx: number | null = null;
            for (let i = 0; i < workareas.length; ++i) {
                const ring = workareas[i].geometry.coordinates[0] ?? [];
                let inside = false;
                for (let j = 0, k = ring.length - 1; j < ring.length; k = j++) {
                    const xj = ring[j][0];
                    const yj = ring[j][1];
                    const xk = ring[k][0];
                    const yk = ring[k][1];
                    const intersect =
                        (yj > cLat) !== (yk > cLat) &&
                        cLon < ((xk - xj) * (cLat - yj)) / (yk - yj + 1e-12) + xj;
                    if (intersect) inside = !inside;
                }
                if (inside) {
                    matchedIdx = mowingAreaIndexById(map, workareas[i].properties.source_working_area_id) ?? null;
                    break;
                }
            }
            result[id] = matchedIdx;
        }
        return result;
    }, [dynamicObstacles, features, offsetX, offsetY, datum, map]);

    const obstacleAreaNames = useMemo(() => {
        const names: Record<number, string> = {};
        const workareas = Object.values(features)
            .filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature)
            .sort((a, b) => (a.getMowingOrder() ?? 9999) - (b.getMowingOrder() ?? 9999));
        for (let i = 0; i < workareas.length; ++i) {
            const areaIndex = mowingAreaIndexById(map, workareas[i].properties.source_working_area_id);
            if (areaIndex === undefined) continue;
            names[areaIndex] = workareas[i].getLabel(
                t('mapAreasList.unnamedArea', {index: workareas[i].getMowingOrder()})
            );
        }
        return names;
    }, [features, t, map]);

    // PENDING obstacle proposals (wheel-slip dig reports). map_server lists
    // them apart from each area's applied obstacles: they block nothing on the
    // robot until the operator accepts one in ObstacleProposalsPanel. Drawn as a
    // filled hole of their REAL polygon, like any obstacle, labelled with their
    // provenance ("DIG #7"), but with a
    // dashed outline so they never read as a real keepout, and kept OUT of the
    // editable feature set so a map save cannot persist one by accident.
    const obstacleProposals = useMemo(() => extractObstacleProposals(map), [map]);
    const proposalCollection = useMemo<GeoJSON.FeatureCollection>(() => ({
        type: "FeatureCollection",
        features: datum[0] === 0 ? [] : obstacleProposals.map(proposal => {
            const ring = (proposal.polygon.points ?? []).map(p => transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0));
            return {
                type: "Feature" as const,
                id: proposal.id,
                geometry: {type: "Polygon" as const, coordinates: [[...ring, ring[0]]]},
                properties: {proposal_id: proposal.id, proposal_label: `${isDigProposal(proposal) ? t('mapObstacleProposals.digMapLabel') : '?'} #${proposal.id}`},
            };
        }),
    }), [obstacleProposals, offsetX, offsetY, datum, t]);
    const renderProposalLayers = () => (
        <Source type={"geojson"} id={"obstacle-proposals"} data={proposalCollection}>
            <Layer type={"fill"} id={"obstacle-proposal-fill"}
                paint={{'fill-color': '#bf0000', 'fill-opacity': ['case', ['==', ['get', 'proposal_id'], selectedProposalId ?? -1], 0.75, 0.5]}}/>
            <Layer type={"line"} id={"obstacle-proposal-outline"}
                paint={{'line-color': '#d48806', 'line-width': 2, 'line-dasharray': [2, 2]}}/>
            <Layer type={"symbol"} id={"obstacle-proposal-label"}
                layout={{
                    'text-field': ['get', 'proposal_label'],
                    'text-size': 13,
                    'text-font': ['Open Sans Bold'],
                    'text-allow-overlap': true,
                }}
                paint={{
                    'text-color': LAYER_COLORS.labelText,
                    'text-halo-color': LAYER_COLORS.labelHalo,
                    'text-halo-width': 1.5,
                }}/>
        </Source>
    );

    // Build the areas list for the sidebar panel
    const areasList = useMemo(() => {
        const polygons = Object.values(features).filter(
            (f): f is MowingFeatureBase => f instanceof MowingFeatureBase
        );
        return polygons
            .sort((a, b) => {
                // workareas first, then navigation, then obstacles
                const typeOrder: Record<string, number> = { workarea: 0, navigation: 1, obstacle: 2 };
                const ta = typeOrder[a.properties.feature_type] ?? 3;
                const tb = typeOrder[b.properties.feature_type] ?? 3;
                if (ta !== tb) return ta - tb;
                return (a.properties.mowing_order ?? 0) - (b.properties.mowing_order ?? 0);
            })
            .map((f, i, arr) => {
                const areaSqm = turfArea(f);
                const areaLabel = formatArea(areaSqm, i18n.language);
                const ftype = f.properties.feature_type;
                let name = '';
                if (f instanceof MowingAreaFeature) {
                    name = f.getLabel(t('mapAreasList.unnamedArea', {index: f.getMowingOrder()}));
                } else if (f instanceof NavigationFeature) {
                    // Short 1-based ordinal within its own type, not the raw id.
                    const navIdx = arr.slice(0, i).filter(x => x instanceof NavigationFeature).length + 1;
                    name = t('mapAreasList.navigationArea', {index: navIdx});
                } else if (f instanceof ObstacleFeature) {
                    const obsIdx = arr.slice(0, i).filter(x => x instanceof ObstacleFeature).length + 1;
                    name = t('mapAreasList.obstacleArea', {index: obsIdx});
                }
                const mowingOrder = f instanceof MowingAreaFeature ? f.getMowingOrder() : undefined;
                return { id: f.id, name, ftype, areaLabel, mowingOrder };
            });
    }, [features, t]);

    const handleReorder = useCallback((id: string, direction: 'up' | 'down') => {
        setFeatures((curr) => {
            const next = {...curr};
            const target = next[id];
            if (!(target instanceof MowingAreaFeature)) return curr;
            const targetOrder = target.getMowingOrder();
            const swapOrder = direction === 'up' ? targetOrder - 1 : targetOrder + 1;
            const swapFeat = Object.values(next).find(
                (f): f is MowingAreaFeature =>
                    f instanceof MowingAreaFeature && f.getMowingOrder() === swapOrder
            );
            if (!swapFeat) return curr;
            target.setMowingOrder(swapOrder);
            swapFeat.setMowingOrder(targetOrder);
            return next;
        });
    }, []);

    function buildFeatures(areas: MapArea[], type: string, fromLiveMap = false) : Record<string, MowingFeatureBase> {


        return areas?.flatMap((area, index) : MowingFeatureBase[] => {
            if (!area.area?.points?.length) {
                return []
            }

            const nfeat = type=="area" ? new MowingAreaFeature(type + "-" + index.toString() + "-area-0", index+1)
                : new NavigationFeature(type + "-" + index.toString() + "-area-0");//, offsetX, offsetY, datum.
            nfeat.setArea(area, offsetX, offsetY, datum);
            // Preserve source identity separately from editable mowing order.
            // Restored/imported maps cannot target live ROS areas until saved.
            // source_working_area_id (mowglinext#637) is the STABLE identity;
            // source_working_area_index is only the array position at build
            // time and can point at a different area by the time it is
            // re-resolved (the area list is rebuilt wholesale on any edit
            // save) — prefer the id via mowingAreaIndexById where the value
            // outlives this render (e.g. a later button click).
            if (fromLiveMap) {
                nfeat.properties.source_working_area_index = index;
                nfeat.properties.source_working_area_id = area.id;
            }

            let obstacles:  ObstacleFeature[] = [];

            if ((nfeat instanceof MowingAreaFeature) && (area.obstacles))
                obstacles = area.obstacles.map((obstacle, oindex) => {
                const nobst =  new ObstacleFeature(
                    type + "-" + index.toString() + "-obstacle-" + oindex.toString(),
                    nfeat
                );
                
                if (obstacle.points)
                    nobst.transpose(obstacle.points, offsetX, offsetY, datum);

                return nobst;

            })
            return [nfeat, ...obstacles ]
        }).reduce((acc, val) :Record<string, MowingFeatureBase> => {
            if (val.id == undefined) {
                return acc
            }
            acc[val.id] = val;
            return acc;
        }, {} as Record<string, MowingFeatureBase>);
    }

    // Build the full editable feature set (areas, obstacles and dock) from a
    // Map message. The stream-driven effect above does the same thing but is
    // skipped while editMap is true, so map restore (which enters edit mode)
    // calls this directly to populate the features it will save.
    function buildFeaturesFromMap(m: MapType): Record<string, MowingFeature> {
        const newFeatures: Record<string, MowingFeature> = {
            ...buildFeatures(m.working_area ?? [], "area"),
            ...buildFeatures(m.navigation_areas ?? [], "navigation"),
        };
        // No dock fields = no dock feature: a missing dock is NOT a dock at
        // (0, 0, 0) (#704). The restore path keeps the current dock instead.
        if (m.dock_x === undefined || m.dock_y === undefined || m.dock_heading === undefined) {
            return newFeatures;
        }
        const dockLonLat = transpose(offsetX, offsetY, datum, m.dock_y, m.dock_x);
        newFeatures["dock"] = new DockFeatureBase(dockLonLat, m.dock_heading);
        return newFeatures;
    }

    const {
        handleSaveMap,
        handleBackupMap,
        handleRestoreMap,
        handleDownloadGeoJSON,
        handleUploadGeoJSON,
        handleImportOpenMower,
        handleReprojectOpenMowerPreview,
        handleApplyOpenMowerImport,
    } = useMapFiles({
        features,
        setFeatures,
        map,
        setMap,
        editMap,
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
        obstacleOriginals: obstacleOriginals.records,
        restoreObstacleOriginals: obstacleOriginals.replaceAll,
        corridors: lidarCorridors.corridors,
        restoreCorridors: lidarCorridors.save,
    });


    const {manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop} = useManualMode({mowerAction, joyStream, stateName: highLevelStatus.highLevelStatus.state_name});

    // Toggle dock placement mode: re-pressing the button (or pressing Escape)
    // cancels it, so the crosshair cursor is not a one-way trap.
    const handleDockPlacement = useCallback(() => {
        setDockPlacementMode(prev => !prev);
    }, []);

    // Escape cancels an armed dock placement.
    useEffect(() => {
        if (!dockPlacementMode) return;
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key === "Escape") setDockPlacementMode(false);
        };
        window.addEventListener("keydown", onKeyDown);
        return () => window.removeEventListener("keydown", onKeyDown);
    }, [dockPlacementMode]);

    // All drawn obstacle polygons, for the toggleable clearance-preview
    // overlay below. Memoised so useObstacleClearancePreview's own content
    // signature stays stable across unrelated re-renders.
    const obstacleFeaturesList = useMemo(
        (): ObstacleFeature[] => Object.values(features).filter((f): f is ObstacleFeature => f instanceof ObstacleFeature),
        [features],
    );
    const obstacleClearancePreview = useObstacleClearancePreview(obstacleFeaturesList, datum, offsetX, offsetY);

    // The "mowing lines" overlay: the real planner's rings and swaths for one
    // area, with a mow angle / perimeter direction to try before saving.
    const mowingAreaFeaturesList = useMemo(
        (): MowingAreaFeature[] => Object.values(features).filter((f): f is MowingAreaFeature => f instanceof MowingAreaFeature),
        [features],
    );
    // An edit of an area's mowing lines goes into the edit session like moving a polygon corner,
    // so it is saved by Save map (which round-trips these fields), dropped by Cancel, and covered
    // by undo/redo (the snapshots carry the area). A NEW area object replaces the old one: that is
    // the same object the server map data holds, and changing it would survive a Cancel.
    const commitCoverageLines = useCallback((featureId: string, overrides: Required<AreaOverrideFields>) => {
        setFeatures((old) => {
            const feature = old[featureId];
            if (!(feature instanceof MowingAreaFeature) || !feature.area) return old;
            feature.area = {...feature.area, ...overrides};
            return {...old};
        });
    }, []);
    const coveragePreview = useCoveragePreview({
        areas: mowingAreaFeaturesList,
        obstacles: obstacleFeaturesList,
        datum,
        offsetX,
        offsetY,
        globalAngleDeg: Number(settings.mow_angle_deg ?? -1),
        globalDirection: Number(settings.mow_direction ?? 0),
        preferredAreaId: editMap ? selectedFeatureIds[0] : undefined,
        // The lines can only be changed while the map is edited, and are then part of that
        // edit: Save map keeps them, Cancel and undo drop them.
        editMode: editMap,
        onCommit: commitCoverageLines,
        savedAreas: map?.working_area,
    });
    const coverageResumeAvailable = useCoverageResumeAvailable();
    const coveragePreviewAreaLabel = (index: number, name: string) =>
        name || t('mapAreasList.unnamedArea', {index: index + 1});

    // Desktop sidebar: one section open at a time, Mowing areas by default.
    const [sidebarOpen, setSidebarOpen] = useState<string | null>('areas');
    // A section that is folded must not hide what the operator just asked for: turning
    // the mowing-lines overlay on (toolbar) or starting to draw a LiDAR-ignore line
    // (whose Finish/Cancel buttons live in that section) opens it. Adjusting state
    // while rendering on a change is React's documented alternative to an effect.
    const [prevPreviewEnabled, setPrevPreviewEnabled] = useState(coveragePreview.enabled);
    if (coveragePreview.enabled !== prevPreviewEnabled) {
        setPrevPreviewEnabled(coveragePreview.enabled);
        if (coveragePreview.enabled) setSidebarOpen('lines');
    }
    const [prevCorridorDrawing, setPrevCorridorDrawing] = useState(corridorDrawing);
    if (corridorDrawing !== prevCorridorDrawing) {
        setPrevCorridorDrawing(corridorDrawing);
        if (corridorDrawing) setSidebarOpen('lidar');
    }

    // The gl-draw feature currently being drawn for a corridor: it is added to
    // gl-draw's OWN store the instant draw_line_string mode starts (onSetup)
    // and kept live-updated on every tap — reading it back via the public
    // getAll() API (not an internal hack) is how both Finish and Cancel below
    // get at "whatever points the operator has placed so far", without us
    // tracking them ourselves. Distinguished from a SAVED corridor already in
    // the store (corridorDrawFeatures below) by id: a fresh gl-draw feature
    // gets gl-draw's own generated id, never our "lidar-corridor-N" scheme.
    const getInProgressCorridorFeature = useCallback((): Feature<LineString> | null => {
        const found = drawRef.current?.getAll().features.find(
            (candidate): candidate is Feature<LineString> =>
                candidate.geometry?.type === "LineString" &&
                !String(candidate.id ?? "").startsWith("lidar-corridor-"));
        return found ?? null;
    }, []);

    // Escape cancels an in-progress ignore line.
    useEffect(() => {
        if (!corridorDrawing) return;
        const onKeyDown = (e: KeyboardEvent) => {
            if (e.key === "Escape") handleCancelDrawingCorridor();
        };
        window.addEventListener("keydown", onKeyDown);
        return () => window.removeEventListener("keydown", onKeyDown);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [corridorDrawing]);

    // Leaving map edit mode abandons an unsaved draw.
    useEffect(() => {
        if (!editMap && corridorDrawing) handleCancelDrawingCorridor();
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [editMap]);

    // Capture the pre-edit corridor snapshot exactly once, on the
    // false -> true edge. lidarCorridors.corridors is deliberately NOT a
    // dependency: including it would re-capture on every 10s poll (or every
    // in-session edit) while editMap stays true, which would keep moving the
    // snapshot forward and defeat the whole point of "what to revert to".
    useEffect(() => {
        if (editMap) corridorEditSnapshotRef.current = lidarCorridors.corridors;
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [editMap]);

    // Saves a finished corridor line (>= 2 points, ROS map-frame). Called
    // either from the Finish button (handleFinishDrawingCorridor, which reads
    // gl-draw's current in-progress feature) or from onDrawCreate below (the
    // fallback path if the operator instead finishes via gl-draw's own
    // double-click/Enter gesture on draw_line_string).
    const handleFinishCorridor = useCallback(async (coords: [number, number][]) => {
        const points = coords.map(([lng, lat]) => {
            const [x, y] = itranspose(offsetX, offsetY, datum, lat, lng);
            return {x, y, z: 0};
        });
        try {
            await lidarCorridors.save([
                ...lidarCorridors.corridors,
                {name: "", polyline: {points}, width_m: DEFAULT_CORRIDOR_WIDTH_M, id: 0},
            ]);
        } catch (error: unknown) {
            notification.error({
                message: t('mapLidarCorridors.saveFailed'),
                description: error instanceof Error ? error.message : undefined,
            });
        }
    }, [offsetX, offsetY, datum, lidarCorridors, notification, t]);

    // The explicit Finish (✓) button: read whatever gl-draw has accumulated
    // so far (>= 2 points required, same rule gl-draw itself enforces for a
    // valid line), remove the in-progress draft feature from gl-draw's store
    // (it is never one of OUR saved corridors — handleFinishCorridor adds the
    // real one once the save round-trips), and leave draw_line_string mode.
    const handleFinishDrawingCorridor = useCallback(() => {
        const f = getInProgressCorridorFeature();
        const raw = f?.geometry.coordinates as [number, number][] | undefined;
        // Still in draw_line_string, the last coordinate is gl-draw's live
        // cursor vertex, not a point the operator placed (dropLiveVertex).
        const coords = raw && drawRef.current?.getMode() === 'draw_line_string'
            ? dropLiveVertex(raw)
            : raw;
        if (f && drawRef.current) drawRef.current.delete(String(f.id));
        drawRef.current?.changeMode('simple_select');
        setCorridorDrawing(false);
        if (coords && coords.length >= 2) void handleFinishCorridor(coords);
    }, [getInProgressCorridorFeature, handleFinishCorridor]);

    // The explicit Cancel (✗) button / Escape / leaving edit mode: drop
    // whatever gl-draw has drawn so far without saving anything.
    const handleCancelDrawingCorridor = useCallback(() => {
        const f = getInProgressCorridorFeature();
        if (f && drawRef.current) drawRef.current.delete(String(f.id));
        drawRef.current?.changeMode('simple_select');
        setCorridorDrawing(false);
    }, [getInProgressCorridorFeature]);

    const handleStartDrawingCorridor = useCallback(() => {
        setCorridorDrawing(true);
        drawRef.current?.changeMode('draw_line_string');
    }, []);

    const handleCorridorChange = useCallback(async (next: typeof lidarCorridors.corridors) => {
        try {
            await lidarCorridors.save(next);
        } catch (error: unknown) {
            notification.error({
                message: t('mapLidarCorridors.saveFailed'),
                description: error instanceof Error ? error.message : undefined,
            });
        }
    }, [lidarCorridors, notification, t]);

    // Corridors are also handed to DrawControl (map edit mode only) so they get
    // the same vertex drag / midpoint add / vertex delete editing as areas.
    const corridorDrawId = (c: {id?: number}) => `lidar-corridor-${c.id}`;
    const isCorridorDrawFeature = (f: Feature) => String(f.id ?? "").startsWith("lidar-corridor-");

    const corridorDrawFeatures = useMemo((): Feature[] => {
        if (!editMap || datum[0] === 0) return [];
        return lidarCorridors.corridors
            .filter((c) => (c.id ?? 0) !== 0 && (c.polyline?.points?.length ?? 0) >= 2)
            .map((c): Feature => ({
                type: "Feature",
                id: corridorDrawId(c),
                properties: {feature_type: "lidar_corridor", color: CORRIDOR_COLOR, width: 4},
                geometry: {
                    type: "LineString",
                    coordinates: (c.polyline?.points ?? []).map((p) => transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0)),
                },
            }));
    }, [editMap, lidarCorridors.corridors, datum, offsetX, offsetY]);

    const drawControlFeatures = useMemo(
        () => [...drawableFeatures, ...corridorDrawFeatures],
        [drawableFeatures, corridorDrawFeatures]);

    // A finished vertex drag / add / remove on a corridor line -> save the list.
    const handleCorridorDrawUpdate = useCallback((changed: Feature[]) => {
        const byId = new globalThis.Map(changed.map((f) => [String(f.id), f]));
        const next = lidarCorridors.corridors.map((c) => {
            const f = byId.get(corridorDrawId(c));
            if (!f || f.geometry.type !== "LineString") return c;
            const points = f.geometry.coordinates.map(([lng, lat]) => {
                const [x, y] = itranspose(offsetX, offsetY, datum, lat, lng);
                return {x, y, z: 0};
            });
            return points.length >= 2 ? {...c, polyline: {points}} : c;
        });
        void handleCorridorChange(next);
    }, [lidarCorridors.corridors, offsetX, offsetY, datum, handleCorridorChange]);

    const handleCorridorDrawDelete = useCallback((deleted: Feature[]) => {
        const ids = new Set(deleted.map((f) => String(f.id)));
        void handleCorridorChange(lidarCorridors.corridors.filter((c) => !ids.has(corridorDrawId(c))));
    }, [lidarCorridors.corridors, handleCorridorChange]);

    const onDrawUpdate = useCallback((e: {features: Feature[]; action: string}) => {
        const corr = e.features.filter(isCorridorDrawFeature);
        const rest = e.features.filter((f) => !isCorridorDrawFeature(f));
        if (corr.length > 0) handleCorridorDrawUpdate(corr);
        if (rest.length > 0) onUpdate({...e, features: rest});
    }, [handleCorridorDrawUpdate, onUpdate]);

    const onDrawDelete = useCallback((e: {features: Feature[]}) => {
        const corr = e.features.filter(isCorridorDrawFeature);
        const rest = e.features.filter((f) => !isCorridorDrawFeature(f));
        if (corr.length > 0) handleCorridorDrawDelete(corr);
        if (rest.length > 0) onDelete({...e, features: rest});
    }, [handleCorridorDrawDelete, onDelete]);

    // Fallback path for finishing a corridor line: normally the operator taps
    // the Finish (✓) button (handleFinishDrawingCorridor, which reads
    // drawRef.current.getAll() directly and never lets gl-draw fire its own
    // draw.create), but gl-draw's OWN double-click/Enter gesture for
    // draw_line_string still works too and DOES fire draw.create — this
    // catches that case so a corridor drawn that way still saves instead of
    // falling through to onCreate's generic "not a Polygon, discard" branch.
    const onDrawCreate = useCallback((e: {features: Feature[]}) => {
        if (corridorDrawing) {
            const f = e.features[0];
            const coords = f?.geometry?.type === "LineString" ? f.geometry.coordinates as [number, number][] : undefined;
            if (f && drawRef.current) drawRef.current.delete(String(f.id));
            setCorridorDrawing(false);
            if (coords && coords.length >= 2) void handleFinishCorridor(coords);
            return;
        }
        onCreate(e);
    }, [corridorDrawing, onCreate, handleFinishCorridor]);

    // Double-click (or Enter on a selected feature — see DirectSelectWithBoxMode)
    // on an ignore line opens its edit modal directly, the same gesture areas
    // already use to open EditAreaModal.
    const onDrawOpenDetails = useCallback((e: {feature?: Feature}) => {
        const feature = e.feature;
        if (feature && isCorridorDrawFeature(feature)) {
            const i = lidarCorridors.corridors.findIndex((c) => corridorDrawId(c) === String(feature.id));
            if (i >= 0) setCorridorWidthModalIndex(i);
            return;
        }
        onOpenDetails(e);
    }, [onOpenDetails, lidarCorridors.corridors]);

    // Index of the line currently selected on the map (for Make curved / Simplify).
    const selectedCorridorIndex = lidarCorridors.corridors.findIndex((c) => selectedFeatureIds.includes(corridorDrawId(c)));

    // "Edit properties" on the mobile toolbar: a selected ignore line has no
    // MowingFeature entry (handleEditSelectedFeature would no-op on it), so open
    // the width modal instead; anything else falls through as before.
    const handleEditSelectedFeatureOrCorridor = useCallback(() => {
        if (selectedCorridorIndex >= 0) {
            setCorridorWidthModalIndex(selectedCorridorIndex);
            return;
        }
        handleEditSelectedFeature();
    }, [selectedCorridorIndex, handleEditSelectedFeature]);
    const handleReshapeSelectedCorridor = (reshape: (points: XY[]) => XY[]) => {
        const i = selectedCorridorIndex;
        if (i < 0) return;
        const points = (lidarCorridors.corridors[i].polyline?.points ?? []).map((p) => ({x: p.x ?? 0, y: p.y ?? 0}));
        const out = reshape(points);
        void handleCorridorChange(lidarCorridors.corridors.map((c, k) =>
            k === i ? {...c, polyline: {points: out.map((p) => ({x: p.x, y: p.y, z: 0}))}} : c));
    };

    // ROS-frame corridors, as GeoJSON for the map. The line currently being
    // drawn is NOT included here any more — it lives in gl-draw's own store
    // (drawRef, draw_line_string mode) and gl-draw renders it itself with its
    // stock in-progress-line styling, the same way it already renders an
    // in-progress polygon.
    const corridorFeatures = useMemo((): FeatureCollection => {
        const features: Feature[] = [];
        if (datum[0] !== 0) {
            lidarCorridors.corridors.forEach((corridor) => {
                // In map edit mode the lines are drawn (and edited) by DrawControl.
                if (editMap) return;
                const pts = corridor.polyline?.points ?? [];
                if (pts.length < 2) return;
                features.push({
                    type: "Feature",
                    properties: {kind: "corridor"},
                    geometry: {
                        type: "LineString",
                        coordinates: pts.map((p) => transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0)),
                    },
                });
            });
        }
        return {type: "FeatureCollection", features};
    }, [lidarCorridors.corridors, editMap, datum, offsetX, offsetY]);

    // Recorded (working + navigation) area outer rings, lng/lat — used to
    // restrict the rendered band below to the side of each corridor that
    // actually overlaps a recorded area, matching costmap_scan_filter_node's
    // point_in_any_area restriction (a beam is only ever suppressed there;
    // the visualization must not claim more coverage than that).
    const recordedAreaRings = useMemo((): Position[][] =>
        Object.values(features)
            .filter((f): f is MowingAreaFeature | NavigationFeature =>
                f instanceof MowingAreaFeature || f instanceof NavigationFeature)
            .map((f) => f.geometry.coordinates[0] ?? [])
            .filter((ring) => ring.length >= 3),
    [features]);
    const insideRecordedArea = useCallback((lng: number, lat: number): boolean =>
        recordedAreaRings.some((ring) => isRingInsidePolygon([[lng, lat]], ring)),
    [recordedAreaRings]);

    // The actual ignored BAND (width_m wide), not just the centerline the
    // operator clicked — so it's visible on the map exactly what area gets
    // an ignore, not only where. Shown in both view and edit mode (DrawControl
    // itself only ever renders the thin centerline + vertex handles). A saved
    // corridor's band only ever covers the recorded-area side of the line
    // (buildCorridorSideRuns, gated on insideRecordedArea above) — the other
    // side is never suppressed by the filter either, so drawing it would be
    // actively misleading. The in-progress draft (not yet saved) falls back
    // to the plain symmetric band: it has no saved width/id yet and is only
    // a rough preview while the operator is still clicking points.
    // Geometry is built in ROS metres, then each vertex is transposed to
    // lng/lat for rendering — corridors persist in ROS map-frame points, so
    // this stays exact regardless of map projection.
    const corridorBandFeatures = useMemo((): FeatureCollection => {
        const features: Feature[] = [];
        if (datum[0] !== 0) {
            lidarCorridors.corridors.forEach((corridor) => {
                const pts = (corridor.polyline?.points ?? []).map((p) => ({x: p.x ?? 0, y: p.y ?? 0}));
                if (pts.length < 2) return;
                const widthM = corridor.width_m ?? DEFAULT_CORRIDOR_WIDTH_M;
                const insideAreaRos = (p: XY): boolean => {
                    const [lng, lat] = transpose(offsetX, offsetY, datum, p.y, p.x);
                    return insideRecordedArea(lng, lat);
                };
                (['left', 'right'] as const).forEach((side) => {
                    buildCorridorSideRuns(pts, widthM, side, insideAreaRos).forEach((ring) => features.push({
                        type: "Feature",
                        properties: {kind: "band"},
                        geometry: {
                            type: "Polygon",
                            coordinates: [ring.map((p) => transpose(offsetX, offsetY, datum, p.y, p.x))],
                        },
                    }));
                });
            });
            // The line currently being drawn has no saved width/id yet and no
            // longer has a JS-side point array to build a preview band from
            // (gl-draw owns those points now — see corridorFeatures above) —
            // it draws with gl-draw's own stock in-progress-line styling
            // instead, same as an in-progress polygon has no fill-band preview
            // either.
        }
        return {type: "FeatureCollection", features};
    // insideRecordedArea (and transitively recordedAreaRings/features) was
    // missing here: corridors and areas load from two independent sources
    // (lidarCorridors polls its own endpoint; features comes from the /map
    // WS stream) — whichever arrives first left this memo permanently
    // cached against a stale (often empty) area list, since none of the
    // OTHER deps necessarily change again afterwards. The band then never
    // recovered until something unrelated (e.g. editing a corridor) forced
    // a recompute. Field-reported 2026-09-28: no band drawn at all.
    }, [lidarCorridors.corridors, datum, offsetX, offsetY, insideRecordedArea]);

    const handleMapClick = useCallback((e: {lngLat: {lng: number; lat: number}}) => {
        if (!dockPlacementMode) return;
        setDockPlacementMode(false);
        const coord: [number, number] = [e.lngLat.lng, e.lngLat.lat];
        setFeatures(prev => {
            const existingDock = prev["dock"];
            const heading = existingDock instanceof DockFeatureBase && existingDock.hasValidHeading()
                ? existingDock.getHeading()
                : undefined;
            return {...prev, dock: new DockFeatureBase(coord, heading)};
        });
        setHasUnsavedChanges(true);
        setDockDirty(true);
    }, [dockPlacementMode, setHasUnsavedChanges]);

    // Map → panel side of the two-way obstacle highlight: while the cursor is
    // over a tracked-obstacle polygon, mirror its id into selectedObstacleId so
    // the matching panel row lights up. e.features only carries the interactive
    // layers (DYN_OBSTACLE_INTERACTIVE_LAYERS); when the cursor leaves every
    // obstacle it is empty → clears the highlight. Hovering the overlay panel
    // does not reach the map canvas, so panel-driven highlights are never
    // clobbered here.
    const handleMapMouseMove = useCallback((e: {features?: Array<{properties?: Record<string, unknown> | null}>}) => {
        const hit = e.features?.find(f => f.properties?.feature_type === 'dyn-obstacle');
        const id = hit ? (hit.properties?.obs_id as number) : null;
        setSelectedObstacleId(prev => (prev === id ? prev : id));
    }, []);

    // Belt-and-suspenders: any time dockDirty flips to true, ensure
    // hasUnsavedChanges is also true so the Save Map button glows. The
    // inline setHasUnsavedChanges(true) above + the useMapEditHistory
    // features-watcher should already cover this, but a stale closure
    // or a state-batching race used to leave the dock-only case where
    // the user pinned a new dock pose but the save toolbar stayed
    // calm. This effect makes the dirty signal sticky.
    useEffect(() => {
        if (dockDirty) setHasUnsavedChanges(true);
    }, [dockDirty, setHasUnsavedChanges]);

    // Mower action callbacks shared between desktop and mobile toolbars.
    // mowingAreas is only rebuilt when `features` changes, which is frozen
    // while editMap is true (see the map/path/plan-rebuild effect above) — a
    // selection made, or held across, an edit session can be older than the
    // latest `map` poll. Resolve the CURRENT ROS index from the area's
    // stable id (mowglinext#637) at the moment this fires, not from a
    // position captured whenever mowingAreas last rebuilt, so a re-indexed
    // area list can never send COMMAND_START at the wrong area.
    const startSelectedArea = (key: string) => {
        const item = mowingAreas.find(item => item.key == key);
        const index = mowingAreaIndexById(map, item?.feat?.properties?.id);
        if (index === undefined) return Promise.reject(new Error(t("crossHatch.areaUnavailable")));
        return mowerAction("start_in_area", {area: index})();
    };

    const mowerActions = useMemo(() => ({
        onStart: mowerAction("high_level_control", {Command: 1}),
        onHome: mowerAction("high_level_control", {Command: 2}),
        onEmergencyOn: mowerAction("emergency", {Emergency: 1}),
        onEmergencyOff: mowerAction("emergency", {Emergency: 0}),
        onAreaRecording: mowerAction("high_level_control", {Command: 3}),
        // Match MapToolbar's isIdle: the BT publishes IDLE_DOCKED as the
        // primary resting state; "IDLE" without a suffix only appears as the
        // manual-mow fallthrough. There is no "pause flag" in the stack (the
        // old mower_logic/manual_pause_mowing OpenMower command does not exist
        // here and returned HTTP 500, which also broke Continue-from-idle by
        // rejecting before the START fired). Use real HighLevelControl commands:
        // Continue = START (mow_progress persists, so it resumes where it left
        // off); Pause = STOP (COMMAND_STOP=8 → StopHoldSequence: mower off, halt
        // in place, Nav2 left up so the mission can resume, no dock drive).
        onContinueOrPause:
            highLevelStatus.highLevelStatus.state_name === "IDLE_DOCKED" ||
            highLevelStatus.highLevelStatus.state_name === "IDLE"
                ? mowerAction("high_level_control", {Command: 1})
                : mowerAction("high_level_control", {Command: 8}),
        // Retain explicit direction/OFF choices in the tree's session policy;
        // direct hardware commands would be overwritten by its next tick.
        onBladeForward: mowerAction("blade_control", {mow_enabled: 1, mow_direction: 0}),
        onBladeBackward: mowerAction("blade_control", {mow_enabled: 1, mow_direction: 1}),
        onBladeOff: mowerAction("blade_control", {mow_enabled: 0, mow_direction: 0}),
        onRecordFinish: mowerAction("high_level_control", {Command: 5}),
        onRecordCancel: mowerAction("high_level_control", {Command: 6}),
    }), [mowerAction, highLevelStatus.highLevelStatus.state_name]);

    // Centered message panel used for the missing-token and missing-datum
    // states — a plain, translated explanation instead of an eternal spinner
    // or a broken map.
    const CenteredMessage: React.FC<{title: string; detail?: string}> = ({title, detail}) => (
        <div style={{
            width: '100%',
            height: '100%',
            minHeight: compact ? undefined : 240,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            gap: 8,
            textAlign: 'center',
            padding: 24,
            color: colors.textSecondary,
            background: colors.bgCard,
            borderRadius: 12,
        }}>
            <div style={{fontSize: compact ? 14 : 16, fontWeight: 600, color: colors.text}}>{title}</div>
            {detail && <div style={{fontSize: compact ? 12 : 14}}>{detail}</div>}
        </div>
    );

    if (!MAPBOX_TOKEN) {
        return <CenteredMessage
            title={t('mapPage.mapboxTokenMissingTitle')}
            detail={t('mapPage.mapboxTokenMissingDetail')}
        />;
    }
    if (_datumLon == 0 || _datumLat == 0) {
        return <CenteredMessage
            title={t('mapPage.noDatumTitle')}
            detail={t('mapPage.noDatumDetail')}
        />;
    }
    if (compact) {
        return (
            <div style={{width: '100%', height: '100%', position: 'relative'}}>
                {map_sw?.length && map_ne?.length ? <Map key={mapKey}
                                                         reuseMaps
                                                         antialias
                                                         projection={{
                                                             name: "globe"
                                                         }}
                                                         mapboxAccessToken={MAPBOX_TOKEN}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                             fitBoundsOptions: {padding: 16},
                                                         }}
                                                         style={{width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         interactive={false}
                                                         onLoad={onMapLoad}
                                                         attributionControl={false}
                >
                    {tileUri ? <Source type={"raster"} id={"custom-raster"} tiles={[tileUri]} tileSize={256}/> : null}
                    {tileUri ? <Layer type={"raster"} source={"custom-raster"} id={"custom-layer"}/> : null}
                    <Source type={"geojson"} id={"labels"} data={labelsCollection}/>
                    <Layer type={"symbol"} id={"mower"} source={"labels"} layout={{
                        "text-field": ['get', 'title'],
                        "text-rotation-alignment": "auto",
                        "text-allow-overlap": true,
                        "text-anchor": "top"
                    }} paint={{
                        "text-color": LAYER_COLORS.labelText,
                        "text-halo-color": LAYER_COLORS.labelHalo,
                        "text-halo-width": 1.5,
                    }}/>
                    <DrawControl
                        drawRef={drawRef}
                        styles={MapStyle}
                        userProperties={true}
                        features={drawableFeatures}
                        position="top-left"
                        displayControlsDefault={false}
                        editMode={false}
                        controls={{}}
                        defaultMode="simple_select"
                        onCreate={() => {}}
                        onUpdate={() => {}}
                        onCombine={() => {}}
                        onDelete={() => {}}
                        onSelectionChange={() => {}}
                        onOpenDetails={() => {}}
                    />
                    <Source type={"geojson"} id={"display-features"} data={mapDisplayData}>
                        <Layer type={"line"} id={"display-lines"} filter={['==', ['geometry-type'], 'LineString']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{
                                'line-color': ['get', 'color'],
                                'line-width': ['get', 'width'],
                            }}/>
                        {/* Dock marker */}
                        <Layer type={"circle"} id={"dock-halo"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 12,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': LAYER_COLORS.dock,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 2,
                            }}/>
                        <Layer type={"symbol"} id={"fleet-peer-label"}
                            filter={['==', ['get', 'feature_type'], 'fleet-peer']}
                            layout={{
                                'text-field': ['get', 'name'],
                                'text-size': 11,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.4],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': FLEET_PEER_COLOR,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        <Layer type={"symbol"} id={"dock-label"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            layout={{
                                'text-field': 'DOCK',
                                'text-size': 10,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.8],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': LAYER_COLORS.dock,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': ['get', 'color'],
                                'fill-opacity': 0.55,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': LAYER_COLORS.mowerOutline,
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': LAYER_COLORS.mower,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                        {/* Persistent tracked-obstacle polygons + id labels (compact overview: no highlight) */}
                        {renderDynObstacleLayers(false)}
                    </Source>
                    {dockImageMarker}
                    {mowerImageMarker}
                    {dockForegroundMarker}
                </Map> : <Spinner/>}
            </div>
        );
    }

    return (
        <div style={{
            // Full-bleed the map across the AppShell's main padding.
            // Desktop main padding is 24px top / 32px horizontal / 48px bottom.
            // Mobile main padding includes the bottom safe area as well.
            position: 'relative',
            height: isMobile ? 'calc(100% + 122px + env(safe-area-inset-bottom, 0px))' : 'calc(100% + 72px)',
            width:  isMobile ? 'calc(100% + 28px)'  : 'calc(100% + 64px)',
            margin: isMobile ? '-12px -14px calc(-110px - env(safe-area-inset-bottom, 0px))' : '-24px -32px -48px',
        }}>
            <NewAreaModal
                open={modalOpen}
                areaType={newAreaType}
                areaName={newAreaName}
                onAreaTypeChange={setNewAreaType}
                onAreaNameChange={setNewAreaName}
                onSave={handleSaveNewArea}
                onCancel={deleteFeature}
            />
            <EditAreaModal
                open={areaModelOpen}
                area={curMowingAreaFeature}
                onChange={setCurMowingAreaFeature}
                onSave={() => void handleSaveAreaModal()}
                onCancel={cancelAreaModal}
            />
            <RestoreMapBackupModal
                open={restoreBackupOpen}
                onClose={() => setRestoreBackupOpen(false)}
                list={mapBackups.list}
                restore={mapBackups.restore}
                onRestored={handleBackupRestored}
            />
            <EditLidarCorridorModal
                key={corridorWidthModalIndex ?? 'none'}
                corridor={corridorWidthModalIndex !== null ? lidarCorridors.corridors[corridorWidthModalIndex] ?? null : null}
                busy={lidarCorridors.busy}
                onSave={(name, widthM) => {
                    if (corridorWidthModalIndex === null) return;
                    void handleCorridorChange(lidarCorridors.corridors.map((c, i) =>
                        i === corridorWidthModalIndex ? {...c, name, width_m: widthM} : c));
                    setCorridorWidthModalIndex(null);
                }}
                onCancel={() => setCorridorWidthModalIndex(null)}
            />

            <div style={{height: '100%', position: 'relative'}}>
                {map_sw?.length && map_ne?.length ? <Map key={mapKey}
                                                         reuseMaps
                                                         antialias
                                                         projection={{
                                                             name: "globe"
                                                         }}
                                                         mapboxAccessToken={MAPBOX_TOKEN}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                             fitBoundsOptions: {padding: 24},
                                                         }}
                                                         style={{width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         maxZoom={MAP_MAX_ZOOM}
                                                         onLoad={onMapLoad}
                                                         onClick={handleMapClick}
                                                         interactiveLayerIds={DYN_OBSTACLE_INTERACTIVE_LAYERS}
                                                         onMouseMove={handleMapMouseMove}
                                                         cursor={dockPlacementMode || corridorDrawing ? 'crosshair' : undefined}
                >
                    {tileUri ? <Source type={"raster"} id={"custom-raster"} tiles={[tileUri]} tileSize={256}/> : null}
                    {tileUri ? <Layer type={"raster"} source={"custom-raster"} id={"custom-layer"}/> : null}
                    <Source type={"geojson"} id={"labels"} data={labelsCollection}/>
                    <Layer type={"symbol"} id={"mower"} source={"labels"} layout={{
                        "text-field": ['get', 'title'],
                        "text-rotation-alignment": "auto",
                        "text-allow-overlap": true,
                        "text-anchor": "top"
                    }} paint={{
                        "text-color": LAYER_COLORS.labelText,
                        "text-halo-color": LAYER_COLORS.labelHalo,
                        "text-halo-width": 1.5,
                    }}/>
                    <DrawControl
                        drawRef={drawRef}
                        styles={MapStyle}
                        userProperties={true}
                        features={drawControlFeatures}
                        position="top-left"
                        displayControlsDefault={false}
                        editMode={editMap}
                        controls={{}}
                        defaultMode="simple_select"
                        onCreate={onDrawCreate}
                        onUpdate={onDrawUpdate}
                        onCombine={onCombine}
                        onDelete={onDrawDelete}
                        onSelectionChange={onSelectionChange}
                        onOpenDetails={onDrawOpenDetails}
                    />
                    {/* Display-only features: mower, dock, heading, paths */}
                    <Source type={"geojson"} id={"display-features"} data={mapDisplayData}>
                        <Layer type={"line"} id={"display-lines"} filter={['==', ['geometry-type'], 'LineString']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{
                                'line-color': ['get', 'color'],
                                'line-width': ['get', 'width'],
                            }}/>
                        {/* Dock marker */}
                        <Layer type={"circle"} id={"dock-halo"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 12,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': LAYER_COLORS.dock,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 2,
                            }}/>
                        <Layer type={"symbol"} id={"fleet-peer-label"}
                            filter={['==', ['get', 'feature_type'], 'fleet-peer']}
                            layout={{
                                'text-field': ['get', 'name'],
                                'text-size': 11,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.4],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': FLEET_PEER_COLOR,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        <Layer type={"symbol"} id={"dock-label"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            layout={{
                                'text-field': 'DOCK',
                                'text-size': 10,
                                'text-font': ['Open Sans Bold'],
                                'text-offset': [0, 1.8],
                                'text-anchor': 'top',
                            }}
                            paint={{
                                'text-color': LAYER_COLORS.dock,
                                'text-halo-color': LAYER_COLORS.halo,
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': ['get', 'color'],
                                'fill-opacity': 0.55,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': LAYER_COLORS.mowerOutline,
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': LAYER_COLORS.mower,
                                'circle-stroke-color': LAYER_COLORS.halo,
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': LAYER_COLORS.halo,
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                        {/* Persistent tracked-obstacle polygons + id labels + hover/select highlight */}
                        {renderDynObstacleLayers(true)}
                    </Source>
                    {dockImageMarker}
                    {mowerImageMarker}
                    {dockForegroundMarker}
                    {/* PENDING obstacle proposals (dig reports): dashed, never a real keepout */}
                    {renderProposalLayers()}
                    {/* Toggleable preview (off by default) of the LIVE obstacle_margin
                        buffer coverage_server actually plans against — a distinct
                        dashed amber outline so it is never mistaken for the drawn
                        obstacle polygon itself. */}
                    {obstacleClearancePreview.enabled && (
                        <Source type={"geojson"} id={"obstacle-clearance-preview"} data={obstacleClearancePreview.features}>
                            <Layer type={"line"} id={"obstacle-clearance-preview-line"}
                                layout={{'line-cap': 'round', 'line-join': 'round'}}
                                paint={{'line-color': OBSTACLE_CLEARANCE_PREVIEW_COLOR, 'line-width': 2, 'line-dasharray': [1, 1.5]}}/>
                        </Source>
                    )}
                    {/* Mowing lines: the planner's headland rings and swaths for the chosen
                        area. Arrowheads are polygons drawn to scale (no font/sprite needed)
                        and show the driving direction; the green dot is where it starts. */}
                    {coveragePreview.enabled && (
                        <>
                            <Source type={"geojson"} id={"coverage-preview-lines"} data={coveragePreview.layers.lines}>
                                <Layer type={"line"} id={"coverage-preview-swaths"}
                                    filter={['==', ['get', 'kind'], 'swath']}
                                    layout={{'line-cap': 'butt', 'line-join': 'round'}}
                                    paint={{'line-color': COVERAGE_SWATH_COLOR, 'line-width': 1.5, 'line-opacity': 0.9}}/>
                                <Layer type={"line"} id={"coverage-preview-rings"}
                                    filter={['==', ['get', 'kind'], 'ring']}
                                    layout={{'line-cap': 'round', 'line-join': 'round'}}
                                    paint={{'line-color': COVERAGE_RING_COLOR, 'line-width': 2, 'line-opacity': 0.95}}/>
                            </Source>
                            <Source type={"geojson"} id={"coverage-preview-arrows"} data={coveragePreview.layers.arrows}>
                                <Layer type={"fill"} id={"coverage-preview-arrow-fill"}
                                    filter={['in', ['get', 'kind'], ['literal', ['ring-arrow', 'swath-arrow']]]}
                                    paint={{
                                        'fill-color': ['match', ['get', 'kind'], 'ring-arrow', COVERAGE_RING_COLOR, COVERAGE_SWATH_COLOR],
                                        'fill-opacity': 1,
                                    }}/>
                                <Layer type={"line"} id={"coverage-preview-arrow-outline"}
                                    filter={['in', ['get', 'kind'], ['literal', ['ring-arrow', 'swath-arrow']]]}
                                    paint={{'line-color': '#000000', 'line-width': 1, 'line-opacity': 0.7}}/>
                            </Source>
                            {/* Where the route starts: draggable. The planner snaps the dropped point onto
                                the outermost ring and answers with the real start, so the dot always shows
                                what the robot will do. Hidden with the rings off (nothing to start on). */}
                            {coveragePreview.startLonLat && coveragePreview.startAdjustable && (
                                <CoverageStartMarker
                                    longitude={coveragePreview.startLonLat[0]}
                                    latitude={coveragePreview.startLonLat[1]}
                                    ring={coveragePreview.outerRingLonLat}
                                    settledCount={coveragePreview.settledCount}
                                    draggable={editMap}
                                    onMove={coveragePreview.moveStartTo}
                                    title={t('coveragePreview.startMarkerTitle')}
                                />
                            )}
                        </>
                    )}
                    {/* The actual ignored band (width_m), under everything else so the
                        centerline / vertex handles / draft points stay legible on top.
                        A narrow band (the default is 0.2 m) can rasterize to a
                        sub-pixel-wide fill at normal zoom and simply disappear —
                        fill-opacity alone is not enough. The outline `line` layers
                        below draw at a fixed PIXEL width regardless of how thin the
                        polygon is geographically (same reason the centerline itself
                        stays a constant 4 px), so the two edges of the band stay
                        visible even for a very narrow line. */}
                    <Source type={"geojson"} id={"lidar-corridor-bands"} data={corridorBandFeatures}>
                        <Layer type={"fill"} id={"lidar-corridor-band-fill"}
                            filter={['==', ['get', 'kind'], 'band']}
                            paint={{'fill-color': CORRIDOR_COLOR, 'fill-opacity': 0.35}}/>
                        <Layer type={"line"} id={"lidar-corridor-band-outline"}
                            filter={['==', ['get', 'kind'], 'band']}
                            paint={{'line-color': CORRIDOR_COLOR, 'line-width': 2, 'line-opacity': 0.9}}/>
                    </Source>
                    {/* Operator-drawn LiDAR-ignore lines. The line currently being drawn
                        is NOT here — it lives in gl-draw's own store (draw_line_string
                        mode) and gl-draw renders it with its own stock styling. */}
                    <Source type={"geojson"} id={"lidar-corridors"} data={corridorFeatures}>
                        <Layer type={"line"} id={"lidar-corridor-lines"}
                            filter={['==', ['get', 'kind'], 'corridor']}
                            layout={{'line-cap': 'round', 'line-join': 'round'}}
                            paint={{'line-color': CORRIDOR_COLOR, 'line-width': 4, 'line-dasharray': [2, 1]}}/>
                    </Source>
                    {/* fusion_graph's LiDAR anchor map (walls as ink, scanned ground as a faint wash). */}
                    {lidarMapImage && (
                        <Source type={"image"} id={"lidar-map"} url={lidarMapImage.url} coordinates={lidarMapImage.coordinates}>
                            <Layer type={"raster"} id={"lidar-map-layer"} paint={{
                                "raster-opacity": 0.85,
                                "raster-fade-duration": 0,
                                "raster-resampling": "nearest",
                            }}/>
                        </Source>
                    )}
                    {mowProgressImage && (
                        <Source type={"image"} id={"mow-progress"} url={mowProgressImage.url} coordinates={mowProgressImage.coordinates}>
                            <Layer type={"raster"} id={"mow-progress-layer"} paint={{
                                "raster-opacity": 1,
                                "raster-fade-duration": 0,
                            }}/>
                        </Source>
                    )}
                    {/* Raw scan points only until the LiDAR map exists — then the map replaces them. */}
                    {!lidarMapImage && (
                        <Source type={"geojson"} id={"lidar"} data={lidarCollection}>
                            <Layer type={"circle"} id={"lidar-points"} paint={{
                                "circle-radius": 3,
                                "circle-color": [
                                    "case",
                                    ["==", ["get", "intensity"], "hit"],
                                    LAYER_COLORS.lidarHit,
                                    LAYER_COLORS.lidarMiss
                                ],
                                "circle-stroke-width": 0,
                            }}/>
                        </Source>
                    )}
                </Map> : <Spinner/>}
                <JoystickOverlay
                    visible={highLevelStatus.highLevelStatus.state_name === "RECORDING" || highLevelStatus.highLevelStatus.state_name === "MANUAL_MOWING" || manualMode}
                    isRecording={highLevelStatus.highLevelStatus.state_name === "RECORDING"}
                    mobile={isMobile}
                    onMove={handleJoyMove}
                    onStop={handleJoyStop}
                    onFinishRecording={mowerActions.onRecordFinish}
                    onCancelRecording={mowerActions.onRecordCancel}
                    onHome={mowerActions.onHome}
                />
                {isMobile && (
                    <MapToolbarMobile
                        editMap={editMap}
                        hasUnsavedChanges={hasUnsavedChanges}
                        manualMode={manualMode}
                        useSatellite={useSatellite}
                        historyIndex={historyIndex}
                        editHistoryLength={editHistory.length}
                        mowingAreas={mowingAreas}
                        onRestoreBackup={() => setRestoreBackupOpen(true)}
                        selectedFeatureCount={selectedFeatureIds.length}
                        onEditMap={handleEditMap}
                        onEditSelectedFeature={handleEditSelectedFeatureOrCorridor}
                        onDrawPolygon={handleDrawPolygon}
                        onDrawShape={handleDrawShape}
                        onDrawEmoji={handleDrawEmoji}
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onPlaceDock={handleDockPlacement}
                        dockPlacementMode={dockPlacementMode}
                        onDrawLidarCorridor={handleStartDrawingCorridor}
                        lidarCorridorDrawing={corridorDrawing}
                        onFinishLidarCorridor={handleFinishDrawingCorridor}
                        onCancelLidarCorridor={handleCancelDrawingCorridor}
                        lidarCorridorSelected={selectedCorridorIndex >= 0}
                        onSaveMap={handleSaveMap}
                        onUndo={handleUndo}
                        onRedo={handleRedo}
                        onToggleSatellite={() => setUseSatellite(!useSatellite)}
                        showObstacleClearance={obstacleClearancePreview.enabled}
                        onToggleObstacleClearance={() => obstacleClearancePreview.setEnabled((v) => !v)}
                        showCoveragePreview={coveragePreview.enabled}
                        onToggleCoveragePreview={() => coveragePreview.setEnabled((v) => !v)}
                        mowerAppearanceId={mowerAppearance.id}
                        onMowerAppearanceChange={handleMowerAppearanceChange}
                        dockAppearanceId={dockAppearance.id}
                        onDockAppearanceChange={handleDockAppearanceChange}
                        onManualMode={handleManualMode}
                        onStopManualMode={handleStopManualMode}
                        onBackupMap={handleBackupMap}
                        onRestoreMap={handleRestoreMap}
                        onDownloadGeoJSON={handleDownloadGeoJSON}
                        onUploadGeoJSON={handleUploadGeoJSON}
                        onImportOpenMower={() => handleImportOpenMower(setImportPreview, setImportFileText)}
                        onMowArea={startSelectedArea}
                        stateName={highLevelStatus.highLevelStatus.state_name}
                        highLevelState={highLevelStatus.highLevelStatus.state}
                        emergency={highLevelStatus.highLevelStatus.emergency}
                        onResetMowingProgress={resetMowingProgress}
                        {...mowerActions}
                    />
                )}
                {/* Mobile: obstacle proposals need an accept/reject surface too — the
                    operator is usually standing next to the robot with a phone. The
                    mobile toolbar lives at the bottom, so this card takes the top. */}
                {isMobile && ((!editMap && obstacleProposals.length > 0) || coveragePreview.enabled) && (
                    <div style={{position: 'absolute', top: 12, left: 12, right: 12, zIndex: 10, maxHeight: '45%', overflowY: 'auto', background: colors.glassBackground, borderRadius: 14, border: colors.glassBorder, boxShadow: colors.glassShadow}}>
                        {!editMap && (
                            <ObstacleProposalsPanel
                                proposals={obstacleProposals}
                                selectedProposalId={selectedProposalId}
                                onHoverProposal={setSelectedProposalId}
                            />
                        )}
                        {coveragePreview.enabled && (
                            <div style={{borderTop: !editMap && obstacleProposals.length > 0 ? `1px solid ${colors.borderSubtle}` : undefined}}>
                                <CoveragePreviewPanel preview={coveragePreview} areaLabel={coveragePreviewAreaLabel} resumeAvailable={coverageResumeAvailable}/>
                            </div>
                        )}
                    </div>
                )}
                {/* Desktop: Edit mode — left vertical toolbar */}
                {!isMobile && editMap && (
                    <MapEditorToolbar
                        hasUnsavedChanges={hasUnsavedChanges}
                        historyIndex={historyIndex}
                        editHistoryLength={editHistory.length}
                        selectedFeatureCount={selectedFeatureIds.length}
                        onSaveMap={handleSaveMap}
                        onCancel={handleEditMap}
                        onUndo={handleUndo}
                        onRedo={handleRedo}
                        onDrawPolygon={handleDrawPolygon}
                        onDrawShape={handleDrawShape}
                        onDrawEmoji={handleDrawEmoji}
                        onDrawLidarCorridor={handleStartDrawingCorridor}
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onEditSelectedFeature={handleEditSelectedFeatureOrCorridor}
                        onPlaceDock={handleDockPlacement}
                        dockPlacementMode={dockPlacementMode}
                        onRestoreBackup={() => setRestoreBackupOpen(true)}
                    />
                )}
                {/* Desktop: View mode — bottom glass toolbar */}
                {!isMobile && !editMap && (
                    <div style={{position: 'absolute', bottom: 12, left: 16, right: 16, zIndex: 10, background: colors.glassBackground, backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, borderRadius: 18, border: colors.glassBorder, boxShadow: colors.glassShadow, padding: '10px 14px'}}>
                        <MapToolbar
                            manualMode={manualMode}
                            useSatellite={useSatellite}
                            mowerAppearanceId={mowerAppearance.id}
                            onMowerAppearanceChange={handleMowerAppearanceChange}
                            dockAppearanceId={dockAppearance.id}
                            onDockAppearanceChange={handleDockAppearanceChange}
                            mowingAreas={mowingAreas}
                            stateName={highLevelStatus.highLevelStatus.state_name}
                            highLevelState={highLevelStatus.highLevelStatus.state}
                            emergency={highLevelStatus.highLevelStatus.emergency}
                            onResetMowingProgress={resetMowingProgress}
                            pitched={pitched}
                            onTogglePitch={togglePitch}
                            onEditMap={handleEditMap}
                            onToggleSatellite={() => setUseSatellite(!useSatellite)}
                            showObstacleClearance={obstacleClearancePreview.enabled}
                            onToggleObstacleClearance={() => obstacleClearancePreview.setEnabled((v) => !v)}
                            showCoveragePreview={coveragePreview.enabled}
                            onToggleCoveragePreview={() => coveragePreview.setEnabled((v) => !v)}
                            onManualMode={handleManualMode}
                            onStopManualMode={handleStopManualMode}
                            onBackupMap={handleBackupMap}
                            onRestoreMap={handleRestoreMap}
                            onDownloadGeoJSON={handleDownloadGeoJSON}
                            onImportOpenMower={() => handleImportOpenMower(setImportPreview, setImportFileText)}
                            onMowArea={startSelectedArea}
                            {...mowerActions}
                        />
                    </div>
                )}
                {/* Desktop: Right panel — collapsible sections, one open at a time */}
                {!isMobile && (
                    <div style={{position: 'absolute', top: 12, right: 16, zIndex: 10, display: 'flex', flexDirection: 'column', gap: 0, width: 240, maxHeight: 'calc(100% - 32px)', background: colors.glassBackground, backdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, WebkitBackdropFilter: displayMode === 'visual' ? 'blur(22px) saturate(140%)' : undefined, borderRadius: 18, border: colors.glassBorder, boxShadow: colors.glassShadow, overflow: 'hidden'}}>
                        <MapSidebarAccordion
                            openKey={sidebarOpen}
                            onOpenChange={setSidebarOpen}
                            sections={[
                                {
                                    key: 'areas',
                                    title: t('mapAreasList.areasHeader', {count: areasList.filter(a => a.ftype === 'workarea').length}),
                                    content: areasList.every(a => a.ftype === 'obstacle') ? (
                                        <div style={{padding: 12, fontSize: 13, color: colors.textSecondary}}>{t('mapSidebar.noAreas')}</div>
                                    ) : (
                                        <AreasListPanel
                                            areas={areasList.filter(a => a.ftype !== 'obstacle')}
                                            hideHeader
                                            onAreaClick={editMap ? handleAreaSelect : undefined}
                                            onReorder={editMap ? handleReorder : undefined}
                                            selectedId={editMap ? selectedFeatureIds[0] : undefined}
                                        />
                                    ),
                                },
                                {
                                    key: 'obstacles',
                                    title: t('mapSidebar.obstacles'),
                                    badge: areasList.filter(a => a.ftype === 'obstacle').length + dynamicObstacles.length + obstacleProposals.length,
                                    content: areasList.every(a => a.ftype !== 'obstacle') && dynamicObstacles.length === 0 && obstacleProposals.length === 0 ? (
                                        <div style={{padding: 12, fontSize: 13, color: colors.textSecondary}}>{t('mapSidebar.noObstacles')}</div>
                                    ) : (
                                        <>
                                            {areasList.some(a => a.ftype === 'obstacle') && (
                                                <div>
                                                    <div style={{
                                                        padding: '8px 12px',
                                                        fontSize: 12,
                                                        fontWeight: 600,
                                                        color: colors.muted,
                                                        textTransform: 'uppercase',
                                                        letterSpacing: '0.05em',
                                                        borderBottom: `1px solid ${colors.borderSubtle}`,
                                                    }}>
                                                        {t('mapSidebar.drawnObstacles', {count: areasList.filter(a => a.ftype === 'obstacle').length})}
                                                    </div>
                                                    <AreasListPanel
                                                        areas={areasList.filter(a => a.ftype === 'obstacle')}
                                                        hideHeader
                                                        onAreaClick={editMap ? handleAreaSelect : undefined}
                                                        selectedId={editMap ? selectedFeatureIds[0] : undefined}
                                                    />
                                                </div>
                                            )}
                                            {dynamicObstacles.length > 0 && (
                                                <div style={{borderTop: areasList.some(a => a.ftype === 'obstacle') ? `1px solid ${colors.borderSubtle}` : undefined}}>
                                                    <TrackedObstaclesPanel
                                                        obstacles={dynamicObstacles}
                                                        obstacleAreaIndex={obstacleAreaIndex}
                                                        areaNames={obstacleAreaNames}
                                                        selectedObstacleId={selectedObstacleId}
                                                        onHoverObstacle={setSelectedObstacleId}
                                                    />
                                                </div>
                                            )}
                                            {obstacleProposals.length > 0 && (
                                                <div style={{borderTop: areasList.some(a => a.ftype === 'obstacle') || dynamicObstacles.length > 0 ? `1px solid ${colors.borderSubtle}` : undefined}}>
                                                    <ObstacleProposalsPanel
                                                        proposals={obstacleProposals}
                                                        selectedProposalId={selectedProposalId}
                                                        onHoverProposal={setSelectedProposalId}
                                                    />
                                                </div>
                                            )}
                                        </>
                                    ),
                                },
                                {
                                    key: 'lines',
                                    title: t('mapSidebar.mowingLines'),
                                    content: coveragePreview.enabled ? (
                                        <CoveragePreviewPanel preview={coveragePreview} areaLabel={coveragePreviewAreaLabel} resumeAvailable={coverageResumeAvailable}/>
                                    ) : (
                                        <div style={{padding: '4px 12px 12px'}}>
                                            <div style={{fontSize: 12, color: colors.textSecondary, marginBottom: 8}}>{t('mapSidebar.mowingLinesOff')}</div>
                                            <Button size="small" onClick={() => coveragePreview.setEnabled(true)}>{t('mapSidebar.mowingLinesShow')}</Button>
                                        </div>
                                    ),
                                },
                                {
                                    key: 'lidar',
                                    title: t('mapLidarCorridors.header', {count: lidarCorridors.corridors.length}),
                                    extra: <LidarCorridorsInfo editable={editMap}/>,
                                    content: (
                                        <LidarCorridorsPanel
                                            hideHeader
                                            corridors={lidarCorridors.corridors}
                                            busy={lidarCorridors.busy}
                                            editable={editMap}
                                            drawing={corridorDrawing}
                                            onFinishDraw={handleFinishDrawingCorridor}
                                            onCancelDraw={handleCancelDrawingCorridor}
                                            onSelect={(index) => {
                                                const c = lidarCorridors.corridors[index];
                                                if (c) handleAreaSelect(corridorDrawId(c));
                                            }}
                                            selectedIndex={selectedCorridorIndex >= 0 ? selectedCorridorIndex : null}
                                            onSmooth={() => handleReshapeSelectedCorridor((pts) => smoothPolyline(pts))}
                                            onSimplify={() => handleReshapeSelectedCorridor((pts) => simplifyPolyline(pts))}
                                        />
                                    ),
                                },
                                {
                                    key: 'offset',
                                    title: t('mapOffsetPanel.mapOffset'),
                                    content: (
                                        <div style={{padding: '4px 12px 12px'}}>
                                            <MapOffsetPanel
                                                offsetX={offsetX}
                                                offsetY={offsetY}
                                                onChangeX={handleOffsetX}
                                                onChangeY={handleOffsetY}
                                            />
                                        </div>
                                    ),
                                },
                                {
                                    key: 'rotation',
                                    title: t('mapOffsetPanel.mapRotation'),
                                    content: (
                                        <div style={{padding: '4px 12px 12px'}}>
                                            <MapRotationPanel bearing={bearing} onChangeBearing={handleBearing}/>
                                        </div>
                                    ),
                                },
                            ]}
                        />
                    </div>
                )}
            </div>
            <ImportOpenMowerModal
                preview={importPreview}
                onApply={async (omDatumLat, omDatumLon, importDatum) => {
                    if (!importFileText) {
                        throw new Error("No imported map text in memory — re-select the file.");
                    }
                    await handleApplyOpenMowerImport(importFileText, omDatumLat, omDatumLon, importDatum);
                }}
                onReproject={async (omDatumLat, omDatumLon) => {
                    if (!importFileText) {
                        throw new Error("No imported map text in memory — re-select the file.");
                    }
                    const summary = await handleReprojectOpenMowerPreview(importFileText, omDatumLat, omDatumLon);
                    setImportPreview(summary);
                    return summary;
                }}
                onClose={() => {
                    setImportPreview(null);
                    setImportFileText(null);
                }}
            />
        </div>
    );
}

//MapPage.whyDidYouRender = true

export default MapPage;
