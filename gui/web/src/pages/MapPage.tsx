import "@mapbox/mapbox-gl-draw/dist/mapbox-gl-draw.css";
import {useApi} from "../hooks/useApi.ts";
import {App} from "antd";
import turfArea from "@turf/area";
import React, {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {MapArea, Marker} from "../types/ros.ts";
import DrawControl from "../components/DrawControl.tsx";
import Map, {Layer, Source} from 'react-map-gl/mapbox';
import type {Map as MapboxMap} from 'mapbox-gl';
import type {Feature} from 'geojson';
import {FeatureCollection, Position} from "geojson";
import {useMowerAction} from "../components/MowerActions.tsx";
import {MapStyle} from "./MapStyle.tsx";
import {drawLine, itranspose, transpose} from "../utils/map.tsx";
import {useSettings} from "../hooks/useSettings.ts";
import {useConfig} from "../hooks/useConfig.tsx";
import {useEnv} from "../hooks/useEnv.tsx";
import {Spinner} from "../components/Spinner.tsx";
import {MowingFeature, MowingAreaFeature, DockFeatureBase, MowingFeatureBase, NavigationFeature, ObstacleFeature, ActivePathFeature, PathFeature} from "../types/map.ts";
import {useMapEditHistory} from "./map/hooks/useMapEditHistory.ts";
import {useMapOffset} from "./map/hooks/useMapOffset.ts";
import {useMapBearing} from "./map/hooks/useMapBearing.ts";
import {useManualMode} from "./map/hooks/useManualMode.ts";
import {useMapEditing} from "./map/hooks/useMapEditing.ts";
import {useMapStreams} from "./map/hooks/useMapStreams.ts";
import {useMapFiles} from "./map/hooks/useMapFiles.ts";
import {NewAreaModal} from "./map/components/NewAreaModal.tsx";
import {EditAreaModal} from "./map/components/EditAreaModal.tsx";
import {AreasListPanel} from "./map/components/AreasListPanel.tsx";
import {MapOffsetPanel} from "./map/components/MapOffsetPanel.tsx";
import {MapToolbar} from "./map/components/MapToolbar.tsx";
import {MapToolbarMobile} from "./map/components/MapToolbarMobile.tsx";
import {MapEditorToolbar} from "./map/components/MapEditorToolbar.tsx";
import {JoystickOverlay} from "./map/components/JoystickOverlay.tsx";
import {useIsMobile} from "../hooks/useIsMobile.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";


export const MapPage: React.FC<{compact?: boolean}> = ({compact = false}) => {
    const {notification} = App.useApp();
    const {colors} = useThemeMode();
    const isMobile = useIsMobile();
    const mowerAction = useMowerAction()

    const {settings} = useSettings()
    const [labelsCollection, setLabelsCollection] = useState<FeatureCollection>({
        type: "FeatureCollection",
        features: []
    })
    const {config, setConfig} = useConfig(["gui.map.offset.x", "gui.map.offset.y", "gui.map.display.bearing"])
    const envs = useEnv()
    const guiApi = useApi()
    const [tileUri, setTileUri] = useState<string | undefined>()
    const [editMap, setEditMap] = useState<boolean>(false)
    const [features, setFeatures] = useState<Record<string, MowingFeature>>({});
    const [dockPlacementMode, setDockPlacementMode] = useState<boolean>(false);
    // Track whether the user actually moved the dock during this edit
    // session. The dock feature is rebuilt from the live /map topic on
    // every render, and saving without this flag would clobber the
    // persisted dock pose with a stale value (e.g. when dock_pose was
    // updated by the calibration service but the /map topic hasn't
    // re-emitted yet).
    const [dockDirty, setDockDirty] = useState<boolean>(false);
    const [mapKey, setMapKey] = useState<string>("origin")
    const [useSatellite, setUseSatellite] = useState(true)
    const robotPoseRef = useRef<{ x: number; y: number; heading: number } | null>(null)
    const mapInstanceRef = useRef<MapboxMap | null>(null)
    const drawRef = useRef<import('@mapbox/mapbox-gl-draw').default | null>(null);

    // Only include editable polygon features for DrawControl — exclude mower,
    // paths, and other display-only features so that frequent pose updates don't
    // trigger DrawControl to deleteAll() + re-add, which wipes out selection state.
    const drawableFeatures = useMemo(
        () => Object.values(features).filter(f => f instanceof MowingFeatureBase),
        [features]
    );

    // Extracted hooks
    const {offsetX, offsetY, handleOffsetX, handleOffsetY} = useMapOffset({config, setConfig, notification});
    const {bearing, handleBearing} = useMapBearing({config, setConfig, notification});

    // Apply bearing imperatively when the user edits it from the rotation
    // panel (slider/input/reset button). Mapbox-GL's `setBearing` rotates
    // the camera without remounting the Map; using initialViewState alone
    // would freeze the rotation at first paint and ignore later changes.
    useEffect(() => {
        const m = mapInstanceRef.current;
        if (!m) return;
        if (Math.abs(m.getBearing() - bearing) > 0.5) {
            m.easeTo({bearing, duration: 200});
        }
    }, [bearing]);

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

    // Display-only features (mower, dock, heading, paths) rendered as separate layers
    const displayFeatures = useMemo<GeoJSON.FeatureCollection>(() => {
        const feats = Object.values(features)
            .filter(f => !(f instanceof MowingFeatureBase))
            .map(f => ({
                type: "Feature" as const,
                id: f.id,
                geometry: f.geometry,
                properties: f.properties,
            }));

        // Add dock heading direction line (longer, with contrasting color)
        const dock = features["dock"];
        if (dock instanceof DockFeatureBase) {
            const coords = dock.getCoordinates();
            const rosCoords = datum[0] !== 0 ? itranspose(offsetX, offsetY, datum, coords[1], coords[0]) : [0, 0];
            const endPoint = drawLine(offsetX, offsetY, datum, rosCoords[1], rosCoords[0], dock.getHeading());
            feats.push({
                type: "Feature" as const,
                id: "dock-heading",
                geometry: {type: "LineString", coordinates: [coords, endPoint]},
                properties: {color: "#00e676", width: 3, feature_type: "dock-heading"},
            });
        }

        return {type: "FeatureCollection", features: feats};
    }, [features, offsetX, offsetY, datum]);

    const [mowingAreas, setMowingAreas] = useState<{ key: string, label: string, feat: Feature }[]>([])

    const {map, setMap, path, plan, lidarCollection, coverageCellsImage, highLevelStatus, joyStream} = useMapStreams({
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

    // Compute map bounds for the Mapbox viewport — depends on map data for centering
    const [map_ne, map_sw] = useMemo<[[number, number], [number, number]]>(() => {
        if (_datumLon == 0 || _datumLat == 0) {
            return [[0, 0], [0, 0]]
        }
        const map_center = (map && map.map_center_y && map.map_center_x) ? transpose(offsetX, offsetY, datum, map.map_center_y, map.map_center_x) : [_datumLon, _datumLat]
        // Use map center as datum for bounds calculation
        const centerDatum: [number, number, number] = [map_center[1], map_center[0], 0]
        const map_sw = transpose(0, 0, centerDatum, -((map?.map_height ?? 10) / 2), -((map?.map_width ?? 10) / 2))
        const map_ne = transpose(0, 0, centerDatum, ((map?.map_height ?? 10) / 2), ((map?.map_width ?? 10) / 2))
        return [map_ne, map_sw]
    }, [_datumLat, _datumLon, map, offsetX, offsetY, datum])

    const {
        hasUnsavedChanges, setHasUnsavedChanges, handleEditMap,
        handleUndo, handleRedo, historyIndex, editHistory,
    } = useMapEditHistory({features, setFeatures, editMap, setEditMap});

    useEffect(() => {
        if (envs) {
            setTileUri(envs.tileUri)
        }
    }, [envs]);

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
    });
    useEffect(() => {
        // Don't rebuild features from stream data while in edit mode —
        // path/plan becoming undefined when streams stop would wipe user edits.
        if (editMap) return;

        let newFeatures: Record<string, MowingFeature> = {}
        if (map) {
            const workingAreas = buildFeatures(map.working_area??[], "area")
            const navigationAreas = buildFeatures(map.navigation_areas??[], "navigation")
            newFeatures = {...workingAreas, ...navigationAreas}

            const dock_lonlat = transpose(offsetX, offsetY, datum, map?.dock_y!!, map?.dock_x!!)
            newFeatures["dock"] = new DockFeatureBase(dock_lonlat, map?.dock_heading ?? 0);
        }
        if (path?.markers) {
            Object.values<Marker>(path.markers).filter((f) => {
                return f.type == 4 && f.action == 0
            }).forEach((marker, index) => {
                const line: Position[] = marker.points?.map(point => {
                    return transpose(offsetX, offsetY, datum, point.y!!, point.x!!)
                }) ?? []

                const feature = new PathFeature("path-" + index.toString(), line, `rgba(${(marker.color?.r ?? 0) * 255}, ${(marker.color?.g ?? 0) * 255}, ${(marker.color?.b ?? 0) * 255}, ${(marker.color?.a ?? 1) * 255})`);
                newFeatures[feature.id] = feature

            })
        }
        if (plan?.poses) {
            const coordinates = plan.poses.map((pose) => {
                return transpose(offsetX, offsetY, datum, pose.pose?.position?.y!, pose.pose?.position?.x!)
            });
            const feature = new ActivePathFeature("plan", coordinates);
            newFeatures[feature.id] = feature
        }
        if (console.debug) {
            console.debug("Set new features");
            console.debug(newFeatures);
        }
        setFeatures(newFeatures)
    }, [map, path, plan, offsetX, offsetY, datum, editMap]);

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
            .map((f) => {
                const areaSqm = turfArea(f);
                const areaLabel = areaSqm >= 10000
                    ? `${(areaSqm / 10000).toFixed(2)} ha`
                    : `${areaSqm.toFixed(0)} m²`;
                const ftype = f.properties.feature_type;
                let name = '';
                if (f instanceof MowingAreaFeature) {
                    name = f.getLabel();
                } else if (f instanceof NavigationFeature) {
                    name = `Navigation ${f.id}`;
                } else if (f instanceof ObstacleFeature) {
                    name = `Obstacle ${f.id}`;
                }
                const mowingOrder = f instanceof MowingAreaFeature ? f.getMowingOrder() : undefined;
                return { id: f.id, name, ftype, areaLabel, mowingOrder };
            });
    }, [features]);

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

    function buildFeatures(areas: MapArea[], type: string) : Record<string, MowingFeatureBase> {


        return areas?.flatMap((area, index) : MowingFeatureBase[] => {
            if (!area.area?.points?.length) {
                return []
            }

            const nfeat = type=="area" ? new MowingAreaFeature(type + "-" + index.toString() + "-area-0", index+1)
                : new NavigationFeature(type + "-" + index.toString() + "-area-0");//, offsetX, offsetY, datum.
            nfeat.setArea(area, offsetX, offsetY, datum);

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

  


    const {
        handleSaveMap,
        handleBackupMap,
        handleRestoreMap,
        handleDownloadGeoJSON,
        handleUploadGeoJSON,
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
    });


    const {manualMode, handleManualMode, handleStopManualMode, handleJoyMove, handleJoyStop} = useManualMode({mowerAction, joyStream});

    const handleDockPlacement = useCallback(() => {
        setDockPlacementMode(true);
    }, []);

    const handleMapClick = useCallback((e: {lngLat: {lng: number; lat: number}}) => {
        if (!dockPlacementMode) return;
        setDockPlacementMode(false);
        const coord: [number, number] = [e.lngLat.lng, e.lngLat.lat];
        setFeatures(prev => {
            const existingDock = prev["dock"];
            const heading = existingDock instanceof DockFeatureBase ? existingDock.getHeading() : 0;
            return {...prev, dock: new DockFeatureBase(coord, heading)};
        });
        setHasUnsavedChanges(true);
        setDockDirty(true);
    }, [dockPlacementMode, setHasUnsavedChanges]);

    const handleDockHeadingChange = useCallback((heading: number) => {
        setFeatures(prev => {
            const dock = prev["dock"];
            if (!(dock instanceof DockFeatureBase)) return prev;
            const updated = new DockFeatureBase(dock.getCoordinates(), heading);
            return {...prev, dock: updated};
        });
        setHasUnsavedChanges(true);
        setDockDirty(true);
    }, [setHasUnsavedChanges]);

    const handleSetDockAtMower = useCallback(() => {
        const pose = robotPoseRef.current;
        if (!pose) {
            notification.warning({message: "No robot position available"});
            return;
        }
        const coord = transpose(offsetX, offsetY, datum, pose.y, pose.x);
        setFeatures(prev => ({...prev, dock: new DockFeatureBase(coord, pose.heading)}));
        setDockPlacementMode(false);
        setHasUnsavedChanges(true);
        setDockDirty(true);
        notification.success({message: "Dock set at robot position"});
    }, [offsetX, offsetY, datum, setHasUnsavedChanges, notification]);

    // Mower action callbacks shared between desktop and mobile toolbars
    const mowerActions = useMemo(() => ({
        onStart: mowerAction("high_level_control", {Command: 1}),
        onHome: mowerAction("high_level_control", {Command: 2}),
        onEmergencyOn: mowerAction("emergency", {Emergency: 1}),
        onEmergencyOff: mowerAction("emergency", {Emergency: 0}),
        onAreaRecording: mowerAction("high_level_control", {Command: 3}),
        onMowNextArea: mowerAction("high_level_control", {Command: 4}),
        // Match MapToolbar's isIdle: the BT publishes IDLE_DOCKED as the
        // primary resting state; "IDLE" without a suffix only appears as the
        // manual-mow fallthrough. Continue unpauses then re-starts; Pause
        // only flips the pause flag — the BT handles the rest.
        onContinueOrPause:
            highLevelStatus.highLevelStatus.state_name === "IDLE_DOCKED" ||
            highLevelStatus.highLevelStatus.state_name === "IDLE"
                ? async () => {
                    await mowerAction("mower_logic", {Config: {Bools: [{Name: "manual_pause_mowing", Value: false}]}})();
                    await mowerAction("high_level_control", {Command: 1})();
                }
                : mowerAction("mower_logic", {Config: {Bools: [{Name: "manual_pause_mowing", Value: true}]}}),
        onBladeForward: mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 0}),
        onBladeBackward: mowerAction("mow_enabled", {MowEnabled: 1, MowDirection: 1}),
        onBladeOff: mowerAction("mow_enabled", {MowEnabled: 0, MowDirection: 0}),
        onRecordFinish: mowerAction("high_level_control", {Command: 5}),
        onRecordCancel: mowerAction("high_level_control", {Command: 6}),
    }), [mowerAction, highLevelStatus.highLevelStatus.state_name]);

    if (_datumLon == 0 || _datumLat == 0) {
        return <Spinner/>
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
                                                         mapboxAccessToken={import.meta.env.VITE_MAPBOX_TOKEN || "pk.eyJ1IjoiY2VkYm9zc25lbyIsImEiOiJjbGxldjB4aDEwOW5vM3BxamkxeWRwb2VoIn0.WOccbQZZyO1qfAgNxnHAnA"}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                         }}
                                                         style={{width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         interactive={false}
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
                        "text-color": "#ffffff",
                        "text-halo-color": "rgba(0, 0, 0, 0.8)",
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
                    <Source type={"geojson"} id={"display-features"} data={displayFeatures}>
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
                                'circle-color': '#ffffff',
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': '#ff00f2',
                                'circle-stroke-color': '#ffffff',
                                'circle-stroke-width': 2,
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
                                'text-color': '#ff00f2',
                                'text-halo-color': '#ffffff',
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': '#00a6ff',
                                'fill-opacity': 0.35,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': '#003d66',
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': '#00a6ff',
                                'circle-stroke-color': '#ffffff',
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': '#ffffff',
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                    </Source>
                    {coverageCellsImage && (
                        <Source type={"image"} id={"coverage-cells"} url={coverageCellsImage.url} coordinates={coverageCellsImage.coordinates}>
                            <Layer type={"raster"} id={"coverage-cells-layer"} paint={{
                                "raster-opacity": 0.7,
                                "raster-fade-duration": 0,
                            }}/>
                        </Source>
                    )}
                </Map> : <Spinner/>}
            </div>
        );
    }

    return (
        <div style={{height: isMobile ? 'calc(100% + 8px)' : 'calc(100% + 10px)', margin: isMobile ? '-8px -8px 0' : '-10px -24px 0', width: isMobile ? 'calc(100% + 16px)' : 'calc(100% + 48px)'}}>
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
                onSave={updateMowingArea}
                onCancel={cancelAreaModal}
            />

            <div style={{height: '100%', position: 'relative'}}>
                {map_sw?.length && map_ne?.length ? <Map key={mapKey}
                                                         reuseMaps
                                                         antialias
                                                         projection={{
                                                             name: "globe"
                                                         }}
                                                         mapboxAccessToken={import.meta.env.VITE_MAPBOX_TOKEN || "pk.eyJ1IjoiY2VkYm9zc25lbyIsImEiOiJjbGxldjB4aDEwOW5vM3BxamkxeWRwb2VoIn0.WOccbQZZyO1qfAgNxnHAnA"}
                                                         initialViewState={{
                                                             bounds: [{lng: map_sw[0], lat: map_sw[1]}, {lng: map_ne[0], lat: map_ne[1]}],
                                                             bearing,
                                                         }}
                                                         style={{width: '100%', height: '100%'}}
                                                         mapStyle={useSatellite ? "mapbox://styles/mapbox/satellite-streets-v12" : "mapbox://styles/mapbox/dark-v11"}
                                                         onLoad={(e) => {
                                                             const m = e.target as unknown as MapboxMap;
                                                             mapInstanceRef.current = m;
                                                             // Capture user-driven rotation (right-click drag on
                                                             // desktop, two-finger rotate on touch — both enabled
                                                             // by default in mapbox-gl) and persist via the same
                                                             // debounced handler the slider uses.
                                                             m.on('rotateend', () => handleBearing(m.getBearing()));
                                                         }}
                                                         onClick={handleMapClick}
                                                         cursor={dockPlacementMode ? 'crosshair' : undefined}
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
                        "text-color": "#ffffff",
                        "text-halo-color": "rgba(0, 0, 0, 0.8)",
                        "text-halo-width": 1.5,
                    }}/>
                    <DrawControl
                        drawRef={drawRef}
                        styles={MapStyle}
                        userProperties={true}
                        features={drawableFeatures}
                        position="top-left"
                        displayControlsDefault={false}
                        editMode={editMap}
                        controls={{}}
                        defaultMode="simple_select"
                        onCreate={onCreate}
                        onUpdate={onUpdate}
                        onCombine={onCombine}
                        onDelete={onDelete}
                        onSelectionChange={onSelectionChange}
                        onOpenDetails={onOpenDetails}
                    />
                    {/* Display-only features: mower, dock, heading, paths */}
                    <Source type={"geojson"} id={"display-features"} data={displayFeatures}>
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
                                'circle-color': '#ffffff',
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"dock-point"}
                            filter={['==', ['get', 'feature_type'], 'dock']}
                            paint={{
                                'circle-radius': 9,
                                'circle-color': '#ff00f2',
                                'circle-stroke-color': '#ffffff',
                                'circle-stroke-width': 2,
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
                                'text-color': '#ff00f2',
                                'text-halo-color': '#ffffff',
                                'text-halo-width': 1.5,
                            }}/>
                        {/* Mower footprint (robot shape from URDF) */}
                        <Layer type={"fill"} id={"mower-footprint-fill"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'fill-color': '#00a6ff',
                                'fill-opacity': 0.35,
                            }}/>
                        <Layer type={"line"} id={"mower-footprint-outline"}
                            filter={['==', ['get', 'feature_type'], 'mower-footprint']}
                            paint={{
                                'line-color': '#003d66',
                                'line-width': 2,
                            }}/>
                        {/* Mower center point */}
                        <Layer type={"circle"} id={"mower-point"}
                            filter={['==', ['get', 'feature_type'], 'mower']}
                            paint={{
                                'circle-radius': 4,
                                'circle-color': '#00a6ff',
                                'circle-stroke-color': '#ffffff',
                                'circle-stroke-width': 1.5,
                            }}/>
                        {/* Other display points (Point geometry only — exclude polygon/line vertices) */}
                        <Layer type={"circle"} id={"display-points-halo"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 8,
                                'circle-color': '#ffffff',
                                'circle-opacity': 0.9,
                            }}/>
                        <Layer type={"circle"} id={"display-points"}
                            filter={['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'feature_type'], 'dock'], ['!=', ['get', 'feature_type'], 'mower']]}
                            paint={{
                                'circle-radius': 5,
                                'circle-color': ['get', 'color'],
                            }}/>
                    </Source>
                    {coverageCellsImage && (
                        <Source type={"image"} id={"coverage-cells"} url={coverageCellsImage.url} coordinates={coverageCellsImage.coordinates}>
                            <Layer type={"raster"} id={"coverage-cells-layer"} paint={{
                                "raster-opacity": 0.7,
                                "raster-fade-duration": 0,
                            }}/>
                        </Source>
                    )}
                    <Source type={"geojson"} id={"lidar"} data={lidarCollection}>
                        <Layer type={"circle"} id={"lidar-points"} paint={{
                            "circle-radius": 3,
                            "circle-color": [
                                "case",
                                ["==", ["get", "intensity"], "hit"],
                                "rgba(255, 50, 50, 0.8)",
                                "rgba(255, 220, 80, 0.4)"
                            ],
                            "circle-stroke-width": 0,
                        }}/>
                    </Source>
                </Map> : <Spinner/>}
                <JoystickOverlay
                    visible={highLevelStatus.highLevelStatus.state_name === "RECORDING" || highLevelStatus.highLevelStatus.state_name === "MANUAL_MOWING" || manualMode}
                    isRecording={highLevelStatus.highLevelStatus.state_name === "RECORDING"}
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
                        selectedFeatureCount={selectedFeatureIds.length}
                        onEditMap={handleEditMap}
                        onEditSelectedFeature={handleEditSelectedFeature}
                        onDrawPolygon={handleDrawPolygon}
                        onDrawShape={handleDrawShape}
                        onDrawEmoji={handleDrawEmoji}
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onPlaceDock={handleDockPlacement}
                        onSetDockAtMower={handleSetDockAtMower}
                        dockPlacementMode={dockPlacementMode}
                        onSaveMap={handleSaveMap}
                        onUndo={handleUndo}
                        onRedo={handleRedo}
                        onToggleSatellite={() => setUseSatellite(!useSatellite)}
                        onManualMode={handleManualMode}
                        onStopManualMode={handleStopManualMode}
                        onBackupMap={handleBackupMap}
                        onRestoreMap={handleRestoreMap}
                        onDownloadGeoJSON={handleDownloadGeoJSON}
                        onUploadGeoJSON={handleUploadGeoJSON}
                        onMowArea={(key) => {
                            const item = mowingAreas.find(item => item.key == key)
                            return mowerAction("start_in_area", {
                                area: item?.feat?.properties?.index,
                            })()
                        }}
                        stateName={highLevelStatus.highLevelStatus.state_name}
                        emergency={highLevelStatus.highLevelStatus.emergency}
                        {...mowerActions}
                    />
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
                        onTrash={handleTrash}
                        onCombine={handleCombine}
                        onSubtract={handleSubtract}
                        onSplit={handleSplit}
                        onEditSelectedFeature={handleEditSelectedFeature}
                        onPlaceDock={handleDockPlacement}
                        onSetDockAtMower={handleSetDockAtMower}
                        dockPlacementMode={dockPlacementMode}
                        dockHeading={features["dock"] instanceof DockFeatureBase ? (features["dock"] as DockFeatureBase).getHeading() : 0}
                        onDockHeadingChange={handleDockHeadingChange}
                    />
                )}
                {/* Desktop: View mode — bottom glass toolbar */}
                {!isMobile && !editMap && (
                    <div style={{position: 'absolute', bottom: 12, left: 16, right: 16, zIndex: 10, background: colors.glassBackground, backdropFilter: 'blur(16px) saturate(180%)', WebkitBackdropFilter: 'blur(16px) saturate(180%)', borderRadius: 12, border: colors.glassBorder, boxShadow: colors.glassShadow, padding: '10px 14px'}}>
                        <MapToolbar
                            manualMode={manualMode}
                            useSatellite={useSatellite}
                            mowingAreas={mowingAreas}
                            stateName={highLevelStatus.highLevelStatus.state_name}
                            emergency={highLevelStatus.highLevelStatus.emergency}
                            onEditMap={handleEditMap}
                            onToggleSatellite={() => setUseSatellite(!useSatellite)}
                            onManualMode={handleManualMode}
                            onStopManualMode={handleStopManualMode}
                            onBackupMap={handleBackupMap}
                            onRestoreMap={handleRestoreMap}
                            onDownloadGeoJSON={handleDownloadGeoJSON}
                            onMowArea={(key) => {
                                const item = mowingAreas.find(item => item.key == key)
                                return mowerAction("start_in_area", {
                                    area: item?.feat?.properties?.index,
                                })()
                            }}
                            {...mowerActions}
                        />
                    </div>
                )}
                {/* Desktop: Right panel — areas list + offset */}
                {!isMobile && (
                    <div style={{position: 'absolute', top: 12, right: 16, zIndex: 10, display: 'flex', flexDirection: 'column', gap: 0, width: 240, maxHeight: 'calc(100% - 32px)', background: colors.glassBackground, backdropFilter: 'blur(16px) saturate(180%)', WebkitBackdropFilter: 'blur(16px) saturate(180%)', borderRadius: 12, border: colors.glassBorder, boxShadow: colors.glassShadow, overflow: 'hidden'}}>
                        <AreasListPanel
                            areas={areasList}
                            onAreaClick={editMap ? handleAreaSelect : undefined}
                            onReorder={editMap ? handleReorder : undefined}
                            selectedId={editMap ? selectedFeatureIds[0] : undefined}
                        />
                        <div style={{borderTop: `1px solid ${colors.borderSubtle}`, padding: 8}}>
                            <MapOffsetPanel
                                offsetX={offsetX}
                                offsetY={offsetY}
                                bearing={bearing}
                                onChangeX={handleOffsetX}
                                onChangeY={handleOffsetY}
                                onChangeBearing={handleBearing}
                            />
                        </div>
                    </div>
                )}
            </div>
        </div>
    );
}

//MapPage.whyDidYouRender = true

export default MapPage;
