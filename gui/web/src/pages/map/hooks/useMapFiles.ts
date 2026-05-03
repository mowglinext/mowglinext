import React, {ChangeEvent} from "react";
import type {NotificationInstance} from "antd/es/notification/interface";
import type {FeatureCollection, Feature} from "geojson";
import type {Map as MapType} from "../../../types/ros.ts";
import {
    MowingFeature,
    MowingAreaFeature,
    NavigationFeature,
    ObstacleFeature,
    DockFeatureBase,
    MowingFeatureBase,
} from "../../../types/map.ts";
import type {Api, MowgliMapArea, MowgliReplaceMapReq} from "../../../api/Api.ts";
import {dedupePoints, getQuaternionFromHeading, itranspose} from "../../../utils/map.tsx";

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
}: UseMapFilesOptions) {
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
            const idDetails = f.id.split("-");
            if (idDetails.length !== 4) {
                console.error("Invalid id " + f.id);
                continue;
            }
            const type = idDetails[0];
            if (!areas[type]) {
                console.error("Unknown area type " + type);
                continue;
            }

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

        // Process obstacles and attach them to their parent area
        for (const f of obstacleFeatures) {
            const parentArea = f.getMowingArea();
            const parentMapping = featureIndexMap[parentArea.id];
            if (!parentMapping) {
                console.error("Obstacle " + f.id + " references unknown parent area " + parentArea.id);
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

        try {
            await guiApi.mowglinext.putMowglinext(updateMsg);
            notification.success({
                message: "Area saved",
            });
            setHasUnsavedChanges(false);
            setEditMap(false);
        } catch (e: any) {
            notification.error({
                message: "Failed to save area",
                description: e.message,
            });
        }

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
            });
            setDockDirty(false);
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
            });
            reader.readAsDataURL(file);
        });
        input.click();
    };

    const handleDownloadGeoJSON = () => {
        const geojson = {
            type: "FeatureCollection",
            features: Object.values(features),
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
                const geojson = JSON.parse(event.target?.result as string) as FeatureCollection;
                const geojsonfeatures = geojson.features.reduce((acc, feature) => {
                    acc[feature.id as string] = feature;
                    return acc;
                }, {} as Record<string, Feature>);

                const newFeatures = {} as Record<string, MowingFeature>;
                Object.values(geojsonfeatures).forEach(element => {
                    const areaType = element?.properties?.feature_type as string;

                    let nfeat = null;
                    if (!element.id)
                        return;

                    if (typeof element.id == 'number')
                        element.id = element.id.toString();

                    if (element.geometry.type == 'Polygon') {
                        switch (areaType) {
                            case 'workarea':
                                nfeat = element as MowingAreaFeature;
                                break;
                            case 'navigation':
                                nfeat = element as NavigationFeature;
                                break;
                            case 'obstacle':
                                nfeat = element as ObstacleFeature;
                                break;
                            default:
                                notification.error({
                                    message: `Unknown type ${areaType}`,
                                });
                                setFeatures({...features}); // revert
                                return;
                        }
                    } else {
                        switch (areaType) {
                            case 'dock':
                                nfeat = element as DockFeatureBase;
                                break;
                            default:
                                notification.error({
                                    message: `Unknown type ${areaType}`,
                                });
                                setFeatures({...features}); // revert
                                return;
                        }
                    }
                    newFeatures[element.id] = nfeat;
                });

                setFeatures(newFeatures);
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
    };
}
