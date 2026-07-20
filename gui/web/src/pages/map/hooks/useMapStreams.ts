import React, { useEffect, useRef, useState } from "react";
import type { Map as MapboxMap } from "mapbox-gl";
import { useWS } from "../../../hooks/useWS.ts";
import { useHighLevelStatus } from "../../../hooks/useHighLevelStatus.ts";
import {
    AbsolutePose,
    LaserScan,
    Map as MapType,
    ObstacleArray,
    OccupancyGrid,
    Path,
    TrackedObstacle,
} from "../../../types/ros.ts";
import {
    LineFeatureBase,
    MowingFeature,
    MowerFeatureBase,
    RobotPartFeature,
    PathFeature,
    DynObstacleFeature,
} from "../../../types/map.ts";
import { drawLine, drawRobotSilhouette, transpose } from "../../../utils/map.tsx";
import { rasterizeMowProgress } from "../../../utils/mowProgress.ts";
import { useRobotDescription } from "../../../hooks/useRobotDescription.ts";

export type MowProgressImage = {
    url: string;
    coordinates: [[number, number], [number, number], [number, number], [number, number]];
};

// Rasterize the mow-progress OccupancyGrid to a Mapbox image source. The heavy
// per-cell pixel pass lives in the shared rasterizeMowProgress util (reused by
// the dashboard mini-map); this only maps the resulting canvas to Mapbox
// lon/lat corners. Invoked from a coalesced rAF, never on the WebSocket message
// handler — so a burst of grids can't stall the pump.
function renderMowProgress(
    grid: OccupancyGrid,
    offsetX: number,
    offsetY: number,
    datum: [number, number, number],
    setImage: (v: MowProgressImage | null) => void,
) {
    const raster = rasterizeMowProgress(grid);
    if (!raster) return;

    const {originX, originY, resolution} = raster;
    const gridWidth = raster.width * resolution;
    const gridHeight = raster.height * resolution;
    // Mapbox image source coords: [top-left, top-right, bottom-right, bottom-left].
    const topLeft = transpose(offsetX, offsetY, datum, originY + gridHeight, originX);
    const topRight = transpose(offsetX, offsetY, datum, originY + gridHeight, originX + gridWidth);
    const bottomRight = transpose(offsetX, offsetY, datum, originY, originX + gridWidth);
    const bottomLeft = transpose(offsetX, offsetY, datum, originY, originX);

    setImage({url: raster.dataUrl, coordinates: [topLeft, topRight, bottomRight, bottomLeft]});
}

interface UseMapStreamsOptions {
    editMap: boolean;
    settings: Record<string, string>;
    offsetX: number;
    offsetY: number;
    datum: [number, number, number];
    setFeatures: React.Dispatch<React.SetStateAction<Record<string, MowingFeature>>>;
    setEditMap: React.Dispatch<React.SetStateAction<boolean>>;
    setMapKey: React.Dispatch<React.SetStateAction<string>>;
    mapInstanceRef: React.RefObject<MapboxMap | null>;
    robotPoseRef: React.RefObject<{ x: number; y: number; heading: number } | null>;
}

export function useMapStreams({
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
}: UseMapStreamsOptions) {
    const [map, setMap] = useState<MapType | undefined>(undefined);
    const [path, setPath] = useState<Path | undefined>(undefined);
    const [plan, setPlan] = useState<Path | undefined>(undefined);
    const [lidarCollection, setLidarCollection] = useState<GeoJSON.FeatureCollection>({
        type: "FeatureCollection",
        features: [],
    });
    const [dynamicObstacles, setDynamicObstacles] = useState<TrackedObstacle[]>([]);
    // Debounce timer for tearing down the teleop joy stream. A single stray
    // non-MANUAL_MOWING/non-RECORDING frame (guard blip ahead of MainLogic) must
    // NOT kill teleop mid-drive — we only stop the joy stream after the mower has
    // stayed out of a joy-eligible state for a sustained window.
    const joyStopTimerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);

    const highLevelStatus = useHighLevelStatus();

    // Robot geometry from the /robot_description URDF — single source of truth
    // for the on-map robot shape, so it matches the sensors-page model.
    const robot = useRobotDescription();

    const poseStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const pose = (e as any) as AbsolutePose;
            const mower_lonlat = transpose(
                offsetX,
                offsetY,
                datum,
                pose.pose?.pose?.position?.y!!,
                pose.pose?.pose?.position?.x!!
            );
            robotPoseRef.current = {
                x: pose.pose?.pose?.position?.x ?? 0,
                y: pose.pose?.pose?.position?.y ?? 0,
                heading: pose.motion_heading ?? 0,
            };
            setFeatures((oldFeatures) => {
                const orientation = pose.motion_heading!!;
                const posX = pose.pose?.pose?.position?.x!!;
                const posY = pose.pose?.pose?.position?.y!!;
                const line = drawLine(offsetX, offsetY, datum, posY, posX, orientation);
                // URDF-derived robot silhouette (chassis + drive wheels + blade)
                // so the map robot matches the sensors-page model exactly.
                const sil = drawRobotSilhouette(
                    offsetX, offsetY, datum, posY, posX, orientation, robot
                );
                return {
                    ...oldFeatures,
                    mower: new MowerFeatureBase(mower_lonlat),
                    ["mower-footprint"]: new RobotPartFeature("mower-footprint", sil.chassis, "#00a6ff"),
                    ["mower-wheel-l"]: new RobotPartFeature("mower-wheel-l", sil.wheelL, "#0b2e3f"),
                    ["mower-wheel-r"]: new RobotPartFeature("mower-wheel-r", sil.wheelR, "#0b2e3f"),
                    ["mower-blade"]: new RobotPartFeature("mower-blade", sil.blade, "#ff6b6b"),
                    ["mower-heading"]: new LineFeatureBase(
                        "mower-heading",
                        [mower_lonlat, line],
                        "#ff0000",
                        "heading"
                    ),
                };
            });
        }
    );

    const mapStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const parse = (e as any) as MapType;
            setMap(parse);
            setMapKey("live");
        }
    );

    const pathStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const parse = (e as any) as Path;
            setPath(parse);
        }
    );

    const planStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const parse = (e as any) as Path;
            setPlan(parse);
        }
    );

    const joyStream = useWS<string>(
        () => {
        },
        () => {
        },
        () => {}
    );

    const lidarStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const scan = (e as any) as LaserScan;
            const pose = robotPoseRef.current;
            if (!pose || !scan.ranges) return;

            const rays: GeoJSON.Feature[] = [];
            const angleMin = scan.angle_min ?? 0;
            const angleInc = scan.angle_increment ?? 0;
            const rangeMin = scan.range_min ?? 0;
            const rangeMax = scan.range_max ?? 12;

            // Scan rays live in the lidar_link frame, which is mounted on the
            // chassis with a static base_footprint→lidar_link transform
            // (lidar_x/y forward+lateral offset, lidar_yaw heading offset — see
            // mowgli_robot.yaml). Compose that mount transform with the robot
            // pose so points land at their true map position instead of being
            // drawn as if the lidar sat at base_footprint with zero yaw.
            const lidarX = parseFloat(settings["lidar_x"]) || 0;
            const lidarY = parseFloat(settings["lidar_y"]) || 0;
            const lidarYaw = parseFloat(settings["lidar_yaw"]) || 0;
            const cosH = Math.cos(pose.heading);
            const sinH = Math.sin(pose.heading);

            // Downsample: take every Nth point for performance
            const step = Math.max(1, Math.floor(scan.ranges.length / 90));
            for (let i = 0; i < scan.ranges.length; i += step) {
                const range = scan.ranges[i];
                if (range < rangeMin || range > rangeMax) continue;

                // Point in the lidar frame (lidar_yaw folded into the ray angle).
                const angle = angleMin + i * angleInc + lidarYaw;
                const px = range * Math.cos(angle);
                const py = range * Math.sin(angle);
                // lidar_link → base_footprint (rotate by lidar_yaw, translate by mount offset).
                const bx = lidarX + px;
                const by = lidarY + py;
                // base_footprint → map (rotate by robot heading, translate by pose).
                const endX = pose.x + bx * cosH - by * sinH;
                const endY = pose.y + bx * sinH + by * cosH;
                const endLonLat = transpose(offsetX, offsetY, datum, endY, endX);

                rays.push({
                    type: "Feature",
                    properties: { intensity: range < rangeMax * 0.8 ? "hit" : "far" },
                    geometry: {
                        type: "Point",
                        coordinates: endLonLat,
                    },
                });
            }
            setLidarCollection({
                type: "FeatureCollection",
                features: rays,
            });
        }
    );

    const obstaclesStream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            const parsed = (e as any) as ObstacleArray;
            if (parsed.obstacles) {
                // Only show persistent obstacles (status=1)
                setDynamicObstacles(parsed.obstacles.filter(o => o.status === 1));

                // Render obstacle polygons on the map
                setFeatures((oldFeatures) => {
                    const newFeatures = { ...oldFeatures };
                    // Remove old dynamic obstacle features
                    Object.keys(newFeatures).forEach(k => {
                        if (k.startsWith("dyn-obs-")) delete newFeatures[k];
                    });
                    // Add current obstacles as semi-transparent polygons
                    (parsed.obstacles ?? []).filter(o => o.status === 1).forEach((obs) => {
                        if (obs.polygon?.points && obs.polygon.points.length >= 3) {
                            const coords = obs.polygon.points.map(p =>
                                transpose(offsetX, offsetY, datum, p.y ?? 0, p.x ?? 0)
                            );
                            // Close the polygon ring (GeoJSON requires first == last)
                            coords.push(coords[0]);
                            newFeatures["dyn-obs-" + obs.id] = new DynObstacleFeature(
                                "dyn-obs-" + obs.id,
                                coords,
                                obs.id ?? 0
                            );
                        }
                    });
                    return newFeatures;
                });
            }
        }
    );

    // Mow-progress overlay: the latest grid waits in a ref and is rasterized at
    // most once per animation frame (the raster + toDataURL is too heavy to run
    // on the WebSocket message handler — it would stall pose/lidar frames).
    const [mowProgressImage, setMowProgressImage] = useState<MowProgressImage | null>(null);
    const mowProgressPendingRef = React.useRef<
        { grid: OccupancyGrid; offsetX: number; offsetY: number; datum: [number, number, number] } | null
    >(null);
    const mowProgressRafRef = React.useRef<number | null>(null);
    const mowProgressStream = useWS<string>(
        () => {},
        () => {},
        (e) => {
            const grid = (e as any) as OccupancyGrid;
            if (!grid.info || !grid.data) return;
            if ((grid.info.width ?? 0) === 0 || (grid.info.height ?? 0) === 0) return;
            mowProgressPendingRef.current = { grid, offsetX, offsetY, datum };
            if (mowProgressRafRef.current == null) {
                mowProgressRafRef.current = requestAnimationFrame(() => {
                    mowProgressRafRef.current = null;
                    const pending = mowProgressPendingRef.current;
                    mowProgressPendingRef.current = null;
                    if (!pending) return;
                    renderMowProgress(pending.grid, pending.offsetX, pending.offsetY, pending.datum, setMowProgressImage);
                });
            }
        }
    );

    const recordingTrajectoryStream = useWS<string>(
        () => {
        },
        () => {
        },
        (e) => {
            const path = (e as any) as Path;
            if (!path.poses || path.poses.length === 0) {
                // Recording cleared — remove trajectory feature
                setFeatures((oldFeatures) => {
                    const newFeatures = { ...oldFeatures };
                    delete newFeatures["recording-trajectory"];
                    return newFeatures;
                });
                return;
            }
            // Draw the recording trajectory as a line on the map
            const coords = path.poses.map(p =>
                transpose(offsetX, offsetY, datum, p.pose?.position?.y ?? 0, p.pose?.position?.x ?? 0)
            );
            setFeatures((oldFeatures) => ({
                ...oldFeatures,
                ["recording-trajectory"]: new PathFeature(
                    "recording-trajectory",
                    coords,
                    "#ff6600",
                    2,
                ),
            }));
        }
    );

    // Keep lidar layer on top of draw layers
    useEffect(() => {
        const m = mapInstanceRef.current;
        if (!m) return;
        try {
            if (m.getLayer("lidar-points")) {
                m.moveLayer("lidar-points");
            }
        } catch { /* layer may not exist yet */ }
    }, [lidarCollection]);

    // Start/stop streams when editMap changes
    useEffect(() => {
        if (editMap) {
            mapStream.stop();
            poseStream.stop();
            pathStream.stop();
            planStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            recordingTrajectoryStream.stop();
            highLevelStatus.stop();
            setPath(undefined);
            setPlan(undefined);
            setLidarCollection({ type: "FeatureCollection", features: [] });
        } else {
            if (
                settings["datum_lon"] == undefined ||
                settings["datum_lat"] == undefined
            ) {
                return;
            }
            highLevelStatus.start("/api/mowglinext/subscribe/highLevelStatus");
            poseStream.start("/api/mowglinext/subscribe/pose");
            mapStream.start("/api/mowglinext/subscribe/map");
            pathStream.start("/api/mowglinext/subscribe/path");
            planStream.start("/api/mowglinext/subscribe/plan");
            lidarStream.start("/api/mowglinext/subscribe/lidar");
            obstaclesStream.start("/api/mowglinext/subscribe/obstacles");
            mowProgressStream.start("/api/mowglinext/subscribe/mowProgress");
        }
    }, [editMap]);

    // Start joy + recording trajectory streams on RECORDING state
    useEffect(() => {
        const stateName = highLevelStatus.highLevelStatus.state_name;
        if (stateName === "RECORDING") {
            clearTimeout(joyStopTimerRef.current);
            joyStopTimerRef.current = undefined;
            joyStream.start("/api/mowglinext/publish/joy");
            recordingTrajectoryStream.start("/api/mowglinext/subscribe/recordingTrajectory");
            setEditMap(false);
            return;
        }
        if (stateName === "MANUAL_MOWING") {
            clearTimeout(joyStopTimerRef.current);
            joyStopTimerRef.current = undefined;
            joyStream.start("/api/mowglinext/publish/joy");
            return;
        }
        // Leaving a joy-eligible state: DEBOUNCE the joy teardown so a single
        // stray guard frame (EMERGENCY/battery/boundary blip) can't kill teleop
        // mid-drive. The recording-trajectory cleanup can happen immediately —
        // it's only visual and re-subscribes instantly if RECORDING returns.
        if (joyStopTimerRef.current === undefined) {
            joyStopTimerRef.current = setTimeout(() => {
                joyStopTimerRef.current = undefined;
                joyStream.stop();
            }, 1200);
        }
        recordingTrajectoryStream.stop();
        // Clear trajectory feature when leaving recording mode
        setFeatures((oldFeatures) => {
            const newFeatures = { ...oldFeatures };
            delete newFeatures["recording-trajectory"];
            return newFeatures;
        });
    }, [highLevelStatus.highLevelStatus.state_name]);

    // Clear the joy-stop debounce on unmount so it can't fire after teardown.
    useEffect(() => {
        return () => clearTimeout(joyStopTimerRef.current);
    }, []);

    // Start streams once the datum is available. Keyed on the datum values
    // ONLY — not the whole `settings` object. The previous `[settings]`
    // dependency re-ran on every settings-object identity change (each poll /
    // partial merge creates a new object), tearing down and re-subscribing all
    // eight streams each time. That re-subscribe storm churned the backend
    // RosSubscribers and left components briefly without data ("stale").
    useEffect(() => {
        if (
            settings["datum_lon"] == undefined ||
            settings["datum_lat"] == undefined
        ) {
            return;
        }
        highLevelStatus.start("/api/mowglinext/subscribe/highLevelStatus");
        poseStream.start("/api/mowglinext/subscribe/pose");
        mapStream.start("/api/mowglinext/subscribe/map");
        pathStream.start("/api/mowglinext/subscribe/path");
        planStream.start("/api/mowglinext/subscribe/plan");
        lidarStream.start("/api/mowglinext/subscribe/lidar");
        obstaclesStream.start("/api/mowglinext/subscribe/obstacles");
        mowProgressStream.start("/api/mowglinext/subscribe/mowProgress");
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [settings["datum_lon"], settings["datum_lat"]]);

    // Cleanup all streams on unmount
    useEffect(() => {
        return () => {
            poseStream.stop();
            mapStream.stop();
            pathStream.stop();
            joyStream.stop();
            planStream.stop();
            lidarStream.stop();
            obstaclesStream.stop();
            mowProgressStream.stop();
            if (mowProgressRafRef.current != null) {
                cancelAnimationFrame(mowProgressRafRef.current);
                mowProgressRafRef.current = null;
            }
            recordingTrajectoryStream.stop();
            highLevelStatus.stop();
        };
    }, []);

    return {
        map,
        dynamicObstacles,
        setMap,
        path,
        plan,
        lidarCollection,
        mowProgressImage,
        highLevelStatus,
        joyStream,
    };
}
