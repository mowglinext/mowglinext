import {assemblyBounds} from "./robot/assemblyBounds";
import {LayeredMower, SensorGraphic} from "./robot/LayeredMower";
import {MowerVisualControls, MowerView} from "./robot/MowerPreview";
import {previewRobotGeometry, type Bounds} from "../utils/robotModel";
import {useMowerVisual} from "../hooks/useMowerVisual";
import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useTranslation } from "react-i18next";
import { Alert, App, Card, InputNumber, Modal, Space, Typography, Row, Col, Tooltip, Button, Tag } from "antd";
import { AimOutlined, CompassOutlined, EnvironmentOutlined, UndoOutlined } from "@ant-design/icons";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { getColors, inkAlpha } from "../theme/colors.ts";
import { useIsMobile } from "../hooks/useIsMobile.ts";
import { useRobotDescription } from "../hooks/useRobotDescription.ts";
import { useCalibrationStatus } from "../hooks/useCalibrationStatus.ts";
import { useImuYawCalibration } from "../hooks/useImuYawCalibration.ts";
import { useApi } from "../hooks/useApi.ts";
import { getQuaternionFromHeading } from "../utils/map.tsx";
import { usePose } from "../hooks/usePose.ts";
import { useStatus } from "../hooks/useStatus.ts";

const { Text } = Typography;


type SensorId = "lidar" | "imu" | "gps";

type SensorConfig = {
    x: number;
    y: number;
    yaw: number;
    z: number;
};

type SensorMeta = {
    id: SensorId;
    label: string;
    color: string;
    colorDark: string;
    shape: "circle" | "rect";
    size: number; // metres
    xKey: string;
    yKey: string;
    yawKey: string;
    zKey: string;
};

// Sensor palette is sourced from the brand tokens (no raw hex). `color` is the
// light-mode tint, `colorDark` the brighter dark-mode variant used on the deep
// canvas. LiDAR = danger (rose), IMU = info/sky (aurora cyan), GPS = primary
// (lime).
const LIGHT_TOKENS = getColors("light");
const DARK_TOKENS = getColors("dark");

const SENSORS: SensorMeta[] = [
    {
        id: "lidar",
        label: "LiDAR",
        color: LIGHT_TOKENS.danger,
        colorDark: DARK_TOKENS.danger,
        shape: "circle",
        size: 0.04,
        xKey: "lidar_x",
        yKey: "lidar_y",
        yawKey: "lidar_yaw",
        zKey: "lidar_z",
    },
    {
        id: "imu",
        label: "IMU",
        color: LIGHT_TOKENS.info,
        colorDark: DARK_TOKENS.sky,
        shape: "rect",
        size: 0.03,
        xKey: "imu_x",
        yKey: "imu_y",
        yawKey: "imu_yaw",
        zKey: "imu_z",
    },
    {
        id: "gps",
        label: "GPS",
        color: LIGHT_TOKENS.primary,
        colorDark: DARK_TOKENS.primary,
        shape: "rect",
        size: 0.05,
        xKey: "gps_x",
        yKey: "gps_y",
        yawKey: "",
        zKey: "gps_z",
    },
];

// Convert robot metres to SVG coordinates.
// Robot frame (REP-103): X = forward, Y = left.
// SVG is a top-down view with the robot facing UP (matching RobotAnatomy and
// the URDF's natural orientation): +X (forward) → up, +Y (left) → left. Drawing
// the robot facing right made the drive-wheel discs — whose diameter runs along
// the rolling/forward axis — appear as horizontal bars (issue #404).
const roundTo = (v: number, decimals: number): number => {
    const f = Math.pow(10, decimals);
    return Math.round(v * f) / f;
};

const radToDeg = (r: number): number => (r * 180) / Math.PI;
const degToRad = (d: number): number => (d * Math.PI) / 180;

// dock_pose_yaw is stored as ROS/ENU yaw in radians (0 = +X / East,
// CCW positive). Operators set the dock with a real compass, which
// reads bearings in 0–360° clockwise from North. These helpers translate
// between the two so the UI can speak compass while the YAML/firmware
// keeps speaking ROS yaw.
const yawRadToCompassBearing = (yawRad: number): number => {
    const yawDeg = radToDeg(yawRad);
    return ((90 - yawDeg) % 360 + 360) % 360;
};
const compassBearingToYawRad = (bearing: number): number => {
    return degToRad(90 - bearing);
};

type Props = {
    values: Record<string, any>;
    onChange: (name: string, value: any) => void;
};

export const RobotComponentEditor: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { colors, mode } = useThemeMode();
    const isMobile = useIsMobile();
    const svgRef = useRef<SVGSVGElement>(null);
    const [dragging, setDragging] = useState<SensorId | null>(null);
    const [rotating, setRotating] = useState<SensorId | null>(null);
    const [hoveredSensor, setHoveredSensor] = useState<SensorId | null>(null);
    const { status: calibrationStatus } = useCalibrationStatus();
    const { notification, modal } = App.useApp();
    const guiApi = useApi();
    const pose = usePose();
    const hwStatus = useStatus();
    const [settingDock, setSettingDock] = useState(false);

    // Pull the robot's current map-frame pose. Yaw is the EKF-fused
    // motion_heading on AbsolutePose. We capture all three (x, y, yaw)
    // because the operator's mental model when clicking "Set dock from
    // current pose" is "treat where the robot is sitting right now as the
    // canonical dock position and orientation".
    const robotX = pose.pose?.pose?.position?.x;
    const robotY = pose.pose?.pose?.position?.y;
    const robotYaw = pose.motion_heading;
    const poseAvailable = robotX != null && robotY != null && robotYaw != null;
    // Require physical charging contact when calibrating the dock pose.
    // Without this guard, operators sometimes clicked "Set dock pose"
    // while the robot was close-but-not-charging (visually parked on the
    // dock, but the contacts hadn't seated). The stored dock_pose then
    // pointed to that off-contact location, and every subsequent
    // re-docking aimed there — robot stopping 1-3 cm short of the
    // charging cradle on every run. By gating the action on
    // is_charging=true we guarantee the captured pose is the exact
    // physical contact point, removing one source of dock-approach drift.
    const isCharging = !!hwStatus.is_charging;

    const writeDockPose = useCallback(
        async (px: number, py: number, yaw: number) => {
            // "Capture current robot position": the robot is physically on the
            // dock. use_gps_position=true tells map_server to capture the dock
            // POSITION from the averaged independent GPS projection, NOT the px/py
            // we send (which come from the fused pose — gauge-reset onto the old
            // dock_pose while charging, so it would be circular).
            //
            // Task #45 (from #44's circularity trace): the SAME gauge-reset
            // circularity poisons the fused YAW while charging just as much as
            // position — fusion_graph pins the fused yaw to the EXISTING
            // dock_pose_yaw, so it can never self-correct via a live capture.
            // map_server's on_set_docking_point now PRESERVES the existing
            // dock_pose_yaw whenever use_gps_position=true and ignores the
            // orientation we send below — we still send a quaternion because the
            // service request shape requires one, but it is a no-op server-side
            // for this call. Dock heading only changes via the motion-derived
            // calibration (calibrate_imu_yaw_node's dock-yaw drive, or the
            // per-undock refinement) — see the "Run Calibration" action.
            //
            // Because the stored position is the GPS one (not our px/py), we must
            // NOT optimistically push px/py into the form — that was the bug: it
            // showed the fused value and a later Settings "Save" would overwrite
            // the GPS value. Instead, after the service persists, read the actual
            // stored dock pose back from /calibration/status (which reads
            // mowgli_robot.yaml) and reflect THAT in the form — this also
            // correctly reflects the now-unchanged yaw.
            const q = getQuaternionFromHeading(yaw);
            try {
                setSettingDock(true);
                await guiApi.mowglinext.mapDockingCreate({
                    docking_pose: {
                        orientation: {x: q.x!, y: q.y!, z: q.z!, w: q.w!},
                        position: {x: px, y: py, z: 0},
                    },
                    use_gps_position: true,
                });
                // Read back the value the service actually stored (GPS-averaged),
                // not the fused px/py we sent.
                let storedX = px;
                let storedY = py;
                let storedYaw = yaw;
                try {
                    const resp = await guiApi.request<{
                        dock?: {dock_pose_x?: number; dock_pose_y?: number; dock_pose_yaw_rad?: number};
                    }>({path: "/calibration/status", method: "GET", format: "json"});
                    const d = resp.data?.dock;
                    if (d?.dock_pose_x != null) storedX = d.dock_pose_x;
                    if (d?.dock_pose_y != null) storedY = d.dock_pose_y;
                    if (d?.dock_pose_yaw_rad != null) storedYaw = d.dock_pose_yaw_rad;
                } catch {
                    // Read-back failed; fall back to the sent values for the form.
                    // The service still persisted the GPS value to yaml.
                }
                onChange("dock_pose_x", roundTo(storedX, 3));
                onChange("dock_pose_y", roundTo(storedY, 3));
                onChange("dock_pose_yaw", roundTo(storedYaw, 4));
                notification.success({
                    message: t("robotComponentEditor.dockPoseSetFromGps"),
                    description: t("robotComponentEditor.dockPoseSetFromGpsDescription"),
                });
            } catch (e: unknown) {
                const message = e instanceof Error ? e.message : t("robotComponentEditor.unknownError");
                notification.error({message: t("robotComponentEditor.failedToSetDockPose"), description: message});
            } finally {
                setSettingDock(false);
            }
        },
        [guiApi, notification, onChange, t],
    );

    // Open a confirmation modal that previews the change and spells out
    // exactly what gets written. The previous tiny icon-only button buried
    // this action without a confirm step (#173).
    const handleSetDockAtRobot = useCallback(() => {
        if (!poseAvailable) {
            notification.warning({message: t("robotComponentEditor.noRobotPoseYet")});
            return;
        }
        const px = robotX;
        const py = robotY;
        const yawRad = robotYaw;
        const yawDeg = roundTo(yawRadToCompassBearing(yawRad), 1);
        const savedX = values.dock_pose_x;
        const savedY = values.dock_pose_y;
        const savedYawRad = values.dock_pose_yaw;
        const fmt = (v: any, suffix: string, digits = 3) =>
            v == null || isNaN(Number(v)) ? "—" : `${roundTo(Number(v), digits)}${suffix}`;

        // Use the App-context modal (App.useApp().modal) instead of the
        // static Modal.confirm — antd v5 deprecated the static API and it
        // renders without inheriting the ConfigProvider theme, which on
        // dark mode shows a white modal on white backdrop and looks like
        // 'click did nothing'. The App-context variant inherits the
        // current theme tokens, z-index stack, and message portal so the
        // confirm dialog actually appears.
        modal.confirm({
            title: (
                <Space>
                    <EnvironmentOutlined/>
                    <span>{t("robotComponentEditor.setDockFromPoseTitle")}</span>
                </Space>
            ),
            width: 520,
            okText: t("robotComponentEditor.setDockPose"),
            okType: "primary",
            cancelText: t("robotComponentEditor.cancel"),
            content: (
                <div>
                    <Typography.Paragraph>
                        {t("robotComponentEditor.setDockConfirmIntro1")}{" "}
                        <strong>{t("robotComponentEditor.setDockConfirmXYHeading")}</strong>{" "}
                        {t("robotComponentEditor.setDockConfirmIntro2")}{" "}
                        <code>mowgli_robot.yaml</code>{" "}
                        {t("robotComponentEditor.setDockConfirmIntro3")}
                    </Typography.Paragraph>
                    <Card size="small" style={{marginBottom: 8}}>
                        <Row gutter={[8, 4]}>
                            <Col span={8}><Text type="secondary" style={{fontSize: 11}}>{t("robotComponentEditor.currentRobot")}</Text></Col>
                            <Col span={5}>x: <strong>{fmt(px, " m")}</strong></Col>
                            <Col span={5}>y: <strong>{fmt(py, " m")}</strong></Col>
                            <Col span={6}>{t("robotComponentEditor.bearing")}: <strong>{yawDeg}°</strong></Col>
                        </Row>
                        <Row gutter={[8, 4]} style={{marginTop: 4}}>
                            <Col span={8}><Text type="secondary" style={{fontSize: 11}}>{t("robotComponentEditor.savedDock")}</Text></Col>
                            <Col span={5}>x: <strong>{fmt(savedX, " m")}</strong></Col>
                            <Col span={5}>y: <strong>{fmt(savedY, " m")}</strong></Col>
                            <Col span={6}>{t("robotComponentEditor.bearing")}:{" "}
                                <strong>{savedYawRad == null ? "—" : `${roundTo(yawRadToCompassBearing(savedYawRad), 1)}°`}</strong>
                            </Col>
                        </Row>
                    </Card>
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginBottom: 0}}>
                        {t("robotComponentEditor.setDockConfirmFooter")}
                    </Typography.Paragraph>
                </div>
            ),
            onOk: () => writeDockPose(px, py, yawRad),
        });
    }, [poseAvailable, robotX, robotY, robotYaw, values.dock_pose_x, values.dock_pose_y, values.dock_pose_yaw, writeDockPose, notification, modal, t]);
    // Dock yaw lives in mowgli_robot.yaml. It is normally written by the
    // IMU auto-calibration service and the "set dock pose" GUI action,
    // but operators can also override it manually here when calibration
    // is unavailable or wrong.
    const dockCal = calibrationStatus?.dock;
    const dockYawRad = values.dock_pose_yaw ?? (dockCal?.present && dockCal.dock_pose_yaw_rad != null
        ? dockCal.dock_pose_yaw_rad
        : 0);
    const dockYawSource = "mowgli_robot.yaml";

    // Shared IMU yaw + dock pose calibration logic. The same hook backs
    // the OnboardingPage's ImuYawStep, so the two surfaces cannot drift
    // (same fetch URL, same 155 s timeout, same 150-sample threshold for
    // promoting pitch/roll, same notifications).
    const {
        calibOpen,
        calibRunning,
        calibResult,
        openCalibration,
        closeCalibration,
        resetCalibration,
        startCalibration,
        applyCalibration,
    } = useImuYawCalibration({
        onApplyValue: onChange,
        currentImuYawRad: values.imu_yaw,
    });

    const liveRobot = useRobotDescription();
    const robot = useMemo(() => previewRobotGeometry(liveRobot, values), [liveRobot, values]);
    const [visual] = useMowerVisual();
    const [dragBounds, setDragBounds] = useState<Bounds | null>(null);
    const bounds = dragBounds ?? assemblyBounds(robot, "top", .12);
    const svgWidth = isMobile ? 340 : 520;
    const svgHeight = isMobile ? 380 : 480;
    const [svgUnitsPerPixel, setSvgUnitsPerPixel] = useState(1);
    useEffect(() => {
        const svg=svgRef.current;
        if (!svg) return;
        const update=()=>{
            const width=svg.getBoundingClientRect().width;
            if (width>0) setSvgUnitsPerPixel(svgWidth/width);
        };
        update();
        const observer=new ResizeObserver(update);
        observer.observe(svg);
        return ()=>observer.disconnect();
    }, [svgWidth]);
    const SCALE = Math.min(svgWidth/bounds.width, svgHeight/bounds.height);
    const cx = (svgWidth-bounds.width*SCALE)/2-bounds.x*SCALE;
    const cy = (svgHeight-bounds.height*SCALE)/2-bounds.y*SCALE;
    const toSvg = useCallback((rx: number, ry: number, cx: number, cy: number): [number, number] => {
        return [cx - ry * SCALE, cy - rx * SCALE];
    }, [SCALE]);

    const fromSvg = useCallback((sx: number, sy: number, cx: number, cy: number): [number, number] => {
        return [(cy - sy) / SCALE, (cx - sx) / SCALE];
    }, [SCALE]);

    const getSensorValue = useCallback(
        (meta: SensorMeta): SensorConfig => {
            const sensor=robot.sensors?.find(s=>s.id === meta.id);
            const mount=sensor?.mount ?? sensor;
            return {
                x: values[meta.xKey] ?? mount?.x ?? 0,
                y: values[meta.yKey] ?? mount?.y ?? 0,
                yaw: meta.yawKey ? (values[meta.yawKey] ?? sensor?.yaw ?? 0) : 0,
                z: values[meta.zKey] ?? mount?.z ?? 0,
            };
        },
        [values, robot.sensors]
    );

    const handlePointerDown = useCallback(
        (sensorId: SensorId, e: React.MouseEvent | React.TouchEvent) => {
            e.preventDefault();
            e.stopPropagation();
            setDragBounds(bounds);
            setDragging(sensorId);
        },
        [bounds]
    );

    const handleRotateDown = useCallback(
        (sensorId: SensorId, e: React.MouseEvent | React.TouchEvent) => {
            e.preventDefault();
            e.stopPropagation();
            setDragBounds(bounds);
            setRotating(sensorId);
        },
        [bounds]
    );

    useEffect(() => {
        if (!dragging && !rotating) return;

        const meta = SENSORS.find((s) => s.id === (dragging || rotating))!;

        const handleMove = (clientX: number, clientY: number) => {
            const svg = svgRef.current;
            if (!svg) return;
            const rect = svg.getBoundingClientRect();
            const sx = (clientX - rect.left) * svgWidth / rect.width;
            const sy = (clientY - rect.top) * svgHeight / rect.height;

            if (dragging) {
                const [rx, ry] = fromSvg(sx, sy, cx, cy);
                const clampedX = roundTo(Math.max(-bounds.y-bounds.height, Math.min(-bounds.y, rx)), 3);
                const clampedY = roundTo(Math.max(-bounds.x-bounds.width, Math.min(-bounds.x, ry)), 3);
                onChange(meta.xKey, clampedX);
                onChange(meta.yKey, clampedY);
            }

            if (rotating && meta.yawKey) {
                const sensorVal = getSensorValue(meta);
                const [ssx, ssy] = toSvg(sensorVal.x, sensorVal.y, cx, cy);
                // Facing-up frame: yaw is CCW from +X (up). A pointer offset
                // (dx, dy) in SVG maps back to robot X = -dy, Y = -dx, so
                // yaw = atan2(-dx, -dy) = atan2(ssx - sx, ssy - sy).
                const angle = Math.atan2(ssx - sx, ssy - sy);
                onChange(meta.yawKey, roundTo(angle, 4));
            }
        };

        const onMouseMove = (e: MouseEvent) => handleMove(e.clientX, e.clientY);
        const onTouchMove = (e: TouchEvent) => {
            e.preventDefault();
            handleMove(e.touches[0].clientX, e.touches[0].clientY);
        };
        const onUp = () => {
            setDragBounds(null);
            setDragging(null);
            setRotating(null);
        };

        window.addEventListener("mousemove", onMouseMove);
        window.addEventListener("mouseup", onUp);
        window.addEventListener("touchmove", onTouchMove, { passive: false });
        window.addEventListener("touchend", onUp);

        return () => {
            window.removeEventListener("mousemove", onMouseMove);
            window.removeEventListener("mouseup", onUp);
            window.removeEventListener("touchmove", onTouchMove);
            window.removeEventListener("touchend", onUp);
        };
    }, [dragging, rotating, cx, cy, SCALE, svgWidth, svgHeight, bounds, onChange, getSensorValue, fromSvg, toSvg]);

    // Grid lines
    const gridLines = useMemo(() => {
        const lines: React.ReactNode[] = [];
        const gridStep = 0.05; // 5cm grid
        const range = 0.35;
        const gridColor = mode === "dark" ? "rgba(255,255,255,0.06)" : "rgba(0,0,0,0.06)";
        const axisColor = mode === "dark" ? "rgba(255,255,255,0.15)" : "rgba(0,0,0,0.12)";

        for (let v = -range; v <= range + 0.001; v += gridStep) {
            const r = roundTo(v, 3);
            const color = Math.abs(r) < 0.001 ? axisColor : gridColor;
            const [x1, y1] = toSvg(-range, r, cx, cy);
            const [x2, y2] = toSvg(range, r, cx, cy);
            lines.push(
                <line key={`h${r}`} x1={x1} y1={y1} x2={x2} y2={y2} stroke={color} strokeWidth={Math.abs(r) < 0.001 ? 1.5 : 0.5} />
            );
            const [x3, y3] = toSvg(r, -range, cx, cy);
            const [x4, y4] = toSvg(r, range, cx, cy);
            lines.push(
                <line key={`v${r}`} x1={x3} y1={y3} x2={x4} y2={y4} stroke={color} strokeWidth={Math.abs(r) < 0.001 ? 1.5 : 0.5} />
            );
        }
        return lines;
    }, [cx, cy, SCALE, mode, toSvg]);

    const robotBody = liveRobot.fromUrdf ? <g transform={`translate(${cx} ${cy}) scale(${SCALE})`}>
        <LayeredMower robot={robot} {...visual} sensors={false}/>
        <path d="M -.01 0 H .01 M 0 -.01 V .01" stroke="#a8ccbe" strokeWidth={.002}/>
    </g> : null;

    // Draw a single sensor
    const renderSensor = useCallback(
        (meta: SensorMeta) => {
            const val = getSensorValue(meta);
            const [sx, sy] = toSvg(val.x, val.y, cx, cy);
            const sizeInPx = meta.size * SCALE;
            const isActive = dragging === meta.id || rotating === meta.id;
            const isHovered = hoveredSensor === meta.id;
            const sensorColor = mode === "dark" ? meta.colorDark : meta.color;

            // Facing-up frame: a heading yaw θ (CCW from +X/up) points along the
            // robot vector (cos θ, sin θ) in (X, Y), which projects to the SVG
            // delta (-sin θ, -cos θ).
            const yawLineLen = 0.06 * SCALE;
            const yawEndX = sx - Math.sin(val.yaw) * yawLineLen;
            const yawEndY = sy - Math.cos(val.yaw) * yawLineLen;

            // Keep 48 CSS-pixel hit targets disjoint even when a large robot
            // or outboard sensor makes the automatically fitted model small.
            const HIT_R = 24 * svgUnitsPerPixel;
            const handleDist = Math.max(0.08 * SCALE, 2 * HIT_R + 8 * svgUnitsPerPixel);
            const handleX = sx - Math.sin(val.yaw) * handleDist;
            const handleY = sy - Math.cos(val.yaw) * handleDist;

            // Transparent ≥44px touch target behind the small visible glyph so
            // dragging works on phones (Apple HIG / Material both call for 44px
            // minimum tap targets). The visible square/circle stays small.
            const handleFill = mode === "dark" ? colors.bgElevated : colors.bgCard;

            return (
                <g key={meta.id} data-sensor-control={meta.id}>
                    {(isActive || isHovered) && (
                        <circle
                            cx={sx} cy={sy} r={sizeInPx + 8}
                            fill="none" stroke={sensorColor} strokeWidth={2}
                            strokeDasharray="4 3" opacity={0.5}
                        />
                    )}

                    {/* Invisible drag hit-area (44px+) */}
                    <circle
                        cx={sx} cy={sy} r={HIT_R}
                        fill="transparent"
                        style={{ cursor: "grab" }}
                        onMouseDown={(e) => handlePointerDown(meta.id, e)}
                        onTouchStart={(e) => handlePointerDown(meta.id, e)}
                        onMouseEnter={() => setHoveredSensor(meta.id)}
                        onMouseLeave={() => setHoveredSensor(null)}
                    />

                    <g transform={`translate(${cx} ${cy}) scale(${SCALE})`} style={{pointerEvents:"none"}}
                        opacity={meta.id === "imu" && !visual.transparent ? .45 : 1}>
                        {robot.sensors?.filter(sensor=>sensor.id === meta.id).map(sensor=>
                            <SensorGraphic key={sensor.id} sensor={sensor} view="top" highlight/>)}
                    </g>

                    {meta.yawKey && (
                        <>
                            <line
                                x1={sx} y1={sy} x2={yawEndX} y2={yawEndY}
                                stroke={sensorColor} strokeWidth={2}
                                style={{ pointerEvents: "none" }}
                            />
                            <polygon
                                points={(() => {
                                    const as = 5;
                                    // Arrowhead from the on-screen line direction,
                                    // independent of the frame projection.
                                    const ang = Math.atan2(yawEndY - sy, yawEndX - sx);
                                    const p1x = yawEndX - as * Math.cos(ang - 0.4);
                                    const p1y = yawEndY - as * Math.sin(ang - 0.4);
                                    const p2x = yawEndX - as * Math.cos(ang + 0.4);
                                    const p2y = yawEndY - as * Math.sin(ang + 0.4);
                                    return `${yawEndX},${yawEndY} ${p1x},${p1y} ${p2x},${p2y}`;
                                })()}
                                fill={sensorColor}
                                style={{ pointerEvents: "none" }}
                            />
                            {/* Invisible rotate hit-area (44px+) */}
                            <circle
                                cx={handleX} cy={handleY} r={HIT_R}
                                fill="transparent"
                                style={{ cursor: "crosshair" }}
                                onMouseDown={(e) => handleRotateDown(meta.id, e)}
                                onTouchStart={(e) => handleRotateDown(meta.id, e)}
                            />
                            <circle
                                cx={handleX} cy={handleY} r={6}
                                fill={handleFill}
                                stroke={sensorColor} strokeWidth={2}
                                style={{ cursor: "crosshair", pointerEvents: "none" }}
                            />
                        </>
                    )}

                    <text
                        x={sx} y={sy - sizeInPx - 8}
                        textAnchor="middle" fontSize={11} fontWeight="bold"
                        fill={sensorColor}
                        style={{ pointerEvents: "none", userSelect: "none" }}
                    >
                        {meta.label}
                    </text>
                </g>
            );
        },
        [cx, cy, SCALE, svgUnitsPerPixel, robot, visual.transparent, dragging, rotating, hoveredSensor, getSensorValue, handlePointerDown, handleRotateDown, mode, colors, toSvg]
    );

    // Scale labels
    const scaleLabels = useMemo(() => {
        const labels: React.ReactNode[] = [];
        const labelColor = mode === "dark" ? "rgba(255,255,255,0.25)" : "rgba(0,0,0,0.25)";
        const step = 0.10;
        // Forward (X) axis runs vertically → label down the left margin.
        // Lateral (Y) axis runs horizontally → label along the bottom margin.
        const leftX = cx - robot.baseWidth / 2 * SCALE - 12;
        const bottomY = cy + (robot.baseLength / 2 - robot.chassisCenterX) * SCALE + 18;
        for (let v = -0.3; v <= 0.3 + 0.001; v += step) {
            const r = roundTo(v, 2);
            if (Math.abs(r) < 0.001) continue;
            const [, ly] = toSvg(r, 0, cx, cy);
            labels.push(
                <text key={`xl${r}`} x={leftX} y={ly + 3} textAnchor="end" fontSize={8} fill={labelColor} fontFamily="monospace">
                    {r.toFixed(1)}m
                </text>
            );
            const [lx] = toSvg(0, r, cx, cy);
            labels.push(
                <text key={`yl${r}`} x={lx} y={bottomY} textAnchor="middle" fontSize={8} fill={labelColor} fontFamily="monospace">
                    {r.toFixed(1)}m
                </text>
            );
        }
        return labels;
    }, [cx, cy, SCALE, mode, robot, toSvg]);

    const resetSensor = useCallback(
        (meta: SensorMeta) => {
            const defaults: Record<string, number> = {
                lidar_x: 0.38, lidar_y: 0, lidar_z: 0.22, lidar_yaw: 0,
                imu_x: 0.18, imu_y: 0, imu_z: 0.095, imu_yaw: 0,
                gps_x: 0.3, gps_y: 0, gps_z: 0.2,
            };
            onChange(meta.xKey, defaults[meta.xKey] ?? 0);
            onChange(meta.yKey, defaults[meta.yKey] ?? 0);
            if (meta.yawKey) onChange(meta.yawKey, defaults[meta.yawKey] ?? 0);
            onChange(meta.zKey, defaults[meta.zKey] ?? 0);
        },
        [onChange]
    );

    return (
        <Card data-testid="sensor-placement"
            title={
                <Space>
                    <AimOutlined />
                    <span>{t("robotComponentEditor.sensorPlacement")}</span>
                    <Tag color="blue" style={{ fontSize: 10, marginLeft: 4 }}>
                        {robot.baseLength.toFixed(2)} x {robot.baseWidth.toFixed(2)} m
                    </Tag>
                </Space>
            }
            style={{ marginBottom: 16 }}
        >
            <Typography.Paragraph type="secondary" style={{ marginBottom: 16 }}>
                {t("robotComponentEditor.dragSensorsHint")}
            </Typography.Paragraph>

            <MowerVisualControls/>
            <Typography.Paragraph type="secondary">{t(liveRobot.fromUrdf ? "mowerVisual.previewNote" : "mowerVisual.waiting")}</Typography.Paragraph>
            <Row gutter={[16, 16]}>
                <Col xs={24} lg={14}>
                    <div
                        style={{
                            background: mode === "dark" ? colors.bgElevated : colors.bgBase,
                            border: `1px solid ${colors.border}`,
                            borderRadius: 8,
                            display: "flex",
                            justifyContent: "center",
                            padding: 8,
                            overflow: "hidden",
                        }}
                    >
                        <svg
                            ref={svgRef}
                            data-testid="sensor-top-editor"
                            width={svgWidth}
                            height={svgHeight}
                            viewBox={`0 0 ${svgWidth} ${svgHeight}`}
                            style={{ userSelect: "none", touchAction: "none", maxWidth:"100%", height:"auto" }}
                        >
                            {gridLines}
                            {scaleLabels}
                            {robotBody}
                            {liveRobot.fromUrdf && [...SENSORS].reverse().map(renderSensor)}
                        </svg>
                    </div>
                    {liveRobot.fromUrdf && <MowerView robot={robot} view="side" highlightSensors maxHeight={190}/>}
                </Col>

                <Col xs={24} lg={10}>
                    {SENSORS.map((meta) => {
                        const val = getSensorValue(meta);
                        const sensorColor = mode === "dark" ? meta.colorDark : meta.color;
                        return (
                            <Card
                                key={meta.id}
                                size="small"
                                title={
                                    <Space>
                                        <div
                                            style={{
                                                width: 12, height: 12,
                                                borderRadius: meta.shape === "circle" ? "50%" : 2,
                                                background: sensorColor,
                                            }}
                                        />
                                        <span>{meta.label}</span>
                                    </Space>
                                }
                                extra={
                                    <Tooltip title={t("robotComponentEditor.resetToDefaults")}>
                                        <Button type="text" size="small" aria-label={t("settingsReset.resetField", {field: meta.label})} icon={<UndoOutlined />}
                                            onClick={() => resetSensor(meta)} />
                                    </Tooltip>
                                }
                                style={{ marginBottom: 8, borderLeft: `3px solid ${sensorColor}` }}
                            >
                                <Row gutter={[8, 4]}>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>{t("robotComponentEditor.xForward")}</Text>
                                        <InputNumber
                                            data-setting-key={meta.xKey}
                                            aria-label={`${meta.label} — ${t("robotComponentEditor.xForward")}, m`}
                                            value={val.x} onChange={(v) => onChange(meta.xKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>{t("robotComponentEditor.yLeft")}</Text>
                                        <InputNumber
                                            data-setting-key={meta.yKey}
                                            aria-label={`${meta.label} — ${t("robotComponentEditor.yLeft")}, m`}
                                            value={val.y} onChange={(v) => onChange(meta.yKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>{t("robotComponentEditor.zHeight")}</Text>
                                        <InputNumber
                                            data-setting-key={meta.zKey}
                                            aria-label={`${meta.label} — ${t("robotComponentEditor.zHeight")}, m`}
                                            value={val.z} onChange={(v) => onChange(meta.zKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    {meta.yawKey && (
                                        <Col span={12}>
                                            <Text type="secondary" style={{ fontSize: 11 }}>{t("robotComponentEditor.yaw")}</Text>
                                            {meta.id === "imu" ? (
                                                <Space.Compact style={{ width: "100%" }}>
                                                    <InputNumber
                                                        data-setting-key={meta.yawKey}
                                            aria-label={`${meta.label} — ${t("robotComponentEditor.yaw")}, °`}
                                                        value={roundTo(radToDeg(val.yaw), 1)}
                                                        onChange={(v) => onChange(meta.yawKey, roundTo(degToRad(v ?? 0), 4))}
                                                        step={1} precision={1} size="small"
                                                        style={{ width: "100%" }} addonAfter="°"
                                                    />
                                                    <Tooltip title={t("robotComponentEditor.autoCalibrateImuTooltip")}>
                                                        <Button
                                                            size="small"
                                                            icon={<CompassOutlined />}
                                                            aria-label={t("robotComponentEditor.autoCalibrateImuTooltip")}
                                                            onClick={openCalibration}
                                                        />
                                                    </Tooltip>
                                                </Space.Compact>
                                            ) : (
                                                <InputNumber
                                                    data-setting-key={meta.yawKey}
                                            aria-label={`${meta.label} — ${t("robotComponentEditor.yaw")}, °`}
                                                        value={roundTo(radToDeg(val.yaw), 1)}
                                                    onChange={(v) => onChange(meta.yawKey, roundTo(degToRad(v ?? 0), 4))}
                                                    step={1} precision={1} size="small"
                                                    style={{ width: "100%" }} addonAfter="°"
                                                />
                                            )}
                                        </Col>
                                    )}
                                </Row>
                            </Card>
                        );
                    })}

                    {/* Dock pose card — heading input, compass widget, and a
                        prominent "capture from robot" action with a confirmation
                        modal previewing the change. See #173. */}
                    <Card
                        size="small"
                        title={
                            <Space>
                                <div style={{ width: 12, height: 12, borderRadius: 2, background: colors.muted }} />
                                <span>{t("robotComponentEditor.dockPose")}</span>
                            </Space>
                        }
                        style={{ marginBottom: 8, borderLeft: `3px solid ${colors.muted}` }}
                    >
                        <Row gutter={[8, 4]} align="middle">
                            <Col span={12}>
                                <Text type="secondary" style={{ fontSize: 11 }}>{t("robotComponentEditor.bearingCompass")}</Text>
                                <InputNumber
                                    aria-label={t("robotComponentEditor.dockHeading")}
                                    value={roundTo(yawRadToCompassBearing(dockYawRad), 1)}
                                    onChange={(v) => {
                                        const bearing = ((Number(v ?? 0) % 360) + 360) % 360;
                                        onChange("dock_pose_yaw", roundTo(compassBearingToYawRad(bearing), 4));
                                    }}
                                    step={1}
                                    precision={1}
                                    min={0}
                                    max={360}
                                    size="small"
                                    style={{ width: "100%" }}
                                    addonAfter="°"
                                />
                            </Col>
                            <Col span={12}>
                                {/* Mini compass */}
                                <div style={{ display: "flex", justifyContent: "center" }}>
                                    <svg width={60} height={60} viewBox="0 0 60 60">
                                        <circle cx={30} cy={30} r={28} fill="none"
                                            stroke={mode === "dark" ? inkAlpha(0.3) : inkAlpha(0.5)} strokeWidth={1.5} />
                                        {["N", "E", "S", "W"].map((d, i) => {
                                            const a = (i * 90 - 90) * Math.PI / 180;
                                            return (
                                                <text key={d} x={30 + 22 * Math.cos(a)} y={30 + 22 * Math.sin(a) + 3}
                                                    textAnchor="middle" fontSize={8} fontFamily="monospace"
                                                    fill={d === "N" ? colors.danger : colors.textMuted}
                                                >
                                                    {d}
                                                </text>
                                            );
                                        })}
                                        {/* Robot heading arrow.
                                            Compass bearing → SVG angle: bearing 0°=N=up, SVG 0°=right (CW),
                                            so svgAngle = bearing - 90. */}
                                        {(() => {
                                            const bearing = yawRadToCompassBearing(dockYawRad);
                                            const svgAngle = bearing - 90;
                                            const rad = svgAngle * Math.PI / 180;
                                            const tipX = 30 + 16 * Math.cos(rad);
                                            const tipY = 30 + 16 * Math.sin(rad);
                                            const tailX = 30 - 8 * Math.cos(rad);
                                            const tailY = 30 - 8 * Math.sin(rad);
                                            return (
                                                <g>
                                                    <line x1={tailX} y1={tailY} x2={tipX} y2={tipY}
                                                        stroke={colors.primary}
                                                        strokeWidth={2.5} strokeLinecap="round" />
                                                    <circle cx={tipX} cy={tipY} r={3}
                                                        fill={colors.primary} />
                                                </g>
                                            );
                                        })()}
                                    </svg>
                                </div>
                            </Col>
                        </Row>
                        <Typography.Paragraph type="secondary" style={{ fontSize: 10, marginTop: 4, marginBottom: 8 }}>
                            {t("robotComponentEditor.compassBearingHint")} ({t("robotComponentEditor.source")}: <code>{dockYawSource}</code>).
                        </Typography.Paragraph>

                        {/* Capture-from-robot action ─────────────────────── */}
                        <div
                            style={{
                                borderTop: `1px dashed ${colors.border}`,
                                paddingTop: 8,
                                marginTop: 4,
                            }}
                        >
                            <Typography.Text strong style={{ fontSize: 12 }}>
                                {t("robotComponentEditor.captureFromRobot")}
                            </Typography.Text>
                            <Typography.Paragraph
                                type="secondary"
                                style={{ fontSize: 11, marginTop: 2, marginBottom: 8 }}
                            >
                                {t("robotComponentEditor.captureIntro1")}{" "}
                                <strong>{t("robotComponentEditor.captureXYYaw")}</strong>{" "}
                                {t("robotComponentEditor.captureIntro2")}
                            </Typography.Paragraph>
                            <Row gutter={[8, 4]} align="middle">
                                <Col flex="auto">
                                    <Typography.Text type="secondary" style={{ fontSize: 11 }}>
                                        {poseAvailable
                                            ? t("robotComponentEditor.robotNow", {x: roundTo(robotX, 2), y: roundTo(robotY, 2), bearing: roundTo(yawRadToCompassBearing(robotYaw), 0)})
                                            : t("robotComponentEditor.waitingForPose")}
                                    </Typography.Text>
                                </Col>
                                <Col flex="none">
                                    <Tooltip
                                        title={!isCharging
                                            ? t("robotComponentEditor.tooltipNotCharging")
                                            : !poseAvailable
                                                ? t("robotComponentEditor.tooltipWaitingPose")
                                                : t("robotComponentEditor.tooltipCapturePose")}
                                    >
                                        {/* span wrapper lets the Tooltip target a disabled Button */}
                                        <span>
                                            <Button
                                                type="primary"
                                                size="small"
                                                icon={<EnvironmentOutlined />}
                                                loading={settingDock}
                                                disabled={!poseAvailable || !isCharging}
                                                onClick={handleSetDockAtRobot}
                                            >
                                                {t("robotComponentEditor.setDockPose")}
                                            </Button>
                                        </span>
                                    </Tooltip>
                                </Col>
                            </Row>
                        </div>
                    </Card>

                    <Typography.Paragraph type="secondary" style={{ fontSize: 11, marginTop: 8 }}>
                        {t("robotComponentEditor.coordinatesHint")}
                    </Typography.Paragraph>
                </Col>
            </Row>

            <Modal
                title={<Space><CompassOutlined />{t("robotComponentEditor.imuCalibTitle")}</Space>}
                open={calibOpen}
                onCancel={closeCalibration}
                maskClosable={!calibRunning}
                closable={!calibRunning}
                footer={null}
                destroyOnClose
            >
                <Typography.Paragraph>
                    {t("robotComponentEditor.imuCalibIntro1")}{" "}
                    <strong>{t("robotComponentEditor.imuCalibDriveItself")}</strong> ~<strong>{t("robotComponentEditor.imuCalibForward")}</strong>{" "}
                    {t("robotComponentEditor.imuCalibIntro2")}{" "}
                    <strong>{t("robotComponentEditor.imuCalibTenSeconds")}</strong>.
                    <ul style={{ marginTop: 8, marginBottom: 4 }}>
                        <li>{t("robotComponentEditor.imuCalibBullet1a")} <strong>{t("robotComponentEditor.imuCalibUndocked")}</strong> {t("robotComponentEditor.imuCalibBullet1b")}</li>
                        <li>{t("robotComponentEditor.imuCalibBullet2a")} <strong>{t("robotComponentEditor.imuCalibAtLeast1m")}</strong> {t("robotComponentEditor.imuCalibBullet2b")}</li>
                        <li>{t("robotComponentEditor.imuCalibBullet3")}</li>
                    </ul>
                    {t("robotComponentEditor.imuCalibCollisionMonitor")}
                </Typography.Paragraph>
                <Typography.Paragraph type="secondary" style={{ fontSize: 12 }}>
                    {t("robotComponentEditor.imuCalibTechnical")}
                </Typography.Paragraph>

                {!calibResult && !calibRunning && (
                    <div style={{ textAlign: "right", marginTop: 16 }}>
                        <Space>
                            <Button onClick={closeCalibration}>{t("robotComponentEditor.cancel")}</Button>
                            <Button type="primary" icon={<CompassOutlined />} onClick={startCalibration}>
                                {t("robotComponentEditor.start")}
                            </Button>
                        </Space>
                    </div>
                )}

                {calibRunning && (
                    <Alert
                        type="info"
                        showIcon
                        message={t("robotComponentEditor.calibRunningMessage")}
                        description={t("robotComponentEditor.calibRunningDescription")}
                        style={{ marginTop: 8 }}
                    />
                )}

                {calibResult && (
                    <>
                        {calibResult.success ? (
                            <Alert
                                type="success"
                                showIcon
                                message={`imu_yaw = ${calibResult.imu_yaw_deg.toFixed(2)}° (${calibResult.imu_yaw_rad.toFixed(4)} rad)`}
                                description={
                                    <>
                                        {t("robotComponentEditor.calibConfidence", {dev: calibResult.std_dev_deg.toFixed(2), samples: calibResult.samples_used})}
                                        <br />
                                        <Typography.Text type="secondary">
                                            {t("robotComponentEditor.calibCurrentValue", {value: roundTo(radToDeg(values.imu_yaw ?? 0), 2)})}
                                        </Typography.Text>
                                    </>
                                }
                            />
                        ) : (
                            <Alert
                                type="error"
                                showIcon
                                message={t("robotComponentEditor.calibFailed")}
                                description={
                                    <>
                                        {calibResult.message}
                                        <br />
                                        <Typography.Text type="secondary">
                                            {t("robotComponentEditor.calibFailedHint")}
                                        </Typography.Text>
                                    </>
                                }
                            />
                        )}
                        <div style={{ textAlign: "right", marginTop: 16 }}>
                            <Space>
                                <Button onClick={() => { resetCalibration(); }}>{t("robotComponentEditor.retry")}</Button>
                                <Button onClick={closeCalibration}>{t("robotComponentEditor.discard")}</Button>
                                {calibResult.success && (
                                    <Button type="primary" onClick={applyCalibration}>
                                        {t("robotComponentEditor.apply")}
                                    </Button>
                                )}
                            </Space>
                        </div>
                    </>
                )}
            </Modal>
        </Card>
    );
};
