import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Alert, App, Card, InputNumber, Modal, Space, Typography, Row, Col, Tooltip, Button, Tag } from "antd";
import { AimOutlined, CompassOutlined, EnvironmentOutlined, UndoOutlined } from "@ant-design/icons";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { useIsMobile } from "../hooks/useIsMobile.ts";
import { useRobotDescription } from "../hooks/useRobotDescription.ts";
import { useCalibrationStatus } from "../hooks/useCalibrationStatus.ts";
import { useImuYawCalibration } from "../hooks/useImuYawCalibration.ts";
import { usePose } from "../hooks/usePose.ts";
import { useApi } from "../hooks/useApi.ts";
import { getQuaternionFromHeading } from "../utils/map.tsx";

const { Text } = Typography;

// SVG coordinate system: 1 metre = SCALE pixels
const SCALE = 500;

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

const SENSORS: SensorMeta[] = [
    {
        id: "lidar",
        label: "LiDAR",
        color: "#E53935",
        colorDark: "#EF5350",
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
        color: "#1565C0",
        colorDark: "#42A5F5",
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
        color: "#2E7D32",
        colorDark: "#66BB6A",
        shape: "rect",
        size: 0.05,
        xKey: "gps_x",
        yKey: "gps_y",
        yawKey: "",
        zKey: "gps_z",
    },
];

// Convert robot metres to SVG coordinates
// Robot frame: X = forward, Y = left
// SVG: robot faces right (+X to right, +Y down = robot's -Y)
const toSvg = (rx: number, ry: number, cx: number, cy: number): [number, number] => {
    return [cx + rx * SCALE, cy - ry * SCALE];
};

const fromSvg = (sx: number, sy: number, cx: number, cy: number): [number, number] => {
    return [(sx - cx) / SCALE, -(sy - cy) / SCALE];
};

const roundTo = (v: number, decimals: number): number => {
    const f = Math.pow(10, decimals);
    return Math.round(v * f) / f;
};

const radToDeg = (r: number): number => (r * 180) / Math.PI;
const degToRad = (d: number): number => (d * Math.PI) / 180;

type Props = {
    values: Record<string, any>;
    onChange: (name: string, value: any) => void;
};

export const RobotComponentEditor: React.FC<Props> = ({ values, onChange }) => {
    const { colors, mode } = useThemeMode();
    const isMobile = useIsMobile();
    const svgRef = useRef<SVGSVGElement>(null);
    const [dragging, setDragging] = useState<SensorId | null>(null);
    const [rotating, setRotating] = useState<SensorId | null>(null);
    const [hoveredSensor, setHoveredSensor] = useState<SensorId | null>(null);
    const { status: calibrationStatus } = useCalibrationStatus();
    const { notification } = App.useApp();
    const guiApi = useApi();
    const pose = usePose();
    const [settingDock, setSettingDock] = useState(false);

    const handleSetDockAtRobot = useCallback(async () => {
        const px = pose.pose?.pose?.position?.x;
        const py = pose.pose?.pose?.position?.y;
        if (px == null || py == null) {
            notification.warning({ message: "No robot pose available yet" });
            return;
        }
        const heading = values.dock_pose_yaw ?? 0;
        const q = getQuaternionFromHeading(heading);
        try {
            setSettingDock(true);
            await guiApi.mowglinext.mapDockingCreate({
                docking_pose: {
                    orientation: { x: q.x!, y: q.y!, z: q.z!, w: q.w! },
                    position: { x: px, y: py, z: 0 },
                },
            });
            onChange("dock_pose_x", roundTo(px, 3));
            onChange("dock_pose_y", roundTo(py, 3));
            notification.success({ message: "Dock pose set to current robot position" });
        } catch (e: unknown) {
            const message = e instanceof Error ? e.message : "Unknown error";
            notification.error({ message: "Failed to set dock pose", description: message });
        } finally {
            setSettingDock(false);
        }
    }, [pose, guiApi, notification, onChange, values.dock_pose_yaw]);
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

    // Robot geometry from /robot_description URDF topic (falls back to defaults)
    const robot = useRobotDescription();

    const svgWidth = isMobile ? 340 : 520;
    const svgHeight = isMobile ? 380 : 480;
    const cx = svgWidth / 2;
    const cy = svgHeight / 2;

    const getSensorValue = useCallback(
        (meta: SensorMeta): SensorConfig => ({
            x: values[meta.xKey] ?? 0,
            y: values[meta.yKey] ?? 0,
            yaw: meta.yawKey ? (values[meta.yawKey] ?? 0) : 0,
            z: values[meta.zKey] ?? 0,
        }),
        [values]
    );

    const handlePointerDown = useCallback(
        (sensorId: SensorId, e: React.MouseEvent | React.TouchEvent) => {
            e.preventDefault();
            e.stopPropagation();
            setDragging(sensorId);
        },
        []
    );

    const handleRotateDown = useCallback(
        (sensorId: SensorId, e: React.MouseEvent | React.TouchEvent) => {
            e.preventDefault();
            e.stopPropagation();
            setRotating(sensorId);
        },
        []
    );

    useEffect(() => {
        if (!dragging && !rotating) return;

        const meta = SENSORS.find((s) => s.id === (dragging || rotating))!;

        const handleMove = (clientX: number, clientY: number) => {
            const svg = svgRef.current;
            if (!svg) return;
            const rect = svg.getBoundingClientRect();
            const sx = clientX - rect.left;
            const sy = clientY - rect.top;

            if (dragging) {
                const [rx, ry] = fromSvg(sx, sy, cx, cy);
                const clampedX = roundTo(Math.max(-0.4, Math.min(0.4, rx)), 3);
                const clampedY = roundTo(Math.max(-0.4, Math.min(0.4, ry)), 3);
                onChange(meta.xKey, clampedX);
                onChange(meta.yKey, clampedY);
            }

            if (rotating && meta.yawKey) {
                const sensorVal = getSensorValue(meta);
                const [ssx, ssy] = toSvg(sensorVal.x, sensorVal.y, cx, cy);
                const angle = Math.atan2(-(sy - ssy), sx - ssx);
                onChange(meta.yawKey, roundTo(angle, 4));
            }
        };

        const onMouseMove = (e: MouseEvent) => handleMove(e.clientX, e.clientY);
        const onTouchMove = (e: TouchEvent) => {
            e.preventDefault();
            handleMove(e.touches[0].clientX, e.touches[0].clientY);
        };
        const onUp = () => {
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
    }, [dragging, rotating, cx, cy, onChange, getSensorValue]);

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
    }, [cx, cy, mode]);

    // Draw robot body from URDF geometry
    const robotBody = useMemo(() => {
        const ccx = robot.chassisCenterX; // chassis centre offset from base_link
        const halfL = robot.baseLength / 2;
        const halfW = robot.baseWidth / 2;
        const bodyColor = mode === "dark" ? "#2d5a2d" : "#4CAF50";
        const bodyStroke = mode === "dark" ? "#3d7a3d" : "#2E7D32";
        const wheelColor = mode === "dark" ? "#555" : "#333";
        const bladeColor = mode === "dark" ? "#888" : "#9E9E9E";
        const casterColor = mode === "dark" ? "#666" : "#555";

        // Chassis rect offset by chassisCenterX (base_link is at wheel axis, not chassis centre)
        const [bx, by] = toSvg(ccx - halfL, halfW, cx, cy);
        const bw = robot.baseLength * SCALE;
        const bh = robot.baseWidth * SCALE;

        const leftWheel = toSvg(robot.wheelXOffset, robot.wheelTrack / 2, cx, cy);
        const rightWheel = toSvg(robot.wheelXOffset, -robot.wheelTrack / 2, cx, cy);
        const ww = robot.wheelRadius * 2 * SCALE;
        const wh = robot.wheelWidth * SCALE;

        const leftCaster = toSvg(robot.casterXOffset, robot.casterTrack / 2, cx, cy);
        const rightCaster = toSvg(robot.casterXOffset, -robot.casterTrack / 2, cx, cy);
        const cr = robot.casterRadius * SCALE;

        const bladeCentre = toSvg(ccx, 0, cx, cy);
        const br = robot.bladeRadius * SCALE;

        const arrowTip = toSvg(ccx + halfL + 0.04, 0, cx, cy);
        const arrowLeft = toSvg(ccx + halfL + 0.01, 0.02, cx, cy);
        const arrowRight = toSvg(ccx + halfL + 0.01, -0.02, cx, cy);
        const arrowColor = mode === "dark" ? "rgba(255,255,255,0.4)" : "rgba(0,0,0,0.3)";

        // Dock charging station in front of the robot (robot drives forward to dock)
        const dockFill = mode === "dark" ? "#333" : "#999";
        const dockStroke = mode === "dark" ? "#555" : "#777";
        const contactColor = mode === "dark" ? "#c90" : "#d4a017";
        const dockLabelColor = mode === "dark" ? "rgba(255,255,255,0.4)" : "rgba(0,0,0,0.35)";
        // Base plate in front of chassis
        const frontEdge = ccx + halfL;
        const plateW = (robot.baseWidth + 0.08) * SCALE;
        const plateH = 0.12 * SCALE;
        const plateX = cx + (frontEdge + 0.02) * SCALE; // just in front of robot
        const plateY = cy - plateW / 2;
        // Back wall (the wall the robot pushes against)
        const wallH = 0.02 * SCALE;
        const wallTall = plateW * 0.6;
        const wallX = plateX + plateH; // far edge of plate
        // Charging contacts (two copper strips on the dock face)
        const contactW = 0.015 * SCALE;
        const contactH = 0.04 * SCALE;
        const contactGap = robot.wheelTrack * 0.35 * SCALE;
        const contactX = plateX + plateH * 0.1; // near the robot-facing edge

        return (
            <g>
                {/* Dock base plate */}
                <rect
                    x={plateX} y={plateY} width={plateH} height={plateW}
                    rx={4} ry={4}
                    fill={dockFill} stroke={dockStroke} strokeWidth={1.5} opacity={0.55}
                />
                {/* Back wall (far edge the robot pushes against) */}
                <rect
                    x={wallX} y={cy - wallTall / 2}
                    width={wallH} height={wallTall}
                    rx={2} ry={2}
                    fill={mode === "dark" ? "#555" : "#777"} opacity={0.7}
                />
                {/* Charging contacts (two copper strips facing the robot) */}
                <rect
                    x={contactX} y={cy - contactGap - contactH / 2}
                    width={contactW} height={contactH}
                    rx={1} fill={contactColor} opacity={0.85}
                />
                <rect
                    x={contactX} y={cy + contactGap - contactH / 2}
                    width={contactW} height={contactH}
                    rx={1} fill={contactColor} opacity={0.85}
                />
                <text
                    x={plateX + plateH / 2} y={plateY - 5}
                    textAnchor="middle" fontSize={7}
                    fill={dockLabelColor} fontFamily="monospace"
                >
                    dock
                </text>
                {/* Robot body */}
                <rect
                    x={bx} y={by} width={bw} height={bh}
                    rx={8} ry={8}
                    fill={bodyColor} stroke={bodyStroke} strokeWidth={2} opacity={0.7}
                />
                <circle
                    cx={bladeCentre[0]} cy={bladeCentre[1]} r={br}
                    fill={bladeColor} opacity={0.4} stroke={bladeColor}
                    strokeWidth={1} strokeDasharray="4 3"
                />
                <rect
                    x={leftWheel[0] - ww / 2} y={leftWheel[1] - wh / 2}
                    width={ww} height={wh} rx={3} fill={wheelColor}
                />
                <rect
                    x={rightWheel[0] - ww / 2} y={rightWheel[1] - wh / 2}
                    width={ww} height={wh} rx={3} fill={wheelColor}
                />
                <circle cx={leftCaster[0]} cy={leftCaster[1]} r={cr} fill={casterColor} />
                <circle cx={rightCaster[0]} cy={rightCaster[1]} r={cr} fill={casterColor} />
                <polygon
                    points={`${arrowTip[0]},${arrowTip[1]} ${arrowLeft[0]},${arrowLeft[1]} ${arrowRight[0]},${arrowRight[1]}`}
                    fill={arrowColor}
                />
                <text
                    x={cx} y={cy + 4}
                    textAnchor="middle" fontSize={9}
                    fill={mode === "dark" ? "rgba(255,255,255,0.3)" : "rgba(0,0,0,0.25)"}
                    fontFamily="monospace"
                >
                    base_link
                </text>
                <text
                    x={arrowTip[0] + 6} y={arrowTip[1] + 4}
                    fontSize={10} fill={arrowColor} fontFamily="monospace"
                >
                    +X
                </text>
            </g>
        );
    }, [cx, cy, mode, robot]);

    // Draw a single sensor
    const renderSensor = useCallback(
        (meta: SensorMeta) => {
            const val = getSensorValue(meta);
            const [sx, sy] = toSvg(val.x, val.y, cx, cy);
            const sizeInPx = meta.size * SCALE;
            const isActive = dragging === meta.id || rotating === meta.id;
            const isHovered = hoveredSensor === meta.id;
            const sensorColor = mode === "dark" ? meta.colorDark : meta.color;

            const yawLineLen = 0.06 * SCALE;
            const yawEndX = sx + Math.cos(val.yaw) * yawLineLen;
            const yawEndY = sy - Math.sin(val.yaw) * yawLineLen;

            const handleDist = 0.08 * SCALE;
            const handleX = sx + Math.cos(val.yaw) * handleDist;
            const handleY = sy - Math.sin(val.yaw) * handleDist;

            return (
                <g key={meta.id}>
                    {(isActive || isHovered) && (
                        <circle
                            cx={sx} cy={sy} r={sizeInPx + 8}
                            fill="none" stroke={sensorColor} strokeWidth={2}
                            strokeDasharray="4 3" opacity={0.5}
                        />
                    )}

                    {meta.shape === "circle" ? (
                        <circle
                            cx={sx} cy={sy} r={sizeInPx}
                            fill={sensorColor}
                            stroke={isActive ? "#FFF" : sensorColor}
                            strokeWidth={isActive ? 2 : 1}
                            opacity={0.9}
                            style={{ cursor: "grab" }}
                            onMouseDown={(e) => handlePointerDown(meta.id, e)}
                            onTouchStart={(e) => handlePointerDown(meta.id, e)}
                            onMouseEnter={() => setHoveredSensor(meta.id)}
                            onMouseLeave={() => setHoveredSensor(null)}
                        />
                    ) : (
                        <rect
                            x={sx - sizeInPx} y={sy - sizeInPx}
                            width={sizeInPx * 2} height={sizeInPx * 2} rx={2}
                            fill={sensorColor}
                            stroke={isActive ? "#FFF" : sensorColor}
                            strokeWidth={isActive ? 2 : 1}
                            opacity={0.9}
                            style={{ cursor: "grab" }}
                            onMouseDown={(e) => handlePointerDown(meta.id, e)}
                            onTouchStart={(e) => handlePointerDown(meta.id, e)}
                            onMouseEnter={() => setHoveredSensor(meta.id)}
                            onMouseLeave={() => setHoveredSensor(null)}
                        />
                    )}

                    {meta.yawKey && (
                        <>
                            <line
                                x1={sx} y1={sy} x2={yawEndX} y2={yawEndY}
                                stroke={sensorColor} strokeWidth={2}
                            />
                            <polygon
                                points={(() => {
                                    const as = 5;
                                    const a = val.yaw;
                                    const p1x = yawEndX + as * Math.cos(Math.PI - a + 0.4);
                                    const p1y = yawEndY + as * Math.sin(Math.PI - a + 0.4);
                                    const p2x = yawEndX + as * Math.cos(Math.PI - a - 0.4);
                                    const p2y = yawEndY + as * Math.sin(Math.PI - a - 0.4);
                                    return `${yawEndX},${yawEndY} ${p1x},${p1y} ${p2x},${p2y}`;
                                })()}
                                fill={sensorColor}
                            />
                            <circle
                                cx={handleX} cy={handleY} r={6}
                                fill={mode === "dark" ? "#333" : "#FFF"}
                                stroke={sensorColor} strokeWidth={2}
                                style={{ cursor: "crosshair" }}
                                onMouseDown={(e) => handleRotateDown(meta.id, e)}
                                onTouchStart={(e) => handleRotateDown(meta.id, e)}
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
        [cx, cy, dragging, rotating, hoveredSensor, getSensorValue, handlePointerDown, handleRotateDown, mode]
    );

    // Scale labels
    const scaleLabels = useMemo(() => {
        const labels: React.ReactNode[] = [];
        const labelColor = mode === "dark" ? "rgba(255,255,255,0.25)" : "rgba(0,0,0,0.25)";
        const step = 0.10;
        for (let v = -0.3; v <= 0.3 + 0.001; v += step) {
            const r = roundTo(v, 2);
            if (Math.abs(r) < 0.001) continue;
            const [lx] = toSvg(r, 0, cx, cy);
            labels.push(
                <text key={`xl${r}`} x={lx} y={cy + robot.baseWidth / 2 * SCALE + 20} textAnchor="middle" fontSize={8} fill={labelColor} fontFamily="monospace">
                    {r.toFixed(1)}m
                </text>
            );
            const [, ly] = toSvg(0, r, cx, cy);
            labels.push(
                <text key={`yl${r}`} x={cx - robot.baseLength / 2 * SCALE - 10} y={ly + 3} textAnchor="end" fontSize={8} fill={labelColor} fontFamily="monospace">
                    {r.toFixed(1)}m
                </text>
            );
        }
        return labels;
    }, [cx, cy, mode, robot]);

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
        <Card
            title={
                <Space>
                    <AimOutlined />
                    <span>Sensor Placement</span>
                    <Tag color="blue" style={{ fontSize: 10, marginLeft: 4 }}>
                        {robot.baseLength.toFixed(2)} x {robot.baseWidth.toFixed(2)} m (from URDF)
                    </Tag>
                </Space>
            }
            style={{ marginBottom: 16 }}
        >
            <Typography.Paragraph type="secondary" style={{ marginBottom: 16 }}>
                Drag sensors on the top-down robot view to set their X/Y position.
                Use the rotation handle (small circle) to set sensor yaw.
                The grid spacing is 5 cm. Robot dimensions are read from the /robot_description topic.
            </Typography.Paragraph>

            <Row gutter={[16, 16]}>
                <Col xs={24} lg={14}>
                    <div
                        style={{
                            background: mode === "dark" ? "#1a1a1a" : "#fafafa",
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
                            width={svgWidth}
                            height={svgHeight}
                            viewBox={`0 0 ${svgWidth} ${svgHeight}`}
                            style={{ userSelect: "none", touchAction: "none" }}
                        >
                            {gridLines}
                            {scaleLabels}
                            {robotBody}
                            {[...SENSORS].reverse().map(renderSensor)}
                        </svg>
                    </div>
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
                                    <Tooltip title="Reset to defaults">
                                        <Button type="text" size="small" icon={<UndoOutlined />}
                                            onClick={() => resetSensor(meta)} />
                                    </Tooltip>
                                }
                                style={{ marginBottom: 8, borderLeft: `3px solid ${sensorColor}` }}
                            >
                                <Row gutter={[8, 4]}>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>X (forward)</Text>
                                        <InputNumber
                                            value={val.x} onChange={(v) => onChange(meta.xKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>Y (left)</Text>
                                        <InputNumber
                                            value={val.y} onChange={(v) => onChange(meta.yKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Text type="secondary" style={{ fontSize: 11 }}>Z (height)</Text>
                                        <InputNumber
                                            value={val.z} onChange={(v) => onChange(meta.zKey, v ?? 0)}
                                            step={0.005} precision={3} size="small"
                                            style={{ width: "100%" }} addonAfter="m"
                                        />
                                    </Col>
                                    {meta.yawKey && (
                                        <Col span={12}>
                                            <Text type="secondary" style={{ fontSize: 11 }}>Yaw</Text>
                                            {meta.id === "imu" ? (
                                                <Space.Compact style={{ width: "100%" }}>
                                                    <InputNumber
                                                        value={roundTo(radToDeg(val.yaw), 1)}
                                                        onChange={(v) => onChange(meta.yawKey, roundTo(degToRad(v ?? 0), 4))}
                                                        step={1} precision={1} size="small"
                                                        style={{ width: "100%" }} addonAfter="°"
                                                    />
                                                    <Tooltip title="Auto-calibrate IMU mounting yaw (robot drives itself ~0.6 m forward then back)">
                                                        <Button
                                                            size="small"
                                                            icon={<CompassOutlined />}
                                                            onClick={openCalibration}
                                                        />
                                                    </Tooltip>
                                                </Space.Compact>
                                            ) : (
                                                <InputNumber
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

                    {/* Dock heading card */}
                    <Card
                        size="small"
                        title={
                            <Space>
                                <div style={{ width: 12, height: 12, borderRadius: 2, background: mode === "dark" ? "#666" : "#888" }} />
                                <span>Dock Heading</span>
                            </Space>
                        }
                        extra={
                            <Tooltip title="Set dock pose to current robot position (writes dock_pose_x/y/yaw to mowgli_robot.yaml)">
                                <Button
                                    type="text"
                                    size="small"
                                    icon={<EnvironmentOutlined />}
                                    loading={settingDock}
                                    onClick={handleSetDockAtRobot}
                                />
                            </Tooltip>
                        }
                        style={{ marginBottom: 8, borderLeft: `3px solid ${mode === "dark" ? "#666" : "#888"}` }}
                    >
                        <Row gutter={[8, 4]} align="middle">
                            <Col span={12}>
                                <Text type="secondary" style={{ fontSize: 11 }}>Heading (compass)</Text>
                                <InputNumber
                                    value={roundTo(radToDeg(dockYawRad), 1)}
                                    onChange={(v) => onChange("dock_pose_yaw", roundTo(degToRad(v ?? 0), 4))}
                                    step={1}
                                    precision={1}
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
                                            stroke={mode === "dark" ? "#555" : "#ccc"} strokeWidth={1.5} />
                                        {["N", "E", "S", "W"].map((d, i) => {
                                            const a = (i * 90 - 90) * Math.PI / 180;
                                            return (
                                                <text key={d} x={30 + 22 * Math.cos(a)} y={30 + 22 * Math.sin(a) + 3}
                                                    textAnchor="middle" fontSize={8} fontFamily="monospace"
                                                    fill={d === "N" ? (mode === "dark" ? "#e55" : "#c00") : (mode === "dark" ? "#999" : "#666")}
                                                >
                                                    {d}
                                                </text>
                                            );
                                        })}
                                        {/* Robot heading arrow */}
                                        {(() => {
                                            const yawDeg = radToDeg(dockYawRad);
                                            // Convert compass bearing to SVG angle (compass 0°=N=up, SVG 0°=right)
                                            const svgAngle = yawDeg - 90;
                                            const rad = svgAngle * Math.PI / 180;
                                            const tipX = 30 + 16 * Math.cos(rad);
                                            const tipY = 30 + 16 * Math.sin(rad);
                                            const tailX = 30 - 8 * Math.cos(rad);
                                            const tailY = 30 - 8 * Math.sin(rad);
                                            return (
                                                <g>
                                                    <line x1={tailX} y1={tailY} x2={tipX} y2={tipY}
                                                        stroke={mode === "dark" ? "#4CAF50" : "#2E7D32"}
                                                        strokeWidth={2.5} strokeLinecap="round" />
                                                    <circle cx={tipX} cy={tipY} r={3}
                                                        fill={mode === "dark" ? "#4CAF50" : "#2E7D32"} />
                                                </g>
                                            );
                                        })()}
                                    </svg>
                                </div>
                            </Col>
                        </Row>
                        <Typography.Paragraph type="secondary" style={{ fontSize: 10, marginTop: 4, marginBottom: 0 }}>
                            Auto-captured on first charge by dock_yaw_to_set_pose
                            (source: <code>{dockYawSource}</code>). Re-runs each time the
                            robot returns to the dock — no manual entry needed.
                        </Typography.Paragraph>
                    </Card>

                    <Typography.Paragraph type="secondary" style={{ fontSize: 11, marginTop: 8 }}>
                        Coordinates are relative to base_link (centre of rear wheel axis).
                        X+ = forward, Y+ = left, Z+ = up. Yaw is rotation around Z axis in degrees
                        (displayed) / radians (stored).
                    </Typography.Paragraph>
                </Col>
            </Row>

            <Modal
                title={<Space><CompassOutlined />IMU yaw auto-calibration</Space>}
                open={calibOpen}
                onCancel={closeCalibration}
                maskClosable={!calibRunning}
                closable={!calibRunning}
                footer={null}
                destroyOnClose
            >
                <Typography.Paragraph>
                    The robot will <strong>drive itself</strong> ~<strong>0.6 m forward</strong>{" "}
                    and then back to the start, taking roughly <strong>10 seconds</strong>.
                    <ul style={{ marginTop: 8, marginBottom: 4 }}>
                        <li>Make sure the robot is <strong>undocked</strong> (it refuses to run while charging).</li>
                        <li>Leave <strong>at least 1 m</strong> of clear space in front and behind.</li>
                        <li>Any active emergency must be cleared first.</li>
                    </ul>
                    Collision monitor stays armed — obstacles will stop the motion.
                </Typography.Paragraph>
                <Typography.Paragraph type="secondary" style={{ fontSize: 12 }}>
                    Estimates the mounting yaw of the IMU chip relative to base_link by
                    comparing the horizontal accelerometer vector to the wheel-odometry
                    body-frame acceleration during the forward and backward ramps.
                    Only samples with |wz| &lt; 0.05 rad/s and |a_body| &gt; 0.1 m/s² are used.
                </Typography.Paragraph>

                {!calibResult && !calibRunning && (
                    <div style={{ textAlign: "right", marginTop: 16 }}>
                        <Space>
                            <Button onClick={closeCalibration}>Cancel</Button>
                            <Button type="primary" icon={<CompassOutlined />} onClick={startCalibration}>
                                Start
                            </Button>
                        </Space>
                    </div>
                )}

                {calibRunning && (
                    <Alert
                        type="info"
                        showIcon
                        message="Calibration running — robot is driving itself"
                        description="Forward leg, pause, backward leg. Stand clear. The motion will stop automatically."
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
                                        Confidence ±{calibResult.std_dev_deg.toFixed(2)}° from{" "}
                                        {calibResult.samples_used} valid samples.
                                        <br />
                                        <Typography.Text type="secondary">
                                            Current value: {roundTo(radToDeg(values.imu_yaw ?? 0), 2)}°
                                        </Typography.Text>
                                    </>
                                }
                            />
                        ) : (
                            <Alert
                                type="error"
                                showIcon
                                message="Calibration failed"
                                description={
                                    <>
                                        {calibResult.message}
                                        <br />
                                        <Typography.Text type="secondary">
                                            Hint: drive faster or longer so the accelerometer sees a
                                            clear forward and backward impulse along the body X axis.
                                        </Typography.Text>
                                    </>
                                }
                            />
                        )}
                        <div style={{ textAlign: "right", marginTop: 16 }}>
                            <Space>
                                <Button onClick={() => { resetCalibration(); }}>Retry</Button>
                                <Button onClick={closeCalibration}>Discard</Button>
                                {calibResult.success && (
                                    <Button type="primary" onClick={applyCalibration}>
                                        Apply
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
