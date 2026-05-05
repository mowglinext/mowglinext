import React, {useMemo} from "react";
import {Alert, Card, Col, Row, Space, Switch, Tag, Typography} from "antd";
import {
    CompassOutlined,
    NodeIndexOutlined,
    RadarChartOutlined,
    SafetyCertificateOutlined,
} from "@ant-design/icons";
import {useThemeMode} from "../../theme/ThemeContext.tsx";
import {useDiagnostics} from "../../hooks/useDiagnostics.ts";

const {Text, Paragraph, Link} = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

const asBool = (v: any): boolean => v === true || v === "true";

type Toggle = {
    key: string;
    title: string;
    summary: string;
    detail: string;
    impliesGraph?: boolean;
};

const FUSION_TOGGLES: Toggle[] = [
    {
        key: "use_fusion_graph",
        title: "Fusion Graph (iSAM2)",
        summary: "Use the GTSAM factor-graph localizer for the map frame.",
        detail:
            "Replaces ekf_map_node with fusion_graph_node. Same inputs " +
            "(wheel + IMU + GPS + COG/mag yaw) plus optional LiDAR scan-matching " +
            "and loop-closure factors. Required to carry the map-frame estimate " +
            "through multi-minute RTK-Float windows.",
    },
    {
        key: "use_scan_matching",
        title: "LiDAR scan matching",
        summary: "Add ICP between-factors from /scan to the graph.",
        detail:
            "Each new graph node runs ICP against the previous scan and adds a " +
            "BetweenFactor with the relative motion. Fights GPS dropouts but " +
            "costs ~5 ms/tick at 10 Hz.",
        impliesGraph: true,
    },
    {
        key: "use_loop_closure",
        title: "Loop closure",
        summary: "Search past scans for revisits and add LC factors.",
        detail:
            "Triggers a candidate search around each new node (5 m radius, " +
            "10 min minimum age). Successful loop closures pull drift back to " +
            "the originally-mapped pose, even mid-session.",
        impliesGraph: true,
    },
];

const MAGNETOMETER_TOGGLE: Toggle = {
    key: "use_magnetometer",
    title: "Magnetometer yaw",
    summary: "Fuse tilt-compensated mag yaw as a yaw unary factor.",
    detail:
        "Off by default — motor-induced bias makes the magnetometer " +
        "unreliable on most chassis. Enable only after running mag " +
        "calibration with motors-off and validating a stable |B|.",
};

interface LidarDiag {
    name: string;
    level: number;
    message: string;
}

/**
 * Look for a DiagnosticStatus entry that almost certainly comes from a
 * LiDAR driver/aggregator. Drivers we ship don't all label themselves the
 * same way (ldlidar, sllidar, rplidar, "LiDAR: scan", …) so we match a
 * loose case-insensitive needle.
 */
function pickLidarDiagnostic(diagnostics: { status?: { name?: string; level?: number; message?: string }[] }): LidarDiag | null {
    const status = diagnostics.status ?? [];
    for (const s of status) {
        const name = s.name ?? "";
        if (/lidar|laser ?scan/i.test(name)) {
            return {name, level: s.level ?? 0, message: s.message ?? ""};
        }
    }
    return null;
}

function lidarBadge(diag: LidarDiag | null, lidarEnabled: boolean): {label: string; color: string} {
    if (!lidarEnabled) return {label: "Disabled in settings", color: "default"};
    if (!diag) return {label: "No driver detected", color: "warning"};
    switch (diag.level) {
        case 0: return {label: "Live", color: "success"};
        case 1: return {label: "Warn", color: "warning"};
        case 2: return {label: "Error", color: "error"};
        default: return {label: "Stale", color: "default"};
    }
}

export const LocalizationSection: React.FC<Props> = ({values, onChange}) => {
    const {colors} = useThemeMode();
    const fusionOn = asBool(values.use_fusion_graph);
    const lidarEnabled = asBool(values.use_lidar ?? values.lidar_enabled ?? true);
    const {diagnostics} = useDiagnostics();
    const lidarDiag = useMemo(() => pickLidarDiagnostic(diagnostics ?? {}), [diagnostics]);
    const badge = lidarBadge(lidarDiag, lidarEnabled);

    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{marginBottom: 16}}
                message="Map-frame localizer choice"
                description={
                    <span>
                        With <Text code>use_fusion_graph</Text> off, the legacy{" "}
                        <Text code>robot_localization</Text> dual-EKF runs as the map-frame fuser.
                        With it on, the GTSAM iSAM2 factor-graph node takes over — identical
                        inputs/outputs but adds optional LiDAR factors below. Restart ROS2 after
                        changing.{" "}
                        <Link
                            href="https://github.com/cedbossneo/mowglinext/wiki/Architecture#optional-factor-graph-localizer-fusion_graph"
                            target="_blank"
                        >
                            Read the architecture notes →
                        </Link>
                    </span>
                }
            />

            {/* ── Group A: LiDAR for obstacle avoidance ─────────────────────── */}
            <Card
                size="small"
                style={{marginBottom: 16}}
                title={
                    <Space>
                        <SafetyCertificateOutlined style={{color: colors.accent}}/>
                        <span>LiDAR — obstacle avoidance</span>
                        <Tag color={badge.color}>{badge.label}</Tag>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 8}}>
                    Always on (when the LiDAR driver is running). The Nav2 costmap
                    obstacle layer and <Text code>collision_monitor</Text> consume{" "}
                    <Text code>/scan</Text> directly to stop the robot before it hits
                    something — this happens regardless of which map-frame localizer
                    you choose below.
                </Paragraph>
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 0, fontSize: 11}}>
                    Toggle the LiDAR driver itself in <Text strong>Sensors → use_lidar</Text>.
                    {lidarDiag?.message ? (
                        <>
                            {" "}
                            Latest diagnostic: <Text code>{lidarDiag.message}</Text>
                        </>
                    ) : null}
                </Paragraph>
            </Card>

            {/* ── Group B: LiDAR for localization (factor graph) ────────────── */}
            <Card
                size="small"
                style={{marginBottom: 16}}
                title={
                    <Space>
                        <RadarChartOutlined style={{color: colors.accent}}/>
                        <span>LiDAR — localization (factor graph)</span>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 12}}>
                    Optional. When enabled, the GTSAM factor-graph node consumes{" "}
                    <Text code>/scan</Text> as scan-matching between-factors and
                    loop-closure factors so the map-frame pose can ride through
                    multi-minute RTK-Float windows. The two sub-options below only
                    take effect when <Text code>use_fusion_graph</Text> is on.
                </Paragraph>
                {FUSION_TOGGLES.map((t) => {
                    const enabled = asBool(values[t.key]);
                    const inactive = t.impliesGraph && !fusionOn;
                    return (
                        <Card
                            key={t.key}
                            size="small"
                            style={{marginBottom: 8, opacity: inactive ? 0.55 : 1}}
                            styles={{body: {padding: "10px 12px"}}}
                        >
                            <Row align="middle" gutter={[16, 8]} wrap={false}>
                                <Col flex="auto">
                                    <Space>
                                        <Text strong style={{fontSize: 14}}>
                                            <NodeIndexOutlined style={{marginRight: 6, color: colors.accent}}/>
                                            {t.title}
                                        </Text>
                                        {inactive && <Tag>requires use_fusion_graph</Tag>}
                                    </Space>
                                    <Paragraph style={{margin: "4px 0 0", fontSize: 12}}>
                                        {t.summary}
                                    </Paragraph>
                                    <Paragraph type="secondary" style={{margin: "4px 0 0", fontSize: 11}}>
                                        {t.detail}
                                    </Paragraph>
                                </Col>
                                <Col flex="none">
                                    <Switch
                                        checked={enabled}
                                        onChange={(v) => onChange(t.key, v)}
                                        disabled={inactive}
                                    />
                                </Col>
                            </Row>
                        </Card>
                    );
                })}
            </Card>

            {/* ── Group C: Other yaw sources ──────────────────────────────── */}
            <Card
                size="small"
                title={
                    <Space>
                        <CompassOutlined style={{color: colors.accent}}/>
                        <span>Yaw sources</span>
                    </Space>
                }
            >
                <Card
                    size="small"
                    style={{marginBottom: 0}}
                    styles={{body: {padding: "10px 12px"}}}
                >
                    <Row align="middle" gutter={[16, 8]} wrap={false}>
                        <Col flex="auto">
                            <Text strong style={{fontSize: 14}}>
                                <CompassOutlined style={{marginRight: 6, color: colors.accent}}/>
                                {MAGNETOMETER_TOGGLE.title}
                            </Text>
                            <Paragraph style={{margin: "4px 0 0", fontSize: 12}}>
                                {MAGNETOMETER_TOGGLE.summary}
                            </Paragraph>
                            <Paragraph type="secondary" style={{margin: "4px 0 0", fontSize: 11}}>
                                {MAGNETOMETER_TOGGLE.detail}
                            </Paragraph>
                        </Col>
                        <Col flex="none">
                            <Switch
                                checked={asBool(values[MAGNETOMETER_TOGGLE.key])}
                                onChange={(v) => onChange(MAGNETOMETER_TOGGLE.key, v)}
                            />
                        </Col>
                    </Row>
                </Card>
            </Card>
        </div>
    );
};
