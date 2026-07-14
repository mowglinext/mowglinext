import React, {useMemo} from "react";
import {Alert, Card, Col, Form, InputNumber, Row, Space, Switch, Tag, Typography} from "antd";
import {
    CompassOutlined,
    NodeIndexOutlined,
    RadarChartOutlined,
    SafetyCertificateOutlined,
} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
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
};

// Optional LiDAR factors that live in the same fusion_graph_node now
// that ekf_map_node is gone. Both default off; turning them on costs a
// few ms/tick but lets the map-frame estimate ride through multi-minute
// RTK-Float windows.
const LIDAR_FACTOR_TOGGLES: Toggle[] = [
    {
        key: "use_scan_matching",
        title: "settingsLocalization.scanMatchingTitle",
        summary: "settingsLocalization.scanMatchingSummary",
        detail: "settingsLocalization.scanMatchingDetail",
    },
    {
        key: "use_loop_closure",
        title: "settingsLocalization.loopClosureTitle",
        summary: "settingsLocalization.loopClosureSummary",
        detail: "settingsLocalization.loopClosureDetail",
    },
];

const MAGNETOMETER_TOGGLE: Toggle = {
    key: "use_magnetometer",
    title: "settingsLocalization.magYawTitle",
    summary: "settingsLocalization.magYawSummary",
    detail: "settingsLocalization.magYawDetail",
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
    if (!lidarEnabled) return {label: "settingsLocalization.badgeDisabled", color: "default"};
    if (!diag) return {label: "settingsLocalization.badgeNoDriver", color: "warning"};
    switch (diag.level) {
        case 0: return {label: "settingsLocalization.badgeLive", color: "success"};
        case 1: return {label: "settingsLocalization.badgeWarn", color: "warning"};
        case 2: return {label: "settingsLocalization.badgeError", color: "error"};
        default: return {label: "settingsLocalization.badgeStale", color: "default"};
    }
}

export const LocalizationSection: React.FC<Props> = ({values, onChange}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const lidarEnabled = asBool(values.lidar_enabled ?? false);
    const {diagnostics} = useDiagnostics();
    const lidarDiag = useMemo(() => pickLidarDiagnostic(diagnostics ?? {}), [diagnostics]);
    const badge = lidarBadge(lidarDiag, lidarEnabled);

    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{marginBottom: 16}}
                message={t("settingsLocalization.mapFrameLocalizerTitle")}
                description={
                    <span>
                        {t("settingsLocalization.mapFrameLocalizerIntro")} (<Text code>fusion_graph_node</Text>){t("settingsLocalization.mapFrameLocalizerMid")}{" "}
                        <Text code>odom→base_footprint</Text>{" "}
                        {t("settingsLocalization.mapFrameLocalizerTfPrefix")} <Text code>ekf_odom_node</Text> {t("settingsLocalization.mapFrameLocalizerTfSuffix")}{" "}
                        <Link
                            href="https://github.com/mowglinext/mowglinext/wiki/Architecture#optional-factor-graph-localizer-fusion_graph"
                            target="_blank"
                        >
                            {t("settingsLocalization.readArchitectureNotes")}
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
                        <span>{t("settingsLocalization.lidarObstacleTitle")}</span>
                        <Tag color={badge.color}>{t(badge.label)}</Tag>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 8}}>
                    {t("settingsLocalization.lidarObstacleIntro")} <Text code>collision_monitor</Text> {t("settingsLocalization.lidarObstacleConsume")}{" "}
                    <Text code>/scan</Text> {t("settingsLocalization.lidarObstacleOutro")}
                </Paragraph>
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 0, fontSize: 11}}>
                    {t("settingsLocalization.lidarTogglePrefix")} <Text strong>Sensors → lidar_enabled</Text>.
                    {lidarDiag?.message ? (
                        <>
                            {" "}
                            {t("settingsLocalization.latestDiagnostic")} <Text code>{lidarDiag.message}</Text>
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
                        <span>{t("settingsLocalization.lidarLocalizationTitle")}</span>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 12}}>
                    {t("settingsLocalization.lidarLocalizationIntro")} <Text code>fusion_graph_node</Text> {t("settingsLocalization.lidarLocalizationConsumes")}{" "}
                    <Text code>/scan</Text> {t("settingsLocalization.lidarLocalizationOutro")}
                </Paragraph>
                {LIDAR_FACTOR_TOGGLES.map((toggle) => {
                    const enabled = asBool(values[toggle.key]);
                    return (
                        <Card
                            key={toggle.key}
                            size="small"
                            style={{marginBottom: 8}}
                            styles={{body: {padding: "10px 12px"}}}
                        >
                            <Row align="middle" gutter={[16, 8]} wrap={false}>
                                <Col flex="auto">
                                    <Space>
                                        <Text strong style={{fontSize: 14}}>
                                            <NodeIndexOutlined style={{marginRight: 6, color: colors.accent}}/>
                                            {t(toggle.title)}
                                        </Text>
                                    </Space>
                                    <Paragraph style={{margin: "4px 0 0", fontSize: 12}}>
                                        {t(toggle.summary)}
                                    </Paragraph>
                                    <Paragraph type="secondary" style={{margin: "4px 0 0", fontSize: 11}}>
                                        {t(toggle.detail)}
                                    </Paragraph>
                                </Col>
                                <Col flex="none">
                                    <Switch
                                        checked={enabled}
                                        onChange={(v) => onChange(toggle.key, v)}
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
                        <span>{t("settingsLocalization.yawSourcesTitle")}</span>
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
                                {t(MAGNETOMETER_TOGGLE.title)}
                            </Text>
                            <Paragraph style={{margin: "4px 0 0", fontSize: 12}}>
                                {t(MAGNETOMETER_TOGGLE.summary)}
                            </Paragraph>
                            <Paragraph type="secondary" style={{margin: "4px 0 0", fontSize: 11}}>
                                {t(MAGNETOMETER_TOGGLE.detail)}
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

            {/* ── Group D: Magnetometer calibration & tuning ──────────────── */}
            <Card
                size="small"
                style={{marginTop: 16}}
                title={
                    <Space>
                        <CompassOutlined style={{color: colors.accent}}/>
                        <span>{t("settingsLocalization.magCalibrationTitle")}</span>
                    </Space>
                }
            >
                <Paragraph type="secondary" style={{marginTop: 0, marginBottom: 12, fontSize: 12}}>
                    {t("settingsLocalization.magCalibrationIntroPrefix")} <Text strong>{t("settingsLocalization.magYawInline")}</Text> {t("settingsLocalization.magCalibrationIntroSuffix")}
                </Paragraph>
                <Card size="small" style={{marginBottom: 12}} styles={{body: {padding: "10px 12px"}}}>
                    <Row align="middle" gutter={[16, 8]} wrap={false}>
                        <Col flex="auto">
                            <Text strong style={{fontSize: 14}}>{t("settingsLocalization.collectCalibrationSamples")}</Text>
                            <Paragraph type="secondary" style={{margin: "4px 0 0", fontSize: 11}}>
                                {t("settingsLocalization.collectCalibrationDescription")}
                            </Paragraph>
                        </Col>
                        <Col flex="none">
                            <Switch
                                checked={asBool(values.enable_mag_cal)}
                                onChange={(v) => onChange("enable_mag_cal", v)}
                            />
                        </Col>
                    </Row>
                </Card>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsLocalization.magneticDeclinationLabel")} tooltip={t("settingsLocalization.magneticDeclinationTooltip")}>
                                <InputNumber
                                    value={values.declination_deg}
                                    onChange={(v) => onChange("declination_deg", v)}
                                    min={-30} max={30} step={0.1} precision={2}
                                    style={{width: "100%"}} addonAfter="°"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsLocalization.minHorizontalFieldLabel")} tooltip={t("settingsLocalization.minHorizontalFieldTooltip")}>
                                <InputNumber
                                    value={values.min_horizontal_uT}
                                    onChange={(v) => onChange("min_horizontal_uT", v)}
                                    min={0} max={100} step={1} precision={1}
                                    style={{width: "100%"}} addonAfter="µT"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsLocalization.magYawVarianceLabel")} tooltip={t("settingsLocalization.magYawVarianceTooltip")}>
                                <InputNumber
                                    value={values.mag_yaw_variance}
                                    onChange={(v) => onChange("mag_yaw_variance", v)}
                                    min={0.0001} max={1} step={0.0001} precision={4}
                                    style={{width: "100%"}} addonAfter="rad²"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
