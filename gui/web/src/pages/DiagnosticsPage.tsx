import {
    Alert,
    Button,
    Card,
    Col,
    Collapse,
    Descriptions,
    Flex,
    notification,
    Progress,
    Row,
    Space,
    Statistic,
    Switch,
    Table,
    Tabs,
    Tag,
    Typography,
} from "antd";
import {
    ApiOutlined,
    CheckCircleOutlined,
    CloseCircleOutlined,
    CloudServerOutlined,
    CompassOutlined,
    DashboardOutlined,
    DeleteOutlined,
    DownloadOutlined,
    PlayCircleOutlined,
    ReloadOutlined,
    SettingOutlined,
    SoundOutlined,
    StopOutlined,
    ThunderboltOutlined,
    VideoCameraOutlined,
    WarningOutlined,
} from "@ant-design/icons";
import {ContentType} from "../api/Api.ts";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useBTLog, isBTNodeStale} from "../hooks/useBTLog.ts";
import {useImu} from "../hooks/useImu.ts";
import {useCogHeading} from "../hooks/useCogHeading.ts";
import {useMagYaw} from "../hooks/useMagYaw.ts";
import {useCalibrationStatus} from "../hooks/useCalibrationStatus.ts";
import {useWheelOdom} from "../hooks/useWheelOdom.ts";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";
import {useWheelRpm} from "../hooks/useWheelRpm.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {useDiagnostics} from "../hooks/useDiagnostics.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {
    displayHorizontalAccuracyM,
    deriveGpsStatus,
} from "../utils/gpsStatus.ts";
import {useEffect, useMemo, useState} from "react";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent, getBatteryLevel} from "../utils/battery.ts";
import {yawFromQuaternion, rollFromQuaternion, pitchFromQuaternion, wrapDeg180} from "../utils/quaternion.ts";
import {useApi} from "../hooks/useApi.ts";
import {useFusionGraphDiagnostics} from "../hooks/useFusionGraphDiagnostics.ts";
import {useRosbag, RosbagRecording} from "../hooks/useRosbag.ts";
import {useFirmwareDebugLogs} from "../hooks/useFirmwareDebugLogs.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {useImuYawCalibration} from "../hooks/useImuYawCalibration.ts";
import {AsyncButton} from "../components/AsyncButton.tsx";
import {TelemetryStat} from "../components/TelemetryStat.tsx";
import {BTStateGraph} from "../components/BTStateGraph.tsx";
import {RobotAnatomy} from "../components/RobotAnatomy.tsx";
import {AlertOutlined} from "@ant-design/icons";
import {DashCard} from "../components/dashboard/Card.tsx";
import {GnssLiveDiagnosticsCard} from "../components/gnss/GnssLiveDiagnosticsCard.tsx";
import {clampTinyToZero} from "../utils/telemetryFormat.ts";
import {detectNav2Recovery} from "../utils/nav2Recovery.ts";
import {groupAlertsByComponent} from "../utils/diagnosticsAlerts.ts";
import {useValueSince} from "../hooks/useValueSince.ts";

// ── helpers ─────────────────────────────────────────────────────────────────

function formatBytes(bytes: number): string {
    if (!bytes || bytes < 0) return "0 B";
    const units = ["B", "KB", "MB", "GB", "TB"];
    let value = bytes;
    let i = 0;
    while (value >= 1024 && i < units.length - 1) {
        value /= 1024;
        i++;
    }
    return `${value.toFixed(value >= 10 || i === 0 ? 0 : 1)} ${units[i]}`;
}

function secondsAgo(timestamp: string): number {
    return Math.floor((Date.now() - new Date(timestamp).getTime()) / 1000);
}

// Client timeout for the magnetometer calibration POST — it drives the robot
// through a rotation, so give it a generous budget then abort.
const MAG_CALIB_CLIENT_TIMEOUT_MS = 120_000;

// A Nav2 recovery persisting this long means the robot is not making
// progress — surface it in the health verdict as "possibly stuck".
const NAV2_RECOVERY_STUCK_MS = 120_000;

// HighLevelStatus.state value for autonomous (mowing) operation.
const HIGH_LEVEL_STATE_AUTONOMOUS = 2;

const DIAG_LEVEL_COLORS: Record<number, string> = {0: "success", 1: "warning", 2: "error", 3: "default"};
// Labels resolve through t(...) at render time (module-level: no hook here).
const DIAG_LEVEL_LABEL_KEYS: Record<number, string> = {
    0: "diagnosticsPage.diagLevelOk",
    1: "diagnosticsPage.diagLevelWarn",
    2: "diagnosticsPage.diagLevelError",
    3: "diagnosticsPage.diagLevelStale",
};

// ESC status codes from mowgli_interfaces/msg/ESCStatus.msg
// labelKey resolves through t(...) at render time (module-level: no hook here).
const ESC_STATUS: Record<number, {labelKey: string; color: string}> = {
    0:   {labelKey: "diagnosticsPage.escOff", color: "default"},
    99:  {labelKey: "diagnosticsPage.escDisconnected", color: "warning"},
    100: {labelKey: "diagnosticsPage.escError", color: "error"},
    150: {labelKey: "diagnosticsPage.escStalled", color: "error"},
    200: {labelKey: "diagnosticsPage.escOk", color: "success"},
    201: {labelKey: "diagnosticsPage.escRunning", color: "success"},
};

// ── sub-components ───────────────────────────────────────────────────────────

function HealthBadge({label, color}: {label: string; color: string}) {
    return <Tag color={color} style={{fontSize: 12, padding: "2px 8px"}}>{label}</Tag>;
}

// A Tag whose color is not the sole signal: the label text never changes
// (e.g. "ESC power"), so pair it with a check/cross icon whose SHAPE (not
// just color) carries the on/off state for color-blind / low-vision users.
function BoolStatusTag({label, ok}: {label: string; ok: boolean}) {
    return (
        <Tag color={ok ? "success" : "default"} icon={ok ? <CheckCircleOutlined/> : <CloseCircleOutlined/>}>
            {label}
        </Tag>
    );
}

// ── main page ────────────────────────────────────────────────────────────────

export const DiagnosticsPage = () => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const isMobile = useIsMobile();

    const {highLevelStatus} = useHighLevelStatus();
    const emergency = useEmergency();
    const power = usePower();
    const status = useStatus();
    const gps = useGPS();
    const gnssStatus = useGnssStatus();
    const pose = useFusionOdom();
    const btNodeStates = useBTLog();
    const imu = useImu();
    const {imu: cogImu, lastMessageAt: cogLastAt} = useCogHeading();
    const {imu: magImu, lastMessageAt: magLastAt} = useMagYaw();
    const wheelOdom = useWheelOdom();
    const wheelTicks = useWheelTicks();
    const {status: calibrationStatus, error: calibrationError, refresh: refreshCalibration} = useCalibrationStatus();

    // Tick state once a second so the "Live/Stale" tags update even when no
    // new message has arrived (staleness is time-based, not message-driven).
    const [nowMs, setNowMs] = useState(Date.now());
    useEffect(() => {
        const id = setInterval(() => setNowMs(Date.now()), 1000);
        return () => clearInterval(id);
    }, []);
    const {modal} = App.useApp();
    const {snapshot, loading, error: snapshotError, refresh} = useDiagnosticsSnapshot();
    const {diagnostics} = useDiagnostics();
    const {settings} = useSettings();
    const guiApi = useApi();
    const wheelRpm = useWheelRpm({wheelRadiusM: settings?.wheel_radius ?? 0.04475});
    const {stats: fusionStats} = useFusionGraphDiagnostics();
    const rosbag = useRosbag(true);

    // ── derived values ───────────────────────────────────────────────────────

    const batteryPercent = useMemo(
        () => computeBatteryPercent(highLevelStatus.battery_percent, power.v_battery, settings),
        [highLevelStatus.battery_percent, power.v_battery, settings],
    );

    const gpsFix = useMemo(() => deriveGpsStatus(gnssStatus), [gnssStatus]);
    const gpsFixType = gpsFix.label;

    const orientation = pose.pose?.pose?.orientation;
    const qx = orientation?.x ?? 0;
    const qy = orientation?.y ?? 0;
    const qz = orientation?.z ?? 0;
    const qw = orientation?.w ?? 1;
    const yaw = yawFromQuaternion(qx, qy, qz, qw);
    const roll = rollFromQuaternion(qx, qy, qz, qw);
    const pitch = pitchFromQuaternion(qx, qy, qz, qw);
    const poseZ = pose.pose?.pose?.position?.z ?? 0;

    const allContainersOk = !snapshot?.containers?.length || snapshot.containers.every(c => c.state === "running");
    const gpsAccuracy = displayHorizontalAccuracyM(gnssStatus);
    const gpsFixValid = gnssStatus.fix_valid ?? false;
    const gpsOk = gpsFixValid && gpsAccuracy !== undefined && gpsAccuracy <= 0.1;
    const gpsWarn = gpsFixValid && (gpsAccuracy === undefined || gpsAccuracy > 0.1);
    const cpuTemp = snapshot?.system?.cpu_temperature ?? 0;

    const alerts = useMemo(
        () => (diagnostics.status ?? []).filter(s =>
            s.level >= 1 &&
            // Filter out transient "no data since last update" from ublox driver
            !s.message?.toLowerCase().includes("no data since last update")
        ),
        [diagnostics.status]
    );
    const alertGroups = useMemo(() => groupAlertsByComponent(alerts), [alerts]);

    // Nav2 recovery tracking (from /behavior_tree_log — Nav2's tree, not the
    // Mowgli BT). A long-running recovery is the "robot is stuck" signal.
    const nav2Recovery = useMemo(() => detectNav2Recovery(btNodeStates, nowMs), [btNodeStates, nowMs]);
    const recoverySince = useValueSince(nav2Recovery.active);
    const recoveryDurationMs = recoverySince !== null ? nowMs - recoverySince : 0;
    const recoveryStuck = nav2Recovery.active && recoveryDurationMs > NAV2_RECOVERY_STUCK_MS;

    // Time in the current high-level state ("MOWING · 12 min").
    const stateSince = useValueSince(highLevelStatus.state_name);
    const formatDuration = (ms: number): string => {
        const totalSec = Math.max(0, Math.floor(ms / 1000));
        return totalSec >= 60
            ? t('diagnosticsPage.durationMin', {minutes: Math.floor(totalSec / 60)})
            : t('diagnosticsPage.durationSec', {seconds: totalSec});
    };
    const stateSinceLabel = stateSince !== null ? formatDuration(nowMs - stateSince) : undefined;

    // ── Health Verdict Hero ──────────────────────────────────────────────────
    // A single beginner-readable tile derived from the same booleans the
    // expert health bar uses. Emergency (or a stopped container / no GPS) is
    // "Urgence"; a soft warning (RTK-Float, warm CPU, low battery) is
    // "Attention requise"; otherwise "Tout va bien".

    const cpuHot = cpuTemp > 70;
    const cpuWarm = cpuTemp > 55 && cpuTemp <= 70;
    const batteryLevel = getBatteryLevel(batteryPercent);
    const batteryLow = batteryLevel === "danger";
    const batteryMid = batteryLevel === "warn";

    // The emergency signal proper — only this drives the big "Emergency"
    // wording. Other danger causes (low battery, hot CPU, stopped container,
    // no GPS) surface as "Critical".
    const emergencyActive = (emergency.active_emergency ?? highLevelStatus.emergency) ?? false;

    const healthLevel: "ok" | "warn" | "danger" =
        emergencyActive || !allContainersOk || (!gpsOk && !gpsWarn) || cpuHot || batteryLow
            ? "danger"
            : gpsWarn || cpuWarm || batteryMid || recoveryStuck || alerts.length > 0
                ? "warn"
                : "ok";

    const healthVerdict =
        healthLevel === "danger"
            ? (emergencyActive ? t('diagnosticsPage.verdictEmergency') : t('diagnosticsPage.verdictCritical'))
            : healthLevel === "warn"
                ? (recoveryStuck ? t('diagnosticsPage.verdictStuck') : t('diagnosticsPage.verdictAttention'))
                : t('diagnosticsPage.verdictAllGood');

    const healthColor =
        healthLevel === "danger" ? colors.danger :
        healthLevel === "warn" ? colors.warning :
        colors.primary;

    const healthSubtitle = emergencyActive
        ? t('diagnosticsPage.subtitleEmergency')
        : !allContainersOk
            ? t('diagnosticsPage.subtitleContainerStopped')
            : healthLevel === "danger"
                ? t('diagnosticsPage.subtitleOutOfRange')
                : recoveryStuck
                    ? t('diagnosticsPage.subtitleRecoveryStuck', {duration: formatDuration(recoveryDurationMs)})
                    : gpsWarn || cpuWarm || batteryMid
                        ? t('diagnosticsPage.subtitleWatch')
                        : alerts.length > 0
                            ? t('diagnosticsPage.subtitleAlerts', {count: alerts.length})
                            : t('diagnosticsPage.subtitleAllOperational');

    const healthHero = (
        <DashCard tone={healthLevel === "danger" ? "danger" : "glow"} style={{marginBottom: 4}}>
            <div style={{
                fontSize: 11, color: colors.textMuted, letterSpacing: "0.08em",
                textTransform: "uppercase" as const, fontWeight: 600, marginBottom: 8,
            }}>
                {t('diagnosticsPage.robotStatus')}
            </div>
            <div className="mn-num" style={{fontSize: isMobile ? 40 : 56, lineHeight: 1, color: healthColor}}>
                {healthVerdict}
            </div>
            <div style={{fontSize: 13, color: colors.textSecondary, marginTop: 10}}>
                {healthSubtitle}
            </div>
        </DashCard>
    );

    // ── Health Summary Bar ───────────────────────────────────────────────────

    const healthBar = (
        <Card size="small" style={{marginBottom: 12}}>
            <Flex wrap gap="small" align="center">
                <Typography.Text type="secondary" style={{fontSize: 12, marginRight: 4}}>{t('diagnosticsPage.statusLabel')}</Typography.Text>
                <HealthBadge
                    label={allContainersOk ? t('diagnosticsPage.containersOk') : t('diagnosticsPage.containerProblem')}
                    color={allContainersOk ? "success" : "error"}
                />
                <HealthBadge
                    label={t('diagnosticsPage.gpsBadge', {value: gpsFixType})}
                    color={gpsOk ? "success" : gpsWarn ? "warning" : "error"}
                />
                <HealthBadge
                    label={t('diagnosticsPage.batteryBadge', {value: batteryPercent.toFixed(0)})}
                    color={batteryLevel === "ok" ? "success" : batteryLevel === "warn" ? "warning" : "error"}
                />
                <HealthBadge
                    label={emergencyActive ? t('diagnosticsPage.emergencyUpper') : t('diagnosticsPage.noEmergency')}
                    color={emergencyActive ? "error" : "success"}
                />
                <HealthBadge
                    label={cpuTemp > 0 ? t('diagnosticsPage.cpuTemp', {value: cpuTemp.toFixed(1)}) : t('diagnosticsPage.cpuTempUnknown')}
                    color={cpuTemp > 70 ? "error" : cpuTemp > 55 ? "warning" : "success"}
                />
            </Flex>
        </Card>
    );

    // ── Section 1: System ────────────────────────────────────────────────────

    const containerColumns = [
        {
            title: t('diagnosticsPage.colName'),
            dataIndex: "name",
            key: "name",
            render: (v: string) => <Typography.Text code style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: t('diagnosticsPage.colState'),
            dataIndex: "state",
            key: "state",
            render: (v: string) => <Tag color={v === "running" ? "success" : "error"}>{v}</Tag>,
        },
        {
            title: t('diagnosticsPage.colStatus'),
            dataIndex: "status",
            key: "status",
            render: (v: string) => <Typography.Text style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: t('diagnosticsPage.colStarted'),
            dataIndex: "started_at",
            key: "started_at",
            render: (v: string) => (
                <Typography.Text style={{fontSize: 12}}>
                    {v ? new Date(v).toLocaleTimeString() : "--"}
                </Typography.Text>
            ),
        },
    ];

    const anatomyGps = deriveGpsStatus(gnssStatus);
    // Yaw from quaternion (Z-axis). pose comes from /odometry/filtered_map —
    // reuse the shared `yaw` derived above.
    const lidarEnabled = (settings?.lidar_enabled ?? settings?.use_lidar) as boolean | undefined;
    // LiDAR freshness proxy: fusion_graph's scans_received counter moving
    // within the last 15 s means the LiDAR is streaming (no /scan
    // subscription needed just for a health dot).
    const scansReceivedRaw = fusionStats?.values?.["scans_received"];
    const scansReceivedSince = useValueSince(scansReceivedRaw);
    const lidarStreaming = scansReceivedSince !== null && (nowMs - scansReceivedSince) < 15_000;
    const anatomyInputs = {
        batteryPct: batteryPercent,
        vBattery: power.v_battery ?? 0,
        motorTempC: status.mower_motor_temperature ?? 0,
        escTempC: status.mower_esc_temperature ?? 0,
        gpsLabel: anatomyGps.label,
        gpsOk: anatomyGps.percent >= 50,
        imuYawDeg: yaw,
        imuOk: imu != null && imu.angular_velocity != null,
        lidarOk: lidarEnabled === false ? false : (lidarStreaming ? true : undefined),
        // Mowgli is rear-axle drive: only the rear wheels are encoded (the
        // fronts are unencoded casters whose RPM is always 0).
        wheelLeftRpm: wheelRpm.rl,
        wheelRightRpm: wheelRpm.rr,
        bladeOn: (status.mower_motor_rpm ?? 0) > 0,
        rain: status.rain_detected ?? false,
        dockCharging: status.is_charging ?? false,
    };

    const [firmwareDebugLoading, setFirmwareDebugLoading] = useState(false);
    const [firmwareDebugTarget, setFirmwareDebugTarget] = useState<boolean | null>(null);

    const firmwareDebugActual = status.firmware_debug_enabled ?? false;
    const firmwareDebugEnabled = firmwareDebugTarget ?? firmwareDebugActual;
    const {
        lines: firmwareDebugLines,
        loading: firmwareDebugLogLoading,
        error: firmwareDebugLogError,
    } = useFirmwareDebugLogs(firmwareDebugEnabled);

    useEffect(() => {
        if (firmwareDebugTarget !== null && firmwareDebugActual === firmwareDebugTarget) {
            setFirmwareDebugTarget(null);
            setFirmwareDebugLoading(false);
        }
    }, [firmwareDebugActual, firmwareDebugTarget]);

    useEffect(() => {
        if (firmwareDebugTarget === null) {
            return;
        }
        const timeoutId = window.setTimeout(() => {
            setFirmwareDebugTarget(null);
            setFirmwareDebugLoading(false);
        }, 4000);
        return () => window.clearTimeout(timeoutId);
    }, [firmwareDebugTarget]);

    const toggleFirmwareDebug = async (checked: boolean) => {
        setFirmwareDebugTarget(checked);
        setFirmwareDebugLoading(true);
        try {
            await guiApi.request({
                path: "/diagnostics/firmware_debug",
                method: "POST",
                type: ContentType.Json,
                format: "json",
                body: {enabled: checked},
            });
        } catch (e: any) {
            setFirmwareDebugTarget(null);
            setFirmwareDebugLoading(false);
            notification.error({
                message: t("diagnosticsPage.firmwareDebugToggleFailed"),
                description: e?.message ?? t("diagnosticsPage.unknownError"),
            });
        }
    };

    const sectionSystem = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <RobotAnatomy inputs={anatomyInputs}/>
            </Col>
            <Col span={24}>
                <Card
                    title={<Space><CloudServerOutlined/> {t('diagnosticsPage.containers')}</Space>}
                    size="small"
                    extra={
                        <Button
                            size="small"
                            icon={<ReloadOutlined spin={loading}/>}
                            onClick={refresh}
                        >
                            {t('diagnosticsPage.refresh')}
                        </Button>
                    }
                >
                    {snapshotError && (
                        <Alert
                            type="error"
                            showIcon
                            style={{marginBottom: 12}}
                            message={t('diagnosticsPage.snapshotErrorTitle')}
                            description={snapshotError}
                        />
                    )}
                    <Table
                        size="small"
                        dataSource={snapshot?.containers ?? []}
                        columns={containerColumns}
                        rowKey="name"
                        pagination={false}
                        scroll={{x: "max-content"}}
                        locale={{emptyText: t('diagnosticsPage.noContainerData')}}
                    />
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card title={<Space><DashboardOutlined/> CPU</Space>} size="small" style={{height: "100%"}}>
                    <Statistic
                        title={t('diagnosticsPage.temperature')}
                        value={cpuTemp > 0 ? cpuTemp : undefined}
                        precision={1}
                        suffix="°C"
                        valueStyle={{
                            color: cpuTemp > 70 ? colors.danger : cpuTemp > 55 ? colors.warning : undefined,
                        }}
                    />
                </Card>
            </Col>
            <Col xs={24} lg={16}>
                <Card
                    size="small"
                    style={{height: "100%"}}
                    title={<Space><ApiOutlined/> {t("diagnosticsPage.firmwareDebugTitle")}</Space>}
                    extra={
                        <Space size={10}>
                            <Switch
                                size="small"
                                checked={firmwareDebugEnabled}
                                loading={firmwareDebugLoading}
                                onChange={toggleFirmwareDebug}
                            />
                            <SettingOutlined style={{color: colors.textMuted, fontSize: 14}} />
                        </Space>
                    }
                >
                    {!firmwareDebugEnabled ? (
                        <div style={{
                            minHeight: 176,
                            display: "flex",
                            flexDirection: "column",
                            justifyContent: "center",
                            gap: 8,
                            color: colors.textMuted,
                        }}>
                            <Typography.Text type="secondary">
                                {t("diagnosticsPage.firmwareDebugOffTitle")}
                            </Typography.Text>
                            <Typography.Text type="secondary">
                                {t("diagnosticsPage.firmwareDebugOffBody")}
                            </Typography.Text>
                        </div>
                    ) : (
                        <div style={{
                            minHeight: 176,
                            maxHeight: 176,
                            overflowY: "auto",
                            borderRadius: 8,
                            border: `1px solid ${colors.borderSubtle}`,
                            background: colors.bgCard,
                            padding: "10px 12px",
                            fontFamily: '"JetBrains Mono", "SF Mono", ui-monospace, monospace',
                            fontSize: 12,
                            lineHeight: 1.55,
                        }}>
                            {firmwareDebugLines.length === 0 ? (
                                <Typography.Text type="secondary">
                                    {firmwareDebugLogLoading
                                        ? t("diagnosticsPage.firmwareDebugConnecting")
                                        : firmwareDebugLogError
                                            ? t("diagnosticsPage.firmwareDebugStreamError")
                                            : t("diagnosticsPage.firmwareDebugWaiting")}
                                </Typography.Text>
                            ) : (
                                firmwareDebugLines.map((line) => (
                                    <div key={line.id} style={{color: colors.primary, whiteSpace: "pre-wrap"}}>
                                        {line.plain}
                                    </div>
                                ))
                            )}
                        </div>
                    )}
                </Card>
            </Col>
            {snapshot?.timestamp && (
                <Col span={24}>
                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                        {t('diagnosticsPage.lastSnapshot', {seconds: secondsAgo(snapshot.timestamp)})}
                    </Typography.Text>
                </Col>
            )}
        </Row>
    );

    // ── Section 2: Localization ──────────────────────────────────────────────

    const flatCheck = Math.abs(roll) < 5 && Math.abs(pitch) < 5;
    const sectionLocalization = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><CompassOutlined/> {t('diagnosticsPage.filteredPose')}</Space>} size="small"
                      extra={pose.pose?.pose?.position ? <Tag color="success">{t('diagnosticsPage.live')}</Tag> : <Tag>{t('diagnosticsPage.waiting')}</Tag>}>
                    <Row gutter={[12, 12]}>
                        <TelemetryStat
                            span={8}
                            title={t('diagnosticsPage.xM')}
                            value={pose.pose?.pose?.position?.x ?? null}
                            precision={3}
                        />
                        <TelemetryStat
                            span={8}
                            title={t('diagnosticsPage.yM')}
                            value={pose.pose?.pose?.position?.y ?? null}
                            precision={3}
                        />
                        <TelemetryStat
                            span={8}
                            title={t('diagnosticsPage.zM')}
                            value={pose.pose?.pose?.position ? poseZ : null}
                            precision={3}
                            suffix="m"
                            tone={poseZ > 2 ? "danger" : poseZ > 0.5 ? "warn" : "default"}
                        />
                        <TelemetryStat span={8} title={t('diagnosticsPage.yawDeg')} value={yaw} precision={1} suffix="°"/>
                        <TelemetryStat span={8} title={t('diagnosticsPage.rollDeg')} value={roll} precision={1} suffix="°"/>
                        <TelemetryStat span={8} title={t('diagnosticsPage.pitchDeg')} value={pitch} precision={1} suffix="°"/>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.flatCheck')}
                                value={flatCheck ? t('diagnosticsPage.flatCheckOk') : t('diagnosticsPage.flatCheckDrift')}
                                valueStyle={{color: flatCheck ? undefined : colors.warning}}
                            />
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <GnssLiveDiagnosticsCard
                    gnssStatus={gnssStatus}
                    latitude={gps.pose?.pose?.position?.x}
                    longitude={gps.pose?.pose?.position?.y}
                    altitudeM={gps.pose?.pose?.position?.z}
                    horizontalAccuracyM={gpsAccuracy}
                />
            </Col>
        </Row>
    );

    // ── Section 2b: Heading Sources ──────────────────────────────────────────
    // Shows the two synthetic absolute-yaw Imu publishers fused by ekf_map
    // alongside the filter output for comparison. Staleness threshold: 5 s.

    const STALE_MS = 5000;
    const cogStale = cogLastAt === null || (nowMs - cogLastAt) > STALE_MS;
    const magStale = magLastAt === null || (nowMs - magLastAt) > STALE_MS;

    const cogYawDeg = cogImu?.orientation
        ? yawFromQuaternion(cogImu.orientation.x, cogImu.orientation.y, cogImu.orientation.z, cogImu.orientation.w)
        : null;
    const magYawDeg = magImu?.orientation
        ? yawFromQuaternion(magImu.orientation.x, magImu.orientation.y, magImu.orientation.z, magImu.orientation.w)
        : null;

    // orientation_covariance is a flat length-9 row-major 3×3; yaw variance
    // sits at index 8 (same convention used by cog_to_imu.py / mag_yaw_publisher.py).
    const cogYawVar = cogImu?.orientation_covariance?.[8];
    const magYawVar = magImu?.orientation_covariance?.[8];
    const cogSigmaDeg = (cogYawVar !== undefined && cogYawVar > 0) ? Math.sqrt(cogYawVar) * (180 / Math.PI) : null;
    const magSigmaDeg = (magYawVar !== undefined && magYawVar > 0) ? Math.sqrt(magYawVar) * (180 / Math.PI) : null;

    // Wrap angle difference into (-180, 180] via the shared quaternion util.
    const deltaFilterMag = (!magStale && magYawDeg !== null) ? wrapDeg180(yaw - magYawDeg) : null;
    const deltaFilterCog = (!cogStale && cogYawDeg !== null) ? wrapDeg180(yaw - cogYawDeg) : null;

    // ── Rosbag recording panel ───────────────────────────────────────────────
    // One-click "record all topics" for field debugging: start a capture, drive
    // the robot to reproduce the issue, stop, then download the .tar.gz to send
    // back for offline analysis. The recorder runs inside mowgli-ros2.

    const handleRosbagStart = async () => {
        try {
            const r = await rosbag.start();
            notification.success({message: t('diagnosticsPage.rosbagStarted'), description: r?.name});
        } catch (e: any) {
            notification.error({message: t('diagnosticsPage.rosbagStartFailed'), description: e.message});
        }
    };
    const handleRosbagStop = async () => {
        try {
            const r = await rosbag.stop();
            notification.success({message: t('diagnosticsPage.rosbagStopped'), description: r?.stopped_name});
        } catch (e: any) {
            notification.error({message: t('diagnosticsPage.rosbagStopFailed'), description: e.message});
        }
    };
    const confirmRosbagDelete = (name: string) => {
        modal.confirm({
            title: t('diagnosticsPage.rosbagDeleteConfirmTitle'),
            content: t('diagnosticsPage.rosbagDeleteConfirmBody', {name}),
            okText: t('diagnosticsPage.rosbagDelete'),
            cancelText: t('diagnosticsPage.calibrationCancel'),
            okButtonProps: {danger: true},
            onOk: async () => {
                try {
                    await rosbag.remove(name);
                    notification.success({message: t('diagnosticsPage.rosbagDeleted'), description: name});
                } catch (e: any) {
                    notification.error({message: t('diagnosticsPage.rosbagDeleteFailed'), description: e.message});
                }
            },
        });
    };

    const rosbagActive = rosbag.status?.active ?? false;
    const rosbagRecordings = rosbag.status?.recordings ?? [];

    const sectionRosbag = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={
                        <Space>
                            <VideoCameraOutlined/>
                            {t('diagnosticsPage.rosbagTitle')}
                            {rosbagActive && (
                                <Tag color="processing">
                                    {t('diagnosticsPage.rosbagRecordingTag', {name: rosbag.status?.active_name ?? ""})}
                                </Tag>
                            )}
                        </Space>
                    }
                    size="small"
                    extra={
                        <Space>
                            <Button
                                size="small"
                                type="primary"
                                icon={<PlayCircleOutlined/>}
                                onClick={handleRosbagStart}
                                loading={rosbag.busy === "start"}
                                disabled={rosbagActive || rosbag.busy !== null}
                            >
                                {t('diagnosticsPage.rosbagStart')}
                            </Button>
                            <Button
                                size="small"
                                danger
                                icon={<StopOutlined/>}
                                onClick={handleRosbagStop}
                                loading={rosbag.busy === "stop"}
                                disabled={!rosbagActive || rosbag.busy !== null}
                            >
                                {t('diagnosticsPage.rosbagStop')}
                            </Button>
                        </Space>
                    }
                >
                    <Space direction="vertical" size="small" style={{width: "100%"}}>
                        <Alert
                            type="info"
                            showIcon
                            message={t('diagnosticsPage.rosbagHint')}
                        />
                        {rosbag.error && (
                            <Alert type="warning" showIcon message={rosbag.error}/>
                        )}
                        <Table<RosbagRecording>
                            size="small"
                            rowKey="name"
                            dataSource={rosbagRecordings}
                            pagination={false}
                            locale={{emptyText: t('diagnosticsPage.rosbagEmpty')}}
                            columns={[
                                {
                                    title: t('diagnosticsPage.rosbagColName'),
                                    dataIndex: "name",
                                    key: "name",
                                    render: (name: string, row: RosbagRecording) => (
                                        <Space>
                                            <Typography.Text>{name}</Typography.Text>
                                            {row.active && <Tag color="processing">{t('diagnosticsPage.rosbagLive')}</Tag>}
                                        </Space>
                                    ),
                                },
                                {
                                    title: t('diagnosticsPage.rosbagColSize'),
                                    dataIndex: "size_bytes",
                                    key: "size_bytes",
                                    render: (size: number) => formatBytes(size),
                                },
                                {
                                    title: t('diagnosticsPage.rosbagColDate'),
                                    dataIndex: "modified_at",
                                    key: "modified_at",
                                    render: (d: string) => (d ? new Date(d).toLocaleString() : "—"),
                                },
                                {
                                    title: t('diagnosticsPage.rosbagColActions'),
                                    key: "actions",
                                    render: (_: unknown, row: RosbagRecording) => (
                                        <Space>
                                            <Button
                                                size="small"
                                                icon={<DownloadOutlined/>}
                                                href={rosbag.downloadUrl(row.name)}
                                                disabled={row.active}
                                                target="_blank"
                                            >
                                                {t('diagnosticsPage.rosbagDownload')}
                                            </Button>
                                            <Button
                                                size="small"
                                                danger
                                                icon={<DeleteOutlined/>}
                                                onClick={() => confirmRosbagDelete(row.name)}
                                                disabled={row.active}
                                            />
                                        </Space>
                                    ),
                                },
                            ]}
                        />
                    </Space>
                </Card>
            </Col>
        </Row>
    );

    // ── Fusion Graph (iSAM2) panel ───────────────────────────────────────────
    // fusion_graph_node is the sole map-frame localizer; the panel
    // surfaces the per-tick GraphStats it publishes on
    // /fusion_graph/diagnostics + the Save/Clear service actions.

    const mowerAction = useMowerAction();
    const resetEmergencyAction = mowerAction("emergency", {Emergency: 0});
    const [fusionBusy, setFusionBusy] = useState<"save" | "clear" | null>(null);
    const callFusionService = async (command: "fusion_graph_save" | "fusion_graph_clear") => {
        setFusionBusy(command === "fusion_graph_save" ? "save" : "clear");
        try {
            const res = await guiApi.mowglinext.callCreate(command, {});
            if (res.error) throw new Error((res.error as any)?.error ?? "service call failed");
            notification.success({
                message: command === "fusion_graph_save" ? t('diagnosticsPage.graphSaved') : t('diagnosticsPage.graphCleared'),
                description: (res.data as any)?.message,
            });
        } catch (e: any) {
            notification.error({message: t('diagnosticsPage.fusionGraphActionFailed'), description: e.message});
        } finally {
            setFusionBusy(null);
        }
    };

    const confirmClearGraph = () => {
        modal.confirm({
            title: t('diagnosticsPage.clearGraphConfirmTitle'),
            content: t('diagnosticsPage.clearGraphConfirmBody'),
            okText: t('diagnosticsPage.clearGraphConfirmOk'),
            cancelText: t('diagnosticsPage.calibrationCancel'),
            okButtonProps: {danger: true},
            onOk: () => callFusionService("fusion_graph_clear"),
        });
    };

    const fusionAgeS = fusionStats ? Math.floor((nowMs - fusionStats.receivedAt) / 1000) : null;
    const fusionStale = fusionAgeS === null || fusionAgeS > 5;
    const fv = fusionStats?.values ?? {};
    const num = (k: string) => {
        const raw = fv[k];
        if (raw === undefined) return null;
        const n = Number(raw);
        return Number.isFinite(n) ? n : null;
    };
    const totalNodes = num("total_nodes");
    const scansAttached = num("scans_attached");
    const loopClosures = num("loop_closures");
    const scansReceived = num("scans_received");
    const scanOk = num("scan_matches_ok");
    const scanFail = num("scan_matches_fail");
    const covXX = num("cov_xx");
    const covYY = num("cov_yy");
    const covYaw = num("cov_yawyaw");
    const sigmaXY = (covXX !== null && covYY !== null && covXX >= 0 && covYY >= 0)
        ? Math.sqrt((covXX + covYY) / 2.0) * 100  // → cm
        : null;
    const sigmaYawDeg = (covYaw !== null && covYaw >= 0)
        ? Math.sqrt(covYaw) * (180 / Math.PI)
        : null;
    const scanTotal = (scanOk ?? 0) + (scanFail ?? 0);
    const scanRate = scanTotal > 0 ? Math.round(((scanOk ?? 0) / scanTotal) * 100) : null;

    // ICP / scan-matching detail (live LiDAR monitor).
    const keyframesTotal = num("keyframes_total");
    const kfOk = num("kf_matches_ok");
    const kfFail = num("kf_matches_fail");
    const kfTotal = (kfOk ?? 0) + (kfFail ?? 0);
    const kfRate = kfTotal > 0 ? Math.round(((kfOk ?? 0) / kfTotal) * 100) : null;
    const rejRmse = num("icp_rejects_rmse");
    const rejInliers = num("icp_rejects_inliers");
    const rejSanity = num("icp_rejects_sanity");
    const rejDiverge = num("icp_rejects_divergence");
    const rejTotal = (rejRmse ?? 0) + (rejInliers ?? 0) + (rejSanity ?? 0) + (rejDiverge ?? 0);
    const gpsRejWrongfix = num("gps_rejects_wrongfix");
    const stationaryHandPush = num("stationary_hand_push");
    // Fraction of received scans that actually became graph factors. The two
    // counters have slightly different lifecycles (attached counts nodes,
    // received counts messages), so cap at 100% to avoid nonsense like 108%.
    const attachRate = (scansReceived !== null && scansReceived > 0 && scansAttached !== null)
        ? Math.min(100, Math.round((scansAttached / scansReceived) * 100))
        : null;

    const sectionFusionGraph = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={
                        <Space>
                            <CompassOutlined/>
                            {t('diagnosticsPage.fusionGraphTitle')}
                            <Tag color={fusionStale ? "default" : (fusionStats?.level ?? 0) >= 1 ? "warning" : "success"}>
                                {fusionStale ? t('diagnosticsPage.stale') : (fusionStats?.message ?? t('diagnosticsPage.running'))}
                            </Tag>
                        </Space>
                    }
                    size="small"
                    extra={
                        <Space>
                            <Button
                                size="small"
                                onClick={() => callFusionService("fusion_graph_save")}
                                loading={fusionBusy === "save"}
                                disabled={fusionBusy !== null}
                            >
                                {t('diagnosticsPage.saveGraph')}
                            </Button>
                            <Button
                                size="small"
                                danger
                                onClick={confirmClearGraph}
                                loading={fusionBusy === "clear"}
                                disabled={fusionBusy !== null}
                            >
                                {t('diagnosticsPage.clearGraph')}
                            </Button>
                        </Space>
                    }
                >
                    <Row gutter={[12, 12]}>
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.nodesInGraph')}
                            value={totalNodes}
                            hint={scansAttached !== null ? t('diagnosticsPage.withScans', {count: scansAttached}) : ""}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.loopClosures')}
                            value={loopClosures}
                            tone={(loopClosures ?? 0) > 0 ? "ok" : "default"}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.icpSuccessRate')}
                            value={scanRate}
                            suffix="%"
                            precision={0}
                            hint={scanTotal > 0 ? t('diagnosticsPage.matches', {ok: scanOk ?? 0, total: scanTotal}) : t('diagnosticsPage.scansReceived', {count: scansReceived ?? 0})}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.poseSigma')}
                            value={sigmaXY}
                            suffix="cm"
                            precision={1}
                            tone={sigmaXY === null ? "danger" : sigmaXY < 5 ? "ok" : sigmaXY < 20 ? "warn" : "danger"}
                            hint={sigmaYawDeg !== null ? t('diagnosticsPage.yawSigma', {value: sigmaYawDeg.toFixed(2)}) : ""}
                        />
                    </Row>
                    <Row gutter={[12, 12]} style={{marginTop: 4}}>
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.icpKeyframes')}
                            value={keyframesTotal}
                            tone={(keyframesTotal ?? 0) > 0 ? "ok" : "warn"}
                            hint={kfTotal > 0
                                ? t('diagnosticsPage.icpKfMatches', {rate: kfRate ?? 0, ok: kfOk ?? 0, total: kfTotal})
                                : t('diagnosticsPage.icpNoKeyframes')}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.icpRejects')}
                            value={rejTotal}
                            tone={rejTotal > 0 ? "warn" : "ok"}
                            hint={t('diagnosticsPage.icpRejectBreakdown', {
                                rmse: rejRmse ?? 0,
                                inliers: rejInliers ?? 0,
                                sanity: rejSanity ?? 0,
                                diverge: rejDiverge ?? 0,
                            })}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.attachRateTitle')}
                            value={attachRate}
                            suffix="%"
                            precision={0}
                            hint={t('diagnosticsPage.scansReceived', {count: scansReceived ?? 0})}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.handPushTitle')}
                            value={stationaryHandPush}
                            tone={(stationaryHandPush ?? 0) > 0 ? "warn" : "default"}
                            hint={t('diagnosticsPage.icpGpsWrongfix', {count: gpsRejWrongfix ?? 0})}
                        />
                    </Row>
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginTop: 8, marginBottom: 0}}>
                        {t('diagnosticsPage.fusionGraphDescPart1')}{" "}
                        <Typography.Text code>/ros2_ws/maps/fusion_graph.*</Typography.Text>;
                        {t('diagnosticsPage.fusionGraphDescPart2')}
                        {t('diagnosticsPage.fusionGraphTopicLabel')} <Typography.Text code>/fusion_graph/diagnostics</Typography.Text>{" "}
                        {fusionAgeS !== null && <span>{t('diagnosticsPage.lastUpdateAgo', {seconds: fusionAgeS})}</span>}
                    </Typography.Paragraph>
                </Card>
            </Col>
        </Row>
    );

    const sectionHeadingSources = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card title={<Space><CompassOutlined/> {t('diagnosticsPage.headingSources')}</Space>} size="small">
                    <Row gutter={[12, 12]}>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>{t('diagnosticsPage.filter')}</Typography.Text>
                                    <Tag color="success">/odometry/filtered_map</Tag>
                                </Space>
                                <Statistic title={t('diagnosticsPage.yawDeg')} value={yaw} precision={1} suffix="°"/>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>
                                    {t('diagnosticsPage.referenceSignal')}
                                </Typography.Text>
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>{t('diagnosticsPage.cogGps')}</Typography.Text>
                                    <Tag color={cogStale ? "default" : "processing"}>
                                        {cogStale ? t('diagnosticsPage.stale') : t('diagnosticsPage.live')}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title={t('diagnosticsPage.yawDeg')}
                                    value={cogYawDeg !== null ? cogYawDeg : "-"}
                                    precision={cogYawDeg !== null ? 1 : undefined}
                                    suffix={cogYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {cogSigmaDeg !== null ? `${cogSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterCog !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        {t('diagnosticsPage.deltaFilterCog', {value: deltaFilterCog.toFixed(1)})}
                                    </Typography.Text>
                                )}
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>{t('diagnosticsPage.magnetometer')}</Typography.Text>
                                    <Tag color={magStale ? "default" : "processing"}>
                                        {magStale ? t('diagnosticsPage.stale') : t('diagnosticsPage.live')}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title={t('diagnosticsPage.yawDeg')}
                                    value={magYawDeg !== null ? magYawDeg : "-"}
                                    precision={magYawDeg !== null ? 1 : undefined}
                                    suffix={magYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {magSigmaDeg !== null ? `${magSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterMag !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        {t('diagnosticsPage.deltaFilterMag', {value: deltaFilterMag.toFixed(1)})}
                                    </Typography.Text>
                                )}
                            </Space>
                        </Col>
                    </Row>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 3: BT State & Coverage ───────────────────────────────────────

    const btStateColor =
        highLevelStatus.state === 0 ? "error" :
        highLevelStatus.state === 2 ? "processing" :
        highLevelStatus.state === 3 ? "warning" :
        highLevelStatus.state === 4 ? "cyan" :
        "default";

    // Live mowing progress straight from HighLevelStatus (the REST snapshot's
    // per-area coverage grid was removed backend-side and always came back
    // empty — see /diagnostics/snapshot).
    const mowingActive = highLevelStatus.state === HIGH_LEVEL_STATE_AUTONOMOUS;
    const totalSwaths = highLevelStatus.total_swaths ?? 0;

    const sectionBtCoverage = (
        <Row gutter={[12, 12]}>
            <Col xs={24}>
                <BTStateGraph current={highLevelStatus.state_name} detail={stateSinceLabel}/>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={<Space><ApiOutlined/> {t('diagnosticsPage.btState')}</Space>} size="small">
                    <Space direction="vertical" style={{width: "100%"}}>
                        <Space>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.state')}</Typography.Text>
                            <Tag color={btStateColor} style={{fontSize: 14, padding: "2px 12px"}}>
                                {highLevelStatus.state_name ?? "--"}
                            </Tag>
                            {stateSinceLabel && (
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    {t('diagnosticsPage.stateSince', {duration: stateSinceLabel})}
                                </Typography.Text>
                            )}
                        </Space>
                        {highLevelStatus.sub_state_name && (
                            <Space>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.subState')}</Typography.Text>
                                <Tag>{highLevelStatus.sub_state_name}</Tag>
                            </Space>
                        )}
                    </Space>
                    <Row gutter={[12, 12]} style={{marginTop: 12}}>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.battery')}
                                value={batteryPercent}
                                precision={0}
                                suffix="%"
                                valueStyle={{
                                    color: batteryPercent < 20 ? colors.danger : batteryPercent < 50 ? colors.warning : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.voltage')}
                                value={power.v_battery}
                                precision={2}
                                suffix="V"

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.charging')}
                                value={highLevelStatus.is_charging ? t('diagnosticsPage.yes') : t('diagnosticsPage.no')}
                                valueStyle={{
                                    color: highLevelStatus.is_charging ? colors.primary : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.chargeCurrent')}
                                value={clampTinyToZero(power.charge_current) ?? undefined}
                                precision={2}
                                suffix="A"
                                valueStyle={{
                                    color: highLevelStatus.is_charging && (power.charge_current ?? 0) > 0
                                        ? colors.primary
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.chargerVoltage')}
                                value={clampTinyToZero(power.v_charge) ?? undefined}
                                precision={2}
                                suffix="V"
                            />
                        </Col>
                    </Row>
                    <div style={{marginTop: 12}}>
                        <Space wrap>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.emergency')}</Typography.Text>
                            <Tag color={emergency.active_emergency ? "error" : emergency.latched_emergency ? "warning" : "default"}>
                                {emergency.active_emergency
                                    ? (emergency.reason ?? t('diagnosticsPage.emergencyActive'))
                                    : emergency.latched_emergency
                                        ? t('diagnosticsPage.latched')
                                        : t('diagnosticsPage.clear')}
                            </Tag>
                            {(emergency.active_emergency || emergency.latched_emergency) && (
                                <AsyncButton
                                    danger
                                    size="small"
                                    icon={<AlertOutlined/>}
                                    onAsyncClick={async () => {
                                        await resetEmergencyAction();
                                        notification.success({message: t('diagnosticsPage.resetEmergencySuccess')});
                                    }}
                                >
                                    {t('diagnosticsPage.resetEmergency')}
                                </AsyncButton>
                            )}
                        </Space>
                    </div>
                    {(btNodeStates.size > 0 || nav2Recovery.active) && (
                        <div style={{marginTop: 12}}>
                            <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 4}}>{t('diagnosticsPage.nav2Activity')}</Typography.Text>
                            {nav2Recovery.active && (
                                <Alert
                                    type="warning"
                                    showIcon
                                    style={{marginBottom: 6, fontSize: 12}}
                                    message={t('diagnosticsPage.nav2RecoveryActive', {
                                        node: nav2Recovery.nodeName ?? t('diagnosticsPage.nav2RecoveryGeneric'),
                                        duration: formatDuration(recoveryDurationMs),
                                    })}
                                />
                            )}
                            <Flex wrap gap={4}>
                                {Array.from(btNodeStates.entries())
                                    .filter(([, state]) => state.status === "RUNNING" && !isBTNodeStale(state, nowMs))
                                    .map(([name]) => (
                                        <Tag key={name} color="processing" style={{fontSize: 11}}>
                                            {name}
                                        </Tag>
                                    ))}
                            </Flex>
                        </div>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={t('diagnosticsPage.mowProgressTitle')} size="small">
                    {mowingActive || totalSwaths > 0 ? (
                        <>
                            <Progress
                                percent={Math.round(highLevelStatus.coverage_percent ?? 0)}
                                status={mowingActive ? "active" : "normal"}
                            />
                            <Row gutter={[12, 12]} style={{marginTop: 12}}>
                                <TelemetryStat
                                    span={8}
                                    title={t('diagnosticsPage.currentAreaLabel')}
                                    value={(highLevelStatus.current_area ?? -1) >= 0 ? highLevelStatus.current_area : null}
                                />
                                <TelemetryStat
                                    span={8}
                                    title={t('diagnosticsPage.subPathsLabel')}
                                    value={totalSwaths > 0
                                        ? `${highLevelStatus.completed_swaths ?? 0} / ${totalSwaths}`
                                        : null}
                                    hint={(highLevelStatus.skipped_swaths ?? 0) > 0
                                        ? t('diagnosticsPage.subPathsSkipped', {count: highLevelStatus.skipped_swaths})
                                        : ""}
                                />
                                <TelemetryStat
                                    span={8}
                                    title={t('diagnosticsPage.colCoverage')}
                                    value={highLevelStatus.coverage_percent}
                                    precision={1}
                                    suffix="%"
                                />
                            </Row>
                        </>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            {t('diagnosticsPage.noActiveMowing')}
                        </Typography.Text>
                    )}
                </Card>
            </Col>
        </Row>
    );

    // ── Section 3b: Configuration Cross-checks ──────────────────────────────
    // Note: SLAM (Cartographer) was removed on the feat/kiss-icp branch. The
    // occupancy grid is now published by map_server_node from recorded area
    // polygons, so there is no pbstream to save/delete.

    const crossChecks = snapshot?.cross_checks;
    const crossCheckStatus = crossChecks?.overall_status ?? "ok";

    const sectionCrossChecks = (
        <Row gutter={[12, 12]}>
            <Col xs={24}>
                <Card
                    title={t('diagnosticsPage.configurationCrossChecks')}
                    size="small"
                    extra={
                        <Tag color={
                            crossCheckStatus === "ok" ? "success" :
                            crossCheckStatus === "warn" ? "warning" : "error"
                        }>
                            {crossCheckStatus === "ok" ? t('diagnosticsPage.crossCheckOk') :
                                crossCheckStatus === "warn" ? t('diagnosticsPage.crossCheckWarn') :
                                    t('diagnosticsPage.crossCheckError')}
                        </Tag>
                    }
                >
                    {crossChecks?.warnings && crossChecks.warnings.length > 0 ? (
                        <Space direction="vertical" style={{width: "100%", marginBottom: 12}}>
                            {crossChecks.warnings.map((w, i) => (
                                <Alert key={i} type="warning" message={w} showIcon style={{fontSize: 12}}/>
                            ))}
                        </Space>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 12}}>
                            {t('diagnosticsPage.noWarnings')}
                        </Typography.Text>
                    )}
                    {crossChecks?.dock_pose && (
                        <Row gutter={[8, 4]}>
                            <Col span={24}>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>{t('diagnosticsPage.dockPose')}</Typography.Text>
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title={t('diagnosticsPage.xM')}
                                    value={crossChecks.dock_pose.configured_x}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title={t('diagnosticsPage.yM')}
                                    value={crossChecks.dock_pose.configured_y}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title={t('diagnosticsPage.yawDeg')}
                                    value={(crossChecks.dock_pose.configured_yaw * 180 / Math.PI).toFixed(1)}
                                    suffix="°"
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title={t('diagnosticsPage.datumLat')}
                                    value={crossChecks.dock_pose.datum_lat}
                                    precision={9}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title={t('diagnosticsPage.datumLon')}
                                    value={crossChecks.dock_pose.datum_lon}
                                    precision={9}
                                />
                            </Col>
                            <Col span={24}>
                                <Space>
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.configPresent')}</Typography.Text>
                                    <Tag color={crossChecks.dock_pose.has_config ? "success" : "warning"}>
                                        {crossChecks.dock_pose.has_config ? t('diagnosticsPage.yes') : t('diagnosticsPage.no')}
                                    </Tag>
                                </Space>
                            </Col>
                        </Row>
                    )}
                </Card>
            </Col>
        </Row>
    );

    // ── Section 3c: Calibration Status ───────────────────────────────────────
    // Shows the three on-disk calibration artefacts alongside a run-button
    // for each. Dock + IMU buttons kick off the same service (the node runs
    // dock pre-phase, then accel calibration, then optional mag rotation).
    // Mag is gated on do_mag_calibration at the ROS node, so we just log a
    // hint — enabling the parameter requires an install-side config change.

    // IMU/dock calibration physically drives the robot. Use the shared
    // useImuYawCalibration hook — it owns the AbortController + ~155 s client
    // timeout + error notification. The dock pose + imu_yaw are persisted
    // server-side to mowgli_robot.yaml, so on this page we only refresh the
    // read-only calibration status afterwards (no form to write into).
    const {
        calibRunning: imuCalibRunning,
        startCalibration: startImuCalibration,
    } = useImuYawCalibration({onApplyValue: () => { /* persisted server-side */ }});

    const [magCalibRunning, setMagCalibRunning] = useState(false);
    // A single busy flag shared by every calibration button: while any
    // calibration POST is in flight, all run-buttons disable so a second
    // click cannot fire a concurrent drive command.
    const calibrationBusy = imuCalibRunning || magCalibRunning;

    const runImuCalibration = () => {
        if (calibrationBusy) return;
        modal.confirm({
            title: t('diagnosticsPage.imuCalibrationConfirmTitle'),
            content: t('diagnosticsPage.imuCalibrationConfirmBody'),
            okText: t('diagnosticsPage.imuCalibrationConfirmOk'),
            cancelText: t('diagnosticsPage.calibrationCancel'),
            okButtonProps: {danger: true},
            onOk: async () => {
                await startImuCalibration();
                refreshCalibration();
            },
        });
    };

    const runMagCalibration = () => {
        if (calibrationBusy) return;
        modal.confirm({
            title: t('diagnosticsPage.magCalibrationConfirmTitle'),
            content: t('diagnosticsPage.magCalibrationConfirmBody'),
            okText: t('diagnosticsPage.magCalibrationConfirmOk'),
            cancelText: t('diagnosticsPage.calibrationCancel'),
            okButtonProps: {danger: true},
            onOk: async () => {
                setMagCalibRunning(true);
                const controller = new AbortController();
                const timeoutId = window.setTimeout(() => controller.abort(), MAG_CALIB_CLIENT_TIMEOUT_MS);
                try {
                    const res = await fetch("/api/calibration/magnetometer", {
                        method: "POST",
                        headers: {"Content-Type": "application/json"},
                        body: JSON.stringify({}),
                        signal: controller.signal,
                    });
                    if (!res.ok) {
                        throw new Error(`HTTP ${res.status}: ${await res.text()}`);
                    }
                    const data = await res.json();
                    if (data.success) {
                        notification.success({message: t('diagnosticsPage.magCalibrationComplete'), description: data.message || t('diagnosticsPage.refreshingStatus')});
                    } else {
                        notification.error({message: t('diagnosticsPage.magCalibrationFailed'), description: data.message || t('diagnosticsPage.unknownError')});
                    }
                    refreshCalibration();
                } catch (e: any) {
                    notification.error({
                        message: t('diagnosticsPage.magCalibrationFailed'),
                        description: e?.name === "AbortError"
                            ? t('diagnosticsPage.calibrationTimedOut')
                            : (e instanceof Error ? e.message : String(e)),
                    });
                } finally {
                    window.clearTimeout(timeoutId);
                    setMagCalibRunning(false);
                }
            },
        });
    };

    const formatTs = (ts?: string): string => {
        if (!ts) return "—";
        try {
            return new Date(ts).toLocaleString();
        } catch {
            return ts;
        }
    };

    const dockCal = calibrationStatus?.dock;
    const imuCal = calibrationStatus?.imu;
    const magCal = calibrationStatus?.mag;

    const sectionCalibrationStatus = (
        <Row gutter={[12, 12]}>
            {calibrationError && (
                <Col xs={24}>
                    <Alert
                        type="error"
                        showIcon
                        message={t('diagnosticsPage.calibrationStatusErrorTitle')}
                        description={calibrationError}
                    />
                </Col>
            )}
            <Col xs={24} lg={8}>
                <Card
                    title={<Space><CompassOutlined/> {t('diagnosticsPage.dockCalibration')}</Space>}
                    size="small"
                    extra={
                        <Tag color={dockCal?.present ? "success" : "warning"}>
                            {dockCal?.present ? t('diagnosticsPage.present') : t('diagnosticsPage.missing')}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            loading={imuCalibRunning}
                            disabled={calibrationBusy}
                            onClick={runImuCalibration}
                        >
                            {t('diagnosticsPage.runCalibration')}
                        </Button>,
                    ]}
                >
                    {dockCal?.present && !dockCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label={t('diagnosticsPage.position')}>
                                ({dockCal.dock_pose_x?.toFixed(3)}, {dockCal.dock_pose_y?.toFixed(3)}) m
                            </Descriptions.Item>
                            <Descriptions.Item label={t('diagnosticsPage.yaw')}>
                                {dockCal.dock_pose_yaw_deg?.toFixed(2)}°
                            </Descriptions.Item>
                        </Descriptions>
                    ) : dockCal?.error ? (
                        <Alert type="error" showIcon message={dockCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            {t('diagnosticsPage.dockPoseNotSet')}
                        </Typography.Text>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card
                    title={<Space><CompassOutlined/> {t('diagnosticsPage.imuBiasCalibration')}</Space>}
                    size="small"
                    extra={
                        <Tag color={imuCal?.present ? "success" : "warning"}>
                            {imuCal?.present ? t('diagnosticsPage.present') : t('diagnosticsPage.missing')}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            loading={imuCalibRunning}
                            disabled={calibrationBusy}
                            onClick={runImuCalibration}
                        >
                            {t('diagnosticsPage.runCalibration')}
                        </Button>,
                    ]}
                >
                    {imuCal?.present && !imuCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label={t('diagnosticsPage.calibratedAt')}>
                                {formatTs(imuCal.calibrated_at)}
                            </Descriptions.Item>
                            <Descriptions.Item label={t('diagnosticsPage.samples')}>
                                {imuCal.samples_used ?? "—"}
                            </Descriptions.Item>
                            <Descriptions.Item label={t('diagnosticsPage.gyroBias')}>
                                [{imuCal.gyro_bias_x?.toFixed(5) ?? "—"},{" "}
                                {imuCal.gyro_bias_y?.toFixed(5) ?? "—"},{" "}
                                {imuCal.gyro_bias_z?.toFixed(5) ?? "—"}]
                            </Descriptions.Item>
                            <Descriptions.Item label={t('diagnosticsPage.impliedPitchRoll')}>
                                {imuCal.implied_pitch_deg?.toFixed(2)}° / {imuCal.implied_roll_deg?.toFixed(2)}°
                            </Descriptions.Item>
                        </Descriptions>
                    ) : imuCal?.error ? (
                        <Alert type="error" showIcon message={imuCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            {t('diagnosticsPage.noImuCalibration')}
                        </Typography.Text>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card
                    title={<Space><CompassOutlined/> {t('diagnosticsPage.magnetometerCalibration')}</Space>}
                    size="small"
                    extra={
                        <Tag color={magCal?.present ? "success" : "default"}>
                            {magCal?.present ? t('diagnosticsPage.present') : t('diagnosticsPage.disabled')}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            loading={magCalibRunning}
                            disabled={calibrationBusy}
                            onClick={runMagCalibration}
                        >
                            {t('diagnosticsPage.enableAndRun')}
                        </Button>,
                    ]}
                >
                    {magCal?.present && !magCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label={t('diagnosticsPage.calibratedAt')}>
                                {formatTs(magCal.calibrated_at)}
                            </Descriptions.Item>
                            <Descriptions.Item label="|B| mean">
                                {magCal.magnitude_mean_uT?.toFixed(2)} µT
                            </Descriptions.Item>
                            <Descriptions.Item label="|B| std">
                                {magCal.magnitude_std_uT?.toFixed(2)} µT
                            </Descriptions.Item>
                            <Descriptions.Item label={t('diagnosticsPage.samples')}>
                                {magCal.sample_count ?? "—"}
                            </Descriptions.Item>
                        </Descriptions>
                    ) : magCal?.error ? (
                        <Alert type="error" showIcon message={magCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            {t('diagnosticsPage.magFusionOffPart1')}<Typography.Text code>do_mag_calibration</Typography.Text>{t('diagnosticsPage.magFusionOffPart2')}
                        </Typography.Text>
                    )}
                </Card>
            </Col>
        </Row>
    );

    // ── Section 4: Sensors ───────────────────────────────────────────────────

    const sectionSensors = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title="IMU" size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.angVelX')} value={imu.angular_velocity?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.angVelY')} value={imu.angular_velocity?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.angVelZ')} value={imu.angular_velocity?.z} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.linAccX')} value={imu.linear_acceleration?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.linAccY')} value={imu.linear_acceleration?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('diagnosticsPage.linAccZ')} value={imu.linear_acceleration?.z} precision={4}/>
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={t('diagnosticsPage.wheelOdometry')} size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.linearVel')}
                                value={wheelOdom.twist?.twist?.linear?.x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.angularVel')}
                                value={wheelOdom.twist?.twist?.angular?.z}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.poseXM')}
                                value={wheelOdom.pose?.pose?.position?.x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.poseYM')}
                                value={wheelOdom.pose?.pose?.position?.y}
                                precision={3}
                            />
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col span={24}>
                <Card title={t('diagnosticsPage.perWheelEncoders')} size="small">
                    <Row gutter={[12, 12]}>
                        {[
                            {label: t('diagnosticsPage.frontLeft'),  rpm: wheelRpm.fl, ticks: wheelTicks.wheel_ticks_fl, dir: wheelTicks.wheel_direction_fl, validMask: 1},
                            {label: t('diagnosticsPage.frontRight'), rpm: wheelRpm.fr, ticks: wheelTicks.wheel_ticks_fr, dir: wheelTicks.wheel_direction_fr, validMask: 2},
                            {label: t('diagnosticsPage.rearLeft'),   rpm: wheelRpm.rl, ticks: wheelTicks.wheel_ticks_rl, dir: wheelTicks.wheel_direction_rl, validMask: 4},
                            {label: t('diagnosticsPage.rearRight'),  rpm: wheelRpm.rr, ticks: wheelTicks.wheel_ticks_rr, dir: wheelTicks.wheel_direction_rr, validMask: 8},
                        // Only show wheels the firmware reports encoders for
                        // (rear-only on Mowgli) — permanent "no encoder
                        // reading" slots are noise. Until the first message
                        // arrives (mask 0/undefined), show all four.
                        ].filter(w => {
                            const mask = wheelTicks.valid_wheels ?? 0;
                            return mask === 0 || (mask & w.validMask) !== 0;
                        }).map(w => {
                            const valid = ((wheelTicks.valid_wheels ?? 0) & w.validMask) !== 0;
                            return (
                                <Col key={w.label} xs={12} md={6}>
                                    <Statistic
                                        title={w.label}
                                        value={valid ? w.rpm : undefined}
                                        precision={0}
                                        suffix="rpm"
                                        valueStyle={{
                                            color: !valid ? colors.muted :
                                                   Math.abs(w.rpm) > 5 ? colors.success : undefined,
                                        }}
                                    />
                                    <Typography.Text type="secondary" style={{fontSize: 11}}>
                                        {valid ? (
                                            <>
                                                {t('diagnosticsPage.ticksWithDirection', {ticks: (w.ticks ?? 0).toLocaleString(), direction: w.dir === 1 ? t('diagnosticsPage.directionForward') : t('diagnosticsPage.directionReverse')})}
                                            </>
                                        ) : (
                                            t('diagnosticsPage.noEncoderReading')
                                        )}
                                    </Typography.Text>
                                </Col>
                            );
                        })}
                    </Row>
                    {wheelTicks.wheel_tick_factor != null && (
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginTop: 12, marginBottom: 0}}>
                        {t('diagnosticsPage.tickFactorLabel')}{" "}
                        <Typography.Text code>{wheelTicks.wheel_tick_factor.toFixed(3)}</Typography.Text>{" "}
                        {t('diagnosticsPage.tickFactorUnit')}
                        {" "}
                        {t('diagnosticsPage.bodyOmegaLabel')}{" "}
                        <Typography.Text code>{wheelRpm.bodyOmega.toFixed(3)}</Typography.Text>{" "}
                        rad/s.
                        <br/>
                            {t('diagnosticsPage.perWheelTempNote')}
                        </Typography.Paragraph>
                    )}
                </Card>
            </Col>
            <Col span={24}>
                <Card title={<Space><SoundOutlined/> {t('diagnosticsPage.hardwareStatus')}</Space>} size="small">
                    <Row gutter={[12, 8]}>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title={t('diagnosticsPage.mowerStatus')}
                                value={status.mower_status === 255 ? t('diagnosticsPage.statusOk') : t('diagnosticsPage.initializing')}
                                valueStyle={{color: status.mower_status === 255 ? undefined : colors.warning}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title={t('diagnosticsPage.rain')}
                                value={status.rain_detected ? t('diagnosticsPage.detected') : t('diagnosticsPage.none')}
                                valueStyle={{color: status.rain_detected ? colors.warning : undefined}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title={t('diagnosticsPage.escStatus')}
                                value={ESC_STATUS[status.mower_esc_status ?? 0]
                                    ? t(ESC_STATUS[status.mower_esc_status ?? 0].labelKey)
                                    : t('diagnosticsPage.escUnknown', {code: status.mower_esc_status})}
                                valueStyle={{
                                    color: ESC_STATUS[status.mower_esc_status ?? 0]?.color === "error" ? colors.danger
                                        : ESC_STATUS[status.mower_esc_status ?? 0]?.color === "warning" ? colors.warning
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={t('diagnosticsPage.escTemp')} value={status.mower_esc_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={t('diagnosticsPage.motorTemp')} value={status.mower_motor_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={t('diagnosticsPage.motorRpm')} value={status.mower_motor_rpm} precision={0}/>
                        </Col>
                    </Row>
                    <Flex wrap gap="small" style={{marginTop: 12}}>
                        <BoolStatusTag label={t('diagnosticsPage.rpiPower')} ok={!!status.raspberry_pi_power}/>
                        <BoolStatusTag label={t('diagnosticsPage.escPower')} ok={!!status.esc_power}/>
                        <BoolStatusTag label={t('diagnosticsPage.uiBoard')} ok={!!status.ui_board_available}/>
                        <BoolStatusTag label={t('diagnosticsPage.soundModule')} ok={!!status.sound_module_available}/>
                        <BoolStatusTag label={t('diagnosticsPage.mowEnabled')} ok={!!status.mow_enabled}/>
                    </Flex>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 5: ROS Diagnostics ───────────────────────────────────────────

    const sectionRosDiagnostics = (
        <Card title={t('diagnosticsPage.rosDiagnostics')} size="small">
            {(diagnostics.status ?? []).length === 0 ? (
                <Typography.Text type="secondary">{t('diagnosticsPage.noDiagnosticMessages')}</Typography.Text>
            ) : (
                <Collapse
                    size="small"
                    ghost
                    items={(diagnostics.status ?? []).map((item, idx) => ({
                        key: idx,
                        label: (
                            <Space>
                                <Tag color={DIAG_LEVEL_COLORS[item.level] ?? "default"}>
                                    {DIAG_LEVEL_LABEL_KEYS[item.level] ? t(DIAG_LEVEL_LABEL_KEYS[item.level]) : String(item.level)}
                                </Tag>
                                <Typography.Text style={{fontSize: 13}}>{item.name}</Typography.Text>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>{item.message}</Typography.Text>
                            </Space>
                        ),
                        children: item.values && item.values.length > 0 ? (
                            <div style={{paddingLeft: 8}}>
                                {item.values.map((kv, i) => (
                                    <div key={i} style={{display: "flex", gap: 8, fontSize: 12, marginBottom: 2}}>
                                        <Typography.Text type="secondary">{kv.key}:</Typography.Text>
                                        <Typography.Text code style={{fontSize: 11}}>{kv.value}</Typography.Text>
                                    </div>
                                ))}
                            </div>
                        ) : (
                            <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.noKeyValuePairs')}</Typography.Text>
                        ),
                    }))}
                />
            )}
        </Card>
    );

    // ── Section 6: Alerts ────────────────────────────────────────────────────

    // Alerts collapse to one row per source component (a GNSS hiccup emits
    // 8 related messages at once — stacking them pushed the actual
    // diagnostics content below the fold).
    const sectionAlerts = alertGroups.length > 0 ? (
        <Card title={<Space><WarningOutlined/> {t('diagnosticsPage.alerts')}</Space>} size="small">
            <Collapse
                size="small"
                ghost
                items={alertGroups.map(group => ({
                    key: group.component,
                    label: (
                        <Space>
                            <Tag color={group.worstLevel === 2 ? "error" : group.worstLevel === 3 ? "default" : "warning"}>
                                {group.worstLevel === 2 ? t('diagnosticsPage.diagLevelError')
                                    : group.worstLevel === 3 ? t('diagnosticsPage.diagLevelStale')
                                    : t('diagnosticsPage.diagLevelWarn')}
                            </Tag>
                            <Typography.Text style={{fontSize: 13}}>{group.component}</Typography.Text>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>
                                {t('diagnosticsPage.alertGroupCount', {count: group.items.length})}
                            </Typography.Text>
                        </Space>
                    ),
                    children: (
                        <Space direction="vertical" style={{width: "100%"}}>
                            {group.items.map((item, idx) => (
                                <Alert
                                    key={idx}
                                    type={item.level === 2 ? "error" : item.level === 3 ? "info" : "warning"}
                                    message={item.name}
                                    description={item.message}
                                    showIcon
                                />
                            ))}
                        </Space>
                    ),
                }))}
            />
        </Card>
    ) : null;

    // ── layout ───────────────────────────────────────────────────────────────

    if (isMobile) {
        return (
            <div style={{display: "flex", flexDirection: "column", gap: 12, paddingBottom: 8}}>
                {healthHero}
                {healthBar}
                {sectionAlerts}
                <Collapse
                    defaultActiveKey={[]}
                    size="small"
                    items={[
                        {
                            key: "system",
                            label: <Space><CloudServerOutlined/> {t('diagnosticsPage.tabSystem')}</Space>,
                            children: sectionSystem,
                        },
                        {
                            key: "localization",
                            label: <Space><CompassOutlined/> {t('diagnosticsPage.tabLocalization')}</Space>,
                            children: sectionLocalization,
                        },
                        {
                            key: "fusion_graph",
                            label: <Space><CompassOutlined/> {t('diagnosticsPage.fusionGraphShort')}</Space>,
                            children: sectionFusionGraph,
                        },
                        {
                            key: "heading_sources",
                            label: <Space><CompassOutlined/> {t('diagnosticsPage.headingSourcesShort')}</Space>,
                            children: sectionHeadingSources,
                        },
                        {
                            key: "bt",
                            label: <Space><ApiOutlined/> {t('diagnosticsPage.btStateAndCoverage')}</Space>,
                            children: sectionBtCoverage,
                        },
                        {
                            key: "cross_checks",
                            label: t('diagnosticsPage.configurationCrossChecks'),
                            children: sectionCrossChecks,
                        },
                        {
                            key: "calibration_status",
                            label: t('diagnosticsPage.calibrationStatus'),
                            children: sectionCalibrationStatus,
                        },
                        {
                            key: "sensors",
                            label: <Space><ThunderboltOutlined/> {t('diagnosticsPage.sensors')}</Space>,
                            children: sectionSensors,
                        },
                        {
                            key: "rosbag",
                            label: <Space><VideoCameraOutlined/> {t('diagnosticsPage.rosbagTitle')}</Space>,
                            children: sectionRosbag,
                        },
                        {
                            key: "ros",
                            label: t('diagnosticsPage.rosDiagnostics'),
                            children: sectionRosDiagnostics,
                        },
                    ]}
                />
            </div>
        );
    }

    // Desktop: 5 tabs to keep the page from sprawling. Health bar and (when
    // non-empty) Alerts stay pinned at the top so an oncall operator never
    // has to dig through tabs to see whether something is on fire.
    const tabItems = [
        {
            key: "system",
            label: <Space><CloudServerOutlined/> {t('diagnosticsPage.tabSystem')}</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionSystem}
                {sectionRosbag}
                {sectionRosDiagnostics}
            </Space>,
        },
        {
            key: "localization",
            label: <Space><CompassOutlined/> {t('diagnosticsPage.tabLocalization')}</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionLocalization}
                {sectionFusionGraph}
                {sectionHeadingSources}
            </Space>,
        },
        {
            key: "robot",
            label: <Space><ApiOutlined/> {t('diagnosticsPage.tabRobot')}</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionBtCoverage}
                {sectionSensors}
            </Space>,
        },
        {
            key: "calibration",
            label: <Space><CompassOutlined/> {t('diagnosticsPage.tabCalibration')}</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionCrossChecks}
                {sectionCalibrationStatus}
            </Space>,
        },
    ];

    return (
        <Space direction="vertical" size="middle" style={{width: "100%"}}>
            {healthHero}
            {healthBar}
            {sectionAlerts}
            <Tabs defaultActiveKey="system" items={tabItems} size="large"/>
        </Space>
    );
};

export default DiagnosticsPage;
