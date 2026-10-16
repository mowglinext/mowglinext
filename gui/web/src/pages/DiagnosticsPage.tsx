import {DiagnosticHealthSummary} from '../components/diagnostics/DiagnosticHealthSummary';
import {deriveDiagnosticHealth, diagnosticTelemetryFresh, containerObservationLevel, unavailableReport, mavrosWheelRequirement} from '../utils/diagnosticHealth';
import {useMultiplexStatus} from '../hooks/useWS';
import {powerDisplayValue} from '../utils/physicalTelemetry';
import {emergencyIsKnown, emergencyClearanceUnverified} from '../utils/statusTelemetry';
import {useBladeTelemetry} from '../hooks/useBladeTelemetry';
import {useHardwareBackend} from '../hooks/useHardwareBackend';
import {motorDisplayValue, wheelControllerFromDiagnostics, wheelControllerFields, showEncoderWheel, type WheelRole} from '../utils/motorTelemetry';
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
    SlidersOutlined,
    SoundOutlined,
    StopOutlined,
    ThunderboltOutlined,
    VideoCameraOutlined,
} from "@ant-design/icons";
import {ContentType} from "../api/Api.ts";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergencyState} from "../hooks/useEmergency.ts";
import {usePowerState} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {BladeDirectionDisplay} from "../components/BladeDirectionDisplay.tsx";
import {useGPS} from "../hooks/useGPS.ts";
import {useGnssStatusState} from "../hooks/useGnssStatus.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useBTLog, isBTNodeStale} from "../hooks/useBTLog.ts";
import {useCogHeading} from "../hooks/useCogHeading.ts";
import {useMagYaw} from "../hooks/useMagYaw.ts";
import {useCalibrationStatus} from "../hooks/useCalibrationStatus.ts";
import {useWheelOdom} from "../hooks/useWheelOdom.ts";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";
import {useWheelRpm} from "../hooks/useWheelRpm.ts";
import {useDiagnosticsSnapshot, SNAPSHOT_STALE_MS} from "../hooks/useDiagnosticsSnapshot.ts";
import {useDiagnostics, type TimestampedDiagnosticStatus} from "../hooks/useDiagnostics.ts";
import {useTimeFormat} from "../hooks/useTimeFormat.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {
    displayHorizontalAccuracyM,
    deriveGpsStatus,
} from "../utils/gpsStatus.ts";
import {lazy, Suspense, useEffect, useMemo, useState} from "react";
import {useSearchParams} from "react-router-dom";
import {App} from "antd";
import {useTranslation} from "react-i18next";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {yawFromQuaternion, rollFromQuaternion, pitchFromQuaternion, wrapDeg180} from "../utils/quaternion.ts";
import {useApi} from "../hooks/useApi.ts";
import {useFusionGraphDiagnostics} from "../hooks/useFusionGraphDiagnostics.ts";
import {useRosbag, RosbagRecording} from "../hooks/useRosbag.ts";
import {useFirmwareDebugLogs} from "../hooks/useFirmwareDebugLogs.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {useImu} from "../hooks/useImu.ts";
import {useImuYawCalibration} from "../hooks/useImuYawCalibration.ts";
import {AsyncButton} from "../components/AsyncButton.tsx";
import {TelemetryStat} from "../components/TelemetryStat.tsx";
import {BTStateGraph} from "../components/BTStateGraph.tsx";
import {RobotAnatomy} from "../components/RobotAnatomy.tsx";
import {AlertOutlined} from "@ant-design/icons";
import {GnssLiveDiagnosticsCard} from "../components/gnss/GnssLiveDiagnosticsCard.tsx";
import {clampTinyToZero} from "../utils/telemetryFormat.ts";
import {detectNav2Recovery} from "../utils/nav2Recovery.ts";
import {autonomousReceptionRequired, backendReceptionReports, type RequiredDiagnosticSource} from '../utils/diagnosticAlertPresentation';
import type {ValidatedTelemetry} from '../hooks/useValidatedTelemetry';
import {DiagnosticAlertsPanel} from '../components/diagnostics/DiagnosticAlertsPanel';
import {deriveLidarAnchor} from "../utils/lidarAnchor.ts";
import {useValueSince} from "../hooks/useValueSince.ts";
import {SystemPowerCard} from "../components/SystemPowerCard.tsx";
import {BlackboxPanel} from "../components/BlackboxPanel.tsx";
import "./DiagnosticsPage.css";
import {apiErrorMessage, errorMessage, isAbortError} from "../utils/errors";
import {voidify} from "../utils/promises";

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

// A Tag whose color is not the sole signal: the label text never changes
// (e.g. "ESC power"), so pair it with a check/cross icon whose SHAPE (not
// just color) carries the on/off state for color-blind / low-vision users.
function BoolStatusTag({label, ok}: {label: string; ok: boolean | undefined}) {
    const {t} = useTranslation();
    if (ok === undefined) return <Tag color="warning">{label}: {t('telemetry.statusUnknown')}</Tag>;
    return (
        <Tag color={ok ? "success" : "default"} icon={ok ? <CheckCircleOutlined/> : <CloseCircleOutlined/>}>
            {label}
        </Tag>
    );
}

// ── main page ────────────────────────────────────────────────────────────────

// The raw ROS parameter editor is for advanced users: it lives under
// Diagnostics instead of the main menu, and loads only when opened.
const ParametersPage = lazy(() => import("./ParametersPage.tsx"));
const sectionParameters = (
    <Suspense fallback={null}>
        <ParametersPage/>
    </Suspense>
);

export const DiagnosticsPage = () => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();
    const {formatAbsolute, formatLogTime} = useTimeFormat();
    const isMobile = useIsMobile();

    const {highLevelStatus, telemetry: highLevelTelemetry} = useHighLevelStatus();
    const {emergency, telemetry: emergencyTelemetry} = useEmergencyState();
    const {telemetry: powerTelemetry} = usePowerState();
    const power = powerTelemetry.data;
    const connected = useMultiplexStatus() === 'open';
    const status = useStatus();
    const hardware = useHardwareBackend();
    const blade = useBladeTelemetry(hardware.loading ? undefined : hardware.backend);
    const bladeEscStatus = blade.fresh ? status.mower_esc_status : undefined;
    const gps = useGPS();
    const {status: gnssStatus, telemetry: gnssTelemetry} = useGnssStatusState();
    const pose = useFusionOdom();
    const btNodeStates = useBTLog();
    const {imu: cogImu, lastMessageAt: cogLastAt} = useCogHeading();
    const {imu: magImu, lastMessageAt: magLastAt} = useMagYaw();
    const wheelOdom = useWheelOdom();
    const wheelTicks = useWheelTicks();
    const {status: calibrationStatus, readError: calibrationReadError, refresh: refreshCalibration} = useCalibrationStatus();

    // Tick state once a second so the "Live/Stale" tags update even when no
    // new message has arrived (staleness is time-based, not message-driven).
    const [nowMs, setNowMs] = useState(Date.now());
    useEffect(() => {
        const id = setInterval(() => setNowMs(Date.now()), 1000);
        return () => clearInterval(id);
    }, []);
    const {modal} = App.useApp();
    const {snapshot, receivedAt: snapshotReceivedAt, loading, error: snapshotError, readError: snapshotReadError, refresh} = useDiagnosticsSnapshot();
    const {diagnostics, disconnectedAt: diagnosticsDisconnectedAt} = useDiagnostics();
    const wheelControllers = {left_wheel: wheelControllerFromDiagnostics(diagnostics, 'left_wheel', nowMs),
        right_wheel: wheelControllerFromDiagnostics(diagnostics, 'right_wheel', nowMs)};
    // Mobile uses collapsible panels; desktop shows sensors in the Robot tab.
    // Subscribe to the high-rate IMU stream only in the visible sensor view.
    // ?tab=parameters opens the advanced ROS parameters editor (formerly its
    // own page in the main menu; /parameters redirects here).
    const [searchParams] = useSearchParams();
    const requestedTab = searchParams.get("tab");
    const [openPanels, setOpenPanels] = useState<string[]>(requestedTab ? [requestedTab] : []);
    const [activeTab, setActiveTab] = useState(requestedTab ?? "system");
    const sensorsPanelOpen = isMobile ? openPanels.includes("sensors") : activeTab === "robot";
    const imu = useImu(sensorsPanelOpen);
    const {settings} = useSettings();
    const guiApi = useApi();
    const wheelRpm = useWheelRpm({wheelRadiusM: (settings?.wheel_radius as number | undefined) ?? 0.04475});
    const {stats: fusionStats} = useFusionGraphDiagnostics();
    const rosbag = useRosbag(true);

    // ── derived values ───────────────────────────────────────────────────────

    const nowMonotonic = performance.now();
    const highLevelFresh = diagnosticTelemetryFresh(highLevelTelemetry, nowMonotonic);
    const powerFresh = diagnosticTelemetryFresh(powerTelemetry, nowMonotonic);
    const batteryPercent = computeBatteryPercent(highLevelFresh ? highLevelStatus.battery_percent : undefined,
        powerFresh ? power.v_battery : undefined, settings);
    const batteryKnown = connected && Number.isFinite(batteryPercent);

    const gpsFix = useMemo(() => deriveGpsStatus(gnssStatus), [gnssStatus]);
    const gpsFixType = gpsFix.label;
    const gpsAccuracy = displayHorizontalAccuracyM(gnssStatus);

    const orientation = pose.pose?.pose?.orientation;
    const qx = orientation?.x ?? 0;
    const qy = orientation?.y ?? 0;
    const qz = orientation?.z ?? 0;
    const qw = orientation?.w ?? 1;
    const yaw = yawFromQuaternion(qx, qy, qz, qw);
    const roll = rollFromQuaternion(qx, qy, qz, qw);
    const pitch = pitchFromQuaternion(qx, qy, qz, qw);
    const poseZ = pose.pose?.pose?.position?.z ?? 0;

    const snapshotFresh = !snapshotError && snapshotReceivedAt !== null && nowMonotonic >= snapshotReceivedAt
        && nowMonotonic - snapshotReceivedAt < SNAPSHOT_STALE_MS;
    const cpuTemp = snapshotFresh ? snapshot?.system?.cpu_temperature : undefined;
    const cpuUsage = snapshotFresh ? snapshot?.system?.cpu_usage : undefined;

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

    const emergencyKnown = emergencyIsKnown(emergency, highLevelStatus);
    const clearanceUnverified = emergencyClearanceUnverified(emergency, highLevelStatus);
    const health = deriveDiagnosticHealth({emergency, highLevel: highLevelStatus, connected,
        emergencyFresh: diagnosticTelemetryFresh(emergencyTelemetry, nowMonotonic), operationalFresh: highLevelFresh,
        batteryFresh: highLevelFresh || powerFresh, batteryPercent, gnss: gnssStatus,
        snapshot, snapshotFresh, diagnostics: diagnostics.status ?? [], nowMs,
        nowMonotonic, diagnosticsDisconnectedAt, settings, recoveryStuck, fusion: fusionStats});
    // Display-only reception watches. Never feed these deadlines back into health.
    const source = (key: string, at: number | null, ageMs = at === null ? 0 : nowMonotonic - at): RequiredDiagnosticSource =>
        ({key, label: t(`diagnosticReception.source.${key}`), publication: at, ageMs});
    const telemetryAt = <T,>(value: ValidatedTelemetry<T>) => value.condition !== 'valid' || value.publicationBlocked === true
        ? null : value.publicationPolicy === 'receipt' ? value.lastValidReceiptAt
            : value.publicationProgressAt ?? value.stampChangedAt ?? value.lastValidReceiptAt;
    const receptionSources = [source('operation', telemetryAt(highLevelTelemetry)),
        source('emergency', telemetryAt(emergencyTelemetry)),
        source('system', snapshotError ? null : snapshotReceivedAt)];
    // Sensor requirements follow explicit configuration and the standard core
    // monitoring producers. Do not declare every arbitrary ROS report required.
    const [backendContext, setBackendContext] = useState(() => ({backend: hardware.backend, required: false, baseline: {} as Record<string, number>}));
    let context = backendContext;
    if (context.backend !== hardware.backend) {
        context = {backend: hardware.backend, required: false, baseline: Object.fromEntries((diagnostics.status ?? [])
            .filter(entry => entry.name === 'Hardware Bridge' || entry.name === 'Odometry'
                || entry.name === 'mowgli_mavros_bridge/wheel_odometry_source')
            .map(entry => [JSON.stringify([entry.hardware_id, entry.name]), entry.healthProgressAt ?? entry.receivedAt]))};
        setBackendContext(context);
    }
    const inBackendContext = (entry: TimestampedDiagnosticStatus) => (entry.healthProgressAt ?? entry.receivedAt)
        > (context.baseline[JSON.stringify([entry.hardware_id, entry.name])] ?? -Infinity);
    const wheelRequirementReport = diagnostics.status?.find(entry => entry.name === 'mowgli_mavros_bridge/wheel_odometry_source' && entry.hardware_id === '');
    // Accumulated diagnostics from the preceding backend cannot declare a new requirement.
    const wheelRequirement = hardware.backend === 'mavros' && connected && wheelRequirementReport
        && wheelRequirementReport.healthBlocked !== true
        && inBackendContext(wheelRequirementReport)
        ? mavrosWheelRequirement(wheelRequirementReport) : undefined;
    if (wheelRequirement !== undefined && wheelRequirement !== context.required) {
        context = {...context, required: wheelRequirement};
        setBackendContext(context);
    }
    const requiredReports = [...backendReceptionReports(hardware.backend, context.required),
        ...(settings.lidar_enabled === true ? [['lidar', 'LiDAR', 'mowgli/lidar']] : [])];
    for (const [key, name, hardwareId] of requiredReports) {
        const entry = diagnostics.status?.find(item => item.name === name && item.hardware_id === hardwareId);
        const at = connected && entry && (key !== 'hardware' && key !== 'odometry'
            || inBackendContext(entry))
            && entry.healthBlocked !== true && entry.level !== 3 && !unavailableReport(entry)
            ? entry.healthProgressAt ?? entry.receivedAt : null;
        receptionSources.push({...source(key, at, at === null ? 0 : nowMs - at),
            context: key === 'hardware' || key === 'odometry' ? hardware.backend : undefined});
    }
    if (settings.gnss_stack === 'universal' || hardware.backend === 'mavros') {
        receptionSources.push(source('gnss', telemetryAt(gnssTelemetry)));
    }
    // Loss of context is not evidence that an autonomous requirement ended.
    const [localizerRequired, setLocalizerRequired] = useState(false);
    const reportedState = highLevelTelemetry.condition === 'valid' ? highLevelStatus.state : undefined;
    const requirement = autonomousReceptionRequired(localizerRequired, reportedState);
    if (requirement !== localizerRequired) setLocalizerRequired(requirement);
    if (requirement) {
        const at = connected && fusionStats && fusionStats.healthBlocked !== true && fusionStats.level !== 3
            ? fusionStats.healthProgressAt ?? fusionStats.distinctMonotonic ?? null : null;
        receptionSources.push(source('localizer', at));
    }
    const alerts = health.alerts;
    const healthSummary = <DiagnosticHealthSummary health={health} gpsLabel={gpsFixType}
        batteryPercent={batteryPercent} cpuTemp={cpuTemp} cpuUsage={cpuUsage}/>;

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
            render: (v: string) => <Tag color={containerObservationLevel(v) === "ok" ? "success" : containerObservationLevel(v) === "unknown" ? "default" : "error"}>{v || t("telemetry.statusUnknown")}</Tag>,
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
                    {formatAbsolute(v, {timeOnly: true})}
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
    // IMU liveness comes from diagnostics_node's 1 Hz "IMU" status, not from a
    // live /imu/data subscription. This page only ever needed a boolean, yet
    // useImu() held a 90 Hz upstream foxglove subscription open for as long as
    // the page was, and that stream is serialised by foxglove_bridge at full
    // rate regardless of client-side decimation. Measured on the docked robot
    // 2026-09-06 it was a visible slice of GUI + bridge CPU. The diagnostics
    // status is also more honest: useTopic kept the last IMU message forever,
    // so imuOk stayed true after the IMU died until a page reload.
    const imuStatus = diagnostics?.status?.find((s) => s.name === "IMU");
    const imuAlive =
        imuStatus != null &&
        imuStatus.level < 2 && // OK or WARN; ERROR (2) / STALE (3) mean no fresh data
        (nowMs - imuStatus.receivedAt) < 15_000;
    const anatomyInputs = {
        batteryPct: batteryKnown ? batteryPercent : Number.NaN,
        vBattery: batteryKnown ? (power.v_battery ?? Number.NaN) : Number.NaN,
        motorTempC: blade.motorTemp,
        escTempC: blade.escTemp,
        gpsLabel: anatomyGps.label,
        gpsOk: health.checks.find(c => c.id === 'gnss')?.level === 'unknown' ? undefined
            : health.checks.find(c => c.id === 'gnss')?.level === 'ok' ? true : false,
        imuYawDeg: yaw,
        imuOk: imuStatus ? imuAlive : undefined,
        lidarDisabled: lidarEnabled === false,
        lidarOk: lidarStreaming ? true : undefined,
        // Mowgli is rear-axle drive: only the rear wheels are encoded (the
        // fronts are unencoded casters whose RPM is always 0).
        wheelLeftRpm: wheelRpm.rl,
        wheelRightRpm: wheelRpm.rr,
        bladeOn: Number.isFinite(blade.rpm) ? blade.rpm > 0 : undefined,
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
        } catch (e) {
            setFirmwareDebugTarget(null);
            setFirmwareDebugLoading(false);
            notification.error({
                message: t("diagnosticsPage.firmwareDebugToggleFailed"),
                description: errorMessage(e) ?? t("diagnosticsPage.unknownError"),
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
                            onClick={voidify(refresh)}
                        >
                            {t('diagnosticsPage.refresh')}
                        </Button>
                    }
                >
                    {snapshotReadError && <Alert data-testid="diagnostic-snapshot-read-error" type="error" showIcon
                        style={{marginBottom: 12}} message={t('diagnosticsPage.snapshotErrorTitle')} description={snapshotReadError}/>}
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
                        className="diagnostics-three-digit-statistic"
                        title={`${t('diagnosticsPage.cpuUsage')} (%)`}
                        value={cpuUsage ?? "—"}
                        precision={1}
                    />
                    <Typography.Paragraph type="secondary" style={{fontSize: 12, marginTop: 4}}>
                        {t('diagnosticsPage.cpuUsageHint')}
                    </Typography.Paragraph>
                    <Statistic
                        title={t('diagnosticsPage.temperature')}
                        value={health.checks.find(c => c.id === 'cpu')?.level !== 'unknown' ? cpuTemp : '—'}
                        precision={1}
                        valueStyle={{
                            color: cpuTemp !== undefined && cpuTemp > 70 ? colors.danger : cpuTemp !== undefined && cpuTemp > 55 ? colors.warning : undefined,
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
                                onChange={voidify(toggleFirmwareDebug)}
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
                                        <span
                                            title={line.tsSource === 'received'
                                                ? t('logTime.approximate')
                                                : formatLogTime(line.tsMs, {withMillis: true})}
                                            style={{color: colors.textDim, marginRight: 8}}
                                        >
                                            {formatLogTime(line.tsMs, {timeOnly: true})}
                                        </span>
                                        {line.plain}
                                    </div>
                                ))
                            )}
                        </div>
                    )}
                </Card>
            </Col>
            <Col span={24}>
                <SystemPowerCard/>
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
                            tone={poseZ > 2 ? "danger" : poseZ > 0.5 ? "warn" : "default"}
                        />
                        <TelemetryStat span={8} title={t('diagnosticsPage.yawDeg')} value={yaw} precision={1}/>
                        <TelemetryStat span={8} title={t('diagnosticsPage.rollDeg')} value={roll} precision={1}/>
                        <TelemetryStat span={8} title={t('diagnosticsPage.pitchDeg')} value={pitch} precision={1}/>
                        <Col span={8}>
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
        } catch (e) {
            notification.error({message: t('diagnosticsPage.rosbagStartFailed'), description: errorMessage(e)});
        }
    };
    const handleRosbagStop = async () => {
        try {
            const r = await rosbag.stop();
            notification.success({message: t('diagnosticsPage.rosbagStopped'), description: r?.stopped_name});
        } catch (e) {
            notification.error({message: t('diagnosticsPage.rosbagStopFailed'), description: errorMessage(e)});
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
                } catch (e) {
                    notification.error({message: t('diagnosticsPage.rosbagDeleteFailed'), description: errorMessage(e)});
                }
            },
        });
    };

    const rosbagActive = rosbag.status?.active ?? false;
    const rosbagRecordings = rosbag.status?.recordings ?? [];

    const sectionRosbag = (
        <Row gutter={[12, 12]}>
            <Col span={24}><BlackboxPanel/></Col>
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
                                onClick={voidify(handleRosbagStart)}
                                loading={rosbag.busy === "start"}
                                disabled={rosbagActive || rosbag.busy !== null}
                            >
                                {t('diagnosticsPage.rosbagStart')}
                            </Button>
                            <Button
                                size="small"
                                danger
                                icon={<StopOutlined/>}
                                onClick={voidify(handleRosbagStop)}
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
                                    render: (d: string) => formatAbsolute(d),
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
    type FusionCommand = "fusion_graph_save" | "fusion_graph_clear" | "fusion_graph_clear_lidar_map";
    const FUSION_SUCCESS_KEY: Record<FusionCommand, string> = {
        fusion_graph_save: "diagnosticsPage.graphSaved",
        fusion_graph_clear: "diagnosticsPage.graphCleared",
        fusion_graph_clear_lidar_map: "diagnosticsPage.lidarMapCleared",
    };
    const [fusionBusy, setFusionBusy] = useState<FusionCommand | null>(null);
    const callFusionService = async (command: FusionCommand) => {
        setFusionBusy(command);
        try {
            const res = await guiApi.mowglinext.callCreate(command, {});
            if (res.error) throw new Error(apiErrorMessage(res.error) ?? "service call failed");
            notification.success({
                message: t(FUSION_SUCCESS_KEY[command]),
                description: (res.data as { message?: string } | undefined)?.message,
            });
        } catch (e) {
            notification.error({message: t('diagnosticsPage.fusionGraphActionFailed'), description: errorMessage(e)});
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

    const confirmClearLidarMap = () => {
        modal.confirm({
            title: t('diagnosticsPage.clearLidarMapConfirmTitle'),
            content: t('diagnosticsPage.clearLidarMapConfirmBody'),
            okText: t('diagnosticsPage.clearLidarMapConfirmOk'),
            cancelText: t('diagnosticsPage.calibrationCancel'),
            okButtonProps: {danger: true},
            onOk: () => callFusionService("fusion_graph_clear_lidar_map"),
        });
    };

    // Sample after the callback that supplied stats, in the same monotonic clock.
    // The one-second wall-clock render ticker can precede a newer callback.
    const fusionNow = performance.now();
    const fusionAgeS = fusionStats?.receivedMonotonic === undefined ? null
        : Math.floor((fusionNow - fusionStats.receivedMonotonic) / 1000);
    const sourceAgeS = fusionStats?.sourceIdentity && fusionStats.distinctMonotonic !== undefined
        ? Math.floor((fusionNow - fusionStats.distinctMonotonic) / 1000) : null;
    const fusionStale = fusionAgeS === null || fusionAgeS > 5 || (sourceAgeS !== null && sourceAgeS > 5);
    const fv = fusionStats?.values ?? {};
    const num = (k: string) => {
        const raw = fv[k];
        if (raw === undefined) return null;
        const n = Number(raw);
        return Number.isFinite(n) ? n : null;
    };
    const totalNodes = num("total_nodes");
    const covXX = num("cov_xx");
    const covYY = num("cov_yy");
    const covYaw = num("cov_yawyaw");
    const sigmaXY = (covXX !== null && covYY !== null && covXX >= 0 && covYY >= 0)
        ? Math.sqrt((covXX + covYY) / 2.0) * 100  // → cm
        : null;
    const sigmaYawDeg = (covYaw !== null && covYaw >= 0)
        ? Math.sqrt(covYaw) * (180 / Math.PI)
        : null;
    const gpsRejWrongfix = num("gps_rejects_wrongfix");
    const stationaryHandPush = num("stationary_hand_push");

    // LiDAR map anchor (optional scan-to-map particle filter). null when the
    // running node predates it, in which case the tile group is not rendered.
    const lidarAnchor = deriveLidarAnchor(fv);
    const LIDAR_ANCHOR_STATE_LABEL = {
        off: "diagnosticsPage.lidarAnchorStateOff",
        waitingForMap: "diagnosticsPage.lidarAnchorStateWaitingForMap",
        mapping: "diagnosticsPage.lidarAnchorStateMapping",
        anchoring: "diagnosticsPage.lidarAnchorStateAnchoring",
    } as const;
    const LIDAR_ANCHOR_STATE_TONE = {off: "default", waitingForMap: "warn", mapping: "default", anchoring: "ok"} as const;
    const LIDAR_ANCHOR_VERDICT_LABEL = {
        accepted: "diagnosticsPage.lidarAnchorVerdictAccepted",
        rejectedScore: "diagnosticsPage.lidarAnchorVerdictRejectedScore",
        rejectedSpread: "diagnosticsPage.lidarAnchorVerdictRejectedSpread",
        rejectedDeadReckoning: "diagnosticsPage.lidarAnchorVerdictRejectedDeadReckoning",
    } as const;
    const lidarAnchorStateText = lidarAnchor
        ? `${t(LIDAR_ANCHOR_STATE_LABEL[lidarAnchor.stateKey])}${lidarAnchor.isShadow ? ` ${t('diagnosticsPage.lidarAnchorShadowSuffix')}` : ""}`
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
                                {fusionStale ? '—' : (fusionStats?.message ?? t('diagnosticsPage.running'))}
                            </Tag>
                        </Space>
                    }
                    size="small"
                    extra={
                        <Space>
                            <Button
                                size="small"
                                onClick={() => { void callFusionService("fusion_graph_save"); }}
                                loading={fusionBusy === "fusion_graph_save"}
                                disabled={fusionBusy !== null}
                            >
                                {t('diagnosticsPage.saveGraph')}
                            </Button>
                            <Button
                                size="small"
                                danger
                                onClick={confirmClearGraph}
                                loading={fusionBusy === "fusion_graph_clear"}
                                disabled={fusionBusy !== null}
                            >
                                {t('diagnosticsPage.clearGraph')}
                            </Button>
                            {lidarAnchor !== null && (
                                <Button
                                    size="small"
                                    danger
                                    onClick={confirmClearLidarMap}
                                    loading={fusionBusy === "fusion_graph_clear_lidar_map"}
                                    disabled={fusionBusy !== null}
                                >
                                    {t('diagnosticsPage.clearLidarMap')}
                                </Button>
                            )}
                        </Space>
                    }
                >
                    <Row gutter={[12, 12]}>
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.nodesInGraph')}
                            value={totalNodes}
                        />
                        <TelemetryStat
                            xs={12} md={6} large
                            title={`${t('diagnosticsPage.poseSigma')} (cm)`}
                            value={sigmaXY}
                            precision={1}
                            tone={sigmaXY === null ? "danger" : sigmaXY < 5 ? "ok" : sigmaXY < 20 ? "warn" : "danger"}
                            hint={sigmaYawDeg !== null ? t('diagnosticsPage.yawSigma', {value: sigmaYawDeg.toFixed(2)}) : ""}
                        />
                    </Row>
                    <Row gutter={[12, 12]} style={{marginTop: 4}}>
                        <TelemetryStat
                            xs={12} md={6} large
                            title={t('diagnosticsPage.handPushTitle')}
                            value={stationaryHandPush}
                            tone={(stationaryHandPush ?? 0) > 0 ? "warn" : "default"}
                            hint={t('diagnosticsPage.gpsWrongfix', {count: gpsRejWrongfix ?? 0})}
                        />
                    </Row>
                    {lidarAnchor !== null && (
                        <>
                            <Typography.Text strong style={{display: "block", marginTop: 12, marginBottom: 4}}>
                                {t('diagnosticsPage.lidarAnchorGroupTitle')}
                            </Typography.Text>
                            <Row gutter={[12, 12]}>
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorState')}
                                    value={lidarAnchorStateText}
                                    tone={LIDAR_ANCHOR_STATE_TONE[lidarAnchor.stateKey]}
                                    hint={lidarAnchor.sigmaM !== null ? t('diagnosticsPage.lidarAnchorSigma', {value: lidarAnchor.sigmaM.toFixed(2)}) : ""}
                                />
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorMapCells')}
                                    value={lidarAnchor.mapCells}
                                    hint={t('diagnosticsPage.lidarAnchorSeeds', {seeds: lidarAnchor.seeds ?? 0, skipped: lidarAnchor.skipped ?? 0})}
                                />
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorVerdict')}
                                    value={lidarAnchor.verdictKey !== null ? t(LIDAR_ANCHOR_VERDICT_LABEL[lidarAnchor.verdictKey]) : t("diagnosticsPage.lidarNoEstimate")}
                                    tone={lidarAnchor.verdictKey === null ? "default" : fusionStale ? "warn" : lidarAnchor.verdictKey === "accepted" ? "ok" : "warn"}
                                    hint={lidarAnchor.hitRatioPct !== null ? t('diagnosticsPage.lidarAnchorHitRatio', {pct: lidarAnchor.hitRatioPct}) : ""}
                                />
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorFactors')}
                                    value={lidarAnchor.factors}
                                    tone={(lidarAnchor.factors ?? 0) > 0 ? "ok" : "default"}
                                    hint={t('diagnosticsPage.lidarAnchorFactorsHint')}
                                />
                            </Row>
                            <Row gutter={[12, 12]} style={{marginTop: 4}}>
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorAccepted')}
                                    value={lidarAnchor.accepted}
                                    tone={lidarAnchor.rejected > 0 ? "warn" : "default"}
                                    hint={t('diagnosticsPage.lidarAnchorRejectBreakdown', {
                                        rejected: lidarAnchor.rejected,
                                        score: lidarAnchor.rejScore,
                                        spread: lidarAnchor.rejSpread,
                                        dr: lidarAnchor.rejDr,
                                    })}
                                />
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorReseeds')}
                                    value={lidarAnchor.reseeds}
                                    tone={(lidarAnchor.reseeds ?? 0) > 0 ? "warn" : "default"}
                                />
                                <TelemetryStat
                                    xs={12} md={6} large
                                    title={t('diagnosticsPage.lidarAnchorOdomRebases')}
                                    value={lidarAnchor.odomRebases}
                                />
                            </Row>
                        </>
                    )}
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginTop: 8, marginBottom: 0}}>
                        {t('diagnosticsPage.fusionGraphDescPart1')}{" "}
                        <Typography.Text code>/ros2_ws/maps/fusion_graph.*</Typography.Text>;
                        {t('diagnosticsPage.fusionGraphDescPart2')}
                        {t('diagnosticsPage.fusionGraphTopicLabel')} <Typography.Text code>/fusion_graph/diagnostics</Typography.Text>{" "}
                        {fusionAgeS !== null && <span>{t('diagnosticsPage.lastUpdateAgo', {seconds: fusionAgeS})}</span>}
                        <br />
                        {fusionStats?.sourceStamp
                            ? t('diagnosticsPage.sourcePublication', {stamp: fusionStats.sourceStamp})
                            : t('diagnosticsPage.sourceIdentityUnknown')}
                        {sourceAgeS !== null && sourceAgeS > 5 && <span> · {t('diagnosticsPage.sourceUnchanged', {seconds: sourceAgeS})}</span>}
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
                                    <Typography.Text strong className="diagnostics-live-label-nowrap">{t('diagnosticsPage.filter')}</Typography.Text>
                                    <Tag color="success">/odometry/filtered_map</Tag>
                                </Space>
                                <Statistic title={t('diagnosticsPage.yawDeg')} value={yaw} precision={1}/>
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
                                        {cogStale ? '—' : t('diagnosticsPage.live')}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title={t('diagnosticsPage.yawDeg')}
                                    value={cogYawDeg !== null ? cogYawDeg : "-"}
                                    precision={cogYawDeg !== null ? 1 : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ (°): {cogSigmaDeg !== null ? cogSigmaDeg.toFixed(2) : "—"}
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
                                        {magStale ? '—' : t('diagnosticsPage.live')}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title={t('diagnosticsPage.yawDeg')}
                                    value={magYawDeg !== null ? magYawDeg : "-"}
                                    precision={magYawDeg !== null ? 1 : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ (°): {magSigmaDeg !== null ? magSigmaDeg.toFixed(2) : "—"}
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
                                value={batteryKnown ? batteryPercent : "—"}
                                precision={0}
                                suffix={batteryKnown ? "%" : undefined}
                                valueStyle={{
                                    color: batteryKnown
                                        ? batteryPercent <= 20 ? colors.danger : batteryPercent <= 50 ? colors.warning : undefined
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.voltage')}
                                value={powerDisplayValue(power.v_battery)}
                                precision={2}

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
                                value={powerDisplayValue(clampTinyToZero(power.charge_current) ?? undefined)}
                                precision={2}
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
                                value={powerDisplayValue(clampTinyToZero(power.v_charge) ?? undefined)}
                                precision={2}
                            />
                        </Col>
                    </Row>
                    <div style={{marginTop: 12}}>
                        <Space wrap>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.emergency')}</Typography.Text>
                            <Tag color={emergency.active_emergency ? "error" : emergency.latched_emergency ? "warning" : "default"}>
                                {clearanceUnverified ? t("telemetry.emergencyClearanceUnverified") : emergency.active_emergency
                                    ? (emergency.reason ?? t('diagnosticsPage.emergencyActive'))
                                    : emergency.latched_emergency
                                        ? t('diagnosticsPage.latched')
                                        : emergencyKnown ? t('diagnosticsPage.clear') : t('telemetry.statusUnknown')}
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
                                showInfo={false}
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
                                    title={`${t('diagnosticsPage.colCoverage')} (%)`}
                                    value={highLevelStatus.coverage_percent}
                                    precision={1}
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

    const crossChecks = snapshotFresh ? snapshot?.cross_checks : undefined;
    const crossCheckStatus = crossChecks?.overall_status ?? "unknown";

    const sectionCrossChecks = (
        <Row gutter={[12, 12]}>
            <Col xs={24}>
                <Card
                    title={t('diagnosticsPage.configurationCrossChecks')}
                    size="small"
                    extra={
                        <Tag color={
                            crossCheckStatus === "ok" ? "success" :
                            crossCheckStatus === "warn" ? "warning" : crossCheckStatus === "unknown" ? "default" : "error"
                        }>
                            {crossCheckStatus === "ok" ? t('diagnosticsPage.crossCheckOk') :
                                crossCheckStatus === "warn" ? t('diagnosticsPage.crossCheckWarn') :
                                    crossCheckStatus === "unknown" ? t("telemetry.statusUnknown") : t('diagnosticsPage.crossCheckError')}
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
                            {crossCheckStatus === 'unknown' ? t('telemetry.statusUnknown') : t('diagnosticsPage.noWarnings')}
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
                                    value={Number.isFinite(crossChecks.dock_pose.configured_yaw)
                                        ? (crossChecks.dock_pose.configured_yaw * 180 / Math.PI).toFixed(1)
                                        : "—"}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    className="diagnostics-coordinate-value"
                                    title={t('diagnosticsPage.datumLat')}
                                    value={crossChecks.dock_pose.datum_lat}
                                    precision={9}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    className="diagnostics-coordinate-value"
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
                void refreshCalibration();
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
                    const data = (await res.json()) as { success?: boolean; message?: string };
                    if (data.success) {
                        notification.success({message: t('diagnosticsPage.magCalibrationComplete'), description: data.message || t('diagnosticsPage.refreshingStatus')});
                    } else {
                        notification.error({message: t('diagnosticsPage.magCalibrationFailed'), description: data.message || t('diagnosticsPage.unknownError')});
                    }
                    void refreshCalibration();
                } catch (e) {
                    notification.error({
                        message: t('diagnosticsPage.magCalibrationFailed'),
                        description: isAbortError(e)
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

    const dockCal = calibrationStatus?.dock;
    const imuCal = calibrationStatus?.imu;
    const magCal = calibrationStatus?.mag;

    const sectionCalibrationStatus = (
        <Row gutter={[12, 12]}>
            {calibrationReadError && <Col xs={24}><Alert data-testid="diagnostic-calibration-read-error" type="error" showIcon
                message={t('diagnosticsPage.calibrationStatusErrorTitle')} description={calibrationReadError}/></Col>}
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
                            <Descriptions.Item label={`${t('diagnosticsPage.position')} (m)`}>
                                ({dockCal.dock_pose_x?.toFixed(3)}, {dockCal.dock_pose_y?.toFixed(3)})
                            </Descriptions.Item>
                            <Descriptions.Item label={`${t('diagnosticsPage.yaw')} (°)`}>
                                {dockCal.dock_pose_yaw_deg?.toFixed(2)}
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
                                {formatAbsolute(imuCal.calibrated_at)}
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
                                {imuCal.implied_pitch_deg?.toFixed(2)} / {imuCal.implied_roll_deg?.toFixed(2)}
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
                                {formatAbsolute(magCal.calibrated_at)}
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
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.angVelX')} value={imu.angular_velocity?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.angVelY')} value={imu.angular_velocity?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.angVelZ')} value={imu.angular_velocity?.z} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.linAccX')} value={imu.linear_acceleration?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.linAccY')} value={imu.linear_acceleration?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic className="diagnostics-sign-statistic" title={t('diagnosticsPage.linAccZ')} value={imu.linear_acceleration?.z} precision={4}/>
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={t('diagnosticsPage.wheelOdometry')} size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={12}>
                            <Statistic
                                className="diagnostics-sign-statistic"
                                title={t('diagnosticsPage.linearVel')}
                                value={wheelOdom.twist?.twist?.linear?.x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                className="diagnostics-sign-statistic"
                                title={t('diagnosticsPage.angularVel')}
                                value={wheelOdom.twist?.twist?.angular?.z}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                className="diagnostics-sign-statistic"
                                title={t('diagnosticsPage.poseXM')}
                                value={wheelOdom.pose?.pose?.position?.x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                className="diagnostics-sign-statistic"
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
                            {role: 'left_wheel' as WheelRole, label: t('diagnosticsPage.rearLeft'),   rpm: wheelRpm.rl, ticks: wheelTicks.wheel_ticks_rl, dir: wheelTicks.wheel_direction_rl, validMask: 4},
                            {role: 'right_wheel' as WheelRole, label: t('diagnosticsPage.rearRight'),  rpm: wheelRpm.rr, ticks: wheelTicks.wheel_ticks_rr, dir: wheelTicks.wheel_direction_rr, validMask: 8},
                        // One wheel renderer: encoder readings plus optional source-normalized controller fields.
                        // Native wheel visibility remains unchanged.
                        // Rear-only on Mowgli — permanent "no encoder
                        // reading" slots are noise. Until the first message
                        // arrives (mask 0/undefined), show all four.
                        ].filter(w => {
                            const mask = wheelTicks.valid_wheels ?? 0;
                            return showEncoderWheel(mask, w.validMask, 'role' in w && w.role ? wheelControllers[w.role] : undefined);
                        }).map(w => {
                            const valid = ((wheelTicks.valid_wheels ?? 0) & w.validMask) !== 0;
                            return (
                                <Col key={w.label} xs={12} md={6}>
                                    <Statistic
                                        className="diagnostics-sign-statistic"
                                        title={w.label}
                                        value={valid && Number.isFinite(w.rpm) ? w.rpm : "—"}
                                        precision={0}
                                        valueStyle={{
                                            color: !valid || !Number.isFinite(w.rpm) ? colors.muted :
                                                   Math.abs(w.rpm) > 5 ? colors.success : undefined,
                                        }}
                                    />
                                    <Typography.Text className="diagnostics-live-value" type="secondary" style={{fontSize: 11}}>
                                        {valid && w.ticks !== undefined && (w.dir === 0 || w.dir === 1) ? (
                                            <>
                                                {t('diagnosticsPage.ticksWithDirection', {ticks: w.ticks?.toLocaleString() ?? '—', direction: w.dir === 1 ? t('diagnosticsPage.directionForward') : t('diagnosticsPage.directionReverse')})}
                                            </>
                                        ) : (
                                            t('diagnosticsPage.noEncoderReading')
                                        )}
                                    </Typography.Text>
                                    {'role' in w && w.role && wheelControllers[w.role] && <Space direction="vertical" size={0} style={{marginTop: 8}}>
                                        <Typography.Text>{t('diagnosticsPage.escStatus')}: {t(`wheelTelemetry.states.${wheelControllers[w.role]!.state}`)}</Typography.Text>
                                        {wheelControllerFields(wheelControllers[w.role]).map(field => <Typography.Text key={field.key} type="secondary" style={{fontSize: 11}}>
                                            {t(`wheelTelemetry.${field.key}`)}: {motorDisplayValue(field.value, field.precision)}{Number.isFinite(field.value) && field.unit ? ` ${field.unit}` : ''}
                                        </Typography.Text>)}
                                    </Space>}
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
                        <Typography.Text code>{motorDisplayValue(wheelRpm.bodyOmega, 3)}</Typography.Text>{" "}
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
                                value={status.mower_status === 255 ? t('diagnosticsPage.statusOk') : status.mower_status === 0 ? t('diagnosticsPage.initializing') : t('telemetry.statusUnknown')}
                                valueStyle={{color: status.mower_status === 255 ? undefined : colors.warning}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title={t('diagnosticsPage.rain')}
                                value={status.rain_detected === true ? t('diagnosticsPage.detected') : status.rain_detected === false ? t('diagnosticsPage.none') : t('telemetry.statusUnknown')}
                                valueStyle={{color: status.rain_detected ? colors.warning : undefined}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title={t('diagnosticsPage.escStatus')}
                                value={ESC_STATUS[bladeEscStatus ?? -1]
                                    ? t(ESC_STATUS[bladeEscStatus ?? -1].labelKey)
                                    : t('diagnosticsPage.escUnknown', {code: bladeEscStatus ?? "—"})}
                                valueStyle={{
                                    color: ESC_STATUS[bladeEscStatus ?? -1]?.color === "error" ? colors.danger
                                        : ESC_STATUS[bladeEscStatus ?? -1]?.color === "warning" ? colors.warning
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={`${t('diagnosticsPage.escTemp')} (°C)`} value={Number.isFinite(blade.escTemp) ? blade.escTemp : "—"} precision={1}/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={`${t('diagnosticsPage.motorTemp')} (°C)`} value={Number.isFinite(blade.motorTemp) ? blade.motorTemp : "—"} precision={1}/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title={t('diagnosticsPage.motorRpm')} value={Number.isFinite(blade.rpm) ? blade.rpm : "—"} precision={0}/>
                        </Col>
                        <Col xs={24} lg={8}>
                            <BladeDirectionDisplay/>
                        </Col>
                    </Row>
                    <Flex wrap gap="small" style={{marginTop: 12}}>
                        <BoolStatusTag label={t('diagnosticsPage.rpiPower')} ok={status.raspberry_pi_power}/>
                        <BoolStatusTag label={t('diagnosticsPage.escPower')} ok={status.esc_power}/>
                        <BoolStatusTag label={t('diagnosticsPage.uiBoard')} ok={status.ui_board_available}/>
                        <BoolStatusTag label={t('diagnosticsPage.soundModule')} ok={status.sound_module_available}/>
                        <BoolStatusTag label={t('diagnosticsPage.mowEnabled')} ok={status.mow_enabled}/>
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

    const sectionAlerts = <DiagnosticAlertsPanel alerts={alerts} observations={diagnostics.status}
        connected={connected} now={nowMonotonic} sources={receptionSources}/>;

    // ── layout ───────────────────────────────────────────────────────────────

    if (isMobile) {
        return (
            <div className="diagnostics-page" style={{display: "flex", flexDirection: "column", gap: 12, paddingBottom: 8}}>
                {healthSummary}
                {sectionAlerts}
                <Collapse
                    activeKey={openPanels}
                    onChange={(keys) => setOpenPanels(Array.isArray(keys) ? keys.map(String) : [String(keys)])}
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
                            label: <Space><VideoCameraOutlined/> {t('blackbox.recordingsTab')}</Space>,
                            children: sectionRosbag,
                        },
                        {
                            key: "ros",
                            label: t('diagnosticsPage.rosDiagnostics'),
                            children: sectionRosDiagnostics,
                        },
                        {
                            key: "parameters",
                            label: <Space><SlidersOutlined/> {t('diagnosticsPage.tabParameters')}</Space>,
                            children: openPanels.includes("parameters") ? sectionParameters : null,
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
        {
            key: "parameters",
            label: <Space><SlidersOutlined/> {t('diagnosticsPage.tabParameters')}</Space>,
            children: activeTab === "parameters" ? sectionParameters : null,
        },
    ];

    return (
        <div className="diagnostics-page" style={{display: "flex", flexDirection: "column", gap: 16, width: "100%"}}>
            {healthSummary}
            {sectionAlerts}
            <Tabs activeKey={activeTab} onChange={setActiveTab} items={tabItems} size="large"/>
        </div>
    );
};

export default DiagnosticsPage;
