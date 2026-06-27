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
    Table,
    Tabs,
    Tag,
    Typography,
} from "antd";
import {
    ApiOutlined,
    CloudServerOutlined,
    CompassOutlined,
    DashboardOutlined,
    ReloadOutlined,
    SoundOutlined,
    ThunderboltOutlined,
    WarningOutlined,
    WifiOutlined,
} from "@ant-design/icons";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useGPS} from "../hooks/useGPS.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useIcpOdom} from "../hooks/useIcpOdom.ts";
import {useBTLog} from "../hooks/useBTLog.ts";
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
    deriveGpsStatus,
    gnssReceiverLabel,
    gnssRtkModeLabel,
    hasGnssCapability,
    readGnssBooleanState,
    readGnssNumber,
} from "../utils/gpsStatus.ts";
import {useEffect, useMemo, useState} from "react";
import {useTranslation} from "react-i18next";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {useApi} from "../hooks/useApi.ts";
import {useFusionGraphDiagnostics} from "../hooks/useFusionGraphDiagnostics.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {BTStateGraph} from "../components/BTStateGraph.tsx";
import {RobotAnatomy} from "../components/RobotAnatomy.tsx";
import {GnssStatusConstants} from "../types/ros.ts";
import {AlertOutlined} from "@ant-design/icons";
import {DashCard} from "../components/dashboard/Card.tsx";

// ── helpers ─────────────────────────────────────────────────────────────────

function yawFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * (180 / Math.PI);
}

function rollFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    return Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * (180 / Math.PI);
}

function pitchFromQuaternion(x = 0, y = 0, z = 0, w = 1): number {
    const sinp = 2 * (w * y - z * x);
    return Math.abs(sinp) >= 1 ? (Math.sign(sinp) * 90) : Math.asin(sinp) * (180 / Math.PI);
}

function secondsAgo(timestamp: string): number {
    return Math.floor((Date.now() - new Date(timestamp).getTime()) / 1000);
}

const DIAG_LEVEL_COLORS: Record<number, string> = {0: "success", 1: "warning", 2: "error", 3: "default"};
const DIAG_LEVEL_LABELS: Record<number, string> = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"};

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
    const icpOdom = useIcpOdom();
    const btNodeStates = useBTLog();
    const imu = useImu();
    const {imu: cogImu, lastMessageAt: cogLastAt} = useCogHeading();
    const {imu: magImu, lastMessageAt: magLastAt} = useMagYaw();
    const wheelOdom = useWheelOdom();
    const wheelTicks = useWheelTicks();
    const {status: calibrationStatus, refresh: refreshCalibration} = useCalibrationStatus();

    // Tick state once a second so the "Live/Stale" tags update even when no
    // new message has arrived (staleness is time-based, not message-driven).
    const [nowMs, setNowMs] = useState(Date.now());
    useEffect(() => {
        const id = setInterval(() => setNowMs(Date.now()), 1000);
        return () => clearInterval(id);
    }, []);
    const {snapshot, loading, refresh} = useDiagnosticsSnapshot();
    const {diagnostics} = useDiagnostics();
    const {settings} = useSettings();
    const wheelRpm = useWheelRpm({wheelRadiusM: settings?.wheel_radius ?? 0.04475});

    // ── derived values ───────────────────────────────────────────────────────

    const batteryPercent = useMemo(
        () => computeBatteryPercent(highLevelStatus.battery_percent, power.v_battery, settings),
        [highLevelStatus.battery_percent, power.v_battery, settings],
    );

    const gpsFix = useMemo(() => deriveGpsStatus(gnssStatus), [gnssStatus]);
    const gpsFixType = gpsFix.label;
    const gpsReceiver = gnssReceiverLabel(gnssStatus);
    const gpsRtkMode = gnssRtkModeLabel(gnssStatus);

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
    const gpsAccuracy = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
        gnssStatus.horizontal_accuracy_m,
    ) ?? gps.position_accuracy;
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

    // ── Health Verdict Hero ──────────────────────────────────────────────────
    // A single beginner-readable tile derived from the same booleans the
    // expert health bar uses. Emergency (or a stopped container / no GPS) is
    // "Urgence"; a soft warning (RTK-Float, warm CPU, low battery) is
    // "Attention requise"; otherwise "Tout va bien".

    const cpuHot = cpuTemp > 70;
    const cpuWarm = cpuTemp > 55 && cpuTemp <= 70;
    const batteryLow = batteryPercent <= 20;
    const batteryMid = batteryPercent > 20 && batteryPercent <= 50;

    const healthLevel: "ok" | "warn" | "danger" =
        emergency.active_emergency || !allContainersOk || (!gpsOk && !gpsWarn) || cpuHot || batteryLow
            ? "danger"
            : gpsWarn || cpuWarm || batteryMid
                ? "warn"
                : "ok";

    const healthVerdict =
        healthLevel === "danger" ? t('diagnosticsPage.verdictEmergency') :
        healthLevel === "warn" ? t('diagnosticsPage.verdictAttention') :
        t('diagnosticsPage.verdictAllGood');

    const healthColor =
        healthLevel === "danger" ? colors.danger :
        healthLevel === "warn" ? colors.warning :
        colors.primary;

    const healthSubtitle = emergency.active_emergency
        ? t('diagnosticsPage.subtitleEmergency')
        : !allContainersOk
            ? t('diagnosticsPage.subtitleContainerStopped')
            : healthLevel === "danger"
                ? t('diagnosticsPage.subtitleOutOfRange')
                : healthLevel === "warn"
                    ? t('diagnosticsPage.subtitleWatch')
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
                    color={batteryPercent > 50 ? "success" : batteryPercent > 20 ? "warning" : "error"}
                />
                <HealthBadge
                    label={emergency.active_emergency ? t('diagnosticsPage.emergencyUpper') : t('diagnosticsPage.noEmergency')}
                    color={emergency.active_emergency ? "error" : "success"}
                />
                <HealthBadge
                    label={cpuTemp > 0 ? `CPU: ${cpuTemp.toFixed(1)}°C` : "CPU: --"}
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
    // Yaw from quaternion (Z-axis). pose comes from /odometry/filtered_map.
    const ori = pose?.pose?.pose?.orientation;
    const yawDeg = ori
        ? (Math.atan2(2 * (ori.w * ori.z + ori.x * ori.y), 1 - 2 * (ori.y * ori.y + ori.z * ori.z)) * 180) / Math.PI
        : 0;
    const anatomyInputs = {
        batteryPct: batteryPercent,
        vBattery: power.v_battery ?? 0,
        motorTempC: status.mower_motor_temperature ?? 0,
        escTempC: status.mower_esc_temperature ?? 0,
        gpsLabel: anatomyGps.label,
        gpsOk: anatomyGps.percent >= 50,
        imuYawDeg: yawDeg,
        imuOk: imu != null && imu.angular_velocity != null,
        lidarOk: true,
        wheelLeftRpm: 0,
        wheelRightRpm: 0,
        bladeOn: (status.mower_motor_rpm ?? 0) > 0,
        rain: status.rain_detected ?? false,
        dockCharging: status.is_charging ?? false,
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
                <Card title={<Space><DashboardOutlined/> CPU</Space>} size="small">
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

    const zDriftColor = poseZ > 2 ? colors.danger : poseZ > 0.5 ? colors.warning : undefined;
    const flatCheck = Math.abs(roll) < 5 && Math.abs(pitch) < 5;
    const gpsFixColor = gpsFix.fixType === "RTK_FIX"
        ? colors.primary
        : gpsFix.fixType === "RTK_FLOAT"
            ? colors.warning
            : colors.danger;
    const gpsHdop = readGnssNumber(gnssStatus, GnssStatusConstants.CAP_HDOP, gnssStatus.hdop);
    const gpsVdop = readGnssNumber(gnssStatus, GnssStatusConstants.CAP_VDOP, gnssStatus.vdop);
    const gpsVerticalAccuracy = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_VERTICAL_ACCURACY,
        gnssStatus.vertical_accuracy_m,
    );
    const gpsHeading = readGnssNumber(gnssStatus, GnssStatusConstants.CAP_HEADING, gnssStatus.heading_deg);
    const gpsHeadingAccuracy = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_HEADING_ACCURACY,
        gnssStatus.heading_accuracy_deg,
    );
    const gpsSatellitesUsed = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_USED,
        gnssStatus.satellites_used,
    );
    const gpsSatellitesVisible = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_VISIBLE,
        gnssStatus.satellites_visible,
    );
    const gpsSatellitesTracked = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_TRACKED,
        gnssStatus.satellites_tracked,
    );
    const gpsCorrectionAge = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_CORRECTION_AGE,
        gnssStatus.correction_age_s,
    );
    const gpsMeanCn0 = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_MEAN_CN0,
        gnssStatus.mean_cn0_db_hz,
    );
    const gpsMaxCn0 = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_MAX_CN0,
        gnssStatus.max_cn0_db_hz,
    );
    const differentialState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_DIFFERENTIAL_CORRECTIONS,
        gnssStatus.differential_corrections,
    );
    const correctionsState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_CORRECTIONS_ACTIVE,
        gnssStatus.corrections_active,
    );
    const dualAntennaState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_DUAL_ANTENNA_STATUS,
        gnssStatus.dual_antenna_heading,
    );
    const interferenceState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_INTERFERENCE_STATUS,
        gnssStatus.interference_detected,
    );
    const jammingState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_JAMMING_STATUS,
        gnssStatus.jamming_detected,
    );
    const formatOptionalBool = (value: ReturnType<typeof readGnssBooleanState>) => {
        switch (value) {
            case "true":
                return t('diagnosticsPage.yes');
            case "false":
                return t('diagnosticsPage.no');
            case "unknown":
                return t('diagnosticsPage.unknown');
            case "unsupported":
            default:
                return t('diagnosticsPage.notAvailable');
        }
    };
    const typedGpsDetails = [
        {flag: GnssStatusConstants.CAP_SATELLITES_USED, label: t('diagnosticsPage.satellitesUsed'), value: gpsSatellitesUsed},
        {flag: GnssStatusConstants.CAP_SATELLITES_VISIBLE, label: t('diagnosticsPage.satellitesVisible'), value: gpsSatellitesVisible},
        {flag: GnssStatusConstants.CAP_SATELLITES_TRACKED, label: t('diagnosticsPage.satellitesTracked'), value: gpsSatellitesTracked},
        {flag: GnssStatusConstants.CAP_HDOP, label: "HDOP", value: gpsHdop, precision: 2},
        {flag: GnssStatusConstants.CAP_VDOP, label: "VDOP", value: gpsVdop, precision: 2},
        {flag: GnssStatusConstants.CAP_VERTICAL_ACCURACY, label: t('diagnosticsPage.verticalAccuracyM'), value: gpsVerticalAccuracy, precision: 3},
        {flag: GnssStatusConstants.CAP_HEADING, label: t('diagnosticsPage.headingDeg'), value: gpsHeading, precision: 1},
        {flag: GnssStatusConstants.CAP_HEADING_ACCURACY, label: t('diagnosticsPage.headingAccuracyDeg'), value: gpsHeadingAccuracy, precision: 2},
        {flag: GnssStatusConstants.CAP_CORRECTION_AGE, label: t('diagnosticsPage.correctionAgeS'), value: gpsCorrectionAge, precision: 1},
        {flag: GnssStatusConstants.CAP_MEAN_CN0, label: "Mean CN0 (dB-Hz)", value: gpsMeanCn0, precision: 1},
        {flag: GnssStatusConstants.CAP_MAX_CN0, label: "Max CN0 (dB-Hz)", value: gpsMaxCn0, precision: 1},
    ];

    const sectionLocalization = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><CompassOutlined/> {t('diagnosticsPage.filteredPose')}</Space>} size="small"
                      extra={pose.pose?.pose?.position ? <Tag color="success">{t('diagnosticsPage.live')}</Tag> : <Tag>{t('diagnosticsPage.waiting')}</Tag>}>
                    <Row gutter={[12, 12]}>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.xM')}
                                value={pose.pose?.pose?.position?.x ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.yM')}
                                value={pose.pose?.pose?.position?.y ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.zM')}
                                value={pose.pose?.pose?.position ? poseZ : "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.yawDeg')}
                                value={yaw}
                                precision={1}
                                suffix="°"

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.rollDeg')}
                                value={roll}
                                precision={1}
                                suffix="°"

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title={t('diagnosticsPage.pitchDeg')}
                                value={pitch}
                                precision={1}
                                suffix="°"

                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.zDrift')}
                                value={poseZ.toFixed(3)}
                                suffix="m"
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}
                            />
                        </Col>
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
                <Card title={<Space><WifiOutlined/> GPS</Space>} size="small">
                    <Row gutter={[12, 12]}>
                        <Col span={24}>
                            <Space>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.fixType')}</Typography.Text>
                                <Tag color={gpsFixColor === colors.primary ? "blue" : gpsFixColor === colors.warning ? "warning" : "error"}>
                                    {gpsFixType}
                                </Tag>
                            </Space>
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.latitude')}
                                value={gps.pose?.pose?.position?.x}
                                precision={7}

                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.longitude')}
                                value={gps.pose?.pose?.position?.y}
                                precision={7}

                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.altitudeM')}
                                value={gps.pose?.pose?.position?.z}
                                precision={3}

                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t('diagnosticsPage.horizontalAccuracyM')}
                                value={gpsAccuracy}
                                precision={3}
                                valueStyle={
                                    (gpsAccuracy ?? 0) > 0.1
                                        ? {color: colors.warning}
                                        : undefined
                                }

                            />
                        </Col>
                        <Col span={24}>
                            <Descriptions size="small" column={2}>
                                <Descriptions.Item label={t('diagnosticsPage.receiver')}>{gpsReceiver}</Descriptions.Item>
                                <Descriptions.Item label={t('diagnosticsPage.backend')}>{gnssStatus.backend || t('diagnosticsPage.unknownLower')}</Descriptions.Item>
                                <Descriptions.Item label={t('diagnosticsPage.rtkMode')}>{gpsRtkMode ?? t('diagnosticsPage.unknown')}</Descriptions.Item>
                                <Descriptions.Item label={t('diagnosticsPage.differentialCorrections')}>{formatOptionalBool(differentialState)}</Descriptions.Item>
                                <Descriptions.Item label={t('diagnosticsPage.correctionsActive')}>{formatOptionalBool(correctionsState)}</Descriptions.Item>
                                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_STATUS) && (
                                    <Descriptions.Item label={t('diagnosticsPage.dualAntennaHeading')}>
                                        {formatOptionalBool(dualAntennaState)}
                                    </Descriptions.Item>
                                )}
                                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_INTERFERENCE_STATUS) && (
                                    <Descriptions.Item label={t('diagnosticsPage.rfInterference')}>
                                        {formatOptionalBool(interferenceState)}
                                    </Descriptions.Item>
                                )}
                                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_JAMMING_STATUS) && (
                                    <Descriptions.Item label={t('diagnosticsPage.jamming')}>
                                        {formatOptionalBool(jammingState)}
                                    </Descriptions.Item>
                                )}
                                {typedGpsDetails
                                    .filter((item) => hasGnssCapability(gnssStatus, item.flag))
                                    .map((item) => (
                                        <Descriptions.Item key={item.label} label={item.label}>
                                            {item.value === undefined
                                                ? t('diagnosticsPage.unknown')
                                                : item.precision !== undefined
                                                    ? item.value.toFixed(item.precision)
                                                    : String(item.value)}
                                        </Descriptions.Item>
                                    ))}
                            </Descriptions>
                            {!typedGpsDetails.some((item) => hasGnssCapability(gnssStatus, item.flag)) && (
                                <Typography.Text type="secondary" style={{fontSize: 11}}>
                                    {t('diagnosticsPage.advancedGnssUnavailable')}
                                </Typography.Text>
                            )}
                        </Col>
                    </Row>
                </Card>
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

    // Wrap angle difference into (-180, 180].
    const wrap180 = (d: number) => ((d + 180) % 360 + 360) % 360 - 180;
    const deltaFilterMag = (!magStale && magYawDeg !== null) ? wrap180(yaw - magYawDeg) : null;
    const deltaFilterCog = (!cogStale && cogYawDeg !== null) ? wrap180(yaw - cogYawDeg) : null;

    // ── Fusion Graph (iSAM2) panel ───────────────────────────────────────────
    // fusion_graph_node is the sole map-frame localizer; the panel
    // surfaces the per-tick GraphStats it publishes on
    // /fusion_graph/diagnostics + the Save/Clear service actions.

    const guiApi = useApi();
    const mowerAction = useMowerAction();
    const resetEmergencyAction = mowerAction("emergency", {Emergency: 0});
    const {stats: fusionStats} = useFusionGraphDiagnostics();
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
    // Fraction of received scans that actually became graph factors.
    const attachRate = (scansReceived !== null && scansReceived > 0 && scansAttached !== null)
        ? Math.round((scansAttached / scansReceived) * 100)
        : null;

    // ICP-only odom vs fused estimate (heading + position drift). icpOdom is
    // empty until scan-matching is on AND a match has been accepted.
    const icpPos = icpOdom.pose?.pose?.position;
    const icpOri = icpOdom.pose?.pose?.orientation;
    const fusedPos = pose.pose?.pose?.position;
    const fusedOri = pose.pose?.pose?.orientation;
    const icpActive = icpPos !== undefined && icpOri !== undefined;
    const icpYawDeg = icpActive ? yawFromQuaternion(icpOri.x, icpOri.y, icpOri.z, icpOri.w) : null;
    const fusedYawDeg = fusedOri ? yawFromQuaternion(fusedOri.x, fusedOri.y, fusedOri.z, fusedOri.w) : null;
    const icpYawDiffDeg = (icpYawDeg !== null && fusedYawDeg !== null)
        ? (() => { let d = icpYawDeg - fusedYawDeg; while (d > 180) d -= 360; while (d < -180) d += 360; return d; })()
        : null;
    const icpPosDiffM = (icpActive && fusedPos !== undefined)
        ? Math.hypot(icpPos.x - fusedPos.x, icpPos.y - fusedPos.y)
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
                                onClick={() => callFusionService("fusion_graph_clear")}
                                loading={fusionBusy === "clear"}
                                disabled={fusionBusy !== null}
                            >
                                {t('diagnosticsPage.clearGraph')}
                            </Button>
                        </Space>
                    }
                >
                    <Row gutter={[12, 12]}>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.nodesInGraph')}
                                value={totalNodes ?? "—"}
                                valueStyle={{fontSize: 18}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {scansAttached !== null ? t('diagnosticsPage.withScans', {count: scansAttached}) : ""}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.loopClosures')}
                                value={loopClosures ?? "—"}
                                valueStyle={{fontSize: 18, color: (loopClosures ?? 0) > 0 ? colors.success : undefined}}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpSuccessRate')}
                                value={scanRate !== null ? scanRate : "—"}
                                suffix={scanRate !== null ? "%" : undefined}
                                precision={0}
                                valueStyle={{fontSize: 18}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {scanTotal > 0 ? t('diagnosticsPage.matches', {ok: scanOk, total: scanTotal}) : t('diagnosticsPage.scansReceived', {count: scansReceived ?? 0})}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.poseSigma')}
                                value={sigmaXY !== null ? sigmaXY : "—"}
                                suffix={sigmaXY !== null ? "cm" : undefined}
                                precision={1}
                                valueStyle={{
                                    fontSize: 18,
                                    color: sigmaXY !== null && sigmaXY < 5 ? colors.success : sigmaXY !== null && sigmaXY < 20 ? colors.warning : colors.danger,
                                }}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {sigmaYawDeg !== null ? t('diagnosticsPage.yawSigma', {value: sigmaYawDeg.toFixed(2)}) : ""}
                            </Typography.Text>
                        </Col>
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

    const sectionIcpMonitor = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={
                        <Space>
                            <CompassOutlined/>
                            {t('diagnosticsPage.icpMonitorTitle')}
                            <Tag color={fusionStale ? "default" : "processing"}>
                                {fusionStale ? t('diagnosticsPage.stale') : t('diagnosticsPage.live')}
                            </Tag>
                        </Space>
                    }
                    size="small"
                >
                    <Row gutter={[12, 12]}>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpScanMatchRate')}
                                value={scanRate !== null ? scanRate : "—"}
                                suffix={scanRate !== null ? "%" : undefined}
                                precision={0}
                                valueStyle={{
                                    fontSize: 18,
                                    color: scanRate === null ? undefined : scanRate >= 95 ? colors.success : scanRate >= 80 ? colors.warning : colors.danger,
                                }}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {t('diagnosticsPage.matches', {ok: scanOk ?? 0, total: scanTotal})}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpRejects')}
                                value={rejTotal}
                                valueStyle={{fontSize: 18, color: rejTotal > 0 ? colors.warning : colors.success}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {t('diagnosticsPage.icpRejectBreakdown', {
                                    rmse: rejRmse ?? 0,
                                    inliers: rejInliers ?? 0,
                                    sanity: rejSanity ?? 0,
                                    diverge: rejDiverge ?? 0,
                                })}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpKeyframes')}
                                value={keyframesTotal ?? "—"}
                                valueStyle={{fontSize: 18, color: (keyframesTotal ?? 0) > 0 ? colors.success : colors.warning}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {kfTotal > 0
                                    ? t('diagnosticsPage.icpKfMatches', {rate: kfRate ?? 0, ok: kfOk ?? 0, total: kfTotal})
                                    : t('diagnosticsPage.icpNoKeyframes')}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.loopClosures')}
                                value={loopClosures ?? "—"}
                                valueStyle={{fontSize: 18, color: (loopClosures ?? 0) > 0 ? colors.success : undefined}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {attachRate !== null ? t('diagnosticsPage.icpAttachRate', {rate: attachRate}) : ""}
                            </Typography.Text>
                        </Col>
                    </Row>
                    <Row gutter={[12, 12]} style={{marginTop: 4}}>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpHeading')}
                                value={icpYawDeg !== null ? icpYawDeg : "—"}
                                suffix={icpYawDeg !== null ? "°" : undefined}
                                precision={1}
                                valueStyle={{fontSize: 18}}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.fusedHeading')}
                                value={fusedYawDeg !== null ? fusedYawDeg : "—"}
                                suffix={fusedYawDeg !== null ? "°" : undefined}
                                precision={1}
                                valueStyle={{fontSize: 18}}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpHeadingDiff')}
                                value={icpYawDiffDeg !== null ? icpYawDiffDeg : "—"}
                                suffix={icpYawDiffDeg !== null ? "°" : undefined}
                                precision={1}
                                valueStyle={{
                                    fontSize: 18,
                                    color: icpYawDiffDeg === null ? undefined
                                        : Math.abs(icpYawDiffDeg) < 5 ? colors.success
                                        : Math.abs(icpYawDiffDeg) < 15 ? colors.warning : colors.danger,
                                }}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title={t('diagnosticsPage.icpPosDiff')}
                                value={icpPosDiffM !== null ? icpPosDiffM : "—"}
                                suffix={icpPosDiffM !== null ? "m" : undefined}
                                precision={2}
                                valueStyle={{fontSize: 18}}
                            />
                        </Col>
                    </Row>
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginTop: 8, marginBottom: 0}}>
                        {t('diagnosticsPage.icpGpsWrongfix', {count: gpsRejWrongfix ?? 0})}
                        {" · "}
                        {t('diagnosticsPage.icpStationaryPush', {count: stationaryHandPush ?? 0})}
                        {" · "}
                        {t('diagnosticsPage.icpMonitorHint')}
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

    const coverageColumns = [
        {title: t('diagnosticsPage.colArea'), dataIndex: "area_index", key: "area_index"},
        {
            title: t('diagnosticsPage.colCoverage'),
            dataIndex: "coverage_percent",
            key: "coverage_percent",
            render: (v: number) => <Progress percent={Math.round(v)} size="small" style={{minWidth: 80}}/>,
        },
        {title: t('diagnosticsPage.colTotalCells'), dataIndex: "total_cells", key: "total_cells"},
        {title: t('diagnosticsPage.colMowed'), dataIndex: "mowed_cells", key: "mowed_cells"},
        {title: t('diagnosticsPage.colObstacles'), dataIndex: "obstacle_cells", key: "obstacle_cells"},
        {title: t('diagnosticsPage.colStripsLeft'), dataIndex: "strips_remaining", key: "strips_remaining"},
    ];

    const sectionBtCoverage = (
        <Row gutter={[12, 12]}>
            <Col xs={24}>
                <BTStateGraph current={highLevelStatus.state_name}/>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={<Space><ApiOutlined/> {t('diagnosticsPage.btState')}</Space>} size="small">
                    <Space direction="vertical" style={{width: "100%"}}>
                        <Space>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>{t('diagnosticsPage.state')}</Typography.Text>
                            <Tag color={btStateColor} style={{fontSize: 14, padding: "2px 12px"}}>
                                {highLevelStatus.state_name ?? "--"}
                            </Tag>
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
                                value={power.charge_current}
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
                                value={power.v_charge}
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
                                <Button
                                    danger
                                    size="small"
                                    icon={<AlertOutlined/>}
                                    onClick={resetEmergencyAction}
                                >
                                    {t('diagnosticsPage.resetEmergency')}
                                </Button>
                            )}
                        </Space>
                    </div>
                    {btNodeStates.size > 0 && (
                        <div style={{marginTop: 12}}>
                            <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 4}}>{t('diagnosticsPage.activeBtNodes')}</Typography.Text>
                            <Flex wrap gap={4}>
                                {Array.from(btNodeStates.entries())
                                    .filter(([, status]) => status === "RUNNING" || status === "SUCCESS")
                                    .map(([name, status]) => (
                                        <Tag
                                            key={name}
                                            color={status === "RUNNING" ? "processing" : status === "SUCCESS" ? "success" : "default"}
                                            style={{fontSize: 11}}
                                        >
                                            {name}
                                        </Tag>
                                    ))}
                            </Flex>
                        </div>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title={t('diagnosticsPage.coverage')} size="small">
                    <Table
                        size="small"
                        dataSource={snapshot?.coverage ?? []}
                        columns={coverageColumns}
                        rowKey="area_index"
                        pagination={false}
                        scroll={{x: "max-content"}}
                        locale={{emptyText: t('diagnosticsPage.noCoverageData')}}
                    />
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
                            {crossCheckStatus.toUpperCase()}
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
                                    precision={7}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title={t('diagnosticsPage.datumLon')}
                                    value={crossChecks.dock_pose.datum_lon}
                                    precision={7}
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

    const runImuCalibration = async () => {
        try {
            notification.info({
                message: t('diagnosticsPage.calibrationStarted'),
                description: t('diagnosticsPage.calibrationStartedDesc'),
            });
            const res = await fetch("/api/calibration/imu-yaw", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({duration_sec: 30}),
            });
            if (!res.ok) {
                throw new Error(`HTTP ${res.status}: ${await res.text()}`);
            }
            notification.success({message: t('diagnosticsPage.calibrationComplete'), description: t('diagnosticsPage.refreshingStatus')});
            refreshCalibration();
        } catch (e) {
            notification.error({
                message: t('diagnosticsPage.calibrationFailed'),
                description: e instanceof Error ? e.message : String(e),
            });
        }
    };

    const runMagCalibration = async () => {
        try {
            notification.info({
                message: t('diagnosticsPage.magCalibrationStarted'),
                description: t('diagnosticsPage.magCalibrationStartedDesc'),
                duration: 6,
            });
            const res = await fetch("/api/calibration/magnetometer", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({}),
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
        } catch (e) {
            notification.error({
                message: t('diagnosticsPage.magCalibrationFailed'),
                description: e instanceof Error ? e.message : String(e),
            });
        }
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
                        ].map(w => {
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
                        <Tag color={status.raspberry_pi_power ? "success" : "default"}>{t('diagnosticsPage.rpiPower')}</Tag>
                        <Tag color={status.esc_power ? "success" : "default"}>{t('diagnosticsPage.escPower')}</Tag>
                        <Tag color={status.ui_board_available ? "success" : "default"}>{t('diagnosticsPage.uiBoard')}</Tag>
                        <Tag color={status.sound_module_available ? "success" : "default"}>{t('diagnosticsPage.soundModule')}</Tag>
                        <Tag color={status.mow_enabled ? "success" : "default"}>{t('diagnosticsPage.mowEnabled')}</Tag>
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
                                    {DIAG_LEVEL_LABELS[item.level] ?? String(item.level)}
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

    const sectionAlerts = alerts.length > 0 ? (
        <Card title={<Space><WarningOutlined/> {t('diagnosticsPage.alerts')}</Space>} size="small">
            <Space direction="vertical" style={{width: "100%"}}>
                {alerts.map((item, idx) => (
                    <Alert
                        key={idx}
                        type={item.level === 2 ? "error" : item.level === 3 ? "info" : "warning"}
                        message={item.name}
                        description={item.message}
                        showIcon
                    />
                ))}
            </Space>
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
                            key: "icp_monitor",
                            label: <Space><CompassOutlined/> {t('diagnosticsPage.icpMonitorShort')}</Space>,
                            children: sectionIcpMonitor,
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
                {sectionRosDiagnostics}
            </Space>,
        },
        {
            key: "localization",
            label: <Space><CompassOutlined/> {t('diagnosticsPage.tabLocalization')}</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionLocalization}
                {sectionFusionGraph}
                {sectionIcpMonitor}
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
