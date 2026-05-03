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
import {useFusionOdom} from "../hooks/useFusionOdom.ts";
import {useBTLog} from "../hooks/useBTLog.ts";
import {useImu} from "../hooks/useImu.ts";
import {useCogHeading} from "../hooks/useCogHeading.ts";
import {useMagYaw} from "../hooks/useMagYaw.ts";
import {useCalibrationStatus} from "../hooks/useCalibrationStatus.ts";
import {useWheelTicks} from "../hooks/useWheelTicks.ts";
import {useDiagnosticsSnapshot} from "../hooks/useDiagnosticsSnapshot.ts";
import {useDiagnostics} from "../hooks/useDiagnostics.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useIsMobile} from "../hooks/useIsMobile";
import {AbsolutePoseConstants} from "../types/ros.ts";
import {useEffect, useMemo, useState} from "react";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {useApi} from "../hooks/useApi.ts";
import {useFusionGraphDiagnostics} from "../hooks/useFusionGraphDiagnostics.ts";
import {useMowerAction} from "../components/MowerActions.tsx";
import {AlertOutlined} from "@ant-design/icons";

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
const ESC_STATUS: Record<number, {label: string; color: string}> = {
    0:   {label: "Off", color: "default"},
    99:  {label: "Disconnected", color: "warning"},
    100: {label: "Error", color: "error"},
    150: {label: "Stalled", color: "error"},
    200: {label: "OK", color: "success"},
    201: {label: "Running", color: "success"},
};

// ── sub-components ───────────────────────────────────────────────────────────

function HealthBadge({label, color}: {label: string; color: string}) {
    return <Tag color={color} style={{fontSize: 12, padding: "2px 8px"}}>{label}</Tag>;
}

// ── main page ────────────────────────────────────────────────────────────────

export const DiagnosticsPage = () => {
    const {colors} = useThemeMode();
    const isMobile = useIsMobile();

    const {highLevelStatus} = useHighLevelStatus();
    const emergency = useEmergency();
    const power = usePower();
    const status = useStatus();
    const gps = useGPS();
    const pose = useFusionOdom();
    const btNodeStates = useBTLog();
    const imu = useImu();
    const {imu: cogImu, lastMessageAt: cogLastAt} = useCogHeading();
    const {imu: magImu, lastMessageAt: magLastAt} = useMagYaw();
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

    // ── derived values ───────────────────────────────────────────────────────

    const batteryPercent = useMemo(
        () => computeBatteryPercent(highLevelStatus.battery_percent, power.v_battery, settings),
        [highLevelStatus.battery_percent, power.v_battery, settings],
    );

    const gpsFlags = gps.flags ?? 0;
    const gpsFixType = useMemo(() => {
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK_FIXED) return "RTK FIX";
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK_FLOAT) return "RTK FLOAT";
        if (gpsFlags & AbsolutePoseConstants.FLAG_GPS_RTK) return "GPS FIX";
        return "No Fix";
    }, [gpsFlags]);

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
    const gpsOk = gpsFlags > 0 && (gps.position_accuracy ?? 999) <= 0.1;
    const gpsWarn = gpsFlags > 0 && (gps.position_accuracy ?? 999) > 0.1;
    const cpuTemp = snapshot?.system?.cpu_temperature ?? 0;

    const alerts = useMemo(
        () => (diagnostics.status ?? []).filter(s =>
            s.level >= 1 &&
            // Filter out transient "no data since last update" from ublox driver
            !s.message?.toLowerCase().includes("no data since last update")
        ),
        [diagnostics.status]
    );

    // Pull the 3 entries published by sensors/gps/gps_health_aggregator.py
    // and expose key/value lookups so the GPS card can render rich detail
    // (sat count, mean CN0, RTCM rate, etc.) instead of just the fix tag.
    const gpsHealth = useMemo(() => {
        const byName: Record<string, Record<string, string>> = {};
        for (const s of diagnostics.status ?? []) {
            if (!s.name?.startsWith("GPS: ")) continue;
            const kv: Record<string, string> = {};
            for (const v of s.values ?? []) {
                kv[v.key] = v.value;
            }
            byName[s.name] = { ...kv, _level: String(s.level), _message: s.message ?? "" };
        }
        return byName;
    }, [diagnostics.status]);

    // ── Health Summary Bar ───────────────────────────────────────────────────

    const healthBar = (
        <Card size="small" style={{marginBottom: 12}}>
            <Flex wrap gap="small" align="center">
                <Typography.Text type="secondary" style={{fontSize: 12, marginRight: 4}}>Health</Typography.Text>
                <HealthBadge
                    label={allContainersOk ? "Containers OK" : "Container Issue"}
                    color={allContainersOk ? "success" : "error"}
                />
                <HealthBadge
                    label={`GPS: ${gpsFixType}`}
                    color={gpsOk ? "success" : gpsWarn ? "warning" : "error"}
                />
                <HealthBadge
                    label={`Battery: ${batteryPercent.toFixed(0)}%`}
                    color={batteryPercent > 50 ? "success" : batteryPercent > 20 ? "warning" : "error"}
                />
                <HealthBadge
                    label={emergency.active_emergency ? "EMERGENCY" : "No Emergency"}
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
            title: "Name",
            dataIndex: "name",
            key: "name",
            render: (v: string) => <Typography.Text code style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: "State",
            dataIndex: "state",
            key: "state",
            render: (v: string) => <Tag color={v === "running" ? "success" : "error"}>{v}</Tag>,
        },
        {
            title: "Status",
            dataIndex: "status",
            key: "status",
            render: (v: string) => <Typography.Text style={{fontSize: 12}}>{v}</Typography.Text>,
        },
        {
            title: "Started",
            dataIndex: "started_at",
            key: "started_at",
            render: (v: string) => (
                <Typography.Text style={{fontSize: 12}}>
                    {v ? new Date(v).toLocaleTimeString() : "--"}
                </Typography.Text>
            ),
        },
    ];

    const sectionSystem = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={<Space><CloudServerOutlined/> Containers</Space>}
                    size="small"
                    extra={
                        <Button
                            size="small"
                            icon={<ReloadOutlined spin={loading}/>}
                            onClick={refresh}
                        >
                            Refresh
                        </Button>
                    }
                >
                    <Table
                        size="small"
                        dataSource={snapshot?.containers ?? []}
                        columns={containerColumns}
                        rowKey="name"
                        pagination={false}
                        locale={{emptyText: "No container data"}}
                    />
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card title={<Space><DashboardOutlined/> CPU</Space>} size="small">
                    <Statistic
                        title="Temperature"
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
                        Last snapshot: {secondsAgo(snapshot.timestamp)}s ago
                    </Typography.Text>
                </Col>
            )}
        </Row>
    );

    // ── Section 2: Localization ──────────────────────────────────────────────

    const zDriftColor = poseZ > 2 ? colors.danger : poseZ > 0.5 ? colors.warning : undefined;
    const flatCheck = Math.abs(roll) < 5 && Math.abs(pitch) < 5;
    const gpsFixColor = gpsFixType === "RTK FIX"
        ? colors.primary
        : gpsFixType === "RTK FLOAT"
            ? colors.warning
            : colors.danger;

    const sectionLocalization = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><CompassOutlined/> Filtered Pose (map frame)</Space>} size="small"
                      extra={pose.pose?.pose?.position ? <Tag color="success">Live</Tag> : <Tag>Waiting...</Tag>}>
                    <Row gutter={[12, 12]}>
                        <Col span={8}>
                            <Statistic
                                title="X (m)"
                                value={pose.pose?.pose?.position?.x ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Y (m)"
                                value={pose.pose?.pose?.position?.y ?? "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Z (m)"
                                value={pose.pose?.pose?.position ? poseZ : "-"}
                                precision={pose.pose?.pose?.position ? 3 : undefined}
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}

                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Yaw (deg)"
                                value={yaw}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Roll (deg)"
                                value={roll}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Pitch (deg)"
                                value={pitch}
                                precision={1}
                                suffix="°"
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Z Drift"
                                value={poseZ.toFixed(3)}
                                suffix="m"
                                valueStyle={zDriftColor ? {color: zDriftColor} : undefined}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Flat Check"
                                value={flatCheck ? "OK" : "DRIFT"}
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
                                <Typography.Text type="secondary" style={{fontSize: 12}}>Fix Type</Typography.Text>
                                <Tag color={gpsFixColor === colors.primary ? "blue" : gpsFixColor === colors.warning ? "warning" : "error"}>
                                    {gpsFixType}
                                </Tag>
                            </Space>
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="X (m)"
                                value={gps.pose?.pose?.position?.x}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Y (m)"
                                value={gps.pose?.pose?.position?.y}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Altitude (m)"
                                value={gps.pose?.pose?.position?.z}
                                precision={3}
                               
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Accuracy (m)"
                                value={gps.position_accuracy}
                                precision={3}
                                valueStyle={
                                    (gps.position_accuracy ?? 0) > 0.1
                                        ? {color: colors.warning}
                                        : undefined
                                }

                            />
                        </Col>

                        {/*
                          * Detail rows backed by sensors/gps/gps_health_aggregator.py.
                          * The aggregator pulls UBX-NAV-SAT / NAV-COV / RXM-RTCM and
                          * republishes structured key/values on /diagnostics, so we
                          * can show satellite counts, signal quality, and RTCM
                          * health without learning the raw UBX schemas in the GUI.
                          */}
                        {(() => {
                            const sat = gpsHealth["GPS: satellites"];
                            const rtcm = gpsHealth["GPS: NTRIP/RTCM"];
                            const fix = gpsHealth["GPS: fix"];
                            if (!sat && !rtcm && !fix) {
                                return (
                                    <Col span={24}>
                                        <Typography.Text type="secondary" style={{fontSize: 11}}>
                                            Waiting for /diagnostics from gps_health_aggregator…
                                        </Typography.Text>
                                    </Col>
                                );
                            }
                            const cnoMean = sat?.mean_cno_db_hz ? parseFloat(sat.mean_cno_db_hz) : null;
                            const cnoOk = cnoMean !== null && cnoMean >= 40;
                            const cnoWarn = cnoMean !== null && cnoMean >= 35 && cnoMean < 40;
                            const ageS = rtcm?.age_of_last_corr_s ? parseFloat(rtcm.age_of_last_corr_s) : null;
                            const sigmaMm = fix?.sigma_xy_mm && fix.sigma_xy_mm !== "n/a"
                                ? parseFloat(fix.sigma_xy_mm) : null;
                            return (
                                <>
                                    <Col span={12}>
                                        <Statistic
                                            title="Satellites (used / visible)"
                                            value={sat ? `${sat.used} / ${sat.visible}` : "—"}
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Statistic
                                            title="Mean CN0 (dB-Hz)"
                                            value={cnoMean ?? "—"}
                                            precision={1}
                                            valueStyle={
                                                cnoOk ? {color: colors.success}
                                                : cnoWarn ? {color: colors.warning}
                                                : cnoMean !== null ? {color: colors.danger}
                                                : undefined
                                            }
                                            suffix={sat?.cno_ge_40_count ? `(${sat.cno_ge_40_count} ≥40)` : undefined}
                                        />
                                    </Col>
                                    {sat?.constellations_used && (
                                        <Col span={24}>
                                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                                Constellations:{" "}
                                            </Typography.Text>
                                            <Typography.Text code style={{fontSize: 11}}>
                                                {sat.constellations_used}
                                            </Typography.Text>
                                        </Col>
                                    )}
                                    <Col span={12}>
                                        <Statistic
                                            title="RTCM (msg/s)"
                                            value={rtcm?.msgs_per_sec ?? "—"}
                                            suffix={rtcm?.msgs_used_pct ? `(${rtcm.msgs_used_pct}% used)` : undefined}
                                            valueStyle={
                                                rtcm && parseFloat(rtcm.msgs_per_sec) >= 1 ? {color: colors.success}
                                                : rtcm ? {color: colors.danger}
                                                : undefined
                                            }
                                        />
                                    </Col>
                                    <Col span={12}>
                                        <Statistic
                                            title="Last correction (s)"
                                            value={ageS ?? "—"}
                                            precision={1}
                                            valueStyle={
                                                ageS !== null && ageS > 5 ? {color: colors.danger}
                                                : ageS !== null && ageS > 2 ? {color: colors.warning}
                                                : ageS !== null ? {color: colors.success}
                                                : undefined
                                            }
                                        />
                                    </Col>
                                    {sigmaMm !== null && (
                                        <Col span={24}>
                                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                                σ<sub>xy</sub>:{" "}
                                            </Typography.Text>
                                            <Typography.Text code style={{fontSize: 11}}>
                                                {sigmaMm.toFixed(1)} mm
                                            </Typography.Text>
                                            {fix?.ttff_s && (
                                                <>
                                                    <Typography.Text type="secondary" style={{fontSize: 11, marginLeft: 12}}>
                                                        TTFF:{" "}
                                                    </Typography.Text>
                                                    <Typography.Text code style={{fontSize: 11}}>
                                                        {fix.ttff_s}s
                                                    </Typography.Text>
                                                </>
                                            )}
                                        </Col>
                                    )}
                                    {rtcm?.types_seen && (
                                        <Col span={24}>
                                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                                RTCM types:{" "}
                                            </Typography.Text>
                                            <Typography.Text code style={{fontSize: 11}}>
                                                {rtcm.types_seen}
                                            </Typography.Text>
                                        </Col>
                                    )}
                                </>
                            );
                        })()}
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
    // Only shown when the operator opted into the GTSAM factor-graph
    // localizer (use_fusion_graph=true). The panel surfaces the per-tick
    // GraphStats published on /fusion_graph/diagnostics + Save/Clear actions.

    const useFusionGraph = String((settings as any)?.use_fusion_graph ?? "false") === "true";
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
                message: command === "fusion_graph_save" ? "Graph saved" : "Graph cleared",
                description: (res.data as any)?.message,
            });
        } catch (e: any) {
            notification.error({message: "Fusion graph action failed", description: e.message});
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

    const sectionFusionGraph = useFusionGraph ? (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card
                    title={
                        <Space>
                            <CompassOutlined/>
                            Fusion Graph (iSAM2)
                            <Tag color={fusionStale ? "default" : (fusionStats?.level ?? 0) >= 1 ? "warning" : "success"}>
                                {fusionStale ? "Stale" : (fusionStats?.message ?? "running")}
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
                                Save graph
                            </Button>
                            <Button
                                size="small"
                                danger
                                onClick={() => callFusionService("fusion_graph_clear")}
                                loading={fusionBusy === "clear"}
                                disabled={fusionBusy !== null}
                            >
                                Clear graph
                            </Button>
                        </Space>
                    }
                >
                    <Row gutter={[12, 12]}>
                        <Col xs={12} md={6}>
                            <Statistic
                                title="Nodes in graph"
                                value={totalNodes ?? "—"}
                                valueStyle={{fontSize: 18}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {scansAttached !== null ? `${scansAttached} with scans` : ""}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title="Loop closures"
                                value={loopClosures ?? "—"}
                                valueStyle={{fontSize: 18, color: (loopClosures ?? 0) > 0 ? colors.success : undefined}}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title="ICP success rate"
                                value={scanRate !== null ? scanRate : "—"}
                                suffix={scanRate !== null ? "%" : undefined}
                                precision={0}
                                valueStyle={{fontSize: 18}}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {scanTotal > 0 ? `${scanOk}/${scanTotal} matches` : `${scansReceived ?? 0} scans received`}
                            </Typography.Text>
                        </Col>
                        <Col xs={12} md={6}>
                            <Statistic
                                title="Pose σ"
                                value={sigmaXY !== null ? sigmaXY : "—"}
                                suffix={sigmaXY !== null ? "cm" : undefined}
                                precision={1}
                                valueStyle={{
                                    fontSize: 18,
                                    color: sigmaXY !== null && sigmaXY < 5 ? colors.success : sigmaXY !== null && sigmaXY < 20 ? colors.warning : colors.danger,
                                }}
                            />
                            <Typography.Text type="secondary" style={{fontSize: 11}}>
                                {sigmaYawDeg !== null ? `yaw ±${sigmaYawDeg.toFixed(2)}°` : ""}
                            </Typography.Text>
                        </Col>
                    </Row>
                    <Typography.Paragraph type="secondary" style={{fontSize: 11, marginTop: 8, marginBottom: 0}}>
                        GTSAM iSAM2 factor graph. Save persists nodes + scans to{" "}
                        <Typography.Text code>/ros2_ws/maps/fusion_graph.*</Typography.Text>;
                        Clear wipes the graph and waits for the next GPS fix or set_pose to re-initialize.
                        Topic: <Typography.Text code>/fusion_graph/diagnostics</Typography.Text>{" "}
                        {fusionAgeS !== null && <span>· last update {fusionAgeS}s ago</span>}
                    </Typography.Paragraph>
                </Card>
            </Col>
        </Row>
    ) : null;

    const sectionHeadingSources = (
        <Row gutter={[12, 12]}>
            <Col span={24}>
                <Card title={<Space><CompassOutlined/> Heading sources {useFusionGraph ? "(fused by fusion_graph)" : "(fused by ekf_map)"}</Space>} size="small">
                    <Row gutter={[12, 12]}>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>Filter</Typography.Text>
                                    <Tag color="success">/odometry/filtered_map</Tag>
                                </Space>
                                <Statistic title="Yaw (deg)" value={yaw} precision={1} suffix="°"/>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>
                                    Reference signal for deltas below.
                                </Typography.Text>
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>COG (GPS)</Typography.Text>
                                    <Tag color={cogStale ? "default" : "processing"}>
                                        {cogStale ? "Stale" : "Live"}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={cogYawDeg !== null ? cogYawDeg : "-"}
                                    precision={cogYawDeg !== null ? 1 : undefined}
                                    suffix={cogYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {cogSigmaDeg !== null ? `${cogSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterCog !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        Δ(filter−cog): {deltaFilterCog.toFixed(1)}°
                                    </Typography.Text>
                                )}
                            </Space>
                        </Col>
                        <Col xs={24} md={8}>
                            <Space direction="vertical" style={{width: "100%"}}>
                                <Space>
                                    <Typography.Text strong>Magnetometer</Typography.Text>
                                    <Tag color={magStale ? "default" : "processing"}>
                                        {magStale ? "Stale" : "Live"}
                                    </Tag>
                                </Space>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={magYawDeg !== null ? magYawDeg : "-"}
                                    precision={magYawDeg !== null ? 1 : undefined}
                                    suffix={magYawDeg !== null ? "°" : undefined}
                                />
                                <Typography.Text type="secondary" style={{fontSize: 12}}>
                                    σ: {magSigmaDeg !== null ? `${magSigmaDeg.toFixed(2)}°` : "—"}
                                </Typography.Text>
                                {deltaFilterMag !== null && (
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>
                                        Δ(filter−mag): {deltaFilterMag.toFixed(1)}°
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
        {title: "Area", dataIndex: "area_index", key: "area_index"},
        {
            title: "Coverage",
            dataIndex: "coverage_percent",
            key: "coverage_percent",
            render: (v: number) => <Progress percent={Math.round(v * 100) / 100} size="small" style={{minWidth: 80}}/>,
        },
        {title: "Total Cells", dataIndex: "total_cells", key: "total_cells"},
        {title: "Mowed", dataIndex: "mowed_cells", key: "mowed_cells"},
        {title: "Obstacles", dataIndex: "obstacle_cells", key: "obstacle_cells"},
        {title: "Strips Left", dataIndex: "strips_remaining", key: "strips_remaining"},
    ];

    const sectionBtCoverage = (
        <Row gutter={[12, 12]}>
            <Col xs={24} lg={12}>
                <Card title={<Space><ApiOutlined/> BT State</Space>} size="small">
                    <Space direction="vertical" style={{width: "100%"}}>
                        <Space>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>State</Typography.Text>
                            <Tag color={btStateColor} style={{fontSize: 14, padding: "2px 12px"}}>
                                {highLevelStatus.state_name ?? "--"}
                            </Tag>
                        </Space>
                        {highLevelStatus.sub_state_name && (
                            <Space>
                                <Typography.Text type="secondary" style={{fontSize: 12}}>Sub-state</Typography.Text>
                                <Tag>{highLevelStatus.sub_state_name}</Tag>
                            </Space>
                        )}
                    </Space>
                    <Row gutter={[12, 12]} style={{marginTop: 12}}>
                        <Col span={8}>
                            <Statistic
                                title="Battery"
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
                                title="Voltage"
                                value={power.v_battery}
                                precision={2}
                                suffix="V"
                               
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Charging"
                                value={highLevelStatus.is_charging ? "Yes" : "No"}
                                valueStyle={{
                                    color: highLevelStatus.is_charging ? colors.primary : undefined,
                                }}
                            />
                        </Col>
                        <Col span={8}>
                            <Statistic
                                title="Charge current"
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
                                title="Charger voltage"
                                value={power.v_charge}
                                precision={2}
                                suffix="V"
                            />
                        </Col>
                    </Row>
                    <div style={{marginTop: 12}}>
                        <Space wrap>
                            <Typography.Text type="secondary" style={{fontSize: 12}}>Emergency</Typography.Text>
                            <Tag color={emergency.active_emergency ? "error" : emergency.latched_emergency ? "warning" : "default"}>
                                {emergency.active_emergency
                                    ? (emergency.reason ?? "ACTIVE")
                                    : emergency.latched_emergency
                                        ? "Latched"
                                        : "Clear"}
                            </Tag>
                            {(emergency.active_emergency || emergency.latched_emergency) && (
                                <Button
                                    danger
                                    size="small"
                                    icon={<AlertOutlined/>}
                                    onClick={resetEmergencyAction}
                                >
                                    Reset emergency
                                </Button>
                            )}
                        </Space>
                    </div>
                    {btNodeStates.size > 0 && (
                        <div style={{marginTop: 12}}>
                            <Typography.Text type="secondary" style={{fontSize: 12, display: "block", marginBottom: 4}}>Active BT Nodes</Typography.Text>
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
                <Card title="Coverage" size="small">
                    <Table
                        size="small"
                        dataSource={snapshot?.coverage ?? []}
                        columns={coverageColumns}
                        rowKey="area_index"
                        pagination={false}
                        locale={{emptyText: "No coverage data"}}
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
                    title="Configuration Cross-checks"
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
                            No warnings.
                        </Typography.Text>
                    )}
                    {crossChecks?.dock_pose && (
                        <Row gutter={[8, 4]}>
                            <Col span={24}>
                                <Typography.Text type="secondary" style={{fontSize: 11}}>Dock pose</Typography.Text>
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="X (m)"
                                    value={crossChecks.dock_pose.configured_x}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="Y (m)"
                                    value={crossChecks.dock_pose.configured_y}
                                    precision={3}
                                />
                            </Col>
                            <Col span={8}>
                                <Statistic
                                    title="Yaw (deg)"
                                    value={(crossChecks.dock_pose.configured_yaw * 180 / Math.PI).toFixed(1)}
                                    suffix="°"
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title="Datum lat"
                                    value={crossChecks.dock_pose.datum_lat}
                                    precision={7}
                                />
                            </Col>
                            <Col span={12}>
                                <Statistic
                                    title="Datum lon"
                                    value={crossChecks.dock_pose.datum_lon}
                                    precision={7}
                                />
                            </Col>
                            <Col span={24}>
                                <Space>
                                    <Typography.Text type="secondary" style={{fontSize: 12}}>Config present</Typography.Text>
                                    <Tag color={crossChecks.dock_pose.has_config ? "success" : "warning"}>
                                        {crossChecks.dock_pose.has_config ? "Yes" : "No"}
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
                message: "Calibration started",
                description: "Running 3 forward/back cycles plus optional dock pre-phase. This may take up to 2 minutes.",
            });
            const res = await fetch("/api/calibration/imu-yaw", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({duration_sec: 30}),
            });
            if (!res.ok) {
                throw new Error(`HTTP ${res.status}: ${await res.text()}`);
            }
            notification.success({message: "Calibration complete", description: "Refreshing status..."});
            refreshCalibration();
        } catch (e) {
            notification.error({
                message: "Calibration failed",
                description: e instanceof Error ? e.message : String(e),
            });
        }
    };

    const runMagCalibration = async () => {
        try {
            notification.info({
                message: "Magnetometer calibration started",
                description: "Robot will run a figure-8 (~30 s). Make sure there's at least 1.5 m clear in front and behind.",
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
                notification.success({message: "Magnetometer calibration complete", description: data.message || "Refreshing status..."});
            } else {
                notification.error({message: "Magnetometer calibration failed", description: data.message || "Unknown error"});
            }
            refreshCalibration();
        } catch (e) {
            notification.error({
                message: "Magnetometer calibration failed",
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
                    title={<Space><CompassOutlined/> Dock calibration</Space>}
                    size="small"
                    extra={
                        <Tag color={dockCal?.present ? "success" : "warning"}>
                            {dockCal?.present ? "Present" : "Missing"}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            onClick={runImuCalibration}
                        >
                            Run calibration
                        </Button>,
                    ]}
                >
                    {dockCal?.present && !dockCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label="Position">
                                ({dockCal.dock_pose_x?.toFixed(3)}, {dockCal.dock_pose_y?.toFixed(3)}) m
                            </Descriptions.Item>
                            <Descriptions.Item label="Yaw">
                                {dockCal.dock_pose_yaw_deg?.toFixed(2)}°
                            </Descriptions.Item>
                        </Descriptions>
                    ) : dockCal?.error ? (
                        <Alert type="error" showIcon message={dockCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            Dock pose not yet set in mowgli_robot.yaml. Run the calibration while the robot is docked, or place the dock manually from the map view.
                        </Typography.Text>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card
                    title={<Space><CompassOutlined/> IMU bias calibration</Space>}
                    size="small"
                    extra={
                        <Tag color={imuCal?.present ? "success" : "warning"}>
                            {imuCal?.present ? "Present" : "Missing"}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            onClick={runImuCalibration}
                        >
                            Run calibration
                        </Button>,
                    ]}
                >
                    {imuCal?.present && !imuCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label="Calibrated at">
                                {formatTs(imuCal.calibrated_at)}
                            </Descriptions.Item>
                            <Descriptions.Item label="Samples">
                                {imuCal.samples_used ?? "—"}
                            </Descriptions.Item>
                            <Descriptions.Item label="Gyro bias (rad/s)">
                                [{imuCal.gyro_bias_x?.toFixed(5) ?? "—"},{" "}
                                {imuCal.gyro_bias_y?.toFixed(5) ?? "—"},{" "}
                                {imuCal.gyro_bias_z?.toFixed(5) ?? "—"}]
                            </Descriptions.Item>
                            <Descriptions.Item label="Implied pitch/roll">
                                {imuCal.implied_pitch_deg?.toFixed(2)}° / {imuCal.implied_roll_deg?.toFixed(2)}°
                            </Descriptions.Item>
                        </Descriptions>
                    ) : imuCal?.error ? (
                        <Alert type="error" showIcon message={imuCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            No imu_calibration.txt yet — hardware_bridge will auto-calibrate on the next dock.
                        </Typography.Text>
                    )}
                </Card>
            </Col>
            <Col xs={24} lg={8}>
                <Card
                    title={<Space><CompassOutlined/> Magnetometer calibration</Space>}
                    size="small"
                    extra={
                        <Tag color={magCal?.present ? "success" : "default"}>
                            {magCal?.present ? "Present" : "Disabled"}
                        </Tag>
                    }
                    actions={[
                        <Button
                            key="run"
                            size="small"
                            type="link"
                            onClick={runMagCalibration}
                        >
                            Enable & run
                        </Button>,
                    ]}
                >
                    {magCal?.present && !magCal?.error ? (
                        <Descriptions size="small" column={1}>
                            <Descriptions.Item label="Calibrated at">
                                {formatTs(magCal.calibrated_at)}
                            </Descriptions.Item>
                            <Descriptions.Item label="|B| mean">
                                {magCal.magnitude_mean_uT?.toFixed(2)} µT
                            </Descriptions.Item>
                            <Descriptions.Item label="|B| std">
                                {magCal.magnitude_std_uT?.toFixed(2)} µT
                            </Descriptions.Item>
                            <Descriptions.Item label="Samples">
                                {magCal.sample_count ?? "—"}
                            </Descriptions.Item>
                        </Descriptions>
                    ) : magCal?.error ? (
                        <Alert type="error" showIcon message={magCal.error}/>
                    ) : (
                        <Typography.Text type="secondary" style={{fontSize: 12}}>
                            Magnetometer fusion is off. Enable <Typography.Text code>do_mag_calibration</Typography.Text> on calibrate_imu_yaw_node to include the rotation phase.
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
                            <Statistic title="Ang Vel X" value={imu.angular_velocity?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Ang Vel Y" value={imu.angular_velocity?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Ang Vel Z" value={imu.angular_velocity?.z} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc X" value={imu.linear_acceleration?.x} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc Y" value={imu.linear_acceleration?.y} precision={4}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Lin Acc Z" value={imu.linear_acceleration?.z} precision={4}/>
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col xs={24} lg={12}>
                <Card title="Wheel Odometry" size="small">
                    <Row gutter={[12, 8]}>
                        <Col span={12}>
                            <Statistic
                                title="Linear Vel (m/s)"
                                value={(wheelTicks as any).linear_velocity_x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Angular Vel (rad/s)"
                                value={(wheelTicks as any).angular_velocity_z}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Pose X (m)"
                                value={(wheelTicks as any).pose_x}
                                precision={3}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title="Pose Y (m)"
                                value={(wheelTicks as any).pose_y}
                                precision={3}
                            />
                        </Col>
                    </Row>
                </Card>
            </Col>
            <Col span={24}>
                <Card title={<Space><SoundOutlined/> Hardware Status</Space>} size="small">
                    <Row gutter={[12, 8]}>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="Mower Status"
                                value={status.mower_status === 255 ? "OK" : "Initializing"}
                                valueStyle={{color: status.mower_status === 255 ? undefined : colors.warning}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="Rain"
                                value={status.rain_detected ? "Detected" : "None"}
                                valueStyle={{color: status.rain_detected ? colors.warning : undefined}}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic
                                title="ESC Status"
                                value={(ESC_STATUS[status.mower_esc_status ?? 0] ?? {label: `Unknown (${status.mower_esc_status})`}).label}
                                valueStyle={{
                                    color: ESC_STATUS[status.mower_esc_status ?? 0]?.color === "error" ? colors.danger
                                        : ESC_STATUS[status.mower_esc_status ?? 0]?.color === "warning" ? colors.warning
                                        : undefined,
                                }}
                            />
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="ESC Temp" value={status.mower_esc_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="Motor Temp" value={status.mower_motor_temperature} precision={1} suffix="°C"/>
                        </Col>
                        <Col xs={12} lg={4}>
                            <Statistic title="Motor RPM" value={status.mower_motor_rpm} precision={0}/>
                        </Col>
                    </Row>
                    <Flex wrap gap="small" style={{marginTop: 12}}>
                        <Tag color={status.raspberry_pi_power ? "success" : "default"}>RPi Power</Tag>
                        <Tag color={status.esc_power ? "success" : "default"}>ESC Power</Tag>
                        <Tag color={status.ui_board_available ? "success" : "default"}>UI Board</Tag>
                        <Tag color={status.sound_module_available ? "success" : "default"}>Sound Module</Tag>
                        <Tag color={status.mow_enabled ? "success" : "default"}>Mow Enabled</Tag>
                    </Flex>
                </Card>
            </Col>
        </Row>
    );

    // ── Section 5: ROS Diagnostics ───────────────────────────────────────────

    const sectionRosDiagnostics = (
        <Card title="ROS Diagnostics" size="small">
            {(diagnostics.status ?? []).length === 0 ? (
                <Typography.Text type="secondary">No diagnostic messages received.</Typography.Text>
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
                            <Typography.Text type="secondary" style={{fontSize: 12}}>No key-value pairs.</Typography.Text>
                        ),
                    }))}
                />
            )}
        </Card>
    );

    // ── Section 6: Alerts ────────────────────────────────────────────────────

    const sectionAlerts = alerts.length > 0 ? (
        <Card title={<Space><WarningOutlined/> Alerts</Space>} size="small">
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
                {healthBar}
                {sectionAlerts}
                <Collapse
                    defaultActiveKey={[]}
                    size="small"
                    items={[
                        {
                            key: "system",
                            label: <Space><CloudServerOutlined/> System</Space>,
                            children: sectionSystem,
                        },
                        {
                            key: "localization",
                            label: <Space><CompassOutlined/> Localization</Space>,
                            children: sectionLocalization,
                        },
                        ...(sectionFusionGraph ? [{
                            key: "fusion_graph",
                            label: <Space><CompassOutlined/> Fusion Graph</Space>,
                            children: sectionFusionGraph,
                        }] : []),
                        {
                            key: "heading_sources",
                            label: <Space><CompassOutlined/> Heading sources</Space>,
                            children: sectionHeadingSources,
                        },
                        {
                            key: "bt",
                            label: <Space><ApiOutlined/> BT State & Coverage</Space>,
                            children: sectionBtCoverage,
                        },
                        {
                            key: "cross_checks",
                            label: "Configuration Cross-checks",
                            children: sectionCrossChecks,
                        },
                        {
                            key: "calibration_status",
                            label: "Calibration status",
                            children: sectionCalibrationStatus,
                        },
                        {
                            key: "sensors",
                            label: <Space><ThunderboltOutlined/> Sensors</Space>,
                            children: sectionSensors,
                        },
                        {
                            key: "ros",
                            label: "ROS Diagnostics",
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
            label: <Space><CloudServerOutlined/> System</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionSystem}
                {sectionRosDiagnostics}
            </Space>,
        },
        {
            key: "localization",
            label: <Space><CompassOutlined/> Localization</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionLocalization}
                {sectionFusionGraph}
                {sectionHeadingSources}
            </Space>,
        },
        {
            key: "robot",
            label: <Space><ApiOutlined/> Robot</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionBtCoverage}
                {sectionSensors}
            </Space>,
        },
        {
            key: "calibration",
            label: <Space><CompassOutlined/> Calibration</Space>,
            children: <Space direction="vertical" size="middle" style={{width: "100%"}}>
                {sectionCrossChecks}
                {sectionCalibrationStatus}
            </Space>,
        },
    ];

    return (
        <Space direction="vertical" size="middle" style={{width: "100%"}}>
            {healthBar}
            {sectionAlerts}
            <Tabs defaultActiveKey="system" items={tabItems} size="large"/>
        </Space>
    );
};

export default DiagnosticsPage;
