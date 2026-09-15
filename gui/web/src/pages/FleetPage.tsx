import {useMemo, useState} from "react";
import {Alert, App, Button, Card, Col, Input, Popconfirm, Progress, Row, Space, Tag, Tooltip, Typography} from "antd";
import {DeleteOutlined, ExportOutlined, HomeOutlined, PauseCircleOutlined, PlayCircleOutlined, PlusOutlined} from "@ant-design/icons";
import {useTranslation} from "react-i18next";
import {useFleet} from "../hooks/useFleet.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {canCommand, deriveFleetRow, projectFleetPositions} from "../utils/fleet.ts";
import type {FleetPhase, FleetRow} from "../utils/fleet.ts";
import {FleetMapMini} from "../components/fleet/FleetMapMini.tsx";
import {StatusOrb} from "../concept/components/StatusOrb.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";

const {Text, Title} = Typography;

const PHASE_TONE: Record<FleetPhase, "live" | "resting" | "alert" | "charging"> = {
    offline: "resting",
    idle: "resting",
    charging: "charging",
    mowing: "live",
    returning: "live",
    recording: "live",
    manual: "live",
    emergency: "alert",
};

const PHASE_TAG: Record<FleetPhase, string> = {
    offline: "default",
    idle: "default",
    charging: "cyan",
    mowing: "green",
    returning: "gold",
    recording: "purple",
    manual: "gold",
    emergency: "red",
};

/**
 * Fleet page: every mower this robot knows about (itself first), with live
 * state, a shared position sketch and per-robot Play / Pause / Home. Data
 * comes from this robot's backend, which mirrors its peers — the browser
 * never talks to another robot directly (docs/MULTI_ROBOT.md).
 */
export default function FleetPage() {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const {colors} = useThemeMode();
    const {settings} = useSettings();
    const fleet = useFleet();
    const [address, setAddress] = useState("");
    const [busy, setBusy] = useState<string | null>(null);

    const rows = useMemo(() => fleet.robots.map(r => deriveFleetRow(r, settings ?? {})), [fleet.robots, settings]);
    const dots = useMemo(() => {
        const byId = new Map(rows.map(r => [r.id, r]));
        return projectFleetPositions(rows).map(p => {
            const r = byId.get(p.id)!;
            return {id: p.id, name: r.name, x: p.x, y: p.y, phase: r.phase, self: r.self};
        });
    }, [rows]);

    const run = async (key: string, fn: () => Promise<unknown>, okText?: string) => {
        setBusy(key);
        try {
            const warning = await fn();
            if (typeof warning === "string" && warning) {
                message.warning(warning, 8);
            } else if (okText) {
                message.success(okText);
            }
        } catch (e: any) {
            message.error(e?.message ?? String(e));
        } finally {
            setBusy(null);
        }
    };

    const command = (row: FleetRow, cmd: number, label: string) =>
        run(`${row.id}:${cmd}`, () => fleet.call(row.id, "high_level_control", {Command: cmd}), t("fleetPage.sent", {robot: row.name, action: label}));

    const addPeer = () => {
        const value = address.trim();
        if (!value) return;
        void run("add", async () => {
            const warning = await fleet.addPeer(value);
            setAddress("");
            return warning;
        }, t("fleetPage.peerAdded"));
    };

    return (
        <div style={{display: "flex", flexDirection: "column", gap: 16}}>
            {fleet.error && (
                <Alert type="warning" showIcon message={t("fleetPage.pollError")} description={fleet.error}/>
            )}

            <Row gutter={[16, 16]}>
                {rows.map(row => (
                    <Col xs={24} md={12} xl={8} key={row.id}>
                        <Card
                            size="small"
                            data-testid={`fleet-card-${row.id}`}
                            title={
                                <Space size={8}>
                                    <StatusOrb tone={PHASE_TONE[row.phase]}/>
                                    <span>{row.name}</span>
                                    {row.self && <Tag color="blue">{t("fleetPage.thisRobot")}</Tag>}
                                </Space>
                            }
                            extra={
                                <Space size={4}>
                                    {row.address && (
                                        <Tooltip title={t("fleetPage.openGui")}>
                                            <Button
                                                size="small" type="text" icon={<ExportOutlined/>}
                                                href={`http://${row.address}/`} target="_blank" rel="noreferrer"
                                            />
                                        </Tooltip>
                                    )}
                                    {!row.self && (
                                        <Popconfirm
                                            title={t("fleetPage.removeConfirm", {robot: row.name})}
                                            onConfirm={() => run(`rm:${row.id}`, () => fleet.removePeer(row.id), t("fleetPage.peerRemoved"))}
                                        >
                                            <Button size="small" type="text" danger icon={<DeleteOutlined/>} loading={busy === `rm:${row.id}`}/>
                                        </Popconfirm>
                                    )}
                                </Space>
                            }
                        >
                            <Space direction="vertical" size={8} style={{width: "100%"}}>
                                <Space wrap size={[6, 6]}>
                                    <Tag color={PHASE_TAG[row.phase]}>{t(`fleetPage.phase.${row.phase}`)}</Tag>
                                    {row.stateName && row.online && <Text type="secondary" style={{fontSize: 12}}>{row.stateName}</Text>}
                                    {row.apiVersionMismatch && <Tag color="orange">{t("fleetPage.apiMismatch")}</Tag>}
                                </Space>
                                <Row gutter={12}>
                                    <Col span={12}>
                                        <Text type="secondary" style={{fontSize: 11}}>{t("fleetPage.battery")}</Text>
                                        <Progress percent={row.batteryPercent} size="small" status={row.batteryPercent <= 20 ? "exception" : "normal"}/>
                                    </Col>
                                    <Col span={12}>
                                        <Text type="secondary" style={{fontSize: 11}}>{t("fleetPage.gps")}</Text>
                                        <div><Text>{row.online ? row.gpsLabel : "—"}</Text></div>
                                    </Col>
                                </Row>
                                {(row.phase === "mowing") && (
                                    <Text type="secondary" style={{fontSize: 12}}>
                                        {t("fleetPage.mowingArea", {area: row.currentArea, pct: Math.round(row.coveragePercent)})}
                                    </Text>
                                )}
                                {!row.online && row.lastSeen && (
                                    <Text type="secondary" style={{fontSize: 12}}>
                                        {t("fleetPage.lastSeen", {when: row.lastSeen.toLocaleTimeString()})}
                                    </Text>
                                )}
                                <Space wrap>
                                    <Button
                                        icon={<PlayCircleOutlined/>} size="small" type="primary"
                                        disabled={!canCommand(row) || row.phase === "mowing"}
                                        loading={busy === `${row.id}:1`}
                                        onClick={() => command(row, 1, t("fleetPage.play"))}
                                    >{t("fleetPage.play")}</Button>
                                    <Button
                                        icon={<PauseCircleOutlined/>} size="small"
                                        disabled={!canCommand(row) || !(row.phase === "mowing" || row.phase === "returning" || row.phase === "manual")}
                                        loading={busy === `${row.id}:8`}
                                        onClick={() => command(row, 8, t("fleetPage.pause"))}
                                    >{t("fleetPage.pause")}</Button>
                                    <Popconfirm
                                        title={t("fleetPage.homeConfirm", {robot: row.name})}
                                        onConfirm={() => command(row, 2, t("fleetPage.home"))}
                                        disabled={!canCommand(row)}
                                    >
                                        <Button
                                            icon={<HomeOutlined/>} size="small"
                                            disabled={!canCommand(row) || row.phase === "charging" || row.phase === "returning"}
                                            loading={busy === `${row.id}:2`}
                                        >{t("fleetPage.home")}</Button>
                                    </Popconfirm>
                                </Space>
                            </Space>
                        </Card>
                    </Col>
                ))}
            </Row>

            <Card size="small" title={t("fleetPage.positions")}>
                <FleetMapMini dots={dots} emptyLabel={t("fleetPage.noPositions")}/>
            </Card>

            <Card size="small" title={t("fleetPage.addPeerTitle")}>
                <Space direction="vertical" size={8} style={{width: "100%"}}>
                    <Text type="secondary">{t("fleetPage.addPeerHelp")}</Text>
                    <Space.Compact style={{width: "100%", maxWidth: 480}}>
                        <Input
                            value={address}
                            onChange={e => setAddress(e.target.value)}
                            onPressEnter={addPeer}
                            placeholder="10.0.0.42 · mower-b.local:4006"
                            aria-label={t("fleetPage.addPeerTitle")}
                        />
                        <Button type="primary" icon={<PlusOutlined/>} loading={busy === "add"} onClick={addPeer}>
                            {t("fleetPage.add")}
                        </Button>
                    </Space.Compact>
                    <Title level={5} style={{margin: "8px 0 0", color: colors.textSecondary}}>{t("fleetPage.requirements")}</Title>
                    <ul style={{margin: 0, paddingLeft: 18, color: colors.textSecondary, fontSize: 12}}>
                        <li>{t("fleetPage.reqName")}</li>
                        <li>{t("fleetPage.reqNetwork")}</li>
                        <li>{t("fleetPage.reqSymmetric")}</li>
                    </ul>
                </Space>
            </Card>
        </div>
    );
}
