import React, { useMemo } from "react";
import { Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { ThunderboltOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import { usePower } from "../../hooks/usePower.ts";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

/** Visual battery gauge showing threshold positions */
const BatteryGauge: React.FC<{
    fullV: number;
    emptyV: number;
    criticalV: number;
    currentV?: number;
}> = ({ fullV, emptyV, criticalV, currentV }) => {
    const { mode } = useThemeMode();
    const w = 240;
    const h = 48;
    const barX = 30;
    const barW = w - 60;
    const barY = 16;
    const barH = 18;

    // Scale voltage to position (use range from criticalV-1 to fullV+1)
    const minV = criticalV - 1;
    const maxV = fullV + 1;
    const toX = (v: number) => barX + ((v - minV) / (maxV - minV)) * barW;

    const markers = useMemo(() => {
        const items: React.ReactNode[] = [];
        // Bar background
        items.push(
            <rect key="bg" x={barX} y={barY} width={barW} height={barH} rx={4}
                fill={mode === "dark" ? "#2a2a2a" : "#eee"} stroke={mode === "dark" ? "#444" : "#ccc"} strokeWidth={1} />
        );

        // Gradient zones
        const critX = toX(criticalV);
        const emptyX = toX(emptyV);
        const fullX = toX(fullV);

        // Critical zone (red)
        items.push(
            <rect key="crit" x={barX + 1} y={barY + 1} width={Math.max(0, critX - barX - 1)} height={barH - 2} rx={3}
                fill={mode === "dark" ? "#5c1a1a" : "#ffccc7"} />
        );
        // Low zone (amber)
        items.push(
            <rect key="low" x={critX} y={barY + 1} width={Math.max(0, emptyX - critX)} height={barH - 2}
                fill={mode === "dark" ? "#5c4a1a" : "#fff3cd"} />
        );
        // Normal zone (green)
        items.push(
            <rect key="normal" x={emptyX} y={barY + 1} width={Math.max(0, fullX - emptyX)} height={barH - 2}
                fill={mode === "dark" ? "#1a4a2a" : "#d4edda"} />
        );

        // Threshold markers
        const markerStyle = { fontSize: 8, fontFamily: "monospace" as const };
        const labelY = barY + barH + 12;

        items.push(
            <g key="m-crit">
                <line x1={critX} y1={barY} x2={critX} y2={barY + barH} stroke="#f5222d" strokeWidth={2} />
                <text x={critX} y={labelY} textAnchor="middle" fill="#f5222d" {...markerStyle}>{criticalV}V</text>
            </g>
        );
        items.push(
            <g key="m-empty">
                <line x1={emptyX} y1={barY} x2={emptyX} y2={barY + barH} stroke="#fa8c16" strokeWidth={2} />
                <text x={emptyX} y={labelY} textAnchor="middle" fill="#fa8c16" {...markerStyle}>{emptyV}V</text>
            </g>
        );
        items.push(
            <g key="m-full">
                <line x1={fullX} y1={barY} x2={fullX} y2={barY + barH} stroke="#52c41a" strokeWidth={2} />
                <text x={fullX} y={labelY} textAnchor="middle" fill="#52c41a" {...markerStyle}>{fullV}V</text>
            </g>
        );

        // Current voltage indicator
        if (currentV !== undefined && currentV > 0) {
            const curX = Math.max(barX, Math.min(barX + barW, toX(currentV)));
            items.push(
                <g key="current">
                    <polygon
                        points={`${curX},${barY - 2} ${curX - 4},${barY - 7} ${curX + 4},${barY - 7}`}
                        fill={mode === "dark" ? "#fff" : "#333"}
                    />
                    <text x={curX} y={barY - 9} textAnchor="middle" fontSize={7} fontFamily="monospace"
                        fill={mode === "dark" ? "#fff" : "#333"}>
                        {currentV.toFixed(1)}V
                    </text>
                </g>
            );
        }

        return items;
    }, [fullV, emptyV, criticalV, currentV, mode]);

    return (
        <svg width={w} height={h} viewBox={`0 0 ${w} ${h}`} style={{ display: "block", margin: "0 auto" }}>
            {markers}
        </svg>
    );
};

export const BatterySection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const power = usePower();

    const fullV = values.battery_full_voltage ?? 28.5;
    const emptyV = values.battery_empty_voltage ?? 24.0;
    const criticalV = values.battery_critical_voltage ?? 23.0;

    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <ThunderboltOutlined style={{ marginRight: 6 }} />
                            {t("settingsBattery.batteryThresholds")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsBattery.batteryThresholdsDescription")}
                        </Paragraph>
                    </div>

                    {/* Visual gauge */}
                    <div style={{
                        background: colors.bgSubtle,
                        borderRadius: 8,
                        padding: "12px 8px 4px",
                        border: `1px solid ${colors.borderSubtle}`,
                    }}>
                        <BatteryGauge
                            fullV={fullV}
                            emptyV={emptyV}
                            criticalV={criticalV}
                            currentV={power?.v_battery}
                        />
                    </div>

                    {/* Voltage thresholds */}
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={8}>
                                <Form.Item
                                    label={<Text style={{ color: "#52c41a", fontSize: 12 }}>{t("settingsBattery.fullVoltage")}</Text>}
                                    tooltip={t("settingsBattery.fullVoltageTooltip")}
                                >
                                    <InputNumber
                                        value={values.battery_full_voltage}
                                        onChange={(v) => onChange("battery_full_voltage", v)}
                                        min={20} max={60} step={0.5} precision={1}
                                        style={{ width: "100%" }} addonAfter="V"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={8}>
                                <Form.Item
                                    label={<Text style={{ color: "#fa8c16", fontSize: 12 }}>{t("settingsBattery.emptyVoltage")}</Text>}
                                    tooltip={t("settingsBattery.emptyVoltageTooltip")}
                                >
                                    <InputNumber
                                        value={values.battery_empty_voltage}
                                        onChange={(v) => onChange("battery_empty_voltage", v)}
                                        min={18} max={55} step={0.5} precision={1}
                                        style={{ width: "100%" }} addonAfter="V"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={8}>
                                <Form.Item
                                    label={<Text style={{ color: "#f5222d", fontSize: 12 }}>{t("settingsBattery.criticalVoltage")}</Text>}
                                    tooltip={t("settingsBattery.criticalVoltageTooltip")}
                                >
                                    <InputNumber
                                        value={values.battery_critical_voltage}
                                        onChange={(v) => onChange("battery_critical_voltage", v)}
                                        min={15} max={50} step={0.5} precision={1}
                                        style={{ width: "100%" }} addonAfter="V"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            {/* Percentage thresholds */}
            <Card size="small" title={t("settingsBattery.percentageThresholds")} style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={8}>
                            <Form.Item
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>{t("settingsBattery.resumeAbove")}</Text>}
                                tooltip={t("settingsBattery.resumeAboveTooltip")}
                            >
                                <InputNumber
                                    value={values.battery_full_percent}
                                    onChange={(v) => onChange("battery_full_percent", v)}
                                    min={50} max={100} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="%"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={8}>
                            <Form.Item
                                label={<Text style={{ color: "#fa8c16", fontSize: 12 }}>{t("settingsBattery.lowDock")}</Text>}
                                tooltip={t("settingsBattery.lowDockTooltip")}
                            >
                                <InputNumber
                                    value={values.battery_low_percent}
                                    onChange={(v) => onChange("battery_low_percent", v)}
                                    min={5} max={50} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="%"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={8}>
                            <Form.Item
                                label={<Text style={{ color: "#f5222d", fontSize: 12 }}>{t("settingsBattery.critical")}</Text>}
                                tooltip={t("settingsBattery.criticalTooltip")}
                            >
                                <InputNumber
                                    value={values.battery_critical_percent}
                                    onChange={(v) => onChange("battery_critical_percent", v)}
                                    min={1} max={30} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="%"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                    <Row gutter={[16, 0]}>
                        <Col xs={8}>
                            <Form.Item
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>{t("settingsBattery.criticalRecovery")}</Text>}
                                tooltip={t("settingsBattery.criticalRecoveryTooltip")}
                            >
                                <InputNumber
                                    value={values.battery_critical_recovery_percent}
                                    onChange={(v) => onChange("battery_critical_recovery_percent", v)}
                                    min={5} max={90} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="%"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
