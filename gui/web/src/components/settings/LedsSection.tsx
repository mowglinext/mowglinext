import React from "react";
import { Alert, Card, Col, Form, Input, InputNumber, Row, Space, Switch, Typography } from "antd";
import { BulbOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
};

/**
 * Ring patterns, in the same priority order the firmware-side renderer uses.
 * The swatches mirror the palette in
 * ros2/src/mowgli_leds/include/mowgli_leds/led_pattern.hpp — keep them in sync
 * so the legend is not quietly lying about what the operator will see.
 */
const LED_MODES: { key: string; color: string }[] = [
    { key: "modeEmergency", color: "#ff0000" },
    { key: "modeCharging", color: "#00ff00" },
    { key: "modeStale", color: "#ff6e00" },
    { key: "modeLowBattery", color: "#ff0000" },
    { key: "modeMowing", color: "#00ff00" },
    { key: "modeMowingDegraded", color: "#ff6e00" },
    { key: "modeRecording", color: "#00c8ff" },
    { key: "modeManual", color: "#aa00ff" },
    { key: "modeIdle", color: "#ffffff" },
];

export const LedsSection: React.FC<Props> = ({
    values,
    onChange,
    isOverridden,
    hasDefault,
    onReset,
}) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    // Absent key renders as OFF, matching the schema/template default. The
    // backend prunes any value equal to that default, so this MUST agree with
    // it — see the led_enabled description in mower_config.schema.json.
    const enabled = values.led_enabled ?? false;

    const label = (key: string, text: string) => (
        <SettingFieldLabel
            settingKey={key}
            label={text}
            overridden={isOverridden?.(key)}
            canReset={hasDefault?.(key)}
            onReset={onReset}
        />
    );

    return (
        <div>
            {/* Master toggle */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <BulbOutlined style={{ marginRight: 6 }} />
                            {t("settingsLeds.statusRing")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsLeds.statusRingDescription")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={enabled}
                        onChange={(checked) => onChange("led_enabled", checked)}
                        aria-label={t("settingsLeds.statusRing")}
                    />
                </div>
            </Card>

            {enabled && (
                <>
                    {/* The two things that will actually stop the ring working. */}
                    <Alert
                        type="info"
                        showIcon
                        style={{ marginBottom: 16 }}
                        message={t("settingsLeds.prerequisitesTitle")}
                        description={
                            <Space direction="vertical" size={4} style={{ fontSize: 12 }}>
                                <span>
                                    {t("settingsLeds.prerequisiteOverlay")}{" "}
                                    <Text code>rk3588-spi4-m0-cs1-spidev.dtbo</Text>
                                </span>
                                <span>
                                    {t("settingsLeds.prerequisiteDevice")}{" "}
                                    <Text code>ls /dev/spidev*</Text>
                                </span>
                                <span>{t("settingsLeds.prerequisiteLevels")}</span>
                            </Space>
                        }
                    />

                    {/* Hardware */}
                    <Card size="small" title={t("settingsLeds.hardware")} style={{ marginBottom: 16 }}>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={8}>
                                    <Form.Item
                                        label={label("led_count", t("settingsLeds.ledCount"))}
                                        tooltip={t("settingsLeds.ledCountTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_count}
                                            onChange={(v) => onChange("led_count", v)}
                                            min={0} max={512} step={1} precision={0}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={10}>
                                    <Form.Item
                                        label={label("led_spi_device", t("settingsLeds.spiDevice"))}
                                        tooltip={t("settingsLeds.spiDeviceTooltip")}
                                    >
                                        <Input
                                            value={values.led_spi_device}
                                            onChange={(e) => onChange("led_spi_device", e.target.value)}
                                            placeholder="/dev/spidev4.1"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={6}>
                                    <Form.Item
                                        label={label("led_spi_speed_hz", t("settingsLeds.spiClock"))}
                                        tooltip={t("settingsLeds.spiClockTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_spi_speed_hz}
                                            onChange={(v) => onChange("led_spi_speed_hz", v)}
                                            min={1} step={100000} precision={0}
                                            style={{ width: "100%" }} addonAfter="Hz"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </Card>

                    {/* Appearance */}
                    <Card size="small" title={t("settingsLeds.appearance")} style={{ marginBottom: 16 }}>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={8}>
                                    <Form.Item
                                        label={label("led_brightness", t("settingsLeds.brightness"))}
                                        tooltip={t("settingsLeds.brightnessTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_brightness}
                                            onChange={(v) => onChange("led_brightness", v)}
                                            min={0} max={1} step={0.05}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={8}>
                                    <Form.Item
                                        label={label("led_idle_scale", t("settingsLeds.idleBrightness"))}
                                        tooltip={t("settingsLeds.idleBrightnessTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_idle_scale}
                                            onChange={(v) => onChange("led_idle_scale", v)}
                                            min={0} max={1} step={0.05}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={8}>
                                    <Form.Item
                                        label={label("led_refresh_hz", t("settingsLeds.refreshRate"))}
                                        tooltip={t("settingsLeds.refreshRateTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_refresh_hz}
                                            onChange={(v) => onChange("led_refresh_hz", v)}
                                            min={1} max={60} step={1}
                                            style={{ width: "100%" }} addonAfter="Hz"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </Card>

                    {/* Thresholds and timing */}
                    <Card size="small" title={t("settingsLeds.behavior")} style={{ marginBottom: 16 }}>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={12} sm={8}>
                                    <Form.Item
                                        label={label("led_low_battery_percent", t("settingsLeds.lowBattery"))}
                                        tooltip={t("settingsLeds.lowBatteryTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_low_battery_percent}
                                            onChange={(v) => onChange("led_low_battery_percent", v)}
                                            min={0} max={100} step={1}
                                            style={{ width: "100%" }} addonAfter="%"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={8}>
                                    <Form.Item
                                        label={label("led_charge_full_percent", t("settingsLeds.chargeFull"))}
                                        tooltip={t("settingsLeds.chargeFullTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_charge_full_percent}
                                            onChange={(v) => onChange("led_charge_full_percent", v)}
                                            min={0} max={100} step={1}
                                            style={{ width: "100%" }} addonAfter="%"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={8}>
                                    <Form.Item
                                        label={label("led_status_timeout_s", t("settingsLeds.statusTimeout"))}
                                        tooltip={t("settingsLeds.statusTimeoutTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_status_timeout_s}
                                            onChange={(v) => onChange("led_status_timeout_s", v)}
                                            min={0.5} step={0.5}
                                            style={{ width: "100%" }} addonAfter="s"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={8}>
                                    <Form.Item
                                        label={label("led_keepalive_s", t("settingsLeds.keepalive"))}
                                        tooltip={t("settingsLeds.keepaliveTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_keepalive_s}
                                            onChange={(v) => onChange("led_keepalive_s", v)}
                                            min={0.2} step={0.5}
                                            style={{ width: "100%" }} addonAfter="s"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={8}>
                                    <Form.Item
                                        label={label("led_device_retry_s", t("settingsLeds.deviceRetry"))}
                                        tooltip={t("settingsLeds.deviceRetryTooltip")}
                                    >
                                        <InputNumber
                                            value={values.led_device_retry_s}
                                            onChange={(v) => onChange("led_device_retry_s", v)}
                                            min={1} step={5}
                                            style={{ width: "100%" }} addonAfter="s"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </Card>

                    {/* What the ring is telling you */}
                    <Card size="small" title={t("settingsLeds.legend")}>
                        <Paragraph type="secondary" style={{ margin: "0 0 12px", fontSize: 12 }}>
                            {t("settingsLeds.legendDescription")}
                        </Paragraph>
                        <Space direction="vertical" size={8} style={{ width: "100%" }}>
                            {LED_MODES.map((mode) => (
                                <div
                                    key={mode.key}
                                    style={{ display: "flex", alignItems: "flex-start", gap: 10 }}
                                >
                                    <span
                                        aria-hidden="true"
                                        style={{
                                            width: 14,
                                            height: 14,
                                            borderRadius: "50%",
                                            flexShrink: 0,
                                            marginTop: 2,
                                            background: mode.color,
                                            border: `1px solid ${colors.border}`,
                                            opacity: mode.key === "modeIdle" ? 0.35 : 1,
                                        }}
                                    />
                                    <div>
                                        <Text strong style={{ fontSize: 12 }}>
                                            {t(`settingsLeds.${mode.key}`)}
                                        </Text>
                                        <br />
                                        <Text type="secondary" style={{ fontSize: 11 }}>
                                            {t(`settingsLeds.${mode.key}Description`)}
                                        </Text>
                                    </div>
                                </div>
                            ))}
                        </Space>
                    </Card>
                </>
            )}
        </div>
    );
};
