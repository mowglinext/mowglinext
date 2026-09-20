import React from "react";
import { Alert, Card, Col, Form, Input, InputNumber, Row, Space, Switch, Typography } from "antd";
import { WifiOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
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
 * MQTT bridge (mowgli_monitoring/mqtt_bridge_node) — bridges status/power/
 * emergency/high_level_status/gps/diagnostics to an MQTT broker and relays
 * <mqtt_topic_prefix>/command to HighLevelControl. See docs/MQTT_CONTROL.md
 * for the full topic/JSON contract (e.g. for a Home Assistant integration).
 */
export const MqttSection: React.FC<Props> = ({
    values,
    onChange,
    isOverridden,
    hasDefault,
    onReset,
}) => {
    const { t } = useTranslation();
    // Absent key renders as OFF, matching the schema/template default. The
    // backend prunes any value equal to that default, so this MUST agree with
    // it — see the mqtt_enabled description in mower_config.schema.json.
    const enabled = values.mqtt_enabled ?? false;

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
                            <WifiOutlined style={{ marginRight: 6 }} />
                            {t("settingsMqtt.bridge")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsMqtt.bridgeDescription")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={enabled}
                        onChange={(checked) => onChange("mqtt_enabled", checked)}
                        aria-label={t("settingsMqtt.bridge")}
                    />
                </div>
            </Card>

            {enabled && (
                <>
                    <Card size="small" style={{ marginBottom: 16 }}>
                        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", gap: 12 }}>
                            <div>
                                <Text strong>{t("settingsMqtt.homeAssistantDiscovery")}</Text>
                                <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                                    {t("settingsMqtt.homeAssistantDiscoveryDescription")}
                                </Paragraph>
                            </div>
                            <Switch
                                checked={values.mqtt_home_assistant_discovery_enabled ?? false}
                                onChange={(checked) => onChange("mqtt_home_assistant_discovery_enabled", checked)}
                                aria-label={t("settingsMqtt.homeAssistantDiscovery")}
                            />
                        </div>
                    </Card>

                    <Alert
                        type="info"
                        showIcon
                        style={{ marginBottom: 16 }}
                        message={t("settingsMqtt.securityTitle")}
                        description={t("settingsMqtt.securityDescription")}
                    />

                    <Card size="small" title={t("settingsMqtt.broker")} style={{ marginBottom: 16 }}>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={14}>
                                    <Form.Item
                                        label={label("mqtt_host", t("settingsMqtt.host"))}
                                        tooltip={t("settingsMqtt.hostTooltip")}
                                    >
                                        <Input
                                            value={values.mqtt_host}
                                            onChange={(e) => onChange("mqtt_host", e.target.value)}
                                            placeholder="localhost"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={4}>
                                    <Form.Item
                                        label={label("mqtt_port", t("settingsMqtt.port"))}
                                    >
                                        <InputNumber
                                            value={values.mqtt_port}
                                            onChange={(v) => onChange("mqtt_port", v)}
                                            min={1} max={65535} step={1} precision={0}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={6}>
                                    <Form.Item
                                        label={label("mqtt_use_ssl", t("settingsMqtt.useSsl"))}
                                        tooltip={t("settingsMqtt.useSslTooltip")}
                                    >
                                        <Switch
                                            checked={values.mqtt_use_ssl ?? false}
                                            onChange={(checked) => onChange("mqtt_use_ssl", checked)}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label={label("mqtt_username", t("settingsMqtt.username"))}
                                        tooltip={t("settingsMqtt.usernameTooltip")}
                                    >
                                        <Input
                                            value={values.mqtt_username}
                                            onChange={(e) => onChange("mqtt_username", e.target.value)}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label={label("mqtt_password", t("settingsMqtt.password"))}
                                    >
                                        <Input.Password
                                            value={values.mqtt_password}
                                            onChange={(e) => onChange("mqtt_password", e.target.value)}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label={label("mqtt_topic_prefix", t("settingsMqtt.topicPrefix"))}
                                        tooltip={t("settingsMqtt.topicPrefixTooltip")}
                                    >
                                        <Input
                                            value={values.mqtt_topic_prefix}
                                            onChange={(e) => onChange("mqtt_topic_prefix", e.target.value)}
                                            placeholder="mowgli"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    </Card>

                    <Card size="small" title={t("settingsMqtt.topicsTitle")}>
                        <Paragraph type="secondary" style={{ margin: "0 0 8px", fontSize: 12 }}>
                            {t("settingsMqtt.topicsDescription")}
                        </Paragraph>
                        <Space direction="vertical" size={2} style={{ width: "100%" }}>
                            <Text code style={{ fontSize: 12 }}>
                                {(values.mqtt_topic_prefix || "mowgli")}/{"{status,power,emergency,high_level_status,gps,diagnostics,available}"}
                            </Text>
                            <Text code style={{ fontSize: 12 }}>
                                {(values.mqtt_topic_prefix || "mowgli")}/command
                            </Text>
                        </Space>
                    </Card>
                </>
            )}
        </div>
    );
};
