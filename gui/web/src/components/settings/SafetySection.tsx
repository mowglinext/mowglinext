import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Typography } from "antd";
import { WarningOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SafetySection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    return (
        <div>
            <Alert
                type="warning"
                showIcon
                icon={<WarningOutlined />}
                message={t("settingsSafety.alertMessage")}
                description={t("settingsSafety.alertDescription")}
                style={{ marginBottom: 16 }}
            />

            {/* Lift / tilt detection is handled by the STM32 firmware,
                not by ROS2. The previous emergency_stop_on_lift /
                emergency_stop_on_tilt switches were UI-only — no node
                in ROS2 ever read them — so they were removed (audit
                2026-05-12). The firmware always emergency-stops on
                lift/tilt when its physical thresholds are tripped;
                this is not configurable from the GUI. */}

            {/* Temperature */}
            <Card size="small" title={t("settingsSafety.motorTemperatureLimits")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsSafety.motorTemperatureLimitsDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#f5222d", fontSize: 12 }}>{t("settingsSafety.stopAbove")}</Text>}
                                tooltip={t("settingsSafety.stopAboveTooltip")}
                            >
                                <InputNumber
                                    value={values.motor_temp_high_c}
                                    onChange={(v) => onChange("motor_temp_high_c", v)}
                                    min={40} max={120} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>{t("settingsSafety.resumeBelow")}</Text>}
                                tooltip={t("settingsSafety.resumeBelowTooltip")}
                            >
                                <InputNumber
                                    value={values.motor_temp_low_c}
                                    onChange={(v) => onChange("motor_temp_low_c", v)}
                                    min={20} max={80} step={5} precision={0}
                                    style={{ width: "100%" }} addonAfter="C"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* max_obstacle_avoidance_distance moved to the Obstacles
                section (ObstaclesSection.tsx) alongside the other
                obstacle-avoidance knobs. */}
        </div>
    );
};
