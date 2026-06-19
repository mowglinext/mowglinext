import React, { useState } from "react";
import { Card, Col, Form, InputNumber, Row, Space, Tag, Typography } from "antd";
import { ToolOutlined, DownOutlined, UpOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";
import { MOWER_MODELS } from "../../constants/mowerModels.ts";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    onBulkChange: (changes: Record<string, any>) => void;
};

export const HardwareSection: React.FC<Props> = ({ values, onChange, onBulkChange }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const [showAdvanced, setShowAdvanced] = useState(false);
    const selectedModel = values.mower_model || "YardForce500";

    const handleModelSelect = (model: string) => {
        onChange("mower_model", model);
        const preset = MOWER_MODELS.find((m) => m.value === model);
        if (preset?.defaults && Object.keys(preset.defaults).length > 0) {
            onBulkChange(preset.defaults);
        }
    };

    return (
        <div>
            {/* Model selection */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <ToolOutlined style={{ marginRight: 6 }} />
                            {t("settingsHardware.robotModel")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsHardware.robotModelDescription")}
                        </Paragraph>
                    </div>
                    <Row gutter={[8, 8]}>
                        {MOWER_MODELS.map((model) => {
                            const isSelected = selectedModel === model.value;
                            return (
                                <Col xs={12} sm={8} lg={6} key={model.value}>
                                    <Card
                                        hoverable
                                        size="small"
                                        onClick={() => handleModelSelect(model.value)}
                                        style={{
                                            border: isSelected
                                                ? `2px solid ${colors.primary}`
                                                : `1px solid ${colors.border}`,
                                            background: isSelected ? colors.primaryBg : undefined,
                                            height: "100%",
                                            cursor: "pointer",
                                        }}
                                        styles={{ body: { padding: "8px 12px" } }}
                                    >
                                        <Space direction="vertical" size={2} style={{ width: "100%" }}>
                                            <Space size={4}>
                                                <Text strong style={{ fontSize: 12 }}>{t(model.label)}</Text>
                                                {model.tag && <Tag color="green" style={{ fontSize: 10 }}>{t(model.tag)}</Tag>}
                                            </Space>
                                            <Text type="secondary" style={{ fontSize: 11 }}>
                                                {t(model.description)}
                                            </Text>
                                        </Space>
                                    </Card>
                                </Col>
                            );
                        })}
                    </Row>
                </Space>
            </Card>

            {/* Essential parameters (always visible) */}
            <Card
                size="small"
                title={t("settingsHardware.wheelsAndBlade")}
                style={{ marginBottom: 16 }}
            >
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.wheelRadius")} tooltip={t("settingsHardware.wheelRadiusTooltip")}>
                                <InputNumber
                                    value={values.wheel_radius}
                                    onChange={(v) => onChange("wheel_radius", v)}
                                    step={0.001} precision={5} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.wheelTrack")} tooltip={t("settingsHardware.wheelTrackTooltip")}>
                                <InputNumber
                                    value={values.wheel_track}
                                    onChange={(v) => onChange("wheel_track", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.bladeRadius")} tooltip={t("settingsHardware.bladeRadiusTooltip")}>
                                <InputNumber
                                    value={values.blade_radius}
                                    onChange={(v) => onChange("blade_radius", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.toolWidth")} tooltip={t("settingsHardware.toolWidthTooltip")}>
                                <InputNumber
                                    value={values.tool_width}
                                    onChange={(v) => onChange("tool_width", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.encoderTicksPerMeter")} tooltip={t("settingsHardware.encoderTicksPerMeterTooltip")}>
                                <InputNumber
                                    value={values.ticks_per_meter}
                                    onChange={(v) => onChange("ticks_per_meter", v)}
                                    step={1} precision={0} style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* Advanced: chassis dimensions */}
            <Card
                size="small"
                title={
                    <Space
                        style={{ cursor: "pointer", userSelect: "none" }}
                        onClick={() => setShowAdvanced(!showAdvanced)}
                    >
                        <span>{t("settingsHardware.chassisAndGeometry")}</span>
                        <Tag color="default" style={{ fontSize: 10 }}>{t("settingsHardware.advanced")}</Tag>
                        {showAdvanced ? <UpOutlined style={{ fontSize: 10 }} /> : <DownOutlined style={{ fontSize: 10 }} />}
                    </Space>
                }
                style={{ marginBottom: 16 }}
                styles={{ body: { display: showAdvanced ? undefined : "none" } }}
            >
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.chassisLength")}>
                                <InputNumber
                                    value={values.chassis_length}
                                    onChange={(v) => onChange("chassis_length", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.chassisWidth")}>
                                <InputNumber
                                    value={values.chassis_width}
                                    onChange={(v) => onChange("chassis_width", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.chassisHeight")}>
                                <InputNumber
                                    value={values.chassis_height}
                                    onChange={(v) => onChange("chassis_height", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.chassisCenterX")} tooltip={t("settingsHardware.chassisCenterXTooltip")}>
                                <InputNumber
                                    value={values.chassis_center_x}
                                    onChange={(v) => onChange("chassis_center_x", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.mass")}>
                                <InputNumber
                                    value={values.chassis_mass_kg}
                                    onChange={(v) => onChange("chassis_mass_kg", v)}
                                    step={0.5} precision={2} style={{ width: "100%" }}
                                    addonAfter="kg"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.wheelWidth")}>
                                <InputNumber
                                    value={values.wheel_width}
                                    onChange={(v) => onChange("wheel_width", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.wheelXOffset")} tooltip={t("settingsHardware.wheelXOffsetTooltip")}>
                                <InputNumber
                                    value={values.wheel_x_offset}
                                    onChange={(v) => onChange("wheel_x_offset", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.casterRadius")}>
                                <InputNumber
                                    value={values.caster_radius}
                                    onChange={(v) => onChange("caster_radius", v)}
                                    step={0.005} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8} lg={6}>
                            <Form.Item label={t("settingsHardware.casterTrack")}>
                                <InputNumber
                                    value={values.caster_track}
                                    onChange={(v) => onChange("caster_track", v)}
                                    step={0.01} precision={3} style={{ width: "100%" }}
                                    addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
