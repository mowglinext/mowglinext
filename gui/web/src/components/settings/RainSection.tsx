import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { CloudOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

const RAIN_MODES = [
    { value: 0, labelKey: "rainModeIgnoreLabel", descriptionKey: "rainModeIgnoreDescription", color: "#8c8c8c" },
    { value: 1, labelKey: "rainModeDockLabel", descriptionKey: "rainModeDockDescription", color: "#1890ff" },
    { value: 2, labelKey: "rainModeDockUntilDryLabel", descriptionKey: "rainModeDockUntilDryDescription", color: "#13c2c2" },
    { value: 3, labelKey: "rainModePauseAutoLabel", descriptionKey: "rainModePauseAutoDescription", color: "#722ed1" },
];

export const RainSection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const currentMode = values.rain_mode ?? 2;

    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <CloudOutlined style={{ marginRight: 6 }} />
                            {t("settingsRain.rainBehavior")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsRain.rainBehaviorDescription")}
                        </Paragraph>
                    </div>

                    <Row gutter={[8, 8]}>
                        {RAIN_MODES.map((mode) => {
                            const isSelected = currentMode === mode.value;
                            return (
                                <Col xs={12} sm={6} key={mode.value}>
                                    <Card
                                        hoverable
                                        size="small"
                                        onClick={() => onChange("rain_mode", mode.value)}
                                        style={{
                                            border: isSelected
                                                ? `2px solid ${mode.color}`
                                                : `1px solid ${colors.border}`,
                                            background: isSelected ? `${mode.color}10` : undefined,
                                            height: "100%",
                                            cursor: "pointer",
                                        }}
                                        styles={{ body: { padding: "8px 10px" } }}
                                    >
                                        <Text strong style={{ fontSize: 12, color: isSelected ? mode.color : undefined }}>
                                            {t(`settingsRain.${mode.labelKey}`)}
                                        </Text>
                                        <br />
                                        <Text type="secondary" style={{ fontSize: 11 }}>
                                            {t(`settingsRain.${mode.descriptionKey}`)}
                                        </Text>
                                    </Card>
                                </Col>
                            );
                        })}
                    </Row>
                </Space>
            </Card>

            {currentMode > 0 && (
                <Card size="small" title={t("settingsRain.timing")} style={{ marginBottom: 16 }}>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12}>
                                <Form.Item label={t("settingsRain.resumeDelay")} tooltip={t("settingsRain.resumeDelayTooltip")}>
                                    <InputNumber
                                        value={values.rain_delay_minutes}
                                        onChange={(v) => onChange("rain_delay_minutes", v)}
                                        min={0} max={240} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="min"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12}>
                                <Form.Item label={t("settingsRain.debounce")} tooltip={t("settingsRain.debounceTooltip")}>
                                    <InputNumber
                                        value={values.rain_debounce_sec}
                                        onChange={(v) => onChange("rain_debounce_sec", v)}
                                        min={1} max={60} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="s"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Card>
            )}
        </div>
    );
};
