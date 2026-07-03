import React from "react";
import { Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { CompassOutlined } from "@ant-design/icons";
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

export const NavigationSection: React.FC<Props> = ({
    values,
    onChange,
    isOverridden,
    hasDefault,
    onReset,
}) => {
    const { t } = useTranslation();
    const fieldLabel = (key: string, label: React.ReactNode) => (
        <SettingFieldLabel
            settingKey={key}
            label={label}
            overridden={isOverridden?.(key) ?? false}
            canReset={hasDefault?.(key) ?? false}
            onReset={onReset}
        />
    );
    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <CompassOutlined style={{ marginRight: 6 }} />
                            {t("settingsNavigation.goalTolerances")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsNavigation.goalTolerancesDescription")}
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label={fieldLabel("xy_goal_tolerance", t("settingsNavigation.transitXyTolerance"))} tooltip={t("settingsNavigation.transitXyToleranceTooltip")}>
                                    <InputNumber
                                        value={values.xy_goal_tolerance}
                                        onChange={(v) => onChange("xy_goal_tolerance", v)}
                                        min={0.1} max={2.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={fieldLabel("yaw_goal_tolerance", t("settingsNavigation.yawTolerance"))} tooltip={t("settingsNavigation.yawToleranceTooltip")}>
                                    <InputNumber
                                        value={values.yaw_goal_tolerance}
                                        onChange={(v) => onChange("yaw_goal_tolerance", v)}
                                        min={0.1} max={3.14} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="rad"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={fieldLabel("coverage_xy_tolerance", t("settingsNavigation.coverageXyTolerance"))} tooltip={t("settingsNavigation.coverageXyToleranceTooltip")}>
                                    <InputNumber
                                        value={values.coverage_xy_tolerance}
                                        onChange={(v) => onChange("coverage_xy_tolerance", v)}
                                        min={0.05} max={1.0} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title={t("settingsNavigation.recovery")} style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label={fieldLabel("progress_timeout_sec", t("settingsNavigation.progressTimeout"))} tooltip={t("settingsNavigation.progressTimeoutTooltip")}>
                                <InputNumber
                                    value={values.progress_timeout_sec}
                                    onChange={(v) => onChange("progress_timeout_sec", v)}
                                    min={10} max={300} step={10} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
