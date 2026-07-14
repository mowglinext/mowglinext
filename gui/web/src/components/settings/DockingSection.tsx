import React from "react";
import { Card, Col, Collapse, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { HomeOutlined } from "@ant-design/icons";
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

export const DockingSection: React.FC<Props> = ({ values, onChange, isOverridden, hasDefault, onReset }) => {
    const { colors } = useThemeMode();
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
                        <Text strong className="mn-display" style={{ fontSize: 14, color: colors.text }}>
                            <HomeOutlined style={{ marginRight: 6, color: colors.primary }} />
                            {t('dockingSection.dockingBehaviour')}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t('dockingSection.dockingBehaviourDescription')}
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item label={fieldLabel("undock_distance", t('dockingSection.undockDistance'))} tooltip={t('dockingSection.undockDistanceTooltip')}>
                                    <InputNumber
                                        value={values.undock_distance}
                                        onChange={(v) => onChange("undock_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label={fieldLabel("undock_speed", t('dockingSection.undockSpeed'))} tooltip={t('dockingSection.undockSpeedTooltip')}>
                                    <InputNumber
                                        value={values.undock_speed}
                                        onChange={(v) => onChange("undock_speed", v)}
                                        min={0.05} max={0.3} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m/s"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label={fieldLabel("dock_approach_distance", t('dockingSection.approachDistance'))} tooltip={t('dockingSection.approachDistanceTooltip')}>
                                    <InputNumber
                                        value={values.dock_approach_distance}
                                        onChange={(v) => onChange("dock_approach_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label={fieldLabel("dock_use_charger_detection", t('dockingSection.chargerDetection'))} tooltip={t('dockingSection.chargerDetectionTooltip')}>
                                    <Switch
                                        checked={values.dock_use_charger_detection ?? true}
                                        onChange={(v) => onChange("dock_use_charger_detection", v)}
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>

                    <Collapse
                        ghost
                        items={[
                            {
                                key: "advanced",
                                label: (
                                    <Text strong style={{ color: colors.textSecondary }}>
                                        {t('dockingSection.advanced')}
                                    </Text>
                                ),
                                children: (
                                    <Form layout="vertical" size="small">
                                        <Row gutter={[16, 0]}>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label={fieldLabel("dock_max_retries", t('dockingSection.maxRetries'))} tooltip={t('dockingSection.maxRetriesTooltip')}>
                                                    <InputNumber
                                                        value={values.dock_max_retries}
                                                        onChange={(v) => onChange("dock_max_retries", v)}
                                                        min={1} max={10} step={1} precision={0}
                                                        style={{ width: "100%" }}
                                                    />
                                                </Form.Item>
                                            </Col>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label={fieldLabel("dock_charging_threshold", t('dockingSection.chargingThreshold'))} tooltip={t('dockingSection.chargingThresholdTooltip')}>
                                                    <InputNumber
                                                        value={values.dock_charging_threshold}
                                                        onChange={(v) => onChange("dock_charging_threshold", v)}
                                                        min={0.05} max={1.0} step={0.05} precision={2}
                                                        style={{ width: "100%" }} addonAfter="A"
                                                    />
                                                </Form.Item>
                                            </Col>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label={fieldLabel("dock_approach_overshoot", t('dockingSection.approachOvershoot'))} tooltip={t('dockingSection.approachOvershootTooltip')}>
                                                    <InputNumber
                                                        value={values.dock_approach_overshoot}
                                                        onChange={(v) => onChange("dock_approach_overshoot", v)}
                                                        min={0} max={0.3} step={0.01} precision={2}
                                                        style={{ width: "100%" }} addonAfter="m"
                                                    />
                                                </Form.Item>
                                            </Col>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label={fieldLabel("dock_pose_yaw_sigma_rad", t('dockingSection.baseHeadingUncertainty'))} tooltip={t('dockingSection.baseHeadingUncertaintyTooltip')}>
                                                    <InputNumber
                                                        value={values.dock_pose_yaw_sigma_rad}
                                                        onChange={(v) => onChange("dock_pose_yaw_sigma_rad", v)}
                                                        min={0.005} max={0.5} step={0.005} precision={3}
                                                        style={{ width: "100%" }} addonAfter="rad"
                                                    />
                                                </Form.Item>
                                            </Col>
                                        </Row>
                                    </Form>
                                ),
                            },
                        ]}
                    />
                </Space>
            </Card>
        </div>
    );
};
