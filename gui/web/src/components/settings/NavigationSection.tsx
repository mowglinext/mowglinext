import {NumberWithAddon} from '../inputs/NumberWithAddon';
import React from "react";
import { Card, Col, Form, Row, Space, Typography } from "antd";
import { CompassOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";
import type {SettingsValues} from "../../types/settings.generated";

const { Text, Paragraph } = Typography;

type Props = {
    values: SettingsValues;
    onChange: (key: string, value: number | null) => void;
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
                                <Form.Item htmlFor="setting-xy_goal_tolerance" data-setting-key="xy_goal_tolerance" label={fieldLabel("xy_goal_tolerance", t("settingsNavigation.transitXyTolerance"))} tooltip={t("settingsNavigation.transitXyToleranceTooltip")}>
                                    <NumberWithAddon aria-label={t("settingsNavigation.transitXyTolerance") + ", m"} aria-description={t("settingsNavigation.transitXyToleranceTooltip")}  id="setting-xy_goal_tolerance"
                                        value={values.xy_goal_tolerance}
                                        onChange={(v) => onChange("xy_goal_tolerance", v)}
                                        min={0.1} max={2.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} after="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item htmlFor="setting-yaw_goal_tolerance" data-setting-key="yaw_goal_tolerance" label={fieldLabel("yaw_goal_tolerance", t("settingsNavigation.yawTolerance"))} tooltip={t("settingsNavigation.yawToleranceTooltip")}>
                                    <NumberWithAddon aria-label={t("settingsNavigation.yawTolerance") + ", rad"} aria-description={t("settingsNavigation.yawToleranceTooltip")}  id="setting-yaw_goal_tolerance"
                                        value={values.yaw_goal_tolerance}
                                        onChange={(v) => onChange("yaw_goal_tolerance", v)}
                                        min={0.1} max={3.14} step={0.1} precision={2}
                                        style={{ width: "100%" }} after="rad"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item htmlFor="setting-coverage_xy_tolerance" data-setting-key="coverage_xy_tolerance" label={fieldLabel("coverage_xy_tolerance", t("settingsNavigation.coverageXyTolerance"))} tooltip={t("settingsNavigation.coverageXyToleranceTooltip")}>
                                    <NumberWithAddon aria-label={t("settingsNavigation.coverageXyTolerance") + ", m"} aria-description={t("settingsNavigation.coverageXyToleranceTooltip")}  id="setting-coverage_xy_tolerance"
                                        value={values.coverage_xy_tolerance}
                                        onChange={(v) => onChange("coverage_xy_tolerance", v)}
                                        min={0.05} max={1.0} step={0.05} precision={2}
                                        style={{ width: "100%" }} after="m"
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
                            <Form.Item htmlFor="setting-progress_timeout_sec" data-setting-key="progress_timeout_sec" label={fieldLabel("progress_timeout_sec", t("settingsNavigation.progressTimeout"))} tooltip={t("settingsNavigation.progressTimeoutTooltip")}>
                                <NumberWithAddon aria-label={t("settingsNavigation.progressTimeout") + ", s"} aria-description={t("settingsNavigation.progressTimeoutTooltip")}  id="setting-progress_timeout_sec"
                                    value={values.progress_timeout_sec}
                                    onChange={(v) => onChange("progress_timeout_sec", v)}
                                    min={10} max={300} step={10} precision={0}
                                    style={{ width: "100%" }} after="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card size="small" title={t("settingsNavigation.boundaryClearance")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ margin: "0 0 12px", fontSize: 12 }}>
                    {t("settingsNavigation.boundaryClearanceDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-boundary_inner_margin_m" data-setting-key="boundary_inner_margin_m" label={fieldLabel("boundary_inner_margin_m", t("settingsNavigation.boundaryInnerMargin"))} tooltip={t("settingsNavigation.boundaryInnerMarginTooltip")}>
                                <NumberWithAddon aria-label={t("settingsNavigation.boundaryInnerMargin") + ", m"} aria-description={t("settingsNavigation.boundaryInnerMarginTooltip")}  id="setting-boundary_inner_margin_m"
                                    value={values.boundary_inner_margin_m}
                                    onChange={(v) => onChange("boundary_inner_margin_m", v)}
                                    min={0} max={1.0} step={0.05} precision={2}
                                    style={{ width: "100%" }} after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-dock_inner_margin_exempt_radius_m" data-setting-key="dock_inner_margin_exempt_radius_m" label={fieldLabel("dock_inner_margin_exempt_radius_m", t("settingsNavigation.dockExemptRadius"))} tooltip={t("settingsNavigation.dockExemptRadiusTooltip")}>
                                <NumberWithAddon aria-label={t("settingsNavigation.dockExemptRadius") + ", m"} aria-description={t("settingsNavigation.dockExemptRadiusTooltip")}  id="setting-dock_inner_margin_exempt_radius_m"
                                    value={values.dock_inner_margin_exempt_radius_m}
                                    onChange={(v) => onChange("dock_inner_margin_exempt_radius_m", v)}
                                    min={0} max={10.0} step={0.5} precision={2}
                                    style={{ width: "100%" }} after="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_transit_margin_m" data-setting-key="obstacle_transit_margin_m" label={fieldLabel("obstacle_transit_margin_m", t("settingsNavigation.obstacleTransitMargin"))} tooltip={t("settingsNavigation.obstacleTransitMarginTooltip")}>
                                <NumberWithAddon aria-label={t("settingsNavigation.obstacleTransitMargin") + ", m"} aria-description={t("settingsNavigation.obstacleTransitMarginTooltip")}  id="setting-obstacle_transit_margin_m"
                                    value={values.obstacle_transit_margin_m}
                                    onChange={(v) => onChange("obstacle_transit_margin_m", v)}
                                    min={0} max={1.0} step={0.05} precision={2}
                                    style={{ width: "100%" }} after="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
