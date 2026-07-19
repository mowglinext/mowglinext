import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";

const { Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
};

/**
 * Obstacles section — operator-tunable obstacle-avoidance margins:
 *   - obstacle_inflation_radius: local-costmap buffer around LiDAR-seen
 *     obstacles (trunks, legs, walls),
 *   - max_obstacle_avoidance_distance: max lateral detour for coverage
 *     skirting + bypass give-up threshold (one knob, two consumers),
 *   - obstacle_margin: extra margin grown around DRAWN map obstacles in both
 *     coverage and transit planning (root zones the 2D LiDAR cannot see),
 *   - obstacle_slowdown_ratio: collision_monitor approach slowdown factor.
 * All keys live in mowgli_robot.yaml (sparse over template) and are injected
 * into Nav2/coverage params at launch — changes need a ROS2 restart.
 */
export const ObstaclesSection: React.FC<Props> = ({
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
            <Alert
                type="info"
                showIcon
                message={t("settingsObstacles.rootZoneHintTitle")}
                description={t("settingsObstacles.rootZoneHintDescription")}
                style={{ marginBottom: 16 }}
            />

            {/* Avoidance margins */}
            <Card size="small" title={t("settingsObstacles.avoidanceMargins")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsObstacles.avoidanceMarginsDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_inflation_radius", t("settingsObstacles.inflationRadius"))}
                                tooltip={t("settingsObstacles.inflationRadiusTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_inflation_radius}
                                    onChange={(v) => onChange("obstacle_inflation_radius", v)}
                                    min={0.58} max={1.5} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("max_obstacle_avoidance_distance", t("settingsObstacles.maxDetourDistance"))}
                                tooltip={t("settingsObstacles.maxDetourDistanceTooltip")}
                            >
                                <InputNumber
                                    value={values.max_obstacle_avoidance_distance}
                                    onChange={(v) => onChange("max_obstacle_avoidance_distance", v)}
                                    min={0.5} max={10} step={0.5} precision={1}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_margin", t("settingsObstacles.drawnObstacleMargin"))}
                                tooltip={t("settingsObstacles.drawnObstacleMarginTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_margin}
                                    onChange={(v) => onChange("obstacle_margin", v)}
                                    min={0} max={1} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_clearance_margin", t("settingsObstacles.clearanceMargin"))}
                                tooltip={t("settingsObstacles.clearanceMarginTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_clearance_margin}
                                    onChange={(v) => onChange("obstacle_clearance_margin", v)}
                                    min={0} max={0.5} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_detection_range_m", t("settingsObstacles.detectionRange"))}
                                tooltip={t("settingsObstacles.detectionRangeTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_detection_range_m}
                                    onChange={(v) => onChange("obstacle_detection_range_m", v)}
                                    min={0.2} max={5} step={0.1} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_wait_timeout_s", t("settingsObstacles.waitTimeout"))}
                                tooltip={t("settingsObstacles.waitTimeoutTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_wait_timeout_s}
                                    onChange={(v) => onChange("obstacle_wait_timeout_s", v)}
                                    min={0.5} max={60} step={0.5} precision={1}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* Approach slowdown */}
            <Card size="small" title={t("settingsObstacles.approachSlowdown")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsObstacles.approachSlowdownDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={fieldLabel("obstacle_slowdown_ratio", t("settingsObstacles.slowdownRatio"))}
                                tooltip={t("settingsObstacles.slowdownRatioTooltip")}
                            >
                                <InputNumber
                                    value={values.obstacle_slowdown_ratio}
                                    onChange={(v) => onChange("obstacle_slowdown_ratio", v)}
                                    min={0.05} max={1} step={0.05} precision={2}
                                    style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
