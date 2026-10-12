import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Segmented, Switch, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { parseBoolish } from "../../utils/settingsValues.ts";
import { SettingFieldLabel } from "./SettingFieldLabel.tsx";
import { besideBodyClearanceM, planningObstacleMarginFloorM } from "../../utils/obstacleMargin.ts";

const { Paragraph } = Typography;

export const DIG_SENSITIVITY_LEVELS = ["off", "low", "medium", "high"] as const;
export type DigSensitivity = (typeof DIG_SENSITIVITY_LEVELS)[number];

/**
 * The yaml may hold a bare `off`, which YAML 1.1 parses as boolean false — the
 * launch side reads that as "off" too (robot_config_util.resolve_dig_sensitivity).
 * Anything unknown shows as the shipped default.
 */
export const normalizeDigSensitivity = (raw: unknown): DigSensitivity => {
    if (raw === false) return "off";
    if (typeof raw !== "string") return "medium";
    const level = raw.trim().toLowerCase();
    return (DIG_SENSITIVITY_LEVELS as readonly string[]).includes(level)
        ? (level as DigSensitivity)
        : "medium";
};

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isOverridden?: (key: string) => boolean;
    hasDefault?: (key: string) => boolean;
    onReset?: (key: string) => void;
    // Used only to fall back to the shipped chassis_width/obstacle_clearance_margin
    // when a sparse (untouched) robot config omits them — see obstacleMargin.ts.
    defaults?: Record<string, any>;
};

/**
 * Obstacles section — automatic dig keepouts and obstacle-avoidance margins:
 *   - obstacle_inflation_radius: local-costmap buffer around LiDAR-seen
 *     obstacles (trunks, legs, walls),
 *   - max_obstacle_avoidance_distance: max lateral detour for coverage
 *     skirting + bypass give-up threshold (one knob, two consumers),
 *   - obstacle_margin: coverage-plan clearance around DRAWN map obstacles (the
 *     transit keepout band is derived from the chassis and follows it up),
 *   - obstacle_slowdown_ratio: collision_monitor approach slowdown factor.
 * All keys live in mowgli_robot.yaml (sparse over template) and are injected
 * into map server/Nav2/coverage params at launch — changes need a ROS2 restart.
 */
export const ObstaclesSection: React.FC<Props> = ({
    values,
    onChange,
    isOverridden,
    hasDefault,
    onReset,
    defaults,
}) => {
    const { t } = useTranslation();

    const chassisWidthM = values.chassis_width ?? defaults?.chassis_width;
    const clearanceMarginM = values.obstacle_clearance_margin ?? defaults?.obstacle_clearance_margin;
    const obstacleMarginFloorM = planningObstacleMarginFloorM(chassisWidthM, clearanceMarginM);
    const requestedMarginM = values.obstacle_margin ?? defaults?.obstacle_margin ?? obstacleMarginFloorM;
    const effectiveMarginM = Math.max(requestedMarginM, obstacleMarginFloorM);
    const isBelowFloor = requestedMarginM < obstacleMarginFloorM;
    const besideBodyCm = Math.round(besideBodyClearanceM(effectiveMarginM, chassisWidthM) * 100);
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

            <Card size="small" title={t("settingsObstacles.digKeepouts")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsObstacles.digKeepoutsDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Form.Item
                        label={fieldLabel("dig_sensitivity", t("settingsObstacles.digSensitivity"))}
                        extra={t("settingsObstacles.digSensitivityHelp")}
                    >
                        <Segmented
                            aria-label={t("settingsObstacles.digSensitivity")}
                            value={normalizeDigSensitivity(values.dig_sensitivity)}
                            onChange={(level) => onChange("dig_sensitivity", level)}
                            options={DIG_SENSITIVITY_LEVELS.map((level) => ({
                                value: level,
                                label: t(`settingsObstacles.digSensitivityLevels.${level}`),
                            }))}
                        />
                    </Form.Item>
                    <Form.Item label={fieldLabel("dig_obstacle_enabled", t("settingsObstacles.digAutoPromotion"))}>
                        <Switch
                            aria-label={t("settingsObstacles.digAutoPromotion")}
                            checked={parseBoolish(values.dig_obstacle_enabled) ?? false}
                            onChange={(enabled) => onChange("dig_obstacle_enabled", enabled)}
                        />
                    </Form.Item>
                </Form>
            </Card>

            {/* Avoidance margins */}
            <Card size="small" title={t("settingsObstacles.avoidanceMargins")} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("settingsObstacles.avoidanceMarginsDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_inflation_radius" data-setting-key="obstacle_inflation_radius"
                                label={fieldLabel("obstacle_inflation_radius", t("settingsObstacles.inflationRadius"))}
                                tooltip={t("settingsObstacles.inflationRadiusTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.inflationRadius") + ", m"} aria-description={t("settingsObstacles.inflationRadiusTooltip")}  id="setting-obstacle_inflation_radius"
                                    value={values.obstacle_inflation_radius}
                                    onChange={(v) => onChange("obstacle_inflation_radius", v)}
                                    min={0.58} max={1.5} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-max_obstacle_avoidance_distance" data-setting-key="max_obstacle_avoidance_distance"
                                label={fieldLabel("max_obstacle_avoidance_distance", t("settingsObstacles.maxDetourDistance"))}
                                tooltip={t("settingsObstacles.maxDetourDistanceTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.maxDetourDistance") + ", m"} aria-description={t("settingsObstacles.maxDetourDistanceTooltip")}  id="setting-max_obstacle_avoidance_distance"
                                    value={values.max_obstacle_avoidance_distance}
                                    onChange={(v) => onChange("max_obstacle_avoidance_distance", v)}
                                    min={0.5} max={10} step={0.5} precision={1}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_margin" data-setting-key="obstacle_margin"
                                label={fieldLabel("obstacle_margin", t("settingsObstacles.drawnObstacleMargin"))}
                                tooltip={t("settingsObstacles.drawnObstacleMarginTooltip")}
                                extra={
                                    isBelowFloor
                                        ? undefined
                                        : t("settingsObstacles.drawnObstacleMarginBesideBody", { cm: besideBodyCm })
                                }
                            >
                                <InputNumber aria-label={t("settingsObstacles.drawnObstacleMargin") + ", m"} aria-description={t("settingsObstacles.drawnObstacleMarginTooltip")}  id="setting-obstacle_margin"
                                    value={values.obstacle_margin}
                                    onChange={(v) => onChange("obstacle_margin", v)}
                                    min={0} max={1} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                            {isBelowFloor && (
                                <Alert
                                    type="warning"
                                    showIcon
                                    style={{ marginTop: -8, marginBottom: 12, fontSize: 12 }}
                                    message={t("settingsObstacles.drawnObstacleMarginFlooredWarning", {
                                        floorM: obstacleMarginFloorM.toFixed(3),
                                        floorCm: besideBodyCm,
                                    })}
                                />
                            )}
                        </Col>
                    </Row>
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_clearance_margin" data-setting-key="obstacle_clearance_margin"
                                label={fieldLabel("obstacle_clearance_margin", t("settingsObstacles.clearanceMargin"))}
                                tooltip={t("settingsObstacles.clearanceMarginTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.clearanceMargin") + ", m"} aria-description={t("settingsObstacles.clearanceMarginTooltip")}  id="setting-obstacle_clearance_margin"
                                    value={values.obstacle_clearance_margin}
                                    onChange={(v) => onChange("obstacle_clearance_margin", v)}
                                    min={0} max={0.5} step={0.05} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_detection_range_m" data-setting-key="obstacle_detection_range_m"
                                label={fieldLabel("obstacle_detection_range_m", t("settingsObstacles.detectionRange"))}
                                tooltip={t("settingsObstacles.detectionRangeTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.detectionRange") + ", m"} aria-description={t("settingsObstacles.detectionRangeTooltip")}  id="setting-obstacle_detection_range_m"
                                    value={values.obstacle_detection_range_m}
                                    onChange={(v) => onChange("obstacle_detection_range_m", v)}
                                    min={0.2} max={5} step={0.1} precision={2}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item htmlFor="setting-obstacle_wait_timeout_s" data-setting-key="obstacle_wait_timeout_s"
                                label={fieldLabel("obstacle_wait_timeout_s", t("settingsObstacles.waitTimeout"))}
                                tooltip={t("settingsObstacles.waitTimeoutTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.waitTimeout") + ", s"} aria-description={t("settingsObstacles.waitTimeoutTooltip")}  id="setting-obstacle_wait_timeout_s"
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
                            <Form.Item htmlFor="setting-obstacle_slowdown_ratio" data-setting-key="obstacle_slowdown_ratio"
                                label={fieldLabel("obstacle_slowdown_ratio", t("settingsObstacles.slowdownRatio"))}
                                tooltip={t("settingsObstacles.slowdownRatioTooltip")}
                            >
                                <InputNumber aria-label={t("settingsObstacles.slowdownRatio")} aria-description={t("settingsObstacles.slowdownRatioTooltip")}  id="setting-obstacle_slowdown_ratio"
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
