import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { AimOutlined, RadarChartOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { RobotComponentEditor } from "../RobotComponentEditor.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SensorsSection: React.FC<Props> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const handleLidarToggle = (enabled: boolean) => {
        onChange("lidar_enabled", enabled);
    };
    const lidarEnabled = values.lidar_enabled ?? false;
    const pwmEnabled = values.lidar_pwm_enabled ?? false;

    return (
        <div>
            {/* LiDAR toggle */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <RadarChartOutlined style={{ marginRight: 6 }} />
                            {t("settingsSensors.lidarSensor")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsSensors.lidarDescription")}
                        </Paragraph>
                    </div>
                    <Switch
                        checked={lidarEnabled}
                        onChange={handleLidarToggle}
                    />
                </div>
            </Card>

            {/* LiDAR motor PWM spin-down (issue #569) */}
            {lidarEnabled && (
                <Card size="small" style={{ marginBottom: 16 }}>
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                        <div>
                            <Text strong style={{ fontSize: 14 }}>
                                {t("settingsSensors.lidarPwm")}
                            </Text>
                            <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                                {t("settingsSensors.lidarPwmDescription")}
                            </Paragraph>
                        </div>
                        <Switch
                            checked={pwmEnabled}
                            onChange={(checked) => onChange("lidar_pwm_enabled", checked)}
                        />
                    </div>

                    {pwmEnabled && (
                        <>
                            <Alert
                                type="warning"
                                showIcon
                                style={{ margin: "16px 0" }}
                                message={t("settingsSensors.lidarPwmPrerequisitesTitle")}
                                description={
                                    <Space direction="vertical" size={4} style={{ fontSize: 12 }}>
                                        <span>{t("settingsSensors.lidarPwmPrerequisiteBootConfig")}</span>
                                        <span>
                                            <Text code>dtoverlay=uart5</Text>{" "}
                                            {t("settingsSensors.lidarPwmPrerequisiteThenLine")}{" "}
                                            <Text code>dtoverlay=pwm,pin=12,func=4</Text>
                                        </span>
                                        <span>{t("settingsSensors.lidarPwmPrerequisiteReboot")}</span>
                                        <span>
                                            {t("settingsSensors.lidarPwmPrerequisiteVerify")}{" "}
                                            <Text code>ls /sys/class/pwm/</Text>
                                        </span>
                                    </Space>
                                }
                            />
                            <Form layout="vertical" size="small">
                                <Row gutter={[16, 0]}>
                                    <Col xs={24} sm={12}>
                                        <Form.Item
                                            label={t("settingsSensors.lidarPwmGpioPin")}
                                            tooltip={t("settingsSensors.lidarPwmGpioPinTooltip")}
                                        >
                                            <InputNumber
                                                value={values.lidar_pwm_gpio_pin}
                                                onChange={(v) => onChange("lidar_pwm_gpio_pin", v)}
                                                min={0} max={27} step={1} precision={0}
                                                style={{ width: "100%" }}
                                            />
                                        </Form.Item>
                                    </Col>
                                </Row>
                            </Form>
                        </>
                    )}
                </Card>
            )}

            {/* Sensor placement visual editor */}
            <RobotComponentEditor values={values} onChange={onChange} />

            {/* IMU bias calibration (hardware_bridge_node, auto-triggered on dock) */}
            <Card size="small" style={{ marginTop: 16 }} title={
                <Text strong style={{ fontSize: 14 }}>
                    <AimOutlined style={{ marginRight: 6 }} />
                    {t("settingsSensors.imuBiasCalibration")}
                </Text>
            }>
                <Paragraph type="secondary" style={{ margin: "0 0 12px", fontSize: 12 }}>
                    {t("settingsSensors.imuBiasCalibrationDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsSensors.calibrationSamples")} tooltip={t("settingsSensors.calibrationSamplesTooltip")}>
                                <InputNumber
                                    value={values.imu_cal_samples}
                                    onChange={(v) => onChange("imu_cal_samples", v)}
                                    min={50} max={2000} step={50} precision={0}
                                    style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsSensors.restWindowBeforeCal")} tooltip={t("settingsSensors.restWindowBeforeCalTooltip")}>
                                <InputNumber
                                    value={values.imu_cal_auto_rest_sec}
                                    onChange={(v) => onChange("imu_cal_auto_rest_sec", v)}
                                    min={1} max={120} step={1} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label={t("settingsSensors.periodicRecalInterval")} tooltip={t("settingsSensors.periodicRecalIntervalTooltip")}>
                                <InputNumber
                                    value={values.imu_cal_periodic_recal_sec}
                                    onChange={(v) => onChange("imu_cal_periodic_recal_sec", v)}
                                    min={0} max={3600} step={30} precision={0}
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
