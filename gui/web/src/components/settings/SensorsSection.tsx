import React from "react";
import { Card, Col, Form, InputNumber, Row, Switch, Typography } from "antd";
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
                        aria-label={t("settingsSensors.lidarSensor")}
                        checked={values.lidar_enabled ?? false}
                        onChange={handleLidarToggle}
                    />
                </div>
            </Card>

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
                            <Form.Item htmlFor="setting-imu_cal_samples" data-setting-key="imu_cal_samples" label={t("settingsSensors.calibrationSamples")} tooltip={t("settingsSensors.calibrationSamplesTooltip")}>
                                <InputNumber aria-label={t("settingsSensors.calibrationSamples")} aria-description={t("settingsSensors.calibrationSamplesTooltip")}  id="setting-imu_cal_samples"
                                    value={values.imu_cal_samples}
                                    onChange={(v) => onChange("imu_cal_samples", v)}
                                    min={50} max={2000} step={50} precision={0}
                                    style={{ width: "100%" }}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item htmlFor="setting-imu_cal_auto_rest_sec" data-setting-key="imu_cal_auto_rest_sec" label={t("settingsSensors.restWindowBeforeCal")} tooltip={t("settingsSensors.restWindowBeforeCalTooltip")}>
                                <InputNumber aria-label={t("settingsSensors.restWindowBeforeCal") + ", s"} aria-description={t("settingsSensors.restWindowBeforeCalTooltip")}  id="setting-imu_cal_auto_rest_sec"
                                    value={values.imu_cal_auto_rest_sec}
                                    onChange={(v) => onChange("imu_cal_auto_rest_sec", v)}
                                    min={1} max={120} step={1} precision={0}
                                    style={{ width: "100%" }} addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item htmlFor="setting-imu_cal_periodic_recal_sec" data-setting-key="imu_cal_periodic_recal_sec" label={t("settingsSensors.periodicRecalInterval")} tooltip={t("settingsSensors.periodicRecalIntervalTooltip")}>
                                <InputNumber aria-label={t("settingsSensors.periodicRecalInterval") + ", s"} aria-description={t("settingsSensors.periodicRecalIntervalTooltip")}  id="setting-imu_cal_periodic_recal_sec"
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
