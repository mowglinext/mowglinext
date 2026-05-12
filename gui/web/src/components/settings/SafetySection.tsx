import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { SafetyOutlined, WarningOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const SafetySection: React.FC<Props> = ({ values, onChange }) => {
    const { colors } = useThemeMode();

    return (
        <div>
            <Alert
                type="warning"
                showIcon
                icon={<WarningOutlined />}
                message="Safety settings protect the robot and its surroundings"
                description="Firmware is the sole safety authority. These settings configure firmware behavior — disabling them reduces protection."
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
            <Card size="small" title="Motor Temperature Limits" style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    Blade motor stops above the high threshold and resumes below the low threshold (hysteresis).
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12}>
                            <Form.Item
                                label={<Text style={{ color: "#f5222d", fontSize: 12 }}>Stop Above</Text>}
                                tooltip="Stop blade motor above this temperature"
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
                                label={<Text style={{ color: "#52c41a", fontSize: 12 }}>Resume Below</Text>}
                                tooltip="Resume blade motor below this temperature"
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

            {/* Obstacle avoidance */}
            <Card size="small" title="Obstacle Avoidance" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Max Detour Distance" tooltip="Maximum distance to detour around an obstacle before giving up">
                                <InputNumber
                                    value={values.max_obstacle_avoidance_distance}
                                    onChange={(v) => onChange("max_obstacle_avoidance_distance", v)}
                                    min={0.5} max={10} step={0.5} precision={1}
                                    style={{ width: "100%" }} addonAfter="m"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
