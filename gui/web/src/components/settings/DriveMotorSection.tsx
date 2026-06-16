import React from "react";
import { Alert, Card, Col, Form, InputNumber, Row, Space, Typography } from "antd";
import { DashboardOutlined } from "@ant-design/icons";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

// Firmware per-wheel velocity PID + feedforward. Ranges match the clamps the
// STM32 applies on receipt (cpp_main.cpp on_set_drive_pid) and the schema.
export const DriveMotorSection: React.FC<Props> = ({ values, onChange }) => {
    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message="Saved and applied live"
                description="Saving stores these in mowgli_robot.yaml and pushes them to the drive controller immediately — no ROS2 restart needed. They persist across reboots: on every boot the robot re-sends the saved values to the firmware. (The firmware itself has no storage, so it runs its built-in defaults only for the brief moment before the controller reconnects, then re-validates and clamps the values you saved.)"
            />
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <DashboardOutlined style={{ marginRight: 6 }} />
                            Wheel Velocity PID
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Closed-loop gains the STM32 firmware uses to track each wheel's commanded
                            speed. Higher Kp/Ki give stiffer tracking; too high causes oscillation or
                            overshoot. Kd is normally 0.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kp" tooltip="Proportional gain (PWM per m/s)">
                                    <InputNumber
                                        value={values.wheel_pid_kp}
                                        onChange={(v) => onChange("wheel_pid_kp", v)}
                                        min={0} max={200} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Ki" tooltip="Integral gain (PWM per m/s·s) — bridges the static-friction deadband">
                                    <InputNumber
                                        value={values.wheel_pid_ki}
                                        onChange={(v) => onChange("wheel_pid_ki", v)}
                                        min={0} max={20000} step={100} precision={0}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kd" tooltip="Derivative gain (PWM per m/s²) — 0 disables">
                                    <InputNumber
                                        value={values.wheel_pid_kd}
                                        onChange={(v) => onChange("wheel_pid_kd", v)}
                                        min={0} max={500} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Integral Limit" tooltip="Anti-windup clamp on the integral term (PWM, motor max 255)">
                                    <InputNumber
                                        value={values.wheel_pid_integral_limit}
                                        onChange={(v) => onChange("wheel_pid_integral_limit", v)}
                                        min={0} max={255} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="PWM"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title="Feedforward" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label="PWM per m/s"
                                tooltip="Open-loop velocity→PWM feedforward scale. Dominant drive term; also sets the idle/deadband mapping. Change with care."
                            >
                                <InputNumber
                                    value={values.wheel_pid_pwm_per_mps}
                                    onChange={(v) => onChange("wheel_pid_pwm_per_mps", v)}
                                    min={50} max={600} step={10} precision={0}
                                    style={{ width: "100%" }} addonAfter="PWM"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>
        </div>
    );
};
