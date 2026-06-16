import React from "react";
import { Card, Col, Collapse, Form, InputNumber, Row, Space, Switch, Typography } from "antd";
import { HomeOutlined } from "@ant-design/icons";
import { useThemeMode } from "../../theme/ThemeContext.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const DockingSection: React.FC<Props> = ({ values, onChange }) => {
    const { colors } = useThemeMode();
    return (
        <div>
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong className="mn-display" style={{ fontSize: 14, color: colors.text }}>
                            <HomeOutlined style={{ marginRight: 6, color: colors.primary }} />
                            Comportement de l'accostage
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Réglez la façon dont le robot s'approche et quitte la base.
                            La position de la base (X, Y, Yaw) se définit depuis l'éditeur de carte.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item label="Distance de désaccostage" tooltip="Distance de recul en quittant la base">
                                    <InputNumber
                                        value={values.undock_distance}
                                        onChange={(v) => onChange("undock_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label="Vitesse de désaccostage" tooltip="Vitesse de recul au désaccostage">
                                    <InputNumber
                                        value={values.undock_speed}
                                        onChange={(v) => onChange("undock_speed", v)}
                                        min={0.05} max={0.3} step={0.05} precision={2}
                                        style={{ width: "100%" }} addonAfter="m/s"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label="Distance d'approche" tooltip="Distance du point de préparation devant la base">
                                    <InputNumber
                                        value={values.dock_approach_distance}
                                        onChange={(v) => onChange("dock_approach_distance", v)}
                                        min={0.5} max={3.0} step={0.1} precision={2}
                                        style={{ width: "100%" }} addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item label="Détection du chargeur" tooltip="Utiliser la tension de charge pour confirmer l'accostage">
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
                                        Avancé
                                    </Text>
                                ),
                                children: (
                                    <Form layout="vertical" size="small">
                                        <Row gutter={[16, 0]}>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label="Tentatives max" tooltip="Nombre d'essais d'accostage avant d'abandonner">
                                                    <InputNumber
                                                        value={values.dock_max_retries}
                                                        onChange={(v) => onChange("dock_max_retries", v)}
                                                        min={1} max={10} step={1} precision={0}
                                                        style={{ width: "100%" }}
                                                    />
                                                </Form.Item>
                                            </Col>
                                            <Col xs={24} sm={12}>
                                                <Form.Item label="Seuil de charge" tooltip="Courant batterie (A) à partir duquel SimpleChargingDock considère la base atteinte. Plus élevé = le robot s'enfonce davantage dans les contacts avant de s'arrêter ; plus bas = s'arrête plus tôt. ~0,3 A donne un contact stable sur le chargeur YardForce.">
                                                    <InputNumber
                                                        value={values.dock_charging_threshold}
                                                        onChange={(v) => onChange("dock_charging_threshold", v)}
                                                        min={0.05} max={1.0} step={0.05} precision={2}
                                                        style={{ width: "100%" }} addonAfter="A"
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
