import React, { useState } from "react";
import { Alert, Button, Card, Col, Form, Input, InputNumber, Row, Select, Space, Switch, Typography } from "antd";
import { GlobalOutlined, WifiOutlined } from "@ant-design/icons";
import { useApi } from "../../hooks/useApi.ts";
import { App } from "antd";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const PositioningSection: React.FC<Props> = ({ values, onChange }) => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);

    const setDatumFromGps = async () => {
        setDatumLoading(true);
        try {
            const res = await guiApi.mowglinext.callCreate("set_datum", {});
            if (res.error) throw new Error(res.error.error);
            const msg: string = (res.data as any)?.message ?? "";
            const parts = msg.split(",");
            if (parts.length === 2) {
                onChange("datum_lat", parseFloat(parts[0]));
                onChange("datum_lon", parseFloat(parts[1]));
                notification.success({ message: "Datum set from current GPS position" });
            }
        } catch (e: any) {
            notification.error({ message: "Failed to get GPS position", description: e.message });
        } finally {
            setDatumLoading(false);
        }
    };

    const ntripEnabled = values.ntrip_enabled ?? true;

    return (
        <div>
            {/* GPS Datum */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <GlobalOutlined style={{ marginRight: 6 }} />
                            Map Origin (Datum)
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            The GPS coordinate used as origin for the local map. Place it near your docking station.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Latitude">
                                    <InputNumber
                                        value={values.datum_lat}
                                        onChange={(v) => onChange("datum_lat", v)}
                                        step={0.0000001} precision={7} style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Longitude">
                                    <InputNumber
                                        value={values.datum_lon}
                                        onChange={(v) => onChange("datum_lon", v)}
                                        step={0.0000001} precision={7} style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={8} style={{ display: "flex", alignItems: "flex-end", paddingBottom: 24 }}>
                                <Button
                                    size="small"
                                    type="link"
                                    loading={datumLoading}
                                    onClick={setDatumFromGps}
                                    style={{ fontSize: 12, padding: 0 }}
                                >
                                    Use current GPS position (requires RTK fix)
                                </Button>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            {/* Serial link */}
            <Card size="small" title="GPS Serial Link" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={12}>
                            <Form.Item label="Device Port" tooltip="Serial device path inside the GPS container — udev maps the USB receiver to this path">
                                <Input
                                    value={values.gps_port ?? "/dev/gps"}
                                    onChange={(e) => onChange("gps_port", e.target.value)}
                                    placeholder="/dev/gps"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={12}>
                            <Form.Item label="Baud Rate" tooltip="Serial baud rate. F9P defaults to 460800; LC29H factory-set to 115200.">
                                <Select
                                    value={values.gps_baudrate ?? 460800}
                                    onChange={(v) => onChange("gps_baudrate", v)}
                                    options={[
                                        { value: 9600, label: "9600" },
                                        { value: 38400, label: "38400" },
                                        { value: 57600, label: "57600" },
                                        { value: 115200, label: "115200" },
                                        { value: 230400, label: "230400" },
                                        { value: 460800, label: "460800" },
                                        { value: 921600, label: "921600" },
                                    ]}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* Protocol & Timeouts */}
            <Card size="small" title="Protocol & Timeouts" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item label="GPS Protocol" tooltip="UBX for u-blox receivers, NMEA for generic">
                                <Select
                                    value={values.gps_protocol ?? "UBX"}
                                    onChange={(v) => onChange("gps_protocol", v)}
                                    options={[
                                        { value: "UBX", label: "UBX (u-blox)" },
                                        { value: "NMEA", label: "NMEA (generic)" },
                                    ]}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label="Wait After Undock" tooltip="Seconds to wait for RTK fix after undocking">
                                <InputNumber
                                    value={values.gps_wait_after_undock_sec}
                                    onChange={(v) => onChange("gps_wait_after_undock_sec", v)}
                                    min={0} step={1} style={{ width: "100%" }}
                                    addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12} sm={8}>
                            <Form.Item label="GPS Timeout" tooltip="Pause mowing if no fix for this long">
                                <InputNumber
                                    value={values.gps_timeout_sec}
                                    onChange={(v) => onChange("gps_timeout_sec", v)}
                                    min={1} step={1} style={{ width: "100%" }}
                                    addonAfter="s"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            {/* NTRIP Configuration */}
            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                        <div>
                            <Text strong style={{ fontSize: 14 }}>
                                <WifiOutlined style={{ marginRight: 6 }} />
                                NTRIP RTK Corrections
                            </Text>
                            <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                                Enable for centimetre-level accuracy via RTK base station.
                            </Paragraph>
                        </div>
                        <Switch
                            checked={ntripEnabled}
                            onChange={(v) => onChange("ntrip_enabled", v)}
                        />
                    </div>

                    {ntripEnabled && (
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={12} lg={8}>
                                    <Form.Item label="Host" tooltip="NTRIP caster hostname or IP">
                                        <Input
                                            value={values.ntrip_host ?? ""}
                                            onChange={(e) => onChange("ntrip_host", e.target.value)}
                                            placeholder="caster.example.com"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Port">
                                        <InputNumber
                                            value={values.ntrip_port ?? 2101}
                                            onChange={(v) => onChange("ntrip_port", v)}
                                            min={1} max={65535} style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Mountpoint">
                                        <Input
                                            value={values.ntrip_mountpoint ?? ""}
                                            onChange={(e) => onChange("ntrip_mountpoint", e.target.value)}
                                            placeholder="RTCM3"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Username">
                                        <Input
                                            value={values.ntrip_user ?? ""}
                                            onChange={(e) => onChange("ntrip_user", e.target.value)}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Password">
                                        <Input.Password
                                            value={values.ntrip_password ?? ""}
                                            onChange={(e) => onChange("ntrip_password", e.target.value)}
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                    )}

                    {!ntripEnabled && (
                        <Alert
                            type="info"
                            showIcon
                            message="Without NTRIP, GPS accuracy is ~2-5m (not suitable for autonomous mowing)."
                            style={{ marginTop: 0 }}
                        />
                    )}
                </Space>
            </Card>
        </div>
    );
};
