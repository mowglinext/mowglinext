import React, { useState } from "react";
import { Alert, Button, Card, Col, Form, Input, InputNumber, Row, Select, Space, Switch, Typography } from "antd";
import { GlobalOutlined, WifiOutlined } from "@ant-design/icons";
import { useApi } from "../../hooks/useApi.ts";
import { App } from "antd";
import { useGnssStatus } from "../../hooks/useGnssStatus.ts";
import { deriveGpsStatus, gnssReceiverLabel } from "../../utils/gpsStatus.ts";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

export const PositioningSection: React.FC<Props> = ({ values, onChange }) => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);
    const gnssStatus = useGnssStatus();
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const statusType: "success" | "warning" | "info" = gpsStatus.fixType === "RTK_FIX"
        ? "success"
        : gpsStatus.fixType === "NO_FIX"
            ? "warning"
            : "info";

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

            <Alert
                showIcon
                type={statusType}
                style={{ marginBottom: 16 }}
                message={`Detected receiver: ${detectedReceiver}`}
                description={`Live Universal GNSS status: ${gpsStatus.label}. Prefer Auto unless you need a specific parser family.`}
            />

            <Card size="small" title="Universal GNSS Receiver" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Receiver Family" tooltip="Auto is recommended. Pick a specific family only when the receiver needs a fixed parser path.">
                                <Select
                                    value={values.gnss_receiver_family ?? "auto"}
                                    onChange={(v) => onChange("gnss_receiver_family", v)}
                                    options={[
                                        { value: "auto", label: "Auto" },
                                        { value: "ublox", label: "u-blox" },
                                        { value: "unicore", label: "Unicore" },
                                        { value: "nmea", label: "NMEA" },
                                    ]}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={12}>
                            <Form.Item label="Serial Device" tooltip="Prefer /dev/serial/by-id/... for USB receivers. Use the UART device path directly for onboard serial.">
                                <Input
                                    value={values.gnss_serial_device ?? "/dev/ttyAMA4"}
                                    onChange={(e) => onChange("gnss_serial_device", e.target.value)}
                                    placeholder="/dev/serial/by-id/..."
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={4}>
                            <Form.Item label="Baud" tooltip="Universal GNSS runtime baud. 921600 is the recommended default.">
                                <Select
                                    value={values.gnss_serial_baud ?? 921600}
                                    onChange={(v) => onChange("gnss_serial_baud", v)}
                                    options={[
                                        { value: 9600, label: "9600" },
                                        { value: 38400, label: "38400" },
                                        { value: 57600, label: "57600" },
                                        { value: 115200, label: "115200" },
                                        { value: 230400, label: "230400" },
                                        { value: 921600, label: "921600" },
                                    ]}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Card size="small" title="Fix Monitoring" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
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
