import React, { useState } from "react";
import { Alert, App, Button, Card, Col, Form, Input, InputNumber, Row, Select, Space, Switch, Tooltip, Typography } from "antd";
import { GlobalOutlined, ReloadOutlined, SaveOutlined, SettingOutlined, WifiOutlined } from "@ant-design/icons";
import { useApi } from "../../hooks/useApi.ts";
import { useDiagnostics } from "../../hooks/useDiagnostics.ts";
import { useGnssStatus } from "../../hooks/useGnssStatus.ts";
import { deriveGpsStatus, gnssReceiverLabel } from "../../utils/gpsStatus.ts";
import {
    GNSS_BAUD_OPTIONS,
    GNSS_PROFILE_OPTIONS,
    GNSS_PROFILE_RATE_OPTIONS,
    GNSS_RECEIVER_FAMILY_OPTIONS,
    GNSS_SIGNAL_PROFILE_OPTIONS,
    GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT,
    normalizeGnssProfile,
    normalizeGnssSignalProfile,
} from "./gnssConfig.ts";
import { GnssSignalProfileHelp } from "./GnssSignalProfileHelp.tsx";
import { UniversalGnssAdvancedSettings } from "./UniversalGnssAdvancedSettings.tsx";
import { UniversalGnssLiveStatusCard } from "./UniversalGnssLiveStatusCard.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isDirty: boolean;
    saving: boolean;
    gpsRestarting: boolean;
    onSave: () => void | Promise<void>;
    onSaveAndRestartGps: () => void | Promise<void>;
};

const PROFILE_APPLY_GAP =
    "Backend API missing: the GUI cannot yet execute gnss_config_apply inside mowgli-gps " +
    "or translate the saved vendor-neutral GUI settings into receiver-specific apply plans.";

export const PositioningSection: React.FC<Props> = ({
    values,
    onChange,
    isDirty,
    saving,
    gpsRestarting,
    onSave,
    onSaveAndRestartGps,
}) => {
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
    const { diagnostics } = useDiagnostics();
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const selectedSignalProfile = normalizeGnssSignalProfile(values.gnss_signal_profile);
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
                                        onChange={(value) => onChange("datum_lat", value)}
                                        step={0.0000001}
                                        precision={7}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Longitude">
                                    <InputNumber
                                        value={values.datum_lon}
                                        onChange={(value) => onChange("datum_lon", value)}
                                        step={0.0000001}
                                        precision={7}
                                        style={{ width: "100%" }}
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
                description={`Live Universal GNSS status: ${gpsStatus.label}. The main UI stays vendor-neutral; expert mode exposes receiver-family-specific tuning only when needed.`}
            />

            <Card
                size="small"
                title="GNSS Profiles"
                extra={(
                    <Space size="small">
                        <Text type="secondary" style={{ fontSize: 12 }}>Expert mode</Text>
                        <Switch size="small" checked={expertMode} onChange={setExpertMode} />
                    </Space>
                )}
                style={{ marginBottom: 16 }}
            >
                <Paragraph type="secondary" style={{ marginTop: 0 }}>
                    Normal settings are vendor-neutral. Expert settings are receiver-family specific and stay hidden until you explicitly enable them.
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={12}>
                            <Form.Item label="Receiver Profile" tooltip="Universal GNSS receiver profile id. Backend translation to receiver-specific commands is still pending.">
                                <Select
                                    value={normalizeGnssProfile(values.gnss_profile)}
                                    onChange={(value) => onChange("gnss_profile", value)}
                                    options={GNSS_PROFILE_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                    }))}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={12}>
                            <Form.Item
                                label="Signal Profile"
                                tooltip="High-level constellation and signal preset. Use Expert mode only when you need family-specific overrides."
                                extra={<GnssSignalProfileHelp selectedProfile={selectedSignalProfile} />}
                            >
                                <Select
                                    value={selectedSignalProfile}
                                    onChange={(value) => onChange("gnss_signal_profile", value)}
                                    options={GNSS_SIGNAL_PROFILE_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                        description: option.description,
                                    }))}
                                    optionRender={(option) => (
                                        <div>
                                            <div>{String(option.data.label)}</div>
                                            <Text type="secondary" style={{ fontSize: 12 }}>
                                                {String(option.data.description ?? "")}
                                            </Text>
                                        </div>
                                    )}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Position Rate" tooltip="Prepared for future backend profile application support.">
                                <Select
                                    value={values.gnss_profile_rate_hz ?? 5}
                                    onChange={(value) => onChange("gnss_profile_rate_hz", value)}
                                    options={GNSS_PROFILE_RATE_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                    }))}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Runtime Baud" tooltip="The runtime baud used by the gps sidecar.">
                                <Select
                                    value={values.gnss_serial_baud ?? 921600}
                                    onChange={(value) => onChange("gnss_serial_baud", value)}
                                    options={GNSS_BAUD_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                    }))}
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={24} sm={8}>
                            <Form.Item label="Configured Receiver Baud" tooltip="Persisted target baud for future receiver-side profile application.">
                                <Select
                                    value={values.gnss_config_baud ?? values.gnss_serial_baud ?? 921600}
                                    onChange={(value) => onChange("gnss_config_baud", value)}
                                    options={GNSS_BAUD_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: option.label,
                                    }))}
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message="Receiver baud and profile guidance"
                description={
                    <span>
                        Changing baud also requires the receiver itself to be configured to the same baud.{" "}
                        <Text strong>460800</Text> is recommended for unstable USB serial links.{" "}
                        <Text strong>921600</Text> may work on direct UART or robust USB adapters, but it must be validated.{" "}
                        Factory reset clears receiver settings before rebuilding the selected profile.{" "}
                        {selectedSignalProfile === "custom" && GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT}
                    </span>
                }
            />

            <UniversalGnssLiveStatusCard
                diagnostics={diagnostics}
                gnssStatus={gnssStatus}
                selectedBaud={values.gnss_serial_baud}
                selectedConfigBaud={values.gnss_config_baud}
                selectedProfile={values.gnss_profile}
                selectedSignalProfile={values.gnss_signal_profile}
                selectedReceiverFamily={values.gnss_receiver_family}
            />

            {expertMode && (
                <>
                    <Card size="small" title={<Space><SettingOutlined /> Expert GNSS Settings</Space>} style={{ marginBottom: 16 }}>
                        <Paragraph type="secondary" style={{ marginTop: 0 }}>
                            Receiver-family selection, raw serial wiring, and vendor-specific overrides live here.
                            Keep these at their defaults unless you know the receiver-side implications.
                        </Paragraph>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={10}>
                                    <Form.Item label="Receiver Family">
                                        <Select
                                            value={values.gnss_receiver_family ?? "auto"}
                                            onChange={(value) => onChange("gnss_receiver_family", value)}
                                            options={GNSS_RECEIVER_FAMILY_OPTIONS.map((option) => ({
                                                value: option.value,
                                                label: option.label,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={14}>
                                    <Form.Item label="Serial Device">
                                        <Input
                                            value={values.gnss_serial_device ?? "/dev/ttyAMA4"}
                                            onChange={(event) => onChange("gnss_serial_device", event.target.value)}
                                            placeholder="/dev/serial/by-id/..."
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                            <Row gutter={[16, 0]}>
                                <Col xs={12} sm={6}>
                                    <Form.Item label="RTK Wait After Undock">
                                        <InputNumber
                                            value={values.gps_wait_after_undock_sec}
                                            onChange={(value) => onChange("gps_wait_after_undock_sec", value)}
                                            min={0}
                                            step={1}
                                            style={{ width: "100%" }}
                                            addonAfter="s"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6}>
                                    <Form.Item label="GPS Timeout">
                                        <InputNumber
                                            value={values.gps_timeout_sec}
                                            onChange={(value) => onChange("gps_timeout_sec", value)}
                                            min={1}
                                            step={1}
                                            style={{ width: "100%" }}
                                            addonAfter="s"
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                        <Alert
                            type="info"
                            showIcon
                            message="Expert-mode scope"
                            description="Signal-group presets, raw signal-group values, PVT algorithm, RTK reliability, RTK timeout, and DGPS timeout are stored here for future backend translation. Raw command textarea, dry-run plan, and reset/apply tooling still need a dedicated API."
                        />
                    </Card>

                    <UniversalGnssAdvancedSettings
                        receiverFamily={values.gnss_receiver_family ?? "auto"}
                        values={values}
                        onChange={onChange}
                    />
                </>
            )}

            <Card size="small" title="Receiver Actions" style={{ marginBottom: 16 }}>
                <Space wrap size={[8, 8]}>
                    <Button
                        type="primary"
                        icon={<SaveOutlined />}
                        onClick={onSave}
                        loading={saving && !gpsRestarting}
                        disabled={!isDirty || gpsRestarting}
                    >
                        Save settings
                    </Button>
                    <Tooltip title={PROFILE_APPLY_GAP}>
                        <span>
                            <Button disabled>
                                Apply profile to receiver
                            </Button>
                        </span>
                    </Tooltip>
                    <Button
                        icon={<ReloadOutlined />}
                        onClick={onSaveAndRestartGps}
                        loading={gpsRestarting}
                        disabled={saving || gpsRestarting}
                    >
                        Save + restart GPS
                    </Button>
                    <Tooltip title={PROFILE_APPLY_GAP}>
                        <span>
                            <Button danger disabled>
                                Factory reset + apply profile
                            </Button>
                        </span>
                    </Tooltip>
                </Space>
                <Alert
                    type="warning"
                    showIcon
                    style={{ marginTop: 12 }}
                    message="Apply/reset actions are placeholders for now"
                    description="The GUI can already persist GNSS_PROFILE, GNSS_SIGNAL_PROFILE, GNSS_PROFILE_RATE_HZ, GNSS_SERIAL_BAUD, GNSS_CONFIG_BAUD, GNSS_RECEIVER_FAMILY, and GNSS_SERIAL_DEVICE, but there is no backend endpoint yet that runs gnss_config_apply, previews a dry-run plan, sends raw developer commands, or performs a confirmed factory reset flow."
                />
            </Card>

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
                            onChange={(value) => onChange("ntrip_enabled", value)}
                        />
                    </div>

                    {ntripEnabled && (
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={12} lg={8}>
                                    <Form.Item label="Host" tooltip="NTRIP caster hostname or IP">
                                        <Input
                                            value={values.ntrip_host ?? ""}
                                            onChange={(event) => onChange("ntrip_host", event.target.value)}
                                            placeholder="caster.example.com"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Port">
                                        <InputNumber
                                            value={values.ntrip_port ?? 2101}
                                            onChange={(value) => onChange("ntrip_port", value)}
                                            min={1}
                                            max={65535}
                                            style={{ width: "100%" }}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Mountpoint">
                                        <Input
                                            value={values.ntrip_mountpoint ?? ""}
                                            onChange={(event) => onChange("ntrip_mountpoint", event.target.value)}
                                            placeholder="RTCM3"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Username">
                                        <Input
                                            value={values.ntrip_user ?? ""}
                                            onChange={(event) => onChange("ntrip_user", event.target.value)}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6} lg={4}>
                                    <Form.Item label="Password">
                                        <Input.Password
                                            value={values.ntrip_password ?? ""}
                                            onChange={(event) => onChange("ntrip_password", event.target.value)}
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
