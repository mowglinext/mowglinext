import React, { useState } from "react";
import { Alert, App, Button, Card, Col, Form, Input, InputNumber, Row, Select, Space, Switch, Typography } from "antd";
import { GlobalOutlined, SettingOutlined } from "@ant-design/icons";
import { useApi } from "../../hooks/useApi.ts";
import { useDiagnostics } from "../../hooks/useDiagnostics.ts";
import { useGnssStatus } from "../../hooks/useGnssStatus.ts";
import { deriveGpsStatus, gnssReceiverLabel } from "../../utils/gpsStatus.ts";
import {
    GNSS_BAUD_OPTIONS,
    GNSS_ACTION_SETTINGS_KEYS,
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
import { GnssReceiverActionsCard } from "./GnssReceiverActionsCard.tsx";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    isDirty: boolean;
    saving: boolean;
    gpsRestarting: boolean;
    onSave: () => void | Promise<void>;
    onSaveAndRestartGps: () => void | Promise<void>;
    onPersistGnssSettings: (settings: Record<string, any>) => Promise<boolean>;
};

export const PositioningSection: React.FC<Props> = ({
    values,
    onChange,
    isDirty,
    saving,
    gpsRestarting,
    onSave,
    onSaveAndRestartGps,
    onPersistGnssSettings,
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

    const persistCurrentGnssSettings = async () => {
        const partial: Record<string, any> = {};
        for (const key of GNSS_ACTION_SETTINGS_KEYS) {
            if (Object.prototype.hasOwnProperty.call(values, key)) {
                partial[key] = values[key];
            }
        }
        return onPersistGnssSettings(partial);
    };
    // The serial-link baud and the baud persisted into the receiver's flash must
    // match, so the operator only ever sets ONE "Baud". Keep the receiver-side
    // value (gnss_config_baud) in lockstep automatically.
    const handleBaudChange = (v: number) => {
        onChange("gnss_serial_baud", v);
        onChange("gnss_config_baud", v);
    };

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
                        <Col xs={24} sm={14}>
                            <Form.Item
                                label="Signal Profile"
                                tooltip="High-level constellation and signal preset. This is all most users need to touch."
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
                        <Col xs={24} sm={10}>
                            <Form.Item
                                label="Baud"
                                tooltip="Serial speed between the robot and the GPS receiver. Saved to the receiver flash so both sides always match — you only set it once."
                            >
                                <Select
                                    value={values.gnss_serial_baud ?? 921600}
                                    onChange={handleBaudChange}
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
                message="Saved to the GPS receiver flash"
                description={
                    <span>
                        Baud and signal profile are firmware-side settings: on save they are written to the
                        GPS receiver's flash and the receiver briefly reconnects.{" "}
                        <Text strong>460800</Text> is the safe baud for USB serial links;{" "}
                        <Text strong>921600</Text> needs a robust UART or adapter.{" "}
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
                                <Col xs={24} sm={12}>
                                    <Form.Item label="Receiver Profile" tooltip="Low-level receiver command set. Backend translation to receiver-specific commands is still being wired — leave on the default unless you know you need it.">
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
                                    <Form.Item label="Position Rate">
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
                            </Row>
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

            {/* The plan/apply/factory-reset receiver tooling is developer-grade —
                keep it for Expert mode only. Basic users save with the page's
                Save button at the bottom, which applies + restarts the receiver. */}
            {expertMode && (
                <GnssReceiverActionsCard
                    isDirty={isDirty}
                    saving={saving}
                    gpsRestarting={gpsRestarting}
                    onSave={onSave}
                    onSaveAndRestartGps={onSaveAndRestartGps}
                    onPersistBeforeAction={persistCurrentGnssSettings}
                    showSaveButtons
                />
            )}
        </div>
    );
};
