import React, { useState } from "react";
import { Alert, App, Button, Card, Col, Form, InputNumber, Row, Select, Space, Switch, Typography } from "antd";
import { GlobalOutlined, SettingOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useApi } from "../../hooks/useApi.ts";
import { useGnssStatus } from "../../hooks/useGnssStatus.ts";
import { deriveGpsStatus, gnssReceiverLabel } from "../../utils/gpsStatus.ts";
import { GnssLiveStatusSummaryCard } from "../gnss/GnssLiveStatusSummaryCard.tsx";
import {
    GNSS_BAUD_OPTIONS,
    GNSS_ACTION_SETTINGS_KEYS,
    GNSS_EXECUTION_BAUD_OPTIONS,
    GNSS_PROFILE_OPTIONS,
    GNSS_PROFILE_RATE_OPTIONS,
    GNSS_RECEIVER_FAMILY_OPTIONS,
    GNSS_SIGNAL_PROFILE_OPTIONS,
    GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT,
    normalizeGnssProfile,
    normalizeGnssString,
    normalizeGnssSignalProfile,
} from "./gnssConfig.ts";
import { GnssSignalProfileHelp } from "./GnssSignalProfileHelp.tsx";
import { UniversalGnssAdvancedSettings } from "./UniversalGnssAdvancedSettings.tsx";
import { GnssReceiverActionsCard } from "./GnssReceiverActionsCard.tsx";
import { GnssSerialDeviceConfigField } from "./GnssSerialDeviceConfigField.tsx";

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
    const { t } = useTranslation();
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const selectedSignalProfile = normalizeGnssSignalProfile(values.gnss_signal_profile);
    const selectedExecutionBaud = (() => {
        const value = normalizeGnssString(values.gnss_execution_baud).toLowerCase();
        return value === "" || value === "auto" ? "auto" : normalizeGnssString(values.gnss_execution_baud);
    })();
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
                notification.success({ message: t("settingsPositioning.datumSetFromGps") });
            }
        } catch (e: any) {
            notification.error({ message: t("settingsPositioning.datumGpsFailed"), description: e.message });
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
    // The sidecar runtime baud and the target receiver config baud should stay
    // aligned for normal operation. The separate execution/probing baud lives
    // in expert mode and is only used by the one-shot configurator flow.
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
                            {t("settingsPositioning.mapOriginTitle")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsPositioning.mapOriginDescription")}
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsPositioning.latitude")}>
                                    <InputNumber
                                        value={values.datum_lat}
                                        onChange={(value) => onChange("datum_lat", value)}
                                        step={0.000000001}
                                        precision={9}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsPositioning.longitude")}>
                                    <InputNumber
                                        value={values.datum_lon}
                                        onChange={(value) => onChange("datum_lon", value)}
                                        step={0.000000001}
                                        precision={9}
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
                                    {t("settingsPositioning.useCurrentGpsPosition")}
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
                message={t("settingsPositioning.detectedReceiver", { receiver: detectedReceiver })}
                description={t("settingsPositioning.liveGnssStatus", { status: gpsStatus.label })}
            />

            <Card
                size="small"
                title={t("settingsPositioning.gnssProfilesTitle")}
                extra={(
                    <Space size="small">
                        <Text type="secondary" style={{ fontSize: 12 }}>{t("settingsPositioning.expertMode")}</Text>
                        <Switch size="small" checked={expertMode} onChange={setExpertMode} />
                    </Space>
                )}
                style={{ marginBottom: 16 }}
            >
                <Paragraph type="secondary" style={{ marginTop: 0 }}>
                    {t("settingsPositioning.gnssProfilesDescription")}
                </Paragraph>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={24} sm={14}>
                            <Form.Item
                                label={t("settingsPositioning.signalProfileLabel")}
                                tooltip={t("settingsPositioning.signalProfileTooltip")}
                                extra={<GnssSignalProfileHelp selectedProfile={selectedSignalProfile} />}
                            >
                                <Select
                                    value={selectedSignalProfile}
                                    onChange={(value) => onChange("gnss_signal_profile", value)}
                                    options={GNSS_SIGNAL_PROFILE_OPTIONS.map((option) => ({
                                        value: option.value,
                                        label: t(option.label),
                                        description: t(option.description),
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
                                label={t("settingsPositioning.baudLabel")}
                                tooltip={t("settingsPositioning.baudTooltip")}
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
                message={t("settingsPositioning.savedToFlashTitle")}
                description={
                    <span>
                        {t("settingsPositioning.savedToFlashIntro")}{" "}
                        <Text strong>460800</Text> {t("settingsPositioning.savedToFlashSafeBaud")}{" "}
                        <Text strong>921600</Text> {t("settingsPositioning.savedToFlashFastBaud")}{" "}
                        {selectedSignalProfile === "custom" && t(GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT)}
                    </span>
                }
            />

            <GnssLiveStatusSummaryCard
                gnssStatus={gnssStatus}
                selectedReceiverFamily={values.gnss_receiver_family}
            />

            {expertMode && (
                <>
                    <Card size="small" title={<Space><SettingOutlined /> {t("settingsPositioning.expertGnssTitle")}</Space>} style={{ marginBottom: 16 }}>
                        <Paragraph type="secondary" style={{ marginTop: 0 }}>
                            {t("settingsPositioning.expertGnssDescription")}
                        </Paragraph>
                        <Form layout="vertical" size="small">
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={12}>
                                    <Form.Item label={t("settingsPositioning.receiverProfileLabel")} tooltip={t("settingsPositioning.receiverProfileTooltip")}>
                                        <Select
                                            value={normalizeGnssProfile(values.gnss_profile)}
                                            onChange={(value) => onChange("gnss_profile", value)}
                                            options={GNSS_PROFILE_OPTIONS.map((option) => ({
                                                value: option.value,
                                                label: t(option.label),
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item label={t("settingsPositioning.positionRateLabel")}>
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
                                    <Form.Item label={t("settingsPositioning.receiverFamilyLabel")}>
                                        <Select
                                            value={values.gnss_receiver_family ?? "auto"}
                                            onChange={(value) => onChange("gnss_receiver_family", value)}
                                            options={GNSS_RECEIVER_FAMILY_OPTIONS.map((option) => ({
                                                value: option.value,
                                                label: t(option.label),
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={14}>
                                    <GnssSerialDeviceConfigField
                                        value={values.gnss_serial_device}
                                        onChange={(value) => onChange("gnss_serial_device", value)}
                                        selectedReceiverFamily={values.gnss_receiver_family}
                                        selectedReceiverModel={values.gnss_receiver_model}
                                        gnssStatus={gnssStatus}
                                    />
                                </Col>
                            </Row>
                            <Row gutter={[16, 0]}>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label={t("settingsPositioning.executionBaudLabel")}
                                        tooltip={t("settingsPositioning.executionBaudTooltip")}
                                        extra={t("settingsPositioning.executionBaudHelpText")}
                                    >
                                        <Select
                                            value={selectedExecutionBaud}
                                            onChange={(value) => onChange("gnss_execution_baud", value)}
                                            options={GNSS_EXECUTION_BAUD_OPTIONS.map((option) => ({
                                                value: option.value,
                                                label: option.value === "auto" ? t(option.label) : option.label,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                            <Row gutter={[16, 0]}>
                                <Col xs={12} sm={6}>
                                    <Form.Item label={t("settingsPositioning.rtkWaitAfterUndockLabel")}>
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
                                    <Form.Item label={t("settingsPositioning.gpsTimeoutLabel")}>
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
                            message={t("settingsPositioning.expertScopeTitle")}
                            description={t("settingsPositioning.expertScopeDescription")}
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
                Save button at the bottom, which persists the settings and
                restarts the receiver container to pick up transport/NTRIP
                changes. The signal profile is a receiver-flash setting: it only
                reaches the receiver via Plan & Apply here, not via a plain
                Save/restart. */}
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
