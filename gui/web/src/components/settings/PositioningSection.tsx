import {NumberWithAddon} from '../inputs/NumberWithAddon';
import {useAsyncHandler} from "../../hooks/useAsyncHandler";
import React, { useState } from "react";
import { Alert, App, Button, Card, Col, Form, InputNumber, Row, Select, Space, Switch, Typography } from "antd";
import { GlobalOutlined, SettingOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { useApi } from "../../hooks/useApi.ts";
import { useGnssStatus } from "../../hooks/useGnssStatus.ts";
import { deriveGpsStatus, gnssReceiverLabel } from "../../utils/gpsStatus.ts";
import { GnssLiveStatusSummaryCard } from "../gnss/GnssLiveStatusSummaryCard.tsx";
import {
    type GnssSelectFieldOption,
    GNSS_BAUD_OPTIONS,
    pickGnssActionSettings,
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
import { GnssReceiverActionsCard } from "./GnssReceiverActionsCard.tsx";
import { GnssSerialDeviceConfigField } from "./GnssSerialDeviceConfigField.tsx";
import type {SettingsValues} from "../../types/settings.generated";
import {isJsonObject} from "../../utils/json";
import {requestErrorMessage} from "../../utils/errors";

const { Text, Paragraph } = Typography;

type Props = {
    values: SettingsValues;
    onChange: (key: string, value: string | number | null) => void;
    isDirty: boolean;
    saving: boolean;
    gpsRestarting: boolean;
    onSave: () => void | Promise<void>;
    onSaveAndRestartGps: () => void | Promise<void>;
    onPersistGnssSettings: (settings: SettingsValues) => Promise<boolean>;
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
    const asyncHandler = useAsyncHandler();
    const { t } = useTranslation();
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
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
            if (!res.ok || res.error) throw new Error(requestErrorMessage(res) ?? t("settingsPositioning.datumGpsFailed"));
            const raw: unknown = res.data;
            if (!isJsonObject(raw) || typeof raw.message !== 'string' || 'error' in raw
                || ('success' in raw && raw.success !== true)) throw new Error(t("settingsPositioning.datumGpsFailed"));
            const parts = raw.message.split(',');
            // The producer sends decimal latitude/longitude, not arbitrary parseFloat prefixes.
            const decimal = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/;
            if (parts.length !== 2 || !parts.every(part => decimal.test(part.trim()))
                || parts.some(part => Number(part) === 0 && /[1-9]/.test(part.split(/[eE]/)[0]))) throw new Error(t("settingsPositioning.datumGpsFailed"));
            const lat = Number(parts[0]);
            const lon = Number(parts[1]);
            if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) throw new Error(t("settingsPositioning.datumGpsFailed"));
            onChange("datum_lat", lat);
            onChange("datum_lon", lon);
            notification.success({ message: t("settingsPositioning.datumSetFromGps") });
        } catch (e) {
            notification.error({ message: t("settingsPositioning.datumGpsFailed"), description: requestErrorMessage(e) ?? t("diagnosticsPage.unknownError") });
        } finally {
            setDatumLoading(false);
        }
    };

    const persistCurrentGnssSettings = async () => {
        const partial: SettingsValues = pickGnssActionSettings(values);
        return onPersistGnssSettings(partial);
    };
    const handleBaudChange = (v: number) => {
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
                                <Form.Item htmlFor="setting-datum_lat" data-setting-key="datum_lat" label={t("settingsPositioning.latitude")}>
                                    <InputNumber aria-label={t("settingsPositioning.latitude")} id="setting-datum_lat"
                                        value={values.datum_lat}
                                        onChange={(value) => onChange("datum_lat", value)}
                                        step={0.000000001}
                                        precision={9}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item htmlFor="setting-datum_lon" data-setting-key="datum_lon" label={t("settingsPositioning.longitude")}>
                                    <InputNumber aria-label={t("settingsPositioning.longitude")} id="setting-datum_lon"
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
                                    onClick={asyncHandler(setDatumFromGps)}
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
                        <Switch aria-label={t("onboardingPage.expertMode")} size="small" checked={expertMode} onChange={setExpertMode} />
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
                                <Select<string, GnssSelectFieldOption>
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
                            <Form.Item htmlFor="setting-gnss_config_baud" data-setting-key="gnss_config_baud"
                                label={t("settingsPositioning.baudLabel")}
                                tooltip={t("settingsPositioning.baudTooltip")}
                            >
                                <Select<number> id="setting-gnss_config_baud"
                                    value={values.gnss_config_baud ?? 921600}
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
                                        <Select<string>
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
                                    <Form.Item htmlFor="setting-gnss_profile_rate_hz" data-setting-key="gnss_profile_rate_hz" label={t("settingsPositioning.positionRateLabel")}>
                                        <Select<number> id="setting-gnss_profile_rate_hz"
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
                                    <Form.Item htmlFor="setting-gnss_receiver_family" data-setting-key="gnss_receiver_family" label={t("settingsPositioning.receiverFamilyLabel")}>
                                        <Select<string> id="setting-gnss_receiver_family"
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
                                <Col xs={12} sm={6}>
                                    <Form.Item htmlFor="setting-gps_wait_after_undock_sec" data-setting-key="gps_wait_after_undock_sec" label={t("settingsPositioning.rtkWaitAfterUndockLabel")}>
                                        <NumberWithAddon aria-label={t("settingsPositioning.rtkWaitAfterUndockLabel") + ", s"} id="setting-gps_wait_after_undock_sec"
                                            value={values.gps_wait_after_undock_sec}
                                            onChange={(value) => onChange("gps_wait_after_undock_sec", value)}
                                            min={0}
                                            step={1}
                                            style={{ width: "100%" }}
                                            after="s"
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={12} sm={6}>
                                    <Form.Item htmlFor="setting-gps_timeout_sec" data-setting-key="gps_timeout_sec" label={t("settingsPositioning.gpsTimeoutLabel")}>
                                        <NumberWithAddon aria-label={t("settingsPositioning.gpsTimeoutLabel") + ", s"} id="setting-gps_timeout_sec"
                                            value={values.gps_timeout_sec}
                                            onChange={(value) => onChange("gps_timeout_sec", value)}
                                            min={1}
                                            step={1}
                                            style={{ width: "100%" }}
                                            after="s"
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
                    manualCurrentBaud={values.gnss_execution_baud}
                    onManualCurrentBaudChange={(value) => onChange("gnss_execution_baud", value)}
                    showSaveButtons
                />
            )}
        </div>
    );
};
