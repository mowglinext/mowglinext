import React, { useCallback, useEffect, useRef, useState } from "react";
import { useSearchParams } from "react-router-dom";
import { useTranslation } from "react-i18next";
import {
    App,
    Button, Card, Col, Row, Steps, Typography, Select, Space, Alert,
    InputNumber, Switch, Form, Divider, Tag,
} from "antd";
import {
    RocketOutlined, SettingOutlined, GlobalOutlined,
    AimOutlined, ThunderboltOutlined, CheckCircleOutlined,
    ArrowLeftOutlined, ArrowRightOutlined, SaveOutlined,
    EnvironmentOutlined, WifiOutlined, CheckOutlined,
} from "@ant-design/icons";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { useIsMobile } from "../hooks/useIsMobile";
import { useSettingsSchema } from "../hooks/useSettingsSchema.ts";
import { useApi } from "../hooks/useApi.ts";
import { useGnssStatus } from "../hooks/useGnssStatus.ts";
import { useCalibrationStatus } from "../hooks/useCalibrationStatus.ts";
import { useImuYawCalibration } from "../hooks/useImuYawCalibration.ts";
import { useFirmwareStatus } from "../hooks/useFirmwareStatus.ts";
import { GnssStatusConstants } from "../types/ros.ts";
import { CompassOutlined } from "@ant-design/icons";
import { deriveGpsStatus, gnssReceiverLabel } from "../utils/gpsStatus.ts";
import { ReadinessStep } from "../components/onboarding/ReadinessStep.tsx";
import {
    STEP_FIRMWARE, STEP_NTRIP, STEP_GPS, STEP_DATUM,
    STEP_CALIBRATION, STEP_COMPLETE, STEP_COUNT,
} from "../components/onboarding/steps.ts";
import { RobotComponentEditor } from "../components/RobotComponentEditor.tsx";
import { FlashBoardComponent } from "../components/FlashBoardComponent.tsx";
import { MOWER_MODELS } from "../constants/mowerModels.ts";
import {
    restartGps,
    GPS_RESTART_KEYS,
} from "../utils/containers.ts";
import { useContainerRestart } from "../hooks/useContainerRestart.ts";
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
} from "../components/settings/gnssConfig.ts";
import { GnssSignalProfileHelp } from "../components/settings/GnssSignalProfileHelp.tsx";
import { UniversalGnssAdvancedSettings } from "../components/settings/UniversalGnssAdvancedSettings.tsx";
import { GnssReceiverActionsCard } from "../components/settings/GnssReceiverActionsCard.tsx";
import { NtripSection } from "../components/settings/NtripSection.tsx";
import { GnssLiveStatusSummaryCard } from "../components/gnss/GnssLiveStatusSummaryCard.tsx";
import { GnssSerialDeviceConfigField } from "../components/settings/GnssSerialDeviceConfigField.tsx";

const { Title, Text, Paragraph } = Typography;

// ── Step 0: Welcome ─────────────────────────────────────────────────────

const WelcomeStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    return (
        <div style={{ textAlign: "center", maxWidth: 760, margin: "0 auto", padding: "32px 0" }}>
            <div style={{
                width: 96, height: 96, borderRadius: 28,
                background: `linear-gradient(135deg, ${colors.accent}, ${colors.accent}99)`,
                boxShadow: `0 12px 32px ${colors.accent}33, inset 0 0 0 1px ${colors.accent}55`,
                display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 28px",
                color: colors.bgBase,
            }}>
                <RocketOutlined style={{ fontSize: 42 }} />
            </div>
            <Title level={2} className="mn-display" style={{
                marginBottom: 10, letterSpacing: "-0.01em",
                fontSize: 42, fontWeight: 400, lineHeight: 1.05,
            }}>
                {t("onboardingPage.welcomeTitlePrefix")} <em>Mowgli</em>{t("onboardingPage.welcomeTitleSuffix")}
            </Title>
            <Paragraph type="secondary" style={{ fontSize: 16, marginBottom: 36, lineHeight: 1.6 }}>
                {t("onboardingPage.welcomeIntro")}
            </Paragraph>

            <Row gutter={[16, 16]} style={{ textAlign: "left", marginBottom: 32 }}>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <SettingOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>{t("onboardingPage.welcomeCardRobotTitle")}</Text>
                                <br />
                                <Text type="secondary">{t("onboardingPage.welcomeCardRobotDesc")}</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <GlobalOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>{t("onboardingPage.welcomeCardGpsTitle")}</Text>
                                <br />
                                <Text type="secondary">{t("onboardingPage.welcomeCardGpsDesc")}</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <AimOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>{t("onboardingPage.welcomeCardSensorsTitle")}</Text>
                                <br />
                                <Text type="secondary">{t("onboardingPage.welcomeCardSensorsDesc")}</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
            </Row>

            <Button type="primary" size="large" onClick={onNext} icon={<ArrowRightOutlined />}>
                {t("onboardingPage.getStarted")}
            </Button>
        </div>
    );
};

// ── Step 1: Robot Model ─────────────────────────────────────────────────

type RobotModelStepProps = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
};

// MOWER_MODELS imported from constants/mowerModels.ts

const RobotModelStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const selectedModel = values.mower_model || "YardForce500";

    const handleModelSelect = (model: string) => {
        onChange("mower_model", model);
        const preset = MOWER_MODELS.find((m) => m.value === model);
        if (preset?.defaults) {
            for (const [k, v] of Object.entries(preset.defaults)) {
                onChange(k, v);
            }
        }
    };

    // Reflect the fallback default as a real selection on mount: without this a
    // user who never taps a card leaves with mower_model unset (no preset
    // applied) even though the YardForce 500 card looks selected. We persist the
    // default so the highlighted card and the saved value always agree.
    useEffect(() => {
        if (!values.mower_model) handleModelSelect("YardForce500");
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    return (
        <div style={{ maxWidth: 760, margin: "0 auto" }}>
            <Title level={4}>
                <SettingOutlined /> {t("onboardingPage.robotModelTitle")}
            </Title>
            <Paragraph type="secondary">
                {t("onboardingPage.robotModelIntro")}
            </Paragraph>

            <Row gutter={[12, 12]} role="radiogroup" aria-label={t("onboardingPage.robotModelTitle")}>
                {MOWER_MODELS.map((model) => {
                    const isSelected = selectedModel === model.value;
                    return (
                        <Col xs={12} sm={8} md={6} key={model.value}>
                            <Card
                                hoverable
                                size="small"
                                role="radio"
                                tabIndex={0}
                                aria-checked={isSelected}
                                aria-label={t(model.label)}
                                onClick={() => handleModelSelect(model.value)}
                                onKeyDown={(e) => {
                                    if (e.key === "Enter" || e.key === " ") {
                                        e.preventDefault();
                                        handleModelSelect(model.value);
                                    }
                                }}
                                style={{
                                    border: isSelected
                                        ? `2px solid ${colors.primary}`
                                        : `1px solid ${colors.border}`,
                                    background: isSelected ? colors.primaryBg : undefined,
                                    height: "100%",
                                    cursor: "pointer",
                                }}
                            >
                                <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                    <Space>
                                        {/* Checkmark is a non-color-only selected affordance. */}
                                        {isSelected && <CheckOutlined style={{ color: colors.primary }} aria-hidden />}
                                        <Text strong>{t(model.label)}</Text>
                                        {(model as any).tag && (
                                            <Tag color="green">{t((model as any).tag)}</Tag>
                                        )}
                                    </Space>
                                    <Text type="secondary" style={{ fontSize: 12 }}>
                                        {t(model.description)}
                                    </Text>
                                </Space>
                            </Card>
                        </Col>
                    );
                })}
            </Row>

            {selectedModel === "CUSTOM" && (
                <>
                    <Divider />
                    <Alert
                        type="info"
                        showIcon
                        message={t("onboardingPage.customConfigTitle")}
                        description={t("onboardingPage.customConfigDesc")}
                        style={{ marginBottom: 16 }}
                    />
                    <Form layout="vertical">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("onboardingPage.wheelRadiusLabel")} tooltip={t("onboardingPage.wheelRadiusTooltip")}>
                                    <InputNumber
                                        value={values.wheel_radius ?? 0.04475}
                                        onChange={(v) => onChange("wheel_radius", v)}
                                        step={0.001} precision={5} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("onboardingPage.wheelTrackLabel")} tooltip={t("onboardingPage.wheelTrackTooltip")}>
                                    <InputNumber
                                        value={values.wheel_track ?? 0.325}
                                        onChange={(v) => onChange("wheel_track", v)}
                                        step={0.001} precision={3} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("onboardingPage.bladeRadiusLabel")} tooltip={t("onboardingPage.bladeRadiusTooltip")}>
                                    <InputNumber
                                        value={values.blade_radius ?? 0.09}
                                        onChange={(v) => onChange("blade_radius", v)}
                                        step={0.01} precision={3} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </>
            )}
        </div>
    );
};

// ── NTRIP step (correction network + base station, before GPS) ──────────

const NtripStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    const { t } = useTranslation();
    return (
        <div style={{ maxWidth: 760, margin: "0 auto" }}>
            <Title level={4}>
                <WifiOutlined /> {t("onboardingPage.ntripTitle")}
            </Title>
            <Paragraph type="secondary">
                {t("onboardingPage.ntripIntro")}
            </Paragraph>
            <NtripSection values={values} onChange={onChange} />
        </div>
    );
};

// ── GPS Configuration step (receiver only, no NTRIP, no datum) ──────────

type GpsStepProps = RobotModelStepProps & {
    gpsRestarting?: boolean;
    onPersistGnssSettings: (settings: Record<string, any>) => Promise<boolean>;
    onJumpToNtrip: () => void;
};

const GpsStep: React.FC<GpsStepProps> = ({ values, onChange, gpsRestarting, onPersistGnssSettings, onJumpToNtrip }) => {
    const { t } = useTranslation();
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    // A typo'd NTRIP credential otherwise reads as "GPS FIX" forever with no
    // explanation — surface whether RTCM corrections are actually flowing.
    const correctionsActive =
        gnssStatus?.correction_stream_status === GnssStatusConstants.CORRECTION_STREAM_STATUS_ACTIVE;
    const selectedSignalProfile = normalizeGnssSignalProfile(values.gnss_signal_profile);
    const selectedExecutionBaud = (() => {
        const value = normalizeGnssString(values.gnss_execution_baud).toLowerCase();
        return value === "" || value === "auto" ? "auto" : normalizeGnssString(values.gnss_execution_baud);
    })();
    const gnssAlertType: "success" | "warning" | "info" = gpsStatus.fixType === "RTK_FIX"
        ? "success"
        : gpsStatus.fixType === "NO_FIX"
            ? "warning"
            : "info";
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
        <div style={{ maxWidth: 760, margin: "0 auto" }}>
            <Title level={4}>
                <GlobalOutlined /> {t("onboardingPage.gpsTitle")}
            </Title>
            <Paragraph type="secondary">
                {t("onboardingPage.gpsIntro")}
            </Paragraph>

            {gpsRestarting && (
                <Alert
                    type="info"
                    showIcon
                    message={t("onboardingPage.gpsRestartingAlertTitle")}
                    description={t("onboardingPage.gpsRestartingAlertDesc")}
                    style={{ marginBottom: 12 }}
                />
            )}

            <Alert
                type={gnssAlertType}
                showIcon
                message={t("onboardingPage.detectedReceiver", { receiver: detectedReceiver })}
                description={t("onboardingPage.liveGnssStatus", { status: gpsStatus.label })}
                style={{ marginBottom: 12 }}
            />

            <Alert
                type={correctionsActive ? "success" : "warning"}
                showIcon
                message={correctionsActive
                    ? t("onboardingPage.gpsCorrectionsActive")
                    : t("onboardingPage.gpsCorrectionsInactive")}
                action={correctionsActive ? undefined : (
                    <Button type="link" size="small" onClick={onJumpToNtrip}>
                        {t("onboardingPage.readinessCtaFixNtrip")}
                    </Button>
                )}
                style={{ marginBottom: 12 }}
            />

            <Card
                size="small"
                title={<Space><WifiOutlined /> {t("onboardingPage.gnssReceiverCardTitle")}</Space>}
                extra={(
                    <Space size="small">
                        <Text type="secondary" style={{ fontSize: 12 }}>{t("onboardingPage.expertMode")}</Text>
                        <Switch size="small" checked={expertMode} onChange={setExpertMode} />
                    </Space>
                )}
                style={{ marginBottom: 16 }}
            >
                <Paragraph type="secondary" style={{ marginTop: 0 }}>
                    {t("onboardingPage.normalVsExpert")}
                </Paragraph>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={24} sm={14}>
                            <Form.Item
                                label={t("onboardingPage.signalProfileLabel")}
                                tooltip={t("onboardingPage.signalProfileTooltip")}
                                extra={<GnssSignalProfileHelp selectedProfile={selectedSignalProfile} />}
                            >
                                <Select
                                    value={selectedSignalProfile}
                                    onChange={(v) => onChange("gnss_signal_profile", v)}
                                    options={GNSS_SIGNAL_PROFILE_OPTIONS.map((option) => ({
                                        label: t(option.label),
                                        value: option.value,
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
                                label={t("onboardingPage.baudLabel")}
                                tooltip={t("onboardingPage.baudTooltip")}
                            >
                                <Select
                                    value={values.gnss_serial_baud ?? 921600}
                                    onChange={handleBaudChange}
                                    options={GNSS_BAUD_OPTIONS.map((option) => ({
                                        label: option.label,
                                        value: option.value,
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
                style={{ marginBottom: 12 }}
                message={t("onboardingPage.savedToFlashTitle")}
                description={`${t("onboardingPage.savedToFlashDesc")}${selectedSignalProfile === "custom" ? ` ${t(GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT)}` : ""}`}
            />

            {expertMode && (
                <>
                    <Card
                        size="small"
                        title={<Space><SettingOutlined /> {t("onboardingPage.expertGnssSettingsTitle")}</Space>}
                        extra={<Tag color="warning">{t('onboardingPage.previewNotActiveYet')}</Tag>}
                        style={{ marginBottom: 16 }}
                    >
                        <Paragraph type="secondary" style={{ marginTop: 0 }}>
                            {t("onboardingPage.expertGnssSettingsDesc")}
                        </Paragraph>
                        <Form layout="vertical">
                            <Row gutter={16}>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label={<Space size={4}>{t("onboardingPage.receiverProfileLabel")} <Tag color="warning" style={{ marginInlineEnd: 0 }}>{t('onboardingPage.preview')}</Tag></Space>}
                                        tooltip={t("onboardingPage.receiverProfileTooltip")}
                                    >
                                        <Select
                                            value={normalizeGnssProfile(values.gnss_profile)}
                                            onChange={(v) => onChange("gnss_profile", v)}
                                            options={GNSS_PROFILE_OPTIONS.map((option) => ({
                                                label: t(option.label),
                                                value: option.value,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item label={t("onboardingPage.positionRateLabel")}>
                                        <Select
                                            value={values.gnss_profile_rate_hz ?? 5}
                                            onChange={(v) => onChange("gnss_profile_rate_hz", v)}
                                            options={GNSS_PROFILE_RATE_OPTIONS.map((option) => ({
                                                label: option.label,
                                                value: option.value,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                            <Row gutter={16}>
                                <Col xs={24} sm={10}>
                                    <Form.Item label={t("onboardingPage.receiverFamilyLabel")}>
                                        <Select
                                            value={values.gnss_receiver_family ?? "auto"}
                                            onChange={(v) => onChange("gnss_receiver_family", v)}
                                            options={GNSS_RECEIVER_FAMILY_OPTIONS.map((option) => ({
                                                label: t(option.label),
                                                value: option.value,
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
                            <Row gutter={16}>
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
                                                label: option.value === "auto" ? t(option.label) : option.label,
                                                value: option.value,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                            </Row>
                        </Form>
                        <Alert
                            type="warning"
                            showIcon
                            message={t('onboardingPage.previewNotActiveYet')}
                            description={t('onboardingPage.expertFieldsNotWired')}
                        />
                    </Card>

                    <UniversalGnssAdvancedSettings
                        receiverFamily={values.gnss_receiver_family ?? "auto"}
                        values={values}
                        onChange={onChange}
                    />
                </>
            )}

            <GnssLiveStatusSummaryCard
                gnssStatus={gnssStatus}
                selectedReceiverFamily={values.gnss_receiver_family}
            />

            {/* The manual plan/apply/factory-reset/restart panel is developer
                tooling — Save & Continue restarts the receiver container for
                transport/NTRIP changes, but only Plan & Apply writes the signal
                profile into the receiver's flash. Keep it for Expert mode only. */}
            {expertMode && (
                <GnssReceiverActionsCard
                    gpsRestarting={gpsRestarting}
                    onPersistBeforeAction={persistCurrentGnssSettings}
                />
            )}

            <Alert
                type="info"
                showIcon
                message={t("onboardingPage.saveContinueReceiverTitle")}
                description={t("onboardingPage.saveContinueReceiverDesc")}
                style={{ marginTop: 8 }}
            />
        </div>
    );
};

// ── Datum step (split out from GPS configuration) ───────────────────────
//
// Lives right after GPS config — the natural mental order — and BEFORE
// sensors and calibration, which are meaningless without a map origin. The
// RTK fix that GPS kicked off keeps acquiring in the background; SBAS /
// RTK-Float datums silently break every later mow, so the "Use current GPS
// position" button stays gated on GnssStatus.FIX_TYPE_RTK_FIXED — the step
// inherently waits for the receiver to settle before it will let you anchor.

type DatumStepProps = RobotModelStepProps & { gpsRestarting?: boolean; requiredError?: boolean };

const DatumStep: React.FC<DatumStepProps> = ({ values, onChange, gpsRestarting, requiredError }) => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const { notification } = App.useApp();
    const [datumLoading, setDatumLoading] = useState(false);

    const gnssStatus = useGnssStatus();
    const fixType = gnssStatus.fix_type ?? GnssStatusConstants.FIX_TYPE_NO_FIX;
    const isRtkFixed = fixType === GnssStatusConstants.FIX_TYPE_RTK_FIXED;
    const isRtkFloat = fixType === GnssStatusConstants.FIX_TYPE_RTK_FLOAT;
    const isPlainFix = fixType === GnssStatusConstants.FIX_TYPE_GPS_FIX;
    const fixLabel = isRtkFixed ? "RTK FIX" : isRtkFloat ? "RTK FLOAT" : isPlainFix ? "GPS FIX" : t("onboardingPage.noFix");

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
            }
        } catch (e: any) {
            notification.error({
                message: t("onboardingPage.datumSetFailedTitle"),
                description: e.message || t("onboardingPage.datumSetFailedDesc"),
            });
        } finally {
            setDatumLoading(false);
        }
    };

    return (
        <div style={{ maxWidth: 760, margin: "0 auto" }}>
            <Title level={4}>
                <EnvironmentOutlined /> {t("onboardingPage.datumTitle")}
            </Title>
            <Paragraph type="secondary">
                {t("onboardingPage.datumIntro")}
            </Paragraph>

            <Card size="small" title={<Space><EnvironmentOutlined /> {t("onboardingPage.datumCoordinatesTitle")}</Space>} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    {t("onboardingPage.datumCoordinatesHint")}
                </Paragraph>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={12}>
                            <Form.Item
                                label={t("onboardingPage.latitudeLabel")}
                                validateStatus={requiredError ? "error" : undefined}
                                help={requiredError ? t("onboardingPage.datumRequiredHelp") : undefined}
                            >
                                <InputNumber
                                    value={values.datum_lat ?? 0}
                                    onChange={(v) => onChange("datum_lat", v)}
                                    step={0.000000001} precision={9} style={{ width: "100%" }}
                                    placeholder="48.8796"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item
                                label={t("onboardingPage.longitudeLabel")}
                                validateStatus={requiredError ? "error" : undefined}
                            >
                                <InputNumber
                                    value={values.datum_lon ?? 0}
                                    onChange={(v) => onChange("datum_lon", v)}
                                    step={0.000000001} precision={9} style={{ width: "100%" }}
                                    placeholder="2.1728"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                    <Button
                        icon={<AimOutlined />}
                        loading={datumLoading || gpsRestarting}
                        onClick={setDatumFromGps}
                        disabled={!isRtkFixed || gpsRestarting}
                        style={{ marginTop: -8 }}
                    >
                        {gpsRestarting
                            ? t("onboardingPage.gpsRestartingShort")
                            : isRtkFixed
                                ? t("onboardingPage.useCurrentGpsPosition")
                                : t("onboardingPage.useCurrentGpsPositionWaiting")}
                    </Button>
                    {gpsRestarting && (
                        <Alert
                            type="info"
                            showIcon
                            message={t("onboardingPage.gpsContainerRestartingTitle")}
                            description={t("onboardingPage.gpsContainerRestartingDesc")}
                            style={{ marginTop: 12 }}
                        />
                    )}
                    {!isRtkFixed && !gpsRestarting && (
                        <Alert
                            type="warning"
                            showIcon
                            message={t("onboardingPage.currentGpsQuality", { fix: fixLabel })}
                            description={
                                isRtkFloat
                                    ? t("onboardingPage.rtkFloatWarning")
                                    : t("onboardingPage.noRtkFixWarning")
                            }
                            style={{ marginTop: 12 }}
                        />
                    )}
                </Form>
            </Card>
        </div>
    );
};

// ── Step 3: Sensor Placement ────────────────────────────────────────────

const SensorStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => {
    const { t } = useTranslation();
    return (
        <div style={{ maxWidth: 760, margin: "0 auto" }}>
            <Title level={4}>
                <AimOutlined /> {t("onboardingPage.sensorPlacementTitle")}
            </Title>
            <Paragraph type="secondary" style={{ marginBottom: 16 }}>
                {t("onboardingPage.sensorPlacementIntro")}
            </Paragraph>
            <RobotComponentEditor values={values} onChange={onChange} />
        </div>
    );
};

// ── Step 4: IMU Yaw Calibration ─────────────────────────────────────────

// A wizard step that walks the operator through the IMU mounting yaw
// auto-calibration — previously buried as a tooltip-only compass icon
// inside the Sensors step's RobotComponentEditor. Without this step, a
// brand-new robot can finish onboarding with imu_yaw=0 and silently
// drift in odom.
//
// The step also captures dock_pose_x/y/yaw as a side effect when the
// robot starts the calibration on the dock (the ROS service's dock
// pre-phase writes it directly into mowgli_robot.yaml — see
// CalibrateImuYaw.srv response fields).

const ImuYawStep: React.FC<RobotModelStepProps> = ({values, onChange}) => {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const {status: calibrationStatus, refresh: refreshCalibrationStatus} = useCalibrationStatus();
    const {
        calibRunning,
        calibResult,
        resetCalibration,
        startCalibration,
        applyCalibration,
    } = useImuYawCalibration({
        onApplyValue: (key, value) => {
            onChange(key, value);
            // Dock pose is persisted server-side into mowgli_robot.yaml; refresh
            // the status panel so the operator sees the updated dock pose card.
            refreshCalibrationStatus();
        },
        currentImuYawRad: values.imu_yaw,
    });

    const dockPresent = !!calibrationStatus?.dock?.present;
    const imuPresent = !!calibrationStatus?.imu?.present;
    const currentImuYawDeg = (values.imu_yaw ?? 0) * 180 / Math.PI;

    return (
        <div style={{maxWidth: 760, margin: "0 auto"}}>
            <Title level={4}>
                <CompassOutlined/> {t("onboardingPage.calibrationTitle")}
            </Title>
            <Paragraph type="secondary" style={{marginBottom: 16}}>
                {t("onboardingPage.calibrationIntro")}
            </Paragraph>

            <Card size="small" style={{marginBottom: 16}}>
                <Paragraph strong style={{marginBottom: 8}}>{t("onboardingPage.measuresHeading")}</Paragraph>
                <ul style={{paddingLeft: 20, marginBottom: 0, color: colors.textSecondary, fontSize: 13}}>
                    <li><Text strong>{t("onboardingPage.measureDockPoseLabel")}</Text> {t("onboardingPage.measureDockPoseBody")} <Text code>mowgli_robot.yaml</Text>.</li>
                    <li><Text strong>{t("onboardingPage.measureImuYawLabel")}</Text>{t("onboardingPage.measureImuYawBody")}</li>
                    <li><Text strong>{t("onboardingPage.measurePitchRollLabel")}</Text>{t("onboardingPage.measurePitchRollBody")}</li>
                    <li><Text strong>{t("onboardingPage.measureMagLabel")}</Text> {t("onboardingPage.measureMagBody")}</li>
                </ul>
            </Card>

            <Card size="small" style={{marginBottom: 16}}>
                <Paragraph strong style={{marginBottom: 8}}>{t("onboardingPage.preflightHeading")}</Paragraph>
                <ul style={{paddingLeft: 20, marginBottom: 0, color: colors.textSecondary, fontSize: 13}}>
                    <li>{t("onboardingPage.preflightDockPrefix")} <Text strong>{t("onboardingPage.preflightDockEmphasis")}</Text> {t("onboardingPage.preflightDockSuffix")}</li>
                    <li>{t("onboardingPage.preflightNtrip")}</li>
                    <li>{t("onboardingPage.preflightEmergency")}</li>
                    <li>{t("onboardingPage.preflightDontTouch")}</li>
                </ul>
            </Card>

            <Card size="small" style={{marginBottom: 16}}>
                <Row gutter={[16, 8]}>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>{t("onboardingPage.currentImuYaw")}</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>{currentImuYawDeg.toFixed(2)}°</div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>{t("onboardingPage.dockPose")}</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {dockPresent ? <Tag color="success">{t("onboardingPage.present")}</Tag> : <Tag color="warning">{t("onboardingPage.missing")}</Tag>}
                        </div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>{t("onboardingPage.imuBias")}</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {imuPresent ? <Tag color="success">{t("onboardingPage.present")}</Tag> : <Tag color="warning">{t("onboardingPage.missing")}</Tag>}
                        </div>
                    </Col>
                </Row>
            </Card>

            <div style={{textAlign: "center", marginBottom: 16}}>
                <Button
                    type="primary"
                    size="large"
                    icon={<CompassOutlined/>}
                    onClick={startCalibration}
                    loading={calibRunning}
                    disabled={calibRunning}
                >
                    {calibResult ? t("onboardingPage.rerunCalibration") : t("onboardingPage.startImuYawCalibration")}
                </Button>
            </div>

            {calibRunning && (
                <Alert
                    type="info"
                    showIcon
                    message={t("onboardingPage.calibRunningTitle")}
                    description={t("onboardingPage.calibRunningDesc")}
                    style={{marginBottom: 16}}
                />
            )}

            {calibResult && calibResult.success && (
                <Alert
                    type="success"
                    showIcon
                    message={`imu_yaw = ${calibResult.imu_yaw_deg.toFixed(2)}° (σ ±${calibResult.std_dev_deg.toFixed(2)}°)`}
                    description={
                        <>
                            <div>{t("onboardingPage.fromValidSamples", { count: calibResult.samples_used })}</div>
                            {calibResult.dock_valid && (
                                <div>
                                    {t("onboardingPage.dockPoseUpdated", {
                                        yaw: calibResult.dock_pose_yaw_deg?.toFixed(2),
                                        sigma: calibResult.dock_yaw_sigma_deg?.toFixed(2),
                                        displacement: calibResult.dock_undock_displacement_m?.toFixed(2) ?? "?",
                                    })}
                                </div>
                            )}
                            <div style={{marginTop: 12}}>
                                <Space>
                                    <Button type="primary" onClick={applyCalibration}>{t("onboardingPage.applyToSettings")}</Button>
                                    <Button onClick={resetCalibration}>{t("onboardingPage.discard")}</Button>
                                </Space>
                            </div>
                        </>
                    }
                    style={{marginBottom: 16}}
                />
            )}

            {calibResult && !calibResult.success && (
                <Alert
                    type="error"
                    showIcon
                    message={t("onboardingPage.calibFailedTitle")}
                    description={
                        <>
                            <div>{calibResult.message}</div>
                            <div style={{marginTop: 8, color: colors.textSecondary}}>
                                {t("onboardingPage.calibFailedHint")}
                            </div>
                            <div style={{marginTop: 12}}>
                                <Button onClick={resetCalibration}>{t("onboardingPage.reset")}</Button>
                            </div>
                        </>
                    }
                    style={{marginBottom: 16}}
                />
            )}

            <Alert
                type="info"
                showIcon
                message={t("onboardingPage.skipStepTitle")}
                description={t("onboardingPage.skipStepDesc")}
                style={{maxWidth: 500, margin: "0 auto"}}
            />
        </div>
    );
};

// ── Step 5: Firmware ────────────────────────────────────────────────────

const FirmwareStep: React.FC<{ onNext: () => void; autoFlash?: boolean }> = ({ onNext, autoFlash = false }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    // `autoFlash` (set by the dashboard deep-link when firmware is incompatible)
    // skips the intro screen and drops the operator straight onto the prebuilt
    // flash form — the one-click path from the "reflash to mow" warning.
    const [showFlash, setShowFlash] = useState(autoFlash);
    const { firmwareCompatible, firmwareVersion } = useFirmwareStatus();

    if (showFlash) {
        return (
            <Card title={t("onboardingPage.flashFirmware")}>
                <FlashBoardComponent onNext={onNext} />
            </Card>
        );
    }

    // Live handshake readback so the operator learns compatibility here rather
    // than six steps later on the dashboard. `null` = board hasn't reported yet.
    const firmwareAlertType: "success" | "error" | "info" =
        firmwareCompatible === true ? "success" : firmwareCompatible === false ? "error" : "info";
    const firmwareAlertMessage =
        firmwareCompatible === true
            ? t("onboardingPage.firmwareCompatibleLive", { version: firmwareVersion || "?" })
            : firmwareCompatible === false
                ? t("onboardingPage.firmwareIncompatibleLive", { version: firmwareVersion || "?" })
                : t("onboardingPage.firmwareWaitingHandshake");

    return (
        <div style={{ maxWidth: 760, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
            <div style={{
                width: 64, height: 64, borderRadius: "50%",
                background: colors.primaryBg, display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 16px",
            }}>
                <ThunderboltOutlined style={{ fontSize: 28, color: colors.primary }} />
            </div>
            <Title level={4}>{t("onboardingPage.firmwareTitle")}</Title>
            <Paragraph type="secondary" style={{ marginBottom: 24 }}>
                {t("onboardingPage.firmwareIntro")}
            </Paragraph>

            <Alert
                type={firmwareAlertType}
                showIcon
                message={firmwareAlertMessage}
                style={{ marginBottom: 24, textAlign: "left" }}
            />

            <Space size="middle">
                <Button type="primary" size="large" onClick={() => setShowFlash(true)}>
                    {t("onboardingPage.flashFirmware")}
                </Button>
                <Button size="large" onClick={onNext}>
                    {firmwareCompatible === true
                        ? t("onboardingPage.skipAlreadyFlashed")
                        : t("onboardingPage.skipWithoutVerifying")}
                </Button>
            </Space>

            <Alert
                type="warning"
                showIcon
                message={t("onboardingPage.flashWarningTitle")}
                description={t("onboardingPage.flashWarningDesc")}
                style={{ marginTop: 24, textAlign: "left" }}
            />
        </div>
    );
};

// ── Step 8: Complete ────────────────────────────────────────────────────
//
// The final "Complete" step is now the readiness gate — see
// components/onboarding/ReadinessStep.tsx. It runs a live checklist, blocks
// "Finish & apply" until every REQUIRED check passes (with an explicit
// "Finish anyway" escape hatch), and only THEN commits onboarding
// (POST settings/status → restartRos2 → restartGui). Committing no longer
// fires from a mount effect.

// ── Main Setup Wizard ───────────────────────────────────────────────────

// Step order rationale (positioning is configured front-to-back, so each step
// builds on the one before it):
//   1. Welcome
//   2. Robot Model — prefills hardware params; needs to come before firmware
//      so the operator knows which board / variant they are flashing.
//   3. Firmware — flashing happens before any configuration depends on a
//      working motherboard / GPS receiver.
//   4. GPS — receiver protocol, port, NTRIP corrections. Saving this step
//      restarts the GPS daemon so an RTK fix can start acquiring.
//   5. Datum — anchor the map origin to the current RTK-Fixed position. Comes
//      right after GPS (the natural mental order) and BEFORE docking/calibration,
//      since those are meaningless without a map origin. The step gates the
//      capture button on RTK-Fixed, so it inherently waits for the receiver to
//      settle.
//   6. Sensors — sensor placement on the chassis.
//   7. Calibration — drives the robot to learn IMU mounting, pitch/roll, mag,
//      and (if on the dock) the dock pose.
//   8. Complete

const STEP_ICONS = [
    <RocketOutlined />,
    <SettingOutlined />,
    <ThunderboltOutlined />,
    <WifiOutlined />,
    <GlobalOutlined />,
    <EnvironmentOutlined />,
    <AimOutlined />,
    <CompassOutlined />,
    <CheckCircleOutlined />,
];

// i18n key strings resolved with t() at render time (see stepItems / mobile
// header). NTRIP / GPS / Datum stay technical tokens via their en values.
const STEP_TITLES = [
    "onboardingPage.stepWelcome",
    "onboardingPage.stepRobotModel",
    "onboardingPage.stepFirmware",
    "onboardingPage.stepNtrip",
    "onboardingPage.stepGps",
    "onboardingPage.stepDatum",
    "onboardingPage.stepSensors",
    "onboardingPage.stepCalibration",
    "onboardingPage.stepComplete",
];

const OnboardingWizard: React.FC = () => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const { notification } = App.useApp();
    const isMobile = useIsMobile();
    const { values: savedValues, saveValues, savePartialValues, loading } = useSettingsSchema();
    const guiApi = useApi();
    // Deep-link support: the dashboard's "Flash firmware" CTA (shown when the
    // firmware handshake reports incompatible) lands here with
    // `?step=firmware&flash=1`, opening the wizard directly on the firmware step
    // with the flash panel already expanded — no hunting for the flash screen.
    const [searchParams] = useSearchParams();
    const deepLinkFirmware = searchParams.get("step") === "firmware";
    const autoFlash = deepLinkFirmware && searchParams.get("flash") === "1";
    const [currentStep, setCurrentStep] = useState(deepLinkFirmware ? STEP_FIRMWARE : 0);
    const [localValues, setLocalValues] = useState<Record<string, any>>({});
    const [saving, setSaving] = useState(false);
    const gpsRestart = useContainerRestart({
        pendingLabel: t('onboardingPage.gpsRestarting'),
        successMessage: t('onboardingPage.gpsRestartedWaitRtk'),
        errorMessage: t('onboardingPage.gpsRestartFailed'),
        skipReadinessProbe: true,
    });
    const gpsRestarting = gpsRestart.pending;
    // Snapshot of saved GPS-related values at the time the user enters
    // step 2 — used to detect whether the GPS container needs an auto-
    // restart when leaving the step.
    const gpsSnapshotRef = useRef<Record<string, any> | null>(null);

    useEffect(() => {
        if (savedValues) {
            setLocalValues(savedValues);
        }
    }, [savedValues]);

    // Snapshot GPS-affecting fields whenever the user enters the GPS step,
    // so we can compare on Next and decide whether to auto-restart mowgli-gps.
    useEffect(() => {
        if (currentStep === STEP_GPS) {
            const snap: Record<string, any> = {};
            for (const k of GPS_RESTART_KEYS) snap[k] = localValues[k];
            gpsSnapshotRef.current = snap;
        }
    }, [currentStep]);

    // Set when the operator tries to leave the Datum step with an unset/(0,0)
    // origin — DatumStep reads it to show an inline required-field error.
    const [datumError, setDatumError] = useState(false);

    const handleChange = useCallback((key: string, value: any) => {
        setLocalValues((prev) => ({ ...prev, [key]: value }));
    }, []);

    // Deep-links back into an in-wizard step (used by the readiness CTAs).
    const jumpToStep = useCallback((idx: number) => setCurrentStep(idx), []);

    // Step indices are single-sourced in components/onboarding/steps.ts so the
    // wizard and the readiness gate never drift.
    const handleNext = useCallback(async () => {
        // Save settings when leaving any config step that mutates settings
        // values: Robot Model (1), NTRIP (3), GPS (4), Datum (5), Sensors (6),
        // Calibration (7). Apply-from-calibration writes through onChange but
        // does not auto-save; this is the one batch save point.
        // Datum required-guard: a (0,0) or unset origin silently breaks every
        // later mow, so block leaving the Datum step until it is captured.
        if (currentStep === STEP_DATUM) {
            const lat = localValues.datum_lat;
            const lon = localValues.datum_lon;
            const datumSet =
                Number.isFinite(lat) && lat !== 0 && Number.isFinite(lon) && lon !== 0;
            if (!datumSet) {
                setDatumError(true);
                notification.warning({
                    message: t("onboardingPage.datumRequiredTitle"),
                    description: t("onboardingPage.datumRequiredHelp"),
                });
                return;
            }
            setDatumError(false);
        }
        const isConfigStep =
            currentStep === 1 ||
            (currentStep >= STEP_NTRIP && currentStep <= STEP_CALIBRATION);
        if (isConfigStep) {
            setSaving(true);
            await saveValues(localValues);
            setSaving(false);
        }
        // Leaving the GPS step: if any GPS/NTRIP/serial field actually
        // changed vs the snapshot taken on entry, bounce the GPS container
        // so the new config is applied. Without this the user has to know
        // to click "Restart GPS" before "Set Datum" can ever see RTK Fix.
        if (currentStep === STEP_GPS && gpsSnapshotRef.current) {
            const snap = gpsSnapshotRef.current;
            let changed = false;
            for (const k of GPS_RESTART_KEYS) {
                if (JSON.stringify(snap[k]) !== JSON.stringify(localValues[k])) {
                    changed = true;
                    break;
                }
            }
            if (changed) {
                await gpsRestart.run(() => restartGps(guiApi));
            }
        }
        setCurrentStep((s) => Math.min(s + 1, STEP_TITLES.length - 1));
    }, [currentStep, localValues, saveValues, guiApi, gpsRestart, notification, t]);

    const handlePrev = useCallback(() => {
        setCurrentStep((s) => Math.max(s - 1, 0));
    }, []);

    const isFirstStep = currentStep === 0;
    const isLastStep = currentStep === STEP_COMPLETE;
    const isFirmwareStep = currentStep === STEP_FIRMWARE;

    const stepItems = STEP_TITLES.map((title, i) => ({ title: t(title), icon: STEP_ICONS[i] }));

    const stepContent = (
        <>
            {currentStep === 0 && <WelcomeStep onNext={handleNext} />}
            {currentStep === 1 && <RobotModelStep values={localValues} onChange={handleChange} />}
            {currentStep === 2 && <FirmwareStep onNext={handleNext} autoFlash={autoFlash} />}
            {currentStep === 3 && <NtripStep values={localValues} onChange={handleChange} />}
            {currentStep === 4 && (
                <GpsStep
                    values={localValues}
                    onChange={handleChange}
                    gpsRestarting={gpsRestarting}
                    onJumpToNtrip={() => jumpToStep(STEP_NTRIP)}
                    onPersistGnssSettings={(settings) => savePartialValues(settings, {
                        silentSuccess: true,
                        errorMessage: t("onboardingPage.persistGnssError"),
                    })}
                />
            )}
            {currentStep === 5 && <DatumStep values={localValues} onChange={handleChange} gpsRestarting={gpsRestarting} requiredError={datumError} />}
            {currentStep === 6 && <SensorStep values={localValues} onChange={handleChange} />}
            {currentStep === 7 && <ImuYawStep values={localValues} onChange={handleChange} />}
            {currentStep === 8 && <ReadinessStep values={localValues} onJumpToStep={jumpToStep} />}
        </>
    );

    // Navigation bar (hidden on welcome, complete, and firmware steps).
    const navBar = !isFirstStep && !isLastStep && !isFirmwareStep && (
        <div
            style={{
                position: "fixed",
                bottom: isMobile ? "calc(env(safe-area-inset-bottom, 0px) + 92px)" : 20,
                left: isMobile ? 0 : undefined,
                right: isMobile ? 0 : undefined,
                padding: isMobile ? "8px 12px" : undefined,
                background: isMobile ? colors.bgCard : undefined,
                borderTop: isMobile ? `1px solid ${colors.border}` : undefined,
                zIndex: 50,
            }}
        >
            <Space>
                <Button icon={<ArrowLeftOutlined />} onClick={handlePrev}>
                    {t("onboardingPage.back")}
                </Button>
                <Button
                    type="primary"
                    icon={currentStep === STEP_DATUM ? <SaveOutlined /> : <ArrowRightOutlined />}
                    onClick={handleNext}
                    loading={saving || loading || gpsRestarting}
                >
                    {gpsRestarting
                        ? t("onboardingPage.restartingGps")
                        : currentStep === STEP_DATUM
                            ? t("onboardingPage.saveAndFinish")
                            : t("onboardingPage.next")}
                </Button>
            </Space>
        </div>
    );

    // Mobile: a cramped 8-icon horizontal stepper reads badly, so show a compact
    // "Step N of M · Title" header with a progress bar instead.
    if (isMobile) {
        const pct = Math.round(((currentStep + 1) / STEP_TITLES.length) * 100);
        return (
            <Row gutter={[0, 12]}>
                <Col span={24}>
                    <div style={{ display: "flex", flexDirection: "column", gap: 8, marginBottom: 4 }}>
                        <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between" }}>
                            <Space size={8}>
                                <span style={{ color: colors.accent, display: "inline-flex" }}>{STEP_ICONS[currentStep]}</span>
                                <Text strong style={{ fontSize: 15 }}>{t(STEP_TITLES[currentStep])}</Text>
                            </Space>
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                {t("onboardingPage.stepCounter", { current: currentStep + 1, total: STEP_TITLES.length })}
                            </Text>
                        </div>
                        <div
                            role="progressbar"
                            aria-label={t("onboardingPage.stepCounter", { current: currentStep + 1, total: STEP_COUNT })}
                            aria-valuenow={currentStep + 1}
                            aria-valuemin={1}
                            aria-valuemax={STEP_COUNT}
                            style={{ height: 4, borderRadius: 2, background: colors.border, overflow: "hidden" }}
                        >
                            <div style={{ height: "100%", width: `${pct}%`, background: colors.accent, transition: "width .3s ease" }} />
                        </div>
                    </div>
                </Col>
                <Col span={24} style={{ paddingBottom: isLastStep || isFirmwareStep ? 16 : 80 }}>
                    {stepContent}
                </Col>
                {navBar}
            </Row>
        );
    }

    // Desktop: left vertical stepper rail + scrollable content. The rail gives
    // each of the 8 steps its own labelled row instead of a cramped horizontal
    // strip, which reads far better with this many steps.
    return (
        <Row gutter={[28, 0]} style={{ minHeight: "calc(100vh - 150px)" }}>
            <Col flex="0 0 220px">
                <Steps
                    direction="vertical"
                    current={currentStep}
                    items={stepItems}
                    style={{ position: "sticky", top: 8 }}
                />
            </Col>
            <Col
                flex="1 1 0"
                style={{
                    minWidth: 0,
                    height: "calc(100vh - 150px)",
                    overflowY: "auto",
                    paddingBottom: 80,
                }}
            >
                {stepContent}
            </Col>
            {navBar}
        </Row>
    );
};

export default OnboardingWizard;
