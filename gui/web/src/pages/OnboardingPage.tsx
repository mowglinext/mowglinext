import React, { useCallback, useEffect, useRef, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useTranslation } from "react-i18next";
import {
    App,
    Button, Card, Col, Row, Steps, Typography, Select, Space, Alert,
    Input, InputNumber, Switch, Form, Divider, Tag, Result,
} from "antd";
import {
    RocketOutlined, SettingOutlined, GlobalOutlined,
    AimOutlined, ThunderboltOutlined, CheckCircleOutlined,
    ArrowLeftOutlined, ArrowRightOutlined, SaveOutlined,
    EnvironmentOutlined, WifiOutlined,
} from "@ant-design/icons";
import { useThemeMode } from "../theme/ThemeContext.tsx";
import { useIsMobile } from "../hooks/useIsMobile";
import { useSettingsSchema } from "../hooks/useSettingsSchema.ts";
import { useApi } from "../hooks/useApi.ts";
import { useGnssStatus } from "../hooks/useGnssStatus.ts";
import { useCalibrationStatus } from "../hooks/useCalibrationStatus.ts";
import { useImuYawCalibration } from "../hooks/useImuYawCalibration.ts";
import { GnssStatusConstants } from "../types/ros.ts";
import { CompassOutlined } from "@ant-design/icons";
import { deriveGpsStatus, gnssReceiverLabel } from "../utils/gpsStatus.ts";
import { RobotComponentEditor } from "../components/RobotComponentEditor.tsx";
import { FlashBoardComponent } from "../components/FlashBoardComponent.tsx";
import { MOWER_MODELS } from "../constants/mowerModels.ts";
import {
    restartRos2,
    restartGui,
    restartGps,
    GPS_RESTART_KEYS,
} from "../utils/containers.ts";
import { useContainerRestart } from "../hooks/useContainerRestart.ts";
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
} from "../components/settings/gnssConfig.ts";
import { GnssSignalProfileHelp } from "../components/settings/GnssSignalProfileHelp.tsx";
import { UniversalGnssAdvancedSettings } from "../components/settings/UniversalGnssAdvancedSettings.tsx";
import { GnssReceiverActionsCard } from "../components/settings/GnssReceiverActionsCard.tsx";
import { NtripSection } from "../components/settings/NtripSection.tsx";
import { GnssLiveStatusSummaryCard } from "../components/gnss/GnssLiveStatusSummaryCard.tsx";

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

            <Row gutter={[12, 12]}>
                {MOWER_MODELS.map((model) => {
                    const isSelected = selectedModel === model.value;
                    return (
                        <Col xs={12} sm={8} md={6} key={model.value}>
                            <Card
                                hoverable
                                size="small"
                                onClick={() => handleModelSelect(model.value)}
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
};

const GpsStep: React.FC<GpsStepProps> = ({ values, onChange, gpsRestarting, onPersistGnssSettings }) => {
    const { t } = useTranslation();
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const selectedSignalProfile = normalizeGnssSignalProfile(values.gnss_signal_profile);
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
    // The serial link baud and the baud persisted into the receiver's flash must
    // match, so the operator only ever sets ONE "Baud". We keep the receiver-side
    // value (gnss_config_baud) in lockstep automatically — they should never
    // diverge from the user's point of view.
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
                                    <Form.Item label={t("onboardingPage.serialDeviceLabel")}>
                                        <Input
                                            value={values.gnss_serial_device ?? "/dev/ttyAMA4"}
                                            onChange={(e) => onChange("gnss_serial_device", e.target.value)}
                                            placeholder="/dev/serial/by-id/..."
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
                tooling — basic onboarding doesn't need it because Save & Continue
                already restarts the receiver. Keep it for Expert mode only. */}
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

type DatumStepProps = RobotModelStepProps & { gpsRestarting?: boolean };

const DatumStep: React.FC<DatumStepProps> = ({ values, onChange, gpsRestarting }) => {
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
                            <Form.Item label={t("onboardingPage.latitudeLabel")}>
                                <InputNumber
                                    value={values.datum_lat ?? 0}
                                    onChange={(v) => onChange("datum_lat", v)}
                                    step={0.000000001} precision={9} style={{ width: "100%" }}
                                    placeholder="48.8796"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item label={t("onboardingPage.longitudeLabel")}>
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

const FirmwareStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const [showFlash, setShowFlash] = useState(false);

    if (showFlash) {
        return (
            <Card title={t("onboardingPage.flashFirmware")}>
                <FlashBoardComponent onNext={onNext} />
            </Card>
        );
    }

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

            <Space size="middle">
                <Button type="primary" size="large" onClick={() => setShowFlash(true)}>
                    {t("onboardingPage.flashFirmware")}
                </Button>
                <Button size="large" onClick={onNext}>
                    {t("onboardingPage.skipAlreadyFlashed")}
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

// ── Step 5: Complete ────────────────────────────────────────────────────

const CompleteStep: React.FC = () => {
    const { t } = useTranslation();
    const { colors } = useThemeMode();
    const guiApi = useApi();
    const navigate = useNavigate();
    const [restarting, setRestarting] = useState(false);
    const [error, setError] = useState<string | null>(null);

    // Calibration completeness check — see docs/ONBOARDING_IMPROVEMENTS.md
    // gap analysis. The wizard never gates on these, so a brand-new robot
    // can finish "configured" with no dock pose, no IMU mounting calibration
    // and no magnetometer. Here we surface what is actually missing and
    // deep-link the operator to the Diagnostics page where they can run
    // each calibration without restarting the wizard.
    const { status: calibrationStatus } = useCalibrationStatus();
    const missingCalibrations: string[] = [];
    if (calibrationStatus) {
        if (!calibrationStatus.dock?.present) missingCalibrations.push(t("onboardingPage.missingDockPose"));
        if (!calibrationStatus.imu?.present) missingCalibrations.push(t("onboardingPage.missingImuBias"));
        // Magnetometer is optional — only warn when use_magnetometer is on
        // (no good signal client-side yet, so we just don't flag mag here).
    }

    useEffect(() => {
        // Mark onboarding as completed and restart ROS2 + GUI containers
        (async () => {
            setRestarting(true);
            try {
                // Mark onboarding done in DB so we don't redirect again
                const base = import.meta.env.DEV
                    ? `http://${(import.meta.env.VITE_API_HOST as string | undefined) ?? 'localhost:4006'}`
                    : '';
                await fetch(`${base}/api/settings/status`, { method: 'POST' });

                // Restart ROS2 container first (picks up new mowgli_robot.yaml)
                await restartRos2(guiApi);
                // Then restart GUI container
                await restartGui(guiApi);
            } catch (e: any) {
                setError(e.message);
            } finally {
                setRestarting(false);
            }
        })();
    }, []);

    if (restarting) {
        return (
            <Result
                icon={<RocketOutlined style={{ color: colors.primary }} spin />}
                title={t("onboardingPage.applyingConfigTitle")}
                subTitle={t("onboardingPage.applyingConfigSubtitle")}
            />
        );
    }

    return (
        <Result
            icon={<CheckCircleOutlined style={{ color: colors.primary }} />}
            title={t("onboardingPage.allSetTitle")}
            subTitle={t("onboardingPage.allSetSubtitle")}
            extra={[
                <Button
                    key="map"
                    type="primary"
                    size="large"
                    icon={<EnvironmentOutlined />}
                    onClick={() => navigate("/map")}
                >
                    {t("onboardingPage.drawMowingArea")}
                </Button>,
                <Button
                    key="dashboard"
                    size="large"
                    onClick={() => navigate("/mowglinext")}
                >
                    {t("onboardingPage.goToDashboard")}
                </Button>,
            ]}
        >
            {missingCalibrations.length > 0 && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("onboardingPage.calibPendingTitle")}
                    description={
                        <>
                            <Text>
                                {t("onboardingPage.calibPendingPrefix")}{" "}
                                <Text strong>{missingCalibrations.join(t("onboardingPage.calibPendingJoin"))}</Text>{" "}
                                {t("onboardingPage.calibPendingSuffix", { count: missingCalibrations.length })}
                            </Text>
                            <br />
                            <Button
                                type="link"
                                style={{ paddingLeft: 0 }}
                                onClick={() => navigate("/diagnostics")}
                            >
                                {t("onboardingPage.openDiagnosticsRunCalibrations")}
                            </Button>
                        </>
                    }
                    style={{ maxWidth: 540, margin: "0 auto 12px", textAlign: "left" }}
                />
            )}
            {error && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("onboardingPage.restartFailedTitle")}
                    description={`${error}. ${t("onboardingPage.restartFailedDesc")}`}
                    style={{ maxWidth: 500, margin: "0 auto" }}
                />
            )}
        </Result>
    );
};

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
    const isMobile = useIsMobile();
    const { values: savedValues, saveValues, savePartialValues, loading } = useSettingsSchema();
    const guiApi = useApi();
    const [currentStep, setCurrentStep] = useState(0);
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

    const handleChange = useCallback((key: string, value: any) => {
        setLocalValues((prev) => ({ ...prev, [key]: value }));
    }, []);

    // Step indices:
    //   0 Welcome
    //   1 Robot Model
    //   2 Firmware            (custom navigation, no Save & Continue)
    //   3 NTRIP Corrections   (network + base station, set before GPS)
    //   4 GPS Configuration
    //   5 Datum
    //   6 Sensors
    //   7 IMU / Sensor Calibration
    //   8 Complete
    const STEP_FIRMWARE = 2;
    const STEP_NTRIP = 3;
    const STEP_GPS = 4;
    const STEP_DATUM = 5;
    const STEP_CALIBRATION = 7;
    const STEP_COMPLETE = STEP_TITLES.length - 1;

    const handleNext = useCallback(async () => {
        // Save settings when leaving any config step that mutates settings
        // values: Robot Model (1), NTRIP (3), GPS (4), Datum (5), Sensors (6),
        // Calibration (7). Apply-from-calibration writes through onChange but
        // does not auto-save; this is the one batch save point.
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
    }, [currentStep, localValues, saveValues, guiApi, gpsRestart, STEP_GPS, STEP_CALIBRATION]);

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
            {currentStep === 2 && <FirmwareStep onNext={handleNext} />}
            {currentStep === 3 && <NtripStep values={localValues} onChange={handleChange} />}
            {currentStep === 4 && (
                <GpsStep
                    values={localValues}
                    onChange={handleChange}
                    gpsRestarting={gpsRestarting}
                    onPersistGnssSettings={(settings) => savePartialValues(settings, {
                        silentSuccess: true,
                        errorMessage: t("onboardingPage.persistGnssError"),
                    })}
                />
            )}
            {currentStep === 5 && <DatumStep values={localValues} onChange={handleChange} gpsRestarting={gpsRestarting} />}
            {currentStep === 6 && <SensorStep values={localValues} onChange={handleChange} />}
            {currentStep === 7 && <ImuYawStep values={localValues} onChange={handleChange} />}
            {currentStep === 8 && <CompleteStep />}
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
                        <div style={{ height: 4, borderRadius: 2, background: colors.border, overflow: "hidden" }}>
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
