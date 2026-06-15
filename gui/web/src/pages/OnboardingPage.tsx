import React, { useCallback, useEffect, useRef, useState } from "react";
import {
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
import { useDiagnostics } from "../hooks/useDiagnostics.ts";
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
import { UniversalGnssLiveStatusCard } from "../components/settings/UniversalGnssLiveStatusCard.tsx";
import { GnssReceiverActionsCard } from "../components/settings/GnssReceiverActionsCard.tsx";
import { NtripSection } from "../components/settings/NtripSection.tsx";

const { Title, Text, Paragraph } = Typography;

// ── Step 0: Welcome ─────────────────────────────────────────────────────

const WelcomeStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { colors } = useThemeMode();
    return (
        <div style={{ textAlign: "center", maxWidth: 640, margin: "0 auto", padding: "32px 0" }}>
            <div style={{
                width: 96, height: 96, borderRadius: 28,
                background: `linear-gradient(135deg, ${colors.accent}, ${colors.accent}99)`,
                boxShadow: `0 12px 32px ${colors.accent}33, inset 0 0 0 1px ${colors.accent}55`,
                display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 28px",
                color: "#0a1a10",
            }}>
                <RocketOutlined style={{ fontSize: 42 }} />
            </div>
            <Title level={2} className="mn-display" style={{
                marginBottom: 10, letterSpacing: "-0.01em",
                fontSize: 42, fontWeight: 400, lineHeight: 1.05,
            }}>
                Let's meet your <em>Mowgli</em>.
            </Title>
            <Paragraph type="secondary" style={{ fontSize: 16, marginBottom: 36, lineHeight: 1.6 }}>
                A few short steps and you're ready to mow. Nothing here is set in
                stone -- you can always change these settings later from the
                Settings page.
            </Paragraph>

            <Row gutter={[16, 16]} style={{ textAlign: "left", marginBottom: 32 }}>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <SettingOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Choose your robot</Text>
                                <br />
                                <Text type="secondary">Select your mower model and firmware</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <GlobalOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Set up GPS</Text>
                                <br />
                                <Text type="secondary">Configure your position and RTK corrections</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
                <Col span={24}>
                    <Card size="small">
                        <Space>
                            <AimOutlined style={{ color: colors.primary, fontSize: 20 }} />
                            <div>
                                <Text strong>Place your sensors</Text>
                                <br />
                                <Text type="secondary">Visually position LiDAR, IMU, and GPS on the robot</Text>
                            </div>
                        </Space>
                    </Card>
                </Col>
            </Row>

            <Button type="primary" size="large" onClick={onNext} icon={<ArrowRightOutlined />}>
                Get Started
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

    return (
        <div style={{ maxWidth: 800, margin: "0 auto" }}>
            <Title level={4}>
                <SettingOutlined /> Choose Your Robot
            </Title>
            <Paragraph type="secondary">
                Select your mower model. This pre-fills hardware parameters like wheel size, battery voltage, and blade dimensions.
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
                                        <Text strong>{model.label}</Text>
                                        {(model as any).tag && (
                                            <Tag color="green">{(model as any).tag}</Tag>
                                        )}
                                    </Space>
                                    <Text type="secondary" style={{ fontSize: 12 }}>
                                        {model.description}
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
                        message="Custom configuration"
                        description="You can fine-tune all hardware parameters in Settings after completing onboarding."
                        style={{ marginBottom: 16 }}
                    />
                    <Form layout="vertical">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Wheel Radius" tooltip="Drive wheel radius in metres">
                                    <InputNumber
                                        value={values.wheel_radius ?? 0.04475}
                                        onChange={(v) => onChange("wheel_radius", v)}
                                        step={0.001} precision={5} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Wheel Track" tooltip="Centre-to-centre wheel distance">
                                    <InputNumber
                                        value={values.wheel_track ?? 0.325}
                                        onChange={(v) => onChange("wheel_track", v)}
                                        step={0.001} precision={3} style={{ width: "100%" }}
                                        addonAfter="m"
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Blade Radius" tooltip="Mowing blade radius">
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

const NtripStep: React.FC<RobotModelStepProps> = ({ values, onChange }) => (
    <div style={{ maxWidth: 820, margin: "0 auto" }}>
        <Title level={4}>
            <WifiOutlined /> NTRIP Corrections
        </Title>
        <Paragraph type="secondary">
            RTK corrections give centimetre accuracy. Pick a correction network and the nearest base station on the
            map — credentials for the free networks are filled in for you. You'll configure the GPS receiver next.
        </Paragraph>
        <NtripSection values={values} onChange={onChange} />
    </div>
);

// ── GPS Configuration step (receiver only, no NTRIP, no datum) ──────────

type GpsStepProps = RobotModelStepProps & {
    gpsRestarting?: boolean;
    onPersistGnssSettings: (settings: Record<string, any>) => Promise<boolean>;
};

const GpsStep: React.FC<GpsStepProps> = ({ values, onChange, gpsRestarting, onPersistGnssSettings }) => {
    const [expertMode, setExpertMode] = useState(false);
    const gnssStatus = useGnssStatus();
    const { diagnostics } = useDiagnostics();
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
                <GlobalOutlined /> GNSS Configuration
            </Title>
            <Paragraph type="secondary">
                Configure the GNSS receiver with vendor-neutral profiles first, then enable Expert mode only when you need
                receiver-family-specific overrides. The map origin (datum) is set later, once the receiver has had a moment
                to acquire an RTK fix.
            </Paragraph>

            {gpsRestarting && (
                <Alert
                    type="info"
                    showIcon
                    message="GNSS receiver is restarting to apply your serial and NTRIP settings"
                    description="Wait ~10–30 s for RTK Fix to come back before setting the datum. Receiver profile apply, factory reset, and GNSS restart actions are available from the receiver actions panel below."
                    style={{ marginBottom: 12 }}
                />
            )}

            <Alert
                type={gnssAlertType}
                showIcon
                message={`Detected receiver: ${detectedReceiver}`}
                description={`Live GNSS status: ${gpsStatus.label}. The normal onboarding UI stays vendor-neutral; family-specific tuning lives in Expert mode.`}
                style={{ marginBottom: 12 }}
            />

            <Card
                size="small"
                title={<Space><WifiOutlined /> GNSS Receiver</Space>}
                extra={(
                    <Space size="small">
                        <Text type="secondary" style={{ fontSize: 12 }}>Expert mode</Text>
                        <Switch size="small" checked={expertMode} onChange={setExpertMode} />
                    </Space>
                )}
                style={{ marginBottom: 16 }}
            >
                <Paragraph type="secondary" style={{ marginTop: 0 }}>
                    Normal settings are vendor-neutral. Expert settings are receiver-family specific.
                </Paragraph>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={24} sm={14}>
                            <Form.Item
                                label="Signal Profile"
                                tooltip="High-level constellation and signal preset. This is all most users need to touch."
                                extra={<GnssSignalProfileHelp selectedProfile={selectedSignalProfile} />}
                            >
                                <Select
                                    value={selectedSignalProfile}
                                    onChange={(v) => onChange("gnss_signal_profile", v)}
                                    options={GNSS_SIGNAL_PROFILE_OPTIONS.map((option) => ({
                                        label: option.label,
                                        value: option.value,
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
                message="Saved to the GPS receiver flash"
                description={`Baud and signal profile are firmware-side settings: on save they are written to the GPS receiver's flash and the receiver briefly reconnects. 460800 is the safe baud for USB serial links; 921600 needs a robust UART or adapter.${selectedSignalProfile === "custom" ? ` ${GNSS_SIGNAL_PROFILE_CUSTOM_HELP_TEXT}` : ""}`}
            />

            {expertMode && (
                <>
                    <Card size="small" title={<Space><SettingOutlined /> Expert GNSS Settings</Space>} style={{ marginBottom: 16 }}>
                        <Paragraph type="secondary" style={{ marginTop: 0 }}>
                            Receiver-family selection, raw serial wiring, and vendor-specific overrides live here.
                        </Paragraph>
                        <Form layout="vertical">
                            <Row gutter={16}>
                                <Col xs={24} sm={12}>
                                    <Form.Item
                                        label="Receiver Profile"
                                        tooltip="Low-level receiver command set. Backend translation to receiver-specific commands is still being wired — leave on the default unless you know you need it."
                                    >
                                        <Select
                                            value={normalizeGnssProfile(values.gnss_profile)}
                                            onChange={(v) => onChange("gnss_profile", v)}
                                            options={GNSS_PROFILE_OPTIONS.map((option) => ({
                                                label: option.label,
                                                value: option.value,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={12}>
                                    <Form.Item label="Position Rate">
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
                                    <Form.Item label="Receiver Family">
                                        <Select
                                            value={values.gnss_receiver_family ?? "auto"}
                                            onChange={(v) => onChange("gnss_receiver_family", v)}
                                            options={GNSS_RECEIVER_FAMILY_OPTIONS.map((option) => ({
                                                label: option.label,
                                                value: option.value,
                                            }))}
                                        />
                                    </Form.Item>
                                </Col>
                                <Col xs={24} sm={14}>
                                    <Form.Item label="Serial Device">
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
                            type="info"
                            showIcon
                            message="Expert-mode scope"
                            description={"For UM982, \"UM982 recommended\" maps to CONFIG SIGNALGROUP 3 6. These expert fields are saved now, but the backend API that translates them into receiver commands is still TODO."}
                        />
                    </Card>

                    <UniversalGnssAdvancedSettings
                        receiverFamily={values.gnss_receiver_family ?? "auto"}
                        values={values}
                        onChange={onChange}
                    />
                </>
            )}

            <UniversalGnssLiveStatusCard
                diagnostics={diagnostics}
                gnssStatus={gnssStatus}
                selectedBaud={values.gnss_serial_baud}
                selectedConfigBaud={values.gnss_config_baud}
                selectedProfile={values.gnss_profile}
                selectedSignalProfile={values.gnss_signal_profile}
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
                message="Save & Continue to start the receiver"
                description="Saving this step writes your signal profile and baud to the GPS receiver and restarts it automatically (NTRIP corrections were set in the previous step). Acquiring an RTK fix can take 30 s to a few minutes — that happens in the background while you set the datum next."
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
    const guiApi = useApi();
    const [datumLoading, setDatumLoading] = useState(false);

    const gnssStatus = useGnssStatus();
    const fixType = gnssStatus.fix_type ?? GnssStatusConstants.FIX_TYPE_NO_FIX;
    const isRtkFixed = fixType === GnssStatusConstants.FIX_TYPE_RTK_FIXED;
    const isRtkFloat = fixType === GnssStatusConstants.FIX_TYPE_RTK_FLOAT;
    const isPlainFix = fixType === GnssStatusConstants.FIX_TYPE_GPS_FIX;
    const fixLabel = isRtkFixed ? "RTK FIX" : isRtkFloat ? "RTK FLOAT" : isPlainFix ? "GPS FIX" : "no fix";

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
            alert(e.message || "Failed to set datum from GPS");
        } finally {
            setDatumLoading(false);
        }
    };

    return (
        <div style={{ maxWidth: 640, margin: "0 auto" }}>
            <Title level={4}>
                <EnvironmentOutlined /> Map Origin (Datum)
            </Title>
            <Paragraph type="secondary">
                Anchor the robot's local map to a GPS coordinate near your docking station. This becomes the
                (0, 0) origin of every later map and mowing area. Drop it on the dock if possible.
            </Paragraph>

            <Card size="small" title={<Space><EnvironmentOutlined /> Datum coordinates</Space>} style={{ marginBottom: 16 }}>
                <Paragraph type="secondary" style={{ fontSize: 12, marginBottom: 12 }}>
                    Either enter the coordinate manually (right-click your dock in Google Maps to copy lat/lon),
                    or capture it directly from the robot once it has an RTK Fix.
                </Paragraph>
                <Form layout="vertical">
                    <Row gutter={16}>
                        <Col xs={12}>
                            <Form.Item label="Latitude">
                                <InputNumber
                                    value={values.datum_lat ?? 0}
                                    onChange={(v) => onChange("datum_lat", v)}
                                    step={0.000001} precision={8} style={{ width: "100%" }}
                                    placeholder="48.8796"
                                />
                            </Form.Item>
                        </Col>
                        <Col xs={12}>
                            <Form.Item label="Longitude">
                                <InputNumber
                                    value={values.datum_lon ?? 0}
                                    onChange={(v) => onChange("datum_lon", v)}
                                    step={0.000001} precision={8} style={{ width: "100%" }}
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
                            ? "GPS restarting…"
                            : `Use current GPS position ${isRtkFixed ? "" : "(waiting for RTK Fix)"}`}
                    </Button>
                    {gpsRestarting && (
                        <Alert
                            type="info"
                            showIcon
                            message="GPS container is restarting to apply your NTRIP / serial settings"
                            description="Wait ~10–30 s for RTK Fix to come back before setting the datum."
                            style={{ marginTop: 12 }}
                        />
                    )}
                    {!isRtkFixed && !gpsRestarting && (
                        <Alert
                            type="warning"
                            showIcon
                            message={`Current GPS quality: ${fixLabel}`}
                            description={
                                isRtkFloat
                                    ? "RTK Float gives ~10–20 cm accuracy and drifts when corrections lapse. Wait for RTK Fix (σ ~3 mm) before anchoring the datum."
                                    : "Without RTK Fix the datum can be metres off, which silently breaks every later mow. Make sure NTRIP corrections are flowing and the antenna has clear sky."
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
    return (
        <div style={{ maxWidth: 900, margin: "0 auto" }}>
            <Title level={4}>
                <AimOutlined /> Sensor Placement
            </Title>
            <Paragraph type="secondary" style={{ marginBottom: 16 }}>
                Position your sensors on the robot. Drag them on the top-down view or use the precise numeric inputs.
                These positions tell the robot exactly where each sensor is mounted relative to the chassis centre.
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
        <div style={{maxWidth: 700, margin: "0 auto"}}>
            <Title level={4}>
                <CompassOutlined/> Sensor Calibration
            </Title>
            <Paragraph type="secondary" style={{marginBottom: 16}}>
                The robot drives itself through a short routine to learn how its sensors are mounted.
                Without this step the robot drifts in odom, can dock at an angle, and loses its heading
                when GPS corrections lapse. Plan for up to 2&nbsp;minutes of autonomous motion — stand
                clear, collision_monitor stays armed.
            </Paragraph>

            <Card size="small" style={{marginBottom: 16}}>
                <Paragraph strong style={{marginBottom: 8}}>What this calibration measures</Paragraph>
                <ul style={{paddingLeft: 20, marginBottom: 0, color: colors.textSecondary, fontSize: 13}}>
                    <li><Text strong>Dock pose</Text> (when started on the dock): the robot reverses ~2&nbsp;m under RTK GPS to capture the dock's lat/lon and heading, then writes them to <Text code>mowgli_robot.yaml</Text>.</li>
                    <li><Text strong>IMU mounting yaw</Text>: 3 forward/backward cycles at 0.5&nbsp;m/s let the accelerometer's body-frame impulse be compared to the wheel-derived acceleration so the IMU's rotation around base_link is recovered.</li>
                    <li><Text strong>Pitch / roll bias</Text>: the stationary baseline windows expose any non-level mounting (1° offsets here matter — gyro integration drifts otherwise).</li>
                    <li><Text strong>Magnetometer hard/soft-iron</Text> (if a mag is publishing): a slow figure-8 fits the ellipsoid offsets so tilt-compensated yaw is usable as an absolute heading source.</li>
                </ul>
            </Card>

            <Card size="small" style={{marginBottom: 16}}>
                <Paragraph strong style={{marginBottom: 8}}>Pre-flight checklist</Paragraph>
                <ul style={{paddingLeft: 20, marginBottom: 0, color: colors.textSecondary, fontSize: 13}}>
                    <li>Place the robot <Text strong>on the dock</Text> if you want the dock pose recorded too — otherwise just leave it parked with ≥&nbsp;1.5&nbsp;m of clear space ahead and behind.</li>
                    <li>NTRIP corrections flowing and an RTK Fix nearby (otherwise the dock pre-phase is skipped, but IMU calibration still runs).</li>
                    <li>No active emergency, blade off, lid closed.</li>
                    <li>Don't move or touch the robot during the run — every bump shows up as accelerometer noise and widens the std-dev.</li>
                </ul>
            </Card>

            <Card size="small" style={{marginBottom: 16}}>
                <Row gutter={[16, 8]}>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>Current imu_yaw</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>{currentImuYawDeg.toFixed(2)}°</div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>Dock pose</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {dockPresent ? <Tag color="success">Present</Tag> : <Tag color="warning">Missing</Tag>}
                        </div>
                    </Col>
                    <Col xs={12}>
                        <Text type="secondary" style={{fontSize: 11}}>IMU bias</Text>
                        <div style={{fontSize: 18, fontWeight: 500}}>
                            {imuPresent ? <Tag color="success">Present</Tag> : <Tag color="warning">Missing</Tag>}
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
                    {calibResult ? "Re-run calibration" : "Start IMU yaw calibration"}
                </Button>
            </div>

            {calibRunning && (
                <Alert
                    type="info"
                    showIcon
                    message="Calibration running — robot is driving itself"
                    description="Forward leg, pause, backward leg. Stand clear. Motion stops automatically. May take up to 2 minutes (longer when the dock pre-phase runs)."
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
                            <div>From {calibResult.samples_used} valid motion samples.</div>
                            {calibResult.dock_valid && (
                                <div>
                                    Dock pose updated: yaw={calibResult.dock_pose_yaw_deg?.toFixed(2)}°
                                    (σ {calibResult.dock_yaw_sigma_deg?.toFixed(2)}°,
                                    displacement {calibResult.dock_undock_displacement_m?.toFixed(2) ?? "?"} m).
                                </div>
                            )}
                            <div style={{marginTop: 12}}>
                                <Space>
                                    <Button type="primary" onClick={applyCalibration}>Apply to settings</Button>
                                    <Button onClick={resetCalibration}>Discard</Button>
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
                    message="Calibration failed"
                    description={
                        <>
                            <div>{calibResult.message}</div>
                            <div style={{marginTop: 8, color: colors.textSecondary}}>
                                Hint: drive faster or longer so the accelerometer sees a clear forward and
                                backward impulse along the body X axis.
                            </div>
                            <div style={{marginTop: 12}}>
                                <Button onClick={resetCalibration}>Reset</Button>
                            </div>
                        </>
                    }
                    style={{marginBottom: 16}}
                />
            )}

            <Alert
                type="info"
                showIcon
                message="You can skip this step"
                description="If you have already calibrated this robot from the Diagnostics page (or know your imu_yaw value), use the Next button. The Complete step will warn you if calibration is still missing."
                style={{maxWidth: 500, margin: "0 auto"}}
            />
        </div>
    );
};

// ── Step 5: Firmware ────────────────────────────────────────────────────

const FirmwareStep: React.FC<{ onNext: () => void }> = ({ onNext }) => {
    const { colors } = useThemeMode();
    const [showFlash, setShowFlash] = useState(false);

    if (showFlash) {
        return (
            <Card title="Flash Firmware">
                <FlashBoardComponent onNext={onNext} />
            </Card>
        );
    }

    return (
        <div style={{ maxWidth: 600, margin: "0 auto", textAlign: "center", padding: "24px 0" }}>
            <div style={{
                width: 64, height: 64, borderRadius: "50%",
                background: colors.primaryBg, display: "flex",
                alignItems: "center", justifyContent: "center",
                margin: "0 auto 16px",
            }}>
                <ThunderboltOutlined style={{ fontSize: 28, color: colors.primary }} />
            </div>
            <Title level={4}>Firmware</Title>
            <Paragraph type="secondary" style={{ marginBottom: 24 }}>
                If this is a new build, you need to flash the Mowgli firmware onto your motherboard.
                If your firmware is already up to date, you can skip this step.
            </Paragraph>

            <Space size="middle">
                <Button type="primary" size="large" onClick={() => setShowFlash(true)}>
                    Flash Firmware
                </Button>
                <Button size="large" onClick={onNext}>
                    Skip — Already Flashed
                </Button>
            </Space>

            <Alert
                type="warning"
                showIcon
                message="Flashing will rewrite your motherboard firmware"
                description="Make sure your mower is connected via USB and powered on. Wrong voltage settings can damage hardware."
                style={{ marginTop: 24, textAlign: "left" }}
            />
        </div>
    );
};

// ── Step 5: Complete ────────────────────────────────────────────────────

const CompleteStep: React.FC = () => {
    const { colors } = useThemeMode();
    const guiApi = useApi();
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
        if (!calibrationStatus.dock?.present) missingCalibrations.push("dock pose");
        if (!calibrationStatus.imu?.present) missingCalibrations.push("IMU bias + mounting");
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
                title="Applying configuration..."
                subTitle="Restarting the mower service with your new settings. This takes a few seconds."
            />
        );
    }

    return (
        <Result
            icon={<CheckCircleOutlined style={{ color: colors.primary }} />}
            title="You're all set!"
            subTitle="Your mower is configured and ready to go. Head to the Map to draw your first mowing area, or check the Dashboard to monitor your robot."
            extra={[
                <Button
                    key="map"
                    type="primary"
                    size="large"
                    icon={<EnvironmentOutlined />}
                    onClick={() => { window.location.href = "/#/map"; }}
                >
                    Draw Mowing Area
                </Button>,
                <Button
                    key="dashboard"
                    size="large"
                    onClick={() => { window.location.href = "/#/mowglinext"; }}
                >
                    Go to Dashboard
                </Button>,
            ]}
        >
            {missingCalibrations.length > 0 && (
                <Alert
                    type="warning"
                    showIcon
                    message="Calibration steps still pending"
                    description={
                        <>
                            <Text>
                                Your robot is configured, but{" "}
                                <Text strong>{missingCalibrations.join(" and ")}</Text>{" "}
                                {missingCalibrations.length === 1 ? "is" : "are"} not calibrated yet.
                                Without these the robot will drift in odom and may dock at an angle.
                            </Text>
                            <br />
                            <Button
                                type="link"
                                style={{ paddingLeft: 0 }}
                                onClick={() => { window.location.href = "/#/diagnostics"; }}
                            >
                                Open Diagnostics → run calibrations →
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
                    message="Could not restart the mower service"
                    description={`${error}. You may need to restart it manually.`}
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

const STEP_TITLES = [
    "Welcome",
    "Robot Model",
    "Firmware",
    "NTRIP",
    "GPS",
    "Datum",
    "Sensors",
    "Calibration",
    "Complete",
];

const OnboardingWizard: React.FC = () => {
    const { colors } = useThemeMode();
    const isMobile = useIsMobile();
    const { values: savedValues, saveValues, savePartialValues, loading } = useSettingsSchema();
    const guiApi = useApi();
    const [currentStep, setCurrentStep] = useState(0);
    const [localValues, setLocalValues] = useState<Record<string, any>>({});
    const [saving, setSaving] = useState(false);
    const gpsRestart = useContainerRestart({
        pendingLabel: "Redémarrage GPS…",
        successMessage: "GPS redémarré — patientez pour le RTK Fix",
        errorMessage: "Échec du redémarrage GPS",
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

    const stepItems = STEP_TITLES.map((title, i) => ({ title, icon: STEP_ICONS[i] }));

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
                        errorMessage: "Failed to save GNSS settings before running the receiver action",
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
                    Back
                </Button>
                <Button
                    type="primary"
                    icon={currentStep === STEP_DATUM ? <SaveOutlined /> : <ArrowRightOutlined />}
                    onClick={handleNext}
                    loading={saving || loading || gpsRestarting}
                >
                    {gpsRestarting
                        ? "Restarting GPS…"
                        : currentStep === STEP_DATUM
                            ? "Save & Finish"
                            : "Next"}
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
                                <Text strong style={{ fontSize: 15 }}>{STEP_TITLES[currentStep]}</Text>
                            </Space>
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                Step {currentStep + 1} of {STEP_TITLES.length}
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
