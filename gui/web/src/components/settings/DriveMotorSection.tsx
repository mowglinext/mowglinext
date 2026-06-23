import React, { useEffect, useMemo, useRef, useState } from "react";
import {
    Alert,
    App,
    Button,
    Card,
    Col,
    Collapse,
    Descriptions,
    Form,
    InputNumber,
    Modal,
    Row,
    Space,
    Switch,
    Tag,
    Typography,
} from "antd";
import { DashboardOutlined, FileTextOutlined, HistoryOutlined, PlayCircleOutlined } from "@ant-design/icons";
import type { TFunction } from "i18next";
import { useDockingSensor } from "../../hooks/useDockingSensor.ts";
import { useDriveTuning } from "../../hooks/useDriveTuning.ts";
import { useEmergency } from "../../hooks/useEmergency.ts";
import { useStatus } from "../../hooks/useStatus.ts";
import { useTranslation } from "react-i18next";
import {
    formatDriveTuningBoolean,
    translateDriveTuningBackendMessage,
    translateDriveTuningInternalTier,
    translateDriveTuningTrialPhase,
    translateDriveTuningTrialQuality,
} from "../../utils/driveTuningI18n.ts";

const { Text, Paragraph } = Typography;

type Props = {
    values: Record<string, any>;
    onChange: (key: string, value: any) => void;
    acceptPersistedValues?: (values: Record<string, any>) => void;
};

const DRIVE_PARAM_KEYS = [
    "ticks_per_meter",
    "wheel_pid_pwm_per_mps",
    "wheel_pid_kp",
    "wheel_pid_ki",
    "wheel_pid_kd",
    "wheel_pid_integral_limit",
];

const DRIVE_PARAM_PRECISION = 3;

const formatDriveParamValue = (key: string, value: unknown) => {
    if (typeof value === "number" && Number.isFinite(value) && DRIVE_PARAM_KEYS.includes(key)) {
        return value.toFixed(DRIVE_PARAM_PRECISION);
    }
    return String(value);
};

const formatReportNumber = (t: TFunction, value?: number | null, digits = 3) => {
    if (typeof value !== "number" || !Number.isFinite(value)) {
        return t("settingsDriveMotor.common.unknown");
    }
    return value.toFixed(digits);
};

const hasFiniteNumber = (value?: number | null): value is number => (
    typeof value === "number" && Number.isFinite(value)
);

const statusTag = (t: TFunction, status: "not_validated" | "validated" | "warning") => {
    if (status === "validated") {
        return <Tag color="success">{t("settingsDriveMotor.tags.validated")}</Tag>;
    }
    if (status === "warning") {
        return <Tag color="warning">{t("settingsDriveMotor.tags.warning")}</Tag>;
    }
    return <Tag>{t("settingsDriveMotor.tags.notValidated")}</Tag>;
};

const jobTag = (t: TFunction, state?: string) => {
    if (state === "running") {
        return <Tag color="processing">{t("settingsDriveMotor.job.running")}</Tag>;
    }
    if (state === "succeeded") {
        return <Tag color="success">{t("settingsDriveMotor.job.completed")}</Tag>;
    }
    if (state === "warning") {
        return <Tag color="warning">{t("settingsDriveMotor.job.completedWithWarning")}</Tag>;
    }
    if (state === "failed") {
        return <Tag color="error">{t("settingsDriveMotor.job.failed")}</Tag>;
    }
    return <Tag>{t("settingsDriveMotor.job.idle")}</Tag>;
};

const formatTimestamp = (t: TFunction, value?: string) => {
    if (!value) return t("settingsDriveMotor.common.never");
    const parsed = new Date(value);
    return Number.isNaN(parsed.getTime()) ? value : parsed.toLocaleString();
};

const pickPersistedDriveValues = (report: Record<string, any> | undefined) => {
    if (!report) return {};
    const picked: Record<string, any> = {};
    for (const key of DRIVE_PARAM_KEYS) {
        if (key in report) {
            picked[key] = report[key];
        }
    }
    return picked;
};

export const DriveMotorSection: React.FC<Props> = ({ values, onChange, acceptPersistedValues }) => {
    const { t } = useTranslation();
    const { notification, modal } = App.useApp();
    const emergency = useEmergency();
    const status = useStatus();
    const dockingSensor = useDockingSensor();
    const {
        status: tuningStatus,
        error: tuningError,
        latestReport,
        loadingLatestReport,
        startFeedForward,
        startPID,
        rollback,
        loadLatestReport,
    } = useDriveTuning();

    const [ffOpen, setFfOpen] = useState(false);
    const [pidOpen, setPidOpen] = useState(false);
    const [reportOpen, setReportOpen] = useState(false);
    const [startingAction, setStartingAction] = useState<"ff" | "pid" | null>(null);
    const [rollingBack, setRollingBack] = useState(false);
    const [ffForm] = Form.useForm();
    const [pidForm] = Form.useForm();
    const appliedJobIdsRef = useRef<Set<string>>(new Set());

    const isEmergencyActive = !!emergency.active_emergency || !!emergency.latched_emergency;
    const onDock = !!dockingSensor.dock_present || !!status.is_charging;
    const runningJob = tuningStatus?.job?.state === "running";
    const latestReportMeta = tuningStatus?.latest_report;

    const feedForwardSummary = tuningStatus?.feed_forward;
    const pidSummary = tuningStatus?.pid;
    const translateBackendMessage = (message?: string | null) => (
        translateDriveTuningBackendMessage(t, message) ?? message ?? undefined
    );
    const formatBackendError = (error: any) => (
        translateDriveTuningBackendMessage(t, error?.error?.error ?? error?.message)
        ?? error?.error?.error
        ?? error?.message
        ?? t("settingsDriveMotor.common.unknownBackendError")
    );
    const translatedFeedForwardSummary = translateBackendMessage(feedForwardSummary?.message)
        ?? t("settingsDriveMotor.common.noReportYetSentence");
    const translatedPidSummary = translateBackendMessage(pidSummary?.message)
        ?? t("settingsDriveMotor.common.noReportYetSentence");
    const translatedJobError = translateBackendMessage(tuningStatus?.job?.error);
    const translatedInternalTier = translateDriveTuningInternalTier(t, latestReport?.parsed?.internal_tuning_tier)
        ?? t("settingsDriveMotor.common.unknown");

    useEffect(() => {
        const job = tuningStatus?.job;
        if (!job || !job.apply || job.state !== "succeeded") {
            return;
        }
        if (appliedJobIdsRef.current.has(job.id)) {
            return;
        }

        let cancelled = false;
        (async () => {
            try {
                const reportResponse = await loadLatestReport();
                const sameReport = reportResponse?.latest_report?.report_path === job.report_path;
                const persistedValues = pickPersistedDriveValues(reportResponse?.parsed?.proposed_params);
                if (!cancelled && sameReport && Object.keys(persistedValues).length > 0) {
                    acceptPersistedValues?.(persistedValues);
                    appliedJobIdsRef.current.add(job.id);
                    notification.success({
                        message: t("settingsDriveMotor.notifications.applied.title"),
                        description: t("settingsDriveMotor.notifications.applied.description"),
                    });
                }
            } catch {
                // The status panel still shows the backend error; keep this effect best-effort.
            }
        })();

        return () => {
            cancelled = true;
        };
    }, [acceptPersistedValues, loadLatestReport, notification, t, tuningStatus?.job]);

    const helperAlerts = useMemo(() => {
        const alerts: React.ReactNode[] = [];
        if (isEmergencyActive) {
            alerts.push(
                <Alert
                    key="emergency"
                    type="error"
                    showIcon
                    message={t("settingsDriveMotor.alerts.emergency.title")}
                    description={t("settingsDriveMotor.alerts.emergency.description")}
                />,
            );
        }
        if (onDock) {
            alerts.push(
                <Alert
                    key="dock"
                    type="warning"
                    showIcon
                    message={t("settingsDriveMotor.alerts.onDock.title")}
                    description={t("settingsDriveMotor.alerts.onDock.description")}
                />,
            );
        }
        if (tuningError) {
            alerts.push(
                <Alert
                    key="backend"
                    type="warning"
                    showIcon
                    message={t("settingsDriveMotor.alerts.backendWarning.title")}
                    description={translateBackendMessage(tuningError)}
                />,
            );
        }
        return alerts;
    }, [isEmergencyActive, onDock, t, tuningError]);

    const openLatestReport = async () => {
        try {
            await loadLatestReport();
            setReportOpen(true);
        } catch (e: any) {
            notification.error({
                message: t("settingsDriveMotor.notifications.loadReportFailed.title"),
                description: formatBackendError(e),
            });
        }
    };

    const handleStartFeedForward = async () => {
        try {
            const values = await ffForm.validateFields();
            setStartingAction("ff");
            await startFeedForward({
                distance_m: values.distance_m,
                test_speed_mps: values.test_speed_mps,
                odom_timeout_s: values.odom_timeout_s,
                passes: values.passes,
                auto_turn: values.auto_turn,
                turn_direction: "right",
                apply: values.apply,
                allow_undock: values.allow_undock,
                undock_distance_m: values.undock_distance_m,
            });
            setFfOpen(false);
            notification.success({
                message: t("settingsDriveMotor.notifications.ffStarted.title"),
                description: values.apply
                    ? t("settingsDriveMotor.notifications.ffStarted.applyDescription")
                    : t("settingsDriveMotor.notifications.ffStarted.reportOnlyDescription"),
            });
        } catch (e: any) {
            if (e?.errorFields) {
                return;
            }
            notification.error({
                message: t("settingsDriveMotor.notifications.ffStartFailed.title"),
                description: formatBackendError(e),
            });
        } finally {
            setStartingAction(null);
        }
    };

    const handleStartPid = async () => {
        try {
            const values = await pidForm.validateFields();
            setStartingAction("pid");
            await startPID({
                max_speed_mps: values.max_speed_mps,
                segment_duration_s: values.segment_duration_s,
                passes: values.passes,
                apply: values.apply,
                allow_undock: values.allow_undock,
                undock_distance_m: values.undock_distance_m,
            });
            setPidOpen(false);
            notification.success({
                message: t("settingsDriveMotor.notifications.pidStarted.title"),
                description: values.apply
                    ? t("settingsDriveMotor.notifications.pidStarted.applyDescription")
                    : t("settingsDriveMotor.notifications.pidStarted.reportOnlyDescription"),
            });
        } catch (e: any) {
            if (e?.errorFields) {
                return;
            }
            notification.error({
                message: t("settingsDriveMotor.notifications.pidStartFailed.title"),
                description: formatBackendError(e),
            });
        } finally {
            setStartingAction(null);
        }
    };

    const confirmRollback = () => {
        modal.confirm({
            title: t("settingsDriveMotor.rollback.confirm.title"),
            content: (
                <Space direction="vertical" size={8}>
                    <Text>
                        {t("settingsDriveMotor.rollback.confirm.bodyPrefix")} <Text code>mowgli_tools tune_drive_pid</Text>
                        {t("settingsDriveMotor.rollback.confirm.bodyMiddle")} <Text code>hardware_bridge</Text>
                        {t("settingsDriveMotor.rollback.confirm.bodySuffix")} <Text code>mowgli_robot.yaml</Text>.
                    </Text>
                    <Text type="warning">
                        {t("settingsDriveMotor.rollback.confirm.warning")}
                    </Text>
                </Space>
            ),
            okText: t("settingsDriveMotor.rollback.confirm.ok"),
            okType: "danger",
            cancelText: t("settingsDriveMotor.common.cancel"),
            onOk: async () => {
                try {
                    setRollingBack(true);
                    const response = await rollback();
                    if (response?.restored) {
                        acceptPersistedValues?.(response.restored);
                    }
                    notification.success({
                        message: translateBackendMessage(response?.message)
                            ?? t("settingsDriveMotor.notifications.rollbackCompleted.title"),
                    });
                } catch (e: any) {
                    notification.error({
                        message: t("settingsDriveMotor.notifications.rollbackFailed.title"),
                        description: formatBackendError(e),
                    });
                } finally {
                    setRollingBack(false);
                }
            },
        });
    };

    const ffApply = Form.useWatch("apply", ffForm);
    const ffAllowUndock = Form.useWatch("allow_undock", ffForm);
    const pidApply = Form.useWatch("apply", pidForm);
    const pidAllowUndock = Form.useWatch("allow_undock", pidForm);

    const latestParsedReport = latestReport?.parsed;
    const latestReasons = latestParsedReport?.reasons ?? [];
    const latestTrials = latestParsedReport?.trials ?? [];
    const latestCmdVelTopic = latestParsedReport?.cmd_vel_topic ?? latestParsedReport?.cmd_topic;
    const latestFailureMessage = latestParsedReport?.failure_message;
    const latestStatusSnapshot = latestParsedReport?.status_snapshot;
    const latestDrivetrain = latestParsedReport?.drivetrain_diagnostics;

    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message={t("settingsDriveMotor.savedLiveTitle")}
                description={t("settingsDriveMotor.savedLiveDescription")}
            />

            <Card
                size="small"
                title={t("settingsDriveMotor.cards.assistants.title")}
                style={{ marginBottom: 16 }}
                extra={
                    <Space wrap>
                        <Button
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onClick={() => setFfOpen(true)}
                            disabled={runningJob || isEmergencyActive}
                        >
                            {t("settingsDriveMotor.cards.assistants.calibrateButton")}
                        </Button>
                        <Button
                            icon={<PlayCircleOutlined />}
                            onClick={() => setPidOpen(true)}
                            disabled={runningJob || isEmergencyActive}
                        >
                            {t("settingsDriveMotor.cards.assistants.pidButton")}
                        </Button>
                    </Space>
                }
            >
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    {helperAlerts}

                    <Alert
                        type="info"
                        showIcon
                        message={t("settingsDriveMotor.cards.assistants.motionProfile.title")}
                        description={t("settingsDriveMotor.cards.assistants.motionProfile.description")}
                    />

                    <Descriptions size="small" column={1} bordered>
                        <Descriptions.Item label={t("settingsDriveMotor.summary.feedForward.label")}>
                            <Space wrap>
                                {statusTag(t, feedForwardSummary?.status ?? "not_validated")}
                                <Text type="secondary">{translatedFeedForwardSummary}</Text>
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label={t("settingsDriveMotor.summary.pid.label")}>
                            <Space wrap>
                                {statusTag(t, pidSummary?.status ?? "not_validated")}
                                <Text type="secondary">{translatedPidSummary}</Text>
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label={t("settingsDriveMotor.summary.currentJob.label")}>
                            <Space wrap>
                                {jobTag(t, tuningStatus?.job?.state)}
                                {tuningStatus?.job && (
                                    <Text type="secondary">
                                        {t("settingsDriveMotor.summary.currentJob.startedAt", {
                                            mode: tuningStatus.job.mode.toUpperCase(),
                                            timestamp: formatTimestamp(t, tuningStatus.job.started_at),
                                        })}
                                    </Text>
                                )}
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label={t("settingsDriveMotor.summary.lastReportDate.label")}>
                            <Text>{formatTimestamp(t, latestReportMeta?.generated_at)}</Text>
                        </Descriptions.Item>
                        <Descriptions.Item label={t("settingsDriveMotor.summary.lastReportPath.label")}>
                            <Text code copyable={!!latestReportMeta?.report_path}>
                                {latestReportMeta?.report_path ?? t("settingsDriveMotor.common.noReportYet")}
                            </Text>
                        </Descriptions.Item>
                    </Descriptions>

                    <Space wrap>
                        <Button type="primary" onClick={() => setFfOpen(true)} disabled={runningJob || isEmergencyActive}>
                            {t("settingsDriveMotor.actions.startFeedForward")}
                        </Button>
                        <Button onClick={() => setPidOpen(true)} disabled={runningJob || isEmergencyActive}>
                            {t("settingsDriveMotor.actions.startPid")}
                        </Button>
                        <Button icon={<FileTextOutlined />} onClick={openLatestReport} loading={loadingLatestReport}>
                            {t("settingsDriveMotor.actions.viewLastReport")}
                        </Button>
                        <Button
                            danger
                            icon={<HistoryOutlined />}
                            onClick={confirmRollback}
                            disabled={runningJob}
                            loading={rollingBack}
                        >
                            {t("settingsDriveMotor.actions.rollbackLastTuning")}
                        </Button>
                    </Space>

                    {tuningStatus?.job && (
                        <Collapse
                            items={[
                                {
                                    key: "job-log",
                                    label: t("settingsDriveMotor.jobLog.title", {
                                        mode: tuningStatus.job.mode.toUpperCase(),
                                    }),
                                    children: (
                                        <Space direction="vertical" size={8} style={{ width: "100%" }}>
                                            {translatedJobError && (
                                                <Alert
                                                    type={tuningStatus.job.state === "failed" ? "error" : "warning"}
                                                    showIcon
                                                    message={translatedJobError}
                                                />
                                            )}
                                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                                {tuningStatus.job.logs?.trim() || t("settingsDriveMotor.jobLog.waiting")}
                                            </pre>
                                        </Space>
                                    ),
                                },
                            ]}
                        />
                    )}
                </Space>
            </Card>

            <Card size="small" style={{ marginBottom: 16 }}>
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    <div>
                        <Text strong style={{ fontSize: 14 }}>
                            <DashboardOutlined style={{ marginRight: 6 }} />
                            {t("settingsDriveMotor.wheelVelocityPid")}
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            {t("settingsDriveMotor.wheelVelocityPidDescription")}
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsDriveMotor.params.kpLabel")} tooltip={t("settingsDriveMotor.kpTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_kp}
                                        onChange={(v) => onChange("wheel_pid_kp", v)}
                                        min={0} max={200} step={0.001} precision={3}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsDriveMotor.params.kiLabel")} tooltip={t("settingsDriveMotor.kiTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_ki}
                                        onChange={(v) => onChange("wheel_pid_ki", v)}
                                        min={0} max={20000} step={0.001} precision={3}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsDriveMotor.params.kdLabel")} tooltip={t("settingsDriveMotor.kdTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_kd}
                                        onChange={(v) => onChange("wheel_pid_kd", v)}
                                        min={0} max={500} step={0.001} precision={3}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label={t("settingsDriveMotor.integralLimit")} tooltip={t("settingsDriveMotor.integralLimitTooltip")}>
                                    <InputNumber
                                        value={values.wheel_pid_integral_limit}
                                        onChange={(v) => onChange("wheel_pid_integral_limit", v)}
                                        min={0} max={255} step={0.001} precision={3}
                                        style={{ width: "100%" }} addonAfter="PWM"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title={t("settingsDriveMotor.feedforward")} style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label={t("settingsDriveMotor.pwmPerMps")}
                                tooltip={t("settingsDriveMotor.pwmPerMpsTooltip")}
                            >
                                <InputNumber
                                    value={values.wheel_pid_pwm_per_mps}
                                    onChange={(v) => onChange("wheel_pid_pwm_per_mps", v)}
                                    min={50} max={600} step={0.001} precision={3}
                                    style={{ width: "100%" }} addonAfter="PWM"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Modal
                title={t("settingsDriveMotor.ffModal.title")}
                open={ffOpen}
                onCancel={() => setFfOpen(false)}
                onOk={handleStartFeedForward}
                okText={t("settingsDriveMotor.ffModal.ok")}
                cancelText={t("settingsDriveMotor.common.cancel")}
                confirmLoading={startingAction === "ff"}
                okButtonProps={{
                    disabled: isEmergencyActive || (onDock && !ffAllowUndock),
                }}
            >
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    {onDock && !ffAllowUndock && (
                        <Alert
                            type="warning"
                            showIcon
                            message={t("settingsDriveMotor.common.robotOnDockTitle")}
                            description={t("settingsDriveMotor.ffModal.onDock.description")}
                        />
                    )}
                    <Alert
                        type="info"
                        showIcon
                        message={t("settingsDriveMotor.ffModal.workflow.title")}
                        description={t("settingsDriveMotor.ffModal.workflow.description")}
                    />
                    <Form
                        form={ffForm}
                        layout="vertical"
                        initialValues={{
                            distance_m: 3,
                            test_speed_mps: 0.3,
                            odom_timeout_s: 4.0,
                            passes: 3,
                            auto_turn: true,
                            apply: false,
                            allow_undock: false,
                            undock_distance_m: 2,
                        }}
                    >
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item name="distance_m" label={t("settingsDriveMotor.ffModal.fields.distance")} rules={[{ required: true }]}>
                                    <InputNumber min={2} max={10} step={0.5} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="test_speed_mps" label={t("settingsDriveMotor.ffModal.fields.testSpeed")} rules={[{ required: true }]}>
                                    <InputNumber min={0.05} max={0.5} step={0.05} precision={2} style={{ width: "100%" }} addonAfter="m/s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="passes" label={t("settingsDriveMotor.common.passes")} rules={[{ required: true }]}>
                                    <InputNumber min={1} max={10} step={1} precision={0} style={{ width: "100%" }} />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item
                                    name="odom_timeout_s"
                                    label={t("settingsDriveMotor.ffModal.fields.odomTimeout")}
                                    tooltip={t("settingsDriveMotor.ffModal.fields.odomTimeoutTooltip")}
                                    rules={[{ required: true }]}
                                >
                                    <InputNumber min={0.5} max={15} step={0.5} precision={1} style={{ width: "100%" }} addonAfter="s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="auto_turn" label={t("settingsDriveMotor.ffModal.fields.autoUTurn")} valuePropName="checked">
                                    <Switch
                                        checkedChildren={t("settingsDriveMotor.common.enabled")}
                                        unCheckedChildren={t("settingsDriveMotor.common.disabled")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="apply" label={t("settingsDriveMotor.common.applyPersistIfSuccessful")} valuePropName="checked">
                                    <Switch
                                        checkedChildren={t("settingsDriveMotor.common.apply")}
                                        unCheckedChildren={t("settingsDriveMotor.common.reportOnly")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="allow_undock" label={t("settingsDriveMotor.common.allowUndockIfDocked")} valuePropName="checked">
                                    <Switch
                                        checkedChildren={t("settingsDriveMotor.common.allow")}
                                        unCheckedChildren={t("settingsDriveMotor.common.no")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item
                                    name="undock_distance_m"
                                    label={t("settingsDriveMotor.common.undockDistance")}
                                    tooltip={t("settingsDriveMotor.common.undockDistanceTooltip")}
                                >
                                    <InputNumber min={0.5} max={5} step={0.1} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                    {ffApply ? (
                        <Alert
                            type="success"
                            showIcon
                            message={t("settingsDriveMotor.ffModal.resultApply.title")}
                            description={t("settingsDriveMotor.ffModal.resultApply.description")}
                        />
                    ) : (
                        <Alert
                            type="info"
                            showIcon
                            message={t("settingsDriveMotor.common.reportOnlyModeTitle")}
                            description={t("settingsDriveMotor.ffModal.resultReportOnly.description")}
                        />
                    )}
                </Space>
            </Modal>

            <Modal
                title={t("settingsDriveMotor.pidModal.title")}
                open={pidOpen}
                onCancel={() => setPidOpen(false)}
                onOk={handleStartPid}
                okText={t("settingsDriveMotor.pidModal.ok")}
                cancelText={t("settingsDriveMotor.common.cancel")}
                confirmLoading={startingAction === "pid"}
                okButtonProps={{
                    disabled: isEmergencyActive || (onDock && !pidAllowUndock),
                }}
            >
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    {feedForwardSummary?.status !== "validated" && (
                        <Alert
                            type="warning"
                            showIcon
                            message={t("settingsDriveMotor.pidModal.precheck.title")}
                            description={t("settingsDriveMotor.pidModal.precheck.description")}
                        />
                    )}
                    {onDock && !pidAllowUndock && (
                        <Alert
                            type="warning"
                            showIcon
                            message={t("settingsDriveMotor.common.robotOnDockTitle")}
                            description={t("settingsDriveMotor.pidModal.onDock.description")}
                        />
                    )}
                    <Alert
                        type="info"
                        showIcon
                        message={t("settingsDriveMotor.pidModal.workflow.title")}
                        description={t("settingsDriveMotor.pidModal.workflow.description")}
                    />
                    <Form
                        form={pidForm}
                        layout="vertical"
                        initialValues={{
                            max_speed_mps: 0.3,
                            segment_duration_s: 5,
                            passes: 3,
                            apply: false,
                            allow_undock: false,
                            undock_distance_m: 2,
                        }}
                    >
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item name="max_speed_mps" label={t("settingsDriveMotor.pidModal.fields.maxSpeed")} rules={[{ required: true }]}>
                                    <InputNumber min={0.1} max={0.5} step={0.05} precision={2} style={{ width: "100%" }} addonAfter="m/s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="segment_duration_s" label={t("settingsDriveMotor.pidModal.fields.segmentDuration")} rules={[{ required: true }]}>
                                    <InputNumber min={2} max={20} step={0.5} precision={1} style={{ width: "100%" }} addonAfter="s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="passes" label={t("settingsDriveMotor.common.passes")} rules={[{ required: true }]}>
                                    <InputNumber min={1} max={10} step={1} precision={0} style={{ width: "100%" }} />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="apply" label={t("settingsDriveMotor.common.applyPersistIfSuccessful")} valuePropName="checked">
                                    <Switch
                                        checkedChildren={t("settingsDriveMotor.common.apply")}
                                        unCheckedChildren={t("settingsDriveMotor.common.reportOnly")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="allow_undock" label={t("settingsDriveMotor.common.allowUndockIfDocked")} valuePropName="checked">
                                    <Switch
                                        checkedChildren={t("settingsDriveMotor.common.allow")}
                                        unCheckedChildren={t("settingsDriveMotor.common.no")}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item
                                    name="undock_distance_m"
                                    label={t("settingsDriveMotor.common.undockDistance")}
                                    tooltip={t("settingsDriveMotor.common.undockDistanceTooltip")}
                                >
                                    <InputNumber min={0.5} max={5} step={0.1} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                    {pidApply ? (
                        <Alert
                            type="success"
                            showIcon
                            message={t("settingsDriveMotor.pidModal.resultApply.title")}
                            description={t("settingsDriveMotor.pidModal.resultApply.description")}
                        />
                    ) : (
                        <Alert
                            type="info"
                            showIcon
                            message={t("settingsDriveMotor.common.reportOnlyModeTitle")}
                            description={t("settingsDriveMotor.pidModal.resultReportOnly.description")}
                        />
                    )}
                </Space>
            </Modal>

            <Modal
                title={t("settingsDriveMotor.report.title")}
                open={reportOpen}
                onCancel={() => setReportOpen(false)}
                footer={<Button onClick={() => setReportOpen(false)}>{t("settingsDriveMotor.common.close")}</Button>}
                width={900}
            >
                {latestReport?.latest_report ? (
                    <Space direction="vertical" size={12} style={{ width: "100%" }}>
                        <Descriptions size="small" column={1} bordered>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.mode")}>{latestReport.latest_report.mode.toUpperCase()}</Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.generatedAt")}>{formatTimestamp(t, latestReport.latest_report.generated_at)}</Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.cmdVelTopic")}>
                                {latestCmdVelTopic ?? t("settingsDriveMotor.common.unknown")}
                            </Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.robotMass")}>
                                {formatReportNumber(t, latestParsedReport?.robot_mass_kg, 2)}
                                {hasFiniteNumber(latestParsedReport?.robot_mass_kg) ? ` ${t("settingsDriveMotor.common.units.kg")}` : ""}
                            </Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.internalTier")}>
                                {translatedInternalTier}
                            </Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.hardwareConfigPath")}>
                                {latestParsedReport?.hardware_config_path ? (
                                    <Text code copyable>{latestParsedReport.hardware_config_path}</Text>
                                ) : (
                                    t("settingsDriveMotor.common.unavailable")
                                )}
                            </Descriptions.Item>
                            <Descriptions.Item label={t("settingsDriveMotor.report.fields.path")}>
                                <Text code copyable>{latestReport.latest_report.report_path}</Text>
                            </Descriptions.Item>
                        </Descriptions>

                        {latestFailureMessage && (
                            <Alert
                                type="warning"
                                showIcon
                                message={t("settingsDriveMotor.report.failure.title")}
                                description={latestFailureMessage}
                            />
                        )}

                        {latestStatusSnapshot && (
                            <Descriptions size="small" column={2} bordered title={t("settingsDriveMotor.report.statusSnapshot.title")}>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.activeEmergency")}>
                                    {formatDriveTuningBoolean(t, latestStatusSnapshot.active_emergency)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.latchedEmergency")}>
                                    {formatDriveTuningBoolean(t, latestStatusSnapshot.latched_emergency)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.isCharging")}>
                                    {formatDriveTuningBoolean(t, latestStatusSnapshot.is_charging)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.mowerStatus")}>
                                    {latestStatusSnapshot.mower_status == null ? t("settingsDriveMotor.common.unknown") : latestStatusSnapshot.mower_status}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.escPower")}>
                                    {formatDriveTuningBoolean(t, latestStatusSnapshot.esc_power)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.wheelTickFactor")}>
                                    {latestStatusSnapshot.wheel_tick_factor == null
                                        ? t("settingsDriveMotor.common.unknown")
                                        : formatReportNumber(t, latestStatusSnapshot.wheel_tick_factor, 3)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.statusSnapshot.lastWheelTickTimestamp")} span={2}>
                                    {latestStatusSnapshot.last_wheel_tick_timestamp
                                        ? formatTimestamp(t, latestStatusSnapshot.last_wheel_tick_timestamp)
                                        : t("settingsDriveMotor.common.unknown")}
                                </Descriptions.Item>
                            </Descriptions>
                        )}

                        {latestDrivetrain && (
                            <Descriptions size="small" column={2} bordered title={t("settingsDriveMotor.report.drivetrain.title")}>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.wheelRadius")}>
                                    {formatReportNumber(t, latestDrivetrain.wheel_radius_m)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.wheelCircumference")}>
                                    {formatReportNumber(t, latestDrivetrain.wheel_circumference_m)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.wheelRevPerMeter")}>
                                    {formatReportNumber(t, latestDrivetrain.estimated_wheel_revolutions_per_meter)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.encoderCountsPerWheelRev")}>
                                    {formatReportNumber(t, latestDrivetrain.estimated_encoder_counts_per_wheel_revolution)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.configuredTicksPerRevolution")}>
                                    {formatReportNumber(t, latestDrivetrain.configured_ticks_per_revolution)}
                                </Descriptions.Item>
                                <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.gearboxRatio")}>
                                    {t("settingsDriveMotor.report.drivetrain.derivedEstimateUnavailable")}
                                </Descriptions.Item>
                                {!!latestDrivetrain.notes?.length && (
                                    <Descriptions.Item label={t("settingsDriveMotor.report.drivetrain.notes")} span={2}>
                                        {latestDrivetrain.notes.join(" | ")}
                                    </Descriptions.Item>
                                )}
                            </Descriptions>
                        )}

                        {latestParsedReport?.proposed_params && (
                            <Descriptions size="small" bordered column={2} title={t("settingsDriveMotor.report.proposedParams.title")}>
                                {Object.entries(latestParsedReport.proposed_params).map(([key, value]) => (
                                    <Descriptions.Item key={key} label={key}>
                                        {formatDriveParamValue(key, value)}
                                    </Descriptions.Item>
                                ))}
                            </Descriptions>
                        )}

                        {latestReasons.length > 0 && (
                            <Card size="small" title={t("settingsDriveMotor.report.recommendations.title")}>
                                <Space direction="vertical" size={6} style={{ width: "100%" }}>
                                    {latestReasons.map((reason, index) => (
                                        <Text key={`${index}-${reason}`}>{reason}</Text>
                                    ))}
                                </Space>
                            </Card>
                        )}

                        {latestTrials.length > 0 && (
                            <Collapse
                                items={[
                                    {
                                        key: "trial-summary",
                                        label: t("settingsDriveMotor.report.trials.title", { count: latestTrials.length }),
                                        children: (
                                            <Space direction="vertical" size={10} style={{ width: "100%" }}>
                                                {latestTrials.map((trial) => (
                                                    <Card key={trial.name} size="small">
                                                        <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                                            <Text strong>{trial.name}</Text>
                                                            <Text type="secondary">
                                                                {t("settingsDriveMotor.report.trials.line1", {
                                                                    phase: translateDriveTuningTrialPhase(t, trial.phase),
                                                                    target: trial.target_speed.toFixed(2),
                                                                    measured: trial.measured_speed_mean.toFixed(3),
                                                                })}
                                                            </Text>
                                                            <Text type="secondary">
                                                                {t("settingsDriveMotor.report.trials.line2", {
                                                                    overshoot: trial.overshoot.toFixed(3),
                                                                    settling: trial.settling_time == null
                                                                        ? t("settingsDriveMotor.common.na")
                                                                        : trial.settling_time.toFixed(3),
                                                                    stall: formatDriveTuningBoolean(t, trial.stall_detected),
                                                                    osc: formatDriveTuningBoolean(t, trial.oscillation_detected),
                                                                    liveOsc: formatDriveTuningBoolean(t, !!trial.live_oscillation_detected),
                                                                    quality: translateDriveTuningTrialQuality(t, trial.trial_quality),
                                                                })}
                                                            </Text>
                                                            {!!trial.warnings?.length && (
                                                                <Text type="warning">
                                                                    {t("settingsDriveMotor.report.trials.warningsPrefix")} {trial.warnings.join(" | ")}
                                                                </Text>
                                                            )}
                                                        </Space>
                                                    </Card>
                                                ))}
                                            </Space>
                                        ),
                                    },
                                ]}
                            />
                        )}

                        <Card size="small" title={t("settingsDriveMotor.report.rawYaml.title")}>
                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                {latestReport.raw_yaml?.trim() || t("settingsDriveMotor.report.rawYaml.empty")}
                            </pre>
                        </Card>
                    </Space>
                ) : (
                    <Alert type="info" showIcon message={t("settingsDriveMotor.report.empty")} />
                )}
            </Modal>
        </div>
    );
};
