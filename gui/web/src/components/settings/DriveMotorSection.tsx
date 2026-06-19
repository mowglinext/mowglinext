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
import { useDockingSensor } from "../../hooks/useDockingSensor.ts";
import { useDriveTuning } from "../../hooks/useDriveTuning.ts";
import { useEmergency } from "../../hooks/useEmergency.ts";
import { useStatus } from "../../hooks/useStatus.ts";

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

const statusTag = (status: "not_validated" | "validated" | "warning") => {
    if (status === "validated") {
        return <Tag color="success">validated</Tag>;
    }
    if (status === "warning") {
        return <Tag color="warning">warning</Tag>;
    }
    return <Tag>not validated</Tag>;
};

const jobTag = (state?: string) => {
    if (state === "running") {
        return <Tag color="processing">running</Tag>;
    }
    if (state === "succeeded") {
        return <Tag color="success">completed</Tag>;
    }
    if (state === "warning") {
        return <Tag color="warning">completed with warning</Tag>;
    }
    if (state === "failed") {
        return <Tag color="error">failed</Tag>;
    }
    return <Tag>idle</Tag>;
};

const formatTimestamp = (value?: string) => {
    if (!value) return "Never";
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
                        message: "Drive tuning applied",
                        description: "The recommended drive parameters were persisted into mowgli_robot.yaml and synced in Settings.",
                    });
                }
            } catch {
                // The status panel still shows the backend error; keep this effect best-effort.
            }
        })();

        return () => {
            cancelled = true;
        };
    }, [acceptPersistedValues, loadLatestReport, notification, tuningStatus?.job]);

    const helperAlerts = useMemo(() => {
        const alerts: React.ReactNode[] = [];
        if (isEmergencyActive) {
            alerts.push(
                <Alert
                    key="emergency"
                    type="error"
                    showIcon
                    message="Emergency active"
                    description="Calibration assistants are blocked while emergency is active or latched."
                />,
            );
        }
        if (onDock) {
            alerts.push(
                <Alert
                    key="dock"
                    type="warning"
                    showIcon
                    message="Robot appears to be on the dock"
                    description="The assistants will refuse to move unless you explicitly enable the undock option in the launch dialog."
                />,
            );
        }
        if (tuningError) {
            alerts.push(
                <Alert
                    key="backend"
                    type="warning"
                    showIcon
                    message="Drive tuning backend warning"
                    description={tuningError}
                />,
            );
        }
        return alerts;
    }, [isEmergencyActive, onDock, tuningError]);

    const openLatestReport = async () => {
        try {
            await loadLatestReport();
            setReportOpen(true);
        } catch (e: any) {
            notification.error({
                message: "Failed to load the last drive tuning report",
                description: e?.error?.error ?? e?.message ?? "Unknown backend error",
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
                passes: values.passes,
                auto_turn: values.auto_turn,
                turn_direction: "right",
                apply: values.apply,
                allow_undock: values.allow_undock,
                undock_distance_m: values.undock_distance_m,
            });
            setFfOpen(false);
            notification.success({
                message: "Odometry / feed-forward calibration started",
                description: values.apply
                    ? "The calibration runs through /cmd_vel_teleop like the IMU calibration tool, then applies the recommendation live only if the run completes successfully."
                    : "This recommendation-only run uses the same /cmd_vel_teleop -> twist_mux path as IMU calibration and will not persist any parameter.",
            });
        } catch (e: any) {
            if (e?.errorFields) {
                return;
            }
            notification.error({
                message: "Failed to start odometry / feed-forward calibration",
                description: e?.error?.error ?? e?.message ?? "Unknown backend error",
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
                message: "PID auto-tune started",
                description: values.apply
                    ? "The autotune runs through the same /cmd_vel_teleop path as IMU calibration and will persist the final recommendation only after a successful run."
                    : "This run is recommendation-only and will restore the original live parameters afterwards.",
            });
        } catch (e: any) {
            if (e?.errorFields) {
                return;
            }
            notification.error({
                message: "Failed to start PID auto-tune",
                description: e?.error?.error ?? e?.message ?? "Unknown backend error",
            });
        } finally {
            setStartingAction(null);
        }
    };

    const confirmRollback = () => {
        modal.confirm({
            title: "Rollback the last drive tuning?",
            content: (
                <Space direction="vertical" size={8}>
                    <Text>
                        This restores the last backup generated by <Text code>mowgli_tools tune_drive_pid</Text>, applies it live to
                        <Text code> hardware_bridge</Text>, and writes the restored values back to <Text code>mowgli_robot.yaml</Text>.
                    </Text>
                    <Text type="warning">
                        Use this only when you want to undo the last accepted tuning result.
                    </Text>
                </Space>
            ),
            okText: "Rollback tuning",
            okType: "danger",
            cancelText: "Cancel",
            onOk: async () => {
                try {
                    setRollingBack(true);
                    const response = await rollback();
                    if (response?.restored) {
                        acceptPersistedValues?.(response.restored);
                    }
                    notification.success({
                        message: response?.message ?? "Drive tuning rollback completed",
                    });
                } catch (e: any) {
                    notification.error({
                        message: "Failed to rollback drive tuning",
                        description: e?.error?.error ?? e?.message ?? "Unknown backend error",
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

    return (
        <div>
            <Alert
                type="info"
                showIcon
                style={{ marginBottom: 16 }}
                message="Saved and applied live"
                description="Saving stores these in mowgli_robot.yaml and pushes them to the drive controller immediately — no ROS2 restart needed. They persist across reboots: on every boot the robot re-sends the saved values to the firmware. (The firmware itself has no storage, so it runs its built-in defaults only for the brief moment before the controller reconnects, then re-validates and clamps the values you saved.)"
            />

            <Card
                size="small"
                title="Calibration assistants"
                style={{ marginBottom: 16 }}
                extra={
                    <Space wrap>
                        <Button
                            type="primary"
                            icon={<PlayCircleOutlined />}
                            onClick={() => setFfOpen(true)}
                            disabled={runningJob || isEmergencyActive}
                        >
                            Calibrate odometry / feed-forward
                        </Button>
                        <Button
                            icon={<PlayCircleOutlined />}
                            onClick={() => setPidOpen(true)}
                            disabled={runningJob || isEmergencyActive}
                        >
                            Auto-tune drive PID
                        </Button>
                    </Space>
                }
            >
                <Space direction="vertical" size={12} style={{ width: "100%" }}>
                    {helperAlerts}

                    <Alert
                        type="info"
                        showIcon
                        message="Conservative motion profile"
                        description="The assistants reuse the IMU calibration motion path: stamped Twist commands on /cmd_vel_teleop, routed through twist_mux to /cmd_vel, with soft ramps and cmd_vel = 0 between motion segments."
                    />

                    <Descriptions size="small" column={1} bordered>
                        <Descriptions.Item label="Odometry / feed-forward">
                            <Space wrap>
                                {statusTag(feedForwardSummary?.status ?? "not_validated")}
                                <Text type="secondary">{feedForwardSummary?.message ?? "No report yet."}</Text>
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label="PID auto-tune">
                            <Space wrap>
                                {statusTag(pidSummary?.status ?? "not_validated")}
                                <Text type="secondary">{pidSummary?.message ?? "No report yet."}</Text>
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label="Current job">
                            <Space wrap>
                                {jobTag(tuningStatus?.job?.state)}
                                {tuningStatus?.job && (
                                    <Text type="secondary">
                                        {tuningStatus.job.mode.toUpperCase()} started {formatTimestamp(tuningStatus.job.started_at)}
                                    </Text>
                                )}
                            </Space>
                        </Descriptions.Item>
                        <Descriptions.Item label="Last report date">
                            <Text>{formatTimestamp(latestReportMeta?.generated_at)}</Text>
                        </Descriptions.Item>
                        <Descriptions.Item label="Last report path">
                            <Text code copyable={!!latestReportMeta?.report_path}>
                                {latestReportMeta?.report_path ?? "No report yet"}
                            </Text>
                        </Descriptions.Item>
                    </Descriptions>

                    <Space wrap>
                        <Button type="primary" onClick={() => setFfOpen(true)} disabled={runningJob || isEmergencyActive}>
                            Start odometry/feed-forward calibration
                        </Button>
                        <Button onClick={() => setPidOpen(true)} disabled={runningJob || isEmergencyActive}>
                            Start PID auto-tune
                        </Button>
                        <Button icon={<FileTextOutlined />} onClick={openLatestReport} loading={loadingLatestReport}>
                            View last report
                        </Button>
                        <Button
                            danger
                            icon={<HistoryOutlined />}
                            onClick={confirmRollback}
                            disabled={runningJob}
                            loading={rollingBack}
                        >
                            Rollback last tuning
                        </Button>
                    </Space>

                    {tuningStatus?.job && (
                        <Collapse
                            items={[
                                {
                                    key: "job-log",
                                    label: `Live job output (${tuningStatus.job.mode.toUpperCase()})`,
                                    children: (
                                        <Space direction="vertical" size={8} style={{ width: "100%" }}>
                                            {tuningStatus.job.error && (
                                                <Alert
                                                    type={tuningStatus.job.state === "failed" ? "error" : "warning"}
                                                    showIcon
                                                    message={tuningStatus.job.error}
                                                />
                                            )}
                                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                                {tuningStatus.job.logs?.trim() || "Waiting for output..."}
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
                            Wheel Velocity PID
                        </Text>
                        <Paragraph type="secondary" style={{ margin: "4px 0 0" }}>
                            Closed-loop gains the STM32 firmware uses to track each wheel's commanded
                            speed. Higher Kp/Ki give stiffer tracking; too high causes oscillation or
                            overshoot. Kd is normally 0.
                        </Paragraph>
                    </div>
                    <Form layout="vertical" size="small">
                        <Row gutter={[16, 0]}>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kp" tooltip="Proportional gain (PWM per m/s)">
                                    <InputNumber
                                        value={values.wheel_pid_kp}
                                        onChange={(v) => onChange("wheel_pid_kp", v)}
                                        min={0} max={200} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Ki" tooltip="Integral gain (PWM per m/s·s) — bridges the static-friction deadband">
                                    <InputNumber
                                        value={values.wheel_pid_ki}
                                        onChange={(v) => onChange("wheel_pid_ki", v)}
                                        min={0} max={20000} step={100} precision={0}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Kd" tooltip="Derivative gain (PWM per m/s²) — 0 disables">
                                    <InputNumber
                                        value={values.wheel_pid_kd}
                                        onChange={(v) => onChange("wheel_pid_kd", v)}
                                        min={0} max={500} step={1} precision={2}
                                        style={{ width: "100%" }}
                                    />
                                </Form.Item>
                            </Col>
                            <Col xs={12} sm={8}>
                                <Form.Item label="Integral Limit" tooltip="Anti-windup clamp on the integral term (PWM, motor max 255)">
                                    <InputNumber
                                        value={values.wheel_pid_integral_limit}
                                        onChange={(v) => onChange("wheel_pid_integral_limit", v)}
                                        min={0} max={255} step={5} precision={0}
                                        style={{ width: "100%" }} addonAfter="PWM"
                                    />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                </Space>
            </Card>

            <Card size="small" title="Feedforward" style={{ marginBottom: 16 }}>
                <Form layout="vertical" size="small">
                    <Row gutter={[16, 0]}>
                        <Col xs={12} sm={8}>
                            <Form.Item
                                label="PWM per m/s"
                                tooltip="Open-loop velocity→PWM feedforward scale. Dominant drive term; also sets the idle/deadband mapping. Change with care."
                            >
                                <InputNumber
                                    value={values.wheel_pid_pwm_per_mps}
                                    onChange={(v) => onChange("wheel_pid_pwm_per_mps", v)}
                                    min={50} max={600} step={10} precision={0}
                                    style={{ width: "100%" }} addonAfter="PWM"
                                />
                            </Form.Item>
                        </Col>
                    </Row>
                </Form>
            </Card>

            <Modal
                title="Calibrate odometry / feed-forward"
                open={ffOpen}
                onCancel={() => setFfOpen(false)}
                onOk={handleStartFeedForward}
                okText="Start calibration"
                cancelText="Cancel"
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
                            message="Robot is on the dock"
                            description="Enable undock explicitly below if you want the assistant to reverse out before starting."
                        />
                    )}
                    <Alert
                        type="info"
                        showIcon
                        message="Workflow"
                        description="The robot runs straight passes through /cmd_vel, compares wheel odometry against RTK/GPS when available, refines ticks_per_meter and wheel_pid_pwm_per_mps, then writes recommendations to a YAML report. Apply is optional."
                    />
                    <Form
                        form={ffForm}
                        layout="vertical"
                        initialValues={{
                            distance_m: 3,
                            test_speed_mps: 0.3,
                            passes: 3,
                            auto_turn: true,
                            apply: false,
                            allow_undock: false,
                            undock_distance_m: 2,
                        }}
                    >
                        <Row gutter={[16, 0]}>
                            <Col xs={24} sm={12}>
                                <Form.Item name="distance_m" label="Distance" rules={[{ required: true }]}>
                                    <InputNumber min={2} max={10} step={0.5} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="test_speed_mps" label="Test speed" rules={[{ required: true }]}>
                                    <InputNumber min={0.05} max={0.5} step={0.05} precision={2} style={{ width: "100%" }} addonAfter="m/s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="passes" label="Passes" rules={[{ required: true }]}>
                                    <InputNumber min={1} max={10} step={1} precision={0} style={{ width: "100%" }} />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="auto_turn" label="Auto U-turn" valuePropName="checked">
                                    <Switch checkedChildren="Enabled" unCheckedChildren="Disabled" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="apply" label="Apply and persist if successful" valuePropName="checked">
                                    <Switch checkedChildren="Apply" unCheckedChildren="Report only" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="allow_undock" label="Allow undock if docked" valuePropName="checked">
                                    <Switch checkedChildren="Allow" unCheckedChildren="No" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="undock_distance_m" label="Undock distance" tooltip="Used only when undock is enabled.">
                                    <InputNumber min={0.5} max={5} step={0.1} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                    {ffApply ? (
                        <Alert
                            type="success"
                            showIcon
                            message="Successful run will persist the recommended feed-forward values"
                            description="ticks_per_meter and wheel_pid_pwm_per_mps will be saved into mowgli_robot.yaml and kept live in hardware_bridge."
                        />
                    ) : (
                        <Alert
                            type="info"
                            showIcon
                            message="Recommendation-only mode"
                            description="The assistant will generate a YAML report and restore the original live drive parameters afterwards."
                        />
                    )}
                </Space>
            </Modal>

            <Modal
                title="Auto-tune drive PID"
                open={pidOpen}
                onCancel={() => setPidOpen(false)}
                onOk={handleStartPid}
                okText="Start auto-tune"
                cancelText="Cancel"
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
                            message="Feed-forward / odometry not validated yet"
                            description="PID auto-tune should run only after ticks_per_meter and wheel_pid_pwm_per_mps were validated. You can still continue, but the recommendation quality may be poor."
                        />
                    )}
                    {onDock && !pidAllowUndock && (
                        <Alert
                            type="warning"
                            showIcon
                            message="Robot is on the dock"
                            description="Enable undock explicitly below if you want the PID auto-tune to leave the dock before running."
                        />
                    )}
                    <Alert
                        type="info"
                        showIcon
                        message="Teleop-based step response"
                        description="This autotune follows the same /cmd_vel_teleop -> twist_mux path as the IMU calibration tool, using conservative ramps: 0 → 0.2, 0.2 → 0.3, 0.3 → 0.1, 0.1 → 0. The backend monitors overshoot, settling, oscillation, integral saturation, and wheel imbalance."
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
                                <Form.Item name="max_speed_mps" label="Maximum speed" rules={[{ required: true }]}>
                                    <InputNumber min={0.1} max={0.5} step={0.05} precision={2} style={{ width: "100%" }} addonAfter="m/s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="segment_duration_s" label="Segment duration" rules={[{ required: true }]}>
                                    <InputNumber min={2} max={20} step={0.5} precision={1} style={{ width: "100%" }} addonAfter="s" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="passes" label="Passes" rules={[{ required: true }]}>
                                    <InputNumber min={1} max={10} step={1} precision={0} style={{ width: "100%" }} />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="apply" label="Apply and persist if successful" valuePropName="checked">
                                    <Switch checkedChildren="Apply" unCheckedChildren="Report only" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="allow_undock" label="Allow undock if docked" valuePropName="checked">
                                    <Switch checkedChildren="Allow" unCheckedChildren="No" />
                                </Form.Item>
                            </Col>
                            <Col xs={24} sm={12}>
                                <Form.Item name="undock_distance_m" label="Undock distance" tooltip="Used only when undock is enabled.">
                                    <InputNumber min={0.5} max={5} step={0.1} precision={1} style={{ width: "100%" }} addonAfter="m" />
                                </Form.Item>
                            </Col>
                        </Row>
                    </Form>
                    {pidApply ? (
                        <Alert
                            type="success"
                            showIcon
                            message="Successful run will persist the recommended PID gains"
                            description="wheel_pid_kp, wheel_pid_ki, wheel_pid_kd, and wheel_pid_integral_limit will be saved into mowgli_robot.yaml and kept live in hardware_bridge."
                        />
                    ) : (
                        <Alert
                            type="info"
                            showIcon
                            message="Recommendation-only mode"
                            description="The assistant will restore the original live wheel PID gains after the report is generated."
                        />
                    )}
                </Space>
            </Modal>

            <Modal
                title="Last drive tuning report"
                open={reportOpen}
                onCancel={() => setReportOpen(false)}
                footer={<Button onClick={() => setReportOpen(false)}>Close</Button>}
                width={900}
            >
                {latestReport?.latest_report ? (
                    <Space direction="vertical" size={12} style={{ width: "100%" }}>
                        <Descriptions size="small" column={1} bordered>
                            <Descriptions.Item label="Report mode">{latestReport.latest_report.mode.toUpperCase()}</Descriptions.Item>
                            <Descriptions.Item label="Generated at">{formatTimestamp(latestReport.latest_report.generated_at)}</Descriptions.Item>
                            <Descriptions.Item label="cmd_vel topic">{latestCmdVelTopic ?? "Unknown"}</Descriptions.Item>
                            <Descriptions.Item label="Path">
                                <Text code copyable>{latestReport.latest_report.report_path}</Text>
                            </Descriptions.Item>
                        </Descriptions>

                        {latestFailureMessage && (
                            <Alert
                                type="warning"
                                showIcon
                                message="Last run ended with a movement fault"
                                description={latestFailureMessage}
                            />
                        )}

                        {latestStatusSnapshot && (
                            <Descriptions size="small" column={2} bordered title="Status snapshot">
                                <Descriptions.Item label="active_emergency">
                                    {latestStatusSnapshot.active_emergency == null ? "Unknown" : String(latestStatusSnapshot.active_emergency)}
                                </Descriptions.Item>
                                <Descriptions.Item label="latched_emergency">
                                    {latestStatusSnapshot.latched_emergency == null ? "Unknown" : String(latestStatusSnapshot.latched_emergency)}
                                </Descriptions.Item>
                                <Descriptions.Item label="is_charging">
                                    {latestStatusSnapshot.is_charging == null ? "Unknown" : String(latestStatusSnapshot.is_charging)}
                                </Descriptions.Item>
                                <Descriptions.Item label="mower_status">
                                    {latestStatusSnapshot.mower_status == null ? "Unknown" : latestStatusSnapshot.mower_status}
                                </Descriptions.Item>
                                <Descriptions.Item label="esc_power">
                                    {latestStatusSnapshot.esc_power == null ? "Unknown" : String(latestStatusSnapshot.esc_power)}
                                </Descriptions.Item>
                                <Descriptions.Item label="wheel_tick_factor">
                                    {latestStatusSnapshot.wheel_tick_factor == null ? "Unknown" : latestStatusSnapshot.wheel_tick_factor}
                                </Descriptions.Item>
                                <Descriptions.Item label="last_wheel_tick_timestamp" span={2}>
                                    {latestStatusSnapshot.last_wheel_tick_timestamp
                                        ? formatTimestamp(latestStatusSnapshot.last_wheel_tick_timestamp)
                                        : "Unknown"}
                                </Descriptions.Item>
                            </Descriptions>
                        )}

                        {latestParsedReport?.proposed_params && (
                            <Descriptions size="small" bordered column={2} title="Proposed parameters">
                                {Object.entries(latestParsedReport.proposed_params).map(([key, value]) => (
                                    <Descriptions.Item key={key} label={key}>
                                        {value}
                                    </Descriptions.Item>
                                ))}
                            </Descriptions>
                        )}

                        {latestReasons.length > 0 && (
                            <Card size="small" title="Recommendations and notes">
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
                                        label: `Trial summary (${latestTrials.length})`,
                                        children: (
                                            <Space direction="vertical" size={10} style={{ width: "100%" }}>
                                                {latestTrials.map((trial) => (
                                                    <Card key={trial.name} size="small">
                                                        <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                                            <Text strong>{trial.name}</Text>
                                                            <Text type="secondary">
                                                                {trial.phase} | target {trial.target_speed.toFixed(2)} m/s | measured {trial.measured_speed_mean.toFixed(3)} m/s
                                                            </Text>
                                                            <Text type="secondary">
                                                                overshoot {trial.overshoot.toFixed(3)} | settling {trial.settling_time ?? "n/a"} | stall {String(trial.stall_detected)} | osc {String(trial.oscillation_detected)}
                                                            </Text>
                                                        </Space>
                                                    </Card>
                                                ))}
                                            </Space>
                                        ),
                                    },
                                ]}
                            />
                        )}

                        <Card size="small" title="Raw YAML">
                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                {latestReport.raw_yaml?.trim() || "No YAML payload available."}
                            </pre>
                        </Card>
                    </Space>
                ) : (
                    <Alert type="info" showIcon message="No drive tuning report available yet." />
                )}
            </Modal>
        </div>
    );
};
