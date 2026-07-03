import React, { useMemo, useState } from "react";
import { Alert, App, Button, Card, Collapse, Space, Tag, Typography } from "antd";
import { PlayCircleOutlined, ReloadOutlined, SaveOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { ContentType } from "../../api/Api.ts";
import { useApi } from "../../hooks/useApi.ts";

const { Paragraph, Text } = Typography;

type GnssActionName = "plan" | "apply" | "factory-reset-apply" | "restart";

type GnssCommandExecution = {
    tool?: string;
    command?: string[];
    exit_code?: number;
    stdout?: string;
    stderr?: string;
    success?: boolean;
};

type GnssActionResponse = {
    success?: boolean;
    partial_failure?: boolean;
    action?: string;
    message?: string;
    warnings?: string[];
    receiver_family?: string;
    profile?: string;
    signal_profile?: string;
    profile_rate_hz?: string;
    serial_device?: string;
    runtime_baud?: string;
    config_baud?: string;
    runtime_baud_differs_from_config?: boolean;
    runtime_baud_updated?: boolean;
    gps_container?: string;
    gps_image?: string;
    gps_container_was_running?: boolean;
    stop_attempted?: boolean;
    restart_attempted?: boolean;
    restart_succeeded?: boolean;
    restart_error?: string;
    executions?: GnssCommandExecution[];
};

type Props = {
    isDirty?: boolean;
    saving?: boolean;
    gpsRestarting?: boolean;
    onSave?: () => void | Promise<void>;
    onSaveAndRestartGps?: () => void | Promise<void>;
    onPersistBeforeAction?: () => Promise<boolean>;
    showSaveButtons?: boolean;
};

const GNSS_ACTION_LABEL_KEYS: Record<GnssActionName, string> = {
    plan: "settingsGnssReceiver.actionPlan",
    apply: "settingsGnssReceiver.actionApply",
    "factory-reset-apply": "settingsGnssReceiver.actionFactoryReset",
    restart: "settingsGnssReceiver.actionRestart",
};

const describeApiError = (error: unknown, unknownErrorText: string): string => {
    if (error && typeof error === "object") {
        const apiError = (error as any).error?.error;
        if (typeof apiError === "string" && apiError.trim()) {
            return apiError;
        }
        if (typeof (error as any).message === "string" && (error as any).message.trim()) {
            return (error as any).message;
        }
        if (typeof (error as any).statusText === "string" && (error as any).statusText.trim()) {
            return (error as any).statusText;
        }
    }
    return unknownErrorText;
};

const responseAlertType = (response: GnssActionResponse | null, errorMessage: string | null): "success" | "warning" | "error" | "info" => {
    if (errorMessage) {
        return "error";
    }
    if (!response) {
        return "info";
    }
    if (response.partial_failure) {
        return "warning";
    }
    if (response.success === false) {
        return "error";
    }
    if (response.success) {
        return "success";
    }
    return "info";
};

const formatCommand = (command?: string[]): string => {
    if (!command || command.length === 0) {
        return "";
    }
    return command.join(" ");
};

export const GnssReceiverActionsCard: React.FC<Props> = ({
    isDirty = false,
    saving = false,
    gpsRestarting = false,
    onSave,
    onSaveAndRestartGps,
    onPersistBeforeAction,
    showSaveButtons = false,
}) => {
    const guiApi = useApi();
    const { t } = useTranslation();
    const { notification, modal } = App.useApp();
    const [pendingAction, setPendingAction] = useState<GnssActionName | null>(null);
    const [lastResponse, setLastResponse] = useState<GnssActionResponse | null>(null);
    const [transportError, setTransportError] = useState<string | null>(null);

    const actionSummary = useMemo(() => {
        if (transportError) {
            return {
                type: "error" as const,
                message: t("settingsGnssReceiver.backendRequestFailed"),
                description: transportError,
            };
        }
        if (!lastResponse) {
            return null;
        }
        if (lastResponse.partial_failure) {
            return {
                type: "warning" as const,
                message: lastResponse.message || t("settingsGnssReceiver.summaryPartialFailure"),
                description: t("settingsGnssReceiver.summaryPartialFailureDesc"),
            };
        }
        if (lastResponse.success === false) {
            return {
                type: "error" as const,
                message: lastResponse.message || t("settingsGnssReceiver.summaryFailed"),
                description: t("settingsGnssReceiver.summaryFailedDesc"),
            };
        }
        return {
            type: "success" as const,
            message: lastResponse.message || t("settingsGnssReceiver.summarySuccess"),
            description: t("settingsGnssReceiver.summarySuccessDesc"),
        };
    }, [lastResponse, transportError, t]);

    const runAction = async (action: GnssActionName, body?: Record<string, any>) => {
        setPendingAction(action);
        setTransportError(null);

        try {
            if (onPersistBeforeAction) {
                const persisted = await onPersistBeforeAction();
                if (!persisted) {
                    setPendingAction(null);
                    return;
                }
            }

            const response = await guiApi.request<GnssActionResponse, { error?: string }>({
                path: `/settings/gnss/${action}`,
                method: "POST",
                body,
                type: ContentType.Json,
                format: "json",
            });

            const data = response.data ?? {};
            setLastResponse(data);

            const actionLabel = t(GNSS_ACTION_LABEL_KEYS[action]);

            if (data.partial_failure) {
                notification.warning({
                    message: data.message || t("settingsGnssReceiver.notifyPartialFailure", { action: actionLabel }),
                    description: t("settingsGnssReceiver.notifyPartialFailureDesc"),
                });
            } else if (data.success === false) {
                notification.error({
                    message: data.message || t("settingsGnssReceiver.notifyFailed", { action: actionLabel }),
                    description: t("settingsGnssReceiver.notifyFailedDesc"),
                });
            } else {
                notification.success({
                    message: data.message || t("settingsGnssReceiver.notifyCompleted", { action: actionLabel }),
                });
            }
        } catch (error) {
            const description = describeApiError(error, t("settingsGnssReceiver.unknownApiError"));
            setTransportError(description);
            notification.error({
                message: t("settingsGnssReceiver.notifyFailed", { action: t(GNSS_ACTION_LABEL_KEYS[action]) }),
                description,
            });
        } finally {
            setPendingAction(null);
        }
    };

    const confirmApply = () => {
        modal.confirm({
            title: t("settingsGnssReceiver.confirmApplyTitle"),
            content: (
                <Space direction="vertical" size={8}>
                    <Text>
                        {t("settingsGnssReceiver.confirmApplyBody")}
                    </Text>
                    <Text type="warning">
                        {t("settingsGnssReceiver.confirmApplyWarning")}
                    </Text>
                </Space>
            ),
            okText: t("settingsGnssReceiver.confirmApplyOk"),
            cancelText: t("settingsGnssReceiver.cancel"),
            onOk: () => runAction("apply", { confirm: true }),
        });
    };

    const confirmFactoryReset = () => {
        modal.confirm({
            title: t("settingsGnssReceiver.confirmFactoryResetTitle"),
            content: (
                <Space direction="vertical" size={8}>
                    <Text strong type="danger">
                        {t("settingsGnssReceiver.confirmFactoryResetDanger")}
                    </Text>
                    <Text>
                        {t("settingsGnssReceiver.confirmFactoryResetBody")}
                    </Text>
                    <Text type="warning">
                        {t("settingsGnssReceiver.confirmFactoryResetWarning")}
                    </Text>
                </Space>
            ),
            okText: t("settingsGnssReceiver.confirmFactoryResetOk"),
            okType: "danger",
            cancelText: t("settingsGnssReceiver.cancel"),
            maskClosable: false,
            onOk: () => runAction("factory-reset-apply", { confirm_factory_reset: true }),
        });
    };

    const loadingActionLabel = pendingAction ? t(GNSS_ACTION_LABEL_KEYS[pendingAction]) : "";
    const actionDisabled = Boolean(pendingAction) || saving || gpsRestarting;

    return (
        <Card size="small" title={t("settingsGnssReceiver.cardTitle")} style={{ marginBottom: 16 }}>
            <Space wrap size={[8, 8]}>
                {showSaveButtons && onSave && (
                    <Button
                        type="primary"
                        icon={<SaveOutlined />}
                        onClick={onSave}
                        loading={saving && !gpsRestarting && !pendingAction}
                        disabled={actionDisabled || !isDirty}
                    >
                        {t("settingsGnssReceiver.saveSettings")}
                    </Button>
                )}
                <Button
                    icon={<PlayCircleOutlined />}
                    onClick={() => runAction("plan")}
                    loading={pendingAction === "plan"}
                    disabled={actionDisabled}
                >
                    {t("settingsGnssReceiver.actionPlan")}
                </Button>
                <Button
                    onClick={confirmApply}
                    loading={pendingAction === "apply"}
                    disabled={actionDisabled}
                >
                    {t("settingsGnssReceiver.actionApply")}
                </Button>
                {showSaveButtons && onSaveAndRestartGps && (
                    <Button
                        icon={<ReloadOutlined />}
                        onClick={onSaveAndRestartGps}
                        loading={gpsRestarting && !pendingAction}
                        disabled={Boolean(pendingAction) || saving || gpsRestarting}
                    >
                        {t("settingsGnssReceiver.saveAndRestartGps")}
                    </Button>
                )}
                <Button
                    icon={<ReloadOutlined />}
                    onClick={() => runAction("restart", {})}
                    loading={pendingAction === "restart"}
                    disabled={actionDisabled}
                >
                    {t("settingsGnssReceiver.actionRestart")}
                </Button>
                <Button
                    danger
                    onClick={confirmFactoryReset}
                    loading={pendingAction === "factory-reset-apply"}
                    disabled={actionDisabled}
                >
                    {t("settingsGnssReceiver.actionFactoryReset")}
                </Button>
            </Space>

            <Alert
                type={isDirty ? "warning" : "info"}
                showIcon
                style={{ marginTop: 12 }}
                message={isDirty
                    ? t("settingsGnssReceiver.dirtyAlertMessage")
                    : t("settingsGnssReceiver.savedAlertMessage")}
                description={isDirty
                    ? t("settingsGnssReceiver.dirtyAlertDescription")
                    : t("settingsGnssReceiver.savedAlertDescription")}
            />

            {pendingAction && (
                <Alert
                    type="info"
                    showIcon
                    style={{ marginTop: 12 }}
                    message={t("settingsGnssReceiver.actionInProgress", { action: loadingActionLabel })}
                    description={t("settingsGnssReceiver.actionInProgressDesc")}
                />
            )}

            {actionSummary && (
                <Alert
                    type={actionSummary.type}
                    showIcon
                    style={{ marginTop: 12 }}
                    message={actionSummary.message}
                    description={actionSummary.description}
                />
            )}

            {(lastResponse || transportError) && (
                <Space direction="vertical" size={12} style={{ width: "100%", marginTop: 12 }}>
                    {lastResponse && (
                        <Card size="small" type="inner" title={t("settingsGnssReceiver.backendResult")}>
                            <Space wrap size={[8, 8]} style={{ marginBottom: 12 }}>
                                <Tag color={responseAlertType(lastResponse, null) === "success" ? "success" : responseAlertType(lastResponse, null) === "warning" ? "warning" : "error"}>
                                    {lastResponse.partial_failure
                                        ? t("settingsGnssReceiver.tagPartialFailure")
                                        : lastResponse.success
                                            ? t("settingsGnssReceiver.tagSuccess")
                                            : t("settingsGnssReceiver.tagFailed")}
                                </Tag>
                                {lastResponse.restart_attempted && (
                                    <Tag color={lastResponse.restart_succeeded ? "success" : "error"}>
                                        {lastResponse.restart_succeeded ? t("settingsGnssReceiver.tagGpsRestarted") : t("settingsGnssReceiver.tagGpsRestartFailed")}
                                    </Tag>
                                )}
                                {lastResponse.runtime_baud_updated && (
                                    <Tag color="processing">{t("settingsGnssReceiver.tagRuntimeBaudUpdated")}</Tag>
                                )}
                                {lastResponse.runtime_baud_differs_from_config && (
                                    <Tag color="warning">{t("settingsGnssReceiver.tagBaudMismatch")}</Tag>
                                )}
                            </Space>

                            <Space direction="vertical" size={4} style={{ width: "100%" }}>
                                {lastResponse.receiver_family && (
                                    <Text><Text strong>{t("settingsGnssReceiver.fieldReceiverFamily")}</Text> {lastResponse.receiver_family}</Text>
                                )}
                                {lastResponse.profile && (
                                    <Text><Text strong>{t("settingsGnssReceiver.fieldProfile")}</Text> {lastResponse.profile}</Text>
                                )}
                                {lastResponse.signal_profile && (
                                    <Text><Text strong>{t("settingsGnssReceiver.fieldSignalProfile")}</Text> {lastResponse.signal_profile}</Text>
                                )}
                                {lastResponse.serial_device && (
                                    <Text><Text strong>{t("settingsGnssReceiver.fieldSerialDevice")}</Text> {lastResponse.serial_device}</Text>
                                )}
                                {(lastResponse.runtime_baud || lastResponse.config_baud) && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldBaud")}</Text> {t("settingsGnssReceiver.baudRuntimeConfigured", { runtime: lastResponse.runtime_baud ?? t("settingsGnssReceiver.unknownValue"), configured: lastResponse.config_baud ?? t("settingsGnssReceiver.unknownValue") })}
                                    </Text>
                                )}
                                {lastResponse.restart_error && (
                                    <Text type="danger"><Text strong>{t("settingsGnssReceiver.fieldRestartError")}</Text> {lastResponse.restart_error}</Text>
                                )}
                            </Space>

                            {lastResponse.warnings && lastResponse.warnings.length > 0 && (
                                <Alert
                                    type="warning"
                                    showIcon
                                    style={{ marginTop: 12 }}
                                    message={t("settingsGnssReceiver.backendWarnings")}
                                    description={(
                                        <ul style={{ margin: 0, paddingLeft: 18 }}>
                                            {lastResponse.warnings.map((warning) => (
                                                <li key={warning}>{warning}</li>
                                            ))}
                                        </ul>
                                    )}
                                />
                            )}
                        </Card>
                    )}

                    {lastResponse?.executions && lastResponse.executions.length > 0 && (
                        <Collapse
                            size="small"
                            items={lastResponse.executions.map((execution, index) => ({
                                key: `${execution.tool ?? "command"}-${index}`,
                                label: (
                                    <Space wrap size={[8, 8]}>
                                        <Text strong>{execution.tool ?? t("settingsGnssReceiver.commandIndex", { index: index + 1 })}</Text>
                                        <Tag color={execution.success ? "success" : "error"}>
                                            {t("settingsGnssReceiver.exitCode", { code: execution.exit_code ?? "?" })}
                                        </Tag>
                                    </Space>
                                ),
                                children: (
                                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                                        <div>
                                            <Text strong>{t("settingsGnssReceiver.commandSummary")}</Text>
                                            <Paragraph
                                                code
                                                style={{
                                                    marginBottom: 0,
                                                    marginTop: 4,
                                                    whiteSpace: "pre-wrap",
                                                    wordBreak: "break-word",
                                                }}
                                            >
                                                {formatCommand(execution.command)}
                                            </Paragraph>
                                        </div>
                                        {execution.stdout && (
                                            <details>
                                                <summary>stdout</summary>
                                                <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", marginTop: 8 }}>
                                                    {execution.stdout}
                                                </pre>
                                            </details>
                                        )}
                                        {execution.stderr && (
                                            <details>
                                                <summary>stderr</summary>
                                                <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", marginTop: 8 }}>
                                                    {execution.stderr}
                                                </pre>
                                            </details>
                                        )}
                                    </Space>
                                ),
                            }))}
                        />
                    )}

                    {transportError && (
                        <Card size="small" type="inner" title={t("settingsGnssReceiver.transportError")}>
                            <pre style={{ whiteSpace: "pre-wrap", wordBreak: "break-word", margin: 0 }}>
                                {transportError}
                            </pre>
                        </Card>
                    )}
                </Space>
            )}
        </Card>
    );
};
