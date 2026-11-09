import {useAsyncHandler} from "../../hooks/useAsyncHandler";
import React, { useEffect, useMemo, useRef, useState } from "react";
import { Alert, App, Button, Card, Collapse, Select, Space, Tag, Typography } from "antd";
import { PlayCircleOutlined, ReloadOutlined, SaveOutlined } from "@ant-design/icons";
import { useTranslation } from "react-i18next";
import { ContentType } from "../../api/Api.ts";
import { useApi } from "../../hooks/useApi.ts";
import { GNSS_EXECUTION_BAUD_OPTIONS, normalizeGnssString } from "./gnssConfig.ts";

import {decodeGnssActionResponse, gnssActionOutcome, type GnssActionName, type GnssActionResponse} from '../../utils/gnssConfigContract';
import {requestErrorMessage} from '../../utils/errors';

const { Paragraph, Text } = Typography;

type Props = {
    isDirty?: boolean;
    saving?: boolean;
    gpsRestarting?: boolean;
    onSave?: () => void | Promise<void>;
    onSaveAndRestartGps?: () => void | Promise<void>;
    onPersistBeforeAction?: () => Promise<boolean>;
    showSaveButtons?: boolean;
    manualCurrentBaud?: unknown;
    onManualCurrentBaudChange?: (value: string) => void;
};

const GNSS_ACTION_LABEL_KEYS: Record<GnssActionName, string> = {
    plan: "settingsGnssReceiver.actionPlan",
    apply: "settingsGnssReceiver.actionApply",
    "factory-reset-apply": "settingsGnssReceiver.actionFactoryReset",
    restart: "settingsGnssReceiver.actionRestart",
};

const responseAlertType = (response: GnssActionResponse | null): "success" | "warning" | "error" | "info" => {
    if (!response) return 'info';
    const outcome = gnssActionOutcome(response);
    return outcome === 'partial' || outcome === 'unconfirmed' ? 'warning' : outcome === 'failed' ? 'error' : 'success';
};

const formatCommand = (command?: string[]): string => {
    if (!command || command.length === 0) {
        return "";
    }
    return command.join(" ");
};

const formatBaudValue = (value: string | undefined, autoLabel: string, unknownLabel: string): string => {
    if (!value || !value.trim()) {
        return unknownLabel;
    }
    return value === "auto" ? autoLabel : value;
};

export const GnssReceiverActionsCard: React.FC<Props> = ({
    isDirty = false,
    saving = false,
    gpsRestarting = false,
    onSave,
    onSaveAndRestartGps,
    onPersistBeforeAction,
    showSaveButtons = false,
    manualCurrentBaud,
    onManualCurrentBaudChange,
}) => {
    const asyncHandler = useAsyncHandler();
    const guiApi = useApi();
    const { t } = useTranslation();
    const { notification, modal } = App.useApp();
    const [pendingAction, setPendingAction] = useState<GnssActionName | null>(null);
    const [lastResponse, setLastResponse] = useState<GnssActionResponse | null>(null);
    const [transportError, setTransportError] = useState<string | null>(null);
    const [lastAction, setLastAction] = useState<GnssActionName | null>(null);
    const [dispatched, setDispatched] = useState(false);
    const active = useRef(true);
    const actionLock = useRef(false);
    useEffect(() => {
        active.current = true;
        return () => { active.current = false; };
    }, []);

    const actionSummary = useMemo(() => {
        if (transportError) {
            return {
                type: "error" as const,
                message: t(dispatched ? "settingsGnssReceiver.operationUnconfirmed" : "settingsGnssReceiver.persistenceUnconfirmed"),
                description: dispatched ? `${transportError} ${t("settingsGnssReceiver.operationUnconfirmedDesc")}` : transportError,
            };
        }
        if (!lastResponse) {
            return null;
        }
        if (gnssActionOutcome(lastResponse) === 'partial') {
            return {
                type: "warning" as const,
                message: lastResponse.message || t("settingsGnssReceiver.summaryPartialFailure"),
                description: t("settingsGnssReceiver.summaryPartialFailureDesc"),
            };
        }
        if (gnssActionOutcome(lastResponse) === 'failed') {
            return {
                type: "error" as const,
                message: lastResponse.message || t("settingsGnssReceiver.summaryFailed"),
                description: t("settingsGnssReceiver.summaryFailedDesc"),
            };
        }
        if (gnssActionOutcome(lastResponse) === 'unconfirmed') return {
            type: 'warning' as const, message: t('settingsGnssReceiver.restartUnconfirmed'),
            description: t('settingsGnssReceiver.operationUnconfirmedDesc'),
        };
        return {
            type: "success" as const,
            message: lastResponse.message || t("settingsGnssReceiver.summarySuccess"),
            description: t(lastAction === "plan" ? "settingsGnssReceiver.planOnlyDesc" : "settingsGnssReceiver.summarySuccessDesc"),
        };
    }, [lastResponse, transportError, dispatched, lastAction, t]);

    const runAction = async (action: GnssActionName, body?: {confirm?: boolean; confirm_factory_reset?: boolean}) => {
        // React state alone cannot prevent two clicks before the next render.
        if (!active.current || actionLock.current || saving || gpsRestarting) return;
        actionLock.current = true;
        setPendingAction(action);
        setLastResponse(null);
        setTransportError(null);
        setLastAction(action);
        setDispatched(false);
        let sent = false;
        try {
            if (onPersistBeforeAction) {
                const persisted = await onPersistBeforeAction();
                if (persisted !== true) throw new Error(t('settingsGnssReceiver.persistenceUnconfirmedDesc'));
            } else if (isDirty) {
                throw new Error(t('settingsGnssReceiver.persistenceUnconfirmedDesc'));
            }
            if (!active.current) return;
            sent = true;
            setDispatched(true);
            const response = await guiApi.request<unknown>({
                path: `/settings/gnss/${action}`, method: 'POST', body, type: ContentType.Json, format: 'json',
            });
            if (!active.current) return;
            if (response.error || response.ok === false || response.status >= 400) throw new Error(requestErrorMessage(response) ?? t('settingsGnssReceiver.unknownApiError'));
            const data = decodeGnssActionResponse(response.data, action);
            setLastResponse(data);
            const outcome = gnssActionOutcome(data);
            const actionLabel = t(GNSS_ACTION_LABEL_KEYS[action]);
            if (outcome === 'partial') {
                notification.warning({message: data.message || t('settingsGnssReceiver.notifyPartialFailure', {action: actionLabel}),
                    description: t('settingsGnssReceiver.notifyPartialFailureDesc')});
            } else if (outcome === 'failed') {
                notification.error({message: data.message || t('settingsGnssReceiver.notifyFailed', {action: actionLabel}),
                    description: t('settingsGnssReceiver.notifyFailedDesc')});
            } else if (outcome === 'unconfirmed') {
                notification.warning({message: t('settingsGnssReceiver.restartUnconfirmed'), description: t('settingsGnssReceiver.operationUnconfirmedDesc')});
            } else {
                notification.success({message: data.message || t('settingsGnssReceiver.notifyCompleted', {action: actionLabel}),
                    ...(action === 'plan' ? {description: t('settingsGnssReceiver.planOnlyDesc')} : {})});
            }
        } catch (error) {
            if (!active.current) return;
            const rawMessage = requestErrorMessage(error);
            const description = rawMessage === 'Invalid GNSS action response'
                ? t('settingsGnssReceiver.invalidResponse') : rawMessage ?? t('settingsGnssReceiver.unknownApiError');
            setTransportError(description);
            notification.error({message: t(sent ? 'settingsGnssReceiver.operationUnconfirmed' : 'settingsGnssReceiver.persistenceUnconfirmed'),
                description: sent ? `${description} ${t('settingsGnssReceiver.operationUnconfirmedDesc')}` : description});
        } finally {
            actionLock.current = false;
            if (active.current) setPendingAction(null);
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
                        onClick={asyncHandler(onSave)}
                        loading={saving && !gpsRestarting && !pendingAction}
                        disabled={actionDisabled || !isDirty}
                    >
                        {t("settingsGnssReceiver.saveSettings")}
                    </Button>
                )}
                <Button
                    icon={<PlayCircleOutlined />}
                    onClick={asyncHandler(() => runAction("plan"))}
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
                        onClick={asyncHandler(onSaveAndRestartGps)}
                        loading={gpsRestarting && !pendingAction}
                        disabled={Boolean(pendingAction) || saving || gpsRestarting}
                    >
                        {t("settingsGnssReceiver.saveAndRestartGps")}
                    </Button>
                )}
                <Button
                    icon={<ReloadOutlined />}
                    onClick={asyncHandler(() => runAction("restart", {}))}
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
                    description={t(dispatched ? "settingsGnssReceiver.actionInProgressDesc" : "settingsGnssReceiver.persistenceInProgressDesc")}
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

            {lastResponse?.manual_baud_required && onManualCurrentBaudChange && (
                <Card size="small" type="inner" title={t("settingsGnssReceiver.manualCurrentBaudTitle")} style={{ marginTop: 12 }}>
                    <Paragraph type="secondary">
                        {t("settingsGnssReceiver.manualCurrentBaudHelp")}
                    </Paragraph>
                    <Select
                        value={normalizeGnssString(manualCurrentBaud) || "115200"}
                        onChange={onManualCurrentBaudChange}
                        options={GNSS_EXECUTION_BAUD_OPTIONS.filter((option) => option.value !== "auto").map((option) => ({
                            value: option.value,
                            label: option.label,
                        }))}
                        style={{ width: 180 }}
                    />
                </Card>
            )}

            {(lastResponse || transportError) && (
                <Space direction="vertical" size={12} style={{ width: "100%", marginTop: 12 }}>
                    {lastResponse && (
                        <Card size="small" type="inner" title={t("settingsGnssReceiver.backendResult")}>
                            <Space wrap size={[8, 8]} style={{ marginBottom: 12 }}>
                                <Tag color={responseAlertType(lastResponse) === "success" ? "success" : responseAlertType(lastResponse) === "warning" ? "warning" : "error"}>
                                    {lastResponse.partial_failure
                                        ? t("settingsGnssReceiver.tagPartialFailure")
                                        : gnssActionOutcome(lastResponse) === 'success'
                                            ? t("settingsGnssReceiver.tagSuccess")
                                            : gnssActionOutcome(lastResponse) === 'unconfirmed' ? t('settingsGnssReceiver.restartUnconfirmed') : t("settingsGnssReceiver.tagFailed")}
                                </Tag>
                                {lastResponse.restart_attempted && (
                                    <Tag color={lastResponse.restart_succeeded === true ? 'success' : lastResponse.restart_succeeded === false || lastResponse.restart_error ? 'error' : 'warning'}>
                                        {lastResponse.restart_succeeded === true ? t('settingsGnssReceiver.tagGpsRestarted') : lastResponse.restart_succeeded === false || lastResponse.restart_error ? t('settingsGnssReceiver.tagGpsRestartFailed') : t('settingsGnssReceiver.restartUnconfirmed')}
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
                                {lastResponse.execution_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldExecutionBaud")}</Text>{" "}
                                        {formatBaudValue(
                                            lastResponse.execution_baud,
                                            t("settingsGnssReceiver.executionBaudAuto"),
                                            t("settingsGnssReceiver.unknownValue"),
                                        )}
                                    </Text>
                                )}
                                {lastResponse.detected_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldDetectedBaud")}</Text>{" "}
                                        {lastResponse.detected_baud}
                                    </Text>
                                )}
                                {lastResponse.target_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldTargetBaud")}</Text>{" "}
                                        {lastResponse.target_baud}
                                    </Text>
                                )}
                                {lastResponse.active_verified_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldActiveVerifiedBaud")}</Text>{" "}
                                        {lastResponse.active_verified_baud}
                                    </Text>
                                )}
                                {lastResponse.runtime_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldRuntimeBaud")}</Text>{" "}
                                        {lastResponse.runtime_baud}
                                    </Text>
                                )}
                                {lastResponse.config_baud && (
                                    <Text>
                                        <Text strong>{t("settingsGnssReceiver.fieldConfigBaud")}</Text>{" "}
                                        {lastResponse.config_baud}
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
