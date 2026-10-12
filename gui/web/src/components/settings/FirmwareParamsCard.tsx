import React, { useEffect, useState } from "react";
import { Alert, Button, Card, Modal, Space, Table, Tag, Tooltip, Typography } from "antd";
import { InfoCircleOutlined } from "@ant-design/icons";
import type { ColumnsType } from "antd/es/table";
import { useTranslation } from "react-i18next";
import { useFirmwareParams } from "../../hooks/useFirmwareParams.ts";
import { useTopic } from "../../hooks/useTopic.ts";
import { useApi } from "../../hooks/useApi.ts";
import { useMultiplexStatus } from "../../hooks/useWS.ts";
import type { FirmwareParam, HighLevelStatus, Status } from "../../types/ros.generated.ts";
import type { WheelOdom } from "../../hooks/useWheelOdom.ts";
import { firmwareParamResetBlockReason } from "../../utils/firmwareParamReset.ts";

const { Paragraph, Text } = Typography;

// FirmwareParams / FirmwareParam constants (mowgli_interfaces). Mirrored as
// plain numbers: the generated const enums are erased at build time.
const BOOT_DEFAULTS = 0;
const BOOT_FLASH = 1;
const BOOT_FLASH_ERASED = 2;
const COMMIT_LOG_FULL = 4;
const COMMIT_ERROR = 5;
const COMMIT_RESET_PENDING = 6;
const STATUS_CLAMPED = 1;
const MAX_STORE_STATUS_AGE_MS = 3000;

const formatValue = (value: number | undefined): string => {
    if (value === undefined || value === null || !Number.isFinite(value)) {
        return "—";
    }
    return Number.isInteger(value) ? String(value) : String(Number(value.toFixed(4)));
};

const differs = (param: FirmwareParam): boolean =>
    !!param.requested_valid &&
    !!param.reported &&
    Math.abs((param.applied ?? 0) - (param.requested ?? 0)) >
        1e-6 * Math.max(1, Math.abs(param.requested ?? 0));

/**
 * View of what the STM32 firmware actually runs (protocol v8): every
 * runtime parameter the ROS2 stack sends, the value the board applied after
 * coercing it into its compiled envelope, and whether it is stored in the
 * board's flash (and therefore applied at power-on, before ROS2 connects).
 */
export const FirmwareParamsCard: React.FC = () => {
    const { t } = useTranslation();
    const guiApi = useApi();
    const {
        report,
        lastMessageAt: firmwareReportAt,
        storeStatusSequence,
        lastStoreStatusAt,
    } = useFirmwareParams();
    const status = useTopic<Status>("status", {}, { withTimestamp: true });
    const highLevel = useTopic<HighLevelStatus>("highLevelStatus", {}, { withTimestamp: true });
    const wheelOdom = useTopic<WheelOdom>("wheelOdom", {}, { withTimestamp: true });
    const socketStatus = useMultiplexStatus();
    const [, setClockTick] = useState(0);
    // Topic receipts also render this component. Evaluate against the current
    // clock, not a cached timer tick that could predate a just-arrived sample.
    const nowMs = Date.now();
    const [resetRequest, setResetRequest] = useState<{
        id: number | null;
        sentAt: number;
        beforeSequence: number;
        phase: "sending" | "awaiting" | "acknowledged" | "uncertain";
    } | null>(null);
    const [resetConfirmOpen, setResetConfirmOpen] = useState(false);
    const [rebootConfirmOpen, setRebootConfirmOpen] = useState(false);
    const [rebootSentId, setRebootSentId] = useState<number | null>(null);
    const [rebootSentAt, setRebootSentAt] = useState<number | null>(null);
    const [actionError, setActionError] = useState<string | null>(null);
    const params = report?.params ?? [];

    useEffect(() => {
        const timer = window.setInterval(() => setClockTick((tick) => tick + 1), 500);
        return () => window.clearInterval(timer);
    }, []);

    const resetSafetyReason = firmwareParamResetBlockReason({
        status,
        highLevelStatus: highLevel,
        wheelOdom,
        nowMs,
    });
    const storeReportSeen = report !== null && firmwareReportAt !== null;
    const storeStatusFresh = lastStoreStatusAt !== null && nowMs >= lastStoreStatusAt &&
        nowMs - lastStoreStatusAt <= MAX_STORE_STATUS_AGE_MS && storeStatusSequence !== 0;
    const boardResetPending = report?.last_commit === COMMIT_RESET_PENDING &&
        (report.reset_request_id ?? 0) > 0;
    const matchingResetAck = !!resetRequest && resetRequest.id !== null &&
        resetRequest.phase !== "uncertain" && storeStatusFresh && storeStatusSequence !== 0 &&
        storeStatusSequence !== resetRequest.beforeSequence && lastStoreStatusAt !== null &&
        lastStoreStatusAt >= resetRequest.sentAt &&
        report?.last_commit === COMMIT_RESET_PENDING &&
        report.reset_request_id === resetRequest.id;
    const resetAcked = matchingResetAck;
    const externalPending = !resetRequest && boardResetPending && storeStatusFresh;
    const rebootTargetId = resetRequest?.id ?? (externalPending ? report?.reset_request_id ?? null : null);
    const canRequestReset = socketStatus === "open" && resetSafetyReason === null &&
        storeReportSeen && !boardResetPending && !resetRequest;
    const canOfferReboot = socketStatus === "open" && resetSafetyReason === null &&
        storeStatusFresh && !!rebootTargetId &&
        (resetAcked || externalPending) && rebootSentId !== rebootTargetId;
    const canRecoverPending = socketStatus === "open" && resetRequest?.phase === "uncertain" &&
        resetSafetyReason === null && storeStatusFresh && boardResetPending;
    const canClearUncertainRequest = socketStatus === "open" && resetRequest?.phase === "uncertain" &&
        !boardResetPending;

    useEffect(() => {
        if (!resetRequest) return;
        if (resetRequest.phase === "awaiting" && matchingResetAck) {
            setResetRequest((current) => current?.id === resetRequest.id
                ? { ...current, phase: "acknowledged" }
                : current);
            return;
        }
        if (resetRequest.phase === "awaiting" && nowMs - resetRequest.sentAt > 10_000) {
            setResetRequest((current) => current?.id === resetRequest.id
                ? { ...current, phase: "uncertain" }
                : current);
            return;
        }
        const telemetryStale = resetSafetyReason === "status_unknown" || resetSafetyReason === "status_stale" ||
            resetSafetyReason === "high_level_unknown" || resetSafetyReason === "high_level_stale" ||
            resetSafetyReason === "odometry_unknown" || resetSafetyReason === "odometry_stale";
        if ((resetRequest.phase === "awaiting" || resetRequest.phase === "acknowledged") &&
            (telemetryStale || socketStatus === "closed")) {
            // Once freshness is lost, a later latched report must not revive an
            // old reboot affordance. The operator can inspect the fresh board
            // pending marker after reconnect and choose the separate reboot.
            setResetRequest((current) => current?.id === resetRequest.id
                ? { ...current, phase: "uncertain" }
                : current);
        }
    }, [matchingResetAck, nowMs, resetRequest, resetSafetyReason, socketStatus]);

    const sendResetRequest = async () => {
        if (!canRequestReset) return;
        const sentAt = Date.now();
        setResetConfirmOpen(false);
        setActionError(null);
        setResetRequest({ id: null, sentAt, beforeSequence: storeStatusSequence, phase: "sending" });
        try {
            const result = await guiApi.mowglinext.callCreate("reset_firmware_param_store", {});
            if (result.error) {
                throw new Error(result.error.error);
            }
            const response = result.data as { message?: string } | undefined;
            const match = response?.message?.match(/request_id=(\d+)/);
            const id = match ? Number(match[1]) : null;
            setResetRequest((current) => {
                if (!current || current.sentAt !== sentAt) return current;
                return {
                    ...current,
                    id,
                    phase: current.phase === "uncertain" || id === null ? "uncertain" : "awaiting",
                };
            });
            if (id === null) {
                setActionError(t("firmwareParams.resetUncertain"));
            }
        } catch (error) {
            setResetRequest((current) => current?.sentAt === sentAt
                ? { ...current, phase: "uncertain" }
                : current);
            setActionError(error instanceof Error ? error.message : String(error));
        }
    };

    const sendBoardReboot = async () => {
        if (!canOfferReboot || rebootTargetId === null) return;
        setRebootConfirmOpen(false);
        setActionError(null);
        try {
            const result = await guiApi.mowglinext.callCreate("reboot_board", {});
            if (result.error) {
                throw new Error(result.error.error);
            }
            setRebootSentId(rebootTargetId);
            setRebootSentAt(Date.now());
        } catch (error) {
            setActionError(error instanceof Error ? error.message : String(error));
        }
    };

    const resetCompleted = rebootSentId !== null && rebootSentAt !== null && storeStatusFresh &&
        lastStoreStatusAt !== null && lastStoreStatusAt > rebootSentAt && report?.reset_request_id === rebootSentId &&
        report.last_commit !== COMMIT_RESET_PENDING && report.boot_source === BOOT_FLASH_ERASED;

    // Editable settings carry their label/tooltip under settingsFields; the
    // parameters edited elsewhere (drive tuning, hardware) or measured (gyro
    // bias) have theirs under firmwareParams.params.
    const label = (name: string) =>
        t(`settingsFields.${name}.label`, {
            defaultValue: t(`firmwareParams.params.${name}.label`, { defaultValue: name }),
        });
    const description = (name: string) =>
        t(`settingsFields.${name}.tooltip`, {
            defaultValue: t(`firmwareParams.params.${name}.description`, { defaultValue: "" }),
        });

    const renderName = (name: string | undefined) => {
        const key = name ?? "";
        const help = description(key);
        return (
            <Space direction="vertical" size={0}>
                <Space size={4}>
                    <Text>{label(key)}</Text>
                    {help && (
                        <Tooltip title={help}>
                            <InfoCircleOutlined aria-label={help} style={{ opacity: 0.6 }} />
                        </Tooltip>
                    )}
                </Space>
                <Text type="secondary" code style={{ fontSize: 11 }}>
                    {key}
                </Text>
            </Space>
        );
    };

    const stateTag = (param: FirmwareParam) => {
        if (!param.reported) {
            return <Tag>{t("firmwareParams.stateUnreported")}</Tag>;
        }
        const tags = [];
        if (param.status === STATUS_CLAMPED || differs(param)) {
            tags.push(
                <Tag color="warning" key="clamped">
                    {t("firmwareParams.stateClamped")}
                </Tag>,
            );
        }
        if (param.is_volatile) {
            tags.push(<Tag key="volatile">{t("firmwareParams.stateVolatile")}</Tag>);
        } else if (param.persisted) {
            tags.push(
                <Tag color="success" key="stored">
                    {t("firmwareParams.stateStored")}
                </Tag>,
            );
        } else {
            tags.push(<Tag key="unstored">{t("firmwareParams.stateNotStored")}</Tag>);
        }
        return <Space size={4} wrap>{tags}</Space>;
    };

    const columns: ColumnsType<FirmwareParam> = [
        {
            title: t("firmwareParams.colParameter"),
            dataIndex: "name",
            key: "name",
            render: (name: string) => renderName(name),
        },
        {
            title: t("firmwareParams.colApplied"),
            key: "applied",
            render: (_, param) => <Text strong>{formatValue(param.reported ? param.applied : undefined)}</Text>,
        },
        {
            title: t("firmwareParams.colRequested"),
            key: "requested",
            render: (_, param) => formatValue(param.requested_valid ? param.requested : undefined),
        },
        {
            title: t("firmwareParams.colRange"),
            key: "range",
            render: (_, param) =>
                param.reported ? `${formatValue(param.min_value)} – ${formatValue(param.max_value)}` : "—",
        },
        {
            title: t("firmwareParams.colState"),
            key: "state",
            render: (_, param) => stateTag(param),
        },
    ];

    const bootMessage = (() => {
        switch (report?.boot_source) {
            case BOOT_DEFAULTS:
                return t("firmwareParams.bootDefaults");
            case BOOT_FLASH:
                return t("firmwareParams.bootFlash");
            case BOOT_FLASH_ERASED:
                return t("firmwareParams.bootErased");
            default:
                return null;
        }
    })();

    return (
        <Card title={t("firmwareParams.title")} size="small" style={{ marginTop: 16 }}>
            <Paragraph type="secondary">{t("firmwareParams.description")}</Paragraph>
            {report?.firmware_incompatible && (
                <Alert type="error" showIcon message={t("firmwareParams.incompatible")} style={{ marginBottom: 12 }} />
            )}
            {report?.last_commit === COMMIT_ERROR && (
                <Alert type="error" showIcon message={t("firmwareParams.commitError")} style={{ marginBottom: 12 }} />
            )}
            {report?.last_commit === COMMIT_LOG_FULL && (
                <Alert type="warning" showIcon message={t("firmwareParams.commitFull")} style={{ marginBottom: 12 }} />
            )}
            {resetRequest && (resetRequest.phase === "sending" || resetRequest.phase === "awaiting") && (
                <Alert
                    type="info"
                    showIcon
                    message={t("firmwareParams.resetWaiting")}
                    description={resetRequest.id !== null
                        ? t("firmwareParams.resetRequestId", { id: resetRequest.id })
                        : undefined}
                    style={{ marginBottom: 12 }}
                />
            )}
            {resetRequest?.phase === "uncertain" && (
                <Alert type="warning" showIcon message={t("firmwareParams.resetUncertain")} style={{ marginBottom: 12 }} />
            )}
            {(resetAcked || externalPending) && !resetCompleted && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("firmwareParams.resetArmed", { id: rebootTargetId })}
                    description={t("firmwareParams.resetArmedWarning")}
                    style={{ marginBottom: 12 }}
                />
            )}
            {rebootSentId !== null && !resetCompleted && (
                <Alert type="info" showIcon message={t("firmwareParams.rebootSent")} style={{ marginBottom: 12 }} />
            )}
            {resetCompleted && (
                <Alert type="success" showIcon message={t("firmwareParams.resetCompleted")} style={{ marginBottom: 12 }} />
            )}
            {actionError && <Alert type="error" showIcon message={actionError} style={{ marginBottom: 12 }} />}
            <Space wrap style={{ marginBottom: 12 }}>
                <Button danger disabled={!canRequestReset} onClick={() => setResetConfirmOpen(true)}>
                    {t("firmwareParams.resetAction")}
                </Button>
                {canOfferReboot && (
                    <Button danger onClick={() => setRebootConfirmOpen(true)}>
                        {t("firmwareParams.rebootAction")}
                    </Button>
                )}
                {canRecoverPending && (
                    <Button onClick={() => setResetRequest(null)}>
                        {t("firmwareParams.resetRecoverPending")}
                    </Button>
                )}
                {canClearUncertainRequest && (
                    <Button onClick={() => setResetRequest(null)}>
                        {t("firmwareParams.resetClearUncertain")}
                    </Button>
                )}
                {!canRequestReset && !canOfferReboot && resetSafetyReason && (
                    <Text type="secondary">{t(`firmwareParams.resetBlocked.${resetSafetyReason}`)}</Text>
                )}
            </Space>
            {bootMessage && (
                <Paragraph>
                    {bootMessage}{" "}
                    {report?.boot_source !== undefined && report.records_left !== undefined && (
                        <Text type="secondary">
                            {t("firmwareParams.recordsLeft", { count: report.records_left })}
                        </Text>
                    )}
                </Paragraph>
            )}
            {params.length === 0 ? (
                <Paragraph type="secondary">{t("firmwareParams.noData")}</Paragraph>
            ) : (
                <Table<FirmwareParam>
                    dataSource={params}
                    columns={columns}
                    rowKey={(param) => String(param.id)}
                    pagination={false}
                    size="small"
                    scroll={{ x: true }}
                />
            )}
            <Modal
                open={resetConfirmOpen}
                title={t("firmwareParams.resetConfirmTitle")}
                okText={t("firmwareParams.resetConfirmOk")}
                okType="danger"
                cancelText={t("firmwareParams.resetConfirmCancel")}
                confirmLoading={resetRequest?.phase === "sending"}
                okButtonProps={{ disabled: !canRequestReset }}
                onCancel={() => setResetConfirmOpen(false)}
                onOk={sendResetRequest}
            >
                <Paragraph>{t("firmwareParams.resetConfirmBody")}</Paragraph>
                <Paragraph type="warning">{t("firmwareParams.resetConfirmWarning")}</Paragraph>
            </Modal>
            <Modal
                open={rebootConfirmOpen}
                title={t("firmwareParams.rebootConfirmTitle")}
                okText={t("firmwareParams.rebootConfirmOk")}
                okType="danger"
                cancelText={t("firmwareParams.resetConfirmCancel")}
                okButtonProps={{ disabled: !canOfferReboot }}
                onCancel={() => setRebootConfirmOpen(false)}
                onOk={sendBoardReboot}
            >
                <Paragraph>{t("firmwareParams.rebootConfirmBody")}</Paragraph>
                <Paragraph type="warning">{t("firmwareParams.resetConfirmWarning")}</Paragraph>
            </Modal>
        </Card>
    );
};
