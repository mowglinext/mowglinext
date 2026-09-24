import React from "react";
import { Alert, Card, Space, Table, Tag, Typography } from "antd";
import type { ColumnsType } from "antd/es/table";
import { useTranslation } from "react-i18next";
import { useFirmwareParams } from "../../hooks/useFirmwareParams.ts";
import type { FirmwareParam } from "../../types/ros.generated.ts";

const { Paragraph, Text } = Typography;

// FirmwareParams / FirmwareParam constants (mowgli_interfaces). Mirrored as
// plain numbers: the generated const enums are erased at build time.
const BOOT_DEFAULTS = 0;
const BOOT_FLASH = 1;
const BOOT_FLASH_ERASED = 2;
const COMMIT_LOG_FULL = 4;
const COMMIT_ERROR = 5;
const STATUS_CLAMPED = 1;

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
 * Read-only view of what the STM32 firmware actually runs (protocol v7): every
 * runtime parameter the ROS2 stack sends, the value the board applied after
 * coercing it into its compiled envelope, and whether it is stored in the
 * board's flash (and therefore applied at power-on, before ROS2 connects).
 */
export const FirmwareParamsCard: React.FC = () => {
    const { t } = useTranslation();
    const report = useFirmwareParams();
    const params = report?.params ?? [];

    const label = (name: string | undefined) =>
        t(`settingsFields.${name}.label`, { defaultValue: name ?? "" });

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
            render: (name: string) => label(name),
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
        </Card>
    );
};
