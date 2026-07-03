import React, { useMemo } from "react";
import { Alert, Form, Input, Select, Space, Tag, Typography } from "antd";
import { useTranslation } from "react-i18next";
import type { GnssStatus } from "../../types/ros.ts";
import { useGnssRuntimeConfig } from "../../hooks/useGnssRuntimeConfig.ts";
import { normalizeGnssReceiverModel } from "./gnssConfig.ts";

const { Text } = Typography;

type Props = {
    value: unknown;
    onChange: (value: string) => void;
    selectedReceiverFamily: unknown;
    selectedReceiverModel: unknown;
    gnssStatus?: GnssStatus;
};

const normalizeFamily = (value: unknown) => String(value ?? "")
    .trim()
    .toLowerCase()
    .replace("u-blox", "ublox");

const detectRuntimeFamily = (gnssStatus?: GnssStatus) => {
    const vendor = String(gnssStatus?.receiver_vendor ?? "").trim().toLowerCase();
    if (vendor.includes("unicore")) {
        return "unicore";
    }
    if (vendor.includes("u-blox") || vendor.includes("ublox")) {
        return "ublox";
    }
    if (vendor.includes("nmea")) {
        return "nmea";
    }
    return "";
};

const sourceTagColor = (source?: string) => {
    switch (source) {
        case "config":
            return "success";
        case "env":
            return "warning";
        default:
            return "default";
    }
};

export const GnssSerialDeviceConfigField: React.FC<Props> = ({
    value,
    onChange,
    selectedReceiverFamily,
    selectedReceiverModel,
    gnssStatus,
}) => {
    const { t } = useTranslation();
    const { runtimeConfig, error } = useGnssRuntimeConfig();
    const activeValue = String(
        value ??
        runtimeConfig?.serial_device?.active_value ??
        "/dev/ttyAMA4",
    );
    const activeBaud = runtimeConfig?.serial_baud?.active_value ?? "921600";
    const currentSource = runtimeConfig?.serial_device?.source ?? "default";

    const options = useMemo(() => {
        const discovered = runtimeConfig?.serial_devices ?? [];
        const values = new Set(discovered.map((option) => option.path));
        const list = discovered.map((option) => ({
            value: option.path,
            label: option.display_label,
        }));
        if (activeValue.trim() !== "" && !values.has(activeValue)) {
            list.unshift({
                value: activeValue,
                label: activeValue,
            });
        }
        return list;
    }, [activeValue, runtimeConfig?.serial_devices]);

    const selectedFamily = normalizeFamily(selectedReceiverFamily);
    const runtimeFamily = detectRuntimeFamily(gnssStatus);
    const selectedModel = normalizeGnssReceiverModel(selectedReceiverModel);
    const runtimeModel = normalizeGnssReceiverModel(gnssStatus?.receiver_model);
    const showEnvFallbackWarning = currentSource === "env"
        && !(runtimeConfig?.serial_device?.configured_value ?? "").trim();
    const showConfiguredDeviceMissing = currentSource === "config"
        && runtimeConfig?.configured_serial_device_exists === false;
    const showActiveDeviceMissing = runtimeConfig?.active_serial_device_exists === false;
    const showFamilyMismatch = selectedFamily !== ""
        && selectedFamily !== "auto"
        && runtimeFamily !== ""
        && runtimeFamily !== selectedFamily;
    const showModelMismatch = selectedModel !== ""
        && runtimeModel !== ""
        && runtimeModel !== selectedModel;

    return (
        <Space direction="vertical" size={10} style={{ width: "100%" }}>
            <Form.Item label={t("gnssRuntimeConfig.detectedDevicesLabel")} style={{ marginBottom: 8 }}>
                <Select
                    showSearch
                    value={activeValue}
                    onChange={onChange}
                    options={options}
                    placeholder={t("gnssRuntimeConfig.detectedDevicesPlaceholder")}
                    optionFilterProp="label"
                />
            </Form.Item>

            <Form.Item label={t("gnssRuntimeConfig.manualSerialPathLabel")} style={{ marginBottom: 8 }}>
                <Input
                    value={activeValue}
                    onChange={(event) => onChange(event.target.value)}
                    placeholder="/dev/serial/by-id/..."
                />
            </Form.Item>

            <Space wrap size={[8, 8]}>
                <Text type="secondary">{t("gnssRuntimeConfig.activeLinkLabel")}</Text>
                <Tag color={sourceTagColor(currentSource)}>{t(`gnssRuntimeConfig.source.${currentSource}`)}</Tag>
                <Text code>{runtimeConfig?.receiver_family?.active_value ?? "auto"}</Text>
                <Text code>{runtimeConfig?.serial_device?.active_value ?? activeValue}</Text>
                <Text code>{activeBaud}</Text>
            </Space>

            {runtimeConfig?.active_serial_resolved_path
                && runtimeConfig.active_serial_resolved_path !== runtimeConfig.serial_device?.active_value && (
                <Text type="secondary">
                    {t("gnssRuntimeConfig.resolvedTarget", { path: runtimeConfig.active_serial_resolved_path })}
                </Text>
            )}

            {showEnvFallbackWarning && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("gnssRuntimeConfig.envFallbackMessage", {
                        family: runtimeConfig?.receiver_family?.active_value ?? "auto",
                        device: runtimeConfig?.serial_device?.active_value ?? activeValue,
                        baud: activeBaud,
                    })}
                />
            )}

            {showConfiguredDeviceMissing && (
                <Alert
                    type="error"
                    showIcon
                    message={t("gnssRuntimeConfig.configuredDeviceMissing", {
                        device: runtimeConfig?.serial_device?.configured_value ?? activeValue,
                    })}
                />
            )}

            {!showConfiguredDeviceMissing && showActiveDeviceMissing && (
                <Alert
                    type="error"
                    showIcon
                    message={t("gnssRuntimeConfig.activeDeviceMissing", {
                        device: runtimeConfig?.serial_device?.active_value ?? activeValue,
                    })}
                />
            )}

            {showFamilyMismatch && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("gnssRuntimeConfig.familyMismatch", {
                        selected: selectedFamily,
                        detected: runtimeFamily,
                    })}
                />
            )}

            {showModelMismatch && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("gnssRuntimeConfig.modelMismatch", {
                        selected: selectedModel,
                        detected: runtimeModel,
                    })}
                />
            )}

            {error && (
                <Alert
                    type="warning"
                    showIcon
                    message={t("gnssRuntimeConfig.loadWarning")}
                    description={error}
                />
            )}
        </Space>
    );
};
