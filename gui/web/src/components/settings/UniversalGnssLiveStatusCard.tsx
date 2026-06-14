import React from "react";
import { Card, Descriptions, Tag, Typography } from "antd";
import { DiagnosticArray } from "../../hooks/useDiagnostics.ts";
import { GnssStatus } from "../../types/ros.ts";
import {
    deriveGpsStatus,
    diagnosticsValueMap,
    findDiagnosticStatusByName,
    gnssReceiverLabel,
    parseDiagnosticBool,
} from "../../utils/gpsStatus.ts";
import { gnssProfileLabel, gnssSignalProfileLabel, normalizeGnssString } from "./gnssConfig.ts";

const { Text } = Typography;

type Props = {
    diagnostics: DiagnosticArray | undefined;
    gnssStatus: GnssStatus | undefined;
    selectedBaud: unknown;
    selectedConfigBaud?: unknown;
    selectedProfile: unknown;
    selectedSignalProfile?: unknown;
    selectedReceiverFamily: unknown;
};

const boolTag = (value: boolean | undefined, trueLabel: string, falseLabel: string) => {
    if (value === true) {
        return <Tag color="success">{trueLabel}</Tag>;
    }
    if (value === false) {
        return <Tag color="warning">{falseLabel}</Tag>;
    }
    return <Tag>Unknown</Tag>;
};

const forwardingTag = (message: string) => {
    const normalized = message.toLowerCase();
    if (normalized.includes("active")) {
        return <Tag color="success">{message}</Tag>;
    }
    if (normalized.includes("unavailable") || normalized.includes("error") || normalized.includes("waiting")) {
        return <Tag color="warning">{message}</Tag>;
    }
    return <Tag>{message}</Tag>;
};

export const UniversalGnssLiveStatusCard: React.FC<Props> = ({
    diagnostics,
    gnssStatus,
    selectedBaud,
    selectedConfigBaud,
    selectedProfile,
    selectedSignalProfile,
    selectedReceiverFamily,
}) => {
    const liveStatus = deriveGpsStatus(gnssStatus);
    const detectedReceiver = gnssReceiverLabel(gnssStatus);
    const summary = findDiagnosticStatusByName(diagnostics, "universal_gnss/summary");
    const forwarding = findDiagnosticStatusByName(diagnostics, "universal_gnss/rtcm_forwarding") ??
        findDiagnosticStatusByName(diagnostics, "universal_gnss_ntrip/rtcm_forwarding");
    const summaryValues = diagnosticsValueMap(summary);
    const forwardingValues = diagnosticsValueMap(forwarding);
    const parserHealthy = parseDiagnosticBool(summaryValues.parser_healthy);
    const correctionAvailable = parseDiagnosticBool(summaryValues.correction_available);
    const receiverHealthy = parseDiagnosticBool(summaryValues.receiver_healthy);
    const receiverCorrectionAvailable = parseDiagnosticBool(forwardingValues.receiver_correction_available);
    const receiverFamily = normalizeGnssString(selectedReceiverFamily) || "auto";

    return (
        <Card size="small" title="Live GNSS Status" style={{ marginBottom: 16 }}>
            <Descriptions size="small" column={1}>
                <Descriptions.Item label="Detected receiver">
                    {detectedReceiver !== "GNSS" ? detectedReceiver : receiverFamily}
                </Descriptions.Item>
                <Descriptions.Item label="Live status">
                    <Tag color={liveStatus.fixType === "RTK_FIX" ? "success" : liveStatus.fixType === "NO_FIX" ? "warning" : "processing"}>
                        {liveStatus.label}
                    </Tag>
                </Descriptions.Item>
                <Descriptions.Item label="Parser health">
                    {boolTag(parserHealthy, "Healthy", "Warning")}
                </Descriptions.Item>
                <Descriptions.Item label="Correction forwarding">
                    {forwardingTag(forwarding?.message ?? "Unknown")}
                </Descriptions.Item>
                <Descriptions.Item label="Receiver corrections">
                    {boolTag(receiverCorrectionAvailable ?? correctionAvailable, "Available", "Unavailable")}
                </Descriptions.Item>
                <Descriptions.Item label="Receiver health">
                    {boolTag(receiverHealthy, "Healthy", "Warning")}
                </Descriptions.Item>
                <Descriptions.Item label="Selected runtime baud">
                    <Text code>{normalizeGnssString(selectedBaud) || "921600"}</Text>
                </Descriptions.Item>
                <Descriptions.Item label="Selected config baud">
                    <Text code>{normalizeGnssString(selectedConfigBaud) || normalizeGnssString(selectedBaud) || "921600"}</Text>
                </Descriptions.Item>
                <Descriptions.Item label="Selected profile">
                    {gnssProfileLabel(selectedProfile)}
                </Descriptions.Item>
                <Descriptions.Item label="Selected signal profile">
                    {gnssSignalProfileLabel(selectedSignalProfile)}
                </Descriptions.Item>
            </Descriptions>
        </Card>
    );
};
