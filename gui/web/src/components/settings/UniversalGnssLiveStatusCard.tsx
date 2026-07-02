import React from "react";
import { Card, Descriptions, Tag, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { DiagnosticArray } from "../../hooks/useDiagnostics.ts";
import { GnssStatus, GnssStatusConstants } from "../../types/ros.ts";
import {
    deriveGpsStatus,
    diagnosticsValueMap,
    findDiagnosticStatusByName,
    gnssBaselineSolutionStatusLabel,
    gnssCorrectionStreamStatusLabel,
    gnssReceiverLabel,
    hasGnssCapability,
    hasGnssValue,
    parseDiagnosticBool,
    readGnssNumber,
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

const boolTag = (value: boolean | undefined, trueLabel: string, falseLabel: string, unknownLabel: string) => {
    if (value === true) {
        return <Tag color="success">{trueLabel}</Tag>;
    }
    if (value === false) {
        return <Tag color="warning">{falseLabel}</Tag>;
    }
    return <Tag>{unknownLabel}</Tag>;
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
    const { t } = useTranslation();
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
    const correctionStream = gnssCorrectionStreamStatusLabel(gnssStatus) ?? t("settingsGnssLiveStatus.unknown");
    const baselineSolutionStatus = gnssBaselineSolutionStatusLabel(gnssStatus);
    const baselineAzimuth = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_BASELINE_AZIMUTH,
        gnssStatus?.baseline_azimuth_deg,
    );
    const baselinePitch = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_BASELINE_PITCH,
        gnssStatus?.baseline_pitch_deg,
    );
    const baselineLength = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_BASELINE_LENGTH,
        gnssStatus?.baseline_length_m,
    );
    const hasMsmSummary = hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MSM_SUMMARY);
    const hasMsmSummaryValue = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_MSM_SUMMARY);
    const dualAntennaBaseline = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE)
        ? gnssStatus?.dual_antenna_baseline
        : undefined;
    const msmSummaryLabel = hasMsmSummaryValue
        ? [
            gnssStatus?.msm_summary_constellations_seen || undefined,
            gnssStatus?.msm_summary_message_type ? `MSM ${gnssStatus.msm_summary_message_type}` : undefined,
            gnssStatus?.msm_summary_station_id ? `station ${gnssStatus.msm_summary_station_id}` : undefined,
            gnssStatus?.msm_summary_satellite_count ? `sat ${gnssStatus.msm_summary_satellite_count}` : undefined,
            gnssStatus?.msm_summary_signal_count ? `sig ${gnssStatus.msm_summary_signal_count}` : undefined,
            gnssStatus?.msm_summary_cell_count ? `cell ${gnssStatus.msm_summary_cell_count}` : undefined,
        ].filter(Boolean).join(" · ")
        : undefined;

    return (
        <Card size="small" title={t("settingsGnssLiveStatus.cardTitle")} style={{ marginBottom: 16 }}>
            <Descriptions size="small" column={1}>
                <Descriptions.Item label={t("settingsGnssLiveStatus.detectedReceiver")}>
                    {detectedReceiver !== "GNSS" ? detectedReceiver : receiverFamily}
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.liveStatus")}>
                    <Tag color={liveStatus.fixType === "RTK_FIX" ? "success" : liveStatus.fixType === "NO_FIX" ? "warning" : "processing"}>
                        {liveStatus.label}
                    </Tag>
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.parserHealth")}>
                    {boolTag(parserHealthy, t("settingsGnssLiveStatus.healthy"), t("settingsGnssLiveStatus.warning"), t("settingsGnssLiveStatus.unknown"))}
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.correctionStream")}>
                    {forwardingTag(correctionStream)}
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.receiverCorrections")}>
                    {boolTag(receiverCorrectionAvailable ?? correctionAvailable, t("settingsGnssLiveStatus.available"), t("settingsGnssLiveStatus.unavailable"), t("settingsGnssLiveStatus.unknown"))}
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.receiverHealth")}>
                    {boolTag(receiverHealthy, t("settingsGnssLiveStatus.healthy"), t("settingsGnssLiveStatus.warning"), t("settingsGnssLiveStatus.unknown"))}
                </Descriptions.Item>
                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE) && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.dualAntennaBaseline")}>
                        {boolTag(dualAntennaBaseline, t("settingsGnssLiveStatus.available"), t("settingsGnssLiveStatus.unavailable"), t("settingsGnssLiveStatus.unknown"))}
                    </Descriptions.Item>
                )}
                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_SOLUTION_STATUS) && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.baselineSolutionStatus")}>
                        {baselineSolutionStatus ?? t("settingsGnssLiveStatus.unknown")}
                    </Descriptions.Item>
                )}
                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_AZIMUTH) && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.baselineAzimuthDeg")}>
                        {baselineAzimuth !== undefined ? baselineAzimuth.toFixed(2) : t("settingsGnssLiveStatus.unknown")}
                    </Descriptions.Item>
                )}
                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_PITCH) && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.baselinePitchDeg")}>
                        {baselinePitch !== undefined ? baselinePitch.toFixed(2) : t("settingsGnssLiveStatus.unknown")}
                    </Descriptions.Item>
                )}
                {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_LENGTH) && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.baselineLengthM")}>
                        {baselineLength !== undefined ? baselineLength.toFixed(3) : t("settingsGnssLiveStatus.unknown")}
                    </Descriptions.Item>
                )}
                {hasMsmSummary && (
                    <Descriptions.Item label={t("settingsGnssLiveStatus.msmSummary")}>
                        {msmSummaryLabel || t("settingsGnssLiveStatus.unknown")}
                    </Descriptions.Item>
                )}
                <Descriptions.Item label={t("settingsGnssLiveStatus.selectedRuntimeBaud")}>
                    <Text code>{normalizeGnssString(selectedBaud) || "921600"}</Text>
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.selectedConfigBaud")}>
                    <Text code>{normalizeGnssString(selectedConfigBaud) || normalizeGnssString(selectedBaud) || "921600"}</Text>
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.selectedProfile")}>
                    {t(gnssProfileLabel(selectedProfile))}
                </Descriptions.Item>
                <Descriptions.Item label={t("settingsGnssLiveStatus.selectedSignalProfile")}>
                    {t(gnssSignalProfileLabel(selectedSignalProfile))}
                </Descriptions.Item>
            </Descriptions>
        </Card>
    );
};
