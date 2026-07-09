import React from "react";
import { WifiOutlined } from "@ant-design/icons";
import { Card, Col, Descriptions, Row, Space, Statistic, Tag, Typography } from "antd";
import { useTranslation } from "react-i18next";
import { GnssStatus, GnssStatusConstants } from "../../types/ros.ts";
import {
    deriveGpsStatus,
    gnssBaselineSolutionStatusLabel,
    gnssCorrectionStreamStatusLabel,
    gnssReceiverLabel,
    hasGnssCapability,
    hasGnssValue,
    readGnssBooleanState,
    readGnssNumber,
} from "../../utils/gpsStatus.ts";
import {
    clampRatio,
    correctionStreamTagColor,
    GNSS_CN0_FULL_SCALE_DB_HZ,
    liveStatusTagColor,
    rtkModeTagColor,
} from "./gnssPresentation.ts";
import { GnssDiagnosticBarRow } from "./GnssDiagnosticBarRow.tsx";
import { normalizeGnssString } from "./gnssFormatting.ts";

const { Text } = Typography;

type Props = {
    gnssStatus: GnssStatus | undefined;
    latitude?: number;
    longitude?: number;
    altitudeM?: number;
    horizontalAccuracyM?: number;
};

const conditionTag = (
    value: ReturnType<typeof readGnssBooleanState>,
    trueLabel: string,
    falseLabel: string,
    unknownLabel: string,
) => {
    if (value === "true") {
        return <Tag color="error">{trueLabel}</Tag>;
    }
    if (value === "false") {
        return <Tag color="success">{falseLabel}</Tag>;
    }
    return <Tag>{unknownLabel}</Tag>;
};

export const GnssLiveDiagnosticsCard: React.FC<Props> = ({
    gnssStatus,
    latitude,
    longitude,
    altitudeM,
    horizontalAccuracyM,
}) => {
    const { t } = useTranslation();
    const unknownLabel = t("diagnosticsPage.unknown");
    const liveStatus = deriveGpsStatus(gnssStatus);
    const receiverLabel = gnssReceiverLabel(gnssStatus);
    const backendLabel = normalizeGnssString(gnssStatus?.backend) || t("diagnosticsPage.unknownLower");
    const correctionStreamLabel = gnssCorrectionStreamStatusLabel(gnssStatus) ?? unknownLabel;
    const baselineSolutionStatus = gnssBaselineSolutionStatusLabel(gnssStatus);
    const rtkModeLabel = (() => {
        if (!hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_RTK_MODE)) {
            return unknownLabel;
        }
        switch (gnssStatus?.rtk_mode) {
            case GnssStatusConstants.RTK_MODE_NONE:
                return t("settingsGnssLiveStatus.rtkModeNone");
            case GnssStatusConstants.RTK_MODE_FLOAT:
                return t("gpsStatus.rtkFloat");
            case GnssStatusConstants.RTK_MODE_FIXED:
                return t("gpsStatus.rtkFixed");
            case GnssStatusConstants.RTK_MODE_UNKNOWN:
            default:
                return unknownLabel;
        }
    })();

    const gpsSatellitesUsed = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_USED,
        gnssStatus?.satellites_used,
    );
    const gpsSatellitesVisible = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_VISIBLE,
        gnssStatus?.satellites_visible,
    );
    const gpsSatellitesTracked = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_TRACKED,
        gnssStatus?.satellites_tracked,
    );
    const gpsMeanCn0 = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_MEAN_CN0,
        gnssStatus?.mean_cn0_db_hz,
    );
    const gpsMaxCn0 = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_MAX_CN0,
        gnssStatus?.max_cn0_db_hz,
    );
    const interferenceState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_INTERFERENCE_STATUS,
        gnssStatus?.interference_detected,
    );
    const jammingState = readGnssBooleanState(
        gnssStatus,
        GnssStatusConstants.CAP_JAMMING_STATUS,
        gnssStatus?.jamming_detected,
    );
    const dualAntennaBaseline = hasGnssValue(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE)
        ? gnssStatus?.dual_antenna_baseline
        : undefined;
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
    const msmState = (() => {
        if (!hasMsmSummaryValue) {
            return { color: undefined, label: unknownLabel };
        }
        if (!gnssStatus?.msm_summary_seen) {
            return { color: undefined, label: t("settingsGnssLiveStatus.msmStateNotSeen") };
        }
        if (!gnssStatus?.msm_summary_decoded) {
            return { color: "warning", label: t("settingsGnssLiveStatus.msmStateSeenUndecoded") };
        }
        if (!gnssStatus?.msm_summary_valid) {
            return { color: "error", label: t("settingsGnssLiveStatus.msmStateDecodedInvalid") };
        }
        return { color: "success", label: t("settingsGnssLiveStatus.msmStateValid") };
    })();

    const showSatellitesSection =
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_SATELLITES_USED) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_SATELLITES_VISIBLE) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_SATELLITES_TRACKED);
    const showCn0Section =
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MEAN_CN0) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MAX_CN0) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_INTERFERENCE_STATUS) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_JAMMING_STATUS);
    const showCorrectionSection =
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_CORRECTION_STREAM) ||
        hasMsmSummary;
    const showBaselineSection =
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_SOLUTION_STATUS) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_AZIMUTH) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_PITCH) ||
        hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_LENGTH);
    const hasAdvancedGnssFields = showSatellitesSection || showCn0Section || showCorrectionSection || showBaselineSection;

    const formatCount = (value: number | undefined) => value !== undefined ? String(value) : unknownLabel;
    const formatNumber = (value: number | undefined, precision: number, suffix = "") =>
        value !== undefined ? `${value.toFixed(precision)}${suffix}` : unknownLabel;

    return (
        <Card
            size="small"
            title={<Space><WifiOutlined /> {t("diagnosticsPage.gpsGnssTitle")}</Space>}
        >
            <Space direction="vertical" size={12} style={{ width: "100%" }}>
                <Space wrap size={[12, 8]}>
                    <Space size={4}>
                        <Text type="secondary">{t("diagnosticsPage.fixType")}</Text>
                        <Tag color={liveStatusTagColor(liveStatus.fixType)}>{liveStatus.label}</Tag>
                    </Space>
                    <Space size={4}>
                        <Text type="secondary">{t("diagnosticsPage.rtkMode")}</Text>
                        <Tag color={rtkModeTagColor(gnssStatus?.rtk_mode)}>{rtkModeLabel}</Tag>
                    </Space>
                    <Space size={4}>
                        <Text type="secondary">{t("diagnosticsPage.correctionStreamStatus")}</Text>
                        <Tag color={correctionStreamTagColor(gnssStatus?.correction_stream_status)}>{correctionStreamLabel}</Tag>
                    </Space>
                </Space>

                <Space direction="vertical" size={8} style={{ width: "100%" }}>
                    <Text strong>{t("diagnosticsPage.position")}</Text>
                    <Row gutter={[12, 12]}>
                        <Col span={12}>
                            <Statistic
                                title={t("diagnosticsPage.latitude")}
                                value={latitude ?? "-"}
                                precision={latitude !== undefined ? 9 : undefined}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t("diagnosticsPage.longitude")}
                                value={longitude ?? "-"}
                                precision={longitude !== undefined ? 9 : undefined}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t("diagnosticsPage.altitudeM")}
                                value={altitudeM ?? "-"}
                                precision={altitudeM !== undefined ? 3 : undefined}
                            />
                        </Col>
                        <Col span={12}>
                            <Statistic
                                title={t("diagnosticsPage.horizontalAccuracyM")}
                                value={horizontalAccuracyM ?? "—"}
                                precision={horizontalAccuracyM !== undefined ? 3 : undefined}
                            />
                        </Col>
                    </Row>
                </Space>

                <Space direction="vertical" size={8} style={{ width: "100%" }}>
                    <Text strong>{t("diagnosticsPage.receiver")}</Text>
                    <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
                        <Descriptions.Item label={t("diagnosticsPage.receiver")}>
                            {receiverLabel}
                        </Descriptions.Item>
                        <Descriptions.Item label={t("diagnosticsPage.backend")}>
                            {backendLabel}
                        </Descriptions.Item>
                    </Descriptions>
                </Space>

                {showSatellitesSection && (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Text strong>{t("settingsGnssLiveStatus.satelliteSectionTitle")}</Text>
                        <GnssDiagnosticBarRow
                            label={t("settingsGnssLiveStatus.satellitesUsedOfVisible")}
                            ratio={clampRatio(gpsSatellitesUsed, gpsSatellitesVisible)}
                            testId="gnss-satellites-used"
                            value={`${formatCount(gpsSatellitesUsed)} / ${formatCount(gpsSatellitesVisible)}`}
                        />
                        {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_SATELLITES_TRACKED) && (
                            <GnssDiagnosticBarRow
                                label={t("settingsGnssLiveStatus.satellitesTrackedOfVisible")}
                                ratio={clampRatio(gpsSatellitesTracked, gpsSatellitesVisible)}
                                testId="gnss-satellites-tracked"
                                value={`${formatCount(gpsSatellitesTracked)} / ${formatCount(gpsSatellitesVisible)}`}
                            />
                        )}
                    </Space>
                )}

                {showCn0Section && (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Text strong>{t("settingsGnssLiveStatus.signalQualitySectionTitle")}</Text>
                        {(hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MEAN_CN0) ||
                            hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MAX_CN0)) && (
                            <Text type="secondary" style={{ fontSize: 12 }}>
                                {t("settingsGnssLiveStatus.cn0ScaleNote")}
                            </Text>
                        )}
                        {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MEAN_CN0) && (
                            <GnssDiagnosticBarRow
                                barColor="#52c41a"
                                label={t("settingsGnssLiveStatus.meanCn0DbHz")}
                                ratio={gpsMeanCn0 !== undefined ? gpsMeanCn0 / GNSS_CN0_FULL_SCALE_DB_HZ : undefined}
                                testId="gnss-cn0-mean"
                                value={formatNumber(gpsMeanCn0, 1, " dB-Hz")}
                            />
                        )}
                        {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_MAX_CN0) && (
                            <GnssDiagnosticBarRow
                                barColor="#faad14"
                                label={t("settingsGnssLiveStatus.maxCn0DbHz")}
                                ratio={gpsMaxCn0 !== undefined ? gpsMaxCn0 / GNSS_CN0_FULL_SCALE_DB_HZ : undefined}
                                testId="gnss-cn0-max"
                                value={formatNumber(gpsMaxCn0, 1, " dB-Hz")}
                            />
                        )}
                        <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_INTERFERENCE_STATUS) && (
                                <Descriptions.Item label={t("diagnosticsPage.rfInterference")}>
                                    {conditionTag(
                                        interferenceState,
                                        t("diagnosticsPage.yes"),
                                        t("diagnosticsPage.no"),
                                        unknownLabel,
                                    )}
                                </Descriptions.Item>
                            )}
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_JAMMING_STATUS) && (
                                <Descriptions.Item label={t("diagnosticsPage.jamming")}>
                                    {conditionTag(
                                        jammingState,
                                        t("diagnosticsPage.yes"),
                                        t("diagnosticsPage.no"),
                                        unknownLabel,
                                    )}
                                </Descriptions.Item>
                            )}
                        </Descriptions>
                    </Space>
                )}

                {showCorrectionSection && (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Text strong>{t("settingsGnssLiveStatus.correctionMsmSectionTitle")}</Text>
                        <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_CORRECTION_STREAM) && (
                                <Descriptions.Item label={t("diagnosticsPage.correctionStreamStatus")}>
                                    <Tag color={correctionStreamTagColor(gnssStatus?.correction_stream_status)}>
                                        {correctionStreamLabel}
                                    </Tag>
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmState")}>
                                    <Tag color={msmState.color}>{msmState.label}</Tag>
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmMessageType")}>
                                    {formatCount(hasMsmSummaryValue ? gnssStatus?.msm_summary_message_type : undefined)}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmStationId")}>
                                    {formatCount(hasMsmSummaryValue ? gnssStatus?.msm_summary_station_id : undefined)}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmConstellationsSeen")}>
                                    {normalizeGnssString(
                                        hasMsmSummaryValue ? gnssStatus?.msm_summary_constellations_seen : undefined,
                                    ) || unknownLabel}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmSatelliteCount")}>
                                    {formatCount(hasMsmSummaryValue ? gnssStatus?.msm_summary_satellite_count : undefined)}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmSignalCount")}>
                                    {formatCount(hasMsmSummaryValue ? gnssStatus?.msm_summary_signal_count : undefined)}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmCellCount")}>
                                    {formatCount(hasMsmSummaryValue ? gnssStatus?.msm_summary_cell_count : undefined)}
                                </Descriptions.Item>
                            )}
                            {hasMsmSummary && (
                                <Descriptions.Item label={t("settingsGnssLiveStatus.msmAgeS")}>
                                    {formatNumber(hasMsmSummaryValue ? gnssStatus?.msm_summary_age_s : undefined, 1)}
                                </Descriptions.Item>
                            )}
                        </Descriptions>
                    </Space>
                )}

                {showBaselineSection && (
                    <Space direction="vertical" size={8} style={{ width: "100%" }}>
                        <Text strong>{t("settingsGnssLiveStatus.baselineSectionTitle")}</Text>
                        <Descriptions size="small" column={{ xs: 1, sm: 2 }}>
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_DUAL_ANTENNA_BASELINE) && (
                                <Descriptions.Item label={t("diagnosticsPage.dualAntennaBaseline")}>
                                    <Tag color={dualAntennaBaseline ? "success" : dualAntennaBaseline === false ? "warning" : undefined}>
                                        {dualAntennaBaseline === undefined
                                            ? unknownLabel
                                            : dualAntennaBaseline
                                                ? t("settingsGnssLiveStatus.available")
                                                : t("settingsGnssLiveStatus.unavailable")}
                                    </Tag>
                                </Descriptions.Item>
                            )}
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_SOLUTION_STATUS) && (
                                <Descriptions.Item label={t("diagnosticsPage.baselineSolutionStatus")}>
                                    {baselineSolutionStatus ?? unknownLabel}
                                </Descriptions.Item>
                            )}
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_AZIMUTH) && (
                                <Descriptions.Item label={t("diagnosticsPage.baselineAzimuthDeg")}>
                                    {formatNumber(baselineAzimuth, 2)}
                                </Descriptions.Item>
                            )}
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_PITCH) && (
                                <Descriptions.Item label={t("diagnosticsPage.baselinePitchDeg")}>
                                    {formatNumber(baselinePitch, 2)}
                                </Descriptions.Item>
                            )}
                            {hasGnssCapability(gnssStatus, GnssStatusConstants.CAP_BASELINE_LENGTH) && (
                                <Descriptions.Item label={t("diagnosticsPage.baselineLengthM")}>
                                    {formatNumber(baselineLength, 3)}
                                </Descriptions.Item>
                            )}
                        </Descriptions>
                    </Space>
                )}

                {!hasAdvancedGnssFields && (
                    <Text type="secondary" style={{ fontSize: 11 }}>
                        {t("diagnosticsPage.advancedGnssUnavailable")}
                    </Text>
                )}
            </Space>
        </Card>
    );
};
