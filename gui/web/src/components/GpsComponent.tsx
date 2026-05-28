import {Col, Row, Statistic} from "antd";
import {useGPS} from "../hooks/useGPS.ts";
import { booleanFormatter, booleanFormatterInverted } from "./utils.tsx";
import { GnssStatusConstants } from "../types/ros.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {deriveGpsStatus, gnssReceiverLabel, readGnssNumber} from "../utils/gpsStatus.ts";

export function GpsComponent() {
    const {colors} = useThemeMode();
    const gps = useGPS();
    const gnssStatus = useGnssStatus();

    const fixTypeCode = gnssStatus.fix_type ?? GnssStatusConstants.FIX_TYPE_NO_FIX;
    const hasRtk =
        fixTypeCode === GnssStatusConstants.FIX_TYPE_RTK_FIXED ||
        fixTypeCode === GnssStatusConstants.FIX_TYPE_RTK_FLOAT;
    const gpsStatus = deriveGpsStatus(gnssStatus);
    const accuracyM = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_HORIZONTAL_ACCURACY,
        gnssStatus.horizontal_accuracy_m,
    );
    const satellitesUsed = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_SATELLITES_USED,
        gnssStatus.satellites_used,
    );
    const correctionAgeS = readGnssNumber(
        gnssStatus,
        GnssStatusConstants.CAP_CORRECTION_AGE,
        gnssStatus.correction_age_s,
    );
    const fixType = gpsStatus.label;
    const receiverLabel = gnssReceiverLabel(gnssStatus);
    const fixColor =
        gpsStatus.fixType === "RTK_FIX" ? colors.primary
        : gpsStatus.fixType === "RTK_FLOAT" ? colors.warning
        : gpsStatus.fixType === "GPS_FIX" ? colors.primary
        : colors.danger;

    return <>
        <Row gutter={[16, 16]}>
            <Col lg={8} xs={24}><Statistic precision={2} title="Position X (m)"
                                        value={gps.pose?.pose?.position?.x}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Position Y (m)"
                                        value={gps.pose?.pose?.position?.y}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Altitude" value={gps.pose?.pose?.position?.z}/></Col>
            <Col lg={8} xs={24}><Statistic precision={2} title="Orientation"
                                        value={gps.pose?.pose?.orientation?.z}/></Col>
            <Col lg={8} xs={24}><Statistic title="Receiver" value={receiverLabel}/></Col>
            <Col lg={8} xs={24}><Statistic precision={3} title="Accuracy (m)" value={accuracyM}/></Col>
            <Col lg={8} xs={24}><Statistic precision={0} title="Satellites used" value={satellitesUsed}/></Col>
            </Row>
        <Row gutter={[16, 16]}>
            <Col lg={8} xs={24}><Statistic title="RTK" value={hasRtk ? "Yes" : "No"}
                                        formatter={booleanFormatter}/></Col>
            <Col lg={8} xs={24}><Statistic title="Fix type" value={fixType}
                                        valueStyle={{color: fixColor}}/></Col>
            <Col lg={8} xs={24}><Statistic title="Dead reckoning" value={fixTypeCode === GnssStatusConstants.FIX_TYPE_DEAD_RECKONING ? "Yes" : "No"}
                                        formatter={booleanFormatterInverted}/></Col>
            <Col lg={8} xs={24}><Statistic precision={1} title="Last correction (s)" value={correctionAgeS}/></Col>
        </Row>
    </>;
}
