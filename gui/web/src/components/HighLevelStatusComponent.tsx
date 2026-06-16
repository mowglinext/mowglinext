import {Col, Row, Statistic} from "antd";
import {booleanFormatter, booleanFormatterInverted, progressFormatter, stateRenderer} from "./utils.tsx";
import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {computeBatteryPercent, BATTERY_DEFAULTS} from "../utils/battery.ts";

export function HighLevelStatusComponent() {
    const {colors} = useThemeMode();
    const {highLevelStatus} = useHighLevelStatus()
    const power = usePower()
    const status = useStatus()
    const emergency = useEmergency()
    const gnssStatus = useGnssStatus()
    const {settings} = useSettings()

    // Derive charging state: prefer highLevelStatus, fall back to status topic
    const isCharging = highLevelStatus.is_charging ?? status.is_charging ?? false;

    // Derive emergency state: prefer highLevelStatus, fall back to emergency topic
    const isEmergency = highLevelStatus.emergency ?? emergency.active_emergency ?? false;

    // Derive battery percentage: prefer highLevelStatus, fall back to voltage-based estimate
    const batteryPercent = computeBatteryPercent(
        highLevelStatus.battery_percent, power.v_battery, settings,
    );

    // Derive GPS quality from the typed GNSS runtime state.
    const gpsQuality = gnssStatus.quality_percent;

    // Derive state name: prefer highLevelStatus, fall back to basic inference
    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        status.mower_status != null ? "IDLE" :
        undefined
    );

    const estimateRemainingChargingTime = () => {
        if (!power.v_battery || !power.charge_current || power.charge_current == 0) {
            return "∞"
        }
        const capacity = (settings["battery_capacity_mah"] ?? "3000.0");
        const full = (settings["battery_full_voltage"] ?? BATTERY_DEFAULTS.fullVoltage);
        const empty = (settings["battery_empty_voltage"] ?? BATTERY_DEFAULTS.emptyVoltage);
        if (!capacity || !full || !empty) {
            return "∞"
        }
        const estimatedAmpsPerVolt = parseFloat(capacity) / (parseFloat(full) - parseFloat(empty))
        const estimatedRemainingAmps = (parseFloat(full) - (power.v_battery ?? 0)) * estimatedAmpsPerVolt;
        if (estimatedRemainingAmps < 10) {
            return "∞"
        }
        const remaining = estimatedRemainingAmps / ((power.charge_current ?? 0) * 1000)
        if (remaining < 0) {
            return "∞"
        }
        return Date.now() + remaining * (1000 * 60 * 60)
    };
    return <Row gutter={[16, 16]}>
        <Col lg={6} xs={12}><Statistic title="État" valueStyle={{color: colors.primary}}
                                       value={stateRenderer(stateName)}/></Col>
        <Col lg={6} xs={12}>{gpsQuality == null ?
            <Statistic title="GPS" value="--"/> :
            <Statistic title="GPS" precision={2}
                       value={gpsQuality}
                       suffix={"%"}/>}
        </Col>
        <Col lg={6} xs={12}><Statistic title="Batterie" value={batteryPercent}
                                       formatter={progressFormatter}/></Col>
        <Col lg={6} xs={12}>{isCharging ?
            <Statistic.Countdown title="Fin de charge" format={"HH:mm"}
                                       value={estimateRemainingChargingTime()}/> :
            <Statistic title="Fin de charge" value="--:--"/>}
        </Col>
        <Col lg={6} xs={12}><Statistic title="En charge" value={isCharging ? "Yes" : "No"}
                                       formatter={booleanFormatter}/></Col>
        <Col lg={6} xs={12}><Statistic title="Arrêt d'urgence" value={isEmergency ? "Yes" : "No"}
                                       formatter={booleanFormatterInverted}/></Col>
    </Row>;
}
