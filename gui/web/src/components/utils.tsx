import {CheckCircleTwoTone, CloseCircleTwoTone} from "@ant-design/icons";
import {Progress} from "antd";
import {COLORS} from "../theme/colors.ts";
import i18n from "../i18n";

export const booleanFormatter = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.primary}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.danger}/>;
export const booleanFormatterInverted = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.danger}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.primary}/>;
// Keep in sync with the state_name strings published by main_tree.xml
// (grep for PublishHighLevelStatus). Unknown values fall through to the
// raw string so a freshly-added BT state still renders — just without a
// pretty label. Values are i18n key suffixes (under the `utils` namespace)
// resolved at render time via the i18n singleton, so labels follow the
// active language without changing this helper's signature (it is called
// from components outside this file).
const STATE_LABEL_KEYS: Record<string, string> = {
    IDLE_DOCKED: "stateAtBase",
    IDLE: "stateIdle",
    CHARGING: "stateCharging",
    PREFLIGHT_CHECK: "statePreflight",
    UNDOCKING: "stateLeavingBase",
    CALIBRATING_HEADING: "stateCalibration",
    MOWING: "stateMowing",
    TRANSIT: "stateTransit",
    PLANNING: "statePlanning",
    OBSTACLE_BACKOFF: "stateObstacleBackoff",
    DYNAMIC_OBSTACLE_CLEARED: "stateObstacleCleared",
    AREA_UNREACHABLE: "stateAreaUnreachable",
    SKIP_STRIP: "stateSkippedStrip",
    RETURNING_HOME: "stateReturningBase",
    MOWING_COMPLETE: "stateMowingComplete",
    RECORDING: "stateRecording",
    RECORDING_COMPLETE: "stateSaved",
    MANUAL_MOWING: "stateManual",
    LOW_BATTERY_DOCKING: "stateLowBattery",
    CRITICAL_BATTERY_DOCKING: "stateCriticalBattery",
    CRITICAL_BATTERY_NAV_FAILED: "stateReturnFailedBattery",
    RAIN_DETECTED_DOCKING: "stateRain",
    RAIN_WAITING: "stateRainWaiting",
    RAIN_TIMEOUT: "stateRainTimeout",
    RESUMING_AFTER_RAIN: "stateResuming",
    RESUMING_UNDOCKING: "stateResumingStart",
    BOUNDARY_RECOVERY: "stateBoundaryRecovery",
    EMERGENCY: "stateEmergencyStop",
    BOUNDARY_EMERGENCY_STOP: "stateBoundaryAlert",
    UNDOCK_FAILED: "stateStartFailed",
    CHARGER_FAILED: "stateChargerFailed",
    NAV_TO_DOCK_FAILED: "stateNavigationFailed",
    COVERAGE_FAILED_DOCKING: "stateCoverageFailed",
};

export const stateRenderer = (value: string | undefined) => {
    if (!value) return i18n.t("utils.stateOffline");
    const labelKey = STATE_LABEL_KEYS[value];
    return labelKey ? i18n.t(`utils.${labelKey}`) : value;
};
export const progressFormatter = (value: any) => {
    return <Progress steps={3} percent={value} size={25} showInfo={false} strokeColor={COLORS.primary}/>
};

export const progressFormatterSmall = (value: any) => {
    return <Progress steps={3} percent={value} size={11} showInfo={false} strokeColor={COLORS.primary}/>
};
