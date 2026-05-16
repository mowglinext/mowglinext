import {CheckCircleTwoTone, CloseCircleTwoTone} from "@ant-design/icons";
import {Progress} from "antd";
import {COLORS} from "../theme/colors.ts";

export const booleanFormatter = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.primary}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.danger}/>;
export const booleanFormatterInverted = (value: any) => (value === "On" || value === "Yes") ?
    <CheckCircleTwoTone twoToneColor={COLORS.danger}/> : <CloseCircleTwoTone
        twoToneColor={COLORS.primary}/>;
// Keep in sync with the state_name strings published by main_tree.xml
// (grep for PublishHighLevelStatus). Unknown values fall through to the
// raw string so a freshly-added BT state still renders — just without a
// pretty label.
const STATE_LABELS: Record<string, string> = {
    IDLE_DOCKED: "Docked",
    IDLE: "Idle",
    CHARGING: "Charging",
    PREFLIGHT_CHECK: "Preflight",
    UNDOCKING: "Undocking",
    CALIBRATING_HEADING: "Calibrating",
    MOWING: "Mowing",
    TRANSIT: "Transit",
    SKIP_STRIP: "Skip Strip",
    RETURNING_HOME: "Returning",
    MOWING_COMPLETE: "Mowing Complete",
    RECORDING: "Recording",
    RECORDING_COMPLETE: "Recorded",
    MANUAL_MOWING: "Manual",
    LOW_BATTERY_DOCKING: "Low Battery",
    CRITICAL_BATTERY_DOCKING: "Critical Battery",
    CRITICAL_BATTERY_NAV_FAILED: "Battery Nav Failed",
    RAIN_DETECTED_DOCKING: "Rain",
    RAIN_WAITING: "Waiting Rain",
    RAIN_TIMEOUT: "Rain Timeout",
    RESUMING_AFTER_RAIN: "Resuming",
    RESUMING_UNDOCKING: "Resuming Undock",
    BOUNDARY_RECOVERY: "Boundary Recovery",
    EMERGENCY: "Emergency",
    BOUNDARY_EMERGENCY_STOP: "Boundary Alert",
    UNDOCK_FAILED: "Undock Failed",
    CHARGER_FAILED: "Charger Failed",
    NAV_TO_DOCK_FAILED: "Nav Failed",
    COVERAGE_FAILED_DOCKING: "Coverage Failed",
    MOWING_COMPLETE_DOCKING: "Mowing Complete — Returning to Dock",
};

export const stateRenderer = (value: string | undefined) => {
    if (!value) return "Offline";
    return STATE_LABELS[value] ?? value;
};
export const progressFormatter = (value: any) => {
    return <Progress steps={3} percent={value} size={25} showInfo={false} strokeColor={COLORS.primary}/>
};

export const progressFormatterSmall = (value: any) => {
    return <Progress steps={3} percent={value} size={11} showInfo={false} strokeColor={COLORS.primary}/>
};
