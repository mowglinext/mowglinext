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
    IDLE_DOCKED: "À la base",
    IDLE: "Au repos",
    CHARGING: "En charge",
    PREFLIGHT_CHECK: "Pré-vol",
    UNDOCKING: "Départ base",
    CALIBRATING_HEADING: "Calibration",
    MOWING: "Tonte",
    TRANSIT: "Transit",
    SKIP_STRIP: "Bande ignorée",
    RETURNING_HOME: "Retour base",
    MOWING_COMPLETE: "Tonte terminée",
    RECORDING: "Enregistrement",
    RECORDING_COMPLETE: "Enregistré",
    MANUAL_MOWING: "Manuel",
    LOW_BATTERY_DOCKING: "Batterie faible",
    CRITICAL_BATTERY_DOCKING: "Batterie critique",
    CRITICAL_BATTERY_NAV_FAILED: "Échec retour (batterie)",
    RAIN_DETECTED_DOCKING: "Pluie",
    RAIN_WAITING: "Attente pluie",
    RAIN_TIMEOUT: "Délai pluie dépassé",
    RESUMING_AFTER_RAIN: "Reprise",
    RESUMING_UNDOCKING: "Reprise départ",
    BOUNDARY_RECOVERY: "Récupération limite",
    EMERGENCY: "Arrêt d'urgence",
    BOUNDARY_EMERGENCY_STOP: "Alerte limite",
    UNDOCK_FAILED: "Échec départ",
    CHARGER_FAILED: "Échec chargeur",
    NAV_TO_DOCK_FAILED: "Échec navigation",
    COVERAGE_FAILED_DOCKING: "Échec couverture",
};

export const stateRenderer = (value: string | undefined) => {
    if (!value) return "Hors ligne";
    return STATE_LABELS[value] ?? value;
};
export const progressFormatter = (value: any) => {
    return <Progress steps={3} percent={value} size={25} showInfo={false} strokeColor={COLORS.primary}/>
};

export const progressFormatterSmall = (value: any) => {
    return <Progress steps={3} percent={value} size={11} showInfo={false} strokeColor={COLORS.primary}/>
};
