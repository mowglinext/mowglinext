import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";
import {restartMowgliNext} from "../utils/containers.ts";
import {useContainerRestart} from "../hooks/useContainerRestart.ts";
import {useMowerAction} from "./MowerActions.tsx";
import {App, Badge, Button, Dropdown, Modal, Space, Tooltip, Typography} from "antd";
import {PoweroffOutlined, ReloadOutlined, DesktopOutlined, WifiOutlined, AlertOutlined} from "@ant-design/icons"
import {stateRenderer} from "./utils.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useApi} from "../hooks/useApi.ts";
import {limeAlpha} from "../theme/colors.ts";
import {SettingOutlined} from "@ant-design/icons";
import type {MenuProps} from "antd";

// Builds the badge pulse keyframes from brand tokens. Green pulse derives
// from the lime hero accent; red pulse derives from colors.danger (rose).
// colors.danger is a hex string ("#FF6B7A"); append an 8-bit alpha suffix
// to get the translucent stops without hardcoding the rose RGB here.
const buildPulseKeyframes = (dangerHex: string) => `
@keyframes mowerPulseGreen {
    0%, 100% { box-shadow: 0 0 0 0 ${limeAlpha(0.6)}; }
    50% { box-shadow: 0 0 0 4px ${limeAlpha(0)}; }
}
@keyframes mowerPulseRed {
    0%, 100% { box-shadow: 0 0 0 0 ${dangerHex}99; }
    50% { box-shadow: 0 0 0 4px ${dangerHex}00; }
}
`;

// Motion states → primary (active); resting states → warning;
// everything else (emergencies, unknown) → danger.
const MOTION_STATES = new Set([
    "MOWING", "TRANSIT", "UNDOCKING", "RETURNING_HOME", "MANUAL_MOWING",
    "RESUMING_AFTER_RAIN", "RESUMING_UNDOCKING", "BOUNDARY_RECOVERY",
    "LOW_BATTERY_DOCKING", "CRITICAL_BATTERY_DOCKING", "RAIN_DETECTED_DOCKING",
    "COVERAGE_FAILED_DOCKING", "SKIP_STRIP", "PREFLIGHT_CHECK",
    "CALIBRATING_HEADING", "RECORDING",
]);
const RESTING_STATES = new Set([
    "IDLE", "IDLE_DOCKED", "CHARGING", "MOWING_COMPLETE",
    "RECORDING_COMPLETE", "RAIN_WAITING", "RAIN_TIMEOUT",
]);

const statusColor = (state: string | undefined, colors: {primary: string; warning: string; danger: string}): string => {
    if (!state) return colors.danger;
    if (MOTION_STATES.has(state)) return colors.primary;
    if (RESTING_STATES.has(state)) return colors.warning;
    return colors.danger;
};

export const MowerStatus = () => {
    const {colors} = useThemeMode();
    const pulseKeyframes = buildPulseKeyframes(colors.danger);
    const {highLevelStatus} = useHighLevelStatus();
    const hwStatus = useStatus();
    const emergencyData = useEmergency();
    const power = usePower();
    const gnss = useGnssStatus();
    const {settings} = useSettings();
    const guiApi = useApi();
    const {notification} = App.useApp();

    // Derive state with fallbacks
    const isEmergency = highLevelStatus.emergency ?? emergencyData.active_emergency ?? false;
    const isCharging = highLevelStatus.is_charging ?? hwStatus.is_charging ?? false;

    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        hwStatus.mower_status != null ? "IDLE" :
        undefined
    );

    const gpsStatus = deriveGpsStatus(gnss);
    const gpsColor =
        gpsStatus.fixType === "RTK_FIX" ? colors.primary :
        gpsStatus.fixType === "RTK_FLOAT" ? colors.warning :
        gpsStatus.fixType === "GPS_FIX" ? colors.warning :
        colors.danger;

    const batteryPercent = computeBatteryPercent(
        highLevelStatus.battery_percent, power.v_battery, settings,
    );

    const isMowing = stateName === "MOWING" || stateName === "TRANSIT" ||
        stateName === "UNDOCKING" || stateName === "RETURNING_HOME" ||
        stateName === "MANUAL_MOWING";

    const pulseAnimation = isEmergency
        ? 'mowerPulseRed 1.5s ease-in-out infinite'
        : isMowing
            ? 'mowerPulseGreen 2s ease-in-out infinite'
            : 'none';

    const hasArea = highLevelStatus.current_area !== undefined && highLevelStatus.current_area >= 0;
    const hasProgress = isMowing && highLevelStatus.current_path_index !== undefined && highLevelStatus.current_path !== undefined && highLevelStatus.current_path > 0;
    const progressPercent = hasProgress
        ? Math.round(((highLevelStatus.current_path_index ?? 0) / (highLevelStatus.current_path ?? 1)) * 100)
        : null;

    // Long-running: container restart + rosbridge reconnect. Lock the menu
    // item until ROS2 is reachable again to prevent duplicate-click storms.
    const mowgliRestart = useContainerRestart({
        pendingLabel: "Redémarrage Mowgli…",
        successMessage: "Mowgli redémarré",
        errorMessage: "Échec du redémarrage Mowgli",
    });
    const restartMowgli = () => mowgliRestart.run(() => restartMowgliNext(guiApi));

    // Latched-emergency reset: firmware is the safety authority and only
    // clears the latch when the physical trigger is no longer asserted, so
    // this is a fire-and-forget request — surfaced as a global icon button
    // alongside the status badges so the operator never has to dig into the
    // dashboard hero card to clear it (issue #149).
    const mowerAction = useMowerAction();
    const resetEmergencyAction = mowerAction("emergency", {Emergency: 0});
    const rebootBoardAction = mowerAction("reboot_board", {});
    const showResetEmergency =
        emergencyData.active_emergency || emergencyData.latched_emergency || isEmergency;

    const rebootSystem = async () => {
        try {
            await guiApi.request({path: "/system/reboot", method: "POST"});
            notification.success({message: "Redémarrage…"});
        } catch (e: any) {
            notification.error({message: "Échec du redémarrage", description: e.message});
        }
    };

    const shutdownSystem = async () => {
        try {
            await guiApi.request({path: "/system/shutdown", method: "POST"});
            notification.success({message: "Extinction…"});
        } catch (e: any) {
            notification.error({message: "Échec de l'extinction", description: e.message});
        }
    };

    const confirmAction = (title: string, content: string, onOk: () => Promise<void>) => {
        Modal.confirm({
            title,
            content,
            okText: "Confirmer",
            okType: "danger",
            cancelText: "Annuler",
            onOk,
        });
    };

    // Beginner-safe items at the top; destructive system/hardware actions
    // (board reset, Pi reboot, shutdown) are tucked into an "Avancé"
    // submenu so they're not hit by accident. Command logic unchanged.
    const powerMenuItems: MenuProps["items"] = [
        {
            key: "restart-mowgli",
            icon: <ReloadOutlined/>,
            label: mowgliRestart.pending ? mowgliRestart.pendingLabel : "Redémarrer Mowgli",
            disabled: mowgliRestart.pending,
            onClick: () => confirmAction("Redémarrer Mowgli", "Cela va redémarrer le conteneur MowgliNext.", restartMowgli),
        },
        {type: "divider"},
        {
            key: "advanced",
            icon: <SettingOutlined/>,
            label: "Avancé",
            children: [
                {
                    key: "reboot-board",
                    icon: <ReloadOutlined/>,
                    label: "Redémarrer la carte (STM32)",
                    onClick: () => confirmAction(
                        "Redémarrer la carte (STM32)",
                        "Réinitialise le firmware STM32 (NVIC_SystemReset). À utiliser pour débloquer une carte figée — par ex. l'IMU renvoyant NaN. Les moteurs/la lame s'arrêtent pendant le reset (~1 s).",
                        rebootBoardAction),
                },
                {
                    key: "reboot",
                    icon: <DesktopOutlined/>,
                    label: "Redémarrer le Raspberry Pi",
                    onClick: () => confirmAction("Redémarrer le Raspberry Pi", "Le système va redémarrer. Vous perdrez la connexion temporairement.", rebootSystem),
                },
                {
                    key: "shutdown",
                    icon: <PoweroffOutlined/>,
                    label: "Éteindre le Raspberry Pi",
                    danger: true,
                    onClick: () => confirmAction("Éteindre le Raspberry Pi", "Le système va s'éteindre. Un accès physique sera nécessaire pour le rallumer.", shutdownSystem),
                },
            ],
        },
    ];

    return (
        <>
            <style>{pulseKeyframes}</style>
            <Space size="small" style={{flexShrink: 0}}>
                <Space size={4}>
                    <Badge
                        color={statusColor(stateName, colors)}
                        style={{animation: pulseAnimation, borderRadius: '50%'}}
                    />
                    <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                        {stateRenderer(stateName)}
                    </Typography.Text>
                </Space>
                {isMowing && hasArea && (
                    <Typography.Text style={{fontSize: 11, color: colors.primary, whiteSpace: 'nowrap'}}>
                        A{(highLevelStatus.current_area ?? 0) + 1}
                        {progressPercent !== null ? ` ${progressPercent}%` : ''}
                    </Typography.Text>
                )}
                <Tooltip title={`GPS: ${gpsStatus.label}`}>
                    <Space size={4}>
                        <WifiOutlined style={{color: gpsColor, fontSize: 13}}/>
                        <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                            {gpsStatus.percent}% · {gpsStatus.label}
                        </Typography.Text>
                    </Space>
                </Tooltip>
                {showResetEmergency && (
                    <Tooltip title="Réarmer l'arrêt d'urgence (le firmware décide si le verrou se libère)">
                        <Button
                            danger
                            size="small"
                            icon={<AlertOutlined/>}
                            onClick={resetEmergencyAction}
                        >
                            Réarmer
                        </Button>
                    </Tooltip>
                )}
                <Dropdown menu={{items: powerMenuItems}} trigger={["click"]} placement="bottomRight">
                    <Space size={4} style={{cursor: "pointer"}}>
                        <PoweroffOutlined style={{
                            color: isCharging ? colors.primary : colors.muted,
                            fontSize: 13,
                        }}/>
                        <Typography.Text style={{fontSize: 12, color: colors.text}}>
                            {batteryPercent}%
                        </Typography.Text>
                    </Space>
                </Dropdown>
            </Space>
        </>
    );
}
