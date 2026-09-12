import {useHighLevelStatus} from "../hooks/useHighLevelStatus.ts";
import {useStatus} from "../hooks/useStatus.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {usePower} from "../hooks/usePower.ts";
import {useGnssStatus} from "../hooks/useGnssStatus.ts";
import {useSettings} from "../hooks/useSettings.ts";
import {computeBatteryPercent} from "../utils/battery.ts";
import {deriveGpsStatus} from "../utils/gpsStatus.ts";
import {PowerMenu} from "./PowerMenu.tsx";
import {useMowerAction} from "./MowerActions.tsx";
import {Badge, Button, Space, Tooltip, Typography} from "antd";
import {PoweroffOutlined, WifiOutlined, AlertOutlined} from "@ant-design/icons"
import {stateRenderer} from "./utils.tsx";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import type {CSSProperties} from "react";
import {useTranslation} from "react-i18next";

// Colour by the NUMERIC high-level state (status_nodes.cpp publishes it every
// tick): 2=AUTONOMOUS / 3=RECORDING / 4=MANUAL_MOWING are active (primary);
// 1=IDLE is resting (warning); 0=NULL/EMERGENCY or unknown is danger. The old
// string-allowlist approach mis-coloured legitimate state=2 substates
// (PLANNING, OBSTACLE_BACKOFF, DYNAMIC_OBSTACLE_CLEARED, AREA_UNREACHABLE) as
// danger-red because the set never enumerated them.
const statusColor = (stateNum: number | undefined, isEmergency: boolean, colors: {primary: string; warning: string; danger: string}): string => {
    if (isEmergency) return colors.danger;
    if (stateNum === 2 || stateNum === 3 || stateNum === 4) return colors.primary;
    if (stateNum === 1) return colors.warning;
    return colors.danger;
};

export const MowerStatus = () => {
    const {t} = useTranslation();
    const {colors, displayMode} = useThemeMode();
    const {highLevelStatus} = useHighLevelStatus();
    const hwStatus = useStatus();
    const emergencyData = useEmergency();
    const power = usePower();
    const gnss = useGnssStatus();
    const {settings} = useSettings();

    // Derive state with fallbacks
    const isEmergency = highLevelStatus.emergency ?? emergencyData.active_emergency ?? false;
    const isCharging = highLevelStatus.is_charging ?? hwStatus.is_charging ?? false;

    const stateName = highLevelStatus.state_name ?? (
        isEmergency ? "EMERGENCY" :
        isCharging ? "CHARGING" :
        hwStatus.mower_status != null ? "IDLE_DOCKED" :
        undefined
    );
    // Numeric high-level state (reliable, published every tick): 2=AUTONOMOUS,
    // 3=RECORDING, 4=MANUAL_MOWING. Drives colour/pulse/mowing so transient
    // state=2 substates (planning, obstacle backoff) never read as idle.
    const stateNum = highLevelStatus.state ?? -1;

    const gpsStatus = deriveGpsStatus(gnss);
    const gpsColor =
        gpsStatus.fixType === "RTK_FIX" ? colors.primary :
        gpsStatus.fixType === "RTK_FLOAT" ? colors.warning :
        gpsStatus.fixType === "GPS_FIX" ? colors.warning :
        colors.danger;

    const batteryPercent = computeBatteryPercent(
        highLevelStatus.battery_percent, power.v_battery, settings,
    );

    const isMowing = stateNum === 2 || stateNum === 3 || stateNum === 4;

    // The colour remains a continuous state cue. Emergency emphasis always
    // pulses; Visual mode can additionally use a restrained active-state cue.
    const pulseAnimation = isEmergency
        ? 'mowerPulseRed 1.5s ease-in-out infinite'
        : isMowing && displayMode === 'visual' ? 'mowerPulseGreen 2.4s ease-in-out infinite' : 'none';

    const hasArea = highLevelStatus.current_area !== undefined && highLevelStatus.current_area >= 0;
    // Coarse sub-path X/Y — the secondary readout.
    const totalSwaths = highLevelStatus.current_path ?? 0;
    const completedSwaths = highLevelStatus.current_path_index ?? 0;
    const hasSwaths = isMowing && totalSwaths > 0;
    // Smooth pose-cursor coverage percent — the PRIMARY %. Read defensively so
    // this compiles before the ROS TS types are regenerated from the updated
    // HighLevelStatus.msg (field added this change).
    const coveragePercent = (highLevelStatus as {coverage_percent?: number}).coverage_percent;
    const hasCoveragePercent = isMowing && coveragePercent !== undefined && coveragePercent > 0;
    // Prefer the smooth coverage_percent; fall back to the coarse swath ratio
    // when it is 0/unset (e.g. at pass start or an older backend).
    const progressPercent = hasCoveragePercent
        ? Math.round(coveragePercent)
        : hasSwaths
            ? Math.round((completedSwaths / totalSwaths) * 100)
            : null;

    // Latched-emergency reset: firmware is the safety authority and only
    // clears the latch when the physical trigger is no longer asserted, so
    // this is a fire-and-forget request — surfaced as a global icon button
    // alongside the status badges so the operator never has to dig into the
    // dashboard hero card to clear it (issue #149).
    const mowerAction = useMowerAction();
    const resetEmergencyAction = mowerAction("emergency", {Emergency: 0});
    const showResetEmergency =
        emergencyData.active_emergency || emergencyData.latched_emergency || isEmergency;

    return (
        <>
            <Space size="small" style={{flexShrink: 0}}>
                <Space size={4}>
                    <Badge
                        color={statusColor(stateNum, isEmergency, colors)}
                        style={{
                            animation: pulseAnimation,
                            borderRadius: '50%',
                            '--mower-status-danger': colors.danger,
                            '--mower-status-active': colors.primary,
                        } as CSSProperties}
                    />
                    <Typography.Text style={{fontSize: 12, color: colors.text, whiteSpace: 'nowrap'}}>
                        {stateRenderer(stateName)}
                    </Typography.Text>
                </Space>
                {isMowing && hasArea && (
                    <Typography.Text style={{fontSize: 11, color: colors.primary, whiteSpace: 'nowrap'}}>
                        A{(highLevelStatus.current_area ?? 0) + 1}
                        {progressPercent !== null ? ` ${progressPercent}%` : ''}
                        {hasSwaths ? ` · ${completedSwaths}/${totalSwaths}` : ''}
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
                    <Tooltip title={t('mowerStatus.rearmEmergencyTooltip')}>
                        <Button
                            danger
                            size="small"
                            icon={<AlertOutlined/>}
                            onClick={resetEmergencyAction}
                        >
                            {t('mowerStatus.rearm')}
                        </Button>
                    </Tooltip>
                )}
                <PowerMenu>
                    {/* Real button (not a Space div) so the power menu is
                        keyboard-focusable/activatable. type="text" keeps the
                        chromeless icon+text look. */}
                    <Button
                        type="text"
                        size="small"
                        aria-label={t('mowerStatus.powerMenuAria')}
                        style={{padding: '0 4px', height: 'auto'}}
                    >
                        <Space size={4}>
                            <PoweroffOutlined style={{
                                color: isCharging ? colors.primary : colors.muted,
                                fontSize: 13,
                            }}/>
                            <Typography.Text style={{fontSize: 12, color: colors.text}}>
                                {batteryPercent}%
                            </Typography.Text>
                        </Space>
                    </Button>
                </PowerMenu>
            </Space>
        </>
    );
}
