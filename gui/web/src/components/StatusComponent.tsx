import {Card, Col, Row, Statistic, Tag, Space, Flex, Collapse, Typography} from "antd";
import {
    ThunderboltOutlined,
    WarningOutlined,
    ApiOutlined,
    SoundOutlined,
    DashboardOutlined,
} from "@ant-design/icons";
import {useStatus} from "../hooks/useStatus.ts";
import {usePower} from "../hooks/usePower.ts";
import {useEmergency} from "../hooks/useEmergency.ts";
import {useDockingSensor} from "../hooks/useDockingSensor.ts";
import {useThemeMode} from "../theme/ThemeContext.tsx";
import {useTranslation} from "react-i18next";
import type {TFunction} from "i18next";

const StatusTag = ({label, active, color}: { label: string; active: boolean; color?: string }) => (
    <Tag color={active ? (color ?? "green") : "default"}>{label}</Tag>
);

type DotLevel = "ok" | "warn" | "danger";

/** Plain green/amber/red dot + label line for the beginner-friendly summary. */
const StatusDot = ({label, level, value}: { label: string; level: DotLevel; value: string }) => {
    const {colors} = useThemeMode();
    const dotColor = level === "ok" ? colors.success : level === "warn" ? colors.warning : colors.danger;
    return (
        <Flex align="center" justify="space-between" gap="small" style={{padding: "4px 0"}}>
            <Space size={8}>
                <span style={{
                    display: "inline-block", width: 9, height: 9, borderRadius: "50%",
                    background: dotColor, flexShrink: 0,
                }}/>
                <Typography.Text style={{color: colors.text}}>{label}</Typography.Text>
            </Space>
            <Typography.Text style={{color: colors.textSecondary, fontSize: 13}}>{value}</Typography.Text>
        </Flex>
    );
};

/**
 * Derive a single charging label from power + status data.
 * Avoids showing 3 separate badges for the same info.
 */
function chargingLabel(chargerEnabled: boolean, isCharging: boolean, t: TFunction): { label: string; active: boolean; color: string } {
    if (isCharging) {
        return {label: t('statusComponent.charging'), active: true, color: "cyan"};
    }
    if (chargerEnabled) {
        return {label: t('statusComponent.chargerActive'), active: true, color: "green"};
    }
    return {label: t('statusComponent.notCharging'), active: false, color: "default"};
}

/**
 * Derive a single emergency label from emergency data.
 */
function emergencyLabel(active: boolean, latched: boolean, t: TFunction, reason?: string): { label: string; color: string } {
    if (active) {
        return {label: reason ? t('statusComponent.emergencyWithReason', {reason}) : t('statusComponent.emergencyStopUpper'), color: "red"};
    }
    if (latched) {
        return {label: reason ?? t('statusComponent.latched'), color: "orange"};
    }
    return {label: "OK", color: "default"};
}

export function StatusComponent({compact}: {compact?: boolean}) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const status = useStatus();
    const power = usePower();
    const emergency = useEmergency();
    const dockingSensor = useDockingSensor();

    const mowerStatusLabel = status.mower_status === 255 ? "OK" : t('statusComponent.initializing');
    const charging = chargingLabel(!!power.charger_enabled, !!status.is_charging, t);
    const emg = emergencyLabel(!!emergency.active_emergency, !!emergency.latched_emergency, t, emergency.reason);

    if (compact) {
        return (
            <div style={{display: 'flex', flexDirection: 'column', gap: 12}}>
                {/* Combined: System Status + Emergency + Docking */}
                <div style={{
                    background: colors.bgCard,
                    borderRadius: 12,
                    padding: 16,
                }}>
                    <Space style={{marginBottom: 8}}>
                        <ApiOutlined style={{color: colors.textSecondary}}/>
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>{t('statusComponent.systemAndSafety')}</span>
                    </Space>
                    <Flex wrap gap="small" style={{marginBottom: 8}}>
                        <StatusTag label={t('statusComponent.mowerWithStatus', {status: mowerStatusLabel})} active={status.mower_status === 255}/>
                        <StatusTag label={t('statusComponent.rpiPower')} active={!!status.raspberry_pi_power}/>
                        <StatusTag label={t('statusComponent.mowEnabled')} active={!!status.mow_enabled}/>
                        <StatusTag label={status.rain_detected ? t('statusComponent.rain') : t('statusComponent.noRain')}
                                   active={!!status.rain_detected} color="blue"/>
                    </Flex>
                    <div style={{borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8, marginBottom: 8}}>
                        <Flex wrap gap="small">
                            <Tag color={emg.color}>{emg.label}</Tag>
                            <StatusTag label={dockingSensor.dock_present ? t('statusComponent.basePresent') : t('statusComponent.baseAbsent')}
                                       active={!!dockingSensor.dock_present} color="cyan"/>
                        </Flex>
                    </div>
                </div>

                {/* Combined: Power + Motor */}
                <div style={{
                    background: colors.bgCard,
                    borderRadius: 12,
                    padding: 16,
                }}>
                    <Space style={{marginBottom: 8}}>
                        <ThunderboltOutlined style={{color: colors.textSecondary}}/>
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>{t('statusComponent.energyAndMotor')}</span>
                    </Space>
                    <Row gutter={[12, 8]}>
                        <Col span={8}>
                            <Statistic title={t('statusComponent.battery')} value={power.v_battery} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('statusComponent.charge')} value={power.v_charge} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title={t('statusComponent.current')} value={power.charge_current} precision={1} suffix="A"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                    </Row>
                    <div style={{borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8, marginTop: 8}}>
                        <Row gutter={[12, 8]}>
                            <Col span={8}>
                                <Statistic title="RPM" value={status.mower_motor_rpm} precision={0}
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                            <Col span={8}>
                                <Statistic title={t('statusComponent.motor')} value={status.mower_motor_temperature} precision={0} suffix="°C"
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                            <Col span={8}>
                                <Statistic title="ESC" value={status.mower_esc_temperature} precision={0} suffix="°C"
                                           valueStyle={{fontSize: 16}}/>
                            </Col>
                        </Row>
                    </div>
                </div>
            </div>
        );
    }

    // ── Beginner-friendly summary levels ─────────────────────────────────────
    const motorTemp = status.mower_motor_temperature ?? 0;
    const escTemp = status.mower_esc_temperature ?? 0;
    const tempLevel = (t: number): DotLevel => (t > 70 ? "danger" : t > 55 ? "warn" : "ok");

    const systemLevel: DotLevel = status.mower_status === 255 ? "ok" : "warn";
    const emgLevel: DotLevel = emergency.active_emergency
        ? "danger"
        : emergency.latched_emergency
            ? "warn"
            : "ok";
    const chargeLevel: DotLevel = status.is_charging ? "ok" : power.charger_enabled ? "ok" : "warn";

    return <Row gutter={[16, 16]}>
        {/* Beginner summary — plain French labels with green/amber/red dots */}
        <Col span={24}>
            <Card title={<Space><ApiOutlined/> {t('statusComponent.summary')}</Space>} size="small">
                <Row gutter={[24, 4]}>
                    <Col xs={24} md={12}>
                        <StatusDot label={t('statusComponent.mowerState')} level={systemLevel} value={mowerStatusLabel}/>
                        <StatusDot label={t('statusComponent.power')} level={status.raspberry_pi_power ? "ok" : "danger"}
                                   value={status.raspberry_pi_power ? "OK" : t('statusComponent.cutOff')}/>
                        <StatusDot label={t('statusComponent.uiBoard')} level={status.ui_board_available ? "ok" : "warn"}
                                   value={status.ui_board_available ? t('statusComponent.available') : t('statusComponent.absent')}/>
                        <StatusDot label={t('statusComponent.rain')} level={status.rain_detected ? "warn" : "ok"}
                                   value={status.rain_detected ? t('statusComponent.detected') : t('statusComponent.none')}/>
                    </Col>
                    <Col xs={24} md={12}>
                        <StatusDot label={t('statusComponent.emergencyStop')} level={emgLevel} value={emg.label}/>
                        <StatusDot label={t('statusComponent.charge')} level={chargeLevel} value={charging.label}/>
                        <StatusDot label={t('statusComponent.base')} level={dockingSensor.dock_present ? "ok" : "warn"}
                                   value={dockingSensor.dock_present ? t('statusComponent.present') : t('statusComponent.absent')}/>
                        <StatusDot label={t('statusComponent.motorTemperature')} level={tempLevel(motorTemp)}
                                   value={`${motorTemp.toFixed(0)} °C`}/>
                    </Col>
                </Row>
            </Card>
        </Col>

        {/* Expert detail behind an "Avancé" collapse so beginners aren't overwhelmed */}
        <Col span={24}>
            <Collapse
                size="small"
                ghost
                items={[{
                    key: "advanced",
                    label: <Space><DashboardOutlined/> {t('statusComponent.advanced')}</Space>,
                    children: (
                        <Row gutter={[16, 16]}>
                            {/* System Status */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><ApiOutlined/> {t('statusComponent.systemStatus')}</Space>} size="small">
                                    <Flex wrap gap="small" style={{marginBottom: 16}}>
                                        <StatusTag label={t('statusComponent.mowerWithStatus', {status: mowerStatusLabel})} active={status.mower_status === 255}/>
                                        <StatusTag label={t('statusComponent.rpiPower')} active={!!status.raspberry_pi_power}/>
                                        <StatusTag label={t('statusComponent.escPower')} active={!!status.esc_power}/>
                                        <StatusTag label={t('statusComponent.uiBoard')} active={!!status.ui_board_available}/>
                                        <StatusTag label={t('statusComponent.mowEnabled')} active={!!status.mow_enabled}/>
                                    </Flex>
                                    <Flex wrap gap="small">
                                        <StatusTag label={t('statusComponent.soundModule')} active={!!status.sound_module_available}/>
                                        <StatusTag label={status.sound_module_busy ? t('statusComponent.soundBusy') : t('statusComponent.soundIdle')}
                                                   active={!!status.sound_module_busy} color="orange"/>
                                        <StatusTag label={status.rain_detected ? t('statusComponent.rainDetected') : t('statusComponent.noRain')}
                                                   active={!!status.rain_detected} color="blue"/>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Power */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><ThunderboltOutlined/> {t('statusComponent.energy')}</Space>} size="small">
                                    <Row gutter={[16, 16]}>
                                        <Col span={8}>
                                            <Statistic title={t('statusComponent.battery')} value={power.v_battery} precision={2} suffix="V"/>
                                        </Col>
                                        <Col span={8}>
                                            <Statistic title={t('statusComponent.charge')} value={power.v_charge} precision={2} suffix="V"/>
                                        </Col>
                                        <Col span={8}>
                                            <Statistic title={t('statusComponent.current')} value={power.charge_current} precision={2} suffix="A"/>
                                        </Col>
                                    </Row>
                                    <Flex wrap gap="small" style={{marginTop: 12}}>
                                        <Tag color={charging.color}>{charging.label}</Tag>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Emergency */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><WarningOutlined/> {t('statusComponent.emergencyStop')}</Space>} size="small"
                                      styles={{body: {paddingBottom: 12}}}>
                                    <Flex wrap gap="small" align="center">
                                        <Tag color={emg.color}>{emg.label}</Tag>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Docking Sensor */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><DashboardOutlined/> {t('statusComponent.baseSensor')}</Space>} size="small"
                                      styles={{body: {paddingBottom: 12}}}>
                                    <Flex wrap gap="small">
                                        <StatusTag label={dockingSensor.dock_present ? t('statusComponent.present') : t('statusComponent.absent')}
                                                   active={!!dockingSensor.dock_present} color="cyan"/>
                                        <StatusTag label={t('statusComponent.distanceValue', {value: dockingSensor.dock_distance?.toFixed(2) ?? "-"})}
                                                   active={(dockingSensor.dock_distance ?? 0) > 0} color="cyan"/>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Mower Motor */}
                            <Col span={24}>
                                <Card title={<Space><SoundOutlined/> {t('statusComponent.cuttingMotor')}</Space>} size="small">
                                    <Row gutter={[16, 16]}>
                                        <Col lg={4} xs={8}>
                                            <Statistic title={t('statusComponent.escStatus')} value={status.mower_esc_status}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title="RPM" value={status.mower_motor_rpm} precision={0}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title={t('statusComponent.current')} value={status.mower_esc_current} precision={2} suffix="A"/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title={t('statusComponent.tempEsc')} value={escTemp} precision={1} suffix="°C"
                                                       valueStyle={{color: tempLevel(escTemp) === "danger" ? colors.danger : tempLevel(escTemp) === "warn" ? colors.warning : undefined}}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title={t('statusComponent.tempMotor')} value={motorTemp} precision={1} suffix="°C"
                                                       valueStyle={{color: tempLevel(motorTemp) === "danger" ? colors.danger : tempLevel(motorTemp) === "warn" ? colors.warning : undefined}}/>
                                        </Col>
                                    </Row>
                                </Card>
                            </Col>
                        </Row>
                    ),
                }]}
            />
        </Col>
    </Row>;
}
