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
function chargingLabel(chargerEnabled: boolean, isCharging: boolean): { label: string; active: boolean; color: string } {
    if (isCharging) {
        return {label: "En charge", active: true, color: "cyan"};
    }
    if (chargerEnabled) {
        return {label: "Chargeur actif", active: true, color: "green"};
    }
    return {label: "Pas en charge", active: false, color: "default"};
}

/**
 * Derive a single emergency label from emergency data.
 */
function emergencyLabel(active: boolean, latched: boolean, reason?: string): { label: string; color: string } {
    if (active) {
        return {label: reason ? `Urgence : ${reason}` : "ARRÊT D'URGENCE", color: "red"};
    }
    if (latched) {
        return {label: reason ?? "Verrouillé (appuyez sur play pour libérer)", color: "orange"};
    }
    return {label: "OK", color: "default"};
}

export function StatusComponent({compact}: {compact?: boolean}) {
    const {colors} = useThemeMode();
    const status = useStatus();
    const power = usePower();
    const emergency = useEmergency();
    const dockingSensor = useDockingSensor();

    const mowerStatusLabel = status.mower_status === 255 ? "OK" : "Initialisation";
    const charging = chargingLabel(!!power.charger_enabled, !!status.is_charging);
    const emg = emergencyLabel(!!emergency.active_emergency, !!emergency.latched_emergency, emergency.reason);

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
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>Système & sécurité</span>
                    </Space>
                    <Flex wrap gap="small" style={{marginBottom: 8}}>
                        <StatusTag label={`Tondeuse : ${mowerStatusLabel}`} active={status.mower_status === 255}/>
                        <StatusTag label="Alim. RPi" active={!!status.raspberry_pi_power}/>
                        <StatusTag label="Tonte activée" active={!!status.mow_enabled}/>
                        <StatusTag label={status.rain_detected ? "Pluie" : "Pas de pluie"}
                                   active={!!status.rain_detected} color="blue"/>
                    </Flex>
                    <div style={{borderTop: `1px solid ${colors.borderSubtle}`, paddingTop: 8, marginBottom: 8}}>
                        <Flex wrap gap="small">
                            <Tag color={emg.color}>{emg.label}</Tag>
                            <StatusTag label={dockingSensor.dock_present ? "Base : présente" : "Base : absente"}
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
                        <span style={{color: colors.textSecondary, fontSize: 13, fontWeight: 500}}>Énergie & moteur</span>
                    </Space>
                    <Row gutter={[12, 8]}>
                        <Col span={8}>
                            <Statistic title="Batterie" value={power.v_battery} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Charge" value={power.v_charge} precision={1} suffix="V"
                                       valueStyle={{fontSize: 16}}/>
                        </Col>
                        <Col span={8}>
                            <Statistic title="Courant" value={power.charge_current} precision={1} suffix="A"
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
                                <Statistic title="Moteur" value={status.mower_motor_temperature} precision={0} suffix="°C"
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
            <Card title={<Space><ApiOutlined/> Résumé</Space>} size="small">
                <Row gutter={[24, 4]}>
                    <Col xs={24} md={12}>
                        <StatusDot label="État de la tondeuse" level={systemLevel} value={mowerStatusLabel}/>
                        <StatusDot label="Alimentation" level={status.raspberry_pi_power ? "ok" : "danger"}
                                   value={status.raspberry_pi_power ? "OK" : "Coupée"}/>
                        <StatusDot label="Carte UI" level={status.ui_board_available ? "ok" : "warn"}
                                   value={status.ui_board_available ? "Disponible" : "Absente"}/>
                        <StatusDot label="Pluie" level={status.rain_detected ? "warn" : "ok"}
                                   value={status.rain_detected ? "Détectée" : "Aucune"}/>
                    </Col>
                    <Col xs={24} md={12}>
                        <StatusDot label="Arrêt d'urgence" level={emgLevel} value={emg.label}/>
                        <StatusDot label="Charge" level={chargeLevel} value={charging.label}/>
                        <StatusDot label="Base" level={dockingSensor.dock_present ? "ok" : "warn"}
                                   value={dockingSensor.dock_present ? "Présente" : "Absente"}/>
                        <StatusDot label="Température moteur" level={tempLevel(motorTemp)}
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
                    label: <Space><DashboardOutlined/> Avancé</Space>,
                    children: (
                        <Row gutter={[16, 16]}>
                            {/* System Status */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><ApiOutlined/> État système</Space>} size="small">
                                    <Flex wrap gap="small" style={{marginBottom: 16}}>
                                        <StatusTag label={`Tondeuse : ${mowerStatusLabel}`} active={status.mower_status === 255}/>
                                        <StatusTag label="Alim. RPi" active={!!status.raspberry_pi_power}/>
                                        <StatusTag label="Alim. ESC" active={!!status.esc_power}/>
                                        <StatusTag label="Carte UI" active={!!status.ui_board_available}/>
                                        <StatusTag label="Tonte activée" active={!!status.mow_enabled}/>
                                    </Flex>
                                    <Flex wrap gap="small">
                                        <StatusTag label="Module son" active={!!status.sound_module_available}/>
                                        <StatusTag label={status.sound_module_busy ? "Son : occupé" : "Son : inactif"}
                                                   active={!!status.sound_module_busy} color="orange"/>
                                        <StatusTag label={status.rain_detected ? "Pluie détectée" : "Pas de pluie"}
                                                   active={!!status.rain_detected} color="blue"/>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Power */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><ThunderboltOutlined/> Énergie</Space>} size="small">
                                    <Row gutter={[16, 16]}>
                                        <Col span={8}>
                                            <Statistic title="Batterie" value={power.v_battery} precision={2} suffix="V"/>
                                        </Col>
                                        <Col span={8}>
                                            <Statistic title="Charge" value={power.v_charge} precision={2} suffix="V"/>
                                        </Col>
                                        <Col span={8}>
                                            <Statistic title="Courant" value={power.charge_current} precision={2} suffix="A"/>
                                        </Col>
                                    </Row>
                                    <Flex wrap gap="small" style={{marginTop: 12}}>
                                        <Tag color={charging.color}>{charging.label}</Tag>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Emergency */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><WarningOutlined/> Arrêt d'urgence</Space>} size="small"
                                      styles={{body: {paddingBottom: 12}}}>
                                    <Flex wrap gap="small" align="center">
                                        <Tag color={emg.color}>{emg.label}</Tag>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Docking Sensor */}
                            <Col lg={12} xs={24}>
                                <Card title={<Space><DashboardOutlined/> Capteur de base</Space>} size="small"
                                      styles={{body: {paddingBottom: 12}}}>
                                    <Flex wrap gap="small">
                                        <StatusTag label={dockingSensor.dock_present ? "Présente" : "Absente"}
                                                   active={!!dockingSensor.dock_present} color="cyan"/>
                                        <StatusTag label={`Distance : ${dockingSensor.dock_distance?.toFixed(2) ?? "-"} m`}
                                                   active={(dockingSensor.dock_distance ?? 0) > 0} color="cyan"/>
                                    </Flex>
                                </Card>
                            </Col>

                            {/* Mower Motor */}
                            <Col span={24}>
                                <Card title={<Space><SoundOutlined/> Moteur de coupe</Space>} size="small">
                                    <Row gutter={[16, 16]}>
                                        <Col lg={4} xs={8}>
                                            <Statistic title="État ESC" value={status.mower_esc_status}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title="RPM" value={status.mower_motor_rpm} precision={0}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title="Courant" value={status.mower_esc_current} precision={2} suffix="A"/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title="Temp. ESC" value={escTemp} precision={1} suffix="°C"
                                                       valueStyle={{color: tempLevel(escTemp) === "danger" ? colors.danger : tempLevel(escTemp) === "warn" ? colors.warning : undefined}}/>
                                        </Col>
                                        <Col lg={5} xs={12}>
                                            <Statistic title="Temp. moteur" value={motorTemp} precision={1} suffix="°C"
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
