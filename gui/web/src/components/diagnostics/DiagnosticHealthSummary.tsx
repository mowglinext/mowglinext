import {Card, Flex, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {DashCard} from '../dashboard/Card';
import {useThemeMode} from '../../theme/ThemeContext';
import {useIsMobile} from '../../hooks/useIsMobile';
import type {deriveDiagnosticHealth, DiagnosticHealthLevel} from '../../utils/diagnosticHealth';

type Health = ReturnType<typeof deriveDiagnosticHealth>;
const tone: Record<DiagnosticHealthLevel, string> = {ok: 'success', warning: 'warning', critical: 'error', unknown: 'default', not_applicable: 'default'};
const NUMBER = '\uFFF0';
function NumericLabel({label, value}: {label: string; value?: string}) {
    const parts = label.split(NUMBER);
    return parts.length > 1 && value !== undefined
        ? <>{parts[0]}<span className="diagnostics-health-badge-number" style={{inlineSize: '3ch'}}>{value}</span>{parts.slice(1).join(NUMBER)}</>
        : <>{label}</>;
}
export function DiagnosticHealthSummary({health, gpsLabel, batteryPercent, cpuTemp, cpuUsage}: {
    health: Health; gpsLabel: string; batteryPercent: number; cpuTemp?: number; cpuUsage?: number | null;
}) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const mobile = useIsMobile();
    const verdict = health.level === 'critical'
        ? health.primary.reason === 'clearance' ? t('telemetry.emergencyClearanceUnverified')
            : health.primary.id === 'emergency' ? t('diagnosticsPage.verdictEmergency') : t('diagnosticsPage.verdictCritical')
        : health.level === 'warning' ? t('diagnosticsPage.verdictAttention')
            : health.level === 'ok' ? t('diagnosticsPage.verdictAllGood') : t('telemetry.statusUnknown');
    const color = health.level === 'critical' ? colors.danger : health.level === 'warning' ? colors.warning
        : health.level === 'ok' ? colors.primary : colors.textMuted;
    const badge = (id: string, label: string, numeric?: string) => {
        const item = health.checks.find(c => c.id === id);
        if (!item) return null;
        return <Tag key={id} data-testid={`diagnostic-health-${id}`} data-level={item.level} color={tone[item.level]}
            style={{fontSize: 12, padding: '2px 8px'}}>
            <NumericLabel label={label} value={numeric}/>
            {' · '}{t(`diagnosticHealth.level.${item.level}`)}
        </Tag>;
    };
    const battery = health.checks.find(c => c.id === 'battery');
    const cpu = health.checks.find(c => c.id === 'cpu');
    return <>
        <DashCard tone={health.level === 'critical' ? 'danger' : 'glow'} style={{marginBottom: 4}}>
            <div style={{fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600, marginBottom: 8}}>
                {t('diagnosticsPage.robotStatus')} · {t('telemetry.freshnessUnverified')}
            </div>
            <div className="mn-num" data-testid="diagnostic-health-verdict" data-level={health.level}
                style={{fontSize: mobile ? 40 : 56, lineHeight: 1, color}}>{verdict}</div>
            <div data-testid="diagnostic-health-cause" style={{fontSize: 13, color: colors.textSecondary, marginTop: 10}}>
                {health.level === 'ok' ? t('diagnosticHealth.allConfirmed') : <>{t(`diagnosticHealth.level.${health.level}`)} · {t(`diagnosticHealth.reason.${health.primary.reason}`)}</>}
                {!!health.primary.evidence?.length && `: ${health.primary.evidence.join('; ')}`}
            </div>
        </DashCard>
        <Card size="small" style={{marginBottom: 12}}>
            <Flex wrap gap="small" align="center">
                <Typography.Text type="secondary" style={{fontSize: 12, marginRight: 4}}>{t('diagnosticsPage.statusLabel')}</Typography.Text>
                {badge('containers', t('diagnosticHealth.containers'))}
                {badge('gnss', t('diagnosticsPage.gpsBadge', {value: gpsLabel}))}
                {badge('battery', battery?.level !== 'unknown' ? t('diagnosticsPage.batteryBadge', {value: NUMBER}) : t('diagnosticsPage.battery'), battery?.level !== 'unknown' ? batteryPercent.toFixed(0) : undefined)}
                {badge('emergency', t('diagnosticsPage.emergency'))}
                {badge('cpu', cpu?.level !== 'unknown' && cpuTemp !== undefined ? t('diagnosticsPage.cpuTemp', {value: cpuTemp.toFixed(1)}) : t('diagnosticsPage.cpuTempUnknown'))}
                {badge('ros', t('diagnosticHealth.ros'))}
                {badge('operation', t('diagnosticHealth.operation'))}
                {badge('configuration', t('diagnosticsPage.configurationCrossChecks'))}
                {badge('lidar', 'LiDAR')}
                {badge('localizer', t('diagnosticHealth.localizer'))}
                <Tag color="default"><NumericLabel label={cpuUsage != null ? t('diagnosticsPage.cpuUsageBadge', {value: NUMBER}) : t('diagnosticsPage.cpuUsageUnknown')} value={cpuUsage?.toFixed(0)}/></Tag>
            </Flex>
        </Card>
    </>;
}
