import {Card, Flex, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {DashCard} from '../dashboard/Card';
import {useThemeMode} from '../../theme/ThemeContext';
import {useIsMobile} from '../../hooks/useIsMobile';
import type {deriveDiagnosticHealth} from '../../utils/diagnosticHealth';

type Health = ReturnType<typeof deriveDiagnosticHealth>;
/** Health remains immediate internally; unknown/OK are not visual alerts. */
export function DiagnosticHealthSummary({health, gpsLabel, batteryPercent, cpuTemp, cpuUsage}: {
    health: Health; gpsLabel: string; batteryPercent: number; cpuTemp?: number; cpuUsage?: number | null;
}) {
    const {t} = useTranslation();
    const {colors} = useThemeMode();
    const mobile = useIsMobile();
    const fault = health.level === 'critical' || health.level === 'warning';
    const verdict = health.level === 'critical'
        ? health.primary.reason === 'clearance' ? t('telemetry.emergencyClearanceUnverified')
            : health.primary.id === 'emergency' ? t('diagnosticsPage.verdictEmergency') : t('diagnosticsPage.verdictCritical')
        : health.level === 'warning' ? t('diagnosticsPage.verdictAttention') : '—';
    return <>
        <DashCard tone={health.level === 'critical' ? 'danger' : 'glow'} style={{marginBottom: 4}}>
            <div style={{fontSize: 11, color: colors.textMuted, letterSpacing: '0.08em', textTransform: 'uppercase', fontWeight: 600, marginBottom: 8}}>{t('diagnosticsPage.robotStatus')}</div>
            <div className="mn-num" data-testid="diagnostic-health-verdict" data-level={health.level}
                style={{fontSize: mobile ? 40 : 56, lineHeight: 1, color: health.level === 'critical' ? colors.danger : health.level === 'warning' ? colors.warning : colors.textMuted}}>{verdict}</div>
            <div data-testid="diagnostic-health-cause" role="region" tabIndex={0} aria-label={t('diagnosticAlertHistory.currentCause')}
                style={{fontSize: 13, color: colors.textSecondary, marginTop: 10, minHeight: '1.6em', overflowWrap: 'anywhere'}}>
                {fault && <>{t(`diagnosticHealth.reason.${health.primary.reason}`)}{!!health.primary.evidence?.length && `: ${health.primary.evidence.join('; ')}`}</>}
            </div>
        </DashCard>
        <Card size="small" style={{marginBottom: 12}}>
            <Flex wrap gap="small" align="center">
                <Typography.Text type="secondary" style={{fontSize: 12, marginRight: 4}}>{t('diagnosticsPage.statusLabel')}</Typography.Text>
                <Tag data-testid="diagnostic-health-containers" data-level={health.checks.find(c => c.id === 'containers')?.level}
                    color={health.checks.find(c => c.id === 'containers')?.level === 'critical' ? 'error' : 'default'}>{t('diagnosticHealth.containers')}</Tag>
                <Tag data-testid="diagnostic-health-gnss" data-level={health.checks.find(c => c.id === 'gnss')?.level}>{t('diagnosticsPage.gpsBadge', {value: gpsLabel})}</Tag>
                <Tag data-testid="diagnostic-health-battery" data-level={health.checks.find(c => c.id === 'battery')?.level}
                    color={health.checks.find(c => c.id === 'battery')?.level === 'critical' ? 'error' : health.checks.find(c => c.id === 'battery')?.level === 'warning' ? 'warning' : 'default'}>
                    {Number.isFinite(batteryPercent) ? t('diagnosticsPage.batteryBadge', {value: batteryPercent.toFixed(0)}) : t('diagnosticsPage.battery')}
                </Tag>
                <Tag data-testid="diagnostic-health-emergency" data-level={health.checks.find(c => c.id === 'emergency')?.level}
                    color={health.checks.find(c => c.id === 'emergency')?.level === 'critical' ? 'error' : 'default'}>
                    {health.checks.find(c => c.id === 'emergency')?.level === 'critical' ? t('diagnosticsPage.emergencyUpper') : t('diagnosticsPage.emergency')}
                </Tag>
                <Tag data-testid="diagnostic-health-cpu" data-level={health.checks.find(c => c.id === 'cpu')?.level}
                    color={health.checks.find(c => c.id === 'cpu')?.level === 'critical' ? 'error' : health.checks.find(c => c.id === 'cpu')?.level === 'warning' ? 'warning' : 'default'}>
                    {cpuTemp !== undefined ? t('diagnosticsPage.cpuTemp', {value: cpuTemp.toFixed(1)}) : t('diagnosticsPage.cpuTempUnknown')}
                </Tag>
                <Tag>{cpuUsage != null ? t('diagnosticsPage.cpuUsageBadge', {value: cpuUsage.toFixed(0)}) : t('diagnosticsPage.cpuUsageUnknown')}</Tag>
            </Flex>
        </Card>
    </>;
}
