import {useState} from 'react';
import {Alert, Button, Card, Collapse, Descriptions, Space, Tag, Typography} from 'antd';
import {WarningOutlined} from '@ant-design/icons';
import {useTranslation} from 'react-i18next';
import type {TimestampedDiagnosticStatus} from '../../hooks/useDiagnostics';
import {alertSeverity} from '../../utils/diagnosticsAlerts';
import {advanceDiagnosticAlertHistory, alertEpisodeVisible, type CurrentDiagnosticAlert, type DiagnosticAlertEpisode} from '../../utils/diagnosticAlertHistory';

const EMPTY: readonly TimestampedDiagnosticStatus[] = [];
type Props = {alerts: readonly CurrentDiagnosticAlert[]; observations?: readonly TimestampedDiagnosticStatus[]; connected: boolean; now: number};

/** Presentation lifetime is separate from the current diagnostic verdict. */
export function DiagnosticAlertsPanel({alerts, observations = EMPTY, connected, now}: Props) {
    const {t} = useTranslation();
    const [openGroups, setOpenGroups] = useState<string[]>([]);
    const [showRecent, setShowRecent] = useState(false);
    const [previous, setPrevious] = useState(() => ({alerts, observations, connected, now, openGroups,
        entries: advanceDiagnosticAlertHistory([], {alerts, observations, connected, now, openGroups})}));
    let entries = previous.entries;
    // Guarded derived state: one transition per input/clock change, no effect
    // timer reset on publication and no ref mutation during render.
    if (previous.alerts !== alerts || previous.observations !== observations || previous.connected !== connected
        || previous.now !== now || previous.openGroups !== openGroups) {
        entries = advanceDiagnosticAlertHistory(entries, {alerts, observations, connected, now, openGroups});
        setPrevious({alerts, observations, connected, now, openGroups, entries});
    }
    const groups = new Map<string, DiagnosticAlertEpisode[]>();
    for (const entry of entries) {
        if (!alertEpisodeVisible(entry, now, openGroups, showRecent)) continue;
        const group = groups.get(entry.groupKey) ?? [];
        group.push(entry); groups.set(entry.groupKey, group);
    }
    const recentCount = entries.filter(entry => !entry.present).length;
    const stateLabel = (state: DiagnosticAlertEpisode['state']) => t(`diagnosticAlertHistory.${state}`);
    const levelLabel = (level: number) => level === 2 ? t('diagnosticsPage.diagLevelError')
        : level === 1 ? t('diagnosticsPage.diagLevelWarn') : level === 3 ? t('diagnosticsPage.diagLevelStale') : t('diagnosticsPage.diagLevelOk');
    return <Card data-testid="diagnostic-alert-panel" title={<Space><WarningOutlined/>{t('diagnosticsPage.alerts')}</Space>} size="small"
        extra={<Button size="small" aria-pressed={showRecent} aria-label={t(showRecent ? 'diagnosticAlertHistory.hideRecent' : 'diagnosticAlertHistory.showRecent', {count: recentCount})}
            onClick={() => setShowRecent(value => !value)}>{t('diagnosticAlertHistory.recent', {count: recentCount})}</Button>}>
        <Button size="small" disabled={!openGroups.length} onClick={() => setOpenGroups([])} style={{marginBottom: 8}}>
            {t('diagnosticAlertHistory.collapseAll')}
        </Button>
        <div style={{minHeight: 48}}>
            {!groups.size && <Typography.Paragraph data-testid="diagnostic-alert-empty" type="secondary" style={{margin: 0, padding: '12px 0'}}>
                {t('diagnosticAlertHistory.empty')}
            </Typography.Paragraph>}
            <Collapse size="small" ghost activeKey={openGroups}
                onChange={keys => setOpenGroups(Array.isArray(keys) ? keys.map(String) : [String(keys)])}
                items={[...groups].map(([key, items]) => {
                    const state = items.some(item => item.state === 'active') ? 'active'
                        : items.some(item => item.state === 'unverified') ? 'unverified' : 'resolved';
                    const severityItems = items.filter(item => item.state === state);
                    const worst = severityItems.reduce((level, item) => alertSeverity(item.report.reportedLevel) > alertSeverity(level) ? item.report.reportedLevel : level, 0);
                    const component = items[0].report.name.split('/')[0] || t('diagnosticAlertHistory.unknownComponent');
                    const source = items[0].report.hardware_id || t('diagnosticAlertHistory.unknownSource');
                    return {key, label: <div data-testid="diagnostic-alert-group" data-state={state}
                        style={{display: 'flex', alignItems: 'center', gap: 8, minWidth: 0}}>
                        <Tag color={state !== 'active' ? 'default' : worst === 2 ? 'error' : 'warning'}>{levelLabel(worst)}</Tag>
                        <span title={`${component} · ${source}`} style={{flex: 1, minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap'}}>{component} · {source}</span>
                        <Tag>{stateLabel(state)}</Tag><span>{items.length}</span>
                    </div>, children: <Space direction="vertical" style={{width: '100%'}}>
                        {items.map(item => <Alert key={item.key} data-testid="diagnostic-alert-episode" data-state={item.state} data-episode={item.episode}
                            type={item.state !== 'active' ? 'info' : item.report.reportedLevel === 2 ? 'error' : 'warning'} showIcon
                            message={<div><div style={{overflowWrap: 'anywhere'}}>{item.report.name}</div>
                                <Space wrap><Tag>{stateLabel(item.state)}</Tag><span>{t('diagnosticAlertHistory.episode', {count: item.episode})}</span></Space>
                            </div>}
                            description={<div style={{overflowWrap: 'anywhere'}}>
                                <div data-testid="diagnostic-alert-message" style={{whiteSpace: 'pre-wrap'}}>{item.report.message}</div>
                                <div><Tag>{levelLabel(item.report.reportedLevel)}</Tag><Typography.Text type="secondary">{t('diagnosticHealth.lastReported', {level: item.report.reportedLevel})}</Typography.Text></div>
                                {item.report.notApplicable && <Tag>{t('diagnosticHealth.level.not_applicable')}</Tag>}
                                <Descriptions size="small" column={1} items={[
                                    {key: 'source', label: t('diagnosticAlertHistory.source'), children: source},
                                    ...item.report.values.map((pair, index) => ({key: `${index}:${pair.key}`, label: pair.key, children: pair.value})),
                                ]}/>
                            </div>}/>) }
                    </Space>};
                })}/>
        </div>
    </Card>;
}
