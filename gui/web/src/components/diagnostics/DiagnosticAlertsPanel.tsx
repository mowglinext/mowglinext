import {useState} from 'react';
import {Alert, Card, Collapse, Space, Tag} from 'antd';
import {WarningOutlined} from '@ant-design/icons';
import {useTranslation} from 'react-i18next';
import type {TimestampedDiagnosticStatus} from '../../hooks/useDiagnostics';
import {unavailableReport} from '../../utils/diagnosticHealth';
import {alertSeverity} from '../../utils/diagnosticsAlerts';
import {advanceReceptionWatches, missingReceptionSources, RECEPTION_ALERT_DELAY_MS, type CurrentDiagnosticAlert, type RequiredDiagnosticSource} from '../../utils/diagnosticAlertPresentation';

const EMPTY: readonly RequiredDiagnosticSource[] = [];
type Props = {alerts: readonly CurrentDiagnosticAlert[]; observations?: readonly TimestampedDiagnosticStatus[]; connected: boolean; now: number;
    sources?: readonly RequiredDiagnosticSource[]};

/** Current faults and delayed missing reception only. No visual history. */
export function DiagnosticAlertsPanel({alerts, connected, now, sources = EMPTY}: Props) {
    const {t} = useTranslation();
    const [openGroups, setOpenGroups] = useState<string[]>([]);
    const [previous, setPrevious] = useState(() => ({sources, now, connected, disconnectedSince: connected ? null : now, watches: advanceReceptionWatches([], sources, now)}));
    let watches = previous.watches;
    let disconnectedSince = previous.disconnectedSince;
    if (previous.sources !== sources || previous.now !== now || previous.connected !== connected) {
        watches = advanceReceptionWatches(watches, sources, now);
        if (!connected) disconnectedSince ??= now;
        // Socket-open and HTTP snapshot polling alone cannot acknowledge ROS recovery.
        const resumed = connected && watches.some(watch => watch.key !== 'system' && watch.publication !== null
            && watch.lastAdmissibleAt >= (disconnectedSince ?? Infinity)
            && watch.publication > (previous.watches.find(old => old.key === watch.key)?.publication ?? -Infinity));
        if (resumed) disconnectedSince = null;
        setPrevious({sources, now, connected, disconnectedSince, watches});
    }
    const groups = new Map<string, CurrentDiagnosticAlert[]>();
    for (const alert of alerts) {
        if (alert.notApplicable || ![1, 2].includes(alert.level) || unavailableReport(alert)) continue;
        const key = JSON.stringify([alert.hardware_id, alert.name.split('/')[0]]);
        const group = groups.get(key) ?? [];
        group.push(alert); groups.set(key, group);
    }
    const communicationMissing = disconnectedSince !== null && now - disconnectedSince >= RECEPTION_ALERT_DELAY_MS;
    const missing = disconnectedSince === null ? missingReceptionSources(watches, now) : [];
    if (!groups.size && !missing.length && !communicationMissing) return null;
    return <Card data-testid="diagnostic-alert-panel" title={<Space><WarningOutlined/>{t('diagnosticsPage.alerts')}</Space>} size="small">
        <div style={{minHeight: 48}}>
            <Space direction="vertical" style={{width: '100%'}}>
                {communicationMissing && <Alert data-testid="diagnostic-reception-alert" data-source="communication" type="warning" showIcon
                    message={t('diagnosticReception.communication')}/>}
                {missing.map(source => <Alert key={source.key} data-testid="diagnostic-reception-alert" data-source={source.key} type="warning" showIcon
                    message={t('diagnosticReception.missing', {source: source.label})}/>) }
            </Space>
            <Collapse size="small" ghost activeKey={openGroups}
                onChange={keys => setOpenGroups(Array.isArray(keys) ? keys.map(String) : [String(keys)])}
                items={[...groups].map(([key, items]) => {
                    const worst = items.reduce((level, item) => alertSeverity(item.level) > alertSeverity(level) ? item.level : level, 0);
                    return {key, label: <div data-testid="diagnostic-alert-group">
                        <Tag color={worst === 2 ? 'error' : 'warning'}>{t(worst === 2 ? 'diagnosticsPage.diagLevelError' : 'diagnosticsPage.diagLevelWarn')}</Tag>
                        {items[0].name.split('/')[0]} · {items[0].hardware_id}
                    </div>, children: <Space direction="vertical" style={{width: '100%'}}>
                        {items.map(item => <Alert key={item.name} data-testid="diagnostic-alert-episode"
                            type={item.level === 2 ? 'error' : 'warning'} showIcon message={item.name}
                            description={<div data-testid="diagnostic-alert-message" style={{overflowWrap: 'anywhere', whiteSpace: 'pre-wrap'}}>{item.message}</div>}/>) }
                    </Space>};
                })}/>
        </div>
    </Card>;
}
