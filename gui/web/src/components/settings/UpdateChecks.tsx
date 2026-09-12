import {Alert, Button, Card, Select, Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {useUpdateChecks} from '../../hooks/useUpdateChecks';

export function UpdateChecks() {
    const {t} = useTranslation();
    const {channel, setChannel, result, checking, error, check} = useUpdateChecks();
    const state = result?.state ?? 'not_checked';
    return <Card title={t('updateChecks.title')} size="small" data-testid="update-checks">
        <Space direction="vertical" size={14} style={{width:'100%'}}>
            <Typography.Text type="secondary">{t('updateChecks.readOnly')}</Typography.Text>
            <div className="installed-version-toolbar">
                <label htmlFor="update-comparison-channel">{t('updateChecks.channel')}</label>
                <Select id="update-comparison-channel" value={channel} onChange={setChannel} style={{minWidth:150}} options={[{value:'stable',label:t('updateChecks.stable')},{value:'dev',label:t('updateChecks.dev')}]}/>
                <Button loading={checking} onClick={() => void check()}>{t('updateChecks.check')}</Button>
            </div>
            {error ? <Alert type="warning" showIcon message={t('updateChecks.failed')} description={t('updateChecks.previous')}/> :
                <Alert type={state === 'unavailable' || state === 'incomplete' ? 'warning' : 'info'} showIcon message={t(`updateChecks.states.${state}`)}/>}
            {result?.version && <Typography.Text>{t('updateChecks.candidate', {version:result.version})} {result.notes_url && <a href={result.notes_url} target="_blank" rel="noreferrer">{t('updateChecks.notes')}</a>}</Typography.Text>}
            {result?.components?.map(component => <div className="installed-version-row" key={component.name}>
                <div className="installed-version-heading"><Typography.Text strong>{component.name}</Typography.Text><Tag>{t(`updateChecks.componentStates.${component.state}`)}</Tag></div>
                <Typography.Text type="secondary">{component.installed_revision?.slice(0,8) || t('updates.unknown')} → {component.available_revision?.slice(0,8) || t('updates.unknown')}</Typography.Text>
                {component.state === 'changed' && <div><Typography.Text>{t(`updateChecks.sourceRelations.${component.source_relation || 'unknown'}`)}</Typography.Text></div>}
                {component.custom_image && <div><Typography.Text type="secondary">{t('updateChecks.custom')}</Typography.Text></div>}
                {component.digest_reference && <div><Typography.Text type="secondary">{t('updateChecks.digestReference')}</Typography.Text></div>}
                {component.available_image && <details><summary>{t('updateChecks.targetImage')}</summary><Typography.Text style={{overflowWrap:'anywhere'}}>{component.available_image}</Typography.Text></details>}
            </div>)}
            {result?.checked_at && <Typography.Text type="secondary">{t('updateChecks.checked', {time:new Date(result.checked_at).toLocaleString()})}</Typography.Text>}
            {result?.last_successful_at && <Typography.Text type="secondary">{t('updateChecks.lastSuccess', {time:new Date(result.last_successful_at).toLocaleString()})}</Typography.Text>}
        </Space>
    </Card>;
}
