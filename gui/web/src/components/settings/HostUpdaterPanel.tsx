import {useEffect, useState} from 'react';
import {Alert, Button, Card, Checkbox, Input, Modal, Select, Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {type UpdatePlan, type UpdatePolicy, updaterRequest, useHostUpdater} from '../../hooks/useHostUpdater';

export function HostUpdaterPanel() {
    const {t} = useTranslation();
    const {data, error, refresh} = useHostUpdater();
    const [policy, setPolicy] = useState<UpdatePolicy>();
    const [selected, setSelected] = useState<string>();
    const [pinned, setPinned] = useState(false);
    const [plan, setPlan] = useState<UpdatePlan>();
    const [busy, setBusy] = useState(false);
    const [failure, setFailure] = useState<string>();
    const savedPolicy = data ? JSON.stringify(data.state.policy) : undefined;
    useEffect(() => {if (savedPolicy) setPolicy(JSON.parse(savedPolicy) as UpdatePolicy);}, [savedPolicy]);
    const act = async (work: () => Promise<void>) => {
        setBusy(true); setFailure(undefined);
        try {await work(); await refresh();} catch (e) {setFailure(e instanceof Error ? e.message : String(e));}
        finally {setBusy(false);}
    };
    const pending = data?.state.job && !['succeeded', 'rolled_back', 'failed'].includes(data.state.job.phase);
    const releases = data?.state.releases ?? [];
    const target = releases.find(r => r.id === selected) ?? releases[0];
    const date = (value?: string) => value && !value.startsWith('0001') ? new Date(value).toLocaleString() : t('updates.unknown');
    return <Card title={t('hostUpdater.title')} size="small" data-testid="host-updater">
        <Space direction="vertical" style={{width: '100%', overflowWrap: 'anywhere'}}>
            {error && <Alert type="warning" showIcon message={t(pending ? 'hostUpdater.reconnecting' : 'hostUpdater.unavailable')} description={error}/>}
            {failure && <Alert type="error" showIcon message={failure}/>}
            {data && policy && <>
                <Typography.Text>{t('hostUpdater.agent')}: <Typography.Text code>{data.agent.version}</Typography.Text> · {data.agent.platform}</Typography.Text>
                {data.agent.error && <Alert type="warning" showIcon message={data.agent.error}/>}
                <Typography.Text>{t('hostUpdater.installed')}: {data.state.active?.id ?? t('hostUpdater.customInstalled')} {data.state.installed_policy?.pinned && <Tag>{t('hostUpdater.pinned')}</Tag>}</Typography.Text>
                <Typography.Text type="secondary">{t('hostUpdater.lastCheck', {time: date(data.state.last_check)})} · {t('hostUpdater.nextCheck', {time: policy.interval_hours ? date(data.state.next_check) : t('hostUpdater.manual')})}</Typography.Text>
                {data.state.check_error && <Alert type="warning" showIcon message={t('hostUpdater.checkFailed')} description={data.state.check_error}/>}
                <Space wrap>
                    <Select aria-label={t('hostUpdater.source')} value={policy.source.track} style={{minWidth: 165}} disabled={!!pending}
                        options={['stable', 'dev', 'custom'].map(value => ({value, label: t(`hostUpdater.tracks.${value}`)}))}
                        onChange={track => setPolicy({...policy, source: {...policy.source, track, branch: track === 'stable' ? 'main' : track === 'dev' ? 'dev' : policy.source.branch}})}/>
                    <Select aria-label={t('hostUpdater.repository')} style={{minWidth: 220, maxWidth: '100%'}} value={policy.source.repository} disabled={!!pending}
                        options={data.trusted_repositories.map(value => ({value, label: value}))} onChange={repository => setPolicy({...policy, source: {...policy.source, repository}})}/>
                    {policy.source.track === 'custom' && <Input aria-label={t('hostUpdater.branch')} placeholder={t('hostUpdater.branch')} value={policy.source.branch} disabled={!!pending}
                        onChange={e => setPolicy({...policy, source: {...policy.source, branch: e.target.value}})}/>}
                    <Select aria-label={t('hostUpdater.interval')} value={policy.interval_hours} style={{minWidth: 150}} disabled={!!pending}
                        options={[0, 1, 4, 24].map(value => ({value, label: value ? t('hostUpdater.hours', {count: value}) : t('hostUpdater.manual')}))}
                        onChange={interval_hours => setPolicy({...policy, interval_hours})}/>
                    <Button loading={busy} disabled={!!pending} onClick={() => void act(async () => {
                        await updaterRequest('policy', policy); await updaterRequest('check', {}); setSelected(undefined);
                    })}>{t('hostUpdater.saveCheck')}</Button>
                </Space>
                <Typography.Text type="secondary">{t('hostUpdater.sourceHelp')}</Typography.Text>
                {releases.length === 0 ? <Typography.Text>{t('hostUpdater.noBuild')}</Typography.Text> : <>
                    <Select aria-label={t('hostUpdater.version')} value={target?.id} style={{width: '100%'}} disabled={!!pending}
                        options={releases.map((r, i) => ({value: r.id, label: `${i === 0 ? t('hostUpdater.latest') + ' · ' : ''}${r.id} · ${r.revision.slice(0, 8)} · ${date(r.published_at)}`}))} onChange={setSelected}/>
                    <Checkbox checked={pinned} disabled={!!pending} onChange={e => setPinned(e.target.checked)}>{t('hostUpdater.pin')}</Checkbox>
                    <Space wrap>
                        <Button type="primary" disabled={!!pending || !target} loading={busy} onClick={() => void act(async () => {
                            setPlan(await updaterRequest<UpdatePlan>('plan', {deployment: target.id, pinned}));
                        })}>{t('hostUpdater.review')}</Button>
                        {target?.updater[data.agent.platform] && <Button disabled={!!pending || target.updater[data.agent.platform].version === data.agent.version} loading={busy}
                            onClick={() => Modal.confirm({title: t('hostUpdater.updateAgent'), content: t('hostUpdater.agentHelp'), onOk: () => act(async () => {await updaterRequest('agent-update', {deployment: target.id});})})}>
                            {t('hostUpdater.updateAgent')}
                        </Button>}
                    </Space>
                </>}
                {data.state.job && <Alert type={data.state.job.error ? 'warning' : 'info'} showIcon message={t(`hostUpdater.phases.${data.state.job.phase}`, {defaultValue: data.state.job.phase})}
                    description={<><Typography.Text code>{data.state.job.id}</Typography.Text>{data.state.job.error && <p>{data.state.job.error}</p>}</>}/>}
                {data.state.job?.phase === 'recovery_required' && <Button loading={busy} onClick={() => void act(async () => {await updaterRequest('recover', {});})}>{t('hostUpdater.recover')}</Button>}
                <details><summary>{t('hostUpdater.history')}</summary>{data.state.history.map(j => <p key={j.id}>{date(j.started_at)} · {j.plan.target.id} · {t(`hostUpdater.phases.${j.phase}`, {defaultValue: j.phase})}</p>)}</details>
                <Button disabled={!!pending || busy || !data.state.history.some(j => j.phase === 'succeeded')} onClick={() => Modal.confirm({title: t('hostUpdater.rollback'), content: t('hostUpdater.rollbackHelp'), onOk: () => act(async () => {await updaterRequest('rollback', {});})})}>{t('hostUpdater.rollback')}</Button>
            </>}
        </Space>
        <Modal title={t('hostUpdater.review')} open={!!plan} onCancel={() => setPlan(undefined)} okText={t('hostUpdater.install')} confirmLoading={busy}
            onOk={() => void act(async () => {if (plan) {await updaterRequest('apply', {plan: plan.id}); setPlan(undefined);}})}>
            {plan && <Space direction="vertical" style={{width: '100%', overflowWrap: 'anywhere'}}>
                <Alert type="warning" showIcon message={t('hostUpdater.interruption')}/>
                <Typography.Text strong>{plan.target.source.repository} · {plan.target.source.branch} · {plan.target.id}</Typography.Text>
                {Object.entries(plan.images).map(([service, image]) => <div key={service} style={{overflowWrap: 'anywhere'}}><Typography.Text strong>{service}</Typography.Text><div>{plan.previous[service]}</div><div>→ {image}</div></div>)}
                <Typography.Text>{t('hostUpdater.expires', {time: date(plan.expires_at)})}</Typography.Text>
            </Space>}
        </Modal>
    </Card>;
}
