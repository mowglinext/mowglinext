import {useEffect, useState} from 'react';
import {Alert, Button, Card, Checkbox, Form, Input, Modal, Select, Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {type Deployment, type UpdatePlan, type UpdatePolicy, updaterRequest, useHostUpdater} from '../../hooks/useHostUpdater';

export function HostUpdaterPanel({advanced = false}: {advanced?: boolean}) {
    const {t} = useTranslation();
    const {data, error, refresh} = useHostUpdater();
    const [policy, setPolicy] = useState<UpdatePolicy>();
    const [selected, setSelected] = useState<string>();
    const [pinned, setPinned] = useState(false);
    const [plan, setPlan] = useState<UpdatePlan>();
    const [busy, setBusy] = useState(false);
    const [failure, setFailure] = useState<string>();
    const savedPolicy = data ? JSON.stringify(data.state.policy) : undefined;
    const installedPin = data?.state.installed_policy?.pinned ?? false;
    useEffect(() => {if (savedPolicy) setPolicy(JSON.parse(savedPolicy) as UpdatePolicy);}, [savedPolicy]);
    useEffect(() => {setPinned(installedPin);}, [installedPin]);
    const act = async (work: () => Promise<void>) => {
        setBusy(true); setFailure(undefined);
        try {await work(); await refresh();} catch (e) {setFailure(e instanceof Error ? e.message : String(e));}
        finally {setBusy(false);}
    };
    const pending = !!data?.state.job && !['succeeded', 'rolled_back', 'failed'].includes(data.state.job.phase);
    const releases = data?.state.releases ?? [];
    const target = (advanced && releases.find(r => r.id === selected)) || releases[0];
    const dirty = !!policy && JSON.stringify(policy) !== savedPolicy;
    const sameDeployment = !!target && target.id === data?.state.active?.id;
    const canRestore = data?.state.history.some(j => j.phase === 'succeeded' && j.plan.target.id === data.state.active?.id);
    const date = (value?: string) => value && !value.startsWith('0001') ? new Date(value).toLocaleString(undefined, {dateStyle: 'medium', timeStyle: 'short'}) : t('updates.unknown');
    const label = (deployment?: Deployment) => !deployment ? t('hostUpdater.customInstalled') : deployment.source.track === 'stable'
        ? deployment.release_tag || deployment.id : `${t(`hostUpdater.tracks.${deployment.source.track}`)} · ${deployment.revision.slice(0, 8)}`;
    const component = (service: string) => t(`updates.components.${service === 'mowgli' ? 'robot' : service}`, {defaultValue: service});
    return <Card title={t('hostUpdater.softwareUpdates')} size="small" data-testid="host-updater">
        <Space direction="vertical" size="middle" style={{width: '100%', overflowWrap: 'anywhere'}}>
            {error && <Alert type="warning" showIcon message={t(pending ? 'hostUpdater.reconnecting' : 'hostUpdater.unavailable')} description={pending ? undefined : t('hostUpdater.unavailableHelp')}/>}
            {failure && <Alert type="error" showIcon message={failure}/>}
            {data && policy && <>
                <div className="installed-version-heading">
                    <div><Typography.Text type="secondary">{t('hostUpdater.currentVersion')}</Typography.Text><div><Typography.Text strong>{label(data.state.active)}</Typography.Text></div></div>
                    {installedPin && <Tag>{t('hostUpdater.pinned')}</Tag>}
                </div>
                <Typography.Text type="secondary">{t('hostUpdater.checkingSource')}: {t(`hostUpdater.tracks.${data.state.policy.source.track}`)}
                    {(data.state.policy.source.track === 'custom' || data.state.policy.source.repository !== 'mowglinext/mowglinext') && <> · {data.state.policy.source.repository} / {data.state.policy.source.branch}</>}
                </Typography.Text>
                {data.state.check_error && <Alert type="warning" showIcon message={t('hostUpdater.checkFailed')} description={advanced ? data.state.check_error : undefined}/>}
                {advanced && <Form layout="vertical" className="updater-options">
                    <Form.Item label={t('hostUpdater.source')}>
                        <Select aria-label={t('hostUpdater.source')} value={policy.source.track} disabled={pending || busy}
                            options={['stable', 'dev', 'custom'].map(value => ({value, label: t(`hostUpdater.tracks.${value}`)}))}
                            onChange={track => setPolicy({...policy, source: {...policy.source, track, branch: track === 'stable' ? 'main' : track === 'dev' ? 'dev' : policy.source.branch}})}/>
                    </Form.Item>
                    <Form.Item label={t('hostUpdater.repository')}>
                        <Select aria-label={t('hostUpdater.repository')} value={policy.source.repository} disabled={pending || busy}
                            options={data.trusted_repositories.map(value => ({value, label: value}))} onChange={repository => setPolicy({...policy, source: {...policy.source, repository}})}/>
                    </Form.Item>
                    {policy.source.track === 'custom' && <Form.Item label={t('hostUpdater.branch')}>
                        <Input aria-label={t('hostUpdater.branch')} placeholder="feat/my-branch" value={policy.source.branch} disabled={pending || busy}
                            onChange={e => setPolicy({...policy, source: {...policy.source, branch: e.target.value}})}/>
                    </Form.Item>}
                    <Form.Item label={t('hostUpdater.interval')}>
                        <Select aria-label={t('hostUpdater.interval')} value={policy.interval_hours} disabled={pending || busy}
                            options={[0, 1, 4, 24].map(value => ({value, label: value ? t('hostUpdater.hours', {count: value}) : t('hostUpdater.manual')}))}
                            onChange={interval_hours => setPolicy({...policy, interval_hours})}/>
                    </Form.Item>
                    <div className="updater-options-help">
                        <Typography.Paragraph type="secondary">{t('hostUpdater.sourceHelp')}</Typography.Paragraph>
                        <details><summary>{t('hostUpdater.forkHelpTitle')}</summary><Typography.Paragraph>{t('hostUpdater.forkHelp')}</Typography.Paragraph></details>
                        <Button loading={busy} disabled={pending || !policy.source.branch.trim()} onClick={() => void act(async () => {
                            await updaterRequest('policy', policy); await updaterRequest('check', {}); setSelected(undefined);
                        })}>{t('hostUpdater.saveCheck')}</Button>
                        {dirty && <Button type="text" onClick={() => setPolicy(data.state.policy)}>{t('hostUpdater.resetSource')}</Button>}
                    </div>
                </Form>}
                {target ? <div>
                    <Typography.Text strong>{sameDeployment ? t('hostUpdater.latestInstalled') : t('hostUpdater.availableVersion')}</Typography.Text>
                    {!sameDeployment && <div>{label(target)} · {date(target.published_at)}</div>}
                </div> : <Typography.Text>{t(data.state.last_check && !data.state.last_check.startsWith('0001') ? 'hostUpdater.noBuild' : 'hostUpdater.notChecked')}</Typography.Text>}
                {advanced && releases.length > 0 && <>
                    <Form.Item label={t('hostUpdater.version')} style={{width: '100%', marginBottom: 0}}>
                        <Select aria-label={t('hostUpdater.version')} value={target?.id} disabled={pending || busy || dirty}
                            options={releases.map((r, i) => ({value: r.id, label: `${i === 0 ? t('hostUpdater.latest') + ' · ' : ''}${label(r)} · ${date(r.published_at)}`}))} onChange={setSelected}/>
                    </Form.Item>
                    <Checkbox checked={pinned} disabled={pending || busy || dirty} onChange={e => setPinned(e.target.checked)}>{t('hostUpdater.pin')}</Checkbox>
                </>}
                {dirty && <Typography.Text type="warning">{t('hostUpdater.unsavedSource')}</Typography.Text>}
                <Space wrap>
                    <Button disabled={pending || dirty} loading={busy} onClick={() => void act(async () => {await updaterRequest('check', {});})}>{t('hostUpdater.checkNow')}</Button>
                    <Button type="primary" disabled={pending || !target || dirty || (sameDeployment && (!advanced || pinned === installedPin))} loading={busy} onClick={() => void act(async () => {
                        if (target) setPlan(await updaterRequest<UpdatePlan>('plan', {deployment: target.id, pinned: advanced ? pinned : installedPin}));
                    })}>{t('hostUpdater.review')}</Button>
                </Space>
                <Typography.Text type="secondary">{t('hostUpdater.lastCheck', {time: date(data.state.last_check)})}</Typography.Text>
                {!advanced && <Typography.Text type="secondary">{t('hostUpdater.simpleHelp')}</Typography.Text>}
                {advanced && <Typography.Text type="secondary">{t('hostUpdater.nextCheck', {time: policy.interval_hours ? date(data.state.next_check) : t('hostUpdater.manual')})}</Typography.Text>}
                {data.agent.error && <Alert type="warning" showIcon message={data.agent.error}/>}
                {target?.updater[data.agent.platform] && target.updater[data.agent.platform].version !== data.agent.version && <Button disabled={pending || dirty} loading={busy}
                    onClick={() => Modal.confirm({title: t('hostUpdater.updateAgent'), content: t('hostUpdater.agentHelp'), onOk: () => act(async () => {await updaterRequest('agent-update', {deployment: target.id});})})}>
                    {t('hostUpdater.updateAgent')}
                </Button>}
                {data.state.job && <Alert type={data.state.job.error ? 'warning' : 'info'} showIcon message={t(`hostUpdater.phases.${data.state.job.phase}`, {defaultValue: data.state.job.phase})}
                    description={<>{advanced && <Typography.Text code>{data.state.job.id}</Typography.Text>}{data.state.job.error && <p>{data.state.job.error}</p>}</>}/>}
                {data.state.job?.phase === 'recovery_required' && <Button loading={busy} onClick={() => void act(async () => {await updaterRequest('recover', {});})}>{t('hostUpdater.recover')}</Button>}
                {canRestore && <Button disabled={pending || busy} onClick={() => Modal.confirm({title: t('hostUpdater.rollback'), content: t('hostUpdater.rollbackHelp'), onOk: () => act(async () => {await updaterRequest('rollback', {});})})}>{t('hostUpdater.rollback')}</Button>}
                {advanced && <details><summary>{t('hostUpdater.technicalDetails')}</summary>
                    <Typography.Paragraph>{t('hostUpdater.installed')}: {data.state.active?.id ?? t('hostUpdater.customInstalled')}</Typography.Paragraph>
                    <Typography.Paragraph>{t('hostUpdater.agent')}: <Typography.Text code>{data.agent.version}</Typography.Text> · {data.agent.platform}</Typography.Paragraph>
                    <Typography.Text strong>{t('hostUpdater.history')}</Typography.Text>
                    {data.state.history.map(j => <p key={j.id}>{date(j.started_at)} · {j.plan.target.id} · {t(`hostUpdater.phases.${j.phase}`, {defaultValue: j.phase})}</p>)}
                </details>}
            </>}
        </Space>
        <Modal title={t('hostUpdater.review')} open={!!plan} onCancel={() => setPlan(undefined)} okText={t('hostUpdater.install')} confirmLoading={busy}
            onOk={() => void act(async () => {if (plan) {await updaterRequest('apply', {plan: plan.id}); setPlan(undefined);}})}>
            {plan && <Space direction="vertical" size="middle" style={{width: '100%', overflowWrap: 'anywhere'}}>
                <Typography.Text strong>{label(plan.target)}</Typography.Text>
                <Typography.Text type="secondary">{plan.target.source.repository} · {plan.target.source.branch}</Typography.Text>
                <Typography.Text>{t('hostUpdater.componentsToUpdate')}: {Object.keys(plan.images).map(component).join(', ')}</Typography.Text>
                <Alert type="warning" showIcon message={t('hostUpdater.interruption')}/>
                <Typography.Text>{t('hostUpdater.backupHelp')}</Typography.Text>
                <details><summary>{t('hostUpdater.imageDetails')}</summary>
                    {Object.entries(plan.images).map(([service, image]) => <div key={service}><Typography.Text strong>{component(service)}</Typography.Text><div>{plan.previous[service]}</div><div>→ {image}</div></div>)}
                </details>
                <Typography.Text type="secondary">{t('hostUpdater.expires', {time: date(plan.expires_at)})}</Typography.Text>
            </Space>}
        </Modal>
    </Card>;
}
