import type {ApiInstalledComponent} from '../../api/Api';
import {imageVersion} from '../../utils/versions';
import {useEffect, useState} from 'react';
import {Alert, Button, Card, Checkbox, Form, Input, Modal, Select, Space, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {type Deployment, type UpdatePlan, type UpdatePolicy, componentCompatibility, updaterRequest, useHostUpdater} from '../../hooks/useHostUpdater';

export function HostUpdaterPanel({advanced = false, inventory = []}: {advanced?: boolean; inventory?: ApiInstalledComponent[]}) {
    const {t} = useTranslation();
    const {data, error, refresh} = useHostUpdater();
    const [policy, setPolicy] = useState<UpdatePolicy>();
    const [selected, setSelected] = useState<string>();
    const [componentSelected, setComponentSelected] = useState<Record<string,string>>({});
    const [agentSelected, setAgentSelected] = useState<string>();
    const [pinned, setPinned] = useState(false);
    const [plan, setPlan] = useState<UpdatePlan>();
    const [agentPlan, setAgentPlan] = useState<Deployment>();
    const [rollbackOpen, setRollbackOpen] = useState(false);
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
    const active = data?.state.active;
    const versions = active && !releases.some(r => r.id === active.id) && JSON.stringify(active.source) === JSON.stringify(data?.state.policy.source)
        ? [...releases, active] : releases;
    const target = (advanced && versions.find(r => r.id === selected)) || releases[0] || (data?.runtime?.selection_pending ? versions.find(r => r.id === active?.id) : undefined);
    const overrides = advanced ? Object.fromEntries(Object.entries(componentSelected).filter(([,id]) => id !== target?.id).map(([name,id]) => [name, versions.find(r => r.id === id)]).filter(([,r]) => !!r)) as Record<string,Deployment> : {};
    const hasOverrides = Object.keys(overrides).length > 0;
    const agentTarget = releases.find(r => r.id === agentSelected) ?? target;
    const serviceChoices = target?.service_choices?.filter(c => Object.entries(c.when ?? {}).every(([key,value]) => (data?.runtime?.selection?.[key] ?? 'none') === value));
    const families: Record<string,string> = serviceChoices ? Object.fromEntries(serviceChoices.map(c => [c.service,c.image])) : Object.fromEntries(Object.entries(data?.runtime?.components ?? {}).map(([name,c]) => [name,c.family ?? ({mowgli:'mowgli-ros2',gui:'mowglinext-gui',gps:'gps'} as Record<string,string>)[name] ?? '']));
    const serviceNames = [...new Set([...Object.keys(data?.runtime?.components ?? {}), ...Object.keys(families)])].sort((a,b) => {const order = ['mowgli','gui','gps','lidar']; return (order.includes(a) ? order.indexOf(a) : 99) - (order.includes(b) ? order.indexOf(b) : 99) || a.localeCompare(b);});
    const managedNames = new Set(Object.values(data?.runtime?.components ?? {}).map(c => c.name));
    const localComponents = data?.runtime?.components ? inventory.filter(c => !managedNames.has(c.name)) : [];
    const runtime = error ? undefined : data?.runtime;
    const identity = runtime?.identity ?? 'unverified';
    const matched = identity === 'matched';
    const dirty = !!policy && JSON.stringify(policy) !== savedPolicy;
    const sameDeployment = !!target && target.id === active?.id && matched && !hasOverrides && !runtime?.selection_pending;
    const canRestore = data?.state.history.some(j => j.phase === 'succeeded' && j.plan.target.id === data.state.active?.id && (!data.state.active_job_id || j.id === data.state.active_job_id));
    const date = (value?: string) => value && !value.startsWith('0001') ? new Date(value).toLocaleString(undefined, {dateStyle: 'medium', timeStyle: 'short'}) : t('updates.unknown');
    const label = (deployment?: Deployment) => !deployment ? t('hostUpdater.customInstalled') : deployment.source.track === 'stable'
        ? deployment.release_tag || deployment.id : `${t(`hostUpdater.tracks.${deployment.source.track}`)} · ${deployment.revision.slice(0, 8)}`;
    const component = (service: string) => t(`updates.components.${service === 'mowgli' ? 'robot' : service}`, {defaultValue: service});
    return <Card title={t('hostUpdater.softwareUpdates')} size="small" data-testid="host-updater" className={advanced ? undefined : "updates-simple"}>
        <Space direction="vertical" size="middle" style={{width: '100%', overflowWrap: 'anywhere'}}>
            {error && <Alert type="warning" showIcon message={t(pending ? 'hostUpdater.reconnecting' : 'hostUpdater.unavailable')} description={pending ? undefined : t('hostUpdater.unavailableHelp')}/>}
            {failure && <Alert type="error" showIcon message={failure}/>}
            {data && policy && <>
                <div className="installed-version-heading">
                    <div><Typography.Text type="secondary">{t('hostUpdater.installed')}</Typography.Text><div><Typography.Text strong>{matched ? label(data.state.active) : t(`hostUpdater.identities.${identity}`, {defaultValue: t('hostUpdater.customInstalled')})}</Typography.Text></div></div>
                    {installedPin && <Tag>{t('hostUpdater.pinned')}</Tag>}
                </div>
                {!matched && active && <Typography.Text type="secondary">{t('hostUpdater.baseVersion')}: {label(active)}</Typography.Text>}
                <Typography.Text type={runtime?.health === 'degraded' ? 'warning' : 'secondary'}>{t('hostUpdater.containerHealth')}: {t(`hostUpdater.health.${runtime?.health ?? 'unknown'}`)}</Typography.Text>
                {['mixed', 'drifted'].includes(identity) && <Typography.Text type="secondary">{t('hostUpdater.mixedHelp')}</Typography.Text>}
                <Typography.Text type="secondary">{t('hostUpdater.checkingSource')}: {t(`hostUpdater.tracks.${data.state.policy.source.track}`)}
                    {(data.state.policy.source.track === 'custom' || data.state.policy.source.repository !== 'mowglinext/mowglinext') && <> · {data.state.policy.source.repository} / {data.state.policy.source.branch}</>}
                </Typography.Text>
                {runtime?.selection_pending && <Alert type="info" showIcon message={t('hostUpdater.selectionPending')}/>}
                {data.state.check_error && <Alert type="warning" showIcon message={t('hostUpdater.checkFailed')} description={advanced ? data.state.check_error : undefined}/>}
                {target ? <div>
                    <Typography.Text strong>{sameDeployment ? t('hostUpdater.latestInstalled') : t('hostUpdater.availableVersion')}</Typography.Text>
                    {!sameDeployment && <><div className="available-release">{label(target)}</div><Typography.Text type="secondary">{date(target.published_at)}</Typography.Text><div>{t('hostUpdater.bundleHelp')}</div></>}
                </div> : <Typography.Text>{t(data.state.last_check && !data.state.last_check.startsWith('0001') ? 'hostUpdater.noBuild' : 'hostUpdater.notChecked')}</Typography.Text>}
                {dirty && <Typography.Text type="warning">{t('hostUpdater.unsavedSource')}</Typography.Text>}
                <Space wrap className="update-actions">
                    <Button disabled={pending} loading={busy} onClick={() => void act(async () => {if (dirty && policy) {await updaterRequest('policy',policy);setSelected(undefined);setComponentSelected({});} await updaterRequest('check', {});})}>{t('hostUpdater.checkNow')}</Button>
                    <Button type="primary" disabled={pending || !target || dirty || (sameDeployment && (!advanced || pinned === installedPin))} loading={busy} onClick={() => void act(async () => {
                        if (target) setPlan(await updaterRequest<UpdatePlan>('plan', {deployment: target.id, pinned: advanced ? pinned : installedPin, ...(hasOverrides ? (data.capabilities?.includes('service-version-overrides') ? {component_deployments:Object.fromEntries(Object.entries(overrides).map(([name,r]) => [name,r.id]))} : {gui_deployment:overrides.gui?.id}) : {})}));
                    })}>{['mixed', 'drifted'].includes(identity) && !hasOverrides ? t('hostUpdater.returnMatched') : t('hostUpdater.review')}</Button>
                </Space>
                <Typography.Text type="secondary">{t('hostUpdater.lastCheck', {time: date(data.state.last_check)})}</Typography.Text>
                {!advanced && <Typography.Text type="secondary">{t('hostUpdater.simpleHelp')}</Typography.Text>}
                {advanced && <Typography.Text type="secondary">{t('hostUpdater.nextCheck', {time: policy.interval_hours ? date(data.state.next_check) : t('hostUpdater.manual')})}</Typography.Text>}
                {advanced && <Form layout="vertical" className="release-selection">
                    <Form.Item label={t('hostUpdater.version')}>
                        <Select aria-label={t('hostUpdater.version')} value={target?.id} placeholder={t('hostUpdater.noVersions')} disabled={pending || busy || dirty || versions.length === 0}
                            options={versions.map((r,i) => ({value:r.id,label:`${i === 0 && releases.length ? t('hostUpdater.latest')+' · ' : ''}${label(r)} · ${date(r.published_at)}`}))}
                            onChange={value => {setSelected(value);setComponentSelected({});}}/>
                    </Form.Item>
                    <Checkbox checked={pinned} disabled={pending || busy || dirty || !target} onChange={e => setPinned(e.target.checked)}>{t('hostUpdater.pin')}</Checkbox>
                </Form>}
                <section className="update-stack" aria-label={t('hostUpdater.stack')}>
                    <div className="update-stack-title"><Typography.Title level={5}>{t('hostUpdater.stack')}</Typography.Title>
                        {advanced && hasOverrides && <Button size="small" onClick={() => setComponentSelected({})}>{t('hostUpdater.resetComponents')}</Button>}
                    </div>
                    {serviceNames.length === 0 && <Typography.Text type="secondary">{t('hostUpdater.stackUnavailable')}</Typography.Text>}
                    {serviceNames.map(name => {
                        const running = runtime?.components?.[name]; const family = families[name];
                        const info = inventory.find(c => c.name === running?.name);
                        const version = info?.version || running?.version || imageVersion(info?.image ?? running?.reference ?? '') || running?.revision?.slice(0,8) || t('updates.unknown');
                        const revision = info?.revision || running?.revision;
                        const runningVersion = revision && !/^v?\d+\.\d+/.test(version) ? `${version} · ${revision.slice(0,8)}` : version;
                        const selectedRelease = overrides[name] ?? target;
                        const choices = releases.filter(r => r.id !== target?.id).map(r => ({release:r, reason:target ? componentCompatibility(target,r,family ?? '',data.agent.platform) : 'contract'}));
                        const supported = data.capabilities?.includes('service-version-overrides') || (name === 'gui' && data.capabilities?.includes('component-overrides'));
                        const noAlternatives = choices.every(c => !!c.reason);
                        return <div className="update-stack-row" key={name} data-testid={`stack-${name}`}>
                            <div><Typography.Text strong>{component(name)}</Typography.Text> {running && <Tag color={runtime?.health === 'unknown' ? undefined : running.healthy ? 'green' : 'orange'}>{runtime?.health === 'unknown' ? t('updates.unknown') : t(`hostUpdater.health.${running.healthy ? 'healthy' : 'degraded'}`)}</Tag>}
                                <div className="stack-version"><Typography.Text type="secondary">{t('hostUpdater.running')}: </Typography.Text>{running ? runningVersion : t('hostUpdater.notInstalled')}</div>
                                {data.state.overrides?.[name] && <Tag>{t('hostUpdater.customComponent')}</Tag>}
                                <details className="stack-image-details"><summary>{t('updates.details')}</summary><Typography.Paragraph code>{running?.reference ?? info?.image ?? family}</Typography.Paragraph><Typography.Paragraph code>{running?.image ?? info?.image_id}</Typography.Paragraph></details>
                            </div>
                            <div className="stack-target">
                                {advanced && family ? <><Select aria-label={`${component(name)} ${t('hostUpdater.versionControl')}`} value={overrides[name]?.id ?? ''}
                                    disabled={pending || busy || dirty || !target || !supported || noAlternatives}
                                    options={[{value:'',label:t('hostUpdater.followRelease')},...choices.map(({release,reason}) => ({value:release.id,disabled:!!reason,label:label(release)+(reason ? ` · ${t('hostUpdater.compatibility.'+reason)}` : '')}))]}
                                    onChange={id => setComponentSelected(current => {const next={...current};if(id)next[name]=id;else delete next[name];return next;})}/>
                                    {overrides[name] && <Tag color="gold">{t('hostUpdater.customComponent')}</Tag>}
                                    {!supported && <Typography.Text type="secondary">{t('hostUpdater.serviceSelectionUnsupported')}</Typography.Text>}
                                    {target && supported && noAlternatives && <Typography.Text type="secondary">{t('hostUpdater.noCompatibleComponent')}</Typography.Text>}
                                </> : !family && target && <Typography.Text type="secondary">{t('hostUpdater.removedByRelease')}</Typography.Text>}
                                {selectedRelease && family && advanced && <Typography.Text type="secondary">{t('hostUpdater.selected')}: {label(selectedRelease)}</Typography.Text>}
                            </div>
                        </div>;
                    })}
                    {localComponents.map(c => <div className="update-stack-row local" key={c.name}><div><Typography.Text strong>{t(`updates.components.${c.component}`, {defaultValue:c.name})}</Typography.Text><div>{c.version || imageVersion(c.image ?? '') || t('updates.unknown')}</div></div><Tag>{t('hostUpdater.localService')}</Tag></div>)}
                    <div className="update-stack-row"><div><Typography.Text strong>{t('hostUpdater.agent')}</Typography.Text><div>{data.agent.version}</div><Typography.Text type="secondary">{t('hostUpdater.separateUpdate')}</Typography.Text></div>
                        <div className="stack-target">{advanced && <Select aria-label={t('hostUpdater.agentVersion')} value={agentTarget?.id} disabled={pending || busy || dirty || releases.length === 0} placeholder={t('hostUpdater.noVersions')}
                            options={releases.filter(r => r.updater[data.agent.platform]).map(r => ({value:r.id,label:label(r)}))} onChange={setAgentSelected}/>}
                            {agentTarget?.updater[data.agent.platform] && agentTarget.updater[data.agent.platform].version !== data.agent.version && <Button disabled={pending || busy || dirty} onClick={() => setAgentPlan(agentTarget)}>{t('hostUpdater.updateAgent')}</Button>}
                        </div>
                    </div>
                </section>
                {data.agent.error && <Alert type="warning" showIcon message={data.agent.error}/>}
                {advanced && <details className="update-preferences"><summary>{t('hostUpdater.preferences')}</summary><Form layout="vertical" className="updater-options">
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
                            await updaterRequest('policy', policy); if (dirty) await updaterRequest('check', {}); setSelected(undefined); setComponentSelected({});
                        })}>{t('hostUpdater.saveSettings')}</Button>
                        {dirty && <Button type="text" onClick={() => setPolicy(data.state.policy)}>{t('hostUpdater.resetSource')}</Button>}
                    </div>
                </Form></details>}
                {data.state.job && <Alert type={data.state.job.error ? 'warning' : 'info'} showIcon message={t(`hostUpdater.phases.${data.state.job.phase}`, {defaultValue: data.state.job.phase})}
                    description={<>{advanced && <Typography.Text code>{data.state.job.id}</Typography.Text>}{data.state.job.error && <p>{data.state.job.error}</p>}</>}/>}
                {data.state.job?.phase === 'recovery_required' && <Button loading={busy} onClick={() => void act(async () => {await updaterRequest('recover', {});})}>{t('hostUpdater.recover')}</Button>}
                {canRestore && <Button disabled={pending || busy} onClick={() => setRollbackOpen(true)}>{t('hostUpdater.rollback')}</Button>}
                {advanced && <details><summary>{t('hostUpdater.history')}</summary>
                    <Typography.Paragraph>{t('hostUpdater.observedAt', {time: date(runtime?.checked_at)})}</Typography.Paragraph>
                    {runtime?.error && <Typography.Paragraph type="warning">{runtime.error}</Typography.Paragraph>}
                    {data.state.history.map(j => <div key={j.id} className="update-history-entry"><Typography.Text>{date(j.started_at)} · {label(j.plan.target)} · {t(`hostUpdater.phases.${j.phase}`, {defaultValue: j.phase})}</Typography.Text>
                        {Object.entries(j.plan.overrides ?? {}).map(([name,r]) => <div key={name}>{component(name)}: {label(r)}</div>)}
                    </div>)}
                </details>}
            </>}
        </Space>
        <Modal title={t('hostUpdater.updateAgent')} open={!!agentPlan} onCancel={() => setAgentPlan(undefined)} confirmLoading={busy}
            onOk={() => void act(async () => {if (agentPlan) {await updaterRequest('agent-update', {deployment: agentPlan.id}); setAgentPlan(undefined);}})}>
            <Typography.Paragraph>{t('hostUpdater.agentHelp')}</Typography.Paragraph>
            <Typography.Text>{agentPlan && label(agentPlan)} · {agentPlan?.updater[data?.agent.platform ?? '']?.version}</Typography.Text>
        </Modal>
        <Modal title={t('hostUpdater.rollback')} open={rollbackOpen} onCancel={() => setRollbackOpen(false)} confirmLoading={busy}
            onOk={() => void act(async () => {await updaterRequest('rollback', {}); setRollbackOpen(false);})}>
            {t('hostUpdater.rollbackHelp')}
        </Modal>
        <Modal title={t('hostUpdater.review')} open={!!plan} onCancel={() => setPlan(undefined)} okText={t('hostUpdater.install')} confirmLoading={busy}
            style={{top: 24, paddingBottom: 24}} styles={{body: {maxHeight: 'calc(100dvh - 180px)', overflowY: 'auto'}}}
            onOk={() => void act(async () => {if (plan) {await updaterRequest('apply', {plan: plan.id}); setPlan(undefined);}})}>
            {plan && <Space direction="vertical" size="middle" style={{width: '100%', overflowWrap: 'anywhere'}}>
                <Typography.Text strong>{label(plan.target)}</Typography.Text>
                {Object.entries(plan.overrides ?? {}).map(([name,r]) => <Alert key={name} type="info" showIcon message={t('hostUpdater.componentReview',{component:component(name),version:label(r)})}/>)}
                <Typography.Text type="secondary">{plan.target.source.repository} · {plan.target.source.branch}</Typography.Text>
                {plan.stack ? <>
                    <Typography.Text type="secondary">{t('hostUpdater.selectionHelp')}</Typography.Text>
                    <div>{Object.entries(plan.stack.selection.options).map(([key, value]) => <Tag key={key}>{t(`hostUpdater.hardware.${key}`, {defaultValue: key})}: {value === 'none' ? t('hostUpdater.disabled') : value === 'universal' ? t('hostUpdater.enabled') : value}</Tag>)}</div>
                    <div data-testid="stack-changes">{plan.stack.changes.map(change => <div key={change.service} style={{display: 'flex', justifyContent: 'space-between', gap: 12, marginBottom: 6}}>
                        <Typography.Text>{component(change.service)}</Typography.Text>
                        <Tag color={change.action === 'add' ? 'green' : change.action === 'remove' ? 'orange' : undefined}>{t(`hostUpdater.stackActions.${change.action}`)}</Tag>
                    </div>)}</div>
                    {plan.stack.changes.some(change => change.action === 'remove') && <Typography.Text type="secondary">{t('hostUpdater.retainedData')}</Typography.Text>}
                </> : <Typography.Text>{t('hostUpdater.componentsToUpdate')}: {Object.keys(plan.images).map(component).join(', ')}</Typography.Text>}
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
