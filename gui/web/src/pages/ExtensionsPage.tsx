import {useEffect, useRef, useState} from 'react';
import {Alert, Button, Collapse, Empty, Input, Modal, Select, Space, Spin, Tag, Typography} from 'antd';
import {PackagePlus, RefreshCw} from 'lucide-react';
import {useTranslation} from 'react-i18next';
import {errorMessage} from '../utils/errors';
import {useExtensions} from './extensions/useExtensions';
import {
    extensionRequest, readBundle, type ExtensionAction, type ExtensionManifest,
    type ExtensionOperation, type ExtensionReview, type ExtensionStatus, type ImportAction, type ImportRequest,
} from './extensions/api';
import './extensions/extensions.css';

const {Text, Title} = Typography;
const MAX_BUNDLE_BYTES = 2 * 1024 * 1024;

function ManifestPreview({manifest, digest, reviewIdentity = false}: {manifest: ExtensionManifest; digest: string; reviewIdentity?: boolean}) {
    const {t} = useTranslation();
    return <div className="extension-preview">
        <Title level={4} style={{margin: 0}}>{manifest.id} <Text type="secondary">{manifest.version}</Text></Title>
        <dl>
            <dt>{t('extensions.source')}</dt><dd>{manifest.source.repository}</dd>
            <dt>{t('extensions.revision')}</dt><dd>{manifest.source.revision}</dd>
            <dt>{t('extensions.runtime')}</dt><dd>{manifest.runtime.kind} / {manifest.profile}</dd>
            {manifest.runtime.units?.length ? <><dt>{t('extensions.units')}</dt><dd>{manifest.runtime.units.join(', ')}</dd></> : null}
            <dt>{t('extensions.permissions')}</dt><dd>{manifest.grants.length ? manifest.grants.map(grant => <Tag key={grant}>{grant}</Tag>) : t('extensions.noPermissions')}</dd>
            <dt>{t(reviewIdentity ? 'extensions.reviewIdentity' : 'extensions.digest')}</dt><dd>{digest}</dd>
        </dl>
        <Alert type="info" showIcon message={t('extensions.healthLimits')} description={t(manifest.health === 'process-only' ? 'extensions.processOnlyHelp' : 'extensions.applicationHelp')}/>
    </div>;
}

function Health({name, value}: {name: string; value: string}) {
    const {t} = useTranslation();
    // A running process says nothing about application readiness or data integrity.
    const good = ['running', 'healthy', 'ready', 'verified'].includes(value);
    const bad = ['failed', 'error', 'unhealthy'].includes(value);
    return <span><Text type="secondary">{name}</Text><Tag color={good ? 'green' : bad ? 'red' : undefined}>{t(`extensions.health.${value}`, {defaultValue: value || t('extensions.health.unknown')})}</Tag></span>;
}

export default function ExtensionsPage() {
    const {t} = useTranslation();
    const {state, error, loading, refresh, acceptOperation, running} = useExtensions();
    const [importOpen, setImportOpen] = useState(false);
    const [bundle, setBundle] = useState<string>();
    const [fileName, setFileName] = useState('');
    const [action, setAction] = useState<ImportAction>('install');
    const [binding, setBinding] = useState({project: '', config_dir: '', data_dir: ''});
    const [review, setReview] = useState<{result: ExtensionReview; request: ImportRequest}>();
    const [pending, setPending] = useState<{action: ExtensionAction; entry?: ExtensionStatus}>();
    const [busy, setBusy] = useState(false);
    const [importError, setImportError] = useState('');
    const [actionError, setActionError] = useState('');
    const version = useRef(0);
    const alive = useRef(true);
    const busyRef = useRef(false);
    const reviewRequest = useRef<AbortController | null>(null);
    useEffect(() => {
        alive.current = true;
        return () => { alive.current = false; reviewRequest.current?.abort(); };
    }, []);
    const invalidate = () => { version.current++; reviewRequest.current?.abort(); setReview(undefined); setImportError(''); };
    const blocked = busy || running || !state?.configured || Boolean(error);
    const recovery = state?.entries.some(entry => entry.recovery_required) || state?.operation?.phase === 'failed';

    async function loadFile(file?: File) {
        invalidate();
        setBundle(undefined);
        setFileName(file?.name ?? '');
        if (!file) return;
        if (file.size > MAX_BUNDLE_BYTES || !file.name.toLowerCase().endsWith('.json')) {
            setImportError(t('extensions.fileLimit')); return;
        }
        const current = version.current;
        try {
            const parsed = await readBundle(file);
            if (alive.current && version.current === current) setBundle(parsed);
        } catch {
            if (alive.current && version.current === current) setImportError(t('extensions.invalidFile'));
        }
    }
    async function reviewImport() {
        if (!bundle || busyRef.current || blocked) return;
        if (action === 'adopt' && !binding.project.trim()) { setImportError(t('extensions.adoptProject')); return; }
        const request: ImportRequest = {action, bundle};
        const explicit = Object.fromEntries(Object.entries(binding).filter(([, value]) => value.trim()).map(([key, value]) => [key, value.trim()]));
        if (Object.keys(explicit).length) request.binding = explicit;
        const current = version.current;
        const controller = new AbortController();
        reviewRequest.current = controller;
        busyRef.current = true; setBusy(true); setImportError('');
        try {
            const result = await extensionRequest<ExtensionReview>('review', request, controller.signal);
            if (alive.current && version.current === current) setReview({result, request});
        } catch (e) {
            if (alive.current && !controller.signal.aborted && version.current === current) setImportError(errorMessage(e));
        } finally {
            busyRef.current = false;
            if (alive.current) setBusy(false);
        }
    }
    async function applyImport() {
        if (!review || review.result.blockers.length || busyRef.current || blocked) return;
        busyRef.current = true; setBusy(true); setImportError('');
        try {
            const operation = await extensionRequest<ExtensionOperation>('apply', {...review.request, id: review.result.manifest.id, digest: review.result.digest, acknowledged: true});
            if (!alive.current) return;
            acceptOperation(operation); setImportOpen(false); setReview(undefined); setBundle(undefined); setFileName('');
        } catch (e) { if (alive.current) { setImportError(errorMessage(e)); await refresh(); } }
        finally { busyRef.current = false; if (alive.current) setBusy(false); }
    }
    async function applyAction() {
        if (!pending || busyRef.current || blocked) return;
        busyRef.current = true; setBusy(true); setActionError('');
        try {
            const operation = await extensionRequest<ExtensionOperation>('action', {action: pending.action, id: pending.entry?.id ?? '', digest: pending.entry?.digest ?? '', ...(pending.action === 'rollback' ? {previous_digest: pending.entry?.previous_digest} : {}), acknowledged: true});
            if (!alive.current) return;
            acceptOperation(operation); setPending(undefined);
        } catch (e) { if (alive.current) { setActionError(errorMessage(e)); await refresh(); } }
        finally { busyRef.current = false; if (alive.current) setBusy(false); }
    }
    function openImport(importAction: ImportAction = 'install') {
        invalidate(); setAction(importAction); setImportOpen(true);
    }
    function ask(next: ExtensionAction, entry?: ExtensionStatus) { setActionError(''); setPending({action: next, entry}); }
    return <div className="extensions-page">
        <div className="extensions-intro">
            <div><Title level={3} style={{marginTop: 0}}>{t('extensions.title')}</Title><p>{t('extensions.intro')}</p></div>
            <div className="extensions-toolbar">
                <Button icon={<RefreshCw size={16}/>} loading={loading} disabled={busy || running} onClick={() => { void refresh(); }}>{t('extensions.refresh')}</Button>
                <Button type="primary" icon={<PackagePlus size={17}/>} disabled={blocked} onClick={() => openImport()}>{t('extensions.customImport')}</Button>
            </div>
        </div>
        {(error || state?.error) && <Alert type="error" showIcon message={t('extensions.unavailable')} description={<>{error || state?.error}<p>{t('extensions.setupHelp')}</p></>}/>}
        {state && !state.configured && !error && <Alert type="info" showIcon message={t('extensions.setupTitle')} description={t('extensions.setupHelp')}/>}
        {state?.operation && <Alert showIcon type={running ? 'info' : state.operation.phase === 'failed' ? 'error' : 'success'}
            message={t(`extensions.operation.${state.operation.phase}`, {action: t(`extensions.actions.${state.operation.action}`, {defaultValue: state.operation.action}), id: state.operation.id})}
            description={state.operation.error}
            action={recovery && <Button disabled={blocked} onClick={() => ask('recover')}>{t('extensions.actions.recover')}</Button>}/>}
        {recovery && !state?.operation && <Alert type="warning" showIcon message={t('extensions.recoveryRequired')} action={<Button disabled={blocked} onClick={() => ask('recover')}>{t('extensions.actions.recover')}</Button>}/>}
        {loading && !state ? <Spin aria-label={t('extensions.loading')}/> : state?.entries.length ? <div className="extensions-list">{state.entries.map(entry => <section className="extension-row" key={entry.id} aria-label={entry.id}>
            <div className="extension-heading">
                <div><div className="extension-name">{entry.id} <Text type="secondary">{entry.version}</Text></div><div className="extension-source">{entry.manifest?.source.repository ?? t('extensions.sourceUnknown')}</div></div>
                <Space size={4} wrap><Tag>{t(entry.managed ? 'extensions.managed' : 'extensions.observeOnly')}</Tag><Tag color={entry.compatible ? 'green' : 'orange'}>{t(entry.compatible ? 'extensions.compatible' : 'extensions.incompatible')}</Tag>{entry.managed && <Tag>{t(entry.enabled ? 'extensions.enabled' : 'extensions.disabled')}</Tag>}</Space>
            </div>
            <div className="extension-health">
                <Health name={t('extensions.process')} value={entry.observation.process}/><Health name={t('extensions.application')} value={entry.observation.application}/><Health name={t('extensions.data')} value={entry.observation.data}/>
            </div>
            {!entry.managed && <Text type="secondary">{t('extensions.externalHelp')}</Text>}
            {(entry.error || entry.recovery_required || entry.blockers?.length > 0) && <Alert type="warning" showIcon message={entry.error || (entry.recovery_required ? t('extensions.recoveryRequired') : t('extensions.blocked'))} description={entry.blockers?.join('; ')}/>}
            <div className="extension-actions">{entry.managed ? <>
                <Button disabled={blocked || entry.recovery_required || (!entry.enabled && !entry.compatible)} onClick={() => ask(entry.enabled ? 'disable' : 'enable', entry)}>{t(entry.enabled ? 'extensions.actions.disable' : 'extensions.actions.enable')}</Button>
                <Button disabled={blocked || entry.recovery_required} onClick={() => openImport('upgrade')}>{t('extensions.update')}</Button>
                {entry.previous_version && <Button disabled={blocked || entry.recovery_required || !entry.previous_digest} onClick={() => ask('rollback', entry)}>{t('extensions.actions.rollback')} ({entry.previous_version})</Button>}
                <Button danger disabled={blocked || entry.enabled || entry.recovery_required} onClick={() => ask('remove', entry)}>{t('extensions.actions.remove')}</Button>
            </> : <Button disabled={blocked} onClick={() => ask('forget', entry)}>{t('extensions.actions.forget')}</Button>}</div>
            {entry.manifest && <Collapse ghost size="small" items={[{key: 'manifest', label: t('extensions.details'), children: <ManifestPreview manifest={entry.manifest} digest={entry.digest}/>}]} />}
        </section>)}</div> : state?.configured && !error ? <Empty description={t('extensions.empty')}/> : null}
        <p className="extension-helper">{t('extensions.repositoriesHelp')}</p>

        <Modal open={importOpen} title={t('extensions.customImport')} width={720} destroyOnHidden maskClosable={!busy} closable={!busy}
            onCancel={() => { if (!busy) { invalidate(); setImportOpen(false); } }}
            footer={<Space wrap><Button disabled={busy} onClick={() => { invalidate(); setImportOpen(false); }}>{t('extensions.cancel')}</Button>
                {review ? <Button type="primary" disabled={blocked || review.result.blockers.length > 0} loading={busy} onClick={() => { void applyImport(); }}>{t('extensions.applyImport')}</Button>
                    : <Button type="primary" disabled={blocked || !bundle || (action === 'adopt' && !binding.project.trim())} loading={busy} onClick={() => { void reviewImport(); }}>{t('extensions.review')}</Button>}</Space>}>
            <div className="extension-import-form">
                <p className="extension-helper">{t('extensions.importHelp')}</p>
                <label>{t('extensions.bundleFile')}<input className="extension-file" type="file" accept=".json,application/json" disabled={busy} onChange={event => { void loadFile(event.target.files?.[0]); }}/></label>
                {fileName && <Text type="secondary">{fileName}</Text>}
                <label>{t('extensions.importAction')}<Select aria-label={t('extensions.importAction')} value={action} disabled={busy} onChange={(value: ImportAction) => { invalidate(); setAction(value); }} options={(['install', 'adopt', 'upgrade'] as const).map(value => ({value, label: t(`extensions.importActions.${value}`)}))}/></label>
                {action === 'adopt' && !binding.project.trim() && <Alert type="info" showIcon message={t('extensions.adoptProject')}/>}
                <Collapse items={[{key: 'binding', label: t('extensions.advancedBinding'), children: <div className="extension-binding"><p className="extension-helper">{t('extensions.bindingHelp')}</p>{(['project', 'config_dir', 'data_dir'] as const).map(key => <label key={key}>{t(`extensions.binding.${key}`)}<Input value={binding[key]} disabled={busy} onChange={event => { invalidate(); setBinding(previous => ({...previous, [key]: event.target.value})); }}/></label>)}</div>}]} />
                {importError && <Alert type="error" showIcon message={importError}/>}
                {review && <>
                    <ManifestPreview manifest={review.result.manifest} digest={review.result.digest} reviewIdentity/>
                    <div className="extension-preview"><dl>{(['project', 'config_dir', 'data_dir'] as const).map(key => <div key={key} style={{display: 'contents'}}><dt>{t(`extensions.binding.${key}`)}</dt><dd>{review.result.binding[key]}</dd></div>)}</dl></div>
                    {review.result.blockers.length > 0 && <Alert type="warning" showIcon message={t('extensions.blocked')} description={<><ul>{review.result.blockers.map(reason => <li key={reason}>{reason}</li>)}</ul><p>{t('extensions.trustHelp')}</p></>}/>}
                    <Alert type="info" showIcon message={t(`extensions.importRetention.${review.result.manifest.runtime.kind === 'external-systemd' ? 'external' : action}`)}/>
                </>}
            </div>
        </Modal>
        <Modal open={Boolean(pending)} title={pending ? t('extensions.confirmTitle', {action: t(`extensions.actions.${pending.action}`), id: pending.entry?.id ?? ''}) : ''}
            confirmLoading={busy} closable={!busy} maskClosable={!busy} onCancel={() => { if (!busy) setPending(undefined); }}
            okText={t('extensions.confirm')} cancelText={t('extensions.cancel')} okButtonProps={{disabled: blocked, danger: pending?.action === 'remove'}} onOk={() => { void applyAction(); }}>
            {pending && <p>{t(`extensions.actionHelp.${pending.action}`)}</p>}
            {pending?.entry && <Text type="secondary" style={{overflowWrap: 'anywhere'}}>{t('extensions.digest')}: {pending.entry.digest}</Text>}
            {pending?.action === 'rollback' && <p style={{overflowWrap: 'anywhere'}}>{t('extensions.previousVersion')}: {pending.entry?.previous_version}<br/>{t('extensions.digest')}: {pending.entry?.previous_digest}</p>}
            {actionError && <Alert type="error" showIcon message={actionError}/>}
        </Modal>
    </div>;
}
