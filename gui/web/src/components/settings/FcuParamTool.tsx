import {useEffect, useMemo, useRef, useState} from 'react';
import {Alert, Button, Card, Checkbox, Descriptions, Input, InputNumber, List, Select, Space, Collapse, Tag, Typography} from 'antd';
import {useTranslation} from 'react-i18next';
import {formatMavrosFirmwareVersion, mavrosAutopilotName, type MavrosInfo} from '../../hooks/useMavrosInfo';
import {exportParamDocument, parseParamDocument, type ParamDocument} from '../../utils/fcuParamDocument';
import {identityFromMavros, identityFromParamComments, type ParamIdentity} from '../../utils/fcuParamIdentity';
import {admissibleParamCorrections, profileForIdentity, referenceParamHardware, reviewParamDocument,
    selectedParamCorrections, validParamHardware, type ParamHardware, type ParamReview} from '../../utils/fcuParamProfile';
import {diagnosticGroups, diagnosticState, groupDiagnosticRows, summarizeDiagnostics, telemetryDiagnostics, type GnssDiagnosticBackend, type DiagnosticGroup} from '../../utils/fcuParamDiagnostics';
import {listParamOriginals, saveParamOriginal, type ParamBackup} from '../../utils/fcuParamStorage';

const download = (filename: string, text: string) => {
    const url = URL.createObjectURL(new Blob([text], {type: 'text/plain;charset=utf-8'}));
    const a = document.createElement('a'); a.href = url; a.download = filename; a.click(); URL.revokeObjectURL(url);
};
async function readParamFile(file: File): Promise<string> {
    if (file.size > 2_000_000 || !/\.param$/i.test(file.name)) throw new Error('invalidFile');
    const bytes = file.arrayBuffer ? await file.arrayBuffer() : await new Promise<ArrayBuffer>((resolve, reject) => {
        const reader = new FileReader(); reader.onload = () => resolve(reader.result as ArrayBuffer);
        reader.onerror = () => reject(new Error('invalidFile')); reader.readAsArrayBuffer(file);
    });
    try {return new TextDecoder('utf-8', {fatal: true, ignoreBOM: true}).decode(bytes);} catch {throw new Error('invalidFile');}
}
export function FcuParamTool({info}: {info?: MavrosInfo}) {
    const {t} = useTranslation();
    const fileInput = useRef<HTMLInputElement>(null);
    const [document, setDocument] = useState<ParamDocument | null>(null);
    const [filename, setFilename] = useState('configuration.param');
    const [identity, setIdentity] = useState<ParamIdentity | null>(null);
    const detected = identityFromMavros(info);
    const target = identity ?? detected;
    const [hardware, setHardware] = useState<ParamHardware>(referenceParamHardware);
    const [backups, setBackups] = useState<ParamBackup[]>([]);
    const [busy, setBusy] = useState(true);
    const [error, setError] = useState<string | null>(null);
    const [selection, setSelection] = useState<{key: string; ids: string[]}>({key: '', ids: []});
    const [confirmation, setConfirmation] = useState<{key: string; confirmed: boolean; reason: string}>({key: '', confirmed: false, reason: ''});
    const [gnss, setGnss] = useState<GnssDiagnosticBackend>('unknown');
    const profile = profileForIdentity(target);
    const planKey = JSON.stringify({filename, original: document?.original, target, hardware});
    const selected = selection.key === planKey ? selection.ids : [];
    const consent = confirmation.key === planKey ? confirmation : {confirmed: false, reason: ''};
    const rows = useMemo(() => document ? reviewParamDocument(document, target, hardware) : [], [document, target, hardware]);
    const groups = groupDiagnosticRows(rows, hardware, gnss);
    const visibleRows = Object.values(groups).flat();
    const flows = telemetryDiagnostics(document, target, hardware, gnss);
    const sensitive = rows.filter(row => selected.includes(row.id) && (row.sensitive || row.provisional));
    const needsConsent = sensitive.length > 0;
    const failure = (caught: unknown) => {
        const key = caught instanceof Error ? caught.message : 'storageError';
        setError(t(`settingsFcuParams.errors.${['invalidFile', 'ambiguousParameter', 'backupCollision', 'confirmationRequired'].includes(key) ? key : 'storageError'}`));
    };
    useEffect(() => {
        let active = true;
        void listParamOriginals().then(stored => {if (active) setBackups(stored);})
            .catch(() => {if (active) setError(t('settingsFcuParams.errors.storageError'));})
            .finally(() => {if (active) setBusy(false);});
        return () => {active = false;};
    }, [t]);
    const load = (backup: ParamBackup) => {
        setDocument(parseParamDocument(backup.original)); setFilename(backup.filename); setIdentity({...backup.identity});
        setSelection({key: '', ids: []}); setConfirmation({key: '', confirmed: false, reason: ''}); setError(null);
    };
    const importFile = async (file: File) => {
        setBusy(true); setError(null);
        try {
            const original = await readParamFile(file); parseParamDocument(original);
            const fromFile = identityFromParamComments(original);
            // Live metadata describes the connected FCU, not necessarily the
            // uploaded export. Identification of that file remains explicit.
            const backup: ParamBackup = {id: `${Date.now()}-${crypto.randomUUID?.() ?? Math.random().toString(36).slice(2)}`,
                importedAt: new Date().toISOString(), filename: file.name, original, identity: fromFile};
            await saveParamOriginal(backup); load(backup); setBackups(previous => [backup, ...previous]);
        } catch (caught) {failure(caught);} finally {setBusy(false);}
    };
    const clearPlan = () => {setSelection({key: '', ids: []}); setConfirmation({key: '', confirmed: false, reason: ''});};
    const changeIdentity = (patch: Partial<ParamIdentity>) => {setIdentity({...target, ...patch, source: 'operator'}); clearPlan();};
    const changeHardware = (patch: Partial<ParamHardware>) => {setHardware(h => ({...h, ...patch})); clearPlan();};
    const selectRule = (row: ParamReview, checked: boolean) => setSelection({key: planKey,
        ids: checked ? [...selected.filter(id => id !== row.id), row.id] : selected.filter(id => id !== row.id)});
    const numberField = (key: keyof ParamHardware, min: number, max: number) => <Space key={key}>
        <Typography.Text>{t(`settingsFcuParams.bindings.${key}`)}</Typography.Text>
        <InputNumber aria-label={t(`settingsFcuParams.bindings.${key}`)} min={min} max={max} precision={0}
            value={hardware[key] as number} onChange={value => {if (value !== null) changeHardware({[key]: value});}}/>
    </Space>;
    const groupFields = (group: DiagnosticGroup) => <Space wrap>
        {group === 'communication' && <>
            <Typography.Text>{t('settingsFcuParams.serialPort')}</Typography.Text>
            <InputNumber aria-label={t('settingsFcuParams.serialPort')} min={0} max={9} precision={0} value={hardware.serialPort} onChange={serialPort => changeHardware({serialPort})}/>
            {numberField('gcsSysId', 1, 255)}{numberField('targetSysId', 1, 255)}
        </>}
        {group === 'actuators' && <>
            <Checkbox checked={hardware.blade} onChange={e => changeHardware({blade: e.target.checked})}>{t('settingsFcuParams.blade')}</Checkbox>
            {hardware.drive !== 'unknown' && <>{numberField('rightOutput', 1, 16)}{numberField('leftOutput', 1, 16)}</>}
            {hardware.blade && numberField('bladeOutput', 1, 16)}
        </>}
        {group === 'can' && hardware.drive === 'vesc-dronecan' && <>
            {numberField('canPort', 1, 2)}{numberField('canDriver', 1, 2)}
            <Checkbox checked={hardware.legacyRpm} onChange={e => changeHardware({legacyRpm: e.target.checked})}>{t('settingsFcuParams.legacyRpm')}</Checkbox>
            {hardware.legacyRpm && <>{numberField('rightEscIndex', 0, 15)}{numberField('leftEscIndex', 0, 15)}</>}
        </>}
        {group === 'navigation' && <Select aria-label={t('settingsFcuParams.gnssBackend')} value={gnss} style={{width: 240}}
            onChange={value => {setGnss(value); clearPlan();}} options={['unknown', 'mavros', 'universal'].map(value => ({value, label: t(`settingsFcuParams.gnssBackends.${value}`)}))}/>}
        {group === 'telemetry' && numberField('mavlinkChannel', 1, 10)}
    </Space>;
    return <Space direction="vertical" style={{width: '100%'}} size={16} data-testid="fcu-param-tool">
        <Alert type="info" showIcon message={t('settingsFcuParams.title')} description={t('settingsFcuParams.scope')}/>
        {error && <Alert type="error" message={error}/>}
        <Card size="small" title={t('settingsFcuParams.importTitle')}>
            <input ref={fileInput} type="file" accept=".param" hidden aria-label={t('settingsFcuParams.importButton')}
                onChange={e => {const file = e.target.files?.[0]; if (file) void importFile(file); e.target.value = '';}}/>
            <Button aria-label={t('settingsFcuParams.importButton')} loading={busy} disabled={busy} onClick={() => fileInput.current?.click()}>{t('settingsFcuParams.importButton')}</Button>
            <Typography.Paragraph>{t('settingsFcuParams.originalPolicy')}</Typography.Paragraph>
        </Card>
        <Card size="small" title={t('settingsFcuParams.identityTitle')}>
            <Descriptions size="small" column={3}>
                <Descriptions.Item label={t('settingsFcuParams.observedFirmware')}>{detected.source === 'mavros' ? mavrosAutopilotName(info?.vehicle?.autopilot) ?? '—' : '—'}</Descriptions.Item>
                <Descriptions.Item label={t('settingsFcuParams.version')}>{detected.source === 'mavros' ? formatMavrosFirmwareVersion(info?.vehicle ?? null) ?? '—' : '—'}</Descriptions.Item>
                <Descriptions.Item label={t('settingsFcuParams.board')}>{detected.board || '—'}</Descriptions.Item>
            </Descriptions>
            <Typography.Paragraph>{t('settingsFcuParams.identifyFile')}</Typography.Paragraph>
            <Space wrap>
                <Select aria-label={t('settingsFcuParams.family')} value={target.family} style={{width: 180}}
                    onChange={family => changeIdentity({family, vehicle: '', version: ''})}
                    options={['unknown', 'ardupilot', 'px4', 'other'].map(value => ({value, label: t(`settingsFcuParams.families.${value}`)}))}/>
                <Select aria-label={t('settingsFcuParams.vehicle')} value={target.vehicle} style={{width: 180}}
                    onChange={vehicle => changeIdentity({vehicle})} options={[{value: '', label: t('settingsFcuParams.unknown')}, {value: 'rover', label: 'Rover'}, {value: 'other', label: t('settingsFcuParams.otherVehicle')}]}/>
                <Input aria-label={t('settingsFcuParams.version')} placeholder={t('settingsFcuParams.version')} value={target.version} style={{width: 130}} onChange={e => changeIdentity({version: e.target.value.trim()})}/>
                <Input aria-label={t('settingsFcuParams.board')} placeholder={t('settingsFcuParams.board')} value={target.board} style={{width: 210}} onChange={e => changeIdentity({board: e.target.value})}/>
                <Button disabled={detected.family === 'unknown'} onClick={() => {changeIdentity(detected); const sysid = info?.vehicle?.sysid; if (sysid && sysid >= 1 && sysid <= 255) changeHardware({targetSysId: sysid});}}>{t('settingsFcuParams.useObserved')}</Button>
            </Space>
            <Typography.Paragraph type="secondary">{t('settingsFcuParams.identitySource', {source: t(`settingsFcuParams.sources.${target.source}`)})}</Typography.Paragraph>
            {!profile && <Alert type="warning" message={t('settingsFcuParams.unsupported')}/>}
        </Card>
        <Card size="small" title={t('settingsFcuParams.hardwareTitle')}>
            <Typography.Paragraph>{t('settingsFcuParams.bindingPolicy')}</Typography.Paragraph>
            <Select aria-label={t('settingsFcuParams.drive')} value={hardware.drive} style={{width: 250}} onChange={drive => changeHardware({drive})}
                options={['unknown', 'vesc-dronecan', 'pwm-skid'].map(value => ({value, label: t(`settingsFcuParams.drives.${value}`)}))}/>
            {!validParamHardware(hardware) && <Alert type="error" message={t('settingsFcuParams.invalidBindings')}/>}
        </Card>
        <Collapse items={diagnosticGroups.map(group => ({key: group, label: <Space wrap>
            <Typography.Text strong>{t(`settingsFcuParams.groups.${group}`)}</Typography.Text>
            {document && profile ? Object.entries(summarizeDiagnostics([...groups[group].map(diagnosticState), ...(group === 'telemetry' ? flows.map(flow => flow.state) : [])]))
                .filter(([, count]) => count > 0).map(([state, count]) => <Tag key={state}>{t(`settingsFcuParams.diagnostics.${state}`)}: {count}</Tag>)
                : <Tag>{t('settingsFcuParams.diagnostics.review')}</Tag>}
        </Space>, children: <Space direction="vertical" style={{width: '100%'}}>
            {groupFields(group)}
            {document && !profile && <Alert type="warning" message={t('settingsFcuParams.unsupported')}/>}
            {document && profile && !groups[group].length && group !== 'telemetry' && <Typography.Text type="secondary">{t('settingsFcuParams.noApplicable')}</Typography.Text>}
            {groups[group].map(row => <details key={row.id} data-testid={`diagnostic-${row.id}`}>
                <summary>{row.parameter} — {t(`settingsFcuParams.diagnostics.${diagnosticState(row)}`)}</summary>
                <Descriptions size="small" column={1}>
                    <Descriptions.Item label={t('settingsFcuParams.current')}>{row.current ?? '—'}</Descriptions.Item>
                    <Descriptions.Item label={t('settingsFcuParams.reason')}>{t(`settingsFcuParams.reasons.${row.reason}`)}</Descriptions.Item>
                    <Descriptions.Item label={t('settingsFcuParams.category')}>{t(`settingsFcuParams.categories.${row.category}`)}{row.provisional && <Tag>{t('settingsFcuParams.provisional')}</Tag>}</Descriptions.Item>
                    <Descriptions.Item label={t('settingsFcuParams.ruleVersions')}>{row.versions.join(', ')}</Descriptions.Item>
                </Descriptions>
                <a href={row.contract} target="_blank" rel="noreferrer">{t('settingsFcuParams.contract')}</a>{' '}
                <a href={row.source} target="_blank" rel="noreferrer">{t('settingsFcuParams.documentation')}</a>
                {row.editable && <Space><Typography.Text>{t('settingsFcuParams.proposed')}: {row.proposed}</Typography.Text>
                    <Checkbox aria-label={`${t('settingsFcuParams.select')} ${row.parameter}`} checked={selected.includes(row.id)} onChange={e => selectRule(row, e.target.checked)}/></Space>}
            </details>)}
            {group === 'telemetry' && profile && <>
                <Alert type="info" message={t('settingsFcuParams.telemetryPolicy')}/>
                {flows.map(flow => <details key={flow.id} data-testid={`telemetry-${flow.id}`}>
                    <summary>{t(`settingsFcuParams.flows.${flow.id}`)} — {t('settingsFcuParams.diagnostics.review')}</summary>
                    <Descriptions size="small" column={1}>
                        <Descriptions.Item label={t('settingsFcuParams.messages')}>{flow.messages}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.consumer')}>{flow.consumer}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.configuredFrequency')}>{flow.parameters.length ? flow.parameters.map(parameter => `${parameter.name}: ${parameter.configuredHz === undefined ? '—' : `${parameter.configuredHz} Hz`}`).join('; ') : t('settingsFcuParams.notEstablished')}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.observedFrequency')}>{t('settingsFcuParams.notMeasured')}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.acquisitionFrequency')}>{t('settingsFcuParams.notMeasured')}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.rosFrequency')}>{t('settingsFcuParams.notMeasured')}</Descriptions.Item>
                        <Descriptions.Item label={t('settingsFcuParams.freshness')}>{flow.freshnessReferenceMs !== undefined ? t('settingsFcuParams.referenceDeadline', {ms: flow.freshnessReferenceMs}) : t('settingsFcuParams.notEstablished')}</Descriptions.Item>
                    </Descriptions>
                    <a href={flow.freshnessSource} target="_blank" rel="noreferrer">{t('settingsFcuParams.contract')}</a>
                </details>)}
            </>}
        </Space>}))}/>
        {document && <Card size="small" title={`${filename} — ${document.entries.size} ${t('settingsFcuParams.parameters')}`}>
            <Typography.Paragraph>{t('settingsFcuParams.hiddenPreserved')}</Typography.Paragraph>
            <Alert type="warning" message={t('settingsFcuParams.provisionalPolicy')} style={{marginBottom: 12}}/>
            <Space wrap>
                <Button disabled={!profile} onClick={() => setSelection({key: planKey, ids: [...new Set([...selected, ...admissibleParamCorrections(visibleRows).map(row => row.id)])]})}>{t('settingsFcuParams.selectAdmissible')}</Button>
                <Button onClick={() => setSelection({key: planKey, ids: []})}>{t('settingsFcuParams.clearSelection')}</Button>
            </Space>
            {needsConsent && <Space direction="vertical" style={{width: '100%', marginTop: 12}}>
                <Alert type="warning" message={t('settingsFcuParams.confirmationTitle')} description={sensitive.map(row => row.parameter).join(', ')}/>
                <Checkbox checked={consent.confirmed} onChange={e => setConfirmation({key: planKey, ...consent, confirmed: e.target.checked})}>{t('settingsFcuParams.confirmation')}</Checkbox>
                <Input.TextArea aria-label={t('settingsFcuParams.justification')} placeholder={t('settingsFcuParams.justification')} value={consent.reason}
                    onChange={e => setConfirmation({key: planKey, ...consent, reason: e.target.value})}/>
            </Space>}
            <Space style={{marginTop: 12}}>
                <Button onClick={() => download(filename, document.original)}>{t('settingsFcuParams.downloadOriginal')}</Button>
                <Button type="primary" disabled={!profile || !validParamHardware(hardware) || !selected.length || (needsConsent && (!consent.confirmed || !consent.reason.trim()))}
                    onClick={() => {try {download(filename.replace(/\.param$/i, '-mowglinext.param'), exportParamDocument(document, selectedParamCorrections(rows, selected, consent)));} catch (caught) {failure(caught);}}}>{t('settingsFcuParams.export')}</Button>
            </Space>
        </Card>}
        <Card size="small" title={t('settingsFcuParams.backups')}>
            <List dataSource={backups} renderItem={backup => <List.Item actions={[
                <Button key="restore" disabled={busy} onClick={() => {try {load(backup);} catch (caught) {failure(caught);}}}>{t('settingsFcuParams.restore')}</Button>,
                <Button key="download" onClick={() => download(backup.filename, backup.original)}>{t('settingsFcuParams.downloadOriginal')}</Button>,
            ]}><List.Item.Meta title={backup.filename} description={backup.importedAt}/></List.Item>}/>
        </Card>
    </Space>;
}
