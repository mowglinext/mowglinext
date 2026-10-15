import {useRef, useState} from 'react';
import {Alert, App, Button, Drawer, Form, Select, Slider, Space, Switch, Tag, Typography, theme} from 'antd';
import {ControlOutlined} from '@ant-design/icons';
import {useTranslation} from 'react-i18next';
import {useBrowserGamepad} from './useBrowserGamepad';
import {
    ACTION_COMMANDS, BUTTON_BINDINGS, DEFAULT_PROFILE, MAX_LINEAR_MPS, POLL_MAX_GAP_MS, controllerKey, loadProfile, loadSelectedControllerKey,
    profileFits, saveProfile, saveSelectedControllerKey, validProfile,
    type ButtonBinding, type ControllerIdentity, type ControllerProfile, type ControllerSample,
} from './controller';

interface Props {
    mowerAction: (action: string, params: Record<string, unknown>) => () => Promise<void>;
    onOwnershipChange: (owns: boolean) => void;
}
export function BrowserController({mowerAction, onOwnershipChange}: Props) {
    const {t} = useTranslation();
    const {message} = App.useApp();
    const {token} = theme.useToken();
    const [open, setOpen] = useState(false);
    const [drawerClosed, setDrawerClosed] = useState(true);
    const [selected, setSelected] = useState<ControllerIdentity | null>(null);
    const selectionExplicit = useRef(false);
    const [preferredController, setPreferredController] = useState(loadSelectedControllerKey);
    const [profile, setProfile] = useState<ControllerProfile>(() => structuredClone(DEFAULT_PROFILE));
    const [profileState, setProfileState] = useState<'default' | 'saved' | 'unsaved'>('default');
    const [learn, setLearn] = useState<ButtonBinding | 'linearAxis' | 'angularAxis' | null>(null);
    const learnNeutral = useRef(false);
    const applySelection = (device: ControllerSample) => {
        setSelected({index: device.index, id: device.id});
        const saved = loadProfile(controllerKey(device));
        setProfile(saved ?? structuredClone(DEFAULT_PROFILE));
        // Unknown layouts require the operator to test and save the mapping first.
        setProfileState(saved ? 'saved' : device.mapping === 'standard' ? 'default' : 'unsaved');
    };
    const observeInventory = (devices: ControllerSample[]) => {
        if (selectionExplicit.current || !preferredController) return;
        const matches = devices.filter(device => controllerKey(device) === preferredController);
        // Gamepad IDs need not be unique. Ambiguous devices need an explicit choice.
        if (matches.length !== 1) { setSelected(null); return; }
        if (selected?.index !== matches[0].index || selected.id !== matches[0].id) applySelection(matches[0]);
    };
    const observeInput = (devices: ControllerSample[]) => {
        const pad = devices.find(p => p.index === selected?.index && p.id === selected.id);
        if (!learn || !pad) return;
        if (!learnNeutral.current) {
            learnNeutral.current = pad.buttons.every(b => !b.pressed && b.value < 0.5) && pad.axes.every(v => Math.abs(v) < 0.25);
            return;
        }
        const axisBinding = learn === 'linearAxis' || learn === 'angularAxis';
        const candidates = axisBinding ? pad.axes.flatMap((v, i) => Math.abs(v) > 0.65 ? [i] : []) :
            pad.buttons.flatMap((b, i) => b.pressed || b.value >= 0.5 ? [i] : []);
        if (candidates.length === 1) {
            setProfile(old => axisBinding ? {...old, [learn]: candidates[0]} : {...old, buttons: {...old.buttons, [learn]: candidates[0]}});
            setProfileState('unsaved');
            setLearn(null);
        }
    };
    const controller = useBrowserGamepad({profile, selected, onOwnershipChange, onSample: observeInput, onInventoryChange: observeInventory, preview: open,
        runAction: action => mowerAction('high_level_control', {Command: ACTION_COMMANDS[action]})(),
    });
    const pad = controller.pads.find(p => p.index === selected?.index && p.id === selected.id);
    const configured = !!pad && profileFits(profile, pad) && profileState !== 'unsaved';
    const update = (next: ControllerProfile) => { selectionExplicit.current = true; setProfile(next); setProfileState('unsaved'); };
    const select = (index: number) => {
        controller.disable();
        setLearn(null);
        const device = controller.pads.find(p => p.index === index);
        if (!device) { setSelected(null); return; }
        selectionExplicit.current = true;
        applySelection(device);
        const key = controllerKey(device);
        setPreferredController(key);
        try { saveSelectedControllerKey(key); }
        catch { void message.warning(t('gamepad.selectionSaveFailed')); }
    };
    const bind = (key: NonNullable<typeof learn>) => { selectionExplicit.current = true; learnNeutral.current = false; setLearn(key); };
    const save = () => {
        if (!pad || !profileFits(profile, pad)) return;
        selectionExplicit.current = true;
        try { saveProfile(controllerKey(pad), profile); setProfileState('saved'); }
        catch { void message.error(t('gamepad.saveFailed')); }
    };
    const openSettings = () => { setDrawerClosed(false); controller.disable(); setOpen(true); };
    const closeSettings = () => { setLearn(null); setOpen(false); };
    const inputLossText = controller.inputLoss ? t(`gamepad.inputLossReasons.${controller.inputLoss.reason}`, {
        ...controller.inputLoss, pollGapMs: Math.round(controller.inputLoss.pollGapMs ?? 0), limit: POLL_MAX_GAP_MS,
    }) : null;
    const buttonLabel = (index: number) => {
        const label = t('gamepad.buttonIndex', {index});
        const names = ['A / ×', 'B / ○', 'X / □', 'Y / △', 'LB / L1', 'RB / R1', 'LT / L2', 'RT / R2'];
        return pad?.mapping === 'standard' && names[index] ? `${names[index]} (${label})` : label;
    };
    const deadman = buttonLabel(profile.buttons.deadman);
    const buttonOptions = pad?.buttons.map((_, value) => ({value, label: buttonLabel(value)})) ?? [];
    const axisOptions = pad?.axes.map((_, value) => ({value, label: t('gamepad.axisIndex', {index: value})})) ?? [];
    return <>
        <Space direction="vertical" size={4} style={{position: 'absolute', top: 12, left: 16, zIndex: 20,
            maxWidth: 'calc(100% - 32px)', padding: 8, borderRadius: token.borderRadiusLG,
            background: token.colorBgElevated, boxShadow: token.boxShadowSecondary}}>
            <Space wrap>
                <Button size="large" aria-label={t('gamepad.title')} icon={<ControlOutlined aria-hidden/>} onClick={openSettings}>{t('gamepad.title')}</Button>
                {selected && <Button size="large" type={controller.enabled ? 'default' : 'primary'} danger={controller.enabled}
                    disabled={open || !drawerClosed || (!controller.enabled && (!configured || typeof navigator.getGamepads !== 'function'))}
                    onClick={event => {
                        if (controller.enabled) { controller.disable(); return; }
                        // An explicit tap on stable page content also requests native
                        // WKContentView focus on iPad. Never activate from Drawer callbacks
                        // or unmount this focused button when the session state changes.
                        event.currentTarget.focus({preventScroll: true});
                        selectionExplicit.current = true;
                        controller.enable();
                    }}>{t(controller.enabled ? 'gamepad.disable' : 'gamepad.enable')}</Button>}
            </Space>
            {selected && <Typography.Text type="secondary" ellipsis style={{maxWidth: 320}}>
                {selected.index + 1}: {selected.id}
            </Typography.Text>}
            {!selected && preferredController && <Typography.Text type="secondary">{t('gamepad.chooseController')}</Typography.Text>}
            {selected && !controller.enabled && !controller.fault && <Typography.Text type="secondary">
                {t(configured ? 'gamepad.enableHelp' : 'gamepad.setupRequired')}
            </Typography.Text>}
            {controller.enabled && <>
                <Typography.Text role="status">{t(`gamepad.states.${controller.state}`, {deadman})}</Typography.Text>
                <Typography.Text type="secondary">{t('gamepad.driveSummary', {deadman, stop: buttonLabel(profile.buttons.stop), speed: (MAX_LINEAR_MPS * profile.speedScale).toFixed(2)})}</Typography.Text>
            </>}
            {!controller.enabled && controller.fault && <Tag color="warning" role="alert" style={{whiteSpace: 'normal', margin: 0}}>
                {inputLossText ?? t(`gamepad.faults.${controller.fault}`)} {t('gamepad.retryHelp')}
            </Tag>}
        </Space>
        <Drawer title={t('gamepad.title')} open={open} destroyOnHidden afterOpenChange={visible => setDrawerClosed(!visible)}
            onClose={closeSettings} width={480} footer={<Button size="large" type="primary" block onClick={closeSettings}>{t('gamepad.done')}</Button>}>
            <Space direction="vertical" size="middle" style={{width: '100%'}}>
                <Typography.Paragraph>{t('gamepad.connectHelp')}</Typography.Paragraph>
                {typeof navigator.getGamepads !== 'function' && <Alert type="warning" showIcon message={t('gamepad.faults.unsupported')}/>}
                <Form layout="vertical">
                    <Form.Item label={t('gamepad.controller')}>
                        <Select style={{width: '100%'}} value={pad?.index} placeholder={t('gamepad.selectController')}
                            onChange={select} notFoundContent={t('gamepad.noController')}
                            options={controller.pads.map(p => ({value: p.index, label: `${p.index + 1}: ${p.id}`}))}/>
                    </Form.Item>
                    {selected && !pad && <Alert type="warning" message={t('gamepad.disconnected')}/>}
                    {pad && <>
                        <Tag>{t(`gamepad.profiles.${profileState}`)}</Tag>
                        <Typography.Paragraph type="secondary">{t('gamepad.storageHelp')}</Typography.Paragraph>
                        <Typography.Paragraph>{t('gamepad.liveInput')}</Typography.Paragraph>
                        <Space wrap aria-label={t('gamepad.liveInput')}>
                            {pad.axes.map((value, index) => <Tag key={`axis-${index}`}>{t('gamepad.axisIndex', {index})}: {value.toFixed(2)}</Tag>)}
                            {pad.buttons.map((button, index) => <Tag key={`button-${index}`} color={button.pressed || button.value >= 0.5 ? 'green' : undefined}>
                                {t('gamepad.buttonIndex', {index})}: {button.value.toFixed(2)}
                            </Tag>)}
                        </Space>
                        {(['linearAxis', 'angularAxis'] as const).map(key => <Form.Item key={key} label={t(`gamepad.${key}`)}>
                            <Space wrap>
                                <Select aria-label={t(`gamepad.${key}`)} style={{width: 130}} value={profile[key]} options={axisOptions}
                                    onChange={value => update({...profile, [key]: value})}/>
                                <Button onClick={() => bind(key)}>{t('gamepad.bind')}</Button>
                                <Switch checked={key === 'linearAxis' ? profile.invertLinear : profile.invertAngular}
                                    aria-label={t('gamepad.invertAxis', {axis: t(`gamepad.${key}`)})}
                                    onChange={value => update({...profile, [key === 'linearAxis' ? 'invertLinear' : 'invertAngular']: value})}/>
                                {t('gamepad.invert')}
                            </Space>
                        </Form.Item>)}
                        <Form.Item label={t('gamepad.deadzone')}>
                            <Slider ariaLabelForHandle={t('gamepad.deadzone')} min={0.05} max={0.4} step={0.01} value={profile.deadzone} onChange={value => update({...profile, deadzone: value})}/>
                        </Form.Item>
                        <Form.Item label={t('gamepad.speedScale')}>
                            <Slider ariaLabelForHandle={t('gamepad.speedScale')} min={0.1} max={1} step={0.05} value={profile.speedScale} onChange={value => update({...profile, speedScale: value})}/>
                        </Form.Item>
                        {BUTTON_BINDINGS.map(key => <Form.Item key={key} label={t(`gamepad.bindings.${key}`)}>
                            <Space wrap>
                                <Select aria-label={t(`gamepad.bindings.${key}`)} style={{width: 160}} value={profile.buttons[key]} options={buttonOptions}
                                    onChange={value => update({...profile, buttons: {...profile.buttons, [key]: value}})}/>
                                <Button onClick={() => bind(key)}>{t('gamepad.bind')}</Button>
                            </Space>
                        </Form.Item>)}
                        {learn && <Alert type="info" role="status" message={t('gamepad.learning', {binding: t(learn === 'linearAxis' || learn === 'angularAxis' ? `gamepad.${learn}` : `gamepad.bindings.${learn}`)})}
                            action={<Button onClick={() => setLearn(null)}>{t('gamepad.cancelBind')}</Button>}/>}
                        {(!validProfile(profile) || !profileFits(profile, pad)) && <Alert type="error" message={t('gamepad.invalidProfile')}/>}
                        <Space wrap>
                            <Button onClick={save} disabled={!profileFits(profile, pad) || !!learn}>{t('gamepad.save')}</Button>
                            <Button onClick={() => {setLearn(null); update(structuredClone(DEFAULT_PROFILE));}}>{t('gamepad.restoreDefaults')}</Button>
                        </Space>
                    </>}
                </Form>
                <Alert type="info" showIcon message={t('gamepad.safetyHelp')}/>
                {controller.fault && <Alert type="warning" showIcon message={t(`gamepad.faults.${controller.fault}`)}/>}
                {controller.inputLoss && <Alert type="warning" showIcon message={inputLossText}
                    description={<Typography.Text code copyable>{JSON.stringify(controller.inputLoss)}</Typography.Text>}/>}
                <Typography.Paragraph type="secondary">{t('gamepad.enableHelp')}</Typography.Paragraph>
            </Space>
        </Drawer>
    </>;
}
