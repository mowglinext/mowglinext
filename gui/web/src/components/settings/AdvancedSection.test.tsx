import {cleanup, fireEvent, render, screen, waitFor} from '@testing-library/react';
import {afterEach, beforeEach, describe, expect, it, vi} from 'vitest';
import type {ArgsProps} from 'antd/es/notification/interface';
import {AdvancedSection} from './AdvancedSection';
import {useSettingsManager} from '../../hooks/useSettingsManager';

const mocks = vi.hoisted(() => ({
    api: {settings: {yamlList: vi.fn(), yamlCreate: vi.fn()}, request: vi.fn(), mowglinext: {mapDockingCreate: vi.fn()}},
    notification: {success: vi.fn<(args: ArgsProps) => void>(), error: vi.fn<(args: ArgsProps) => void>(), warning: vi.fn<(args: ArgsProps) => void>(), info: vi.fn<(args: ArgsProps) => void>()},
}));
vi.mock('../../hooks/useApi.ts', () => ({useApi: () => mocks.api}));
vi.mock('../../hooks/useHardwareBackend.ts', () => ({useHardwareBackend: () => ({authority: 'confirmed', backend: 'mowgli', runtimeRouting: 'available', parameterRoutes: {}, defaultOverrides: {}, loading: false})}));
vi.mock('../../hooks/useContainerRestart.ts', () => ({useContainerRestart: () => ({pending: false, run: vi.fn()})}));
vi.mock('antd', async original => ({...await original<typeof import('antd')>(), App: {useApp: () => ({notification: mocks.notification})}}));
const initial = {str_true: 'true', str_number: '42', empty: '', num: 2, flag: false, list: [0, false], object: {inner: {x: 2}}};
function Harness() {
    const manager = useSettingsManager();
    if (manager.loading) return <span>Loading</span>;
    return <>
        <AdvancedSection values={manager.values} advancedKeys={manager.advancedKeys} onChange={manager.handleChange} validateNewKey={manager.validateAdvancedKey}/>
        <output data-testid="values">{JSON.stringify(manager.values)}</output>
        <output data-testid="dirty">{manager.dirtyCount}</output>
        <button onClick={() => {void manager.save();}}>Save settings</button>
        <button onClick={manager.revert}>Revert settings</button>
    </>;
}
async function setup() {const view = render(<Harness/>); await screen.findByTestId('values'); return view;}
const field = (key: string) => screen.getByRole('textbox', {name: `Value for ${key}`});
const form = () => screen.getByTestId('values');
async function typeSelection(type: 'String' | 'Number' | 'Boolean') {
    fireEvent.mouseDown(screen.getByRole('combobox', {name: 'Value type'}));
    const option = await screen.findByText(type, {selector: '.ant-select-item-option-content'});
    fireEvent.click(option);
}
async function save() {
    fireEvent.click(screen.getByText('Save settings'));
    await waitFor(() => expect(mocks.api.settings.yamlCreate).toHaveBeenCalledOnce());
    await waitFor(() => expect(screen.getByTestId('dirty')).toHaveTextContent('0'));
}
beforeEach(() => {
    vi.clearAllMocks(); sessionStorage.clear();
    mocks.api.settings.yamlList.mockResolvedValue({data: {...initial}});
    mocks.api.settings.yamlCreate.mockResolvedValue({});
    mocks.api.request.mockResolvedValue({data: {}});
});
afterEach(cleanup);

describe('advanced scalar editing and general Save integration', () => {
    it.each(['true', '42', ''])('adds a literal string %j without inference or premature backend writes', async value => {
        await setup();
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter name'}), {target: {value: 'new_param'}});
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter value'}), {target: {value}});
        fireEvent.click(screen.getByRole('button', {name: /Add$/}));
        expect(field('new_param')).toHaveValue(value);
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({new_param: value});
    });
    it('adds an explicitly selected finite number and boolean false with their original types', async () => {
        await setup(); await typeSelection('Number');
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter name'}), {target: {value: 'new_number'}});
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter value'}), {target: {value: '-1e-2'}});
        fireEvent.click(screen.getByRole('button', {name: /Add$/}));
        await typeSelection('Boolean');
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter name'}), {target: {value: 'new_flag'}});
        fireEvent.click(screen.getByRole('button', {name: /Add$/}));
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({new_number: -.01, new_flag: false});
    });
    it('preserves existing string types, boolean false and empty strings through edits', async () => {
        await setup();
        fireEvent.change(field('str_true'), {target: {value: '42'}});
        fireEvent.change(field('str_number'), {target: {value: 'true'}});
        fireEvent.change(field('empty'), {target: {value: 'temporary'}});
        fireEvent.change(field('empty'), {target: {value: ''}});
        const flag = screen.getByRole('switch', {name: 'Value for flag'});
        fireEvent.click(flag); fireEvent.click(flag);
        expect(flag).toHaveAttribute('aria-checked', 'false');
        await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({str_true: '42', str_number: 'true'});
        expect(form().textContent).toContain('"empty":""'); expect(form().textContent).toContain('"flag":false');
    });
    it('allows intermediate numeric drafts and commits complete finite zero or exponent values only', async () => {
        await setup();
        for (const text of ['-', '.', '1e']) {
            fireEvent.change(field('num'), {target: {value: text}});
            expect(field('num')).toHaveValue(text);
            expect(form().textContent).toContain('"num":2');
        }
        fireEvent.change(field('num'), {target: {value: '-0'}}); fireEvent.blur(field('num'));
        expect(form().textContent).toContain('"num":0');
        fireEvent.change(field('num'), {target: {value: '-1e-2'}}); fireEvent.keyDown(field('num'), {key: 'Enter', code: 'Enter'});
        expect(form().textContent).toContain('"num":-0.01');
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({num: -.01});
    });
    it.each(['', 'Infinity', '-Infinity', 'NaN', '1e999', '0x10', '1.'])('ignores invalid numeric draft %j without a type change, deletion or write', async text => {
        await setup(); fireEvent.change(field('num'), {target: {value: text}}); fireEvent.blur(field('num'));
        expect(field('num')).toHaveValue('2');
        expect(form().textContent).toContain('"num":2'); expect(screen.getByTestId('dirty')).toHaveTextContent('0');
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        expect(screen.getByText(/Invalid or incomplete number ignored/)).toBeInTheDocument();
    });
    it('renders arrays/objects as read-only JSON and preserves them through an unrelated scalar Save', async () => {
        await setup();
        expect(screen.queryByRole('textbox', {name: 'Value for list'})).not.toBeInTheDocument();
        expect(screen.queryByRole('textbox', {name: 'Value for object'})).not.toBeInTheDocument();
        expect(screen.getAllByText('Structured value (read-only)')).toHaveLength(2);
        expect(screen.getByText(/"inner"/, {selector: 'pre'})).toBeInTheDocument();
        expect(form().textContent).not.toContain('[object Object]');
        fireEvent.change(field('str_true'), {target: {value: 'changed'}}); await save();
        expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({str_true: 'changed'});
        expect(form().textContent).toContain('"list":[0,false]');
        expect(form().textContent).toContain('"object":{"inner":{"x":2}}');
    });
    it.each(['save', 'revert'])('keeps deletion explicit and pending, then supports %s', async action => {
        await setup(); fireEvent.click(screen.getByRole('button', {name: 'Delete str_true'}));
        fireEvent.click(await screen.findByRole('button', {name: 'Remove'}));
        await waitFor(() => expect(screen.queryByRole('textbox', {name: 'Value for str_true'})).not.toBeInTheDocument());
        expect(form().textContent).toContain('"str_true":null');
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        if (action === 'save') {
            await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({str_true: null});
            expect(form().textContent).not.toContain('"str_true"');
        } else {
            fireEvent.click(screen.getByText('Revert settings'));
            expect(field('str_true')).toHaveValue('true'); expect(screen.getByTestId('dirty')).toHaveTextContent('0');
            expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        }
    });
    it.each(['num', 'dock_pose_x', 'dock_pose_yaw', 'gnss_receiver_family', 'wheel_radius', 'use_scan_matching', 'outline_passes', '', ' num ', 'my-key'])('rejects duplicate, protected, retired or invalid key %j', async name => {
        await setup();
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter name'}), {target: {value: name}});
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter value'}), {target: {value: 'bad'}});
        expect(screen.getByRole('button', {name: /Add$/})).toBeDisabled();
        fireEvent.keyDown(screen.getByRole('textbox', {name: 'New parameter name'}), {key: 'Enter'});
        expect(screen.getByTestId('dirty')).toHaveTextContent('0');
        expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
        expect(mocks.api.mowglinext.mapDockingCreate).not.toHaveBeenCalled();
    });
    it('does not normalize a colliding name and preserves the exact case of valid unknown keys', async () => {
        mocks.api.settings.yamlList.mockResolvedValue({data: {...initial, my_key: 'original'}});
        await setup(); const name = screen.getByRole('textbox', {name: 'New parameter name'});
        fireEvent.change(name, {target: {value: 'my-key'}});
        expect(name).toHaveValue('my-key'); expect(screen.getByRole('button', {name: /Add$/})).toBeDisabled();
        expect(field('my_key')).toHaveValue('original');
        fireEvent.change(name, {target: {value: 'UPPER'}});
        fireEvent.change(screen.getByRole('textbox', {name: 'New parameter value'}), {target: {value: '42'}});
        fireEvent.click(screen.getByRole('button', {name: /Add$/}));
        expect(field('UPPER')).toHaveValue('42');
        await save(); expect(mocks.api.settings.yamlCreate).toHaveBeenCalledWith({UPPER: '42'});
    });
});


it('commits Enter then blur once even before the parent echoes the changed numeric value', () => {
    const onChange = vi.fn();
    render(<AdvancedSection values={{num: 2}} advancedKeys={['num']} onChange={onChange} validateNewKey={() => 'duplicateKey'}/>);
    fireEvent.change(field('num'), {target: {value: '3'}});
    fireEvent.keyDown(field('num'), {key: 'Enter', code: 'Enter'});
    fireEvent.blur(field('num'));
    expect(onChange).toHaveBeenCalledExactlyOnceWith('num', 3);
    expect(mocks.api.settings.yamlCreate).not.toHaveBeenCalled();
});
